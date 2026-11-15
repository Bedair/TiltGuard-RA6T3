/*
 * jog.c
 */

#include "jog.h"
#include "control_tick.h"
#include "../actuators/motors.h"
#include "../sensors/encoders.h"
#include "../sensors/battery.h"
#include "../telemetry/telemetry.h"

static bool              s_running;
static int32_t           s_percent;
static uint32_t          s_start_tick;
static uint32_t          s_start_left_total;
static uint32_t          s_start_right_total;
static bool              s_readback_taken;
static fsp_err_t         s_readback_err;
static motors_readback_t s_readback;
static uint32_t          s_battery_before_mv; /* Single-sample V_bat just before driving. */
static uint32_t          s_battery_during_mv; /* ...and at the readback, while driven. */
static bool              s_self_test;         /* The running jog is the S1 self-test. */
static bool              s_self_test_done;    /* Result waiting for jog_get_self_test_result(). */
static bool              s_self_test_passed;

static const char * jog_verdict (uint32_t edges)
{
    return (edges >= JOG_MIN_EDGES) ? "moved" : "DID NOT MOVE";
}

static uint32_t jog_battery_mv (void)
{
    battery_snapshot_t battery;
    battery_get_snapshot(&battery);

    float const volts = (battery.voltage_raw_v > 0.0f) ? battery.voltage_raw_v : 0.0f;

    return (uint32_t) ((volts * 1000.0f) + 0.5f);
}

static void jog_print_readback (void)
{
    if (!s_readback_taken)
    {
        telemetry_printf("jog: no driver readback (stopped before %u ms)\r\n", (unsigned) JOG_READBACK_TICKS);

        return;
    }

    telemetry_printf("jog: driver readback at %u ms (MCU side): STBY=%u AIN1=%u BIN1=%u ",
                     (unsigned) JOG_READBACK_TICKS,
                     (unsigned) s_readback.stby_high,
                     (unsigned) s_readback.ain1_high,
                     (unsigned) s_readback.bin1_high);
    telemetry_printf("PWM timers L=%s R=%s err=%d (expect STBY=1 AIN1=0 BIN1=0, both running)\r\n",
                     s_readback.left_timer_running ? "running" : "STOPPED",
                     s_readback.right_timer_running ? "running" : "STOPPED",
                     (int) s_readback_err);
    telemetry_printf("jog: battery sense before=%u mV during=%u mV (a 2S pack reads 7000..8400)\r\n",
                     (unsigned) s_battery_before_mv, (unsigned) s_battery_during_mv);
}

static void jog_finish (const char * p_reason, bool completed)
{
    motors_disable();
    s_running = false;

    uint32_t left_total;
    uint32_t right_total;
    encoders_get_totals(&left_total, &right_total);

    uint32_t const left_edges  = left_total - s_start_left_total;
    uint32_t const right_edges = right_total - s_start_right_total;

    telemetry_printf("jog: %s - motors off. encL=%u encR=%u edges -> L %s, R %s\r\n",
                     p_reason,
                     (unsigned) left_edges,
                     (unsigned) right_edges,
                     jog_verdict(left_edges),
                     jog_verdict(right_edges));

    jog_print_readback();

    if (s_self_test)
    {
        s_self_test        = false;
        s_self_test_done   = true;
        s_self_test_passed = completed && (left_edges >= JOG_MIN_EDGES) && (right_edges >= JOG_MIN_EDGES);
    }

    if ((left_edges < JOG_MIN_EDGES) || (right_edges < JOG_MIN_EDGES))
    {
        if ((s_battery_before_mv < JOG_MIN_BATTERY_MV) && (s_battery_during_mv < JOG_MIN_BATTERY_MV))
        {
            telemetry_printf("jog: likely cause: NO MOTOR SUPPLY - the battery sense never reached %u mV. "
                             "Battery switch on? Pack charged and plugged in?\r\n", (unsigned) JOG_MIN_BATTERY_MV);
        }

        telemetry_printf("jog: no motion seen. If the readback is as expected, the MCU side is fine: check battery "
                         "switch ON, buck 5 V at the shifter HV side, STBY ~5 V at robot D8 during the jog, ");
        telemetry_printf("motor and encoder connectors.\r\n");

        if (s_percent < JOG_MAX_PERCENT)
        {
            telemetry_printf("jog: %d%% may be below the gearmotor deadband - try 'jog %d'.\r\n",
                             (int) s_percent, JOG_MAX_PERCENT);
        }
    }
}

static void jog_start_common (int32_t percent, bool self_test)
{
    safety_state_t const state = safety_get_state();

    if (s_running)
    {
        telemetry_printf("rejected: jog already running\r\n");

        return;
    }

    if (SAFETY_STATE_READY != state)
    {
        telemetry_printf("rejected: jog only in READY (not armed, no fault)\r\n");

        return;
    }

    if ((percent < 1) || (percent > JOG_MAX_PERCENT))
    {
        telemetry_printf("rejected: jog <pct> needs a whole number 1..%d\r\n", JOG_MAX_PERCENT);

        return;
    }

    int16_t const permille = (int16_t) (percent * 10);

    motors_command_t command;
    motors_plan(permille, permille, &command);

    encoders_get_totals(&s_start_left_total, &s_start_right_total);
    s_start_tick     = control_tick_count();
    s_percent        = percent;
    s_readback_taken = false;
    s_battery_before_mv = jog_battery_mv();
    s_battery_during_mv = 0U;

    if (!motors_apply(&command))
    {
        telemetry_printf("jog: motor driver error - motors disabled\r\n");

        return;
    }

    s_running   = true;
    s_self_test = self_test;

    telemetry_printf("jog: L=%s %u R=%s %u permille for 1 s - both wheels must roll FORWARD\r\n",
                     motors_direction_name(motors_direction(command.left_reverse, command.left_duty_counts)),
                     (unsigned) motors_duty_permille(command.left_duty_counts),
                     motors_direction_name(motors_direction(command.right_reverse, command.right_duty_counts)),
                     (unsigned) motors_duty_permille(command.right_duty_counts));
}

void jog_update (uint32_t tick, safety_state_t safety_state)
{
    if (!s_running)
    {
        return;
    }

    if (SAFETY_STATE_READY != safety_state)
    {
        jog_finish("aborted (left READY)", false);

        return;
    }

    uint32_t const elapsed = tick - s_start_tick;

    if ((!s_readback_taken) && (elapsed >= JOG_READBACK_TICKS))
    {
        s_readback_err      = motors_readback(&s_readback);
        s_readback_taken    = true;
        s_battery_during_mv = jog_battery_mv();
    }

    if (elapsed >= JOG_DURATION_TICKS)
    {
        jog_finish("done", true);
    }
}

void jog_start (int32_t percent)
{
    jog_start_common(percent, false);
}

void jog_start_self_test (void)
{
    s_self_test_done = false;
    jog_start_common(JOG_SELF_TEST_PERCENT, true);
}

void jog_abort (const char * p_reason)
{
    if (s_running)
    {
        jog_finish(p_reason, false);
    }
}

bool jog_get_self_test_result (bool * p_passed)
{
    if (!s_self_test_done)
    {
        return false;
    }

    s_self_test_done = false;
    *p_passed        = s_self_test_passed;

    return true;
}

bool jog_is_running (void)
{
    return s_running;
}
