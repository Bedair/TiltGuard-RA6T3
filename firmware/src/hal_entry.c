/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <math.h>
#include "hal_data.h"
#include "telemetry/telemetry.h"
#include "control/control_tick.h"
#include "imu/mpu6050.h"
#include "sensors/encoders.h"
#include "sensors/ultrasonic.h"
#include "sensors/battery.h"
#include "safety/safety.h"
#include "safety/button.h"
#include "actuators/motors.h"
#include "control/balance.h"
#include "control/jog.h"
#include "control/drive.h"
#include "remote/bt_remote.h"
#include "telemetry/console.h"
#include "safety/fault.h"
#include "telemetry/blackbox.h"
#include "build_info.h"

extern bsp_leds_t g_bsp_leds;

/* 1 kHz tick thresholds. */
#define LED_BLINK_TICKS     (500U)  /* 1 Hz LED1 heartbeat, and LED2's ACTIVE blink rate. */
#define LED_FAST_BLINK_TICKS (50U)  /* LED1 at 10 Hz: S1 arm refused (angle beyond the limit)... */
#define LED_REFUSED_TICKS   (2000U) /* ...for 2 s. */
#define CSV_REPORT_TICKS    (10U)   /* 100 Hz CSV on RTT channel 1, while armed. */
#define SENSOR_REPORT_TICKS (100U)  /* 10 Hz: the armed line, or encoder/ultrasonic/battery/safety when not armed. */
#define IMU_REPORT_TICKS    (500U)  /* 2 Hz IMU telemetry - kept slower so the output stays readable. */
#define TICK_REPORT_TICKS   (1000U) /* 1 Hz tick-timing telemetry. */
#define CYCLES_PER_US       (200U)  /* Core clock is 200 MHz; see CLAUDE.md. */

/*
 * Bench-test mode (Phase 2 bring-up): uncomment, or add MOTOR_BENCH_TEST to
 * the compiler's defined symbols, to build it in. Then arming (S1 in READY)
 * runs the bench sequence instead of balancing: one motor at a time, max
 * 15% duty, then auto-disarms. The default build balances
 * (control/balance.c) and never runs the bench sequence.
 */
/* #define MOTOR_BENCH_TEST */

#ifdef MOTOR_BENCH_TEST
#define BENCH_PHASE_TICKS    (2000U) /* 2 s per phase. */
#define BENCH_DUTY_PERMILLE  (150)   /* 15% max, per the bench-test spec. */

typedef enum e_bench_phase
{
    BENCH_PHASE_LEFT_FWD = 0,
    BENCH_PHASE_LEFT_REV,
    BENCH_PHASE_RIGHT_FWD,
    BENCH_PHASE_RIGHT_REV,
    BENCH_PHASE_DONE,
} bench_phase_t;

static const char * bench_phase_name (bench_phase_t phase)
{
    switch (phase)
    {
        case BENCH_PHASE_LEFT_FWD:
        {
            return "left fwd";
        }

        case BENCH_PHASE_LEFT_REV:
        {
            return "left rev";
        }

        case BENCH_PHASE_RIGHT_FWD:
        {
            return "right fwd";
        }

        case BENCH_PHASE_RIGHT_REV:
        {
            return "right rev";
        }

        case BENCH_PHASE_DONE:
        default:
        {
            return "done";
        }
    }
}
#endif

/*
 * SEGGER_RTT_printf has no %f. Splits a float into a sign string plus integer
 * and 2-digit fractional parts so small negative values (e.g. -0.25) keep
 * their sign instead of losing it to (int) truncating -0 to 0.
 */
typedef struct st_decimal2
{
    const char * p_sign;
    int          int_part;
    unsigned     frac_part;
} decimal2_t;

static void split_decimal2 (float value, decimal2_t * p_out)
{
    float const abs_value = fabsf(value);

    p_out->p_sign    = (value < 0.0f) ? "-" : "";
    p_out->int_part  = (int) abs_value;
    p_out->frac_part = (unsigned) (((abs_value - (float) p_out->int_part) * 100.0f) + 0.5f);

    if (p_out->frac_part >= 100U)
    {
        p_out->frac_part = 0U;
        p_out->int_part += 1;
    }
}

static const char * safety_state_name (safety_state_t state)
{
    switch (state)
    {
        case SAFETY_STATE_INIT:
        {
            return "INIT";
        }

        case SAFETY_STATE_CALIBRATING:
        {
            return "CALIBRATING";
        }

        case SAFETY_STATE_READY:
        {
            return "READY";
        }

        case SAFETY_STATE_ACTIVE:
        {
            return "ACTIVE";
        }

        case SAFETY_STATE_FAULT:
        default:
        {
            return "FAULT";
        }
    }
}

/* Safe state: hold LED2 on and halt rather than run with a module that failed to init. */
static void fault_halt (bsp_leds_t const * p_leds, const char * p_module, fsp_err_t err)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite((bsp_io_port_pin_t) p_leds->p_leds[1], BSP_IO_LEVEL_HIGH);
    R_BSP_PinAccessDisable();

    telemetry_printf("%s_init failed: err=%d\r\n", p_module, (int) err);

    while (1)
    {
        ;
    }
}

/*
 * Main-loop helpers. Each report lives in its own non-inlined function, so
 * their locals (snapshots, decimal splits, printf argument blocks) reuse one
 * stack region one at a time instead of all being live in hal_entry()'s frame
 * together. The main stack is 1 KB and must hold the deepest of these plus
 * nested interrupts on top; see docs/hardware-findings.md (stack budget).
 */
#define HAL_NOINLINE __attribute__((noinline))

static HAL_NOINLINE uint32_t echo_timeout_count (void)
{
    ultrasonic_snapshot_t ultrasonic;
    ultrasonic_get_snapshot(&ultrasonic);

    return ultrasonic.timeout_count;
}

#ifndef MOTOR_BENCH_TEST
static HAL_NOINLINE void print_armed_legend (void)
{
    telemetry_printf("line: ang/sp deg, gyro dps | bal spd turn = stock 8-bit terms before clamping, "
                     "pwm_L = bal-spd-turn, pwm_R = bal-spd+turn | ");
    telemetry_printf("L/R = driver command dir + duty permille | encL/encR = edges per 40 ms (no sign) | ");
    telemetry_printf("mot = did the wheel turn: ok, NONE (driven >= %u permille, no edges), low, idle (not driven), "
                     "UNEXP (turned while not driven)\r\n", (unsigned) BALANCE_MOTION_MIN_DUTY_PERMILLE);
    telemetry_printf("expect: ang-sp > +%d deg -> L=FWD R=FWD, ang-sp < -%d deg -> L=REV R=REV\r\n",
                     (int) BALANCE_CHECK_MIN_ERROR_DEG, (int) BALANCE_CHECK_MIN_ERROR_DEG);
}

/* Records this run's parameters and mode. Called after try_arm() has
 * returned, so its frame isn't under the printf chain. */
static HAL_NOINLINE void report_arm (float angle_deg)
{
    balance_params_t params;
    balance_get_params(&params);

    console_print_armed(angle_deg, &params);
    console_print_mode();
    console_print_speed_loop();
    print_armed_legend();
}

/* LED1 fast blink until this tick (S1 arm refused), so it's visible without the laptop. */
static uint32_t s_led1_fast_until_tick;

/* Returns true if it requested ACTIVE; prints the refusal and starts the
 * LED1 fast blink otherwise. */
static HAL_NOINLINE bool try_arm (float angle_deg)
{
    if (fabsf(angle_deg) <= BALANCE_ARM_LIMIT_DEG)
    {
        safety_request_active();

        return true;
    }

    decimal2_t angle_dec;
    split_decimal2(angle_deg, &angle_dec);

    telemetry_printf("arm refused: angle=%s%d.%02u, limit=+/-%d deg (LED1 blinks fast for 2 s)\r\n",
                     angle_dec.p_sign, angle_dec.int_part, angle_dec.frac_part, (int) BALANCE_ARM_LIMIT_DEG);

    s_led1_fast_until_tick = control_tick_count() + LED_REFUSED_TICKS;

    return false;
}

/* Once per READY entry: the angle S1 would start from (e.g. resting on its support). */
static HAL_NOINLINE void report_ready (float angle_deg)
{
    decimal2_t angle_dec;
    split_decimal2(angle_deg, &angle_dec);

    telemetry_printf("READY: angle=%s%d.%02u deg - S1 stands it up from within +/-%d deg "
                     "(resting on its support is fine)\r\n",
                     angle_dec.p_sign, angle_dec.int_part, angle_dec.frac_part, (int) BALANCE_ARM_LIMIT_DEG);
}

static HAL_NOINLINE void report_standup_done (float angle_deg)
{
    decimal2_t angle_dec;
    split_decimal2(angle_deg, &angle_dec);

    telemetry_printf("stand-up window over (2 s): angle=%s%d.%02u deg, tilt cut-out now +/-%d deg\r\n",
                     angle_dec.p_sign, angle_dec.int_part, angle_dec.frac_part, (int) SAFETY_TILT_LIMIT_DEG);
}

/* Leaving ACTIVE: freeze the black box with why the run ended. */
static HAL_NOINLINE void freeze_blackbox (safety_state_t safety_state)
{
    uint32_t const    faults = safety_get_fault_reasons();
    blackbox_reason_t reason;

    if (SAFETY_STATE_FAULT != safety_state)
    {
        reason = BLACKBOX_REASON_S1; /* S1 is the only way to disarm in this build. */
    }
    else if (0U != (faults & SAFETY_FAULT_TILT))
    {
        reason = BLACKBOX_REASON_TILT;
    }
    else
    {
        reason = BLACKBOX_REASON_FAULT;
    }

    blackbox_freeze(reason, faults);
}
#endif

/* S1: short press in READY arms, in ACTIVE disarms, in FAULT resets (back to
 * INIT - CALIBRATING's settle window runs again before READY). Debounced in
 * button.c; this just reacts to a new press. */
#ifndef MOTOR_BENCH_TEST
/* Set by S1 in FAULT: arm automatically once the reset reaches READY. */
static bool s_arm_pending;

static HAL_NOINLINE void handle_pending_arm (float angle_deg)
{
    if (!s_arm_pending)
    {
        return;
    }

    safety_state_t const state = safety_get_state();

    if (SAFETY_STATE_READY == state)
    {
        s_arm_pending = false;

        if (try_arm(angle_deg))
        {
            report_arm(angle_deg);
        }
    }
    else if (SAFETY_STATE_FAULT == state)
    {
        /* Faulted again during the settle window (e.g. still tilted past
         * the limit): don't arm later by surprise. */
        s_arm_pending = false;
        telemetry_printf("not standing up: fault again (faults=0x%x) - press S1 again\r\n",
                         (unsigned) safety_get_fault_reasons());
    }
    else
    {
        /* INIT / CALIBRATING: keep waiting. */
    }
}
#endif

static HAL_NOINLINE void handle_button_press (safety_state_t safety_state, float angle_deg)
{
    switch (safety_state)
    {
        case SAFETY_STATE_READY:
        {
            if (jog_is_running())
            {
                /* S1 during a jog or the self-test: stop it, don't arm. */
                jog_abort("aborted by S1");
                break;
            }

#ifdef MOTOR_BENCH_TEST
            (void) angle_deg;
            safety_request_active();
#else
            /* As stock: set it on the floor (resting on its support is fine)
             * and press S1 - the law runs from any |angle| up to
             * BALANCE_ARM_LIMIT_DEG, the cut-out is skipped for 2 s
             * (SAFETY_STANDUP_*), and from rest the angle term saturates
             * and kicks it upright. */
            if (try_arm(angle_deg))
            {
                report_arm(angle_deg);
            }
#endif
            break;
        }

        case SAFETY_STATE_ACTIVE:
        {
            safety_disarm();
            telemetry_printf("disarmed (S1)\r\n");
            break;
        }

        case SAFETY_STATE_FAULT:
        {
            /* One press after a fall: reset, and stand up as soon as the
             * 0.5 s settle window ends (handle_pending_arm()). */
            safety_reset();
#ifndef MOTOR_BENCH_TEST
            s_arm_pending = true;
            telemetry_printf("S1: fault reset - it stands up in ~0.5 s\r\n");
#endif
            break;
        }

        case SAFETY_STATE_INIT:
        case SAFETY_STATE_CALIBRATING:
        default:
        {
            break; /* No action yet while still settling. */
        }
    }
}

#ifdef MOTOR_BENCH_TEST
static HAL_NOINLINE void bench_step (uint32_t tick, safety_state_t safety_state)
{
    static bench_phase_t  s_bench_phase            = BENCH_PHASE_LEFT_FWD;
    static uint32_t       s_bench_phase_start_tick = 0U;
    static safety_state_t s_prev_safety_state      = SAFETY_STATE_INIT;

    if ((SAFETY_STATE_ACTIVE == safety_state) && (SAFETY_STATE_ACTIVE != s_prev_safety_state))
    {
        /* Just armed - (re)start the bench sequence from the top. */
        s_bench_phase            = BENCH_PHASE_LEFT_FWD;
        s_bench_phase_start_tick = tick;
        telemetry_printf("bench test: %s\r\n", bench_phase_name(s_bench_phase));
    }

    if (SAFETY_STATE_ACTIVE == safety_state)
    {
        if ((tick - s_bench_phase_start_tick) >= BENCH_PHASE_TICKS)
        {
            s_bench_phase            = (bench_phase_t) (s_bench_phase + 1);
            s_bench_phase_start_tick = tick;

            if (BENCH_PHASE_DONE != s_bench_phase)
            {
                telemetry_printf("bench test: %s\r\n", bench_phase_name(s_bench_phase));
            }
        }

        switch (s_bench_phase)
        {
            case BENCH_PHASE_LEFT_FWD:
            {
                motors_set(BENCH_DUTY_PERMILLE, 0);
                break;
            }

            case BENCH_PHASE_LEFT_REV:
            {
                motors_set((int16_t) -BENCH_DUTY_PERMILLE, 0);
                break;
            }

            case BENCH_PHASE_RIGHT_FWD:
            {
                motors_set(0, BENCH_DUTY_PERMILLE);
                break;
            }

            case BENCH_PHASE_RIGHT_REV:
            {
                motors_set(0, (int16_t) -BENCH_DUTY_PERMILLE);
                break;
            }

            case BENCH_PHASE_DONE:
            default:
            {
                telemetry_printf("bench test: done, disarming\r\n");
                safety_disarm();
                break;
            }
        }
    }

    s_prev_safety_state = safety_state;
}
#endif

/*
 * Armed reporting. All of it comes from one balance_get_status() snapshot
 * (the most recent control step), so the fields of a line always belong to
 * the same step. The line buffer and snapshot are static: they'd otherwise
 * add ~250 B to the main stack (see docs/hardware-findings.md).
 */
static telemetry_line_t s_line;
static balance_status_t s_bal;

#define CSV_HEADER "t_ms,ang,sp,gyro,bal,spd,turn,dirL,dutyL,dirR,dutyR,encL,encR,motL,motR,mode"

static const char * armed_mode_name (balance_status_t const * p_status)
{
    return p_status->dry_run ? "DRY-RUN" : "ACTIVE";
}

static void line_direction_duty (telemetry_line_t * p, bool reverse, uint32_t duty_counts, char separator)
{
    telemetry_line_str(p, motors_direction_name(motors_direction(reverse, duty_counts)));
    telemetry_line_char(p, separator);
    telemetry_line_u32(p, (uint32_t) motors_duty_permille(duty_counts));
}

/* 10 Hz on channel 0 while armed:
 * "ang=+3.2 sp=+0.0 gyro=-12.1 | bal=+176 spd=+0 turn=+0 | L=FWD 690 R=FWD 690 | encL=45 encR=44 | ACTIVE" */
static HAL_NOINLINE void report_armed_line (void)
{
    telemetry_line_t * const p = &s_line;

    telemetry_line_begin(p);
    telemetry_line_str(p, "ang=");
    telemetry_line_fixed_signed(p, s_bal.angle_deg, 1U);
    telemetry_line_str(p, " sp=");
    telemetry_line_fixed_signed(p, s_bal.setpoint_deg, 1U);
    telemetry_line_str(p, " gyro=");
    telemetry_line_fixed_signed(p, s_bal.gyro_dps, 1U);
    telemetry_line_str(p, " | bal=");
    telemetry_line_fixed_signed(p, s_bal.bal_8bit, 0U);
    telemetry_line_str(p, " spd=");
    telemetry_line_fixed_signed(p, s_bal.spd_8bit, 0U);
    telemetry_line_str(p, " turn=");
    telemetry_line_fixed_signed(p, s_bal.turn_8bit, 0U);
    telemetry_line_str(p, " | L=");
    line_direction_duty(p, s_bal.command.left_reverse, s_bal.command.left_duty_counts, ' ');
    telemetry_line_str(p, " R=");
    line_direction_duty(p, s_bal.command.right_reverse, s_bal.command.right_duty_counts, ' ');
    telemetry_line_str(p, " | encL=");
    telemetry_line_u32(p, s_bal.enc_left_window);
    telemetry_line_str(p, " encR=");
    telemetry_line_u32(p, s_bal.enc_right_window);
    telemetry_line_str(p, " | mot L=");
    telemetry_line_str(p, balance_motion_name(s_bal.motion_left));
    telemetry_line_str(p, " R=");
    telemetry_line_str(p, balance_motion_name(s_bal.motion_right));
    telemetry_line_str(p, " | drv=");
    telemetry_line_fixed_signed(p, s_bal.speed_target, 0U);
    telemetry_line_str(p, " trn=");
    telemetry_line_fixed_signed(p, s_bal.turn_target, 0U);
    telemetry_line_str(p, " | ");
    telemetry_line_str(p, armed_mode_name(&s_bal));

    (void) telemetry_line_send(p, 0U);
}

/* 100 Hz on channel 1 while armed: the same fields as CSV, t_ms first. */
static HAL_NOINLINE void report_armed_csv (uint32_t tick)
{
    telemetry_line_t * const p = &s_line;

    telemetry_line_begin(p);
    telemetry_line_u32(p, tick);
    telemetry_line_char(p, ',');
    telemetry_line_fixed(p, s_bal.angle_deg, 2U);
    telemetry_line_char(p, ',');
    telemetry_line_fixed(p, s_bal.setpoint_deg, 2U);
    telemetry_line_char(p, ',');
    telemetry_line_fixed(p, s_bal.gyro_dps, 2U);
    telemetry_line_char(p, ',');
    telemetry_line_fixed(p, s_bal.bal_8bit, 1U);
    telemetry_line_char(p, ',');
    telemetry_line_fixed(p, s_bal.spd_8bit, 1U);
    telemetry_line_char(p, ',');
    telemetry_line_fixed(p, s_bal.turn_8bit, 1U);
    telemetry_line_char(p, ',');
    line_direction_duty(p, s_bal.command.left_reverse, s_bal.command.left_duty_counts, ',');
    telemetry_line_char(p, ',');
    line_direction_duty(p, s_bal.command.right_reverse, s_bal.command.right_duty_counts, ',');
    telemetry_line_char(p, ',');
    telemetry_line_u32(p, s_bal.enc_left_window);
    telemetry_line_char(p, ',');
    telemetry_line_u32(p, s_bal.enc_right_window);
    telemetry_line_char(p, ',');
    telemetry_line_str(p, balance_motion_name(s_bal.motion_left));
    telemetry_line_char(p, ',');
    telemetry_line_str(p, balance_motion_name(s_bal.motion_right));
    telemetry_line_char(p, ',');
    telemetry_line_str(p, armed_mode_name(&s_bal));

    (void) telemetry_line_send(p, 1U); /* Dropped whole if the host isn't keeping up. */
}

#ifndef MOTOR_BENCH_TEST
static HAL_NOINLINE void report_csv_header (void)
{
    telemetry_line_t * const p = &s_line;

    telemetry_line_begin(p);
    telemetry_line_str(p, CSV_HEADER);
    (void) telemetry_line_send(p, 1U);
}

#endif

/* 1 Hz while armed ("check: ...") and once when leaving ACTIVE ("logic check: ..."). */
static HAL_NOINLINE void report_logic_check (const char * p_prefix)
{
    balance_check_t const * const p_check = &s_bal.check;

    if (p_check->speed_loop_on)
    {
        telemetry_printf("%s skipped (speed loop on)\r\n", p_prefix);

        return;
    }

    telemetry_printf("%s samples=%u sign_err=%u (not checked: %u with D/turn >= P/2)\r\n",
                     p_prefix,
                     (unsigned) p_check->samples,
                     (unsigned) p_check->sign_err,
                     (unsigned) p_check->skipped);
}

/* "motion: L moved=a/b none=c unexp=d | R ...": a/b = windows with edges / windows driven >= 30%. */
static HAL_NOINLINE void report_motion_wheel (const char * p_prefix, balance_motion_count_t const * p_count)
{
    telemetry_printf("%s moved=%u/%u none=%u unexp=%u",
                     p_prefix,
                     (unsigned) p_count->moved,
                     (unsigned) p_count->judged,
                     (unsigned) p_count->none,
                     (unsigned) p_count->unexpected);
}

static HAL_NOINLINE void report_motion (const char * p_prefix)
{
    report_motion_wheel(p_prefix, &s_bal.motion_count_left);
    report_motion_wheel(" | R", &s_bal.motion_count_right);
    telemetry_printf(" (40 ms windows; judged = driven >= %u permille)\r\n", (unsigned) BALANCE_MOTION_MIN_DUTY_PERMILLE);
}

#ifndef MOTOR_BENCH_TEST
static HAL_NOINLINE void report_disarmed (safety_state_t safety_state)
{
    balance_get_status(&s_bal); /* Check counters are held after disarm, until the next arm. */

    if (SAFETY_STATE_FAULT == safety_state)
    {
        telemetry_printf("disarmed: FAULT faults=0x%x\r\n", (unsigned) safety_get_fault_reasons());
    }

    report_logic_check("logic check:");
    report_motion("motion check: L");

    if (s_bal.dry_run)
    {
        telemetry_printf("motion check: DRY-RUN, motors were never driven - judged=0 is expected; "
                         "unexp>0 means a wheel turned with the motors off\r\n");
    }
    else if ((0U == s_bal.motion_count_left.judged) && (0U == s_bal.motion_count_right.judged))
    {
        telemetry_printf("motion check: never driven >= %u permille for a whole window - no verdict\r\n",
                         (unsigned) BALANCE_MOTION_MIN_DUTY_PERMILLE);
    }
    else
    {
        /* Verdicts are in the counts above. */
    }
}

static HAL_NOINLINE void print_conventions (void)
{
    telemetry_printf("conventions: ang>0 = tilted forward, pwm>0 = wheel driven forward (FWD); "
                     "forward = the way 'jog' rolls the wheels\r\n");
    telemetry_printf("correct balance: tilted forward -> L=FWD R=FWD (wheels drive under the falling body)\r\n");
}

#endif

/* 10 Hz encoder/ultrasonic/battery/safety line on channel 0. Printed in three
 * parts (same text as one line) to keep each printf's argument block small. */
static HAL_NOINLINE void report_sensors_encoders (void)
{
    encoders_snapshot_t encoders;
    encoders_get_snapshot(&encoders);

    decimal2_t left_rps;
    decimal2_t right_rps;
    split_decimal2(encoders.left_rev_per_s, &left_rps);
    split_decimal2(encoders.right_rev_per_s, &right_rps);

    telemetry_printf("enc_L=%u enc_R=%u spd_L=%u/5ms(%s%d.%02u rps) spd_R=%u/5ms(%s%d.%02u rps) ",
                     (unsigned) encoders.left_total, (unsigned) encoders.right_total,
                     (unsigned) encoders.left_counts_per_5ms, left_rps.p_sign, left_rps.int_part, left_rps.frac_part,
                     (unsigned) encoders.right_counts_per_5ms, right_rps.p_sign, right_rps.int_part, right_rps.frac_part);
}

static HAL_NOINLINE void report_sensors_distance (void)
{
#if !ULTRASONIC_ENABLED
    telemetry_printf("ultrasonic=off ");
#else
    ultrasonic_snapshot_t ultrasonic;
    ultrasonic_get_snapshot(&ultrasonic);

    if (ultrasonic.distance_valid)
    {
        decimal2_t distance;
        split_decimal2(ultrasonic.distance_cm, &distance);
        telemetry_printf("dist_cm=%s%d.%02u ", distance.p_sign, distance.int_part, distance.frac_part);
    }
    else
    {
        telemetry_printf("dist_cm=no_echo ");
    }

    telemetry_printf("echo_timeouts=%u ", (unsigned) ultrasonic.timeout_count);
#endif
}

static HAL_NOINLINE void report_sensors_battery_safety (safety_state_t safety_state)
{
    battery_snapshot_t battery;
    battery_get_snapshot(&battery);

    decimal2_t battery_v;
    decimal2_t battery_raw_v;
    split_decimal2(battery.voltage_v, &battery_v);
    split_decimal2(battery.voltage_raw_v, &battery_raw_v);

    telemetry_printf("v_bat=%s%d.%02u v_bat_raw=%s%d.%02u safety=%s faults=0x%x warn=0x%x\r\n",
                     battery_v.p_sign, battery_v.int_part, battery_v.frac_part,
                     battery_raw_v.p_sign, battery_raw_v.int_part, battery_raw_v.frac_part,
                     safety_state_name(safety_state),
                     (unsigned) safety_get_fault_reasons(), (unsigned) safety_get_warnings());
}

static HAL_NOINLINE void report_imu (void)
{
    mpu6050_angle_t angle;
    mpu6050_get_angle(&angle);

    decimal2_t accel_deg;
    decimal2_t filt_deg;
    decimal2_t gyro_dps;
    split_decimal2(angle.accel_angle_deg, &accel_deg);
    split_decimal2(angle.filtered_angle_deg, &filt_deg);
    split_decimal2(angle.gyro_rate_dps, &gyro_dps);

    telemetry_printf("accel_deg=%s%d.%02u filt_deg=%s%d.%02u gyro_dps=%s%d.%02u ",
                     accel_deg.p_sign, accel_deg.int_part, accel_deg.frac_part,
                     filt_deg.p_sign, filt_deg.int_part, filt_deg.frac_part,
                     gyro_dps.p_sign, gyro_dps.int_part, gyro_dps.frac_part);
    telemetry_printf("read_us=%u ctrl_cycles=%u missed=%u failed=%u\r\n",
                     mpu6050_get_last_read_cycles() / CYCLES_PER_US,
                     (unsigned) mpu6050_get_last_control_step_cycles(),
                     (unsigned) mpu6050_get_missed_read_count(),
                     (unsigned) mpu6050_get_failed_read_count());
}

static HAL_NOINLINE void report_tick_timing (uint32_t * p_window_start_cyccnt)
{
    uint32_t const window_cycles = DWT->CYCCNT - *p_window_start_cyccnt;
    *p_window_start_cyccnt = DWT->CYCCNT;

    uint32_t min_cycles;
    uint32_t max_cycles;
    control_tick_get_interval_stats(&min_cycles, &max_cycles);
    control_tick_reset_interval_stats();

    telemetry_printf("ticks/s=1000 cycles/1000ticks=%u min_cycles=%u max_cycles=%u overruns=%u ",
                     window_cycles, min_cycles, max_cycles, (unsigned) control_tick_get_overrun_count());
    telemetry_printf("stack_free=%u/%u\r\n",
                     (unsigned) fault_stack_free_bytes(), (unsigned) fault_stack_size_bytes());
}

/*******************************************************************************************************************//**
 * @brief  TiltGuard entry point.
 *
 * Starts the 1 kHz control tick (GPT0), the MPU-6050 IMU, the wheel
 * encoders, the HC-SR04 ultrasonic sensor, the battery ADC, the motor
 * driver, S1, and the safety state machine. Motors stay disabled (STBY
 * low, duty 0) from boot regardless of safety state, until S1 arms them
 * from READY. Main loop: LED1 blinks at 1 Hz, LED2 reflects safety state
 * (on=FAULT, blinking=ACTIVE, off otherwise), a compact CSV line goes out
 * on RTT channel 1 at 100 Hz while armed, encoder/ultrasonic/battery/safety telemetry
 * reports on channel 0 at 10 Hz, IMU telemetry at 2 Hz, and tick-timing
 * telemetry at 1 Hz. On any init failure, LED2 is held on and the loop
 * halts in a safe state.
 **********************************************************************************************************************/
void hal_entry (void)
{
#if BSP_TZ_SECURE_BUILD

    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif

    /* LED type structure */
    bsp_leds_t leds = g_bsp_leds;

    /* Wake up 2nd core if this is first core and we are inside a multicore project. */
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    R_BSP_SecondaryCoreStart();
#endif

    /* If this board has no LEDs then trap here */
    if (0 == leds.led_count)
    {
        while (1)
        {
            ;                          // There are no LEDs on this board
        }
    }

    /* Motors first: STBY/AIN1/BIN1 low and both PWM duties 0 before any other
     * module runs. The pins are already driven low from R_BSP_WarmStart
     * (hal_warmstart.c); this puts the GPT channels into a known 0% state too.
     * The error is reported after telemetry_init(), so RTT can print it. */
    fsp_err_t const motors_err = motors_init();

    /* Fault handlers with an RTT dump, separate MemManage/Bus/UsageFault, and
     * the stack high-water paint - before anything else can fault. */
    fault_init();

    telemetry_init();
    telemetry_printf("build id: %s\r\n", g_build_id);

    {
        uint32_t const fpccr = fault_get_fpccr();

        telemetry_printf("boot: main stack=%u bytes, FPCCR=0x%08X (ASPEN=%u LSPEN=%u; both 1 = FP context saved in ISRs)\r\n",
                         (unsigned) fault_stack_size_bytes(),
                         (unsigned) fpccr,
                         (unsigned) ((fpccr >> 31U) & 1U),
                         (unsigned) ((fpccr >> 30U) & 1U));
    }

    if (FSP_SUCCESS != motors_err)
    {
        motors_disable();
        fault_halt(&leds, "motors", motors_err);
    }

    fsp_err_t err = mpu6050_init();

    telemetry_printf("i2c bus recovery: 0x%x (0..9 = clocks to free SDA, fffffffe = SDA stuck, "
                     "fffffffd = SCL stuck)\r\n",
                     (unsigned) mpu6050_get_bus_recovery_result());

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "mpu6050", err);
    }

    err = encoders_init();

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "encoders", err);
    }

    err = ultrasonic_init();

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "ultrasonic", err);
    }

    err = battery_init();

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "battery", err);
    }

    err = button_init();

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "button", err);
    }

    /* Not a safety function: on error it prints why and stays off (no halt). */
    (void) bt_remote_init();

    /* What survived the reset - before anything can start a new recording. */
    blackbox_init();

    /* Before the tick starts: the first IMU sample after control_tick_init()
     * already calls balance_step(). */
    balance_init();
#ifndef MOTOR_BENCH_TEST
    balance_set_enabled(true);
#endif

    err = control_tick_init();

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "control_tick", err);
    }

    safety_init();

#ifdef MOTOR_BENCH_TEST
    telemetry_printf("build: MOTOR_BENCH_TEST - S1 runs the bench sequence, balancing is off\r\n");
#else
    telemetry_printf("build: balance (stock ELEGOO law). Set it on the floor (resting on its support is fine), "
                     "press S1: it stands up and balances. S1 again disarms. After a fall: press S1 once.\r\n");
    telemetry_printf("start: S1 arms within +/-%d deg; cut-out +/-%d deg for the first 2 s (stand-up), then +/-%d deg\r\n",
                     (int) BALANCE_ARM_LIMIT_DEG, (int) SAFETY_STANDUP_TILT_LIMIT_DEG, (int) SAFETY_TILT_LIMIT_DEG);
    print_conventions();
    console_print_mode();
#endif
    console_print_params();
    console_print_speed_loop();

    bsp_io_level_t led1_last_written       = BSP_IO_LEVEL_LOW;
    bsp_io_level_t led2_last_written       = BSP_IO_LEVEL_LOW;
    bsp_io_level_t led2_blink_phase        = BSP_IO_LEVEL_LOW;
    uint32_t       last_tick_seen          = 0U;
    uint32_t       last_button_press_count = 0U;
    uint32_t       next_led2_blink_tick    = LED_BLINK_TICKS;
    uint32_t       next_csv_tick           = CSV_REPORT_TICKS;
    uint32_t       next_sensor_report_tick = SENSOR_REPORT_TICKS;
    uint32_t       next_imu_report_tick    = IMU_REPORT_TICKS;
    uint32_t       next_tick_report_tick   = TICK_REPORT_TICKS;
    uint32_t       window_start_cyccnt     = DWT->CYCCNT;
#ifndef MOTOR_BENCH_TEST
    safety_state_t prev_safety_state       = SAFETY_STATE_INIT;
    bool           prev_standup            = false;
#endif

    while (1)
    {
        uint32_t const tick = control_tick_count();

        if (tick == last_tick_seen)
        {
            continue;
        }

        last_tick_seen = tick;

        mpu6050_angle_t angle;
        mpu6050_get_angle(&angle);

        safety_update(angle.filtered_angle_deg,
                      mpu6050_get_sequence(),
                      mpu6050_get_failed_read_count(),
                      echo_timeout_count(),
                      control_tick_get_overrun_count(),
                      motors_get_error_count(),
                      balance_get_stall_fault_count());

        safety_state_t const safety_state = safety_get_state();

        uint32_t const button_press_count = button_get_press_count();

        if (button_press_count != last_button_press_count)
        {
            last_button_press_count = button_press_count;
            handle_button_press(safety_state, angle.filtered_angle_deg);
        }

        jog_update(tick, safety_state); /* Both builds: a jog must always time out. */
        drive_update(tick, safety_state); /* drv/trn deadman; clears both targets when not armed. */

#ifndef MOTOR_BENCH_TEST
        handle_pending_arm(angle.filtered_angle_deg);
#endif

#ifdef MOTOR_BENCH_TEST
        bench_step(tick, safety_state);
#else
        if ((SAFETY_STATE_ACTIVE == safety_state) && (SAFETY_STATE_ACTIVE != prev_safety_state))
        {
            report_csv_header(); /* One header per armed run on channel 1. */
        }
        else if ((SAFETY_STATE_ACTIVE != safety_state) && (SAFETY_STATE_ACTIVE == prev_safety_state))
        {
            freeze_blackbox(safety_state);
            report_disarmed(safety_state);
        }
        else if ((SAFETY_STATE_READY == safety_state) && (SAFETY_STATE_READY != prev_safety_state))
        {
            report_ready(angle.filtered_angle_deg);
        }
        else
        {
            /* No transition. */
        }

        bool const standup = safety_in_standup();

        if (prev_standup && (!standup) && (SAFETY_STATE_ACTIVE == safety_state))
        {
            report_standup_done(angle.filtered_angle_deg);
        }

        prev_standup      = standup;
        prev_safety_state = safety_state;
#endif

        /* LED1: 1 Hz heartbeat; 10 Hz for 2 s after S1 was refused (angle beyond the arming limit). */
        {
#ifndef MOTOR_BENCH_TEST
            bool const refused = ((int32_t) (s_led1_fast_until_tick - tick)) > 0;
#else
            bool const refused = false;
#endif
            uint32_t const       half_period  = refused ? LED_FAST_BLINK_TICKS : LED_BLINK_TICKS;
            bsp_io_level_t const led1_desired = (0U != ((tick / half_period) & 1U)) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

            if (led1_desired != led1_last_written)
            {
                led1_last_written = led1_desired;

                R_BSP_PinAccessEnable();
                R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[0], led1_desired);
                R_BSP_PinAccessDisable();
            }
        }

        /* LED2: on=FAULT, blinking=ACTIVE, off otherwise (READY/INIT/CALIBRATING). */
        if (SAFETY_STATE_ACTIVE != safety_state)
        {
            /* Hold the blink schedule at "now" so arming starts a clean blink
             * (on for 500 ms) instead of fast-toggling to catch up. */
            next_led2_blink_tick = tick;
            led2_blink_phase     = BSP_IO_LEVEL_LOW;
        }
        else if (tick >= next_led2_blink_tick)
        {
            next_led2_blink_tick += LED_BLINK_TICKS;
            led2_blink_phase       = (BSP_IO_LEVEL_LOW == led2_blink_phase) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
        }

        bsp_io_level_t led2_desired;

        if (SAFETY_STATE_FAULT == safety_state)
        {
            led2_desired = BSP_IO_LEVEL_HIGH;
        }
        else if (SAFETY_STATE_ACTIVE == safety_state)
        {
            led2_desired = led2_blink_phase;
        }
        else
        {
            led2_desired = BSP_IO_LEVEL_LOW;
        }

        if (led2_desired != led2_last_written)
        {
            led2_last_written = led2_desired;

            R_BSP_PinAccessEnable();
            R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[1], led2_desired);
            R_BSP_PinAccessDisable();
        }

        console_poll();
        bt_remote_poll();
        blackbox_dump_poll();

        /* Armed (balance law running, live or dry-run): the combined line
         * and the CSV replace the separate sensor/IMU lines. */
        bool const report_due = (tick >= next_csv_tick) || (tick >= next_sensor_report_tick) ||
                                (tick >= next_imu_report_tick) || (tick >= next_tick_report_tick);

        if (report_due)
        {
            balance_get_status(&s_bal);
        }

        bool const armed = s_bal.armed && (SAFETY_STATE_ACTIVE == safety_state);

        if (tick >= next_csv_tick)
        {
            next_csv_tick += CSV_REPORT_TICKS;

            if (armed)
            {
                report_armed_csv(tick);
            }
        }

        if (tick >= next_sensor_report_tick)
        {
            next_sensor_report_tick += SENSOR_REPORT_TICKS;

            if (armed)
            {
                report_armed_line();
            }
            else
            {
                report_sensors_encoders();
                report_sensors_distance();
                report_sensors_battery_safety(safety_state);
            }
        }

        if (tick >= next_imu_report_tick)
        {
            next_imu_report_tick += IMU_REPORT_TICKS;

            if (!armed)
            {
                report_imu();
            }
        }

        if (tick >= next_tick_report_tick)
        {
            next_tick_report_tick += TICK_REPORT_TICKS;
            report_tick_timing(&window_start_cyccnt);

            if (armed)
            {
                report_logic_check("check:");
                report_motion("motion: L");
            }
        }
    }
}
