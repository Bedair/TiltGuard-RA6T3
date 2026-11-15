/*
 * drive.c
 *
 * See drive.h. Owns the speed/turn targets and their deadman timers, and
 * hands them to balance.c with one masked copy (balance_set_targets()).
 */

#include "drive.h"
#include "balance.h"
#include "control_tick.h"
#include "../telemetry/telemetry.h"

typedef struct st_drive_axis
{
    int32_t  value;        /* Current target, 0 when idle. */
    uint32_t hold_ticks;   /* How long the current value holds. */
    uint32_t expire_tick;  /* Tick at which a non-zero value returns to 0. */
} drive_axis_t;

static drive_axis_t s_speed;
static drive_axis_t s_turn;

static void drive_publish (void)
{
    balance_set_targets((float) s_speed.value, (float) s_turn.value);
}

static bool drive_in_range (int32_t value, int32_t max, const char * p_name)
{
    if ((value < -max) || (value > max))
    {
        telemetry_printf("%s refused: %d is outside -%d..+%d\r\n", p_name, (int) value, (int) max, (int) max);

        return false;
    }

    return true;
}

static bool drive_armed (const char * p_name)
{
    if (SAFETY_STATE_ACTIVE != safety_get_state())
    {
        telemetry_printf("%s refused: not armed - press S1 first\r\n", p_name);

        return false;
    }

    return true;
}

static void drive_axis_set (drive_axis_t * p_axis, int32_t value, uint32_t hold_ticks, uint32_t now)
{
    p_axis->value       = value;
    p_axis->hold_ticks  = hold_ticks;
    p_axis->expire_tick = now + hold_ticks;
}

void drive_command_speed (int32_t speed)
{
    if (drive_armed("drv") && drive_in_range(speed, DRIVE_SPEED_MAX, "drv"))
    {
        drive_axis_set(&s_speed, speed, DRIVE_HOLD_TICKS, control_tick_count());
        drive_publish();
        telemetry_printf("drv: speed target %d counts/40 ms for %u ms (+ = FWD; resend to keep going, 'stop' to stop)\r\n",
                         (int) speed, (unsigned) DRIVE_HOLD_TICKS);
    }
}

void drive_command_turn (int32_t turn)
{
    if (drive_armed("trn") && drive_in_range(turn, DRIVE_TURN_MAX, "trn"))
    {
        drive_axis_set(&s_turn, turn, DRIVE_HOLD_TICKS, control_tick_count());
        drive_publish();
        telemetry_printf("trn: turn offset %d (8-bit units, + = right wheel faster) for %u ms\r\n",
                         (int) turn, (unsigned) DRIVE_HOLD_TICKS);
    }
}

bool drive_command (int32_t speed, int32_t turn, uint32_t hold_ticks, const char * p_source)
{
    if (!drive_armed(p_source) || !drive_in_range(speed, DRIVE_SPEED_MAX, p_source) ||
        !drive_in_range(turn, DRIVE_TURN_MAX, p_source))
    {
        return false;
    }

    uint32_t const now = control_tick_count();

    drive_axis_set(&s_speed, speed, hold_ticks, now);
    drive_axis_set(&s_turn, turn, hold_ticks, now);
    drive_publish();
    telemetry_printf("%s: speed %+d turn %+d (hold %u ms max)\r\n", p_source, (int) speed, (int) turn,
                     (unsigned) hold_ticks);

    return true;
}

void drive_stop (void)
{
    s_speed.value = 0;
    s_turn.value  = 0;
    drive_publish();
    telemetry_printf("stop: speed and turn targets 0\r\n");
}

static void drive_expire (drive_axis_t * p_axis, uint32_t tick, const char * p_name)
{
    if ((0 != p_axis->value) && (((int32_t) (tick - p_axis->expire_tick)) >= 0))
    {
        p_axis->value = 0;
        drive_publish();
        telemetry_printf("%s: back to 0 (%u ms hold over)\r\n", p_name, (unsigned) p_axis->hold_ticks);
    }
}

void drive_update (uint32_t tick, safety_state_t safety_state)
{
    if (SAFETY_STATE_ACTIVE != safety_state)
    {
        if ((0 != s_speed.value) || (0 != s_turn.value))
        {
            s_speed.value = 0;
            s_turn.value  = 0;
            drive_publish();
        }

        return;
    }

    drive_expire(&s_speed, tick, "drv");
    drive_expire(&s_turn, tick, "trn");
}
