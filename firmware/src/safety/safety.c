/*
 * safety.c
 */

#include <math.h>
#include "safety.h"
#include "../actuators/motors.h"

static volatile safety_state_t s_state; /* Read from the balance step (ISR). */
static uint32_t        s_fault_reasons;
static uint32_t        s_warnings;
static bool            s_active_requested;

static uint32_t s_last_imu_sequence;
static uint32_t s_ticks_since_imu_update;
static uint32_t s_last_imu_failed_read_count;
static uint32_t s_last_echo_timeout_count;
static uint32_t s_last_tick_overrun_count;
static uint32_t s_last_motor_error_count;
static uint32_t s_last_stall_fault_count;
static uint32_t s_calibration_ticks_remaining;
static uint32_t s_active_ticks; /* Ticks since entering ACTIVE, saturating at SAFETY_STANDUP_TICKS. */

/* The counters passed to safety_update() are monotonic totals since boot,
 * not since the last reset. After init/reset, the first update only records
 * them as the baseline - otherwise any failure that happened before a
 * safety_reset() would look like a new increment and re-latch FAULT
 * immediately, making FAULT unrecoverable without a reboot. */
static bool s_baseline_pending;

void safety_init (void)
{
    s_state                      = SAFETY_STATE_INIT;
    s_fault_reasons               = 0U;
    s_warnings                    = 0U;
    s_active_requested             = false;
    s_last_imu_sequence           = 0U;
    s_ticks_since_imu_update      = 0U;
    s_last_imu_failed_read_count  = 0U;
    s_last_echo_timeout_count     = 0U;
    s_last_tick_overrun_count     = 0U;
    s_last_motor_error_count      = 0U;
    s_last_stall_fault_count      = 0U;
    s_calibration_ticks_remaining = SAFETY_CALIBRATION_TICKS;
    s_active_ticks                = 0U;
    s_baseline_pending            = true;
}

void safety_request_active (void)
{
    s_active_requested = true;
}

void safety_disarm (void)
{
    if (SAFETY_STATE_ACTIVE == s_state)
    {
        s_state             = SAFETY_STATE_READY;
        s_active_requested  = false; /* Otherwise the READY case below re-arms on the very next tick. */
        motors_disable();
    }
}

void safety_reset (void)
{
    safety_init();
}

safety_state_t safety_get_state (void)
{
    return s_state;
}

bool safety_in_standup (void)
{
    return (SAFETY_STATE_ACTIVE == s_state) && (s_active_ticks < SAFETY_STANDUP_TICKS);
}

uint32_t safety_get_fault_reasons (void)
{
    return s_fault_reasons;
}

uint32_t safety_get_warnings (void)
{
    return s_warnings;
}

void safety_update (float    tilt_angle_deg,
                     uint32_t imu_sequence,
                     uint32_t imu_failed_read_count,
                     uint32_t echo_timeout_count,
                     uint32_t tick_overrun_count,
                     uint32_t motor_error_count,
                     uint32_t stall_fault_count)
{
    if (s_baseline_pending)
    {
        s_baseline_pending           = false;
        s_last_imu_sequence          = imu_sequence;
        s_last_imu_failed_read_count = imu_failed_read_count;
        s_last_echo_timeout_count    = echo_timeout_count;
        s_last_tick_overrun_count    = tick_overrun_count;
        s_last_motor_error_count     = motor_error_count;
        s_last_stall_fault_count     = stall_fault_count;
    }

    if (imu_sequence != s_last_imu_sequence)
    {
        s_last_imu_sequence      = imu_sequence;
        s_ticks_since_imu_update = 0U;
    }
    else
    {
        s_ticks_since_imu_update++;
    }

    bool const imu_failed_this_tick = (imu_failed_read_count != s_last_imu_failed_read_count);
    s_last_imu_failed_read_count = imu_failed_read_count;

    bool const echo_timed_out_this_tick = (echo_timeout_count != s_last_echo_timeout_count);
    s_last_echo_timeout_count = echo_timeout_count;

    bool const tick_overrun_this_tick = (tick_overrun_count != s_last_tick_overrun_count);
    s_last_tick_overrun_count = tick_overrun_count;

    bool const motor_error_this_tick = (motor_error_count != s_last_motor_error_count);
    s_last_motor_error_count = motor_error_count;

    bool const stall_this_tick = (stall_fault_count != s_last_stall_fault_count);
    s_last_stall_fault_count = stall_fault_count;

    s_warnings = echo_timed_out_this_tick ? SAFETY_WARNING_ECHO_TIMEOUT : 0U;

    uint32_t new_faults = 0U;

    if (SAFETY_STATE_ACTIVE == s_state)
    {
        /* See SAFETY_STANDUP_* in safety.h: wider limit for the first 2 s. */
        float const tilt_limit_deg = (s_active_ticks < SAFETY_STANDUP_TICKS) ? SAFETY_STANDUP_TILT_LIMIT_DEG
                                                                             : SAFETY_TILT_LIMIT_DEG;

        if (s_active_ticks < SAFETY_STANDUP_TICKS)
        {
            s_active_ticks++;
        }

        if (fabsf(tilt_angle_deg) > tilt_limit_deg)
        {
            new_faults |= SAFETY_FAULT_TILT;
        }
    }

    if (imu_failed_this_tick || (s_ticks_since_imu_update > SAFETY_IMU_STALE_TICKS))
    {
        new_faults |= SAFETY_FAULT_IMU_STALE;
    }

    if (tick_overrun_this_tick)
    {
        new_faults |= SAFETY_FAULT_TICK_OVERRUN;
    }

    if (motor_error_this_tick)
    {
        new_faults |= SAFETY_FAULT_ACTUATOR;
    }

    if (stall_this_tick)
    {
        new_faults |= SAFETY_FAULT_ENCODER;
    }

    if (0U != new_faults)
    {
        s_fault_reasons |= new_faults;

        if (SAFETY_STATE_FAULT != s_state)
        {
            s_state = SAFETY_STATE_FAULT;
            motors_disable();
        }

        return;
    }

    switch (s_state)
    {
        case SAFETY_STATE_INIT:
        {
            s_state                      = SAFETY_STATE_CALIBRATING;
            s_calibration_ticks_remaining = SAFETY_CALIBRATION_TICKS;
            break;
        }

        case SAFETY_STATE_CALIBRATING:
        {
            if (s_calibration_ticks_remaining > 0U)
            {
                s_calibration_ticks_remaining--;
            }
            else
            {
                s_state = SAFETY_STATE_READY;
            }

            break;
        }

        case SAFETY_STATE_READY:
        {
            if (s_active_requested)
            {
                s_active_requested = false;
                s_active_ticks     = 0U; /* Stand-up window starts now. */
                s_state            = SAFETY_STATE_ACTIVE;
            }

            break;
        }

        case SAFETY_STATE_ACTIVE:
        case SAFETY_STATE_FAULT:
        default:
        {
            break;
        }
    }
}
