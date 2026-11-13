/*
 * safety.c
 */

#include <math.h>
#include "safety.h"
#include "../actuators/motors.h"

static safety_state_t s_state;
static uint32_t        s_fault_reasons;
static uint32_t        s_warnings;
static bool            s_active_requested;

static uint32_t s_last_imu_sequence;
static uint32_t s_ticks_since_imu_update;
static uint32_t s_last_imu_failed_read_count;
static uint32_t s_last_echo_timeout_count;
static uint32_t s_last_tick_overrun_count;
static uint32_t s_calibration_ticks_remaining;

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
    s_calibration_ticks_remaining = SAFETY_CALIBRATION_TICKS;
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
                     uint32_t tick_overrun_count)
{
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

    s_warnings = echo_timed_out_this_tick ? SAFETY_WARNING_ECHO_TIMEOUT : 0U;

    uint32_t new_faults = 0U;

    if (fabsf(tilt_angle_deg) > SAFETY_TILT_LIMIT_DEG)
    {
        new_faults |= SAFETY_FAULT_TILT;
    }

    if (imu_failed_this_tick || (s_ticks_since_imu_update > SAFETY_IMU_STALE_TICKS))
    {
        new_faults |= SAFETY_FAULT_IMU_STALE;
    }

    if (tick_overrun_this_tick)
    {
        new_faults |= SAFETY_FAULT_TICK_OVERRUN;
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
                s_state = SAFETY_STATE_ACTIVE;
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
