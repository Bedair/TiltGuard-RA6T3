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

extern bsp_leds_t g_bsp_leds;

/* 1 kHz tick thresholds. */
#define LED_BLINK_TICKS     (500U)  /* 1 Hz LED1 heartbeat, and LED2's ACTIVE blink rate. */
#define CSV_REPORT_TICKS    (5U)    /* 200 Hz compact CSV on RTT channel 1. */
#define SENSOR_REPORT_TICKS (100U)  /* 10 Hz encoder/ultrasonic/battery/safety telemetry. */
#define IMU_REPORT_TICKS    (500U)  /* 2 Hz IMU telemetry - kept slower so the output stays readable. */
#define TICK_REPORT_TICKS   (1000U) /* 1 Hz tick-timing telemetry. */
#define CYCLES_PER_US       (200U)  /* Core clock is 200 MHz; see CLAUDE.md. */

/*
 * Bench-test mode: uncomment to enable. Only runs after arming (S1 in
 * READY); one motor at a time, max 15% duty, then auto-disarms. Without
 * this defined, ACTIVE does nothing to the motors at all - arming just
 * means "ready for a control law that doesn't exist yet" (Phase 3).
 */
#define MOTOR_BENCH_TEST /* Phase 2 bring-up. Comment out again before the Phase 3 control law. */

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

/*******************************************************************************************************************//**
 * @brief  TiltGuard entry point.
 *
 * Starts the 1 kHz control tick (GPT0), the MPU-6050 IMU, the wheel
 * encoders, the HC-SR04 ultrasonic sensor, the battery ADC, the motor
 * driver, S1, and the safety state machine. Motors stay disabled (STBY
 * low, duty 0) from boot regardless of safety state, until S1 arms them
 * from READY. Main loop: LED1 blinks at 1 Hz, LED2 reflects safety state
 * (on=FAULT, blinking=ACTIVE, off otherwise), a compact CSV line goes out
 * on RTT channel 1 at 200 Hz, encoder/ultrasonic/battery/safety telemetry
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

    telemetry_init();

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

    err = control_tick_init();

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "control_tick", err);
    }

    safety_init();

    bsp_io_level_t led1_level              = BSP_IO_LEVEL_LOW;
    bsp_io_level_t led2_last_written       = BSP_IO_LEVEL_LOW;
    bsp_io_level_t led2_blink_phase        = BSP_IO_LEVEL_LOW;
    uint32_t       last_tick_seen          = 0U;
    uint32_t       last_button_press_count = 0U;
    uint32_t       next_blink_tick         = LED_BLINK_TICKS;
    uint32_t       next_led2_blink_tick    = LED_BLINK_TICKS;
    uint32_t       next_csv_tick           = CSV_REPORT_TICKS;
    uint32_t       next_sensor_report_tick = SENSOR_REPORT_TICKS;
    uint32_t       next_imu_report_tick    = IMU_REPORT_TICKS;
    uint32_t       next_tick_report_tick   = TICK_REPORT_TICKS;
    uint32_t       window_start_cyccnt     = DWT->CYCCNT;

#ifdef MOTOR_BENCH_TEST
    bench_phase_t  bench_phase            = BENCH_PHASE_LEFT_FWD;
    uint32_t       bench_phase_start_tick = 0U;
    safety_state_t prev_safety_state      = SAFETY_STATE_INIT;
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

        encoders_snapshot_t encoders;
        encoders_get_snapshot(&encoders);

        ultrasonic_snapshot_t ultrasonic;
        ultrasonic_get_snapshot(&ultrasonic);

        battery_snapshot_t battery;
        battery_get_snapshot(&battery);

        safety_update(angle.filtered_angle_deg,
                      mpu6050_get_sequence(),
                      mpu6050_get_failed_read_count(),
                      ultrasonic.timeout_count,
                      control_tick_get_overrun_count());

        safety_state_t const safety_state = safety_get_state();

        /* S1: short press in READY arms, in ACTIVE disarms, in FAULT resets
         * (back to INIT - CALIBRATING's settle window runs again before
         * READY). Debounced in button.c; this just reacts to the count. */
        uint32_t const button_press_count = button_get_press_count();

        if (button_press_count != last_button_press_count)
        {
            last_button_press_count = button_press_count;

            switch (safety_state)
            {
                case SAFETY_STATE_READY:
                {
                    safety_request_active();
                    break;
                }

                case SAFETY_STATE_ACTIVE:
                {
                    safety_disarm();
                    break;
                }

                case SAFETY_STATE_FAULT:
                {
                    safety_reset();
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

        if ((SAFETY_STATE_ACTIVE == safety_state) && (SAFETY_STATE_ACTIVE != prev_safety_state))
        {
            /* Just armed - (re)start the bench sequence from the top. */
            bench_phase            = BENCH_PHASE_LEFT_FWD;
            bench_phase_start_tick = tick;
            telemetry_printf("bench test: %s\r\n", bench_phase_name(bench_phase));
        }

        if (SAFETY_STATE_ACTIVE == safety_state)
        {
            if ((tick - bench_phase_start_tick) >= BENCH_PHASE_TICKS)
            {
                bench_phase            = (bench_phase_t) (bench_phase + 1);
                bench_phase_start_tick = tick;

                if (BENCH_PHASE_DONE != bench_phase)
                {
                    telemetry_printf("bench test: %s\r\n", bench_phase_name(bench_phase));
                }
            }

            switch (bench_phase)
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

        prev_safety_state = safety_state;
#endif

        if (tick >= next_blink_tick)
        {
            next_blink_tick += LED_BLINK_TICKS;

            led1_level = (BSP_IO_LEVEL_LOW == led1_level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

            R_BSP_PinAccessEnable();
            R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[0], led1_level);
            R_BSP_PinAccessDisable();
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

        if (tick >= next_csv_tick)
        {
            next_csv_tick += CSV_REPORT_TICKS;

            int16_t motor_left_permille;
            int16_t motor_right_permille;
            motors_get_command(&motor_left_permille, &motor_right_permille);

            telemetry_data_printf(
                "%u,%u,%d,%d,%d,%d,%u,%u,%u\r\n",
                tick,
                (unsigned) mpu6050_get_sequence(),
                (int) (angle.filtered_angle_deg * 100.0f),
                (int) (angle.gyro_rate_dps * 100.0f),
                (int) motor_left_permille,
                (int) motor_right_permille,
                (unsigned) encoders.left_counts_per_5ms,
                (unsigned) encoders.right_counts_per_5ms,
                (unsigned) safety_state);
        }

        if (tick >= next_sensor_report_tick)
        {
            next_sensor_report_tick += SENSOR_REPORT_TICKS;

            decimal2_t left_rps;
            decimal2_t right_rps;
            decimal2_t battery_v;
            decimal2_t battery_raw_v;
            split_decimal2(encoders.left_rev_per_s, &left_rps);
            split_decimal2(encoders.right_rev_per_s, &right_rps);
            split_decimal2(battery.voltage_v, &battery_v);
            split_decimal2(battery.voltage_raw_v, &battery_raw_v);

            if (ultrasonic.distance_valid)
            {
                decimal2_t distance;
                split_decimal2(ultrasonic.distance_cm, &distance);

                telemetry_printf(
                    "enc_L=%u enc_R=%u spd_L=%u/5ms(%s%d.%02u rps) spd_R=%u/5ms(%s%d.%02u rps) "
                    "dist_cm=%s%d.%02u echo_timeouts=%u v_bat=%s%d.%02u v_bat_raw=%s%d.%02u "
                    "safety=%s faults=0x%x warn=0x%x\r\n",
                    (unsigned) encoders.left_total, (unsigned) encoders.right_total,
                    (unsigned) encoders.left_counts_per_5ms, left_rps.p_sign, left_rps.int_part, left_rps.frac_part,
                    (unsigned) encoders.right_counts_per_5ms, right_rps.p_sign, right_rps.int_part, right_rps.frac_part,
                    distance.p_sign, distance.int_part, distance.frac_part,
                    (unsigned) ultrasonic.timeout_count,
                    battery_v.p_sign, battery_v.int_part, battery_v.frac_part,
                    battery_raw_v.p_sign, battery_raw_v.int_part, battery_raw_v.frac_part,
                    safety_state_name(safety_state),
                    (unsigned) safety_get_fault_reasons(), (unsigned) safety_get_warnings());
            }
            else
            {
                telemetry_printf(
                    "enc_L=%u enc_R=%u spd_L=%u/5ms(%s%d.%02u rps) spd_R=%u/5ms(%s%d.%02u rps) "
                    "dist_cm=no_echo echo_timeouts=%u v_bat=%s%d.%02u v_bat_raw=%s%d.%02u "
                    "safety=%s faults=0x%x warn=0x%x\r\n",
                    (unsigned) encoders.left_total, (unsigned) encoders.right_total,
                    (unsigned) encoders.left_counts_per_5ms, left_rps.p_sign, left_rps.int_part, left_rps.frac_part,
                    (unsigned) encoders.right_counts_per_5ms, right_rps.p_sign, right_rps.int_part, right_rps.frac_part,
                    (unsigned) ultrasonic.timeout_count,
                    battery_v.p_sign, battery_v.int_part, battery_v.frac_part,
                    battery_raw_v.p_sign, battery_raw_v.int_part, battery_raw_v.frac_part,
                    safety_state_name(safety_state),
                    (unsigned) safety_get_fault_reasons(), (unsigned) safety_get_warnings());
            }
        }

        if (tick >= next_imu_report_tick)
        {
            next_imu_report_tick += IMU_REPORT_TICKS;

            decimal2_t accel_deg;
            decimal2_t filt_deg;
            decimal2_t gyro_dps;
            split_decimal2(angle.accel_angle_deg, &accel_deg);
            split_decimal2(angle.filtered_angle_deg, &filt_deg);
            split_decimal2(angle.gyro_rate_dps, &gyro_dps);

            telemetry_printf(
                "accel_deg=%s%d.%02u filt_deg=%s%d.%02u gyro_dps=%s%d.%02u read_us=%u ctrl_cycles=%u missed=%u failed=%u\r\n",
                accel_deg.p_sign, accel_deg.int_part, accel_deg.frac_part,
                filt_deg.p_sign, filt_deg.int_part, filt_deg.frac_part,
                gyro_dps.p_sign, gyro_dps.int_part, gyro_dps.frac_part,
                mpu6050_get_last_read_cycles() / CYCLES_PER_US,
                (unsigned) mpu6050_get_last_control_step_cycles(),
                (unsigned) mpu6050_get_missed_read_count(),
                (unsigned) mpu6050_get_failed_read_count());
        }

        if (tick >= next_tick_report_tick)
        {
            next_tick_report_tick += TICK_REPORT_TICKS;

            uint32_t const window_cycles = DWT->CYCCNT - window_start_cyccnt;
            window_start_cyccnt = DWT->CYCCNT;

            uint32_t min_cycles;
            uint32_t max_cycles;
            control_tick_get_interval_stats(&min_cycles, &max_cycles);
            control_tick_reset_interval_stats();

            telemetry_printf("ticks/s=1000 cycles/1000ticks=%u min_cycles=%u max_cycles=%u overruns=%u\r\n",
                              window_cycles,
                              min_cycles,
                              max_cycles,
                              (unsigned) control_tick_get_overrun_count());
        }
    }
}
