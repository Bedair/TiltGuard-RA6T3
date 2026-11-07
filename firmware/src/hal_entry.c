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

extern bsp_leds_t g_bsp_leds;

/* 1 kHz tick thresholds. */
#define LED_BLINK_TICKS     (500U)  /* 1 Hz LED1 blink. */
#define SENSOR_REPORT_TICKS (100U)  /* 10 Hz encoder/ultrasonic/battery telemetry. */
#define IMU_REPORT_TICKS    (500U)  /* 2 Hz IMU telemetry - kept slower so the output stays readable. */
#define TICK_REPORT_TICKS   (1000U) /* 1 Hz tick-timing telemetry. */
#define CYCLES_PER_US       (200U)  /* Core clock is 200 MHz; see CLAUDE.md. */

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
 * encoders, the HC-SR04 ultrasonic sensor, and the battery ADC, then runs a
 * non-blocking main loop: LED1 blinks at 1 Hz, encoder/ultrasonic/battery
 * telemetry reports at 10 Hz, IMU telemetry at 2 Hz, and tick-timing
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

    telemetry_init();

    fsp_err_t err = mpu6050_init();

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

    err = control_tick_init();

    if (FSP_SUCCESS != err)
    {
        fault_halt(&leds, "control_tick", err);
    }

    bsp_io_level_t led1_level            = BSP_IO_LEVEL_LOW;
    uint32_t       last_tick_seen        = 0U;
    uint32_t       next_blink_tick       = LED_BLINK_TICKS;
    uint32_t       next_sensor_report_tick = SENSOR_REPORT_TICKS;
    uint32_t       next_imu_report_tick  = IMU_REPORT_TICKS;
    uint32_t       next_tick_report_tick = TICK_REPORT_TICKS;
    uint32_t       window_start_cyccnt   = DWT->CYCCNT;

    while (1)
    {
        uint32_t const tick = control_tick_count();

        mpu6050_update();

        if (tick == last_tick_seen)
        {
            continue;
        }

        last_tick_seen = tick;

        if (tick >= next_blink_tick)
        {
            next_blink_tick += LED_BLINK_TICKS;

            led1_level = (BSP_IO_LEVEL_LOW == led1_level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

            R_BSP_PinAccessEnable();
            R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[0], led1_level);
            R_BSP_PinAccessDisable();
        }

        if (tick >= next_sensor_report_tick)
        {
            next_sensor_report_tick += SENSOR_REPORT_TICKS;

            encoders_snapshot_t encoders;
            encoders_get_snapshot(&encoders);

            ultrasonic_snapshot_t ultrasonic;
            ultrasonic_get_snapshot(&ultrasonic);

            battery_snapshot_t battery;
            battery_get_snapshot(&battery);

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
                    "dist_cm=%s%d.%02u echo_timeouts=%u v_bat=%s%d.%02u v_bat_raw=%s%d.%02u\r\n",
                    (unsigned) encoders.left_total, (unsigned) encoders.right_total,
                    (unsigned) encoders.left_counts_per_5ms, left_rps.p_sign, left_rps.int_part, left_rps.frac_part,
                    (unsigned) encoders.right_counts_per_5ms, right_rps.p_sign, right_rps.int_part, right_rps.frac_part,
                    distance.p_sign, distance.int_part, distance.frac_part,
                    (unsigned) ultrasonic.timeout_count,
                    battery_v.p_sign, battery_v.int_part, battery_v.frac_part,
                    battery_raw_v.p_sign, battery_raw_v.int_part, battery_raw_v.frac_part);
            }
            else
            {
                telemetry_printf(
                    "enc_L=%u enc_R=%u spd_L=%u/5ms(%s%d.%02u rps) spd_R=%u/5ms(%s%d.%02u rps) "
                    "dist_cm=no_echo echo_timeouts=%u v_bat=%s%d.%02u v_bat_raw=%s%d.%02u\r\n",
                    (unsigned) encoders.left_total, (unsigned) encoders.right_total,
                    (unsigned) encoders.left_counts_per_5ms, left_rps.p_sign, left_rps.int_part, left_rps.frac_part,
                    (unsigned) encoders.right_counts_per_5ms, right_rps.p_sign, right_rps.int_part, right_rps.frac_part,
                    (unsigned) ultrasonic.timeout_count,
                    battery_v.p_sign, battery_v.int_part, battery_v.frac_part,
                    battery_raw_v.p_sign, battery_raw_v.int_part, battery_raw_v.frac_part);
            }
        }

        if (tick >= next_imu_report_tick)
        {
            next_imu_report_tick += IMU_REPORT_TICKS;

            mpu6050_angle_t angle;
            mpu6050_get_angle(&angle);

            decimal2_t accel_deg;
            decimal2_t filt_deg;
            decimal2_t gyro_dps;
            split_decimal2(angle.accel_angle_deg, &accel_deg);
            split_decimal2(angle.filtered_angle_deg, &filt_deg);
            split_decimal2(angle.gyro_rate_dps, &gyro_dps);

            telemetry_printf(
                "accel_deg=%s%d.%02u filt_deg=%s%d.%02u gyro_dps=%s%d.%02u read_us=%u missed=%u failed=%u\r\n",
                accel_deg.p_sign, accel_deg.int_part, accel_deg.frac_part,
                filt_deg.p_sign, filt_deg.int_part, filt_deg.frac_part,
                gyro_dps.p_sign, gyro_dps.int_part, gyro_dps.frac_part,
                mpu6050_get_last_read_cycles() / CYCLES_PER_US,
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

            telemetry_printf("ticks/s=1000 cycles/1000ticks=%u min_cycles=%u max_cycles=%u\r\n",
                              window_cycles,
                              min_cycles,
                              max_cycles);
        }
    }
}
