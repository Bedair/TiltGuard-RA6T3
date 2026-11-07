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

extern bsp_leds_t g_bsp_leds;

/* 1 kHz tick thresholds. */
#define LED_BLINK_TICKS    (500U)  /* 1 Hz LED1 blink. */
#define IMU_REPORT_TICKS   (100U)  /* 10 Hz IMU telemetry. */
#define TICK_REPORT_TICKS  (1000U) /* 1 Hz tick-timing telemetry. */
#define CYCLES_PER_US      (200U)  /* Core clock is 200 MHz; see CLAUDE.md. */

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

static decimal2_t split_decimal2 (float value)
{
    decimal2_t    d;
    float const   abs_value = fabsf(value);

    d.p_sign    = (value < 0.0f) ? "-" : "";
    d.int_part  = (int) abs_value;
    d.frac_part = (unsigned) (((abs_value - (float) d.int_part) * 100.0f) + 0.5f);

    if (d.frac_part >= 100U)
    {
        d.frac_part = 0U;
        d.int_part += 1;
    }

    return d;
}

/*******************************************************************************************************************//**
 * @brief  TiltGuard entry point.
 *
 * Starts the 1 kHz control tick (GPT0) and the MPU-6050 IMU, then runs a
 * non-blocking main loop: LED1 blinks at 1 Hz, IMU angle/rate telemetry
 * reports at 10 Hz, and tick-timing telemetry reports at 1 Hz. On any
 * init failure, LED2 is held on and the loop halts in a safe state.
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
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[1], BSP_IO_LEVEL_HIGH);
        R_BSP_PinAccessDisable();

        telemetry_printf("mpu6050_init failed: err=%d\r\n", (int) err);

        while (1)
        {
            ;
        }
    }

    err = control_tick_init();

    if (FSP_SUCCESS != err)
    {
        /* Safe state: no trustworthy tick, so hold LED2 on and stop here rather than run degraded. */
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[1], BSP_IO_LEVEL_HIGH);
        R_BSP_PinAccessDisable();

        telemetry_printf("control_tick_init failed: err=%d\r\n", (int) err);

        while (1)
        {
            ;
        }
    }

    bsp_io_level_t led1_level          = BSP_IO_LEVEL_LOW;
    uint32_t       last_tick_seen      = 0U;
    uint32_t       next_blink_tick     = LED_BLINK_TICKS;
    uint32_t       next_imu_report_tick = IMU_REPORT_TICKS;
    uint32_t       next_tick_report_tick = TICK_REPORT_TICKS;
    uint32_t       window_start_cyccnt = DWT->CYCCNT;

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

        if (tick >= next_imu_report_tick)
        {
            next_imu_report_tick += IMU_REPORT_TICKS;

            mpu6050_angle_t angle;
            mpu6050_get_angle(&angle);

            decimal2_t const accel_deg = split_decimal2(angle.accel_angle_deg);
            decimal2_t const filt_deg  = split_decimal2(angle.filtered_angle_deg);
            decimal2_t const gyro_dps  = split_decimal2(angle.gyro_rate_dps);

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
