/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hal_data.h"
#include "telemetry/telemetry.h"
#include "control/control_tick.h"

extern bsp_leds_t g_bsp_leds;

/* 1 kHz tick thresholds: LED1 toggles every 500 ticks (1 Hz), telemetry reports every 1000 ticks (1 Hz). */
#define LED_BLINK_TICKS   (500U)
#define REPORT_TICKS      (1000U)
#define CYCLES_PER_US     (200U) /* Core clock is 200 MHz; see CLAUDE.md. */

/*******************************************************************************************************************//**
 * @brief  TiltGuard entry point.
 *
 * Starts the 1 kHz control tick (GPT0) and runs a non-blocking main loop that
 * blinks LED1 at 1 Hz and reports tick-timing telemetry once per second.
 * On any control_tick_init() failure, LED2 is held on and the loop halts in
 * a safe state rather than running without a trustworthy tick.
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

    fsp_err_t err = control_tick_init();

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

    bsp_io_level_t led1_level     = BSP_IO_LEVEL_LOW;
    uint32_t       last_tick_seen = 0U;
    uint32_t       next_blink_tick  = LED_BLINK_TICKS;
    uint32_t       next_report_tick = REPORT_TICKS;
    uint32_t       window_start_cyccnt = DWT->CYCCNT;

    while (1)
    {
        uint32_t const tick = control_tick_count();

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

        if (tick >= next_report_tick)
        {
            next_report_tick += REPORT_TICKS;

            uint32_t const window_cycles = DWT->CYCCNT - window_start_cyccnt;
            window_start_cyccnt = DWT->CYCCNT;

            uint32_t min_cycles;
            uint32_t max_cycles;
            control_tick_get_interval_stats(&min_cycles, &max_cycles);
            control_tick_reset_interval_stats();

            telemetry_printf("ticks/s=1000 cycles/1000ticks=%u min_us=%u max_us=%u\r\n",
                              window_cycles,
                              min_cycles / CYCLES_PER_US,
                              max_cycles / CYCLES_PER_US);
        }
    }
}
