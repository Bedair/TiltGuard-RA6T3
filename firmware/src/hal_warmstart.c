/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "hal_data.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);

FSP_CPP_FOOTER

/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart (bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
        /*
         * TiltGuard: drive every motor-control line low as early as possible
         * after reset. Out of reset these pins are Hi-Z inputs, and the BSS138
         * shifter's LV-side 10 k pull-ups would otherwise pull them high, which
         * the TB6612FNG sees as STBY/PWM/IN high and spins the motors (see
         * docs/hardware-findings.md, "Motor lines float high during reset").
         *
         * This runs before clock init and before the C runtime, so it touches
         * the port registers directly (PCNTR1 is not write-protected, unlike
         * PmnPFS). Out of reset PODR = 0, so setting PDR makes each pin a
         * low-driven output. R_IOPORT_Open() at BSP_WARM_START_POST_C then
         * applies the configured pin setup (STBY/AIN1/BIN1 output-low, PWMA/PWMB
         * to GPT with duty 0). It cannot cover the time while RES# is held low;
         * the external pull-down on the LV side handles that.
         */
        uint32_t const port0_mask = (1UL << 6U) | (1UL << 8U);   /* P006 STBY, P008 AIN1. */
        uint32_t const port1_mask = (1UL << 10U) | (1UL << 13U); /* P110 BIN1, P113 PWMB. */
        uint32_t const port4_mask = (1UL << 9U);                 /* P409 PWMA. */

        R_PORT0->PCNTR1 = (R_PORT0->PCNTR1 & ~(port0_mask << 16U)) | port0_mask;
        R_PORT1->PCNTR1 = (R_PORT1->PCNTR1 & ~(port1_mask << 16U)) | port1_mask;
        R_PORT4->PCNTR1 = (R_PORT4->PCNTR1 & ~(port4_mask << 16U)) | port4_mask;

#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

#if BSP_CFG_OSPI_B_STARTUP_ENABLED && defined(BSP_CFG_OSPI_B_STARTUP_FN)
    if (BSP_WARM_START_POST_CLOCK == event)
    {
        /* Setup OSPI_B SiP flash and initialize it. */
        R_BSP_OspiBInit(BSP_CFG_OSPI_B_STARTUP_FN, true);
    }
#endif

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open(&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);

#if BSP_CFG_SDRAM_ENABLED

        /* Setup SDRAM and initialize it. Must configure pins first. */
        R_BSP_SdramInit(true);
#endif
    }
}
