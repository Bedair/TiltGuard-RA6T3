/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = gpt_counter_overflow_isr, /* GPT0 COUNTER OVERFLOW (Overflow) */
            [1] = i3c_rcv_isr, /* I3C0 RCV STATUS (Receive status buffer full) */
            [2] = i3c_resp_isr, /* I3C0 RESPONSE (Response status buffer full) */
            [3] = i3c_rx_isr, /* I3C0 RX (Receive) */
            [4] = i3c_tx_isr, /* I3C0 TX (Transmit) */
            [5] = i3c_ibi_isr, /* I3C0 IBI (IBI status buffer full) */
            [6] = i3c_eei_isr, /* I3C0 EEI (Error) */
            [7] = r_icu_isr, /* ICU IRQ7 (External pin interrupt 7) */
            [8] = r_icu_isr, /* ICU IRQ1 (External pin interrupt 1) */
            [9] = r_icu_isr, /* ICU IRQ5 (External pin interrupt 5) */
            [10] = adc_scan_end_isr, /* ADC0 SCAN END (End of A/D scanning operation) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_GPT0_COUNTER_OVERFLOW,GROUP0), /* GPT0 COUNTER OVERFLOW (Overflow) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_I3C0_RCV_STATUS,GROUP1), /* I3C0 RCV STATUS (Receive status buffer full) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_I3C0_RESPONSE,GROUP2), /* I3C0 RESPONSE (Response status buffer full) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_I3C0_RX,GROUP3), /* I3C0 RX (Receive) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_I3C0_TX,GROUP4), /* I3C0 TX (Transmit) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_I3C0_IBI,GROUP5), /* I3C0 IBI (IBI status buffer full) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_I3C0_EEI,GROUP6), /* I3C0 EEI (Error) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ7,GROUP7), /* ICU IRQ7 (External pin interrupt 7) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ1,GROUP0), /* ICU IRQ1 (External pin interrupt 1) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ5,GROUP1), /* ICU IRQ5 (External pin interrupt 5) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_ADC0_SCAN_END,GROUP2), /* ADC0 SCAN END (End of A/D scanning operation) */
        };
        #endif
        #endif
