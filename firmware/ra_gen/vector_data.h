/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (15)
#endif
/* ISR prototypes */
void gpt_counter_overflow_isr(void);
void i3c_rcv_isr(void);
void i3c_resp_isr(void);
void i3c_rx_isr(void);
void i3c_tx_isr(void);
void i3c_ibi_isr(void);
void i3c_eei_isr(void);
void r_icu_isr(void);
void adc_scan_end_isr(void);
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW ((IRQn_Type) 0) /* GPT0 COUNTER OVERFLOW (Overflow) */
#define GPT0_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 0) /* GPT0 COUNTER OVERFLOW (Overflow) */
#define VECTOR_NUMBER_I3C0_RCV_STATUS ((IRQn_Type) 1) /* I3C0 RCV STATUS (Receive status buffer full) */
#define I3C0_RCV_STATUS_IRQn          ((IRQn_Type) 1) /* I3C0 RCV STATUS (Receive status buffer full) */
#define VECTOR_NUMBER_I3C0_RESPONSE ((IRQn_Type) 2) /* I3C0 RESPONSE (Response status buffer full) */
#define I3C0_RESPONSE_IRQn          ((IRQn_Type) 2) /* I3C0 RESPONSE (Response status buffer full) */
#define VECTOR_NUMBER_I3C0_RX ((IRQn_Type) 3) /* I3C0 RX (Receive) */
#define I3C0_RX_IRQn          ((IRQn_Type) 3) /* I3C0 RX (Receive) */
#define VECTOR_NUMBER_I3C0_TX ((IRQn_Type) 4) /* I3C0 TX (Transmit) */
#define I3C0_TX_IRQn          ((IRQn_Type) 4) /* I3C0 TX (Transmit) */
#define VECTOR_NUMBER_I3C0_IBI ((IRQn_Type) 5) /* I3C0 IBI (IBI status buffer full) */
#define I3C0_IBI_IRQn          ((IRQn_Type) 5) /* I3C0 IBI (IBI status buffer full) */
#define VECTOR_NUMBER_I3C0_EEI ((IRQn_Type) 6) /* I3C0 EEI (Error) */
#define I3C0_EEI_IRQn          ((IRQn_Type) 6) /* I3C0 EEI (Error) */
#define VECTOR_NUMBER_ICU_IRQ7 ((IRQn_Type) 7) /* ICU IRQ7 (External pin interrupt 7) */
#define ICU_IRQ7_IRQn          ((IRQn_Type) 7) /* ICU IRQ7 (External pin interrupt 7) */
#define VECTOR_NUMBER_ICU_IRQ4 ((IRQn_Type) 8) /* ICU IRQ4 (External pin interrupt 4) */
#define ICU_IRQ4_IRQn          ((IRQn_Type) 8) /* ICU IRQ4 (External pin interrupt 4) */
#define VECTOR_NUMBER_ADC0_SCAN_END ((IRQn_Type) 9) /* ADC0 SCAN END (End of A/D scanning operation) */
#define ADC0_SCAN_END_IRQn          ((IRQn_Type) 9) /* ADC0 SCAN END (End of A/D scanning operation) */
#define VECTOR_NUMBER_ICU_IRQ9 ((IRQn_Type) 10) /* ICU IRQ9 (External pin interrupt 9) */
#define ICU_IRQ9_IRQn          ((IRQn_Type) 10) /* ICU IRQ9 (External pin interrupt 9) */
#define VECTOR_NUMBER_SCI0_RXI ((IRQn_Type) 11) /* SCI0 RXI (Receive data full) */
#define SCI0_RXI_IRQn          ((IRQn_Type) 11) /* SCI0 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI0_TXI ((IRQn_Type) 12) /* SCI0 TXI (Transmit data empty) */
#define SCI0_TXI_IRQn          ((IRQn_Type) 12) /* SCI0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI0_TEI ((IRQn_Type) 13) /* SCI0 TEI (Transmit end) */
#define SCI0_TEI_IRQn          ((IRQn_Type) 13) /* SCI0 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI0_ERI ((IRQn_Type) 14) /* SCI0 ERI (Receive error) */
#define SCI0_ERI_IRQn          ((IRQn_Type) 14) /* SCI0 ERI (Receive error) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (15)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
