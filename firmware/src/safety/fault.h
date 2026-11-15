/*
 * fault.h
 *
 * Fault diagnostics: HardFault / MemManage / BusFault / UsageFault handlers
 * that dump the fault state over RTT channel 0 and halt in a safe state, plus
 * a main-stack high-water monitor.
 *
 * On any of these faults the handler:
 *   1. captures MSP, PSP, EXC_RETURN and MSPLIM before touching the stack,
 *   2. disables the stack limit and switches MSP to a dedicated fault stack -
 *      a stack overflow (UsageFault STKOF) leaves MSP at MSPLIM, so the
 *      handler couldn't push anything on the main stack,
 *   3. drives STBY (P006) low by a direct port write - motors off,
 *   4. prints CFSR, HFSR, MMFAR, BFAR, the stacked R0-R3, R12, LR, PC, xPSR
 *      (from MSP or PSP per EXC_RETURN bit 2), MSP/MSPLIM and a short decode,
 *   5. loops forever. RTT channel 0 is switched to blocking for the dump, so
 *      it waits for RTT Viewer if the buffer is full rather than dropping it.
 *
 * If the debugger has vector catch enabled for faults, it halts before the
 * handler runs - press Resume to let the dump print.
 */

#ifndef FAULT_H_
#define FAULT_H_

#include <stdint.h>

/* Enables the separate MemManage, BusFault and UsageFault exceptions
 * (otherwise they all escalate to HardFault with HFSR.FORCED) and paints the
 * unused main stack for the high-water monitor. Call once, early in
 * hal_entry(), with interrupts not yet in use. */
void fault_init(void);

/* Bytes of the main stack never touched since fault_init() (high-water
 * margin). Scans the painted region; call from the main loop at a low rate. */
uint32_t fault_stack_free_bytes(void);

/* Total main stack size (BSP_CFG_STACK_MAIN_BYTES). */
uint32_t fault_stack_size_bytes(void);

/* FPU->FPCCR, for the boot log: ASPEN (bit 31) and LSPEN (bit 30) must both
 * be 1 so ISRs that use floats get their FP context saved automatically. */
uint32_t fault_get_fpccr(void);

#endif /* FAULT_H_ */
