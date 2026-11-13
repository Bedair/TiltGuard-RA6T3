/*
 * button.h
 *
 * S1 (P304, IRQ9), debounced in software. The ICU edge interrupt (both
 * edges, priority 7 - lower than everything else in this project) only
 * counts raw entries; the actual debounce is a simple "stable for N ticks"
 * counter run from button_on_tick(), polling the live pin level rather
 * than trusting which specific edge fired. S1 is wired active-low (press
 * pulls the pin low) - if that's backwards on this board, flip
 * BUTTON_ACTIVE_LOW in button.c.
 *
 * NOT YET IMPLEMENTED: this header is the interface only. The .c file
 * depends on g_irq_button_ctrl/_cfg, which doesn't exist until the S1
 * r_icu stack is added in the FSP configurator - see the Phase 2 safety
 * follow-up conversation. hal_entry.c can already be written against this
 * header; only button.c's body is pending.
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include "bsp_api.h"
#include "r_external_irq_api.h"

#define BUTTON_DEBOUNCE_TICKS (30U) /* >=30 ms at the 1 kHz tick. */

/* Opens and enables the S1 external IRQ channel. Checks every fsp_err_t. */
fsp_err_t button_init(void);

/* Call every 1 kHz tick from control_tick_isr. Polls the live pin level
 * and runs the debounce counter; increments the press count once a
 * debounced press edge is confirmed. Integer-only, no float math. */
void button_on_tick(void);

/* Monotonic count of debounced press events since button_init(). Single-
 * word read, atomic on Cortex-M - compare against your own last-seen
 * value to detect a new press, same pattern as control_tick_count(). */
uint32_t button_get_press_count(void);

/* Unconditional ISR entry count, before debouncing - kept for the safety
 * monitor later, same pattern as the other sensor modules. */
uint32_t button_get_isr_entries(void);

#ifndef button_isr
void button_isr(external_irq_callback_args_t * p_args);
#endif

#endif /* BUTTON_H_ */
