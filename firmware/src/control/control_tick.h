/*
 * control_tick.h
 *
 * 1 kHz control-loop tick, driven by GPT0 (g_timer_control_tick) in periodic
 * mode. The ISR only increments a counter and tracks the inter-tick interval
 * via the DWT cycle counter; no FSP calls, no telemetry, no blocking.
 */

#ifndef CONTROL_TICK_H_
#define CONTROL_TICK_H_

#include <stdint.h>
#include "bsp_api.h"
#include "r_timer_api.h"

/*
 * Enables the DWT cycle counter and opens/starts g_timer_control_tick.
 * Checks every FSP return value; returns the first failing fsp_err_t.
 * Call once at startup, before relying on control_tick_count().
 */
fsp_err_t control_tick_init(void);

/* Number of 1 kHz ticks since control_tick_init(). Single-word read, atomic on Cortex-M. */
uint32_t control_tick_count(void);

/*
 * Consistent snapshot of the min/max inter-tick interval, in DWT cycles,
 * observed since the last control_tick_reset_interval_stats() call.
 */
void control_tick_get_interval_stats(uint32_t * p_min_cycles, uint32_t * p_max_cycles);

/* Resets the min/max interval stats. Call from the main loop between report windows. */
void control_tick_reset_interval_stats(void);

#ifndef control_tick_isr
void control_tick_isr(timer_callback_args_t * p_args);
#endif

#endif /* CONTROL_TICK_H_ */
