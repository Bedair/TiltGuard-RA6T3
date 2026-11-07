/*
 * encoders.h
 *
 * Single-channel wheel encoders (M2A = left on IRQ7/P408, M1A = right on
 * IRQ1/P205), both edges counted, exactly like the stock firmware's CHANGE
 * interrupts. No direction yet - that comes from the motor command in
 * Phase 2, so counts only ever increase.
 *
 * COUNTS_PER_WHEEL_REV = 660: 22 edges per motor revolution (11 pulses,
 * both edges counted) x 30:1 gearing. Hardware confirmed working 2026-11
 * (ISR entry counts matched processed counts on both wheels); a hand test
 * of 10 slow turns gave 694-746 counts/turn, which overcounts because
 * hand-turning isn't perfectly smooth in one direction on a single-channel
 * encoder (any back-and-forth adds counts) - that test doesn't confirm or
 * refute 660 by itself. To be verified properly in Phase 2 by driving the
 * robot 1 m under motor control and checking for ~3140 counts.
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

#include <stdint.h>
#include "bsp_api.h"
#include "r_external_irq_api.h"

#define COUNTS_PER_WHEEL_REV (660.0f) /* 22 edges/motor-rev x 30:1 - see comment above. */

typedef struct st_encoders_snapshot
{
    uint32_t left_total;
    uint32_t right_total;
    uint32_t left_counts_per_5ms;  /* Raw, verifiable ground truth. */
    uint32_t right_counts_per_5ms;
    float    left_rev_per_s;       /* Derived using COUNTS_PER_WHEEL_REV. */
    float    right_rev_per_s;

    /* Unconditional ISR entry count, before any processing - kept for the
     * safety monitor (stuck/missing-encoder detection) later. */
    uint32_t left_isr_entries;
    uint32_t right_isr_entries;
} encoders_snapshot_t;

/* Opens and enables both external IRQ channels. Checks every fsp_err_t. */
fsp_err_t encoders_init(void);

/* Call every 1 kHz tick from control_tick_isr. Every 5th call, snapshots the
 * counts-per-5ms delta for both wheels. Integer-only, no float math. */
void encoders_on_tick(void);

/* Consistent snapshot, safe to call from the main loop. Computes rev/s from
 * the latest counts-per-5ms at call time. */
void encoders_get_snapshot(encoders_snapshot_t * p_snapshot);

#ifndef encoder_left_isr
void encoder_left_isr(external_irq_callback_args_t * p_args);
#endif

#ifndef encoder_right_isr
void encoder_right_isr(external_irq_callback_args_t * p_args);
#endif

#endif /* ENCODERS_H_ */
