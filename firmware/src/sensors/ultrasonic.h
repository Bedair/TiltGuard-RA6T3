/*
 * ultrasonic.h
 *
 * HC-SR04 over Trig (P109, GPIO out) / Echo (P410, IRQ5, both edges).
 *
 * A 10 us trigger pulse fires every 50 ms from the 1 kHz tick. The pulse
 * itself is a short bounded busy-wait timed off DWT->CYCCNT (2000 cycles at
 * 200 MHz) - deliberately not an FSP blocking call, just a fixed, tiny spin
 * well inside the tick's 1 ms budget, which is the only way to hold a
 * ~10 us pulse width without jitter from this ISR. The echo pulse width is
 * timestamped by the two edges of ultrasonic_echo_isr; distance_cm =
 * pulse_us / 58. A pulse that doesn't complete within 30 ms is a timeout
 * ("no echo"), counted separately.
 *
 * Hardware confirmed working 2026-11 on P410/D0: stable 22.00 +/- 0.02 cm,
 * tracked a hand down to ~3 cm.
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdbool.h>
#include <stdint.h>
#include "bsp_api.h"
#include "r_external_irq_api.h"

/*
 * 0 (default since 2026-11-15): the ultrasonic is not used. P410/D0 is now
 * the Bluetooth module's RX (SCI0 RXD0, remote/bt_remote.c) and the
 * g_irq_echo (IRQ5) stack is removed from the FSP configuration. The API
 * stays, with stubs: init only drives Trig low, the tick does nothing, and the
 * snapshot reports no distance and no timeouts (so no echo warnings).
 * Re-enabling needs the g_irq_echo stack back AND another pin for the BT RX.
 */
#ifndef ULTRASONIC_ENABLED
 #define ULTRASONIC_ENABLED (0)
#endif

typedef struct st_ultrasonic_snapshot
{
    float    distance_cm;
    bool     distance_valid; /* false if the most recent measurement timed out. */
    uint32_t timeout_count;
    uint32_t sequence;

    /* Unconditional ISR entry count, before any processing - kept for the
     * safety monitor (stuck/missing-echo detection) later. */
    uint32_t echo_isr_entries;
} ultrasonic_snapshot_t;

/* Checks every fsp_err_t. */
fsp_err_t ultrasonic_init(void);

/* Call every 1 kHz tick from control_tick_isr. Fires the trigger every 50
 * ticks and checks the in-flight echo for a 30 ms timeout every tick. */
void ultrasonic_on_tick(void);

/* Consistent snapshot, safe to call from the main loop. */
void ultrasonic_get_snapshot(ultrasonic_snapshot_t * p_snapshot);

#if ULTRASONIC_ENABLED
 #ifndef ultrasonic_echo_isr
void ultrasonic_echo_isr(external_irq_callback_args_t * p_args);
 #endif
#endif

#endif /* ULTRASONIC_H_ */
