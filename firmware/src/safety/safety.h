/*
 * safety.h
 *
 * State machine: INIT -> CALIBRATING -> READY -> ACTIVE -> FAULT.
 *
 * READY <-> ACTIVE is driven by S1 (debounced in button.c): short press in
 * READY arms (safety_request_active()); short press in ACTIVE disarms
 * (safety_disarm()). Short press in FAULT calls safety_reset(), which goes
 * to INIT, not directly to READY or ACTIVE - CALIBRATING's settle window
 * runs again before READY. See hal_entry.c for the button-to-state wiring.
 *
 * FAULT is latched: once entered (tilt beyond +/-25 deg, an IMU read
 * failure or >5 ms without a fresh sample, a 1 kHz tick overrun, or a
 * motor-driver FSP error, or a driven wheel that doesn't turn),
 * motors_disable() is called exactly once and the state stays FAULT -
 * including if the triggering condition clears - until safety_reset() is
 * called explicitly ("manual reset"). Echo timeout is a warning only: it
 * never latches FAULT, and clears on its own the next tick it isn't
 * reported.
 *
 * Encoder implausibility (SAFETY_FAULT_ENCODER): a wheel driven at >= 30%
 * duty for ~0.5 s produced no encoder edges - no battery, a disconnected
 * motor or encoder, or a blocked wheel (balance.h, motion check).
 *
 * This module only calls motors_disable() in software. It does not touch
 * any FSP peripheral itself, and runs from the main loop only - it is the
 * only writer of the state. The balance step (IMU callback, ISR) reads the
 * state via safety_get_state() and drives the motors only while ACTIVE;
 * every transition out of ACTIVE stores the new state *before* calling
 * motors_disable(), so the ISR can never re-enable the motors afterwards.
 *
 * TODO (later phase): this is a software-only cutoff with normal 1 kHz
 * reaction latency. For a hardware-speed cutoff independent of this state
 * machine, add a POEG (Port Output Enable Group) stack linked to GPT1/
 * GPT2's "Output Disable POEG Trigger" extended setting, driven from
 * whatever fast fault signal is available (e.g. a comparator or GPIO).
 * That would gate the PWM output in hardware without waiting for this
 * code to run at all - keep this state machine as the software layer on
 * top of it, not a replacement for it.
 */

#ifndef SAFETY_H_
#define SAFETY_H_

#include <stdbool.h>
#include <stdint.h>

#define SAFETY_TILT_LIMIT_DEG    (25.0f) /* Stock cuts at +/-22; 35 was too late to recover and made falls more violent. */
#define SAFETY_IMU_STALE_TICKS   (5U)   /* >5 ms without a fresh IMU sample. */
#define SAFETY_CALIBRATION_TICKS (500U) /* 0.5 s settle window after init. */

typedef enum e_safety_state
{
    SAFETY_STATE_INIT = 0,
    SAFETY_STATE_CALIBRATING,
    SAFETY_STATE_READY,
    SAFETY_STATE_ACTIVE,
    SAFETY_STATE_FAULT,
} safety_state_t;

#define SAFETY_FAULT_TILT         (1UL << 0)
#define SAFETY_FAULT_IMU_STALE    (1UL << 1)
#define SAFETY_FAULT_ENCODER      (1UL << 2) /* Driven >= 30% for ~0.5 s with no encoder edges (balance.h motion check). */
#define SAFETY_FAULT_TICK_OVERRUN (1UL << 3)
#define SAFETY_FAULT_ACTUATOR     (1UL << 4) /* motors_set() hit an FSP error. */

#define SAFETY_WARNING_ECHO_TIMEOUT (1UL << 0)

void safety_init(void);

/*
 * Call once per 1 kHz tick from the main loop. Pass the latest filtered
 * tilt angle and three monotonically-increasing counters (IMU failed-read
 * count, ultrasonic echo-timeout count, control-tick overrun count) - this
 * module detects increments itself, so pass the raw totals, not deltas.
 */
void safety_update(float tilt_angle_deg,
                    uint32_t imu_sequence,
                    uint32_t imu_failed_read_count,
                    uint32_t echo_timeout_count,
                    uint32_t tick_overrun_count,
                    uint32_t motor_error_count,
                    uint32_t stall_fault_count);

/* Request moving from READY to ACTIVE. No effect outside READY. Driven by
 * a short press of S1 in READY (see button.c). */
void safety_request_active(void);

/* Request moving from ACTIVE back to READY ("disarm"). Calls
 * motors_disable() unconditionally. No effect outside ACTIVE. Driven by a
 * short press of S1 in ACTIVE. */
void safety_disarm(void);

/* Clears a latched FAULT and returns to INIT (forcing recalibration).
 * Does not check whether the triggering condition has actually cleared -
 * that judgement call belongs to whoever calls this ("manual reset"). */
void safety_reset(void);

safety_state_t safety_get_state(void);
uint32_t safety_get_fault_reasons(void);
uint32_t safety_get_warnings(void);

#endif /* SAFETY_H_ */
