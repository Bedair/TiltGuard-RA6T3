/*
 * jog.h
 *
 * "jog <pct>" console command: drives both motors at +pct percent duty for
 * 1 s, then stops - so a person can confirm by eye that a positive command
 * rolls the wheels forward (the sign convention in CLAUDE.md).
 *
 * Allowed only in the READY state (not armed), and it works in dry-run too,
 * since it is an explicit command. While a jog runs:
 *   - arming is refused (hal_entry.c), so the balance step never shares the
 *     motors with it;
 *   - the safety monitor keeps running; a FAULT disables the motors itself,
 *     and any state other than READY aborts the jog (motors_disable()).
 *
 * When it ends it prints, without anyone having to watch the wheels:
 *   - the encoder edges per wheel and a verdict ("moved" / "DID NOT MOVE");
 *   - a readback taken halfway: STBY/AIN1/BIN1 pin levels and whether both
 *     PWM timers are counting (MCU side of the level shifters).
 * The direction can't be measured (single-channel encoders) - that one
 * still needs an eye on the wheels.
 *
 * Main loop only (console_poll() and the loop's jog_update() call).
 */

#ifndef JOG_H_
#define JOG_H_

#include <stdbool.h>
#include <stdint.h>
#include "../safety/safety.h"

#define JOG_MAX_PERCENT    (20)
#define JOG_DURATION_TICKS (1000U) /* 1 s at the 1 kHz tick. */
#define JOG_READBACK_TICKS (500U)  /* Pin/timer readback halfway through. */
#define JOG_MIN_BATTERY_MV (5000U) /* Battery sense below this before and during the jog = no motor supply. */
#define JOG_MIN_EDGES      (10U)   /* Fewer encoder edges in the 1 s = "did not move" (a turning wheel gives hundreds). */

#define JOG_SELF_TEST_PERCENT (JOG_MAX_PERCENT) /* S1 self-test duty: 20%, above the gearmotor deadband. */

/* Starts a jog if the state is READY and pct is 1..JOG_MAX_PERCENT; prints
 * the outcome (started, or why it was refused) on RTT channel 0. */
void jog_start(int32_t percent);

/* Same jog at JOG_SELF_TEST_PERCENT, flagged as the S1 self-test: when it
 * ends, jog_get_self_test_result() reports pass (both wheels moved) or fail. */
void jog_start_self_test(void);

/* Stops a running jog now (motors off) and prints its result; a self-test
 * stopped this way counts as failed. No effect if no jog is running. */
void jog_abort(const char * p_reason);

/* True once, after a self-test has ended; *p_passed = both wheels moved and
 * it ran the full second. */
bool jog_get_self_test_result(bool * p_passed);

/* Call once per tick from the main loop, after safety_update(). Stops the
 * jog after JOG_DURATION_TICKS, or at once if the state left READY. */
void jog_update(uint32_t tick, safety_state_t safety_state);

bool jog_is_running(void);

#endif /* JOG_H_ */
