/*
 * drive.h
 *
 * Drive and turn on command, through the stock law's own setpoints
 * (balance.h: speed_target in the speed-PI integral, turn_target added to the
 * turn term), in stock units:
 *
 *   drv <v>   speed target, encoder counts per 40 ms (both wheels averaged),
 *             -DRIVE_SPEED_MAX..+DRIVE_SPEED_MAX. + = FWD (the way "jog" rolls).
 *             Stock: 40 (default), 80 (Bluetooth/IR), 20 (follow mode).
 *             40 = 1000 counts/s = ~1.5 wheel rev/s.
 *   trn <v>   turn offset, 8-bit PWM units, -DRIVE_TURN_MAX..+DRIVE_TURN_MAX.
 *             + = pwm_R up, pwm_L down. Stock: 50 (default), 80 (Bluetooth).
 *   stop      both targets to 0 at once.
 *
 * Differences from stock, deliberate:
 *   - Deadman: every drv/trn command holds for DRIVE_HOLD_TICKS, then its
 *     target returns to 0 (stock's app keeps re-sending while a button is
 *     held and sends stop on release; a console command has no release).
 *     Re-send to keep going.
 *   - Refused unless ACTIVE (armed); both targets cleared whenever the state
 *     leaves ACTIVE (stock STOP mode clears setting_car_speed too).
 *   - balance.c ignores the targets until the speed-loop soft start has
 *     finished (1.5 s after arming), so they can't disturb the stand-up.
 *
 * Main loop only (console_poll() and the loop's drive_update() call).
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdbool.h>
#include <stdint.h>
#include "../safety/safety.h"

#define DRIVE_SPEED_MAX  (80)    /* Counts per 40 ms - stock Bluetooth/IR speed. */
#define DRIVE_TURN_MAX   (80)    /* 8-bit PWM units - stock Bluetooth turn. */
#define DRIVE_HOLD_TICKS (2000U) /* 2 s deadman per command. */

/* Console: set the speed / turn target for DRIVE_HOLD_TICKS. Print the
 * outcome (accepted, or why refused) on RTT channel 0. */
void drive_command_speed(int32_t speed);
void drive_command_turn(int32_t turn);

/* Remote: set both targets at once, each held for hold_ticks (a remote that
 * sends one update per change uses a long backstop instead of the console's
 * 2 s). Prints "<p_source>: speed .. turn .." or why it was refused.
 * Returns true if accepted. */
bool drive_command(int32_t speed, int32_t turn, uint32_t hold_ticks, const char * p_source);

/* Console "stop": both targets to 0 now. */
void drive_stop(void);

/* Call once per tick from the main loop, after safety_update(): expires
 * held commands and clears both targets whenever not ACTIVE. */
void drive_update(uint32_t tick, safety_state_t safety_state);

#endif /* DRIVE_H_ */
