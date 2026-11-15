/*
 * console.h
 *
 * Live tuning over RTT down-channel 0 (type into J-Link RTT Viewer's input
 * line, press Enter). One command per line:
 *
 *   kp <v>    balance P          (0 .. 200)
 *   kd <v>    balance D          (0 .. 20)
 *   sp <v>    setpoint, degrees  (-20 .. 20)
 *   skp <v>   speed P            (0 .. 50)
 *   ski <v>   speed I            (0 .. 5)
 *   tkd <v>   turn (yaw) D       (0 .. 10)
 *   dry <0|1> dry-run off/on (default off, BALANCE_DRY_RUN_AT_BOOT); refused while armed
 *   jog <pct> both wheels at +pct (1 .. 20) for 1 s; READY only, works in dry-run
 *   dump      black-box recording as CSV on channel 1 (telemetry/blackbox.h); not while armed
 *   show      print all parameters
 *   help      list commands
 *
 * The space between name and value is optional: "skp0" is "skp 0".
 *
 * Gains are in the stock 8-bit PWM units (see control/balance.h). Each
 * accepted change is applied with balance_set_params() - one atomic copy
 * between two control steps - and the full parameter set is echoed back.
 * Out-of-range or malformed values are rejected with a message and change
 * nothing.
 *
 * Runs in the main loop only. Non-blocking: reads whatever bytes RTT has
 * and returns. No dynamic memory, no libc float parsing (newlib's strtof
 * can allocate), so values are parsed by a small fixed-point parser: an
 * optional sign, digits, an optional '.' and up to 6 fraction digits.
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include "../control/balance.h"

/* Call from the main loop, once per tick or more often. */
void console_poll(void);

/* Prints the current balance parameters on RTT channel 0. */
void console_print_params(void);

/* One line at arming, so every test run's log records the parameters it ran with:
 * "armed: angle=..., setpoint=..., kp=..., kd=..., skp=..., ski=..., tkd=..." (3 decimals). */
void console_print_armed(float angle_deg, balance_params_t const * p_params);

/* "mode: DRY-RUN ..." or "mode: LIVE ...". */
void console_print_mode(void);

/* "speed loop: OFF ..." (skp = ski = 0) or "speed loop: ON (skp=... ski=...)". */
void console_print_speed_loop(void);

#endif /* CONSOLE_H_ */
