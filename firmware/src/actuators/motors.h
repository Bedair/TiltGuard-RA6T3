/*
 * motors.h
 *
 * TB6612FNG motor driver over GPT1 (PWMA, P409/D5) and GPT2 (PWMB, P113/D6),
 * both 20 kHz, plus AIN1 (P008), BIN1 (P110), STBY (P006) direction/enable
 * GPIOs - all through a non-inverting 3.3 V -> 5 V level shifter.
 *
 * Forward/reverse mapping on AIN1/BIN1 matches the ELEGOO Tumbller stock
 * firmware's carForward()/carBack() (BalanceCar.h): AIN1 drives the left
 * motor (paired with PWMA_LEFT), BIN1 drives the right motor (paired with
 * PWMB_RIGHT). Forward = AIN1/BIN1 low with positive PWM duty.
 *
 * Safe by default: motors_init() leaves STBY low and both duties at 0
 * before anything else can run, and motors_disable() (STBY low, duty 0 on
 * both channels) is what the safety module calls on FAULT.
 *
 * Sign convention (CLAUDE.md "Conventions"): a positive command drives the
 * wheel forward, and forward is defined as the way the wheels roll for a
 * positive command - check it by eye with the console's "jog <pct>".
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdbool.h>
#include <stdint.h>
#include "bsp_api.h"

/* Command range for motors_set(), in per-mille of full PWM duty. Sign is direction. */
#define MOTORS_COMMAND_MAX (1000)
#define MOTORS_COMMAND_MIN (-1000)

/*
 * Opens GPT1/GPT2, configures AIN1/BIN1/STBY as GPIO outputs, and leaves
 * everything in the safe state (STBY low, both PWM duties 0). Checks every
 * fsp_err_t and returns the first failure.
 */
fsp_err_t motors_init(void);

/* Direction a motor is commanded in, decoded from the driver command itself
 * (AIN1/BIN1 level and the duty actually written), not from the request. */
typedef enum e_motors_direction
{
    MOTORS_DIR_STOP = 0, /* Duty 0 (either pin level). */
    MOTORS_DIR_FWD,      /* AIN1/BIN1 low, duty > 0. */
    MOTORS_DIR_REV,      /* AIN1/BIN1 high, duty > 0. */
} motors_direction_t;

/*
 * One complete driver command: exactly what motors_apply() writes to the
 * pins and GPT channels. Built by motors_plan(), so a caller that doesn't
 * apply it (balance dry-run) can still log the precise command it would
 * have sent - same saturation, same direction mapping, same duty counts.
 */
typedef struct st_motors_command
{
    int16_t  left_permille;     /* Requested, after saturation to +/-1000. */
    int16_t  right_permille;
    bool     left_reverse;      /* AIN1 level: true = high = reverse. */
    bool     right_reverse;     /* BIN1 level. */
    uint32_t left_duty_counts;  /* GPT1 compare value written to GTIOCA. */
    uint32_t right_duty_counts; /* GPT2 compare value. */
} motors_command_t;

/* Saturates and maps a request to a driver command. Pure: touches no
 * hardware. */
void motors_plan(int16_t left_permille, int16_t right_permille, motors_command_t * p_command);

/* Writes a planned command: direction pins, both duties, then STBY high.
 * Returns false if an FSP call failed - the motors are then disabled and
 * the error counted (motors_get_error_count()). */
bool motors_apply(motors_command_t const * p_command);

/* Direction of one channel of a command (FWD/REV/STOP). */
motors_direction_t motors_direction(bool reverse, uint32_t duty_counts);

/* Duty counts back to permille of the PWM period, rounded - the duty that
 * was actually written, for telemetry. */
uint16_t motors_duty_permille(uint32_t duty_counts);

/* What the MCU side of the driver actually looks like right now: the
 * STBY/AIN1/BIN1 pin levels read back from the port input register (PIDR),
 * and whether both PWM timers are counting. This is the 3.3 V side; it
 * can't see past the BSS138 shifters. */
typedef struct st_motors_readback
{
    bool stby_high;
    bool ain1_high;
    bool bin1_high;
    bool left_timer_running;  /* GPT1 (PWMA). */
    bool right_timer_running; /* GPT2 (PWMB). */
} motors_readback_t;

/* Fills p_readback. Returns the first failing FSP status (timers then read as not running). */
fsp_err_t motors_readback(motors_readback_t * p_readback);

/* "FWD", "REV" or "STOP". */
const char * motors_direction_name(motors_direction_t direction);

/*
 * Sets left/right motor command, each saturated to
 * [MOTORS_COMMAND_MIN, MOTORS_COMMAND_MAX] per-mille of full duty. Drives
 * AIN1/BIN1 for direction, sets STBY high (enabled), and writes both PWM
 * duties (motors_plan() + motors_apply()). Does nothing to recover from a fault - the safety module is
 * responsible for not calling this while latched in FAULT.
 */
void motors_set(int16_t left_permille, int16_t right_permille);

/*
 * Safe state: STBY low, both PWM duties forced to 0. Called once during
 * motors_init() and by the safety module on every transition into FAULT.
 */
void motors_disable(void);

/* Last commanded values (0,0 if never set or currently disabled) - for
 * telemetry (tools/telemetry/'s CSV stream). */
void motors_get_command(int16_t * p_left, int16_t * p_right);

/* Count of failed R_GPT_DutyCycleSet calls inside motors_set() (each one
 * also disables the motors). Monotonic; fed to safety_update(). */
uint32_t motors_get_error_count(void);

#endif /* MOTORS_H_ */
