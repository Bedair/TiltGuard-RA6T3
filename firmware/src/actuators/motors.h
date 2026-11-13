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
 * NOT YET IMPLEMENTED: this header is the interface only. The .c file
 * depends on g_pwm_motor_left/g_pwm_motor_right (GPT1/GPT2), which are not
 * generated yet - see the FSP configurator instructions for Phase 2 step 1.
 * safety.c can already be written and compiled against this header; only
 * the real PWM/GPIO behavior is pending.
 */

#ifndef MOTORS_H_
#define MOTORS_H_

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

/*
 * Sets left/right motor command, each saturated to
 * [MOTORS_COMMAND_MIN, MOTORS_COMMAND_MAX] per-mille of full duty. Drives
 * AIN1/BIN1 for direction, sets STBY high (enabled), and writes both PWM
 * duties. Does nothing to recover from a fault - the safety module is
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

#endif /* MOTORS_H_ */
