/*
 * balance.h
 *
 * Phase 3 step 1: a faithful port of the ELEGOO Tumbller V1.1 stock balance
 * law (Tumbller_Code_20191012, BalanceCar.h: balanceCar()), with only the
 * units converted. Structure, signs, gains and limits are the stock ones:
 *
 *   balance_out = kp * (angle - setpoint) + kd * gyro_x          (every step)
 *
 *   every 40 ms (stock: every 8th 5 ms cycle):
 *     car_speed    = (enc_L + enc_R) / 2      signed counts in the window
 *     speed_filter = 0.7 * speed_filter + 0.3 * car_speed
 *     integral    += speed_filter - speed_target           (target = 0)
 *     integral     = clamp(integral, -3000, 3000)
 *     speed_out    = -skp * speed_filter - ski * integral
 *     turn_out     = turn_target + tkd * yaw_rate          (target = 0)
 *
 *   pwm_L = clamp(balance_out - speed_out - turn_out, -255, 255)
 *   pwm_R = clamp(balance_out - speed_out + turn_out, -255, 255)
 *
 * Units, identical to stock: angle in degrees, rates in deg/s, encoder
 * counts both edges (660 per wheel rev, same encoder and gearing),
 * speed in counts per 40 ms, and gains/outputs in 8-bit PWM units
 * (+/-255). The only conversion is at the very end: pwm x 1000/255 -> the
 * +/-1000 permille that motors_set() takes. Gains therefore keep their
 * stock values (kp 55, kd 0.75, skp 10, ski 0.26, tkd 0.5).
 *
 * Encoder direction: the encoders are single-channel, so each step's new
 * counts are signed by the sign of the previous step's PWM command, exactly
 * as the stock firmware does (pwm_left < 0 ? -count : count).
 *
 * Differences from stock, deliberate and documented:
 *   - Runs at 1 kHz (IMU callback) instead of 200 Hz. The PD has no dt term,
 *     so its gains carry over unchanged. The speed loop keeps the stock 40 ms
 *     period (every 40th step), so its units are unchanged too.
 *   - Angle comes from mpu6050.c's complementary filter, not the stock
 *     Kalman filter. Same axis convention: atan2(ay, az), gyro_x. Its time
 *     constant (1 s) matches the stock Kalman's effective accel weighting
 *     (~1.5 s), so wheel acceleration on the floor doesn't read as tilt.
 *   - Yaw rate is float, with its bias removed. The stock code computes
 *     Gyro_z = -gz / 131 in integer arithmetic (truncated to whole deg/s)
 *     and doesn't remove the bias. The sign (negated) is kept.
 *   - kp_turn (2.5) is declared in the stock code but never used; it is
 *     not ported.
 *   - Speed PI: stock gains, but its integral is frozen while both wheels
 *     are saturated in the direction it would push (anti-windup), and it
 *     soft-starts after each arm (BALANCE_SPEED_START_*).
 *   - Start: stock runs the law from power-on in motion_mode START, with
 *     its +/-22 deg cut-out skipped for 2 s, so it stands up from rest. Here
 *     S1 starts it instead (no motor action at boot), from any |angle| up to
 *     BALANCE_ARM_LIMIT_DEG, and safety.c skips the normal cut-out for the
 *     same 2 s (SAFETY_STANDUP_*), with a 45 deg bound stock doesn't have.
 *   - Speed loop off (BALANCE_SPEED_LOOP_OFF, skp = ski = 0): the speed
 *     filter and integrator are held at 0, so the PD alone drives.
 *   - Dry-run mode (balance_set_dry_run()): armed, law running, output not
 *     applied. For verifying the logic from the RTT log alone.
 *   - Output is gated: the motors are driven only when balancing is
 *     enabled AND safety_get_state() == ACTIVE AND not dry-run. When not
 *     armed (disabled or not ACTIVE), the controller
 *     state (integrator, filter, encoder accumulators) is held at reset,
 *     like the stock STOP mode (car_speed_integeral = 0, pwm = 0).
 *
 * Concurrency: balance_step() runs in the I3C read-complete ISR. Parameters
 * are written from the main loop through balance_set_params(), which
 * copies them with interrupts masked, so a step always sees one consistent
 * set. Status is read back the same way via balance_get_status().
 */

#ifndef BALANCE_H_
#define BALANCE_H_

#include <stdbool.h>
#include <stdint.h>
#include "../actuators/motors.h"

/*
 * Balance point (degrees). 0.0, as stock (angle_zero = 0): the speed PI's
 * integral absorbs a centre-of-mass / accel-offset error. A +3.5 deg value
 * was tried from one "quiet" CSV stretch, but that stretch was recorded while
 * the robot was being held by hand, so it measured the hand, not the robot.
 * Re-measure only from a hands-off run: the mean of sp + spd/kp while the
 * wheels are near standstill. Live with the "sp" console command; the arming
 * limit (|angle| <= BALANCE_ARM_LIMIT_DEG) doesn't depend on it.
 */
#define BALANCE_SETPOINT_DEG_DEFAULT  (0.0f)

/*
 * BALANCE_SPEED_LOOP_OFF (defined for Phase 3 sign/logic verification):
 * boot with skp = ski = 0, i.e. the balance PD alone, without typing it.
 * Whenever skp == ski == 0 (live values, so also when set from the console)
 * the speed filter, integrator and output are held at 0, so nothing from
 * the speed loop can leak into the command. "skp"/"ski" still work and
 * re-enable it live. Comment out to boot with the stock speed PI.
 *
 * Now OFF (stock speed PI at boot): the PD alone balanced on the floor but
 * cannot hold position - any lean the PD settles at (the true balance point
 * is not exactly 0 deg) needs constant acceleration, so the robot ran away
 * in one direction until the duty saturated at 1000 and it fell. The speed
 * PI is what makes it stand still. Re-define for sign checks without it.
 */
/* #define BALANCE_SPEED_LOOP_OFF */

/* Stock gains (BalanceCar.h), stock units - see above. */
#define BALANCE_KP_DEFAULT            (55.0f)
#define BALANCE_KD_DEFAULT            (0.75f)
#ifdef BALANCE_SPEED_LOOP_OFF
 #define BALANCE_SPEED_KP_DEFAULT     (0.0f)
 #define BALANCE_SPEED_KI_DEFAULT     (0.0f)
#else
 /* Stock. (Halving them was tried on runs that turned out to be hand-held,
  * so that evidence didn't count; the encoder count per rev was confirmed
  * at 658 vs 660 expected, i.e. stock units.) */
 #define BALANCE_SPEED_KP_DEFAULT     (10.0f)
 #define BALANCE_SPEED_KI_DEFAULT     (0.26f)
#endif
#define BALANCE_TURN_KD_DEFAULT       (0.5f)

/* Stock limits and timing. */
#define BALANCE_PWM_LIMIT_8BIT        (255.0f)
#define BALANCE_SPEED_INTEGRAL_LIMIT  (3000.0f)
#define BALANCE_SPEED_PERIOD_STEPS    (40U)   /* 40 ms at the 1 kHz step rate (stock: 8 x 5 ms). */
#define BALANCE_SPEED_FILTER_OLD      (0.7f)
#define BALANCE_SPEED_FILTER_NEW      (0.3f)

/*
 * Speed-loop soft start (difference from stock). The speed PI is positive
 * feedback on wheel speed (moving REV -> drive more REV); it relies on the
 * free body tipping so the angle term pulls it back. If the body is held by
 * hand right after arming, it can't tip, and the speed term runs the wheels
 * up to full speed - when released, nothing is left to catch it. Floor logs
 * showed exactly that: wheels ramping for 0.7 s while the body rotated < 2 deg.
 * So after each arm the speed loop (filter, integral, output) is held at 0 for
 * BALANCE_SPEED_START_DELAY_STEPS, then its output fades in linearly over
 * BALANCE_SPEED_START_RAMP_STEPS. Let go of the robot right after S1.
 */
#define BALANCE_SPEED_START_DELAY_STEPS (500U)  /* 0.5 s at 1 kHz. */
#define BALANCE_SPEED_START_RAMP_STEPS  (1000U) /* Then 1.0 s ramp to full. */

/* Arming limit: S1 arms only if |angle| is at most this (absolute angle, like
 * stock's limits and the safety cut-out). Covers the robot resting on its
 * support (-18.6 / -21.1 deg in the 2026-11-14 boot logs) with margin: the
 * angle term saturates, the wheels kick and it stands up, inside safety.c's
 * 2 s stand-up window (SAFETY_STANDUP_*). Must stay below
 * SAFETY_STANDUP_TILT_LIMIT_DEG (45), or arming would fault at once. The
 * old 22 deg window sat right at the resting angle, and the 25 deg tilt
 * fault applied even in READY - see docs/status.md, 2026-11-15. */
#define BALANCE_ARM_LIMIT_DEG         (40.0f)

/*
 * Built-in logic check (runs every step while armed, speed loop off only).
 * With skp = ski = 0 the command is pwm = kp*e + kd*gyro -/+ turn, where
 * e = angle - setpoint. Whenever |e| > BALANCE_CHECK_MIN_ERROR_DEG, both
 * wheels' commanded direction (decoded from the driver command) must be
 * FWD for e > 0 and REV for e < 0. A mismatch is counted in sign_err.
 *
 * Steps where the rest of the law (kd*gyro and the turn term) is at least
 * BALANCE_CHECK_P_DOMINANCE of the P term are counted as skipped, not
 * checked: there a direction opposite to e is correct PD behaviour (e.g.
 * the D term braking a fast return towards the setpoint), not a sign error.
 * On every checked step the P term outweighs everything else 2:1, so the
 * expected direction is unambiguous.
 */
#define BALANCE_CHECK_MIN_ERROR_DEG   (2.0f)
#define BALANCE_CHECK_P_DOMINANCE     (0.5f)

typedef struct st_balance_params
{
    float kp;           /* Balance P, 8-bit PWM per degree. */
    float kd;           /* Balance D, 8-bit PWM per deg/s. */
    float setpoint_deg; /* Balance point. */
    float speed_kp;     /* Speed P, 8-bit PWM per (counts per 40 ms). */
    float speed_ki;     /* Speed I, 8-bit PWM per accumulated count. */
    float turn_kd;      /* Yaw-rate damping, 8-bit PWM per deg/s. */
} balance_params_t;

/*
 * Motion check: did each wheel actually turn? Judged per 40 ms speed-loop
 * window from the encoder edges against what the driver was commanded
 * (single-channel encoders: edges say "it turned", not which way):
 *
 *   ok     driven and edges > 0
 *   NONE   driven >= BALANCE_MOTION_MIN_DUTY_PERMILLE for the whole window,
 *          0 edges - the motor should have turned and didn't
 *   low    driven, but below that duty part of the window, 0 edges - not
 *          judged (may be inside the gearmotor's deadband)
 *   idle   not driven (dry-run), 0..BALANCE_MOTION_UNEXPECTED_EDGES edges
 *   UNEXP  not driven, more edges than that - the wheel turned with the
 *          motors off (pushed by hand, or the driver isn't really off)
 *
 * BALANCE_MOTION_STALL_WINDOWS consecutive NONE windows on a wheel (~0.5 s)
 * count one stall fault (balance_get_stall_fault_count()), which latches
 * SAFETY_FAULT_ENCODER: no motion while driven means no battery, a
 * disconnected motor or encoder, or a blocked wheel - balancing can't work
 * in any of those.
 */
#define BALANCE_MOTION_MIN_DUTY_PERMILLE (300U)
#define BALANCE_MOTION_STALL_WINDOWS     (13U)  /* 13 x 40 ms = 520 ms. */
#define BALANCE_MOTION_UNEXPECTED_EDGES  (3U)

typedef enum e_balance_motion
{
    BALANCE_MOTION_IDLE = 0,
    BALANCE_MOTION_LOW,
    BALANCE_MOTION_OK,
    BALANCE_MOTION_NONE,
    BALANCE_MOTION_UNEXPECTED,
} balance_motion_t;

typedef struct st_balance_motion_count
{
    uint32_t judged;     /* Windows driven >= min duty throughout (ok or NONE possible). */
    uint32_t moved;      /* Of those, windows with edges. */
    uint32_t none;       /* Of those, windows with 0 edges. */
    uint32_t unexpected; /* Windows with edges while not driven. */
} balance_motion_count_t;

/* "ok", "NONE", "low", "idle", "UNEXP". */
const char * balance_motion_name(balance_motion_t motion);

/* Monotonic count of stall faults (see above), for safety_update(). */
uint32_t balance_get_stall_fault_count(void);

typedef struct st_balance_check
{
    uint32_t samples;        /* Steps checked (|e| > 2 deg, P term dominant). */
    uint32_t sign_err;       /* Of those, steps where a wheel's direction didn't match sign(e). */
    uint32_t skipped;        /* |e| > 2 deg but D/turn >= half the P term - not checked. */
    bool     speed_loop_on;  /* skp or ski was non-zero at some step of this run: check void. */
} balance_check_t;

typedef struct st_balance_status
{
    float    angle_deg;           /* Inputs of the most recent step. */
    float    gyro_dps;
    float    setpoint_deg;
    float    bal_8bit;            /* Terms in stock 8-bit units, before clamping: */
    float    spd_8bit;            /*   pwm_L = clamp(bal - spd - turn, +/-255) */
    float    turn_8bit;           /*   pwm_R = clamp(bal - spd + turn, +/-255) */
    float    speed_integral;      /* Stock units (accumulated counts), clamped to +/-3000. */
    motors_command_t command;     /* The driver command of this step: applied if driving, */
                                  /* only planned in dry-run; all-zero (STOP) when not armed. */
    uint16_t enc_left_window;     /* Unsigned encoder edges in the last complete 40 ms window. */
    uint16_t enc_right_window;
    int32_t  enc_left_counts;     /* Signed cumulative position while armed; 0 when not. */
    int32_t  enc_right_counts;
    bool     armed;               /* Law running: enabled and safety ACTIVE. */
    bool     dry_run;             /* Armed but output suppressed (STBY low, PWM 0). */
    bool     driving;             /* This step's command was applied to the driver. */
    bool     speed_loop_on;       /* skp or ski non-zero at this step. */
    balance_check_t check;        /* Since the most recent arm; held after disarm. */
    balance_motion_t motion_left; /* Verdict of the last complete 40 ms window. */
    balance_motion_t motion_right;
    balance_motion_count_t motion_count_left;  /* Since the most recent arm; held after disarm. */
    balance_motion_count_t motion_count_right;
    uint32_t step_count;          /* Monotonic, every balance_step() call. */
} balance_status_t;

/* Loads the stock defaults and the default setpoint. Balancing starts disabled. */
void balance_init(void);

/* Allows balance_step() to drive the motors while the safety state is
 * ACTIVE. Not called in MOTOR_BENCH_TEST builds, where the bench sequence
 * owns the motors instead. */
void balance_set_enabled(bool enabled);

/* One control step. Call only from the IMU read-complete callback, once per
 * fresh sample (1 kHz). No blocking calls. */
void balance_step(float angle_deg, float gyro_x_dps, float gyro_z_dps);

/*
 * Dry-run (boot default BALANCE_DRY_RUN_AT_BOOT, below): when armed, the full law runs and is logged,
 * but the command is only planned (motors_plan()), never applied - STBY
 * stays low and both PWM duties stay 0. Changing it is refused while armed
 * (returns false): switching to live mid-run would start the motors
 * without a fresh arm.
 */
bool balance_set_dry_run(bool dry_run);

/* Dry-run state at boot. false: one S1 press runs the motor self-test and
 * arms with the motors live (hal_entry.c), no console needed. Set true to
 * boot in dry-run ("dry 0" then enables the motors). */
#define BALANCE_DRY_RUN_AT_BOOT       (false)
bool balance_get_dry_run(void);

/* Atomic copy in/out (interrupts masked for the copy only). */
void balance_get_params(balance_params_t * p_params);
void balance_set_params(balance_params_t const * p_params);
void balance_get_status(balance_status_t * p_status);

#endif /* BALANCE_H_ */
