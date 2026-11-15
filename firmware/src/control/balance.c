/*
 * balance.c
 *
 * See balance.h for the control law, units and the differences from stock.
 */

#include <math.h>
#include <string.h>
#include "balance.h"
#include "bsp_api.h"
#include "../actuators/motors.h"
#include "../safety/safety.h"
#include "../sensors/encoders.h"
#include "../telemetry/blackbox.h"

#define BALANCE_PERMILLE_PER_8BIT ((float) MOTORS_COMMAND_MAX / BALANCE_PWM_LIMIT_8BIT) /* 1000 / 255. */

/* Parameters: written by the main loop (masked), read by the ISR. */
static balance_params_t s_params;
static volatile bool    s_enabled;
static volatile bool    s_dry_run;

/* Drive setpoints: written by the main loop (masked), read by the ISR. */
typedef struct st_balance_targets
{
    float speed; /* Counts per 40 ms. */
    float turn;  /* 8-bit PWM units. */
} balance_targets_t;

static balance_targets_t s_targets;
static float             s_status_speed_target; /* Targets in effect this step, ISR-only. */
static float             s_status_turn_target;

/* Controller state, ISR-only. */
static uint32_t s_enc_left_prev_total;
static uint32_t s_enc_right_prev_total;
static int32_t  s_enc_left_window;   /* Signed counts in the current 40 ms window. */
static int32_t  s_enc_right_window;
static uint32_t s_enc_left_window_abs; /* Unsigned edges in the current 40 ms window. */
static uint32_t s_enc_right_window_abs;
static uint16_t s_enc_left_last_window; /* Unsigned edges in the last complete window. */
static uint16_t s_enc_right_last_window;
static int32_t  s_enc_left_position; /* Signed cumulative counts, for telemetry. */
static int32_t  s_enc_right_position;
static uint32_t s_speed_step_count;
static float    s_speed_filter;
static float    s_speed_integral;
static float    s_speed_out;         /* 8-bit units, held between speed-loop runs (stock). */
static float    s_turn_out;
static float    s_pwm_left;          /* Previous step's clamped 8-bit command - signs the encoders. */
static float    s_pwm_right;
static uint32_t s_step_count;
static bool     s_was_armed;
static uint32_t s_armed_steps;     /* Steps since the most recent arm (speed-loop soft start), saturating. */
static uint32_t s_armed_total;     /* Steps since the most recent arm, for the black-box time stamp. */
static blackbox_record_t s_record; /* Static, like s_command: not on the stack the ISR runs on. */
static motors_command_t s_command; /* This step's driver command. Static, not on the (main) stack the ISR runs on. */
static balance_check_t s_check;

/* Motion check (see balance.h), ISR-only except the counts copied to s_status. */
typedef struct st_balance_wheel_motion
{
    uint32_t               driven_steps; /* Steps in this window whose command was applied. */
    uint32_t               strong_steps; /* Of those, at >= BALANCE_MOTION_MIN_DUTY_PERMILLE. */
    uint32_t               none_run;     /* Consecutive NONE windows. */
    balance_motion_t       last;         /* Verdict of the last complete window. */
    balance_motion_count_t count;        /* Since the most recent arm. */
} balance_wheel_motion_t;

static balance_wheel_motion_t s_motion_left;
static balance_wheel_motion_t s_motion_right;
static volatile uint32_t      s_stall_fault_count; /* Monotonic, read by the main loop for safety_update(). */

static void balance_motion_reset_window(balance_wheel_motion_t * p_motion);

/* Status: written by the ISR, read by the main loop (masked). */
static balance_status_t s_status;

static float balance_clamp (float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

/* Rounded and saturated to int16, for the black-box record. */
static int16_t balance_to_i16 (float value)
{
    if (value >= 32767.0f)
    {
        return INT16_MAX;
    }

    if (value <= -32768.0f)
    {
        return INT16_MIN;
    }

    return (int16_t) lroundf(value);
}

static int16_t balance_signed_duty (bool reverse, uint32_t duty_counts)
{
    int16_t const duty = (int16_t) motors_duty_permille(duty_counts);

    return reverse ? (int16_t) -duty : duty;
}

/* Black box: one record of this step (see blackbox.h for the fields). */
static void balance_record_step (float angle_deg, float gyro_x_dps, float balance_out, float speed_applied,
                                 motors_command_t const * p_command)
{
    blackbox_record_t * const p_rec = &s_record;

    p_rec->t_10ms    = (uint16_t) (s_armed_total / BLACKBOX_PERIOD_STEPS);
    p_rec->ang_x100  = balance_to_i16(angle_deg * 100.0f);
    p_rec->gyro_x10  = balance_to_i16(gyro_x_dps * 10.0f);
    p_rec->pwm_left  = balance_signed_duty(p_command->left_reverse, p_command->left_duty_counts);
    p_rec->pwm_right = balance_signed_duty(p_command->right_reverse, p_command->right_duty_counts);
    p_rec->bal       = balance_to_i16(balance_out);
    p_rec->spd       = balance_to_i16(speed_applied);
    p_rec->enc_left  = (int16_t) s_enc_left_position;  /* Wraps (modulo) past +/-32768, by design. */
    p_rec->enc_right = (int16_t) s_enc_right_position;

    blackbox_record(p_rec);
}

static int16_t balance_to_permille (float pwm_8bit)
{
    float const permille = pwm_8bit * BALANCE_PERMILLE_PER_8BIT;

    /* pwm_8bit is already clamped to +/-255, so this is within +/-1000. */
    return (int16_t) lroundf(permille);
}

/* Holds the controller at its reset state - stock STOP mode. */
static void balance_reset_state (void)
{
    s_enc_left_window       = 0;
    s_enc_right_window      = 0;
    s_enc_left_window_abs   = 0U;
    s_enc_right_window_abs  = 0U;
    s_enc_left_last_window  = 0U;
    s_enc_right_last_window = 0U;
    s_enc_left_position     = 0;
    s_enc_right_position    = 0;
    s_speed_step_count      = 0U;
    s_speed_filter          = 0.0f;
    s_speed_integral        = 0.0f;
    s_speed_out             = 0.0f;
    s_turn_out              = 0.0f;
    s_pwm_left              = 0.0f;
    s_pwm_right             = 0.0f;
    balance_motion_reset_window(&s_motion_left);
    balance_motion_reset_window(&s_motion_right);
}

static void balance_motion_reset_window (balance_wheel_motion_t * p_motion)
{
    p_motion->driven_steps = 0U;
    p_motion->strong_steps = 0U;
    p_motion->none_run     = 0U;
    p_motion->last         = BALANCE_MOTION_IDLE;
}

/* Called once per armed step, after the command was (or wasn't) applied. */
static void balance_motion_accumulate (balance_wheel_motion_t * p_motion, bool driving, uint32_t duty_counts)
{
    if (driving)
    {
        p_motion->driven_steps++;

        if (motors_duty_permille(duty_counts) >= BALANCE_MOTION_MIN_DUTY_PERMILLE)
        {
            p_motion->strong_steps++;
        }
    }
}

/* Called at the end of each 40 ms window with that window's edge count. */
static void balance_motion_judge (balance_wheel_motion_t * p_motion, uint32_t edges)
{
    bool const       judged = (p_motion->strong_steps >= BALANCE_SPEED_PERIOD_STEPS);
    balance_motion_t verdict;

    if (0U == p_motion->driven_steps)
    {
        verdict = (edges > BALANCE_MOTION_UNEXPECTED_EDGES) ? BALANCE_MOTION_UNEXPECTED : BALANCE_MOTION_IDLE;
    }
    else if (edges > 0U)
    {
        verdict = BALANCE_MOTION_OK;
    }
    else if (judged)
    {
        verdict = BALANCE_MOTION_NONE;
    }
    else
    {
        verdict = BALANCE_MOTION_LOW;
    }

    if (judged)
    {
        p_motion->count.judged++;

        if (edges > 0U)
        {
            p_motion->count.moved++;
        }
        else
        {
            p_motion->count.none++;
        }
    }

    if (BALANCE_MOTION_UNEXPECTED == verdict)
    {
        p_motion->count.unexpected++;
    }

    if (BALANCE_MOTION_NONE == verdict)
    {
        p_motion->none_run++;

        if (BALANCE_MOTION_STALL_WINDOWS == p_motion->none_run)
        {
            s_stall_fault_count++; /* Once per stall episode. */
        }
    }
    else
    {
        p_motion->none_run = 0U;
    }

    p_motion->last         = verdict;
    p_motion->driven_steps = 0U;
    p_motion->strong_steps = 0U;
}

const char * balance_motion_name (balance_motion_t motion)
{
    switch (motion)
    {
        case BALANCE_MOTION_LOW:
        {
            return "low";
        }

        case BALANCE_MOTION_OK:
        {
            return "ok";
        }

        case BALANCE_MOTION_NONE:
        {
            return "NONE";
        }

        case BALANCE_MOTION_UNEXPECTED:
        {
            return "UNEXP";
        }

        case BALANCE_MOTION_IDLE:
        default:
        {
            return "idle";
        }
    }
}

uint32_t balance_get_stall_fault_count (void)
{
    return s_stall_fault_count;
}

static uint16_t balance_saturate_u16 (uint32_t value)
{
    return (value > 0xFFFFU) ? (uint16_t) 0xFFFFU : (uint16_t) value;
}

/* See BALANCE_CHECK_* in balance.h. Called on armed steps only. */
static void balance_check_step (float error_deg,
                                float p_term,
                                float rest_left,
                                float rest_right,
                                motors_command_t const * p_command)
{
    if (fabsf(error_deg) <= BALANCE_CHECK_MIN_ERROR_DEG)
    {
        return;
    }

    float const p_magnitude = fabsf(p_term);

    if ((fabsf(rest_left) >= (BALANCE_CHECK_P_DOMINANCE * p_magnitude)) ||
        (fabsf(rest_right) >= (BALANCE_CHECK_P_DOMINANCE * p_magnitude)))
    {
        s_check.skipped++;

        return;
    }

    motors_direction_t const expected = (error_deg > 0.0f) ? MOTORS_DIR_FWD : MOTORS_DIR_REV;
    motors_direction_t const left     = motors_direction(p_command->left_reverse, p_command->left_duty_counts);
    motors_direction_t const right    = motors_direction(p_command->right_reverse, p_command->right_duty_counts);

    s_check.samples++;

    if ((left != expected) || (right != expected))
    {
        s_check.sign_err++;
    }
}

void balance_init (void)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    s_enabled              = false;
    s_dry_run              = BALANCE_DRY_RUN_AT_BOOT;
    s_params.kp            = BALANCE_KP_DEFAULT;
    s_params.kd            = BALANCE_KD_DEFAULT;
    s_params.setpoint_deg  = BALANCE_SETPOINT_DEG_DEFAULT;
    s_params.speed_kp      = BALANCE_SPEED_KP_DEFAULT;
    s_params.speed_ki      = BALANCE_SPEED_KI_DEFAULT;
    s_params.turn_kd       = BALANCE_TURN_KD_DEFAULT;
    s_targets.speed        = 0.0f;
    s_targets.turn         = 0.0f;
    balance_reset_state();
    s_was_armed = false;
    (void) memset(&s_check, 0, sizeof(s_check));
    (void) memset(&s_motion_left, 0, sizeof(s_motion_left));
    (void) memset(&s_motion_right, 0, sizeof(s_motion_right));
    (void) memset(&s_status, 0, sizeof(s_status));
    s_status.dry_run = BALANCE_DRY_RUN_AT_BOOT;
    encoders_get_totals(&s_enc_left_prev_total, &s_enc_right_prev_total);
    __set_PRIMASK(primask);
}

void balance_set_enabled (bool enabled)
{
    s_enabled = enabled;
}

bool balance_set_dry_run (bool dry_run)
{
    /* Main loop only, which is also the only writer of the safety state, so
     * the state can't become ACTIVE between this check and the write. */
    if (SAFETY_STATE_ACTIVE == safety_get_state())
    {
        return false;
    }

    s_dry_run = dry_run;

    return true;
}

bool balance_get_dry_run (void)
{
    return s_dry_run;
}

void balance_step (float angle_deg, float gyro_x_dps, float gyro_z_dps)
{
    s_step_count++;

    /* New encoder edges since the last step, signed by the previous command
     * (stock: pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a). */
    uint32_t left_total;
    uint32_t right_total;
    encoders_get_totals(&left_total, &right_total);

    uint32_t const left_new  = left_total - s_enc_left_prev_total;
    uint32_t const right_new = right_total - s_enc_right_prev_total;
    s_enc_left_prev_total  = left_total;
    s_enc_right_prev_total = right_total;

    bool const armed   = s_enabled && (SAFETY_STATE_ACTIVE == safety_get_state());
    bool const dry_run = s_dry_run;

    balance_params_t const p = s_params; /* The main loop can't run inside this ISR, so this is consistent. */

    bool const  speed_loop_on = (p.speed_kp > 0.0f) || (p.speed_ki > 0.0f); /* Both are >= 0 (console ranges). */
    float const error_deg     = angle_deg - p.setpoint_deg;
    float const p_term        = p.kp * error_deg;
    float const d_term        = p.kd * gyro_x_dps;
    float const balance_out   = p_term + d_term;

    motors_command_t * const p_command = &s_command;
    bool                     driving   = false;

    if (!armed)
    {
        balance_reset_state();
        s_was_armed           = false;
        s_status_speed_target = 0.0f;
        s_status_turn_target  = 0.0f;
        motors_plan(0, 0, p_command); /* STOP: nothing is commanded while not armed. */
    }
    else
    {
        if (!s_was_armed)
        {
            /* First step of a new run. The controller state (speed filter,
             * integral and output, turn term, 40 ms window phase, encoder
             * windows and position, previous PWM) is already at reset:
             * balance_reset_state() runs on every step while not armed. The
             * D term has no history (kd * gyro of this step). So every arm -
             * including a stand-up from rest - starts clean. The logic
             * check and the black box start from zero too. */
            s_was_armed   = true;
            s_armed_steps = 0U;
            s_armed_total = 0U;

            blackbox_run_info_t const info =
            {
                .kp            = p.kp,
                .kd            = p.kd,
                .setpoint_deg  = p.setpoint_deg,
                .speed_kp      = p.speed_kp,
                .speed_ki      = p.speed_ki,
                .turn_kd       = p.turn_kd,
                .arm_angle_deg = angle_deg,
                .dry_run       = dry_run,
                .speed_loop_on = speed_loop_on,
            };
            blackbox_start(&info);
            (void) memset(&s_check, 0, sizeof(s_check));
            (void) memset(&s_motion_left.count, 0, sizeof(s_motion_left.count));
            (void) memset(&s_motion_right.count, 0, sizeof(s_motion_right.count));
        }

        int32_t const left_signed  = (s_pwm_left < 0.0f) ? -(int32_t) left_new : (int32_t) left_new;
        int32_t const right_signed = (s_pwm_right < 0.0f) ? -(int32_t) right_new : (int32_t) right_new;

        s_enc_left_window      += left_signed;
        s_enc_right_window     += right_signed;
        s_enc_left_window_abs  += left_new;
        s_enc_right_window_abs += right_new;
        s_enc_left_position    += left_signed;
        s_enc_right_position   += right_signed;

        s_speed_step_count++;

        /* Drive/turn setpoints only once the soft start is over (see balance_set_targets()). */
        bool const  targets_on   = (s_armed_steps >= (BALANCE_SPEED_START_DELAY_STEPS + BALANCE_SPEED_START_RAMP_STEPS));
        float const speed_target = targets_on ? s_targets.speed : 0.0f;
        float const turn_target  = targets_on ? s_targets.turn : 0.0f;

        s_status_speed_target = speed_target;
        s_status_turn_target  = turn_target;

        if (s_speed_step_count >= BALANCE_SPEED_PERIOD_STEPS)
        {
            s_speed_step_count = 0U;

            float const car_speed = (float) (s_enc_left_window + s_enc_right_window) * 0.5f;
            s_enc_left_window       = 0;
            s_enc_right_window      = 0;
            s_enc_left_last_window  = balance_saturate_u16(s_enc_left_window_abs);
            s_enc_right_last_window = balance_saturate_u16(s_enc_right_window_abs);
            balance_motion_judge(&s_motion_left, s_enc_left_window_abs);
            balance_motion_judge(&s_motion_right, s_enc_right_window_abs);
            s_enc_left_window_abs   = 0U;
            s_enc_right_window_abs  = 0U;

            s_speed_filter = (s_speed_filter * BALANCE_SPEED_FILTER_OLD) + (car_speed * BALANCE_SPEED_FILTER_NEW);

            /* Stock: integral += speed_filter; integral += -setting_car_speed.
             * Difference from stock - anti-windup: the integral enters the
             * command as +ski*integral, so while both wheels are already
             * clamped at full duty in the direction this window's term would
             * push it further, don't integrate. Otherwise it winds up to the
             * clamp during a saturated run-away and the robot can't recover. */
            float const integrand     = s_speed_filter - speed_target;
            bool const  saturated_fwd = (s_pwm_left >= BALANCE_PWM_LIMIT_8BIT) && (s_pwm_right >= BALANCE_PWM_LIMIT_8BIT);
            bool const  saturated_rev = (s_pwm_left <= -BALANCE_PWM_LIMIT_8BIT) && (s_pwm_right <= -BALANCE_PWM_LIMIT_8BIT);
            bool const  windup        = (saturated_fwd && (integrand > 0.0f)) || (saturated_rev && (integrand < 0.0f));

            if (!windup)
            {
                s_speed_integral = balance_clamp(s_speed_integral + integrand, BALANCE_SPEED_INTEGRAL_LIMIT);
            }

            s_speed_out = (-p.speed_kp * s_speed_filter) - (p.speed_ki * s_speed_integral);

            /* Stock: setting_turn_speed + kd_turn * Gyro_z, with Gyro_z = -gz / 131. */
            float const yaw_rate_stock_dps = -gyro_z_dps;
            s_turn_out = turn_target + (p.turn_kd * yaw_rate_stock_dps);
        }

        if (s_armed_steps < (BALANCE_SPEED_START_DELAY_STEPS + BALANCE_SPEED_START_RAMP_STEPS))
        {
            s_armed_steps++;
        }

        if ((!speed_loop_on) || (s_armed_steps <= BALANCE_SPEED_START_DELAY_STEPS))
        {
            /* skp = ski = 0, or still inside the soft-start delay: hold the
             * speed loop at zero every step, so nothing it accumulated can
             * reach the command. */
            s_speed_filter   = 0.0f;
            s_speed_integral = 0.0f;
            s_speed_out      = 0.0f;
        }

        /* Soft start: fade the speed output in after the delay (see balance.h). */
        float speed_applied = s_speed_out;

        if (s_armed_steps < (BALANCE_SPEED_START_DELAY_STEPS + BALANCE_SPEED_START_RAMP_STEPS))
        {
            uint32_t const ramp_steps = (s_armed_steps > BALANCE_SPEED_START_DELAY_STEPS)
                                            ? (s_armed_steps - BALANCE_SPEED_START_DELAY_STEPS) : 0U;
            speed_applied *= (float) ramp_steps / (float) BALANCE_SPEED_START_RAMP_STEPS;
        }

        s_pwm_left  = balance_clamp(balance_out - speed_applied - s_turn_out, BALANCE_PWM_LIMIT_8BIT);
        s_pwm_right = balance_clamp(balance_out - speed_applied + s_turn_out, BALANCE_PWM_LIMIT_8BIT);

        motors_plan(balance_to_permille(s_pwm_left), balance_to_permille(s_pwm_right), p_command);

        if (!dry_run)
        {
            driving = motors_apply(p_command);
        }

        if (0U == (s_armed_total % BLACKBOX_PERIOD_STEPS))
        {
            balance_record_step(angle_deg, gyro_x_dps, balance_out, speed_applied, p_command);
        }

        s_armed_total++;

        balance_motion_accumulate(&s_motion_left, driving, p_command->left_duty_counts);
        balance_motion_accumulate(&s_motion_right, driving, p_command->right_duty_counts);

        if (speed_loop_on)
        {
            s_check.speed_loop_on = true;
        }
        else
        {
            balance_check_step(error_deg, p_term, d_term - s_turn_out, d_term + s_turn_out, p_command);
        }
    }

    s_status.angle_deg        = angle_deg;
    s_status.gyro_dps         = gyro_x_dps;
    s_status.setpoint_deg     = p.setpoint_deg;
    s_status.bal_8bit         = balance_out; /* Unclamped, for tuning. */
    s_status.spd_8bit         = s_speed_out;
    s_status.turn_8bit        = s_turn_out;
    s_status.speed_target     = s_status_speed_target;
    s_status.turn_target      = s_status_turn_target;
    s_status.speed_integral   = s_speed_integral;
    s_status.command          = *p_command;
    s_status.enc_left_window  = s_enc_left_last_window;
    s_status.enc_right_window = s_enc_right_last_window;
    s_status.enc_left_counts  = s_enc_left_position;
    s_status.enc_right_counts = s_enc_right_position;
    s_status.armed            = armed;
    s_status.dry_run          = dry_run;
    s_status.driving          = driving;
    s_status.speed_loop_on    = speed_loop_on;
    s_status.check            = s_check;
    s_status.motion_left        = s_motion_left.last;
    s_status.motion_right       = s_motion_right.last;
    s_status.motion_count_left  = s_motion_left.count;
    s_status.motion_count_right = s_motion_right.count;
    s_status.step_count       = s_step_count;
}

void balance_set_targets (float speed_target, float turn_target)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    s_targets.speed = speed_target;
    s_targets.turn  = turn_target;
    __set_PRIMASK(primask);
}

void balance_get_params (balance_params_t * p_params)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    *p_params = s_params;
    __set_PRIMASK(primask);
}

void balance_set_params (balance_params_t const * p_params)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    s_params = *p_params;
    __set_PRIMASK(primask);
}

void balance_get_status (balance_status_t * p_status)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    *p_status = s_status;
    __set_PRIMASK(primask);
}
