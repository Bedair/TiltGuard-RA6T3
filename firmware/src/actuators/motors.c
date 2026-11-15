/*
 * motors.c
 */

#include <stdlib.h>
#include "motors.h"
#include "hal_data.h"

#define MOTORS_AIN1_PIN (BSP_IO_PORT_00_PIN_08) /* P008. */
#define MOTORS_BIN1_PIN (BSP_IO_PORT_01_PIN_10) /* P110. */
#define MOTORS_STBY_PIN (BSP_IO_PORT_00_PIN_06) /* P006. */

static int16_t s_last_left_permille;
static int16_t s_last_right_permille;
static volatile uint32_t s_error_count;

static int16_t motors_saturate (int16_t value)
{
    if (value > MOTORS_COMMAND_MAX)
    {
        return (int16_t) MOTORS_COMMAND_MAX;
    }

    if (value < MOTORS_COMMAND_MIN)
    {
        return (int16_t) MOTORS_COMMAND_MIN;
    }

    return value;
}

static uint32_t motors_duty_counts (int16_t permille)
{
    uint32_t const period_counts = g_pwm_motor_left_cfg.period_counts; /* Same for both channels. */
    uint32_t const magnitude     = (uint32_t) abs((int) permille);

    return (magnitude * period_counts) / (uint32_t) MOTORS_COMMAND_MAX;
}

void motors_disable (void)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(MOTORS_STBY_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();

    (void) R_GPT_DutyCycleSet(&g_pwm_motor_left_ctrl, 0U, GPT_IO_PIN_GTIOCA);
    (void) R_GPT_DutyCycleSet(&g_pwm_motor_right_ctrl, 0U, GPT_IO_PIN_GTIOCA);

    s_last_left_permille  = 0;
    s_last_right_permille = 0;
}

fsp_err_t motors_init (void)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(MOTORS_AIN1_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_PinWrite(MOTORS_BIN1_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_PinWrite(MOTORS_STBY_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();

    fsp_err_t err = R_GPT_Open(&g_pwm_motor_left_ctrl, &g_pwm_motor_left_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_GPT_Open(&g_pwm_motor_right_ctrl, &g_pwm_motor_right_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_GPT_Start(&g_pwm_motor_left_ctrl);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_GPT_Start(&g_pwm_motor_right_ctrl);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    motors_disable(); /* Belt-and-suspenders: guarantee the safe state regardless of the above. */

    return FSP_SUCCESS;
}

void motors_plan (int16_t left_permille, int16_t right_permille, motors_command_t * p_command)
{
    int16_t const left  = motors_saturate(left_permille);
    int16_t const right = motors_saturate(right_permille);

    /* Direction mapping matches the ELEGOO stock firmware's balanceCar():
     * AIN1/BIN1 high = reverse, low = forward; magnitude always goes to PWM. */
    p_command->left_permille     = left;
    p_command->right_permille    = right;
    p_command->left_reverse      = (left < 0);
    p_command->right_reverse     = (right < 0);
    p_command->left_duty_counts  = motors_duty_counts(left);
    p_command->right_duty_counts = motors_duty_counts(right);
}

bool motors_apply (motors_command_t const * p_command)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(MOTORS_AIN1_PIN, p_command->left_reverse ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinWrite(MOTORS_BIN1_PIN, p_command->right_reverse ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();

    fsp_err_t err = R_GPT_DutyCycleSet(&g_pwm_motor_left_ctrl, p_command->left_duty_counts, GPT_IO_PIN_GTIOCA);

    if (FSP_SUCCESS != err)
    {
        s_error_count++;
        motors_disable();

        return false;
    }

    err = R_GPT_DutyCycleSet(&g_pwm_motor_right_ctrl, p_command->right_duty_counts, GPT_IO_PIN_GTIOCA);

    if (FSP_SUCCESS != err)
    {
        s_error_count++;
        motors_disable();

        return false;
    }

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(MOTORS_STBY_PIN, BSP_IO_LEVEL_HIGH);
    R_BSP_PinAccessDisable();

    s_last_left_permille  = p_command->left_permille;
    s_last_right_permille = p_command->right_permille;

    return true;
}

void motors_set (int16_t left_permille, int16_t right_permille)
{
    motors_command_t command;

    motors_plan(left_permille, right_permille, &command);
    (void) motors_apply(&command); /* A failure is counted and disables the motors; safety_update() latches FAULT. */
}

motors_direction_t motors_direction (bool reverse, uint32_t duty_counts)
{
    if (0U == duty_counts)
    {
        return MOTORS_DIR_STOP;
    }

    return reverse ? MOTORS_DIR_REV : MOTORS_DIR_FWD;
}

uint16_t motors_duty_permille (uint32_t duty_counts)
{
    uint32_t const period_counts = g_pwm_motor_left_cfg.period_counts;

    if (0U == period_counts)
    {
        return 0U;
    }

    return (uint16_t) (((duty_counts * (uint32_t) MOTORS_COMMAND_MAX) + (period_counts / 2U)) / period_counts);
}

fsp_err_t motors_readback (motors_readback_t * p_readback)
{
    p_readback->stby_high = (BSP_IO_LEVEL_HIGH == R_BSP_PinRead(MOTORS_STBY_PIN));
    p_readback->ain1_high = (BSP_IO_LEVEL_HIGH == R_BSP_PinRead(MOTORS_AIN1_PIN));
    p_readback->bin1_high = (BSP_IO_LEVEL_HIGH == R_BSP_PinRead(MOTORS_BIN1_PIN));

    timer_status_t status;
    fsp_err_t      err = R_GPT_StatusGet(&g_pwm_motor_left_ctrl, &status);

    p_readback->left_timer_running = (FSP_SUCCESS == err) && (TIMER_STATE_COUNTING == status.state);

    if (FSP_SUCCESS != err)
    {
        p_readback->right_timer_running = false;

        return err;
    }

    err = R_GPT_StatusGet(&g_pwm_motor_right_ctrl, &status);

    p_readback->right_timer_running = (FSP_SUCCESS == err) && (TIMER_STATE_COUNTING == status.state);

    return err;
}

const char * motors_direction_name (motors_direction_t direction)
{
    switch (direction)
    {
        case MOTORS_DIR_FWD:
        {
            return "FWD";
        }

        case MOTORS_DIR_REV:
        {
            return "REV";
        }

        case MOTORS_DIR_STOP:
        default:
        {
            return "STOP";
        }
    }
}

uint32_t motors_get_error_count (void)
{
    return s_error_count;
}

void motors_get_command (int16_t * p_left, int16_t * p_right)
{
    *p_left  = s_last_left_permille;
    *p_right = s_last_right_permille;
}
