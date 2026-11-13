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

void motors_set (int16_t left_permille, int16_t right_permille)
{
    int16_t const left  = motors_saturate(left_permille);
    int16_t const right = motors_saturate(right_permille);

    /* Direction mapping matches the ELEGOO stock firmware's balanceCar():
     * AIN1/BIN1 high = reverse, low = forward; magnitude always goes to PWM. */
    bsp_io_level_t const ain1_level = (left < 0) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
    bsp_io_level_t const bin1_level = (right < 0) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(MOTORS_AIN1_PIN, ain1_level);
    R_BSP_PinWrite(MOTORS_BIN1_PIN, bin1_level);
    R_BSP_PinAccessDisable();

    fsp_err_t err = R_GPT_DutyCycleSet(&g_pwm_motor_left_ctrl, motors_duty_counts(left), GPT_IO_PIN_GTIOCA);

    if (FSP_SUCCESS != err)
    {
        motors_disable();

        return;
    }

    err = R_GPT_DutyCycleSet(&g_pwm_motor_right_ctrl, motors_duty_counts(right), GPT_IO_PIN_GTIOCA);

    if (FSP_SUCCESS != err)
    {
        motors_disable();

        return;
    }

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(MOTORS_STBY_PIN, BSP_IO_LEVEL_HIGH);
    R_BSP_PinAccessDisable();

    s_last_left_permille  = left;
    s_last_right_permille = right;
}

void motors_get_command (int16_t * p_left, int16_t * p_right)
{
    *p_left  = s_last_left_permille;
    *p_right = s_last_right_permille;
}
