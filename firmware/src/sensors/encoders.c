/*
 * encoders.c
 */

#include "encoders.h"
#include "hal_data.h"

#define ENCODER_SPEED_WINDOW_TICKS (5U)   /* 5 ms at the 1 kHz tick. */
#define ENCODER_WINDOWS_PER_SECOND (200.0f)

static volatile uint32_t s_left_total;
static volatile uint32_t s_right_total;
static volatile uint32_t s_left_counts_per_5ms;
static volatile uint32_t s_right_counts_per_5ms;

/* ISR entry counts - see encoders.h. */
static volatile uint32_t s_left_isr_entries;
static volatile uint32_t s_right_isr_entries;

static uint32_t s_subtick;
static uint32_t s_left_prev_total;
static uint32_t s_right_prev_total;

fsp_err_t encoders_init (void)
{
    fsp_err_t err = R_ICU_ExternalIrqOpen(&g_irq_encoder_left_ctrl, &g_irq_encoder_left_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_ICU_ExternalIrqOpen(&g_irq_encoder_right_ctrl, &g_irq_encoder_right_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_ICU_ExternalIrqEnable(&g_irq_encoder_left_ctrl);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_ICU_ExternalIrqEnable(&g_irq_encoder_right_ctrl);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return FSP_SUCCESS;
}

void encoder_left_isr (external_irq_callback_args_t * p_args)
{
    s_left_isr_entries++; /* First thing, unconditional. */

    (void) p_args;

    s_left_total++;
}

void encoder_right_isr (external_irq_callback_args_t * p_args)
{
    s_right_isr_entries++; /* First thing, unconditional. */

    (void) p_args;

    s_right_total++;
}

void encoders_on_tick (void)
{
    s_subtick++;

    if (s_subtick < ENCODER_SPEED_WINDOW_TICKS)
    {
        return;
    }

    s_subtick = 0U;

    uint32_t const left_now  = s_left_total;
    uint32_t const right_now = s_right_total;

    s_left_counts_per_5ms  = left_now - s_left_prev_total;
    s_right_counts_per_5ms = right_now - s_right_prev_total;

    s_left_prev_total  = left_now;
    s_right_prev_total = right_now;
}

void encoders_get_snapshot (encoders_snapshot_t * p_snapshot)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    p_snapshot->left_total           = s_left_total;
    p_snapshot->right_total          = s_right_total;
    p_snapshot->left_counts_per_5ms  = s_left_counts_per_5ms;
    p_snapshot->right_counts_per_5ms = s_right_counts_per_5ms;
    p_snapshot->left_isr_entries     = s_left_isr_entries;
    p_snapshot->right_isr_entries    = s_right_isr_entries;
    __set_PRIMASK(primask);

    p_snapshot->left_rev_per_s  =
        ((float) p_snapshot->left_counts_per_5ms * ENCODER_WINDOWS_PER_SECOND) / COUNTS_PER_WHEEL_REV;
    p_snapshot->right_rev_per_s =
        ((float) p_snapshot->right_counts_per_5ms * ENCODER_WINDOWS_PER_SECOND) / COUNTS_PER_WHEEL_REV;
}
