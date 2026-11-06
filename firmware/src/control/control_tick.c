/*
 * control_tick.c
 */

#include "control_tick.h"
#include "hal_data.h"

static volatile uint32_t s_tick_count          = 0U;
static volatile uint32_t s_previous_cyccnt      = 0U;
static volatile uint32_t s_min_interval_cycles  = UINT32_MAX;
static volatile uint32_t s_max_interval_cycles  = 0U;

fsp_err_t control_tick_init (void)
{
    /* Enable the DWT cycle counter (core clock, 200 MHz) used for interval timing. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0U;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    s_tick_count         = 0U;
    s_previous_cyccnt     = DWT->CYCCNT;
    s_min_interval_cycles = UINT32_MAX;
    s_max_interval_cycles = 0U;

    fsp_err_t err = R_GPT_Open(&g_timer_control_tick_ctrl, &g_timer_control_tick_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_GPT_Start(&g_timer_control_tick_ctrl);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return FSP_SUCCESS;
}

uint32_t control_tick_count (void)
{
    return s_tick_count;
}

void control_tick_get_interval_stats (uint32_t * p_min_cycles, uint32_t * p_max_cycles)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    *p_min_cycles = s_min_interval_cycles;
    *p_max_cycles = s_max_interval_cycles;
    __set_PRIMASK(primask);
}

void control_tick_reset_interval_stats (void)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    s_min_interval_cycles = UINT32_MAX;
    s_max_interval_cycles = 0U;
    __set_PRIMASK(primask);
}

void control_tick_isr (timer_callback_args_t * p_args)
{
    if (TIMER_EVENT_CYCLE_END != p_args->event)
    {
        return;
    }

    uint32_t const now      = DWT->CYCCNT;
    uint32_t const interval = now - s_previous_cyccnt;

    s_previous_cyccnt = now;

    if (interval < s_min_interval_cycles)
    {
        s_min_interval_cycles = interval;
    }

    if (interval > s_max_interval_cycles)
    {
        s_max_interval_cycles = interval;
    }

    s_tick_count++;
}
