/*
 * button.c
 */

#include <stdbool.h>
#include "button.h"
#include "hal_data.h"

#define BUTTON_PIN (BSP_IO_PORT_03_PIN_04) /* P304 / S1. */

/* ISR entry count - diagnostic only, debounce below doesn't depend on
 * which edge fired. */
static volatile uint32_t s_isr_entries;

/* Debounce state, touched only from button_on_tick() (ISR context, called
 * from control_tick_isr) - no locking needed for s_press_count since it's
 * a single word, read-only from the main loop (same pattern as
 * control_tick_count()). */
static uint32_t          s_candidate_ticks;
static bool               s_candidate_pressed;
static bool               s_stable_pressed;
static volatile uint32_t s_press_count;

fsp_err_t button_init (void)
{
    s_isr_entries        = 0U;
    s_candidate_ticks     = 0U;
    s_candidate_pressed   = false;
    s_stable_pressed      = false;
    s_press_count         = 0U;

    fsp_err_t err = R_ICU_ExternalIrqOpen(&g_irq_button_ctrl, &g_irq_button_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return R_ICU_ExternalIrqEnable(&g_irq_button_ctrl);
}

void button_isr (external_irq_callback_args_t * p_args)
{
    s_isr_entries++; /* First thing, unconditional. */

    (void) p_args;
}

void button_on_tick (void)
{
    /* Active-low: press pulls the pin to GND. Flip this comparison if S1
     * turns out to be wired active-high on this board. */
    bool const raw_pressed = (0U == R_BSP_PinRead(BUTTON_PIN));

    if (raw_pressed == s_candidate_pressed)
    {
        if (s_candidate_ticks < BUTTON_DEBOUNCE_TICKS)
        {
            s_candidate_ticks++;
        }
    }
    else
    {
        s_candidate_pressed = raw_pressed;
        s_candidate_ticks   = 1U;
    }

    if ((s_candidate_ticks >= BUTTON_DEBOUNCE_TICKS) && (s_candidate_pressed != s_stable_pressed))
    {
        s_stable_pressed = s_candidate_pressed;

        if (s_stable_pressed)
        {
            s_press_count++; /* Debounced press edge - this is the "short press" event. */
        }
    }
}

uint32_t button_get_press_count (void)
{
    return s_press_count;
}

uint32_t button_get_isr_entries (void)
{
    return s_isr_entries;
}
