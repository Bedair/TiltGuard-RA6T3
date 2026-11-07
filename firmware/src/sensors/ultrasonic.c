/*
 * ultrasonic.c
 */

#include "ultrasonic.h"
#include "hal_data.h"

#define ULTRASONIC_TRIG_PIN              (BSP_IO_PORT_01_PIN_09) /* P109 / D11. */

#define ULTRASONIC_TRIGGER_PERIOD_TICKS  (50U)                   /* 50 ms. */
#define ULTRASONIC_TRIGGER_PULSE_CYCLES  (2000U)                 /* 10 us @ 200 MHz. */
#define ULTRASONIC_ECHO_TIMEOUT_US       (30000U)                /* 30 ms. */
#define ULTRASONIC_ECHO_TIMEOUT_CYCLES   (ULTRASONIC_ECHO_TIMEOUT_US * 200U)
#define ULTRASONIC_CYCLES_PER_US         (200U)
#define ULTRASONIC_US_PER_CM             (58.0f)

static uint32_t s_subtick;

static volatile bool     s_echo_pending;
static volatile bool     s_echo_rising_seen;
static volatile uint32_t s_echo_rising_cyccnt;
static volatile uint32_t s_echo_trigger_cyccnt; /* Timeout reference. */

static volatile float    s_distance_cm;
static volatile bool     s_distance_valid;
static volatile uint32_t s_timeout_count;
static volatile uint32_t s_sequence;

/* ISR entry count - see ultrasonic.h. */
static volatile uint32_t s_echo_isr_entries;

fsp_err_t ultrasonic_init (void)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(ULTRASONIC_TRIG_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();

    fsp_err_t err = R_ICU_ExternalIrqOpen(&g_irq_echo_ctrl, &g_irq_echo_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return R_ICU_ExternalIrqEnable(&g_irq_echo_ctrl);
}

static void ultrasonic_fire_trigger (void)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(ULTRASONIC_TRIG_PIN, BSP_IO_LEVEL_HIGH);

    uint32_t const pulse_start = DWT->CYCCNT;

    while ((DWT->CYCCNT - pulse_start) < ULTRASONIC_TRIGGER_PULSE_CYCLES)
    {
        ;
    }

    R_BSP_PinWrite(ULTRASONIC_TRIG_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();

    s_echo_rising_seen     = false;
    s_echo_trigger_cyccnt  = DWT->CYCCNT;
    s_echo_pending         = true;
}

void ultrasonic_on_tick (void)
{
    s_subtick++;

    if ((s_subtick >= ULTRASONIC_TRIGGER_PERIOD_TICKS) && !s_echo_pending)
    {
        s_subtick = 0U;
        ultrasonic_fire_trigger();
    }

    if (s_echo_pending)
    {
        uint32_t const elapsed = DWT->CYCCNT - s_echo_trigger_cyccnt;

        if (elapsed > ULTRASONIC_ECHO_TIMEOUT_CYCLES)
        {
            s_echo_pending   = false;
            s_distance_valid = false;
            s_timeout_count++;
        }
    }
}

void ultrasonic_echo_isr (external_irq_callback_args_t * p_args)
{
    s_echo_isr_entries++; /* First thing, unconditional. */

    (void) p_args;

    if (!s_echo_pending)
    {
        /* No measurement in flight - ignore (noise, or a late edge after a timeout). */
        return;
    }

    uint32_t const now = DWT->CYCCNT;

    if (!s_echo_rising_seen)
    {
        s_echo_rising_cyccnt = now;
        s_echo_rising_seen   = true;

        return;
    }

    uint32_t const pulse_cycles = now - s_echo_rising_cyccnt;
    uint32_t const pulse_us     = pulse_cycles / ULTRASONIC_CYCLES_PER_US;

    s_distance_cm    = (float) pulse_us / ULTRASONIC_US_PER_CM;
    s_distance_valid = true;
    s_echo_pending   = false;
    s_sequence++;
}

void ultrasonic_get_snapshot (ultrasonic_snapshot_t * p_snapshot)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    p_snapshot->distance_cm     = s_distance_cm;
    p_snapshot->distance_valid  = s_distance_valid;
    p_snapshot->timeout_count   = s_timeout_count;
    p_snapshot->sequence        = s_sequence;
    p_snapshot->echo_isr_entries = s_echo_isr_entries;
    __set_PRIMASK(primask);
}
