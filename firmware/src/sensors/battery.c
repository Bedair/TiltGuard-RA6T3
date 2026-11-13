/*
 * battery.c
 */

#include <stdbool.h>
#include "battery.h"
#include "hal_data.h"

#define BATTERY_SCAN_PERIOD_TICKS (100U)  /* 100 ms. */
#define BATTERY_ADC_VREF_V        (3.3f)
#define BATTERY_ADC_MAX_COUNT     (4095.0f) /* 12-bit. */
#define BATTERY_DIVIDER_RATIO     (11.5f / 1.5f)
#define BATTERY_FILTER_ALPHA      (0.1f)

/*
 * Confirmed on hardware: the first ADC conversion after boot reads an
 * unsettled ~1.72 V (sample-and-hold / reference not fully settled yet).
 * Discard the first few scans so the filter seeds from a real reading.
 */
#define BATTERY_WARMUP_SCANS_TO_DISCARD (4U)

static uint32_t s_subtick;
static uint32_t s_warmup_scans_remaining;
static bool     s_filter_initialized;

static volatile float    s_filtered_voltage_v;
static volatile float    s_raw_voltage_v; /* Most recent single-sample V_bat, unfiltered. */
static volatile uint32_t s_scan_count;
static volatile uint32_t s_scan_failure_count;

fsp_err_t battery_init (void)
{
    s_subtick                = 0U;
    s_warmup_scans_remaining = BATTERY_WARMUP_SCANS_TO_DISCARD;
    s_filter_initialized     = false;
    s_filtered_voltage_v     = 0.0f;
    s_raw_voltage_v          = 0.0f;
    s_scan_count             = 0U;
    s_scan_failure_count     = 0U;

    fsp_err_t err = R_ADC_Open(&g_adc_battery_ctrl, &g_adc_battery_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return R_ADC_ScanCfg(&g_adc_battery_ctrl, &g_adc_battery_channel_cfg);
}

void battery_on_tick (void)
{
    s_subtick++;

    if (s_subtick < BATTERY_SCAN_PERIOD_TICKS)
    {
        return;
    }

    s_subtick = 0U;

    fsp_err_t const err = R_ADC_ScanStart(&g_adc_battery_ctrl);

    if (FSP_SUCCESS != err)
    {
        s_scan_failure_count++;
    }
}

void battery_adc_callback (adc_callback_args_t * p_args)
{
    if (ADC_EVENT_SCAN_COMPLETE != p_args->event)
    {
        return;
    }

    uint16_t  raw_count = 0U;
    fsp_err_t err       = R_ADC_Read(&g_adc_battery_ctrl, ADC_CHANNEL_16, &raw_count);

    if (FSP_SUCCESS != err)
    {
        s_scan_failure_count++;

        return;
    }

    float const v_pin = ((float) raw_count / BATTERY_ADC_MAX_COUNT) * BATTERY_ADC_VREF_V;
    float const v_bat = v_pin * BATTERY_DIVIDER_RATIO;

    s_raw_voltage_v = v_bat;

    if (s_warmup_scans_remaining > 0U)
    {
        s_warmup_scans_remaining--;
        s_scan_count++;

        return;
    }

    if (!s_filter_initialized)
    {
        s_filtered_voltage_v = v_bat;
        s_filter_initialized = true;
    }
    else
    {
        s_filtered_voltage_v += BATTERY_FILTER_ALPHA * (v_bat - s_filtered_voltage_v);
    }

    s_scan_count++;
}

void battery_get_snapshot (battery_snapshot_t * p_snapshot)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    p_snapshot->voltage_v          = s_filtered_voltage_v;
    p_snapshot->voltage_raw_v      = s_raw_voltage_v;
    p_snapshot->scan_count         = s_scan_count;
    p_snapshot->scan_failure_count = s_scan_failure_count;
    __set_PRIMASK(primask);

    if (p_snapshot->voltage_v >= BATTERY_FULL_THRESHOLD_V)
    {
        p_snapshot->status = BATTERY_STATUS_FULL;
    }
    else if (p_snapshot->voltage_v <= BATTERY_LOW_THRESHOLD_V)
    {
        p_snapshot->status = BATTERY_STATUS_LOW;
    }
    else
    {
        p_snapshot->status = BATTERY_STATUS_NORMAL;
    }
}
