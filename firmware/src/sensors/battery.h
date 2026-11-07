/*
 * battery.h
 *
 * Battery sense on AN002 (P002), 10 k / 1.5 k divider: V_bat = V_pin * 11.5/1.5.
 * Scanned every 100 ms via a software-triggered single ADC scan; the result
 * is exponentially low-pass filtered (alpha = 0.1, ~1 s settling at the
 * 100 ms sample rate) before the full/low thresholds are applied, so a
 * single noisy sample can't flip the flag.
 *
 * Hardware confirmed working 2026-11: the 12-bit/3.3 V/divider conversion
 * itself was correct (no calibration gain needed). A ~10% high reading
 * turned out to be a ~0.13 V ground offset between the robot and FPB
 * grounds (see docs/hardware-findings.md) - fixed with extra GND wires,
 * not in software. The first ADC conversion after boot is unsettled
 * (~1.72 V observed); BATTERY_WARMUP_SCANS_TO_DISCARD below drops the first
 * few scans so the filter is seeded from a real reading.
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdint.h>
#include "bsp_api.h"
#include "r_adc_api.h"

#define BATTERY_FULL_THRESHOLD_V (7.8f) /* Stock firmware threshold. */
#define BATTERY_LOW_THRESHOLD_V  (7.0f) /* Stock firmware threshold. */

typedef enum e_battery_status
{
    BATTERY_STATUS_LOW,
    BATTERY_STATUS_NORMAL,
    BATTERY_STATUS_FULL,
} battery_status_t;

typedef struct st_battery_snapshot
{
    float            voltage_v;     /* Filtered V_bat. */
    float            voltage_raw_v; /* Most recent single-sample V_bat, before filtering. */
    battery_status_t status;
    uint32_t         scan_count;
    uint32_t         scan_failure_count;
} battery_snapshot_t;

/* Checks every fsp_err_t. */
fsp_err_t battery_init(void);

/* Call every 1 kHz tick from control_tick_isr. Starts a scan every 100
 * ticks. Non-blocking - R_ADC_ScanStart() only kicks the scan. */
void battery_on_tick(void);

/* Consistent snapshot, safe to call from the main loop. */
void battery_get_snapshot(battery_snapshot_t * p_snapshot);

#ifndef battery_adc_callback
void battery_adc_callback(adc_callback_args_t * p_args);
#endif

#endif /* BATTERY_H_ */
