/*
 * telemetry.h
 *
 * SEGGER RTT-based telemetry output. No UART is used; all diagnostic text
 * goes out over RTT so the J-Link OB can be read by RTT Viewer / J-Link
 * GDB Server without consuming a UART or Arduino header pin.
 *
 * Two channels: channel 0 carries the human-readable status lines
 * (telemetry_printf), channel 1 carries the compact 200 Hz CSV stream for
 * tools/telemetry/ (telemetry_data_printf) - kept separate so the Python
 * capture tool doesn't have to parse the readable text, and the readable
 * text doesn't get interleaved with/throttled by the high-rate stream.
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdarg.h>

/* Call once at startup, before the first telemetry_printf()/telemetry_data_printf().
 * Initializes RTT and configures channel 1's up-buffer. */
void telemetry_init(void);

/*
 * Formatted RTT output on channel 0 (human-readable). Non-blocking: if the
 * host isn't reading fast enough, the RTT buffer drops data
 * (SEGGER_RTT_MODE_NO_BLOCK_SKIP) rather than stalling the caller. Safe to
 * call from the 1 kHz control ISR for short messages, but prefer
 * buffering telemetry and flushing from hal_entry's idle loop to keep ISR
 * execution time bounded.
 */
int telemetry_printf(const char * p_format, ...);

/* Same semantics as telemetry_printf(), but on channel 1 - the compact
 * CSV stream tools/telemetry/ captures. */
int telemetry_data_printf(const char * p_format, ...);

#endif /* TELEMETRY_H_ */
