/*
 * telemetry.h
 *
 * SEGGER RTT-based telemetry output. No UART is used; all diagnostic text
 * goes out over RTT channel 0 so the J-Link OB can be read by RTT Viewer /
 * J-Link GDB Server without consuming a UART or Arduino header pin.
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdarg.h>

/* Call once at startup, before the first telemetry_printf(). */
void telemetry_init(void);

/*
 * Formatted RTT output. Non-blocking: if the host isn't reading fast enough,
 * the RTT buffer drops data (SEGGER_RTT_MODE_NO_BLOCK_SKIP) rather than
 * stalling the caller. Safe to call from the 1 kHz control ISR for short
 * messages, but prefer buffering telemetry and flushing from hal_entry's
 * idle loop to keep ISR execution time bounded.
 */
int telemetry_printf(const char * p_format, ...);

#endif /* TELEMETRY_H_ */
