/*
 * telemetry.h
 *
 * SEGGER RTT-based telemetry output. No UART is used; all diagnostic text
 * goes out over RTT so the J-Link OB can be read by RTT Viewer / J-Link
 * GDB Server without consuming a UART or Arduino header pin.
 *
 * Two channels: channel 0 carries the human-readable status lines
 * (telemetry_printf), channel 1 carries the compact 100 Hz CSV stream for
 * tools/telemetry/ (telemetry_data_printf) - kept separate so the Python
 * capture tool doesn't have to parse the readable text, and the readable
 * text doesn't get interleaved with/throttled by the high-rate stream.
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

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

/*
 * Line builder for the CSV and the armed status line: formats into a caller-owned buffer with
 * no printf (whose 64-byte buffer and ~280-byte call chain dominate main-
 * stack use), then sends the line with one RTT write - so in NO_BLOCK_SKIP
 * mode a row is either sent whole or dropped whole, never split. Not
 * reentrant per line object; main loop only.
 */
#define TELEMETRY_LINE_MAX (160U)

typedef struct st_telemetry_line
{
    char     text[TELEMETRY_LINE_MAX];
    unsigned length;
    bool     overflow; /* Set if anything didn't fit; the line is then not sent. */
} telemetry_line_t;

void telemetry_line_begin(telemetry_line_t * p_line);
void telemetry_line_char(telemetry_line_t * p_line, char c);
void telemetry_line_u32(telemetry_line_t * p_line, uint32_t value);
void telemetry_line_i32(telemetry_line_t * p_line, int32_t value);

void telemetry_line_str(telemetry_line_t * p_line, const char * p_text);

/* Fixed-point decimal, rounded: value 1.237 with 2 decimals -> "1.24". decimals 0..4. */
void telemetry_line_fixed(telemetry_line_t * p_line, float value, unsigned decimals);

/* Same, always signed: "+1.24", "-0.50", and "+0.0" for anything that rounds to zero. */
void telemetry_line_fixed_signed(telemetry_line_t * p_line, float value, unsigned decimals);

/* Appends "\r\n" and writes the line to RTT up-channel `channel` in one call.
 * Returns false if the line overflowed or RTT had no room (line dropped). */
bool telemetry_line_send(telemetry_line_t * p_line, unsigned channel);

/* Non-blocking read from RTT down-channel 0 (host -> target, typed in RTT
 * Viewer). Returns the number of bytes copied (0 if nothing is pending). */
unsigned telemetry_read(char * p_buffer, unsigned size);

#endif /* TELEMETRY_H_ */
