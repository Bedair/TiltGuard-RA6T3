/*
 * telemetry.c
 */

#include <math.h>
#include "telemetry.h"
#include "segger_rtt/SEGGER_RTT.h"

/* 100 Hz x ~70-byte CSV lines = ~7 kB/s; 2 kB gives ~280 ms of slack if the
 * host stalls before lines start being dropped (NO_BLOCK_SKIP). */
#define TELEMETRY_DATA_BUFFER_SIZE (2048U)

static char s_data_up_buffer[TELEMETRY_DATA_BUFFER_SIZE];

void telemetry_init (void)
{
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1, "data", s_data_up_buffer, sizeof(s_data_up_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

int telemetry_printf (const char * p_format, ...)
{
    va_list param_list;
    int written;

    va_start(param_list, p_format);
    written = SEGGER_RTT_vprintf(0, p_format, &param_list);
    va_end(param_list);

    return written;
}

void telemetry_line_begin (telemetry_line_t * p_line)
{
    p_line->length   = 0U;
    p_line->overflow = false;
}

void telemetry_line_char (telemetry_line_t * p_line, char c)
{
    /* Keep room for the "\r\n" telemetry_line_send() appends. */
    if (p_line->length < (TELEMETRY_LINE_MAX - 2U))
    {
        p_line->text[p_line->length] = c;
        p_line->length++;
    }
    else
    {
        p_line->overflow = true;
    }
}

void telemetry_line_u32 (telemetry_line_t * p_line, uint32_t value)
{
    char     digits[10]; /* 4294967295 */
    unsigned count = 0U;

    do
    {
        digits[count] = (char) ('0' + (value % 10U));
        count++;
        value /= 10U;
    } while (0U != value);

    while (count > 0U)
    {
        count--;
        telemetry_line_char(p_line, digits[count]);
    }
}

void telemetry_line_i32 (telemetry_line_t * p_line, int32_t value)
{
    if (value < 0)
    {
        telemetry_line_char(p_line, '-');
        telemetry_line_u32(p_line, (uint32_t) (-(value + 1)) + 1U); /* Safe for INT32_MIN. */
    }
    else
    {
        telemetry_line_u32(p_line, (uint32_t) value);
    }
}

void telemetry_line_str (telemetry_line_t * p_line, const char * p_text)
{
    while ('\0' != *p_text)
    {
        telemetry_line_char(p_line, *p_text);
        p_text++;
    }
}

static void telemetry_line_fixed_impl (telemetry_line_t * p_line, float value, unsigned decimals, bool force_sign);

void telemetry_line_fixed (telemetry_line_t * p_line, float value, unsigned decimals)
{
    telemetry_line_fixed_impl(p_line, value, decimals, false);
}

void telemetry_line_fixed_signed (telemetry_line_t * p_line, float value, unsigned decimals)
{
    telemetry_line_fixed_impl(p_line, value, decimals, true);
}

static void telemetry_line_fixed_impl (telemetry_line_t * p_line, float value, unsigned decimals, bool force_sign)
{
    static const uint32_t s_scale[] = { 1U, 10U, 100U, 1000U, 10000U };

    if (decimals > 4U)
    {
        decimals = 4U;
    }

    float const magnitude = fabsf(value);

    if ((!isfinite(value)) || (magnitude > 100000.0f))
    {
        p_line->overflow = true; /* Never expected for telemetry values; don't send garbage. */

        return;
    }

    /* Round once, at the last printed digit, so "-0.004" with 2 decimals is "0.00", not "-0.00". */
    uint32_t const scaled = (uint32_t) lroundf(magnitude * (float) s_scale[decimals]);

    if ((value < 0.0f) && (0U != scaled))
    {
        telemetry_line_char(p_line, '-');
    }
    else if (force_sign)
    {
        telemetry_line_char(p_line, '+');
    }
    else
    {
        /* Unsigned style: no '+'. */
    }

    telemetry_line_u32(p_line, scaled / s_scale[decimals]);

    if (decimals > 0U)
    {
        uint32_t frac = scaled % s_scale[decimals];

        telemetry_line_char(p_line, '.');

        for (unsigned i = decimals; i > 0U; i--)
        {
            uint32_t const place = s_scale[i - 1U];
            telemetry_line_char(p_line, (char) ('0' + (frac / place)));
            frac %= place;
        }
    }
}

bool telemetry_line_send (telemetry_line_t * p_line, unsigned channel)
{
    if (p_line->overflow)
    {
        return false;
    }

    p_line->text[p_line->length]      = '\r';
    p_line->text[p_line->length + 1U] = '\n';

    unsigned const total   = p_line->length + 2U;
    unsigned const written = SEGGER_RTT_Write(channel, p_line->text, total);

    return written == total;
}

unsigned telemetry_read (char * p_buffer, unsigned size)
{
    return SEGGER_RTT_Read(0U, p_buffer, size);
}

unsigned telemetry_write_space (unsigned channel)
{
    return SEGGER_RTT_GetAvailWriteSpace(channel);
}

int telemetry_data_printf (const char * p_format, ...)
{
    va_list param_list;
    int written;

    va_start(param_list, p_format);
    written = SEGGER_RTT_vprintf(1, p_format, &param_list);
    va_end(param_list);

    return written;
}
