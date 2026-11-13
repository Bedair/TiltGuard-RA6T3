/*
 * telemetry.c
 */

#include "telemetry.h"
#include "segger_rtt/SEGGER_RTT.h"

#define TELEMETRY_DATA_BUFFER_SIZE (512U) /* A few 200 Hz CSV lines of slack. */

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

int telemetry_data_printf (const char * p_format, ...)
{
    va_list param_list;
    int written;

    va_start(param_list, p_format);
    written = SEGGER_RTT_vprintf(1, p_format, &param_list);
    va_end(param_list);

    return written;
}
