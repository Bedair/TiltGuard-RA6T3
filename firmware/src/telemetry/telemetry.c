/*
 * telemetry.c
 */

#include "telemetry.h"
#include "segger_rtt/SEGGER_RTT.h"

void telemetry_init (void)
{
    SEGGER_RTT_Init();
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
