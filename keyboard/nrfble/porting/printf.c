/**
 * @file printf.c
 */

#include <stdarg.h>
#include "printf.h"
#include "nrf_log.h"

static void nrf_printf(char *fmt, va_list* args)
{
    NRF_LOG_INFO(fmt, args);
}

void xprintf(char* fmt, ...)
{
    va_list args = {0};
    va_start(args, fmt);

    nrf_printf(fmt, &args);

    va_end(args);
}


