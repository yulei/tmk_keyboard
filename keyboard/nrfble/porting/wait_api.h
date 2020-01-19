/**
 * wait_api.h
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if defined(PROTOCOL_NRF)
#   include "nrf_delay.h"
#   define wait_ms(ms) nrf_delay_ms(ms)
#   define wait_us(us) nrf_delay_us(us)
#endif

#ifdef __cplusplus
}
#endif
