/**
 * @file ws2812_nrf.h
 */

#pragma once

#include <stdint.h>

void ws2812_nrf_init(void);
void ws2812_nrf_setleds(uint8_t* leds, uint16_t number);