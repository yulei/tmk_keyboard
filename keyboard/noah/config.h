/**
 * config.h
 *
 */

#pragma once

/* USB Device descriptor parameter */
#define VENDOR_ID       0xBEEF
#define PRODUCT_ID      0xAB65      //abel 65
#define DEVICE_VER      0x0001
//#define MANUFACTURER    matrix
//#define PRODUCT         NOAH
//#define DESCRIPTION     65% keybaord based on the abelx

/* in python2: list(u"whatever".encode('utf-16-le')) */
/*   at most 32 characters or the ugly hack in usb_main.c borks */
#define MANUFACTURER "MATRIX"
#define USBSTR_MANUFACTURER    'M', '\x00', 'A', '\x00', 'T', '\x00', 'R', '\x00', 'I', '\x00', 'X', '\x00'
#define PRODUCT "NOAH"
#define USBSTR_PRODUCT         'N', '\x00', 'O', '\x00', 'A', '\x00', 'H', '\x00'
#define DESCRIPTION "65% keyboard by Matrix"

/* key matrix size */
#define MATRIX_ROWS 5
#define MATRIX_COLS 15

#define DIODE_DIRECTION COL2ROW
#define DEBOUNCING_DELAY 5

//rgb matrix setting
#define DRIVER_ADDR_1 0b1110100
#define DRIVER_ADDR_2 0b1110110
#define DRIVER_COUNT 2
#define DRIVER_1_LED_TOTAL 36
#define DRIVER_2_LED_TOTAL 36
#define DRIVER_LED_TOTAL DRIVER_1_LED_TOTAL + DRIVER_2_LED_TOTAL

/* indicator rgb */
#define WS2812_LED_N    4
#define RGBLED_NUM      WS2812_LED_N
#define WS2812_TIM_N    3
#define WS2812_TIM_CH   3
#define PORT_WS2812     GPIOB
#define PIN_WS2812      1
#define WS2812_DMA_STREAM STM32_DMA1_STREAM2  // DMA stream for TIMx_UP (look up in reference manual under DMA Channel selection)
#define WS2812_DMA_CHANNEL 5                  // DMA channel for TIMx_UP
#define RGBLIGHT_ANIMATIONS
//#define WS2812_EXTERNAL_PULLUP

// tapping setting
#define TAPPING_TERM    200
#define RETRO_TAPPING
#define PERMISSIVE_HOLD

/* key combination for command */
#define IS_COMMAND() ( \
    keyboard_report->mods == (MOD_BIT(KC_LSHIFT) | MOD_BIT(KC_RSHIFT)) \
)
