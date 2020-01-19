/**
 * config.h
 *
 */
#pragma once

/* USB Device descriptor parameter */
#define VENDOR_ID       0x60BE
#define PRODUCT_ID      0x00BE
#define DEVICE_VER      0x0001
#define MANUFACTURER    astro
#define PRODUCT         Dumplings
#define DESCRIPTION     60% rgb keyboard with ble extension

/* key matrix size */
//#define MATRIX_ROWS 5
//#define MATRIX_COLS 14
#define UNUSED_PINS
#define DIODE_DIRECTION COL2ROW
#define MATRIX_ROW_PINS { 3,  2, 31, 30, 29}
#define MATRIX_COL_PINS { 4,  5,  6,  7,  8, 9, 10, 11, 12, 13, 14, 15, 16, 17}
/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed */
#define DEBOUNCE_DELAY 5
