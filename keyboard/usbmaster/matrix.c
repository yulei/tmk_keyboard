/*
Copyright 2012 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"

/*
 * scan matrix
 */
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"
#include "wait.h"
#include "host.h"
#include "report.h"

#include <string.h>

// uart configuration
#define UART_READY_PORT GPIOB
#define UART_READY_PIN 0
#define UART_BUF_SIZE 64

static volatile uint8_t uart_recv_size = 0;
static volatile bool uart_data_ready = false;
static uint8_t uart_recv_buf[UART_BUF_SIZE];
/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend(UARTDriver *uartp) { (void)uartp; }

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  uart_recv_size = c;
  chSysLockFromISR();
  uartStartReceiveI(uartp, uart_recv_size - 1, &uart_recv_buf[0]);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;

  // clear receivable signal
  palClearPad(UART_READY_PORT, UART_READY_PIN);
  uart_data_ready = true;
}

static UARTConfig uart_cfg = {txend,  NULL, rxend, rxchar, rxerr,
                              115200, 0,    0,     0};

// tmk hooking
void matrix_init(void) {
  palSetPadMode(UART_READY_PORT, UART_READY_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  uartStart(&UARTD3, &uart_cfg);
  palSetPad(UART_READY_PORT, UART_READY_PIN);
}

extern host_driver_t chibios_driver;

uint8_t matrix_scan(void) {
  if (uart_data_ready) {
    switch (uart_recv_buf[0]) {
      case 1:  // key
      {
        static report_keyboard_t report;
        memcpy(&report, &uart_recv_buf[1], sizeof(report));
        chibios_driver.send_keyboard(&report);
      } break;
      case 2:  // cc
      {
#ifdef EXTRAKEY_ENABLE
        static int16_t key;
        memcpy(&key, &uart_recv_buf[1], sizeof(key));
        if (IS_SYSTEM(key)) {
          chibios_driver.send_system(key);
        } else {
          chibios_driver.send_consumer(key);
        }
#endif
      } break;
      case 3:  // mouse
      {
#ifdef MOUSE_ENABLE
        static report_mouse_t report;
        memcpy(&report, &uart_recv_buf[1], sizeof(report));
        chibios_driver.send_mouse(&report);
#endif
      } break;
    }
    uart_data_ready = false;
    palSetPad(UART_READY_PORT, UART_READY_PIN);
  }
  return 1;
}

matrix_row_t matrix_get_row(uint8_t row) {
  (void)row;
  return 0;
}

void matrix_print(void) {}