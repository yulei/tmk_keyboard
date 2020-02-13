/**
 * @file ble_keyboard.c
 */

#include "ble_keyboard.h"
#include "ble_hid_service.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_drv_gpiote.h"
#include "nrf_uart.h"

#include "report.h"
#include "host.h"
#include "keyboard.h"

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

typedef enum {
  CMD_KEY_REPORT,
  CMD_MOUSE_REPORT,
  CMD_SYSTEM_REPORT,
  CMD_CONSUMER_REPORT,
  CMD_RESET_TO_BOOTLOADER,
} command_t;

#define UART_TX_BUF_SIZE 128                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 128                        /**< UART RX buffer size. */

#define KEYBOARD_SCAN_INTERVAL APP_TIMER_TICKS(10) // keyboard scan interval
APP_TIMER_DEF(m_keyboard_timer_id); // keyboard scan timer id

static void keyboard_timer_init(void);
static void keyboard_timout_handler(void *p_context);
static void usb_sense_init(void);
static void usb_sense_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void uart_init(void);
static void uart_error_handle(app_uart_evt_t * p_event);
static void uart_send_cmd(command_t cmd, const uint8_t* report, uint32_t size);
static uint8_t compute_checksum(const uint8_t* data, uint32_t size);

/*static void keyboard_matrix_trigger_init(void);
static void keyboard_matrix_trigger_uninit(void);
static void keyboard_matrix_scan_init(void);
static void keyboard_matrix_scan_uninit(void);
*/

/* Host driver */
static uint8_t keyboard_leds(void);
static void    send_keyboard(report_keyboard_t *report);
static void    send_mouse(report_mouse_t *report);
static void    send_system(uint16_t data);
static void    send_consumer(uint16_t data);

host_driver_t kbd_driver = {
    .keyboard_leds = keyboard_leds,
    .send_keyboard = send_keyboard,
    .send_mouse = send_mouse,
    .send_system = send_system,
    .send_consumer = send_consumer,
};


void ble_keyboard_init(void)
{
    keyboard_setup();
    keyboard_init();
    host_set_driver(&kbd_driver);
    keyboard_timer_init();
    usb_sense_init();
    uart_init();
}

void ble_keyboard_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_keyboard_timer_id, KEYBOARD_SCAN_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void keyboard_timer_init(void)
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_keyboard_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            keyboard_timout_handler);
    APP_ERROR_CHECK(err_code);
}

static void keyboard_timout_handler(void *p_context) {
    keyboard_task();
}

static uint8_t keyboard_leds(void) {
    return ble_driver.keyboard_led;
}

static void send_keyboard(report_keyboard_t *report) {
    //ble_hid_service_send_report(NRF_REPORT_ID_KEYBOARD, &(report->raw[0]));
    if (ble_driver.usb_status) {
        uart_send_cmd(CMD_KEY_REPORT, (uint8_t*)report, sizeof(*report));
        NRF_LOG_INFO("Key report:");
        for (int i = 0; i < 8; i++) {
            NRF_LOG_INFO("0x%x", report->raw[i]);
        }
        NRF_LOG_INFO("\n");
    }
}

#ifdef MOUSEKEY_ENABLE
static void send_mouse(report_mouse_t *report) {
    ble_hid_service_send_report(NRF_REPORT_ID_MOUSE, (uint8_t *)report);
}
#else
static void send_mouse(report_mouse_t *report) { (void)report; }
#endif
#ifdef EXTRAKEY_ENABELE
static void send_system(uint16_t data) {
    ble_hid_service_send_report(NRF_REPORT_ID_SYSTEM, (uint8_t *)&data);
}
static void send_consumer(uint16_t data) {
    ble_hid_service_send_report(NRF_REPORT_ID_CONSUMER, (uint8_t *)&data);
}
#else
static void send_system(uint16_t data) { (void)data; }
static void send_consumer(uint16_t data) { (void)data; }
#endif

static void usb_sense_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    if (pin == USB_SENSE_PIN) {
        int state = nrf_gpio_pin_read(pin);
        if (state > 0) {
            ble_driver.usb_status = 1;
        } else {
            ble_driver.usb_status = 0;
        }
        NRF_LOG_INFO("usb sense: action=%d, status = %d\n", action, ble_driver.usb_status);
    } else {
        NRF_LOG_WARNING("usb sense: unknown pin=%d, action=%d\n", pin, action);
    }
}

static void usb_sense_init(void) {
    ret_code_t err_code;
    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    err_code = nrf_drv_gpiote_in_init(USB_SENSE_PIN, &in_config, usb_sense_handler);
    APP_ERROR_CHECK(err_code);
    int state = nrf_gpio_pin_read(USB_SENSE_PIN);
    if (state > 0) {
        ble_driver.usb_status = 1;
    } else {
        ble_driver.usb_status = 0;
    }
    NRF_LOG_INFO("usb init state: status = %d\n", ble_driver.usb_status);
    nrf_drv_gpiote_in_event_enable(USB_SENSE_PIN, true);
}

static void uart_error_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type) {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
            app_uart_flush();
            break;
        case APP_UART_DATA_READY:
            break;
        case APP_UART_TX_EMPTY:
            break;
        default:
            break;
    }
}

static void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params = {
        .rx_pin_no = UART_RX_PIN,
        .tx_pin_no = UART_TX_PIN,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity = false,
        .baud_rate = NRF_UART_BAUDRATE_115200,
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

static void uart_send_cmd(command_t cmd, const uint8_t* report, uint32_t size)
{
    uint8_t checksum = cmd;
    checksum += compute_checksum(report, size);
    app_uart_put(SYNC_BYTE_1);
    app_uart_put(SYNC_BYTE_2);
    app_uart_put(size+3);
    app_uart_put(checksum);
    app_uart_put(cmd);
    for (uint32_t i = 0; i < size; i++) {
        app_uart_put(report[i]);
    }
}

static uint8_t compute_checksum(const uint8_t* data, uint32_t size)
{
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    return checksum;
}

/*
extern uint32_t row_pins[];
extern uint32_t col_pins[];

static void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
}

static void keyboard_matrix_trigger_init(void){
    ret_code_t err_code;
    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
    for (int i = 0; i < MATRIX_COLS; i++)  {
      err_code = nrf_drv_gpiote_out_init(col_pins[i], &out_config);
      APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    for (int i = 0; i < MATRIX_ROWS; i++) {
      err_code = nrf_drv_gpiote_in_init(row_pins[i], &in_config, in_pin_handler);
      APP_ERROR_CHECK(err_code);
      nrf_drv_gpiote_in_event_enable(row_pins[i], true);
    }
}

static void keyboard_matrix_trigger_uninit(void){
    for (int i = 0; i < MATRIX_COLS; i++)  {
      nrf_drv_gpiote_out_uninit(col_pins[i]);
    }
    for (int i = 0; i < MATRIX_ROWS; i++) {
      nrf_drv_gpiote_in_event_disable(row_pins[i]);
      nrf_drv_gpiote_in_uninit(row_pins[i]);
    }
}

static void keyboard_matrix_scan_init(void) {
    for (int i = 0; i < MATRIX_COLS; i++) {
        nrf_gpio_cfg_output(col_pins[i]);
        nrf_gpio_pin_clear(col_pins[i]);
    }

    for (int i = 0; i < MATRIX_ROWS; i++) {
        nrf_gpio_cfg_input(row_pins[i], NRF_GPIO_PIN_PULLDOWN);
    }
}

static void keyboard_matrix_scan_uninit(void) {
}*/