/**
 * @file ble_keyboard.c
 */

#include "ble_keyboard.h"
#include "ble_hid_service.h"
#include "app_timer.h"
#include "report.h"
#include "host.h"
#include "keyboard.h"
#include "nrf_drv_gpiote.h"


#define KEYBOARD_SCAN_INTERVAL APP_TIMER_TICKS(10) // keyboard scan interval
APP_TIMER_DEF(m_keyboard_timer_id); // keyboard scan timer id

static void keyboard_timer_init(void);
static void keyboard_timout_handler(void *p_context);
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
    ble_hid_service_send_report(NRF_REPORT_ID_KEYBOARD, &(report->raw[0]));
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