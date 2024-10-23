#ifndef PERF_H
#define PERF_H

void init_outputs();
// void init_pullup_i2c_pins();

// void usb_driver_set_power(bool state);

void led_task(void *pvParameters);
void led_blink(int delay);

void register_alarm_input();
static void radar_active_cb(void *arg, void *usr_data);
static void radar_deactive_cb(void *arg, void *usr_data);

void register_button();
static void button_single_click_cb(void *arg, void *usr_data);
static void button_long_press_cb(void *arg, void *usr_data);

void chip_temp_task(void *pvParameters);

void test_task(void *pvParameters);

void run_sensors();

void identify_task(void *pvParameters);

#endif // PERF_H