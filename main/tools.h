#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>
#include <stdbool.h>

void get_rtc_time();
bool int_to_bool(int32_t value);

void setup_NVS();
int read_NVS(const char *nvs_key);
bool write_NVS(const char *nvs_key, int value);

const char *get_endpoint_name(int endpoint);
float random_float(float min, float max);
float round_to_decimals(float value, int decimals);

void set_zcl_string(char *buffer, char *value);
void set_bit(uint8_t *byte, uint8_t bit_index, bool value);

void print_chip_info();
void heap_stats();

void debug_task(void *pvParameters);

void scan_i2c_bus(gpio_num_t sda_pin, gpio_num_t scl_pin);

#endif // TOOLS_H