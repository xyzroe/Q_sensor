#ifndef ZIGUSB_H
#define ZIGUSB_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_zigbee_core.h"

typedef struct
{
    // bool USB_state;
    // bool ext_led_mode;
    int old_identify_time;
    bool led_mode;
    bool radar_state;
    // int start_up_on_off;
    uint16_t chip_temp;
    uint16_t aht_temp;
    uint16_t aht_hum;
    uint16_t bmp_temp;
    uint16_t bmp_press;
    uint16_t scd_co2;
    uint16_t scd_temp;
    uint16_t scd_hum;
    uint16_t lux;
    uint16_t voc;
    uint16_t imu_pos;
    int32_t imu_pitch;
    int32_t imu_roll;
    int32_t imu_yaw;
    uint16_t adc1;
    uint16_t adc2;
    uint8_t zone_status;
} DataStructure;

extern float led_hz;
extern char strftime_buf[64];
extern DataStructure data;

extern uint16_t manuf_id;
extern char manufacturer[16];
extern char model[16];
extern char firmware_version[16];
extern char firmware_date[16];
extern bool time_updated;
extern bool connected;

extern TaskHandle_t ledTaskHandle;

typedef enum
{
    ATTRIBUTE_ALL,
    ATTRIBUTE_CHIP_TEMP,
    ATTRIBUTE_AHT,
    ATTRIBUTE_BMP,
    ATTRIBUTE_SCD,
    ATTRIBUTE_LUX,
    ATTRIBUTE_VOC,
    ATTRIBUTE_IMU,
    ATTRIBUTE_ADC,
} attribute_t;

void app_main(void);

#endif // ZIGUSB_H
