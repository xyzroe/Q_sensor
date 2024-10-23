#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"

#include "sensor_bmp280.h"
#include "bmp280.h"

#include "../../const.h"
#include "../../zigbee.h"
#include "../../main.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static void sensor_bmp280_task(void *pvParameters)
{

    gpio_num_t pin_SCL = I2C_SCL_GPIO;
    gpio_num_t pin_SDA = I2C_SDA_GPIO;

    int I2C_ADDRESS_int = 0x77;

    bmp280_params_t params_bmp280;
    bmp280_init_default_params(&params_bmp280);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    esp_err_t init_desc_err = bmp280_init_desc(&dev, I2C_ADDRESS_int, 0, pin_SDA, pin_SCL);
    if (init_desc_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize device descriptor: %s", esp_err_to_name(init_desc_err));
        vTaskDelete(NULL);
    }

    esp_err_t init_err = bmp280_init(&dev, &params_bmp280);
    if (init_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize BMP280: %s", esp_err_to_name(init_err));
        vTaskDelete(NULL);
    }

    bool bme280p = dev.id == BME280_CHIP_ID;

    float pressure, temperature, humidity;

    while (1)
    {

        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) == ESP_OK)
        {

            ESP_LOGI(__func__, "Temperature: %.1f °C, Pressure: %.1f Pa (%.1fmm)", temperature, pressure, pressure / 1333.224);

            uint16_t new_bmp_temp = (float)(temperature * 100);
            uint16_t new_bmp_press = (float)(pressure / 100);

            if ((new_bmp_temp != data.bmp_temp || new_bmp_press != data.bmp_press) && connected)
            {
                data.bmp_temp = new_bmp_temp;
                data.bmp_press = new_bmp_press;
                update_attributes(ATTRIBUTE_BMP);
            }
        }
        else
        {
            ESP_LOGE(__func__, "Could not read data from sensor bmp280");
        }
        vTaskDelay(pdMS_TO_TICKS(BMP280_TASK_INTERVAL));
    }
}

void sensor_bmp280()
{
    ESP_LOGW(__func__, "BMP280 sensor task started");
    xTaskCreate(sensor_bmp280_task, "sensor_bmp280_task", 4096, NULL, 5, NULL);
}
