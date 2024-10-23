#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"

#include "sensor_aht.h"
#include "aht.h"

#include "../../const.h"
#include "../../zigbee.h"
#include "../../main.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define AHT_TYPE_AHT1x AHT_TYPE

static void sensor_aht_task(void *pvParameters)
{

    gpio_num_t pin_SCL = I2C_SCL_GPIO;
    gpio_num_t pin_SDA = I2C_SDA_GPIO;

    int I2C_ADDRESS_int = AHT_I2C_ADDRESS_GND;

    aht_t dev = {0};
    dev.mode = AHT_MODE_NORMAL;
    dev.type = AHT_TYPE_AHT20;

    esp_err_t init_desc_err = aht_init_desc(&dev, I2C_ADDRESS_int, 0, pin_SDA, pin_SCL);
    if (init_desc_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize device descriptor: %s", esp_err_to_name(init_desc_err));
        vTaskDelete(NULL);
    }

    esp_err_t init_err = aht_init(&dev);
    if (init_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize AHT: %s", esp_err_to_name(init_err));
        vTaskDelete(NULL);
    }

    bool calibrated;
    ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
    if (calibrated)
        ESP_LOGI(__func__, "Sensor calibrated");
    else
        ESP_LOGW(__func__, "Sensor not calibrated!");

    float temperature, humidity;

    while (1)
    {

        if (aht_get_data(&dev, &temperature, &humidity) == ESP_OK)
        {
            ESP_LOGI(__func__, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);

            uint16_t new_aht_temp = (float)(temperature * 100);
            uint16_t new_aht_hum = (float)(humidity * 100);

            if ((new_aht_temp != data.aht_temp || new_aht_hum != data.aht_hum) && connected)
            {
                data.aht_temp = new_aht_temp;
                data.aht_hum = new_aht_hum;
                update_attributes(ATTRIBUTE_AHT);
            }
        }
        else
        {
            ESP_LOGE(__func__, "Could not read data from sensor AHT");
        }

        vTaskDelay(pdMS_TO_TICKS(AHT_TASK_INTERVAL));
    }
}

void sensor_aht()
{
    ESP_LOGW(__func__, "AHT sensor task started");
    xTaskCreate(sensor_aht_task, "sensor_aht_task", 4096, NULL, 5, NULL);
}
