#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "string.h"
#include "../../const.h"
#include "../../main.h"
#include "../../zigbee.h"

#include "ags10.h"

static const char *TAG = "sensor_ags10";

static void sensor_ags10_task(void *pvParameters)
{

    gpio_num_t param_pin_SCL = I2C_SCL_GPIO;
    gpio_num_t param_pin_SDA = I2C_SDA_GPIO;

    int I2C_ADDRESS_int = 0x1A;

    i2c_dev_t dev;
    esp_err_t init_err = ags10_init_desc(&dev, I2C_NUM_0, I2C_ADDRESS_int, param_pin_SDA, param_pin_SCL);
    if (init_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize AGS10: %s", esp_err_to_name(init_err));
        vTaskDelete(NULL);
    }

    uint8_t version;
    esp_err_t ver_err = ags10_read_version(&dev, &version);
    if (ver_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read AGS10 version: %s", esp_err_to_name(init_err));
        vTaskDelete(NULL);
    }
    ESP_LOGW(TAG, "AGS10 version: v%d", version);

    uint32_t tvoc;
    // uint32_t resistance;

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the device to boot

    /*
        esp_err_t res = ags10_set_zero_point_with_factory_defaults(&dev);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not set zero point with factory defaults: %d (%s)", res, esp_err_to_name(res));
        }
        else
        {
            ESP_LOGI(TAG, "Zero point set with factory defaults");
        }
    */
    while (1)
    {
        esp_err_t res = ags10_read_tvoc(&dev, &tvoc);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read TVOC: %d (%s)", res, esp_err_to_name(res));
        }
        else
        {
            ESP_LOGI(TAG, "TVOC: %lu", tvoc);
            data.voc = tvoc;
            update_attributes(ATTRIBUTE_VOC);
        }

        /*
        res = ags10_read_resistance(&dev, &resistance);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read resistance: %d (%s)", res, esp_err_to_name(res));
        }
        else
        {
            ESP_LOGI(TAG, "Resistance: %lu", resistance);
        }
        */

        vTaskDelay(pdMS_TO_TICKS(AGS10_TASK_INTERVAL));
    }

    ags10_free_desc(&dev);
}

void sensor_ags10()
{
    ESP_LOGW(__func__, "AGS10 sensor task started");
    xTaskCreate(sensor_ags10_task, "sensor_ags10_task", 4096, NULL, 5, NULL);
}
