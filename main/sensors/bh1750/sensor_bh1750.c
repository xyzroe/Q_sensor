#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"

#include "bh1750.h"

#include "../../const.h"
#include "../../zigbee.h"
#include "../../main.h"

static void sensor_bh1750_task(void *pvParameters)
{

    gpio_num_t pin_SCL = I2C_SCL_GPIO;
    gpio_num_t pin_SDA = I2C_SDA_GPIO;

    int I2C_ADDRESS_int = BH1750_ADDR_LO;

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    esp_err_t init_desc_err = bh1750_init_desc(&dev, I2C_ADDRESS_int, 0, pin_SDA, pin_SCL);
    if (init_desc_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize device descriptor: %s", esp_err_to_name(init_desc_err));
        vTaskDelete(NULL);
    }

    esp_err_t setup_err = bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH2);
    if (setup_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to setup BH1750: %s", esp_err_to_name(setup_err));
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the device to boot

    while (1)
    {
        uint16_t lux;
        esp_err_t res = bh1750_read(&dev, &lux);
        if (res != ESP_OK)
        {
            ESP_LOGE(__func__, "Could not read lux data: %d (%s)", res, esp_err_to_name(res));
        }
        else
        {
            ESP_LOGI(__func__, "Lux: %d", lux);

            uint16_t new_bh1750_lux = 10000.0 * log10(lux) + 1;

            if ((new_bh1750_lux != data.lux) && connected)
            {
                data.lux = new_bh1750_lux;
                update_attributes(ATTRIBUTE_LUX);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BH1750_TASK_INTERVAL));
    }
}

void sensor_bh1750()
{
    ESP_LOGW(__func__, "Task: BH1750 created.");
    xTaskCreate(sensor_bh1750_task, "sensor_bh1750_task", 4096, NULL, 5, NULL);
}
