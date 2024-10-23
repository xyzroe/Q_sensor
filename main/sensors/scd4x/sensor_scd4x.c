#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"

#include "scd4x.h"

#include "../../const.h"
#include "../../zigbee.h"
#include "../../main.h"

static void sensor_scd4x_task(void *pvParameters)
{

    gpio_num_t pin_SCL = I2C_SCL_GPIO;
    gpio_num_t pin_SDA = I2C_SDA_GPIO;

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    esp_err_t init_desc_err = scd4x_init_desc(&dev, 0, pin_SDA, pin_SCL);
    if (init_desc_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize device descriptor: %s", esp_err_to_name(init_desc_err));
        vTaskDelete(NULL);
    }

    ESP_LOGI(__func__, "Initializing sensor...");

    if (scd4x_wake_up(&dev) != ESP_OK)
    {
        ESP_LOGW(__func__, "Failed to wake up sensor. Maybe it's already awake?");
    }

    if (scd4x_stop_periodic_measurement(&dev) != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to stop periodic measurement");
        vTaskDelete(NULL);
    }

    if (scd4x_reinit(&dev) != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to reinit sensor. Need to power cycle the device");
    }

    uint16_t serial[3];
    if (scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2) != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to get sensor serial number");
    }
    else
    {
        ESP_LOGI(__func__, "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);
    }

    if (scd4x_start_periodic_measurement(&dev) != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to start periodic measurement");
        vTaskDelete(NULL);
    }

    uint16_t co2;
    float temperature, humidity;

    while (1)
    {
        // bool com_ok = false;
        // int retries1 = 0;
        // while (!com_ok && retries1 < 3)
        //{
        // esp_err_t res1 = scd4x_measure_single_shot(&dev);
        // retries1++;
        /*if (res1 != ESP_OK)
        {
            ESP_LOGE(__func__, "Error asking results %d (%s) retries: %d", res1, esp_err_to_name(res1), retries1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }*/
        //}
        // Wait for data to be ready
        uint16_t pressure = data.bmp_press;
        if (pressure != 0)
        {
            if (scd4x_set_ambient_pressure(&dev, pressure) != ESP_OK)
            {
                ESP_LOGE(__func__, "Failed to set ambient pressure");
            }
        }

        bool data_ready = false;
        int retries = 0;
        while (!data_ready && retries < 10)
        {
            vTaskDelay(pdMS_TO_TICKS(2000)); // Poll every 100ms
            esp_err_t res2 = scd4x_get_data_ready_status(&dev, &data_ready);
            retries++;
            if (res2 != ESP_OK)
            {
                ESP_LOGE(__func__, "Error polling results %d (%s) retries: %d", res2, esp_err_to_name(res2), retries);
                continue;
            }
        }

        esp_err_t res = scd4x_read_measurement(&dev, &co2, &temperature, &humidity);
        if (res != ESP_OK)
        {
            ESP_LOGE(__func__, "Error reading results %d (%s)", res, esp_err_to_name(res));
            continue;
        }

        if (co2 == 0)
        {
            ESP_LOGW(__func__, "Invalid sample detected, skipping");
            continue;
        }

        ESP_LOGI(__func__, "CO2: %u ppm, Temperature: %.2f °C, Humidity: %.2f %%", co2, temperature, humidity);

        uint16_t new_scd4x_co2 = (uint16_t)(co2);
        uint16_t new_scd4x_temp = (uint16_t)(temperature * 100);
        uint16_t new_scd4x_hum = (uint16_t)(humidity * 100);

        if ((new_scd4x_co2 != data.scd_co2 || new_scd4x_temp != data.scd_temp || new_scd4x_hum != data.scd_hum) && connected)
        {
            data.scd_co2 = new_scd4x_co2;
            data.scd_temp = new_scd4x_temp;
            data.scd_hum = new_scd4x_hum;
            update_attributes(ATTRIBUTE_SCD);
        }
        vTaskDelay(pdMS_TO_TICKS(SCD4X_TASK_INTERVAL));
    }
}

void sensor_scd4x()
{
    ESP_LOGW(__func__, "SCD4x sensor task started");
    xTaskCreate(sensor_scd4x_task, "sensor_scd4x_task", 4096, NULL, 5, NULL);
}