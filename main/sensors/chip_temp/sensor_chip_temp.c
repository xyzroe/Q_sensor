#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "string.h"
#include "driver/temperature_sensor.h"
#include "../../const.h"
#include "../../zigbee.h"
#include "../../main.h"

void chip_temp_task(void *pvParameters)
{
    ESP_LOGI(__func__, "initializing temperature sensor");
    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 60);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));

    ESP_LOGI(__func__, "starting the loop");
    while (1)
    {
        // Enable temperature sensor
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

        // Get converted sensor data
        float tsens_out;
        uint16_t new_chip_temp;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));

        ESP_LOGI(__func__, "Temperature in %f °C", tsens_out);

        // Disable the temperature sensor if it is not needed and save the power
        ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

        new_chip_temp = (float)(tsens_out * 100);

        if ((new_chip_temp != data.chip_temp) && connected)
        {
            data.chip_temp = new_chip_temp;
            update_attributes(ATTRIBUTE_CHIP_TEMP);
        }

        vTaskDelay(pdMS_TO_TICKS(CPU_TEMP_INTERVAL));
    }
}

void sensor_chip_temp()
{
    ESP_LOGW(__func__, "CPU temperature sensor task started");
    xTaskCreate(chip_temp_task, "chip_temp_task", 4096, NULL, 3, NULL);
}