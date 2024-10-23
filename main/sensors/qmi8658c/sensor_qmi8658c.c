#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"

#include "qmi8658c.h"
#include <math.h>

#include "../../const.h"
#include "../../zigbee.h"
#include "../../main.h"

void calculate_angles(qmi8658c_data_t *data, float *pitch, float *roll, float *yaw)
{
    *pitch = atan2(-data->acc.x, sqrt(data->acc.y * data->acc.y + data->acc.z * data->acc.z)) * 180 / M_PI;
    *roll = atan2(data->acc.y, data->acc.z) * 180 / M_PI;
    *yaw = atan2(data->gyro.z, sqrt(data->gyro.x * data->gyro.x + data->gyro.y * data->gyro.y)) * 180 / M_PI;
}

bool is_stable(qmi8658c_data_t *data, float threshold)
{
    return fabs(data->gyro.x) < threshold && fabs(data->gyro.y) < threshold && fabs(data->gyro.z) < threshold;
}

typedef enum
{
    POSITION_INCORRECT,
    POSITION_HORIZONTAL,
    POSITION_VERTICAL
} device_position_t;

#define PITCH_TOLERANCE 10.0
#define ROLL_TOLERANCE 25.0

device_position_t determine_position(float pitch, float roll)
{
    float table_pitch = 0;
    float table_roll = 0;

    float wall_pitch = 90;
    float wall_roll = 180;

    if (fabs(pitch - table_pitch) < PITCH_TOLERANCE && fabs(roll - table_roll) < ROLL_TOLERANCE)
    {
        return POSITION_HORIZONTAL;
    }
    else if (fabs(pitch - wall_pitch) < PITCH_TOLERANCE && fabs(roll - wall_roll) < ROLL_TOLERANCE)
    {
        return POSITION_VERTICAL;
    }
    else
    {
        return POSITION_INCORRECT;
    }
}

static void sensor_qmi8658c_task(void *pvParameters)
{

    gpio_num_t pin_SCL = I2C_SCL_GPIO;
    gpio_num_t pin_SDA = I2C_SDA_GPIO;

    int I2C_ADDRESS_int = 0x6B;

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    esp_err_t init_desc_err = qmi8658c_init_desc(&dev, I2C_ADDRESS_int, 0, pin_SDA, pin_SCL);
    if (init_desc_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize device descriptor: %s", esp_err_to_name(init_desc_err));
        vTaskDelete(NULL);
    }

    qmi8658c_config_t config = {
        .mode = QMI8658C_MODE_DUAL,
        .acc_scale = QMI8658C_ACC_SCALE_4G,
        .acc_odr = QMI8658C_ACC_ODR_1000,
        .gyro_scale = QMI8658C_GYRO_SCALE_64DPS,
        .gyro_odr = QMI8658C_GYRO_ODR_8000,
    }; // Dual, 4G, 1000, 64DPS, 8000

    esp_err_t setup_err = qmi8658c_setup(&dev, &config);
    if (setup_err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to setup QMI8658C: %s", esp_err_to_name(setup_err));
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the device to boot
    int runs = 0;
    while (1)
    {
        qmi8658c_data_t imu_data;
        esp_err_t res = qmi8658c_read_data(&dev, &imu_data);
        if (res != ESP_OK)
        {
            ESP_LOGE(__func__, "Could not read sensor data: %d (%s)", res, esp_err_to_name(res));
        }
        else
        {
            bool stable = is_stable(&imu_data, 15); // Threshold for stability
            send_zone_1_state(2, !stable);          // send as tamper state
            ESP_LOGW(__func__, "Device is %s", stable ? "stable" : "not stable");

            // ESP_LOGI(__func__, "Acc: x=%f, y=%f, z=%f; Gyro: x=%f, y=%f, z=%f; Temp: %f",
            //          imu_data.acc.x, imu_data.acc.y, imu_data.acc.z, imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z, imu_data.temperature);

            if (runs == 2)
            {
                runs = 0;
            }
            if (runs == 0)
            {
                float pitch, roll, yaw;
                calculate_angles(&imu_data, &pitch, &roll, &yaw);

                ESP_LOGI(__func__, "Pitch: %f, Roll: %f, Yaw: %f", pitch, roll, yaw);

                device_position_t position = determine_position(pitch, roll);

                switch (position)
                {
                case POSITION_HORIZONTAL:
                    ESP_LOGW(__func__, "horizontal");
                    break;
                case POSITION_VERTICAL:
                    ESP_LOGW(__func__, "vertical");
                    break;
                default:
                    ESP_LOGW(__func__, "position is incorrect");
                    break;
                }

                uint16_t new_imu_pos = (uint16_t)position;
                int32_t new_imu_pitch = (int32_t)(pitch * 1000000);
                int32_t new_imu_roll = (int32_t)(roll * 1000000);
                int32_t new_imu_yaw = (int32_t)(yaw * 1000000);

                if ((new_imu_pos != data.imu_pos || new_imu_pitch != data.imu_pitch || new_imu_roll != data.imu_roll || new_imu_yaw != data.imu_yaw) && connected)
                {
                    data.imu_pos = new_imu_pos;
                    data.imu_pitch = new_imu_pitch;
                    data.imu_roll = new_imu_roll;
                    data.imu_yaw = new_imu_yaw;
                    update_attributes(ATTRIBUTE_IMU);
                }
            }
            runs++;
        }

        vTaskDelay(pdMS_TO_TICKS(QMI8658C_TASK_INTERVAL));
    }
}

void sensor_qmi8658c()
{
    ESP_LOGW(__func__, "QMI8658C sensor task started");
    xTaskCreate(sensor_qmi8658c_task, "sensor_qmi8658c_task", 4096, NULL, 5, NULL);
}