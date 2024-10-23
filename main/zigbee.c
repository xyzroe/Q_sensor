
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include <time.h>
#include <sys/time.h>

#include "ha/esp_zigbee_ha_standard.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "zboss_api.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/zb_zcl_common.h"

#include "iot_button.h"

#include "const.h"
#include "main.h"
#include "perf.h"
#include "tools.h"
#include "zigbee.h"

/*------ Global definitions -----------*/

static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

/* Manual reporting atribute to coordinator */
static void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

static void update_attribute_value(uint8_t endpoint, uint16_t cluster_id, uint8_t role, uint16_t attr_id, void *value, const char *attr_name)
{
    // uint16_t val = *((uint16_t *)value);
    // ESP_LOGW(__func__, "Value: %d, val = %d", *((uint16_t *)value), val);
    ESP_LOGW(__func__, "Setting on endpoint %d, cluster 0x%04x, attribute 0x%04x, value %d", endpoint, cluster_id, attr_id, *((uint16_t *)value)); // val);

    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(endpoint, cluster_id, role, attr_id, value, false); //&val, false);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(__func__, "Setting %s attribute failed, with status %d. Value: %d", attr_name, status, *((uint16_t *)value)); // val);
    }
}

void update_attributes(attribute_t attribute)
{
    ESP_LOGI(__func__, "updating %d", attribute);

    if (attribute == ATTRIBUTE_CHIP_TEMP || attribute == ATTRIBUTE_ALL)
    {
        update_attribute_value(DEVICE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &data.chip_temp, "temperature");
    }

    if (attribute == ATTRIBUTE_AHT || attribute == ATTRIBUTE_ALL)
    {
        update_attribute_value(AHT20_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &data.aht_temp, "aht temperature");
        update_attribute_value(AHT20_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &data.aht_hum, "aht humidity");
    }

    if (attribute == ATTRIBUTE_BMP || attribute == ATTRIBUTE_ALL)
    {
        update_attribute_value(BMP280_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &data.bmp_temp, "bmp temperature");
        update_attribute_value(BMP280_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &data.bmp_press, "bmp pressure");
    }

    if (attribute == ATTRIBUTE_SCD || attribute == ATTRIBUTE_ALL)
    {
        float scd_co2_value = (float)data.scd_co2 / 1000000.0f;
        update_attribute_value(SCD40_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &scd_co2_value, "scd co2");
        update_attribute_value(SCD40_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &data.scd_temp, "scd temperature");
        update_attribute_value(SCD40_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &data.scd_hum, "scd humidity");
    }

    if (attribute == ATTRIBUTE_LUX || attribute == ATTRIBUTE_ALL)
    {
        update_attribute_value(BH1750_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &data.lux, "lux");
    }

    if (attribute == ATTRIBUTE_VOC || attribute == ATTRIBUTE_ALL)
    {
        // update_attribute_value(AGS10_ENDPOINT, TVOC_CLUSTER, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, TVOC_ATTR, &data.voc, "VOC");

        float voc_value = data.voc * 1.0f;
        update_attribute_value(AGS10_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &voc_value, "VOC");
    }

    if (attribute == ATTRIBUTE_IMU || attribute == ATTRIBUTE_ALL)
    {
        /*
        char imu_txt[99];
        char zb_imu_str[100];

        snprintf(imu_txt, sizeof(imu_txt), "P: %.3f R: %.3f Y: %.3f",
                 data.imu_pitch / 1000.0, data.imu_roll / 1000.0, data.imu_yaw / 1000.0);

        set_zcl_string(zb_imu_str, imu_txt);

         update_attribute_value(IMU_BASE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_MULTISTATE_DESCRIPTION_ID, &zb_imu_str, "imu txt");
        */

        update_attribute_value(IMU_BASE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_MULTISTATE_VALUE_PRESENT_VALUE_ID, &data.imu_pos, "imu pos");

        float imu_pos_value = data.imu_pos * 1.0f;
        update_attribute_value(IMU_BASE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &imu_pos_value, "imu position");

        float pitch = (data.imu_pitch / 1000000.0f);
        ESP_LOGW(__func__, "Pitch: %f", pitch);
        update_attribute_value(IMU_BASE_ENDPOINT + 1, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &pitch, "pitch");

        float roll = (data.imu_roll / 1000000.0f);
        ESP_LOGW(__func__, "Roll: %f", roll);
        update_attribute_value(IMU_BASE_ENDPOINT + 2, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &roll, "roll");

        float yaw = (data.imu_yaw / 1000000.0f);
        ESP_LOGW(__func__, "Yaw: %f", yaw);
        update_attribute_value(IMU_BASE_ENDPOINT + 3, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &yaw, "yaw");
    }

    if (attribute == ATTRIBUTE_ADC || attribute == ATTRIBUTE_ALL)
    {
        float adc1_value = data.adc1 * 1.0f;
        float adc2_value = data.adc2 * 1.0f;
        update_attribute_value(ADC1_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &adc1_value, "adc 1");
        update_attribute_value(ADC2_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &adc2_value, "adc 2");
    }
}

/* Task for update all values */
void force_update_task()
{
    while (1)
    {
        vTaskDelay(WAIT_BEFORE_FIRST_UPDATE / portTICK_PERIOD_MS);
        force_update();
        vTaskDelay((UPDATE_ATTRIBUTE_INTERVAL - WAIT_BEFORE_FIRST_UPDATE) / portTICK_PERIOD_MS);
    }
}

/* Function to update all values */
void force_update()
{

    if (connected)
    {
        led_blink(500);

        // send_bin_cfg_option(DEVICE_ENDPOINT, data.USB_state);
        // send_bin_cfg_option(INT_LED_ENDPOINT, data.led_mode);
        // send_bin_cfg_option(EXT_LED_ENDPOINT, data.ext_led_mode);
        // send_bin_cfg_option(INV_USB_ENDPOINT, 0);

        send_zone_1_state(0, 2);

        update_attributes(ATTRIBUTE_ALL);
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, __func__, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, __func__, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(__func__, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == DEVICE_ENDPOINT)
    {
        switch (message->info.cluster)
        {
        case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:

            int time = *(int *)message->attribute.data.value;
            if (time > data.old_identify_time)
            {
                data.old_identify_time = time;

                ESP_LOGI(__func__, "Identify pressed, time: %ds", time);

                if (time < 3) // Minimum time is 3 seconds
                {
                    ESP_LOGW(__func__, "Identify time is too short, setting to 3 seconds");
                    time = 3;
                }

                xTaskCreate(identify_task, "identify_task", 4096, (void *)time, 3, NULL);
            }

            break;
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_START_UP_ON_OFF)
            {
                int value = *(int *)message->attribute.data.value;
                if (value == 0 || value == 1 || value == 2 || value == 255)
                {
                    ESP_LOGW(__func__, "Power-on behavior %d", value);
                    // data.start_up_on_off = value;
                    // write_NVS("start_up_on_off", data.start_up_on_off);
                }
                else
                {
                    ESP_LOGE(__func__, "Invalid power-on behavior value: %d", value);
                }
            }
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME)
            {
                ESP_LOGI(__func__, "On time %d", *(int *)message->attribute.data.value);
            }
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_OFF_WAIT_TIME)
            {
                ESP_LOGI(__func__, "Off wait time %d", *(int *)message->attribute.data.value);
            }
            break;
        default:
            ESP_LOGI(__func__, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    // bool usb_state = 0;
    /*if (message->info.dst_endpoint == HA_ONOFF_SWITCH_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                usb_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : usb_state;
                ESP_LOGI(__func__, "USB sets to %s", usb_state ? "On" : "Off");
                usb_driver_set_power(usb_state);
            }
        }
    }*/
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
    {
        // ESP_LOGI(__func__, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
        {
            /*
            if (message->info.dst_endpoint == DEVICE_ENDPOINT)
            {
                bool usb_state =
                    message->attribute.data.value ? *(bool *)message->attribute.data.value : 0;
                ESP_LOGI(__func__, "USB sets to %s", usb_state ? "On" : "Off");
                // usb_driver_set_power(usb_state);
            }
            else
            {
                bool switch_state =
                    message->attribute.data.value ? *(bool *)message->attribute.data.value : 0;
                ESP_LOGI(__func__, "Endpoint %d, sets to %s", message->info.dst_endpoint, switch_state ? "On" : "Off");

                if (message->info.dst_endpoint == INT_LED_ENDPOINT)
                {
                    if (switch_state == 0)
                    {
                        gpio_set_level(INT_LED_GPIO, LED_OFF_STATE);
                    }
                    data.led_mode = switch_state;
                    write_NVS("led_mode", switch_state);
                }
                else if (message->info.dst_endpoint == EXT_LED_ENDPOINT)
                {
                    if (switch_state == 0)
                    {
                        ESP_LOGI(__func__, "cmd is OFF, so EXT_LED_GPIO (1)");
                        gpio_set_level(EXT_LED_GPIO, 1);
                    }
                    else if (data.USB_state == 1)
                    {
                        ESP_LOGI(__func__, "USB is ON, so EXT_LED_GPIO (0)");
                        gpio_set_level(EXT_LED_GPIO, 0);
                    }
                    data.ext_led_mode = switch_state;
                    write_NVS("ext_led_mode", switch_state);
                }
                else if (message->info.dst_endpoint == INV_USB_ENDPOINT)
                {
                    // inverted logic to make possible onWithOff work
                    bool new_state = !(switch_state);
                    usb_driver_set_power(new_state);
                    send_bin_cfg_option(DEVICE_ENDPOINT, new_state);
                }
            }
            */
        }
    }
    return ret;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, __func__, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, __func__, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable)
    {
        ESP_LOGI(__func__, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);

        if (message->info.dst_endpoint == DEVICE_ENDPOINT)
        {
            switch (message->info.cluster)
            {
            case ESP_ZB_ZCL_CLUSTER_ID_TIME:
                ESP_LOGW(__func__, "Server time recieved %lu", *(uint32_t *)variable->attribute.data.value);
                struct timeval tv;
                tv.tv_sec = *(uint32_t *)variable->attribute.data.value + 946684800;
                settimeofday(&tv, NULL);
                time_updated = true;

                uint32_t boot_time = *(uint32_t *)variable->attribute.data.value;
                ESP_LOGI(__func__, "Write new boot time %lu", boot_time);
                esp_zb_zcl_status_t state_boot_time = esp_zb_zcl_set_attribute_val(DEVICE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TIME, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TIME_LAST_SET_TIME_ID, &boot_time, false);
                if (state_boot_time != ESP_ZB_ZCL_STATUS_SUCCESS)
                {
                    ESP_LOGE(__func__, "Setting boot time attribute failed!");
                }

                break;
            default:
                ESP_LOGI(__func__, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, variable->attribute.id);
            }
        }

        variable = variable->next;
    }

    return ESP_OK;
}

size_t ota_data_len_;
size_t ota_header_len_;
bool ota_upgrade_subelement_;
uint8_t ota_header_[6];

static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;

    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        switch (message.upgrade_status)
        {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(__func__, "-- OTA upgrade start");
            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            assert(s_ota_partition);
            ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, __func__, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            size_t payload_size = message.payload_size;
            const uint8_t *payload = message.payload;

            total_size = message.ota_header.image_size;
            offset += payload_size;

            ESP_LOGI(__func__, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);

            /* Read and process the first sub-element, ignoring everything else */
            while (ota_header_len_ < 6 && payload_size > 0)
            {
                ota_header_[ota_header_len_] = payload[0];
                ota_header_len_++;
                payload++;
                payload_size--;
            }

            if (!ota_upgrade_subelement_ && ota_header_len_ == 6)
            {
                if (ota_header_[0] == 0 && ota_header_[1] == 0)
                {
                    ota_upgrade_subelement_ = true;
                    ota_data_len_ =
                        (((int)ota_header_[5] & 0xFF) << 24) | (((int)ota_header_[4] & 0xFF) << 16) | (((int)ota_header_[3] & 0xFF) << 8) | ((int)ota_header_[2] & 0xFF);
                    ESP_LOGD(__func__, "OTA sub-element size %zu", ota_data_len_);
                }
                else
                {
                    ESP_LOGE(__func__, "OTA sub-element type %02x%02x not supported", ota_header_[0], ota_header_[1]);
                    return ESP_FAIL;
                }
            }

            if (ota_data_len_)
            {
                payload_size = fmin(ota_data_len_, payload_size);
                ota_data_len_ -= payload_size;

                if (message.payload_size && message.payload)
                {
                    ret = esp_ota_write(s_ota_handle, payload, payload_size);
                    ESP_RETURN_ON_ERROR(ret, __func__, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
                }
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(__func__, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            ESP_LOGI(__func__, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(__func__, "-- OTA Finish");
            ESP_LOGI(__func__, "-- OTA Information: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     message.ota_header.file_version, message.ota_header.manufacturer_code, message.ota_header.image_type,
                     message.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
            ret = esp_ota_end(s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, __func__, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
            ret = esp_ota_set_boot_partition(s_ota_partition);
            ESP_RETURN_ON_ERROR(ret, __func__, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
            ESP_LOGW(__func__, "Prepare to restart system");
            esp_restart();
            break;
        default:
            ESP_LOGI(__func__, "OTA status: %d", message.upgrade_status);
            break;
        }
    }
    return ret;
}

static esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
    esp_err_t ret = ESP_OK;
    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGI(__func__, "Queried OTA image from address: 0x%04hx, endpoint: %d", message.server_addr.u.short_addr, message.server_endpoint);
        ESP_LOGI(__func__, "Image version: 0x%lx, manufacturer code: 0x%x, image size: %ld", message.file_version, message.manufacturer_code,
                 message.image_size);
    }
    if (ret == ESP_OK)
    {
        ESP_LOGI(__func__, "Approving OTA image upgrade");
    }
    else
    {
        ESP_LOGI(__func__, "Rejecting OTA image upgrade, status: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    // case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
    // ret = esp_zb_zcl_identify_cmd_req((esp_zb_zcl_identify_cmd_t *)message);
    // ESP_LOGW(__func__, "ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID");
    // break;
    case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID:
        ESP_LOGW(__func__, "ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID");
        break;
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        ESP_LOGW(__func__, "CORE_SET_ATTR_VALUE_CB action(0x%x) callback", callback_id);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        ESP_LOGW(__func__, "CORE_CMD_READ_ATTR_RESP_CB action(0x%x) callback", callback_id);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_SRV_QUERY_IMAGE_CB_ID:
        ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
        break;
    default:
        ESP_LOGD(__func__, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void read_server_time()
{
    esp_zb_zcl_read_attr_cmd_t read_req;
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    uint16_t attributes[] = {ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID};

    read_req.attr_number = sizeof(attributes) / sizeof(uint16_t);
    read_req.attr_field = attributes;

    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME;

    read_req.zcl_basic_cmd.dst_endpoint = 1;               // Coordinator
    read_req.zcl_basic_cmd.src_endpoint = DEVICE_ENDPOINT; // Device main endpoint
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        esp_zb_set_node_descriptor_manufacturer_code(manuf_id);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status != ESP_OK)
        {
            connected = false;
            led_hz = 2;
            ESP_LOGW(__func__, "Stack %s failure with %s status, steering", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        else
        {
            /* device auto start successfully and on a formed network */
            connected = true;
            led_hz = 0;
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(__func__, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            read_server_time();
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET)
        {
            ESP_LOGI(__func__, "Reset device");
            esp_zb_factory_reset();
        }
        break;
    default:
        ESP_LOGI(__func__, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

void print_custom_attributes(const esp_zb_attribute_list_t *attr_list)
{
    const esp_zb_attribute_list_t *current = attr_list;
    int index = 0;

    while (current != NULL)
    {
        ESP_LOGI(__func__, "Attribute %d: Cluster ID=0x%04x, Attribute ID=0x%04x, Type=0x%02x, Access=0x%02x, Manuf Code=0x%04x",
                 index, current->cluster_id, current->attribute.id, current->attribute.type, current->attribute.access, current->attribute.manuf_code);

        // Вывод значения атрибута в зависимости от его типа
        switch (current->attribute.type)
        {
        case ZB_ZCL_ATTR_TYPE_U8:
            ESP_LOGI(__func__, "Value: %u", *(uint8_t *)current->attribute.data_p);
            break;
        case ZB_ZCL_ATTR_TYPE_U16:
            ESP_LOGI(__func__, "Value: %u", *(uint16_t *)current->attribute.data_p);
            break;
        case ZB_ZCL_ATTR_TYPE_U32:
            ESP_LOGI(__func__, "Value: %lu", *(uint32_t *)current->attribute.data_p);
            break;
        case ZB_ZCL_ATTR_TYPE_S8:
            ESP_LOGI(__func__, "Value: %d", *(int8_t *)current->attribute.data_p);
            break;
        case ZB_ZCL_ATTR_TYPE_S16:
            ESP_LOGI(__func__, "Value: %d", *(int16_t *)current->attribute.data_p);
            break;
        case ZB_ZCL_ATTR_TYPE_S32:
            ESP_LOGI(__func__, "Value: %li", *(int32_t *)current->attribute.data_p);
            break;
        case ZB_ZCL_ATTR_TYPE_CHAR_STRING:
            ESP_LOGI(__func__, "Value: %s", (char *)current->attribute.data_p);
            break;
        default:
            ESP_LOGI(__func__, "Value: (unknown type)");
            break;
        }

        current = current->next;
        index++;
    }
}

static void zigbee_config_endpoints_attributes()
{
    uint16_t undefined_value;
    undefined_value = 0x8000;

    uint16_t hundred_value = 10000;
    uint16_t one_value = 1;
    uint16_t zero_value = 0;

    float undefined_float = 0;
    float float_zero = 0;
    float float_one = 1;

    uint16_t cluster_id;
    uint16_t attr_id;
    uint8_t attr_type = ESP_ZB_ZCL_ATTR_TYPE_U16;
    uint8_t attr_access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    // DEVICE_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list1 = esp_zb_zcl_cluster_list_create();

    // Basic cluster start
    set_zcl_string(manufacturer, HW_MANUFACTURER);
    set_zcl_string(model, HW_MODEL);
    char ota_upgrade_file_version[10];
    sprintf(ota_upgrade_file_version, "%d", OTA_FW_VERSION);
    set_zcl_string(firmware_version, ota_upgrade_file_version);
    set_zcl_string(firmware_date, FW_BUILD_DATE);
    uint8_t power_source = 4; /**< DC source. */

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, firmware_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, firmware_date);

    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list1, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Basic cluster end

    // Identify cluster start
    uint16_t identify_id = 0;
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_CMD_IDENTIFY_IDENTIFY_ID, &identify_id);

    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list1, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Identify cluster end

    // Time cluster start
    esp_zb_attribute_list_t *esp_zb_server_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);

    esp_zb_attribute_list_t *esp_zb_client_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    esp_zb_time_cluster_add_attr(esp_zb_client_time_cluster, ESP_ZB_ZCL_ATTR_TIME_LAST_SET_TIME_ID, &undefined_value);

    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list1, esp_zb_server_time_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list1, esp_zb_client_time_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Time cluster end

    // OTA cluster start
    /** Create ota client cluster with attributes.
     *  Manufacturer code, image type and file version should match with configured values for server.
     *  If the client values do not match with configured values then it shall discard the command and
     *  no further processing shall continue.
     */
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_file_version = OTA_FW_VERSION,        // OTA_UPGRADE_RUNNING_FILE_VERSION,
        .ota_upgrade_downloaded_file_ver = OTA_FW_VERSION, // OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
        .ota_upgrade_manufacturer = manuf_id,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);

    /** add client parameters to ota client cluster */
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };

    uint16_t ota_upgrade_server_addr = 0xffff;
    uint8_t ota_upgrade_server_ep = 0xff;
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = 1,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEST_DEVICE_ID,
        .app_device_version = 0,
    };

    /* create cluster list with ota cluster */
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, (void *)&ota_upgrade_server_addr);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, (void *)&ota_upgrade_server_ep);

    esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list1, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    // OTA cluster end

    // IAS cluster start
    esp_zb_ias_zone_cluster_cfg_t ias_cluster_cfg = {
        .zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_STANDARD_CIE,
        .zone_state = ESP_ZB_ZCL_IAS_ZONE_ZONESTATE_NOT_ENROLLED,
        .zone_status = ESP_ZB_ZCL_IAS_ZONE_ZONE_STATUS_ALARM1,
        .ias_cie_addr = ESP_ZB_ZCL_ZONE_IAS_CIE_ADDR_DEFAULT,
        .zone_id = 0,
    };
    esp_zb_attribute_list_t *esp_zb_ias_zone_cluster = esp_zb_ias_zone_cluster_create(&ias_cluster_cfg);

    esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list1, esp_zb_ias_zone_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // IAS cluster end

    // Temperature cluster start
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster1 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster1, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster1, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster1, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list1, esp_zb_temperature_meas_cluster1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Temperature cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config1 = {
        .endpoint = DEVICE_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list1, endpoint_config1);
    // Endpoint config end
    // DEVICE_ENDPOINT end

    // AHT20_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list2 = esp_zb_zcl_cluster_list_create();

    // Temperature cluster start
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster2 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster2, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster2, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster2, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list2, esp_zb_temperature_meas_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Temperature cluster end

    // Humidity cluster start
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster2 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster2, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster2, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &zero_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster2, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list2, esp_zb_humidity_meas_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Humidity cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config2 = {
        .endpoint = AHT20_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list2, endpoint_config2);
    // Endpoint config end
    // AHT20_ENDPOINT end

    // BME280_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list3 = esp_zb_zcl_cluster_list_create();

    /*
    // On/Off cluster start
    esp_zb_on_off_cluster_cfg_t on_off_cfg3 = {};
    esp_zb_attribute_list_t *esp_zb_on_off_cluster3 = esp_zb_on_off_cluster_create(&on_off_cfg3);

    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list3, esp_zb_on_off_cluster3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // On/Off cluster end
    */

    // Temperature cluster start
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster3 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster3, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster3, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster3, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list3, esp_zb_temperature_meas_cluster3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Temperature cluster end

    // Pressure cluster start
    esp_zb_attribute_list_t *esp_zb_press_meas_cluster3 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_press_meas_cluster3, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_press_meas_cluster3, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_press_meas_cluster3, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list3, esp_zb_press_meas_cluster3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Pressure cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config3 = {
        .endpoint = BMP280_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list3, endpoint_config3);
    // Endpoint config end
    // BME280_ENDPOINT end

    // SCD40_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list4 = esp_zb_zcl_cluster_list_create();

    /*
    // On/Off cluster start
    esp_zb_on_off_cluster_cfg_t on_off_cfg4 = {};
    esp_zb_attribute_list_t *esp_zb_on_off_cluster4 = esp_zb_on_off_cluster_create(&on_off_cfg4);
    esp_zb_on_off_cluster_add_attr(esp_zb_on_off_cluster4, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &zero_value);
    esp_zb_on_off_cluster_add_attr(esp_zb_on_off_cluster4, ESP_ZB_ZCL_ATTR_ON_OFF_OFF_WAIT_TIME, &zero_value);

    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list4, esp_zb_on_off_cluster4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // On/Off cluster end
    */

    // CarbonDioxide cluster start
    esp_zb_attribute_list_t *esp_zb_carbon_dioxide_measurement_cluster4 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT);
    esp_zb_carbon_dioxide_measurement_cluster_add_attr(esp_zb_carbon_dioxide_measurement_cluster4, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &undefined_float);
    esp_zb_carbon_dioxide_measurement_cluster_add_attr(esp_zb_carbon_dioxide_measurement_cluster4, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MIN_MEASURED_VALUE_ID, &float_zero); // https://github.com/espressif/esp-zigbee-sdk/issues/147#issuecomment-1820778678
    esp_zb_carbon_dioxide_measurement_cluster_add_attr(esp_zb_carbon_dioxide_measurement_cluster4, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MAX_MEASURED_VALUE_ID, &float_one);
    // esp_zb_carbon_dioxide_measurement_cluster_add_attr(esp_zb_carbon_dioxide_measurement_cluster, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_TOLERANCE_ID, &undefined_value);

    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(esp_zb_cluster_list4, esp_zb_carbon_dioxide_measurement_cluster4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // CarbonDioxide cluster end

    // Temperature cluster start
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster4 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster4, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster4, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster4, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list4, esp_zb_temperature_meas_cluster4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Temperature cluster end

    // Humidity cluster start
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster4 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster4, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster4, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &zero_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster4, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list4, esp_zb_humidity_meas_cluster4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Humidity cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config4 = {
        .endpoint = SCD40_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list4, endpoint_config4);
    // Endpoint config end
    // SCD40_ENDPOINT end

    // BH1750_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list5 = esp_zb_zcl_cluster_list_create();

    // Illuminance cluster start
    esp_zb_attribute_list_t *esp_zb_illuminance_meas_cluster5 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster5, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &undefined_value);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster5, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE_ID, &undefined_value);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster5, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE_ID, &undefined_value);

    esp_zb_cluster_list_add_illuminance_meas_cluster(esp_zb_cluster_list5, esp_zb_illuminance_meas_cluster5, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Illuminance cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config5 = {
        .endpoint = BH1750_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list5, endpoint_config5);
    // Endpoint config end
    // BH1750_ENDPOINT end

    // AGS10_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list6 = esp_zb_zcl_cluster_list_create();

    /*
    // Custom cluster for TVOC start
    cluster_id = TVOC_CLUSTER; // if TVOC - 0xfc81 // heimanSpecificAirQuality no attributes can be readed.  but no errors. but if use 0x101 - error (Guru Meditation Error: Core  0 panic'ed (Load access fault). Exception was unhandled.)
    attr_id = TVOC_ATTR;
    // attr_type = ESP_ZB_ZCL_ATTR_TYPE_U16;
    // attr_access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;

    esp_zb_attribute_list_t *custom_tvoc_attributes_list6 = esp_zb_zcl_attr_list_create(cluster_id);
    esp_err_t err;

    err = esp_zb_cluster_add_attr(custom_tvoc_attributes_list6, cluster_id, attr_id, attr_type, attr_access, &undefined_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to add custom attribute 1: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(__func__, "Successfully added custom attribute 1");
    }

    // print_custom_attributes(custom_tvoc_attributes_list6);

    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list6, custom_tvoc_attributes_list6, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Custom cluster for TVOC end
    */

    // Analog Input cluster for TVOC start
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster6 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster6, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &undefined_value);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list6, esp_zb_analog_input_cluster6, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Analog Input cluster for TVOC end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config6 = {
        .endpoint = AGS10_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list6, endpoint_config6);
    // Endpoint config end
    // AGS10_ENDPOINT end

    // ADC1_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list7 = esp_zb_zcl_cluster_list_create();

    // Analog Input cluster start
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster7 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster7, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &undefined_value);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list7, esp_zb_analog_input_cluster7, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Analog Input cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config7 = {
        .endpoint = ADC1_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list7, endpoint_config7);
    // Endpoint config end
    // ADC1_ENDPOINT end

    // ADC2_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list8 = esp_zb_zcl_cluster_list_create();

    // Analog Input cluster start
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster8 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster8, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &undefined_value);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list8, esp_zb_analog_input_cluster8, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Analog Input cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config8 = {
        .endpoint = ADC2_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list8, endpoint_config8);
    // Endpoint config end
    // ADC2_ENDPOINT end

    // IMU_BASE_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list10 = esp_zb_zcl_cluster_list_create();

    // Analog Input cluster start
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster10 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster10, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &undefined_value);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list10, esp_zb_analog_input_cluster10, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Analog Input cluster end

    // Multistate Value cluster start
    esp_zb_attribute_list_t *esp_zb_multistate_value_cluster10 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE);
    // esp_zb_multistate_value_cluster_add_attr(esp_zb_multistate_value_cluster10, ESP_ZB_ZCL_ATTR_MULTISTATE_VALUE_PRESENT_VALUE_ID, &undefined_value);
    // esp_zb_multistate_value_cluster_add_attr(esp_zb_multistate_value_cluster11, ESP_ZB_ZCL_ATTR_MULTISTATE_VALUE_NUMBER_OF_STATES_ID, &undefined_value);

    // esp_zb_multistate_value_cluster_add_attr(esp_zb_multistate_value_cluster7, ESP_ZB_ZCL_ATTR_MULTISTATE_DESCRIPTION_ID, &undefined_value);

    cluster_id = ESP_ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE;
    attr_id = ESP_ZB_ZCL_ATTR_MULTISTATE_VALUE_PRESENT_VALUE_ID;
    esp_zb_cluster_add_attr(esp_zb_multistate_value_cluster10, cluster_id, attr_id, attr_type, attr_access, &zero_value);

    esp_zb_cluster_list_add_multistate_value_cluster(esp_zb_cluster_list10, esp_zb_multistate_value_cluster10, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Multistate Value cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config10 = {
        .endpoint = IMU_BASE_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list10, endpoint_config10);
    // Endpoint config end
    // IMU_BASE_ENDPOINT end

    // IMU_FIRST_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list11 = esp_zb_zcl_cluster_list_create();

    // Analog Input cluster start
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster11 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster11, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &undefined_value);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list11, esp_zb_analog_input_cluster11, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Analog Input cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config11 = {
        .endpoint = IMU_BASE_ENDPOINT + 1,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list11, endpoint_config11);
    // Endpoint config end
    // IMU_FIRST_ENDPOINT end

    // IMU_SECOND_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list12 = esp_zb_zcl_cluster_list_create();

    // Analog Input cluster start
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster12 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster12, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &undefined_value);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list12, esp_zb_analog_input_cluster12, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Analog Input cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config12 = {
        .endpoint = IMU_BASE_ENDPOINT + 2,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list12, endpoint_config12);
    // Endpoint config end
    // IMU_SECOND_ENDPOINT end

    // IMU_THIRD_ENDPOINT start
    esp_zb_cluster_list_t *esp_zb_cluster_list13 = esp_zb_zcl_cluster_list_create();

    // Analog Input cluster start
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster13 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster13, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &undefined_value);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list13, esp_zb_analog_input_cluster13, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // Analog Input cluster end

    // Endpoint config start
    esp_zb_endpoint_config_t endpoint_config13 = {
        .endpoint = IMU_BASE_ENDPOINT + 3,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list13, endpoint_config13);
    // Endpoint config end
    // IMU_THIRD_ENDPOINT end

    esp_zb_device_register(esp_zb_ep_list);
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();

    esp_zb_init(&zb_nwk_cfg);

    zigbee_config_endpoints_attributes();

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(true));

    esp_zb_main_loop_iteration();
}

void send_bin_cfg_option(int endpoint, bool value)
{
    if (connected)
    {
        const char *endpoint_name = get_endpoint_name(endpoint);
        ESP_LOGI(__func__, "attribute to %d on %s (endpoint %d)", value, endpoint_name, endpoint);

        esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(endpoint,
                                                                     ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                                     ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                                     &value,
                                                                     false);

        if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            ESP_LOGE(__func__, "Setting cfg option On/Off attribute failed! error %d", state_tmp);
        }
    }
    else
    {
        ESP_LOGW(__func__, "no connection! attribute to %d", value);
    }
}

void send_zone_1_state(uint8_t bit_index, uint8_t value)
{
    bool should_send = false;

    if (value < 2)
    {
        uint8_t old_zone_status = data.zone_status;
        set_bit(&data.zone_status, bit_index, value);
        ESP_LOGI(__func__, "new zone status: 0x%02x", data.zone_status);

        if (data.zone_status != old_zone_status)
        {
            should_send = true;
        }
    }
    else
    {
        should_send = true;
    }

    if (should_send && connected)
    {
        esp_zb_zcl_ias_zone_status_change_notif_cmd_t cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = 0x0000,
                .dst_endpoint = 1,
                .src_endpoint = 1,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .zone_status = data.zone_status,
            .zone_id = 0,
            .delay = 0,
        };

        esp_zb_zcl_ias_zone_status_change_notif_cmd_req(&cmd);
    }
    else if (!connected)
    {
        ESP_LOGW(__func__, "no connection! zone status: 0x%02x", data.zone_status);
    }
}

void zigbee_setup()
{
    ESP_LOGI("Zigbee", "Start Zigbee stack");

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    esp_zb_aps_src_binding_table_size_set(SRC_BINDING_TABLE_SIZE);
    esp_zb_aps_dst_binding_table_size_set(DST_BINDING_TABLE_SIZE);

    xTaskCreate(esp_zb_task, "esp_zb_task", 4096, NULL, 5, NULL);
}