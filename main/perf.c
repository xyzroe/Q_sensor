#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "iot_button.h"
#include "i2cdev.h"
#include "string.h"

#include "main.h"
#include "const.h"
#include "perf.h"
#include "tools.h"
#include "zigbee.h"

#include "sensors/adc/sensor_adc.h"
#include "sensors/chip_temp/sensor_chip_temp.h"
#include "sensors/aht/sensor_aht.h"
#include "sensors/bmp280/sensor_bmp280.h"
#include "sensors/scd4x/sensor_scd4x.h"
#include "sensors/ags10/sensor_ags10.h"
#include "sensors/bh1750/sensor_bh1750.h"
#include "sensors/qmi8658c/sensor_qmi8658c.h"

void init_outputs()
{
    ESP_LOGI(__func__, "setup LED gpio");
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(LED_GPIO);
    gpio_set_level(LED_GPIO, LED_OFF_STATE);

    /*
        gpio_reset_pin(INT_LED_GPIO);
        gpio_set_direction(INT_LED_GPIO, GPIO_MODE_OUTPUT);
        gpio_pulldown_en(INT_LED_GPIO);
        gpio_set_level(INT_LED_GPIO, LED_OFF_STATE);

        ESP_LOGI(__func__, "setup USB gpio");
        gpio_reset_pin(USB_GPIO);
        gpio_set_direction(USB_GPIO, GPIO_MODE_OUTPUT);
        gpio_pullup_en(USB_GPIO);
        */
    /*
    switch (data.start_up_on_off)
    {
    case 0:
        usb_driver_set_power(0);
        break;
    case 1:
        usb_driver_set_power(1);
        break;
    case 2:
        usb_driver_set_power(!data.USB_state);
        break;
    case 255:
        usb_driver_set_power(data.USB_state);
        break;
    default:
        break;
    }
    */
}

/*(void init_pullup_i2c_pins()
{
    ESP_LOGI(__func__, "setup I2C_SDA_GPIO");
    gpio_reset_pin(I2C_SDA_GPIO);
    gpio_pullup_en(I2C_SDA_GPIO);
    ESP_LOGI(__func__, "setup I2C_SCL_GPIO");
    gpio_reset_pin(I2C_SCL_GPIO);
    gpio_pullup_en(I2C_SCL_GPIO);
}*/

/*
void usb_driver_set_power(bool state)
{
    gpio_set_level(USB_GPIO, state);
    ext_led_action(state);
    ESP_LOGI(__func__, "Setting USB power to %d", state);
    data.USB_state = state;
    if (data.start_up_on_off > 1)
    {
        write_NVS("USB_state", state);
    }
}
*/

void led_task(void *pvParameters)
{
    ESP_LOGI(__func__, "starting");
    // led_hz = 1;
    while (1)
    {
        if (led_hz > 0)
        {
            led_blink(150);
            vTaskDelay(pdMS_TO_TICKS(1000 / led_hz));
        }
        else
        {
            // LED disabled

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void led_blink(int delay)
{
    if (data.led_mode)
    {
        gpio_set_level(LED_GPIO, LED_ON_STATE);
        vTaskDelay(pdMS_TO_TICKS(delay));
        gpio_set_level(LED_GPIO, LED_OFF_STATE);
    }
}

void register_alarm_input()
{
    button_config_t gpio_alarm_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = ALARM_GPIO,
            .active_level = 0,
        },
    };

    button_handle_t gpio_alarm = iot_button_create(&gpio_alarm_cfg);
    if (NULL == gpio_alarm)
    {
        ESP_LOGE(__func__, "Alarm input create failed");
    }
#define NO_MOTION 0
#define MOTION 1
    iot_button_register_cb(gpio_alarm, NO_MOTION, radar_deactive_cb, NULL);
    iot_button_register_cb(gpio_alarm, MOTION, radar_active_cb, NULL);
}

static void radar_active_cb(void *arg, void *usr_data)
{
    ESP_LOGI(__func__, "motion detected");

    send_zone_1_state(0, 1);
}

static void radar_deactive_cb(void *arg, void *usr_data)
{
    ESP_LOGI(__func__, "no motion detected");

    send_zone_1_state(0, 0);
}

void register_button(int pin)
{
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = LONG_PRESS_TIME,
        .short_press_time = SHORT_PRESS_TIME,
        .gpio_button_config = {
            .gpio_num = pin,
            .active_level = 0,
        },
    };

    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn)
    {
        ESP_LOGE(__func__, "Button on pin %d create failed", pin);
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_cb, NULL);
    ESP_LOGI(__func__, "Button on pin %d registered", pin);
}

static void button_single_click_cb(void *arg, void *usr_data)
{
    ESP_LOGI(__func__, "single click");
    force_update();
}

static void button_long_press_cb(void *arg, void *usr_data)
{
    ESP_LOGI(__func__, "long press - leave & reset");
    // data.ext_led_mode = 1; // Force turn LED ON
    for (int i = 0; i < 5; i++)
    {
        led_blink(150);
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    esp_zb_bdb_reset_via_local_action();
    esp_zb_factory_reset();
}

void test_task(void *pvParameters)
{

    ESP_LOGI(__func__, "starting test task");
    while (1)
    {
        float ina_bus_voltage = round_to_decimals(random_float(4.5, 5.5), 4);
        float ina_current = round_to_decimals(random_float(0.0, 2.0), 4);
        float ina_power = round_to_decimals((ina_bus_voltage * ina_current), 4);

        ESP_LOGW(__func__, "VBUS: %.04f V, IBUS: %.04f A, PBUS: %.04f W",
                 ina_bus_voltage, ina_current, ina_power);

        // ZCL must be V,A,W = uint16. But we need more accuaracy - so use *100.
        // voltage = (float)(ina_bus_voltage * 100);
        // current = (float)(ina_current * 100);
        // power = (float)(ina_power * 100);

        // update_attributes(ATTRIBUTE_ELECTRO);

        vTaskDelay(pdMS_TO_TICKS(CPU_TEMP_INTERVAL));
    }
}

void run_sensors()
{
    scan_i2c_bus(I2C_SDA_GPIO, I2C_SCL_GPIO);

    ESP_LOGI(__func__, "starting sensors tasks");

    sensor_adc();

    sensor_chip_temp();

    sensor_aht();

    sensor_bmp280();

    sensor_ags10();

    sensor_bh1750();

    sensor_qmi8658c();

    sensor_scd4x();
}

void identify_task(void *pvParameters)
{
    int time = (int)pvParameters;

    float old_led_hz = led_hz;
    bool old_led_mode = data.led_mode;
    data.led_mode = 1;
    led_hz = 2;

    if (ledTaskHandle != NULL)
    {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = NULL;
    }
    xTaskCreate(led_task, "led_task", 4096, NULL, 3, &ledTaskHandle);

    vTaskDelay(time * 1000 / portTICK_PERIOD_MS);

    if (ledTaskHandle != NULL)
    {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = NULL;
    }
    led_hz = old_led_hz;
    led_blink(10);

    data.led_mode = old_led_mode;

    data.old_identify_time = 0;

    ESP_LOGI(__func__, "Identify exit");
    vTaskDelete(NULL);
}