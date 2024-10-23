#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "string.h"

#include "../../const.h"
#include "../../zigbee.h"
#include "../../main.h"

#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_2
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12

static int adc_raw[2][10];
static int voltage3[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

void adc_task(void *pvParameters)
{
    ESP_LOGI(__func__, "initializing ADC sensor");

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
    bool do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);

    ESP_LOGI(__func__, "starting the loop");
    while (1)
    {

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        if (do_calibration1_chan0)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage3[0][0]));
        }

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[0][1]));
        if (do_calibration1_chan1)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage3[0][1]));
        }

        ESP_LOGI(__func__, "ADC%d Channel[%d] Cali Voltage: %d mV, Channel[%d] Cali Voltage: %d mV",
                 ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage3[0][0], EXAMPLE_ADC1_CHAN1, voltage3[0][1]);

        uint16_t new_adc1 = voltage3[0][0];
        uint16_t new_adc2 = voltage3[0][1];

        if ((new_adc1 != data.adc1 || new_adc2 != data.adc2) && connected)
        {
            data.adc1 = new_adc1;
            data.adc2 = new_adc2;
            update_attributes(ATTRIBUTE_ADC);
        }

        /*
        uint16_t new_aht_temp = (float)(temperature * 100);
        uint16_t new_aht_hum = (float)(humidity * 100);

        if ((new_aht_temp != data.aht_temp || new_aht_hum != data.aht_hum) && connected)
        {
            data.aht_temp = new_aht_temp;
            data.aht_hum = new_aht_hum;
            update_attributes(ATTRIBUTE_AHT);
        }
        */

        vTaskDelay(pdMS_TO_TICKS(ADC_INTERVAL));
    }
}

void sensor_adc()
{
    ESP_LOGW(__func__, "ADC sensor task started");
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 3, NULL);
}

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(__func__, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(__func__, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(__func__, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(__func__, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(__func__, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(__func__, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(__func__, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}