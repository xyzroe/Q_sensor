#ifndef CONST_H
#define CONST_H

/* Text constants */
#define T_STATUS_FAILED "Failed!"
#define T_STATUS_DONE "Done"

/* Number constants */
#define LED_ON_STATE 0
#define LED_OFF_STATE 1

/* Zigbee configuration */
#define OTA_UPGRADE_MANUFACTURER 0x3443 /* The attribute indicates the value for the manufacturer of the device */
#define OTA_UPGRADE_IMAGE_TYPE 0x1011   /* The attribute indicates the type of the image */
#define OTA_UPGRADE_HW_VERSION 0x0101   /* The parameter indicates the version of hardware */
#define OTA_UPGRADE_MAX_DATA_SIZE 64    /* The parameter indicates the maximum data size of query block image */

#define MAX_CHILDREN 10                                                  /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE false                                  /* enable the install code policy for security */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

#define SRC_BINDING_TABLE_SIZE 64
#define DST_BINDING_TABLE_SIZE 64

#define HW_MANUFACTURER "xyzroe" /* The parameter indicates the manufacturer of the device */
#define HW_MODEL "Q_sensor"      /* The parameter indicates the model of the device */

#define DEVICE_ENDPOINT 1 /* the endpoint number for the device */
#define AHT20_ENDPOINT 2  /* the endpoint number for the AHT20 sensor */
#define BMP280_ENDPOINT 3 /* the endpoint number for the BMP280 sensor */
#define SCD40_ENDPOINT 4  /* the endpoint number for the SCD40 sensor */
#define BH1750_ENDPOINT 5 /* the endpoint number for the BH1750 sensor */
#define AGS10_ENDPOINT 6  /* the endpoint number for the AGS10 sensor */
#define ADC1_ENDPOINT 7   /* the endpoint number for the ADC1 sensor */
#define ADC2_ENDPOINT 8   /* the endpoint number for the ADC2 sensor */

#define IMU_BASE_ENDPOINT 10 /* the first endpoint number for the QMI8658C sensor */

// #define TVOC_CLUSTER 0xfc81 // 64641 // 0xfc81 // heimanSpecificAirQuality
// #define TVOC_ATTR 0xf004    // 61444    // 0xf004 // tvocMeasuredValue
// #define TVOC_CLUSTER 0x042b // msFormaldehyde
// #define TVOC_ATTR 0x0000    // measuredValue

#define ESP_ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE 0x0014
// #define ESP_ZB_ZCL_ATTR_MULTISTATE_VALUE_STATE_TEXT_ID 0x000E    // ERROR while CREATE
#define ESP_ZB_ZCL_ATTR_MULTISTATE_VALUE_PRESENT_VALUE_ID 0x0055    // OK //
#define ESP_ZB_ZCL_ATTR_MULTISTATE_VALUE_NUMBER_OF_STATES_ID 0x004A // OK
#define ESP_ZB_ZCL_ATTR_MULTISTATE_DESCRIPTION_ID 0x001C            // OK

#define OTA_FW_VERSION 0x00000016 /* The attribute indicates the version of the firmware */
#define FW_BUILD_DATE "20241023"  /* The parameter indicates the build date of the firmware */

/* GPIO configuration */
#define BTN_GPIO 10 /* Button */

#define ALARM_GPIO 4 /* Radar pin */
#define LED_GPIO 5   /* blue LED */

#define ADC_1_GPIO 2 /* ADC1 */
#define ADC_2_GPIO 0 /* ADC2 */

#define I2C_SDA_GPIO 6 /* I2C SDA */
#define I2C_SCL_GPIO 7 /* I2C SCL */

/* Time constants */

#define LONG_PRESS_TIME 5000             /* to make factory reset */
#define SHORT_PRESS_TIME 150             /* to toggle USB power */
#define UPDATE_ATTRIBUTE_INTERVAL 600000 /* 10 minutes to FORCE update all states */
#define WAIT_BEFORE_FIRST_UPDATE 15000   /* 15 seconds to wait before first update */

#define DEBUG_TASK_INTERVAL 10000 /* Debug task interval */

#define CPU_TEMP_INTERVAL 15000     /* CPU temp task interval */
#define ADC_INTERVAL 10000          /* ADC task interval */
#define AHT_TASK_INTERVAL 10000     /* AHT task interval */
#define BMP280_TASK_INTERVAL 10000  /* BMP280 task interval */
#define SCD4X_TASK_INTERVAL 10000   /* SCD4X task interval */
#define BH1750_TASK_INTERVAL 10000  /* BH1750 task interval */
#define QMI8658C_TASK_INTERVAL 5000 /* QMI8658C task interval */
#define AGS10_TASK_INTERVAL 10000   /* AGS10 task interval */

#endif // CONST_H
