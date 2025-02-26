# Q_sensor

<div align="center"> 
<a href="https://github.com/xyzroe/Q_sensor/releases"><img src="https://img.shields.io/github/release/xyzroe/Q_sensor.svg" alt="GitHub version"></img></a>
<a href="https://github.com/xyzroe/Q_sensor/actions/workflows/build.yml"><img src="https://img.shields.io/github/actions/workflow/status/xyzroe/Q_sensor/build.yml" alt="GitHub Actions Workflow Status"></img></a>
<a href="https://github.com/xyzroe/Q_sensor/releases/latest"><img src="https://img.shields.io/github/downloads/xyzroe/Q_sensor/total.svg" alt="GitHub download"></img></a>
<a href="https://github.com/xyzroe/Q_sensor/issues"><img src="https://img.shields.io/github/issues/xyzroe/Q_sensor" alt="GitHub Issues or Pull Requests"></img></a>
</div>

## Overview

Q_sensor is a multi-functional Zigbee air quality sensor that integrates various environmental sensors to provide comprehensive monitoring of air quality: temperature, humidity, pressure, CO2 level, VOC index, illuminance.

<div style="display: flex; align-items: center;">

<div style="flex: 0 0 60%;">

### Sensors:
- **AGS10** - VOC sensor (0x1a)
- **SCD40** - CO2 sensor (0x62) 
- **AHT20** - Temperature sensor (0x38) 
- **BH1750** - Ambient Light sensor (0x23) 
- **BMP280** - Barometric pressure sensor (0x77) 
- **QMI8658C** - 6-axis attitude gyro sensor (0x6b) 
- **BS5820** - 5.8G radar sensor (Binary)

Found something at address 0x7e?

### GPIO
- **ADC pin V1** - IO2
- **ADC pin V2** - IO0
- **Button** - IO10
- **Radar** - IO4
- **LED** - IO5
- **SDA** - IO6
- **SCL** - IO7

</div>
<img src="./images/1.png" width="35%" alt="Overview" style="margin-left: auto; margin-top: 50px">
</div>

## zigbee2mqtt

<div style="display: flex;">
<div style="flex: 0 0 60%;">

External converter <a href=./external_converter/q.js>q.js</a>

### Features
- Almost all values use standard clusters (so it will work on any system)
- Configurable reporting
- OTA update possible
- SCD40 gets pressure data from BMP280 for more accurate readings
- All sensors read every 10 seconds
  
  
If you don't need some value - go to reporting tab and set  
`Min rep interval`, `Max rep interval` to `65534` and `Min rep change` to `0`

</div>
<img src="./images/z2m.png" width="25%" alt="Z2M" style="margin-left: auto; margin-top: 50px">
</div>

## Install

You can find the latest release on <a href="./releases">releases page</a> and install like any other ESP32 firmware.

## Where to buy?

[Official store](https://www.tindie.com/products/adz1122/esp32-c6-multi-sensor-co2-voc-imu/)

## License

<div align="center">
<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />
This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>
</div>