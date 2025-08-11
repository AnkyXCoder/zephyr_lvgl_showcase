# Zephyr LVGL Showcase

This project is a comprehensive demonstration of various capabilities available within the Zephyr RTOS ecosystem. It showcases a rich user interface built with LVGL, real-time sensor data visualization, and IoT connectivity via MQTT.

## 🎯 Setup Zephyr Environment

First, ensure you have a working Zephyr development environment. If you don't, please follow the official [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

- **Zephyr version:** 4.2.0
- **Zephyr SDK:** v0.17.0

- **Tested boards:**
  - nucleo_g0b1re
  - nrf52840dk/nrf52840
  - esp32s3_devkitc/esp32s3/procpu


## 🛠️ Build

```shell
# From the project's root directory
west build -b <board_name>
```

## ⚡️ Flash

```shell
west flash
```

## 🔧 Project Structure

```tree
zephyr_lvgl_showcase/
├── CMakeLists.txt      # Main CMake build script
├── prj.conf            # Kconfig project configuration
├── README.md           # This file
└── src/
    ├── main.c              # Main application entry point and thread setup
    ├── hum_temp_sensor.h
    ├── hum_temp_sensor.c   # Humidity-Temperature Sensor setup
    ├── pressure_senosr.h
    └── pressure_senosr.c   # Pressure Sensor setup
```

## 📅 TODO list

- [x] Add Temperature-Humidity sensor
- [x] Add Pressure Sensor
- [ ] Add IMU sensor
- [ ] Add basic interface in LVGL with styles
