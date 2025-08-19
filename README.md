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
    ├── hum_sensor.c        # Humidity Sensor setup
    ├── imu_sensor.c        # IMU Sensor setup
    ├── magn_sensor.c       # Magnetometer Sensor setup
    ├── pressure_sensor.c   # Pressure Sensor setup
    ├── sensor_logger.c     # Sensor Logger setup
    └── temp_sensor.c       # Temperature Sensor setup
```

## 📅 TODO list

- [x] Add Temperature Sensor
- [x] Add Humidity Sensor
- [x] Add Pressure Sensor
- [x] Add IMU Sensor
- [x] Add Magnetometer Sensor
- [ ] Sensor Logger functionality
- [ ] IPC Mechanism to send data to Sensor Logger
- [ ] BT Peripheral functionality
- [ ] WiFi functionality
- [ ] MQTT functionality
- [ ] Display sensor data on SSD1306
- [ ] Display sensor data on Touch Display
