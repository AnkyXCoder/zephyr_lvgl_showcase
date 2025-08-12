/**
 * @file imu_sensor.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief
 * @version 0.1
 * @date 12-08-2025
 *
 * @copyright Copyright (c) 2025
 *
 */
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <cerrno>

//------------------------------------------------------------------------------
// #define/Constants/Macros
//------------------------------------------------------------------------------

#if DT_NODE_EXISTS(DT_ALIAS(imu_sensor))
#define HUM_TEMP_NODE DT_ALIAS(imu_sensor)
static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu_sensor));
#else
#error("IMU sensor not found.");
#endif

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(imu);

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void imu_sensor_sample_process(void);
static int imu_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/**
 * @brief Process and log the IMU sensor sample.
 *
 * This function fetches the latest sensor data, checks if the device is ready,
 * and logs the accelerometer and gyroscope values.
 */
static void imu_sensor_sample_process(void)
{
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("sensor: %s device not ready.", imu_dev->name);
        return;
    }

    if (sensor_sample_fetch(imu_dev) < 0) {
        LOG_ERR("sensor: %s sample update error", imu_dev->name);
        return;
    }
    char out_str[64];
    struct sensor_value accel_x, accel_y, accel_z;
    struct sensor_value gyro_x, gyro_y, gyro_z;

    /* lsm6dsl accel */
    sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

    sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2", sensor_value_to_double(&accel_x),
            sensor_value_to_double(&accel_y), sensor_value_to_double(&accel_z));
    LOG_INF("Accel: %s", out_str);

    /* lsm6dsl gyro */
    sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

    sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps", sensor_value_to_double(&gyro_x),
            sensor_value_to_double(&gyro_y), sensor_value_to_double(&gyro_z));
    LOG_INF("Gyro: %s", out_str);
}

/**
 * @brief Initialize the IMU sensor.
 *
 * This function checks if the IMU device is ready and sets the sampling frequency
 * for accelerometer and gyroscope channels.
 *
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int imu_sensor_init(void)
{
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("sensor: %s device not ready.", imu_dev->name);
        return -ENODEV;
    }

    /* set accel/gyro sampling frequency to 104 Hz */
    struct sensor_value odr_attr;
    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("sensor: %s cannot set sampling frequency for accelerometer.", imu_dev->name);
        return -EINVAL;
    }

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("sensor: %s cannot set sampling frequency for gyro.", imu_dev->name);
        return -EINVAL;
    }
    LOG_INF("IMU sensor %s initialized.", imu_dev->name);
    return 0;
}

SYS_INIT(imu_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
