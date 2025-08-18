/**
 * @file imu_sensor.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief IMU sensor interface.
 *
 * This file implements initialization, sampling, and logging of IMU sensor data
 * (accelerometer and gyroscope) using Zephyr's sensor API. It creates a thread
 * for periodic sampling and logs the sensor values for use in the application.
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

//------------------------------------------------------------------------------
// #define/Constants/Macros
//------------------------------------------------------------------------------
#define IMU_THREAD_STACK_SIZE 768
#define IMU_THREAD_PRIORITY   5

#define IMU_SAMPLE_DELAY 500 // 500 milli-seconds

#if DT_NODE_EXISTS(DT_ALIAS(imu_sensor))
#define HUM_TEMP_NODE DT_ALIAS(imu_sensor)
static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu_sensor));
#else
#error("IMU sensor not found.");
#endif

//------------------------------------------------------------------------------
// typedef/structs/enums
//------------------------------------------------------------------------------

typedef struct {
    double accel_x;
    double accel_y;
    double accel_z;
} accel_t;

typedef struct {
    double gyro_x;
    double gyro_y;
    double gyro_z;
} gyro_t;

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(imu);

K_THREAD_STACK_DEFINE(imu_stack_area, IMU_THREAD_STACK_SIZE);

static struct k_thread imu_thread;
static k_tid_t imu_tid;

static struct sensor_value accel_x, accel_y, accel_z;
static struct sensor_value gyro_x, gyro_y, gyro_z;

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void imu_sensor_thread(void *arg0, void *arg1, void *arg2);
static int imu_sensor_sample_process(accel_t *accel_val, gyro_t *gyro_val);
static int imu_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/* IMU Sensor Thread Function */
static void imu_sensor_thread(void *arg0, void *arg1, void *arg2)
{
    while (1) {
        accel_t accel;
        gyro_t gyro;
        char out_str[64];

        if (imu_sensor_sample_process(&accel, &gyro) < 0) {
            return;
        }

        sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2", accel.accel_x, accel.accel_y, accel.accel_z);
        LOG_INF("Accel: %s", out_str);

        sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        LOG_INF("Gyro: %s", out_str);
        k_sleep(K_MSEC(IMU_SAMPLE_DELAY));
    }
}
/**
 * @brief Fetch and process IMU sensor sample.
 *
 * This function checks if the IMU device is ready, fetches the latest sensor data,
 * reads accelerometer and gyroscope values, and stores them in the provided structures.
 *
 * @param accel_val Pointer to accel_t structure to store accelerometer values.
 * @param gyro_val Pointer to gyro_t structure to store gyroscope values.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int imu_sensor_sample_process(accel_t *accel_val, gyro_t *gyro_val)
{
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("sensor: %s device not ready.", imu_dev->name);
        return -ENODEV;
    }

    if (sensor_sample_fetch(imu_dev) < 0) {
        LOG_ERR("sensor: %s sample update error", imu_dev->name);
        return -EIO;
    }

    /* IMU accel */
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
        LOG_ERR("sensor: %s sample accel update error", imu_dev->name);
        return -EIO;
    }

    if ((sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x) < 0) ||
        (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y) < 0) ||
        (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z) < 0)) {
        LOG_ERR("sensor: %s read accel channel failed", imu_dev->name);
        return -EINVAL;
    }
    accel_val->accel_x = sensor_value_to_double(&accel_x);
    accel_val->accel_y = sensor_value_to_double(&accel_y);
    accel_val->accel_z = sensor_value_to_double(&accel_z);

    /* IMU gyro */
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ) < 0) {
        LOG_ERR("sensor: %s sample gyro update error", imu_dev->name);
        return -EIO;
    }

    if ((sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x) < 0) ||
        (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y) < 0) ||
        (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z) < 0)) {
        LOG_ERR("sensor: %s read gyro channel failed", imu_dev->name);
        return -EINVAL;
    }

    gyro_val->gyro_x = sensor_value_to_double(&gyro_x);
    gyro_val->gyro_y = sensor_value_to_double(&gyro_y);
    gyro_val->gyro_z = sensor_value_to_double(&gyro_z);
    return 0;
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

    imu_tid = k_thread_create(&imu_thread, imu_stack_area, K_THREAD_STACK_SIZEOF(imu_stack_area), imu_sensor_thread,
                              NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, K_NO_WAIT);

    LOG_INF("IMU sensor %s initialized.", imu_dev->name);
    return 0;
}

SYS_INIT(imu_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
