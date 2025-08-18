/**
 * @file hum_sensor.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief Humidity sensor interface.
 *
 * This file implements initialization, sampling, and logging of humidity
 * data from a sensor using Zephyr's sensor API. It creates a thread to periodically
 * fetch and display sensor readings.
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
#define HUMID_THREAD_STACK_SIZE 512
#define HUMID_THREAD_PRIORITY   5

#define HUMID_SAMPLE_DELAY 1 // 1 Second

#if DT_NODE_EXISTS(DT_ALIAS(hum_sensor))
#define HUMID_NODE DT_ALIAS(hum_sensor)
static const struct device *const hum_dev = DEVICE_DT_GET(HUMID_NODE);
#else
#error("Humidity sensor not found.");
#endif

//------------------------------------------------------------------------------
// typedef/structs/enums
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(hum);

K_THREAD_STACK_DEFINE(hum_stack_area, HUMID_THREAD_STACK_SIZE);

static struct k_thread hum_thread;
static k_tid_t hum_tid;

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void humidity_sensor_thread(void *arg0, void *arg1, void *arg2);
static int humidity_sensor_process_sample(double *hum);
static int humidity_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/* Humidity Sensor Thread Function */
static void humidity_sensor_thread(void *arg0, void *arg1, void *arg2)
{
    while (1) {
        double hum;
        if (humidity_sensor_process_sample(&hum) < 0) {
            return;
        }

        /* display humidity */
        LOG_INF("Relative Humidity:%.1f%%", hum);
        k_sleep(K_SECONDS(HUMID_SAMPLE_DELAY));
    }
}

/**
 * @brief Process and log the humidity sensor sample.
 *
 * This function fetches the latest sensor data, checks if the device is ready,
 * and logs the temperature values. The measured humidity are returned via the provided pointers.
 *
 * @param hum Pointer to store the measured humidity value.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int humidity_sensor_process_sample(double *hum)
{
    if (!device_is_ready(hum_dev)) {
        LOG_ERR("sensor: %s device not ready.", hum_dev->name);
        return -ENODEV;
    }

    if (sensor_sample_fetch(hum_dev) < 0) {
        LOG_ERR("sensor: %s sample update error", hum_dev->name);
        return -EIO;
    }

    struct sensor_value temp_val, hum_val;
    if (sensor_channel_get(hum_dev, SENSOR_CHAN_HUMIDITY, &hum_val) < 0) {
        LOG_ERR("sensor: %s read humidity channel failed", hum_dev->name);
        return -EINVAL;
    }
    *hum = sensor_value_to_double(&hum_val);

    if (sensor_channel_get(hum_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val) < 0) {
        LOG_ERR("sensor: %s read temperature channel failed", hum_dev->name);
        return -EINVAL;
    }
    return 0;
}

/**
 * @brief Initialize the humidity sensor.
 *
 * This function checks if the sensor device is ready and processes an initial sample.
 *
 * @return int Returns 0 upon successful initialization, or a negative error code on failure.
 */
static int humidity_sensor_init(void)
{
    if (!device_is_ready(hum_dev)) {
        LOG_ERR("sensor: %s device not ready.", hum_dev->name);
        return -ENODEV;
    }

    hum_tid = k_thread_create(&hum_thread, hum_stack_area, K_THREAD_STACK_SIZEOF(hum_stack_area),
                              humidity_sensor_thread, NULL, NULL, NULL, HUMID_THREAD_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Humidity sensor %s initialized.", hum_dev->name);
    return 0;
}

SYS_INIT(humidity_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
