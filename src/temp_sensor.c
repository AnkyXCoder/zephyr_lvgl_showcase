/**
 * @file temp_sensor.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief Temperature sensor interface.
 *
 * This file implements initialization, sampling, and logging of temperature
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
#define TEMP_THREAD_STACK_SIZE 512
#define TEMP_THREAD_PRIORITY   5

#define TEMP_SAMPLE_DELAY 1 // 1 Second

#if DT_NODE_EXISTS(DT_ALIAS(temp_sensor))
#define TEMP_NODE DT_ALIAS(temp_sensor)
static const struct device *const temp_dev = DEVICE_DT_GET(TEMP_NODE);
#else
#error("Temperature sensor not found.");
#endif

//------------------------------------------------------------------------------
// typedef/structs/enums
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(temp);

K_THREAD_STACK_DEFINE(temp_stack_area, TEMP_THREAD_STACK_SIZE);

static struct k_thread temp_thread;
static k_tid_t temp_tid;

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void temp_sensor_thread(void *arg0, void *arg1, void *arg2);
static int temp_sensor_process_sample(double *temp);
static int temp_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/* Temperature Sensor Thread Function */
static void temp_sensor_thread(void *arg0, void *arg1, void *arg2)
{
    while (1) {
        double temp;
        if (temp_sensor_process_sample(&temp) < 0) {
            return;
        }
        /* display temperature */
        LOG_INF("Temperature:%.1f C", temp);

        k_sleep(K_SECONDS(TEMP_SAMPLE_DELAY));
    }
}

/**
 * @brief Process and log the temperature sensor sample.
 *
 * This function fetches the latest sensor data, checks if the device is ready,
 * and logs the humidity values. The measured temperature are returned via the provided pointers.
 *
 * @param temp Pointer to store the measured temperature value.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int temp_sensor_process_sample(double *temp)
{
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("sensor: %s device not ready.", temp_dev->name);
        return -ENODEV;
    }

    if (sensor_sample_fetch(temp_dev) < 0) {
        LOG_ERR("sensor: %s sample update error", temp_dev->name);
        return -EIO;
    }

    struct sensor_value temp_val;
    if (sensor_channel_get(temp_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val) < 0) {
        LOG_ERR("sensor: %s read temperature channel failed", temp_dev->name);
        return -EINVAL;
    }
    *temp = sensor_value_to_double(&temp_val);

    return 0;
}

/**
 * @brief Initialize the temperature sensor.
 *
 * This function checks if the sensor device is ready and processes an initial sample.
 *
 * @return int Returns 0 upon successful initialization, or a negative error code on failure.
 */
static int temp_sensor_init(void)
{
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("sensor: %s device not ready.", temp_dev->name);
        return -ENODEV;
    }

    temp_tid = k_thread_create(&temp_thread, temp_stack_area, K_THREAD_STACK_SIZEOF(temp_stack_area),
                               temp_sensor_thread, NULL, NULL, NULL, TEMP_THREAD_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Temperature sensor %s initialized.", temp_dev->name);
    return 0;
}

SYS_INIT(temp_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
