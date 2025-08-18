/**
 * @file hum_temp_sensor.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief Humidity and Temperature sensor interface.
 *
 * This file implements initialization, sampling, and logging of humidity and temperature
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
#define HTS_THREAD_STACK_SIZE 512
#define HTS_THREAD_PRIORITY   5

#define HTS_SAMPLE_DELAY 10 // 10 Seconds

#if DT_NODE_EXISTS(DT_ALIAS(ht_sensor))
#define HUM_TEMP_NODE DT_ALIAS(ht_sensor)
static const struct device *const hts_dev = DEVICE_DT_GET(DT_ALIAS(ht_sensor));
#else
#error("Humidity-Temperature sensor not found.");
#endif

//------------------------------------------------------------------------------
// typedef/structs/enums
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(ht_sensor);

K_THREAD_STACK_DEFINE(hts_stack_area, HTS_THREAD_STACK_SIZE);

static struct k_thread hts_thread;
static k_tid_t hts_tid;

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void hts_sensor_thread(void *arg0, void *arg1, void *arg2);
static int hum_temp_sensor_process_sample(double *hum, double *temp);
static int hun_temp_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/* Humidity-Temperature Sensor Thread Function */
static void hts_sensor_thread(void *arg0, void *arg1, void *arg2)
{
    while (1) {
        double hum, temp;
        if (hum_temp_sensor_process_sample(&hum, &temp) < 0) {
            return;
        }
        /* display temperature */
        LOG_INF("Temperature:%.1f C", temp);

        /* display humidity */
        LOG_INF("Relative Humidity:%.1f%%", hum);
        k_sleep(K_SECONDS(HTS_SAMPLE_DELAY));
    }
}

/**
 * @brief Process and log the humidity and temperature sensor sample.
 *
 * This function fetches the latest sensor data, checks if the device is ready,
 * and logs the temperature and humidity values. The measured humidity and temperature
 * are returned via the provided pointers.
 *
 * @param hum Pointer to store the measured humidity value.
 * @param temp Pointer to store the measured temperature value.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int hum_temp_sensor_process_sample(double *hum, double *temp)
{
    if (!device_is_ready(hts_dev)) {
        LOG_ERR("sensor: %s device not ready.", hts_dev->name);
        return -ENODEV;
    }

    if (sensor_sample_fetch(hts_dev) < 0) {
        LOG_ERR("sensor: %s sample update error", hts_dev->name);
        return -EIO;
    }

    struct sensor_value temp_val, hum_val;
    if (sensor_channel_get(hts_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val) < 0) {
        LOG_ERR("sensor: %s read temperature channel failed", hts_dev->name);
        return -EINVAL;
    }
    *temp = sensor_value_to_double(&temp_val);

    if (sensor_channel_get(hts_dev, SENSOR_CHAN_HUMIDITY, &hum_val) < 0) {
        LOG_ERR("sensor: %s read humidity channel failed", hts_dev->name);
        return -EINVAL;
    }
    *hum = sensor_value_to_double(&hum_val);
    return 0;
}

/**
 * @brief Initialize the humidity and temperature sensor.
 *
 * This function checks if the sensor device is ready and processes an initial sample.
 *
 * @return int Returns 0 upon successful initialization, or a negative error code on failure.
 */
static int hun_temp_sensor_init(void)
{
    if (!device_is_ready(hts_dev)) {
        LOG_ERR("sensor: %s device not ready.", hts_dev->name);
        return -ENODEV;
    }

    hts_tid = k_thread_create(&hts_thread, hts_stack_area, K_THREAD_STACK_SIZEOF(hts_stack_area), hts_sensor_thread,
                              NULL, NULL, NULL, HTS_THREAD_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Humidity-Temperature sensor %s initialized.", hts_dev->name);
    return 0;
}

SYS_INIT(hun_temp_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
