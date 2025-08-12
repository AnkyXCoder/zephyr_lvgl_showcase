/**
 * @file hum_temp_sensor.c
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

//------------------------------------------------------------------------------
// #define/Constants/Macros
//------------------------------------------------------------------------------

#if DT_NODE_EXISTS(DT_ALIAS(ht_sensor))
#define HUM_TEMP_NODE DT_ALIAS(ht_sensor)
static const struct device *const hts_dev = DEVICE_DT_GET(DT_ALIAS(ht_sensor));
#else
#error("Humidity-Temperature sensor not found.");
#endif

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(hum_temp);

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void hum_temp_sensor_process_sample(void);
static int hun_temp_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/**
 * @brief Process and log the humidity and temperature sensor sample.
 *
 * This function fetches the latest sensor data, checks if the device is ready,
 * and logs the temperature and humidity values.
 */
static void hum_temp_sensor_process_sample(void)
{
    if (!device_is_ready(hts_dev)) {
        LOG_ERR("sensor: %s device not ready.", hts_dev->name);
        return;
    }

    if (sensor_sample_fetch(hts_dev) < 0) {
        LOG_ERR("sensor: %s sample update error", hts_dev->name);
        return;
    }

    struct sensor_value temp, hum;
    if (sensor_channel_get(hts_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
        LOG_ERR("sensor: %s read temperature channel failed", hts_dev->name);
        return;
    }

    if (sensor_channel_get(hts_dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
        LOG_ERR("sensor: %s read humidity channel failed", hts_dev->name);
        return;
    }

    /* display temperature */
    LOG_INF("Temperature:%.1f C", sensor_value_to_double(&temp));

    /* display humidity */
    LOG_INF("Relative Humidity:%.1f%%", sensor_value_to_double(&hum));
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
    LOG_INF("Humidity-Temperature sensor %s initialized.", hts_dev->name);
    return 0;
}

SYS_INIT(hun_temp_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);