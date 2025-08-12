/**
 * @file pressure_sensor.c
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
// #define Constants/Macros
//------------------------------------------------------------------------------

#if DT_NODE_EXISTS(DT_ALIAS(pressure_sensor))
#define PRESSURE_NODE DT_ALIAS(pressure_sensor)
const struct device *const pressure_dev = DEVICE_DT_GET(DT_ALIAS(pressure_sensor));
#else
#error("Pressure sensor not found.");
#endif

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(pressure);

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void pressure_sensor_process_sample(void);
static int pressure_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/**
 * @brief Process and log the pressure sensor sample.
 *
 * This function fetches the latest sensor data, checks if the device is ready,
 * and logs the pressure value.
 */
static void pressure_sensor_process_sample(void)
{
    if (!device_is_ready(pressure_dev)) {
        LOG_ERR("sensor: %s device not ready.", pressure_dev->name);
        return;
    }

    if (sensor_sample_fetch(pressure_dev) < 0) {
        LOG_INF("sensor: %s sample update error", pressure_dev->name);
        return;
    }

    struct sensor_value pressure;
    if (sensor_channel_get(pressure_dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
        LOG_ERR("sensor: %s cannot read pressure channel", pressure_dev->name);
        return;
    }

    /* display pressure */
    LOG_INF("Pressure:%.1f kPa", sensor_value_to_double(&pressure));
}

/**
 * @brief Initialize the pressure sensor.
 *
 * This function checks if the pressure device is ready.
 *
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int pressure_sensor_init(void)
{
    if (!device_is_ready(pressure_dev)) {
        LOG_ERR("sensor: %s device not ready.", pressure_dev->name);
        return -ENODEV;
    }
    return 0;
}

SYS_INIT(pressure_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
