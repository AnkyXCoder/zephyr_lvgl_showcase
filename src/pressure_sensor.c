/**
 * @file pressure_sensor.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief Pressure sensor interface.
 *
 * This file implements initialization, sampling, and logging of pressure sensor data
 * using Zephyr's sensor API. It creates a thread to periodically read and display
 * pressure values from the configured sensor device.
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
// #define Constants/Macros
//------------------------------------------------------------------------------
#define PS_THREAD_STACK_SIZE 512
#define PS_THREAD_PRIORITY   5

#define PS_SAMPLE_DELAY 10 // 10 Seconds

#if DT_NODE_EXISTS(DT_ALIAS(pressure_sensor))
#define PRESSURE_NODE DT_ALIAS(pressure_sensor)
const struct device *const pressure_dev = DEVICE_DT_GET(DT_ALIAS(pressure_sensor));
#else
#error("Pressure sensor not found.");
#endif

//------------------------------------------------------------------------------
// typedef/structs/enums
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(pressure);

K_THREAD_STACK_DEFINE(ps_stack_area, PS_THREAD_STACK_SIZE);

static struct k_thread ps_thread;
static k_tid_t ps_tid;

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void ps_sensor_thread(void *arg0, void *arg1, void *arg2);
static int pressure_sensor_process_sample(double *pressure);
static int pressure_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/* Pressure Sensor Thread Function */
static void ps_sensor_thread(void *arg0, void *arg1, void *arg2)
{
    while (1) {
        double pressure;
        if (pressure_sensor_process_sample(&pressure) < 0) {
            return;
        }

        /* display pressure */
        LOG_INF("Pressure:%.3f kPa", pressure);

        k_sleep(K_SECONDS(PS_SAMPLE_DELAY));
    }
}

/**
 * @brief Process and log the pressure sensor sample.
 *
 * This function fetches the latest sensor data, checks if the device is ready,
 * and logs the pressure value.
 *
 * @param pressure Pointer to store the measured pressure value.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int pressure_sensor_process_sample(double *pressure)
{
    if (!device_is_ready(pressure_dev)) {
        LOG_ERR("sensor: %s device not ready.", pressure_dev->name);
        return -ENODEV;
    }

    if (sensor_sample_fetch(pressure_dev) < 0) {
        LOG_INF("sensor: %s sample update error", pressure_dev->name);
        return -EIO;
    }

    struct sensor_value pressure_val, temp_val;
    if (sensor_channel_get(pressure_dev, SENSOR_CHAN_PRESS, &pressure_val) < 0) {
        LOG_ERR("sensor: %s cannot read pressure channel", pressure_dev->name);
        return -EINVAL;
    }

    *pressure = sensor_value_to_double(&pressure_val);
    if (sensor_channel_get(pressure_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val) < 0) {
        printf("Cannot read LPS22HH temperature channel\n");
        return -EINVAL;
    }

    return 0;
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

    ps_tid = k_thread_create(&ps_thread, ps_stack_area, K_THREAD_STACK_SIZEOF(ps_stack_area), ps_sensor_thread, NULL,
                             NULL, NULL, PS_THREAD_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Pressure sensor %s initialized.", pressure_dev->name);
    return 0;
}

SYS_INIT(pressure_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
