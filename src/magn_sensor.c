/**
 * @file magn_sensor.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief Magnetometer sensor interface.
 *
 * This file implements initialization, sampling, and logging of Magnetometer sensor data
 * using Zephyr's sensor API. It creates a thread for periodic sampling and logs the
 * sensor values for use in the application.
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
#define MAGN_THREAD_STACK_SIZE 768
#define MAGN_THREAD_PRIORITY   5

#define MAGN_SAMPLE_DELAY 1 // 1 Second

#if DT_NODE_EXISTS(DT_ALIAS(magn_sensor))
#define MAGN_NODE DT_ALIAS(magn_sensor)
static const struct device *const magn_dev = DEVICE_DT_GET(MAGN_NODE);
#else
#error("Magnetometer sensor not found.");
#endif

//------------------------------------------------------------------------------
// typedef/structs/enums
//------------------------------------------------------------------------------

typedef struct {
    double magn_x;
    double magn_y;
    double magn_z;
} magn_t;

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(magn);

K_THREAD_STACK_DEFINE(magn_stack_area, MAGN_THREAD_STACK_SIZE);

static struct k_thread magn_thread;
static k_tid_t magn_tid;

//------------------------------------------------------------------------------
// Private Function Prototypes
//------------------------------------------------------------------------------

static void magn_sensor_thread(void *arg0, void *arg1, void *arg2);
static int magn_sensor_sample_process(magn_t *magn_val);
static int magn_sensor_init(void);

//------------------------------------------------------------------------------
// Public Function Implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private Function Implementation
//------------------------------------------------------------------------------

/* IMU Sensor Thread Function */
static void magn_sensor_thread(void *arg0, void *arg1, void *arg2)
{
    while (1) {
        magn_t magn;

        if (magn_sensor_sample_process(&magn) < 0) {
            return;
        }

        LOG_INF("Magn (gauss): x:%.3f y:%.3f z:%.3f", magn.magn_x, magn.magn_y, magn.magn_z);

        k_sleep(K_SECONDS(MAGN_SAMPLE_DELAY));
    }
}
/**
 * @brief Fetch and process Magnetometer sensor sample.
 *
 * This function checks if the Magnetometer device is ready, fetches the latest sensor data,
 * reads magnetometer values, and stores them in the provided structures.
 *
 * @param magn_val Pointer to accel_t structure to store magnetometer values.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
static int magn_sensor_sample_process(magn_t *magn_val)
{
    if (!device_is_ready(magn_dev)) {
        LOG_ERR("sensor: %s device not ready.", magn_dev->name);
        return -ENODEV;
    }

    if (sensor_sample_fetch(magn_dev) < 0) {
        LOG_ERR("sensor: %s sample update error", magn_dev->name);
        return -EIO;
    }

    /* magnetometer */
    struct sensor_value magn[3];
    if (sensor_channel_get(magn_dev, SENSOR_CHAN_MAGN_XYZ, magn) < 0) {
        LOG_ERR("sensor: %s read magn channel failed", magn_dev->name);
        return -EINVAL;
    }
    magn_val->magn_x = sensor_value_to_double(&magn[0]);
    magn_val->magn_y = sensor_value_to_double(&magn[1]);
    magn_val->magn_z = sensor_value_to_double(&magn[2]);

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
static int magn_sensor_init(void)
{
    if (!device_is_ready(magn_dev)) {
        LOG_ERR("sensor: %s device not ready.", magn_dev->name);
        return -ENODEV;
    }

    /* set sampling frequency to 100 Hz */
    struct sensor_value odr_attr;
    odr_attr.val1 = 100;
    odr_attr.val2 = 0;

    if (sensor_attr_set(magn_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("sensor: %s cannot set sampling frequency for magn.", magn_dev->name);
        return -EINVAL;
    }

    magn_tid = k_thread_create(&magn_thread, magn_stack_area, K_THREAD_STACK_SIZEOF(magn_stack_area),
                               magn_sensor_thread, NULL, NULL, NULL, MAGN_THREAD_PRIORITY, 0, K_NO_WAIT);

    LOG_INF("Magnetometer sensor %s initialized.", magn_dev->name);
    return 0;
}

SYS_INIT(magn_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
