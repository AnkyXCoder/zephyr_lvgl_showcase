/**
 * @file main.c
 * @author Ankit Modi (ankit.5036@gmail.com)
 * @brief Entry point for the Zephyr LVGL Showcase Application.
 *
 * This file contains the main function for the Zephyr LVGL Showcase Application.
 * It initializes the logging module and prints a startup message indicating
 * the board on which the application is running.
 *
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

#include <zephyr/logging/log.h>

//------------------------------------------------------------------------------
// Private Variables
//------------------------------------------------------------------------------

LOG_MODULE_REGISTER(main);

//------------------------------------------------------------------------------
// Main Entry Function
//------------------------------------------------------------------------------

/**
 * @brief Main entry point of the application.
 * @return int Returns 0 upon successful execution.
 */

int main(void)
{
    LOG_INF("Zephyr LVGL Showcase Application Started on %s.", CONFIG_BOARD);
    return 0;
}
