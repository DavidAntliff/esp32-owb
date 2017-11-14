#pragma once

typedef struct
{
    int gpio; ///< Value of the GPIO connected to the 1-Wire bus

    OneWireBus bus;
} owb_gpio_driver_info;

/**
 * @return OneWireBus*, pass this into the other OneWireBus public API functions
 */
OneWireBus* owb_gpio_initialize(owb_gpio_driver_info *driver_info, int gpio);

/**
 * @brief Clean up after a call to owb_gpio_initialize()
 */
void owb_gpio_uninitialize(owb_gpio_driver_info *driver_info);
