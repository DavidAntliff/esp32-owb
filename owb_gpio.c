/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 * Copyright (c) 2017 Chris Morgan <chmorgan@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file
 */

#include <stddef.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "owb.h"
#include "owb_gpio.h"

static const char * TAG = "owb_gpio";

// Define PHY_DEBUG to enable GPIO output around when the bus is sampled
// by the master (this library). This GPIO output makes it possible to
// validate the master's sampling using an oscilloscope.
//
// For the debug GPIO the idle state is low and made high before the 1-wire sample
// point and low again after the sample point
#undef PHY_DEBUG

#ifdef PHY_DEBUG
// Update these defines to a pin that you can access
#define PHY_DEBUG_GPIO GPIO_NUM_27
#define PHY_DEBUG_GPIO_MASK GPIO_SEL_27
#endif

/// @cond ignore
struct _OneWireBus_Timing
{
    uint32_t A, B, C, D, E, F, G, H, I, J;
};
//// @endcond

// 1-Wire timing delays (standard) in microseconds.
// Labels and values are from https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
static const struct _OneWireBus_Timing _StandardTiming = {
        6,    // A - read/write "1" master pull DQ low duration
        64,   // B - write "0" master pull DQ low duration
        60,   // C - write "1" master pull DQ high duration
        10,   // D - write "0" master pull DQ high duration
        9,    // E - read master pull DQ high duration
        55,   // F - complete read timeslot + 10ms recovery
        0,    // G - wait before reset
        480,  // H - master pull DQ low duration
        70,   // I - master pull DQ high duration
        410,  // J - complete presence timeslot + recovery
};

static void _us_delay(uint32_t time_us)
{
    ets_delay_us(time_us);
}

#define info_from_bus(owb) container_of(owb, owb_gpio_driver_info, bus)

/**
 * @brief Generate a 1-Wire reset (initialization).
 * @param[in] bus Initialised bus instance.
 * @param[out] is_present true if device is present, otherwise false.
 * @return status
 */
static owb_status _reset(const OneWireBus * bus, bool * is_present)
{
    bool present = false;
    portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&timeCriticalMutex);

    owb_gpio_driver_info *i = info_from_bus(bus);

    gpio_set_direction(i->gpio, GPIO_MODE_OUTPUT);
    _us_delay(bus->timing->G);
    gpio_set_level(i->gpio, 0);  // Drive DQ low
    _us_delay(bus->timing->H);
    gpio_set_direction(i->gpio, GPIO_MODE_INPUT); // Release the bus
    gpio_set_level(i->gpio, 1);  // Reset the output level for the next output
    _us_delay(bus->timing->I);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 1);
#endif

    int level1 = gpio_get_level(i->gpio);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 0);
#endif

    _us_delay(bus->timing->J);   // Complete the reset sequence recovery

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 1);
#endif

    int level2 = gpio_get_level(i->gpio);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 0);
#endif

    taskEXIT_CRITICAL(&timeCriticalMutex);

    present = (level1 == 0) && (level2 == 1);   // Sample for presence pulse from slave
    ESP_LOGD(TAG, "reset: level1 0x%x, level2 0x%x, present %d", level1, level2, present);

    *is_present = present;

    return OWB_STATUS_OK;
}

/**
 * @brief Send a 1-Wire write bit, with recovery time.
 * @param[in] bus Initialised bus instance.
 * @param[in] bit The value to send.
 */
static void _write_bit(const OneWireBus * bus, int bit)
{
    int delay1 = bit ? bus->timing->A : bus->timing->C;
    int delay2 = bit ? bus->timing->B : bus->timing->D;
    owb_gpio_driver_info *i = info_from_bus(bus);

    portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&timeCriticalMutex);

    gpio_set_direction(i->gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(i->gpio, 0);  // Drive DQ low
    _us_delay(delay1);
    gpio_set_level(i->gpio, 1);  // Release the bus
    _us_delay(delay2);

    taskEXIT_CRITICAL(&timeCriticalMutex);
}

/**
 * @brief Read a bit from the 1-Wire bus and return the value, with recovery time.
 * @param[in] bus Initialised bus instance.
 */
static int _read_bit(const OneWireBus * bus)
{
    int result = 0;
    owb_gpio_driver_info *i = info_from_bus(bus);

    portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&timeCriticalMutex);

    gpio_set_direction(i->gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(i->gpio, 0);  // Drive DQ low
    _us_delay(bus->timing->A);
    gpio_set_direction(i->gpio, GPIO_MODE_INPUT); // Release the bus
    gpio_set_level(i->gpio, 1);  // Reset the output level for the next output
    _us_delay(bus->timing->E);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 1);
#endif

    int level = gpio_get_level(i->gpio);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 0);
#endif

    _us_delay(bus->timing->F);   // Complete the timeslot and 10us recovery

    taskEXIT_CRITICAL(&timeCriticalMutex);

    result = level & 0x01;

    return result;
}

/**
 * @brief Write 1-Wire data byte.
 * NOTE: The data is shifted out of the low bits, eg. it is written in the order of lsb to msb
 * @param[in] bus Initialised bus instance.
 * @param[in] data Value to write.
 * @param[in] number_of_bits_to_read bits to write
 */
static owb_status _write_bits(const OneWireBus * bus, uint8_t data, int number_of_bits_to_write)
{
    ESP_LOGD(TAG, "write 0x%02x", data);
    for (int i = 0; i < number_of_bits_to_write; ++i)
    {
        _write_bit(bus, data & 0x01);
        data >>= 1;
    }

    return OWB_STATUS_OK;
}

/**
 * @brief Read 1-Wire data byte from  bus.
 * NOTE: Data is read into the high bits, eg. each bit read is shifted down before the next bit is read
 * @param[in] bus Initialised bus instance.
 * @return Byte value read from bus.
 */
static owb_status _read_bits(const OneWireBus * bus, uint8_t *out, int number_of_bits_to_read)
{
    uint8_t result = 0;
    for (int i = 0; i < number_of_bits_to_read; ++i)
    {
        result >>= 1;
        if (_read_bit(bus))
        {
            result |= 0x80;
        }
    }
    ESP_LOGD(TAG, "read 0x%02x", result);
    *out = result;

    return OWB_STATUS_OK;
}

static owb_status _uninitialize(const OneWireBus * bus)
{
    // Nothing to do here for this driver_info
    return OWB_STATUS_OK;
}

static const struct owb_driver gpio_function_table =
{
    .name = "owb_gpio",
    .uninitialize = _uninitialize,
    .reset = _reset,
    .write_bits = _write_bits,
    .read_bits = _read_bits
};

OneWireBus* owb_gpio_initialize(owb_gpio_driver_info *driver_info, int gpio)
{
    ESP_LOGI(TAG, "%s(): gpio %d\n", __func__, gpio);

    driver_info->gpio = gpio;
    driver_info->bus.driver = &gpio_function_table;
    driver_info->bus.timing = &_StandardTiming;

    // platform specific:
    gpio_pad_select_gpio(driver_info->gpio);

#ifdef PHY_DEBUG
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = PHY_DEBUG_GPIO_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
#endif

    return &(driver_info->bus);
}
