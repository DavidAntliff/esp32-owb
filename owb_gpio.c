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

    gpio_set_direction(bus->gpio, GPIO_MODE_OUTPUT);
    _us_delay(bus->timing->G);
    gpio_set_level(bus->gpio, 0);  // Drive DQ low
    _us_delay(bus->timing->H);
    gpio_set_direction(bus->gpio, GPIO_MODE_INPUT); // Release the bus
    gpio_set_level(bus->gpio, 1);  // Reset the output level for the next output
    _us_delay(bus->timing->I);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 1);
#endif

    int level1 = gpio_get_level(bus->gpio);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 0);
#endif

    _us_delay(bus->timing->J);   // Complete the reset sequence recovery

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 1);
#endif

    int level2 = gpio_get_level(bus->gpio);

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

    portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&timeCriticalMutex);

    gpio_set_direction(bus->gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(bus->gpio, 0);  // Drive DQ low
    _us_delay(delay1);
    gpio_set_level(bus->gpio, 1);  // Release the bus
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

    portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&timeCriticalMutex);

    gpio_set_direction(bus->gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(bus->gpio, 0);  // Drive DQ low
    _us_delay(bus->timing->A);
    gpio_set_direction(bus->gpio, GPIO_MODE_INPUT); // Release the bus
    gpio_set_level(bus->gpio, 1);  // Reset the output level for the next output
    _us_delay(bus->timing->E);

#ifdef PHY_DEBUG
    gpio_set_level(PHY_DEBUG_GPIO, 1);
#endif

    int level = gpio_get_level(bus->gpio);

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
 * @param[in] bus Initialised bus instance.
 * @param[in] data Value to write.
 */
static void _write_byte(const OneWireBus * bus, uint8_t data)
{
    ESP_LOGD(TAG, "write 0x%02x", data);
    for (int i = 0; i < 8; ++i)
    {
        _write_bit(bus, data & 0x01);
        data >>= 1;
    }
}

/**
 * @brief Read 1-Wire data byte from  bus.
 * @param[in] bus Initialised bus instance.
 * @return Byte value read from bus.
 */
static uint8_t _read_byte(const OneWireBus * bus)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; ++i)
    {
        result >>= 1;
        if (_read_bit(bus))
        {
            result |= 0x80;
        }
    }
    ESP_LOGD(TAG, "read 0x%02x", result);
    return result;
}

/**
 * @param Read a block of bytes from 1-Wire bus.
 * @param[in] bus Initialised bus instance.
 * @param[in,out] buffer Pointer to buffer to receive read data.
 * @param[in] len Number of bytes to read, must not exceed length of receive buffer.
 * @return status
 */
static owb_status _read_block(const OneWireBus * bus, uint8_t * buffer, size_t len)
{
    for (int i = 0; i < len; ++i)
    {
        *buffer++ = _read_byte(bus);
    }

    return OWB_STATUS_OK;
}

/**
 * @param Write a block of bytes from 1-Wire bus.
 * @param[in] bus Initialised bus instance.
 * @param[in] buffer Pointer to buffer to write data from.
 * @param[in] len Number of bytes to write.
 * @return status
 */
static owb_status _write_block(const OneWireBus * bus, const uint8_t * buffer, size_t len)
{
    for (int i = 0; i < len; ++i)
    {
        _write_byte(bus, buffer[i]);
    }

    return OWB_STATUS_OK;
}

/* @param[out] is_found true if a device was found, false if not
 * @return status
 */
static owb_status _search(const OneWireBus * bus, OneWireBus_SearchState * state, bool *is_found)
{
    // Based on https://www.maximintegrated.com/en/app-notes/index.mvp/id/187

    // initialize for search
    int id_bit_number = 1;
    int last_zero = 0;
    int rom_byte_number = 0;
    int id_bit = 0;
    int cmp_id_bit = 0;
    uint8_t rom_byte_mask = 1;
    uint8_t search_direction = 0;
    bool search_result = false;
    uint8_t crc8 = 0;
    owb_status status;

    // if the last call was not the last one
    if (!state->last_device_flag)
    {
        // 1-Wire reset
        bool is_present;
        _reset(bus, &is_present);
        if (!is_present)
        {
            // reset the search
            state->last_discrepancy = 0;
            state->last_device_flag = false;
            state->last_family_discrepancy = 0;
            *is_found = false;
            return OWB_STATUS_OK;
        }

        // issue the search command
        _write_byte(bus, OWB_ROM_SEARCH);

        // loop to do the search
        do
        {
            // read a bit and its complement
            id_bit = _read_bit(bus);
            cmp_id_bit = _read_bit(bus);

            // check for no devices on 1-wire
            if ((id_bit == 1) && (cmp_id_bit == 1))
                break;
            else
            {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit)
                    search_direction = id_bit;  // bit write value for search
                else
                {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < state->last_discrepancy)
                        search_direction = ((state->rom_code.bytes[rom_byte_number] & rom_byte_mask) > 0);
                    else
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == state->last_discrepancy);

                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0)
                    {
                        last_zero = id_bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9)
                            state->last_family_discrepancy = last_zero;
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                    state->rom_code.bytes[rom_byte_number] |= rom_byte_mask;
                else
                    state->rom_code.bytes[rom_byte_number] &= ~rom_byte_mask;

                // serial number search direction write bit
                _write_bit(bus, search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    crc8 = owb_crc8_byte(crc8, state->rom_code.bytes[rom_byte_number]);  // accumulate the CRC
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!((id_bit_number < 65) || (crc8 != 0)))
        {
            // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
            state->last_discrepancy = last_zero;

            // check for last device
            if (state->last_discrepancy == 0)
                state->last_device_flag = true;

            search_result = true;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !state->rom_code.bytes[0])
    {
        state->last_discrepancy = 0;
        state->last_device_flag = false;
        state->last_family_discrepancy = 0;
        search_result = false;
    }

    status = OWB_STATUS_OK;

    *is_found = search_result;

    return status;
}

owb_status _init(OneWireBus * bus, int gpio)
{
    owb_status status;

    if (bus != NULL)
    {
        bus->gpio = gpio;
        bus->timing = &_StandardTiming;

        // platform specific:
        gpio_pad_select_gpio(bus->gpio);

#ifdef PHY_DEBUG
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = PHY_DEBUG_GPIO_MASK;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
#endif

        status = OWB_STATUS_OK;
    }
    else
    {
        ESP_LOGE(TAG, "bus is NULL");
        status = OWB_STATUS_ERR;
    }

    return status;
}

static struct owb_driver gpio_driver =
{
    .name = "owb_gpio",
    .init = _init,
    .search = _search,
    .reset = _reset,
    .write_bytes = _write_block,
    .read_bytes = _read_block
};

const struct owb_driver* owb_gpio_get_driver()
{
    return &gpio_driver;
}
