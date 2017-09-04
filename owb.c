/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
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
 * @file owb.c
 */

#include <stddef.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "owb.h"

static const char * TAG = "owb";

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

bool _is_init(const OneWireBus * bus)
{
    bool ok = false;
    if (bus != NULL)
    {
        if (bus->init)
        {
            // OK
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "bus is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "bus is NULL");
    }
    return ok;
}

/**
 * @brief Generate a 1-Wire reset (initialization).
 * @param[in] bus Initialised bus instance.
 * @return true if device is present, otherwise false.
 */
static bool _reset(const OneWireBus * bus)
{
    bool present = false;
    if (_is_init(bus))
    {
        // For some reason this crashes the ESP32
//        ESP_LOGD(TAG, "Enter critical region: reset");
//        portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
//        taskENTER_CRITICAL(&timeCriticalMutex);

        gpio_set_direction(bus->gpio, GPIO_MODE_OUTPUT);
        _us_delay(bus->timing->G);
        gpio_set_level(bus->gpio, 0);  // Drive DQ low
        _us_delay(bus->timing->H);
        gpio_set_level(bus->gpio, 1);  // Release the bus
        _us_delay(bus->timing->I);

        gpio_set_direction(bus->gpio, GPIO_MODE_INPUT);
        int level1 = gpio_get_level(bus->gpio);
        _us_delay(bus->timing->J);   // Complete the reset sequence recovery
        int level2 = gpio_get_level(bus->gpio);

//        taskEXIT_CRITICAL(&timeCriticalMutex);
//        ESP_LOGD(TAG, "Exit critical region: reset");

        present = (level1 == 0) && (level2 == 1);   // Sample for presence pulse from slave
        ESP_LOGD(TAG, "reset: level1 0x%x, level2 0x%x, present %d", level1, level2, present);
    }
    return present;
}

/**
 * @brief Send a 1-Wire write bit, with recovery time.
 * @param[in] bus Initialised bus instance.
 * @param[in] bit The value to send.
 */
static void _write_bit(const OneWireBus * bus, int bit)
{
    if (_is_init(bus))
    {
        int delay1 = bit ? bus->timing->A : bus->timing->C;
        int delay2 = bit ? bus->timing->B : bus->timing->D;

//        ESP_LOGD(TAG, "Enter critical region: write_bit");
//        portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
//        taskENTER_CRITICAL(&timeCriticalMutex);

        gpio_set_direction(bus->gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(bus->gpio, 0);  // Drive DQ low
        _us_delay(delay1);
        gpio_set_level(bus->gpio, 1);  // Release the bus
        _us_delay(delay2);

//        taskEXIT_CRITICAL(&timeCriticalMutex);
//        ESP_LOGD(TAG, "Exit critical region: write_bit");
    }
}

/**
 * @brief Read a bit from the 1-Wire bus and return the value, with recovery time.
 * @param[in] bus Initialised bus instance.
 */
static int _read_bit(const OneWireBus * bus)
{
    int result = 0;
    if (_is_init(bus))
    {
//        portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
//        taskENTER_CRITICAL(&timeCriticalMutex);

        ESP_LOGD(TAG, "Enter critical region: read_bit");
        gpio_set_direction(bus->gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(bus->gpio, 0);  // Drive DQ low
        _us_delay(bus->timing->A);
        gpio_set_level(bus->gpio, 1);  // Release the bus
        _us_delay(bus->timing->E);

        gpio_set_direction(bus->gpio, GPIO_MODE_INPUT);
        int level = gpio_get_level(bus->gpio);
        _us_delay(bus->timing->F);   // Complete the timeslot and 10us recovery

//        taskEXIT_CRITICAL(&timeCriticalMutex);
//        ESP_LOGD(TAG, "Exit critical region: read_bit");

        result = level & 0x01;
    }
    return result;
}

/**
 * @brief Write 1-Wire data byte.
 * @param[in] bus Initialised bus instance.
 * @param[in] data Value to write.
 */
static void _write_byte(const OneWireBus * bus, uint8_t data)
{
    if (_is_init(bus))
    {
        ESP_LOGD(TAG, "write 0x%02x", data);
        for (int i = 0; i < 8; ++i)
        {
            _write_bit(bus, data & 0x01);
            data >>= 1;
        }
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
    if (_is_init(bus))
    {
        for (int i = 0; i < 8; ++i)
        {
            result >>= 1;
            if (_read_bit(bus))
            {
                result |= 0x80;
            }
        }
        ESP_LOGD(TAG, "read 0x%02x", result);
    }
    return result;
}

/**
 * @param Read a block of bytes from 1-Wire bus.
 * @param[in] bus Initialised bus instance.
 * @param[in,out] buffer Pointer to buffer to receive read data.
 * @param[in] len Number of bytes to read, must not exceed length of receive buffer.
 * @return Pointer to receive buffer.
 */
static uint8_t * _read_block(const OneWireBus * bus, uint8_t * buffer, unsigned int len)
{
    for (int i = 0; i < len; ++i)
    {
        *buffer++ = _read_byte(bus);
    }
    return buffer;
}

/**
 * @param Write a block of bytes from 1-Wire bus.
 * @param[in] bus Initialised bus instance.
 * @param[in] buffer Pointer to buffer to write data from.
 * @param[in] len Number of bytes to write.
 * @return Pointer to write buffer.
 */
static const uint8_t * _write_block(const OneWireBus * bus, const uint8_t * buffer, unsigned int len)
{
    for (int i = 0; i < len; ++i)
    {
        _write_byte(bus, buffer[i]);
    }
    return buffer;
}

/**
 * @brief 1-Wire 8-bit CRC lookup.
 * @param[in] crc Starting CRC value. Pass in prior CRC to accumulate.
 * @param[in] data Byte to feed into CRC.
 * @return Resultant CRC value.
 */
static uint8_t _calc_crc(uint8_t crc, uint8_t data)
{
    // https://www.maximintegrated.com/en/app-notes/index.mvp/id/27
    static const uint8_t table[256] = {
            0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
            157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
            35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
            190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
            70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
            219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
            101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
            248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
            140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
            17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
            175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
            50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
            202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
            87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
            233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
            116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
    };

    return table[crc ^ data];
}

static uint8_t _calc_crc_block(uint8_t crc, const uint8_t * buffer, size_t len)
{
    do
    {
        crc = _calc_crc(crc, *buffer++);
        ESP_LOGD(TAG, "crc 0x%02x, len %d", crc, len);
    }
    while (--len > 0);
    return crc;
}

static bool _search(const OneWireBus * bus, OneWireBus_SearchState * state)
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

    if (_is_init(bus))
    {
        // if the last call was not the last one
        if (!state->last_device_flag)
        {
            // 1-Wire reset
            if (!_reset(bus))
            {
                // reset the search
                state->last_discrepancy = 0;
                state->last_device_flag = false;
                state->last_family_discrepancy = 0;
                return false;
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
                        crc8 = _calc_crc(crc8, state->rom_code.bytes[rom_byte_number]);  // accumulate the CRC
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
    }
    return search_result;
}


// Public API

OneWireBus * owb_malloc()
{
    OneWireBus * bus = malloc(sizeof(*bus));
    if (bus != NULL)
    {
        memset(bus, 0, sizeof(*bus));
        ESP_LOGD(TAG, "malloc %p", bus);
    }
    else
    {
        ESP_LOGE(TAG, "malloc failed");
    }
    return bus;
}

void owb_free(OneWireBus ** bus)
{
    if (bus != NULL && (*bus != NULL))
    {
        ESP_LOGD(TAG, "free %p", *bus);
        free(*bus);
        *bus = NULL;
    }
}

void owb_init(OneWireBus * bus, int gpio)
{
    if (bus != NULL)
    {
        bus->gpio = gpio;
        bus->timing = &_StandardTiming;
        bus->init = true;

        // platform specific:
        gpio_pad_select_gpio(bus->gpio);
    }
    else
    {
        ESP_LOGE(TAG, "bus is NULL");
    }
}

void owb_use_crc(OneWireBus * bus, bool use_crc)
{
    if (_is_init(bus))
    {
        bus->use_crc = use_crc;
        ESP_LOGD(TAG, "use_crc %d", bus->use_crc);
    }
}

OneWireBus_ROMCode owb_read_rom(const OneWireBus * bus)
{
    OneWireBus_ROMCode rom_code = {0};
    if (_is_init(bus))
    {
        if (_reset(bus))
        {
            _write_byte(bus, OWB_ROM_READ);
            _read_block(bus, rom_code.bytes, sizeof(rom_code));

            if (bus->use_crc)
            {
                if (owb_crc8_bytes(0, rom_code.bytes, sizeof(rom_code)) != 0)
                {
                    ESP_LOGE(TAG, "CRC failed");
                    memset(rom_code.bytes, 0, sizeof(rom_code));
                }
            }
            char rom_code_s[17];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            ESP_LOGD(TAG, "rom_code %s", rom_code_s);
        }
        else
        {
            ESP_LOGE(TAG, "ds18b20 device not responding");
        }
    }
    return rom_code;
}

bool owb_verify_rom(const OneWireBus * bus, OneWireBus_ROMCode rom_code)
{
    bool result = false;
    if (_is_init(bus))
    {
        OneWireBus_SearchState state = {
            .last_discrepancy = 64,
            .last_device_flag = false,
        };

        if (_search(bus, &state))
        {
            result = true;
            for (int i = 0; i < sizeof(state.rom_code.bytes) && result; ++i)
            {
                result = rom_code.bytes[i] == state.rom_code.bytes[i];
                ESP_LOGD(TAG, "%02x %02x", rom_code.bytes[i], state.rom_code.bytes[i]);
            }
            ESP_LOGD(TAG, "rom code %sfound", result ? "" : "not ");
        }
    }
    return result;
}

bool owb_reset(const OneWireBus * bus)
{
    return _reset(bus);
}

void owb_write_byte(const OneWireBus * bus, uint8_t data)
{
    _write_byte(bus, data);
}

uint8_t owb_read_byte(const OneWireBus * bus)
{
    return _read_byte(bus);
}

uint8_t * owb_read_bytes(const OneWireBus * bus, uint8_t * buffer, unsigned int len)
{
    return _read_block(bus, buffer, len);
}

const uint8_t * owb_write_bytes(const OneWireBus * bus, const uint8_t * buffer, unsigned int len)
{
    return _write_block(bus, buffer, len);
}

void owb_write_rom_code(const OneWireBus * bus, OneWireBus_ROMCode rom_code)
{
    _write_block(bus, (uint8_t *)&rom_code, sizeof(rom_code));
}

uint8_t owb_crc8_byte(uint8_t crc, uint8_t data)
{
    return _calc_crc(crc, data);
}

uint8_t owb_crc8_bytes(uint8_t crc, const uint8_t * data, size_t len)
{
    return _calc_crc_block(crc, data, len);
}

bool owb_search_first(const OneWireBus * bus, OneWireBus_SearchState * state)
{
    bool result = false;
    if (state != NULL)
    {
        memset(&state->rom_code, 0, sizeof(state->rom_code));
        state->last_discrepancy = 0;
        state->last_family_discrepancy = 0;
        state->last_device_flag = false;
        result = _search(bus, state);
    }
    else
    {
        ESP_LOGE(TAG, "state is NULL");
    }
    return result;
}

bool owb_search_next(const OneWireBus * bus, OneWireBus_SearchState * state)
{
    bool result = false;
    if (state != NULL)
    {
        result = _search(bus, state);
    }
    else
    {
        ESP_LOGE(TAG, "state is NULL");
    }
    return result;
}

char * owb_string_from_rom_code(OneWireBus_ROMCode rom_code, char * buffer, size_t len)
{
    for (int i = sizeof(rom_code.bytes) - 1; i >= 0; i--)
    {
        sprintf(buffer, "%02x", rom_code.bytes[i]);
        buffer += 2;
    }
    return buffer;
}
