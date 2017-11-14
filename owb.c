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

static const char * TAG = "owb";

static bool _is_init(const OneWireBus * bus)
{
    bool ok = false;
    if (bus != NULL)
    {
        if (bus->driver)
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
        ESP_LOGD(TAG, "crc 0x%02x, len %d", (int)crc, (int)len);
    }
    while (--len > 0);
    return crc;
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

owb_status owb_init(OneWireBus * bus, int gpio)
{
    owb_status status;

    // default to the gpio driver for now
    bus->driver = owb_gpio_get_driver();

    status = bus->driver->init(bus, gpio);

    return status;
}

owb_status owb_use_crc(OneWireBus * bus, bool use_crc)
{
    owb_status status;

    if (_is_init(bus))
    {
        bus->use_crc = use_crc;
        ESP_LOGD(TAG, "use_crc %d", bus->use_crc);

        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_read_rom(const OneWireBus * bus, OneWireBus_ROMCode *rom_code)
{
    owb_status status = OWB_STATUS_ERR;

    memset(rom_code, 0, sizeof(OneWireBus_ROMCode));

    if (_is_init(bus))
    {
        bool is_present;
        bus->driver->reset(bus, &is_present);
        if (is_present)
        {
            uint8_t value = OWB_ROM_READ;
            bus->driver->write_bytes(bus, &value, sizeof(value));
            bus->driver->read_bytes(bus, rom_code->bytes, sizeof(OneWireBus_ROMCode));

            if (bus->use_crc)
            {
                if (owb_crc8_bytes(0, rom_code->bytes, sizeof(OneWireBus_ROMCode)) != 0)
                {
                    ESP_LOGE(TAG, "CRC failed");
                    memset(rom_code->bytes, 0, sizeof(OneWireBus_ROMCode));
                } else
                {
                    status = OWB_STATUS_OK;
                }
            } else
            {
                status = OWB_STATUS_OK;
            }
            char rom_code_s[17];
            owb_string_from_rom_code(*rom_code, rom_code_s, sizeof(rom_code_s));
            ESP_LOGD(TAG, "rom_code %s", rom_code_s);
        }
        else
        {
            ESP_LOGE(TAG, "ds18b20 device not responding");
        }
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_verify_rom(const OneWireBus * bus, OneWireBus_ROMCode rom_code, bool* is_present)
{
    owb_status status;
    bool result = false;
    if (_is_init(bus))
    {
        OneWireBus_SearchState state = {
            .last_discrepancy = 64,
            .last_device_flag = false,
        };

        bool is_found;
        bus->driver->search(bus, &state, &is_found);
        if (is_found)
        {
            result = true;
            for (int i = 0; i < sizeof(state.rom_code.bytes) && result; ++i)
            {
                result = rom_code.bytes[i] == state.rom_code.bytes[i];
                ESP_LOGD(TAG, "%02x %02x", rom_code.bytes[i], state.rom_code.bytes[i]);
            }
            ESP_LOGD(TAG, "rom code %sfound", result ? "" : "not ");
        }

        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    *is_present = result;

    return status;
}

owb_status owb_reset(const OneWireBus * bus, bool* a_device_present)
{
    owb_status status;

    if (_is_init(bus))
    {
        bus->driver->reset(bus, a_device_present);
        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_write_byte(const OneWireBus * bus, uint8_t data)
{
    owb_status status;

    if (_is_init(bus))
    {
        bus->driver->write_bytes(bus, &data, sizeof(data));
        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_read_byte(const OneWireBus * bus, uint8_t *out)
{
    owb_status status;

    if (_is_init(bus))
    {
        bus->driver->read_bytes(bus, out, sizeof(uint8_t));
        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_read_bytes(const OneWireBus * bus, uint8_t * buffer, unsigned int len)
{
    owb_status status;

    if (_is_init(bus))
    {
        bus->driver->read_bytes(bus, buffer, len);
        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_write_bytes(const OneWireBus * bus, const uint8_t * buffer, unsigned int len)
{
    owb_status status;

    if (_is_init(bus))
    {
        bus->driver->write_bytes(bus, buffer, len);
        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_write_rom_code(const OneWireBus * bus, OneWireBus_ROMCode rom_code)
{
    owb_status status;

    if (_is_init(bus))
    {
        bus->driver->write_bytes(bus, (uint8_t *)&rom_code, sizeof(rom_code));
        status = OWB_STATUS_OK;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

uint8_t owb_crc8_byte(uint8_t crc, uint8_t data)
{
    return _calc_crc(crc, data);
}

uint8_t owb_crc8_bytes(uint8_t crc, const uint8_t * data, size_t len)
{
    return _calc_crc_block(crc, data, len);
}

owb_status owb_search_first(const OneWireBus * bus, OneWireBus_SearchState * state, bool* found_device)
{
    owb_status status;
    bool result = false;

    if (_is_init(bus))
    {
        if (state != NULL)
        {
            memset(&state->rom_code, 0, sizeof(state->rom_code));
            state->last_discrepancy = 0;
            state->last_family_discrepancy = 0;
            state->last_device_flag = false;
            bus->driver->search(bus, state, &result);
            status = OWB_STATUS_OK;
        }
        else
        {
            ESP_LOGE(TAG, "state is NULL");
            status = OWB_STATUS_ERR;
        }
        *found_device = result;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
}

owb_status owb_search_next(const OneWireBus * bus, OneWireBus_SearchState * state, bool* found_device)
{
    owb_status status;
    bool result = false;

    if (_is_init(bus))
    {
        if (state != NULL)
        {
            bus->driver->search(bus, state, &result);
            status = OWB_STATUS_OK;
        }
        else
        {
            ESP_LOGE(TAG, "state is NULL");
            status = OWB_STATUS_ERR;
        }
        *found_device = result;
    } else
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }

    return status;
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
