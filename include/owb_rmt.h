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
 * @brief Interface definitions for ESP32 RMT driver used to communicate with devices
 *        on the One Wire Bus.
 *
 * This is the recommended driver.
 */

#pragma once
#ifndef OWB_RMT_H
#define OWB_RMT_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#include "owb.h"  // for tyoe OneWireBus

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RMT driver information
 */
typedef struct
{
  rmt_channel_handle_t tx_channel_handle;     ///< RMT transmit channel, allocated by RMT driver
  rmt_channel_handle_t rx_channel_handle;     ///< RMT receive channel, allocated by RMT driver
  rmt_encoder_handle_t copy_encoder_handle;   ///< RMT built-in data encoder that sends pre-defined symbols
  rmt_encoder_handle_t bytes_encoder_handle;  ///< RMT built-in data encoder that converts bits into symbols
  rmt_symbol_word_t *rx_buffer;               ///< RMT receive channel symbol buffer
  size_t rx_buffer_size_in_bytes;             ///< Size of the RMT received buffer, in bytes
  QueueHandle_t rx_queue;                     ///< xQueue to receive RMT symbols from the `on_recv_done` callback
  int gpio;                                   ///< OneWireBus GPIO
OneWireBus bus;                               ///< OneWireBus instance (see owb.h)
} owb_rmt_driver_info;

/**
 * @brief Initialise the RMT driver.
 * @param[in] info Pointer to an uninitialized owb_rmt_driver_info structure.
 *                 Note: the structure must remain in scope for the lifetime of this component.
 * @param[in] gpio_num The GPIO number to use as the One Wire bus data line.
 * @param tx_channel NOW IGNORED: the new RMT driver allocates channels dynamically.
 * @param rx_channel NOW IGNORED: the new RMT driver allocates channels dynamically.
 * @return OneWireBus *, pass this into the other OneWireBus public API functions
 */
OneWireBus* owb_rmt_initialize(owb_rmt_driver_info * info, gpio_num_t gpio_num,
                               rmt_channel_t tx_channel, rmt_channel_t rx_channel);


/**
 * @brief Legacy RMT channel IDs.
 * 
 * These are no longer used for anything. They are defined here purely so
 * that code written for the old rmt driver can still compile.
 */
typedef enum {
    RMT_CHANNEL_0,  /*!< RMT channel number 0 */
    RMT_CHANNEL_1,  /*!< RMT channel number 1 */
    RMT_CHANNEL_2,  /*!< RMT channel number 2 */
    RMT_CHANNEL_3,  /*!< RMT channel number 3 */
#if SOC_RMT_CHANNELS_PER_GROUP > 4
    RMT_CHANNEL_4,  /*!< RMT channel number 4 */
    RMT_CHANNEL_5,  /*!< RMT channel number 5 */
    RMT_CHANNEL_6,  /*!< RMT channel number 6 */
    RMT_CHANNEL_7,  /*!< RMT channel number 7 */
#endif
    RMT_CHANNEL_MAX /*!< Number of RMT channels */
} rmt_channel_t;


#ifdef __cplusplus
}
#endif

#endif // OWB_RMT_H
