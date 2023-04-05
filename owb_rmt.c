/**
 * Copyright (c) 2023 mjcross
 *
 * SPDX-License-Identifier: MIT
**/

#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#include "owb.h"
#include "owb_rmt_bus_timings.h"
#include "owb_rmt_bus_symbols.h"

#define OWB_RMT_CLK_HZ 1000000              // run the RMT at 1MHz to get 1us ticks
#define OWB_RMT_TX_MEM_BLOCK_SYMBOLS 64     // size of TX memory block in units of rmt_symbol_word_t (must be even)
#define OWB_RMT_TX_QUEUE_DEPTH 4            // max pending TX transfers
#define OWB_RMT_MAX_READ_BITS 64            // maximum number of bits that will be read at once (used to calculate buffer size)
#define OWB_RMT_RX_MEM_BLOCK_SYMBOLS (OWB_RMT_MAX_READ_BITS + 2)     // size of RX memory block in units of rmt_symbol_word_t (must be even)
#define OWB_RMT_RX_MIN_NS 1000              // RMT receive channel glitch rejection threshold (ns)
#define OWB_RMT_TIMEOUT_MS 1000             // timeout threshold for an RMT task (ms)
#define OWB_TIMING_MARGIN 3                 // timing variation permitted by our event parsing functions (in microsec)

// debug parsing of RMT raw symbols
//#define OWB_RMT_DEBUG

// tag for log messages
static const char * TAG = "owb_rmt";


//------
// private API functions and constants
//------

// onewire bus symbols as rmt_symbol_word_t
static const rmt_symbol_word_t owb_rmt_symbol_0bit = OWB_RMT_SYMBOL_0BIT;
static const rmt_symbol_word_t owb_rmt_symbol_1bit = OWB_RMT_SYMBOL_1BIT;
static const rmt_symbol_word_t owb_rmt_symbol_reset = OWB_RMT_SYMBOL_RESET;

// RMT transmit configuration for the OWB: transmit symbols once then release the bus
static const rmt_transmit_config_t owb_rmt_transmit_config = {
    .loop_count = 0,                        // don't send any repeats
    .flags = {
        .eot_level = OWB_RMT_BUS_RELEASED   // release the bus after the transmission
    }
};

// RMT receiver configuration for a onewire reset pulse
static const rmt_receive_config_t rx_config_owb_reset = {
    .signal_range_min_ns = OWB_RMT_RX_MIN_NS,                                   // glitch rejection threshold (ns)
    .signal_range_max_ns = (OWB_TIMING_PARAM_H + OWB_TIMING_PARAM_I) * 1000     // stop condition (ns)
};

// RMT receiver configuration for a sequence of onewire data bits
static const rmt_receive_config_t rx_config_owb_bits = {
    .signal_range_min_ns = OWB_RMT_RX_MIN_NS,                                   // glitch rejection threshold (ns)
    .signal_range_max_ns = (OWB_TIMING_PARAM_A + OWB_TIMING_PARAM_B) * 1000     // stop condition (ns)
};


/**
 * @brief Uninstalls a onewire bus driver and releases the associated resources.
 * @param bus A previously-initialised OneWireBus.
 * @return owb_status OWB_STATUS_OK on success, otherwise an error code (see owb.h)
 */
static owb_status _uninitialize(const OneWireBus *bus) {

    // fetch the parent `owb_rmt_driver_info` structure for `bus`
    owb_rmt_driver_info *info = __containerof(bus, owb_rmt_driver_info, bus);    // (pointer, type, member)
    if (info == NULL) {
        ESP_LOGE(TAG, "err uninitialize: no bus container");
        return OWB_STATUS_PARAMETER_NULL;
    }

    // release RMT device symbol buffer and queue
    free (info->rx_buffer);
    vQueueDelete (info->rx_queue);

    // disable and release RMT resources
    if (rmt_disable (info->rx_channel_handle) == ESP_OK &&
        rmt_del_channel (info->rx_channel_handle) == ESP_OK &&
        rmt_disable (info->tx_channel_handle) == ESP_OK &&
        rmt_del_channel (info->tx_channel_handle) == ESP_OK &&
        rmt_del_encoder (info->copy_encoder_handle) == ESP_OK &&
        rmt_del_encoder (info->bytes_encoder_handle) == ESP_OK ) {
            // all resources successfully released
            return OWB_STATUS_OK;
        }

    // an error occurred
    ESP_LOGE(TAG, "err uninitializing");
    return OWB_STATUS_HW_ERROR;
}


/**
 * @brief Parses the RMT symbols received during a onewire bus reset.
 * @param[in] num_symbols The number of symbols passed.
 * @param[in] symbols An array of RMT symbols.
 * @param[out] slave_is_present Whether a slave presence signal was detected.
 * @return OWB_STATUS_OK if the symbols pass basic valdation; otherwise an error code (see owb.h).
 */
static owb_status _parse_reset_symbols (size_t num_symbols, rmt_symbol_word_t *symbols, bool *slave_is_present) {
    *slave_is_present = false;

    if (num_symbols == 0 || symbols == NULL) {
        return OWB_STATUS_PARAMETER_NULL;
    }

    #ifdef OWB_RMT_DEBUG
    // display raw RMT symbols
    ESP_LOGI(TAG, "parse reset: %d symbols", (int)num_symbols);
    for (int i = 0; i < num_symbols; i += 1) {
        ESP_LOGI (TAG, "\t%u (%uus), %u (%uus)", symbols->level0, symbols->duration0, 
        symbols->level1, symbols->duration1);
    }
    #endif

    // check the duration of the reset pulse
    if (abs (symbols[0].duration0 - OWB_TIMING_PARAM_H) > OWB_TIMING_MARGIN) {
        return OWB_STATUS_HW_ERROR;
    }

    // check for a valid 'no slave' event
    if (num_symbols == 1 && symbols[0].duration1 == 0) {
            *slave_is_present = false;
            return OWB_STATUS_OK;
    }

    // check for a valid 'slave present' event
    if (num_symbols == 2 &&                                                     // no 'extra' symbols after the presence pulse
        symbols[0].duration1 < OWB_TIMING_PARAM_I &&                            // presence pulse must arrive before the sample point
        (symbols[1].duration0 + symbols[0].duration1) >= OWB_TIMING_PARAM_I     // presence pulse must not finish before the sample point
        ) {
            *slave_is_present = true;
            return OWB_STATUS_OK;
    }

    // anything else is invalid
    return OWB_STATUS_HW_ERROR;
}


/**
 * @brief Parses the RMT symbols received during the transmission of up to 64 onewire bits.
 * @param[in] num_symbols The number of symbols passed. 
 * @param[in] symbols An array of RMT symbols.
 * @param[out] result The decoded bits (max 64, lsb first)
 * @return int The number of bits decoded
 */
static int _parse_bit_symbols (size_t num_symbols, rmt_symbol_word_t *p_symbol, uint64_t *result) {
    *result = 0;
    int bit_count = 0;
    rmt_symbol_word_t *p_last_symbol = p_symbol + num_symbols;

    #ifdef OWB_RMT_DEBUG
    // display raw RMT symbols
    ESP_LOGI(TAG, "parse bits: %d symbols", (int)num_symbols);
    #endif

    while (p_symbol < p_last_symbol && bit_count < 64) {
        #ifdef OWB_RMT_DEBUG
        ESP_LOGI (TAG, "\t%u (%uus), %u (%uus)", p_symbol->level0, p_symbol->duration0, 
                    p_symbol->level1, p_symbol->duration1);
        #endif
        if (abs (p_symbol->duration0 - OWB_TIMING_PARAM_A) <= OWB_TIMING_MARGIN &&
            (p_symbol->duration1 == 0 || p_symbol->duration1 >= OWB_TIMING_PARAM_E)) {
                // bus was released at the sample point: detect a '1'   
                *result |= (1ull << bit_count);
                bit_count += 1;

                #ifdef OWB_RMT_DEBUG
                ESP_LOGI (TAG, "\t\tdetect '1' -> 0x%llx", *result);
                #endif

        } else if (p_symbol->duration0 >= (OWB_TIMING_PARAM_A + OWB_TIMING_PARAM_E)) {
            // bus was asserted at the sample point: detect a '0'
            bit_count += 1;

            #ifdef OWB_RMT_DEBUG
            ESP_LOGI (TAG, "\t\tdetect '0' -> 0x%llx", *result);
            #endif            
        }
        p_symbol += 1;        // next symbol
    }

    return bit_count;
}


/**
 * @brief Sends a onewire bus reset pulse and listens for slave presence responses.
 * @param[in] bus Points to the OneWireBus structure (see owb.h).
 * @param[out] is_present Points to a bool that will receive the detection result.
 * @return OWB_STATUS_OK if the call succeeded; otherwise an owb_status error code (see owb.h).
 */
static owb_status _reset (const OneWireBus *bus, bool *is_present) {

    esp_err_t esp_status;

    // identify the rmt_driver_info structure that contains `bus`
    owb_rmt_driver_info *info = __containerof(bus, owb_rmt_driver_info, bus);

    // start the receiver before the transmitter so that it sees the leading edge of the pulse
    esp_status = rmt_receive (
        info->rx_channel_handle, 
        info->rx_buffer,
        info->rx_buffer_size_in_bytes, 
        &rx_config_owb_reset);
    if (esp_status != ESP_OK) {
        ESP_LOGE(TAG, "owb_reset: rx err");
        return OWB_STATUS_HW_ERROR;
    }

    // encode and transmit the reset pulse using the RMT 'copy' encoder
    esp_status = rmt_transmit (
        info->tx_channel_handle, 
        info->copy_encoder_handle, 
        &owb_rmt_symbol_reset, 
        sizeof (owb_rmt_symbol_reset),
        &owb_rmt_transmit_config);    
    if (esp_status != ESP_OK) {
        ESP_LOGE(TAG, "owb_reset: tx err");
        return OWB_STATUS_HW_ERROR;
    }

    // wait for the transmission to finish (or timeout with an error)
    if (rmt_tx_wait_all_done (info->tx_channel_handle, OWB_RMT_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "owb_reset: tx timeout");
        return OWB_STATUS_DEVICE_NOT_RESPONDING;        // tx timeout
    }

    // wait for the recv_done event data from our callback
    rmt_rx_done_event_data_t rx_done_event_data;
    if (xQueueReceive (info->rx_queue, &rx_done_event_data, pdMS_TO_TICKS(OWB_RMT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "owb_reset: no rx symbol");       // rx timeout
        return OWB_STATUS_DEVICE_NOT_RESPONDING;
    }
    
    // parse the event data and return the result
    return _parse_reset_symbols (rx_done_event_data.num_symbols, rx_done_event_data.received_symbols, is_present);
}

/**
 * @brief Writes a number of bytes to the onewire bus (slightly more efficient than sending them individually).
 * @param bus A previously-initialised OneWireBus.
 * @param bytes The bytes to be sent.
 * @param number_of_bytes_to_write How many bytes to send.
 * @return owb_status OWB_STATUS_OK on success, otherwise an error code (see owb.h).
 */
static owb_status _write_bytes(const OneWireBus *bus, uint8_t *bytes, int number_of_bytes_to_write) {
    esp_err_t esp_status;

    // identify the rmt_driver_info structure that contains `bus`
    owb_rmt_driver_info *info = __containerof(bus, owb_rmt_driver_info, bus);

    // encode and transmit the bits using the RMT 'bytes' encoder
    esp_status = rmt_transmit (
        info->tx_channel_handle,
        info->bytes_encoder_handle,
        bytes,
        (size_t)number_of_bytes_to_write,
        &owb_rmt_transmit_config);
    if (esp_status != ESP_OK) {
        ESP_LOGE(TAG, "owb_write: tx err");
        return OWB_STATUS_HW_ERROR;
    }

    // wait for the transmission to finish (or timeout with an error)
    if (rmt_tx_wait_all_done (info->tx_channel_handle, OWB_RMT_TIMEOUT_MS) != ESP_OK) {
        return OWB_STATUS_DEVICE_NOT_RESPONDING;    // tx timeout
    }
    return OWB_STATUS_OK;    
}


/**
 * @brief Writes 1-8 bits to the onewire bus.
 * @param bus A previously-initialised OneWireBus.
 * @param bytes A byte with the bits to be sent (lsb first).
 * @param number_of_bits_to_write How many bits to send (maximum 8).
 * @return owb_status OWB_STATUS_OK on success, otherwise an error code (see owb.h).
 */
static owb_status _write_bits(const OneWireBus *bus, uint8_t out, int number_of_bits_to_write) {

    // send 8 bits as a byte instead
    if (number_of_bits_to_write == 8) {
        return _write_bytes (bus, &out, 1);
    }

    if (number_of_bits_to_write < 1 || number_of_bits_to_write > 8) {
        ESP_LOGE(TAG, "owb_write_bits: bad num of bits (%d)", number_of_bits_to_write);
        return OWB_STATUS_TOO_MANY_BITS;
    }

    // identify the rmt_driver_info structure that contains `bus`
    owb_rmt_driver_info *info = __containerof(bus, owb_rmt_driver_info, bus);

    // send data as individual bits using the `copy` encoder
    const rmt_symbol_word_t *symbol_ptr;
    esp_err_t esp_status;
    for (int b = 0; b < number_of_bits_to_write; b += 1) {
        if ((out & (1 << b)) == 0) {
            symbol_ptr = &owb_rmt_symbol_0bit; 
        } else {
            symbol_ptr = &owb_rmt_symbol_1bit;
        }

        // send bit symbol
        esp_status = rmt_transmit (
            info->tx_channel_handle, 
            info->copy_encoder_handle, 
            symbol_ptr,
            sizeof (rmt_symbol_word_t),
            &owb_rmt_transmit_config);    
        if (esp_status != ESP_OK) {
            ESP_LOGE(TAG, "owb_write_bit: tx err");
            return OWB_STATUS_HW_ERROR;
        }
    }

    // wait for the transmission to finish (or timeout with an error)
    if (rmt_tx_wait_all_done (info->tx_channel_handle, OWB_RMT_TIMEOUT_MS) != ESP_OK) {
        return OWB_STATUS_DEVICE_NOT_RESPONDING;    // tx timeout
    }

    return OWB_STATUS_OK;
}


/**
 * @brief Reads up to 8 bytes from the onewire bus (this is faster than reading individual bits).
 * @param bus A previously-initialised OneWireBus.
 * @param result The resulting data, stored lsb first in a uint64_t.
 * @param number_of_bytes_to_read The number of bytes to read.
 * @return owb_status OWB_STATUS_OK on success, otherwise and error code (see owb.h)
 */
static owb_status _read_bytes(const OneWireBus *bus, uint64_t *result_ptr, int number_of_bytes_to_read) {
    static uint8_t ff_bytes[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    esp_err_t esp_status;

    if (number_of_bytes_to_read > 8) {
        ESP_LOGE(TAG, "owb_read_bytes: max 8");
        return OWB_STATUS_TOO_MANY_BITS;
    }

    // identify the rmt_driver_info structure that contains `bus`
    owb_rmt_driver_info *info = __containerof(bus, owb_rmt_driver_info, bus);

    // start the receiver before the transmitter so that it sees the first edge
    esp_status = rmt_receive (
        info->rx_channel_handle, 
        info->rx_buffer,
        info->rx_buffer_size_in_bytes, 
        &rx_config_owb_bits);
    if (esp_status != ESP_OK) {
        ESP_LOGE(TAG, "owb_read_bytes: rx err");
        return OWB_STATUS_HW_ERROR;
    }

    // generate read slots
    esp_status = rmt_transmit (
        info->tx_channel_handle,
        info->bytes_encoder_handle,
        ff_bytes,
        (size_t)number_of_bytes_to_read,
        &owb_rmt_transmit_config);
    if (esp_status != ESP_OK) {
        ESP_LOGE(TAG, "owb_read_bytes: tx err");
        return OWB_STATUS_HW_ERROR;
    }
    
    // wait for the transmission to finish (or timeout with an error)
    if (rmt_tx_wait_all_done (info->tx_channel_handle, OWB_RMT_TIMEOUT_MS) != ESP_OK) {
        return OWB_STATUS_DEVICE_NOT_RESPONDING;    // tx timeout
    }

    // wait for the recv_done event data from our callback
    rmt_rx_done_event_data_t rx_done_event_data;
    if (xQueueReceive (info->rx_queue, &rx_done_event_data, pdMS_TO_TICKS(OWB_RMT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "owb_read_bytes: no rx symbols");     // rx timeout
        return OWB_STATUS_DEVICE_NOT_RESPONDING;
    }

    // decode upto 64 data bits from the received RMT symbols 
    if (_parse_bit_symbols(rx_done_event_data.num_symbols, rx_done_event_data.received_symbols, result_ptr) == 0) {
        ESP_LOGE(TAG, "owb_read_bytes: no bits");
    }

    return OWB_STATUS_OK;
}


/**
 * @brief Reads up to 8 bits from the onewire bus.
 * @param bus A previously-initialised OneWireBus.
 * @param result A byte containing the bits read (lsb first).
 * @param number_of_bits_to_read The number of bits to read.
 * @return owb_status OWB_STATUS_OK on success, otherwise an error code (see owb.h)
 */
static owb_status _read_bits(const OneWireBus *bus, uint8_t *result, int number_of_bits_to_read) {
    esp_err_t esp_status;

    if (number_of_bits_to_read > 8) {
        ESP_LOGE(TAG, "owb_read_bits: max 8");
        return OWB_STATUS_TOO_MANY_BITS;
    }

    // it's quicker to read 8 bits as a whole byte
    if (number_of_bits_to_read == 8) {
        uint64_t result_64;
        owb_status status;
        status = _read_bytes (bus, &result_64, 1);
        *result = (uint8_t)result_64;
        return status; 
    }

    // identify the rmt_driver_info structure that contains `bus`
    owb_rmt_driver_info *info = __containerof(bus, owb_rmt_driver_info, bus);

    // with the copy encoder then it's most efficient to receive each bit individually
    // because we don't accurately know the interval between bits.
    // It would be nice to use `rmt_transmit_config.loop_count` here, but it's not supported
    // on all chips. In any case the user almost certainly only wants a single bit.
    *result = 0;
    for (int bit_index = 0; bit_index < number_of_bits_to_read; bit_index += 1) {

        // start the receiver before the transmitter so that it sees the first edge
        esp_status = rmt_receive (
            info->rx_channel_handle, 
            info->rx_buffer,
            info->rx_buffer_size_in_bytes, 
            &rx_config_owb_bits);
        if (esp_status != ESP_OK) {
            ESP_LOGE(TAG, "owb_read_bits: rx err");
            return OWB_STATUS_HW_ERROR;
        }

        // send a '1' symbol to generate a read slot
        esp_status = rmt_transmit (
            info->tx_channel_handle, 
            info->copy_encoder_handle, 
            &owb_rmt_symbol_1bit,
            sizeof (rmt_symbol_word_t),
            &owb_rmt_transmit_config);    
        if (esp_status != ESP_OK) {
            ESP_LOGE(TAG, "owb_read_bits: tx err");
            return OWB_STATUS_HW_ERROR;
        }

        // wait for the transmission to finish (or timeout with an error)
        if (rmt_tx_wait_all_done (info->tx_channel_handle, OWB_RMT_TIMEOUT_MS) != ESP_OK) {
            return OWB_STATUS_DEVICE_NOT_RESPONDING;    // tx timeout
        }

        // wait for the recv_done event data from our callback
        rmt_rx_done_event_data_t rx_done_event_data;
        if (xQueueReceive (info->rx_queue, &rx_done_event_data, pdMS_TO_TICKS(OWB_RMT_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "owb_read_bits: no rx symbol");       // rx timeout
            return OWB_STATUS_DEVICE_NOT_RESPONDING;
        }

        // parse the event data
        uint64_t bits = 0;
        if (_parse_bit_symbols (rx_done_event_data.num_symbols, rx_done_event_data.received_symbols, &bits) == 0) {
            ESP_LOGE(TAG, "owb_read_bits: no bits");
            return OWB_STATUS_HW_ERROR;
        }

        // add the bit to `result` (lsb is received first)
        if ((bits & 1) != 0) {
            *result |= (1 << bit_index);
        }
    }

    return OWB_STATUS_OK;    
}


/**
 * @brief Handle the RMT `recv_done` event by copying the event data structure to the specified queue.
 * @param[in] channel The handle of the RMT channel that generated the event.
 * @param[in] edata A pointer to the RMT event data structure (the pointer is valid only within this function).
 * @param[in] context A pointer to the user-provided context, in this case the queue handle.
 * @return True if sending to the queue caused a higher priority task to unblock; otherwise False.
 */
static bool IRAM_ATTR _recv_done_callback (rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data, void *user_data) {
    // Copy a pointer to the event data structure to the queue identified in the user_data.
    //* NOTE: this is an interrupt handler so it needs IRAM_ATTR, may only use `ISR` calls and must return promptly.
    //
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR ((QueueHandle_t)user_data, event_data, &pxHigherPriorityTaskWoken);
    if (pxHigherPriorityTaskWoken == pdTRUE) {
        return true;
    }
    return false;
}


//-----
// Public API functions
//-----

// RMT version of the OWB driver api (will be stored as info->bus->driver)
//
static struct owb_driver rmt_driver_functions = {
    .name = "owb_rmt",
    .uninitialize = _uninitialize,
    .reset = _reset,
    .write_bits = _write_bits,
    .write_bytes = _write_bytes,    // new addition to the API
    .read_bits = _read_bits,
    .read_bytes = _read_bytes       // new addition to the API
};


// configure and allocate resources
//
OneWireBus* owb_rmt_initialize (owb_rmt_driver_info *info, gpio_num_t gpio_num, int tx_channel, int rx_channel)
{
    //* The function now ignores tx_channel and rx_channel as the new RMT driver allocates channels on demand.
    //* The parameters are kept in the call to preserve compatibility with previous versions.

    // the steps to enable the RMT resources are documented in:
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html

    //  Note: keeping the TX and RX initialisations together in one function simplifies the error handling

    (void)tx_channel;   // avoid compiler warning about unused parameter
    (void)rx_channel;   // avoid compiler warning about unused parameter

    // sanity check
    if (info == NULL) {
        ESP_LOGE(TAG, "info is NULL");
        goto exit_err;
    } 
    
    // ----- receive channel -----

    // channel config
    const rmt_rx_channel_config_t rx_channel_config = {
        .gpio_num = (int)gpio_num,
        .clk_src = RMT_CLK_SRC_APB,         // use the APB clock (might reduce during light sleep)
        .resolution_hz = OWB_RMT_CLK_HZ,
        .mem_block_symbols = (size_t)OWB_RMT_RX_MEM_BLOCK_SYMBOLS,
                .flags = {
            .invert_in = 0,                 // don't hardware invert the input
            .with_dma = 0,                  // don't enable DMA
            .io_loop_back = 0               // we define the loopback in the tx config
        }
    };

    // request channel
    //* note: to get a wired-OR bus you must apply the rx_config first, _then_ the rx_config
    if (rmt_new_rx_channel (&rx_channel_config, &(info->rx_channel_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "err requesting rx_channel");
        goto exit_err;
    }

    // create queue for RMT `rx_done` event data struct (from callback)
    info->rx_queue = xQueueCreate (1, sizeof (rmt_rx_done_event_data_t));
    if (info->rx_queue == NULL) {
        ESP_LOGE(TAG, "err creating rx_queue");
        goto exit_delete_rx_channel;
    }

    // allocate rx symbol buffer for RMT driver
    info->rx_buffer_size_in_bytes = OWB_RMT_MAX_READ_BITS * sizeof (rmt_symbol_word_t);
    info->rx_buffer = (rmt_symbol_word_t *)malloc (info->rx_buffer_size_in_bytes);
    if (info->rx_buffer == NULL) {
        ESP_LOGE(TAG, "err allocating rx_buffer");
        goto exit_delete_rx_queue;
    }

    // register rx channel callback (rx_queue is passed as user context)
    const rmt_rx_event_callbacks_t rmt_rx_event_callbacks = {
        .on_recv_done = _recv_done_callback
    };
    if (rmt_rx_register_event_callbacks (info->rx_channel_handle, &rmt_rx_event_callbacks, info->rx_queue) != ESP_OK) {
        ESP_LOGE(TAG, "err registering rx_callbacks");
        goto exit_release_rx_buffer;
    }

    // enable channel
    if (rmt_enable (info->rx_channel_handle) != ESP_OK) {
        ESP_LOGE(TAG, "err enabling rx_channel");
        goto exit_release_rx_buffer;
    }

    // ----- transmit channel -----

    // channel config
    const rmt_tx_channel_config_t tx_channel_config = {
        .gpio_num = (int)gpio_num,
        .clk_src = RMT_CLK_SRC_APB,         // use the APB clock (might reduce during light sleep)
        .resolution_hz = OWB_RMT_CLK_HZ,
        .mem_block_symbols = (size_t)OWB_RMT_TX_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = OWB_RMT_TX_QUEUE_DEPTH,
        .flags = {
            .invert_out = 1,                // invert the output (so that the bus is initially released)
            .with_dma = 0,                  // don't enable DMA
            .io_loop_back = 1,              // enable reading of actual voltage of output pin
            .io_od_mode = 1                 // enable open-drain output, so as to achieve a 'wired-OR' bus
        }
    };

    // request channel
    if (rmt_new_tx_channel (&tx_channel_config, &(info->tx_channel_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "err requesting tx_channel");
        goto exit_disable_rx_channel;
    }

    // enable channel
    if (rmt_enable (info->tx_channel_handle) != ESP_OK) {
        ESP_LOGE(TAG, "err enabling tx_channel");
        goto exit_delete_tx_channel;
    }

    // obtain a 'copy' encoder (an RMT built-in used for sending fixed bit patterns)
    const rmt_copy_encoder_config_t rmt_copy_encoder_config = {};   // config is "reserved for future expansion"
    if (rmt_new_copy_encoder (&rmt_copy_encoder_config, &(info->copy_encoder_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "err requesting copy encoder");
        goto exit_disable_tx_channel;
    }

    // otain a 'bytes' encoder (an RMT built-in used for sending variable bit patterns)
    const rmt_bytes_encoder_config_t rmt_bytes_encoder_config = {
        .bit0 = OWB_RMT_SYMBOL_0BIT,
        .bit1 = OWB_RMT_SYMBOL_1BIT,
        .flags = {
            .msb_first = 0                  // onewire bus on-the-wire bit order is lsb first
        }
    };
    if (rmt_new_bytes_encoder(&rmt_bytes_encoder_config, &info->bytes_encoder_handle) != ESP_OK) {
        ESP_LOGE(TAG, "err requesting bytes encoder");
        goto exit_delete_copy_encoder;
    }


    // ----- success ------
    info->gpio = gpio_num;
    info->bus.driver = &rmt_driver_functions;   // route driver API calls to the functions in this file
    ESP_LOGI(TAG, "%s: OK", __func__);
    return &(info->bus);

    // ----- error: unwind allocated resources -----
exit_delete_copy_encoder:
    ESP_ERROR_CHECK(rmt_del_encoder(info->copy_encoder_handle));
exit_disable_tx_channel:
    ESP_ERROR_CHECK(rmt_disable (info->tx_channel_handle));
exit_delete_tx_channel:
    ESP_ERROR_CHECK(rmt_del_channel (info->tx_channel_handle));
exit_disable_rx_channel:
    ESP_ERROR_CHECK(rmt_disable (info->rx_channel_handle));
exit_release_rx_buffer:
    free (info->rx_buffer);
exit_delete_rx_queue:
    vQueueDelete (info->rx_queue);
exit_delete_rx_channel:
    ESP_ERROR_CHECK(rmt_del_channel (info->rx_channel_handle));
exit_err:
    ESP_LOGE(TAG, "%s: failed", __func__);
    return NULL;
}
