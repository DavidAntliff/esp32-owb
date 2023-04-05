/**
 * Copyright (c) 2023 mjcross
 *
 * SPDX-License-Identifier: MIT
**/

#include "owb_rmt_bus_timings.h"

// RMT transmit channel symbols for the onewire bus signals and conditions
//

// basic bus levels
// ----------------
// note: we configure the transmit channel to be hardware inverted,
//       so that the bus initialises in the 'released' state
#define OWB_RMT_BUS_ASSERTED 1
#define OWB_RMT_BUS_RELEASED 0

// bus symbols as `rmt_symbol_word_t`
// ----------------------------------

// send 'zero' bit
#define OWB_RMT_SYMBOL_0BIT { \
    .level0 = OWB_RMT_BUS_ASSERTED, \
    .duration0 = OWB_TIMING_PARAM_C, \
    .level1 = OWB_RMT_BUS_RELEASED, \
    .duration1 = OWB_TIMING_PARAM_D }

// send 'one' bit
#define OWB_RMT_SYMBOL_1BIT { \
    .level0 = OWB_RMT_BUS_ASSERTED, \
    .duration0 = OWB_TIMING_PARAM_A, \
    .level1 = OWB_RMT_BUS_RELEASED, \
    .duration1 = OWB_TIMING_PARAM_B }

// send bus reset 
#define OWB_RMT_SYMBOL_RESET { \
    .level0 = OWB_RMT_BUS_ASSERTED, \
    .duration0 = OWB_TIMING_PARAM_H, \
    .level1 = OWB_RMT_BUS_RELEASED, \
    .duration1 = OWB_TIMING_PARAM_I + OWB_TIMING_PARAM_J }
