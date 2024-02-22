/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _SNAND_FLASH_H_
#define _SNAND_FLASH_H_

#include "snand.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// NAND property info for operation
typedef struct _snand_property_info
{
    mixspi_pad_t                mixspiPad;
    mixspi_root_clk_freq_t      mixspiRootClkFreq;
    mixspi_read_sample_clock_t  mixspiReadSampleClock;
    const uint32_t             *mixspiCustomLUTVendor;
} snand_property_info_t;

#define CUSTOM_LUT_LENGTH                64


/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#endif /* _SNAND_FLASH_H_ */
