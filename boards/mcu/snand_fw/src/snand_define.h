/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _SNAND_DEFINE_H_
#define _SNAND_DEFINE_H_

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define VRA_MIXSPI_MODULE_IS_FLEXSPI 0U
#define VRA_MIXSPI_MODULE_IS_QUADSPI 1U

// Supported Mixspi clock defn
typedef enum _mixspi_root_clk_freq
{
    kMixspiRootClkFreq_30MHz  = 1,
    kMixspiRootClkFreq_50MHz  = 2,
    kMixspiRootClkFreq_60MHz  = 3,
    kMixspiRootClkFreq_80MHz  = 4,
    kMixspiRootClkFreq_100MHz = 5,
    kMixspiRootClkFreq_120MHz = 6,
    kMixspiRootClkFreq_133MHz = 7,
    kMixspiRootClkFreq_166MHz = 8,
    kMixspiRootClkFreq_200MHz = 9,
    kMixspiRootClkFreq_240MHz = 10,
    kMixspiRootClkFreq_266MHz = 11,
    kMixspiRootClkFreq_332MHz = 12,
    kMixspiRootClkFreq_400MHz = 13,
} mixspi_root_clk_freq_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

int snand_printf(const char *fmt_s, ...);

#endif /* _SNAND_DEFINE_H_ */
