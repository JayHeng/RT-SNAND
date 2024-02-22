/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _SNAND_ADAPTER_H_
#define _SNAND_ADAPTER_H_

#include "port_mixspi_info.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// Definition to select spi peripheral
#ifndef SNAND_MIXSPI_MODULE
#define SNAND_MIXSPI_MODULE SNAND_MIXSPI_MODULE_IS_FLEXSPI
#endif

#if SNAND_MIXSPI_MODULE == SNAND_MIXSPI_MODULE_IS_FLEXSPI
#define mixspi_pad_t                 flexspi_pad_t                
#define mixspi_read_sample_clock_t   flexspi_read_sample_clock_t  
#define MIXSPI_Type                  FLEXSPI_Type
#elif SNAND_MIXSPI_MODULE == SNAND_MIXSPI_MODULE_IS_QUADSPI
#define mixspi_pad_t                 uint32_t  
#define mixspi_read_sample_clock_t   qspi_dqs_read_sample_clock_t
#define MIXSPI_Type                  QuadSPI_Type
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


#endif /* _SNAND_ADAPTER_H_ */
