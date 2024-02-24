/*
 * Copyright (c) 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__SNAND_MEMORY_H__)
#define __SNAND_MEMORY_H__

#include "snand_flash.h"

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////
enum
{
    //! @brief Max page size used to create a page buffer
    kSpiNandMemory_MaxPageSize = 4096U,
    //! @brief Max items in DBBT
    kSpiNandMemory_MaxDBBTSize = 256U,
};

typedef enum _spinand_property
{
    kSPINANDProperty_InitStatus = 0,
    kSPINANDProperty_StartAddress = 1,            //!< Tag used to retrieve start address
    kSPINANDProperty_TotalFlashSizeInKBytes = 2, //!< Tag used to retrieve total flash size in terms of KByte
    kSPINANDProperty_PageSize = 3,                //!< Tag used to retreive page size in terms of byte
    kSPINANDProperty_SectorSize = 4,              //!< Tag used to retrieve sector size in term of byte
    kSPINANDProperty_BlockSize = 5,               //!< Tag used to retrieve block size in terms of byte

    kSPINANDProperty_TotalFlashSize = 0x10, //!< Tag used to retrieve total flash size in terms of byte
} spinand_property_t;

typedef struct _spinand_fcb
{
    flexspi_nand_config_t config; //!< [0x100-0x2ff];
} spinand_fcb_t;

typedef struct _spinand_dbbt
{
    uint32_t badBlockNumber;
    uint32_t badBlockTable[kSpiNandMemory_MaxDBBTSize];
} spinand_dbbt_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! @name SPI NAND memory
//@{

//! @brief Initialize SPI NAND memory
status_t spinand_mem_init(void);

//! @brief Configure SPI NAND memory
status_t spinand_mem_config(uint32_t *config);

//! @brief Read SPI NAND memory.
status_t spinand_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

//! @brief Write SPI NAND memory.
status_t spinand_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief  Erase SPI NAND memory
status_t spinand_mem_erase(uint32_t address, uint32_t length);

//! @brief Erase all SPI NAND memory
status_t spinand_mem_erase_all(void);

//! @brief flush cached data to SPI NAND memory
status_t spinand_mem_flush(void);

//! @brief finalize the read/write operation of SPI NAND memory
status_t spinand_mem_finalize(void);


#endif //
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
