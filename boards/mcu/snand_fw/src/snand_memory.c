/*
 * Copyright (c) 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "snand_memory.h"
#include "fsl_device_registers.h"
#include "snand_flash.h"
#include <string.h>

#ifndef SPINAND_INSTANCE
#define SPINAND_INSTANCE 0
#endif

#ifndef SPINAND_ERASE_VERIFY
#define SPINAND_ERASE_VERIFY 1
#endif

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum
{
    kFlashDefaultPattern = 0xFF,
};

enum
{
    kSPINANDStartAddress = 0,
};

enum
{
    kNandAddressType_ByteAddress = 0,
    kNandAddressType_BlockAddress = 1,
};

//! @brief SPI NAND memory feature inforamation
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operatations.
typedef struct _spinand_mem_context
{
    bool isConfigured;
    bool needToUpdateDbbt;
    uint32_t nandAddressType;
    uint32_t startBlockId;
    bool readwriteInProgress;
    uint32_t readBuffer[kSpiNandMemory_MaxPageSize / sizeof(uint32_t)];
    bool isReadBufferValid;
    uint32_t readBufferPageAddr;
    uint32_t writeBuffer[kSpiNandMemory_MaxPageSize / sizeof(uint32_t)];
    bool isWriteBufferValid;
    uint32_t writeBufferOffset;
    uint32_t writeBufferPageAddr;
    uint32_t skippedBlockCount;
    uint32_t instance;
} spinand_mem_context_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
static status_t spinand_mem_load_buffer(uint32_t pageAddr);

static status_t spinand_mem_flush_buffer(void);

static status_t spinand_mem_block_backup(uint32_t srcPageAddr, uint32_t destBlockAddr);

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
static bool is_erased_memory(uint32_t pageAddr, uint32_t pageCount);
#endif

static status_t spinand_memory_read(uint32_t pageAddr, uint32_t length, uint8_t *buffer);

static status_t spinand_memory_write_and_verify(uint32_t pageAddr, uint32_t length, uint8_t *buffer);

static status_t spinand_memory_erase_and_verify(uint32_t blockAddr);

static status_t spinand_memory_spi_init(flexspi_nand_config_t *config);

static status_t spinand_memory_read_page(uint32_t pageAddr, uint32_t length, uint8_t *buffer);

static status_t spinand_memory_program_page(uint32_t pageAddr, uint32_t length, uint8_t *buffer);

static status_t spinand_memory_erase_block(uint32_t blockAddr);

static bool is_spinand_configured(void);

static status_t spinand_mem_creat_empty_dbbt(void);

static bool is_bad_block_in_dbbt(uint32_t blockAddr);

static status_t bad_block_discovered(uint32_t blockAddr);

static status_t skip_bad_blocks(uint32_t *pageAddr);

static bool need_to_check_dbbt_before_read(uint32_t blockAddr);

static bool need_to_check_dbbt_before_write(uint32_t blockAddr);

static bool is_read_page_cached(uint32_t pageAddr);

static bool is_write_page_cached(uint32_t pageAddr);

static status_t nand_generate_fcb(spinand_fcb_t *fcb, serial_nand_config_option_t *option);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

static spinand_fcb_t s_spinandFcb;

static spinand_dbbt_t s_spinandDbbt;

static spinand_mem_context_t s_spinandContext = {
    .isConfigured = false,
    .skippedBlockCount = 0,
    .isReadBufferValid = false,
    .isWriteBufferValid = false,
    .instance = 0,
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See qspi_memory.h for documentation on this function.
status_t spinand_mem_init(void)
{
    status_t status;

    // Load default config block from efuse.
    status = flexspi_nand_get_default_cfg_blk(&s_spinandFcb.config);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Init SPI peripheral to enable fcb read.
    status = spinand_memory_spi_init(&s_spinandFcb.config);
    if (status != kStatus_Success)
    {
        return status;
    }

    //status = spinand_mem_load_dbbt(&s_spinandFcb);
    //if (status != kStatus_Success)
    {
        // Do not create a new DBBT during init.
        // There is a risk, that the old DBBT is crashed.
        // User needs to re-configure.
        return kStatus_Fail;
    }
}

status_t nand_generate_fcb(spinand_fcb_t *fcb, serial_nand_config_option_t *option)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((fcb == NULL) || (option == NULL))
        {
            break;
        }

        memset(fcb, 0, sizeof(spinand_fcb_t));

        status = flexspi_nand_get_config(SPINAND_INSTANCE, &fcb->config, option);
        if (status != kStatus_Success)
        {
            break;
        }

        s_spinandContext.nandAddressType = kNandAddressType_ByteAddress;

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t spinand_mem_config(uint32_t *config)
{
    status_t status = kStatus_InvalidArgument;

    bool isNandConfig = false;
    
    serial_nand_config_option_t *option = (serial_nand_config_option_t *)config;

    do
    {
        status = nand_generate_fcb(&s_spinandFcb, option);
        if (status != kStatus_Success)
        {
            break;
        }

        // First, mark SPI NAND as not configured.
        s_spinandContext.isConfigured = false;
        isNandConfig = true;

        if (isNandConfig)
        {
            status = spinand_memory_spi_init(&s_spinandFcb.config);
            if (status != kStatus_Success)
            {
                break;
            }

            spinand_mem_creat_empty_dbbt();
            // All configuration steps are success. SPI NAND can be accessable.
            s_spinandContext.isConfigured = true;
        }
    } while (0);

    return status;
}

// See qspi_memory.h for documentation on this function.
status_t spinand_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (buffer == NULL)
        {
            break;
        }

        // SPI NAND should be configured before access.
        if (!is_spinand_configured())
        {
            status = kStatusMemoryNotConfigured;
            break;
        }

        uint32_t pageSize = s_spinandFcb.config.pageDataSize;
        uint32_t columnAddr;
        uint32_t pageAddr;

        if (s_spinandContext.nandAddressType == kNandAddressType_BlockAddress)
        {
            // If the Write transfer is not started yet, log the address as block address (index)
            if (!s_spinandContext.readwriteInProgress)
            {
                // Ensure the block address is a valid block address
                if (address >= s_spinandFcb.config.blocksPerDevice)
                {
                    break;
                }
                s_spinandContext.readwriteInProgress = true;
                s_spinandContext.startBlockId = address;
                columnAddr = 0;
                pageAddr = address * s_spinandFcb.config.pagesPerBlock;
            }
            // Otherwise, need to convert the address to actual physical address
            else
            {
                // Actual physical address caluclation formula: blockId * page size * pages per block + address -
                // blockId
                uint32_t actualAddress = s_spinandContext.startBlockId * s_spinandFcb.config.pagesPerBlock *
                                             s_spinandFcb.config.pageDataSize +
                                         (address - s_spinandContext.startBlockId);
                columnAddr = actualAddress % pageSize;
                pageAddr = actualAddress / pageSize;
            }
        }
        else // Address is actual NAND address
        {
            columnAddr = address % pageSize;
            pageAddr = address / pageSize;
        }

        // Skip the skipped blocks during a read operation.
        // No need to change the columnAddr.
        skip_bad_blocks(&pageAddr);

        uint32_t readLength;

        while (length)
        {
            // Check if current page to read is already read to readbuffer.
            // If no, need to read the whole page to buffer.
            if (!is_read_page_cached(pageAddr))
            {
                // Check if the page to read and cached page is in the same block.
                // If no, need to check if the block is a bad block.
                uint32_t blockAddr = pageAddr / s_spinandFcb.config.pagesPerBlock;
                if (need_to_check_dbbt_before_read(blockAddr))
                {
                    // Due to skipping bad blocks, blockAddr might cross the end boundary.
                    // Need to check the range.
                    if (blockAddr >= s_spinandFcb.config.blocksPerDevice)
                    {
                        status = kStatusMemoryRangeInvalid;
                        break;
                    }
                    // Check if reading a bad block.
                    if (is_bad_block_in_dbbt(blockAddr))
                    {
                        // If yes, skip the bad block and read the next block.
                        s_spinandContext.skippedBlockCount++;
                        pageAddr += s_spinandFcb.config.pagesPerBlock;
                        continue;
                    }
                }

                // Good block and not cached, then read the page to buffer.
                status = spinand_mem_load_buffer(pageAddr);
                if (status != kStatus_Success)
                {
                    break;
                }
            }
            // If it is a read accoss the page, divide it into two steps.
            if (columnAddr + length <= pageSize)
            {
                readLength = length;
            }
            else
            {
                readLength = pageSize - columnAddr;
            }
            uint8_t *p_readBuffer_8 = (uint8_t *)s_spinandContext.readBuffer;
            memcpy(buffer, &p_readBuffer_8[columnAddr], readLength);
            length -= readLength;
            buffer += readLength;
            columnAddr += readLength;
            if (columnAddr >= pageSize)
            {
                columnAddr -= pageSize;
                pageAddr++;
            }
            // Mark current loop is successfully executed.
            status = kStatus_Success;
        }

        // Terminate current transfer if errors happen during transfer
        if (status != kStatus_Success)
        {
            s_spinandContext.readwriteInProgress = false;
        }

    } while (0);

    return status;
}

status_t spinand_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (buffer == NULL)
        {
            break;
        }

        // SPI NAND should be configured before access.
        if (!is_spinand_configured())
        {
            status = kStatusMemoryNotConfigured;
            break;
        }

        uint32_t pageSize = s_spinandFcb.config.pageDataSize;
        uint32_t columnAddr;
        uint32_t pageAddr;
        static uint32_t expectedNextActualAddr;
        if (s_spinandContext.nandAddressType == kNandAddressType_BlockAddress)
        {
            // If the Write transfer is not started yet, log the address as block address (index)
            if (!s_spinandContext.readwriteInProgress)
            {
                // Ensure the block address is a valid block address
                if (address >= s_spinandFcb.config.blocksPerDevice)
                {
                    break;
                }
                s_spinandContext.isWriteBufferValid = false;
                s_spinandContext.readwriteInProgress = true;
                s_spinandContext.startBlockId = address;
                columnAddr = 0;
                pageAddr = address * s_spinandFcb.config.pagesPerBlock;
                expectedNextActualAddr = pageAddr * pageSize + length;
            }
            // Otherwise, need to convert the address to actual physical address
            else
            {
                // Actual physical address caluclation formula: blockId * page size * pages per block + address -
                // blockId
                uint32_t actualAddress = s_spinandContext.startBlockId * s_spinandFcb.config.pagesPerBlock *
                                             s_spinandFcb.config.pageDataSize +
                                         (address - s_spinandContext.startBlockId);
                columnAddr = actualAddress % pageSize;
                pageAddr = actualAddress / pageSize;

                // The address is continuous in a transfer, so once the address is not continuous,
                // Flush data in buffer into SPI NAND and then re-start a new transfer
                if (actualAddress != expectedNextActualAddr)
                {
                    if (s_spinandContext.isWriteBufferValid)
                    {
                        status = spinand_mem_flush_buffer();
                        if (status != kStatus_Success)
                        {
                            // Terminate transfer if error occurs.
                            s_spinandContext.readwriteInProgress = false;
                            break;
                        }
                    }
                    s_spinandContext.readwriteInProgress = true;
                    s_spinandContext.startBlockId = address;
                    columnAddr = 0;
                    pageAddr = s_spinandContext.startBlockId * s_spinandFcb.config.pagesPerBlock;
                    actualAddress = pageAddr * pageSize;
                }
                expectedNextActualAddr = actualAddress + length;
            }
        }
        else // Address is actual NAND address
        {
            columnAddr = address % pageSize;
            pageAddr = address / pageSize;
        }

        // Skip the skipped blocks during a read operation.
        // No need to change the columnAddr.
        skip_bad_blocks(&pageAddr);

        uint32_t writeLength;

        while (length)
        {
            // Check if current page to write is already cached to writebuffer.
            // If no, need to init the writebuffer.
            if (!is_write_page_cached(pageAddr))
            {
                uint32_t blockAddr = pageAddr / s_spinandFcb.config.pagesPerBlock;
                // Check if the page to write and cached page is in the same block.
                // If no, need to check if the block is a bad block.
                if (need_to_check_dbbt_before_write(blockAddr))
                {
                    // Due to skipping bad blocks, blockAddr might cross the end boundary.
                    // Need to check the range.

                    if (blockAddr >= s_spinandFcb.config.blocksPerDevice)
                    {
                        status = kStatusMemoryRangeInvalid;
                        break;
                    }
                    // Check if writting a bad block.
                    if (is_bad_block_in_dbbt(blockAddr))
                    {
                        // If yes, skip the bad block and write to the next block.
                        s_spinandContext.skippedBlockCount++;
                        pageAddr += s_spinandFcb.config.pagesPerBlock;
                        continue;
                    }
                }
                // There is data already cached in the buffer, flush it to SPI NAND.
                if (s_spinandContext.isWriteBufferValid)
                {
                    status = spinand_mem_flush_buffer();
                    if (status != kStatus_Success)
                    {
                        break;
                    }
                }
                // Start a new page write. The address must page size aligned.
                if (columnAddr != 0)
                {
                    status = kStatus_FlexSPINAND_WriteAlignmentError;
                    break;
                }
                s_spinandContext.writeBufferOffset = columnAddr;
                s_spinandContext.writeBufferPageAddr = pageAddr;
                s_spinandContext.isWriteBufferValid = true;
            }

            // If the address is not continuous, start a new page write.
            if (s_spinandContext.writeBufferOffset != columnAddr)
            {
                status = spinand_mem_flush_buffer();
                if (status != kStatus_Success)
                {
                    break;
                }
                continue;
            }

            if (columnAddr + length <= pageSize)
            {
                writeLength = length;
            }
            else
            {
                writeLength = pageSize - columnAddr;
            }
            uint8_t *p_writeBuffer_8 = (uint8_t *)s_spinandContext.writeBuffer;
            memcpy(&p_writeBuffer_8[columnAddr], buffer, writeLength);
            s_spinandContext.writeBufferOffset += writeLength;
            length -= writeLength;
            buffer += writeLength;
            columnAddr += writeLength;
            if (columnAddr >= pageSize)
            {
                columnAddr -= pageSize;
                pageAddr++;
            }
            // Mark current loop is successfully executed.
            status = kStatus_Success;
        }

        // Terminate current transfer if errors happen during transfer
        if (status != kStatus_Success)
        {
            s_spinandContext.readwriteInProgress = false;
        }
    } while (0);

    return status;
}

status_t spinand_mem_flush(void)
{
    status_t status = kStatus_Success;
    // If there still is data in the buffer, then flush them to SPI NAND.
    if (s_spinandContext.isWriteBufferValid)
    {
        status = spinand_mem_flush_buffer();
    }
    return status;
}

// See qspi_memory.h for documentation on this function.
status_t spinand_mem_finalize(void)
{
    status_t status = kStatus_Success;

    // Mark buffer to invalid.
    s_spinandContext.isWriteBufferValid = false;
    s_spinandContext.isReadBufferValid = false;
    // A read / write operation is finished. Clear the skipped block count.
    s_spinandContext.skippedBlockCount = 0;

    s_spinandContext.readwriteInProgress = false;

    return status;
}

// See qspi_memory.h for documentation on this function.
status_t spinand_mem_erase(uint32_t address, uint32_t length)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        // SPI NAND should be configured before access.
        if (!is_spinand_configured())
        {
            status = kStatusMemoryNotConfigured;
            break;
        }
        // length = 0 means no erase operation will be executed. Just return success.
        if (length == 0)
        {
            status = kStatus_Success;
            break;
        }
        // else means 1 block at lest to be erased.

        uint32_t totalBlocks = s_spinandFcb.config.blocksPerDevice;
        uint32_t startBlockAddr;
        uint32_t blockCount;
        // The address[30:0] is block id if address[31] is 1
        if (s_spinandContext.nandAddressType == kNandAddressType_BlockAddress)
        {
            startBlockAddr = address;
            blockCount = length;
        }
        else // Address is actual NAND address
        {
            startBlockAddr = address / s_spinandFcb.config.pageDataSize / s_spinandFcb.config.pagesPerBlock;
            // Don't get block count from length. Address to address + length might across block boundary.
            blockCount = (address + length - 1) / s_spinandFcb.config.pageDataSize / s_spinandFcb.config.pagesPerBlock -
                         startBlockAddr + 1;
        }

        // Due to bad block is skipped,
        // then also need to check if the block to erase is not cross the memory end.
        while (blockCount && (startBlockAddr < totalBlocks))
        {
            if (!is_bad_block_in_dbbt(startBlockAddr))
            {
                status = spinand_memory_erase_and_verify(startBlockAddr);
                if (status != kStatus_Success)
                {
                    bad_block_discovered(startBlockAddr);
                }
                else
                {
                    // Don't count in the bad blocks.
                    blockCount--;
                }
            }
            startBlockAddr++;
        }
    } while (0);

    return status;
}

// See memory.h for documentation on this function.
status_t spinand_mem_erase_all(void)
{
    // SPI NAND should be configured before access.
    if (!is_spinand_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    uint32_t startBlockAddr = 0;
    uint32_t totalBlocks = s_spinandFcb.config.blocksPerDevice;
    // In case SPI NAND is over 4G, do not call spinand_mem_erase() here.
    while (startBlockAddr < totalBlocks)
    {
        if (!is_bad_block_in_dbbt(startBlockAddr))
        {
            status_t status = spinand_memory_erase_and_verify(startBlockAddr);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
        startBlockAddr++;
    }

    return kStatus_Success;
}

static status_t spinand_mem_load_buffer(uint32_t pageAddr)
{
    status_t status;

    s_spinandContext.isReadBufferValid = false; // Mark read buffer invalid.

    // Read the page to read buffer.
    status =
        spinand_memory_read(pageAddr, s_spinandFcb.config.pageDataSize, (uint8_t *)&s_spinandContext.readBuffer[0]);
    if (status == kStatus_Success)
    {
        s_spinandContext.isReadBufferValid = true;
        s_spinandContext.readBufferPageAddr = pageAddr;
    }

    return status;
}

static status_t spinand_mem_flush_buffer(void)
{
    status_t status;

    // Terminate current transfer if the write buffer size is less than page size when the flush API is called
    if (s_spinandContext.writeBufferOffset != s_spinandFcb.config.pageDataSize)
    {
        s_spinandContext.readwriteInProgress = false;
    }

    s_spinandContext.isWriteBufferValid = false;

    uint32_t srcPageAddr = s_spinandContext.writeBufferPageAddr;

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
    if (!is_erased_memory(srcPageAddr, 1))
    {
        return kStatusMemoryCumulativeWrite;
    }
#endif // #if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE

    // Flush the data in the write buffer to SPI NAND
    status =
        spinand_memory_write_and_verify(srcPageAddr, s_spinandContext.writeBufferOffset, (uint8_t*)s_spinandContext.writeBuffer);
    if (status == kStatus_Success)
    {
        // Write success, return.
        return status;
    }
    // else write failed, and need to move data to the next good block.

    uint32_t srcBlockAddr = srcPageAddr / s_spinandFcb.config.pagesPerBlock;

    bad_block_discovered(srcBlockAddr);
    s_spinandContext.skippedBlockCount++;

    uint32_t totalBlocks = s_spinandFcb.config.blocksPerDevice;
    uint32_t destBlockAddr = srcBlockAddr + 1; // First destination block is next block.
    // Should not cross the end boundary.
    while (destBlockAddr < totalBlocks)
    {
        // Check if destination block is a good block.
        if (!is_bad_block_in_dbbt(destBlockAddr))
        {
            // If a good block, try to backup the datas to next block.
            status = spinand_mem_block_backup(srcPageAddr, destBlockAddr);
#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
            // Return if success or the next good block is not erased.
            if ((status == kStatus_Success) || (status == kStatusMemoryCumulativeWrite))
#else
            if (status == kStatus_Success)
#endif // #if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
            {
                return status;
            }
            // Backup failed means destination block is also a bad block.
            bad_block_discovered(destBlockAddr);
        }

        // Move to next block.
        destBlockAddr++;
        s_spinandContext.skippedBlockCount++;
    }
    // No erased good block left.
    return kStatusMemoryRangeInvalid;
}

static status_t spinand_mem_block_backup(uint32_t srcPageAddr, uint32_t destBlockAddr)
{
    status_t status = kStatus_Success;

    uint32_t startPageAddr = srcPageAddr - (srcPageAddr % s_spinandFcb.config.pagesPerBlock); // First page to backup.
    uint32_t endPageAddr = srcPageAddr; // The last page to backup. The last page is the page needs to flush.
    uint32_t destPageAddr =
        destBlockAddr * s_spinandFcb.config.pagesPerBlock; // Destination page to store the first page.

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
    // Firstly, need to check if the destination is erased.
    if (!is_erased_memory(destPageAddr, endPageAddr - startPageAddr + 1))
    {
        return kStatusMemoryCumulativeWrite;
    }
#endif // #if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE

    // Move all pages in the block to the good block, except the last page.
    while (startPageAddr < endPageAddr)
    {
        // Read the page needs to backup.
        status = spinand_memory_read(startPageAddr, s_spinandFcb.config.pageDataSize,
                                     (uint8_t *)&s_spinandContext.readBuffer[0]);
        if (status != kStatus_Success)
        {
            // Read failed, skip to next block to execute the backup progress.
            return status;
        }
        // Write the read page to the destination memory.
        status = spinand_memory_write_and_verify(destPageAddr, s_spinandFcb.config.pageDataSize,
                                                 (uint8_t *)&s_spinandContext.readBuffer[0]);
        if (status != kStatus_Success)
        {
            // Write failed, then skip to next block to execute the backup progress.
            return status;
        }
        // Move the source and destination pointer.
        startPageAddr++;
        destPageAddr++;
    }
    // Flush the last page. The data is contained in the write buffer.
    return spinand_memory_write_and_verify(destPageAddr, s_spinandFcb.config.pageDataSize,
                                           (uint8_t*)s_spinandContext.writeBuffer);
}

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
static bool is_erased_memory(uint32_t pageAddr, uint32_t pageCount)
{
    status_t status;
    uint32_t pageSize = s_spinandFcb.config.pageDataSize;
    uint32_t offset, *buffer;
    while (pageCount)
    {
        // Read the page firstly.
        status = spinand_memory_read(pageAddr++, pageSize, (uint8_t *)&s_spinandContext.readBuffer[0]);
        if (status != kStatus_Success)
        {
            // If read failed, return false.
            return false;
        }

        offset = 0;
        buffer = (uint32_t *)s_spinandContext.readBuffer;
        while (offset < pageSize)
        {
            // Check if all 0xFFs
            if (*buffer != 0xffffffff)
            {
                return false;
            }
            buffer++;
            offset += 4;
        }
        pageCount--;
    }
    return true;
}
#endif // #if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE

static status_t spinand_memory_read(uint32_t pageAddr, uint32_t length, uint8_t *buffer)
{
    assert(buffer);

    status_t status;
    uint32_t size;

    while (length)
    {
        size = MIN(s_spinandFcb.config.pageDataSize, length);
        status = spinand_memory_read_page(pageAddr, size, buffer);
        if (status != kStatus_Success)
        {
            return status;
        }
        buffer += size;
        length -= size;
        pageAddr++;
    }

    return status;
}

static status_t spinand_memory_write_and_verify(uint32_t pageAddr, uint32_t length, uint8_t *buffer)
{
    assert(buffer);

    status_t status;
    uint32_t size;

    while (length)
    {
        size = MIN(s_spinandFcb.config.pageDataSize, length);
        status = spinand_memory_program_page(pageAddr, size, buffer);
        if (status != kStatus_Success)
        {
            return status;
        }

        status = spinand_memory_read_page(pageAddr, size, (uint8_t *)&s_spinandContext.readBuffer[0]);
        if (status != kStatus_Success)
        {
            return status;
        }

        if (memcmp(buffer, s_spinandContext.readBuffer, size))
        {
            return kStatus_Fail;
        }

        buffer += size;
        length -= size;
        pageAddr++;
    }

    return status;
}

status_t spinand_memory_erase_and_verify(uint32_t blockAddr)
{
#if SPI_NAND_ERASE_VERIFY
    status_t status;

    uint32_t pageAddr = blockAddr * s_spinandFcb.config.pagesPerBlock;

    // Erase the block.
    status = spinand_memory_erase_block(blockAddr);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Check if the memory is erased ( All 0xFFs).
    if (is_erased_memory(pageAddr, s_spinandFcb.config.pagesPerBlock))
    {
        return kStatus_Success;
    }
    else
    {
        // If not all 0xFFs, means erase operation is failed.
        return kStatus_FlexSPINAND_EraseBlockFail;
    }
#else
    return spinand_memory_erase_block(blockAddr);
#endif // defined(SPI_NAND_ERASE_VERIFY)
}

status_t spinand_memory_spi_init(flexspi_nand_config_t *config)
{
    return flexspi_nand_init(SPINAND_INSTANCE, config);
}

static status_t spinand_memory_read_page(uint32_t pageAddr, uint32_t length, uint8_t *buffer)
{
    return flexspi_nand_read_page(SPINAND_INSTANCE, &s_spinandFcb.config, pageAddr, (uint32_t *)buffer, length);
}

static status_t spinand_memory_program_page(uint32_t pageAddr, uint32_t length, uint8_t *buffer)
{
    return flexspi_nand_program_page(SPINAND_INSTANCE, &s_spinandFcb.config, pageAddr, (uint32_t *)buffer, length);
}

static status_t spinand_memory_erase_block(uint32_t blockAddr)
{
    return flexspi_nand_erase_block(SPINAND_INSTANCE, &s_spinandFcb.config, blockAddr);
}

static bool is_spinand_configured(void)
{
    return s_spinandContext.isConfigured;
}

static status_t spinand_mem_creat_empty_dbbt(void)
{
    // Create empty dbbt;
    // Fill DBBT to all 0xFFs.
    memset(&s_spinandDbbt, kFlashDefaultPattern, sizeof(spinand_dbbt_t));

    s_spinandDbbt.badBlockNumber = 0;

    return kStatus_Success;
}

static bool is_bad_block_in_dbbt(uint32_t blockAddr)
{
    uint32_t i;
    // Traversal. No sort when adding items.
    for (i = 0; i < s_spinandDbbt.badBlockNumber; i++)
    {
        if (s_spinandDbbt.badBlockTable[i] == blockAddr)
        {
            return true;
        }
    }
    return false;
}

static status_t bad_block_discovered(uint32_t blockAddr)
{
    if (s_spinandDbbt.badBlockNumber < kSpiNandMemory_MaxDBBTSize)
    {
        s_spinandDbbt.badBlockTable[s_spinandDbbt.badBlockNumber++] = blockAddr;
        return kStatus_Success;
    }
    else
    {
        return kStatus_Fail;
    }
}

static status_t skip_bad_blocks(uint32_t *addr)
{
    *addr += s_spinandContext.skippedBlockCount * s_spinandFcb.config.pagesPerBlock;
    return kStatus_Success;
}

static bool need_to_check_dbbt_before_read(uint32_t blockAddr)
{
    return (!s_spinandContext.isReadBufferValid) ||
           (blockAddr != (s_spinandContext.readBufferPageAddr / s_spinandFcb.config.pagesPerBlock));
}

static bool need_to_check_dbbt_before_write(uint32_t blockAddr)
{
    return (!s_spinandContext.isWriteBufferValid) ||
           (blockAddr != (s_spinandContext.writeBufferPageAddr / s_spinandFcb.config.pagesPerBlock));
}

static bool is_read_page_cached(uint32_t pageAddr)
{
    return (s_spinandContext.readBufferPageAddr == pageAddr) && (s_spinandContext.isReadBufferValid);
}

static bool is_write_page_cached(uint32_t pageAddr)
{
    return (s_spinandContext.writeBufferPageAddr == pageAddr) && (s_spinandContext.isWriteBufferValid);
}


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
