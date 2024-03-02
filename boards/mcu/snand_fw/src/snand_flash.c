/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "snand_flash.h"
#include "fsl_device_registers.h"
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define FREQ_396MHz (396UL * 1000 * 1000)
#define FREQ_528MHz (528UL * 1000 * 1000)
#define FREQ_24MHz (24UL * 1000 * 1000)
#define FREQ_480MHz (480UL * 1000 * 1000)

enum
{
    kMaxAHBClock = 144000000UL,
};

/*
 *  Fuse definition for safe boot frequency
 */
enum
{
    kFlexSpi_Nand_50MHz = 0,
    kFlexSpi_Nand_30MHz = 1,
    kFlexSpi_Nand_20MHz = 2,
    kFlexSpi_Nand_10MHz = 3,
};

/*
 * Fuse definition for CS interval between two commands
 */
enum
{
    kFlexSpi_Nand_Cs_Interval_100Ns = 0,
    kFlexSpi_Nand_Cs_Interval_200Ns = 1,
    kFlexSpi_Nand_Cs_Interval_400Ns = 2,
    kFlexSpi_Nand_Cs_Interval_50Ns = 3,
};

/*
 * Fuse definition for the Column address width
 */
enum
{
    kFlexSpi_Nand_Cs_Column_Addr_Width_12Bit = 0,
    kFlexSpi_Nand_Cs_Column_Addr_Width_13Bit = 1,
    kFlexSpi_Nand_Cs_Column_Addr_Width_14Bit = 2,
    kFlexSpi_Nand_Cs_Column_Addr_Width_15Bit = 3,
};

/*
 *  Common Serial NAND commands
 */
enum
{
    kSerialNAND_Command_PageRead = 0x13,
    kSerialNAND_Command_ReadCache = 0x03,
    kSerialNAND_Command_WriteEnable = 0x06,
    kSerialNAND_Command_ProgramLoad = 0x02,
    kSerialNAND_Command_ProtramExecute = 0x10,
    kSerialNAND_Command_ReadStatus = 0x0F,
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
// Default Read Status command(Get Feature)
static const uint32_t s_read_status_lut[4] = {
    FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, FLEXSPI_1PAD, 0x0F, kFLEXSPI_Command_SDR, FLEXSPI_1PAD, 0xC0),
    FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, FLEXSPI_1PAD, 0x01, kFLEXSPI_Command_STOP, FLEXSPI_1PAD, 0), 0, 0
};

// Default Read ECC Status
static const uint32_t s_read_ecc_status_lut[4] = {
    FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, FLEXSPI_1PAD, 0x0F, kFLEXSPI_Command_SDR, FLEXSPI_1PAD, 0xC0),
    FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, FLEXSPI_1PAD, 0x01, kFLEXSPI_Command_STOP, FLEXSPI_1PAD, 0), 0, 0
};
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

mixspi_root_clk_freq_t flexspi_convert_clock_for_ddr(mixspi_root_clk_freq_t freq)
{
    switch(freq)
    {

        case kMixspiRootClkFreq_50MHz:
            return kMixspiRootClkFreq_100MHz; 
        case kMixspiRootClkFreq_60MHz:
            return kMixspiRootClkFreq_120MHz; 
        case kMixspiRootClkFreq_80MHz:
            return kMixspiRootClkFreq_166MHz; 
        case kMixspiRootClkFreq_100MHz:
            return kMixspiRootClkFreq_200MHz; 
        case kMixspiRootClkFreq_120MHz:
            return kMixspiRootClkFreq_240MHz; 
        case kMixspiRootClkFreq_133MHz:
            return kMixspiRootClkFreq_266MHz; 
        case kMixspiRootClkFreq_166MHz:
            return kMixspiRootClkFreq_332MHz; 
        case kMixspiRootClkFreq_200MHz:
            return kMixspiRootClkFreq_400MHz;
        case kMixspiRootClkFreq_30MHz:
        default:
            return kMixspiRootClkFreq_60MHz;
    }
}

status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
// This is an example that shows how to override the default pad setting in ROM, for now, the pad setting in ROM is
// idential to below values
// So, below codes are not required.
#if 0
        // See IOMUXC pad setting definitions for more details.
        config->controllerMiscOption |= (1<<kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride = 0x130f1;
        config->sclkPadSettingOverride = 0x10f1;
        config->csPadSettingOverride = 0x10f1;
        config->dataPadSettingOverride = 0x10f1;
#endif
        if (config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            if (config->controllerMiscOption & (1 << kFlexSpiMiscOffset_DdrModeEnable))
            {
                config->dataValidTime[0].time_100ps = 15; // 1.5 ns // 1/4 * cycle of 166MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 30; // 3 ns // 1/2 * cycle of 166MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}

// Get max supported Frequency in this SoC
status_t flexspi_get_max_supported_freq(FLEXSPI_Type *base, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((base != NULL) || (freq == NULL))
        {
            break;
        }

        if (kFlexSpiClk_DDR == clkMode)
        {
            *freq = (166UL * 1000 * 1000);
        }
        else
        {
            *freq = (166UL * 1000 * 1000);
        }

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_nand_get_default_cfg_blk(flexspi_nand_config_t *config)
{
    uint32_t s_read_from_cache_lut[4] = { 0 };
    uint32_t s_page_read_lut[4] = { 0 };
    uint32_t columnAddrWidth = 12;
    uint32_t readCacheCommand = kSerialNAND_Command_ReadCache;
    uint32_t readPageCommand = kSerialNAND_Command_PageRead;

    flexspi_mem_config_t *memCfg = &config->memConfig;
    flexspi_nand_config_t *nandCfg = config;

    memset(memCfg, 0, sizeof(flexspi_mem_config_t));

    memCfg->tag = FLEXSPI_CFG_BLK_TAG;
    memCfg->version = FLEXSPI_CFG_BLK_VERSION;
    memCfg->deviceType = kFlexSpiDeviceType_SerialNAND;

    // Get safe frequency for Serial NAND.
    memCfg->serialClkFreq = kMixspiRootClkFreq_30MHz;

    // Configure default size of Serial NAND.
    // Configured size = actual size * 2
    memCfg->sflashA1Size = 128UL * 1024 * 1024 * 2; // 128M

    memCfg->csHoldTime = 3;
    memCfg->csSetupTime = 3;
    memCfg->sflashPadType = kSerialFlash_1Pad;
    memCfg->timeoutInMs = 100;

    // Get LUT for Cache read
    s_read_from_cache_lut[0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, FLEXSPI_1PAD, readCacheCommand, kFLEXSPI_Command_CADDR_SDR, FLEXSPI_1PAD, 0x10);
    s_read_from_cache_lut[1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_SDR, FLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, FLEXSPI_1PAD, 0x04);

    // Get LUT for Page Read
    s_page_read_lut[0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, FLEXSPI_1PAD, readPageCommand, kFLEXSPI_Command_RADDR_SDR, FLEXSPI_1PAD, 0x18);

    // Get Column Address Width
    uint8_t tempOpt = kFlexSpi_Nand_Cs_Column_Addr_Width_12Bit;
    switch (tempOpt)
    {
        default:
        case kFlexSpi_Nand_Cs_Column_Addr_Width_12Bit:
            columnAddrWidth = 12;
            break;
        case kFlexSpi_Nand_Cs_Column_Addr_Width_13Bit:
            columnAddrWidth = 13;
            break;
        case kFlexSpi_Nand_Cs_Column_Addr_Width_14Bit:
            columnAddrWidth = 14;
            break;
        case kFlexSpi_Nand_Cs_Column_Addr_Width_15Bit:
            columnAddrWidth = 15;
            break;
    }

    memCfg->columnAddressWidth = columnAddrWidth;
    nandCfg->pageTotalSize = 1 << columnAddrWidth;
    nandCfg->pageDataSize = 1 << (columnAddrWidth - 1);
    memcpy(memCfg->lookupTable, s_read_from_cache_lut, sizeof(s_read_from_cache_lut));
    memcpy(&memCfg->lookupTable[4 * NAND_CMD_LUT_SEQ_IDX_READPAGE], s_page_read_lut, sizeof(s_page_read_lut));

    // Get LUT for Read Status if it is not bypassed.
    memcpy(&memCfg->lookupTable[4 * NAND_CMD_LUT_SEQ_IDX_READSTATUS], s_read_status_lut, sizeof(s_read_status_lut));

    // Get LUT for ECC read if it is not bypassed.
    nandCfg->bypassEccRead = false;
    memcpy(&memCfg->lookupTable[4 * NAND_CMD_LUT_SEQ_IDX_READECCSTAT], s_read_ecc_status_lut,
           sizeof(s_read_ecc_status_lut));

    // Get the command interval
    tempOpt = kFlexSpi_Nand_Cs_Interval_100Ns;
    switch (tempOpt)
    {
        default:
        case kFlexSpi_Nand_Cs_Interval_100Ns:
            memCfg->commandInterval = 100;
            break;
        case kFlexSpi_Nand_Cs_Interval_200Ns:
            memCfg->commandInterval = 200;
            break;
        case kFlexSpi_Nand_Cs_Interval_400Ns:
            memCfg->commandInterval = 400;
            break;
        case kFlexSpi_Nand_Cs_Interval_50Ns:
            memCfg->commandInterval = 50;
            break;
    }

    return kStatus_Success;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
