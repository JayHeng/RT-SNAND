/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bl_flexspi.h"
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
    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x0F, CMD_SDR, FLEXSPI_1PAD, 0xC0),
    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x01, STOP, FLEXSPI_1PAD, 0), 0, 0
};

// Default Read ECC Status
static const uint32_t s_read_ecc_status_lut[4] = {
    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x0F, CMD_SDR, FLEXSPI_1PAD, 0xC0),
    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x01, STOP, FLEXSPI_1PAD, 0), 0, 0
};
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//!@brief Configure clock for FlexSPI peripheral
void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
{
    uint32_t pfd480 = 0;
    uint32_t cscmr1 = 0;
    uint32_t frac = 0;
    uint32_t podf = 0;

    typedef struct _flexspi_clock_param
    {
        uint8_t frac;
        uint8_t podf;
    } flexspi_clock_param_t;

    const flexspi_clock_param_t k_sdr_clock_config[kMixspiRootClkFreq_200MHz + 1] = {
        // Reserved, 30MHz     50MHz     60MHz        75MHz    80MHz       100MHz   133MHz       166MHz   200MHz
        { 0, 0 }, { 34, 8 }, { 22, 8 }, { 24, 6 }, { 30, 4 }, { 18, 6 }, { 14, 6 }, { 17, 4 }, { 26, 2 }, { 22, 2 }
    };
    const flexspi_clock_param_t k_ddr_clock_config[kMixspiRootClkFreq_200MHz + 1] = {
        // Reserved, 30MHz,  50MHz,       60MHz,      75MHz,   80Mhz,   100MHz,      133MHz,   166MHz,     200MHz
        { 0, 0 }, { 24, 6 }, { 22, 4 }, { 12, 6 }, { 30, 2 }, { 18, 3 }, { 22, 2 }, { 33, 1 }, { 26, 1 }, { 22, 1 }
    };

    do
    {
        if ((sampleClkMode != kFlexSpiClk_SDR) && (sampleClkMode != kFlexSpiClk_DDR))
        {
            break;
        }

        pfd480 = CCM_ANALOG->PFD_480 & (~CCM_ANALOG_PFD_480_PFD0_FRAC_MASK);
        cscmr1 = CCM->CSCMR1 & (~CCM_CSCMR1_FLEXSPI_PODF_MASK);

        // Note: Per ANALOG IP Owner's recommendation, FRAC should be even number,
        //       PODF should be even nubmer as well if the divider is greater than 1

        const flexspi_clock_param_t *flexspi_config_array = NULL;
        if (sampleClkMode == kFlexSpiClk_SDR)
        {
            flexspi_config_array = &k_sdr_clock_config[0];
        }
        else
        {
            flexspi_config_array = &k_ddr_clock_config[0];
        }

        if (freq >= kMixspiRootClkFreq_30MHz)
        {
            if (freq > kMixspiRootClkFreq_200MHz)
            {
                freq = kMixspiRootClkFreq_30MHz;
            }

            frac = flexspi_config_array[freq].frac;
            podf = flexspi_config_array[freq].podf;

            pfd480 |= CCM_ANALOG_PFD_480_PFD0_FRAC(frac);
            cscmr1 |= CCM_CSCMR1_FLEXSPI_PODF(podf - 1);

            FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            flexspi_clock_gate_disable(instance);

            if (pfd480 != CCM_ANALOG->PFD_480)
            {
                CCM_ANALOG->PFD_480 = pfd480;
            }
            if (cscmr1 != CCM->CSCMR1)
            {
                CCM->CSCMR1 = cscmr1;
            }
            flexspi_clock_gate_enable(instance);
            FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
        }
        else
        {
            // Do nothing
        }
    } while (0);
}

//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable(uint32_t instance)
{
    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable(uint32_t instance)
{
    CCM->CCGR6 &= (uint32_t)~CCM_CCGR6_CG5_MASK;
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
status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((instance != 0) || (freq == NULL))
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

//!@brief Get Clock for FlexSPI peripheral
status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    uint32_t ahbBusDivider;
    uint32_t seralRootClkDivider;
    uint32_t arm_clock = SystemCoreClock;

    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = SystemCoreClock;
            break;
        case kFlexSpiClock_AhbClock:
        {
            // Note: In I.MXRT_512, actual AHB clock is IPG_CLOCK_ROOT
            ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
            clockFrequency = arm_clock / ahbBusDivider;
        }
        break;
        case kFlexSpiClock_SerialRootClock:
        {
            uint32_t pfdFrac;
            uint32_t pfdClk;

            // FLEXPI CLK SEL
            uint32_t flexspi_clk_src =
                (CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK) >> CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT;

            // PLL_480_PFD0
            pfdFrac = (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT;
            pfdClk = FREQ_480MHz / pfdFrac * 18;

            seralRootClkDivider = ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_PODF_MASK) >> CCM_CSCMR1_FLEXSPI_PODF_SHIFT) + 1;

            clockFrequency = pfdClk / seralRootClkDivider;
        }
        break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
    *freq = clockFrequency;

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
    s_read_from_cache_lut[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, readCacheCommand, CADDR_SDR, FLEXSPI_1PAD, 0x10);
    s_read_from_cache_lut[1] = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_1PAD, 0x08, READ_SDR, FLEXSPI_1PAD, 0x04);

    // Get LUT for Page Read
    s_page_read_lut[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, readPageCommand, RADDR_SDR, FLEXSPI_1PAD, 0x18);

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
