/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <assert.h>
#include <stdbool.h>

#include "snand_flash.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FLEXSPI_LUT_KEY_VAL (0x5AF05AF0ul)    //!< FlesSPI Unlock/Lock Key
#define FLEXSPI_WAIT_TIMEOUT_NS (500000000UL) //!< FlexSPI timeout value, 500ms
#define FLEXSPI_FREQ_1GHz (1000000000UL)

#define FREQ_1MHz (1000000UL)
#define FLEXSPI_DLLCR_DEFAULT (0x100UL)

#define CMD_LUT_FOR_IP_CMD 1 //!< LUT sequence id for IP command

enum
{
    kFlexSpiDelayCellUnit_Min = 75,  // 75ps
    kFlexSpiDelayCellUnit_Max = 225, // 225ps
};

#define FLEXSPI_PINMUX_VAL 0x08

/*******************************************************************************
 * Local variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//!@brief Unlock LUT
static void flexspi_unlock_lut(FLEXSPI_Type *base);
//!@brief Lock LUT
static void flexspi_lock_lut(FLEXSPI_Type *base);
//!@brief Clear IP TXFIFO
static void flexspi_clear_ip_txfifo(FLEXSPI_Type *base);
//!@brief Clear IP RXFIO
static void flexspi_clear_ip_rxfifo(FLEXSPI_Type *base);
//!@brief Wait Until FlexSPI IP is idle
static void flexspi_wait_until_ip_idle(FLEXSPI_Type *base);
//!@brief Clear FlexSPI error status
static void flexspi_clear_error_status(FLEXSPI_Type *base);
//!@brief Get interval ticks based on provided interval in terms of nano-seconds, frequency and unit
status_t flexspi_get_ticks(uint32_t *ticks, uint32_t intervalNs, uint32_t freq, uint32_t unit);
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
//!@brief Extract received data under parallel mode
static status_t flexspi_extract_parallel_data(uint32_t *dst0, uint32_t *dst1, uint32_t *src, uint32_t length);
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE

//!@brief Configure Device workmode via FlexSPI
static status_t flexspi_device_workmode_config(FLEXSPI_Type *base, flexspi_mem_config_t *config, uint32_t baseAddr);
static status_t flexspi_device_workmode_config_all_chips(FLEXSPI_Type *base, flexspi_mem_config_t *config);
//!@brief Configure Device registers via FlexSPI
static status_t flexspi_device_cmd_config(FLEXSPI_Type *base, flexspi_mem_config_t *config, uint32_t baseAddr);
static status_t flexspi_device_cmd_config_all_chips(FLEXSPI_Type *base, flexspi_mem_config_t *config);

/*******************************************************************************
 * Codes
 ******************************************************************************/

static void flexspi_unlock_lut(FLEXSPI_Type *base)
{
    base->LUTKEY = FLEXSPI_LUT_KEY_VAL;
    base->LUTCR = 0x02;
}

static void flexspi_lock_lut(FLEXSPI_Type *base)
{
    base->LUTKEY = FLEXSPI_LUT_KEY_VAL;
    base->LUTCR = 0x01;
}

bool flexspi_is_parallel_mode(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_ParallelEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_padsetting_override_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_PadSettingOverrideEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_differential_clock_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_DiffClkEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_word_addressable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_WordAddressableEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_ck2_enabled(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_Ck2Enable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_ddr_mode_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//!@brief Configure DLL register
status_t flexspi_configure_dll(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    bool mdisConfigRequired;

    do
    {
        bool isUnifiedConfig = true;
        uint32_t flexspiDll[2] = { FLEXSPI_DLLCR_DEFAULT, FLEXSPI_DLLCR_DEFAULT };

        if ((base == NULL) || (config == NULL) ||
            (config->readSampleClkSrc > kFLEXSPI_ReadSampleClkExternalInputFromDqsPad))
        {
            break;
        }

        switch (config->readSampleClkSrc)
        {
            case kFLEXSPI_ReadSampleClkLoopbackInternally:
            case kFLEXSPI_ReadSampleClkLoopbackFromDqsPad:
            case kFLEXSPI_ReadSampleClkLoopbackFromSckPad:
                isUnifiedConfig = true;
                break;
            case kFLEXSPI_ReadSampleClkExternalInputFromDqsPad:
                if (flexspi_is_ck2_enabled(config))
                {
                    isUnifiedConfig = true;
                }
                else
                {
                    isUnifiedConfig = false;
                }
                break;
            default: // Never reach here
                break;
        }

        if (isUnifiedConfig)
        {
            flexspiDll[0] = FLEXSPI_DLLCR_DEFAULT; // 1 fixed delay cells in DLL delay chain)
            flexspiDll[1] = FLEXSPI_DLLCR_DEFAULT; // 1 fixed delay cells in DLL delay chain)
        }
#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
        else
        {
            uint32_t flexspiRootClk = mixspi_get_clock(base);
            uint32_t dllValue;
            uint32_t temp;
            bool useDLL = false;

            // See FlexSPI Chapter for more details
            if ((flexspiRootClk >= 100 * FREQ_1MHz) &&
                (!(config->controllerMiscOption & (1U << kFlexSpiMiscOffset_UseValidTimeForAllFreq))))
            {
                useDLL = true;
            }
            if (useDLL)
            {
                flexspiDll[0] = FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET(0x0F);
                flexspiDll[1] = FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET(0x0F);
            }
            // If Serial root closk is lower than 100MHz, DLL is unable to lock on
            // half cycle of serial root clock because the dealy cell number is limited
            // in delay chain, Then DLL should be configured as following instead:
            // OVRDEN = 0x01
            // OVRDVAL=N; each dealy cell in DLL is about 75ps - 225ps.
            // The delay of DLL delay chain ( N * delay_cell_delay) should be larger
            // than device output data valid time (from SCK edge to data valid).
            // The other condition is that some devices may be incompatible with current
            // FlexSPI defintions, so, use a backup way to support it.
            else
            {
                for (uint32_t i = 0; i < 2; i++)
                {
                    uint32_t dataValidTimeH = config->dataValidTime[i].delay_cells;
                    uint32_t dataValidTimeL = config->dataValidTime[i].time_100ps;
                    if (dataValidTimeH < 1)
                    {
                        // Convert the data valid time to n ps.
                        temp = dataValidTimeL * 100ul;
                        if (temp < 1)
                        {
                            uint32_t maxFreq;
                            bool is_ddr_enabled = flexspi_is_ddr_mode_enable(config);
                            flexspi_get_max_supported_freq(base, &maxFreq, is_ddr_enabled);
                            // For SDR mode, the delay cell configuration must ensure that the delay time is greater
                            // than
                            // Half cycle of max  supported frequency
                            if (!is_ddr_enabled)
                            {
                                dllValue = FLEXSPI_FREQ_1GHz / maxFreq / 2 * 1000 / kFlexSpiDelayCellUnit_Min + 1;
                            }
                            // For SDR mode, the delay cell configuration must ensure that the delay time is greater
                            // than
                            // 1/4 cycle of max supported frequency
                            else
                            {
                                dllValue = FLEXSPI_FREQ_1GHz / maxFreq / 4 * 1000 / kFlexSpiDelayCellUnit_Min + 1;
                            }
                        }
                        else
                        {
                            dllValue = temp / kFlexSpiDelayCellUnit_Min;
                            if (dllValue * kFlexSpiDelayCellUnit_Min < temp)
                            {
                                dllValue++;
                            }
                        }
                    }
                    else
                    {
                        dllValue = dataValidTimeH;
                    }
                    // Calculate maximum dll value;
                    temp = (FLEXSPI_DLLCR_OVRDVAL_MASK >> FLEXSPI_DLLCR_OVRDVAL_SHIFT);
                    if (dllValue > temp)
                    {
                        dllValue = temp;
                    }
                    flexspiDll[i] = FLEXSPI_DLLCR_OVRDEN(1) | FLEXSPI_DLLCR_OVRDVAL(dllValue);
                }
            }
        }
#endif // FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT

        if (base->MCR0 & FLEXSPI_MCR0_MDIS_MASK)
        {
            mdisConfigRequired = false;
        }
        else
        {
            mdisConfigRequired = true;
        }

        if (mdisConfigRequired)
        {
            base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
        }

        if (config->sflashA1Size || config->sflashA2Size)
        {
            base->DLLCR[0] = flexspiDll[0];
        }
        if (config->sflashB1Size || config->sflashB2Size)
        {
            base->DLLCR[1] = flexspiDll[1];
        }

        // Override the DLLCR setting by Customizable value   
        if (config->dll0CrVal)
        {
            base->DLLCR[0] =  config->dll0CrVal;
        }

        if (config->dll1CrVal)
        {
            base->DLLCR[1] =  config->dll1CrVal;
        }
        
        if (mdisConfigRequired)
        {
            base->MCR0 &= (uint32_t)~FLEXSPI_MCR0_MDIS_MASK;
        }

        status = kStatus_Success;
    } while (0);

    return status;
}

// Calculate ticks for the timeout interms of specified unit.
status_t flexspi_get_ticks(uint32_t *ticks, uint32_t intervalNs, uint32_t freq, uint32_t unit)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((ticks == NULL) || (freq < 1) || (unit < 1))
        {
            break;
        }

        // Get clock cycle in terms of ns
        int32_t calculatedTicks;
        uint32_t cycleNs = FLEXSPI_FREQ_1GHz / freq;

        calculatedTicks = intervalNs / (cycleNs * unit);
        while (calculatedTicks * cycleNs * unit < intervalNs)
        {
            calculatedTicks++;
        }

        *ticks = calculatedTicks;

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_config_mcr1(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    uint32_t seqWaitTicks = 0xFFFFu;
    uint32_t ahbBusWaitTicks = 0xFFFFu;
    uint32_t serialRootClockFreq;
    uint32_t ahbBusClockFreq;

    if ((base == NULL) || (config == NULL))
    {
        return kStatus_InvalidArgument;
    }
    serialRootClockFreq = mixspi_get_clock(base);
    ahbBusClockFreq = cpu_get_ahb_clock();
    flexspi_get_ticks(&seqWaitTicks, FLEXSPI_WAIT_TIMEOUT_NS, serialRootClockFreq, 1024);
    flexspi_get_ticks(&ahbBusWaitTicks, FLEXSPI_WAIT_TIMEOUT_NS, ahbBusClockFreq, 1024);

    if (seqWaitTicks > 0xFFFF)
    {
        seqWaitTicks = 0xFFFF;
    }
    if (ahbBusWaitTicks > 0xFFFF)
    {
        ahbBusWaitTicks = 0xFFFF;
    }

    // Configure MCR1
    base->MCR1 = FLEXSPI_MCR1_SEQWAIT(seqWaitTicks) | FLEXSPI_MCR1_AHBBUSWAIT(ahbBusWaitTicks);

    return kStatus_Success;
}

static void flexspi_clear_ip_txfifo(FLEXSPI_Type *base)
{
    base->IPTXFCR |= FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
    while (base->IPTXFCR & FLEXSPI_IPTXFCR_CLRIPTXF_MASK)
    {
    }
}

static void flexspi_clear_ip_rxfifo(FLEXSPI_Type *base)
{
    base->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
    while (base->IPRXFCR & FLEXSPI_IPRXFCR_CLRIPRXF_MASK)
    {
    }
}

static void flexspi_wait_until_ip_idle(FLEXSPI_Type *base)
{
    while ((base->STS0 & FLEXSPI_STS0_SEQIDLE_MASK) != FLEXSPI_STS0_SEQIDLE_MASK)
    {
    }
    while ((base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK) != FLEXSPI_STS0_ARBIDLE_MASK)
    {
    }
}

void flexspi_clear_error_status(FLEXSPI_Type *base)
{
    base->INTR |= FLEXSPI_INTR_AHBCMDERR_MASK | FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_AHBCMDGE_MASK |
                  FLEXSPI_INTR_IPCMDGE_MASK;
}

status_t flexspi_config_flash_control_registers(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        uint32_t index;
        uint32_t flashSize;
        uint32_t temp;
        uint32_t serialClockFrequency;
        uint32_t csIntervalTicks = 0;

        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        uint32_t *flashSizeStart = (uint32_t *)&config->sflashA1Size;

        for (index = 0; index < FLEXSPI_FLSHCR0_COUNT; index++)
        {
            // Configure FLSHCR0
            flashSize = *flashSizeStart++;
#if defined(XIP_FLASH_TO_ACCESS_NAND)
            if (!flashSize)
            {
                continue;
            }
#endif
            base->FLSHCR0[index] = flashSize / 1024;

            // Configure FLSHCR1
            temp = FLEXSPI_FLSHCR1_TCSS(config->csSetupTime) | FLEXSPI_FLSHCR1_TCSH(config->csHoldTime) |
                   FLEXSPI_FLSHCR1_CAS(config->columnAddressWidth);
            if (flexspi_is_word_addressable(config))
            {
                temp |= FLEXSPI_FLSHCR1_WA_MASK;
            }
            // Calculate CS interval
            if (config->commandInterval)
            {
                serialClockFrequency = mixspi_get_clock(base);
                flexspi_get_ticks(&csIntervalTicks, config->commandInterval, serialClockFrequency, 1);

                temp |= FLEXSPI_FLSHCR1_CSINTERVAL(csIntervalTicks);
            }
            base->FLSHCR1[index] = temp;

            // Configure FLSHCR2
            temp = FLEXSPI_FLSHCR2_ARDSEQID(NAND_CMD_LUT_SEQ_IDX_READCACHE);

            if (config->lutCustomSeqEnable && FLEXSPI_FLSHCR2_ARDSEQNUM(config->lutCustomSeq[NAND_CMD_INDEX_PROGRAMLOAD].seqNum))
            {
                temp |= FLEXSPI_FLSHCR2_AWRSEQID(config->lutCustomSeq[NAND_CMD_INDEX_PROGRAMLOAD].seqId);
                temp |= FLEXSPI_FLSHCR2_AWRSEQNUM(config->lutCustomSeq[NAND_CMD_INDEX_PROGRAMLOAD].seqNum - 1);
            }
            else
            {
                temp |= FLEXSPI_FLSHCR2_AWRSEQID(NAND_CMD_LUT_SEQ_IDX_PROGRAMLOAD);
            }
            base->FLSHCR2[index] = temp;
        }

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_config_ahb_buffers(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    uint32_t temp;
    uint32_t index;
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        if (config->deviceType == kFlexSpiDeviceType_SerialNOR)
        {
#if FLEXSPI_AHBCR_APAREN_MASK
            // Configure AHBCR
            temp = base->AHBCR & (~FLEXSPI_AHBCR_APAREN_MASK);
#else
            temp = base->AHBCR;
#endif
            // Remove alignment limitation when Flash device works under DDR mode.
            temp |= FLEXSPI_AHBCR_READADDROPT_MASK;
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE && FLEXSPI_AHBCR_APAREN_MASK
            if (flexspi_is_parallel_mode(config))
            {
                temp |= FLEXSPI_AHBCR_APAREN_MASK;
            }
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
            base->AHBCR = temp;
        }
        // Enable prefetch feature
        base->AHBCR |= FLEXSPI_AHBCR_PREFETCHEN_MASK;

        // Configure AHB RX buffer
        for (index = 0; index < FLEXSPI_AHBRXBUFCR0_COUNT - 1; index++)
        {
            base->AHBRXBUFCR0[index] &=
                ~(FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK | FLEXSPI_AHBRXBUFCR0_MSTRID_MASK | FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK);
        }
        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_device_write_enable(FLEXSPI_Type *base,
                                     flexspi_mem_config_t *config,
                                     bool isParallelMode,
                                     uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
        flexspi_xfer_t flashXfer;
        flashXfer.operation = kFlexSpiOperation_Command;
        flashXfer.seqNum = 1;
        flashXfer.seqId = NAND_CMD_LUT_SEQ_IDX_WRITEENABLE;
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        flashXfer.isParallelModeEnable = isParallelMode;
#else
        flashXfer.isParallelModeEnable = false;
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        flashXfer.baseAddress = baseAddr;

        if (config->lutCustomSeqEnable && config->lutCustomSeq[NAND_CMD_INDEX_WRITEENABLE].seqNum)
        {
            flashXfer.seqId = config->lutCustomSeq[NAND_CMD_INDEX_WRITEENABLE].seqId;
            flashXfer.seqNum = config->lutCustomSeq[NAND_CMD_INDEX_WRITEENABLE].seqNum;
        }
        flexspi_update_lut(base, CMD_LUT_FOR_IP_CMD, &config->lookupTable[4 * flashXfer.seqId], flashXfer.seqNum);
        flashXfer.seqId = CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(base, &flashXfer);

    } while (0);

    return status;
}

status_t flexspi_device_workmode_config(FLEXSPI_Type *base, flexspi_mem_config_t *config, uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        // If device is working under DPI/QPI/OPI mode, ignore SPI2XPI command
        uint32_t read_cmd_pads = (base->LUT[0] >> 8) & 0x03;
        if ((read_cmd_pads > kFLEXSPI_1PAD) && (config->deviceModeType == kDeviceConfigCmdType_Spi2Xpi))
        {
            status = kStatus_Success;
            break;
        }

        flexspi_xfer_t flashXfer;
        flashXfer.operation = kFlexSpiOperation_Config;
        flashXfer.baseAddress = baseAddr;
        flashXfer.seqId = config->deviceModeSeq.seqId;
        flashXfer.seqNum = config->deviceModeSeq.seqNum;
        flashXfer.isParallelModeEnable = false;
        flashXfer.txBuffer = &config->deviceModeArg;
        flashXfer.txSize = 4;

        flexspi_device_write_enable(base, config, false, baseAddr);
        // Update LUT 1 for device mode config command
        flexspi_update_lut(base, CMD_LUT_FOR_IP_CMD, &config->lookupTable[4 * flashXfer.seqId], flashXfer.seqNum);
        flashXfer.seqId = CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(base, &flashXfer);
        if (status != kStatus_Success)
        {
            break;
        }
        if ((!config->waitTimeCfgCommands) && (config->deviceModeType != (uint8_t)kDeviceConfigCmdType_Spi2Xpi) &&
            (config->deviceModeType != (uint8_t)kDeviceConfigCmdType_Xpi2Spi))
        {
            status = flexspi_device_wait_busy(base, config, false, baseAddr);
        }
        else
        {
            mixspi_sw_delay_us(config->waitTimeCfgCommands * 100UL);
        }
    } while (0);

    return status;
}
status_t flexspi_device_workmode_config_all_chips(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (config == NULL)
        {
            break;
        }
        uint32_t baseAddr = 0;
        uint32_t index;
        uint32_t currentFlashSize;
        uint32_t *flashSizeStart = &config->sflashA1Size;

        for (index = 0; index < 4; index++)
        {
            currentFlashSize = *flashSizeStart++;
            if (currentFlashSize > 0)
            {
                status = flexspi_device_workmode_config(base, config, baseAddr);
                if (status != kStatus_Success)
                {
                    break;
                }
                baseAddr += currentFlashSize;
            }
        }

    } while (0);

    return status;
}

status_t flexspi_device_cmd_config(FLEXSPI_Type *base, flexspi_mem_config_t *config, uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        flexspi_xfer_t flashXfer;
        uint32_t index;
        flashXfer.operation = kFlexSpiOperation_Config;
        flashXfer.baseAddress = baseAddr;
        flashXfer.isParallelModeEnable = false;
        flashXfer.txSize = 4;

        for (index = 0; index < 3; index++)
        {
            if (config->configCmdSeqs[index].seqId > 0)
            {
                // If device is working under DPI/QPI/OPI mode, ignore SPI2XPI command
                uint32_t read_cmd_pads = (base->LUT[0] >> 8) & 0x03;
                if ((read_cmd_pads > kFLEXSPI_1PAD) && (config->configModeType[index] == kDeviceConfigCmdType_Spi2Xpi))
                {
                    continue;
                }

                flashXfer.seqId = config->configCmdSeqs[index].seqId;
                flashXfer.seqNum = config->configCmdSeqs[index].seqNum;
                flashXfer.txBuffer = &config->configCmdArgs[index];

                status = flexspi_device_write_enable(base, config, false, baseAddr);
                if (status != kStatus_Success)
                {
                    return status;
                }

                flexspi_update_lut(base, CMD_LUT_FOR_IP_CMD, &config->lookupTable[4 * flashXfer.seqId],
                                   flashXfer.seqNum);
                flashXfer.seqId = CMD_LUT_FOR_IP_CMD;
                status = flexspi_command_xfer(base, &flashXfer);
                if (status != kStatus_Success)
                {
                    return status;
                }

                if ((!config->waitTimeCfgCommands) &&
                    (config->configModeType[index] != (uint8_t)kDeviceConfigCmdType_Spi2Xpi) &&
                    (config->configModeType[index] != (uint8_t)kDeviceConfigCmdType_Xpi2Spi))
                {
                    status = flexspi_device_wait_busy(base, config, false, baseAddr);

                    if (status != kStatus_Success)
                    {
                        return status;
                    }
                }
                else
                {
                    mixspi_sw_delay_us(config->waitTimeCfgCommands * 100UL);
                }
            }
        }
    } while (0);

    return status;
}

status_t flexspi_device_cmd_config_all_chips(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (config == NULL)
        {
            break;
        }
        uint32_t baseAddr = 0;
        uint32_t index;
        uint32_t currentFlashSize;
        uint32_t *flashSizeStart = &config->sflashA1Size;

        for (index = 0; index < 4; index++)
        {
            currentFlashSize = *flashSizeStart++;
            if (currentFlashSize > 0)
            {
                status = flexspi_device_cmd_config(base, config, baseAddr);
                if (status != kStatus_Success)
                {
                    break;
                }
                baseAddr += currentFlashSize;
            }
        }
    } while (0);

    return status;
}

/* Common FlexSPI config */
flexspi_device_config_t g_deviceconfig = {
    .flexspiRootClk       = 27400000,
    .flashSize            = 0x4000, /* 128Mb/KByte */
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 2,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};

status_t flexspi_init2(FLEXSPI_Type *base, flexspi_mem_config_t *memConfig)
{
    flexspi_config_t config;

    mixspi_clock_init(base, kMixspiRootClkFreq_30MHz);

    /*Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&config);

    /*Set AHB buffer size for reading data through AHB bus. */
    config.ahbConfig.enableAHBPrefetch = true;
    config.rxSampleClock               = (flexspi_read_sample_clock_t)memConfig->readSampleClkSrc;
#if !(defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN) && FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN)
    flexspi_port_t port = EXAMPLE_MIXSPI_PORT;
    if ((port == kFLEXSPI_PortA1) || (port == kFLEXSPI_PortA2))
    {
        config.enableCombination = true;
    }
    else
    {
        config.enableCombination = false;
    }
#endif
    config.ahbConfig.enableAHBBufferable = true;
    config.ahbConfig.enableAHBCachable   = true;

    FLEXSPI_Init(base, &config);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(base, &g_deviceconfig, EXAMPLE_MIXSPI_PORT);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

#if defined(EXAMPLE_INVALIDATE_FLEXSPI_CACHE)
    EXAMPLE_INVALIDATE_FLEXSPI_CACHE();
#endif
    
    return kStatus_Success;
}

#if !defined(XIP_FLASH_TO_ACCESS_NAND)
status_t flexspi_init(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    uint32_t mcr0;
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        /* Determine if the flexspi serial clock should be configured to safe frequency
         *
         *  ROM configures the FlexSPI Serial clock to 30MHz if one of below case is met:
         *  1. ROM is required to configure external device to certain modes at safe frequency
         */
        bool need_safe_freq = (config->deviceModeCfgEnable || config->configCmdEnable) &&
                              ((config->controllerMiscOption & (1 << kFlexSpiMiscOffset_SafeConfigFreqEnable)));

        /*
         *  !!! Important !!!
         *  The module clock must be disabled during clock switch in order to avoid glitch
         */
        // Do not re-configure FLEXSPI IP when it is busy
        if (is_mixspi_clock_enabled(base))
        {
            flexspi_wait_idle(base);
        }
        mixspi_clock_gate_disable(base);
        //flexspi_iomux_config(base, config);
        if (need_safe_freq)
        {
            // Configure FlexSPI serial clock using safe frequency
            mixspi_clock_init(base, (mixspi_root_clk_freq_t)kFlexSpiSerialClk_SafeFreq);
        }
        else
        {
            // Configure FlexSPI serial clock with specified frequency
            if (flexspi_is_ddr_mode_enable(config))
            {
                mixspi_clock_init(base, flexspi_convert_clock_for_ddr((mixspi_root_clk_freq_t)config->serialClkFreq));
            }
            else
            {
                mixspi_clock_init(base, (mixspi_root_clk_freq_t)config->serialClkFreq);
            }
        }
        // Enable FlexSPI Clock Gate
        mixspi_clock_gate_enable(base);

        base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
        FLEXSPI_SoftwareReset(base);

        // Disable FlexSPI module during configuring control registers.
        base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;

        // Set Clock divider and sample clock source
        mcr0 = base->MCR0 &
               (uint32_t) ~(FLEXSPI_MCR0_RXCLKSRC_MASK | FLEXSPI_MCR0_IPGRANTWAIT_MASK | FLEXSPI_MCR0_AHBGRANTWAIT_MASK
#if !defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN) || (FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN == 0)
                            | FLEXSPI_MCR0_COMBINATIONEN_MASK
#endif
#if !defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_ATDFEN) || (FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_ATDFEN == 0)
                            | FLEXSPI_MCR0_ATDFEN_MASK
#else
                            | (0x80u)
#endif
#if !defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_ARDFEN) || (FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_ARDFEN == 0)
                            | FLEXSPI_MCR0_ARDFEN_MASK
#else
                            | (0x40u)
#endif
               );

#if defined(FLEXSPI_MCR0_DOZEEN_MASK)
        //mcr0 |= FLEXSPI_MCR0_DOZEEN_MASK;
#endif
#if defined(FLEXSPI_MCR0_LEARNEN_MASK)
        //mcr0 &= ~FLEXSPI_MCR0_LEARNEN_MASK;
#endif

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        // If this condition meets, it means FlexSPI PORT B exists, the the 8 bit is supported by combining PORTA[3:0]
        // with PORTB[3:0]
        if ((sizeof(base->FLSHCR1) / sizeof(base->FLSHCR1[0])) > 2)
        {
            // Enable Combined mode if Serial FLASH works using Octal pad instructions.
            if (config->sflashPadType == kFLEXSPI_8PAD)
            {
#if !defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN) || (FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN == 0)
                mcr0 |= FLEXSPI_MCR0_COMBINATIONEN_MASK;
#endif
            }
        }
#endif

        // Configure AHBGRANTWAIT and IPGRANTWAIT
        mcr0 |= FLEXSPI_MCR0_IPGRANTWAIT_MASK | FLEXSPI_MCR0_AHBGRANTWAIT_MASK;
        // Configure Read sample clock source
        flexspi_port_t thisPort = EXAMPLE_MIXSPI_PORT;
        if ((thisPort == kFLEXSPI_PortA1) || (thisPort == kFLEXSPI_PortA2))
        {
            mcr0 |= FLEXSPI_MCR0_RXCLKSRC(config->readSampleClkSrc);
        }
        base->MCR0 = mcr0;

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        // Configure MCR1
        flexspi_config_mcr1(base, config);
#endif

        // Configure MCR2
        base->MCR2 &= ~FLEXSPI_MCR2_SAMEDEVICEEN_MASK;

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT && FLEXSPI_MCR2_SCKBDIFFOPT_MASK
        // If this condition meets, it means FlexSPI PORT B exists, SCKB pads used as PORTB SCK with be used a inverted
        // SCK for PORTA
        // Enable differential clock as needed.
        if ((sizeof(base->FLSHCR1) / sizeof(base->FLSHCR1[0])) > 2)
        {
            if (flexspi_is_differential_clock_enable(config))
            {
                base->MCR2 |= FLEXSPI_MCR2_SCKBDIFFOPT_MASK;
            }
        }
#endif
        if ((thisPort == kFLEXSPI_PortB1) || (thisPort == kFLEXSPI_PortB2))
        {
            uint32_t configValue = base->MCR2;
            configValue |= 
#if defined(FSL_FEATURE_FLEXSPI_SUPPORT_SEPERATE_RXCLKSRC_PORTB) && FSL_FEATURE_FLEXSPI_SUPPORT_SEPERATE_RXCLKSRC_PORTB
                           FLEXSPI_MCR2_RXCLKSRC_B(config->readSampleClkSrc) |
#endif
#if defined(FSL_FEATURE_FLEXSPI_SUPPORT_RXCLKSRC_DIFF) && FSL_FEATURE_FLEXSPI_SUPPORT_RXCLKSRC_DIFF
                           FLEXSPI_MCR2_RX_CLK_SRC_DIFF(1) |
#endif
                           0x0;
            base->MCR2 = configValue;
        }

        // Configure AHB buffer
        flexspi_config_ahb_buffers(base, config);

        // Configure Flash related control registers
        flexspi_config_flash_control_registers(base, config);

        // Configure DLLCR
        flexspi_configure_dll(base, config);

        // Enable FlexSPI before updating LUT.
        base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

        // Reset all registers except control registers
        FLEXSPI_SoftwareReset(base);

        if (config->deviceModeCfgEnable)
        {
            status = flexspi_device_workmode_config_all_chips(base, config);
            if (status != kStatus_Success)
            {
                break;
            }
        }

        if (config->configCmdEnable)
        {
            status = flexspi_device_cmd_config_all_chips(base, config);
            if (status != kStatus_Success)
            {
                break;
            }
        }

        // Restore clock
        if (need_safe_freq)
        {
            /*
             *  !!! Important !!!
             *  The module clock must be disabled during clock switch in order to avoid glitch
             */
            base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            // Re-configure FlexSPI Serial clock frequency in order to acheive high performance.
            if (flexspi_is_ddr_mode_enable(config))
            {
                mixspi_clock_init(base, flexspi_convert_clock_for_ddr((mixspi_root_clk_freq_t)config->serialClkFreq));
            }
            else
            {
                mixspi_clock_init(base, (mixspi_root_clk_freq_t)config->serialClkFreq);
            }
#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
            // Re-Configure MCR1
            flexspi_config_mcr1(base, config);
#endif
            // Re-configure DLLCR
            flexspi_configure_dll(base, config);
            base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
        }

        status = kStatus_Success;

    } while (0);

    return status;
}
#else
status_t flexspi_init(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        // Configure MCR2
        base->MCR2 &= ~FLEXSPI_MCR2_SAMEDEVICEEN_MASK;

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT && FLEXSPI_MCR2_SCKBDIFFOPT_MASK
        // If this condition meets, it means FlexSPI PORT B exists, SCKB pads used as PORTB SCK with be used a inverted
        // SCK for PORTA
        // Enable differential clock as needed.
        if ((sizeof(base->FLSHCR1) / sizeof(base->FLSHCR1[0])) > 2)
        {
            if (flexspi_is_differential_clock_enable(config))
            {
                base->MCR2 |= FLEXSPI_MCR2_SCKBDIFFOPT_MASK;
            }
        }
#endif
        flexspi_port_t thisPort = EXAMPLE_MIXSPI_PORT;
        if ((thisPort == kFLEXSPI_PortB1) || (thisPort == kFLEXSPI_PortB2))
        {
            uint32_t configValue = base->MCR2;
            configValue |= 
#if defined(FSL_FEATURE_FLEXSPI_SUPPORT_SEPERATE_RXCLKSRC_PORTB) && FSL_FEATURE_FLEXSPI_SUPPORT_SEPERATE_RXCLKSRC_PORTB
                           FLEXSPI_MCR2_RXCLKSRC_B(config->readSampleClkSrc) |
#endif
#if defined(FSL_FEATURE_FLEXSPI_SUPPORT_RXCLKSRC_DIFF) && FSL_FEATURE_FLEXSPI_SUPPORT_RXCLKSRC_DIFF
                           FLEXSPI_MCR2_RX_CLK_SRC_DIFF(1) |
#endif
                           0x0;
            base->MCR2 = configValue;
        }

        // Configure Flash related control registers
        flexspi_config_flash_control_registers(base, config);

        // Configure DLLCR
        flexspi_configure_dll(base, config);

        if (config->deviceModeCfgEnable)
        {
            status = flexspi_device_workmode_config_all_chips(base, config);
            if (status != kStatus_Success)
            {
                break;
            }
        }

        if (config->configCmdEnable)
        {
            status = flexspi_device_cmd_config_all_chips(base, config);
            if (status != kStatus_Success)
            {
                break;
            }
        }

        status = kStatus_Success;

    } while (0);

    return status;
}
#endif // !defined(XIP_FLASH_TO_ACCESS_NAND)

status_t flexspi_update_lut(FLEXSPI_Type *base, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (lutBase == NULL) || (((uint64_t)seqIndex + seqNumber) > 16))
        {
            break;
        }

        flexspi_wait_until_ip_idle(base);

        uint32_t start_index = 0;
        uint32_t end_index = 0;
        volatile uint32_t *flexspiLutPtr;

        flexspi_unlock_lut(base);

        start_index = 4 * seqIndex;
        end_index = 4 * (seqIndex + seqNumber);
        flexspiLutPtr = &base->LUT[start_index];
        while (start_index < end_index)
        {
            *flexspiLutPtr = *lutBase;
            start_index++;
            flexspiLutPtr++;
            lutBase++;
        }

        flexspi_lock_lut(base);

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_command_xfer2(FLEXSPI_Type *base, flexspi_transfer_t *xfer)
{
    status_t status = FLEXSPI_TransferBlocking(base, xfer);

    flexspi_clear_cache(base);

    return status;
}

status_t flexspi_command_xfer(FLEXSPI_Type *base, flexspi_xfer_t *xfer)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (xfer == NULL) || (xfer->operation > kFlexSpiOperation_End))
        {
            break;
        }

        uint32_t temp = 0;
#if FLEXSPI_IPCR1_IPAREN_MASK
#if !FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        bool isParallelMode = false;
#else
        bool isParallelMode = xfer->isParallelModeEnable;
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
#endif

        flexspi_wait_until_ip_idle(base);

        // Clear sequence pointer before sending data to external devices
        flexspi_clear_sequence_pointer(base);

        // Clear former pending status before start this transfer.
        flexspi_clear_error_status(base);

        // Configure base address
        base->IPCR0 = xfer->baseAddress;

        if (xfer->operation == kFlexSpiOperation_Write)
        {
            temp = FLEXSPI_IPCR1_IDATSZ(xfer->txSize);
        }

        if (xfer->operation == kFlexSpiOperation_Read)
        {
            temp = FLEXSPI_IPCR1_IDATSZ(xfer->rxSize);
        }

        temp |= FLEXSPI_IPCR1_ISEQID(xfer->seqId) | FLEXSPI_IPCR1_ISEQNUM(xfer->seqNum - 1);
#if FLEXSPI_IPCR1_IPAREN_MASK
        temp |= FLEXSPI_IPCR1_IPAREN((uint32_t)isParallelMode);
#endif

        base->IPCR1 = temp;

        if (xfer->operation == kFlexSpiOperation_Read)
        {
            register uint32_t xferRemainingSize = xfer->rxSize;
            register uint32_t *xferBufferPtr = xfer->rxBuffer;
            if ((size_t)xferBufferPtr & 0x03)
            {
                break;
            }
            // Set Half RX FIFO size as Watermark size
            uint32_t rx_fifo_size = sizeof(base->RFDR);
            uint32_t watermark = rx_fifo_size / 8 / 2;
            uint32_t burst_rx_size = rx_fifo_size / 2;
            base->IPRXFCR = FLEXSPI_IPRXFCR_RXWMRK(watermark - 1);

            // Clear FIFO before read
            flexspi_clear_ip_rxfifo(base);

            // Start Read
            base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
            while (xferRemainingSize > 0)
            {
                register volatile uint32_t *rx_fifo_reg = (volatile uint32_t *)&base->RFDR[0];
                if (xferRemainingSize >= burst_rx_size)
                {
                    register uint32_t burst_rx_round = burst_rx_size / sizeof(uint32_t);
                    // Make sure the RX FIFO contains valid data before read
                    if (base->INTR & FLEXSPI_INTR_IPRXWA_MASK)
                    {
                        while (burst_rx_round--)
                        {
                            *xferBufferPtr++ = *rx_fifo_reg++;
                        }
                        xferRemainingSize -= burst_rx_size;
                        // Pop up data to RXFIFO for next read.
                        base->INTR |= FLEXSPI_INTR_IPRXWA_MASK;
                    }
                }
                else if ((8 * ((base->IPRXFSTS & FLEXSPI_IPRXFSTS_FILL_MASK) >> FLEXSPI_IPRXFSTS_FILL_SHIFT)) >=
                         xferRemainingSize)
                {
                    while (xferRemainingSize >= 4)
                    {
                        *xferBufferPtr++ = *rx_fifo_reg++;
                        xferRemainingSize -= 4;
                    }
                    if (xferRemainingSize)
                    {
                        uint32_t buf = *rx_fifo_reg;
                        uint8_t *src = (uint8_t *)&buf;
                        uint8_t *dst = (uint8_t *)xferBufferPtr;
                        for (uint32_t index = 0; index < xferRemainingSize; index++)
                        {
                            *dst++ = *src++;
                        }
                    }
                    xferRemainingSize = 0;
                }

                // Ensure no error occurs during read
                if ((base->INTR & FLEXSPI_INTR_IPCMDERR_MASK) != 0U)
                {
                    break;
                }
            }
        }

        if ((xfer->operation == kFlexSpiOperation_Write) || (xfer->operation == kFlexSpiOperation_Config))
        {
            register int32_t xferRemainingSize = (int32_t)xfer->txSize;
            register uint32_t *xferBufferPtr = xfer->txBuffer;
            // Enusre that the tx buffer is 4 words aligned for better performance
            if ((size_t)xferBufferPtr & 0x03)
            {
                break;
            }

            // Set Half TX FIFO size as Watermark size
            uint32_t tx_fifo_size = sizeof(base->TFDR);
            uint32_t watermark = tx_fifo_size / 8 / 2;
            uint32_t burst_tx_size = tx_fifo_size / 2;
            base->IPTXFCR = FLEXSPI_IPTXFCR_TXWMRK(watermark - 1);

            // Clear FIFO before write
            flexspi_clear_ip_txfifo(base);

            bool is_transfer_started = false;
            while (xferRemainingSize > 0)
            {
                register volatile uint32_t *tx_fifo_reg = &base->TFDR[0];
                // Make sure the TXFIFO is not full before write
                if (base->INTR & FLEXSPI_INTR_IPTXWE_MASK)
                {
                    register uint32_t burst_tx_round = burst_tx_size / sizeof(uint32_t);
                    if (xferRemainingSize >= burst_tx_size)
                    {
                        while (burst_tx_round--)
                        {
                            *tx_fifo_reg++ = *xferBufferPtr++;
                        }
                        xferRemainingSize -= burst_tx_size;
                    }
                    else
                    {
                        while (burst_tx_round--)
                        {
                            if (xferRemainingSize > 0)
                            {
                                xferRemainingSize -= 4;
                                *tx_fifo_reg++ = *xferBufferPtr++;
                            }
                            else
                            {
                                *tx_fifo_reg++ = 0x0;
                            }
                        }
                    }
                    if (!is_transfer_started)
                    {
                        base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
                        is_transfer_started = true;
                    }

                    base->INTR |= FLEXSPI_INTR_IPTXWE_MASK;
                }

                // Ensure no error occurs during write
                if ((base->INTR & FLEXSPI_INTR_IPCMDERR_MASK) != 0U)
                {
                    break;
                }
            }
        }
        if (xfer->operation == kFlexSpiOperation_Command)
        {
            base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
        }

        flexspi_wait_until_ip_idle(base);

        if (base->INTR & FLEXSPI_INTR_IPCMDERR_MASK)
        {
            switch ((base->STS1 & FLEXSPI_STS1_IPCMDERRCODE_MASK) >> FLEXSPI_STS1_IPCMDERRCODE_SHIFT)
            {
                default:
                    status = kStatus_FLEXSPI_InvalidSequence;
                    break;
                case kFlexSpiIpCmdError_SequenceExecutionTimeout:
                    status = kStatus_FLEXSPI_DeviceTimeout;
                    break;
            }
        }
        else
        {
            status = kStatus_Success;
        }

    } while (0);

    return status;
}

void flexspi_wait_idle(FLEXSPI_Type *base)
{
    do
    {
        if (base == NULL)
        {
            break;
        }

        // Wait until FlexSPI controller becomes idle
        while (!(base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK))
        {
        }

    } while (0);
}

void flexspi_clear_cache(FLEXSPI_Type *base)
{
    do
    {
        if (base == NULL)
        {
            break;
        }

        flexspi_wait_idle(base);

        /* Do software reset or clear AHB buffer directly. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT) && defined(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK) && \
    defined(FLEXSPI_AHBCR_CLRAHBTXBUF_MASK)
        // Clear AHB buffer
        base->AHBCR |= (FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
        while (base->AHBCR & (FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK))
        {
        }
#else
        FLEXSPI_SoftwareReset(base);
#endif

    } while (0);
}

#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
static status_t flexspi_extract_parallel_data(uint32_t *dst0, uint32_t *dst1, uint32_t *src, uint32_t length)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        uint8_t *dst0Byte = (uint8_t *)dst0;
        uint8_t *dst1Byte = (uint8_t *)dst1;
        uint8_t *srcByte = (uint8_t *)src;
        if ((length < 1) || (length & 1) || (dst0 == NULL) || (dst1 == NULL))
        {
            break;
        }

        while (length > 0)
        {
            *dst0Byte++ = *srcByte++;
            *dst1Byte++ = *srcByte++;
            length -= 2;
        }
        status = kStatus_Success;

    } while (0);

    return status;
}
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE

status_t flexspi_device_wait_busy(FLEXSPI_Type *base,
                                  flexspi_mem_config_t *config,
                                  bool isParallelMode,
                                  uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (config == NULL)
        {
            break;
        }
        uint32_t statusDataBuffer[2];
        uint32_t busyMask;
        uint32_t busyPolarity;
        bool isBusy = false;
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        uint32_t status0, status1;
#else
        isParallelMode = false;
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        flexspi_xfer_t flashXfer;

        busyMask = 1 << config->busyOffset;
        busyPolarity = config->busyBitPolarity;

        // Always try to find out the base address  for the parameter "baseAddr"
        uint32_t baseAddrTmp = 0;
        uint32_t *flashSizeArray = &config->sflashA1Size;
        for (uint32_t i = 0; i < 4; i++)
        {
            uint32_t lastAddr = baseAddrTmp;
            baseAddrTmp += flashSizeArray[i];
            if (baseAddr < baseAddrTmp)
            {
                baseAddr = lastAddr;
                break;
            }
        }
        
        flashXfer.baseAddress = baseAddr;
        flashXfer.operation = kFlexSpiOperation_Read;
        flashXfer.seqNum = 1;
        flashXfer.seqId = NAND_CMD_LUT_SEQ_IDX_READSTATUS;
        flashXfer.rxBuffer = &statusDataBuffer[0];

        flashXfer.rxSize = isParallelMode ? sizeof(statusDataBuffer) : sizeof(statusDataBuffer[0]);

        if (config->busyOffset < 16u)
        {
            flashXfer.rxSize = flashXfer.rxSize / 2;
        }

        flashXfer.isParallelModeEnable = isParallelMode;

        if (config->lutCustomSeqEnable && config->lutCustomSeq[NAND_CMD_INDEX_READSTATUS].seqNum)
        {
            flashXfer.seqId = config->lutCustomSeq[NAND_CMD_INDEX_READSTATUS].seqId;
            flashXfer.seqNum = config->lutCustomSeq[NAND_CMD_INDEX_READSTATUS].seqNum;
        }

        flexspi_update_lut(base, flashXfer.seqId, &config->lookupTable[4 * flashXfer.seqId], flashXfer.seqNum);

        bool enableTimeoutCheck = config->timeoutInMs ? true : false;
        uint64_t remainingMs = config->timeoutInMs;

        do
        {
            status = flexspi_command_xfer(base, &flashXfer);
            if (status != kStatus_Success)
            {
                break;
            }
            else
            {
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
                if (isParallelMode)
                {
                    // Extract parallel data to serial data
                    flexspi_extract_parallel_data(&status0, &status1, statusDataBuffer, sizeof(statusDataBuffer));

                    if (busyPolarity)
                    {
                        isBusy = ((~status0) & busyMask) | ((~status1) & busyMask);
                    }
                    else
                    {
                        isBusy = (status0 & busyMask) | (status1 & busyMask);
                    }
                }
                else
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
                {
                    // Busy bit is 0 if polarity is 1
                    if (busyPolarity)
                    {
                        if (config->busyOffset < 16)
                        {
                            uint16_t tmp = (uint16_t)statusDataBuffer[0] & 0xffffu;
                            statusDataBuffer[0] = tmp;
                        }
                        isBusy = (~statusDataBuffer[0]) & busyMask;
                    }
                    else
                    {
                        isBusy = statusDataBuffer[0] & busyMask;
                    }
                }
            }

            if (isBusy && enableTimeoutCheck)
            {
                if (remainingMs > 0)
                {
                    mixspi_sw_delay_us(1000);
                    remainingMs--;
                }
                else
                {
                    status = kStatus_FLEXSPI_DeviceTimeout;
                    break;
                }
            }

        } while (isBusy);

    } while (0);

    return status;
}

void flexspi_clear_sequence_pointer(FLEXSPI_Type *base)
{
    uint32_t index;

    do
    {
        if (base == NULL)
        {
            break;
        }

        for (index = 0; index < FLEXSPI_FLSHCR2_COUNT; index++)
        {
            base->FLSHCR2[index] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;
        }

    } while (0);
}

void flexspi_half_clock_control(FLEXSPI_Type *base, uint32_t option)
{
    do
    {
        if (base == NULL)
        {
            break;
        }

        flexspi_wait_until_ip_idle(base);

        if (option)
        {
            base->MCR0 |= FLEXSPI_MCR0_HSEN_MASK;
        }
        else
        {
            base->MCR0 &= (uint32_t)~FLEXSPI_MCR0_HSEN_MASK;
        }

    } while (0);
}
