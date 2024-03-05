/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __BL_FLEXSPI_H__
#define __BL_FLEXSPI_H__

#include "fsl_common.h"
#include "port_mixspi_info.h"

/**********************************************************************************************************************
 * Definitions
 *********************************************************************************************************************/

/* FLEXSPI Feature related definitions */
#define FLEXSPI_FEATURE_HAS_PARALLEL_MODE 0

//!@brief Defintions for FlexSPI Serial Clock Frequency
typedef enum _FlexSpiSerialClockFreq
{
    kFlexSpiSerialClk_SafeFreq = 1,
} flexspi_serial_clk_freq_t;

//!@brief FlexSPI IP Error codes
typedef enum _FlexSpiIpCmdError
{
    kFlexSpiIpCmdError_NoError = 0,
    kFlexSpiIpCmdError_DataSizeNotEvenUnderParallelMode = 1,
    kFlexSpiIpCmdError_JumpOnCsInIpCmd = 2,
    kFlexSpiIpCmdError_UnknownOpCode = 3,
    kFlexSpiIpCmdError_SdrDummyInDdrSequence = 4,
    kFlexSpiIpCmdError_DDRDummyInSdrSequence = 5,
    kFlexSpiIpCmdError_InvalidAddress = 6,
    kFlexSpiIpCmdError_SequenceExecutionTimeout = 0x0E,
    kFlexSpiIpCmdError_FlashBoundaryAcrosss = 0x0F
} flexspi_ipcmd_error_t;

/* status code for flexspi */
enum _flexspi_status
{
    kStatus_FLEXSPI_InvalidSequence = MAKE_STATUS(kStatusGroup_FLEXSPI, 1), //!< Status for Invalid Sequence
    kStatus_FLEXSPI_DeviceTimeout = MAKE_STATUS(kStatusGroup_FLEXSPI, 2),   //!< Status for Device timeout
};

//!@brief Misc feature bit definitions
enum
{
    kFlexSpiMiscOffset_DiffClkEnable = 0,            //!< Bit for Differential clock enable
    kFlexSpiMiscOffset_Ck2Enable = 1,                //!< Bit for CK2 enable
    kFlexSpiMiscOffset_ParallelEnable = 2,           //!< Bit for Parallel mode enable
    kFlexSpiMiscOffset_WordAddressableEnable = 3,    //!< Bit for Word Addressable enable
    kFlexSpiMiscOffset_SafeConfigFreqEnable = 4,     //!< Bit for Safe Configuration Frequency enable
    kFlexSpiMiscOffset_PadSettingOverrideEnable = 5, //!< Bit for Pad setting override enable
    kFlexSpiMiscOffset_DdrModeEnable = 6,            //!< Bit for DDR clock confiuration indication.
    kFlexSpiMiscOffset_UseValidTimeForAllFreq = 7,   //!< Bit for DLLCR settings under all modes
    kFlexSpiMiscOffset_SecondPinMux = 8,             //!< Bit for Second Pinmux group
    kFlexSpiMiscOffset_SecondDqsPinMux = 9,          //!< Bit for Second DQS Pinmux
};

//!@brief Flash Type Definition
enum
{
    kFlexSpiDeviceType_SerialNOR = 1,       //!< Flash devices are Serial NOR
    kFlexSpiDeviceType_SerialNAND = 2,      //!< Flash devices are Serial NAND
    kFlexSpiDeviceType_SerialRAM = 3,       //!< Flash devices are Serial RAM/HyperFLASH
    kFlexSpiDeviceType_MCP_NOR_NAND = 0x12, //!< Flash device is MCP device, A1 is Serial NOR, A2 is Serial NAND
    kFlexSpiDeviceType_MCP_NOR_RAM = 0x13,  //!< Flash deivce is MCP device, A1 is Serial NOR, A2 is Serial RAMs
};

//!@brief FlexSPI LUT Sequence structure
typedef struct _lut_sequence
{
    uint8_t seqNum; //!< Sequence Number, valid number: 1-16
    uint8_t seqId;  //!< Sequence Index, valid number: 0-15
    uint16_t reserved;
} flexspi_lut_seq_t;

//!@brief Flash Configuration Command Type
enum
{
    kDeviceConfigCmdType_Generic,    //!< Generic command, for example: configure dummy cycles, drive strength, etc
    kDeviceConfigCmdType_QuadEnable, //!< Quad Enable command
    kDeviceConfigCmdType_Spi2Xpi,    //!< Switch from SPI to DPI/QPI/OPI mode
    kDeviceConfigCmdType_Xpi2Spi,    //!< Switch from DPI/QPI/OPI to SPI mode
    kDeviceConfigCmdType_Spi2NoCmd,  //!< Switch to 0-4-4/0-8-8 mode
    kDeviceConfigCmdType_Reset,      //!< Reset device command
};

typedef struct
{
    uint8_t time_100ps;  // Data valid time, in terms of 100ps
    uint8_t delay_cells; // Data valid time, in terms of delay cells
} flexspi_dll_time_t;

//!@brief FlexSPI Memory Configuration Block
typedef struct _FlexSPIConfig
{
    uint32_t reservedX0[3];
    /////////////////////////////
    uint8_t readSampleClkSrc;   //!< [0x00c-0x00c] Read Sample Clock Source, valid value: 0/1/3
    uint8_t csHoldTime;         //!< [0x00d-0x00d] CS hold time, default value: 3
    uint8_t csSetupTime;        //!< [0x00e-0x00e] CS setup time, default value: 3
    uint8_t columnAddressWidth; //!< [0x00f-0x00f] Column Address with, for HyperBus protocol, it is fixed to 3, For
    //! Serial NAND, need to refer to datasheet
    uint8_t deviceModeCfgEnable; //!< [0x010-0x010] Device Mode Configure enable flag, 1 - Enable, 0 - Disable
    uint8_t deviceModeType; //!< [0x011-0x011] Specify the configuration command type:Quad Enable, DPI/QPI/OPI switch,
    //! Generic configuration, etc.
    uint16_t waitTimeCfgCommands; //!< [0x012-0x013] Wait time for all configuration commands, unit: 100us, Used for
    //! DPI/QPI/OPI switch or reset command
    flexspi_lut_seq_t deviceModeSeq; //!< [0x014-0x017] Device mode sequence info, [7:0] - LUT sequence id, [15:8] - LUt
    //! sequence number, [31:16] Reserved
    uint32_t deviceModeArg;    //!< [0x018-0x01b] Argument/Parameter for device configuration
    uint8_t configCmdEnable;   //!< [0x01c-0x01c] Configure command Enable Flag, 1 - Enable, 0 - Disable
    uint8_t configModeType[3]; //!< [0x01d-0x01f] Configure Mode Type, similar as deviceModeTpe
    flexspi_lut_seq_t
        configCmdSeqs[3]; //!< [0x020-0x02b] Sequence info for Device Configuration command, similar as deviceModeSeq
    uint32_t reserved1;   //!< [0x02c-0x02f] Reserved for future use
    uint32_t configCmdArgs[3];     //!< [0x030-0x03b] Arguments/Parameters for device Configuration commands
    uint32_t reserved2;            //!< [0x03c-0x03f] Reserved for future use
    uint32_t controllerMiscOption; //!< [0x040-0x043] Controller Misc Options, see Misc feature bit definitions for more
    //! details
    uint8_t deviceType;    //!< [0x044-0x044] Device Type:  See Flash Type Definition for more details
    uint8_t sflashPadType; //!< [0x045-0x045] Serial Flash Pad Type: 1 - Single, 2 - Dual, 4 - Quad, 8 - Octal
    uint8_t serialClkFreq; //!< [0x046-0x046] Serial Flash Frequencey, device specific definitions, See System Boot
    //! Chapter for more details
    uint8_t lutCustomSeqEnable; //!< [0x047-0x047] LUT customization Enable, it is required if the program/erase cannot
    //! be done using 1 LUT sequence, currently, only applicable to HyperFLASH
    uint32_t reserved3[2];               //!< [0x048-0x04f] Reserved for future use
    uint32_t sflashA1Size;               //!< [0x050-0x053] Size of Flash connected to A1
    uint32_t sflashA2Size;               //!< [0x054-0x057] Size of Flash connected to A2
    uint32_t sflashB1Size;               //!< [0x058-0x05b] Size of Flash connected to B1
    uint32_t sflashB2Size;               //!< [0x05c-0x05f] Size of Flash connected to B2
    uint32_t reservedX2[4];
    /////////////////////////////
    uint32_t timeoutInMs;                //!< [0x070-0x073] Timeout threshold for read status command
    uint32_t commandInterval;            //!< [0x074-0x077] CS deselect interval between two commands
    flexspi_dll_time_t dataValidTime[2]; //!< [0x078-0x07b] CLK edge to data valid time for PORT A and PORT B
    uint16_t busyOffset;                 //!< [0x07c-0x07d] Busy offset, valid value: 0-31
    uint16_t busyBitPolarity; //!< [0x07e-0x07f] Busy flag polarity, 0 - busy flag is 1 when flash device is busy, 1 -
    //! busy flag is 0 when flash device is busy
    uint32_t lookupTable[64];           //!< [0x080-0x17f] Lookup table holds Flash command sequences
    flexspi_lut_seq_t lutCustomSeq[12]; //!< [0x180-0x1af] Customizable LUT Sequences
    uint32_t dll0CrVal;                 //!> [0x1b0-0x1b3] Customizable DLL0CR setting
    uint32_t dll1CrVal;                 //!> [0x1b4-0x1b7] Customizable DLL1CR setting  
    uint32_t reserved4[2];              //!< [0x1b8-0x1bf] Reserved for future use
} flexspi_mem_config_t;

typedef enum _FlexSPIOperationType
{
    kFlexSpiOperation_Command, //!< FlexSPI operation: Only command, both TX and
    //! RX buffer are ignored.
    kFlexSpiOperation_Config, //!< FlexSPI operation: Configure device mode, the
    //! TX FIFO size is fixed in LUT.
    kFlexSpiOperation_Write, //!< FlexSPI operation: Write,  only TX buffer is
    //! effective
    kFlexSpiOperation_Read, //!< FlexSPI operation: Read, only Rx Buffer is
    //! effective.
    kFlexSpiOperation_End = kFlexSpiOperation_Read,
} flexspi_operation_t;

//!@brief FlexSPI Transfer Context
typedef struct _FlexSpiXfer
{
    flexspi_operation_t operation; //!< FlexSPI operation
    uint32_t baseAddress;          //!< FlexSPI operation base address
    uint32_t seqId;                //!< Sequence Id
    uint32_t seqNum;               //!< Sequence Number
    bool isParallelModeEnable;     //!< Is a parallel transfer
    uint32_t *txBuffer;            //!< Tx buffer
    uint32_t txSize;               //!< Tx size in bytes
    uint32_t *rxBuffer;            //!< Rx buffer
    uint32_t rxSize;               //!< Rx size in bytes
} flexspi_xfer_t;

//!@brief Generate bit mask
#define FLEXSPI_BITMASK(bit_offset) (1u << (bit_offset))

#ifndef FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
#define FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT (0)
#endif

#ifndef FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
#define FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT (0)
#endif

/**********************************************************************************************************************
 * API
 *********************************************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    //!@brief Initialize FlexSPI
    status_t flexspi_init(FLEXSPI_Type *base, flexspi_mem_config_t *config);

    //!@brief Send Write enable command to Serial Memory device
    status_t flexspi_device_write_enable(FLEXSPI_Type *base,
                                         flexspi_mem_config_t *config,
                                         bool isParallelMode,
                                         uint32_t baseAddr);

    //!@brief Wait until device is idle
    status_t flexspi_device_wait_busy(FLEXSPI_Type *base,
                                      flexspi_mem_config_t *config,
                                      bool isParallelMode,
                                      uint32_t baseAddr);

    //!@brief Configure FlexSPI Lookup table
    status_t flexspi_update_lut(FLEXSPI_Type *base, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);

    //!@brief Perform FlexSPI command
    status_t flexspi_command_xfer2(FLEXSPI_Type *base, flexspi_transfer_t *xfer);
    status_t flexspi_command_xfer(FLEXSPI_Type *base, flexspi_xfer_t *xfer);

    //!@brief Get FlexSPI Clock frequency
    //extern status_t flexspi_get_clock(FLEXSPI_Type *base, flexspi_clock_type_t type, uint32_t *freq);
    extern mixspi_root_clk_freq_t flexspi_convert_clock_for_ddr(mixspi_root_clk_freq_t freq);

    //!@brief Wait until FlexSPI controller becomes idle
    void flexspi_wait_idle(FLEXSPI_Type *base);

    //!@brief Clear FlexSPI cache
    void flexspi_clear_cache(FLEXSPI_Type *base);

    //!@brief Clear FlexSPI sequence pointer
    void flexspi_clear_sequence_pointer(FLEXSPI_Type *base);

    //!@brief Configure IOMUX for FlexSPI
    extern void flexspi_iomux_config(FLEXSPI_Type *base, flexspi_mem_config_t *config);

    //!@brief Configure Clock for FlexSPI
    //extern void flexspi_clock_config(FLEXSPI_Type *base, uint32_t freq, uint32_t sampleClkMode);

    //!@brief Check whether Pad Setting Override feature is enabled.
    bool flexspi_is_padsetting_override_enable(flexspi_mem_config_t *config);

    //!@brief Check whether Differential clock feature is enabled.
    bool flexspi_is_differential_clock_enable(flexspi_mem_config_t *config);

    //!@brief Check whether DDR mode feature is enabled.
    bool flexspi_is_ddr_mode_enable(flexspi_mem_config_t *config);

    //!@brief Check whether CK2 feature is enabled.
    bool flexspi_is_ck2_enabled(flexspi_mem_config_t *config);

    //!@brief Check whether Parallel mode feature is enabled.
    bool flexspi_is_parallel_mode(flexspi_mem_config_t *config);

    //!@brief Check whether device works under word addressable mode
    bool flexspi_is_word_addressable(flexspi_mem_config_t *config);

    //!@brief Configure FlexSPI DLL register
    status_t flexspi_configure_dll(FLEXSPI_Type *base, flexspi_mem_config_t *config);

    //!@brief Half FlexSPI Clock
    void flexspi_half_clock_control(FLEXSPI_Type *base, uint32_t option);

    //!@brief Set Failfase setting info
    extern status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config);

    //!@brief Get Maximumn clock frequency
    extern status_t flexspi_get_max_supported_freq(FLEXSPI_Type *base, uint32_t *freq, uint32_t clkMode);

//    extern void flexspi_sw_delay_us(uint64_t us);

    extern void flexspi_update_padsetting(flexspi_mem_config_t *config, uint32_t driveStrength);

#ifdef __cplusplus
}
#endif

#endif
