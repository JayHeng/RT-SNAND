/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "snand_memory.h"
#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_FLASH_PAGE_SIZE 0x800

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

uint32_t g_flashRwBuffer[EXAMPLE_FLASH_PAGE_SIZE/4];

uint32_t g_flashRwBuffer2[EXAMPLE_FLASH_PAGE_SIZE/4];

/*******************************************************************************
 * Code
 ******************************************************************************/

int snand_printf(const char *fmt_s, ...)
{
#if SNAND_DEBUG_LOG_INFO_ENABLE
    PRINTF(fmt_s);
#endif
    
    return 0;
}

void snand_main(void)
{
    status_t status = kStatus_InvalidArgument;

    snand_printf("SNAND: i.MXRT Serial NAND Access solution.\r\n");
    snand_printf("SNAND: Get CPU root clock.\r\n");
    /* Show CPU clock source */
    cpu_show_clock_source();

    /* Configure FlexSPI pinmux */
    mixspi_pin_init(EXAMPLE_MIXSPI, EXAMPLE_MIXSPI_PORT, kFLEXSPI_4PAD);

    serial_nand_config_option_t nandOpt = 
    {
        .option0.U = 0xc1010026,
        .option1.U = 0x000000ef,
    };
    
    status = spinand_mem_config((uint32_t *)&nandOpt);
    if (status != kStatus_Success)
    {
        snand_printf("SNAND: Failed to config Flash.\r\n");
        return;
    }

    for (uint32_t idx = 0; idx < EXAMPLE_FLASH_PAGE_SIZE / sizeof(uint32_t); idx++)
    {
        g_flashRwBuffer[idx] = 0 + idx * sizeof(uint32_t);
    }

    status = spinand_mem_erase(0x0, 0x800);
    if (status != kStatus_Success)
    {
        snand_printf("SNAND: Failed to erase Flash.\r\n");
        return;
    }

    status = spinand_mem_write(0x0, 0x800, (const uint8_t *)&g_flashRwBuffer);
    if (status != kStatus_Success)
    {
        snand_printf("SNAND: Failed to write Flash.\r\n");
        return;
    }
    status = spinand_mem_flush();
    if (status != kStatus_Success)
    {
        snand_printf("SNAND: Failed to flush Flash.\r\n");
        return;
    }

    status = spinand_mem_read(0x0, 0x800, (uint8_t *)&g_flashRwBuffer2);
    if (status != kStatus_Success)
    {
        snand_printf("SNAND: Failed to read Flash.\r\n");
        return;
    }
    
    if (!memcmp((void *)&g_flashRwBuffer, (void *)&g_flashRwBuffer2, sizeof(g_flashRwBuffer)))
    {
        snand_printf("SNAND: Passed Config/Erase/Write/Read flash.\r\n");
    }
    else
    {
        snand_printf("SNAND: Failed to compare Flash data.\r\n");
    }
}
