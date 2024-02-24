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


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/


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
    
    spinand_mem_config((uint32_t *)&nandOpt);

}
