/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_power.h"
#include "fsl_reset.h"
#include "snand.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

#if !defined(FSL_SDK_DRIVER_QUICK_ACCESS_ENABLE)
    POWER_DisablePD(kPDRUNCFG_APD_FLEXSPI0_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_FLEXSPI0_SRAM);
    POWER_ApplyPD();
#endif

    snand_printf("\r\nSNAND: Target i.MXRT500.\r\n");
    snand_printf("\r\n-------------------------------------\r\n");
    snand_main();
    snand_printf("-------------------------------------\r\n");

    while (1)
    {
    }
}
