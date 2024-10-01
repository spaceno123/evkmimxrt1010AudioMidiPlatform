/*
 * boardEx.c
 *
 *  Created on: 2024/09/26
 *      Author: M.Akino
 */

/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "boardEx.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define VECTORENTRY (192)//(256)

__attribute__((section(".vectorTableRam"), aligned(0x400))) uint32_t g_vectorTable[VECTORENTRY] = {0};

extern uint32_t __Vectors[];
extern void _vStackTop(void);
#define __VECTOR_TABLE __Vectors
#define __StackTop     _vStackTop

/*******************************************************************************
 * Code
 ******************************************************************************/

void BOARD_RelocateVectorTableToRam(void)
{
    uint32_t n;
    uint32_t irqMaskValue;

    irqMaskValue = DisableGlobalIRQ();

    SCB_DisableDCache();
    SCB_DisableICache();

    /* Copy the vector table from ROM to RAM */
    for (n = 0; n < /*((uint32_t)0x400)*/sizeof(g_vectorTable) / sizeof(uint32_t); n++)
    {
        g_vectorTable[n] = __VECTOR_TABLE[n];
    }

    /* Set application defined stack pointer */
    volatile unsigned int vStackTop = (unsigned int)&__StackTop;
    g_vectorTable[0]                = vStackTop;

    /* Point the VTOR to the position of vector table */
    SCB->VTOR = (uint32_t)g_vectorTable;
    __DSB();

    SCB_EnableICache();
    SCB_EnableDCache();

    EnableGlobalIRQ(irqMaskValue);
}

