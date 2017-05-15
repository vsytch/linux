/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2008 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   hal.h                                                                                                 */
/*            This file contains configurations for BMC HAL                                                */
/*  Project:                                                                                               */
/*            Linux Kernel                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _HAL_CONFIG_H
#define _HAL_CONFIG_H

/*---------------------------------------------------------------------------------------------------------*/
/* Linux Includes                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#include <linux/types.h>
#include <linux/dma-direction.h>

#include <asm/types.h>
#include <asm/string.h>
#include <asm/page.h>
#include <defs.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Switching between Linux CPU defines to BMC HAL BOARD_NAME define                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef  CONFIG_MACH_WPCM450
#define BOARD_NAME   HermonSVB
#endif

#ifdef  CONFIG_MACH_NPCM650
#define BOARD_NAME   YarkonSVB
#endif

#ifdef  CONFIG_MACH_NPCM750
#define BOARD_NAME   PolegSVB
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Setting BMC compilation configurations                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define NO_LIBC                         1
#define NO_INTERNAL_IRQ_HANDLER         1


/*---------------------------------------------------------------------------------------------------------*/
/* Defining base addresses for the BMC HAL Modules in Linux Kernel                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef REAL_MODE_PHASE
    #include <asm/cacheflush.h>

    /*-----------------------------------------------------------------------------------------------------*/
    /* Virtual memory handlers                                                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    #define VIRT_TO_PHYS(x)                     virt_to_phys(x)
    #define PHYS_TO_VIRT(x)                     phys_to_virt(x)

//    #define DCACHE_INV_RANGE(start, end)        dmac_inv_range(start, end -start, DMA_FROM_DEVICE)
//    #define DCACHE_CLEAN_RANGE(start, end)      dmac_clean_range(start, end -start, DMA_TO_DEVICE)
//    #define DCACHE_FLUSH_RANGE(start, end)      dmac_flush_range(start, end, DMA_BIDIRECTIONAL)


//    #define DCACHE_INV_RANGE(start, size)        dmac_map_area(start, size, DMA_TO_DEVICE)
//    #define DCACHE_CLEAN_RANGE(start, size)      dmac_map_area(start, size, DMA_FROM_DEVICE)
//    #define DCACHE_FLUSH_RANGE(start, size)      dmac_map_area(start, size, DMA_BIDIRECTIONAL)


    /*-----------------------------------------------------------------------------------------------------*/
    /* For debug mode, enable all debug messages from HAL                                                  */
    /*-----------------------------------------------------------------------------------------------------*/
    #if 0
        #include <linux/kernel.h>
        #define printf printk

        #define VERBOSE_GMAC          1
    #endif

    /*-----------------------------------------------------------------------------------------------------*/
    /* Including Linux Mapping                                                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    #include <mach/map.h>

#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Adding Chip.h AFTER setting BMC configurations                                                          */
/*---------------------------------------------------------------------------------------------------------*/
// #include "../../BMC_HAL/Boards/board.h"
#include "board_PolegSVB.h"

#endif // _HAL_CONFIG_H
