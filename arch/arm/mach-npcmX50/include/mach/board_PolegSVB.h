/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   PolegSVB.h                                                                                           */
/*            This file contains PolegSVB definitions                                                     */
/* Project:                                                                                                */
/*            BMC HAL                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef _BOARD_POLEG_SVB_H_
#define _BOARD_POLEG_SVB_H_

/*---------------------------------------------------------------------------------------------------------*/
/*                                             Chip interface                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include "chip_npcm750.h"



/*---------------------------------------------------------------------------------------------------------*/
/*                                       Board dependent parameters                                        */
/*---------------------------------------------------------------------------------------------------------*/

//Trego - Add board specific

//#define NPCM750_POLEG_DRB_HW         1



/*---------------------------------------------------------------------------------------------------------*/
/* PHY assignment per module                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_BOARD_ETH0_PHY_OPS              BCM5221_Ops        // EMC1
#define NPCMX50_BOARD_ETH1_PHY_OPS              BCM5221_Ops        // EMC2

#define NPCMX50_ETH0_MAC_ADDRESS {0x00,0x00,0xF7,0xA0,0x00,0x45}   // EMC1
#define NPCMX50_ETH1_MAC_ADDRESS {0x00,0x00,0xF7,0xA0,0x00,0x46}   // EMC2
#define NPCMX50_ETH2_MAC_ADDRESS {0x00,0x00,0xF7,0xA0,0x00,0x47}   // GMAC1
#define NPCMX50_ETH3_MAC_ADDRESS {0x00,0x00,0xF7,0xA0,0x00,0x48}   // GMAC2


/*---------------------------------------------------------------------------------------------------------*/
/* SPI Flash                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_CONFIG_FLASH_BASE               NPCMX50_FLASH_BASE_ADDR(0)

#define NPCMX50_CONFIG_SYS_MAX_FLASH_SECT       (_16MB_ / _4KB_)

#define NPCMX50_CONFIG_SPI3_ENABLE

#ifdef NPCMX50_CONFIG_SPI3_ENABLE
#define NPCMX50_CONFIG_SYS_MAX_FLASH_BANKS      8
#else
#define NPCMX50_CONFIG_SYS_MAX_FLASH_BANKS      4
#endif

#define NPCMX50_BOOTER_VER_ADDR                0x800002F4


/*---------------------------------------------------------------------------------------------------------*/
/* EMC configuration                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
//#define NPCM750_BOARD_EMC_USING_GMAC_MDIO       1

/*---------------------------------------------------------------------------------------------------------*/
/*                                          Peripherals interface                                          */
/*---------------------------------------------------------------------------------------------------------*/
// to check: #include "../../Peripherals/spi_flash/spi_flash.h"
// to check: #include "../../Peripherals/eth_phy/eth_phy.h"



#endif //_BOARD_POLEG_SVB_H_

