/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2008 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   yarkon_palladium.h                                                                                    */
/*            This file contains chip definitions for the given project                                    */
/*  Project:                                                                                               */
/*            BMC HAL                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _NPCM650_CHIP_H
#define _NPCM650_CHIP_H

/*---------------------------------------------------------------------------------------------------------*/
/* Chip Name                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCM650_CHIP

/*---------------------------------------------------------------------------------------------------------*/
/* On-Chip ROM                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_ROM_BASE_ADDR                  0xFFFF0000
#define NPCMX50_ROM_MEMORY_SIZE                _32KB_
#define NPCMX50_ROMCODE_VER_ADDR               0xFFFF0054

/*---------------------------------------------------------------------------------------------------------*/
/* On-Chip SRAM                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SRAM_BASE_ADDR                  0x00000000
#define NPCMX50_SRAM_MEMORY_SIZE                _32KB_

/*---------------------------------------------------------------------------------------------------------*/
/* External SDRAM Space DDR3                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SDRAM_BASE_ADDR                 0x00008000
#define NPCMX50_SDRAM_MAPPED_SIZE               (_1GB_ - SRAM_MEMORY_SIZE)

/*---------------------------------------------------------------------------------------------------------*/
/* SPI Flash                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_FLASH_BASE_ADDR                 0x40000000
#define NPCMX50_FLASH_MAPPED_SIZE               _64MB_

/*---------------------------------------------------------------------------------------------------------*/
/* XBus                                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_XBUS_BASE_ADDR                  0x44000000
#define NPCMX50_XBUS_MAPPED_SIZE                _32MB_

/*---------------------------------------------------------------------------------------------------------*/
/* Memory Mapped IO                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_IOMEMORY_BASE_ADDR              0xF0000000
#define NPCMX50_IOMEMORY_SIZE                   0x0F004000


/*---------------------------------------------------------------------------------------------------------*/
/* Defining IO Memory continuesly mapped blocks                                                            */
/*---------------------------------------------------------------------------------------------------------*/
                                    /* Map:Phisical Addr | Virtual Addr | Block Size     */
                                    /* -----------------------------------------------    */
#define NPCMX50_IOMEMORY_BLOCKS             {                                                                \
                                        {   0xF0000000, IOMEMORY(0xF0000000), 0x00040000   },        \
                                        {   0xF0400000, IOMEMORY(0xF0400000), 0x00020000   },        \
                                        {   0xF0440000, IOMEMORY(0xF0440000), 0x00040000   },        \
                                        {   0xF1F00000, IOMEMORY(0xF1F00000), 0x00002000   },        \
                                        {   0xF1FFE000, IOMEMORY(0xF1FFE000), 0x00004000   },        \
                                        {   0xF4200000, IOMEMORY(0xF4200000), 0x00300000   },        \
                                        {   0xF8000000, IOMEMORY(0xF8000000), 0x00003000   },        \
                                        {   0xFF000000, IOMEMORY(0xFF000000), 0x00004000   },        \
                                    }



/*---------------------------------------------------------------------------------------------------------*/
/* Palladium pll0 global register                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef _PALLADIUM_
#define NPCMX50_PALLADIUM_PLL0_ACCESS                      MEM
#define NPCMX50_PALLADIUM_PLL0_PHYS_BASE_ADDR              0xffff3fc4

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PALLADIUM_PLL0_BASE_ADDR                   PALLADIUM_PLL0_PHYS_BASE_ADDR
#else
#define NPCMX50_PALLADIUM_PLL0_BASE_ADDR                   PALLADIUM_PLL0_VIRT_BASE_ADDR
#endif
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* General Ethernet configuration                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_CHIP_NUM_OF_ETH                 2
#define NPCMX50_CHIP_ETH0_TYPE                  EMC
#define NPCMX50_CHIP_ETH1_TYPE                  GMAC
#define NPCMX50_CHIP_NUM_OF_EMC_ETH             1
#define NPCMX50_CHIP_NUM_OF_GMAC_ETH            1
/*---------------------------------------------------------------------------------------------------------*/
/* General Module                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GCR_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_GCR_ACCESS                      MEM
#define NPCMX50_GCR_PHYS_BASE_ADDR              0xF0400000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GCR_BASE_ADDR                   GCR_PHYS_BASE_ADDR
#else
#define NPCMX50_GCR_BASE_ADDR                   GCR_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* UART Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_UART_MODULE_TYPE                Hermon_IP
#define NPCMX50_UART_ACCESS                     MEM
#define NPCMX50_UART_PHYS_BASE_ADDR(module)     (0xF0001000 + ((module) * 0x1000))
#define NPCMX50_UART_INTERRUPT(module)          (module + UART0_INT)
#define NPCMX50_UART_NUM_OF_MODULES             2

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_UART_BASE_ADDR(module)          UART_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_UART_BASE_ADDR(module)          UART_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PCI MailBox Module                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PCIMBX_ACCESS                   MEM
#define NPCMX50_PCIMBX_PHYS_BASE_ADDR           0xF0401000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PCIMBX_BASE_ADDR                PCIMBX_PHYS_BASE_ADDR
#else
#define NPCMX50_PCIMBX_BASE_ADDR                PCIMBX_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PCTL (Memory) Module                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PCTL_ACCESS                     MEM
#define NPCMX50_PCTL_PHYS_BASE_ADDR             0xFF001000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PCTL_BASE_ADDR                  PCTL_PHYS_BASE_ADDR
#else
#define NPCMX50_PCTL_BASE_ADDR                  PCTL_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Timer Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_TIMER_MODULE_TYPE               Hermon_IP
#define NPCMX50_TIMER_ACCESS                    MEM
#define NPCMX50_TIMER_PHYS_BASE_ADDR(module)    (0xF0007000 + ((module)*0x1000))
#define NPCMX50_TIMER_INTERRUPT(port)           (((port) <2)? T_INT0 + (port) : T_INT_GRP)
#define NPCMX50_TIMER_GROUP_INTERRUPT(port)     ((((port) >=2) && ((port) <5)) ? (16 + ((port)-2)) : AIC_GROUP_INTERRUPT_NONE)

#define NPCMX50_TIMER_NUM_OF_MODULES            2
#define NPCMX50_TIMER_NUM_OF_PORTS              10

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_TIMER_BASE_ADDR(port)           TIMER_PHYS_BASE_ADDR(port)
#else
#define NPCMX50_TIMER_BASE_ADDR(port)           TIMER_VIRT_BASE_ADDR(port)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* SMB Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SMB_MODULE_TYPE                 Hermon_IP
#define NPCMX50_SMB_ACCESS                      MEM
#define NPCMX50_SMB_PHYS_BASE_ADDR(module)      (0xF0020000+((module)*0x1000))

#define NPCMX50_SMB_MAX_BUSES                   8
#define NPCMX50_SMB_SCL_GPIOS                   114, 116, 118, 31, 29, 27, 171, 173
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SMB_BASE_ADDR(module)           SMB_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_SMB_BASE_ADDR(module)           SMB_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* KCS Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_KCS_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_KCS_ACCESS                      MEM
#define NPCMX50_KCS_PHYS_BASE_ADDR              0xF002C000

#define NPCMX50_KCS_MAX_CHANNELS                3
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_KCS_BASE_ADDR                   KCS_PHYS_BASE_ADDR
#else
#define NPCMX50_KCS_BASE_ADDR                   KCS_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_ADC_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_ADC_ACCESS                      MEM
#define NPCMX50_ADC_PHYS_BASE_ADDR              0xF002D000

#define NPCMX50_ADC_MAX_CHANNELS                8
#define NPCMX50_ADC_GROUP_INTERRUPT             8
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_ADC_BASE_ADDR                   ADC_PHYS_BASE_ADDR
#else
#define NPCMX50_ADC_BASE_ADDR                   ADC_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PWM Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PWM_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_PWM_ACCESS                      MEM
#define NPCMX50_PWM_PHYS_BASE_ADDR(module)      (0xF0028000 + (0x1000 * (module)))

#define NPCMX50_PWM_MAX_MODULES                 2
#define NPCMX50_PWM_MAX_PORTS                   8
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PWM_BASE_ADDR(module)           PWM_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_PWM_BASE_ADDR(module)           PWM_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* FUI Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_FIU_MODULE_TYPE                 Hermon_IP
#define NPCMX50_FIU_ACCESS                      MEM
#define NPCMX50_FIU_PHYS_BASE_ADDR(module)      0xF8000000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_FIU_BASE_ADDR                   FIU_PHYS_BASE_ADDR(0)
#else
#define NPCMX50_FIU_BASE_ADDR                   FIU_VIRT_BASE_ADDR(0)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Clock Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_CLK_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_CLK_ACCESS                      MEM
#define NPCMX50_CLK_PHYS_BASE_ADDR              0xF0400200

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_CLK_BASE_ADDR                   CLK_PHYS_BASE_ADDR
#else
#define NPCMX50_CLK_BASE_ADDR                   CLK_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Shared Memory (SHM) Module                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SHM_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_SHM_ACCESS                      MEM
#define NPCMX50_SHM_PHYS_BASE_ADDRESS           0xF8001000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SHM_BASE_ADDR                   SHM_PHYS_BASE_ADDRESS
#else
#define NPCMX50_SHM_BASE_ADDR                   SHM_VIRT_BASE_ADDRESS
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* AIC Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_AIC_MODULE_TYPE                 Hermon_IP
#define NPCMX50_AIC_ACCESS                      MEM
#define NPCMX50_AIC_PHYS_BASE_ADDR              0xF0000000
#define NPCMX50_AIC_INTERRUPT_NUM               32

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_AIC_BASE_ADDR                   AIC_PHYS_BASE_ADDR
#else
#define NPCMX50_AIC_BASE_ADDR                   AIC_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Ethernet 100 Mac Controler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_EMC_MODULE_TYPE                 Hermon_IP
#define NPCMX50_EMC_ACCESS                      MEM
#define NPCMX50_EMC_PHYS_BASE_ADDR(module)      0xF0402000
#define NPCMX50_EMC_RX_INTERRUPT(module)        EMC1_RX_INT
#define NPCMX50_EMC_TX_INTERRUPT(module)        EMC1_TX_INT
#define NPCMX50_EMC_NUM_OF_MODULES              1

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_EMC_BASE_ADDR(module)           EMC_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_EMC_BASE_ADDR(module)           EMC_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* GMAC Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GMAC_MODULE_TYPE                Yarkon_IP
#define NPCMX50_GMAC_ACCESS                     MEM
#define NPCMX50_GMAC_PHYS_BASE_ADDR(module)     0xF040A000            // Only GMAC2 on Yarkon
#define NPCMX50_GMAC_INTERRUPT(module)          GMAC_INT
#define NPCMX50_GMAC_NUM_OF_MODULES             1

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GMAC_BASE_ADDR(module)          GMAC_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_GMAC_BASE_ADDR(module)          GMAC_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Memory Control                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_MC_MODULE_TYPE                  Yarkon_IP
#define NPCMX50_MC_ACCESS                       MEM
#define NPCMX50_MC_PHYS_BASE_ADDR               0xFF000000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_MC_BASE_ADDR                    MC_PHYS_BASE_ADDR
#else
#define NPCMX50_MC_BASE_ADDR                    MC_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* SD Module                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
//#define NPCMX50_SD_MODULE_TYPE                  Hermon_IP
//#define NPCMX50_SD_ACCESS                       MEM
#define NPCMX50_SD_PHYS_BASE_ADDR(module)       (0xF0407000 + (0x1000 * (module)))
#define NPCMX50_SD_GROUP_INTERRUPT(module)      (14 + (module))
#define NPCMX50_SD_INTERRUPT(module)            SDHC_INT
#define NPCMX50_SD_NUM_OF_MODULES               2

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SD_BASE_ADDR(module)            SD_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_SD_BASE_ADDR(module)            SD_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* AES Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_AES_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_AES_ACCESS                      MEM
#define NPCMX50_AES_PHYS_BASE_ADDR              0xF4200000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_AES_BASE_ADDR                   AES_PHYS_BASE_ADDR
#else
#define NPCMX50_AES_BASE_ADDR                   AES_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* DES Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_DES_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_DES_ACCESS                      MEM
#define NPCMX50_DES_PHYS_BASE_ADDR              0xF4201000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_DES_BASE_ADDR                   DES_PHYS_BASE_ADDR
#else
#define NPCMX50_DES_BASE_ADDR                   DES_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* STRAP Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_STRP_MODULE_TYPE                Yarkon_IP

/*---------------------------------------------------------------------------------------------------------*/
/* FUSE Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_FUSE_MODULE_TYPE                Yarkon_IP
#define NPCMX50_FUSE_OTP_ACCESS                 MEM
#define NPCMX50_FUSE_PHYS_BASE_ADDR(module)     (0xF0019000 + ((module) * 0x1000))

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_FUSE_BASE_ADDR(module)          FUSE_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_FUSE_BASE_ADDR(module)          FUSE_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* VCD Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_VCD_MODULE_TYPE                 Hermon_IP
#define NPCMX50_VCD_ACCESS                      MEM
#define NPCMX50_VCD_PHYS_BASE_ADDR              0xF0410000
#define NPCMX50_VCD_INTERRUPT                   VCD_INT
#define NPCMX50_VCD_GROUP_INTERRUPT             12

#define NPCMX50_VCD_MEM_PORT1                   3
#define NPCMX50_VCD_MEM_PORT2                   4

#define NPCMX50_VCD_MAX_WIDTH                   2047
#define NPCMX50_VCD_MAX_HIGHT                   1536
#define NPCMX50_USE_INTERNAL_GFX

#define NPCMX50_VCD_FRAME_A_PHYS_BASE_ADDRESS   0x6C00000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_VCD_BASE_ADDR                   VCD_PHYS_BASE_ADDR
#else
#define NPCMX50_VCD_BASE_ADDR                   VCD_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* GFX Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GFX_MODULE_TYPE                 Hermon_IP
#define NPCMX50_GFX_ACCESS                      MEM
#define NPCMX50_GFX_PHYS_BASE_ADDR              0xF0004000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GFX_BASE_ADDR                   GFX_PHYS_BASE_ADDR
#else
#define NPCMX50_GFX_BASE_ADDR                   GFX_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* ECE Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_ECE_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_ECE_ACCESS                      MEM
#define NPCMX50_ECE_PHYS_BASE_ADDR              0xFF002000
#define NPCMX50_ECE_INTERRUPT                   VCD_INT
#define NPCMX50_ECE_GROUP_INTERRUPT             11

#define NPCMX50_ECE_ED_PHYS_BASE_ADDRESS        0x6800000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_ECE_BASE_ADDR                   ECE_PHYS_BASE_ADDR
#else
#define NPCMX50_ECE_BASE_ADDR                   ECE_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* VDM Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_VDM_MODULE_TYPE                Yarkon_IP
#define NPCMX50_VDM_ACCESS                     MEM
#define NPCMX50_VDM_PHYS_BASE_ADDR     		   (0xF1F01000)
#define NPCMX50_VDMA_PHYS_BASE_ADDR     	   (0xF1F00000)


#ifndef DYNAMIC_BASE_ADDRESS
	#define NPCMX50_VDM_BASE_ADDR         		VDM_PHYS_BASE_ADDR 
	#define NPCMX50_VDMA_BASE_ADDR         		VDMA_PHYS_BASE_ADDR 
#else
	#define NPCMX50_VDM_BASE_ADDR 		        VDM_VIRT_BASE_ADDR 
	#define NPCMX50_VDMA_BASE_ADDR 		        VDMA_VIRT_BASE_ADDR 
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* GPIO Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GPIO_MODULE_TYPE                Hermon_IP
#define NPCMX50_GPIO_ACCESS                     MEM
#define NPCMX50_GPIO_PHYS_BASE_ADDR(module)     (0xF000A000 + ((module)*0x1000))
#define NPCMX50_GPIO_INTERRUPT(gpio)            ((gpio) < 4 ? GPIO_INT0 : (gpio) < 12 ? GPIO_INT1 : (gpio) < 26 ? GPIO_INT2 : GPIO_INT3)

#define NPCMX50_GPIO_NUM_OF_MODULES             2
#define NPCMX50_GPIO_NUM_OF_PORTS               13
#define NPCMX50_GPIO_NUM_OF_GPIOS               208

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GPIO_BASE_ADDR(module)          GPIO_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_GPIO_BASE_ADDR(module)          GPIO_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PSPI Module (aka SSPI)                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PSPI_MODULE_TYPE                Hermon_IP
#define NPCMX50_PSPI_ACCESS                     MEM
#define NPCMX50_PSPI_PHYS_BASE_ADDR(module)     (0xF0005000 + ((module) * 0x1000))
#define NPCMX50_PSPI_NUM_OF_MODULES             2
#define NPCMX50_PSPI_GROUP_INTERRUPT(module)    (9 + (module))


#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PSPI_BASE_ADDR(module)          PSPI_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_PSPI_BASE_ADDR(module)          PSPI_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* AHB2 SRAM Module                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SRAM2_ACCESS                    MEM
#define NPCMX50_SRAM2_PHYS_BASE_ADDR            0xF1FFE000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SRAM2_BASE_ADDR                 SRAM2_PHYS_BASE_ADDR
#else
#define NPCMX50_SRAM2_BASE_ADDR                 SRAM2_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* AHB7 SRAM Module                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SRAM7_ACCESS                    MEM
#define NPCMX50_SRAM7_PHYS_BASE_ADDR            0xF2000000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SRAM7_BASE_ADDR                 SRAM7_PHYS_BASE_ADDR
#else
#define NPCMX50_SRAM7_BASE_ADDR                 SRAM7_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* USB Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
//#define NPCMX50_USB_MODULE_TYPE                 Hermon_IP
#define NPCMX50_USB_ACCESS                      MEM
#define NPCMX50_USB_PHYS_BASE_ADDR(module)      ((module == 1) ? 0xF0406000 : (module == 2) ? 0xF0405000 : (module == 3) ? 0xF0404000 : (module == 4) ? 0xF040E000 : (module == 5) ? 0xF040D000 : 0xF040C000)
#define NPCMX50_USB_INTERRUPT(module)           (((module == 1) || (module == 4)) ? USBD14_INT : ((module == 2) || (module == 5)) ? USBD25_INT : USBD36_INT)
#define NPCMX50_USB_IS_FULL_SPEED(module)       ((module) == 1 ? 1 : 0)
#define NPCMX50_USB_DESC_PHYS_BASE_ADDR(module) ((module % 2) ? SRAM2_PHYS_BASE_ADDR + 0x800 * (module / 2) : SRAM7_PHYS_BASE_ADDR + 0x800 * ((module-1) / 2))
#define NPCMX50_USB_DESC_VIRT_BASE_ADDR(module) ((module % 2) ? SRAM2_BASE_ADDR + 0x800 * (module / 2) : SRAM7_BASE_ADDR + 0x800 * ((module-1) / 2))
#define NPCMX50_USB_NUM_OF_MODULES              6

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_USB_BASE_ADDR(module)           USB_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_USB_BASE_ADDR(module)           USB_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    NPCMX50_SWD_INT         = 0,    /* System Watch Dog and Timer Module 2 Interrupt                               */
    NPCMX50_WDT_INT         = 1,    /* Watch Dog Timer Interrupt                                                   */
    NPCMX50_GPIO_INT0       = 2,    /* GPIO Interrupt Group 0 containing GPIOE3-0                                  */
    NPCMX50_GPIO_INT1       = 3,    /* GPIO Interrupt Group 1 containing GPIOE11-4                                 */
    NPCMX50_GPIO_INT2       = 4,    /* GPIO Interrupt Group 2 containing GPIOE15-12, GPIO25-24                     */
    NPCMX50_GPIO_INT3       = 5,    /* GPIO Interrupt Group 3 containing GPIOE143-128, GPIOE153-152                */
    NPCMX50_PECI_INT        = 6,    /* PECI and VSYNC Interrupt group                                              */
    NPCMX50_UART0_INT       = 7,    /* UART0 Interrupt and VDMA interrupt group                                    */
    NPCMX50_UART1_INT       = 8,    /* UART1 Interrupt                                                             */
    NPCMX50_KCS_HIB_INT     = 9,    /* KCS/HIB Interrupt (from host interface)                                     */
    NPCMX50_FIU_SPI_INT     = 10,   /* FIU_SPI interrupt                                                           */
    NPCMX50_SHM_INT         = 11,   /* SHM Interrupt and PCI mailbox interrupt group                               */
    NPCMX50_T_INT0          = 12,   /* Timer Interrupt 0                                                           */
    NPCMX50_T_INT1          = 13,   /* Timer Interrupt 1                                                           */
    NPCMX50_T_INT_GRP       = 14,   /* Timer Interrupt Group containing Timer2, Timer3, Timer4                     */
    NPCMX50_EMC1_RX_INT     = 15,   /* EMC1 Rx Interrupt                                                           */
    NPCMX50_EMC1_TX_INT     = 16,   /* EMC1 Tx Interrupt                                                           */
    NPCMX50_GMAC_INT        = 17,   /* GMAC Interrupt                                                              */
    NPCMX50_USBD36_INT      = 18,   /* USB Device3 and USB Device6 Interrupt Interrupt                             */
    NPCMX50_SIOX_INT        = 19,   /* SIOX Serial GPIO Expander modules                                           */
    NPCMX50_USBD25_INT      = 20,   /* USB Device2 and USB Device5 Interrupt                                       */
    NPCMX50_USBD14_INT      = 21,   /* USB Device1 and USB Device4 Interrupt                                       */
    NPCMX50_VCD_INT         = 22,   /* VCD interrupt, DVC interrupt and ECE interrupt group                        */
    NPCMX50_SDHC_INT        = 23,   /* SDHC1 and SDHC2 Interrupt                                                   */
    NPCMX50_MFT03_INT       = 24,   /* Tachometer Timer 0-3 (MFT0) Interrupt                                       */
    NPCMX50_MFT47_INT       = 25,   /* Tachometer Timer 4-7 (MFT1) Interrupt                                       */
    NPCMX50_SMB_GRP1_INT    = 26,   /* SMBus Interrupt Group containing SMBus0, SMBus1, SMBus2                     */
    NPCMX50_SMB_GRP3_INT    = 27,   /* SMBus Interrupt Group containing SMBus6, SMBus7                             */
    NPCMX50_PWM_INT         = 28,   /* PWM  Interrupt Group containing PWM0-PWM7                                   */
    NPCMX50_SMB_GRP2_INT    = 29,   /* SMBus Interrupt Group containing SMBus3, SMBus4, SMBus5                     */
    NPCMX50_MCTP_INT        = 30,   /* VDMX module interrupt                                                       */
    NPCMX50_INT31_GRP_INT   = 31,   /* INT31 group containing ADC, PSPI1, PSPI2                                    */
} npcm650_irq_list_t;


/*---------------------------------------------------------------------------------------------------------*/
/* GPIOs definition table                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GPIO_DEFINITION_TABLE                                                                                                                                                                       \
{                                                                                                                                                                                                   \
    /* Port 0: GPIO 0-15 */                                                                                                                                                                         \
    GPIO_TABLE_ENTRY( 0, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(0),  GPIO_CAP_DEBOUNCE(0),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 30, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(1),  GPIO_CAP_DEBOUNCE(1),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 30, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(2),  GPIO_CAP_DEBOUNCE(2),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 30, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(3),  GPIO_CAP_DEBOUNCE(3),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 30, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(4),  GPIO_CAP_DEBOUNCE(4),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 14, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(5),  GPIO_CAP_DEBOUNCE(5),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 14, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(6),  GPIO_CAP_DEBOUNCE(6),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 14, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(7),  GPIO_CAP_DEBOUNCE(7),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 14, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(8),  GPIO_CAP_DEBOUNCE(8),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(9),  GPIO_CAP_DEBOUNCE(9),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(10), GPIO_CAP_DEBOUNCE(10), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 18, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(11), GPIO_CAP_DEBOUNCE(11), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 18, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(12), GPIO_CAP_DEBOUNCE(12), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 24, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(13), GPIO_CAP_DEBOUNCE(13), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 24, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(14), GPIO_CAP_DEBOUNCE(14), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 24, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 0,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(15), GPIO_CAP_DEBOUNCE(15), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 24, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 1: GPIO 16-31 */                                                                                                                                                                        \
    GPIO_TABLE_ENTRY( 1, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(0),  GPIO_MUX_REGID_MFSL3, 15, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 1, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(4),  GPIO_MUX_REGID_MFSL3, 13, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(8),  GPIO_MUX_REGID_MFSL3, 13, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(12), GPIO_MUX_REGID_MFSL3, 13, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(16), GPIO_MUX_REGID_MFSL2, 24, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(20), GPIO_MUX_REGID_MFSL2, 25, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(24), GPIO_MUX_REGID_MFSL2, 26, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(28), GPIO_MUX_REGID_MFSL2, 27, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(16), GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 28, 1, 0,  GPIO_MUX_REGID_MFSL3, 18, 1, 0), \
    GPIO_TABLE_ENTRY( 1, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(17), GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 29, 1, 0,  GPIO_MUX_REGID_MFSL3, 18, 1, 0), \
    GPIO_TABLE_ENTRY( 1,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  2, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 1,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  2, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 1,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  1, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 1,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  1, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 1,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  0, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 1,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  0, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 2: GPIO 32-47 */                                                                                                                                                                        \
    GPIO_TABLE_ENTRY( 2, 0, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  3, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 1, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  4, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  5, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 29, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 28, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 2,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 3: GPIO 48-63 */                                                                                                                                                                        \
    GPIO_TABLE_ENTRY( 3, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 3, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 11, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 12, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 13, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 13, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 30, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 31, 1, 0,  GPIO_MUX_REGID_MFSL3, 19, 1, 0), \
    GPIO_TABLE_ENTRY( 3,13, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 3,14, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 3,15, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 4: GPIO 64-79 */                                                                                                                                                                        \
    GPIO_TABLE_ENTRY( 4, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  0, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  1, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  2, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  3, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  4, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  6, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  7, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  8, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 11, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 13, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 14, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 4,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 15, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 5: GPIO 80-94 */                                                                                                                                                                        \
    GPIO_TABLE_ENTRY( 5, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 16, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 5, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 17, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 5, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 18, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 5, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL2, 19, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 5, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 14, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 5, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 14, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 5, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 14, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 5, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 14, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 5, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 14, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 5, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 14, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 5,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 15, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY( 5,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 16, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 5,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 16, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 5,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 17, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 5,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 17, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* GPIO 95 Undefined */                                                                                                                                                                         \
    GPIO_TABLE_ENTRY( 5,15, GPIO_CAP_NO_INPUT, GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 6: GPIO 96-113 */                                                                                                                                                                       \
    GPIO_TABLE_ENTRY( 6, 0, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 1, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 2, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 3, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 4, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 5, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 6, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 7, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 8, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6, 9, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,10, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,11, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,12, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,13, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,14, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,15, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,16, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 6,17, GPIO_CAP_INPUT,    GPIO_CAP_NO_OUTPUT, GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 7: GPIO 114-127 */                                                                                                                                                                      \
    GPIO_TABLE_ENTRY( 7, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  6, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  6, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  7, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  7, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  8, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1,  8, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 7,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 8: GPIO 128-143 */                                                                                                                                                                      \
    GPIO_TABLE_ENTRY( 8, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(0),  GPIO_CAP_DEBOUNCE(0),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  6, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(1),  GPIO_CAP_DEBOUNCE(1),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  7, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(2),  GPIO_CAP_DEBOUNCE(2),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  7, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(3),  GPIO_CAP_DEBOUNCE(3),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  3, 2, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(4),  GPIO_CAP_DEBOUNCE(4),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 15, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(5),  GPIO_CAP_DEBOUNCE(5),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 15, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(6),  GPIO_CAP_DEBOUNCE(6),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 15, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(7),  GPIO_CAP_DEBOUNCE(7),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 15, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(8),  GPIO_CAP_DEBOUNCE(8),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(9),  GPIO_CAP_DEBOUNCE(9),  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(10), GPIO_CAP_DEBOUNCE(10), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(11), GPIO_CAP_DEBOUNCE(11), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(12), GPIO_CAP_DEBOUNCE(12), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(13), GPIO_CAP_DEBOUNCE(13), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(14), GPIO_CAP_DEBOUNCE(14), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 8,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(15), GPIO_CAP_DEBOUNCE(15), GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 12, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 9: GPIO 144-159 */                                                                                                                                                                      \
    GPIO_TABLE_ENTRY( 9, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(0),  GPIO_MUX_REGID_MFSL2, 20, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(4),  GPIO_MUX_REGID_MFSL2, 21, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(8),  GPIO_MUX_REGID_MFSL2, 22, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(12), GPIO_MUX_REGID_MFSL2, 23, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(16), GPIO_MUX_REGID_MFSL3, 11, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(20), GPIO_MUX_REGID_MFSL3, 11, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(24), GPIO_MUX_REGID_MFSL3, 11, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_BLINK(28), GPIO_MUX_REGID_MFSL3, 11, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(16), GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_EVENT(17), GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY( 9,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 10, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 10: GPIO 160-175 */                                                                                                                                                                     \
    GPIO_TABLE_ENTRY(10, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 21, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 26, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 31, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 26, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 26, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 26, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 26, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 26, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 16, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  0, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 22, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  1, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  1, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  2, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  2, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(10,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  3, 2, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 11: GPIO 176-191 */                                                                                                                                                                     \
    GPIO_TABLE_ENTRY(11, 0, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  3, 2, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  3, 2, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  8, 1, 0,  GPIO_MUX_REGID_MFSL3, 31, 1, 0), \
    GPIO_TABLE_ENTRY(11, 8, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11, 9, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11,10, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11,11, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11,12, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11,13, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11,14, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(11,15, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  5, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
                                                                                                                                                                                                    \
    /* Port 12: GPIO 192-207 */                                                                                                                                                                     \
    GPIO_TABLE_ENTRY(12, 0, GPIO_CAP_NO_INPUT, GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 17, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 1, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 2, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 3, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 4, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 5, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 6, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 7, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12, 8, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL1, 14, 1, 0,  GPIO_MUX_REGID_MFSL3, 15, 1, 0), \
    GPIO_TABLE_ENTRY(12, 9, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3,  9, 1, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12,10, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_NONE,   0, 0, 0,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12,11, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 15, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12,12, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 22, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12,13, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 22, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12,14, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 22, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
    GPIO_TABLE_ENTRY(12,15, GPIO_CAP_INPUT,    GPIO_CAP_OUTPUT,    GPIO_CAP_NO_EVENT,  GPIO_CAP_NO_DEBOUNCE,  GPIO_CAP_NO_BLINK,  GPIO_MUX_REGID_MFSL3, 22, 1, 1,  GPIO_MUX_REGID_NONE,   0, 0, 0), \
}


#endif //_NPCM650_CHIP_H

