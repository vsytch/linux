/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2014 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   npcm750.h                                                                                             */
/*            This file contains chip definitions for the given project                                    */
/*  Project:                                                                                               */
/*            BMC HAL                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _CHIP_NPCM750_H_
#define _CHIP_NPCM750_H_

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                              CHIP                                                       */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Chip Name                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCM750_CHIP
//#define NPCMX50__PALLADIUM_                       1       // Activate Palladium bypasses

/*---------------------------------------------------------------------------------------------------------*/
/* On-Chip POLEG NPCM750 VERSIONS                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCM750_POLEG_Z1                0x00A92750
#define NPCM750_POLEG_Z2                0x04A92750
#define NPCM750_POLEG_A1                0x10A92750

#define NPCMX50_EXT_CLOCK_FREQUENCY_MHZ     25
#define NPCMX50_EXT_CLOCK_FREQUENCY_KHZ     (NPCMX50_EXT_CLOCK_FREQUENCY_MHZ*_1KHz_)
#define NPCMX50_EXT_CLOCK_FREQUENCY_HZ      (NPCMX50_EXT_CLOCK_FREQUENCY_MHZ*_1MHz_)

/*---------------------------------------------------------------------------------------------------------*/
/* On-Chip ROM                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_ROM_BASE_ADDR                  0xFFFF0000
#define NPCMX50_ROM_MEMORY_SIZE                _64KB_
#define NPCMX50_ROMCODE_VER_ADDR               0xFFFF00FC

/*---------------------------------------------------------------------------------------------------------*/
/* External SDRAM Space DDR4                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SDRAM_BASE_ADDR                 0x00000000
#define NPCMX50_SDRAM_MAPPED_SIZE               (_2GB_)

/*---------------------------------------------------------------------------------------------------------*/
/* External Coprocessor (CR16C+) area                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_COPR_BASE_ADDR                  0xF0600000
#define NPCMX50_COPR_MAPPED_SIZE               (_2MB_)

/*---------------------------------------------------------------------------------------------------------*/
/* SPI0 Flash                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SPI0CS0_BASE_ADDR               0x80000000      /* SPI0 direct access CS0  */
#define NPCMX50_SPI0CS1_BASE_ADDR               0x88000000      /* SPI0 direct access CS1  */
#define NPCMX50_SPI0CS2_BASE_ADDR               0x90000000      /* SPI0 direct access CS2  */
#define NPCMX50_SPI0CS3_BASE_ADDR               0x98000000      /* SPI0 direct access CS3  */

#define NPCMX50_SPI3CS0_BASE_ADDR               0xA0000000      /* SPI3 direct access CS0  */
#define NPCMX50_SPI3CS1_BASE_ADDR               0xA8000000      /* SPI3 direct access CS1  */
#define NPCMX50_SPI3CS2_BASE_ADDR               0xB0000000      /* SPI3 direct access CS2  */
#define NPCMX50_SPI3CS3_BASE_ADDR               0xB8000000      /* SPI3 direct access CS3  */

#define NPCMX50_FLASH_MEMORY_SIZE(device)       _128MB_

#define NPCMX50_FLASH_BASE_ADDR(device)         (NPCMX50_SPI0CS0_BASE_ADDR + (device) * (NPCMX50_FLASH_MEMORY_SIZE(device)))

#define NPCMX50_FIU_DEVICES_PER_MODULE           4
#define NPCMX50_FLASH_NUM_OF_DEVICES             8           /* Number of Chip select per FIU module ! */

#define NPCMX50_SPI0_MEMORY_SIZE                _512KB_
#define NPCMX50_SPI3_MEMORY_SIZE                _512KB_
/*---------------------------------------------------------------------------------------------------------*/
/* SPI Expansion Flash                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SPIXCS0_BASE_ADDR               0xF8000000      /* SPIX direct access CS0  */
#define NPCMX50_SPIXCS1_BASE_ADDR               0xF9000000      /* SPIX direct access CS1  */
#define NPCMX50_SPIX_MEMORY_SIZE                _64KB_


/*---------------------------------------------------------------------------------------------------------*/
/* XBus                                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_XBUS_BASE_ADDR                  0xC2000000      /* XBUS direct access  */
#define NPCMX50_XBUS_MAPPED_SIZE                _32MB_

/*---------------------------------------------------------------------------------------------------------*/
/* Memory Mapped IO                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_IOMEMORY_BASE_ADDR              0xF0000000
#define NPCMX50_IOMEMORY_SIZE                   0x0F004000

/*---------------------------------------------------------------------------------------------------------*/
/* RAM3                                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_RAM3_BASE_ADDR                  0xC0008000      /* RAM3  */
#define NPCMX50_RAM3_MAPPED_SIZE                _4KB_


/*---------------------------------------------------------------------------------------------------------*/
/* OTP                                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_OTPROM_BASE_ADDR                0xF0189000      /* OTP ROM  */
#define NPCMX50_OTP2ROM_BASE_ADDR               0xF018A000      /* OTP2 ROM  */
#define NPCMX50_OTPROM_MAPPED_SIZE              _128B_

/*---------------------------------------------------------------------------------------------------------*/
/* On-Chip SRAM     (SRAM2)                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_RAM2_BASE_ADDR                  0xFFFD0000
#define NPCMX50_RAM2_MEMORY_SIZE                _128KB_

/*---------------------------------------------------------------------------------------------------------*/
/* SRAM2 lockable  (Lockable in 2x 1KB pieces)                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SRAM_LOCK_BASE_ADDR                  0xFFFEF800      /* SRAM, lockable  */
#define NPCMX50_SRAM_LOCK_MAPPED_SIZE                _2KB_

/*---------------------------------------------------------------------------------------------------------*/
/* first 256 byte in RAM2 mirrored to this address. if RAMV = 1, default 0 (ROM)                           */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SRAM_VECOTR_BASE_ADDR           0xFFFF0000      /* SRAM, mapped to vector area  */
#define NPCMX50_SRAM_VECOTR_MAPPED_SIZE         _256B_


/* Following modules either new to Poleg, or do not have drivers, or empty allocations on the AHB \ APB */
#define NPCMX50_MAIN_MEM_BASE_ADDR              0x00000000      /* DDR4 Memory Controller  */
#define NPCMX50_AHB1_BASE_ADDR                  0xF0000000      /* AHB1 allocation (Including APB allocations)  */
#define NPCMX50_AHB8_BASE_ADDR                  0xF0800000      /* AHB8 allocation  */
#define NPCMX50_DVC_BASE_ADDR                   0xF0808000      /* DVC (KVM) registers  */
#define NPCMX50_MCR_BASE_ADDR                   0xF0824000      /* Memory Controller Registers  */
#define NPCMX50_CRDRDR_BASE_ADDR                0xF0840000      /* SDHC1: Card reader Control registers SD Controller registers  */
#define NPCMX50_MMC1_BASE_ADDR                  0xF0842000      /* SDHC2: MMC registers  */
#define NPCMX50_GDMA0_BASE_ADDR                 0xF0850000      /* GDMA0  */
#define NPCMX50_GDMA1_BASE_ADDR                 0xF0851000      /* GDMA1  */
#define NPCMX50_GDMA2_BASE_ADDR                 0xF0852000      /* GDMA2  */
#define NPCMX50_GDMA3_BASE_ADDR                 0xF0853000      /* GDMA3  */
#define NPCMX50_SECACC_BASE_ADDR                0xF085B000      /* SECACC  */
#define NPCMX50_AHB18_BASE_ADDR                 0x80000000      /* AHB18 allocation  */
#define NPCMX50_AHB3_BASE_ADDR                  0xA0000000      /* AHB3 allocation  */
#define NPCMX50_XBUSR_BASE_ADDR                 0xC0002000      /* XBUS registers  */
#define NPCMX50_AHB14_BASE_ADDR                 0xE0000000      /* AHB14 Allocation  */
#define NPCMX50_APB14_BASE_ADDR                 0xE0000000      /* APB14 Allocation  */
#define NPCMX50_VDMX_BASE_ADDR                  0xE0800000      /* VDMX  */
#define NPCMX50_APBS_BASE_ADDR                  0xF0000000      /* APBs Allocation  */
#define NPCMX50_APB1_BASE_ADDR                  0xF0000000      /* APB1 Allocation  */
#define NPCMX50_BT_BASE_ADDR                    0xF000D000      /* BT (Block Transfer)  */
#define NPCMX50_SHRDIO_BASE_ADDR                0xF001C000      /* 16B Shared I/O (IBM)  */
#define NPCMX50_APB2_BASE_ADDR                  0xF0080000      /* APB2 Allocation  */
#define NPCMX50_APB2_BASE_ADDR                  0xF0080000      /* APB2 Allocation  */
#define NPCMX50_APB3_BASE_ADDR                  0xF0100000      /* APB3 Allocation  */
#define NPCMX50_PECI_BASE_ADDR                  0xF0100000      /* PECI registers  */
#define NPCMX50_SIOX1_BASE_ADDR                 0xF0101000      /* Serial GPIO Expansion 1  */
#define NPCMX50_SIOX2_BASE_ADDR                 0xF0102000      /* Serial GPIO Expansion 2  */
#define NPCMX50_APB4_BASE_ADDR                  0xF0180000      /* APB4 Allocation  */
#define NPCMX50_THRMS_BASE_ADDR                 0xF0188000      /* Thermal Sensor  */
#define NPCMX50_APB2SIB_BASE_ADDR               0xF018C000      /* APB2SIB  */
#define NPCMX50_APB5_BASE_ADDR                  0xF0200000      /* APB5 Allocation  */
#define NPCMX50_A9_BASE_ADDR                    0xF03FC000      /* Level 2 Cache Registers + A9 Peripherals Registers Allocation  */
#define NPCMX50_DAP_BASE_ADDR                   0xF0500000      /* Cortex A9 Debug Access Port + MCPHY Allocation  */
#define NPCMX50_AHB19_BASE_ADDR                 0xFFF00000      /* AHB19 allocation  */

#define NPCMX50_USBH_EHCI_BASE_ADDR             0xF0806000
#define NPCMX50_USBH_OHCI_BASE_ADDR             0xF0807000

#define NPCMX50_AHB_PCI_BASE_ADDR               0xF0400000      /* AHB to PCI bridge  */

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
/* Multi-Function Timer Module                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_MFT_MODULE_TYPE                 Poleg_IP
#define NPCMX50_MFT_ACCESS                      MEM

#define NPCMX50_MFT0_BASE_ADDR                  0xF0180000      /* Tachmeter MFT 0  */
#define NPCMX50_MFT1_BASE_ADDR                  0xF0181000      /* Tachmeter MFT 1  */
#define NPCMX50_MFT2_BASE_ADDR                  0xF0182000      /* Tachmeter MFT 2  */
#define NPCMX50_MFT3_BASE_ADDR                  0xF0183000      /* Tachmeter MFT 3  */
#define NPCMX50_MFT4_BASE_ADDR                  0xF0184000      /* Tachmeter MFT 4  */
#define NPCMX50_MFT5_BASE_ADDR                  0xF0185000      /* Tachmeter MFT 5  */
#define NPCMX50_MFT6_BASE_ADDR                  0xF0186000      /* Tachmeter MFT 6  */
#define NPCMX50_MFT7_BASE_ADDR                  0xF0187000      /* Tachmeter MFT 7  */

#define NPCMX50_MFT_PHYS_BASE_ADDR(module)      (NPCMX50_MFT0_BASE_ADDR + ((module) * 0x1000L))
#define NPCMX50_MFT_INTERRUPT(module)           (NPCMX50_MFT_INTERRUPT_1 + module)

#define NPCMX50_MFT_NUM_OF_MODULES              2

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_MFT_BASE_ADDR(module)           NPCMX50_MFT_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_MFT_BASE_ADDR(module)           MFT_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* General Ethernet configuration                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
/* EMC1/2  ETH0/ETH1 - see BMC_HAL/EMC and npcmX50_eth.c code */
#define NPCMX50_CHIP_NUM_OF_EMC_ETH             2
#define NPCMX50_CHIP_NUM_OF_ETH                 CHIP_NUM_OF_EMC_ETH
#define NPCMX50_CHIP_ETH0_TYPE                  EMC
#define NPCMX50_CHIP_ETH1_TYPE                  EMC

/* GMAC1/2  ETH2/ETH3 - see GMAC synopsys code */
#define NPCMX50_CHIP_NUM_OF_GMAC_ETH            2
#define NPCMX50_GMAC1_ETH2                      2
#define NPCMX50_GMAC2_ETH3                      3

/*---------------------------------------------------------------------------------------------------------*/
/* General Module  : System Global Registers                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GCR_MODULE_TYPE                 Poleg_IP
#define NPCMX50_GCR_ACCESS                      MEM
#define NPCMX50_GCR_PHYS_BASE_ADDR              0xF0800000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GCR_BASE_ADDR                   NPCMX50_GCR_PHYS_BASE_ADDR
#else
#define NPCMX50_GCR_BASE_ADDR                   GCR_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* UART Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_UART_MODULE_TYPE                Hermon_IP
#define NPCMX50_UART_ACCESS                     MEM
#define NPCMX50_UART0_BASE_ADDR                 0xF0001000      /* UART0 registers  */
#define NPCMX50_UART1_BASE_ADDR                 0xF0002000      /* UART1 registers  */
#define NPCMX50_UART2_BASE_ADDR                 0xF0003000      /* UART2 registers  */
#define NPCMX50_UART3_BASE_ADDR                 0xF0004000      /* UART3 registers  */
#define NPCMX50_UART_PHYS_BASE_ADDR(module)     (NPCMX50_UART0_BASE_ADDR + ((module) * 0x1000))
#define NPCMX50_UART_INTERRUPT(module)          (NPCMX50_UART_INTERRUPT_0 + module)
#define NPCMX50_UART_NUM_OF_MODULES             4

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_UART_BASE_ADDR(module)          NPCMX50_UART_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_UART_BASE_ADDR(module)          UART_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PCI MailBox Module                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#define NPCMX50_PCIMBX_PHYS_BASE_ADDR           0xF0848000      /* PCI Mailbox direct mapping . Mapped till 0xEFFFFFFF. */

#define NPCMX50_PCIMBX_ACCESS                   MEM
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PCIMBX_BASE_ADDR                NPCMX50_PCIMBX_PHYS_BASE_ADDR
#else
#define NPCMX50_PCIMBX_BASE_ADDR                PCIMBX_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PCTL (Memory) Module                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PCTL_ACCESS                     MEM
#define NPCMX50_PCTL_PHYS_BASE_ADDR             0xFF001000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PCTL_BASE_ADDR                  NPCMX50_PCTL_PHYS_BASE_ADDR
#else
#define NPCMX50_PCTL_BASE_ADDR                  PCTL_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Timer Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_TIMER_MODULE_TYPE               Hermon_IP
#define NPCMX50_TIMER_ACCESS                    MEM
#define NPCMX50_TMR0_BASE_ADDR                  0xF0008000      /* Timer0  module registers  */
#define NPCMX50_TMR1_BASE_ADDR                  0xF0009000      /* Timer1  module registers  */
#define NPCMX50_TMR2_BASE_ADDR                  0xF000A000      /* Timer2  module registers  */
#define NPCMX50_TIMER_PHYS_BASE_ADDR(module)    (NPCMX50_TMR0_BASE_ADDR + ((module)*0x1000))
#define NPCMX50_TIMER_INTERRUPT(port)           (NPCMX50_TIMER_INTERRUPT_0 + (port))         /* (TIMER_INTERRUPT_0 to TIMER_INTERRUPT_14) */
#define NPCMX50_TIMER_GROUP_INTERRUPT(port)     ((((port) >=2) && ((port) <5)) ? (16 + ((port)-2)) : AIC_GROUP_INTERRUPT_NONE)
#define NPCMX50_WDG_INTERRUPT(port)             (NPCMX50_WDG_INTERRUPT_0 + (port))           /* (WDG_INTERRUPT_0 to WDG_INTERRUPT_2) */

#define NPCMX50_TIMER_NUM_OF_MODULES            3
#define NPCMX50_TIMER_NUM_OF_PORTS              15

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_TIMER_BASE_ADDR(port)           NPCMX50_TIMER_PHYS_BASE_ADDR(port)
#else
#define NPCMX50_TIMER_BASE_ADDR(port)           TIMER_VIRT_BASE_ADDR(port)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* SMB Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SMB_MODULE_TYPE                 Hermon_IP
#define NPCMX50_SMB_ACCESS                      MEM
#define NPCMX50_SMB_PHYS_BASE_ADDR(module)      (0xF0080000+((module)*0x1000))

#define NPCMX50_SMB_MAX_BUSES                   16
#define NPCMX50_SMB_SCL_GPIOS                   114, 116, 118, 31, 29, 27, 171, 173, 128, 130, 132, 134, 220, 222, 23, 21 
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SMB_BASE_ADDR(module)           NPCMX50_SMB_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_SMB_BASE_ADDR(module)           SMB_VIRT_BASE_ADDR(module)
#endif

#define NPCMX50_SMB_INTERRUPT(module)           (NPCMX50_SMB_INTERRUPT_0 + module)


/*---------------------------------------------------------------------------------------------------------*/
/* KCS Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_KCS_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_KCS_ACCESS                      MEM
#define NPCMX50_KCS_PHYS_BASE_ADDR              0xF0007000

#define NPCMX50_KCS_MAX_CHANNELS                3
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_KCS_BASE_ADDR                   NPCMX50_KCS_PHYS_BASE_ADDR
#else
#define NPCMX50_KCS_BASE_ADDR                   KCS_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_ADC_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_ADC_ACCESS                      MEM
#define NPCMX50_ADC_PHYS_BASE_ADDR              0xF000C000

#define NPCMX50_ADC_MAX_CHANNELS                8
#define NPCMX50_ADC_GROUP_INTERRUPT             8
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_ADC_BASE_ADDR                   NPCMX50_ADC_PHYS_BASE_ADDR
#else
#define NPCMX50_ADC_BASE_ADDR                   ADC_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PWM Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PWM_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_PWM_ACCESS                      MEM
#define NPCMX50_PWM_PHYS_BASE_ADDR(module)      (0xF0103000 + (0x1000 * (module)))

#define NPCMX50_PWM_MAX_MODULES                 2
#define NPCMX50_PWM_MAX_PORTS                   8
 
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PWM_BASE_ADDR(module)           NPCMX50_PWM_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_PWM_BASE_ADDR(module)           PWM_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* FIU Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_FIU_MODULE_TYPE                 Poleg_IP
#define NPCMX50_FIU_ACCESS                      MEM

#define NPCMX50_FIU0_BASE_ADDR                  0xFB000000      /* SPI0 registers  */
#define NPCMX50_FIUX_BASE_ADDR                  0xFB001000      /* SPIX registers  */
#define NPCMX50_FIU3_BASE_ADDR                  0xC0000000      /* SPI3 registers  */

#define NPCMX50_FIU_PHYS_BASE_ADDR(n)           ( ( (n) == 0 ) ? NPCMX50_FIU0_BASE_ADDR : ( (n) == 3 ) ? NPCMX50_FIU3_BASE_ADDR : NPCMX50_FIUX_BASE_ADDR)
#define NPCMX50_FIU_NUM_OF_MODULES              3
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_FIU_BASE_ADDR(module)           NPCMX50_FIU_PHYS_BASE_ADDR(module)    // Use this when handle FIU for on Poleg_IP inside HAL
#else
#define NPCMX50_FIU_BASE_ADDR(module)           FIU_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Clock Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_CLK_MODULE_TYPE                 Poleg_IP
#define NPCMX50_CLK_ACCESS                      MEM
#define NPCMX50_CLK_PHYS_BASE_ADDR              0xF0801000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_CLK_BASE_ADDR                   NPCMX50_CLK_PHYS_BASE_ADDR
#else
#define NPCMX50_CLK_BASE_ADDR                   CLK_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Shared Memory (SHM) Module                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SHM_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_SHM_ACCESS                      MEM
#define NPCMX50_SHM_PHYS_BASE_ADDR              0xC0001000
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SHM_BASE_ADDR                   NPCMX50_SHM_PHYS_BASE_ADDR
#else
#define NPCMX50_SHM_BASE_ADDR                   SHM_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* AIC/GIC A9 Peripheral Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_AIC_MODULE_TYPE                 Poleg_IP      // Using AIC API only.
#define NPCMX50_AIC_ACCESS                      MEM
#define NPCMX50_AIC_PHYS_BASE_ADDR              0xF03FE000

#define NPCMX50_GIC_INTERRUPT_NUM               160           // GIC (INTC for A9) first 32 interrupts are not used (128 irq's connected)
#define NPCMX50_GIC_CPU_INTERFACE_OFFSET        0x0100
#define NPCMX50_GIC_DISTRIBUTOR_OFFSET          0x1000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_AIC_BASE_ADDR                   NPCMX50_AIC_PHYS_BASE_ADDR
#else
#define NPCMX50_AIC_BASE_ADDR                   AIC_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* SCU A9 Peripheral Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SCU_PHYS_BASE_ADDR              0xF03FE000
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SCU_BASE_ADDR                   NPCMX50_SCU_PHYS_BASE_ADDR
#else
#define NPCMX50_SCU_BASE_ADDR                   SCU_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* TWD A9 Peripheral Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_TWD_PHYS_BASE_ADDR              0xF03FE600
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_TWD_BASE_ADDR                   NPCMX50_TWD_PHYS_BASE_ADDR
#else
#define NPCMX50_TWD_BASE_ADDR                   TWD_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Ethernet 100 Mac Controler    (10/100 Ethernet Controller)                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_EMC_MODULE_TYPE                 Hermon_IP
#define NPCMX50_EMC_ACCESS                      MEM
#define NPCMX50_EMC1_BASE_ADDR                  0xF0825000      /* EMC1 (10/100 Ethernet Controller)  */
#define NPCMX50_EMC2_BASE_ADDR                  0xF0826000      /* EMC2 (10/100 Ethernet Controller)  */
#define NPCMX50_EMC_PHYS_BASE_ADDR(module)      ((module == 0) ? NPCMX50_EMC1_BASE_ADDR : NPCMX50_EMC2_BASE_ADDR)
#define NPCMX50_EMC_RX_INTERRUPT(module)        ((module == 0) ? NPCMX50_EMC1RX_INTERRUPT  : NPCMX50_EMC2RX_INTERRUPT)
#define NPCMX50_EMC_TX_INTERRUPT(module)        ((module == 0) ? NPCMX50_EMC1TX_INTERRUPT  : NPCMX50_EMC2TX_INTERRUPT)
#define NPCMX50_EMC_NUM_OF_MODULES              2

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_EMC_BASE_ADDR(module)           NPCMX50_EMC_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_EMC_BASE_ADDR(module)           EMC_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* GMAC Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GMAC_MODULE_TYPE                Yarkon_IP
#define NPCMX50_GMAC_ACCESS                     MEM
#define NPCMX50_GMAC1_BASE_ADDR                 0xF0802000
#define NPCMX50_GMAC2_BASE_ADDR                 0xF0804000
#define NPCMX50_GMAC_PHYS_BASE_ADDR(module)      ((module == 0) ? NPCMX50_GMAC1_BASE_ADDR : NPCMX50_GMAC2_BASE_ADDR)
#define NPCMX50_GMAC_INTERRUPT(module)           ((module == 0) ? NPCMX50_GMAC1_INTERRUPT : NPCMX50_GMAC2_INTERRUPT)
#define NPCMX50_GMAC_NUM_OF_MODULES             2

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GMAC_BASE_ADDR(module)          NPCMX50_GMAC_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_GMAC_BASE_ADDR(module)          GMAC_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Memory Control                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_MC_MODULE_TYPE                  Poleg_IP
#define NPCMX50_MC_ACCESS                       MEM
#define NPCMX50_MC_PHYS_BASE_ADDR               0xF0824000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_MC_BASE_ADDR                    NPCMX50_MC_PHYS_BASE_ADDR
#else
#define NPCMX50_MC_BASE_ADDR                    MC_VIRT_BASE_ADDR
#endif

#define NPCMX50_MCPHY_ACCESS                    MEM
#define NPCMX50_MCPHY_PHYS_BASE_ADDR            0xF05F0000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_MCPHY_BASE_ADDR                 NPCMX50_MCPHY_PHYS_BASE_ADDR
#else
#define NPCMX50_MCPHY_BASE_ADDR                 MCPHY_VIRT_BASE_ADDR
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* SD Module                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SD_MODULE_TYPE                  Hermon_IP
#define NPCMX50_SD_ACCESS                       MEM
#define NPCMX50_SD_PHYS_BASE_ADDR(module)       (0xF0840000 + (0x2000 * (module)))
#define NPCMX50_SD_GROUP_INTERRUPT(module)      (14 + (module))
#define NPCMX50_SD_INTERRUPT(module)            (((module) == 0)? NPCMX50_SDRDR_INTERRUPT : NPCMX50_MMC_INTERRUPT)
#define NPCMX50_SD_NUM_OF_MODULES               2
#define NPCMX50_SD_CLK_TARGET_FREQ_Z1           (48 * _1MHz_)
#define NPCMX50_SD_CLK_TARGET_FREQ_A1           (50 * _1MHz_)


#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SD_BASE_ADDR(module)            NPCMX50_SD_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_SD_BASE_ADDR(module)            SD_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* AES Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
// #define NPCMX50_AES_MODULE_TYPE                 Yarkon_IP /* Using driver under /drivers/crypto/npcmx50/NPCMX50_aes */
#define NPCMX50_AES_ACCESS                      MEM
#define NPCMX50_AES_PHYS_BASE_ADDR              0xF0858000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_AES_BASE_ADDR                   NPCMX50_AES_PHYS_BASE_ADDR
#else
#define NPCMX50_AES_BASE_ADDR                   AES_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* DES Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_DES_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_DES_ACCESS                      MEM
#define NPCMX50_DES_PHYS_BASE_ADDR              0xF0859000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_DES_BASE_ADDR                   NPCMX50_DES_PHYS_BASE_ADDR
#else
#define NPCMX50_DES_BASE_ADDR                   DES_VIRT_BASE_ADDR
#endif
#define NPCMX50_DES_INTERRUPT_POLARITY           INTERRUPT_POLARITY_LEVEL_HIGH
#define NPCMX50_DES_INTERRUPT_PRIORITY           0
#define NPCMX50_DES_INTERRUPT_PROVIDER           CHIP_INTERRUPT_PROVIDER
#define NPCMX50_DES_NUM_OF_INPUTS                3
#define NPCMX50_DES_CLK                          2000000
#define NPCMX50_DES_SOURCE_CLOCK                 PLL2
#define NPCMX50_DES_MUX(input)                   CHIP_MuxDES(input)


/*---------------------------------------------------------------------------------------------------------*/
/* STRAP Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_STRP_MODULE_TYPE                Poleg_IP

#define NPCMX50_STRP_INTERRUPT_POLARITY          INTERRUPT_POLARITY_LEVEL_HIGH
#define NPCMX50_STRP_INTERRUPT_PRIORITY          0
#define NPCMX50_STRP_INTERRUPT_PROVIDER          CHIP_INTERRUPT_PROVIDER
#define NPCMX50_STRP_NUM_OF_INPUTS               3
#define NPCMX50_STRP_CLK                         2000000
#define NPCMX50_STRP_SOURCE_CLOCK                PLL2
#define NPCMX50_STRP_MUX(input)                  CHIP_MuxSTRP(input)


/*---------------------------------------------------------------------------------------------------------*/
/* FUSE Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_FUSE_MODULE_TYPE                Poleg_IP
#define NPCMX50_FUSE_OTP_ACCESS                 MEM
#define NPCMX50_FUSE_PHYS_BASE_ADDR(module)     (0xF0189000 + ((module) * 0x1000))

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_FUSE_BASE_ADDR(module)          NPCMX50_FUSE_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_FUSE_BASE_ADDR(module)          FUSE_VIRT_BASE_ADDR(module)
#endif
#define NPCMX50_FUSE_INTERRUPT_POLARITY          INTERRUPT_POLARITY_LEVEL_HIGH
#define NPCMX50_FUSE_INTERRUPT_PRIORITY          0
#define NPCMX50_FUSE_INTERRUPT_PROVIDER          CHIP_INTERRUPT_PROVIDER
#define NPCMX50_FUSE_NUM_OF_INPUTS               3
#define NPCMX50_FUSE_CLK                         2000000
#define NPCMX50_FUSE_SOURCE_CLOCK                PLL2
#define NPCMX50_FUSE_MUX(input)                  CHIP_MuxFUSE(input)


/*---------------------------------------------------------------------------------------------------------*/
/* VCD Module  (VIDEO CAPTURE AND DIFFERENTIATION)                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_VCD_MODULE_TYPE                 Hermon_IP
#define NPCMX50_VCD_ACCESS                      MEM
#define NPCMX50_VCD_PHYS_BASE_ADDR              0xF0810000

#define NPCMX50_VCD_MEM_PORT1                   3
#define NPCMX50_VCD_MEM_PORT2                   4

#define NPCMX50_VCD_MAX_WIDTH                   2032
#define NPCMX50_VCD_MAX_HIGHT                   1536

#define NPCMX50_USE_INTERNAL_GFX

#define NPCMX50_VCD_FRAME_A_PHYS_BASE_ADDRESS   0x3E200000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_VCD_BASE_ADDR                   NPCMX50_VCD_PHYS_BASE_ADDR
#else
#define NPCMX50_VCD_BASE_ADDR                   VCD_VIRT_BASE_ADDR
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* GFX Module                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GFX_MODULE_TYPE                 Hermon_IP
#define NPCMX50_GFX_ACCESS                      MEM

#define NPCMX50_GFXI_BASE_ADDR                  0xF000E000      /* Graphics Core information registers  */
#define NPCMX50_GFXOL_BASE_ADDR                 0xF0828000      /* GFX Overlay FIFO and registers  */

#define NPCMX50_GFX_PHYS_BASE_ADDR              NPCMX50_GFXI_BASE_ADDR

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GFX_BASE_ADDR                   NPCMX50_GFX_PHYS_BASE_ADDR
#else
#define NPCMX50_GFX_BASE_ADDR                   GFX_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* ECE Module:  Video Compression map ECE                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_ECE_MODULE_TYPE                 Poleg_IP
#define NPCMX50_ECE_ACCESS                      MEM
#define NPCMX50_ECE_PHYS_BASE_ADDR              0xF0820000
#define NPCMX50_ECE_ED_PHYS_BASE_ADDRESS        0x3E800000
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_ECE_BASE_ADDR                   NPCMX50_ECE_PHYS_BASE_ADDR
#else
#define NPCMX50_ECE_BASE_ADDR                   ECE_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* VDM Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_VDM_MODULE_TYPE                 Yarkon_IP
#define NPCMX50_VDM_ACCESS                      MEM
#define NPCMX50_VDM_PHYS_BASE_ADDR     	        (0xE0800000)
#define NPCMX50_VDMA_PHYS_BASE_ADDR             (0xF0822000)

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_VDM_BASE_ADDR         	        NPCMX50_VDM_PHYS_BASE_ADDR 
#define NPCMX50_VDMA_BASE_ADDR         	        NPCMX50_VDMA_PHYS_BASE_ADDR 
#else
#define NPCMX50_VDM_BASE_ADDR 	                VDM_VIRT_BASE_ADDR 
#define NPCMX50_VDMA_BASE_ADDR 	                VDMA_VIRT_BASE_ADDR 
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PCIE RC Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PCIERC_MODULE_TYPE                 Poleg_IP
#define NPCMX50_PCIERC_ACCESS                      MEM
#define NPCMX50_PCIERC_PHYS_BASE_ADDR     	        (0xE1000000)

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PCIERC_BASE_ADDR         	        NPCMX50_PCIERC_PHYS_BASE_ADDR 
#else
#define NPCMX50_PCIERC_BASE_ADDR 	                PCIERC_VIRT_BASE_ADDR 
#endif

#define NPCMX50_PCIERC_NUM_OF_MSI					32

/*---------------------------------------------------------------------------------------------------------*/
/* GPIO Module                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_GPIO_MODULE_TYPE                Hermon_IP
#define NPCMX50_GPIO_ACCESS                     MEM
#define NPCMX50_GPIO0_BASE_ADDR                 0xF0010000      /* GPIO0 module registers  */
#define NPCMX50_GPIO1_BASE_ADDR                 0xF0011000      /* GPIO1 module registers  */
#define NPCMX50_GPIO2_BASE_ADDR                 0xF0012000      /* GPIO2 module registers  */
#define NPCMX50_GPIO3_BASE_ADDR                 0xF0013000      /* GPIO3 module registers  */
#define NPCMX50_GPIO4_BASE_ADDR                 0xF0014000      /* GPIO4 module registers  */
#define NPCMX50_GPIO5_BASE_ADDR                 0xF0015000      /* GPIO5 module registers  */
#define NPCMX50_GPIO6_BASE_ADDR                 0xF0016000      /* GPIO6 module registers  */
#define NPCMX50_GPIO7_BASE_ADDR                 0xF0017000      /* GPIO7 module registers  */
#define NPCMX50_GPIO_PHYS_BASE_ADDR(port)       (NPCMX50_GPIO0_BASE_ADDR + ((port)*0x1000))
#define NPCMX50_GPIO_INTERRUPT(gpio)            ((gpio) < 32  ? NPCMX50_GPIO_INTERRUPT0 : (gpio) < 64  ? NPCMX50_GPIO_INTERRUPT1 : (gpio) < 96  ? NPCMX50_GPIO_INTERRUPT2 :   \
                                         (gpio) < 128 ? NPCMX50_GPIO_INTERRUPT3 : (gpio) < 160 ? NPCMX50_GPIO_INTERRUPT4 : (gpio) < 192 ? NPCMX50_GPIO_INTERRUPT5 :  \
                                         (gpio) < 224 ? NPCMX50_GPIO_INTERRUPT6 :  NPCMX50_GPIO_INTERRUPT7)

#define NPCMX50_GPIO_NUM_OF_MODULES              1
#define NPCMX50_GPIO_NUM_OF_PORTS                8
#define NPCMX50_GPIO_NUM_OF_GPIOS                256

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_GPIO_BASE_ADDR(port)           NPCMX50_GPIO_PHYS_BASE_ADDR(port)
#else
#define NPCMX50_GPIO_BASE_ADDR(port)           GPIO_VIRT_BASE_ADDR(port)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* PSPI Module (aka SSPI)                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PSPI_MODULE_TYPE                Hermon_IP
#define NPCMX50_PSPI_ACCESS                     MEM
#define NPCMX50_PSPI1_BASE_ADDR                 0xF0200000      /* Peripheral SPI 1 registers  */
#define NPCMX50_PSPI2_BASE_ADDR                 0xF0201000      /* Peripheral SPI 2 registers  */

#define NPCMX50_ESPI_BASE_ADDR                  0xF008F000      /* eSPI registers  */


#define NPCMX50_PSPI_PHYS_BASE_ADDR(module)     (NPCMX50_PSPI1_BASE_ADDR + ((module) * 0x1000))
#define NPCMX50_PSPI_NUM_OF_MODULES             2

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PSPI_BASE_ADDR(module)          NPCMX50_PSPI_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_PSPI_BASE_ADDR(module)          PSPI_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* AHB2 SRAM Module (SRAM2)                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SRAM2_ACCESS                    MEM
#define NPCMX50_SRAM2_PHYS_BASE_ADDR             NPCMX50_RAM2_BASE_ADDR
#define NPCMX50_SRAM2_MEMORY_SIZE               NPCMX50_RAM2_MEMORY_SIZE

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SRAM2_BASE_ADDR                 NPCMX50_SRAM2_PHYS_BASE_ADDR
#else
#define NPCMX50_SRAM2_BASE_ADDR                 SRAM2_VIRT_BASE_ADDR
#endif

#define NPCMX50_SRAM_BASE_ADDR                  NPCMX50_SRAM2_BASE_ADDR
#define NPCMX50_SRAM_MEMORY_SIZE                NPCMX50_SRAM2_MEMORY_SIZE

/*---------------------------------------------------------------------------------------------------------*/
/* USB Module                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_USB_MODULE_TYPE                 Hermon_IP
#define NPCMX50_USB_ACCESS                      MEM
#define NPCMX50_UDC_EPQ_DTD_SIZE                0x800

#define NPCMX50_USB0_BASE_ADDR                  0xF0830000      /* USB (2.0) Device 0 registers  */
#define NPCMX50_USB1_BASE_ADDR                  0xF0831000      /* USB (2.0) Device 1 registers  */
#define NPCMX50_USB2_BASE_ADDR                  0xF0832000      /* USB (2.0) Device 2 registers  */
#define NPCMX50_USB3_BASE_ADDR                  0xF0833000      /* USB (2.0) Device 3 registers  */
#define NPCMX50_USB4_BASE_ADDR                  0xF0834000      /* USB (2.0) Device 4 registers  */
#define NPCMX50_USB5_BASE_ADDR                  0xF0835000      /* USB (2.0) Device 5 registers  */
#define NPCMX50_USB6_BASE_ADDR                  0xF0836000      /* USB (2.0) Device 6 registers  */
#define NPCMX50_USB7_BASE_ADDR                  0xF0837000      /* USB (2.0) Device 7 registers  */
#define NPCMX50_USB8_BASE_ADDR                  0xF0838000      /* USB (2.0) Device 8 registers  */
#define NPCMX50_USB9_BASE_ADDR                  0xF0839000      /* USB (2.0) Device 9 registers  */

#define NPCMX50_USB_PHYS_BASE_ADDR(module)      (NPCMX50_USB0_BASE_ADDR + (0x1000 * module))

#define NPCMX50_USB_INTERRUPT(module)           (NPCMX50_USB_DEV_INTERRUPT_0 + module)
#define NPCMX50_USB_IS_FULL_SPEED(module)       0
#define NPCMX50_USB_DESC_PHYS_BASE_ADDR(module) (NPCMX50_SRAM2_PHYS_BASE_ADDR + NPCMX50_UDC_EPQ_DTD_SIZE * (module))
#define NPCMX50_USB_DESC_VIRT_BASE_ADDR(module) (NPCMX50_SRAM2_BASE_ADDR + NPCMX50_UDC_EPQ_DTD_SIZE * (module))
#define NPCMX50_USB_NUM_OF_MODULES              10

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_USB_BASE_ADDR(module)           NPCMX50_USB_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_USB_BASE_ADDR(module)           USB_VIRT_BASE_ADDR(module)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* L2 cache Module                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_L2_CACHE_PHYS_BASE_ADDR         NPCMX50_A9_BASE_ADDR

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_L2_CACHE_BASE_ADDR              NPCMX50_L2_CACHE_PHYS_BASE_ADDR
#else
#define NPCMX50_L2_CACHE_BASE_ADDR              L2_CACHE_VIRT_BASE_ADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Public Key Accelerator Module                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_PKA_MODULE_TYPE                  4
#define NPCMX50_PKA_ACCESS                       MEM
#define NPCMX50_PKA_PHYS_BASE_ADDR               0xF085B000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_PKA_BASE_ADDR                   NPCMX50_PKA_PHYS_BASE_ADDR
#else
#define NPCMX50_PKA_BASE_ADDR                   PKA_VIRT_BASE_ADDR
#endif

#define NPCMX50_PKA_INT_NUM                      NPCMX50_PKA_INTERRUPT
#define NPCMX50_PKA_INT_POLARITY                 INTERRUPT_POLARITY_LEVEL_HIGH
#define NPCMX50_PKA_INTERRUPT_PRIORITY           0
#define NPCMX50_PKA_INT_PROVIDER                 CHIP_INTERRUPT_PROVIDER
#define NPCMX50_PKA_NUM_OF_MODULES               1
#define NPCMX50_PKA_SOURCE_CLOCK(module)         PLL2
#define NPCMX50_PKA_MUX(module)                  CHIP_MuxPKA(module)


/*---------------------------------------------------------------------------------------------------------*/
/* SHA-1 and SHA-256 Module                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_SHA_MODULE_TYPE                  1
#define NPCMX50_SHA_ACCESS                       MEM
#define NPCMX50_SHA_PHYS_BASE_ADDR               0xF085A000

#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_SHA_BASE_ADDR                   NPCMX50_SHA_PHYS_BASE_ADDR
#else
#define NPCMX50_SHA_BASE_ADDR                   SHA_VIRT_BASE_ADDR
#endif

//#define NPCMX50_SHA_INTERRUPT(module)            (SHA_INTERRUPT)
#define NPCMX50_SHA_INTERRUPT_POLARITY            INTERRUPT_POLARITY_LEVEL_HIGH
#define NPCMX50_SHA_INTERRUPT_PRIORITY           0
#define NPCMX50_SHA_INTERRUPT_PROVIDER           CHIP_INTERRUPT_PROVIDER
#define NPCMX50_SHA_NUM_OF_MODULES               1
#define NPCMX50_SHA_SOURCE_CLOCK(module)         PLL2
#define NPCMX50_SHA_MUX(module)                  CHIP_MuxSHA(module)


/*---------------------------------------------------------------------------------------------------------*/
/* RNG                                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_RNG_MODULE_TYPE                 2
#define NPCMX50_RNG_PHYS_BASE_ADDR              0xF000B000      /* RNG  */
#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_RNG_BASE_ADDR                   NPCMX50_RNG_PHYS_BASE_ADDR
#else
#define NPCMX50_RNG_BASE_ADDR                   RNG_VIRT_BASE_ADDR
#endif
#define NPCMX50_RNG_ACCESS                      MEM

//#define NPCMX50_RNG_INT_NUM                     RNG_INTERRUPT  
//#define NPCMX50_RNG_INT_POLARITY                INTERRUPT_POLARITY_RISING_EDGE
//#define NPCMX50_RNG_INT_PROVIDER                CHIP_INTERRUPT_PROVIDER
//#define NPCMX50_RNG_INTERRUPT_PRIORITY          0


/* Notice: RNG does not have an intterupt. current version of RNG driver works only with interrupts. 
   Before using the driver need to change it to do polling too */

#define NPCMX50_RNG_PRESCALER_CLOCK             CLK_GetAPBFreq(APB1)


/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_IRQ_GIC_START 32

typedef enum
{
	NPCMX50_TWD_LOCALTIMER_INTERRUPT 	 = 29,
	NPCMX50_ADC_INTERRUPT                = 0 + NPCMX50_IRQ_GIC_START,     /* ADC Module                                         */
	NPCMX50_COPR_INTERRUPT               = 1 + NPCMX50_IRQ_GIC_START,     /* Coprocessor subsystem                              */
	NPCMX50_UART_INTERRUPT_0             = 2 + NPCMX50_IRQ_GIC_START,     /* UART 0 Module                                      */
	NPCMX50_UART_INTERRUPT_1             = 3 + NPCMX50_IRQ_GIC_START,     /* UART 1 Module                                      */
	NPCMX50_UART_INTERRUPT_2             = 4 + NPCMX50_IRQ_GIC_START,     /* UART 2 Module                                      */
	NPCMX50_UART_INTERRUPT_3             = 5 + NPCMX50_IRQ_GIC_START,     /* UART 3 Module                                      */
	NPCMX50_PECI_INTERRUPT               = 6 + NPCMX50_IRQ_GIC_START,     /* PECI module                                        */
	NPCMX50_VSYNC_INTERRUPT              = 7 + NPCMX50_IRQ_GIC_START,     /* Graphics module via System Manager module          */
	NPCMX50_PCIMBX_INTERRUPT             = 8 + NPCMX50_IRQ_GIC_START,     /* PCI mailbox module                                 */
	NPCMX50_KCS_HIB_INTERRUPT            = 9 + NPCMX50_IRQ_GIC_START,     /* KCS/HIB  (from host interface) modules             */
	NPCMX50_LPC_MBX_INTERRUPT            = 10 + NPCMX50_IRQ_GIC_START,    /* LPC or eSPI Mailbox (new, if we do)                */
	NPCMX50_SHM_INTERRUPT                = 11 + NPCMX50_IRQ_GIC_START,    /* SHM module                                         */
	NPCMX50_PS2_INTERRUPT                = 12 + NPCMX50_IRQ_GIC_START,    /* PS/2 Module ???                                    */
	NPCMX50_BT_INTERRUPT                 = 13 + NPCMX50_IRQ_GIC_START,    /* Block transfer, If we do                           */
	NPCMX50_GMAC1_INTERRUPT              = 14 + NPCMX50_IRQ_GIC_START,    /* GMAC1 module                                       */
	NPCMX50_EMC1RX_INTERRUPT             = 15 + NPCMX50_IRQ_GIC_START,    /* EMC1 Rx module                                     */
	NPCMX50_EMC1TX_INTERRUPT             = 16 + NPCMX50_IRQ_GIC_START,    /* EMC1 Tx module                                     */
	NPCMX50_GMAC2_INTERRUPT              = 17 + NPCMX50_IRQ_GIC_START,    /* GMAC2 module                                       */
	NPCMX50_ESPI_INTERRUPT               = 18 + NPCMX50_IRQ_GIC_START,    /* eSPI Module                                        */
	NPCMX50_SIOX_INTERRUPT_1             = 19 + NPCMX50_IRQ_GIC_START,    /* SIOX Serial GPIO Expander module 1                 */
	NPCMX50_SIOX_INTERRUPT_2             = 20 + NPCMX50_IRQ_GIC_START,    /* SIOX Serial GPIO Expander module 2                 */
	NPCMX50_L2_CACHE_ERR                 = 21 + NPCMX50_IRQ_GIC_START,    /* A combined interrupt on L2 Cahche errors           */
	NPCMX50_VCD_INTERRUPT                = 22 + NPCMX50_IRQ_GIC_START,    /* VCD module                                         */
	NPCMX50_DVC_INTERRUPT                = 23 + NPCMX50_IRQ_GIC_START,    /* DVC module                                         */
	NPCMX50_ECE_INTERRUPT                = 24 + NPCMX50_IRQ_GIC_START,    /* ECE module                                         */
	NPCMX50_MC_INTERRUPT                 = 25 + NPCMX50_IRQ_GIC_START,    /* Memory Controller Interrupt                        */
	NPCMX50_MMC_INTERRUPT                = 26 + NPCMX50_IRQ_GIC_START,    /* MMC Module (SDHC2)                                 */
	NPCMX50_SDRDR_INTERRUPT              = 27 + NPCMX50_IRQ_GIC_START,    /* SDHC1 - SD Card Reader side interface (if required)*/
	NPCMX50_PSPI2_INTERRUPT              = 28 + NPCMX50_IRQ_GIC_START,    /* Slow Peripheral SPI2                               */
	NPCMX50_VDMA_INTERRUPT               = 29 + NPCMX50_IRQ_GIC_START,    /* VDMA module                                        */
	NPCMX50_MCTP_INTERRUPT               = 30 + NPCMX50_IRQ_GIC_START,    /* VDMX module. Not used if VDMA used                 */
	NPCMX50_PSPI1_INTERRUPT              = 31 + NPCMX50_IRQ_GIC_START,    /* Slow Peripheral SPI1                               */
	NPCMX50_TIMER_INTERRUPT_0            = 32 + NPCMX50_IRQ_GIC_START,    /* Timer Module 0 Timer 0                             */
	NPCMX50_TIMER_INTERRUPT_1            = 33 + NPCMX50_IRQ_GIC_START,    /* Timer Module 0 Timer 1                             */
	NPCMX50_TIMER_INTERRUPT_2            = 34 + NPCMX50_IRQ_GIC_START,    /* Timer Module 0 Timer 2                             */
	NPCMX50_TIMER_INTERRUPT_3            = 35 + NPCMX50_IRQ_GIC_START,    /* Timer Module 0 Timer 3                             */
	NPCMX50_TIMER_INTERRUPT_4            = 36 + NPCMX50_IRQ_GIC_START,    /* Timer Module 0 Timer 4                             */
	NPCMX50_TIMER_INTERRUPT_5            = 37 + NPCMX50_IRQ_GIC_START,    /* Timer Module 1 Timer 0                             */
	NPCMX50_TIMER_INTERRUPT_6            = 38 + NPCMX50_IRQ_GIC_START,    /* Timer Module 1 Timer 1                             */
	NPCMX50_TIMER_INTERRUPT_7            = 39 + NPCMX50_IRQ_GIC_START,    /* Timer Module 1 Timer 2                             */
	NPCMX50_TIMER_INTERRUPT_8            = 40 + NPCMX50_IRQ_GIC_START,    /* Timer Module 1 Timer 3                             */
	NPCMX50_TIMER_INTERRUPT_9            = 41 + NPCMX50_IRQ_GIC_START,    /* Timer Module 1 Timer 4                             */
	NPCMX50_TIMER_INTERRUPT_10           = 42 + NPCMX50_IRQ_GIC_START,    /* Timer Module 2 Timer 0                             */
	NPCMX50_TIMER_INTERRUPT_11           = 43 + NPCMX50_IRQ_GIC_START,    /* Timer Module 2 Timer 1                             */
	NPCMX50_TIMER_INTERRUPT_12           = 44 + NPCMX50_IRQ_GIC_START,    /* Timer Module 2 Timer 2                             */
	NPCMX50_TIMER_INTERRUPT_13           = 45 + NPCMX50_IRQ_GIC_START,    /* Timer Module 2 Timer 3                             */
	NPCMX50_TIMER_INTERRUPT_14           = 46 + NPCMX50_IRQ_GIC_START,    /* Timer Module 2 Timer 4                             */
	NPCMX50_WDG_INTERRUPT_0              = 47 + NPCMX50_IRQ_GIC_START,    /* Timer Module 0 watchdog (also on NMI)              */
	NPCMX50_WDG_INTERRUPT_1              = 48 + NPCMX50_IRQ_GIC_START,    /* Timer Module 1 watchdog (also on NMI)              */
	NPCMX50_WDG_INTERRUPT_2              = 49 + NPCMX50_IRQ_GIC_START,    /* Timer Module 2 watchdog (also on NMI)              */
	NPCMX50_USB_DEV_INTERRUPT_0          = 51 + NPCMX50_IRQ_GIC_START,    /* USB Device 0                                       */
	NPCMX50_USB_DEV_INTERRUPT_1          = 52 + NPCMX50_IRQ_GIC_START,    /* USB Device 1                                       */
	NPCMX50_USB_DEV_INTERRUPT_2          = 53 + NPCMX50_IRQ_GIC_START,    /* USB Device 2                                       */
	NPCMX50_USB_DEV_INTERRUPT_3          = 54 + NPCMX50_IRQ_GIC_START,    /* USB Device 3                                       */
	NPCMX50_USB_DEV_INTERRUPT_4          = 55 + NPCMX50_IRQ_GIC_START,    /* USB Device 4                                       */
	NPCMX50_USB_DEV_INTERRUPT_5          = 56 + NPCMX50_IRQ_GIC_START,    /* USB Device 5                                       */
	NPCMX50_USB_DEV_INTERRUPT_6          = 57 + NPCMX50_IRQ_GIC_START,    /* USB Device 6                                       */
	NPCMX50_USB_DEV_INTERRUPT_7          = 58 + NPCMX50_IRQ_GIC_START,    /* USB Device 7                                       */
	NPCMX50_USB_DEV_INTERRUPT_8          = 59 + NPCMX50_IRQ_GIC_START,    /* USB Device 8                                       */
	NPCMX50_USB_DEV_INTERRUPT_9          = 60 + NPCMX50_IRQ_GIC_START,    /* USB Device 9                                       */
	NPCMX50_USB_HST_INTERRUPT_0          = 61 + NPCMX50_IRQ_GIC_START,    /* USB Host 0                                         */
	NPCMX50_USB_HST_INTERRUPT_1          = 62 + NPCMX50_IRQ_GIC_START,    /* USB Host 1                                         */

	NPCMX50_SMB_INTERRUPT_0              = 64 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 0                             */
	NPCMX50_SMB_INTERRUPT_1              = 65 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 1                             */
	NPCMX50_SMB_INTERRUPT_2              = 66 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 2                             */
	NPCMX50_SMB_INTERRUPT_3              = 67 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 3                             */
	NPCMX50_SMB_INTERRUPT_4              = 68 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 4                             */
	NPCMX50_SMB_INTERRUPT_5              = 69 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 5                             */
	NPCMX50_SMB_INTERRUPT_6              = 70 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 6                             */
	NPCMX50_SMB_INTERRUPT_7              = 71 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 7                             */
	NPCMX50_SMB_INTERRUPT_8              = 72 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 8                             */
	NPCMX50_SMB_INTERRUPT_9              = 73 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 9                             */
	NPCMX50_SMB_INTERRUPT_10             = 74 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 10                            */
	NPCMX50_SMB_INTERRUPT_11             = 75 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 11                            */
	NPCMX50_SMB_INTERRUPT_12             = 76 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 12                            */
	NPCMX50_SMB_INTERRUPT_13             = 77 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 13                            */
	NPCMX50_SMB_INTERRUPT_14             = 78 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 14                            */
	NPCMX50_SMB_INTERRUPT_15             = 79 + NPCMX50_IRQ_GIC_START,    /* SMBus and I2C Module 15                            */

	NPCMX50_AES_INTERRUPT                = 80 + NPCMX50_IRQ_GIC_START,    /* AES Module                                         */
	NPCMX50_DES_INTERRUPT                = 81 + NPCMX50_IRQ_GIC_START,    /* 3DES Module                                        */
	NPCMX50_SHA_INTERRUPT                = 82 + NPCMX50_IRQ_GIC_START,    /* SHA module                                         */
    NPCMX50_PKA_INTERRUPT                = 83 + NPCMX50_IRQ_GIC_START ,   /* (SECACC) ECC and RSA accelerator module            */

	NPCMX50_SPI_INTERRUPT_0              = 85 + NPCMX50_IRQ_GIC_START,    /* FIU module 0 if required                           */
	NPCMX50_SPI_INTERRUPT_X              = 86 + NPCMX50_IRQ_GIC_START,    /* FIU module X if required                           */
	NPCMX50_SPI_INTERRUPT_3              = 87 + NPCMX50_IRQ_GIC_START,    /* FIU module 3 if required                           */
	NPCMX50_GDMA_INTERRUPT_0             = 88 + NPCMX50_IRQ_GIC_START,    /* GDMA Module 0                                      */
	NPCMX50_GDMA_INTERRUPT_1             = 89 + NPCMX50_IRQ_GIC_START,    /* GDMA Module 1                                      */
	NPCMX50_GDMA_INTERRUPT_2             = 90 + NPCMX50_IRQ_GIC_START,    /* GDMA Module 2                                      */
	NPCMX50_GDMA_INTERRUPT_3             = 91 + NPCMX50_IRQ_GIC_START,    /* GDMA Module 3 If required                          */
	NPCMX50_OTP_INTERRUPT                = 92 + NPCMX50_IRQ_GIC_START,    /* Fustraps and Key arrays                            */
	NPCMX50_PWM_INTERRUPT_0_PWM0_3       = 93 + NPCMX50_IRQ_GIC_START,    /* PWM Module 0 outputing PWM0-3                      */
	NPCMX50_PWM_INTERRUPT_1_PWM4_7       = 94 + NPCMX50_IRQ_GIC_START,    /* PWM Module 1 outputing PWM4-7                      */

	NPCMX50_MFT_INTERRUPT_0              = 96 + NPCMX50_IRQ_GIC_START,    /* MFT Module 0                                       */
	NPCMX50_MFT_INTERRUPT_1              = 97 + NPCMX50_IRQ_GIC_START,    /* MFT Module 1                                       */
	NPCMX50_MFT_INTERRUPT_2              = 98 + NPCMX50_IRQ_GIC_START,    /* MFT Module 2                                       */
	NPCMX50_MFT_INTERRUPT_3              = 99 + NPCMX50_IRQ_GIC_START,    /* MFT Module 3                                       */
	NPCMX50_MFT_INTERRUPT_4              = 100 + NPCMX50_IRQ_GIC_START,   /* MFT Module 4                                       */
	NPCMX50_MFT_INTERRUPT_5              = 101 + NPCMX50_IRQ_GIC_START,   /* MFT Module 5                                       */
	NPCMX50_MFT_INTERRUPT_6              = 102 + NPCMX50_IRQ_GIC_START,   /* MFT Module 6                                       */
	NPCMX50_MFT_INTERRUPT_7              = 103 + NPCMX50_IRQ_GIC_START,   /* MFT Module 7                                       */
	NPCMX50_PWM_INTERRUPT_0              = 104 + NPCMX50_IRQ_GIC_START,   /* PWM module 0                                       */
	NPCMX50_PWM_INTERRUPT_1              = 105 + NPCMX50_IRQ_GIC_START,   /* PWM module 1                                       */
	NPCMX50_PWM_INTERRUPT_2              = 106 + NPCMX50_IRQ_GIC_START,   /* PWM module 2                                       */
	NPCMX50_PWM_INTERRUPT_3              = 107 + NPCMX50_IRQ_GIC_START,   /* PWM module 3                                       */
	NPCMX50_PWM_INTERRUPT_4              = 108 + NPCMX50_IRQ_GIC_START,   /* PWM module 4                                       */
	NPCMX50_PWM_INTERRUPT_5              = 109 + NPCMX50_IRQ_GIC_START,   /* PWM module 5                                       */
	NPCMX50_PWM_INTERRUPT_6              = 110 + NPCMX50_IRQ_GIC_START,   /* PWM module 6                                       */
	NPCMX50_PWM_INTERRUPT_7              = 111 + NPCMX50_IRQ_GIC_START,   /* PWM module 7                                       */

	NPCMX50_EMC2RX_INTERRUPT             = 114 + NPCMX50_IRQ_GIC_START,   /* EMC2 Rx module                                     */
	NPCMX50_EMC2TX_INTERRUPT             = 115 + NPCMX50_IRQ_GIC_START,   /* EMC2 Tx module                                     */
	NPCMX50_GPIO_INTERRUPT0              = 116 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIO0-31                     */
	NPCMX50_GPIO_INTERRUPT1              = 117 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIONPCMX50_IRQ_GIC_START-63                    */
	NPCMX50_GPIO_INTERRUPT2              = 118 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIO64-95                    */
	NPCMX50_GPIO_INTERRUPT3              = 119 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIO96-127                   */
	NPCMX50_GPIO_INTERRUPT4              = 120 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIO128-159                  */
	NPCMX50_GPIO_INTERRUPT5              = 121 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIO160-191                  */
	NPCMX50_GPIO_INTERRUPT6              = 122 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIO192-223                  */
	NPCMX50_GPIO_INTERRUPT7              = 123 + NPCMX50_IRQ_GIC_START,   /* GPIO module outputing GPIO224-255                  */

	NPCMX50_PCIE_RC_INTERRUPT            = 127 + NPCMX50_IRQ_GIC_START,   /* PCIe Root Complex  (combined A-D and MSI)          */
	
	// NPCMX50_GIC_INTERRUPT_NUM
} npcm750_irq_list_t;

#define NPCMX50_KCS_HIB_INT KCS_HIB_INTERRUPT
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


/*---------------------------------------------------------------------------------------------------------*/
/* The non-cached area is not HW, we spare 32MB for linux to use it                                        */
/* Map 32MB at the last 48MB of 2GB (since GFX takes 16MB at the end) for non-cached area,                 */
/* since DDR is wrapping around this should work even if actual DDR is smaller                             */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCMX50_NON_CACHED_PHYS_BASE_ADDR       0x7D000000 // _2GB_ - _16MB_ - _32MB_ = 0x7D000000


#ifndef DYNAMIC_BASE_ADDRESS
#define NPCMX50_NON_CACHED_BASE_ADDR(module)    NON_CACHED_PHYS_BASE_ADDR(module)
#else
#define NPCMX50_NON_CACHED_BASE_ADDR(module)    NON_CACHED_VIRT_BASE_ADDR(module)
#endif

/* Defining IO Memory continuesly mapped blocks                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#if 0
                                    /* Map:Phisical Addr | Virtual Addr | Block Size     */
                                    /* -----------------------------------------------    */
#define NPCMX50_IOMEMORY_BLOCKS             {                                                                \
                                        {   NPCMX50_VDM_PHYS_BASE_ADDR, IOMEMORY(NPCMX50_VDM_PHYS_BASE_ADDR), 0x00001000   },        \
                                        {   NPCMX50_AHB1_BASE_ADDR, IOMEMORY(NPCMX50_AHB1_BASE_ADDR), 0x00020000   },        \
                                        {   NPCMX50_APB2_BASE_ADDR, IOMEMORY(NPCMX50_APB2_BASE_ADDR), 0x00020000   },        \
                                        {   NPCMX50_APB3_BASE_ADDR, IOMEMORY(NPCMX50_APB3_BASE_ADDR), 0x00010000   },        \
                                        {   NPCMX50_APB4_BASE_ADDR, IOMEMORY(NPCMX50_APB4_BASE_ADDR), 0x00010000   },        \
                                        {   NPCMX50_APB5_BASE_ADDR, IOMEMORY(NPCMX50_APB5_BASE_ADDR), 0x00005000   },        \
                                        {   NPCMX50_A9_BASE_ADDR,   IOMEMORY(NPCMX50_A9_BASE_ADDR),   0x00004000   },        \
                                        {   NPCMX50_DAP_BASE_ADDR,  IOMEMORY(NPCMX50_DAP_BASE_ADDR),  0x00100000   },        \
                                        {   NPCMX50_AHB8_BASE_ADDR, IOMEMORY(NPCMX50_AHB8_BASE_ADDR), 0x00060000   },        \
                                        {   NPCMX50_FIU0_BASE_ADDR, IOMEMORY(NPCMX50_FIU0_BASE_ADDR), 0x00002000   },        \
                                        {   NPCMX50_FIU3_BASE_ADDR, (NPCMX50_FIU0_BASE_ADDR + 0x3000),  0x00001000   },        \
										{	NPCMX50_RAM2_BASE_ADDR, IOMEMORY(NPCMX50_RAM2_BASE_ADDR), 0x00020000   },		 \
										{	NPCMX50_ROM_BASE_ADDR,  IOMEMORY(NPCMX50_ROM_BASE_ADDR),  0x00010000   },		 \
                                    }
#endif
                                    /* Map:Phisical Addr | Virtual Addr | Block Size     */
                                    /* -----------------------------------------------    */
#define NPCMX50_IOMEMORY_BLOCKS             {                                                                \
                                        {   NPCMX50_AHB1_BASE_ADDR, IOMEMORY(NPCMX50_AHB1_BASE_ADDR), 0x00018000   },        \
                                        {   NPCMX50_A9_BASE_ADDR,   IOMEMORY(NPCMX50_A9_BASE_ADDR),   0x00004000   },        \
                                        {   NPCMX50_DAP_BASE_ADDR,  IOMEMORY(NPCMX50_DAP_BASE_ADDR),  0x00100000   },        \
                                        {   NPCMX50_AHB8_BASE_ADDR, IOMEMORY(NPCMX50_AHB8_BASE_ADDR), 0x00002000   },        \
										{	NPCMX50_RAM2_BASE_ADDR, IOMEMORY(NPCMX50_RAM2_BASE_ADDR), 0x00020000   },		 \
										{	NPCMX50_ROM_BASE_ADDR,  IOMEMORY(NPCMX50_ROM_BASE_ADDR),  0x00010000   },		 \
                                    }


#endif //_CHIP_NPCM750_H_

