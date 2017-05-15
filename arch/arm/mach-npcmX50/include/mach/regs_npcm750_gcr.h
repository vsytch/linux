/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2008 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   gcr_regs.h                                                                                            */
/*            This file contains definitions of global control registers for Yarkon                        */
/*  Project:                                                                                               */
/*            ROM code                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef _NPCMX50_GCR_REGS_H
#define _NPCMX50_GCR_REGS_H



/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                      Chip Configuration Registers                                       */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/**************************************************************************************************************************/
/*   Product Identifier Register (PDID) updated for Poleg                                                                 */
/**************************************************************************************************************************/
#define  PDID                           (GCR_VIRT_BASE_ADDR + 0x000) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+000h */
#define  PDID_VER                        24 , 8             /* 31-24 Version (Tapeout Version).                                                                                      */
#define  PDID_CHRID                      2 , 14             /* 15-2 CHRID (Chip ID). The Chip identifier is A9_2750h.                                                                */


/**************************************************************************************************************************/
/*   Power-On Setting Register (PWRON) Updated for Poleg                                                                  */
/**************************************************************************************************************************/
#define  PWRON                          (GCR_VIRT_BASE_ADDR + 0x004) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+004h */
#define  PWRON_SB1                       22 , 1              /* 22 SB1. Security bypass. (Unchanged for Poleg)                                                                        */
#define  PWRON_STRAP13                   12 , 1              /* 12 STRAP13. SPI0 bus powered by VSBV3 at 1.8V (SPI0F18). (added for Poleg)                                            */
#define  PWRON_STRAP12                   11 , 1              /* 11 STRAP12. System Flash Attached to BMC (SFAB). (added for Poleg)                                                    */
#define  PWRON_STRAP11                   10 , 1              /* 10 STRAP11. BSP Alternate pins (BSPA). (added for Poleg)                                                              */
#define  PWRON_STRAP10_9                 8 , 2              /* 9-8 STRAP10-9. Flash UART Command routine enable (FUP). (Changed for Poleg)                                           */
#define  PWRON_STRAP8                    7 , 1               /* 7 STRAP8. Security Enable (SECEN). Moved in Poleg.                                                                    */
#define  PWRON_STRAP7                    6 , 1               /* 6 STRAP7. HI-Z state control (HI-Z). (Unchanged for Poleg                                                             */
#define  PWRON_STRAP6                    5 , 1               /* 5 STRAP6. ECC Enable (ECC). (Moved for Poleg)                                                                         */
#define  PWRON_STRAP5                    4 , 1               /* 4 STRAP5. Co-Proccesor skip init                                                                                      */
#define  PWRON_STRAP4                    3 , 1               /* 3 STRAP4. JTAG2 Enable (J2EN). (New in Poleg)                                                                         */
#define  PWRON_STRAP3_1                  0 , 3              /* 2-0 STRAP3-1. CPU core, system and DDR4 memory frequency (CKFRQ). (Changed for Poleg)                                 */


/**************************************************************************************************************************/
/*   Arbitration Control Register (ARBCON) - Removed in                                                                   */
/**************************************************************************************************************************/
#define  ARBCON                         (GCR_VIRT_BASE_ADDR + 0x008) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 008h */

/**************************************************************************************************************************/
/*   Multiple Function Pin Select Register 1 (MFSEL1) Updated for                                                         */
/**************************************************************************************************************************/
#define  MFSEL1                         (GCR_VIRT_BASE_ADDR + 0x00C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 00Ch */
#define  MFSEL1_SIRQSEL                  31 , 1              /* 31 SIRQSEL (Serial IRQ Select). Selects GPIO or SERIRQ.                                                               */
#define  MFSEL1_IOX1SEL                  30 , 1              /* 30 IOX1SEL (Serial GPIO Expander 1 Select). Selects I/O Expander 1 interface option.                                  */
#define  MFSEL1_DVH1SEL                  27 , 1              /* 27 DVH1SEL (Digital Video Head 1 Select). Selects VCD digital video input source when internal.                       */
#define  MFSEL1_LPCSEL                   26 , 1              /* 26 LPCSEL (LPC Select). Selects GPIOs or LPC signals.                                                                 */
#define  MFSEL1_PECIB                    25 , 1              /* 25 PECIB (PECI Bypass). Enables PECI PHY bypass on pins GPIOE11-10. When this bit is 1, MFSEL3.IOXHSEL                */
#define  MFSEL1_GSPISEL                  24 , 1              /* 24 GSPISEL (Graphics SPI Select). Selects Graphics Core SPI Signals or GPIO option.                                   */
#define  MFSEL1_SMISEL                   22 , 1              /* 22 SMISEL (SMI Select). Selects nSMI or GPIO170 option.                                                               */
#define  MFSEL1_CLKOSEL                  21 , 1              /* 21 CLKOSEL (Clockout Select). Selects CLKOUT or GPIIO160 option.                                                      */
#define  MFSEL1_DVOSEL                   18 , 3             /* 20-18 DVOSEL (DVO Select). Selects DVO output/input signals or GPIO option:                                           */
#define  MFSEL1_KBCICSEL                 17 , 1              /* 17 KBCICSEL (KBC Interface Controller Select). Selects Keyboard Controller Interface Control Signals or GPIO          */
#define  MFSEL1_R2MDSEL                  16 , 1              /* 16 R2MDSEL (RMII2 MDIO Select). Selects RMII2 MDIO or GPIO option.                                                    */
#define  MFSEL1_R2ERRSEL                 15 , 1              /* 15 R2ERRSEL (RMII2 R2RXERR Select). Selects RMII2 R2RXERR or GPIO90 option.                                           */
#define  MFSEL1_RMII2SEL                 14 , 1              /* 14 RMII2SEL (RMII2 Select). Selects RMII2 (GMAC2 module) or GPIO option.                                              */
#define  MFSEL1_R1MDSEL                  13 , 1              /* 13 R1MDSEL (RMII1 MDIO Select). Selects RMII1 MDIO or GPIO option.                                                    */
#define  MFSEL1_R1ERRSEL                 12 , 1              /* 12 R1ERRSEL (RMII1 R1RXERR Select). Selects RMII1 R1RXERR or GPIO56 option.                                           */
#define  MFSEL1_HSI2SEL                  11 , 1              /* 11 HSI2SEL (Host Serial Interface 2 Select). Selects Host Serial Interface 2 or GPIO option.                          */
#define  MFSEL1_HSI1SEL                  10 , 1              /* 10 HSI1SEL (Host Serial Interface 1 Select). Selects Host Serial Interface 1 or GPIO option.                          */
#define  MFSEL1_BSPSEL                   9 , 1               /* 9 BSPSEL (BMC Serial Port Select). Selects Core Serial Port 0 or GPIO option.                                         */
#define  MFSEL1_SMB2SEL                  8 , 1               /* 8 SMB2SEL (SMB2 Select). Selects SMB2 or GPIO option.                                                                 */
#define  MFSEL1_SMB1SEL                  7 , 1               /* 7 SMB1SEL (SMB1 Select). Selects SMB1 or GPIO option.                                                                 */
#define  MFSEL1_SMB0SEL                  6 , 1               /* 6 SMB0SEL (SMB0 Select). Selects SMB0 or GPIO option.                                                                 */
#define  MFSEL1_S0CS3SEL                 5 , 1               /* 5 S0CS3SEL (SPI0CS3 Select). Selects nSPI0CS3/SPI0D3 or GPIO34 option. When this bit is set, SP0QSEL bit              */
#define  MFSEL1_S0CS2SEL                 4 , 1               /* 4 S0CS2SEL (SPI0CS2 Select). Selects nSPI0CS2/SPI0D2 or GPIO33 option. When this bit is set, SP0QSEL bit              */
#define  MFSEL1_S0CS1SEL                 3 , 1               /* 3 S0CS1SEL (SPI0CS1 Select). Selects nSPI0CS1 or GPIO32 option.                                                       */
#define  MFSEL1_SMB5SEL                  2 , 1               /* 2 SMB5SEL (SMBus 5 Select). Selects SMB5 or GPIO option.                                                              */
#define  MFSEL1_SMB4SEL                  1 , 1               /* 1 SMB4SEL (SMBus 4 Select). Selects SMB4 or GPIO option.                                                              */
#define  MFSEL1_SMB3SEL                  0 , 1               /* 0 SMB3SEL (SMBus 3 Select). Selects SMB3 or GPIO option.                                                              */

enum MFSEL1_DVOSEL_T
{
    MFSEL1_DVOSEL_OUTPUT_HEAD1  = 5,
    MFSEL1_DVOSEL_OUTPUT_HEAD2  = 1,
    MFSEL1_DVOSEL_INPUT_KVM     = 3,
};

/**************************************************************************************************************************/
/*   Multiple Function Pin Select Register 2 (MFSEL2) updated for                                                         */
/**************************************************************************************************************************/
#define  MFSEL2                         (GCR_VIRT_BASE_ADDR + 0x010) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 010h */
#define  MFSEL2_HG7SEL                   31 , 1              /* 31 HG7SEL. Selects HGPIO7 or GPIO60 option.                                                                           */
#define  MFSEL2_HG6SEL                   30 , 1              /* 30 HG6SEL. Selects HGPIO6 or GPIO59 option.                                                                           */
#define  MFSEL2_HG5SEL                   29 , 1              /* 29 HG5SEL. Selects HGPIO5 or GPIO25 option. When this bit is 1, MFSEL3.IOXHSEL should be 0.                           */
#define  MFSEL2_HG4SEL                   28 , 1              /* 28 HG4SEL. Selects HGPIO4 or GPIO24 option. When this bit is 1, MFSEL3.IOXHSEL should be 0.                           */
#define  MFSEL2_HG3SEL                   27 , 1              /* 27 HG3SEL. Selects HGPIO3 or GPIO23 option.                                                                           */
#define  MFSEL2_HG2SEL                   26 , 1              /* 26 HG2SEL. Selects HGPIO2 or GPIO22 option.                                                                           */
#define  MFSEL2_HG1SEL                   25 , 1              /* 25 HG1SEL. Selects HGPIO1 or GPIO21 option.                                                                           */
#define  MFSEL2_HG0SEL                   24 , 1              /* 24 HG0SEL (Host GPIO0 Select). Selects HGPIO0 or GPIO20 option.When this bit is set, the following bits should        */
#define  MFSEL2_PWM7SEL                  23 , 1              /* 23 PWM7SEL (PWM7 Select). Selects PWM7 or GPIO147 option.                                                             */
#define  MFSEL2_PWM6SEL                  22 , 1              /* 22 PWM6SEL (PWM6 Select). Selects PWM6 or GPIO146 option.                                                             */
#define  MFSEL2_PWM5SEL                  21 , 1              /* 21 PWM5SEL (PWM5 Select). Selects PWM5 or GPIO145 option.                                                             */
#define  MFSEL2_PWM4SEL                  20 , 1              /* 20 PWM4SEL (PWM4 Select). Selects PWM4 or GPIO144 option.                                                             */
#define  MFSEL2_PWM3SEL                  19 , 1              /* 19 PWM3SEL (PWM3 Select). Selects PWM3 or GPIO83 option.                                                              */
#define  MFSEL2_PWM2SEL                  18 , 1              /* 18 PWM2SEL (PWM2 Select). Selects PWM2 or GPIO82 option.                                                              */
#define  MFSEL2_PWM1SEL                  17 , 1              /* 17 PWM1SEL (PWM1 Select). Selects PWM1 or GPIO81 option.                                                              */
#define  MFSEL2_PWM0SEL                  16 , 1              /* 16 PWM0SEL (PWM0 Select). Selects PWM0 or GPIO80 option.                                                              */
#define  MFSEL2_FI15SEL                  15 , 1              /* 15 FI15SEL (FANIN15 Select). Selects FANIN15 or GPIO79 option.                                                        */
#define  MFSEL2_FI14SEL                  14 , 1              /* 14 FI14SEL (FANIN14 Select). Selects FANIN14 or GPIO78 option.                                                        */
#define  MFSEL2_FI13SEL                  13 , 1              /* 13 FI13SEL (FANIN13 Select). Selects FANIN13 or GPIO77 option.                                                        */
#define  MFSEL2_FI12SEL                  12 , 1              /* 12 FI12SEL (FANIN12 Select). Selects FANIN12 or GPIO76 option.                                                        */
#define  MFSEL2_FI11SEL                  11 , 1              /* 11 FI11SEL (FANIN11 Select). Selects FANIN11 or GPIO75 option.                                                        */
#define  MFSEL2_FI10SEL                  10 , 1              /* 10 FI10SEL (FANIN10 Select). Selects FANIN10 or GPIO74 option.                                                        */
#define  MFSEL2_FI9SEL                   9 , 1               /* 9 FI9SEL (FANIN9 Select). Selects FANIN9 or GPIO73 option.                                                            */
#define  MFSEL2_FI8SEL                   8 , 1               /* 8 FI8SEL (FANIN8 Select). Selects FANIN8 or GPIO72 option.                                                            */
#define  MFSEL2_FI7SEL                   7 , 1               /* 7 FI7SEL (FANIN7 Select). Selects FANIN7 or GPIO71 option.                                                            */
#define  MFSEL2_FI6SEL                   6 , 1               /* 6 FI6SEL (FANIN6 Select). Selects FANIN6 or GPIO70 option.                                                            */
#define  MFSEL2_FI5SEL                   5 , 1               /* 5 FI5SEL (FANIN5 Select). Selects FANIN5 or GPIO69 option.                                                            */
#define  MFSEL2_FI4SEL                   4 , 1               /* 4 FI4SEL (FANIN4 Select). Selects FANIN4 or GPIO68 option.                                                            */
#define  MFSEL2_FI3SEL                   3 , 1               /* 3 FI3SEL (FANIN3 Select). Selects FANIN3 or GPIO67 option.                                                            */
#define  MFSEL2_FI2SEL                   2 , 1               /* 2 FI2SEL (FANIN2 Select). Selects FANIN2 or GPIO66 option.                                                            */
#define  MFSEL2_FI1SEL                   1 , 1               /* 1 FI1SEL (FANIN1 Select). Selects FANIN1 or GPIO65 option.                                                            */
#define  MFSEL2_FI0SEL                   0 , 1               /* 0 FI0SEL (FANIN0 Select). Selects FANIN0 or GPIO64 option.                                                            */

/**************************************************************************************************************************/
/*   Miscellaneous Pin Pull-Up/Down Enable Register (MISCPE) (Updated for )                                               */
/**************************************************************************************************************************/
#define  MISCPE                         (GCR_VIRT_BASE_ADDR + 0x014) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 014h */
#define  MISCPE_PWRGD_PS                 0 , 1               /* 0 PWRGD_PS. PPE (Pin Pull-Up/Down Enable) for PWRGD_PS. This control bit controls the pull up/down                    */

/**************************************************************************************************************************/
/*   Serial Ports Switch Control Register (SPSWC) Updated for                                                             */
/**************************************************************************************************************************/
#define  SPSWC                          (GCR_VIRT_BASE_ADDR + 0x038) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 038h */
#define  SPSWC_UEM3M                     12 , 1              /* 12 UEM3M (UART Easy Setting Mode 3 Modifier). Modifies UES1 and UES2 settings when any of them                        */
#define  SPSWC_UES2                      10 , 2             /* 11-10 UES2 (UART Easy Setting 2). Controls the way the BMC UART2 and Host Serial Port 2 are                           */
#define  SPSWC_UES1                      8 , 2              /* 9-8 UES1 (UART Easy Setting 1). Controls the way the BMC UART1 and Host Serial Port 1 are                             */
#define  SPSWC_RTSS                      7 , 1               /* 7 RTSS (RTS Setting). Controls the value for the nRTS output of an otherwise not driven Serial Interface              */
#define  SPSWC_DTRS                      6 , 1               /* 6 DTRS (DTR Setting). Controls the value for the nDTR output of an otherwise not driven Serial Interface              */
#define  SPSWC_DCDI                      5 , 1               /* 5 DCDI (Serial Interface 2 DCD input). nDCD2 input status. This bit is read-only. Enables monitoring the              */
#define  SPSWC_SPMOD                     0 , 3              /* 2-0 SPMOD (Serial Port Mode). Configures the Serial Port connectivity scheme, as defined in “Serial                   */


/*---------------------------------------------------------------------------------------------------------*/
/* SPSWC_SPMOD field values definition                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
/*
2 1 0 Mode
0 0 0: Mode 1 - Core Snoop.
0 0 1: Mode 2 - Core Take-over.
0 1 0: Mode 3 - Core Direct to Host Serial Port 2; Host SP1 connected to SI1 (default).
1 1 0: Mode 4 - Core Direct to Host Serial Port 2; Host SP1 connected to SI2.
Else: Reserved.
Note: mode 4 should be used only if SP2 is selected (MFSEL1.HSP2SEL = 1).
*/
#define SPSWC_SPMOD_CORESNOOP         0
#define SPSWC_SPMOD_CORETAKEOVER      1
#define SPSWC_SPMOD_SP1TOSI1          2
#define SPSWC_SPMOD_SP1TOSI2          3


/**************************************************************************************************************************/
/*   Integration Control Register (INTCR) (Unchanged for )                                                                */
/**************************************************************************************************************************/
#define  INTCR                          (GCR_VIRT_BASE_ADDR + 0x03C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 03Ch */
#define  INTCR_DUDKSMOD                  30 , 2             /* 31-30 DUDKSMOD (Display update during KVM-scan Mode).                                                                 */
#define  INTCR_DDC3I                     29 , 1              /* 29 DDC3I (Spare Graphics Control). Spare control for graphics (TBD if used).                                          */
#define  INTCR_KVMSI                     28 , 1              /* 28 KVMSI (KVM Session Indication) DDC2I. Used to indicate to the windows graphics driver that a KVM session           */
#define  INTCR_DEHS                      27 , 1              /* 27 DEHS (DE or HSYNC from Graphics Core). Selects (head 2 and head 1 - added in Yarkon) the DE signal or              */
#define  INTCR_GGPCT2_0                  24 , 3             /* 26-24 GGPCT2-0 (Graphics General Purpose Control). Controls graphics, TBD--future definition.                         */
#define  INTCR_SGC2                      23 , 1              /* 23 SGC2 (Spare Graphics Control 2). Spare control for graphics (TBD if used).                                         */
#define  INTCR_DSNS_TRIG                 21 , 1              /* 21 DSNS_TRIG (DAC Sense Trigger). (Changed from Hermon.) Triggers the DAC sense mechanism for inputs of               */
#define  INTCR_DAC_SNS                   20 , 1              /* 20 DAC_SNS (DAC Sense). (Changed from Hermon.) Controls the DAC sense mechanism for inputs of the                     */
#define  INTCR_SGC1                      19 , 1              /* 19 SGC1 (Spare Graphics Control 1). Spare control for graphics (TBD if used).                                         */
#define  INTCR_LDDRB                     18 , 1              /* 18 LDDRB (Local Display Disable with Reduced Bandwidth).                                                              */
#define  INTCR_GIRST                     17 , 1              /* 17 GIRST (Graphics Interface Head 2 Reset). Resets the graphics second head level 0.                                  */
#define  INTCR_DUDKSEN                   16 , 1              /* 16 DUDKSEN (Display Update During KVM-scan Enable). Controls display updating during KVM scan.                        */
#define  INTCR_DACOFF                    15 , 1              /* 15 DACOFF (Turn Off Graphics DAC). Controls the blanking of the DACs by the BMC, if the BMC wants to turn             */
#define  INTCR_DACSEL                    14 , 1              /* 14 DACSEL (DAC Select). Selects which DAC will have the display instead of DACOSEL pin, if override enable            */
#define  INTCR_GFXINT                    12 , 1              /* 12 GFXINT (Graphics Interrupt). Generates an interrupt to the graphics driver so it will retry reading the EDID       */
#define  INTCR_DACOSOVR                  10 , 2             /* 11-10 DACOSOVR (DAC Output Select Override).                                                                          */
#define  INTCR_GFXIFDIS                  8 , 2              /* 9-8 GFXIFDIS (Graphics Interface Disable). Disables graphics interface to KVM.                                        */
#define  INTCR_H2RQDIS                   9 , 1               /* 9 H2RQDIS (Head 2 Request Disable). Disables head2 (requests) access to memory (read refresh for KVM                  */
#define  INTCR_H2DISPOFF                 8 , 1               /* 8 H2DISPOFF (Head 2 Display Off). Disables display from head2. Syncs and data are stopped at 0.                       */
#define  INTCR_GFXINT2                   7 , 1               /* 7 GFXINT2 (Graphics Interrupt 2). May generate an interrupt to the graphics driver.                                   */
#define  INTCR_VGAIOEN                   6 , 1               /* 6 VGAIOEN (Enable VGA Core to Decode I/O Addresses). The value of this bit is connected to VGA core                   */
#define  INTCR_R1EN                      5 , 1               /* 5 R1EN (Enable RMII Outputs). Controls the HI-Z status of RMII 1.                                                     */
#define  INTCR_PSPIFEN                   4 , 1               /* 4 PSPIFEN (PSPI Freeze Enable). Controls the freezing of the PSPI modules. When set, and nDBGACK internal             */
#define  INTCR_HIFEN                     3 , 1               /* 3 HIFEN (Host Interface Freeze Enable). Controls the freezing of the Host Interface modules (Keyboard and             */
#define  INTCR_SMBFEN                    2 , 1               /* 2 SMBFEN (SMB Freeze Enable). Controls the freezing of the SMB modules. When set, and nDBGACK internal                */
#define  INTCR_MFTFEN                    1 , 1               /* 1 MFTFEN (MFT Freeze Enable). Controls the freezing of the MFT modules. When set, and nDBGACK internal                */
#define  INTCR_KCSRST_MODE               0 , 1               /* 0 KCSRST_MODE (Host Interface Modules Reset Mode Select). Selects the reset source for both the host and              */

/**************************************************************************************************************************/
/*   Integration Status Register (INTSR) (Unchanged in )                                                                  */
/**************************************************************************************************************************/
#define  INTSR                          (GCR_VIRT_BASE_ADDR + 0x040) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 040h */
#define  INTSR_USBHOSTST                 16 , 14            /* 29-16 USBHOSTST (USB Host Status). Indication or controls From USB Host:                                              */
#define  INTSR_SDHC2PC                   14 , 1              /* 14 SDHC2PC (SDHC2 Power Control). Indication for control MMC power supply.                                            */
#define  INTSR_SDHC1PC                   13 , 1              /* 13 SDHC1PC (SDHC1 Power Control). Indication for control SD power supply.                                             */
#define  INTSR_BLUS                      12 , 1              /* 12 BLUS (Blue Connection Sense).                                                                                      */
#define  INTSR_GRNS                      11 , 1              /* 11 GRNS (Green Connection Sense).                                                                                     */
#define  INTSR_REDS                      10 , 1              /* 10 REDS (Red Connection Sense).                                                                                       */
#define  INTSR_MGAMODE                   9 , 1               /* 9 MGAMODE (MGA or VGA Status).                                                                                        */
#define  INTSR_VGAWRS                    8 , 1               /* 8 VGAWRS (VGA Attribute Write Status).                                                                                */
#define  INTSR_DOSIN                     7 , 1               /* 7 DOSIN (DACOSEL Input State).                                                                                        */
#define  INTSR_DDC3O                     6 , 1               /* 6 DDC3O (Spare Graphics Status). Spare status from Graphics (TBD if used).                                            */
#define  INTSR_DDC2O                     5 , 1               /* 5 DDC2O (Spare Graphics Status). Spare status from Graphics (TBD if used).                                            */
#define  INTSR_DWGENGST                  4 , 1               /* 4 DWGENGST (Drawing Engine Stall). Indicates drawing engine stall status.                                             */
#define  INTSR_DWGENGBUSY                3 , 1               /* 3 DWGENGBUSY (Drawing Engine Busy). Indicates drawing engine busy status.                                             */
#define  INTSR_GGPST2_0                  0 , 3              /* 2-0 GGPST2-0 (Graphics General Purpose Status). Graphics status, future definition.                                   */

/**************************************************************************************************************************/
/*   Observability Control Register 1-2 (OBSCR1-2) (Unchanged in )                                                        */
/**************************************************************************************************************************/
#define  OBSCR1                         (GCR_VIRT_BASE_ADDR + 0x044) , NPCMX50_GCR_ACCESS, 32 		/* Offset: OBSCR1: GCR_BA + 044h */
#define  OBSCR2                         (GCR_VIRT_BASE_ADDR + 0x0C4) , NPCMX50_GCR_ACCESS, 32 		/* OBSCR2: GCR_BA + 0C4h  */
#define  OBSCR2_OBSMSEL                  11 , 5             /* 15-11 OBSMSEL (Observability Module Select).                                                                          */
#define  OBSCR2_OBSDSEL                  7 , 4              /* 10-7 OBSDSEL (Observability Data Select).                                                                             */
#define  OBSCR2_MUXC                     5 , 2              /* 6-5 MUXC (Mux Control). Selects where to output the observability output signals:                                     */
#define  OBSCR2_ULSEL                    3 , 2              /* 4-3 ULSEL (Observability Upper/Lower Data Select). Selects which of the 8/16-bit observability signal groups is       */
#define  OBSCR2_OBSBW                    1 , 2              /* 2-1 OBSBW (Observability Bus Width). This field exists only in OBSCR1.                                                */
#define  OBSCR2_Width                    10 , 1              /* 10 Width                                                                                                              */
#define  OBSCR2_OBSEN                    0 , 1               /* 0 OBSEN (Observe Enable).                                                                                             */

/**************************************************************************************************************************/
/*   Observability Data Register 1-2 (OBSDR1-2) (Unchanged in )                                                           */
/**************************************************************************************************************************/
#define  OBSD1                          (GCR_VIRT_BASE_ADDR + 0x048) , NPCMX50_GCR_ACCESS, 32 		/* Offset: OBSD1: GCR_BA + 048h */
#define  OBSD2                          (GCR_VIRT_BASE_ADDR + 0x0C8) , NPCMX50_GCR_ACCESS, 32 		/* OBSD2: GCR_BA + 0C8h  */
#define  OBSD2_OBSDATA                   0 , 16             /* 15-0 OBSDATA (Observability Data).                                                                                    */

/**************************************************************************************************************************/
/*   NPCM750Host Interface Control Register (HIFCR) (Updated for )                                                        */
/**************************************************************************************************************************/
#define  HIFCR                          (GCR_VIRT_BASE_ADDR + 0x050) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 050h */
#define  HIFCR_LPC_ADDR                0 , 16             /* 15-0 LPC_ADDR (LPC Address). Defines LPC address (when BADDR strap selects BMC CORE selectable address                */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 1 (SDnIRV1) (reg40_dt) Updated for                                              */
/**************************************************************************************************************************/
#define  SD1IRV1                        (GCR_VIRT_BASE_ADDR + 0x54) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 54h */
#define  SD2IRV1                        (GCR_VIRT_BASE_ADDR + 0xB4) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + B4h  */
#define  SDnIRV1_corecfg_8bitsupport     27 , 1             /* 27 (8-Bit Support). Eight-bit support for embedded devices. The default is 1 (Core supports 8bit MMC interface)       */
#define  SDnIRV1_corecfg_maxblklength    25 , 2             /* 26-25 corecfg_maxblklength (Maximum Block Length). Maximum block length is supported by the Core.                     */
#define  SDnIRV1_corecfg_timeoutclkunit  24 , 1              /* 24 corecfg_timeoutclkunit (Timeout Clock Unit). The timeout clock unit can be either MHz or KHz. The default          */
#define  SDnIRV1_corecfg_timeoutclkfreq  19 , 5             /* 23-19 corecfg_timeoutclkfreq (Timeout Clock Frequency). The default is 25 MHz (see corecfg_timeoutclkunit             */
#define  SDnIRV1_corecfg_tuningcount     13 , 6             /* 18-13 corecfg_tuningcount (Tuning Count). Selects the looped clock phase (tap). Assumes tuning is used (instead       */
#define  SDnIRV1_test_mode               12 , 1              /* 12 test_mode (Test Mode). Enable Test mode. The Test Mode signal is used for DFT purposes. It muxes in the            */
#define  SDnIRV1_corectrl_otapdlysel     8 , 4              /* 11-8 corectrl_otapdlysel (Output Tap Delay). The delay of the output clock to the SD (rxclk_out), for SD sampling.    */
#define  SDnIRV1_corectrl_otapdlyena     7 , 1               /* 7 corectrl_otapdlyena (Output Tap Delay Enable). Not Supported.                                                       */
#define  SDnIRV1_corectrl_itapchgwin     6 , 1               /* 6 corectrl_itapchgwin. Not Supported.                                                                                 */
#define  SDnIRV1_corectrl_itapdlysel     1 , 5              /* 5-1 corectrl_itapdlysel (Input Tap Delay). The delay of the looped clock from SD (rxclk_in). It must be configured    */
#define  SDnIRV1_corectrl_itapdlyena     0 , 1               /* 0 corectrl_itapdlyena (Input Tap Delay Enable). Used to enable selective Tap delay line on the loop-backed SD         */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 2 (SDnIRV2) (reg44_dt) Updated for                                              */
/**************************************************************************************************************************/
#define  SD1IRV2                        (GCR_VIRT_BASE_ADDR + 0x58) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 58h */
#define  SD2IRV2                        (GCR_VIRT_BASE_ADDR + 0xB8) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + B8h  */
#define  SDnIRV2_corecfg_asyncwkupena    30 , 1              /* 30 corecfg_asyncwkupena (Asynchronous Wake-Up). Determines the wake-up signal generation mode.                        */
#define  SDnIRV2_corecfg_retuningmodes   26 , 2             /* 27-26 corecfg_retuningmodes (Retuning Modes). Must be set to ‘00’ (Mode0 retuning).                                   */
#define  SDnIRV2_corecfg_tuningforsdr50  25 , 1              /* 25 corecfg_tuningforsdr50 (Tuning for SDR50). Set to 1 if the Application wants Tuning be used for SDR50              */
#define  SDnIRV2_corecfg_retuningtimercnt  21 , 4             /* 24-21 corecfg_retuningtimercnt (Timer Count for Retuning). This is the Timer Count for the Retuning timer for         */
#define  SDnIRV2_corecfg_sdr50support      15 , 1             /* 15 corecfg_sdr50support (SDR50 Support). Simple Data Rate: 50 MHz, 50 Mbps.                                           */
#define  SDnIRV2_corecfg_slottype        13 , 2             /* 14-13 corecfg_slottype (Slot Type). Used by the card detection. 1 for eMMC, 0 for SD.                                 */
#define  SDnIRV2_corecfg_baseclkfreq     0 , 8              /* 7-0 corecfg_baseclkfreq (Base Clock Frequency). Base clock frequency for the SD clock. This is the frequency          */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 3 (SDnIRV3) (reg48_dt) (Updated for )                                           */
/**************************************************************************************************************************/
#define  SD1IRV3                        (GCR_VIRT_BASE_ADDR + 0x5C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 5Ch */
#define  SD2IRV3                        (GCR_VIRT_BASE_ADDR + 0xBC) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + BCh  */
#define  SDnIRV3_dbg_sel                 24 , 3             /* 26-24 dbg_sel (Debug Select). Selects observability bus (16 bits). See User guide for details                         */
#define  SDnIRV3_corecfg_maxcurrent3p3v  16 , 8             /* 23-16 corecfg_maxcurrent3p3v (Maximum Current for 3.3V). Default is 5 for MMC and 0Bh for SD. This stands for         */
#define  SDnIRV3_corecfg_maxcurrent3p0v  8 , 8              /* 15-8 corecfg_maxcurrent3p0v (Maximum Current for 3.0V). No 3.0V support.                                              */
#define  SDnIRV3_corecfg_maxcurrent1p8v  0 , 8              /* 7-0 corecfg_maxcurrent1p8v (Maximum Current for 1.8V). No 1.8V support.                                               */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 4 (SDnIRV4) (reg50_dt) New for                                                  */
/**************************************************************************************************************************/
#define  SD1IRV4                        (GCR_VIRT_BASE_ADDR + 0x200) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 200h */
#define  SD2IRV4                        (GCR_VIRT_BASE_ADDR + 0x220) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 220h  */
#define  SDnIRV4_corecfg_initpresetval   0 , 13             /* 12-0 corecfg_initpresetval (Preset Value for Initialization). Assumes driver strength “type B”, SPECLK=50 MHz.        */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 5 (SDnIRV5) (reg54_dt) New for                                                  */
/**************************************************************************************************************************/
#define  SD1IRV5                        (GCR_VIRT_BASE_ADDR + 0x204) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 204h */
#define  SD2IRV5                        (GCR_VIRT_BASE_ADDR + 0x224) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 224h  */
#define  SDnIRV5_corecfg_dsppresetval    0 , 13             /* 12-0 corecfg_dsppresetval (Preset Value for Default Speed). Assumes driver strength “type B”, SPECLK=50 MHz.          */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 6 (SDnIRV6) (reg58_dt) New for                                                  */
/**************************************************************************************************************************/
#define  SD1IRV6                        (GCR_VIRT_BASE_ADDR + 0x208) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 208h */
#define  SD2IRV6                        (GCR_VIRT_BASE_ADDR + 0x228) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 228h  */
#define  SDnIRV6_corecfg_hsppresetval    0 , 13             /* 12-0 corecfg_hsppresetval (Preset Value for High Speed). Assumes driver strength “type B”, SPECLK=50 MHz.             */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 7 (SDnIRV7) (reg5C_dt) New for                                                  */
/**************************************************************************************************************************/
#define  SD1IRV7                        (GCR_VIRT_BASE_ADDR + 0x20C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 20Ch */
#define  SD2IRV7                        (GCR_VIRT_BASE_ADDR + 0x22C) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 22Ch  */
#define  SDnIRV7_corecfg_sdr12presetval  0 , 13             /* 12-0 corecfg_sdr12presetval (Preset Value for SDR12). Assumes driver strength “type B”, SPECLK=50 MHz. See            */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 8 (SDnIRV8) (reg60_dt) New for                                                  */
/**************************************************************************************************************************/
#define  SD1IRV8                        (GCR_VIRT_BASE_ADDR + 0x210) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 210h */
#define  SD2IRV8                        (GCR_VIRT_BASE_ADDR + 0x230) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 230h  */
#define  SDnIRV8_corecfg_sdr25presetval  0 , 13             /* 12-0 corecfg_sdr25presetval (Preset Value for SDR25). Assumes driver strength “type B”, SPECLK=50 MHz. See            */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 9 (SDnIRV9) (reg64_dt) New for                                                  */
/**************************************************************************************************************************/
#define  SD1IRV9                        (GCR_VIRT_BASE_ADDR + 0x214) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 214h */
#define  SD2IRV9                        (GCR_VIRT_BASE_ADDR + 0x234) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 234h  */
#define  SDnIRV9_corecfg_sdr50presetval  0 , 13             /* 12-0 corecfg_sdr50presetval (Preset Value for SDR50). Assumes driver strength “type B”, SPECLK=50 MHz. See            */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 10 (SDnIRV10) (reg68_dt) New for                                                */
/**************************************************************************************************************************/
#define  SD1IRV10                       (GCR_VIRT_BASE_ADDR + 0x218) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 218h */
#define  SD2IRV10                       (GCR_VIRT_BASE_ADDR + 0x238) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 238h  */
#define  SDnIRV10_corecfg_sdr104presetval  0 , 13             /* 12-0 corecfg_sdr104presetval (Preset Value for SDR104). Assumes driver strength “type B”, SPECLK=50 MHz.              */

/**************************************************************************************************************************/
/*   SDHCn Interface Reset Value Register 11 (SDnIRV11) (reg6C_dt) New for                                                */
/**************************************************************************************************************************/
#define  SD1IRV11                       (GCR_VIRT_BASE_ADDR + 0x21C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: SD: GCR_BA + 21Ch */
#define  SD2IRV11                       (GCR_VIRT_BASE_ADDR + 0x23C) , NPCMX50_GCR_ACCESS, 32 		/* MMC: GCR_BA + 23Ch  */
#define  SDnIRV11_corecfg_ddr50presetval  0 , 13             /* 12-0 corecfg_ddr50presetval (Preset Value for DDR50). Assumes driver strength “type B”, SPECLK=50 MHz. See            */

/**************************************************************************************************************************/
/*   Integration Control Register 2 (INTCR2) (Updated for )                                                               */
/**************************************************************************************************************************/
#define  INTCR2                         (GCR_VIRT_BASE_ADDR + 0x060) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 060h */
#define  INTCR2_PORST                    31 , 1              /* 31 PORST (Standby Power On* Reset Status). Updated by ROM code from RESSR register; indicates a Standby               */
#define  INTCR2_CORST                    30 , 1              /* 30 CORST (Core Domain Reset Status). Updated by ROM code from RESSR register; indicates a Core Domain                 */
#define  INTCR2_WD0RST                   29 , 1              /* 29 WD0RST (Watchdog 0 Reset Status). Updated by ROM code from RESSR register; indicates a Watchdog 0                  */
#define  INTCR2_SWR1ST                   28 , 1              /* 28 SWR1ST (Software Reset 1 Status). Updated by ROM code from RESSR register; indicates a Software reset              */
#define  INTCR2_SWR2ST                   27 , 1              /* 27 SWR2ST (Software Reset 2 Status). Updated by ROM code from RESSR register; indicates a Software reset              */
#define  INTCR2_SWR3ST                   26 , 1              /* 26 SWR3ST (Software Reset 3 Status). Updated by ROM code from RESSR register; indicates a Software reset              */
#define  INTCR2_SWR4ST                   25 , 1              /* 25 SWR4ST (Software Reset 4 Status). Updated by ROM code from RESSR register; indicates a Software reset              */
#define  INTCR2_WD1RST                   24 , 1              /* 24 WD1RST (Watchdog 1 Reset Status). Updated by ROM code from RESSR register; indicates a Watchdog 1                  */
#define  INTCR2_WD2RST                   23 , 1              /* 23 WD2RST (Watchdog 2 Reset Status). Updated by ROM code from RESSR register; indicates a Watchdog 2                  */
#define  INTCR2_WDC                      21 , 1              /* 21 WDC (Watchdog Counter). Scratch pad. This bit is used by ROM code for watchdog sequence.                           */
#define  INTCR2_rChosenImage             20 , 1              /* 20 rChosenImage (Current Chosen Image). Scratch pad. This bit is used by ROM code for watchdog sequence.              */
#define  INTCR2_MC_INIT                  19 , 1              /* 19 MC_INIT. Indicates that Memory controller and PHY initialization was done. The software that initialize the Memory 
                                                                   controller and PHY, should clear this bit before initialization and set it at the end of it                        */
#define  INTCR2_CFG_DONE                 18 , 1              /* 18 CFGDone (Clock Configuration indication). Scratch pad. The ROM code sets this bit to indicate to upper             */
#define  INTCR2_Scratchpad2              10 , 8              /* 17-10 Scratchpad. May be used by software for any purpose. Check ROM code and Boot Loader documentation for           */
#define  INTCR2_CP_FUSTRAP5_4             8 , 2              /* 9-8 Scratchpad. In Z1 the BMC ROM code should put here the value of CP_FUSTRAP5-4 after calculating its majority rule */
                                                              /* CP_FUSTRAP5-4 (Boot Source). Selects the start address of the CP boot sequence when CP_FUSTRAP1 is ‘1’.
                                                                Bits
                                                                4 3 Boot Address
                                                                0 0: Address 8000_0000 - SPI0 chip-select 0 (default)
                                                                0 1: Address 8800_0000 - SPI0 chip-select 1.
                                                                1 0: Address A000_0000 - SPI3 chip-select 0.
                                                                1 1: Address A800_0000 - SPI3 chip-select 1. */
#define  INTCR2_CP_FUSTRAP1              7,  1               /* CP fustarp1. CP_FUSTRAP1 (CP Core Release). Enables the CP core to run immediately after reset if set:
                                                                    0: CP core held at reset until released by the BMC CPU software by setting CPRSTREL bit in CPCTL
                                                                       register (default). In this case the BMC CPU determines the CP start address as programmed in
                                                                       B2CPST1-0 registers. See COPROCESSOR ROM CODE on page 771.
                                                                    1: CP core runs immediately after reset.
                                                                    used for a workaround for a bug in reading fustraps for CP. */
#define  INTCR2_GIVCRST                   6 , 1              /* 6 GIVCRST (Graphics (second head) interface vertical counters reset enable).                                          */
#define  INTCR2_GIHCRST                   5 , 1              /* 5 GIHCRST (Graphics (second head) interface Horizontal Counters Reset Enable).                                        */
#define  INTCR2_H2RBRST                   4 , 1              /* 4 H2RBRST (GFX (second head) Interface Register Block Reset).                                                         */
#define  INTCR2_H2RST1                    3 , 1              /* 3 H2RST1 (GFX (second head) interface Reset Level 1).                                                                 */
#define  INTCR2_GIRST2                    2 , 1              /* 2 GIRST2 (Graphics (second head) interface Reset Level 2).                                                            */
#define  INTCR2_USB2FS                    1 , 1              /* 1 USB2FS (USB2 PHY Force Suspend). Forces the USB 2.0 PHY to suspend mode.                                            */

/**************************************************************************************************************************/
/*   Multiple Function Pin Select Register 3 (MFSEL3) Updated for                                                         */
/**************************************************************************************************************************/
#define  MFSEL3                         (GCR_VIRT_BASE_ADDR + 0x064) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 064h */
#define  MFSEL3_HSDVOSEL                 26 , 1              /* 26 HSDVOSEL (HSTL DVO Select). Selects DDR DVO on RGMII2 pins (HSTL levels). In Z2 and later. */
#define  MFSEL3_MMCCDSEL                 25 , 1              /* 25 MMCCDSEL (nMMCCD Select). Selects GPIO155 or nMMCCD. In Z2 and later.                         */
#define  MFSEL3_GPOCSEL                  22 , 1              /* 22 GPOCSEL (GPOC Select). Selects either GPOI207-206 and GPIO205-204 or CRT2 digital signals.                         */
#define  MFSEL3_WDO2SEL                  20 , 1              /* 20 WDO2SEL (nWDO2 Select). Selects GPIO219 or nWDO2.                                                                  */
#define  MFSEL3_WDO1SEL                  19 , 1              /* 19 WDO1SEL (nWDO1 Select). Selects GPIO218 or nWDO1.                                                                  */
#define  MFSEL3_IOXHSEL                  18 , 1              /* 18 IOXHSEL (Host Serial I/O Expander Select). Selects Host SIOX pins or other options. When this bit is set,          */
#define  MFSEL3_PCIEPUSE                 17 , 1              /* 17 PCIEPUSE (PCI Express PHY Usage). Selects the PCI Express interface connected to the PHY. PIPE bus                 */
#define  MFSEL3_CLKRUNSEL                16 , 1              /* 16 CLKRUNSEL (CLKRUN Select). Selects GPIO168 or LPC signal nCLKRUN. When this bit is 1, MFSEL4.8                     */
#define  MFSEL3_IOX2SEL                  14 , 1              /* 14 IOX2SEL (I/O Expander 2 Select). Selects I/O Expander 2 interface option.                                          */
#define  MFSEL3_PSPI2SEL                 13 , 1              /* 13 PSPI2SEL (PSPI2 Select). Selects PSPI Signals or GPIO option.                                                      */
#define  MFSEL3_SD1SEL                   12 , 1              /* 12 SD1SEL (SD1 Select). Selects SD1 or GPIO option.                                                                   */
#define  MFSEL3_MMC8SEL                  11 , 1              /* 11 MMC8SEL (MMC Select). Selects four additional data lines for MMC or GPIO option.                                   */
#define  MFSEL3_MMCSEL                   10 , 1              /* 10 MMCSEL (MMC Select). Selects MMC or GPIO option.                                                                   */
#define  MFSEL3_RMII1SEL                 9 , 1               /* 9 RMII1SEL (RMII1 Select). Selects RMII1 (EMC1 or GMAC1 module) or GPIO option.                                       */
#define  MFSEL3_SMB15SEL                 8 , 1               /* 8 SMB15SEL (SMB15 Select). Selects SMBus15 signals or GPIO21-20. When this bit is set, MFSEL2 bits 25-24              */
#define  MFSEL3_SMB14SEL                 7 , 1               /* 7 SMB14SEL (SMB14 Select). Selects SMBus14 signals or GPIO23-22. When this bit is set, MFSEL2 bits 27-26              */
#define  MFSEL3_SMB13SEL                 6 , 1               /* 6 SMB13SEL (SMB13 Select). Selects SMBus13 signals or GPIO223-222.                                                    */
#define  MFSEL3_SMB12SEL                 5 , 1               /* 5 SMB12SEL (SMB12 Select). Selects SMBus12 signals or GPIO221-220.                                                    */
#define  MFSEL3_PSPI1SEL                 3 , 2              /* 4-3 PSPI1SEL (PSPI1 Select). Selects PSPI1, FANINs or GPIOs.                                                          */
#define  MFSEL3_SMB7SEL                  2 , 1               /* 2 SMB7SEL (SMB7 Select). Selects SMBus7 signals or GPIO174-173.                                                       */
#define  MFSEL3_SMB6SEL                  1 , 1               /* 1 SMB6SEL (SMB6 Select). Selects SMBus6 signals or GPIO172-171.                                                       */
#define  MFSEL3_SCISEL                   0 , 1               /* 0 SCISEL (SCI Select). Selects nSCIPME or GPIO169.                                                                    */

/**************************************************************************************************************************/
/*   Slew Rate Control Register (SRCNT) (Updated in )                                                                     */
/**************************************************************************************************************************/
#define  SRCNT                          (GCR_VIRT_BASE_ADDR + 0x068) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 068h */
#define  SRCNT_TDO                       4 , 1               /* 4 TDO. Slew Rate for TDO output. Controls the slew rate of TDO.                                                       */
#define  SRCNT_ESPI                      3 , 1               /* 3 ESPI. Slew Rate for ESPI and LPC. Controls the two I/O cells slew rate of LAD3-0/ ESPI_IO3-0/ GPIO167-164.          */
#define  SRCNT_SPI0C                     2 , 1               /* 2 SPI0C. Slew Rate for SPI0CK Signal.                                                                                 */
#define  SRCNT_SPI0D                     1 , 1               /* 1 SPI0D. Slew Rate for SPI0D1-0 Signals.                                                                              */

/**************************************************************************************************************************/
/*   Reset Status Register (RESSR) (Changed in )                                                                          */
/**************************************************************************************************************************/
#define  RESSR                          (GCR_VIRT_BASE_ADDR + 0x06C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 06Ch */
#define  RESSR_PORST                     31 , 1              /* 31 PORST (Standby Power On Reset Status). This bit is set by Standby Power-Up reset (nPORST is asserted).             */
#define  RESSR_CORST                     30 , 1              /* 30 CORST (Core Domain Reset Status). This bit is set by nCORST assertion. See INTCR2 bit 30 for indication.           */
#define  RESSR_WD0RST                    29 , 1              /* 29 WD0RST (Watchdog 0 Reset Status). This bit is set by Watchdog 0 reset event See INTCR2 bit 29 for                  */
#define  RESSR_SWRST1                    28 , 1              /* 28 SWRST1 (Software Reset 1 Status). This bit is set by Software reset 1 (PMCON.SWRST1 was set). See                  */
#define  RESSR_SWRST2                    27 , 1              /* 27 SWRST2 (Software Reset 2 Status). This bit is set by Software reset 2 (PMCON.SWRST2 was set) See                   */
#define  RESSR_SWRST3                    26 , 1              /* 26 SWRST3 (Software Reset 3 Status). This bit is set by Software reset 3 (PMCON.SWRST3 was set). See                  */
#define  RESSR_SWRST4                    25 , 1              /* 25 SWRST4 (Software Reset 4 Status). This bit is set by Software reset 4 (PMCON.SWRST4 was set). See                  */
#define  RESSR_WD1RST                    24 , 1              /* 24 WD1RST (Watchdog 1 Reset Status). This bit is set by Watchdog 1 reset event. See INTCR2 bit 24 for                 */
#define  RESSR_WD2RST                    23 , 1              /* 23 WD2RST (Watchdog 2 Reset Status). This bit is set by Watchdog 2 reset event See INTCR2 bit 23 for                  */


/**************************************************************************************************************************/
/*   Register Lock Register 1 (RLOCKR1) (Updated in )                                                                     */
/**************************************************************************************************************************/
#define  RLOCKR1                        (GCR_VIRT_BASE_ADDR + 0x070) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 070h */
#define  RLOCKR1_CORSTLK                 27 , 1              /* 27 CORSTLK. Locks CORSTC register, when set. (added in )                                                              */
#define  RLOCKR1_SWR34LK                 26 , 1              /* 26 SWR34LK. Locks SWRSTC3 and SWRSTC4 registers, when set. (added in )                                                */
#define  RLOCKR1_SWR12LK                 25 , 1              /* 25 SWR12LK. Locks SWRSTC1 and SWRSTC2 registers, when set. (added in )                                                */
#define  RLOCKR1_WD2RLK                  24 , 1              /* 24 WD2RLK. Locks WD2RCR register, when set. (added in )                                                               */
#define  RLOCKR1_WD1RLK                  23 , 1              /* 23 WD1RLK. Locks WD1RCR register, when set. (added in )                                                               */
#define  RLOCKR1_WD0RLK                  22 , 1              /* 22 WD0RLK. Locks WD0RCR register, when set. (added in )                                                               */
#define  RLOCKR1_IPR3LK                  21 , 1              /* 21 IPR3LK. Locks IPSRST3 register, when set. (added in )                                                              */
#define  RLOCKR1_CKE3LK                  20 , 1              /* 20 CKE3LK. Locks CLKEN3 register, when set. (added in )                                                               */
#define  RLOCKR1_PLL2LK                  19 , 1              /* 19 PLL2LK. Locks PLLCON2 register and PLLCONG register, when set. (added in )                                         */
#define  RLOCKR1_SEL4K                   18 , 1              /* 18 SEL4K. Locks MFSEL4 register, when set. (added in )                                                                */
#define  RLOCKR1_IPR2LK                  17 , 1              /* 17 IPR2LK. Locks IPSRST2 register, when set.                                                                          */
#define  RLOCKR1_IPR1LK                  16 , 1              /* 16 IPR1LK. Locks IPSRST1 register, when set.                                                                          */
#define  RLOCKR1_PLL1LK                  15 , 1              /* 15 PLL1LK. Locks PLLCON1 register, when set.                                                                          */
#define  RLOCKR1_PLL0LK                  14 , 1              /* 14 PLL0LK. Locks PLLCON0 register, when set.                                                                          */
#define  RLOCKR1_CKDVLK                  13 , 1              /* 13 CKDVLK. Locks CLKDIV1, CLKDIV2 and CLKDIV3 registers, when set.                                                    */
#define  RLOCKR1_CKSLLK                  12 , 1              /* 12 CKSLLK. Locks CLKSEL register, when set.                                                                           */
#define  RLOCKR1_CKE2LK                  11 , 1              /* 11 CKE2LK. Locks CLKEN2 register, when set.                                                                           */
#define  RLOCKR1_CKE1LK                  10 , 1              /* 10 CKE1LK. Locks CLKEN1 register, when set.                                                                           */
#define  RLOCKR1_DS1LK                   9 , 1               /* 9 DS1LK. Locks DSCNT register, when set.                                                                              */
#define  RLOCKR1_HIFLK                   8 , 1               /* 8 HIFLK. Locks HIFCR register, when set.                                                                              */
#define  RLOCKR1_SR1LK                   6 , 1               /* 6 SR1LK. Locks SRCNT register, when set.                                                                              */
#define  RLOCKR1_SEL3LK                  5 , 1               /* 5 SEL3LK. Locks MFSEL3 register, when set.                                                                            */
#define  RLOCKR1_VSRCLK                  4 , 1               /* 4 VSRCLK. Locks VSRCR register, when set. (changed in )                                                               */
#define  RLOCKR1_I2CSSLK                 3 , 1               /* 3 I2CSSLK. Locks I2CSEGSEL register, when set. (changed in )                                                          */
#define  RLOCKR1_MPELK                   2 , 1               /* 2 MPELK. Locks MISCPE register, when set.                                                                             */
#define  RLOCKR1_SEL2LK                  1 , 1               /* 1 SEL2LK. Locks MFSEL2 register, when set.                                                                            */
#define  RLOCKR1_SEL1LK                  0 , 1               /* 0 SEL1LK. Locks MFSEL1 register, when set.                                                                            */

/**************************************************************************************************************************/
/*   Function Lock Register 1 (FLOCKR1) (Updated in )                                                                     */
/**************************************************************************************************************************/
#define  FLOCKR1                        (GCR_VIRT_BASE_ADDR + 0x074) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 074h */
#define  FLOCKR1_ICCFGLK                 23 , 1              /* 23 ICCFGLK. Locks itself and INCCFGD bit.                                                                             */
#define  FLOCKR1_INTCCFGD                22 , 1              /* 22 INTCCFGD. Disables write access to specific Interrupt Controller LSPI registers. (Connected to                     */
#define  FLOCKR1_PSMILK                  21 , 1              /* 21 PSMILK. Locks itself and PSMISEL bit.                                                                              */
#define  FLOCKR1_PSMISEL                 20 , 1              /* 20 PSMISEL. Defines the selection of Periodic SMI or GPIO pin. (function added in )                                   */
#define  FLOCKR1_RAMVLK                  19 , 1              /* 19 RAMVLK. Locks itself and FAMV bit.                                                                                 */
#define  FLOCKR1_RAMV                    18 , 1              /* 18 RAMV. Defines what is mapped to address FFFF_0000h to FFFF_00FFh (256 bytes). This bit is reset                    */
#define  FLOCKR1_JDISLK                  17 , 1              /* 17 JDISLK (JTAG Disable Lock). Locks itself and JTAGDIS bit.                                                          */
#define  FLOCKR1_JTAGDIS                 16 , 1              /* 16 JTAGDIS (JTAG Disable). Disables JTAG access to NPCM750.                                                           */
#define  FLOCKR1_LKSRAM2                 15 , 1              /* 15 LKSRAM2 (SRAM2 Lock). Locks itself and FCTL7 bit.                                                                  */
#define  FLOCKR1_CNTSRAM2                14 , 1              /* 14 CNTSRAM2 (SRAM2 Control). Function control bit. This enables writing to 1 KB block in RAM2.                        */
#define  FLOCKR1_LKSRAM1                 13 , 1              /* 13 LKSRAM1 (SRAM1 Lock). Locks itself and FCTL6 bit.                                                                  */
#define  FLOCKR1_CNTSRAM1                12 , 1              /* 12 CNTSRAM1 (SRAM1 Control). Function control bit. This enables writing to 1 KB block in RAM2.                        */
#define  FLOCKR1_LKGPO2VLK               11 , 1              /* 11 LKGPO2VLK (LKGPO2 Value Lock). Locks itself and FCTL5.                                                             */
#define  FLOCKR1_LKGPO2VAL               10 , 1              /* 10 LKGPO2VAL (LKGPO2 Value). Function control bit. The bit controls the value of LKGPO2.                              */
#define  FLOCKR1_LKGPO2SLK               9 , 1               /* 9 LKGPO2SLK (LKGPO2 Select Lock). Locks itself and FCTL4.                                                             */
#define  FLOCKR1_LKGPO2SEL               8 , 1               /* 8 LKGPO2SEL (LKGPO2 Select). Function control bit. The bit controls selection of LKGPO2.                              */
#define  FLOCKR1_LKGPO1VLK               7 , 1               /* 7 LKGPO1VLK (LKGPO1 Value Lock). Locks itself and FCTL3.                                                              */
#define  FLOCKR1_LKGPO1VAL               6 , 1               /* 6 LKGPO1VAL (LKGPO1 Value). Function control bit. The bit controls the value of LKGPO1.                               */
#define  FLOCKR1_LKGPO1SLK               5 , 1               /* 5 LKGPO1SLK (LKGPO1 Select Lock). Locks itself and FCTL2.                                                             */
#define  FLOCKR1_LKGPO1SEL               4 , 1               /* 4 LKGPO1SEL (LKGPO1 Select). Function control bit. The bit controls selection of LKGPO1.                              */
#define  FLOCKR1_LKGPO0VLK               3 , 1               /* 3 LKGPO0VLK (LKGPO0 Value Lock). Locks itself and FCTL1.                                                              */
#define  FLOCKR1_LKGPO0VAL               2 , 1               /* 2 LKGPO0VAL (LKGPO0 Value). Function control bit. The bit controls the value of LKGPO0.                               */
#define  FLOCKR1_LKGPO0SLK               1 , 1               /* 1 LKGPO0SLK (LKGPO0 Select Lock). Locks itself and FCTL0.                                                             */
#define  FLOCKR1_LKGPO0SEL               0 , 1               /* 0 LKGPO0SEL (LKGPO0 Select). Function control bit. The bit controls selection of LKGPO0.                              */


/**************************************************************************************************************************/
/*   Drive Strength Control Register (DSCNT) (Updated for )                                                               */
/**************************************************************************************************************************/
#define  DSCNT                          (GCR_VIRT_BASE_ADDR + 0x078) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 078h */
#define  DSCNT_SPLD                      9 , 1               /* 9 SPLD. Drive Strength for nSPILOAD.                                                                                  */
#define  DSCNT_ESPI                      6 , 3              /* 8-6 ESPI. Drive Strength for ESPI and LPC. Controls the 2 I/O cells drive strength of LAD3-0/ESPI_IO3-0/              */
#define  DSCNT_JTDO                      5 , 1               /* 5 JTDO. Drive Strength for JTAG TDO.                                                                                  */
#define  DSCNT_DDC1                      4 , 1               /* 4 DDC1. Drive Strength for DDC1SDA and DDC1SCL.                                                                       */
#define  DSCNT_SYNC1                     3 , 1               /* 3 SYNC1. Drive Strength for HSYNC1 and VSYNC1.                                                                        */
#define  DSCNT_SPI0C                     2 , 1               /* 2 SPI0C. Drive Strength for SPI0CK.                                                                                   */
#define  DSCNT_SPI0D                     1 , 1               /* 1 SPI0D. Drive Strength for SPI0D1-0.                                                                                 */

/**************************************************************************************************************************/
/*   Module Disable Lock Register (MDLR) (Updated for )                                                                   */
/**************************************************************************************************************************/
#define  MDLR                           (GCR_VIRT_BASE_ADDR + 0x7C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 7Ch */
#define  MDLR_SMB1                       31 , 1              /* 31 SMB1 Module Disable Control.                                                                                       */
#define  MDLR_SMB0                       30 , 1              /* 30 SMB0 Module Disable Control.                                                                                       */
#define  MDLR_PSP1                       29 , 1              /* 29 PSP1 and PSPI2 Module Disable Control.                                                                             */
#define  MDLR_MMC                        28 , 1              /* 28 MMC Controller Module Disable Control.                                                                             */
#define  MDLR_ADC                        27 , 1              /* 27 ADC Module Disable Control.                                                                                        */
#define  MDLR_SD                         26 , 1              /* 26 SD Controller Module Disable Control.                                                                              */
#define  MDLR_SIOX1                      25 , 1              /* 25 SIOX1 and SIOX2 Module Disable Control.                                                                            */
#define  MDLR_PECI                       24 , 1              /* 24 PECI Module Disable Control.                                                                                       */
#define  MDLR_SMB15_8                    23 , 1              /* 23 SMB15-8 Modules Disable Control.                                                                                   */
#define  MDLR_USBD5_9                    22 , 1              /* 22 USBD5-9 Module Disable Control. USB Devices 5-9.                                                                   */
#define  MDLR_USBD2_4                    21 , 1              /* 21 USBD2-4 Module Disable Control. USB Devices 2-4.                                                                   */
#define  MDLR_AHBTOPCIBRG                20 , 1              /* 20 AHBTOPCIBRG Module Disable Control. AHB-to-PCI Bridge.                                                             */
#define  MDLR_ECE                        19 , 1              /* 19 ECE Module Disable Control.                                                                                        */
#define  MDLR_SHA                        18 , 1              /* 18 SHA Module Disable Control.                                                                                        */
#define  MDLR_PKA                        17 , 1              /* 17 PKA Module Disable Control.                                                                                        */
#define  MDLR_SMB7_5                     16 , 1              /* 16 SMB7-5 Module Disable Control.                                                                                     */
#define  MDLR_SMB4                       15 , 1              /* 15 SMB4 Module Disable Control.                                                                                       */
#define  MDLR_SMB3                       14 , 1              /* 14 SMB3 Module Disable Control.                                                                                       */
#define  MDLR_SMB2                       13 , 1              /* 13 SMB2 Module Disable Control.                                                                                       */
#define  MDLR_KEYOTP                     12 , 1              /* 12 KEYOTP Module Disable Control. Key OTP fuse block.                                                                 */
#define  MDLR_AES                        10 , 1              /* 10 AES Module Disable Control.                                                                                        */
#define  MDLR_USBD0                      9 , 1               /* 9 USBD0 Module Disable Control.                                                                                       */
#define  MDLR_USBD1                      8 , 1               /* 8 USBD1 Module Disable Control.                                                                                       */
#define  MDLR_GMAC1_2                    7 , 1               /* 7 GMAC1-2 Modules Disable Control.                                                                                    */
#define  MDLR_EMC1                       6 , 1               /* 6 EMC1 and EMC2 Module Disable Control.                                                                               */
#define  MDLR_GDMA                       5 , 1               /* 5 GDMA Modules Disable Control.                                                                                       */
#define  MDLR_DVC                        4 , 1               /* 4 DVC Module Disable Control.                                                                                         */
#define  MDLR_VDMA                       3 , 1               /* 3 VDMA Module Disable Control.                                                                                        */
#define  MDLR_VGAONLY                    2 , 1               /* 2 VGAONLY. Graphics core accelerators are disabled; only VGA functionality is enabled.                                */
#define  MDLR_VDMX                       1 , 1               /* 1 VDMX Module Disable Control.                                                                                        */
#define  MDLR_VCD                        0 , 1               /* 0 VCD Module Disable Control.                                                                                         */

/**************************************************************************************************************************/
/*   Scratchpad Register 2 (SCRPAD2) (New in POLEG A1)                                                                    */
/**************************************************************************************************************************/
#define  SCRPAD2                        (GCR_VIRT_BASE_ADDR + 0x084) , NPCMX50_GCR_ACCESS, 32        /* Offset: GCR_BA + 084h */

/**************************************************************************************************************************/
/*   Scratchpad Register 2 (SCRPAD3) (New in POLEG A1)                                                                    */
/**************************************************************************************************************************/
#define  SCRPAD3                        (GCR_VIRT_BASE_ADDR + 0x080) , NPCMX50_GCR_ACCESS, 32        /* Offset: GCR_BA + 080h */

/**************************************************************************************************************************/
/*   Slew Rate Control Register 2 (SRCNT2) (Removed in )                                                                  */
/**************************************************************************************************************************/
#define  SRCNT2                         (GCR_VIRT_BASE_ADDR + 0x094) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 094h */

/**************************************************************************************************************************/
/*   DAC Level Control Register (DACLVLR) (updated in )                                                                   */
/**************************************************************************************************************************/
#define  DACLVLR                        (GCR_VIRT_BASE_ADDR + 0x098) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 098h */
#define  DACLVLR_CONNSNSLVL              24 , 8             /* 31-24 CONNSNSLVL. Used as level setting by the Connection sense mechanism. (Eng note: 8 bit MSB of the required       */
#define  DACLVLR_BLUELEVEL               12 , 6             /* 17-12 BLUELEVEL. Controls the output full scale current of the blue DAC. Setting as in bits 5-0 of this register.     */
#define  DACLVLR_GREENLEVEL              6 , 6              /* 11-6 GREENLEVEL. Controls the output full scale current of the green DAC. Setting as in bits 5-0 of this register.    */
#define  DACLVLR_REDLEVEL                0 , 6              /* 5-0 REDLEVEL. Controls the output full scale current of the red DAC. Reasonable values are 0Bh to 13h. The            */

/**************************************************************************************************************************/
/*   Integration Control Register 3 (INTCR3) (Updated for )                                                               */
/**************************************************************************************************************************/
#define  INTCR3                         (GCR_VIRT_BASE_ADDR + 0x09C) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 09Ch */
#define  INTCR3_SSIDPWR                  31 , 1              /* 31 SSIDPWR (Subsystem ID Reset). Graphics core PCI subsystem ID Reset setting.                                        */
#define  INTCR3_USBLPBK                  24 , 1              /* 24 USBLPBK (USB Loopback Mode). USB loopback mode enables the USB host to be connected to the internal                */
#define  INTCR3_DACSBYOFF                23 , 1              /* 23 DACSBYOFF (DAC Standby Off). DAC off in standby.                                                                   */
#define  INTCR3_RCCORER                  22 , 1              /* 22 RCCORER (Root Complex Core Reset). PCIe Root Complex Core Reset.                                                   */
#define  INTCR3_GFX_ORL                  20 , 2             /* 21-20 GFX_ORL (Graphics Outstanding Read Request Limit). Limits the number of the outstanding read requests           */
#define  INTCR3_PECIVSEL                 19 , 1              /* 19 PECIVSEL (PECI Voltage Select). Adapts PECI I/O interface to voltage range. The interface is always                */
#define  INTCR3_A2PZ1C                   18 , 1              /* 18 A2PZ1C (AHB-to-PCI Z1 Compatibility). AHB2PCI Yarkon Z1 compatibility.                                             */
#define  INTCR3_PCIEDEM                  17 , 1              /* 17 PCIEDEM (PCI Express De-Emphasis). Selects the level of de-emphasis for an PCI Express upstream                    */
#define  INTCR3_DRAMINIT                 16 , 1              /* 16 DRAMINIT (DRAM Initialization). Stalls the streaming interfaces from GFX to MC stalled.                            */
#define  INTCR3_NAT_ON2                  15 , 1              /* 15 NAT_ON2 (EMC2_NAT_ON). EMC2 NAT_ON input strap. Used in as 0. An engineering setting that we                       */
#define  INTCR3_NAT_ON1                  14 , 1              /* 14 NAT_ON1 (EMC1 NAT_ON). EMC1 NAT_ON input strap. Used in as 0. An engineering setting that we                       */
#define  INTCR3_USBPHYSW                 12 , 2             /* 13-12 USBPHYSW (USB PHY2 Switch). Selects USB Host, USB PHY2 and USB Device 0 UTMI connection.                        */
#define  INTCR3_eSPI_INT_POL             11 , 1              /* 11 eSPI_INT_POL (eSPI Interrupt Polarity). When set to 1, the interrupt polarity bit at index 71 of the selected      */
#define  INTCR3_GMMAP                    8 , 3              /* 10-8 GMMAP (Graphics Memory Map). Configures Graphics core address mapping into main memory, according to             */
#define  INTCR3_PCIeENHD                 7 , 1               /* 7 PCIeENHD (PCI Express Bridge Enhancement Disable).                                                                  */
#define  INTCR3_GFXACCDIS                4 , 1               /* 4 GFXACCDIS (Graphics Core Access Disable). Disables access to PCI configuration of the Graphics core.                */
#define  INTCR3_HSRDIS                   2 , 1               /* 2 HSRDIS. Disables Host PCIe Secondary Reset to PCI devices. This bit added as an engineering feature.                */
#define  INTCR3_BIOSEN                   1 , 1               /* 1 BIOSEN (BIOS Enable). BIOS boot ROM included in a flash connected to SPI3.                                          */
#define  INTCR3_HHRDIS                   0 , 1               /* 0 HHRDIS (Host PCIe Hot Reset Disable). Disables Host PCIe Hot Reset to PCI devices.                                  */


/**************************************************************************************************************************/
/*   VSYNC Interrupt Control Register (VSINTR) (Unchanged in )                                                            */
/**************************************************************************************************************************/
#define  VSINTR                         (GCR_VIRT_BASE_ADDR + 0x0AC) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 0ACh */
#define  VSINTR_VSINTST                  2 , 1               /* 2 VSINTST (VSYNC Interrupt Status). Head 1 VSYNC Interrupt status.                                                    */
#define  VSINTR_VSINTEDGE                1 , 1               /* 1 VSINTEDGE (VSYNC Interrupt Edge Select).                                                                            */
#define  VSINTR_VSINTEN                  0 , 1               /* 0 VSINTEN (VSYNC Interrupt Enable).                                                                                   */

/**************************************************************************************************************************/
/*   Multiple Function Pin Select Register 4 (MFSEL4) (Updated in )                                                       */
/**************************************************************************************************************************/
#define  MFSEL4                         (GCR_VIRT_BASE_ADDR + 0x0B0) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 0B0h */
#define  MFSEL4_SMB11DDC                 29 , 2             /* 30-29 SMB11DDC. Enables SMB11 to control or emulate the DDC signals.                                                  */
#define  MFSEL4_SXCS1SEL                 28 , 1              /* 28 SXCS1SEL (SPIX Chip Select 1 Select). Selects SPIXCS1 or GPIO signals.                                             */
#define  MFSEL4_SPXSEL                   27 , 1              /* 27 SPXSEL (SPIX Select). Selects SPIX or GPIO signals.                                                                */
#define  MFSEL4_RG2SEL                   24 , 1              /* 24 RG2SEL (RGMII 2 Select). Selects RGMII 2 (for GMAC2) instead of GPIO.                                              */
#define  MFSEL4_RG2MSEL                  23 , 1              /* 23 RG2MSEL (RGMII 2 MDIO Select). Selects RGMII 2 MDIO or GPIO.                                                       */
#define  MFSEL4_RG1SEL                   22 , 1              /* 22 RG1SEL (RGMII 1 Select). Selects RGMII 1 (for GMAC1) instead of GPIO.                                              */
#define  MFSEL4_RG1MSEL                  21 , 1              /* 21 RG1MSEL (RGMII 1 MDIO Select). Selects RGMII1 MDIO or GPIO.                                                        */
#define  MFSEL4_SP3QSEL                  20 , 1              /* 20 SP3QSEL (SPI3 Quad Select). Selects SPI3 quad data option. When this bit is set, MFSEL4 bits S3CS2SEL              */
#define  MFSEL4_S3CS3SEL                 19 , 1              /* 19 S3CS3SEL (SPI3CS3 Select). Selects SPI3CS3 or GPIO signals.                                                        */
#define  MFSEL4_S3CS2SEL                 18 , 1              /* 18 S3CS2SEL (SPI3CS2 Select). Selects SPI3CS2 or GPIO signals.                                                        */
#define  MFSEL4_S3CS1SEL                 17 , 1              /* 17 S3CS1SEL (SPI3CS1 Select). Selects SPI3CS1 or GPIO signals.                                                        */
#define  MFSEL4_SP3SEL                   16 , 1              /* 16 SP3SEL (SPI3 Select). Selects SPI3 or GPIO signals.                                                                */
#define  MFSEL4_SP0QSEL                  15 , 1              /* 15 SP0QSEL (SPI0 Quad Select). Selects SPI0 quad data option. When this bit is set, MFSEL1 bits S0CS2SEL              */
#define  MFSEL4_SMB11SEL                 14 , 1              /* 14 SMB11SEL (SMB11 Select). Selects SMB11 or GPIO option.                                                             */
#define  MFSEL4_SMB10SEL                 13 , 1              /* 13 SMB10SEL (SMB10 Select). Selects SMB10 or GPIO option.                                                             */
#define  MFSEL4_SMB9SEL                  12 , 1              /* 12 SMB9SEL (SMB9 Select). Selects SMB9 or GPIO option.                                                                */
#define  MFSEL4_SMB8SEL                  11 , 1              /* 11 SMB8SEL (SMB8 Select). Selects SMB8 or GPIO option.                                                                */
#define  MFSEL4_DBGTRSEL                 10 , 1              /* 10 DBGTRSEL (Debug Trace Select). Selects debug trace or GPIO signals.                                                */
#define  MFSEL4_CKRQSEL                  9 , 1               /* 9 CKRQSEL (nCKRQ Signal Select). Selects GPIO or PCIe nCLKREQ signal.                                                 */
#define  MFSEL4_ESPISEL                  8 , 1               /* 8 ESPISEL (eSPI Signal Select). Selects either LPC or eSPI signals. If this bit is set, MFSEL1 bit LPCSEL must        */
#define  MFSEL4_MMCRSEL                  6 , 1               /* 6 MMCRSEL (MMC Reset Control Select). Selects either MMC reset control or card detect signals. If this bit is         */
#define  MFSEL4_SD1PSEL                  5 , 1               /* 5 SD1PSEL (SDHC1 Power Control Select). Selects either SDHC1 power control or card detect signals. If this            */
#define  MFSEL4_ROSEL                    4 , 1               /* 4 ROSEL (RIng Oscillator Output Select). Selects either RNGOSCOUT (Ring Oscillator output) or                         */
#define  MFSEL4_ESPIPMESEL               2 , 2              /* 3-2 ESPIPMESEL (ESPI PME Connection Select). Selects nSCIPME connection to PME# and SCI# virtual wires                */
#define  MFSEL4_BSPASEL                  1 , 1               /* 1 BSPASEL (BMC Serial Port Alternate Port Select). Selects BSP (BMC UART0) onto Serial Interface 2 pins.              */
#define  MFSEL4_JTAG2SEL                 0 , 1               /* 0 JTAG2SEL (Coprocessor Debug Port Select). Selects SI1/GPIOs or JTAG2.                                               */

/**************************************************************************************************************************/
/*   I2C Segment Pin Select Register (I2CSEGSEL) (New in )                                                                */
/**************************************************************************************************************************/
#define  I2CSEGSEL                      (GCR_VIRT_BASE_ADDR + 0x0E0) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 0E0h */
#define  I2CSEGSEL_S4DESEL               23 , 1              /* 23 S4DESEL. Selects either GPIO or SMBus 4 Drive Enable signals. If this bit is set, MFSEL3 bit 13 must be 0.         */
#define  I2CSEGSEL_S0DESEL               22 , 1              /* 22 S0DESEL. Selects either GPIO or SMBus 0 Drive Enable signals. If this bit is set, MFSEL1 bits 18-20 must be 0.     */
#define  I2CSEGSEL_S5SDSEL               21 , 1              /* 21 S5SDSEL. Selects either GPIOs or SMBus 5 segment D signals. If this bit is set, MFSEL1 bit 17 must be 0.           */
#define  I2CSEGSEL_S5CSEL                20 , 1              /* 20 S5CSEL. Selects either GPIOs or SMBus 5 segment C signals. If this bit is set, MFSEL1 bit 24 must be 0.            */
#define  I2CSEGSEL_S5SBSEL               19 , 1              /* 19 S5SBSEL. Selects either GPIOs or SMBus 5 segment B signals. If this bit is set, MFSEL1 bit 24 must be 0.           */
#define  I2CSEGSEL_S4DECFG               17 , 2             /* 18-17 S4DECFG. Selects SMBus 4 Drive Enable controls.                                                                 */
#define  I2CSEGSEL_S4SDSEL               16 , 1              /* 16 S4SDSEL. Selects either GPIOs or SMBus 4 segment D signals. If this bit is set, MFSEL2 bits 27-26 must be 0.       */
#define  I2CSEGSEL_S4CSEL                15 , 1              /* 15 S4CSEL. Selects either GPIOs or SMBus 4 segment C signals. If this bit is set, MFSEL2 bits 25-24 must be 0.        */
#define  I2CSEGSEL_S4SBSEL               14 , 1              /* 14 S4SBSEL. Selects either GPIOs or SMBus 4 segment B signals. If this bit is set, MFSEL3 bit 13 must be 0.           */
#define  I2CSEGSEL_S3SDSEL               13 , 1              /* 13 S3SDSEL. Selects either GPIOs or SMBus 3 segment D signals. If this bit is set, MFSEL2.31-30 must be 0.            */
#define  I2CSEGSEL_S3SCSEL               12 , 1              /* 12 S3SCSEL. Selects either GPIOs or SMBus 3 segment C signals. If this bit is set, MFSEL1.DVOSEL must be 0.           */
#define  I2CSEGSEL_S3SBSEL               11 , 1              /* 11 S3SBSEL. Selects either GPIOs or SMBus 3 segment B signals. If this bit is set, MFSEL1.DVOSEL must be 0.           */
#define  I2CSEGSEL_S2SDSEL               10 , 1              /* 10 S2SDSEL. Selects either GPIOs or SMBus 2 segment D signals. If this bit is set, MFSEL3.14 must be 0.               */
#define  I2CSEGSEL_S2SCSEL               9 , 1               /* 9 S2SCSEL. Selects either GPIOs or SMBus 2 segment C signals. If this bit is set, MFSEL1.DVOSEL must be 0.            */
#define  I2CSEGSEL_S2SBSEL               8 , 1               /* 8 S2SBSEL. Selects either GPIOs or SMBus 2 segment B signals. If this bit is set, MFSEL1.DVOSEL must be 0.            */
#define  I2CSEGSEL_S1SDSEL               7 , 1               /* 7 S1SDSEL. Selects either GPIOs or SMBus 1 segment D signals. If this bit is set, MFSEL3.14 must be 0.                */
#define  I2CSEGSEL_S1SCSEL               6 , 1               /* 6 S1SCSEL. Selects either GPIOs or SMBus 1 segment C signals. If this bit is set, MFSEL1.DVOSEL must be 0.            */
#define  I2CSEGSEL_S1SBSEL               5 , 1               /* 5 S1SBSEL. Selects either GPIOs or SMBus 1 segment B signals. If this bit is set, MFSEL1.DVOSEL must be 0.            */
#define  I2CSEGSEL_S0DECFG               3 , 2              /* 4-3 S0DECFG. Selects SMBus 0 Drive Enable controls.                                                                   */
#define  I2CSEGSEL_S0SDSEL               2 , 1               /* 2 S0SDSEL. Selects either GPIOs or SMBus 0 segment D signals. If this bit is set, MFSEL1.DVOSEL must be 0.            */
#define  I2CSEGSEL_S0SCSEL               1 , 1               /* 1 S0SCSEL. Selects either GPIOs or SMBus 0 segment C signals. If this bit is set, MFSEL1.DVOSEL must be 0.            */
#define  I2CSEGSEL_S0SBSEL               0 , 1               /* 0 S0SBSEL. Selects either GPIOs or SMBus 0 segment B signals. If this bit is set, MFSEL1.DVOSEL must be 0.            */

/**************************************************************************************************************************/
/*   I2C Segment Control Register (I2CSEGCTL) (New in )                                                                   */
/**************************************************************************************************************************/
#define  I2CSEGCTL                      (GCR_VIRT_BASE_ADDR + 0x0E4) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 0E4h */
#define  I2CSEGCTL_S4DWE                 25 , 1              /* 25 S4DWE. This bit must be written as 1 to enable writing to S4DEN bit in the same write transaction.                 */
#define  I2CSEGCTL_S0DWE                 21 , 1              /* 21 S0DWE. This bit must be written as 1 to enable writing to S0DEN bit in the same write transaction.                 */
#define  I2CSEGCTL_WEN5SS                17 , 1              /* 17 WEN5SS. This bit must be written as 1 to enable writing to SMB5SS field in the same write                          */
#define  I2CSEGCTL_WEN4SS                16 , 1              /* 16 WEN4SS. This bit must be written as 1 to enable writing to SMB4SS field in the same write                          */
#define  I2CSEGCTL_WEN3SS                15 , 1              /* 15 WEN3SS. This bit must be written as 1 to enable writing to SMB3SS field in the same write                          */
#define  I2CSEGCTL_WEN2SS                14 , 1              /* 14 WEN2SS. This bit must be written as 1 to enable writing to SMB2SS field in the same write                          */
#define  I2CSEGCTL_WEN1SS                13 , 1              /* 13 WEN1SS. This bit must be written as 1 to enable writing to SMB1SS field in the same write                          */
#define  I2CSEGCTL_WEN0SS                12 , 1              /* 12 WEN0SS. This bit must be written as 1 to enable writing to SMB0SS field in the same write                          */
#define  I2CSEGCTL_SMB5SS                11 , 0             /* SMB5SS. Selects SMB5 sub-segment operation. Signals in segments that are not selected float. If the                   */
#define  I2CSEGCTL_SMB4SS                8 , 2              /* SMB4SS. Selects SMB4 sub-segment operation. Signals in segments that are not selected float. If the                   */
#define  I2CSEGCTL_SMB3SS                6 , 2              /* SMB3SS. Selects SMB0 sub-segment operation. Signals in segments that are not selected float. If the                   */
#define  I2CSEGCTL_SMB2SS                4 , 2              /* SMB2SS. Selects SMB2 sub-segment operation. Signals in segments that are not selected float. If the                   */
#define  I2CSEGCTL_SMB1SS                2 , 2              /* SMB1SS. Selects SMB1 sub-segment operation. Signals in segments that are not selected float. If the                   */
#define  I2CSEGCTL_SMB0SS                0 , 2              /* SMB0SS. Selects SMB0 sub-segment operation. Signals in segments that are not selected float. If the                   */

/**************************************************************************************************************************/
/*   Voltage Supply Report Register (VSRCR) (New in )                                                                     */
/**************************************************************************************************************************/
#define  VSRCR                          (GCR_VIRT_BASE_ADDR + 0x0E8) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 0E8h */
#define  VSRCR_VSIF                      14 , 1              /* 14 VSIF. Reports the voltage level on supply VSBSIF.                                                                  */
#define  VSRCR_V14                       13 , 1              /* 13 V14. Reports the voltage level on supply VSBV14.                                                                   */
#define  VSRCR_V13                       12 , 1              /* 12 V13. Reports the voltage level on supply VSBV13.                                                                   */
#define  VSRCR_V12                       11 , 1              /* 11 V12. Reports the voltage level on supply VSBV12.                                                                   */
#define  VSRCR_V11                       10 , 1              /* 10 V11. Reports the voltage level on supply VSBV11.                                                                   */
#define  VSRCR_V10                       9 , 1               /* 9 V10. Reports the voltage level on supply VSBV10.                                                                    */
#define  VSRCR_V9                        8 , 1               /* 8 V9. Reports the voltage level on supply VSBV9.                                                                      */
#define  VSRCR_V8                        7 , 1               /* 7 V8. Reports the voltage level on supply VSBV8.                                                                      */
#define  VSRCR_V7                        6 , 1               /* 6 V7. Reports the voltage level on supply VSBV7.                                                                      */
#define  VSRCR_V6                        5 , 1               /* 5 V6. Reports the voltage level on supply VSBV6.                                                                      */
#define  VSRCR_V5                        4 , 1               /* 4 V5. Reports the voltage level on supply VSBV5.                                                                      */
#define  VSRCR_V4                        3 , 1               /* 3 V4. Reports the voltage level on supply VSBV4.                                                                      */
#define  VSRCR_V3                        2 , 1               /* 2 V3. Reports the voltage level on supply VSBV3.                                                                      */
#define  VSRCR_V2                        1 , 1               /* 1 V2. Reports the voltage level on supply VSBV2.                                                                      */
#define  VSRCR_V1                        0 , 1               /* 0 V1. Reports the voltage level on supply VSBV1.                                                                      */

/**************************************************************************************************************************/
/*   Coprocessor Control Register (CPCTL) (Changed in )                                                                   */
/**************************************************************************************************************************/
#define  CPCTL                          (GCR_VIRT_BASE_ADDR + 0x0D0) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 0D0h */
#define  CPCTL_CP_FUSTRAP10_1            16 , 10            /* 25-16 CP_FUSTRAP10-1 (Coprocessor FUSTRAP10-1). Values of Coprocessor CP_FUSTRAPs.                                    */
#define  CPCTL_CPBOOTC                   8 , 2              /* 9-8 CPBOOTC (Coprocessor Boot Control). These bits select the boot address of the CP core boot                        */
#define  CPCTL_BMCRST_STS                7 , 1               /* 7 BMCRST_STS (BMC Core Reset Status).                                                                                 */
#define  CPCTL_CPRST_STS                 6 , 1               /* 6 CPRST_STS (Coprocessor Reset Status).                                                                               */
#define  CPCTL_BMCRSTREL                 1 , 1               /* 1 BMCRSTREL (BMC Core Reset Release). This functionality is not implemented in .                                      */
#define  CPCTL_CPRSTREL                  0 , 1               /* 0 CPRSTREL (Coprocessor Reset Release). Releases coprocessor from reset after VSB Power-Up                            */

/**************************************************************************************************************************/
/*   This register reflects CP to BMC Notification Register 0 (CP2BNT0) on page 713 and CP to BMC Notification Register 1 */
/**************************************************************************************************************************/
#define  CP2BST                         (GCR_VIRT_BASE_ADDR + 0x0D4) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+0D4h */
#define  CP2BST_CP2BST                   0 , 32             /* 31-0 CP2BST (Coprocessor to BMC Core status).                                                                         */

/**************************************************************************************************************************/
/*   This register is reflected in registers BMC to CP Status Register 0 (B2CPST0) on page 712 and CP to BMC Notification Register*/
/**************************************************************************************************************************/
#define  B2CPNT                          (GCR_VIRT_BASE_ADDR + 0x0D8) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+0D8h */
#define  B2CPNT_B2CPNT                   0 , 32             /* 31-0 B2CPNT (BMC to Coprocessor Core Notification).                                                                   */

/**************************************************************************************************************************/
/*   Coprocessor Protection Control Register (CPPCTL) (Changed in )                                                       */
/**************************************************************************************************************************/
#define  CPPCTL                         (GCR_VIRT_BASE_ADDR + 0x0DC) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 0DCh */
#define  CPPCTL_CPD3L                    3 , 1               /* 3 CPD3L (Coprocessor and GDMA AHB3 Lock). Locks CP and GDMA access to AHB3. Disables any                              */
#define  CPPCTL_CPCL                     0 , 1               /* 0 CPCL (Coprocessor Control Registers Lock). Locks (Disables) CP write capability to System                           */

/**************************************************************************************************************************/
/*   Engineering Test Straps Register (ETSR) (Updated in Poleg)                                                           */
/**************************************************************************************************************************/
#define  ETSR                           (GCR_VIRT_BASE_ADDR + 0x110) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+110h */
#define  ETSR_ESTRP13_9                  9 , 5              /* 13-9 ESTRP13-9. TBD Engineering test straps.                                                                          */
#define  ETSR_ESTRP8                     8 , 1               /* 8 ESTRP8. SPI-QUAD Run Enable (XSRE).                                                                                 */
#define  ETSR_ESTRP7_1                   1 , 7              /* 7-1 ESTRP7-1. TBD Engineering test straps.                                                                            */

/**************************************************************************************************************************/
/*   Engineering DFT 1 Register (DFT1R)                                                                                   */
/**************************************************************************************************************************/
#define  DFT1R                          (GCR_VIRT_BASE_ADDR + 0x114) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+114h */

/**************************************************************************************************************************/
/*   Engineering DFT 2 Register (DFT2R)                                                                                   */
/**************************************************************************************************************************/
#define  DFT2R                          (GCR_VIRT_BASE_ADDR + 0x118) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+118h */

/**************************************************************************************************************************/
/*   Engineering DFT 3 Register (DFT3R)                                                                                   */
/**************************************************************************************************************************/
#define  DFT3R                          (GCR_VIRT_BASE_ADDR + 0x11C) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+11Ch */

/**************************************************************************************************************************/
/*   Engineering Debug Flip-Flop Scan Register (EDFFSR)                                                                   */
/**************************************************************************************************************************/
#define  EDFFSR                         (GCR_VIRT_BASE_ADDR + 0x120) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+120h */
#define  EDFFSR_AUTOSCAN                 4 , 1               /* 4 AUTOSCAN. (This bit and operation is optional, not implemented) Enables a very easy use of scan mode.               */
#define  EDFFSR_SCANMODE                 3 , 1               /* 3 SCANMODE. Enables easy entry to scan mode. Only TEST mode need be asserted, if this bit is set.                     */
#define  EDFFSR_SWSCANEVT                2 , 1               /* 2 SWSCANEVT. If set will trigger an event for clock change. Event from software (this bit) and hardware (selected     */
#define  EDFFSR_SCANMUX                  0 , 2              /* 1-0 SCANMUX. Enables one of the following signals to become an hardware event trigger for the Filp-Flop scan.         */

/**************************************************************************************************************************/
/*   Engineering Integration Control Register for PCIe 0 (INTCRPCE0) (TBD** in Poleg)                                     */
/**************************************************************************************************************************/
#define  INTCRPCE0                      (GCR_VIRT_BASE_ADDR + 0x130) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+130h */
#define  INTCRPCE0_cko_alive_con         0 , 32             /* 31-0 cko_alive_con = INTCRPCE0[1:0]; //Default value is 2'b00 (cko_alive clock is not used)                           */

/**************************************************************************************************************************/
/*   Engineering Integration Control Register for PCIe 1 (INTCRPCE1) (TBD** in Poleg)                                     */
/**************************************************************************************************************************/
#define  INTCRPCE1                      (GCR_VIRT_BASE_ADDR + 0x134) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+134h */
#define  INTCRPCE1_phy_tx_lvl            0 , 32             /* 31-0 phy_tx_lvl = INTCRPCE1[4:0]; //Default is 5'b10000 in Z1A, 6 in Z1.                                              */

/**************************************************************************************************************************/
/*   Engineering DAC Test Register (DACTEST) (TBD** in Poleg)                                                             */
/**************************************************************************************************************************/
#define  DACTEST                        (GCR_VIRT_BASE_ADDR + 0x138) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+138h */
#define  DACTEST_Entry                   0 , 32             /* 31-0 Entry into the test mode:                                                                                        */

/**************************************************************************************************************************/
/*   JUMP ADDRESS FOR SMP                                                   */
/**************************************************************************************************************************/
#define  SCRPAD                         (GCR_VIRT_BASE_ADDR + 0x13C) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+13Ch */

/**************************************************************************************************************************/
/*   Engineering Analog Test Control Register (ANTEST) (TBD** in Poleg)                                                   */
/**************************************************************************************************************************/
#define  ANTEST                         (GCR_VIRT_BASE_ADDR + 0x13C) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+13Ch */
#define  ANTEST_Regulator                0 , 8              /* 7-0 Regulator Calibration Value. One bit only must be set at a time.                                                  */

/**************************************************************************************************************************/
/*   USB PHY 1 Control Register (USB1PHYCTL) (Updated in Poleg)                                                           */
/**************************************************************************************************************************/
#define  USBTEST                        (GCR_VIRT_BASE_ADDR + 0x140) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+140h */
#define  USB1PHYCTL                     (GCR_VIRT_BASE_ADDR + 0x140) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+140h */
#define  USBPHY1_RS                     28 , 1             /* 28 This bit should be set after the PHY reset bit is released and the host is initialized. */


/**************************************************************************************************************************/
/*   USB PHY 2 Control Register (USB2PHYCTL) (Updated in Poleg)                                                           */
/**************************************************************************************************************************/
#define  USB2TEST                       (GCR_VIRT_BASE_ADDR + 0x144) , NPCMX50_GCR_ACCESS, 32 		/* Offset: GCR_BA + 144h */
#define  USB2PHYCTL                     (GCR_VIRT_BASE_ADDR + 0x144) , NPCMX50_GCR_ACCESS, 32 		/* Location: GCR_BA+144h */
#define  USBPHY2_RS                     28 , 1             /* 28 This bit should be set after the PHY reset bit is released and the host is initialized. */



/*---------------------------------------------------------------------------------------------------------*/
/* MFSEL dynamic access                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define MFSEL(x)            (GCR_VIRT_BASE_ADDR + (((x) < 2) ? (0x00C + (4*(x))) : 0x064)) , NPCMX50_GCR_ACCESS, 32

/*---------------------------------------------------------------------------------------------------------*/
/* GPIOPxPE dynamic access                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIOPxPE(x)         ( GCR_VIRT_BASE_ADDR + (((x) < 8) ? 0x18 + (4*(x)) : 0x80 + (4*((x)-8)))), NPCMX50_GCR_ACCESS, 32


#endif /* _NPCMX50_GCR_REGS_H */
