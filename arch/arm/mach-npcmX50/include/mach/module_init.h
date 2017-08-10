/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2016 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   hal.h                                                                                                 */
/*            This file contains configurations for BMC HAL                                                */
/*  Project:                                                                                               */
/*            Linux Kernel                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _MODULE_INIT_H_
#define _MODULE_INIT_H_

extern void npcmx50_module_init(void);
extern void npcmx50_udc_replace_usb9(void);
extern int npcmx50_udc_enable_devices(int udc_id);
extern void npcmx50_sdhci_probe_mux(int sdhci_id);
extern void npcmx50_sdhci_probe_rst(int sdhci_id);

#endif // _MODULE_INIT_H_
