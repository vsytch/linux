/*
 * Copyright (c) 2014 Nuvoton Technology corporation.
 * Copyright 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>

static const char *const npcm7xx_dt_match[] = {
	"nuvoton,npcm750",
	NULL
};

DT_MACHINE_START(NPCM7XX_DT, "NPCMX50 Chip family")
//	.map_io         = npcmX50_map_io,
	.atag_offset	= 0x100,
	.dt_compat	= npcm7xx_dt_match,
	.l2c_aux_val	= 0x0,
	.l2c_aux_mask	= ~0x0,
MACHINE_END
