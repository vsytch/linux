/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 * Copyright 2017 Google, Inc.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>

/*
 * The boot ROM does not start secondary CPUs in SVC mode, so we need to do that
 * here.
 */
ENTRY(npcm7xx_secondary_startup)
	safe_svcmode_maskall r0

	b	secondary_startup
ENDPROC(npcm7xx_secondary_startup)
