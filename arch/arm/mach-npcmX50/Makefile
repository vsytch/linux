#
# Makefile for the linux kernel.
#
#EXTERNAL_BMC_CONFIG := $(shell pwd)/arch/arm/mach-npcmX50/include/mach/hal.h
EXTERNAL_BMC_CONFIG := $$PWD/arch/arm/mach-npcmX50/include/mach/hal.h
EXTRA_CFLAGS += "-DEXTERNAL_INCLUDE_FILE=$(EXTERNAL_BMC_CONFIG)"

# Object file lists.
obj-y			:= npcmX50_timer.o npcmX50_module_init.o npcmX50_overlay.o
obj-m			:=
obj-n			:=
obj-			:=

obj-$(CONFIG_SMP)			+= npcmX50_smp.o headsmp.o 

# machine specific support
obj-$(CONFIG_ARCH_NPCMX50)	+= mach-npcmX50.o

obj-$(CONFIG_PCI)	+= npcmX50_pcie.o

# obj-y			+= BMC_HAL/Boards/board.o 

