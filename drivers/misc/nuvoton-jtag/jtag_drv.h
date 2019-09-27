/* SPDX-License-Identifier: GPL-2.0
 *
 * Description   : JTAG driver
 *
 * Copyright (C) 2018 NuvoTon Corporation
 *
 */

#ifndef __JTAG_DRV_H__
#define __JTAG_DRV_H__

#define JTAG_SCAN_LEN	256
#define JTAG_MAX_XFER_DATA_LEN 65535

struct tck_bitbang {
	unsigned char     tms;
	unsigned char     tdi;        // TDI bit value to write
	unsigned char     tdo;        // TDO bit value to read
};
struct bitbang_packet {
	struct tck_bitbang *data;
	__u32	length;
} __attribute__((__packed__));

struct scan_xfer {
	unsigned int     length;      // number of bits
	unsigned char    tdi[JTAG_SCAN_LEN];
	unsigned int     tdi_bytes;
	unsigned char    tdo[JTAG_SCAN_LEN];
	unsigned int     tdo_bytes;
	unsigned int     end_tap_state;
};

struct jtag_xfer {
	__u8	type;
	__u8	direction;
	__u8	from;
	__u8	endstate;
	__u8	padding;
	__u32	length;
	__u64	tdio;
};

struct jtag_tap_state {
	__u8	reset;
	__u8	from;
	__u8	endstate;
	__u8	tck;
};
enum JtagStates {
	JtagTLR,
	JtagRTI,
	JtagSelDR,
	JtagCapDR,
	JtagShfDR,
	JtagEx1DR,
	JtagPauDR,
	JtagEx2DR,
	JtagUpdDR,
	JtagSelIR,
	JtagCapIR,
	JtagShfIR,
	JtagEx1IR,
	JtagPauIR,
	JtagEx2IR,
	JtagUpdIR,
	JTAG_STATE_CURRENT
};

enum JTAG_PIN {
	pin_TCK,
	pin_TDI,
	pin_TDO,
	pin_TMS,
	pin_NUM,
};

enum jtag_reset {
	JTAG_NO_RESET = 0,
	JTAG_FORCE_RESET = 1,
};

enum jtag_xfer_type {
	JTAG_SIR_XFER = 0,
	JTAG_SDR_XFER = 1,
};

enum jtag_xfer_direction {
	JTAG_READ_XFER = 1,
	JTAG_WRITE_XFER = 2,
	JTAG_READ_WRITE_XFER = 3,
};

/* legacy ioctl definitions */
#define JTAGIOC_BASE    'T'
#define JTAG_SIOCFREQ_OLD         _IOW(JTAGIOC_BASE, 3, unsigned int)
#define JTAG_GIOCFREQ_OLD         _IOR(JTAGIOC_BASE, 4, unsigned int)
#define JTAG_BITBANG          _IOWR(JTAGIOC_BASE, 5, struct tck_bitbang)
#define JTAG_SET_TAPSTATE     _IOW(JTAGIOC_BASE, 6, unsigned int)
#define JTAG_READWRITESCAN    _IOWR(JTAGIOC_BASE, 7, struct scan_xfer)
#define JTAG_SLAVECONTLR      _IOW(JTAGIOC_BASE, 8, unsigned int)
#define JTAG_RUNTEST          _IOW(JTAGIOC_BASE, 9, unsigned int)
#define JTAG_DIRECTGPIO       _IOW(JTAGIOC_BASE, 10, unsigned int)
#define JTAG_PSPI             _IOW(JTAGIOC_BASE, 11, unsigned int)
#define JTAG_PSPI_IRQ         _IOW(JTAGIOC_BASE, 12, unsigned int)

/* ioctl definitions for ASD */
#define __JTAG_IOCTL_MAGIC	0xb2
#define JTAG_SIOCSTATE	_IOW(__JTAG_IOCTL_MAGIC, 0, struct jtag_tap_state)
#define JTAG_SIOCFREQ	_IOW(__JTAG_IOCTL_MAGIC, 1, unsigned int)
#define JTAG_GIOCFREQ	_IOR(__JTAG_IOCTL_MAGIC, 2, unsigned int)
#define JTAG_IOCXFER	_IOWR(__JTAG_IOCTL_MAGIC, 3, struct jtag_xfer)
#define JTAG_GIOCSTATUS _IOWR(__JTAG_IOCTL_MAGIC, 4, enum JtagStates)
#define JTAG_SIOCMODE	_IOW(__JTAG_IOCTL_MAGIC, 5, unsigned int)
#define JTAG_IOCBITBANG	_IOW(__JTAG_IOCTL_MAGIC, 6, unsigned int)
#endif
