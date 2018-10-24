// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NPCM7xx SMB Controller driver
 *
 * Copyright (C) 2018 Nuvoton Technologies tali.perry@nuvoton.com
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk/nuvoton.h>
#include <linux/crc8.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/jiffies.h>

#define I2C_VERSION "0.0.5"

enum smb_mode {
	SMB_SLAVE = 1,
	SMB_MASTER
};

/*
 * External SMB Interface driver xfer indication values, which indicate status
 * of the bus.
 */
enum smb_state_ind {
	SMB_NO_STATUS_IND = 0,
	SMB_SLAVE_RCV_IND = 1,
	SMB_SLAVE_XMIT_IND = 2,
	SMB_SLAVE_XMIT_MISSING_DATA_IND = 3,
	SMB_SLAVE_RESTART_IND = 4,
	SMB_SLAVE_DONE_IND = 5,
	SMB_MASTER_DONE_IND = 6,
	SMB_NO_DATA_IND = 7,
	SMB_NACK_IND = 8,
	SMB_BUS_ERR_IND = 9,
	SMB_WAKE_UP_IND = 10,
	SMB_MASTER_PEC_ERR_IND = 11,
	SMB_BLOCK_BYTES_ERR_IND = 12,
	SMB_SLAVE_PEC_ERR_IND = 13,
	SMB_SLAVE_RCV_MISSING_DATA_IND = 14,
};

// SMBus Operation type values
enum smb_oper {
	SMB_NO_OPER = 0,
	SMB_WRITE_OPER = 1,
	SMB_READ_OPER = 2
};

// SMBus Bank (FIFO mode)
enum smb_bank {
	SMB_BANK_0 = 0,
	SMB_BANK_1 = 1
};

// Internal SMB states values (for the SMB module state machine).
enum smb_state {
	SMB_DISABLE = 0,
	SMB_IDLE,
	SMB_MASTER_START,
	SMB_SLAVE_MATCH,
	SMB_OPER_STARTED,
	SMB_STOP_PENDING
};

// Module supports setting multiple own slave addresses:
enum smb_addr {
	SMB_SLAVE_ADDR1 = 0,
	SMB_SLAVE_ADDR2,
	SMB_SLAVE_ADDR3,
	SMB_SLAVE_ADDR4,
	SMB_SLAVE_ADDR5,
	SMB_SLAVE_ADDR6,
	SMB_SLAVE_ADDR7,
	SMB_SLAVE_ADDR8,
	SMB_SLAVE_ADDR9,
	SMB_SLAVE_ADDR10,
	SMB_GC_ADDR,
	SMB_ARP_ADDR
};

// global regs
static struct regmap *gcr_regmap;
static struct regmap *clk_regmap;

#define NPCM_I2CSEGCTL  0xE4
#define NPCM_SECCNT	0x68
#define NPCM_CNTR25M	0x6C
#define I2CSEGCTL_VAL	0x0333F000

// Common regs
#define NPCM_SMBSDA			0x000
#define NPCM_SMBST			0x002
#define NPCM_SMBCST			0x004
#define NPCM_SMBCTL1			0x006
#define NPCM_SMBADDR1			0x008
#define NPCM_SMBCTL2			0x00A
#define NPCM_SMBADDR2			0x00C
#define NPCM_SMBCTL3			0x00E
#define NPCM_SMBCST2			0x018
#define NPCM_SMBCST3			0x019
#define SMB_VER				0x01F

// BANK 0 regs
#define NPCM_SMBADDR3			0x010
#define NPCM_SMBADDR7			0x011
#define NPCM_SMBADDR4			0x012
#define NPCM_SMBADDR8			0x013
#define NPCM_SMBADDR5			0x014
#define NPCM_SMBADDR9			0x015
#define NPCM_SMBADDR6			0x016
#define NPCM_SMBADDR10			0x017

// SMBADDR array: because the addr regs are sprincled all over the address space
const int  NPCM_SMBADDR[10] = {NPCM_SMBADDR1, NPCM_SMBADDR2, NPCM_SMBADDR3,
			       NPCM_SMBADDR4, NPCM_SMBADDR5, NPCM_SMBADDR6,
			       NPCM_SMBADDR7, NPCM_SMBADDR8, NPCM_SMBADDR9,
			       NPCM_SMBADDR10};

#define NPCM_SMBCTL4			0x01A
#define NPCM_SMBCTL5			0x01B
#define NPCM_SMBSCLLT			0x01C // SCL Low Time
#define NPCM_SMBFIF_CTL			0x01D // FIFO Control
#define NPCM_SMBSCLHT			0x01E // SCL High Time

// BANK 1 regs
#define NPCM_SMBFIF_CTS			0x010 // Both FIFOs Control and status
#define NPCM_SMBTXF_CTL			0x012 // Tx-FIFO Control
#define NPCM_SMBT_OUT			0x014 // Bus T.O.
#define NPCM_SMBPEC			0x016 // PEC Data
#define NPCM_SMBTXF_STS			0x01A // Tx-FIFO Status
#define NPCM_SMBRXF_STS			0x01C // Rx-FIFO Status
#define NPCM_SMBRXF_CTL			0x01E // Rx-FIFO Control

// NPCM_SMBST reg fields
#define NPCM_SMBST_XMIT			BIT(0)
#define NPCM_SMBST_MASTER		BIT(1)
#define NPCM_SMBST_NMATCH		BIT(2)
#define NPCM_SMBST_STASTR		BIT(3)
#define NPCM_SMBST_NEGACK		BIT(4)
#define NPCM_SMBST_BER			BIT(5)
#define NPCM_SMBST_SDAST		BIT(6)
#define NPCM_SMBST_SLVSTP		BIT(7)

// NPCM_SMBCST reg fields
#define NPCM_SMBCST_BUSY		BIT(0)
#define NPCM_SMBCST_BB			BIT(1)
#define NPCM_SMBCST_MATCH		BIT(2)
#define NPCM_SMBCST_GCMATCH		BIT(3)
#define NPCM_SMBCST_TSDA		BIT(4)
#define NPCM_SMBCST_TGSCL		BIT(5)
#define NPCM_SMBCST_MATCHAF		BIT(6)
#define NPCM_SMBCST_ARPMATCH		BIT(7)

// NPCM_SMBCTL1 reg fields
#define NPCM_SMBCTL1_START		BIT(0)
#define NPCM_SMBCTL1_STOP		BIT(1)
#define NPCM_SMBCTL1_INTEN		BIT(2)
#define NPCM_SMBCTL1_EOBINTE		BIT(3)
#define NPCM_SMBCTL1_ACK		BIT(4)
#define NPCM_SMBCTL1_GCMEN		BIT(5)
#define NPCM_SMBCTL1_NMINTE		BIT(6)
#define NPCM_SMBCTL1_STASTRE		BIT(7)

// RW1S fields (inside a RW reg):
#define NPCM_SMBCTL1_RWS_FIELDS	  (NPCM_SMBCTL1_START | NPCM_SMBCTL1_STOP | \
				   NPCM_SMBCTL1_ACK)
// NPCM_SMBADDR reg fields
#define NPCM_SMBADDR_ADDR		GENMASK(6, 0)
#define NPCM_SMBADDR_SAEN		BIT(7)

// NPCM_SMBCTL2 reg fields
#define SMBCTL2_ENABLE			BIT(0)
#define SMBCTL2_SCLFRQ6_0		GENMASK(7, 1)

// NPCM_SMBCTL3 reg fields
#define SMBCTL3_SCLFRQ8_7		GENMASK(1, 0)
#define SMBCTL3_ARPMEN			BIT(2)
#define SMBCTL3_IDL_START		BIT(3)
#define SMBCTL3_400K_MODE		BIT(4)
#define SMBCTL3_BNK_SEL			BIT(5)
#define SMBCTL3_SDA_LVL			BIT(6)
#define SMBCTL3_SCL_LVL			BIT(7)

// NPCM_SMBCST2 reg fields
#define NPCM_SMBCST2_MATCHA1F		BIT(0)
#define NPCM_SMBCST2_MATCHA2F		BIT(1)
#define NPCM_SMBCST2_MATCHA3F		BIT(2)
#define NPCM_SMBCST2_MATCHA4F		BIT(3)
#define NPCM_SMBCST2_MATCHA5F		BIT(4)
#define NPCM_SMBCST2_MATCHA6F		BIT(5)
#define NPCM_SMBCST2_MATCHA7F		BIT(5)
#define NPCM_SMBCST2_INTSTS		BIT(7)

// NPCM_SMBCST3 reg fields
#define NPCM_SMBCST3_MATCHA8F		BIT(0)
#define NPCM_SMBCST3_MATCHA9F		BIT(1)
#define NPCM_SMBCST3_MATCHA10F		BIT(2)
#define NPCM_SMBCST3_EO_BUSY		BIT(7)

// NPCM_SMBCTL4 reg fields
#define SMBCTL4_HLDT			GENMASK(5, 0)
#define SMBCTL4_LVL_WE			BIT(7)

// NPCM_SMBCTL5 reg fields
#define SMBCTL5_DBNCT			GENMASK(3, 0)

// NPCM_SMBFIF_CTS reg fields
#define NPCM_SMBFIF_CTS_RXF_TXE		BIT(1)
#define NPCM_SMBFIF_CTS_RFTE_IE		BIT(3)
#define NPCM_SMBFIF_CTS_CLR_FIFO	BIT(6)
#define NPCM_SMBFIF_CTS_SLVRSTR		BIT(7)

// NPCM_SMBTXF_CTL reg fields
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBTXF_CTL_TX_THR		GENMASK(5, 0)
#else
#define NPCM_SMBTXF_CTL_TX_THR		GENMASK(4, 0)
#endif
#define NPCM_SMBTXF_CTL_THR_TXIE	BIT(6)

// NPCM_SMBT_OUT reg fields
#define NPCM_SMBT_OUT_TO_CKDIV		GENMASK(5, 0)
#define NPCM_SMBT_OUT_T_OUTIE		BIT(6)
#define NPCM_SMBT_OUT_T_OUTST		BIT(7)

// NPCM_SMBTXF_STS reg fields
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBTXF_STS_TX_BYTES	GENMASK(5, 0)
#else
#define NPCM_SMBTXF_STS_TX_BYTES	GENMASK(4, 0)
#endif
#define NPCM_SMBTXF_STS_TX_THST		BIT(6)

// NPCM_SMBRXF_STS reg fields
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBRXF_STS_RX_BYTES	GENMASK(5, 0)
#else
#define NPCM_SMBRXF_STS_RX_BYTES	GENMASK(4, 0)
#endif
#define NPCM_SMBRXF_STS_RX_THST		BIT(6)

// NPCM_SMBFIF_CTL reg fields
#define NPCM_SMBFIF_CTL_FIFO_EN		BIT(4)

// NPCM_SMBRXF_CTL reg fields
// Note: on the next HW version of this module, this HW is about to switch to
//	 32 bytes FIFO. This size will be set using a config.
//	 on current version 16 bytes FIFO is set using a define
#ifdef SMB_CAPABILITY_32B_FIFO
#define NPCM_SMBRXF_CTL_RX_THR		GENMASK(5, 0)
#define NPCM_SMBRXF_CTL_THR_RXIE	BIT(6)
#define NPCM_SMBRXF_CTL_LAST_PEC	BIT(7)
#define SMBUS_FIFO_SIZE			32
#else
#define NPCM_SMBRXF_CTL_RX_THR		GENMASK(4, 0)
#define NPCM_SMBRXF_CTL_LAST_PEC	BIT(5)
#define NPCM_SMBRXF_CTL_THR_RXIE	BIT(6)
#define SMBUS_FIFO_SIZE			16
#endif

// SMB_VER reg fields
#define SMB_VER_VERSION			GENMASK(6, 0)
#define SMB_VER_FIFO_EN			BIT(7)

// stall/stuck timeout
const unsigned int DEFAULT_STALL_COUNT =	25;

// Data abort timeout
const unsigned int ABORT_TIMEOUT =	 10000;

// SMBus spec. values in KHZ
const unsigned int SMBUS_FREQ_MIN = 10;
const unsigned int SMBUS_FREQ_MAX = 1000;
const unsigned int SMBUS_FREQ_100KHZ = 100;
const unsigned int SMBUS_FREQ_400KHZ = 400;
const unsigned int SMBUS_FREQ_1MHZ = 1000;

// SCLFRQ min/max field values
const unsigned int SCLFRQ_MIN = 10;
const unsigned int SCLFRQ_MAX = 511;

// SCLFRQ field position
#define SCLFRQ_0_TO_6		GENMASK(6, 0)
#define SCLFRQ_7_TO_8		GENMASK(8, 7)

// SMB Maximum Retry Trials (on Bus Arbitration Loss)
const unsigned int SMB_RETRY_MAX_COUNT = 2;
const unsigned int SMB_NUM_OF_ADDR = 10;

// for logging:
#define NPCM_I2C_EVENT_START	BIT(0)
#define NPCM_I2C_EVENT_STOP	BIT(1)
#define NPCM_I2C_EVENT_ABORT	BIT(2)
#define NPCM_I2C_EVENT_WRITE	BIT(3)

#define NPCM_I2C_EVENT_READ	BIT(4)
#define NPCM_I2C_EVENT_BER	BIT(5)
#define NPCM_I2C_EVENT_NACK	BIT(6)
#define NPCM_I2C_EVENT_TO	BIT(7)

#define NPCM_I2C_EVENT_EOB	BIT(8)
#define NPCM_I2C_EVENT_STALL	BIT(9)
#define NPCM_I2C_EVENT_CB	BIT(10)
#define NPCM_I2C_EVENT_DONE	BIT(11)

#define NPCM_I2C_EVENT_READ1	BIT(12)
#define NPCM_I2C_EVENT_READ2	BIT(13)
#define NPCM_I2C_EVENT_READ3	BIT(14)
#define NPCM_I2C_EVENT_READ4	BIT(15)


#define NPCM_I2C_EVENT_LOG(event)   (bus->event_log |= event)

#define SMB_RECOVERY_SUPPORT

// slave mode: if end device reads more data than available, ask issuer or
// request for more data:
#define SMB_WRAP_AROUND_BUFFER

// Status of one SMBus module
struct npcm_i2c {
	struct i2c_adapter	adap;
	struct device		*dev;
	unsigned char __iomem	*reg;
	spinlock_t		lock;   /* IRQ synchronization */
	struct completion	cmd_complete;
	int			irq;
	int			cmd_err;
	struct i2c_msg		*msgs;
	int			msgs_num;
	int			num;
	u32			apb_clk;
	enum smb_state		state;
	enum smb_oper		operation;
	enum smb_mode		master_or_slave;
	enum smb_state_ind	stop_ind;
	u8			dest_addr;
	u8			*rd_buf;
	u16			rd_size;
	u16			rd_ind;
	u8			*wr_buf;
	u16			wr_size;
	u16			wr_ind;
	bool			fifo_use;
	u16			threshold_fifo;

	// PEC bit mask per slave address.
	//		1: use PEC for this address,
	//		0: do not use PEC for this address
	u16			PEC_mask;
	bool			PEC_use;
	u8			crc_data;
	bool			read_block_use;
	u8			retry_count;
	u8			int_cnt;
	u32			event_log;
	u32			event_log_prev;
	u32			clk_period_us;
	unsigned long 			int_time_stamp;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	u8				own_slave_addr;
	struct i2c_client		*slave;

	// currently I2C slave IF only supports single byte operations.
	// in order to utilyze the npcm HW FIFO, the driver will ask for 16bytes
	// at a time, pack them in buffer, and then transmit them all together
	// to the FIFO and onward to the bus .
	// NACK on read will be once reached to bus->adap->quirks->max_read_len
	// sending a NACK whever the backend requests for it is not supported.

	// This module can be master and slave at the same time. seperate ptrs
	// and counters:
	u16			slv_rd_size;
	u16			slv_rd_ind;
	u16			slv_wr_size;
	u16			slv_wr_ind;

	u8 			slv_rd_buf[SMBUS_FIFO_SIZE];
	u8 			slv_wr_buf[SMBUS_FIFO_SIZE];
#endif

};



static inline void npcm_smb_select_bank(struct npcm_i2c *bus,
					enum smb_bank bank)
{
	if (bank == SMB_BANK_0)
		iowrite8(ioread8(bus->reg + NPCM_SMBCTL3) & ~SMBCTL3_BNK_SEL,
			 bus->reg + NPCM_SMBCTL3);
	else
		iowrite8(ioread8(bus->reg + NPCM_SMBCTL3) | SMBCTL3_BNK_SEL,
			 bus->reg + NPCM_SMBCTL3);
}


static inline bool npcm_smb_is_quick(struct npcm_i2c *bus);
static void pdebug(struct npcm_i2c *bus, char str[20])
{
	char str2[65];
	char *s = str2;
	int rd_size, wr_size, rd_ind, wr_ind;

	return; // for debug, remove this line..

	if (bus->master_or_slave == SMB_MASTER) {
		rd_size = bus->rd_size;
		wr_size = bus->wr_size;
		rd_ind = bus->rd_ind;
		wr_ind = bus->wr_ind;
	} else {
#if IS_ENABLED(CONFIG_I2C_SLAVE)		
		rd_size = bus->slv_rd_size;
		wr_size = bus->slv_wr_size;
		rd_ind = bus->slv_rd_ind;
		wr_ind = bus->slv_wr_ind;
#endif		
	}

	s += sprintf(s, "bus%d ", bus->num);
	s += sprintf(s, "SA%02x ", bus->dest_addr);

	switch (bus->stop_ind) {
	case SMB_NO_STATUS_IND:
		s += sprintf(s, "NO_STATUS_IND	            ");
		break;
	case SMB_SLAVE_RCV_IND:
		s += sprintf(s, "SLAVE_RCV_IND	            ");
		break;
	case SMB_SLAVE_XMIT_IND:
		s += sprintf(s, "SLAVE_XMIT_IND	            ");
		break;
	case SMB_SLAVE_XMIT_MISSING_DATA_IND:
		s += sprintf(s, "SLAVE_XMIT_MISSING_DATA_IND ");
		break;
	case SMB_SLAVE_RESTART_IND:
		s += sprintf(s, "SLAVE_RESTART_IND	    ");
		break;
	case SMB_SLAVE_DONE_IND:
		s += sprintf(s, "SLAVE_DONE_IND	            ");
		break;
	case SMB_MASTER_DONE_IND:
		s += sprintf(s, "MASTER_DONE_IND	    ");
		break;
	case SMB_NO_DATA_IND:
		s += sprintf(s, "NO_DATA_IND		    ");
		break;
	case SMB_NACK_IND:
		return;
		s += sprintf(s, "NACK_IND		    ");
		break;
	case SMB_BUS_ERR_IND:
		s += sprintf(s, "BUS_ERR_IND		    ");
		break;
	case SMB_WAKE_UP_IND:
		s += sprintf(s, "WAKE_UP_IND                ");
		break;
	case SMB_MASTER_PEC_ERR_IND:
		s += sprintf(s, "MASTER_PEC_ERR_IND	    ");
		break;
	case SMB_BLOCK_BYTES_ERR_IND:
		s += sprintf(s, "BLOCK_BYTES_ERR_IND	    ");
		break;
	case SMB_SLAVE_PEC_ERR_IND:
		s += sprintf(s, "SLAVE_PEC_ERR_IND	    ");
		break;
	case SMB_SLAVE_RCV_MISSING_DATA_IND:
		s += sprintf(s, "SLAVE_RCV_MISSING_DATA_IND ");
		break;
	}
	switch (bus->operation) {
	case SMB_NO_OPER:
		s += sprintf(s, "NO_OPER    ");
		break;
	case SMB_WRITE_OPER:
		s += sprintf(s, "WRITE_OPER ");
		break;
	case SMB_READ_OPER:
		s += sprintf(s, "READ_OPER  ");
		break;
	}
	switch (bus->state) {
	case  SMB_DISABLE:
		s += sprintf(s, "DISABLE	");
		break;
	case  SMB_IDLE:
		s += sprintf(s, "IDLE	        ");
		break;
	case  SMB_MASTER_START:
		s += sprintf(s, "MASTER_START   ");
		break;
	case  SMB_SLAVE_MATCH:
		s += sprintf(s, "SLAVE_MATCH    ");
		break;
	case  SMB_OPER_STARTED:
		s += sprintf(s, "OPER_STARTED   ");
		break;
	case  SMB_STOP_PENDING:
		s += sprintf(s, "STOP_PENDING   ");
		break;
	}

	npcm_smb_select_bank(bus, SMB_BANK_1);

	pr_err("%s %s wr%d,%d rd%d,%d int%d ev0x%02x (prv=0x%02x) blk%d quick%d\tST=%x\tCST=%x\tCTL1=%x\t CTL3=%x\tCST2=%x\tCST3=%x\tFIF_CTS=%x\tTXF_CTL=%x\tT_OUT=%x\tCST2=%x\tCST3=%x\tTXF_STS=%x\tRXF_STS=%x\tRXF_CTL=%x\n",
		str2, str, wr_size, wr_ind, rd_size, rd_ind,
		bus->int_cnt, bus->event_log, bus->event_log_prev, (int)bus->read_block_use,
		npcm_smb_is_quick(bus),


	ioread8(bus->reg + NPCM_SMBST),
	ioread8(bus->reg + NPCM_SMBCST),
	ioread8(bus->reg + NPCM_SMBCTL1 ),
	ioread8(bus->reg + NPCM_SMBCTL3 ),
	ioread8(bus->reg + NPCM_SMBCST2 ),
	ioread8(bus->reg + NPCM_SMBCST3 ),

	ioread8(bus->reg + NPCM_SMBFIF_CTS ),
	ioread8(bus->reg + NPCM_SMBTXF_CTL ),
	ioread8(bus->reg + NPCM_SMBT_OUT),
	ioread8(bus->reg + NPCM_SMBCST2 ),
	ioread8(bus->reg + NPCM_SMBCST3 ),
	ioread8(bus->reg + NPCM_SMBTXF_STS ),
	ioread8(bus->reg + NPCM_SMBRXF_STS ),
	ioread8(bus->reg + NPCM_SMBRXF_CTL )  );

	npcm_smb_select_bank(bus, SMB_BANK_0);
	pr_err("\tCTL4=%x\tCTL5=%x\tFIF_CTL=%x\n",
		ioread8(bus->reg + NPCM_SMBCTL4 ),
		ioread8(bus->reg + NPCM_SMBCTL5 ),
		ioread8(bus->reg + NPCM_SMBFIF_CTL ));
	npcm_smb_select_bank(bus, SMB_BANK_1);


}

static void pdebug_lvl2(struct npcm_i2c *bus, char str[20])
{
	// remove for in-depth debug:
	return;
	pdebug(bus, str);
}


DECLARE_CRC8_TABLE(npcm7xx_crc8);

static u8 npcm_smb_calc_crc8(u8 crc_data, u8 data)
{
	crc_data = crc8(npcm7xx_crc8, &data, 1, crc_data);
	return crc_data;
}

static void npcm_smb_calc_PEC(struct npcm_i2c *bus, u8 data)
{
	if (bus->PEC_use)
		bus->crc_data = npcm_smb_calc_crc8(bus->crc_data, data);
}

static inline void npcm_smb_wr_byte(struct npcm_i2c *bus, u8 data)
{
	iowrite8(data, bus->reg + NPCM_SMBSDA);
	npcm_smb_calc_PEC(bus, data);
}

static inline void npcm_smb_rd_byte(struct npcm_i2c *bus, u8 *data)
{
	*data = ioread8(bus->reg + NPCM_SMBSDA);
	npcm_smb_calc_PEC(bus, *data);
}

static inline u8 npcm_smb_get_PEC(struct npcm_i2c *bus)
{
	if (bus->PEC_use)
		return bus->crc_data;
	else
		return 0;
}

static inline void npcm_smb_write_PEC(struct npcm_i2c *bus)
{
	if (bus->PEC_use) {
		// get PAC value and write to the bus:
		npcm_smb_wr_byte(bus, npcm_smb_get_PEC(bus));
	}
}

static int npcm_smb_get_SCL(struct i2c_adapter *_adap)
{
	unsigned int ret = 0;
	struct npcm_i2c *bus = container_of(_adap, struct npcm_i2c, adap);

	// Get SCL level
	ret = FIELD_GET(SMBCTL3_SCL_LVL,  ioread8(bus->reg + NPCM_SMBCTL3));

	return ret;
}

static int npcm_smb_get_SDA(struct i2c_adapter *_adap)
{
	unsigned int ret = 0;
	struct npcm_i2c *bus = container_of(_adap, struct npcm_i2c, adap);

	// Get SDA level
	ret = FIELD_GET(SMBCTL3_SDA_LVL,  ioread8(bus->reg + NPCM_SMBCTL3));

	return ret;
}

static inline u16 npcm_smb_get_index(struct npcm_i2c *bus)
{
	u16 index = 0;

	if (bus->operation == SMB_READ_OPER)
		index = bus->rd_ind;
	else if (bus->operation == SMB_WRITE_OPER)
		index = bus->wr_ind;

	return index;
}

// quick protocol:
static inline bool npcm_smb_is_quick(struct npcm_i2c *bus)
{
	if (bus->wr_size == 0 && bus->rd_size == 0)
		return true;
	return false;
}

static void npcm_smb_disable(struct npcm_i2c *bus)
{
	int i;

	// select bank 0 for SMB addresses
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// Slave Addresses Removal
	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++)
		iowrite8(0, bus->reg + NPCM_SMBADDR[i]);

	// select bank 0 for SMB addresses
	npcm_smb_select_bank(bus, SMB_BANK_1);

	// Disable module.
	iowrite8(ioread8(bus->reg + NPCM_SMBCTL2) & ~SMBCTL2_ENABLE,
		 bus->reg + NPCM_SMBCTL2);

	// Set module disable
	bus->state = SMB_DISABLE;
}

static void npcm_smb_enable(struct npcm_i2c *bus)
{
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL2) | SMBCTL2_ENABLE),
		 bus->reg + NPCM_SMBCTL2);
}

// enable\disable end of busy (EOB) interrupt
static inline void npcm_smb_eob_int(struct npcm_i2c *bus, bool enable)
{
	if (enable) {
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) |
			 NPCM_SMBCTL1_EOBINTE)  & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
	} else {
		iowrite8(ioread8(bus->reg + NPCM_SMBCTL1) &
			 ~NPCM_SMBCTL1_EOBINTE & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);

		// Clear EO_BUSY pending bit:
		iowrite8(ioread8(bus->reg + NPCM_SMBCST3) |
			 NPCM_SMBCST3_EO_BUSY, bus->reg + NPCM_SMBCST3);
	}
}

static inline bool npcm_smb_tx_fifo_empty(struct npcm_i2c *bus)
{
	u8 tx_fifo_sts = ioread8(bus->reg + NPCM_SMBTXF_STS);

	// check if TX FIFO is not empty
	if ((tx_fifo_sts & NPCM_SMBTXF_STS_TX_BYTES) == 0)
		return false;

	// check if TX FIFO empty:
	return (bool)FIELD_GET(NPCM_SMBTXF_STS_TX_THST,
			       ioread8(bus->reg + NPCM_SMBTXF_STS));
}

static inline bool npcm_smb_rx_fifo_full(struct npcm_i2c *bus)
{
	u8 rx_fifo_sts = ioread8(bus->reg + NPCM_SMBRXF_STS);

	// check if RX FIFO is not empty:
	if ((rx_fifo_sts & NPCM_SMBRXF_STS_RX_BYTES) == 0)
		return false;

	// check if rx fifo full status is set:
	return (bool)FIELD_GET(NPCM_SMBRXF_STS_RX_THST,
			       ioread8(bus->reg + NPCM_SMBRXF_STS));
}


static inline void npcm_smb_clear_fifo_int(struct npcm_i2c *bus)
{
	iowrite8((ioread8(bus->reg + NPCM_SMBFIF_CTS) &
			NPCM_SMBFIF_CTS_SLVRSTR) |
			NPCM_SMBFIF_CTS_RXF_TXE,
			bus->reg + NPCM_SMBFIF_CTS);
}


static inline void npcm_smb_clear_tx_fifo(struct npcm_i2c *bus)
{
	iowrite8(ioread8(bus->reg + NPCM_SMBTXF_STS) | NPCM_SMBTXF_STS_TX_THST,
		 bus->reg + NPCM_SMBTXF_STS);
}

static inline void npcm_smb_clear_rx_fifo(struct npcm_i2c *bus)
{
	iowrite8(ioread8(bus->reg + NPCM_SMBRXF_STS) | NPCM_SMBRXF_STS_RX_THST,
			 bus->reg + NPCM_SMBRXF_STS);
}

static void npcm_smb_int_enable(struct npcm_i2c *bus, bool enable)
{
	if (enable)
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) |
			 NPCM_SMBCTL1_INTEN) & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
	else
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) &
			 ~NPCM_SMBCTL1_INTEN) & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
}

static inline void npcm_smb_master_start(struct npcm_i2c *bus)
{
	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_START);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_START) &
		 ~(NPCM_SMBCTL1_STOP | NPCM_SMBCTL1_ACK),
		 bus->reg + NPCM_SMBCTL1);
}

static void npcm_smb_set_fifo(struct npcm_i2c *bus, int nread, int nwrite);


static inline void npcm_smb_master_stop(struct npcm_i2c *bus)
{
	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_STOP);

	pdebug_lvl2(bus, "stop  ");

	// override HW issue: SMBus may fail to supply stop condition in Master
	// Write operation.
	// Need to delay at least 5 us from the last int, before issueing a stop
	udelay(10);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_STOP) &
		 ~(NPCM_SMBCTL1_START | NPCM_SMBCTL1_ACK),
		 bus->reg + NPCM_SMBCTL1);

	if (bus->fifo_use) {
		npcm_smb_select_bank(bus, SMB_BANK_1);

		if (bus->operation == SMB_READ_OPER)
			npcm_smb_clear_rx_fifo(bus);
		else
			npcm_smb_clear_tx_fifo(bus);

		iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTS) |
				 NPCM_SMBFIF_CTS_SLVRSTR |
				 NPCM_SMBFIF_CTS_RXF_TXE,
				 bus->reg + NPCM_SMBFIF_CTS);

		iowrite8(0, bus->reg + NPCM_SMBTXF_CTL);
	}
}


static inline void npcm_smb_stall_after_start(struct npcm_i2c *bus, bool stall)
{
	if (stall)
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) |
			 NPCM_SMBCTL1_STASTRE)  & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
	else
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) &
			 ~NPCM_SMBCTL1_STASTRE)  & ~NPCM_SMBCTL1_RWS_FIELDS,
			 bus->reg + NPCM_SMBCTL1);
}

static inline void npcm_smb_nack(struct npcm_i2c *bus)
{
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_ACK) &
		 ~(NPCM_SMBCTL1_STOP | NPCM_SMBCTL1_START),
		 bus->reg + NPCM_SMBCTL1);
}

static void npcm_smb_reset(struct npcm_i2c *bus)
{
	// Save NPCM_SMBCTL1 relevant bits. It is being cleared when the
	// module is disabled
	u8 smbctl1;
	pdebug(bus, "*** reset bus ***");

	smbctl1 = ioread8(bus->reg + NPCM_SMBCTL1);

	// Disable the SMB module
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL2) & ~SMBCTL2_ENABLE),
		 bus->reg + NPCM_SMBCTL2);

	// Enable the SMB module
	npcm_smb_enable(bus);

	// Restore NPCM_SMBCTL1 status
	iowrite8(smbctl1 & ~NPCM_SMBCTL1_RWS_FIELDS, bus->reg + NPCM_SMBCTL1);

	// Clear BB (BUS BUSY) bit
	iowrite8(NPCM_SMBCST_BB, bus->reg + NPCM_SMBCST);

	iowrite8(0xFF, bus->reg + NPCM_SMBST);

	// Clear EOB bit
	iowrite8(NPCM_SMBCST3_EO_BUSY, bus->reg + NPCM_SMBCST3);

	// Clear all fifo bits:
	iowrite8(NPCM_SMBFIF_CTS_CLR_FIFO, bus->reg + NPCM_SMBFIF_CTS);

	// Reset driver status
	bus->state = SMB_IDLE;
}

static inline bool npcm_smb_is_master(struct npcm_i2c *bus)
{
	return (bool)FIELD_GET(NPCM_SMBST_MASTER,
			       ioread8(bus->reg + NPCM_SMBST));
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static bool npcm_smb_slave_start_receive(struct npcm_i2c *bus, u16 nread,
					 u8 *read_data);
static bool npcm_smb_slave_start_xmit(struct npcm_i2c *bus, u16 nwrite,
				      u8 *write_data);
static void npcm_smb_slave_abort(struct npcm_i2c *bus);
//static int  npcm_i2c_slave_get_rd_buf(struct npcm_i2c *bus);
static int  npcm_i2c_slave_get_wr_buf(struct npcm_i2c *bus);
static void npcm_i2c_slave_send_rd_buf(struct npcm_i2c *bus);
static void npcm_i2c_slave_restart_or_stop_flush_wr_buf(struct npcm_i2c *bus);
#endif

static void npcm_smb_callback(struct npcm_i2c *bus,
			      enum smb_state_ind op_status, u16 info)
{
	struct i2c_msg *msgs = bus->msgs;
	int msgs_num = bus->msgs_num;

	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_CB);

	//if (op_status != 6)
		pdebug_lvl2(bus, "CB");
	switch (op_status) {
	case SMB_MASTER_DONE_IND:
	// Master transaction finished and all transmit bytes were sent
	// info: number of bytes actually received after the Master
	//	receive operation (if Master didn't issue receive it
	//	should be 0)
	// Notify that not all data was received on Master or Slave
	// info:
	//	on receive: number of actual bytes received
	//	when PEC is used even if 'info' is the expected number
	//	of bytes, it means that PEC error occurred.
	{
		if (msgs[0].flags & I2C_M_RD)
			msgs[0].len = info;
		else if (msgs_num == 2 && msgs[1].flags & I2C_M_RD)
			msgs[1].len = info;

		bus->cmd_err = 0;
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_DONE);

		if((ioread8(bus->reg + NPCM_SMBCST) & NPCM_SMBCST_BB) != 0)
			pdebug(bus, "WARNING busy at done");
		complete(&bus->cmd_complete);

	}
	break;

	case SMB_NO_DATA_IND:
	// Notify that not all data was received on Master or Slave
	// info:
	//on receive: number of actual bytes received
	//	when PEC is used even if 'info' is the expected number
	//	of bytes,it means that PEC error occurred.
	{
		if (msgs[0].flags & I2C_M_RD)
			msgs[0].len = info;
		else if (msgs_num == 2 && msgs[1].flags & I2C_M_RD)
			msgs[1].len = info;

		bus->cmd_err = -EFAULT;
		pdebug(bus, "CB NO DATA");
		if (bus->master_or_slave == SMB_MASTER)
			complete(&bus->cmd_complete);
	}
	break;
	case SMB_NACK_IND:
		// MASTER transmit got a NAK before transmitting all bytes
		// info: number of transmitted bytes
		bus->cmd_err = -EAGAIN;
		pdebug_lvl2(bus, "CB-NACK ");
		if (bus->master_or_slave == SMB_MASTER)
			complete(&bus->cmd_complete);

		break;
	case SMB_BUS_ERR_IND:
		// Bus error
		// info: has no meaning
		bus->cmd_err = -EIO;
		pdebug(bus, "CB BER  ");
		if (bus->master_or_slave == SMB_MASTER)
			complete(&bus->cmd_complete);
		break;
	case SMB_WAKE_UP_IND:
		pdebug(bus, "wake_up");
		// SMBus wake up
		// info: has no meaning
		break;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
		//extern u8  rd_buf[PAGE_SIZE];
		//extern u16 rd_size;
		//extern u8  wr_buf[32];
		//extern u16 wr_size;
	case SMB_SLAVE_RCV_IND:
		// Slave got an address match with direction bit clear so it
		//	should receive datathe int must call
		//      npcm_smb_slave_start_receive()
		//      info: the enum SMB_ADDR_T address match


		//npcm_i2c_slave_get_rd_buf(bus); // if the buffer is empty nothing will be sent

		// this module does not support saying no to bytes. it will always ACK.

		if(!npcm_smb_slave_start_receive(bus, 32*1024, bus->slv_rd_buf))
			printk("npcm_smb_slave_start_receive fail");

		pdebug_lvl2(bus, "CB start slv rcv   ");
		break;

	case SMB_SLAVE_RESTART_IND: // if switch from read to write forget the bytes saved from before.
	case SMB_SLAVE_XMIT_IND:
		// Slave got an address match with direction bit set so it
		//      should transmit data
		//	the int must call npcm_smb_slave_start_xmit()
		// info: the enum SMB_ADDR_T address match

		npcm_i2c_slave_send_rd_buf(bus); // send up whatever is on the buffer.

		npcm_i2c_slave_restart_or_stop_flush_wr_buf(bus);

		if(!npcm_smb_slave_start_xmit(bus,bus->slv_wr_size,
					  bus->slv_wr_buf))
			printk("npcm_smb_slave_start_xmit fail");

		pdebug_lvl2(bus, "CB start slv xmit  ");
		break;
	case SMB_SLAVE_DONE_IND:
		// Slave done transmitting or receiving
		// info:
		//	on receive: number of actual bytes received
		//	on transmit: number of actual bytes transmitted,
		//	when PEC is used 'info' should be (nwrite+1) which means
		//		 that 'nwrite' bytes were sent + the PEC byte
		//		'nwrite' is the second parameter
		//               npcm_smb_slave_start_xmit()

		npcm_i2c_slave_send_rd_buf(bus); // if the buffer is empty nothing will be sent

		i2c_slave_event(bus->slave, I2C_SLAVE_STOP, (u8 *)&info);

		bus->int_cnt = 0;
		pdebug_lvl2(bus, "CB_DONE");
		break;
#endif // CONFIG_I2C_SLAVE
	default:
		pdebug(bus, "CB default");
		break;
	}
}


static u32 npcm_smb_get_fifo_fullness(struct npcm_i2c *bus)
{
	if (bus->operation == SMB_WRITE_OPER)
		return FIELD_GET(NPCM_SMBTXF_STS_TX_BYTES,
				 ioread8(bus->reg + NPCM_SMBTXF_STS));
	else if (bus->operation == SMB_READ_OPER)
		return FIELD_GET(NPCM_SMBRXF_STS_RX_BYTES,
				 ioread8(bus->reg + NPCM_SMBRXF_STS));
	return 0;
}


static void npcm_smb_write_to_fifo_master(struct npcm_i2c *bus,
					  u16 max_bytes_to_send)
{
	pdebug_lvl2(bus, "wr_fifo_master");
	// Fill the FIFO, while the FIFO is not full and there are more bytes to
	// write
	while ((max_bytes_to_send--) && (SMBUS_FIFO_SIZE -
					 npcm_smb_get_fifo_fullness(bus))) {
		// write the data
		if (bus->wr_ind < bus->wr_size) {
			if (bus->PEC_use && (bus->wr_ind + 1 == bus->wr_size)) {
				// Master send PEC in write protocol, Slave send
				// PEC in read protocol.
				npcm_smb_write_PEC(bus);
				bus->wr_ind++;
			} else {
				npcm_smb_wr_byte(bus,
						 bus->wr_buf[bus->wr_ind++]);
			}
		} else {
			npcm_smb_wr_byte(bus, 0xFF);
		}
	}
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static void npcm_smb_write_to_fifo_slave(struct npcm_i2c *bus, u16 max_bytes_to_send)
{
	pdebug_lvl2(bus, "wr_fifo");
	// Fill the FIFO, while the FIFO is not full and there are more bytes to
	// write
	while ((max_bytes_to_send--) && (SMBUS_FIFO_SIZE -
					 npcm_smb_get_fifo_fullness(bus))) {
		// write the data
		if (bus->slv_wr_ind < bus->slv_wr_size) {
			if (bus->PEC_use &&
			    (bus->slv_wr_ind + 1 == bus->slv_wr_size) &&
			    (bus->slv_rd_size == 0 ||
			     bus->master_or_slave == SMB_SLAVE)) {
				// Master send PEC in write protocol, Slave send
				// PEC in read protocol.
				npcm_smb_write_PEC(bus);
				bus->slv_wr_ind++;
			} else {
				npcm_smb_wr_byte(bus,
						 bus->slv_wr_buf[bus->slv_wr_ind++]);
			}
		} else {
#ifdef SMB_WRAP_AROUND_BUFFER
			// We're out of bytes. Ask the higher level for
			// more bytes. Let it know that driver
			// used all its' bytes

			npcm_smb_clear_tx_fifo(bus);

			// Reset state for the remaining bytes transaction
			bus->state = SMB_SLAVE_MATCH;

			// Notify upper layer of transaction completion
			npcm_smb_callback(bus, SMB_SLAVE_XMIT_MISSING_DATA_IND,
					  bus->slv_wr_ind);

			iowrite8(NPCM_SMBST_SDAST, bus->reg + NPCM_SMBST);
#else
			npcm_smb_wr_byte(bus, 0xFF);
#endif
		}
	}
}
#endif

// configure the FIFO before using it. If nread is -1 RX FIFO will not be
// configured. same for	nwrite
static void npcm_smb_set_fifo(struct npcm_i2c *bus, int nread, int nwrite)
{
	u16 rxf_ctl = 0;
	if (!bus->fifo_use)
		return;
	npcm_smb_select_bank(bus, SMB_BANK_1);
	npcm_smb_clear_tx_fifo(bus);
	npcm_smb_clear_rx_fifo(bus);

	// configure RX FIFO
	if (nread > 0) {

		rxf_ctl = min((u16)nread, (u16)SMBUS_FIFO_SIZE);
		rxf_ctl |= NPCM_SMBRXF_CTL_THR_RXIE;

		// set LAST bit. if LAST is set enxt FIFO packet is nacked at the end.

		// regular read of less then buffer size:
		if (nread <= SMBUS_FIFO_SIZE)
			rxf_ctl |= NPCM_SMBRXF_CTL_LAST_PEC;
		// if we are about to read the first byte in blk rd mode,
		// don't NACK it. BTW , if slave return zero size HW can't NACK
		// it immidiattly, it will read enxtra byte and then NACK.
		if (bus->rd_ind == 0 && bus->read_block_use){
			rxf_ctl = NPCM_SMBRXF_CTL_THR_RXIE | 0x01;
		}

		// set fifo size:
		iowrite8(rxf_ctl, bus->reg + NPCM_SMBRXF_CTL);

	}

	// configure TX FIFO
	if (nwrite > 0) {
		if (nwrite > SMBUS_FIFO_SIZE)
			// data to send is more then FIFO size.
			// Configure the FIFO int to be mid of FIFO.
			iowrite8(NPCM_SMBTXF_CTL_THR_TXIE |
				(SMBUS_FIFO_SIZE), //  / 2),
				bus->reg + NPCM_SMBTXF_CTL);
		else
			iowrite8(NPCM_SMBTXF_CTL_THR_TXIE |
				nwrite, //  / 2),
				bus->reg + NPCM_SMBTXF_CTL);

		npcm_smb_clear_tx_fifo(bus);
	}

}

static void npcm_smb_read_from_fifo(struct npcm_i2c *bus, u8 bytes_in_fifo)
{
	u8 data;

	while (bytes_in_fifo--) {
		npcm_smb_rd_byte(bus, &data);

		if (bus->master_or_slave == SMB_MASTER) {
			if (bus->rd_ind < bus->rd_size) {
				bus->rd_buf[bus->rd_ind++] = data;
				if (bus->rd_ind == 1 && bus->read_block_use){

					// First byte indicates length in block protocol
					bus->rd_size = data;
					pdebug(bus, "blk rcv");
				}
			}
		} else { // SMB_SLAVE:
#if IS_ENABLED(CONFIG_I2C_SLAVE)		
			if (bus->slv_rd_ind < bus->slv_rd_size) {
				bus->slv_rd_buf[bus->slv_rd_ind++] = data;
				if (bus->slv_rd_ind == 1 && bus->read_block_use)
					// First byte indicates length in block protocol
					bus->rd_size = data;
			}
#endif			
		}
	}
}


static int npcm_smb_master_abort(struct npcm_i2c *bus)
{
	int ret = 0;
	u8 data;
	int len;

	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_ABORT);

	// Only current master is allowed to issue Stop Condition
	if (npcm_smb_is_master(bus)) {
		// stoping in the middle, not waing for interrupts anymore
		npcm_smb_eob_int(bus,  false);

		// Generate a STOP condition (after next wr\rd from fifo:
		npcm_smb_master_stop(bus);
		if (bus->operation == SMB_WRITE_OPER){
			npcm_smb_set_fifo(bus, 0, 1);
			// dummy write to FIFO:
			npcm_smb_wr_byte(bus, 0xFF);
			pdebug(bus, " abort data wr");
		}
		else {
			// gracefully abort read transaction
			len = npcm_smb_get_fifo_fullness(bus);

			if (len > 0)
				npcm_smb_read_from_fifo(bus, len);
			npcm_smb_set_fifo(bus, 1, 0);
			npcm_smb_rd_byte(bus, &data);
			pdebug(bus, " abort data rd");
		}

		udelay(100); // TODO, replace with TO polling on BB bit.

		// Clear NEGACK, STASTR and BER bits
		iowrite8(0xFF, bus->reg + NPCM_SMBST);
	}

	pdebug(bus, " abort data 3 ");

	npcm_smb_reset(bus);

	return ret;
}


#if IS_ENABLED(CONFIG_I2C_SLAVE)

static irqreturn_t npcm_i2c_bus_irq(int irq, void *dev_id);


static int  npcm_smb_slave_enable_l(struct npcm_i2c *bus,
			enum smb_addr addr_type, u8 addr, bool enable)
{
	u8 SmbAddrX_Addr = FIELD_PREP(NPCM_SMBADDR_ADDR, addr) |
		FIELD_PREP(NPCM_SMBADDR_SAEN, enable);


	pdebug_lvl2(bus, " slave enable ");
	if (addr_type == SMB_GC_ADDR) {
		iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) &
			~NPCM_SMBCTL1_GCMEN) |
			FIELD_PREP(NPCM_SMBCTL1_GCMEN, enable),
			bus->reg + NPCM_SMBCTL1);
		return 0;
	}
	if (addr_type == SMB_ARP_ADDR) {

		iowrite8((ioread8(bus->reg + NPCM_SMBCTL3) &
			~SMBCTL3_ARPMEN) |
			FIELD_PREP(SMBCTL3_ARPMEN, enable),
			bus->reg + NPCM_SMBCTL3);
		return 0;
	}
	if (addr_type >= SMB_NUM_OF_ADDR)
		return -EFAULT;

	// Disable ints and select bank 0 for address 3 to ...
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// Set and enable the address
	iowrite8(SmbAddrX_Addr, bus->reg + NPCM_SMBADDR[(int)addr_type]);

	// return to bank 1 and enable ints (if needed)
	npcm_smb_select_bank(bus, SMB_BANK_1);

	pdebug(bus, " slave enable done ");

	return 0;
}

static u8 npcm_smb_get_slave_addr_l(struct npcm_i2c *bus,
				       enum smb_addr addr_type)
{
	unsigned long flags;
	u8 slaveAddress;

	// disable ints and select bank 0 for address 3 to ...
	spin_lock_irqsave(&bus->lock, flags);
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// printk("I2C%d get addr %d", bus->num, (int)addr_type);

	slaveAddress = ioread8(bus->reg + NPCM_SMBADDR[(int)addr_type]);

	// return to bank 1 and enable ints (if needed)
	npcm_smb_select_bank(bus, SMB_BANK_1);
	return  slaveAddress;
}

#if defined(TODO_MULTI_SA) // poleg support up to 10 SA.
static bool npcm_smb_is_slave_addr_exist(struct npcm_i2c *bus, u8 addr)
{
	int i;

	addr |= 0x80; //Set the enable bit

	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++)
		if (addr == npcm_smb_get_slave_addr_l(bus, (enum smb_addr)i))
			return true;

	return false;
}

static int  npcm_smb_add_slave_addr(struct npcm_i2c *bus,
				       u8 slaveAddrToAssign, bool use_PEC)
{
	u16 i;
	int ret = -EFAULT;
	// printk("slaveAddrToAssign = %02X\n", slaveAddrToAssign);

	slaveAddrToAssign |= 0x80; //set the enable bit

	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++) {
		u8 currentSlaveAddr = npcm_smb_get_slave_addr_l(bus,
							(enum smb_addr)i);
		if (currentSlaveAddr == slaveAddrToAssign) {
			ret = 0;
			break;
		}
		else if ((currentSlaveAddr & 0x7F) == 0) {
			ret = npcm_smb_slave_enable_l(bus,
				(enum smb_addr)i, slaveAddrToAssign, true);
			break;
		}
	}

	if (ret == 0) {
		if (use_PEC)
			bus->PEC_mask |= 1 << i;
		else
			bus->PEC_mask &= ~(1 << i);
	}
	return ret;
}

static int  npcm_smb_get_current_slave_addr(struct npcm_i2c *bus,
					       u8 *currSlaveAddr)
{
	if (currSlaveAddr != NULL) {
		*currSlaveAddr = bus->own_slave_addr;
		return 0;
	}

	return -EFAULT;
}
#endif //TODO_MULTI_SA

static int  npcm_smb_remove_slave_addr(struct npcm_i2c *bus,
					  u8 slaveAddrToRemove)
{
	int i;
	unsigned long flags;

	slaveAddrToRemove |= 0x80; //Set the enable bit

	// disable ints and select bank 0 for address 3 to ...
	spin_lock_irqsave(&bus->lock, flags);
	npcm_smb_select_bank(bus, SMB_BANK_0);

	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++) {
		if (ioread8(bus->reg + NPCM_SMBADDR[i]) == slaveAddrToRemove)
			iowrite8(0, bus->reg + NPCM_SMBADDR[i]);
	}

	// return to bank 1 and enable ints (if needed)
	npcm_smb_select_bank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->lock, flags);

	return 0;
}

#if defined(TODO_MULTI_SA)  // poleg support up to 10 SA.
static int  npcm_smb_slave_global_call_enable(struct npcm_i2c *bus,
						 bool enable)
{
	return npcm_smb_slave_enable_l(bus, SMB_GC_ADDR, 0, enable);
}

static int  npcm_smb_slave_ARP_enable(struct npcm_i2c *bus, bool enable)
{
	return npcm_smb_slave_enable_l(bus, SMB_ARP_ADDR, 0, enable);
}
#endif // TODO_MULTI_SA

static bool npcm_smb_slave_start_receive(struct npcm_i2c *bus, u16 nread,
					    u8 *read_data)
{

	// Allow only if bus is not busy
	if ((bus->state != SMB_SLAVE_MATCH) || nread > 32*1024)
		return false;

	// Update driver state
	bus->state = SMB_OPER_STARTED;
	bus->operation	 = SMB_READ_OPER;
	//bus->slv_rd_buf   = read_data;
	bus->slv_rd_size	 = nread;
	bus->slv_rd_ind	= 0;
	bus->slv_wr_size	= 0;
	bus->slv_wr_ind	= 0;

	npcm_smb_set_fifo(bus, min(nread, bus->threshold_fifo), -1);

	// triggers new data reception
	iowrite8(NPCM_SMBST_NMATCH, bus->reg + NPCM_SMBST);

	return true;
}

static bool npcm_smb_slave_start_xmit(struct npcm_i2c *bus, u16 nwrite,
					 u8 *write_data)
{

	// Allow only if bus is not busy
	if ((bus->state != SMB_SLAVE_MATCH) || (nwrite == 0))
		return false;


	// Update driver state
	if (bus->PEC_use)
		nwrite++;

	bus->state = SMB_OPER_STARTED;
	bus->operation	 = SMB_WRITE_OPER;
	bus->slv_wr_size	= nwrite;
	bus->slv_wr_ind	= 0;

	if (bus->fifo_use == true) {
		// triggers new data reception
		iowrite8(NPCM_SMBST_NMATCH, bus->reg + NPCM_SMBST);

		if (nwrite > 0) {

			npcm_smb_set_fifo(bus, -1, min((u16)bus->threshold_fifo, nwrite));

			// Fill the FIFO with data
			npcm_smb_write_to_fifo_slave(bus, min((u16)bus->threshold_fifo, nwrite));
		}
	}
	else // bus->fifo_use == FALSE
		npcm_smb_wr_byte(bus, bus->slv_wr_buf[bus->slv_wr_ind++]);

	return true;
}


static int npcm_smb_int_slave_handler(struct npcm_i2c *bus)
{
	enum smb_state_ind ind;

	pdebug_lvl2(bus, "int_slave_handler ");

	// Slave: A negative acknowledge has occurred
	if (FIELD_GET(NPCM_SMBST_NEGACK , ioread8(bus->reg + NPCM_SMBST))) {
		pdebug_lvl2(bus, "NACK slave");
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_NACK);
		if (bus->fifo_use) {

			// if there are still untransmitted bytes in TX FIFO reduce them from slv_wr_ind
			bus->slv_wr_ind -= FIELD_GET(NPCM_SMBTXF_STS_TX_BYTES,
						      ioread8(bus->reg + NPCM_SMBTXF_STS));

			if (bus->operation == SMB_WRITE_OPER)
				bus->slv_wr_ind -= npcm_smb_get_fifo_fullness(bus);
			// clear the FIFO
			iowrite8(NPCM_SMBFIF_CTS_CLR_FIFO,
				 bus->reg + NPCM_SMBFIF_CTS);
		}

		// In slave write operation, NACK is OK, otherwise it is a problem
		if (!(	(bus->slv_wr_ind != 0) && (bus->slv_wr_ind == bus->slv_wr_size)))
			// Either not slave, or number of bytes sent to master less than required
			// In either case notify upper layer. If we are slave - the upper layer
			// should still wait for a Slave Stop.
		{


			// iowrite8(NPCM_SMBST_NEGACK, bus->reg + NPCM_SMBST);
			bus->state = SMB_IDLE;
			npcm_smb_callback(bus, SMB_NACK_IND, bus->slv_wr_ind);
		}


		// Slave has to wait for SMB_STOP to decide this is the end of the transaction.
		// Therefore transaction is not yet considered as done
		iowrite8(NPCM_SMBST_NEGACK, bus->reg + NPCM_SMBST);

		return 0;
	}

#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)

	// Slave: A Bus Timeout has been identified
	if ((FIELD_GET(NPCM_SMBT_OUT_T_OUTIE, ioread8(bus->reg + NPCM_SMBT_OUT) ) == 1) &&  // bus timeout int is on
	    (FIELD_GET(NPCM_SMBT_OUT_T_OUTST, ioread8(bus->reg + NPCM_SMBT_OUT) ))) {         // and bus timeout status is set
		pdebug(bus, "TO slave ");
		// Reset the module
		npcm_smb_reset(bus);
		iowrite8(ioread8(bus->reg + NPCM_SMBT_OUT) | NPCM_SMBT_OUT_T_OUTST, bus->reg + NPCM_SMBT_OUT);// Clear EO_BUSY pending bit
		bus->state = SMB_IDLE;
		npcm_smb_callback(bus, SMB_BUS_ERR_IND, npcm_smb_get_index(bus));
		return 0;
	}
#endif

	// A Slave End of Busy (meaning Stop Condition happened)
	if ((FIELD_GET(NPCM_SMBCTL1_EOBINTE,
		       ioread8(bus->reg + NPCM_SMBCTL1)) == 1) &&
	    (FIELD_GET(NPCM_SMBCST3_EO_BUSY,
		       ioread8(bus->reg + NPCM_SMBCST3)))) {
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_EOB);
		pdebug_lvl2(bus, "slave EOB");

		npcm_smb_eob_int(bus, false);

		bus->state = SMB_IDLE;
		npcm_smb_callback(bus, bus->stop_ind, bus->slv_rd_ind);
		return 0;
	}

	// A Slave Stop Condition has been identified
	if (FIELD_GET(NPCM_SMBST_SLVSTP , ioread8(bus->reg + NPCM_SMBST))) {
		enum smb_state_ind ind;

		pdebug_lvl2(bus, "slave stop ");
		if (bus->master_or_slave != SMB_SLAVE)
			pdebug(bus, "slave error ");
		iowrite8(NPCM_SMBST_SLVSTP, bus->reg + NPCM_SMBST);


		// Check whether bus arbitration or Start or Stop during data xfer
		bus->state = SMB_IDLE;
		if (bus->fifo_use) {
			if (bus->operation == SMB_READ_OPER) {
				npcm_smb_read_from_fifo(bus,
							   FIELD_GET(NPCM_SMBRXF_STS_RX_BYTES,
							   ioread8(bus->reg + NPCM_SMBRXF_STS)));

				// Be prepared for new transactions
				//bus->state = SMB_IDLE;

				// if PEC is not used or PEC is used and PEC is correct
				if ((bus->PEC_use == false) ||
				    (npcm_smb_get_PEC(bus) == 0)){
					ind = SMB_SLAVE_DONE_IND;
				}

				// PEC value is not correct
				else {
					ind = SMB_SLAVE_PEC_ERR_IND;
				}
				npcm_smb_callback(bus, ind, bus->slv_rd_ind);
			}
			if (bus->operation == SMB_WRITE_OPER) {
				//bus->state = SMB_IDLE;
				npcm_smb_callback(bus, SMB_SLAVE_DONE_IND,
						     bus->slv_wr_ind);
			}

			iowrite8(NPCM_SMBFIF_CTS_SLVRSTR |
				 NPCM_SMBFIF_CTS_CLR_FIFO |
				 NPCM_SMBFIF_CTS_RXF_TXE, bus->reg + NPCM_SMBFIF_CTS);
		}

		// FIFO is not used (slave)
		else {
			if (bus->operation == SMB_READ_OPER) {

				// if PEC is not used or PEC is used and PEC is correct
				if ((bus->PEC_use == false) ||
				    (npcm_smb_get_PEC(bus) == 0))
				    ind = SMB_SLAVE_DONE_IND;

				// PEC value is not correct
				else
					ind = SMB_SLAVE_PEC_ERR_IND;

				npcm_smb_callback(bus, ind, bus->slv_rd_ind);
			} else
				//bus->state = SMB_IDLE;
				npcm_smb_callback(bus, SMB_SLAVE_DONE_IND,
				bus->slv_wr_ind);
		}

		return 0;
	}

	// A Slave restart Condition has been identified
	if (bus->fifo_use && FIELD_GET(NPCM_SMBFIF_CTS_SLVRSTR,
				       ioread8(bus->reg + NPCM_SMBFIF_CTS))) {
		pdebug_lvl2(bus, "slave restart ");
		if (bus->master_or_slave != SMB_SLAVE)
			pdebug(bus, "slave restart error ");

		if (bus->operation == SMB_READ_OPER)
			npcm_smb_read_from_fifo(bus,
						FIELD_GET(NPCM_SMBRXF_STS_RX_BYTES,
						ioread8(bus->reg + NPCM_SMBRXF_STS)));

		iowrite8(NPCM_SMBFIF_CTS_SLVRSTR, bus->reg + NPCM_SMBFIF_CTS);
	}

	// A Slave Address Match has been identified
	if (FIELD_GET(NPCM_SMBST_NMATCH , ioread8(bus->reg + NPCM_SMBST))) {
		bool slave_tx;
		enum smb_state_ind ind = SMB_NO_STATUS_IND;
		u8 info = 0;

		pdebug_lvl2(bus, "slave match ");

		if (bus->fifo_use == false)
			iowrite8(NPCM_SMBST_NMATCH, bus->reg + NPCM_SMBST);

		if (FIELD_GET(NPCM_SMBST_XMIT, ioread8(bus->reg + NPCM_SMBST))) {
			slave_tx = true;
			ind = SMB_SLAVE_XMIT_IND;
		} else {
			slave_tx = false;
			if (bus->state == SMB_IDLE)
				i2c_slave_event(bus->slave, I2C_SLAVE_WRITE_REQUESTED, &info);
			ind = SMB_SLAVE_RCV_IND;
		}

		if (bus->state == SMB_IDLE) {
			// Check which type of address match
			if (FIELD_GET(NPCM_SMBCST_MATCH , ioread8(bus->reg + NPCM_SMBCST))) {
				u16 address_match = ((ioread8(bus->reg + NPCM_SMBCST3) & 0x7) << 7) | (ioread8(bus->reg + NPCM_SMBCST2) & 0x7F);
				info = 0;

				while (address_match) {
					if (address_match & 1)
						break;
					info++;
					address_match = address_match >> 1;
				}
				bus->own_slave_addr = FIELD_GET(NPCM_SMBADDR_ADDR, npcm_smb_get_slave_addr_l(bus, (enum smb_addr)info));
				if (bus->PEC_mask & BIT(info)) {
					bus->PEC_use = true;
					bus->crc_data = 0;
					if (slave_tx)
						npcm_smb_calc_PEC(bus, (bus->own_slave_addr & 0x7F) << 1 | 1);
					else
						npcm_smb_calc_PEC(bus, (bus->own_slave_addr & 0x7F) << 1);
				} else
					bus->PEC_use = false;
			} else {
				if (FIELD_GET(NPCM_SMBCST_GCMATCH , ioread8(bus->reg + NPCM_SMBCST))) {
					info = (u8)SMB_GC_ADDR;
					bus->own_slave_addr = 0;
				} else {
					if (FIELD_GET(NPCM_SMBCST_ARPMATCH , ioread8(bus->reg + NPCM_SMBCST))) {
						info = (u8)SMB_ARP_ADDR;
						bus->own_slave_addr = 0x61;
					}
				}
				// printk("slave match addr 0x%x", bus->own_slave_addr);
			}
		} else {
			//  Slave match can happen in two options:
			//  1. Start, SA, read	( slave read without further ado).
			//  2. Start, SA, read , data , restart, SA, read,  ... ( slave read in fragmented mode)
			//  3. Start, SA, write, data, restart, SA, read, .. ( regular write-read mode)
			if ((bus->state == SMB_OPER_STARTED &&
				bus->operation == SMB_READ_OPER && slave_tx) ||
				(!slave_tx)){
				// slave transmit after slave receive w/o Slave Stop implies repeated start
				ind = SMB_SLAVE_RESTART_IND;
				info = (u8)(bus->slv_rd_ind);
				npcm_smb_calc_PEC(bus, (bus->own_slave_addr & 0x7F) << 1 | 1);
				pdebug_lvl2(bus, "slave Sr ");
			}
		}

		// Address match automatically implies slave mode
		bus->master_or_slave = SMB_SLAVE;
		bus->state = SMB_SLAVE_MATCH;

		// Notify upper layer
		// Upper layer must at this stage call the driver routine for slave tx or rx,
		// to eliminate a condition of slave being notified but not yet starting
		// transaction - and thus an endless int from SDAST for the slave RCV or TX !
		npcm_smb_callback(bus, ind, info);

#ifdef SMB_RECOVERY_SUPPORT

		// By now, SMB operation state should have been changed from MATCH to SMB_OPER_STARTED.
		// If state hasn't been changed already, this may suggest that the SMB slave is not ready to
		// transmit or receive data.
		//
		// In addition, when using FIFO, NMATCH bit is cleared only when moving to SMB_OPER_STARTED state.
		// If NMATCH is not cleared, we would get an endless SMB int.
		// Therefore, Abort the slave, such that SMB HW and state machine return to a default, functional
		// state.
		if (bus->state == SMB_SLAVE_MATCH) {
			npcm_smb_slave_abort(bus);
			npcm_smb_callback(bus, SMB_BUS_ERR_IND, npcm_smb_get_index(bus));
			return 0;
		}

		// Slave abort data
		// if the SMBus's status is not match current status reg of XMIT
		// the Slave device will enter dead-lock and stall bus forever
		// Add this check rule to avoid this condition
		if ((bus->operation == SMB_READ_OPER  &&
			ind == SMB_SLAVE_XMIT_IND) ||
			(bus->operation == SMB_WRITE_OPER
			&& ind == SMB_SLAVE_RCV_IND)) {
			npcm_smb_slave_abort(bus);

			npcm_smb_callback(bus, SMB_BUS_ERR_IND, npcm_smb_get_index(bus));
			return 0;
		}
#endif
		return 0;

		// If none of the above - BER should occur
	}



	// Slave SDA status is set - transmit or receive, slave
	if (FIELD_GET(NPCM_SMBST_SDAST, ioread8(bus->reg + NPCM_SMBST)) ||
	    (bus->fifo_use   &&
	    (npcm_smb_tx_fifo_empty(bus) || npcm_smb_rx_fifo_full(bus)))) {
		// Status Bit is cleared by writing to or reading from SDA
		// (depending on current direction)

		pdebug_lvl2(bus, "SDA slave set");

		// SDA status is set - transmit or receive: Handle slave mode

		// Perform slave read. No need to distinguish between last byte and the rest of the bytes.
		if ((bus->operation == SMB_READ_OPER)) {
			if (bus->fifo_use == false) {
				u8 data;

				(void)npcm_smb_rd_byte(bus, &data);
				if (bus->slv_rd_ind < bus->slv_rd_size) {
					// Keep read data
					bus->slv_rd_buf[bus->slv_rd_ind++] = data;
					if ((bus->slv_rd_ind == 1) && bus->read_block_use)
						// First byte indicates length in block protocol
						bus->slv_rd_size = data;

#ifdef SMB_WRAP_AROUND_BUFFER
					if (bus->slv_rd_ind == bus->slv_rd_size) {
						// Reset state for the remaining bytes transaction
						bus->state = SMB_SLAVE_MATCH;

						// Notify upper layer of that a byte had received
						npcm_smb_callback(bus,
								     SMB_SLAVE_RCV_MISSING_DATA_IND,
								     npcm_smb_get_index(bus));
					}
#endif
				}
			}
			// FIFO is used
			else {
				if (npcm_smb_rx_fifo_full(bus)) {
					npcm_smb_read_from_fifo(bus,
								   FIELD_GET(NPCM_SMBRXF_CTL_RX_THR,
								   ioread8(bus->reg + NPCM_SMBRXF_CTL)));

					npcm_smb_clear_rx_fifo(bus);
				}
			}
		}
		// Perform slave write.
		else {
			// More bytes to write
			if ((bus->operation == SMB_WRITE_OPER) && (bus->slv_wr_ind < bus->slv_wr_size)) {
				if (bus->fifo_use == false) {
					if (bus->slv_wr_ind + 1 == bus->slv_wr_size)
						npcm_smb_write_PEC(bus);
					else if (bus->slv_wr_ind < bus->slv_wr_size)
						npcm_smb_wr_byte(bus, bus->slv_wr_buf[bus->slv_wr_ind]);
					bus->slv_wr_ind++;
				}
				// FIFO is used
				else {
					u16 wcount;
					wcount =  (bus->slv_wr_size - bus->slv_wr_ind);

					// TODO: replace :
					if (wcount >= SMBUS_FIFO_SIZE)
						wcount = SMBUS_FIFO_SIZE;

					iowrite8((u8)wcount, bus->reg + NPCM_SMBTXF_CTL);

					// with:
					npcm_smb_set_fifo(bus, -1, wcount);
					npcm_smb_write_to_fifo_slave(bus, wcount);

					npcm_smb_clear_tx_fifo(bus);
				}
			}

			// If all bytes were written, ignore further master read requests.
			else {
				if (bus->fifo_use == false) {
					// Clear SDA Status bit
					if (bus->slv_wr_ind != 0){
						// Was writing
#ifdef SMB_WRAP_AROUND_BUFFER
						// We're out of bytes. Ask the higher level for more bytes. Let it know that driver used all its' bytes

						// Reset state for the remaining bytes transaction
						bus->state = SMB_SLAVE_MATCH;

						// Notify upper layer of transaction completion
						// by overrind ind and info_p in next EXECUTE_FUNC()
						ind = SMB_SLAVE_XMIT_MISSING_DATA_IND;
						//  TBD : info_p = &(bus->slv_wr_ind);
#else
						npcm_smb_wr_byte(bus, 0xFF);
#endif
					} else {
						u8 data;
						// Was reading
						(void)npcm_smb_rd_byte(bus, &data);
					}
				}
				// write\read redundant bytes with FIFO (if there are any bytes to write)
				else {
					// Set threshold size
					iowrite8((u8)SMBUS_FIFO_SIZE, bus->reg + NPCM_SMBTXF_CTL);

					// TODO: replace with:
					// npcm_smb_set_fifo(bus, -1, wcount);

					npcm_smb_write_to_fifo_slave(bus, SMBUS_FIFO_SIZE);

					npcm_smb_clear_tx_fifo(bus);
				}
				// Notify upper layer of transaction completion
				npcm_smb_callback(bus, SMB_NO_DATA_IND, bus->slv_rd_ind);
			} // All bytes sent/received
		}
		return 0;
	} //SDAST
	return 1;
}

static void npcm_smb_slave_abort(struct npcm_i2c *bus)
{
	volatile u8 temp;

	// Disable int.
	npcm_smb_int_enable(bus, false);

	// Dummy read to clear interface.
	temp = ioread8(bus->reg + NPCM_SMBSDA);

	// Clear NMATCH and BER bits by writing 1s to them.
	iowrite8(ioread8(bus->reg + NPCM_SMBST) | NPCM_SMBST_BER
		 | NPCM_SMBST_NMATCH,
		 bus->reg + NPCM_SMBST);


	// Reset driver status
	bus->state = SMB_IDLE;

	// Disable SMB Module
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL2)	& ~SMBCTL2_ENABLE),
			bus->reg + NPCM_SMBCTL2);

	// Delay 100 us
	udelay(100); // TBD must be out of int

	// Enable SMB Module
	npcm_smb_enable(bus);

	// Enable int.
	npcm_smb_int_enable(bus, true);
}


// currently slave IF only supports single byte operations.
// in order to utilyze the npcm HW FIFO, the driver will ask for 16 bytes
// at a time, pack them in buffer, and then transmit them all together
// to the FIFO and onward to the bus .
// NACK on read will be once reached to bus->adap->quirks->max_read_len
// sending a NACK wherever the backend requests for it is not supported.
// the next two functions allow reading to local buffer before writung it all
// to the HW FIFO.
// ret val: number of bytes read form the IF:

static int npcm_i2c_slave_get_wr_buf(struct npcm_i2c *bus)
{
	int max_length = 0;
	u8 value = 0;
	int ret = bus->slv_wr_ind;
	int i;

	max_length = bus->adap.quirks->max_write_len;

	if (bus->slv_wr_size == SMBUS_FIFO_SIZE) {
		//printk("fifo full, ind=%d", bus->slv_wr_ind);
		return 0; // you can't fill a cup which is already full
	}

	for (i = bus->slv_wr_ind; i < min(max_length, SMBUS_FIFO_SIZE); i++){
		i2c_slave_event(bus->slave, I2C_SLAVE_READ_REQUESTED, &value);
		bus->slv_wr_buf[i] = value;
		bus->slv_wr_size++;
		i2c_slave_event(bus->slave, I2C_SLAVE_READ_PROCESSED, &value);
	}

	bus->slv_wr_ind = 0;

	// printk("I2C%d get wr buf %d bytes slv_wr_size\n\t  %x %x %x %x   %x %x %x %x    %x %x %x %x    %x %x %x %x\n",
	//bus->num, bus->slv_wr_size,
	//bus->slv_wr_buf[0], bus->slv_wr_buf[1], bus->slv_wr_buf[2], bus->slv_wr_buf[3],
	//bus->slv_wr_buf[4], bus->slv_wr_buf[5], bus->slv_wr_buf[6], bus->slv_wr_buf[7],
	//bus->slv_wr_buf[8], bus->slv_wr_buf[9], bus->slv_wr_buf[10], bus->slv_wr_buf[11],
	//bus->slv_wr_buf[12], bus->slv_wr_buf[13], bus->slv_wr_buf[14], bus->slv_wr_buf[15]);

	return SMBUS_FIFO_SIZE - ret;
}


static void npcm_i2c_slave_send_rd_buf(struct npcm_i2c *bus)
{
	int i;

	for (i = 0; i < bus->slv_rd_ind; i++){
		// printk("->   send 0x%x\n", bus->slv_rd_buf[i]);
		i2c_slave_event(bus->slave, I2C_SLAVE_WRITE_RECEIVED, &bus->slv_rd_buf[i]);
	}

	bus->slv_rd_ind = 0;
	bus->slv_rd_size = 32*1024;

	// printk("I2C%d npcm_i2c_slave_send_rd_buf %d bytes\n", bus->num, i);

}

// handle un processed bytes
static void npcm_i2c_slave_restart_or_stop_flush_wr_buf(struct npcm_i2c *bus)
{
	int i;

	if (bus->operation != SMB_WRITE_OPER) {
		// forget those bytes, master changed address:
		// printk("npcm_i2c_slave_restart_or_stop_flush_wr_buf: forget");
		bus->slv_wr_ind = 0;
		bus->slv_wr_size = 0;
	} else {
		// shift remaining requested bytes to start of buffer. There they will
		// wait till next read command will come. If write will come can forget those bytes.
		bus->slv_wr_size = SMBUS_FIFO_SIZE - bus->slv_wr_ind;
		for (i = bus->slv_wr_ind; i < SMBUS_FIFO_SIZE ; i++){
			bus->slv_wr_buf[i - bus->slv_wr_ind] = bus->slv_wr_buf[i];
		}

		bus->slv_wr_ind = SMBUS_FIFO_SIZE - bus->slv_wr_ind;
	}

	npcm_i2c_slave_get_wr_buf(bus);

	// printk("I2C%d npcm_i2c_slave_restart_or_stop_flush_wr_buf %d\n\t  %x %x %x %x   %x %x %x %x    %x %x %x %x    %x %x %x %x\n",
	// bus->num, bus->slv_wr_size,
	// bus->slv_wr_buf[0], bus->slv_wr_buf[1], bus->slv_wr_buf[2], bus->slv_wr_buf[3],
	// bus->slv_wr_buf[4], bus->slv_wr_buf[5], bus->slv_wr_buf[6], bus->slv_wr_buf[7],
	// bus->slv_wr_buf[8], bus->slv_wr_buf[9], bus->slv_wr_buf[10], bus->slv_wr_buf[11],
	// bus->slv_wr_buf[12], bus->slv_wr_buf[13], bus->slv_wr_buf[14], bus->slv_wr_buf[15]);

	bus->operation = SMB_WRITE_OPER;
}





static int  npcm_i2c_reg_slave(struct i2c_client *client)
{
	unsigned long lock_flags;
	struct npcm_i2c *bus = i2c_get_adapdata(client->adapter);

	bus->slave = client;

	if (!bus->slave)	{
		return -EINVAL;
	}

	if (client->flags & I2C_CLIENT_TEN)
		return -EAFNOSUPPORT;

	spin_lock_irqsave(&bus->lock, lock_flags);


	bus->PEC_use = false;
	bus->PEC_mask = 0;
	bus->master_or_slave = SMB_SLAVE;

	// printk("I2C%d register slave SA=0x%x, PEC=%d, BA = 0x%p\n", bus->num,
	//		client->addr, bus->PEC_use, bus->reg);

	//if (0 != npcm_smb_add_slave_addr(bus, client->addr, bus->PEC_use)) {
	//	// printk("I2C%d can't add addr", bus->num);
	//	return -EINVAL;
	//}


	// printk("slave irq = %d\n", bus->irq);


	// Disable ints and select bank 0 for address 3 to ...
	npcm_smb_select_bank(bus, SMB_BANK_0);

	// printk("I2C%d config addr 1 only\n", bus->num);

	// Set and enable the address
	iowrite8(client->addr, bus->reg + NPCM_SMBADDR1);

	// printk("I2C%d config addr 1 done\n", bus->num);

	// return to bank 1 and enable ints (if needed)
	npcm_smb_select_bank(bus, SMB_BANK_1);

	npcm_smb_slave_enable_l(bus, SMB_SLAVE_ADDR1, client->addr, true);

	//npcm_i2c_slave_get_wr_buf(bus);

	//npcm_i2c_slave_get_rd_buf(bus);

	spin_unlock_irqrestore(&bus->lock, lock_flags);

	return 0;
}

static int  npcm_i2c_unreg_slave(struct i2c_client *client)
{
	struct npcm_i2c *bus = client->adapter->algo_data;
	unsigned long lock_flags;

	spin_lock_irqsave(&bus->lock, lock_flags);
	if (!bus->slave) {
		spin_unlock_irqrestore(&bus->lock, lock_flags);
		return -EINVAL;
	}


	// Turn off slave mode.
	npcm_smb_remove_slave_addr(bus, client->addr);

	bus->slave = NULL;
	spin_unlock_irqrestore(&bus->lock, lock_flags);

	return 0;
}
#endif // CONFIG_I2C_SLAVE


static void npcm_smb_master_fifo_read(struct npcm_i2c *bus)
{
	u16 rcount;
	u8 fifo_bytes;
	enum smb_state_ind ind = SMB_MASTER_DONE_IND;

	fifo_bytes = npcm_smb_get_fifo_fullness(bus);

	rcount = bus->rd_size - bus->rd_ind;

	// In order not to change the RX_TRH during transaction (we found that
	// this might be problematic if it takes too much time to read the FIFO)
	//  we read the data in the following way. If the number of bytes to
	// read == FIFO Size + C (where C < FIFO Size)then first read C bytes
	// and in the next int we read rest of the data.
	if (rcount < (2 * SMBUS_FIFO_SIZE) && rcount > SMBUS_FIFO_SIZE)
		fifo_bytes = (u8)(rcount - SMBUS_FIFO_SIZE);


	if (rcount - fifo_bytes == 0) {
		// last bytes are about to be read - end of transaction.
		// Stop should be set before reading last byte.

		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_READ4);


		if (npcm_smb_get_PEC(bus) != 0)
			ind = SMB_MASTER_PEC_ERR_IND;

		bus->state = SMB_STOP_PENDING;
		bus->stop_ind = ind;

		npcm_smb_eob_int(bus, true);
		npcm_smb_master_stop(bus);

		npcm_smb_read_from_fifo(bus, fifo_bytes);

	} else {

		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_READ3);
		npcm_smb_read_from_fifo(bus, fifo_bytes);
		rcount = bus->rd_size - bus->rd_ind;
		npcm_smb_set_fifo(bus, rcount, -1);
	}
}

static void npcm_smb_int_master_handler_write(struct npcm_i2c *bus)
{
	u16 wcount;

	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_WRITE);
	pdebug_lvl2(bus, "handler wr");
	if (bus->fifo_use)
		npcm_smb_clear_tx_fifo(bus); // clear the TX fifo status bit

	// Master write operation - last byte handling
	if (bus->wr_ind == bus->wr_size) {
		pdebug_lvl2(bus, "last wr");
		if (bus->fifo_use && npcm_smb_get_fifo_fullness(bus) > 0)
	// No more bytes to send (to add to the FIFO), however the FIFO is not
	// empty yet. It is still in the middle of tx. Currently there's nothing
	// to do except for waiting to the end of the tx.
	// We will get an int when the FIFO will get empty.
			return;

		if (bus->rd_size == 0) {
			// all bytes have been written, in a pure wr operation
			npcm_smb_eob_int(bus, true);

			bus->state = SMB_STOP_PENDING;
			bus->stop_ind = SMB_MASTER_DONE_IND;

			// Issue a STOP condition on the bus
			npcm_smb_master_stop(bus);
			// Clear SDA Status bit (by writing dummy byte)
			npcm_smb_wr_byte(bus, 0xFF);

		} else {
			// last write-byte written on previous int - need to
			// restart & send slave address
			if (bus->PEC_use && !bus->read_block_use &&
			    !npcm_smb_is_quick(bus))
			    // PEC is used but the protocol is not block read
			    // then we add extra bytes for PEC support
				bus->rd_size += 1;

			if (bus->fifo_use) {
				if (bus->rd_size == 1 || bus->read_block_use) {
					// SMBus Block read transaction.
					iowrite8(0, bus->reg + NPCM_SMBTXF_CTL);
					// set fifo to read one byte:
					iowrite8(1, bus->reg + NPCM_SMBRXF_CTL);
				}
			}

			npcm_smb_set_fifo(bus, bus->rd_size, -1);

			// Generate (Repeated) Start upon next write to SDA
			npcm_smb_master_start(bus);

			if (bus->rd_size == 1)

	// Receiving one byte only - stall after successful completion of send
	// address byte. If we NACK here, and slave doesn't ACK the address, we
	// might unintentionally NACK the next multi-byte read

				npcm_smb_stall_after_start(bus, true);

			// Next int will occur on read
			bus->operation = SMB_READ_OPER;



			// send the slave address in read direction
			npcm_smb_wr_byte(bus, bus->dest_addr | 0x1);

		}
	} else {
		if (bus->PEC_use && !npcm_smb_is_quick(bus))
			// extra bytes for PEC support
			bus->wr_size += 1;

		// write next byte not last byte and not slave address
		if (!bus->fifo_use || bus->wr_size == 1) {
			if (bus->PEC_use && bus->rd_size == 0 &&
			    (bus->wr_ind + 1 == bus->wr_size)) {
				// Master write protocol to send PEC byte.
				npcm_smb_write_PEC(bus);
				bus->wr_ind++;
			} else {
				npcm_smb_wr_byte(bus,
						 bus->wr_buf[bus->wr_ind++]);
			}
		} else { // FIFO is used
			wcount = bus->wr_size - bus->wr_ind;
			npcm_smb_set_fifo(bus, -1, wcount);
			//   WHY NOT: npcm_smb_write_to_fifo_master(bus, wcount);
			npcm_smb_write_to_fifo_master(bus, wcount);
		}
	}
}

static void npcm_smb_int_master_handler_read(struct npcm_i2c *bus)
{
	u16 block_zero_bytes;
	u8 data;

	// Master read operation (pure read or following a write operation).
	NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_READ);
	pdebug_lvl2(bus, "handler rd");

	if (bus->rd_ind > bus->rd_size)
		pdebug(bus, "master rd error");

	// Initialize number of bytes to include only the first byte (presents
	// a case where number of bytes to read is zero); add PEC if applicable
	block_zero_bytes = 1;
	if (bus->PEC_use)
		block_zero_bytes++;

	// Perform master read, distinguishing between last byte and the rest of
	// the bytes. The last byte should be read when the clock is stopped
	if (bus->rd_ind == 0) { //first byte handling:
		// in block protocol first byte is the size
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_READ1);
		if (bus->read_block_use) {
			npcm_smb_rd_byte(bus, &data);

			// First byte indicates length in block protocol
			bus->rd_buf[bus->rd_ind++] = data;
			// first byte in block protocol
			// is zero -> not supported. read at
			// least one byte

			// slave returned illgal size. read up to 32 bytes.
			if (data >= I2C_SMBUS_BLOCK_MAX)
				data = I2C_SMBUS_BLOCK_MAX;

			if (data == 0)
				bus->rd_size = 1;
			else
				bus->rd_size = data + 1;

			if (bus->PEC_use) {
				bus->rd_size += 1;
				data += 1;
			}

			// clear RX FIFO interrupt status:
			if (bus->fifo_use) {
				iowrite8(NPCM_SMBFIF_CTS_RXF_TXE |
					 ioread8(bus->reg +
						 NPCM_SMBFIF_CTS),
					 bus->reg + NPCM_SMBFIF_CTS);
				//npcm_smb_clear_fifo_int(bus);


			}

			// Reset stall
			npcm_smb_stall_after_start(bus, false);

			npcm_smb_set_fifo(bus, (bus->rd_size - 1), -1);
			npcm_smb_master_fifo_read(bus);
		} else {

			npcm_smb_clear_tx_fifo(bus);
			npcm_smb_master_fifo_read(bus);

		}

	} else {

		if (bus->rd_size == block_zero_bytes &&
		    bus->read_block_use) {
		    	bus->state = SMB_STOP_PENDING;
			bus->stop_ind = SMB_BLOCK_BYTES_ERR_IND;
			npcm_smb_eob_int(bus, true);
			npcm_smb_master_stop(bus);
			npcm_smb_read_from_fifo(bus, npcm_smb_get_fifo_fullness(bus));

		} else {
			NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_READ2);
			npcm_smb_master_fifo_read(bus);
		}

	}

}

static void npcm_smb_int_master_handler(struct npcm_i2c *bus)
{
	// A negative acknowledge has occurred
	if (FIELD_GET(NPCM_SMBST_NEGACK, ioread8(bus->reg + NPCM_SMBST))) {
		pdebug_lvl2(bus, "NACK");
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_NACK);
		if (bus->fifo_use) {
			// if there are still untransmitted bytes in TX FIFO
			// reduce them from wr_ind

			if (bus->operation == SMB_WRITE_OPER)
				bus->wr_ind -= npcm_smb_get_fifo_fullness(bus);
			// clear the FIFO
			iowrite8(NPCM_SMBFIF_CTS_CLR_FIFO,
				 bus->reg + NPCM_SMBFIF_CTS);
		}

		// In master write operation, NACK is a problem
		// number of bytes sent to master less than required
		bus->stop_ind = SMB_NACK_IND;
		npcm_smb_master_abort(bus);
		bus->state = SMB_IDLE;

		// In Master mode, NEGACK should be cleared only after
		// generating STOP.
		// In such case, the bus is released from stall only after the
		// software clears NEGACK bit.
		// Then a Stop condition is sent.
		iowrite8(NPCM_SMBST_NEGACK, bus->reg + NPCM_SMBST);
		npcm_smb_callback(bus, SMB_NACK_IND, bus->wr_ind);
		return;
	}

	// Master mode: a Bus Error has been identified
	if (FIELD_GET(NPCM_SMBST_BER, ioread8(bus->reg + NPCM_SMBST))) {
		// Check whether bus arbitration or Start or Stop during data
		// xfer bus arbitration problem should not result in recovery
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_BER);
		if (npcm_smb_is_master(bus)) {
			// Only current master is allowed to issue stop
			bus->stop_ind = SMB_BUS_ERR_IND;
			npcm_smb_master_abort(bus);
		} else {
			// Bus arbitration loss
			if (bus->retry_count-- > 0) {
				// Perform a retry (generate a start condition)
				// as soon as the SMBus is free
				pdebug(bus, "retry-BER");
				iowrite8(NPCM_SMBST_BER, bus->reg + NPCM_SMBST);
				npcm_smb_master_start(bus);
				return;
			}
		}
		iowrite8(NPCM_SMBST_BER, bus->reg + NPCM_SMBST);
		bus->state = SMB_IDLE;
		npcm_smb_callback(bus, SMB_BUS_ERR_IND,
				  npcm_smb_get_index(bus));
		return;
	}

	// A Master End of Busy (meaning Stop Condition happened)
	// End of Busy int is on and End of Busy is set
	if ((FIELD_GET(NPCM_SMBCTL1_EOBINTE,
		       ioread8(bus->reg + NPCM_SMBCTL1)) == 1) &&
	    (FIELD_GET(NPCM_SMBCST3_EO_BUSY,
		       ioread8(bus->reg + NPCM_SMBCST3)))) {
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_EOB);
		pdebug_lvl2(bus, "EOB  ");
		npcm_smb_eob_int(bus, false);
		bus->state = SMB_IDLE;
		if (npcm_smb_is_quick(bus))
			npcm_smb_callback(bus, bus->stop_ind, 0);
		else
			npcm_smb_callback(bus, bus->stop_ind, bus->rd_ind);
		return;
	}

	// Address sent and requested stall occurred (Master mode)
	if (FIELD_GET(NPCM_SMBST_STASTR, ioread8(bus->reg + NPCM_SMBST))) {
		pdebug_lvl2(bus, "stall");
		NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_STALL);

		// Check for Quick Command SMBus protocol
		if (npcm_smb_is_quick(bus)) {
			// Update status
			bus->state = SMB_STOP_PENDING;
			bus->stop_ind = SMB_MASTER_DONE_IND;
			npcm_smb_eob_int(bus, true);
			npcm_smb_master_stop(bus);

		} else if ((bus->rd_size == 1) && !bus->read_block_use){
			// Receiving one byte only - set NACK after ensuring
			// slave ACKed the address byte
			npcm_smb_nack(bus);
		}

		// Reset stall-after-address-byte
		npcm_smb_stall_after_start(bus, false);

		// Clear stall only after setting STOP
		iowrite8(NPCM_SMBST_STASTR, bus->reg + NPCM_SMBST);

		return;
	}

	// SDA status is set - transmit or receive, master
	if (FIELD_GET(NPCM_SMBST_SDAST, ioread8(bus->reg + NPCM_SMBST)) ||
	    (bus->fifo_use &&
	    (npcm_smb_tx_fifo_empty(bus) || npcm_smb_rx_fifo_full(bus)))) {
		// Status Bit is cleared by writing to or reading from SDA
		// (depending on current direction)
		switch (bus->state) {
		// Handle unsuccessful bus mastership
		case SMB_IDLE:
			bus->stop_ind = SMB_WAKE_UP_IND;
			npcm_smb_master_abort(bus);
			return;

		case SMB_MASTER_START:
			if (npcm_smb_is_master(bus)) {
				u8 addr_byte = bus->dest_addr;

				bus->crc_data = 0;
				if (npcm_smb_is_quick(bus)) {
					// Need to stall after successful
					// completion of sending address byte
					npcm_smb_stall_after_start(bus, true);
				} else if (bus->wr_size == 0) {
					// Set direction to Read
					addr_byte |= (u8)0x1;
					bus->operation = SMB_READ_OPER;
				} else {
					bus->operation = SMB_WRITE_OPER;
				}

	// Receiving one byte only - stall after successful completion of
	// sending address byte. If we NACK here, and slave doesn't ACK the
	// address, we might unintentionally NACK the next multi-byte read
				if (bus->wr_size == 0 && bus->rd_size == 1)
					npcm_smb_stall_after_start(bus, true);

				// Write the address to the bus
				bus->state = SMB_OPER_STARTED;
				npcm_smb_wr_byte(bus, addr_byte);
			} else {
				dev_err(bus->dev,
					"SDA, bus%d is not master, wr %d 0x%x...\n",
					bus->num, bus->wr_size,
					bus->wr_buf[0]);
			}
			break;

		// SDA status is set - transmit or receive: Handle master mode
		case SMB_OPER_STARTED:
			if (bus->operation == SMB_WRITE_OPER)
				npcm_smb_int_master_handler_write(bus);
			else if (bus->operation == SMB_READ_OPER)
				npcm_smb_int_master_handler_read(bus);
			else
				pr_err("I2C%d: unknown operation\n", bus->num);
			break;
		default:
			dev_err(bus->dev, "i2c%d master sda err on state machine\n",
				bus->num);
		}
	}
}

static int npcm_smb_recovery(struct i2c_adapter *_adap)
{
	u8   iter = 27;	  // Allow one byte to be sent by the Slave
	u16  timeout;
	bool done = false;
	struct npcm_i2c *bus = container_of(_adap, struct npcm_i2c, adap);

	dev_info(bus->dev, "recovery bus%d\n", bus->num);

	might_sleep();

	// Disable int
	npcm_smb_int_enable(bus, false);

	// Check If the SDA line is active (low)
	if (FIELD_GET(NPCM_SMBCST_TSDA, ioread8(bus->reg + NPCM_SMBCST)) == 0) {
		// Repeat the following sequence until SDA is released
		do {
			// Issue a single SCL cycle
			iowrite8(NPCM_SMBCST_TGSCL, bus->reg + NPCM_SMBCST);
			timeout = ABORT_TIMEOUT;
			while (timeout != 0 &&
			       FIELD_GET(NPCM_SMBCST_TGSCL,
					 ioread8(bus->reg + NPCM_SMBCST) == 0))
				timeout--;

			// If SDA line is inactive (high), stop
			if (FIELD_GET(NPCM_SMBCST_TSDA,
				      ioread8(bus->reg + NPCM_SMBCST)) == 1)
				done = true;
		} while ((!done) && (--iter != 0));

		// If SDA line is released (high)
		if (done) {
			// Clear BB (BUS BUSY) bit
			iowrite8(NPCM_SMBCST_BB, bus->reg + NPCM_SMBCST);

			// Generate a START, to synchronize Master and Slave
			npcm_smb_master_start(bus);

			// Wait until START condition is sent, or timeout
			timeout = ABORT_TIMEOUT;
			while (timeout != 0 && !npcm_smb_is_master(bus))
				timeout--;

			// If START condition was sent
			if (timeout > 0) {
				// Send an address byte in write direction:
				npcm_smb_wr_byte(bus, bus->dest_addr);

				// Generate a STOP condition
				npcm_smb_master_stop(bus);
			}
			return 0;
		}
	}

	npcm_smb_int_enable(bus, true);

	return -(ENOTRECOVERABLE);
}

static bool npcm_smb_init_clk(struct npcm_i2c *bus, enum smb_mode mode,
			      u32 bus_freq)
{
	u32  k1 = 0;
	u32  k2 = 0;
	u8   dbnct = 0;
	u32  sclfrq = 0;
	u8   hldt = 7;
	bool fast_mode = false;
	u32  src_clk_freq; // in KHz

	src_clk_freq = bus->apb_clk / 1000;

	if (bus_freq <= SMBUS_FREQ_100KHZ) {
		sclfrq = src_clk_freq / (bus_freq * 4);

		if (sclfrq < SCLFRQ_MIN || sclfrq > SCLFRQ_MAX)
			return false;

		if (src_clk_freq >= 40000)
			hldt = 17;
		else if (src_clk_freq >= 12500)
			hldt = 15;
		else
			hldt = 7;
	}

	else if (bus_freq == SMBUS_FREQ_400KHZ) {
		sclfrq = 0;
		fast_mode = true;

		if ((mode == SMB_MASTER && src_clk_freq < 7500) ||
		    (mode == SMB_SLAVE && src_clk_freq < 10000))
		  // 400KHZ cannot be supported for master core clock < 7.5 MHZ
		  // or slave core clock < 10 MHZ
			return false;

		// Master or Slave with frequency > 25 MHZ
		if (mode == SMB_MASTER || src_clk_freq > 25000) {
			hldt = (u8)__KERNEL_DIV_ROUND_UP(src_clk_freq * 300,
							 1000000) + 7;
			if (mode == SMB_MASTER) {
				k1 = __KERNEL_DIV_ROUND_UP(src_clk_freq * 1600,
							   1000000);
				k2 = __KERNEL_DIV_ROUND_UP(src_clk_freq * 900,
							   1000000);
				k1 = round_up(k1, 2);
				k2 = round_up(k2 + 1, 2);
				if (k1 < SCLFRQ_MIN || k1 > SCLFRQ_MAX ||
				    k2 < SCLFRQ_MIN || k2 > SCLFRQ_MAX)
					return false;
			}
		} else { // Slave with frequency 10-25 MHZ
			hldt = 7;
			dbnct = 2;
		}
	}

	else if (bus_freq == SMBUS_FREQ_1MHZ) {
		sclfrq = 0;
		fast_mode = true;

		if ((mode == SMB_MASTER && src_clk_freq < 15000) ||
		    (mode == SMB_SLAVE	&& src_clk_freq < 24000))
		// 1MHZ cannot be supported for master core clock < 15 MHZ
		// or slave core clock < 24 MHZ
			return false;

		if (mode == SMB_MASTER) {
			k1 = round_up((__KERNEL_DIV_ROUND_UP(src_clk_freq * 620,
							     1000000)), 2);
			k2 = round_up((__KERNEL_DIV_ROUND_UP(src_clk_freq * 380,
							     1000000) + 1), 2);
			if (k1 < SCLFRQ_MIN || k1 > SCLFRQ_MAX ||
			    k2 < SCLFRQ_MIN || k2 > SCLFRQ_MAX) {
				return false;
			}
		}

		// Master or Slave with frequency > 40 MHZ
		if (mode == SMB_MASTER || src_clk_freq > 40000) {
			// Set HLDT:
			// SDA hold time:  (HLDT-7) * T(CLK) >= 120
			// HLDT = 120/T(CLK) + 7 = 120 * FREQ(CLK) + 7
			hldt = (u8)__KERNEL_DIV_ROUND_UP(src_clk_freq * 120,
							 1000000) + 7;

		// Slave with frequency 24-40 MHZ
		} else {
			hldt = 7;
			dbnct = 2;
		}
	}

	// Frequency larger than 1 MHZ
	else
		return false;

	// After clock parameters calculation update the reg
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL2)
		& ~SMBCTL2_SCLFRQ6_0) | FIELD_PREP(SMBCTL2_SCLFRQ6_0,
		sclfrq & 0x7F), bus->reg + NPCM_SMBCTL2);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL3) & ~SMBCTL3_SCLFRQ8_7) |
		 FIELD_PREP(SMBCTL3_SCLFRQ8_7, (sclfrq >> 7) & 0x3),
		 bus->reg + NPCM_SMBCTL3);

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL3) & ~SMBCTL3_400K_MODE) |
		 FIELD_PREP(SMBCTL3_400K_MODE, fast_mode),
		 bus->reg + NPCM_SMBCTL3);

	// Select Bank 0 to access NPCM_SMBCTL4/NPCM_SMBCTL5
	npcm_smb_select_bank(bus, SMB_BANK_0);

	if (bus_freq >= SMBUS_FREQ_400KHZ) {
		// k1 and k2 are relevant for master mode only
		if (mode == SMB_MASTER) {
			// Set SCL Low/High Time:
			// k1 = 2 * SCLLT7-0 -> Low Time  = k1 / 2
			// k2 = 2 * SCLLT7-0 -> High Time = k2 / 2
			iowrite8((u8)k1 / 2, bus->reg + NPCM_SMBSCLLT);
			iowrite8((u8)k2 / 2, bus->reg + NPCM_SMBSCLHT);
		} else { // DBNCT is relevant for slave mode only
			iowrite8((ioread8(bus->reg + NPCM_SMBCTL5) &
				 ~SMBCTL5_DBNCT) |
				 FIELD_PREP(SMBCTL5_DBNCT, dbnct),
				 bus->reg + NPCM_SMBCTL5);
		}
	}

	iowrite8((ioread8(bus->reg + NPCM_SMBCTL4) & ~SMBCTL4_HLDT)
		 | FIELD_PREP(SMBCTL4_HLDT, hldt), bus->reg + NPCM_SMBCTL4);

	// Return to Bank 1, and stay there by default:
	npcm_smb_select_bank(bus, SMB_BANK_1);

	dev_dbg(bus->dev, "k1 = %d k2 = %d dbnct = %d sclfrq = %d hldt = %d src_clk_freq %d fast_mode %d\n",
		k1, k2, dbnct, sclfrq, hldt, src_clk_freq, fast_mode);

	return true;
}

static bool npcm_smb_init_module(struct npcm_i2c *bus, enum smb_mode mode,
				 u32 bus_freq)
{
	// Check whether module already enabled or frequency is out of bounds
	if ((bus->state != SMB_DISABLE && bus->state != SMB_IDLE) ||
	    bus_freq < SMBUS_FREQ_MIN || bus_freq > SMBUS_FREQ_MAX)
		return false;
	// Configure FIFO disabled mode so slave will not use fifo
	// (maste will set it on if supported)
	bus->threshold_fifo = SMBUS_FIFO_SIZE;

	npcm_smb_select_bank(bus, SMB_BANK_0);
	iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTL) & ~NPCM_SMBFIF_CTL_FIFO_EN,
		 bus->reg + NPCM_SMBFIF_CTL);

	npcm_smb_select_bank(bus, SMB_BANK_1);

	bus->fifo_use = false;

	// Configure SMB module clock frequency
	if (!npcm_smb_init_clk(bus, mode, bus_freq)) {
		pr_err("npcm_smb_init_clk failed\n");
		return false;
	}
	npcm_smb_disable(bus);

	// Enable module (before configuring CTL1)
	npcm_smb_enable(bus);
	bus->state = SMB_IDLE;

	// Enable SMB int and New Address Match int source
	iowrite8((ioread8(bus->reg + NPCM_SMBCTL1) | NPCM_SMBCTL1_NMINTE) &
		 ~NPCM_SMBCTL1_RWS_FIELDS,
		 bus->reg + NPCM_SMBCTL1);

	npcm_smb_int_enable(bus, true);
	return true;
}

static int __npcm_i2c_init(struct npcm_i2c *bus, struct platform_device *pdev)
{
	u32 clk_freq;
	int ret;

	// Initialize the internal data structures
	bus->state = SMB_DISABLE;
	bus->master_or_slave = SMB_SLAVE;
	bus->int_time_stamp = 0;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &clk_freq);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not read bus-frequency property\n");
		clk_freq = 100000;
	}

	ret = npcm_smb_init_module(bus, SMB_MASTER, clk_freq / 1000);
	if (!ret) {
		dev_err(&pdev->dev,
			"npcm_smb_init_module() failed\n");
		return -1;
	}

	crc8_populate_lsb(npcm7xx_crc8, 0x07);
	crc8_populate_msb(npcm7xx_crc8, 0x07);
	return 0;
}

static irqreturn_t npcm_i2c_bus_irq(int irq, void *dev_id)
{
	struct npcm_i2c *bus = dev_id;

	bus->int_cnt++;

	if (bus->master_or_slave == SMB_MASTER)	{
 		bus->int_time_stamp = jiffies;
		npcm_smb_int_master_handler(bus);
		return IRQ_HANDLED;
	}
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	else if (!npcm_smb_int_slave_handler(bus)) {
		return IRQ_HANDLED;
	}
#endif

	dev_err(bus->dev, "int unknown on bus%d\n", bus->num);
	return IRQ_NONE;
}

static bool npcm_smb_master_start_xmit(struct npcm_i2c *bus,
				       u8 slave_addr, u16 nwrite, u16 nread,
				       u8 *write_data, u8 *read_data,
				       bool use_PEC)
{
	u8 smbfif_cts;

	//
	// Allow only if bus is not busy
	//
	if (bus->state != SMB_IDLE) {
		dev_err(bus->dev, "\tbus%d->state != SMB_IDLE\n", bus->num);
		return false;
	}

	// Configure FIFO mode :
	if (FIELD_GET(SMB_VER_FIFO_EN, ioread8(bus->reg + SMB_VER))) {
		bus->fifo_use = true;
		npcm_smb_select_bank(bus, SMB_BANK_0);
		iowrite8(ioread8(bus->reg + NPCM_SMBFIF_CTL) |
			 NPCM_SMBFIF_CTL_FIFO_EN, bus->reg + NPCM_SMBFIF_CTL);
		npcm_smb_select_bank(bus, SMB_BANK_1);
	} else {
		bus->fifo_use = false;
	}

	// Update driver state
	bus->master_or_slave = SMB_MASTER;
	bus->state = SMB_MASTER_START;
	if (nwrite > 0)
		bus->operation = SMB_WRITE_OPER;
	else
		bus->operation = SMB_READ_OPER;

	if (npcm_smb_is_quick(bus))
		bus->operation = SMB_WRITE_OPER; // send the address with W bit.

	bus->dest_addr = (u8)(slave_addr << 1);// Translate 7bit to 8bit format
	bus->wr_buf = write_data;
	bus->wr_size = nwrite;
	bus->wr_ind = 0;
	bus->rd_buf = read_data;
	bus->rd_size = nread;
	bus->rd_ind = 0;
	bus->PEC_use = use_PEC;
	bus->PEC_mask = 0;
	bus->retry_count = SMB_RETRY_MAX_COUNT;

	// clear BER just in case it is set due to a previous transaction
	iowrite8(NPCM_SMBST_BER, bus->reg + NPCM_SMBST);

	// Initiate SMBus master transaction
	// Generate a Start condition on the SMBus
	if (bus->fifo_use) {
		// select bank 1 for FIFO regs
		npcm_smb_select_bank(bus, SMB_BANK_1);

		smbfif_cts = ioread8(bus->reg + NPCM_SMBFIF_CTS);

		// clear FIFO and relevant status bits.
		iowrite8((smbfif_cts & NPCM_SMBFIF_CTS_SLVRSTR) |
			 NPCM_SMBFIF_CTS_CLR_FIFO,
			 bus->reg + NPCM_SMBFIF_CTS);

		// and enable it (if slave was enabled don't disable).

		iowrite8((smbfif_cts & NPCM_SMBFIF_CTS_SLVRSTR) |
			 NPCM_SMBFIF_CTS_RXF_TXE,
			 bus->reg + NPCM_SMBFIF_CTS);

		if (bus->operation == SMB_READ_OPER) {
			//This is a read only operation. Configure the FIFO
			//threshold according to the needed # of bytes to read.
			npcm_smb_set_fifo(bus, nread, -1);
		} else if (bus->operation == SMB_WRITE_OPER) {
			npcm_smb_set_fifo(bus, -1, nwrite);
		}
	}

	bus->int_cnt = 0;
	bus->event_log = 0;
	pdebug_lvl2(bus, "xmit ");


	npcm_smb_master_start(bus);

	return true;
}

static int npcm_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
				int num)
{
	struct npcm_i2c *bus = adap->algo_data;
	struct i2c_msg *msg0, *msg1;
	unsigned long time_left, flags;
	u16 nwrite, nread;
	u8 *write_data, *read_data;
	u8 slave_addr;
	int ret = 0;
	int timeout = bus->adap.timeout;

	spin_lock_irqsave(&bus->lock, flags);
	bus->cmd_err = -EPERM;
	bus->int_cnt = 0;
	bus->stop_ind = SMB_NO_STATUS_IND;
	bus->read_block_use = false;
	bus->event_log_prev = bus->event_log;
	bus->int_time_stamp = 0;

	if(ioread8(bus->reg + NPCM_SMBCST) & NPCM_SMBCST_BB){
		pdebug(bus, " xfer bus busy ");
		npcm_smb_master_abort(bus); // recover the bus
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EAGAIN;
	}


	iowrite8(0xFF, bus->reg + NPCM_SMBST);

	if (num > 2 || num < 1) {
		pr_err("I2C command not supported, num of msgs = %d\n", num);
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	msg0 = &msgs[0];
	slave_addr = msg0->addr;
	if (msg0->flags & I2C_M_RD) { // read
		if (num == 2) {
			pr_err(" num = 2 but first msg is rd instead of wr\n");
			spin_unlock_irqrestore(&bus->lock, flags);
			return -EINVAL;
		}
		nwrite = 0;
		write_data = NULL;
		if (msg0->flags & I2C_M_RECV_LEN) {
			nread = 1;
			bus->read_block_use = true;
		} else {
			nread = msg0->len;
		}
		read_data = msg0->buf;

	} else { // write
		nwrite = msg0->len;
		write_data = msg0->buf;
		nread = 0;
		read_data = NULL;
		if (num == 2) {
			msg1 = &msgs[1];
			if (slave_addr != msg1->addr) {
				pr_err("SA==%02x but msg1->addr == %02x\n",
				       slave_addr, msg1->addr);
				spin_unlock_irqrestore(&bus->lock, flags);
				return -EINVAL;
			}
			if ((msg1->flags & I2C_M_RD) == 0) {
				pr_err("num = 2 but both msg are write.\n");
				spin_unlock_irqrestore(&bus->lock, flags);
				return -EINVAL;
			}
			if (msg1->flags & I2C_M_RECV_LEN) {
				nread = 1;
				bus->read_block_use = true;
			} else {
				nread = msg1->len;
				bus->read_block_use = false;
			}

			read_data = msg1->buf;
		}
	}

	bus->msgs = msgs;
	bus->msgs_num = num;

	if(nread == 0 && nwrite == 0)
		timeout = msecs_to_jiffies(1);
	else if (bus->read_block_use)
		timeout = usecs_to_jiffies((2 + I2C_SMBUS_BLOCK_MAX + nwrite)*1300);
	else
		// resonable assumption which leaves time for clock stretching.
		timeout = usecs_to_jiffies((2 + nread + nwrite)*1300);


	if (nwrite >= 32 * 1024 ||  nread >= 32 * 1024) {
		pr_err("i2c%d buffer too big\n", bus->num);
		return -EINVAL;
	}

	reinit_completion(&bus->cmd_complete);

	if (npcm_smb_master_start_xmit(bus, slave_addr, nwrite, nread,
				       write_data, read_data, 0) == false)
		ret = -(EBUSY);

	if (ret != -(EBUSY)) {
		time_left = wait_for_completion_timeout(&bus->cmd_complete,
							timeout);

		if (time_left == 0 && bus->cmd_err == -EPERM) {
			NPCM_I2C_EVENT_LOG(NPCM_I2C_EVENT_TO);
			pdebug(bus, "xfer TO");

			// timeout while bus is busy:
			npcm_smb_master_abort(bus); // don't stop in the middle.

			// Reset driver status
			bus->state = SMB_IDLE;

			ret = -ETIMEDOUT;
		} else {
			ret = bus->cmd_err;
		}
	} else {
#if defined(CONFIG_I2C_DEBUG_BUS)
		pdebug(bus, "busy");
#endif
	}

	bus->msgs = NULL;
	bus->msgs_num = 0;

	spin_unlock_irqrestore(&bus->lock, flags);

	// If nothing went wrong, return number of messages xferred.
	if (ret >= 0)
		return num;
	else
		return ret;
}

static u32 npcm_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA | I2C_FUNC_SLAVE;
}

static const struct i2c_adapter_quirks npcm_i2c_quirks = {
	.max_read_len = 32768,
	.max_write_len = 32768,
	.max_num_msgs = 2,
	.flags = I2C_AQ_COMB_WRITE_THEN_READ
};


static const struct i2c_algorithm npcm_i2c_algo = {
	.master_xfer = npcm_i2c_master_xfer,
	.functionality = npcm_i2c_functionality,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave	= npcm_i2c_reg_slave,
	.unreg_slave	= npcm_i2c_unreg_slave,
#endif // CONFIG_I2C_SLAVE
};

static struct i2c_bus_recovery_info npcm_i2c_recovery = {
	.recover_bus = npcm_smb_recovery,
	.get_scl = npcm_smb_get_SCL,
	.get_sda = npcm_smb_get_SDA,
};

static int  npcm_i2c_probe_bus(struct platform_device *pdev)
{
	struct npcm_i2c *bus;
	struct i2c_adapter *adap;
	struct resource *res;
	struct clk *i2c_clk;
	int ret;
	int num;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

#ifdef CONFIG_OF
	num = of_alias_get_id(pdev->dev.of_node, "i2c");
	bus->num = num;
	i2c_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c_clk)) {
		pr_err(" I2C probe failed: can't read clk.\n");
		return	-EPROBE_DEFER;
	}
	bus->apb_clk = clk_get_rate(i2c_clk);
	dev_dbg(bus->dev, "I2C APB clock is %d\n", bus->apb_clk);
#endif //  CONFIG_OF

	gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
	if (IS_ERR(gcr_regmap)) {
		pr_err("%s: failed to find nuvoton,npcm750-gcr\n", __func__);
		return IS_ERR(gcr_regmap);
	}
	regmap_write(gcr_regmap, NPCM_I2CSEGCTL, I2CSEGCTL_VAL);
	dev_dbg(bus->dev, "I2C%d: gcr mapped\n", bus->num);

	clk_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-clk");
	if (IS_ERR(clk_regmap)) {
		pr_err("%s: failed to find nuvoton,npcm750-clk\n", __func__);
		return IS_ERR(clk_regmap);
	}
	dev_dbg(bus->dev, "I2C%d: clk mapped\n", bus->num);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev_dbg(bus->dev, "resource: %pR\n", res);
	bus->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR((bus)->reg))
		return PTR_ERR((bus)->reg);
	dev_dbg(bus->dev, "base = %p\n", bus->reg);

	// Initialize the I2C adapter
	spin_lock_init(&bus->lock);
	init_completion(&bus->cmd_complete);

	adap = &bus->adap;
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD | I2C_CLIENT_SLAVE;
	adap->retries = 0;
	adap->timeout = msecs_to_jiffies(25);
	adap->algo = &npcm_i2c_algo;
	adap->quirks = &npcm_i2c_quirks;
	adap->algo_data = bus;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->bus_recovery_info = &npcm_i2c_recovery;
	adap->nr = pdev->id;

	bus->dev = &pdev->dev;

	ret = __npcm_i2c_init(bus, pdev);
	if (ret < 0)
		return ret;

	bus->irq = platform_get_irq(pdev, 0);
	if (bus->irq < 0) {
		pr_err("I2C platform_get_irq error.");
		return -ENODEV;
	}
	dev_dbg(bus->dev, "irq = %d\n", bus->irq);

	ret = devm_request_irq(&pdev->dev, bus->irq, npcm_i2c_bus_irq, 0,
			  dev_name(&pdev->dev), (void *)bus);

	if (ret) {
		dev_err(&pdev->dev, "I2C%d: request_irq fail\n", bus->num);
		return ret;
	}

	i2c_set_adapdata(adap, bus);


	snprintf(bus->adap.name, sizeof(bus->adap.name), "Nuvoton i2c");

	ret = i2c_add_numbered_adapter(&bus->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "I2C%d: i2c_add_numbered_adapter fail\n", bus->num);
		return ret;
	}

	platform_set_drvdata(pdev, bus);
	pr_info("i2c bus %d registered\n", bus->adap.nr);

	return 0;
}

static int  npcm_i2c_remove_bus(struct platform_device *pdev)
{
	unsigned long lock_flags;
	struct npcm_i2c *bus = platform_get_drvdata(pdev);

	spin_lock_irqsave(&bus->lock, lock_flags);
	npcm_smb_disable(bus);
	spin_unlock_irqrestore(&bus->lock, lock_flags);
	i2c_del_adapter(&bus->adap);

	return 0;
}

static const struct of_device_id npcm_i2c_bus_of_table[] = {
	{ .compatible = "nuvoton,npcm750-i2c", },
	{},
};
MODULE_DEVICE_TABLE(of, npcm_i2c_bus_of_table);

static struct platform_driver npcm_i2c_bus_driver = {
	.probe = npcm_i2c_probe_bus,
	.remove = npcm_i2c_remove_bus,
	.driver = {
		.name = "nuvoton-i2c",
		.of_match_table = npcm_i2c_bus_of_table,
	}
};
module_platform_driver(npcm_i2c_bus_driver);

MODULE_AUTHOR("Avi Fishman <avi.fishman@gmail.com>");
MODULE_AUTHOR("Tali Perry <tali.perry@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton I2C Bus Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(I2C_VERSION);
