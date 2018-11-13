/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/vmalloc.h>
#include <linux/regmap.h>

#include <asm/sizes.h>
#include <mtd/mtd-abi.h>

#define NPCM7XX_BUSY_TIMEOUT	15000

/* Flash Interface Unit (FIU) Registers */
#define FIU_DRD_CFG		0x00
#define FIU_DWR_CFG		0x04
#define FIU_UMA_CFG		0x08
#define FIU_UMA_CTS		0x0C
#define FIU_UMA_CMD		0x10
#define FIU_UMA_ADDR		0x14
#define FIU_PRT_CFG  		0x18
#define FIU_UMA_DW0		0x20
#define FIU_UMA_DW1		0x24
#define FIU_UMA_DW2		0x28
#define FIU_UMA_DW3		0x2C
#define FIU_UMA_DR0		0x30
#define FIU_UMA_DR1		0x34
#define FIU_UMA_DR2		0x38
#define FIU_UMA_DR3		0x3C
#define FIU_PRT_CMD0		0x40
#define FIU_PRT_CMD1		0x44
#define FIU_PRT_CMD2		0x48
#define FIU_PRT_CMD3		0x4C
#define FIU_PRT_CMD4		0x50
#define FIU_PRT_CMD5		0x54
#define FIU_PRT_CMD6		0x58
#define FIU_PRT_CMD7		0x5C
#define FIU_PRT_CMD8		0x60
#define FIU_PRT_CMD9		0x64
#define FIU_CFG			0x78
#define FIU_VER			0x7C
#define FIU_MAX_REG_LIMIT	0x80

/*
 *   FIU Direct Read Configuration Register
 */
#define FIU_DRD_CFG_LCK		BIT(31)
#define FIU_DRD_CFG_R_BURST	GENMASK(25, 24)
#define FIU_DRD_CFG_ADDSIZ	GENMASK(17, 16)
#define FIU_DRD_CFG_DBW		GENMASK(13, 12)
#define FIU_DRD_CFG_ACCTYPE	GENMASK(9, 8)
#define FIU_DRD_CFG_RDCMD	GENMASK(7, 0)

/*
 * FIU Direct Write Configuration Register
 */
#define FIU_DWR_CFG_LCK		BIT(31)
#define FIU_DWR_CFG_W_BURST	GENMASK(25, 24)
#define FIU_DWR_CFG_ADDSIZ	GENMASK(17, 16)
#define FIU_DWR_CFG_ABPCK	GENMASK(11, 10)
#define FIU_DWR_CFG_DBPCK	GENMASK(9, 8)
#define FIU_DWR_CFG_WRCMD	GENMASK(7, 0)

/*
 *   FIU UMA Configuration Register
 */
#define  FIU_UMA_CFG_LCK	BIT(31)
#define  FIU_UMA_CFG_CMMLCK	BIT(30)
#define  FIU_UMA_CFG_RDATSIZ	GENMASK(28, 24)
#define  FIU_UMA_CFG_DBSIZ	GENMASK(23, 21)
#define  FIU_UMA_CFG_WDATSIZ	GENMASK(20, 16)
#define  FIU_UMA_CFG_ADDSIZ	GENMASK(13, 11)
#define  FIU_UMA_CFG_CMDSIZ	BIT(10)
#define  FIU_UMA_CFG_RDBPCK	GENMASK(9, 8)
#define  FIU_UMA_CFG_DBPCK	GENMASK(7, 6)
#define  FIU_UMA_CFG_WDBPCK	GENMASK(5, 4)
#define  FIU_UMA_CFG_ADBPCK	GENMASK(3, 2)
#define  FIU_UMA_CFG_CMBPCK	GENMASK(1, 0)

/*
 * FIU UMA Control and Status Register
 */
#define  FIU_UMA_CTS_RDYIE		BIT(25)
#define  FIU_UMA_CTS_RDYST		BIT(24)
#define  FIU_UMA_CTS_SW_CS		BIT(16)
#define  FIU_UMA_CTS_DEV_NUM		GENMASK(9, 8)
#define  FIU_UMA_CTS_EXEC_DONE		BIT(0)

/*
 * FIU UMA Command Register
 */
#define  FIU_UMA_CMD_DUM3           GENMASK(31, 24)
#define  FIU_UMA_CMD_DUM2           GENMASK(23, 16)
#define  FIU_UMA_CMD_DUM1           GENMASK(15, 8)
#define  FIU_UMA_CMD_CMD            GENMASK(7, 0)

/*
 *   FIU UMA Address Register
 */
#define  FIU_UMA_ADDR_UMA_ADDR          GENMASK(31, 0)
#define  FIU_UMA_ADDR_AB3               GENMASK(31, 24)
#define  FIU_UMA_ADDR_AB2               GENMASK(23, 16)
#define  FIU_UMA_ADDR_AB1               GENMASK(15, 8) 
#define  FIU_UMA_ADDR_AB0               GENMASK(7, 0)  

/*
 * FIU UMA Write Data Bytes 0-3 Register
 */
#define  FIU_UMA_DW0_WB3              GENMASK(31, 24)
#define  FIU_UMA_DW0_WB2              GENMASK(23, 16)
#define  FIU_UMA_DW0_WB1              GENMASK(15, 8) 
#define  FIU_UMA_DW0_WB0              GENMASK(7, 0)  

/*
 * FIU UMA Write Data Bytes 4-7 Register
 */
#define  FIU_UMA_DW1_WB7              GENMASK(31, 24)
#define  FIU_UMA_DW1_WB6              GENMASK(23, 16)
#define  FIU_UMA_DW1_WB5              GENMASK(15, 8) 
#define  FIU_UMA_DW1_WB4              GENMASK(7, 0)  

/*
 * FIU UMA Write Data Bytes 8-11 Register
 */
#define  FIU_UMA_DW2_WB11             GENMASK(31, 24)
#define  FIU_UMA_DW2_WB10             GENMASK(23, 16)
#define  FIU_UMA_DW2_WB9              GENMASK(15, 8) 
#define  FIU_UMA_DW2_WB8              GENMASK(7, 0)  

/*
 * FIU UMA Write Data Bytes 12-15 Register
 */
#define  FIU_UMA_DW3_WB15            GENMASK(31, 24)
#define  FIU_UMA_DW3_WB14            GENMASK(23, 16)
#define  FIU_UMA_DW3_WB13            GENMASK(15, 8) 
#define  FIU_UMA_DW3_WB12            GENMASK(7, 0)  

/*
 * FIU UMA Read Data Bytes 0-3 Register
 */
#define  FIU_UMA_DR0_RB3              GENMASK(31, 24)
#define  FIU_UMA_DR0_RB2              GENMASK(23, 16)
#define  FIU_UMA_DR0_RB1              GENMASK(15, 8) 
#define  FIU_UMA_DR0_RB0              GENMASK(7, 0)  

/*
 * FIU UMA Read Data Bytes 4-7 Register
 */
#define  FIU_UMA_DR1_RB15             GENMASK(31, 24)
#define  FIU_UMA_DR1_RB14             GENMASK(23, 16)
#define  FIU_UMA_DR1_RB13             GENMASK(15, 8) 
#define  FIU_UMA_DR1_RB12             GENMASK(7, 0)  

/*
 * FIU UMA Read Data Bytes 8-11 Register
 */
#define  FIU_UMA_DR2_RB15            GENMASK(31, 24)
#define  FIU_UMA_DR2_RB14            GENMASK(23, 16)
#define  FIU_UMA_DR2_RB13            GENMASK(15, 8) 
#define  FIU_UMA_DR2_RB12            GENMASK(7, 0)  

/*
 * FIU UMA Read Data Bytes 12-15 Register
 */
#define  FIU_UMA_DR3_RB15            GENMASK(31, 24)
#define  FIU_UMA_DR3_RB14            GENMASK(23, 16)
#define  FIU_UMA_DR3_RB13            GENMASK(15, 8) 
#define  FIU_UMA_DR3_RB12            GENMASK(7, 0)  

/*
 * FIU Protection Configuration Register
 */
#define  FIU_PRT_CFG_LCK             BIT(31)
#define  FIU_PRT_CFG_PEN             BIT(30)
#define  FIU_PRT_CFG_DEVSIZ          GENMASK(10, 8)
#define  FIU_PRT_CFG_OCALWD          BIT(4)
#define  FIU_PRT_CFG_PRTASIZ         GENMASK(1, 0)

/*
 * FIU Protection Command Register
 */
#define  FIU_PRT_CMD9_ADBPCKB         GENMASK(30, 29)
#define  FIU_PRT_CMD9_CMBPCKB         GENMASK(28, 27)
#define  FIU_PRT_CMD9_ADDSZB          BIT(26)
#define  FIU_PRT_CMD9_FRBDCB          GENMASK(25, 24)
#define  FIU_PRT_CMD9_CMDB            GENMASK(23, 16)
#define  FIU_PRT_CMD9_ADBPCKA         GENMASK(14, 13)
#define  FIU_PRT_CMD9_CMBPCKA         GENMASK(12, 11)
#define  FIU_PRT_CMD9_ADDSZA          BIT(10)
#define  FIU_PRT_CMD9_FRBDCA          GENMASK(9, 8)
#define  FIU_PRT_CMD9_CMDA            GENMASK(7, 0)  

/*
 * FIU Configuration Register
 */
#define  FIU_CFG_SPI_CS_INACT         GENMASK(3, 1)
#define  FIU_CFG_INCRSING             BIT(0)

/*
 * FIU Version Register (FIU_VER) updated
 */
#define  FIU_VER_FIU_VER	GENMASK(7, 0) 

/*
 * Defines
 */
#define FIU_CAPABILITY_QUAD_READ
#define FIU_CAPABILITY_CHIP_SELECT

#define WIN_LIMIT_4K_SHIFT  12
#define BITS_7_0            0xFF
#define BITS_15_8           0xFF00
#define BITS_23_16          0xFF0000


/*
 * Typedef Definitions
 */
typedef enum _spi_w_burst_t {
	FIU_W_BURST_ONE_BYTE        = 0,
	FIU_W_BURST_FOUR_BYTE       = 2,
	FIU_W_BURST_SIXTEEN_BYTE    = 3
} SPI_w_burst_t;

typedef enum _spi_r_burst_t {
	FIU_R_BURST_ONE_BYTE        = 0,
	FIU_R_BURST_FOUR_BYTE       = 2,
	FIU_R_BURST_SIXTEEN_BYTE    = 3
} SPI_r_burst_t;

typedef enum _spi_w_protect_int_t {
	SPI_W_PROTECT_INT_DISABLE = 0,
	SPI_W_PROTECT_INT_ENABLE  = 1
} SPI_w_protect_int_t;

typedef enum _spi_incorect_access_int_t {
	SPI_INCORECT_ACCESS_INT_DISABLE   = 0,
	SPI_INCORECT_ACCESS_INT_ENABLE    = 1
} SPI_incorect_access_int_t;

/*
 * FIU Read Mode
 */
typedef enum _spi_read_mode_t {
	FIU_NORMAL_READ             = 0,
	FIU_FAST_READ               = 1,
	FIU_FAST_READ_DUAL_OUTPUT   = 2,
	FIU_FAST_READ_DUAL_IO       = 3,
	FIU_FAST_READ_QUAD_IO       = 4,
	FIU_FAST_READ_SPI_X         = 5,
	FIU_READ_MODE_NUM
} SPI_read_mode_t;

/*
 * FIU UMA data size
 */
typedef enum _spi_uma_data_size_t {
	FIU_UMA_DATA_SIZE_0         = 0,
	FIU_UMA_DATA_SIZE_1         = 1,
	FIU_UMA_DATA_SIZE_2         = 2,
	FIU_UMA_DATA_SIZE_3         = 3,
	FIU_UMA_DATA_SIZE_4         = 4,
	FIU_UMA_DATA_SIZE_5         = 5,
	FIU_UMA_DATA_SIZE_6         = 6,
	FIU_UMA_DATA_SIZE_7         = 7,
	FIU_UMA_DATA_SIZE_8         = 8,
	FIU_UMA_DATA_SIZE_9         = 9,
	FIU_UMA_DATA_SIZE_10         = 10,
	FIU_UMA_DATA_SIZE_11         = 11,
	FIU_UMA_DATA_SIZE_12         = 12,
	FIU_UMA_DATA_SIZE_13         = 13,
	FIU_UMA_DATA_SIZE_14         = 14,
	FIU_UMA_DATA_SIZE_15         = 15,
	FIU_UMA_DATA_SIZE_16         = 16
} SPI_uma_data_size_t;

/*
 * FIU Field value enumeration
 */
typedef enum _spi_drd_cfg_addsiz_t {
	FIU_DRD_CFG_ADDSIZE_24BIT = 0,
	FIU_DRD_CFG_ADDSIZE_32BIT = 1
}  SPI_drd_cfg_addsiz_t;

typedef enum _spi_trans_status_t {
	FIU_TRANS_STATUS_DONE        = 0,
	FIU_TRANS_STATUS_IN_PROG     = 1
} SPI_trans_status_t;

/*
 * SPI commands
 */
#define NPCM_SPI_RD_STATUS_3_REG_CMD		0x15
#define NPCM_SPI_EN_RST_CMD			0x66
#define NPCM_SPI_RST_DEVICE_CMD			0x99
#define NPCM_SPI_WR_EXT_ADDR_REG_CMD		0xC5

#define DRIVER_NAME "npcm7xx_spinor"

#define SIZE_16MB 0x1000000
#define MAX_READY_WAIT_COUNT	1000000

#define WRITE_TRANSFER_16_BYTES

#ifdef WRITE_TRANSFER_16_BYTES
#define CHUNK_SIZE  16
#endif

#ifdef WRITE_TRANSFER_17_BYTES
#define CHUNK_SIZE  17
#endif

struct npcm7xx_fiu_bus;

struct npcm7xx_chip {
	u32 clkrate;
	u32 chipselect;
	void __iomem *flash_region_mapped_ptr;
	struct spi_nor	nor;
	struct npcm7xx_fiu_bus *host;
};

#define NPCM7XX_MAX_CHIP_NUM		4

struct npcm7xx_fiu_bus {
	struct device *dev;
	struct mutex lock;
	struct regmap *regmap;
	void __iomem *regbase;
	struct resource *res_mem;
	u32 MaxChipAddMap;
	resource_size_t iosize;
	struct clk *clk;
	bool Direct_Read;
	bool spix_mode;
	struct npcm7xx_chip *chip[NPCM7XX_MAX_CHIP_NUM];
};


/* #define NPCMX50_MTD_SPINOR_DEBUG */
#ifdef NPCMX50_MTD_SPINOR_DEBUG
	#define DEBUG_FLASH(format, args...)    printk(format, ##args)
	#define DUMP_MSG(format, args...)       dump_msg(format, ## args)
#else
	#define DEBUG_FLASH(format, args...)
	#define DUMP_MSG(format, args...)
#endif


/*
 * MTD static functions:
 */
#ifdef NPCMX50_MTD_SPINOR_DEBUG
static void dump_msg(const char *label, const unsigned char *buf,
		     unsigned int length);
#endif

static const struct regmap_config npcm_mtd_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = FIU_MAX_REG_LIMIT,
};

//static void npcm7xx_fiu_flash_unlock_protection(struct spi_nor *nor);

/*
 * FIU Functions
 */
static int npcm7xx_fiu_direct_read(struct mtd_info *mtd, loff_t from, size_t len,
					size_t *retlen, u_char *buf)
{
	struct spi_nor *nor = mtd->priv;
	struct npcm7xx_chip *chip = nor->priv;

	memcpy(buf, chip->flash_region_mapped_ptr + from , len);
	wmb();

	*retlen = len;
	return 0;
}

/*TODO: like tomer says, if somebody wants to use SPI0/3 in this mode they can use this API (i.e. "Direct_Read" ) */
static int npcm7xx_fiu_direct_write(struct mtd_info *mtd, loff_t to, size_t len,
					size_t *retlen, const u_char *buf)
{
	struct spi_nor *nor = mtd->priv;
	struct npcm7xx_chip *chip = nor->priv;

	memcpy(chip->flash_region_mapped_ptr + to,buf, len);
	wmb();

	*retlen = len;
	return 0;
}

static int npcm7xx_fiu_uma_read(struct spi_nor *nor, u8 transaction_code,
				u32 address, bool is_address_size, u8 *data,
				u32 data_size)
{
	u32 data_reg[4];
	u32 uma_cfg = BIT(10);
	int ret = 0;
	u32 address_size = 0;
	u32 val;

	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_fiu_bus *host = chip->host;

	switch (chip->chipselect) {
	case 0:
	case 1:
	case 2:
	case 3:
		regmap_update_bits(host->regmap, FIU_UMA_CTS,
		     FIU_UMA_CTS_DEV_NUM, (chip->chipselect<<8));
		break;
	default:
		return -ENODEV;
	}
	regmap_update_bits(host->regmap, FIU_UMA_CMD,
	     FIU_UMA_CMD_CMD, transaction_code);

	if (is_address_size)
		address_size = nor->addr_width;

	uma_cfg |= (address_size << 11);

	regmap_write(host->regmap, FIU_UMA_ADDR, address);
	uma_cfg |= (data_size << 24);

	regmap_write(host->regmap, FIU_UMA_CFG, uma_cfg);
	regmap_write_bits(host->regmap, FIU_UMA_CTS,
			   FIU_UMA_CTS_EXEC_DONE, FIU_UMA_CTS_EXEC_DONE);

	/*
	 * wait for indication that transaction has terminated
	 */
	ret = regmap_read_poll_timeout(host->regmap, FIU_UMA_CTS, val,
	(!(val & FIU_UMA_CTS_EXEC_DONE)), 0, 0);
	if (ret)
		return ret;
	
	if (data_size >= FIU_UMA_DATA_SIZE_1)
		regmap_read(host->regmap, FIU_UMA_DR0, &data_reg[0]);
	if (data_size >= FIU_UMA_DATA_SIZE_5)
		regmap_read(host->regmap, FIU_UMA_DR1, &data_reg[1]);
	if (data_size >= FIU_UMA_DATA_SIZE_9)
		regmap_read(host->regmap, FIU_UMA_DR2, &data_reg[2]);
	if (data_size >= FIU_UMA_DATA_SIZE_13)
		regmap_read(host->regmap, FIU_UMA_DR3, &data_reg[3]);

	memcpy(data, data_reg, data_size);

	return ret;
}


static int npcm7xx_fiu_uma_write(struct spi_nor *nor, u8 transaction_code,
				 u32 address, bool is_address_size, u8 *data,
				 u32 data_size)
{
	u32 data_reg[4] = {0};
	u32 uma_reg   = BIT(10);
	int ret = 0;
	u32 address_size = 0;
	u32 val;

	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_fiu_bus *host = chip->host;

	switch (chip->chipselect) {
	case 0:
	case 1:
	case 2:
	case 3:
		regmap_update_bits(host->regmap, FIU_UMA_CTS,
				   FIU_UMA_CTS_DEV_NUM, (chip->chipselect<<8));
		break;
	default:
		return -ENODEV;
	}

	regmap_update_bits(host->regmap, FIU_UMA_CMD,
			   FIU_UMA_CMD_CMD, transaction_code);
	if (is_address_size)
		address_size = nor->addr_width;

	uma_reg |= (address_size << 11);
	regmap_write(host->regmap, FIU_UMA_ADDR, address);

	memcpy(data_reg, data, data_size);

	if (data_size >= FIU_UMA_DATA_SIZE_1)
		regmap_write(host->regmap, FIU_UMA_DW0, data_reg[0]);
	if (data_size >= FIU_UMA_DATA_SIZE_5)
		regmap_write(host->regmap, FIU_UMA_DW1, data_reg[1]);
	if (data_size >= FIU_UMA_DATA_SIZE_9)
		regmap_write(host->regmap, FIU_UMA_DW2, data_reg[2]);
	if (data_size >= FIU_UMA_DATA_SIZE_13)
		regmap_write(host->regmap, FIU_UMA_DW3, data_reg[3]);

	uma_reg |= (data_size << 16);

	regmap_write(host->regmap, FIU_UMA_CFG, uma_reg);
	regmap_write_bits(host->regmap, FIU_UMA_CTS,
			   FIU_UMA_CTS_EXEC_DONE, FIU_UMA_CTS_EXEC_DONE);

	/*
	 * wait for indication that transaction has terminated
	 */
	ret = regmap_read_poll_timeout(host->regmap, FIU_UMA_CTS, val,
				       (!(val & FIU_UMA_CTS_EXEC_DONE)), 0, 0);

	return ret;
}


static int npcm7xx_fiu_manualwrite(struct spi_nor *nor, u8 transaction_code,
				   u32 address, u8 *data, u32 data_size)
{
	u32  num_data_chunks;
	u32  remain_data;
	u32  idx = 0;

	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_fiu_bus *host = chip->host;

	num_data_chunks  = data_size / CHUNK_SIZE;
	remain_data  = data_size % CHUNK_SIZE;

	regmap_update_bits(host->regmap, FIU_UMA_CTS,
			   FIU_UMA_CTS_DEV_NUM, (chip->chipselect<<8));
	regmap_update_bits(host->regmap, FIU_UMA_CTS, FIU_UMA_CTS_SW_CS, 0);

	npcm7xx_fiu_uma_write(nor, transaction_code, address, true, NULL, 0);
/*
 * Starting the data writing loop in multiples of 8
 */
	for (idx = 0; idx < num_data_chunks; ++idx) {
		npcm7xx_fiu_uma_write(nor, data[0], (u32)NULL, false, &data[1],
				      CHUNK_SIZE-1);
		data += CHUNK_SIZE;
	}

/*
 * Handling chunk remains
 */
	if (remain_data > 0)
		npcm7xx_fiu_uma_write(nor, data[0], (u32)NULL, false, &data[1],
				      remain_data-1);

	regmap_update_bits(host->regmap, FIU_UMA_CTS, FIU_UMA_CTS_SW_CS,
			   FIU_UMA_CTS_SW_CS);

	return 0;
}

/*
 * SPI Functions
 */
static void npcm7xx_fiu_flash_high_addr_wr(struct spi_nor *nor, u8 HighAddr)
{
	npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
	npcm7xx_fiu_uma_write(nor, NPCM_SPI_WR_EXT_ADDR_REG_CMD, 0, false,
			      &HighAddr, sizeof(u8));
}

static void npcm7xx_fiu_flash_common_getstatus(struct spi_nor *nor, u8 *status)
{
	npcm7xx_fiu_uma_read(nor, SPINOR_OP_RDSR, 0, 0, status, 1);
}

static int npcm7xx_fiu_flash_common_waittillready(struct spi_nor *nor)
{
	u8 busy = 1;

	do {
		npcm7xx_fiu_flash_common_getstatus(nor, &busy);
		/* Keep only "busy" bit 0 */
		busy &= 0x01;
	} while (busy);
	
	return 0;
}

static int npcm7xx_fiu_flash_common_write(struct spi_nor *nor, u32 destAddr,
				     u8 *data, u32 size)
{
	if ((destAddr >> 24) && (nor->addr_width == 3)) {
		npcm7xx_fiu_flash_high_addr_wr(nor, destAddr >> 24);
		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
		npcm7xx_fiu_manualwrite(nor, SPINOR_OP_PP,
					(destAddr & 0xFFFFFF), data, size);
		if (npcm7xx_fiu_flash_common_waittillready(nor))
			return -1;
		npcm7xx_fiu_flash_high_addr_wr(nor, 0);
	} else {
		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
		npcm7xx_fiu_manualwrite(nor, SPINOR_OP_PP, destAddr, data,
					size);
		if (npcm7xx_fiu_flash_common_waittillready(nor))
			return -1;
	}

	return 0;
}

static int npcm7xx_fiu_flash_unlock_protection(struct spi_nor *nor)
{
	u8 status_reg_val = 0;

	npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
	npcm7xx_fiu_uma_write(nor, SPINOR_OP_WRSR, 0, false,
			      &status_reg_val, sizeof(u8));
	if (npcm7xx_fiu_flash_common_waittillready(nor))
		return -1;

	return 0;
}

static ssize_t npcm7xx_fiu_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *write_buf)
{
	u32 local_addr = (u32) to;
	u32 cnt = (u32) len;
	u32 actual_size     = 0;
	struct mtd_info *mtd;
	int ret;

	mtd = &nor->mtd;

	DEBUG_FLASH("mtd_spinor: %s %s 0x%08x, len %zd\n", __func__,
		    "to", (u32)to, len);

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > mtd->size)
		return -EINVAL;

	if (cnt != 0) {
		while (cnt) {
			actual_size = ((((local_addr)/mtd->writesize) + 1)
				       *mtd->writesize) - (local_addr);
			if (actual_size > cnt)
				actual_size = cnt;

			ret = npcm7xx_fiu_flash_common_write(nor, local_addr,
							     (u_char *)write_buf,
							     actual_size);
			if (ret)
				return ret;
			
			write_buf += actual_size;
			local_addr += actual_size;
			cnt -= actual_size;
		}
	}

	return (len - cnt);
}

static ssize_t npcm7xx_fiu_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *read_buf)
{
	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_fiu_bus *host = chip->host;
	struct mtd_info *mtd;
	int i, readlen, currlen;
	u32 addr;
	u8 *buf_ptr;
	size_t retlen = 0;

	mtd = &nor->mtd;

	DEBUG_FLASH("mtd_spinor: %s %s 0x%08x, len %zd\n", __func__, "from",
		(u32)from, len);

	if (!len)
		return 0;

	if (from + len > (u32)mtd->size)
		return -EINVAL;

	DEBUG_FLASH("mtd_spinor: %s , mtd->size 0x%08x %p\n", __func__,
		    (u32) mtd->size, read_buf);


	if (host->Direct_Read) {

		if (mtd->size > SIZE_16MB) {
			if (nor->addr_width == 3)
				regmap_update_bits(host->regmap, FIU_DRD_CFG,
						   FIU_DRD_CFG_RDCMD, SPINOR_OP_READ_1_2_2_4B);
			else
				regmap_update_bits(host->regmap, FIU_DRD_CFG,
						   FIU_DRD_CFG_RDCMD, SPINOR_OP_WRSR);
			regmap_update_bits(host->regmap, FIU_DRD_CFG,
					   FIU_DRD_CFG_ADDSIZ, 1<<16);
		} else {
			regmap_update_bits(host->regmap, FIU_DRD_CFG,
					   FIU_DRD_CFG_RDCMD, SPINOR_OP_READ_1_2_2);
			regmap_update_bits(host->regmap, FIU_DRD_CFG,
					   FIU_DRD_CFG_ADDSIZ, 0);
		}

		npcm7xx_fiu_direct_read(mtd, from, len, &retlen, read_buf);

		npcm7xx_fiu_flash_high_addr_wr(nor, 0);
	} else {
		/* NOTE:  OPCODE_FAST_READ (if available) is faster... */
		i = 0;
		currlen = (int) len;

		do {
			addr = ((u32) from + i);
			if (currlen < 4)
				readlen = currlen;
			else
				readlen = 4;

			DEBUG_FLASH("mtd_spinor: ori_addr=0x%x, addr=0x%x\n",
				    ((u32) from + i), addr);

			buf_ptr = read_buf + i;

			if ((addr >> 24) && (nor->addr_width == 3))
				npcm7xx_fiu_flash_high_addr_wr(nor, addr >> 24);

			npcm7xx_fiu_uma_read(nor, SPINOR_OP_READ,
					     (addr&0xFFFFFF), true, buf_ptr,
					     readlen);

			if ((addr >> 24) && (nor->addr_width == 3))
				npcm7xx_fiu_flash_high_addr_wr(nor, 0);

			DEBUG_FLASH("mtd_spinor: buf_ptr=0x%x buf_val=0x%x "
				    "i=%d readlen =%d\n", (u32)buf_ptr,
				    *((u32 *)buf_ptr), i, readlen);

			i += readlen;
			currlen -= 4;
		} while (currlen > 0);

		retlen = i;
	}

	DUMP_MSG("MTD_READ", read_buf, i);
	return retlen;
}

static int npcm7xx_fiu_erase(struct spi_nor *nor, loff_t offs)
{
	u32 addr = (u32)offs;
	struct mtd_info *mtd;
	mtd = &nor->mtd;

	if ((addr >> 24) && (nor->addr_width == 3)) {
		npcm7xx_fiu_flash_high_addr_wr(nor, addr >> 24);

		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL,
				      0);
		if (mtd->erasesize == 4096)
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_BE_4K,
					      (addr & 0xFFFFFF), true, NULL, 0);
		else
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_SE,
					      (addr & 0xFFFFFF), true, NULL, 0);
		if (npcm7xx_fiu_flash_common_waittillready(nor))
			return -1;
		npcm7xx_fiu_flash_high_addr_wr(nor, 0);
	} else {
		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL,
				      0);
		if (mtd->erasesize == 4096)
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_BE_4K, addr,
					      true, NULL, 0);
		else
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_SE, addr,
					      true, NULL, 0);
		if (npcm7xx_fiu_flash_common_waittillready(nor))
			return -1;
	}

	return 0;
}

static int npcm7xx_fiu_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len)
{
	return npcm7xx_fiu_uma_read(nor, opcode, 0, 0, buf, len);
}

static int npcm7xx_fiu_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				 int len)
{
	return npcm7xx_fiu_uma_write(nor, opcode, (u32)NULL, false, buf, len);
}

static int npcm7xx_fiu_nor_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_fiu_bus *host = chip->host;

	mutex_lock(&host->lock);

	return 0;
}

static void npcm7xx_fiu_nor_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_fiu_bus *host = chip->host;

	mutex_unlock(&host->lock);
}

#ifdef NPCMX50_MTD_SPINOR_DEBUG
static void dump_msg(const char *label, const unsigned char *buf,
		     unsigned int length)
{
	unsigned int	start, num, i;
	char		line[52], *p;

	if (length > 32)
		length = 32;

	DEBUG_FLASH("MTD_SPI: %s, length %u:\n", label, length);

	start = 0;
	while (length > 0) {
		num = min(length, 16u);
		p = line;
		for (i = 0; i < num; ++i) {
			if (i == 8)
				*p++ = ' ';
			sprintf(p, " %02x", buf[i]);
			p += 3;
		}
		*p = 0;
		pr_info("%6x: %s\n", start, line);
		buf += num;
		start += num;
		length -= num;
	}
}
#endif

/*
 * Expansion bus registers as mtd_ram device
 */
static int npcm7xx_mtd_ram_register(struct npcm7xx_fiu_bus *host)
{
	struct device *dev = host->dev;
	struct spi_nor *nor;		/* Hacky but needed to have common direct APIs */
	struct npcm7xx_chip *chip;
	struct mtd_info *mtd;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->host = host;
	chip->chipselect = 0;	/* TODO: from dt */
	nor = &chip->nor;
	mtd = &nor->mtd;

	nor->dev = dev;
	nor->priv = chip;

	chip->flash_region_mapped_ptr = devm_ioremap(dev, (host->res_mem->start +
	     (host->MaxChipAddMap * chip->chipselect)),host->MaxChipAddMap);

	if (!chip->flash_region_mapped_ptr)
		return -EIO;

	/* Populate mtd_info data structure */
	*mtd = (struct mtd_info) {
		.dev		= { .parent = dev },
		.name		= "exp-bus",
		.type		= MTD_RAM,
		.priv		= nor,
		.size		= (host->MaxChipAddMap),	/* TODO: should this be configurable? */
		.writesize	= 1,	/* TODO: Make it based of DRD_CFG */
		.writebufsize	= 1,	/* same above */
		.flags		= MTD_CAP_RAM,
		._read		= npcm7xx_fiu_direct_read,
		._write		= npcm7xx_fiu_direct_write,
	};

	/* set access to SPI-X mode*/
	regmap_update_bits(host->regmap, FIU_DRD_CFG,
			   FIU_DRD_CFG_ACCTYPE, (3<<8));
	
	/* set burst to 16byte*/
	regmap_update_bits(host->regmap, FIU_DWR_CFG_W_BURST,
		   FIU_DWR_CFG_W_BURST, (3<<24));

	return mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
}


/*
 * Get spi flash device information and register it as a mtd device.
 */
static int npcm7xx_fiu_nor_register(struct device_node *np,
				struct npcm7xx_fiu_bus *host)
{
	struct device *dev = host->dev;
	struct spi_nor *nor;
	struct npcm7xx_chip *chip;
	struct mtd_info *mtd;
	u32 chipselect;
	int ret;
	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_READ_1_1_2 |
			SNOR_HWCAPS_READ_1_2_2 |
			SNOR_HWCAPS_READ_2_2_2 |
			SNOR_HWCAPS_READ_1_1_4 |
			SNOR_HWCAPS_READ_1_4_4 |
			SNOR_HWCAPS_READ_4_4_4 |
			SNOR_HWCAPS_PP |
			SNOR_HWCAPS_PP_1_1_4 |
			SNOR_HWCAPS_PP_1_4_4 |
			SNOR_HWCAPS_PP_4_4_4,
	};

	//pr_info("npcm7xx_fiu_nor_register 1\n");
	/* This driver does not support NAND or NOR flash devices. */
	if (!of_device_is_compatible(np, "jedec,spi-nor")) {
		dev_err(dev, "The device is no compatible to jedec,spi-nor\n");
		return -ENOMEM;
	}

	//pr_info("npcm7xx_fiu_nor_register 2\n");
	ret = of_property_read_u32(np, "reg", &chipselect);
	if (ret) {
		dev_err(dev, "There's no reg property for %s\n", np->full_name);
		return ret;
	}

	//pr_info("npcm7xx_fiu_nor_register 3\n");
	if (chipselect >= NPCM7XX_MAX_CHIP_NUM) {
		dev_warn(dev, "Flash device number exceeds the maximum "
			 "chipselect number\n");
		return -ENOMEM;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->host = host;
	chip->chipselect = chipselect;

	chip->flash_region_mapped_ptr = devm_ioremap(dev, (host->res_mem->start +
	     (host->MaxChipAddMap * chip->chipselect)),host->MaxChipAddMap);

	if (!chip->flash_region_mapped_ptr)
		return -EIO;

	nor = &chip->nor;
	mtd = &nor->mtd;

	nor->dev = dev;
	nor->priv = chip;

	spi_nor_set_flash_node(nor, np);

/*	ret = of_property_read_u32(np, "spi-max-frequency",
 *		&chip->clkrate);
 *	if (ret) {
 *	    dev_err(dev, "There's no spi-max-frequency property for %s\n",
 *		np->full_name);
 *	    return ret;
 *	}
 */
	nor->prepare = npcm7xx_fiu_nor_prep;
	nor->unprepare = npcm7xx_fiu_nor_unprep;

	nor->read_reg = npcm7xx_fiu_read_reg;
	nor->write_reg = npcm7xx_fiu_write_reg;
	nor->read = npcm7xx_fiu_read;
	nor->write = npcm7xx_fiu_write;
	nor->erase = npcm7xx_fiu_erase;

	//pr_info("npcm7xx_fiu_nor_register 4\n");
	ret = spi_nor_scan(nor, NULL, &hwcaps);
	if (ret)
		return ret;

	if (mtd->size > SIZE_16MB) {
		/* If Flash size is over 16MB the spi_nor_scan sets
		   automatically the FLASH to work with 4 byte addressing.
		   Our driver handle Flash size over 16MB with 3 byte address.
		   Revert back to 3 byte address size cause issues so the
		   sequence below resets WINBOND and MACRONIX FLASH to work
		   again with 3 byte address (From Kernel 4.14 and above
		   the address width statically configured by the driver)*/
		if (npcm7xx_fiu_flash_common_waittillready(nor))
			return -1;
		npcm7xx_fiu_uma_write(nor, NPCM_SPI_EN_RST_CMD, 0,
		   		      false, NULL, 0);
		   npcm7xx_fiu_uma_write(nor, NPCM_SPI_RST_DEVICE_CMD, 0,
		   		      false, NULL, 0);
		if (npcm7xx_fiu_flash_common_waittillready(nor))
			return -1;
		nor->addr_width = 3;
	}

	//pr_info("npcm7xx_fiu_nor_register 5\n");
	ret = npcm7xx_fiu_flash_unlock_protection(nor);
	if (ret)
		return ret;

	//mtd->name = np->name;
	ret = mtd_device_register(mtd, NULL, 0);
	if (ret)
		return ret;

	//pr_info("npcm7xx_fiu_nor_register 6\n");
	host->chip[chip->chipselect] = chip;
	return 0;
}

static void npcm7xx_fiu_nor_unregister_all(struct npcm7xx_fiu_bus *host)
{
	struct npcm7xx_chip *chip;
	int n;

	for (n = 0; n < NPCM7XX_MAX_CHIP_NUM; n++) {
		chip = host->chip[n];
		if (chip)
			mtd_device_unregister(&chip->nor.mtd);
	}
}

static int npcm7xx_fiu_nor_register_all(struct npcm7xx_fiu_bus *host)
{
	struct device *dev = host->dev;
	struct device_node *np;
	int ret;
	host->Direct_Read = false;

	if (host->res_mem) {
		/* set access to Dual I/O */
		regmap_update_bits(host->regmap, FIU_DRD_CFG,
				   FIU_DRD_CFG_ACCTYPE, (1<<8));
		/* set dummy byte to 1 */
		regmap_update_bits(host->regmap, FIU_DRD_CFG,
				   FIU_DRD_CFG_DBW, (1<<12));
		host->Direct_Read = true;
	} else
		host->Direct_Read = false;

	for_each_available_child_of_node(dev->of_node, np) {
		ret = npcm7xx_fiu_nor_register(np, host);
		if (ret)
			goto fail;
	}

	return 0;

fail:
	npcm7xx_fiu_nor_unregister_all(host);
	return ret;
}

static int npcm7xx_fiu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct npcm7xx_fiu_bus *host;
	int ret;

	//pr_info("npcm7xx_fiu_probe 1\n");
	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	platform_set_drvdata(pdev, host);
	host->dev = dev;

	//pr_info("npcm7xx_fiu_probe 2\n");
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "control");
	host->regbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->regbase))
		return PTR_ERR(host->regbase);

	//pr_info("npcm7xx_fiu_probe 3\n");
	host->regmap = devm_regmap_init_mmio(dev, host->regbase,
					  &npcm_mtd_regmap_config);
	if (IS_ERR(host->regmap)) {
		dev_err(dev, "Failed to create regmap\n");
		return PTR_ERR(host->regmap);
	}

	//pr_info("npcm7xx_fiu_probe 4\n");
	host->res_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "memory");
	host->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(host->clk))
		return PTR_ERR(host->clk);

	ret = of_property_read_u32(dev->of_node, "chip-max-address-map",
				   &host->MaxChipAddMap);
	if (ret) {
		pr_info("There's no chip-max-address-map property "
			     "for %s adjust 128Mb\n",
			dev->of_node->full_name);
		host->MaxChipAddMap = 0x8000000;
	}

	host->spix_mode = of_property_read_bool(dev->of_node, "spix-mode");

	//pr_info("npcm7xx_fiu_probe 5\n");
	mutex_init(&host->lock);
	//clk_prepare_enable(host->clk);

	if(host->spix_mode)
		ret = npcm7xx_mtd_ram_register(host);
	else
		ret = npcm7xx_fiu_nor_register_all(host);

	if (ret) {
		mutex_destroy(&host->lock);
		//clk_disable_unprepare(host->clk);
		pr_info("npcm7xx register failed\n");
	}

	//pr_info("npcm7xx_fiu_probe 6\n");
	DEBUG_FLASH("mtd_spinor: %s() date=%s time=%s\n\n", __func,  __date__,
		     __time__);

	return ret;
}

static int npcm7xx_fiu_remove(struct platform_device *pdev)
{
	struct npcm7xx_fiu_bus *host = platform_get_drvdata(pdev);

	npcm7xx_fiu_nor_unregister_all(host);
	mutex_destroy(&host->lock);
	clk_disable_unprepare(host->clk);
	return 0;
}

static const struct of_device_id npcm7xx_fiu_dt_ids[] = {
	{ .compatible = "nuvoton,npcm750-fiu" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, npcm7xx_fiu_dt_ids);

static struct platform_driver npcm7xx_fiu_driver = {
	.driver = {
		.name	= "NPCM-FIU",
		.bus	= &platform_bus_type,
		.of_match_table = npcm7xx_fiu_dt_ids,
	},
	.probe      = npcm7xx_fiu_probe,
	.remove	    = npcm7xx_fiu_remove,
};
module_platform_driver(npcm7xx_fiu_driver);

MODULE_DESCRIPTION("Nuvoton FLASH Interface Unit SPI Controller Driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
