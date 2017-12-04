/*
* Copyright (c) 2013-2017 by Nuvoton Technology Corporation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify it under the terms of the
* GNU General Public License Version 2 (or later) as published by the Free Software Foundation.
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
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>

#include <mtd/mtd-abi.h>
#include <asm/uaccess.h>

#include <asm/io.h>
#include <asm/sizes.h>
#include <linux/vmalloc.h>

#include "regs_fiudrv.h"

//#include <mach/hal.h>

/*typedef struct bit_field {
    u8 offset;
    u8 size;
} bit_field_t;*/

#ifdef REG_READ
#undef REG_READ
#endif
static inline u32 REG_READ(unsigned int __iomem *mem ) {
    return ioread32(mem);
}

#ifdef REG_WRITE
#undef REG_WRITE
#endif   
static inline void REG_WRITE(unsigned int __iomem *mem, u32 val ) {
    iowrite32(val, mem);
}

#ifdef SET_REG_FIELD
#undef SET_REG_FIELD
#endif  
/*---------------------------------------------------------------------------------------------------------*/
/* Set field of a register / variable according to the field offset and size                               */
/*---------------------------------------------------------------------------------------------------------*/
static inline void SET_REG_FIELD(unsigned int __iomem *mem, bit_field_t bit_field, u32 val) {
    u32 tmp = ioread32(mem);
    tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset); // mask the field size
    tmp |= val << bit_field.offset;  // or with the requested value
    iowrite32(tmp, mem);
}

#ifdef SET_VAR_FIELD
#undef SET_VAR_FIELD
#endif 
// bit_field should be of bit_field_t type
#define SET_VAR_FIELD(var, bit_field, value) { \
    typeof(var) tmp = var;                 \
    tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset); /* mask the field size */ \
    tmp |= value << bit_field.offset;  /* or with the requested value */               \
    var = tmp;                                                                         \
}

#ifdef READ_REG_FIELD
#undef READ_REG_FIELD
#endif 
/*---------------------------------------------------------------------------------------------------------*/
/* Get field of a register / variable according to the field offset and size                               */
/*---------------------------------------------------------------------------------------------------------*/
static inline u8 READ_REG_FIELD(unsigned int __iomem *mem, bit_field_t bit_field) {
    u8 tmp = ioread32(mem);
    tmp = tmp >> bit_field.offset;     // shift right the offset
    tmp &= (1 << bit_field.size) - 1;  // mask the size
    return tmp;
}

#ifdef READ_VAR_FIELD
#undef READ_VAR_FIELD
#endif 
// bit_field should be of bit_field_t type
#define READ_VAR_FIELD(var, bit_field) ({ \
    typeof(var) tmp = var;           \
    tmp = tmp >> bit_field.offset;     /* shift right the offset */ \
    tmp &= (1 << bit_field.size) - 1;  /* mask the size */          \
    tmp;                                                     \
})

#ifdef MASK_FIELD
#undef MASK_FIELD
#endif 
/*---------------------------------------------------------------------------------------------------------*/
/* Build a mask of a register / variable field                                                             */
/*---------------------------------------------------------------------------------------------------------*/
// bit_field should be of bit_field_t type
#define MASK_FIELD(bit_field) \
    (((1 << bit_field.size) - 1) << bit_field.offset) /* mask the field size */ 

#ifdef BUILD_FIELD_VAL
#undef BUILD_FIELD_VAL
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Expand the value of the given field into its correct position                                           */
/*---------------------------------------------------------------------------------------------------------*/
// bit_field should be of bit_field_t type
#define BUILD_FIELD_VAL(bit_field, value)  \
    ((((1 << bit_field.size) - 1) & (value)) << bit_field.offset)


#ifdef SET_REG_MASK
#undef SET_REG_MASK
#endif  
/*---------------------------------------------------------------------------------------------------------*/
/* Set field of a register / variable according to the field offset and size                               */
/*---------------------------------------------------------------------------------------------------------*/
static inline void SET_REG_MASK(unsigned int __iomem *mem, u32 val) {
    iowrite32(ioread32(mem) | val, mem);
}

#define DRIVER_NAME "npcm750_spinor"

#define SIZE_16MB 0x1000000
#define MAX_READY_WAIT_COUNT	1000000

#define WRITE_TRANSFER_16_BYTES

#ifdef WRITE_TRANSFER_16_BYTES      // command=1  data=15
#define CHUNK_SIZE  16
#endif

#ifdef WRITE_TRANSFER_17_BYTES      // command=1 data=16
#define CHUNK_SIZE  17
#endif

struct npcm750_spi_bus;

struct npcm750_chip {
	u32 fiu_num;
	u32 clkrate;
    u32 chipselect;
    struct spi_nor	nor;
	struct npcm750_spi_bus *host;
};

#define NPCM750_MAX_CHIP_NUM		4

struct npcm750_spi_bus {
	struct device *dev;
	struct mutex lock;

	void __iomem *regbase;
    struct resource *res_mem;
    u32 MaxChipAddMap;
    resource_size_t iosize;
	struct clk *clk;
    u32 fiu_num;
    bool Direct_Read;

    struct npcm750_chip *chip[NPCM750_MAX_CHIP_NUM];
};

void __iomem *fiu_base[FIU_MAX_MODULE_NUM];

//#define NPCMX50_MTD_SPINOR_DEBUG
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
static void dump_msg(const char *label, const unsigned char *buf, unsigned int length);
#endif
static int npcmx50_spinor_read(struct spi_nor *nor, loff_t from, size_t len, size_t *retlen, u_char *buf);
static int npcmx50_spinor_write(struct spi_nor *nor, loff_t to, size_t len, size_t *retlen, const u8 *buf);
static int npcmx50_spinor_erase(struct spi_nor *nor, loff_t offs);
static void spi_flash_unlock_protection(struct spi_nor *nor);

/*---------------------------------------------------------------------------------------------------------*/
/*                                              FIU Functions                                              */
/*---------------------------------------------------------------------------------------------------------*/

int FIU_UMA_Read(struct spi_nor *nor,
                        u8    transaction_code,
                        u32   address,
                        bool  is_address_size,
                        u8 *  data,
                        u32   data_size)
{

	u32 data_reg[4];
    u32 uma_cfg = 0x0;
    int ret = 0;
    u32 address_size = 0;

    struct npcm750_chip *chip = nor->priv;
    struct npcm750_spi_bus *host = chip->host;

    /*-----------------------------------------------------------------------------------------------------*/
    /* set device number - DEV_NUM in FIU_UMA_CTS                                                          */
    /* legal device numbers are 0,1,2,3                                                                    */
    /*-----------------------------------------------------------------------------------------------------*/
    switch(chip->chipselect)
    {
        case 0 :
        case 1 :
        case 2 :
        case 3 :
            SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_DEV_NUM, (u32)chip->chipselect); 
            break;
        default:
            return -ENODEV;
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* set transaction code in FIU_UMA_CODE                                                                */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(FIU_UMA_CMD(host->fiu_num), FIU_UMA_CMD_CMD, transaction_code);
    SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_CMDSIZ, 1);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set address size bit                                                                                */
    /*-----------------------------------------------------------------------------------------------------*/
    if (is_address_size) 
        address_size = nor->addr_width;

    SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_ADDSIZ, address_size);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set the UMA address registers                                                                       */
    /*-----------------------------------------------------------------------------------------------------*/
    REG_WRITE(FIU_UMA_ADDR(host->fiu_num), address);
    /*-----------------------------------------------------------------------------------------------------*/
    /* Set data size and direction                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_RDATSIZ, data_size);
    SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_WDATSIZ, 0);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set UMA CFG                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/
    REG_WRITE(FIU_UMA_CFG(host->fiu_num), uma_cfg);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Initiate the read                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE, 1);


    /*-----------------------------------------------------------------------------------------------------*/
    /* wait for indication that transaction has terminated                                                 */
    /*-----------------------------------------------------------------------------------------------------*/
    while (READ_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE) == FIU_TRANS_STATUS_IN_PROG){}

    /*-----------------------------------------------------------------------------------------------------*/
    /* copy read data from FIU_UMA_DB0-3 regs to data buffer                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    /*-----------------------------------------------------------------------------------------------------*/
    /* Set the UMA data registers - FIU_UMA_DB0-3                                                          */
    /*-----------------------------------------------------------------------------------------------------*/
    if (data_size >= FIU_UMA_DATA_SIZE_1)
    {
		data_reg[0] = REG_READ(FIU_UMA_DR0(host->fiu_num));
    }
    if (data_size >= FIU_UMA_DATA_SIZE_5)
    {
		data_reg[1] = REG_READ(FIU_UMA_DR1(host->fiu_num));
    }
    if (data_size >= FIU_UMA_DATA_SIZE_9)
    {
		data_reg[2] = REG_READ(FIU_UMA_DR2(host->fiu_num));
    }
    if (data_size >= FIU_UMA_DATA_SIZE_13)
    {
		data_reg[3] = REG_READ(FIU_UMA_DR3(host->fiu_num));
    }

    memcpy(data,data_reg,data_size);
	
    return ret;
}


int FIU_UMA_Write(
           struct spi_nor *nor,
           u8 transaction_code,
           u32 address,
           bool is_address_size,
           u8 * data,
           u32 data_size)
{

	u32 data_reg[4] = {0};
    u32 uma_reg   = 0x0;
    int ret = 0;
    u32 address_size = 0;

    struct npcm750_chip *chip = nor->priv;
    struct npcm750_spi_bus *host = chip->host;

    /*-----------------------------------------------------------------------------------------------------*/
    /* set device number - DEV_NUM in FIU_UMA_CTS                                                          */
    /* legal device numbers are 0,1,2,3                                                                    */
    /*-----------------------------------------------------------------------------------------------------*/
    switch(chip->chipselect)
    {
        case 0 :
        case 1 :
        case 2 :
        case 3 :
            SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_DEV_NUM, (u32)chip->chipselect);
            break;
        default:
            return -ENODEV;
    }
    /*-----------------------------------------------------------------------------------------------------*/
    /* Set transaction code (command byte source)                                                          */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(FIU_UMA_CMD(host->fiu_num), FIU_UMA_CMD_CMD, transaction_code);
    SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_CMDSIZ, 1);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set address size bit                                                                                */
    /*-----------------------------------------------------------------------------------------------------*/
    if (is_address_size) 
        address_size = nor->addr_width;

    SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_ADDSIZ, address_size);
    /*-----------------------------------------------------------------------------------------------------*/
    /* Set the UMA address registers                                                                       */
    /*-----------------------------------------------------------------------------------------------------*/
    REG_WRITE(FIU_UMA_ADDR(host->fiu_num), address);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set the UMA data registers - FIU_UMA_DB0-3                                                          */
    /*-----------------------------------------------------------------------------------------------------*/

    memcpy(data_reg,data,data_size);

    if (data_size >= FIU_UMA_DATA_SIZE_1)
    {
        REG_WRITE(FIU_UMA_DW0(host->fiu_num), data_reg[0]);
    }
    if (data_size >= FIU_UMA_DATA_SIZE_5)
    {
        REG_WRITE(FIU_UMA_DW1(host->fiu_num), data_reg[1]);
    }
    if (data_size >= FIU_UMA_DATA_SIZE_9)
    {
        REG_WRITE(FIU_UMA_DW2(host->fiu_num), data_reg[2]);
    }
    if (data_size >= FIU_UMA_DATA_SIZE_13)
    {
        REG_WRITE(FIU_UMA_DW3(host->fiu_num), data_reg[3]);
    }


    /*-----------------------------------------------------------------------------------------------------*/
    /* Set data size and direction                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_WDATSIZ, data_size);
    SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_RDATSIZ, 0);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set UMA status                                                                                      */
    /*-----------------------------------------------------------------------------------------------------*/
    REG_WRITE(FIU_UMA_CFG(host->fiu_num), uma_reg);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Initiate the read                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE, 1);

    /*-----------------------------------------------------------------------------------------------------*/
    /* wait for indication that transaction has terminated                                                 */
    /*-----------------------------------------------------------------------------------------------------*/
    while (READ_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE) == FIU_TRANS_STATUS_IN_PROG){}

    return ret;
}


int FIU_ManualWrite(struct spi_nor *nor, u8 transaction_code, u32 address, u8 * data, u32 data_size)
{
    u8   uma_cfg  = 0x0;
    u32  num_data_chunks;
    u32  remain_data;
    u32  idx = 0;

    struct npcm750_chip *chip = nor->priv;
    struct npcm750_spi_bus *host = chip->host;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Constructing var for FIU_UMA_CFG register status                                                            */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_WDATSIZ, 16);        // Setting Write Data size

    /*-----------------------------------------------------------------------------------------------------*/
    /* Calculating relevant data                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    num_data_chunks  = data_size / CHUNK_SIZE;
    remain_data  = data_size % CHUNK_SIZE;	

    /*-----------------------------------------------------------------------------------------------------*/
    /* First we activate Chip Select (CS) for the given flash device                                       */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_DEV_NUM, (u32)chip->chipselect);
    SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_SW_CS, 0);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Writing the transaction code and the address to the bus                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    FIU_UMA_Write(nor, transaction_code, address, true, NULL, 0);
    /*-----------------------------------------------------------------------------------------------------*/
    /* Starting the data writing loop in multiples of 8                                                    */
    /*-----------------------------------------------------------------------------------------------------*/
    for(idx=0; idx<num_data_chunks; ++idx)
    {
        /*-------------------------------------------------------------------------------------------------*/
        /* first byte command and follow 3 bytes address used as DATA                                                    */
        /*-------------------------------------------------------------------------------------------------*/
		FIU_UMA_Write(nor, data[0], (u32)NULL, false, &data[1], CHUNK_SIZE-1);
		
        data += CHUNK_SIZE;
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Handling chunk remains                                                                              */
    /*-----------------------------------------------------------------------------------------------------*/
    if (remain_data > 0)
    {
		FIU_UMA_Write(nor, data[0], (u32)NULL, false, &data[1], remain_data-1);
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Finally we de-activating the Chip select and returning to "automatic" CS control                    */
    /*-----------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_SW_CS, 1);

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                              SPI Functions                                              */
/*---------------------------------------------------------------------------------------------------------*/

void spi_flash_high_addr_wr(struct spi_nor *nor,u8 HighAddr)
{
    
    FIU_UMA_Write(nor, SPI_WRITE_ENABLE_CMD, 0, false, NULL, 0);
    
    FIU_UMA_Write(nor, SPI_WRITE_EXTENDED_ADDR_REG_CMD, 0, false, &HighAddr, sizeof(u8)); 

}

void SPI_Flash_Common_GetStatus(struct spi_nor *nor, u8* status)
{
    FIU_UMA_Read(nor, SPI_READ_STATUS_REG_CMD, 0, 0, status, 1);
}

void SPI_Flash_Common_SectorErase(struct spi_nor *nor, u32 addr)
{
    if ((addr >> 24)&&(nor->addr_width == 3)) 
    {
        spi_flash_high_addr_wr(nor,addr >> 24);
		
         FIU_UMA_Write(
             nor,                           // only one flash device
             SPI_WRITE_ENABLE_CMD,              // write enable transaction code
             0,                                 // address irrelevant
             false,                             // no address for transaction 
             NULL,                              // no write data
             0);                                // no data
      
         FIU_UMA_Write(
             nor,                               // only one flash device
             SPI_4K_SECTOR_ERASE_CMD,           // sector erase transaction code
             (addr & 0xFFFFFF),                 // address relevant
             true,                              // address for transaction 
             NULL,                              // no write data
             0);                                // no data
                             
         SPI_Flash_Common_WaitTillReady(nor);   

         spi_flash_high_addr_wr(nor,0);
    }
     else
    {
        FIU_UMA_Write(
            nor,                           // only one flash device
            SPI_WRITE_ENABLE_CMD,              // write enable transaction code
            0,                                 // address irrelevant
            false,                             // no address for transaction 
            NULL,                              // no write data
            0);                                // no data

        FIU_UMA_Write(
            nor,                           // only one flash device
            SPI_4K_SECTOR_ERASE_CMD,           // sector erase transaction code
            addr,                              // address relevant
            true,                              // address for transaction 
            NULL,                              // no write data
            0); 

        SPI_Flash_Common_WaitTillReady(nor);   
    }
}

void SPI_Flash_Common_Write(struct spi_nor *nor, u32 destAddr, u8* data, u32 size)
{
    if ((destAddr >> 24)&&(nor->addr_width == 3)) 
    {
        spi_flash_high_addr_wr(nor,destAddr >> 24);

        /*-----------------------------------------------------------------------------------------------------*/
        /* Write Flash Using 256 Page EXTENDED MODE                                                            */
        /*-----------------------------------------------------------------------------------------------------*/
        FIU_UMA_Write(nor, SPI_WRITE_ENABLE_CMD, 0, false, NULL, 0);
 
        FIU_ManualWrite(nor, SPI_PAGE_PRGM_CMD, (destAddr & 0xFFFFFF), data, size);

        SPI_Flash_Common_WaitTillReady(nor); 
          
        spi_flash_high_addr_wr(nor,0);
    }
    else
    {
        FIU_UMA_Write(nor, SPI_WRITE_ENABLE_CMD, 0, false, NULL, 0);
        
        FIU_ManualWrite(nor, SPI_PAGE_PRGM_CMD, destAddr, data, size);

        SPI_Flash_Common_WaitTillReady(nor);   
    }
}


void SPI_Flash_Common_WaitTillReady(struct spi_nor *nor) 
{
    u8 busy=1;

    do 
    {
        SPI_Flash_Common_GetStatus(nor, &busy);

        /* Keep only "busy" bit 0 */
        busy &= 0x01;

    } while (busy);

}


static void spi_flash_unlock_protection(struct spi_nor *nor)
{
    u8 status_reg_val=0;

    
    FIU_UMA_Write(nor, SPI_WRITE_ENABLE_CMD, 0, false, NULL, 0);
    
    FIU_UMA_Write(nor, SPI_WRITE_STATUS_REG_CMD, 0, false, &status_reg_val, sizeof(u8));

    SPI_Flash_Common_WaitTillReady(nor);
}


void SPI_Flash_WritePageAligned_L(struct spi_nor *nor, u8 *src, u32 addr, u32 cnt)
{
    u32  local_addr      = addr;
    u32  actual_size     = 0;
    struct mtd_info *mtd;

    mtd = &nor->mtd;


    if(cnt != 0)
    {
        while (cnt)
        {
            /*---------------------------------------------------------------------------------------------*/
            /* Calculating the size from current address to the start of the next page                     */
            /*---------------------------------------------------------------------------------------------*/
            actual_size = ((((local_addr)/mtd->writesize) + 1)*mtd->writesize) - (local_addr);

            /*---------------------------------------------------------------------------------------------*/
            /* If we smaller amount to write                                                               */
            /*---------------------------------------------------------------------------------------------*/
            if (actual_size > cnt)
                actual_size = cnt;

            /*---------------------------------------------------------------------------------------------*/
            /* Executing the Page Programming                                                              */
            /*---------------------------------------------------------------------------------------------*/
            SPI_Flash_Common_Write(nor, local_addr, src, actual_size);
            
            /*---------------------------------------------------------------------------------------------*/
            /* Updating loop variables                                                                     */
            /*---------------------------------------------------------------------------------------------*/
            src         += actual_size;
            local_addr  += actual_size;
            cnt         -= actual_size;
        }
    }
}

int SPI_Flash_Write(struct spi_nor *nor, u32 destAddr, u8* data, u32 size)
{  
    SPI_Flash_WritePageAligned_L(nor, data, destAddr, size);

    return 0;
}

#ifdef NPCMX50_MTD_SPINOR_DEBUG
static void dump_msg(const char *label, const unsigned char *buf, unsigned int length)
{
    unsigned int	start, num, i;
    char		line[52], *p;

    if (length > 32)
    {
        length = 32;
    }
    
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
        printk("%6x: %s\n", start, line);
        buf += num;
        start += num;
        length -= num;
    }
}
#endif

static int npcmx50_spinor_read(struct spi_nor *nor, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
    struct npcm750_chip *chip = nor->priv;
    struct npcm750_spi_bus *host = chip->host;   
    void __iomem  *flash_region_mapped_ptr=NULL;
    struct mtd_info *mtd;
    int i, readlen, currlen;
    u32 addr;
    u8 *buf_ptr;

	mtd = &nor->mtd;

    DEBUG_FLASH("mtd_spinor: %s %s 0x%08x, len %zd\n", __FUNCTION__, "from", (u32)from, len);

    /* sanity checks */
    if (!len)
    {
        return 0;
    }
    
    if (from + len > (u32)mtd->size)
    {
        return -EINVAL;
    }
    
    /* Byte count starts at zero. */
    if (retlen)
    {
        *retlen = 0;
    }
    
    DEBUG_FLASH("mtd_spinor: %s , mtd->size 0x%08x %p\n", __FUNCTION__, (u32) mtd->size, buf);


    if (host->Direct_Read) 
    {      
        if (!request_mem_region((host->res_mem->start + (host->MaxChipAddMap * chip->chipselect)) + from, len, mtd->name)) 
            return -EBUSY;
         
        flash_region_mapped_ptr = (u32 *)ioremap_nocache((host->res_mem->start + (host->MaxChipAddMap * chip->chipselect)) + from, len);

        if (!flash_region_mapped_ptr) 
        {
             printk(KERN_ERR ": Failed to ioremap window!\n");
             release_mem_region((host->res_mem->start + (host->MaxChipAddMap * chip->chipselect)) + from, len);
             return -ENOMEM;
        }

        if (mtd->size > SIZE_16MB) 
        {
            if (nor->addr_width == 3) 
            {
                SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num),  FIU_DRD_CFG_RDCMD, SPI_READ_DATA_DUAL_IO_3_ADDR_4_ADDR_CMD); 
            }
            else
            {
                SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num),  FIU_DRD_CFG_RDCMD, SPI_READ_DATA_DUAL_IO_4_ADDR_CMD); 
            }
            SET_REG_FIELD( FIU_DRD_CFG(host->fiu_num),  FIU_DRD_CFG_ADDSIZ, 0x1);
        }
        else
        {
            SET_REG_FIELD( FIU_DRD_CFG(host->fiu_num),  FIU_DRD_CFG_RDCMD, SPI_READ_DATA_DUAL_IO_3_ADDR_CMD);
            SET_REG_FIELD( FIU_DRD_CFG(host->fiu_num),  FIU_DRD_CFG_ADDSIZ, 0x0);
        }

        memcpy(buf, flash_region_mapped_ptr, len); 
        wmb();

        spi_flash_high_addr_wr(nor,0);
        if(flash_region_mapped_ptr)
        {
           iounmap((void __iomem *)flash_region_mapped_ptr);
        }
        release_mem_region((host->res_mem->start + (host->MaxChipAddMap * chip->chipselect)) + from, len);

        *retlen = len;
    }
    else
    {
        /* NOTE:  OPCODE_FAST_READ (if available) is faster... */
        i = 0;
        currlen = (int) len;

        do
        {
            addr = ((u32) from + i);
                      
            if (currlen < 4)
            {
                readlen = currlen;
            }
            else
            {
                readlen = 4;
            }

            DEBUG_FLASH("mtd_spinor: ori_addr=0x%x, addr=0x%x \n", ((u32) from + i), addr);

            buf_ptr = buf + i;

            if ((addr >> 24)&&(nor->addr_width == 3))
                spi_flash_high_addr_wr(nor,addr >> 24);

            FIU_UMA_Read(nor,                   // only one flash device
                         SPI_READ_DATA_CMD,     // read transaction code
                         (addr&0xFFFFFF),                  // address offset inside cs or device
                         true,                  // transaction has address 
                         buf_ptr,               // buffer to store read data
                         readlen);              // read data size

            if ((addr >> 24)&&(nor->addr_width == 3))
                spi_flash_high_addr_wr(nor,0);

            DEBUG_FLASH("mtd_spinor: buf_ptr=0x%x buf_val=0x%x i=%d readlen =%d \n",(u32)buf_ptr, *((u32 *)buf_ptr), i, readlen);
            
            i += readlen;
            currlen -= 4;	 
        }
        while (currlen > 0);

        *retlen = i;
    }

    DUMP_MSG("MTD_READ", buf, i);
    return 0;
}

static int npcmx50_spinor_write(struct spi_nor *nor, loff_t to, size_t len,
                            	size_t *retlen, const u8 *buf)
{    
    u32 addr = (u32) to;
    u32 cnt = (u32) len;
    struct mtd_info *mtd;
    int ret;
	
	mtd = &nor->mtd;

    DEBUG_FLASH("mtd_spinor: %s %s 0x%08x, len %zd\n", __FUNCTION__, "to", (u32)to, len);

    if (retlen)
    {
        *retlen = 0;
    }
    
    /* sanity checks */
    if (!len)
    {
        return(0);
    }
    
    if (to + len > mtd->size)
    {
        return -EINVAL;
    }
    
    if ((SPI_Flash_Write(nor, addr, (u8 *)buf, cnt)) == 0) 
    {
        *retlen = cnt;
		ret = 0;
    }
    else
    {
        *retlen = 0;
        ret = -EINVAL;
    }
 
    return ret;
}

static int npcmx50_spinor_erase(struct spi_nor *nor, loff_t offs)
{
    SPI_Flash_Common_SectorErase(nor, offs);
    return 0;
}


static ssize_t npcm750_spi_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *write_buf)
{
    u32 retlen;

    npcmx50_spinor_write(nor, to, len, &retlen, write_buf);

    return retlen;
}

static ssize_t npcm750_spi_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *read_buf)
{
    u32 retlen;

    npcmx50_spinor_read(nor, from, len, &retlen, read_buf);

    return retlen;
}

static int npcm750_spi_erase(struct spi_nor *nor, loff_t offs)
{
    return npcmx50_spinor_erase(nor, offs); 
}

static int npcm750_spi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret;    

    ret = FIU_UMA_Read(nor, opcode, 0, 0, buf, len); 

	return ret;
}

static int npcm750_spi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret;

    ret = FIU_UMA_Write(nor, opcode, (u32)NULL, false, buf, len);

	return ret;
}

static int npcm750_spi_nor_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
    struct npcm750_chip *chip = nor->priv;
    struct npcm750_spi_bus *host = chip->host;

    mutex_lock(&host->lock);

    return 0;
}

static void npcm750_spi_nor_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
    struct npcm750_chip *chip = nor->priv;
    struct npcm750_spi_bus *host = chip->host;

    mutex_unlock(&host->lock);
}

/**
 * Get spi flash device information and register it as a mtd device.
 */
static int npcm750_spi_nor_register(struct device_node *np,
				struct npcm750_spi_bus *host)
{
	struct device *dev = host->dev;
	struct spi_nor *nor;
	struct npcm750_chip *chip;
	struct mtd_info *mtd;
    u32 chipselect;
	int ret;
    u8 status_reg_val;
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

    /* This driver does not support NAND or NOR flash devices. */
    if (!of_device_is_compatible(np, "jedec,spi-nor"))
    {
        dev_err(dev, "The device is no compatible to jedec,spi-nor\n");
        return -ENOMEM;
    }

    ret = of_property_read_u32(np, "reg", &chipselect);
    if (ret) {
        dev_err(dev, "There's no reg property for %s\n",
            np->full_name);
        return ret;
    }

    if (chipselect >= NPCM750_MAX_CHIP_NUM) 
    {
        dev_warn(dev, "Flash device number exceeds the maximum chipselect number\n");
        return -ENOMEM;
    }

    chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    chip->host = host;
    chip->chipselect = chipselect;

    nor = &chip->nor;
    mtd = &nor->mtd;

	nor->dev = dev;
    nor->priv = chip;

	spi_nor_set_flash_node(nor, np);

	/*ret = of_property_read_u32(np, "spi-max-frequency",
			&chip->clkrate);
	if (ret) {
		dev_err(dev, "There's no spi-max-frequency property for %s\n",
			np->full_name);
		return ret;
	}*/

    nor->prepare = npcm750_spi_nor_prep;
    nor->unprepare = npcm750_spi_nor_unprep;

    nor->read_reg = npcm750_spi_read_reg;
    nor->write_reg = npcm750_spi_write_reg;
    nor->read = npcm750_spi_read;
    nor->write = npcm750_spi_write;
    nor->erase = npcm750_spi_erase;

	ret = spi_nor_scan(nor, NULL, &hwcaps);
	if (ret)
		return ret;

    if (mtd->size > SIZE_16MB) 
    {
        FIU_UMA_Read(nor, SPI_READ_STATUS_3_REG_CMD, 0, false, &status_reg_val, sizeof(u8)); 
        if (status_reg_val & 0x1) 
        {
            SPI_Flash_Common_WaitTillReady(nor);

            FIU_UMA_Write(nor, SPI_ENABLE_RESET_CMD, 0, false, NULL, 0);
            FIU_UMA_Write(nor, SPI_RESET_DEVICE_CMD, 0, false, NULL, 0);

            SPI_Flash_Common_WaitTillReady(nor);
        }
        
        FIU_UMA_Read(nor, SPI_READ_STATUS_3_REG_CMD, 0, false, &status_reg_val, sizeof(u8));
        if ((status_reg_val & 0x1) == 0x0)
        {
            nor->addr_width = 3;
        }
    }

    spi_flash_unlock_protection(nor);

	//mtd->name = np->name;
	ret = mtd_device_register(mtd, NULL, 0);
	if (ret)
		return ret;

    host->chip[chip->chipselect] = chip;

	return 0;
}

static void npcm750_spi_nor_unregister_all(struct npcm750_spi_bus *host)
{
	struct npcm750_chip *chip;
    int n;

    for (n = 0; n < NPCM750_MAX_CHIP_NUM; n++) {
        chip = host->chip[n];
        if (chip)
            mtd_device_unregister(&chip->nor.mtd);
    }
}

static int npcm750_spi_nor_register_all(struct npcm750_spi_bus *host)
{
	struct device *dev = host->dev;
	struct device_node *np;
	int ret;

	for_each_available_child_of_node(dev->of_node, np) {
		ret = npcm750_spi_nor_register(np, host);
		if (ret)
        {
			goto fail;
        }
	}

	return 0;

fail:
	npcm750_spi_nor_unregister_all(host);
	return ret;
}

static int npcm750_spi_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct resource *res;
    struct npcm750_spi_bus *host;    
    static u32 fiu_num = 0;
    int ret;

    host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
    if (!host)
            return -ENOMEM;

    platform_set_drvdata(pdev, host);
    host->dev = dev;

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "control");
    host->regbase = devm_ioremap_resource(dev, res);
    if (IS_ERR(host->regbase))
            return PTR_ERR(host->regbase);

    host->res_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "memory");
    if (host->res_mem) 
    {
        host->Direct_Read = true;
    }
    else
    {        
        host->Direct_Read = false;
    }

    host->clk = devm_clk_get(dev, NULL);
    if (IS_ERR(host->clk))
            return PTR_ERR(host->clk);

    ret = of_property_read_u32(dev->of_node, "chip-max-address-map",
            &host->MaxChipAddMap);
    if (ret) {
        dev_err(dev, "There's no chip-max-address-map property for %s adjust 128Mb\n",dev->of_node->full_name);
        host->MaxChipAddMap = 0x8000000; 
    }

    mutex_init(&host->lock);
    //clk_prepare_enable(host->clk);
    fiu_base[fiu_num] = host->regbase;
    host->fiu_num = fiu_num;

    ret = npcm750_spi_nor_register_all(host);
    if (ret)
    {
        mutex_destroy(&host->lock);
        //clk_disable_unprepare(host->clk);
        printk(KERN_INFO "npcm750_spi_nor_register_all failed\n\n\n");
    }

    fiu_num++;

    DEBUG_FLASH("mtd_spinor: %s() date=%s time=%s\n\n", __FUNCTION__,  __DATE__, __TIME__);

    return ret;
}

static int npcm750_spi_remove(struct platform_device *pdev)
{
    struct npcm750_spi_bus *host = platform_get_drvdata(pdev);

    npcm750_spi_nor_unregister_all(host);
    mutex_destroy(&host->lock);
    clk_disable_unprepare(host->clk);
    return 0;
}

static const struct of_device_id npcm750_spi_dt_ids[] = {
	{ .compatible = "nuvoton,npcm750-spi" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, npcm750_spi_dt_ids);

static struct platform_driver npcm750_spi_driver = {
	.driver = {
		.name	= "npcm750-spi",
		.bus	= &platform_bus_type,
		.of_match_table = npcm750_spi_dt_ids,
	},
	.probe      = npcm750_spi_probe,
	.remove	    = npcm750_spi_remove,
};
module_platform_driver(npcm750_spi_driver);

MODULE_DESCRIPTION("Nuvoton SPI Controller Driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
