#ifndef __NPCM_REG_H__
#define __NPCM_REG_H__

#define DRV_MSG2(x...)

#define regdef(flags, offset)	 offset
#define regbitdef(flags, bitpos) BIT(bitpos)

#define gcrreg(reg, shift, bits) ((GCR_ ## reg) << 8 | (bits << 5) | (shift))

/*
 * GCR module
 */
#define GCR_BA		0xF0800000
#define GCR_BANKSIZE	0x1000

#define GCR_PDID	regdef(ro, 0x00)

#define GCR_MFSEL1	regdef(rw, 0x0c)
#define GCR_MFSEL2	regdef(rw, 0x10)
#define GCR_MFSEL3	regdef(rw, 0x64)
#define GCR_MFSEL4	regdef(rw, 0xb0)

#define GCR_CPCTL	regdef(xx,   0xD0)
#define GCR_CP2BST	regdef(rw1c, 0xD4)
#define GCR_B2CPNT	regdef(rw1s, 0xD8)

#define GCR_I2CSEGSEL	regdef(rw, 0xE0)

#define GCR_I2CSEGCTL	regdef(rw, 0xE4)
#define	  SMBXX_BITS	2
#define	  SMB0SS_SHIFT	0
#define	  SMB1SS_SHIFT	2
#define	  SMB2SS_SHIFT	4
#define	  SMB3SS_SHIFT	6
#define	  SMB4SS_SHIFT	8
#define	  SMB5SS_SHIFT	10
#define	  WEN0_SS	BIT(12)
#define	  WEN1_SS	BIT(13)
#define	  WEN2_SS	BIT(14)
#define	  WEN3_SS	BIT(15)
#define	  WEN4_SS	BIT(16)
#define	  WEN5_SS	BIT(17)

#define GCR_SRCNT	regdef(rw, 0x68)
#define SRCNT_ESPI	   BIT(3)
/* SPI0D = 1:1
 * SPI0C = 2:1
 * ESPI	 = 3:1
 * TDO	 = 4:1
 */
#define GCR_FLOCKR1	regdef(xx, 0x74)

#define GCR_DSCNT	regdef(rw, 0x78)
/* SPI0D = 1:1 (8,12)
 * SPI0C = 2:1 (8,12)
 * SYNC1 = 3:1 (4,8)
 * ESPI	 = 6:2 (8,12,16,24)
 * SPLD	 = 9:1 (2,4)
 */

/*
 * GPIO module
 */
#define GPIO_BA		0xF0010000
#define GPIO_BANKSIZE	0x1000
#define GPIO_PER_BANK	32
#define GPIO_NBANKS	8

#define GPnTLOCK1	regdef(rw, 0x00)
#define GPnDIN		regdef(ro, 0x04) /* Data IN */
#define GPnPOL		regdef(rw, 0x08) /* Polarity */
#define GPnDOUT		regdef(rw, 0x0c) /* Data OUT */

#define GPnOE		regdef(rw, 0x10) /* Output Enable */
#define GPnOTYP		regdef(rw, 0x14)
#define GPnMP		regdef(rw, 0x18)
#define GPnPU		regdef(rw, 0x1c) /* Pull-up */

#define GPnPD		regdef(rw, 0x20) /* Pull-down */
#define GPnDBNC		regdef(rw, 0x24) /* Debounce */
#define GPnEVTYP	regdef(rw, 0x28) /* Event Type */
#define GPnEVBE		regdef(rw, 0x2c) /* Event Both Edge */

#define GPnOBL0		regdef(rw, 0x30)
#define GPnOBL1		regdef(rw, 0x34)
#define GPnOBL2		regdef(rw, 0x38)
#define GPnOBL3		regdef(rw, 0x3c)

#define GPnEVEN		regdef(rw, 0x40) /* Event Enable */
#define GPnEVENS	regdef(wo, 0x44) /* Event Set (enable) */
#define GPnEVENC	regdef(wo, 0x48) /* Event Clear (disable) */
#define GPnEVST		regdef(rc, 0x4c) /* Event Status */

#define GPnSPLCK	regdef(rs, 0x50)
#define GPnMPLCK	regdef(rs, 0x54)
#define GPnIEM		regdef(rw, 0x58) /* Input Enable */
#define GPnOSRC		regdef(rw, 0x5c)

#define GPnODSC		regdef(rw, 0x60)
#define GPnDOS		regdef(wo, 0x68) /* Data OUT Set */
#define GPnDOC		regdef(wo, 0x6c) /* Data OUT Clear */

#define GPnOES		regdef(wo, 0x70) /* Output Enable Set */
#define GPnOEC		regdef(wo, 0x74) /* Output Enable Clear */
#define GPnTLOCK2	regdef(w,  0x7c)

/*
 * I2C module
 */
#define SMB_INTERRUPT  (64+32)
#define SMB_BA		0xF0080000
#define SMB_BANKSIZE	0x1000
#define SMB_NBANKS	16

#define SMBnSDA		regdef(rw, 0x00)
#define SMBnST		regdef(xx, 0x02)
#define	 ST_SLVSTP	   regbitdef(rw1c, 7)
#define	 ST_SDAST	   regbitdef(ro,   6)
#define	 ST_BER		   regbitdef(rw1c, 5)
#define	 ST_NEGACK	   regbitdef(rw1c, 4)
#define	 ST_STASTR	   regbitdef(rw1c, 3)
#define	 ST_NMATCH	   regbitdef(rw1c, 2)
#define	 ST_MASTER	   regbitdef(ro,   1)
#define	 ST_XMIT	   regbitdef(ro,   0)
#define SMBnCST		regdef(xx, 0x04)
#define	 CST_ARPMATCH	   regbitdef(ro,   7)
#define	 CST_MATCHAF	   regbitdef(ro,   6)
#define	 CST_TGSCL	   regbitdef(rw1s, 5)
#define	 CST_TDSDA	   regbitdef(ro,   4)
#define	 CST_GCMATCH	   regbitdef(ro,   3)
#define	 CST_MATCH	   regbitdef(ro,   2)
#define	 CST_BB		   regbitdef(rw1c, 1)
#define	 CST_BUSY	   regbitdef(ro,   0)
#define SMBnCTL1	regdef(xx, 0x06)
#define	 CTL_STASTRE	   regbitdef(rw,   7)
#define	 CTL_NMINTE	   regbitdef(rw,   6)
#define	 CTL_GCMEN	   regbitdef(rw,   5)
#define	 CTL_ACK	   regbitdef(rw,   4)
#define	 CTL_EOBINTE	   regbitdef(rw,   3)
#define	 CTL_INTEN	   regbitdef(rw,   2)
#define	 CTL_STOP	   regbitdef(rw1s, 1)
#define	 CTL_START	   regbitdef(rw1s, 0)
#define SMBnADDR1	regdef(rw, 0x08)
#define SMBnCTL2	regdef(rw, 0x0a)
#define	 CTL_SCLFREQ06	   regbitdef(rw,   1)
#define	 CTL_ENABLE	   regbitdef(rw,   0)
#define SMBnADDR2	regdef(rw, 0x0c)
#define SMBnCTL3	regdef(xx, 0x0e)
#define SMBnADDR3	regdef(rw, 0x10)
#define SMBnADDR7	regdef(rw, 0x11)
#define SMBnADDR4	regdef(rw, 0x12)
#define SMBnADDR8	regdef(rw, 0x13)
#define SMBnADDR5	regdef(rw, 0x14)
#define SMBnADDR6	regdef(rw, 0x16)
#define SMBnCST2	regdef(ro, 0x18)
#define SMBnCST3	regdef(xx, 0x19)
#define SMBnCTL4	regdef(rw, 0x1a)
#define SMBnCTL5	regdef(rw, 0x1b)
#define SMBnSCLLT	regdef(rw, 0x1c)
#define SMBnFIF_CTL	regdef(xx, 0x1d)
#define SMBnSCLHT	regdef(rw, 0x1e)

/* Structure for register banks */
struct npcm_reg {
	u8		*base;
	int		irqbase;
	int		irq;
	spinlock_t	lock;
	void		*priv;
};

static inline u8 npcm_read8(const void __iomem *addr)
{
	return *(const u8 *)addr;
}

static inline void npcm_write8(void __iomem *addr, u8 val)
{
	*(u8 *)addr = val;
}

static inline void npcm_setbit8(void __iomem *addr, u8 mask)
{
	*(u8 *)addr |= mask;
}

static inline void npcm_clrbit8(void __iomem *addr, u8 mask)
{
	*(u8 *)addr &= ~mask;
}

static inline u16 npcm_read16(const void __iomem *addr)
{
	return *(const u16 *)addr;
}

static inline void npcm_write16(void __iomem *addr, u16 val)
{
	*(u16 *)addr = val;
}

static inline void npcm_setbit16(void __iomem *addr, u16 mask)
{
	*(u16 *)addr |= mask;
}

static inline void npcm_clrbit16(void __iomem *addr, u16 mask)
{
	*(u16 *)addr &= ~mask;
}

static inline u32 npcm_read32(const void __iomem *addr)
{
	return *(const u32 *)addr;
}

static inline void npcm_write32(void __iomem *addr, u32 val)
{
	*(u32 *)addr = val;
}

static inline void npcm_setbit32(void __iomem *addr, u32 mask)
{
	*(u32 *)addr |= mask;
}

static inline void npcm_clrbit32(void __iomem *addr, u32 mask)
{
	*(u32 *)addr &= ~mask;
}

/* Read-modify-Write register */
static inline void npcm_rmw32(void __iomem *addr, int shift, int bits, u32 nv)
{
	u32 v, mask = (1L << bits)-1;

	v = npcm_read32(addr) & ~(mask << shift);
	npcm_write32(addr, v | ((nv & mask) << shift));
}

static inline void npcm_rmw8(void __iomem *addr, int shift, int bits, u8 nv)
{
	u8 v, mask = (1L << bits)-1;

	v = npcm_read32(addr) & ~(mask << shift);
	npcm_write32(addr, v | ((nv & mask) << shift));
}
#endif
