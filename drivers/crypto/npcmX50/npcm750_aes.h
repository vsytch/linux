/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2014-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   npcm750_aes.h                                                                                         */
/*            This file contains Advanced Encryption Standard (AES) interface                              */
/* Project:                                                                                                */
/*            SWC HAL                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef NPCM750_AES_H
#define NPCM750_AES_H



#include <crypto/internal/hash.h>



/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                                 DEFINES                                                 */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                           Registers definition                                          */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/**************************************************************************************************************************/
/*   Key 0 Register (AES_KEY_0)                                                                                           */
/**************************************************************************************************************************/
#define  AES_KEY_0                      (aes_base + 0x400) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+400h */

/**************************************************************************************************************************/
/*   Initialization Vector 0 Register (AES_IV_0)                                                                          */
/**************************************************************************************************************************/
#define  AES_IV_0                       (aes_base + 0x440) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+440h */
#define  AES_IV_0_AES_IV_0               0 , 32             /* 31-0 AES_IV_0. Initialization Vector 0.                                                                               */

/**************************************************************************************************************************/
/*   Initial Counter Value Register (AES_CTR0)                                                                            */
/**************************************************************************************************************************/
#define  AES_CTR0                       (aes_base + 0x460) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+460h */


//#define AES_CTR_ADDR            REG_ADDR(AES_CTR0)   /* 16 byte   */
//#define AES_CTR                 ((PTR32)(AES_CTR_ADDR))


/**************************************************************************************************************************/
/*   Busy Register (AES_BUSY)                                                                                             */
/**************************************************************************************************************************/
#define  AES_BUSY                       (aes_base + 0x470) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+470h */
#define  AES_BUSY_AES_BUSY               0 , 1              /* 0 AES_BUSY. Reading checks the engine’s internal status. Writing switches from configuration mode to data */
#define  AES_BUSY_NOT_BUSY               0                  /* 0 On read:  AES not busy status.                                                                            */
#define  AES_BUSY_IS_BUSY                1                  /* 1 On read:  AES busy status.                                                                                */
#define  AES_BUSY_Configuration          0                  /* 0 On write: Configuration mode.                                                                             */
#define  AES_BUSY_Data                   1                  /* 1 On write: Data processing mode.                                                                           */

/**************************************************************************************************************************/
/*   Sample Key Register (AES_SK)                                                                                         */
/**************************************************************************************************************************/
#define  AES_SK                         (aes_base + 0x478) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+478h */
#define  AES_SK_AES_SK                   0 , 1               /* ???                                                                                             */

/**************************************************************************************************************************/
/*   Previous IV 0 Value Register (AES_PREV_IV_0)                                                                         */
/**************************************************************************************************************************/
#define  AES_PREV_IV_0                  (aes_base + 0x490) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+490h */
#define  AES_PREV_IV_0_AES_PREV_IV_0     0 , 32             /* 31-0 AES_PREV_IV_0. The previous IV 0 value (for XCBC support).                                                       */

/**************************************************************************************************************************/
/*   Direct Data Access for Register (AES_DIN_DOUT)                                                                       */
/**************************************************************************************************************************/
#define  AES_DIN_DOUT                   (aes_base + 0x4A0) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+4A0h */

/**************************************************************************************************************************/
/*   Control Register (AES_CONTROL)                                                                                       */
/**************************************************************************************************************************/
#define  AES_CONTROL                    (aes_base + 0x4C0) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+4C0h */
#define  AES_CONTROL_WORD_HIGH          (aes_base + 0x4C2) , NPCMX50_AES_ACCESS, 16        /* Location: AES_BA+4C2h */
#define  AES_CONTROL_DIRECT_ACC_N_DIN_DOUT  31 , 1              /* 31 DIRECT_ACC_N_DIN_DOUT. This bit should be set to 1 for direct data read/write from/to AES_DIN_DOUT.                */
#define  AES_CONTROL_NK_KEY0                12 , 2             /* 13-12 NK_KEY0.                                                                                                        */
#define  AES_CONTROL_MODE_KEY0              2 , 3              /* 4-2 MODE_KEY0.                                                                                                        */
#define  AES_CONTROL_DEC_ENC                0 , 1               /* 0 DEC_KEY0.                                                                                                           */

/**************************************************************************************************************************/
/*   Version Register (AES_VERSION)                                                                                       */
/**************************************************************************************************************************/
#define  AES_VERSION                    (aes_base + 0x4C4) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+4C4h */
#define  AES_VERSION_MAJOR_VERSION_NUMBER  12 , 4             /* 15-12 MAJOR_VERSION_NUMBER.                                                                                           */
#define  AES_VERSION_MINOR_VERSION_NUMBER  8 , 4              /* 11-8 MINOR_VERSION_NUMBER.                                                                                            */
#define  AES_VERSION_FIXES               0 , 8              /* 7-0 FIXES.                                                                                                            */

/**************************************************************************************************************************/
/*   Hardware Flag Register (AES_HW_FLAGS)                                                                                */
/**************************************************************************************************************************/
#define  AES_HW_FLAGS                   (aes_base + 0x4C8) , NPCMX50_AES_ACCESS, 32        /* Location: Base+4C8h */
#define  AES_HW_FLAGS_DFA_CNTRMSR_EXIST  12 , 1              /* 12 DFA_CNTRMSR_EXIST.                                                                                                 */
#define  AES_HW_FLAGS_SECOND_REGS_SET_EXISTS  11 , 1              /* 11 SECOND_REGS_SET_EXISTS.                                                                                            */
#define  AES_HW_FLAGS_TUNNELING_ENB      10 , 1              /* 10 TUNNELING_ENB.                                                                                                     */
#define  AES_HW_FLAGS_AES_SUPPORT_PREV_IV  9 , 1               /* 9 AES_SUPPORT_PREV_IV.                                                                                                */
#define  AES_HW_FLAGS_USE_5_SBOXES       8 , 1               /* 8 USE_5_SBOXES.                                                                                                       */
#define  AES_HW_FLAGS_USE_SBOX_TABLE     5 , 1               /* 5 USE_SBOX_TABLE.                                                                                                     */
#define  AES_HW_FLAGS_ONLY_ENCRYPT       4 , 1               /* 4 ONLY_ENCRYPT.                                                                                                       */
#define  AES_HW_FLAGS_CTR_EXIST          3 , 1               /* 3 CTR_EXIST.                                                                                                          */
#define  AES_HW_FLAGS_DPA_CNTRMSR_EXIST  2 , 1               /* 2 DPA_CNTRMSR_EXIST.                                                                                                  */
#define  AES_HW_FLAGS_AES_LARGE_RKEK     1 , 1               /* 1 AES_LARGE_RKEK.                                                                                                     */
#define  AES_HW_FLAGS_SUPPORT_256_192_KEY  0 , 1               /* 0 SUPPORT_256_192_KEY.                                                                                                */

/**************************************************************************************************************************/
/*   Software Reset Register (AES_SW_RESET)                                                                               */
/**************************************************************************************************************************/
#define  AES_SW_RESET                   (aes_base + 0x4F4) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+4F4h */
#define  AES_SW_RESET_AES_SW_RESET       0 , 1               /* 0 AES_SW_RESET. Writing to this register generates a Software reset to the AES block.                                 */

/**************************************************************************************************************************/
/*   DFA Error Status Register (AES_DFA_ERROR_STATUS)                                                                     */
/**************************************************************************************************************************/
#define  AES_DFA_ERROR_STATUS           (aes_base + 0x4F8) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+4F8h */
#define  AES_DFA_ERROR_STATUS_AES_DFA_ERROR_STATUS  0 , 1               /* 0 AES_DFA_ERROR_STATUS. After a DFA violation, this register is set and the AES block is disabled until the           */

/**************************************************************************************************************************/
/*   RBG Seeding Ready Register (AES_RBG_SEEDING_READY)                                                                   */
/**************************************************************************************************************************/
#define  AES_RBG_SEEDING_READY          (aes_base + 0x4FC) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+4FCh */
#define  AES_RBG_SEEDING_READY_AES_RBG_SEEDING_READY  0 , 1               /* 0 AES_RBG_SEEDING_READY. RGB seeding should be performed only when this register is set to 1.                         */

/**************************************************************************************************************************/
/*   AES FIFO Data Register (AES_FIFO_DATA)                                                                               */
/**************************************************************************************************************************/
#define  AES_FIFO_DATA                  (aes_base + 0x500) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+500h */
#define  AES_FIFO_DATA_AES_FIFO_DATA     0 , 32             /* 31-0 AES_FIFO_DATA.                                                                                                   */

/**************************************************************************************************************************/
/*   AES FIFO Status Register (AES_FIFO_STATUS)                                                                           */
/**************************************************************************************************************************/
#define  AES_FIFO_STATUS                (aes_base + 0x600) , NPCMX50_AES_ACCESS, 32        /* Location: AES_BA+600h */
#define  AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW  5 , 1               /* 5 DOUT_FIFO_UNDERFLOW.                                                                                                */
#define  AES_FIFO_STATUS_DIN_FIFO_OVERFLOW  4 , 1               /* 4 DIN_FIFO_OVERFLOW.                                                                                                  */
#define  AES_FIFO_STATUS_DOUT_FIFO_EMPTY  3 , 1               /* 3 DOUT_FIFO_EMPTY.                                                                                                    */
#define  AES_FIFO_STATUS_DOUT_FIFO_FULL  2 , 1               /* 2 DOUT_FIFO_FULL.                                                                                                     */
#define  AES_FIFO_STATUS_DIN_FIFO_EMPTY  1 , 1               /* 1 DIN_FIFO_EMPTY.                                                                                                     */
#define  AES_FIFO_STATUS_DIN_FIFO_FULL   0 , 1               /* 0 DIN_FIFO_FULL.                                                                                                      */





/*---------------------------------------------------------------------------------------------------------*/
/* AES Mode                                                                                                */
/* Do no change order, hardware dependent                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCMX50_AES_MODE_T
{
    AES_MODE_ECB = 0,   /* Electronic Codebook          */
    AES_MODE_CBC = 1,   /* Cipher Block Chaining        */
    AES_MODE_CTR = 2,   /* Counter                      */
    AES_MODE_MAC = 3    /* Message Authentication Code  */
} NPCMX50_AES_MODE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* AES Key Size                                                                                            */
/* Do no change order, hardware dependent                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCMX50_AES_KEY_SIZE_T
{
    AES_KEY_SIZE_128  = 0 ,
    AES_KEY_SIZE_192  = 1 ,
    AES_KEY_SIZE_256  = 2 ,
    AES_KEY_SIZE_USE_LAST = 0xFF  // AES Key is preloaed by npcmx50_aes_set_key. 
} NPCMX50_AES_KEY_SIZE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* AES Operation                                                                                           */
/* Do not change the order, hardware dependent                                                             */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCMX50_AES_OP_T
{
    AES_OP_ENCRYPT = 0,
    AES_OP_DECRYPT = 1
} NPCMX50_AES_OP_T;


/*---------------------------------------------------------------------------------------------------------*/
/* AES module macro definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
/* The bit length of supported keys for the AES algorithm */
#define AES_KEY_BIT_SIZE(size)      (128 + (NPCMX50_AES_KEY_SIZE_T)(size) * 64)

/* The byte length of supported keys for the AES algorithm */
#define AES_KEY_BYTE_SIZE(size)     (AES_KEY_BIT_SIZE(size) / 8)

/* The bit length of supported keys for the AES algorithm */
#define NPCMX50_AES_KEY_SIZE_TO_ENUM(size)  ((size==32) ? AES_KEY_SIZE_256 : ((size==24) ? AES_KEY_SIZE_192 : AES_KEY_SIZE_128))

/* # of bytes needed to represent a key */
// #define AES_MAX_KEY_SIZE            AES_KEY_BYTE_SIZE(AES_KEY_SIZE_256)

/* The byte length of a block for the AES algorithm (b = 128 bit) */
// #define AES_BLOCK_SIZE              AES_KEY_BYTE_SIZE(AES_KEY_SIZE_128)

/* # of bytes needed to represent an IV  */
#define AES_MAX_IV_SIZE             (AES_BLOCK_SIZE)

/* # of bytes needed to represent a counter  */
#define AES_MAX_CTR_SIZE            (AES_BLOCK_SIZE)

/* Calculate the number of blocks in the formatted message */
#define AES_COMPLETE_BLOCKS(size)   (((size) + (AES_BLOCK_SIZE - 1)) / AES_BLOCK_SIZE)

#define AES_BLOCK_MASK	(~(AES_BLOCK_SIZE-1))

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                           INTERFACE FUNCTIONS                                           */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/




#endif //NPCM750_AES_H