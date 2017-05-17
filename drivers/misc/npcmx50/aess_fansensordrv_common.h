/*
 *
 * Copyright (C) 2009-2015 Avocent Corporation
 *
 * This file is subject to the terms and conditions of the GNU
 * General Public License Version 2. This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License Version 2 for more details.
 *
 *----------------------------------------------------------------------------\n
 *  MODULES     FanTach sensor driver\n
 *----------------------------------------------------------------------------\n
 * @file    aess_fansensordrv_common.h
 * @brief   fantach sensor driver header
 *
 * @ internal
 *---------------------------------------------------------------------------- */

#if !defined (AESSFANSENSORDRV_COMMON_H)
#define AESSFANSENSORDRV_COMMON_H

#define FANSENSOR_DRIVER_NAME "aess_fansensordrv"  /**< Define fansensor driver name. */

/******************************************************************************
*   STRUCT      :   sFanTachData
******************************************************************************/
/**
 *  @brief   Structure to FanSensor driver related data parameter.
 *
 *****************************************************************************/
typedef struct
{
    /** Set Fan Channel no. (zero-based) */
    UINT8 u8ChannelNum;

    /** Pulse Revolution */
    UINT8 u8FanPulsePerRev;

    /** Fan Speed Reading (unit: ticks) */
    UINT16 u16FanSpeedReading;

    /** Input Clock */
    UINT32 u32InputClock;
} sFanTachData;

/** Fan sensor driver ioctl magic number.  */
#define AESS_FANSENSORDRV_IOC_MAGIC     0xBD
/** Fan sensor ioctl command, used to configure fan tachometer. */
#define AESS_FANTACH_CONFIG             _IOWR(AESS_FANSENSORDRV_IOC_MAGIC, 0, sFanTachData)
/** Fan sensor ioctl command, used to read speed ticks. */
#define AESS_FANTACH_READ               _IOWR(AESS_FANSENSORDRV_IOC_MAGIC, 1, sFanTachData)


#endif   /* AESSFANSENSORDRV_COMMON_H */

/* End of code */
