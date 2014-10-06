/*-----------------------------------------------------------------------------
|  Project :  CSST
+------------------------------------------------------------------------------
|             Copyright 2005 Texas Instruments.
|             All rights reserved.
|
|             This file is confidential and a trade secret of Texas
|             Instruments .
|             The receipt of or possession of this file does not convey
|             any rights to reproduce or disclose its contents or to
|             manufacture, use, or sell anything it may describe, in
|             whole, or in part, without the specific written consent of
|             Texas Instruments.
+------------------------------------------------------------------------------
| Filename: i2c_drv.h
| Author : PROCSYS
|Purpose: This is the h-file for the i2c Driver (i2c_drv.c)
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL =================================================*/
#ifndef CSST_I2C_DRV_H
#define CSST_I2C_DRV_H

/*==== INCLUDES ============================================================*/

#include "i2c_dis.h"

/*==== DEFINES ============================================================*/
#define I2C_MAX_INSTANCES	3

/*==== TYPES ===============================================================*/


/*Function Prototypes */
S32 i2c_init(const void * init_str, U32 * dis_addr); /*Initializes I2C*/
S32 i2c_read(U32 dis_addr, U32 key, U32 *len,U8 *buf); /*Read from the I2C*/
S32 i2c_write(U32 dis_addr, U32 key, U32 *len, U8 *buf); /*Write to the I2C*/
S32 i2c_deinit(U32 dis_addr);                        /*deinitialize*/

typedef struct
{
    T_I2C_INIT_STRUCTURE i2c_initstr; /* init structure*/
                                      /*Add extra members here*/
} T_I2C_DIS;
/* this structure is used in the device driver*/

#endif /* CSST_I2C_DRV_H */
