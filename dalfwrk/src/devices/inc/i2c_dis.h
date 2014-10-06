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
| Filename: i2c_dis.h
| Author: PROCSYS
| Purpose: Header file with general I2C definitions for the
|          Device Information Structure (DIS) and i2c initialisation structure
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef CSST_I2C_DIS_H
#define CSST_I2C_DIS_H

/*==== INCLUDES =============================================================*/
#include "csst_tgt.h"

/*=====DEFINES===============================================================*/
#define SID_I2C1 			0
#define SID_I2C2			1
#define SID_I2C3			2


#define I2C_100K_CLK		1 /* Stardard mode  - 100 kbits/s */
#define I2C_400K_CLK        2 /* Fast mode  - 400 kbits/s */
#define I2C_1P95M_CLK    	3 /* High Speed - 1.95 Mbits/s */
#define I2C_2P6M_CLK    	4 /* High Speed - 2.6 Mbits/s */
#define I2C_3P4M_CLK    	5 /* High Speed - 3.4 Mbits/s */
/* OLD IMPLEMENTATION
#define I2C_TAG_BASE 0
#define I2C_PID_KEY			I2C_TAG_BASE		 //Primary ID Key
#define I2C_SID_KEY			(I2C_TAG_BASE + 4)	 //Second ID Key
#define I2C_SLAVE_ADDR		(I2C_TAG_BASE + 8)     //Slave Address
#define I2C_CLOCK_SPEED		(I2C_TAG_BASE + 9)     //Clock Setting
#define I2C_NOADDRESS_DATA	(I2C_TAG_BASE + 12)
#define I2C_DATA			(I2C_TAG_BASE + 16) // Data Fifo for reading and writing
*/

#define I2C_1_BYTE_ADDRESS 1
#define I2C_2_BYTE_ADDRESS 2

/*NEW IMPLEMENTATION: flags in "key"/"reg" in bits 16..31*/
#define I2C_PID_KEY                                 0x80000000 //Flag for Primary ID - bit 31
#define I2C_SID_KEY                                 0x40000000//Flag for Secondary ID - bit 30
#define I2C_SLAVE_ADDR                          0x20000000//Flag for Slave Address - bit 29
#define I2C_CLOCK_SPEED                         0x10000000//Flag for Clock Setting n- bit 28
#define I2C_NOADDRESS_DATA                 0x08000000//Flag for noaddress data - bit 27
#define I2C_DATA                                      0x04000000//Flag for data transfer - single byte subaddress, one byte data transfered per i2c operation - bit 26
#define I2C_DATA_USING_FIFO                0x02000000//Flag for data transfer using buffered data (FIFO) - bit 25
/*Bit 24 below can be set together with the two previous flags: I2C_DATA and I2C_DATA_USING_FIFO*/
#define I2C_USING_2BYTE_SUBADDRESS  0x01000000//Flag for using I2C 2-byte subaddress - bit 24

/*Macro to give offset of the read or write*/
/*#define I2C_OFFSET(x) (I2C_DATA + (x))//OLD IMPLEMENTATION */
#define I2C_OFFSET(X) (I2C_DATA | (X))  /*Offset stays in the lower 16 bits -> flag is set at bit 26*/
#define I2C_OFFSET_USING_FIFO(X) (I2C_DATA_USING_FIFO|(X))
#define I2C_USE_2BYTE_SUBADDRESS(X) (I2C_USING_2BYTE_SUBADDRESS | (X))
/*==== CONSTS ==============================================================*/

/*==== TYPES ===============================================================*/

/*==== EXPORTS =============================================================*/
/*Initialization structure*/

typedef struct
{
    U16 pid;							/* Primary ID */
    U16 sid;                            /* Second ID */
    U8 slave_addr;                      /* Slave Address*/
    U16 clock_speed;                    /* Clock Speed*/
} T_I2C_INIT_STRUCTURE;

extern U8 disp_i2c_abort;

#endif /* CSST_I2C_DIS_H */
