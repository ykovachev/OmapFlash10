/**
 * @file i2c.h
 * @author 
 *
 * @section LICENSE
 *
 * Copyright (c) 2010, Texas Instruments, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *   
 *  - Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *  - Neither the name of Texas Instruments nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software  
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 * 
 * This is the h-file for the i2c Driver (i2c.c)
 * 
 */


#include "csst_tgt.h"
/*==== DECLARATION CONTROL =================================================*/
#ifndef I2C_H
#define I2C_H

#define I2C1_REG_BASE							(0x48070000)
#define I2C2_REG_BASE							(0x48072000)
#define I2C3_REG_BASE							(0x48060000)

#define I2C_REV_OFFSET 							0x00
#define I2C_IE_OFFSET 							0x04
#define I2C_STAT_OFFSET 						0x08
#define I2C_SYSS_OFFSET							0x10
#define I2C_BUF_OFFSET							0x14
#define I2C_CNT_OFFSET							0x18
#define I2C_DATA_OFFSET 						0x1C
#define I2C_SYSC_OFFSET							0x20
#define I2C_CON_OFFSET							0x24
#define I2C_OA_OFFSET							0x28
#define I2C_SA_OFFSET							0x2C
#define I2C_PSC_OFFSET							0x30
#define I2C_SCLL_OFFSET							0x34
#define I2C_SCLH_OFFSET							0x38
#define I2C_SYSTEST_OFFSET						0x3C
#define I2C_BUFSTAT_OFFSET						0x40

#define I2C_OA_VAL								0x0E
#define I2C_PSC_VAL								0x07
#define I2C_SCLL_VAL							0x35
#define I2C_SCLH_VAL							0x37
#define I2C_FS_PSC_VAL							0x04
#define I2C_FS_SCLL_VAL							0x12
#define I2C_FS_SCLH_VAL							0x12
#define I2C_HSPSC_VAL							0x04
#define I2C_HSSCLL_VAL							0x0512
#define I2C_HSSCLH_VAL							0x0C12
#define I2C_SLAVE_MASK							0x007f
#define I2C_DATA_CNT_VAL3						3
#define I2C_DATA_CNT_VAL2						2
#define I2C_DATA_CNT_VAL1						1

/* Fields of Status register*/
#define I2C_STAT_AL 							0x0001
#define I2C_STAT_NACK 							0x0002
#define I2C_STAT_ARDY							0x0004
#define I2C_STAT_RRDY							0x0008
#define I2C_STAT_XRDY							0x0010
#define I2C_STAT_GC								0x0020
#define I2C_STAT_BF								0x0100
#define I2C_STAT_AAS							0x0200
#define I2C_STAT_XUDF							0x0400
#define I2C_STAT_ROVR							0x0800
#define I2C_STAT_BB								0x1000
#define I2C_STAT_RDR							0x2000
#define I2C_STAT_XDR							0x4000
#define I2C_STAT_SBD							0x8000
#define I2C_MASTER_CODE                         0x4000



/* Fields of Connection register*/
#define I2C_CON_STT								0x0001
#define I2C_CON_STP								0x0002
#define I2C_CON_TRX								0x0200
#define I2C_CON_MST			 					0x0400
#define I2C_CON_BE								0x4000
#define I2C_CON_EN								0x8000
#define I2C_CON_DISABLE							0x0000
#define I2C_CON_HS_MODE                         0x1000
#define I2C_CON_SD_MODE                         0x0000

#define I2C_SYSC_SRST                           0x0002
#define I2C_SYSS_RDONE							0x0001

/* Fields of Bufstat register */
#define I2C_BUFSTAT_TXSTAT						0x003F
#define I2C_BUFSTAT_RXSTAT						0x3F00
#define I2C_BUFSTAT_FIFODEPTH					0xC000


/* Fields of buffer setup register */
#define I2C_BUF_XTRSH                                            0x003F
#define I2C_BUF_TXFIFO_CLR                                   0x0040
#define I2C_BUF_XDMA_EN                                       0x0080
#define I2C_BUF_RTRSH                                            0x3F00
#define I2C_BUF_RXFIFO_CLR                                   0x4000
#define I2C_BUF_RDMA_EN                                       0x8000



#define I2C_CLK_MASK    						0x00180000


#define CONTROL_PADCONF_I2C1_SCL           ((volatile unsigned char *)0x49002121)
#define CONTROL_PADCONF_I2C1_SDA           ((volatile unsigned char *)0x49002122)

#define CONTROL_PADCONF_I2C2_SCL           ((volatile unsigned char *)0x49002123)
#define CONTROL_PADCONF_I2C2_SDA           ((volatile unsigned char *)0x49002124)


#define DAL_I2C_OK 0
#define DAL_I2C_SUCCESS 0
#define DAL_I2C_FAIL    1

#define TEN 10
#define HUNDRED 100
#define THOUSAND 1000
#define TENTHOUSAND 10000

/* Function prototypes*/
void configure_i2c(U16 i2c_number, U16 clock_speed);
void set_i2c_clocks(U16 clock_speed);
S32 check_i2c_status (U16 status);
S32 clear_i2c() ;
S32 en_i2c(U16 reg,U8 chk);
S32 reset_i2c();
S32 write_i2c_data(U16 device, U8 subaddr, U8 data);
S32 write_i2c_nodata(U16 device, U8 subaddr);
S32 read_i2c_data(U16 device, U8 subaddr, U8 * data);
S32 read_next_i2c(U16 device, U8 * data);
S32 deinit_i2c(void);
void dl_lazy_delay(unsigned long time);
U16 get_hssclh_val(U16 clock_speed);
U16 get_hsscll_val(U16 clock_speed);


/*Dummy functions - used in i2c_drv.c for i2c writes/reads using the HW FIFO*/
S32 write_i2c_with_dataptr(U16 device, U16 length, U8* dataptr);
S32 read_i2c_with_dataptr_2byte_subaddr(U16 device, U16 subaddr, U8 address_mode, U16 size, U8* data);
void write_i2c_onlydata(U16 device, U8 * buf, U32 * len);
S32 read_i2c_onlydata(U16 device, U8 * buf, U32 * len);

#endif /* I2C_H */
