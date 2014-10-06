/**
 * @file mmc_dis.h
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
 * Header file with MMC DIS and Init structure
 * 
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef MMC_DIS_H
#define MMC_DIS_H

/*=====DEFINES===============================================================*/
#define MMC_TAG_BASE 0
#define MMC_PID_TAG			MMC_TAG_BASE		 /* Primary ID Key */
#define MMC_SID_TAG			(MMC_TAG_BASE + 4)	 /* Second ID Key  */
#define CARD_TYPE           (MMC_TAG_BASE + 8)   /*to get card type*/
#define MMC_ERASE_FLAG 		(MMC_TAG_BASE + 12)   /* Tag to Erase*/
#define MMC_CID_START_ADDR  (MMC_TAG_BASE + 16)  /* To get cid*/
#define MMC_CSD_START_ADDR        (MMC_TAG_BASE + 20)  /* To get csd*/
#define MMC_DATA_WIDTH            (MMC_TAG_BASE + 24)
#define MMC_MAX_DATA_WIDTH        (MMC_TAG_BASE + 28)
#define MMC_DATA_TRANSFER_CLK     (MMC_TAG_BASE + 32)
#define MMC_MAX_DATA_TRANSFER_CLK (MMC_TAG_BASE + 36)
#define CARD_SIZE                 (MMC_TAG_BASE + 40)  /* To read & write data*/
#define MMC_DATA                  (MMC_TAG_BASE + 44)  /* To read & write data*/

#define MMC_SID_1  0
#define MMC_SID_2  1

#define MMC_SLOT_1     1
#define MMC_SLOT_2     2

#define MMC_VOLT_1_8V  1
#define MMC_VOLT_3_0V  2


#define MMC_CARD		1 // MMC card
#define SD_CARD			2 // SD card
#define HS_MMC_CARD		3 // MMC card

#define ERASE_ENABLE    1
#define ERASE_DISABLE   0
/*=====INITILIZATION STRUCTURE===============================================*/
//typedef struct
//{
//    U16 pid;
//    U16 sid;
//    U16 mmc_volt;
//	U8 data_width;
//} T_MMC_INIT_STR;


typedef struct
{
    U8 mfg_id;                     /* 8 bit Manufacturer’s ID*/
    U8 product_name[6];            /* 7 character Product Name*/
    U16 hw_rev;                    /* 4 bit Hardware  Revision Number*/
    U16 fw_rev;                    /* 4 bit Firmware Revision Number*/
    U8 serial_number[4];           /* 24 bit Serial Number*/
    U16 month_code;                /* 4 bit Manufacturing Date (Month)*/
    U16 year_code;                 /* 4 bit Manufacturing Date (Year) */
    U16 day_code;                  /* 4 bit Manufacturing Date (Day) */
    U16 crc;                       /*7 bit crc*/
    U32 app_id;
    U32 sd_unique_id;
    U32 mmc_v3unique_id;
} T_MMC_CID;


typedef struct{
    U16 erase_blk_len;
    U16 erase_sector_size;
    U16 csd_structure;             // 2 bit structure type field
    U16 mmc_prot;                  // 2 bit MMC protocol
    U16 taac;                      // 8 bit TAAC
    U16 nsac;                      // 8 bit NSAC
    U16 tran_speed;                // 8 bit max data transmission speed
    U16 ccc;                       // 12 bit card command classes
    U16 read_blk_len;              // 4 bit maximum Read Block Length
    U16 read_blk_partial;          // 1 bit indicates if partial read blocks allowed
    U16 write_blk_misalign;        // 1 bit flag indicates write block misalignment
    U16 read_blk_misalign;         // 1 bit flag indicates read block misalignment
    U16 dsr_imp;                   // 1 bit flag indicates whether card has DSR reg
    U16 csize;                     // 12 bit device size
    U16 vdd_rcurr_min;             // 3 bit Max. Read Current @ Vdd Min
    U16 vdd_rcurr_max;             // 3 bit Max. Read Current @ Vdd Max
    U16 vdd_wcurr_min;             // 3 bit Max. Write Current @ Vdd Min
    U16 vdd_wcurr_max;             // 3 bit Max. Write Current @ Vdd Max
    U16 csize_mult;                // 3 bit device size multiplier
    U16 erase_grp_mult;            // 5 bit erase sector size
    U16 erase_grp_size;            // 5 bit erase group size
    U16 wp_grp_size;               // 5 bit write protect group size
    U16 wp_grp_enable;             // 1 bit write protect enable flag
    U16 default_ecc;               // 2 bit Manufacturer Default ECC
    U16 r2w_factor;                // 3 bit stream write factor
    U16 write_bl_len;              // 4 bit maximum write block length
    U16 write_bl_partial;          // 1 bit indicates if partial write blocks allowed
    U16 copy;                      // 1 bit copy flag
    U16 perm_write_protect;        // 1 bit to dis/en-able permanent write protection
    U16 tmp_write_protect;         // 1 bit to dis/en-able temporary write protection
    U16 crc;                       // 7 bit r/w/e redundancy check
} T_MMC_CSD;

/*=====DIS===================================================================*/

/*

-----------------------------------------
|										|
|		CARD_TYPE						|
|										|
-----------------------------------------
|										|
|		MMC_ERASE_FLAG					|
|										|
-----------------------------------------
|										|
|		MMC_CID_START_ADDRESS			|
|										|
-----------------------------------------
|										|
|		MMC_CSD_START_ADDRESS			|
|										|
-----------------------------------------
|										|
|		MMC_DATA_FLAG					|
|										|
-----------------------------------------

*/


#endif /*MMC_DIS_H*/
