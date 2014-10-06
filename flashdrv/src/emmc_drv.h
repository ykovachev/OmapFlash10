/**
 * @file emmc_drv.h
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
 * 
 * 
 */

/*==== INCLUDES ==============================================================*/
#ifndef EMMC_DRV_H
#define EMMC_DRV_H

#define RPAPI(offset) (get_rpapi_base() + offset)

#include "mmc.h"
///@todo include romapi.h instead of following lines but replace silicon.h by config.h in romapi.h first
#include "config.h"
#ifdef OMAP4
#include "romapi_4430.h"
#endif
#ifdef OMAP5
#include "romapi_5430.h"
#endif

typedef struct
{
  U16 sid;							/* Second ID */
  U16 mmc_volt;
  U16 card_rca;
  U8  card_type;
  //U8 erase_flag;
  U8  data_width;
  U8  data_width_support;
  U32 transfer_clk;
  U32 transfer_clk_max;
  U64 card_size;
  U8  write_protected_card;	/* to identify the write protected cards */
  U8  mmc_high_density_card;	/*flag used for the High density cards */
} T_MMC_DIS;


typedef struct T_page
{
  U32  address;
  U32  offset;
  U8   data[1];
} T_page;

#define NO_MMC_BLOCK_ID (~0)

typedef enum
{
  USER_DATA_AREA,
  BOOT_AREA_PARTITION_1,
  BOOT_AREA_PARTITION_2
} T_emmc_partition;

typedef enum
{
  temp_disable = 0,
  perm_enable,
  perm_disable,
} T_rst_n;

typedef enum
{
  sparse_disabled,
  sparse_check_needed,
  sparse_inactive,
  sparse_active
} T_sparse_state;

typedef struct  
{
  T_sparse_state  state;
  sparse_header_t file_header;
  chunk_header_t  chunk_header;
  U32             chunk_header_bytes;
  U32             chunk_index;
  struct  
  {
    U32 raw;
    U64 raw_size;
    U32 fill;
    U64 fill_size;
    U32 dc;
    U64 dc_size;
    U32 crc;
  } chunk_stats;
  BOOLEAN         chunk_new;
  U64             output_address;
  U64             start_address;
  U64             uncompressed_size;
} T_sparse_data;

typedef struct T_driver_data
{
  U32               rpapi_base;
  T_MMC_DIS         local_dis;
  U32               delay;
  U32               devinfo;
  U32               mblock;
  U32               hd;
  U32               maxclk;
  U32               bw;
  T_emmc_partition  partition;
  BOOLEAN           make_bootable;
  T_rst_n           rst_n;
#if defined OMAP4 || defined OMAP5
#ifdef ROMAPI_DEVICE_MEM
  const SYS_DriverMem_t *driverMem;
  MEM_DeviceDesc_t deviceDesc;
#endif //ROMAPI_DEVICE_MEM
#endif
  U32               block_id;
  U8           *    block_data;  // used to store blocks when we are operating at non whole blocks
  U32               block_id_partial;
  U8           *    block_data_partial;
  VU32              delay_variable;
  U32               timeout;                // timeout value before returning MMC_CMD_TO_ER
  U32               timeout_tick;           // how often to check the status
  U32               init_timeout;           // total timeout to wait during init function of the mmc
  U32               cmd01delay;               // time to wait between CMD0 and first CMD1 dyring init of mmc
  U32               ddr;                    // if set to 1 then DDR mode is used for data access. otherwise SDR mode.
  omap_adma2_desc * adma_desc;
  U32               adma_desc_num;
  T_ext_csd *       ext_csd;
  T_sparse_data     sparse;
} T_db;

struct T_driver_data *get_driver_data(void);
S32 check_mmc_slot1_8bit_interface_status(void);
U32 get_rpapi_base(void);

#endif //EMMC_DRV_H
