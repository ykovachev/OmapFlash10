/**
 * @file nor_amd_drv.c
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
 * AMD NOR Flash driver
 */


/*==== DECLARATION CONTROL ==================================================*/
#ifndef _NOR_AMD_DRV_H
#define _NOR_AMD_DRV_H

/*==== INCLUDES =============================================================*/
#include "types.h"

/*==== CONSTS ===============================================================*/

#define PAGE_SIZE               1024
#define PAGE_MASK               0xFFFFFC00
#define SECTOR_MASK_32K         0xFFFF8000

#define HEX_AA  (0x00AA)
//#define HEX_AAA_SST (0xAAAA)
//#define HEX_555_SST (0x5555)
//#define HEX_AAA_NSST (0x555 << 1)
//#define HEX_555_NSST (0x2AA << 1)
//#define HEX_AAA_NSST (0x555)
//#define HEX_555_NSST (0x2AA)
#define HEX_555 (0x555 << 1)
#define HEX_2AA (0x2AA << 1)

/* Status bits for Spansion (AMD/Fujitsu) floating gate flash devices */
#define DQ5_EXCEEDED_TIMING_LIMIT (0x20)
#define DQ6_TOGGLE_BIT_I          (0x40)
#define DQ7_DATA_POLLING          (0x80)


/*==== TYPES ================================================================*/
typedef struct
{
    U32 buffer_size;
    U32 device_size;
    U32 cs_base_addr;
    U32 log;
    U32 wrbuf;
    U32 echeck;
    U16 data_buffer[PAGE_SIZE/2];
    U32 buf_write_addr;
    U32 buf_write_size;
    U8 buf_valid;
} T_NOR_PARAMS;

/*==== EXPORTS ==============================================================*/

U32 nor_amd_init(T_driver_config * config, char ** result);
U32 nor_amd_deinit(T_driver_config * config, char ** result);
U32 nor_amd_erase(T_driver_config * config, char ** result, U64 address, U64 length);
U32 nor_amd_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more);
U32 nor_amd_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
U32 nor_amd_get_info(T_driver_config * config, T_driver_info * info);

#endif /* _NOR_AMD_DRV_H */
/*==== END OF FILE ==========================================================*/
