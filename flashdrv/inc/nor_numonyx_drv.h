/**
 * @file nor_numonyx_drv.c
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
 * Numonyx NOR Flash driver
 */


/*==== DECLARATION CONTROL ==================================================*/
#ifndef _NOR_NUMONYX_DRV_H
#define _NOR_NUMONYX_DRV_H

/*==== INCLUDES =============================================================*/
#include "types.h"

/*==== CONSTS ===============================================================*/

#define PAGE_SIZE               (8 * 1024)
#define PAGE_MASK               0xFFFFE000
//#define PAGE_SIZE               1024
//#define PAGE_MASK               0xFFFFFC00
#define SECTOR_MASK_32K         0xFFFF8000
#define BLOCK_SIZE              (32 * 1024)

/* Commands */

#define CMD_ALT_PROGRAM_SETUP         0x10
#define CMD_BLOCK_ERASE_SETUP         0x20
#define CMD_ENH_PROGRAM_SETUP         0x30
#define CMD_DW_PROGRAM_SETUP          0x35
#define CMD_PROGRAM_SETUP             0x40
#define CMD_CLEAR_STATUS_REG          0x50
#define CMD_QW_PROGRAM_SETUP          0x56
#define CMD_BLOCK_LOCK_SETUP          0x60
#define CMD_BLOCK_UNLOCK_SETUP        0x60
#define CMD_BLOCK_LOCK_DOWN_SETUP     0x60
#define CMD_SET_CFG_REG_SETUP         0x60
#define CMD_READ_STATUS_REG           0x70
#define CMD_QE_FACT_PROGRAM_SETUP     0x75
#define CMD_READ_ELECTRONIC_SIGNATURE 0x90
#define CMD_READ_CFI_QUERY            0x98
#define CMD_BLOCK_ERASE_CONFIRM       0xD0
#define CMD_BLOCK_UNLOCK_CONFIRM      0xD0
#define CMD_E_FACT_PROGRAM_CONFIRM    0xD0
#define CMD_READ_ARRAY                0xFF

/* CFI Fields */

#define CFI_ADR(adr)                      ((adr) << 1)
#define CFI_MANUFACTURER_CODE             CFI_ADR(0x00)
#define CFI_DEVICE_CODE                   CFI_ADR(0x01)
#define CFI_Q                             CFI_ADR(0x10)
#define CFI_R                             CFI_ADR(0x11)
#define CFI_Y                             CFI_ADR(0x12)
#define CFI_BLOCK_REGIONS                 CFI_ADR(0x2C)
#define CFI_NUM_ERASE_BLOCKS_LOW(R)       CFI_ADR(0x2D + R * 4)
#define CFI_NUM_ERASE_BLOCKS_HIGH(R)      CFI_ADR(0x2E + R * 4)
#define CFI_SIZE_ERASE_BLOCKS_LOW(R)      CFI_ADR(0x2F + R * 4)
#define CFI_SIZE_ERASE_BLOCKS_HIGH(R)     CFI_ADR(0x30 + R * 4)
#define CFI_DEVICE_SIZE                   CFI_ADR(0x27)

/* Status bits */

#define STATUS_P_EC(value)                ((value >> 7) & 0x0001)
#define STATUS_ERASE_SUSPEND(value)       ((value >> 6) & 0x0001)
#define STATUS_ERASE(value)               ((value >> 5) & 0x0001)
#define STATUS_PROGRAM(value)             ((value >> 4) & 0x0001)
#define STATUS_VPP(value)                 ((value >> 3) & 0x0001)
#define STATUS_PROGRAM_SUSPEND(value)     ((value >> 2) & 0x0001)
#define STATUS_BLOCK_PROTECTION(value)    ((value >> 1) & 0x0001)
#define STATUS_BANK_WRITE(value)          ((value) & 0x0001)
#define STATUS_MULT_WORD_PROGRAM(value)   ((value) & 0x0001)
#define STATUS_QCHECK_SEQ(value)          ((value & 0x003A) > 0)
#define STATUS_QCHECK_NO_SEQ(value)       ((value & 0x001A) > 0)

/*==== TYPES ================================================================*/

typedef enum
{
  write_single_word  = 0x01,
  write_double_word  = 0x02,
  write_quad_word    = 0x04,
  write_single_efact = 0xe1
} T_write_mode;

typedef struct
{
  U32           buffer_size;
  U32           device_size;
  U32           cs_base_addr;
  U32           log;
  U32           echeck;
  T_write_mode  wmode;
  U16           data_buffer[PAGE_SIZE/2];
  U32           buf_write_addr;
  U32           buf_write_size;
  U8            buf_valid;
} T_NOR_PARAMS;

/*==== EXPORTS ==============================================================*/

U32 nor_numonyx_init(T_driver_config * config, char ** result);
U32 nor_numonyx_deinit(T_driver_config * config, char ** result);
U32 nor_numonyx_erase(T_driver_config * config, char ** result, U64 address, U64 length);
U32 nor_numonyx_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more);
U32 nor_numonyx_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
U32 nor_numonyx_get_info(T_driver_config * config, T_driver_info * info);

#endif /* _NOR_AMD_DRV_H */
/*==== END OF FILE ==========================================================*/
