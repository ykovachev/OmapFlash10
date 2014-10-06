/**
 * @file omapconfig.h
 * @author
 *
 * @section LICENSE
 *
 * Copyright (c) 2009, Texas Instruments, Inc.
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
 * This file contains configuration routines for OMAP devices.
 */

#ifndef OMAPCONFIG_H
#define OMAPCONFIG_H

#include "types.h"

#define RPAPI_BASE        omap_rom_code_public_api_base()
#define RPAPI(offset)     (RPAPI_BASE + offset)

#define OPCODE_MASK 0xFFFF0000
#define N_MASK      0x0000FFFF

typedef enum
{
  IOPC_NOP          = 0x00000000, // no opcode
  IOPC_WRITE        = 0x00010000, // write a value to a register
  IOPC_WRITE_N      = 0x00020000, // write a value to n registers
  IOPC_MODIFY       = 0x00100000, // modify the value of a register
  IOPC_MODIFY_N     = 0x00200000, // modify the value of n registers
  IOPC_MODIFY_N_M   = 0x00300000, // modify the value of n registers with fixed mask
  IOPC_MODIFY_N_M_V = 0x00400000, // modify the value of n registers with fixed mask and value
  IOPC_POLL_ZERO    = 0x01000000, // poll a register value until zero
  IOPC_POLL_NZERO   = 0x02000000, // poll a register value until not zero
  IOPC_POLL_VAL     = 0x03000000, // poll a register until value
  IOPC_WAIT_N       = 0x10000000, // loop n
  IOPC_SPIN         = 0x20000000, // spin forever
  IOPC_RPAPI_BASE   = 0x40000000, // set ROM code public API base address
  IOPC_INTERFACE    = 0x80000000, // set boot interface (USB = 0, UART3 = 1), value from N field
  IOPC_MODE_16      = 0x00160000, // 16 bit register access mode
  IOPC_MODE_32      = 0x00320000, // 32 bit register access mode
  IOPC_DEBUG_REG    = 0x00110000, // Configuration of debug register for trace and status
  IOPC_COPY         = 0x00120000, // Copy register content
  IOPC_DEBUG_UART   = 0x00130000, // Set debug UART for traces
  IOPC_HEAP_ADDR    = 0x00140000, // Set the base address of the heap in DDR
  IOPC_SKIP         = 0x00150000, // Do not execute commands from this point
  IOPC_UNSKIP       = 0x00170000, // Stop skipping commands from this point
  IOPC_SKIP_COND    = 0x00180000  // Do not execute command from this point if condition is met
} T_init_opcode;

// Command structures:
//
// WRITE          : OPCODE REGISTER VALUE
// WRITE_N        : OPCODE/N REGISTER1 VALUE1 REGISTER2 VALUE2 ... REGISTERN VALUEN
// MODIFY         : OPCODE REGISTER MASK VALUE
// MODIFY_N       : OPCODE/N REGISTER1 MASK1 VALUE1 REGISTER2 MASK2 VALUE2 ... REGISTERN MASKN VALUEN
// MODIFY_N_M     : OPCODE/N MASK REGISTER1 VALUE1 REGISTER2 VALUE2 ... REGISTERN VALUEN
// MODIFY_N_M_V   : OPCODE/N MASK VALUE REGISTER1 REGISTER2 ... REGISTERN
// POLL_ZERO      : OPCODE REGISTER MASK
// POLL_NZERO     : OPCODE REGISTER MASK
// POLL_VAL       : OPCODE REGISTER MASK VALUE
// SPIN           : no parameters
// RPAPI_BASE     : OPCODE ADDRESS
// INTERFACE      : OPCODE/N, where N is the interface value
// MODE_16        : no parameters
// MODE_32        : no parameters
// DEBUG_REG      : OPCODE/N REGISTER MASK VALUE, where N is the register ID in the code
// COPY           : OPCODE REGISTER_FROM REGISTER_TO
// DEBUG_UART     : OPCODE/N, where N is the UART to use
// HEAP_ADDR      : OPCODE ADDRESS
// SKIP           : no parameters
// UNSKIP         : no parameters
// SKIP_COND      : OPCODE ADDRESS MASK VALUE

#define HEAP_SPACE 0x2000000
#define DRV_SPACE  0x100000

typedef enum
{
  DLIF_USB_OTG = 0,
  DLIF_UART3   = 1,
  DLIF_UNKNOWN = 2
} T_download_if;

extern T_download_if download_if(void);
extern void omap_config(void);
extern U32 omap_rom_code_public_api_base(void);
extern U32 get_config_data_area(unsigned char ** p);
extern U32 get_heap_address(void);
extern U32 get_drv_address(void);
#endif

