/**
 * @file omapconfig.c
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

#define OMAPCONFIG_C

#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include "omapconfig.h"
#include "disp.h"
#include "debug.h"
#include "uart.h"

#define CONFIG_UART_TRACE

/* The pragmas below are there to define a couple of data sections in that are
   used by the TMS470 linker job to place the anchor at the end of the second loader
   binary and the configuration sequence (2 kB) just after in SRAM */

#pragma DATA_SECTION(config_sequence_anchor,".config_anchor:config_anchor");
const char config_sequence_anchor[16] = { 'E', 'N', 'D', ' ', 'O', 'F', ' ','2', 'N', 'D', 'L', 'O', 'A', 'D', 'E', 'R'};

#pragma DATA_SECTION(config_sequence,".config:config_sequence");
U32 config_sequence[512];

static T_download_if dlif = DLIF_UNKNOWN; 
static U32 config_rpapi_base = 0;
static U32 config_heap_address = 0;
static U32 config_drv_address = 0;
static BOOLEAN config_skip_active = FALSE;

/*------------------------------------------------------------------------------
| Function    : get_config_data_area
+------------------------------------------------------------------------------
| Description : Function for getting reference and size for config data area
|
| Parameters  : p - pointer to pointer to area
|
| Returns     : size
+----------------------------------------------------------------------------*/
U32 get_config_data_area(unsigned char ** p)
{
  *p = (unsigned char *)config_sequence;
  return(sizeof(config_sequence));
}

/*------------------------------------------------------------------------------
| Function    : get_heap_address
+------------------------------------------------------------------------------
| Description : Get the address of the heap in DDR
|
| Parameters  : None
|
| Returns     : Heap address
+----------------------------------------------------------------------------*/
U32 get_heap_address(void)
{
  return config_heap_address;
}

/*------------------------------------------------------------------------------
| Function    : get_drv_address
+------------------------------------------------------------------------------
| Description : Get the load address for memory drivers in DDR
|
| Parameters  : None
|
| Returns     : Load address for drivers
+----------------------------------------------------------------------------*/
U32 get_drv_address(void)
{
  return config_drv_address;
}

/*------------------------------------------------------------------------------
| Function    : config_set
+------------------------------------------------------------------------------
| Description : Function for setting the value of a register
|
| Parameters  : mode    - 32/16 bit address mode for register
|               address - address of register to modify
|               value   - value to apply
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_set(T_init_opcode mode, U32 address, U32 value)
{
  switch(mode)
  {
    case IOPC_MODE_32:
      *((PREG_U32)address) = value;
      break;
    case IOPC_MODE_16:
      *((PREG_U16)address) = (U16)(value & 0x0000FFFF);
      break;
  }
}

/*------------------------------------------------------------------------------
| Function    : config_set_interface
+------------------------------------------------------------------------------
| Description : Function for setting the value of the download interface
|               parameter (which interface was used to download the second
|               to SRAM)
|
| Parameters  : download_if - interface used
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_set_interface(T_download_if download_if)
{
	dlif = download_if;
}

/*------------------------------------------------------------------------------
| Function    : config_modify
+------------------------------------------------------------------------------
| Description : Modify a set of bits in a register to a specified value
|
| Parameters  : mode    - 32/16 bit address mode for register
|               address - address of register to modify
|               mask    - mask specifying bits to modify
|               value   - value to apply
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_modify(T_init_opcode mode, U32 address, U32 mask, U32 value)
{
  switch(mode)
  {
    case IOPC_MODE_32:
      *((PREG_U32)address) = (*((PREG_U32)address) & ~mask) | (value & mask);
      break;
    case IOPC_MODE_16:
      *((PREG_U16)address) = ((*((PREG_U16)address) & ~mask) | (value & mask)) & 0x0000FFFF;
      break;
  }
}

/*------------------------------------------------------------------------------
| Function    : config_poll_zero
+------------------------------------------------------------------------------
| Description : Poll the value of a set of bits in a register until zero
|
| Parameters  : mode    - 32/16 bit address mode for register
|               address - address of register to poll
|               mask    - mask specifying bits to check
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_poll_zero(T_init_opcode mode, U32 address, U32 mask)
{
  switch(mode)
  {
    case IOPC_MODE_32:
      while((*((PREG_U32)address) & mask));
      break;
    case IOPC_MODE_16:
      while((*((PREG_U16)address) & mask));
      break;
  }
}

/*------------------------------------------------------------------------------
| Function    : config_poll_nzero
+------------------------------------------------------------------------------
| Description : Poll a set of bits in a register until value is non-zero
|
| Parameters  : mode    - 32/16 bit address mode for register
|               address - address of register to poll
|               mask    - mask specifying bits to check
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_poll_nzero(T_init_opcode mode, U32 address, U32 mask)
{
  switch(mode)
  {
    case IOPC_MODE_32:
      while(!(*((PREG_U32)address) & mask));
      break;
    case IOPC_MODE_16:
      while(!(*((PREG_U16)address) & mask));
      break;
  }
}

/*------------------------------------------------------------------------------
| Function    : config_poll_value
+------------------------------------------------------------------------------
| Description : Function called to poll a register until a subset of bits in
|               the register value are as specified
|
| Parameters  : mode    - 32/16 bit address mode for register
|               address - address of register to poll
|               mask    - mask specifying bits to check
|               value   - value to check the masked out register value for
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_poll_value(T_init_opcode mode, U32 address, U32 mask, U32 value)
{
  switch(mode)
  {
    case IOPC_MODE_32:
      while((*((PREG_U32)address) & mask) != value);
      break;
    case IOPC_MODE_16:
      while((*((PREG_U16)address) & mask) != (value & 0x0000FFFF));
      break;
  }
}

/*------------------------------------------------------------------------------
| Function    : config_skip_cond
+------------------------------------------------------------------------------
| Description : Function called to check for skip condition and set skip flag
|
| Parameters  : address - address to check
|               mask    - mask specifying bits to check
|               value   - value to check the masked out value for
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_skip_cond(U32 address, U32 mask, U32 value)
{
  config_skip_active = ((*((PREG_U32)address) & mask) == value) ? TRUE : FALSE;
}

/*------------------------------------------------------------------------------
| Function    : omap_rom_code_public_api_base
+------------------------------------------------------------------------------
| Description : Function for getting the public ROM code API base address (the
|               address is set in the board configuration file on the host side
|               and stored during processing of the configuration on target
|               side)
|
| Parameters  : void
|
| Returns     : API base address
+----------------------------------------------------------------------------*/
U32 omap_rom_code_public_api_base(void)
{
  // Emergency brake - base address not set!
  while(!config_rpapi_base);
  return config_rpapi_base;
}

/*------------------------------------------------------------------------------
| Function    : download_if
+------------------------------------------------------------------------------
| Description : Function for getting the download interface (the interface 
|               through which the second loader was downloaded to SRAM)
|
| Parameters  : void
|
| Returns     : download interface 
+----------------------------------------------------------------------------*/
T_download_if download_if(void)
{
  return(dlif);
}

/*------------------------------------------------------------------------------
| Function    : config_copy_register
+------------------------------------------------------------------------------
| Description : Function that will copy the value from one register to another
|               register
|
| Parameters  : from - address of register to copy
|               to - address of register to modify
|
| Returns     : void
+----------------------------------------------------------------------------*/
void config_copy_register(U32 from, U32 to)
{
  volatile U32 * pfrom = (volatile U32 * )from;
  volatile U32 * pto   = (volatile U32 * )to;
  *pto = *pfrom;
}

/*------------------------------------------------------------------------------
| Function    : omap_config
+------------------------------------------------------------------------------
| Description : Configuration function for the OMAP. Will process the data
|               structure appended to the second loader binary during download
|               and apply the settings specified by the board configuration 
|               file on the host side to the OMAP registers listed.
|
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
void omap_config(void)
{
  int index = 0;
  int n, m, v;
  T_init_opcode mode = IOPC_MODE_32;

  debug_reg_init();

  while(config_sequence[index] != IOPC_NOP)
  {
    if((config_skip_active == FALSE) || (config_sequence[index] == IOPC_UNSKIP))
    {
      debug_reg_toggle(debug_reg_id_cfg, 32000, 32000); // Note: This will only do anything if already configured by the OMAP configuration itself.
    }

    switch(config_sequence[index] & OPCODE_MASK)
    {
      // Overwrite the value of a register 

      case IOPC_WRITE:
        if(config_skip_active == FALSE)
        {
          config_set(mode, config_sequence[index + 1], config_sequence[index + 2]);
        }
        index += 3;
        break;

      // Overwrite the values of a sequence of registers
      
      case IOPC_WRITE_N:
        n = config_sequence[index++] & N_MASK;
        while(n)
        {
          if(config_skip_active == FALSE)
          {
            config_set(mode, config_sequence[index], config_sequence[index + 1]);
            if(n > 1)
            {
              debug_reg_toggle(debug_reg_id_cfg, 32000, 32000); // Note: This will only do anything if already configured by the OMAP configuration itself.
            }
          }
          index += 2;
          n--;
        }
        break;

      // Modify the value of a register
      
      case IOPC_MODIFY:
        if(config_skip_active == FALSE)
        {
          config_modify(mode, config_sequence[index + 1], config_sequence[index + 2], config_sequence[index + 3]);
        }
        index += 4;
        break;

      // Modify the value of a sequence of registers with individual register masks
      
      case IOPC_MODIFY_N:
        n = config_sequence[index++] & N_MASK;
        while(n)
        {
          if(config_skip_active == FALSE)
          {
            config_modify(mode, config_sequence[index], config_sequence[index + 1], config_sequence[index + 2]);
            if(n > 1)
            {
              debug_reg_toggle(debug_reg_id_cfg, 32000, 32000); // Note: This will only do anything if already configured by the OMAP configuration itself.
            }
          }
          index += 3;
          n--;
        }
        break;

      // Modify the values of a sequence of registers with the same register mask

      case IOPC_MODIFY_N_M:
        n = config_sequence[index++] & N_MASK;
        m = config_sequence[index++];
        while(n)
        {
          if(config_skip_active == FALSE)
          {
            config_modify(mode, config_sequence[index], m, config_sequence[index + 1]);
            if(n > 1)
            {
              debug_reg_toggle(debug_reg_id_cfg, 32000, 32000); // Note: This will only do anything if already configured by the OMAP configuration itself.
            }
          }
          index += 2;
          n--;
        }
        break;

      // Modify the values of a sequence of registers with the same register mask and value
      
      case IOPC_MODIFY_N_M_V:
        n = config_sequence[index++] & N_MASK;
        m = config_sequence[index++];
        v = config_sequence[index++];
        while(n)
        {
          if(config_skip_active == FALSE)
          {
            config_modify(mode, config_sequence[index], m, v);
            if(n > 1)
            {
              debug_reg_toggle(debug_reg_id_cfg, 32000, 32000); // Note: This will only do anything if already configured by the OMAP configuration itself.
            }
          }
          index++;
          n--;
        }
        break;

      // Poll a register until a set of bits are zero
      
      case IOPC_POLL_ZERO:
        if(config_skip_active == FALSE)
        {
          config_poll_zero(mode, config_sequence[index + 1], config_sequence[index + 2]);
        }
        index += 3;
        break;

      // Poll a register until a set of bits are non-zero
      
      case IOPC_POLL_NZERO:
        if(config_skip_active == FALSE)
        {
          config_poll_nzero(mode, config_sequence[index + 1], config_sequence[index + 2]);
        }
        index += 3;
        break;

      // Poll a register until a set of bits are a particular value

      case IOPC_POLL_VAL:
        if(config_skip_active == FALSE)
        {
          config_poll_value(mode, config_sequence[index + 1], config_sequence[index + 2], config_sequence[index + 3]);
        }
        index += 4;
        break;

      // Spin the wheels for a number of cycles
      
      case IOPC_WAIT_N:
        n = config_sequence[index++] & N_MASK;
        if(config_skip_active == FALSE)
        {
          while(n--);
        }
        break;

      // Stop here... Can be used for debug w JTAG 

      case IOPC_SPIN:
        if(config_skip_active == FALSE)
        {
          volatile int spin = 1;
          while(spin);
          index++;
        }
        break;

      // Set the ROM code public API base address

      case IOPC_RPAPI_BASE:
        if(config_skip_active == FALSE)
        {
          config_rpapi_base = config_sequence[index + 1];
        }
        index += 2;
        break;

      // Set the address of the heap in DDR

      case IOPC_HEAP_ADDR:
        if(config_skip_active == FALSE)
        {
          config_heap_address = config_sequence[index + 1];
          config_drv_address = config_heap_address - DRV_SPACE;
        }
        index += 2;
        break;

      // Set the interface used for download of the second loader (peripheral boot)

      case IOPC_INTERFACE:
        config_set_interface((T_download_if)(config_sequence[index] & N_MASK));
        index++;
        break;

      // Change the addressing mode used for registers (16 or 32 bit access)

      case IOPC_MODE_16:
      case IOPC_MODE_32:
        if(config_skip_active == FALSE)
        {
          mode = (T_init_opcode)config_sequence[index];
        }
        index++;
        break;
      
      // Configure debug register functionality (e.g. for toggling a GPIO or LED)

      case IOPC_DEBUG_REG:
        if(config_skip_active == FALSE)
        {
          debug_reg_setup((T_debug_reg_id)(config_sequence[index] & N_MASK), config_sequence[index + 1], config_sequence[index + 2], config_sequence[index + 3]);
        }
        index += 4;
        break;

      // Copy the value of a register to another register

      case IOPC_COPY:
        if(config_skip_active == FALSE)
        {
          config_copy_register(config_sequence[index + 1], config_sequence[index + 2]);
        }
        index += 3;
        break;

      case IOPC_DEBUG_UART:
#ifdef DEBUG_UART
        if(config_skip_active == FALSE)
        {
          set_debug_uart((U8)(config_sequence[index] & N_MASK));
        }
#endif
        index++;
        break;

      case IOPC_SKIP:
        config_skip_active = TRUE;
        index++;
        break;

      case IOPC_UNSKIP:
        config_skip_active = FALSE;
        index++;
        break;

      case IOPC_SKIP_COND:
        config_skip_cond(config_sequence[index + 1], config_sequence[index + 2], config_sequence[index + 3]);
        index += 4;
        break;
    }
  }
}
