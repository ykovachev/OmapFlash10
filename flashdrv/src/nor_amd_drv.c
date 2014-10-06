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

/*==== INCLUDES =============================================================*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "types.h"
#include "flash_drv.h"
#include "nor_amd_drv.h"

/*==== MACROS ===============================================================*/

#define LOG_EVENT(e) if(pNOR->log) config->log_event(e)

/*==== CONSTS ===============================================================*/
#ifndef SIMULATION
#pragma DATA_SECTION(driver_if,".consttbl");
#endif
const T_driver_if driver_if = 
{
  {
    OMAPFLASH_DRIVER_HEADER_STRING_V7,
    "NOR AMD",
    CTRL_Z
  },
  {
    &driver_if,
    nor_amd_init,
    nor_amd_read,
    nor_amd_write,
    nor_amd_erase,
    nor_amd_deinit,
    nor_amd_get_info
  }
};

typedef enum
{
  C_ADDRESS,
  C_WRBUF,
  C_ECHECK,
  C_LOG
} T_driver_setup_index;

const T_driver_setup_const setup_const[] =
{
  { "address", DEFAULT, TRUE  },
  { "wrbuf",   DEFAULT, FALSE },
  { "echeck",  DEFAULT, FALSE },
  { "log",     DEFAULT, FALSE },
  { "",        DEFAULT, FALSE }
};

T_driver_setup_var setup_var[sizeof(setup_const) / sizeof(T_driver_setup_const)];

/*==== TYPES ================================================================*/

/*==== LOCALS ===============================================================*/

/*==== PRIVATE FUNCTIONS ====================================================*/
/*-----------------------------------------------------------------------------
| Function    : poll_status_bits
+------------------------------------------------------------------------------
| Description : Waits until pending write or erase operation is finished
|
| Parameters  : p_addr [IN] - last written address
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 poll_status_bits(U16 *p_addr)
{
    volatile U16 status_1;
    volatile U16 status_2;
    U16 dq6_toggles;

    /* In situations where the flash is not erased before writing occurs
    (skip-erase), DQ7_DATA_POLLING status may not be reliable. Instead, we
    check DQ6 which changes its value for each read to the device as long as
    programming/erasing is in progress; in other words, we wait for the
    toggling to stop. */

    /* Begin polling for status */
    status_1 = FLASH_READ((U32)p_addr);
    status_2 = FLASH_READ((U32)p_addr);

    /* Does DQ6 toggle? */
    dq6_toggles = (status_1 ^ status_2) & DQ6_TOGGLE_BIT_I;

    while (dq6_toggles)
    {
        status_1 = FLASH_READ((U32)p_addr);
        status_2 = FLASH_READ((U32)p_addr);
        dq6_toggles = (status_1 ^ status_2) & DQ6_TOGGLE_BIT_I;

        /* Is time-out condition set? */
        if (DQ5_EXCEEDED_TIMING_LIMIT & status_2)
        {
            /* Toggling could have been interrupted, read again to be sure */
            status_1 = FLASH_READ((U32)p_addr);
            status_2 = FLASH_READ((U32)p_addr);
            dq6_toggles = (status_1 ^ status_2) & DQ6_TOGGLE_BIT_I;

            if (dq6_toggles && (DQ5_EXCEEDED_TIMING_LIMIT & status_2))
            {
                return FLASH_DRV_ERR_WRITE_TIMEOUT;
            }
        }
    }
    return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : poll_status_dq7
+------------------------------------------------------------------------------
| Description : Waits until pending write or erase operation is finished
|
| Parameters  : p_addr [IN] - last written address
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 poll_status_dq7(U16 * p_addr)
{
  /* Begin polling for status */
  volatile U16 status = FLASH_READ((U32)p_addr);

  while (!(status & DQ7_DATA_POLLING))
  {
    status = FLASH_READ((U32)p_addr);

    /* Is time-out condition set? */
    if (DQ5_EXCEEDED_TIMING_LIMIT & status)
    {
      status = FLASH_READ((U32)p_addr);
  
      if (!(status & DQ7_DATA_POLLING) && (DQ5_EXCEEDED_TIMING_LIMIT & status))
      {
        return FLASH_DRV_ERR_WRITE_TIMEOUT;
      }
    }
  }
  return FLASH_DRV_SUCCESS;
}


/*-----------------------------------------------------------------------------
| Function    : 
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : 
+----------------------------------------------------------------------------*/
BOOLEAN start_query_nor_flash(T_driver_config * config)
{
  T_NOR_PARAMS * pNOR = (T_NOR_PARAMS *)(config->data);
  U16 parms[3];

  /* Query CFI parameters */
  FLASH_WRITE (pNOR->cs_base_addr + HEX_AA, 0xF0); /* AMD and SST Reset Command */
  FLASH_WRITE (pNOR->cs_base_addr + HEX_AA, 0x98); /* Set CFI Query Mode */

  /* Check if x16 CFI-compatible device in 16-bit mode */
  parms[0] = FLASH_READ(pNOR->cs_base_addr + 0x20);
  parms[1] = FLASH_READ(pNOR->cs_base_addr + 0x22);
  parms[2] = FLASH_READ(pNOR->cs_base_addr + 0x24);

  /* Read CFI parameters if data == "QRY" */
  if (parms[0] == 'Q' && parms[1] == 'R' && parms[2] == 'Y')
  {
    return TRUE;
  }
  else
  {
    config->send_info("flash is non CFI (found: %#04X %#04X %#04X)", parms[0], parms[1], parms[2]);
    return FALSE;
  }
}

/*-----------------------------------------------------------------------------
| Function    : 
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : 
+----------------------------------------------------------------------------*/
U8 query_nor_flash_number_block_regions(T_driver_config * config) 
{
  T_NOR_PARAMS * pNOR = (T_NOR_PARAMS *)(config->data);
  U8 number_block_regions = (U8)FLASH_READ(pNOR->cs_base_addr + 0x58);
  return number_block_regions;
}

/*-----------------------------------------------------------------------------
| Function    : 
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : 
+----------------------------------------------------------------------------*/
U32 query_nor_flash_erase_block_info(T_driver_config * config, U8 block_index)
{
  T_NOR_PARAMS * pNOR = (T_NOR_PARAMS *)(config->data);
  U32 erase_block_info = FLASH_READ(pNOR->cs_base_addr + 0x5A + 8 * block_index) + 
                         (FLASH_READ(pNOR->cs_base_addr + 0x5C + 8 * block_index) << 8) + 
                         (FLASH_READ(pNOR->cs_base_addr + 0x5E + 8 * block_index) << 16) + 
                         (FLASH_READ(pNOR->cs_base_addr + 0x60 + 8 * block_index) << 24);
  return erase_block_info;
}

/*-----------------------------------------------------------------------------
| Function    : 
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : 
+----------------------------------------------------------------------------*/
static void fill_data_buffer(T_driver_config * config, T_NOR_PARAMS *params, U32 dst_addr, U8 *src_ptr,
                             U32 size)
{
    U8 odd_start_addr = dst_addr % 2;
    U8 *buf_ptr = (U8 *)params->data_buffer;

    /* Set buffer variables */
    params->buf_valid = TRUE;
    params->buf_write_size = odd_start_addr + size;
    params->buf_write_addr = dst_addr - odd_start_addr;

    /* Copy data into buffer (enure that data is word-aligned) */
    if(odd_start_addr)
    {
        config->drv_memset(buf_ptr, 0xFF, 1);
    }
    config->drv_memcpy(buf_ptr + odd_start_addr, src_ptr, size);
}

/*==== PUBLIC FUNCTIONS =====================================================*/

/*-----------------------------------------------------------------------------
| Function    : nor_amd_init
+------------------------------------------------------------------------------
| Description : Flash initialization (queries write buffer size)
|
| Parameters  : p_flash [OUT] - flash parameters
|               addr     [IN] - address
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
U32 nor_amd_init(T_driver_config * config, char ** result)
{
  T_driver_setup_var *setup = get_setup_var();  
  T_NOR_PARAMS *pNOR = NULL;
  *result = NULL;

  /* Parse configuration string */

  if(drv_parse_config(get_setup_const(), setup, config->cstring, config) == FLASH_DRV_ERROR)
  {
    *result = "UNABLE TO FIND CONFIGURATION PARAMETER DURING INITIALIZATION";
    return FLASH_DRV_ERROR;
  }

  /* Update callback functions pointers and check that the necessary functions are there */

  config->data = config->drv_malloc(sizeof(T_NOR_PARAMS));

  pNOR = (T_NOR_PARAMS *)(config->data);
  
  if(!pNOR)
  {
    *result = "DRIVER DATA MEMORY ALLOCATION ERROR DURING INITIALIZATION";
    return FLASH_DRV_ERROR;
  }

  pNOR->cs_base_addr = setup[C_ADDRESS].value;
  pNOR->log          = setup[C_LOG].valid ? setup[C_LOG].value : FALSE;
  pNOR->wrbuf        = setup[C_WRBUF].valid ? setup[C_WRBUF].value : FALSE;
  pNOR->echeck       = setup[C_ECHECK].valid ? setup[C_ECHECK].value : FALSE;
  pNOR->buffer_size  = 0;
  pNOR->device_size  = 0;
  pNOR->buf_valid    = FALSE;

  if(start_query_nor_flash(config))
  {

    pNOR->buffer_size = 1 << (FLASH_READ(pNOR->cs_base_addr + 0x54) + (FLASH_READ(pNOR->cs_base_addr + 0x56) << 8));
    pNOR->device_size = 1 << FLASH_READ(pNOR->cs_base_addr + 0x4E);
  }
  else
  {
    *result = "CFI QUERY FAILED - NOT A CFI DEVICE";
    return FLASH_DRV_ERROR;
  }

  config->send_info("- Device size: %d", pNOR->device_size);
  config->send_info("Resetting device...");

  FLASH_WRITE (pNOR->cs_base_addr + HEX_AA, 0xF0); /* AMD and SST Reset Command */

  config->send_info("Initialization complete...");
  return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : nor_amd_deinit
+------------------------------------------------------------------------------
| Description : AMD NOR Flash deinitialization
|
| Parameters  : config   - Flash parms
|               result   - Returned status
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_amd_deinit(T_driver_config * config, char ** result)
{
  config->drv_free(config->data);
  config->data = NULL;
  config->send_info("NOR AMD DRIVER DEINIT COMPLETE");
  return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : nor_amd_get_info
+------------------------------------------------------------------------------
| Description : AMD NOR Flash info
|
| Parameters  : config   - Flash parms
|               info     - Returned information
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_amd_get_info(T_driver_config * config, T_driver_info * info)
{
  T_NOR_PARAMS * pNOR  = (T_NOR_PARAMS *)config->data;
  info->device_base_address = pNOR->cs_base_addr;
  info->device_size         = pNOR->device_size;
  return FLASH_DRV_SUCCESS;
}


/*-----------------------------------------------------------------------------
| Function    : nor_amd_erase
+------------------------------------------------------------------------------
| Description : Erase flash sector
|
| Parameters  : p_flash [IN] - flash parameters (not used)
|               addr    [IN] - block address
|               length  [IN] - length (not used)
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
U32 nor_amd_erase_block(T_driver_config * config, char ** result, U32 address, U32 length)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *) config->data;
  U32 cs_addr = pNOR->cs_base_addr;
  U32 cmd1, cmd2;

  U16 * pw = (U16 *)address;
  U16 * pe = (U16 *)(address + length);

  while(pw < pe)
  {
    if(*pw != 0xFFFF) break;
    pw++;
  }
  if(*pw != 0xFFFF)
  {
    cmd1 = HEX_555;
    cmd2 = HEX_2AA;

    LOG_EVENT("ERASE BLOCK");
    LOG_EVENT("S: Erase sequence");

    FLASH_WRITE(cs_addr + cmd1, 0xF0); /* Added for MV */
    FLASH_WRITE(cs_addr + cmd1, 0x60); /* Sector Unlock Sequence (addr is dont-care) */
    FLASH_WRITE(cs_addr + cmd1, 0x60); /* Sector Unlock Sequence (addr is dont-care) */
    FLASH_WRITE((address | 0x84) & (~(0x2)), 0x60); /* Sector Unlock Sequence (address 6 = VIH, address 1 = VIH (needed for Samsung memory), address 0 = VIL(needed for Samsung memory)) */
    FLASH_WRITE(cs_addr + cmd1, 0xF0); /* Exit locking sequence */
    
    FLASH_WRITE(cs_addr + cmd1, 0xAA); /* AMD Unlock Cycle 1 */
    FLASH_WRITE(cs_addr + cmd2, 0x55); /* AMD Unlock Cycle 2 */
    FLASH_WRITE(cs_addr + cmd1, 0x80); /* AMD Erase Unlock */

    FLASH_WRITE(cs_addr + cmd1, 0xAA); /* AMD Unlock Cycle 1 */
    FLASH_WRITE(cs_addr + cmd2, 0x55); /* AMD Unlock Cycle 2 */

    FLASH_WRITE(address, 0x30); /* Block Erase Command */

    LOG_EVENT("E: Erase sequence");

    LOG_EVENT("S: Poll status");
    result_code = poll_status_bits((U16 *) address);
    LOG_EVENT("E: Poll status");

    if(result_code != FLASH_DRV_SUCCESS)
    {
      *result = "TIMEOUT WHILE ERASING BLOCK";
    }
  }
  return result_code;
}


/*-----------------------------------------------------------------------------
| Function    : 
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : 
+----------------------------------------------------------------------------*/
U32 nor_amd_erase(T_driver_config * config, char ** result, U64 address, U64 length)
{
  /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
  if(address > 0xFFFFFFFF || length > 0xFFFFFFFF)
  {
    if(length > 0xFFFFFFFF)
      config->send_info("64-bit LENGTH (%#08x) NOT SUPPORTED.", length);
    else
      config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", address);
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }

  if (!start_query_nor_flash(config))
  {
    *result = "QUERY ERROR - INIT";
    return FLASH_DRV_ERROR;
  }
  else 
  {
    T_NOR_PARAMS * params = (T_NOR_PARAMS *)(config->data);
    U32 offset = address - params->cs_base_addr;
    U32 size = length;
    U8 number_block_regions = query_nor_flash_number_block_regions(config);
    U32 current_region = 0;
    U32 current_offset = offset;
    U32 target_size = size;
    U8 i;

    for(i=0; i < number_block_regions; i++) 
    {
      U32 erase_block_info = query_nor_flash_erase_block_info(config, i);
      U32 block_size = (erase_block_info & 0xFFFF0000) >> 8;
      U32 nb_blocks = (erase_block_info & 0xFFFF) + 1;
      U32 region_size = nb_blocks * block_size;
      U32 next_region = current_region + region_size;
	    U32 actual_size = 0;

      if(size == 0)
      {
        size = region_size - !i * offset;
        target_size = size;
      }

      if (current_region <= current_offset && current_offset < next_region)
      {
        U32 local_offset = current_offset - current_region;

        if (local_offset % block_size != 0)
        {
            if (offset == current_offset)
              config->send_info("Erase start address does not match block boundary at %#08X", params->cs_base_addr + offset);
            else
                config->send_info("Internal Error: Block start address does not match block boundary at %#08X", params->cs_base_addr + offset);
            return FLASH_DRV_INVALID_PARAMETER;
        }
        else
        {
          if (current_offset + size >= next_region)
              config->send_info("Erasing region of %#02X blocks of %#02X bytes at %#08X", size / block_size, block_size, params->cs_base_addr + current_offset);
          else
              config->send_info("Erasing %#02X blocks in region of %#02X blocks of %#02X bytes at %#08X", size / block_size, nb_blocks, block_size, params->cs_base_addr + current_offset);

          while (current_offset < next_region)
          {
            U32 block_base_addr = params->cs_base_addr + current_offset;
            U32 result_code;

            if (size < block_size)
            {
                *result = "Erase end address don't match block boundary"; 
                config->send_info("%s for %#02X bytes at %#08X", *result, size, block_base_addr);
                return FLASH_DRV_INVALID_PARAMETER;
            }

            actual_size += block_size;
            config->send_status("Erase progress", actual_size, target_size);

            result_code = nor_amd_erase_block(config, result, block_base_addr, block_size);

            if (result_code != FLASH_DRV_SUCCESS)
            {
              return result_code;
            }
            current_offset += block_size;
            size -= block_size;
            if (size == 0)
            {
              //send_ok("Region from %8X to %8X erased (%8X bytes)", config->device_base_address, config->device_base_address + current_offset - 1, current_offset);
              *result = "Erase complete";
              return FLASH_DRV_SUCCESS;
            }
          }
        }
        if (!start_query_nor_flash(config))
        {
            *result = "QUERY ERROR - NEXT";
            return FLASH_DRV_ERROR;
        }
      }
      current_region = next_region;
    }
    *result = "Erase size too big";
    return FLASH_DRV_ERROR;
  }
}

/*-----------------------------------------------------------------------------
| Function    : amd_write_page
+------------------------------------------------------------------------------
| Description : Function for writing a page of data to the flash memory
|
| Parameters  : config  - flash parameters
|               addr    - destination address
|               buffer  - data
|               size    - size of data
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 amd_write_page(T_driver_config * config, U32 addr, U16 *buffer, U32 size)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *) config->data;
  U16 *p_dst = (U16 *) addr;
  U16 *p_src = (U16 *) buffer;
  U32 cs_addr = pNOR->cs_base_addr;
  U32 word_size = size >> 1;
  U32 cmd1, cmd2;

  cmd1 = HEX_555;
  cmd2 = HEX_2AA;

  LOG_EVENT("WR PAGE");
  LOG_EVENT("S: Unlock sector");

  FLASH_WRITE(cs_addr + cmd1, 0xF0);  /* Added for MV */
  FLASH_WRITE(cs_addr + cmd1, 0x60); /* Sector Unlock Sequence (addr is dont-care) */
  FLASH_WRITE(cs_addr + cmd1, 0x60); /* Sector Unlock Sequence (addr is dont-care) */
  FLASH_WRITE((addr | 0x84) & (~(0x2)), 0x60); /* Sector Unlock Sequence (address 6 = VIH, address 1 = VIH (needed for Samsung memory), address 0 = VIL(needed for Samsung memory)) */
  FLASH_WRITE(cs_addr + cmd1, 0xF0);  /* Exit locking sequence */

  LOG_EVENT("E: Unlock sector");

  LOG_EVENT("S: program cycle");
  while ((result_code == FLASH_DRV_SUCCESS) && (word_size > 0))
  {
    FLASH_WRITE(cs_addr + cmd1, 0xAA); /* AMD Unlock Cycle 1 */
    FLASH_WRITE(cs_addr + cmd2, 0x55); /* AMD Unlock Cycle 2 */

    FLASH_WRITE(cs_addr + cmd1, 0xA0); /* AMD Program Command */
    FLASH_WRITE((U32)p_dst, *p_src);

    result_code = poll_status_bits(p_dst);

    p_dst++;
    p_src++;
    word_size--;
  }
  LOG_EVENT("E: program cycle");

  return result_code;
}


/*-----------------------------------------------------------------------------
| Function    : amd_write_page_buffered
+------------------------------------------------------------------------------
| Description : Function for writing a page of data to the flash memory (buffered)
|
| Parameters  : config  - flash parameters
|               addr    - destination address
|               buffer  - data
|               size    - size of data
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 amd_write_page_buffered(T_driver_config * config, U32 addr, U16 * buffer, U32 size)
{
  T_NOR_PARAMS * pNOR        = (T_NOR_PARAMS *)config->data;
  U32            cs_addr     = pNOR->cs_base_addr;
  U8             result_code = FLASH_DRV_SUCCESS;
  U16            word_size   = (U16)(size >> 1); /* 16-bit word size assumed */
  U16          * p_dst       = (U16 *)addr;
  U16          * p_src       = (U16 *)buffer;
  U16          * p_sector    = (U16 *)((U32)p_dst & SECTOR_MASK_32K);
  U32            cmd1, cmd2;

  cmd1 = HEX_555;
  cmd2 = HEX_2AA;

  LOG_EVENT("WR PAGE BUF");
  LOG_EVENT("S: Unlock sector");
  FLASH_WRITE(cs_addr + cmd1, 0xF0);
  FLASH_WRITE(cs_addr + cmd1, 0x60);
  FLASH_WRITE(cs_addr + cmd1, 0x60);
  FLASH_WRITE((addr | 0x84) & (~(0x2)), 0x60); /* Sector Unlock Sequence (address 6 = VIH, address 1 = VIH (needed for Samsung memory), address 0 = VIL(needed for Samsung memory)) */
  FLASH_WRITE(cs_addr + cmd1, 0xF0);  /* Exit locking sequence */
  LOG_EVENT("E: Unlock sector");

  LOG_EVENT("S: Program cycle");
  FLASH_WRITE(cs_addr + cmd1, 0xAA); /* Unlock command 1 */
  FLASH_WRITE(cs_addr + cmd2, 0x55); /* Unlock command 2 */

  FLASH_WRITE((U32)p_sector, 0x25);
  FLASH_WRITE((U32)p_sector, word_size - 1);
  while(word_size > 0)
  {
    FLASH_WRITE((U32)p_dst, *p_src);
    p_dst++;
    p_src++;
    word_size--;
  }
  
  FLASH_WRITE((U32)p_sector, 0x29);
  LOG_EVENT("E: Program cycle");

  LOG_EVENT("S: poll status");
  result_code = poll_status_dq7(--p_dst);
  LOG_EVENT("E: poll status");

  return(result_code);
} 


/*-----------------------------------------------------------------------------
| Function    : 
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : 
+----------------------------------------------------------------------------*/
U32 nor_amd_write(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more)
{
  U8 buffer_write_flag=FALSE, write_flag=FALSE;
  U32 offset = 0, start_offset = 0, write_size = 0;
  T_NOR_PARAMS *params = (T_NOR_PARAMS *)config->data;

  U8 result_code = FLASH_DRV_SUCCESS;
  U8 *data_ptr = (U8 *)params->data_buffer;

  *result = NULL;

  /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
  if(dst_addr > 0xFFFFFFFF)
  {
    config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", dst_addr);
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }

  do
  {
    if(params->buf_valid)
    {
      if(((params->buf_write_addr & PAGE_MASK) + PAGE_SIZE) > dst_addr)
      { /* write_req to same region as wirte buffer content */
        if((params->buf_write_addr & PAGE_MASK) + PAGE_SIZE < dst_addr + size)
        { /* Flash region is full: write data from write buffer */
          write_size = (params->buf_write_addr & PAGE_MASK) + PAGE_SIZE -dst_addr;
          buffer_write_flag = TRUE;
        }
        else
        { /* Flash region is not full yet */
          write_size = size;

          if(more == NO_MORE_DATA)
          { /* Write buffer content if last write_req received
            (else await next write_req). */
            buffer_write_flag = TRUE;
          }
        }

        /* Fill empty part of write buffer with 0xFFs. */
        config->drv_memset(data_ptr + params->buf_write_size, 0xFF, dst_addr -params->buf_write_addr -params->buf_write_size);

        /* Copy data into write buffer. */
        config->drv_memcpy(data_ptr + dst_addr - params->buf_write_addr, (U8 *)src_addr, write_size);

        params->buf_write_size = dst_addr -params->buf_write_addr + write_size;
      }
      else
      { /* write_req to new region compared to write buffer content */
        buffer_write_flag = TRUE;
      }
    }
    else
    {
      start_offset = ((dst_addr + offset) % PAGE_SIZE);

      if(start_offset + size - offset < PAGE_SIZE)
      { /* Flash region is not full yet */
        write_size = size -offset;

        if(more == NO_MORE_DATA)
        { /* Write data if last write_req received. */
          if((start_offset & 1) || ((src_addr + offset) & 1))
          { /* Fill data buffer if start write addr or src_addr
            not word-aligned. */
            fill_data_buffer(config, params, dst_addr + offset,
                             (U8 *)(src_addr + offset), write_size);
            buffer_write_flag = TRUE;
          }
          else
          {
            write_flag = TRUE;
          }
        }
        else
        { /* Copy data into write buffer and await next write_req
          if more write_reqs to come. */
          fill_data_buffer(config, params, dst_addr + offset,
                           (U8 *)(src_addr + offset), write_size);
        }
      }
      else
      { /* Flash region is full: write data */
        write_size = PAGE_SIZE - start_offset;

        if((start_offset % 2) || ((src_addr + offset) % 2))
        { /* Fill data buffer if start write addr or src_addr
          not word-aligned. */
          fill_data_buffer(config, params, dst_addr + offset,
                           (U8 *)(src_addr + offset), write_size);
          buffer_write_flag = TRUE;
        }
        else
        {
          write_flag = TRUE;
        }
      }
    }

    if(buffer_write_flag)
    {
      /* Write data from write buffer*/
      
      if(params->wrbuf)
      {
        result_code = amd_write_page_buffered(config, params->buf_write_addr, params->data_buffer, params->buf_write_size);
      }
      else
      {
        result_code = amd_write_page(config, params->buf_write_addr, params->data_buffer, params->buf_write_size);
      }

      /* Clean-up */
      params->buf_valid = FALSE;
      buffer_write_flag = FALSE;
    }
    else if(write_flag)
    {
      /* Write data from write_req*/
      if(params->wrbuf)
      {
        result_code = amd_write_page_buffered(config, dst_addr + offset, (U16 *)(src_addr + offset), write_size);
      }
      else
      {
        result_code = amd_write_page(config, dst_addr + offset, (U16 *)(src_addr + offset), write_size);
      }

      /* Clean-up */
      write_flag = FALSE;
    }

    offset += write_size;
  } while((result_code==FLASH_DRV_SUCCESS) && (offset<size));

  if(result_code != FLASH_DRV_SUCCESS)
  {
    config->send_info("WRITE FAILED AT ADDRESS %#08X", dst_addr + offset);
    *result = "FAILED TO WRITE DATA FOR BLOCK";
  }

  return result_code;
}

/*-----------------------------------------------------------------------------
| Function    : nor_amd_read
+------------------------------------------------------------------------------
| Description : Read range from flash
|
| Parameters  : p_flash  [IN] - flash parameters (not used)
|               dst_addr [IN] - destination address
|               src_addr [IN] - source address
|               size     [IN] - size of data
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
U32 nor_amd_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size)
{
  /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
  if(src_addr > 0xFFFFFFFF)
  {
    config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", src_addr);
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }

  *result = NULL;    
  config->drv_memcpy ((void *)dst_addr, (const void *)src_addr, size);
  return FLASH_DRV_SUCCESS;
}

/*==== END OF FILE ==========================================================*/
