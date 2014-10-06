/**
 * @file nor_inter_sibley_drv.c
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
 * Intel Sibley NOR Flash driver
 */

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES ==============================================================*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "types.h"
#include "flash_drv.h"
#include "nor_intel_sibley_drv.h"

/*==== MACROS ================================================================*/

/*==== CONSTS ================================================================*/
#ifndef SIMULATION
#pragma DATA_SECTION(driver_if,".consttbl");
#endif
const T_driver_if driver_if = 
{
  {
    OMAPFLASH_DRIVER_HEADER_STRING_V7,
    "NOR INTEL SIBLEY",
    CTRL_Z
  },
  {
    &driver_if,
    nor_intel_sibley_init,
    nor_intel_sibley_read,
    nor_intel_sibley_write,
    nor_intel_sibley_erase,
    nor_intel_sibley_deinit,
    nor_intel_sibley_get_info
  }
};

typedef enum
{
  C_ADDRESS
} T_driver_setup_index;

const T_driver_setup_const setup_const[] =
{
  { "address", DEFAULT, TRUE  },
  { "",        DEFAULT, FALSE }
};

T_driver_setup_var setup_var[sizeof(setup_const) / sizeof(T_driver_setup_const)];

/*==== TYPES =================================================================*/

/*==== LOCALS ================================================================*/

U8 query_nor_flash_number_block_regions(T_driver_config * config);
U32 query_nor_flash_erase_block_info(T_driver_config * config, U8 block_index);
BOOLEAN start_query_nor_flash(T_driver_config * config);

/*==== PRIVATE FUNCTIONS =====================================================*/

/*-----------------------------------------------------------------------------
| Function    : sibley_soft_reset
+------------------------------------------------------------------------------
| Description : Software reset of flash device (places partition in "Read
|               Array" mode.
|
| Parameters  : addr     - Address within partition
|
| Returns     : void
+-----------------------------------------------------------------------------*/
static void sibley_soft_reset(U32 addr)
{
    /* Set partition in "Read Array" mode */
    FLASH_WRITE(addr, 0xFF);
}

/*-----------------------------------------------------------------------------
| Function    : sibley_unlock_block
+------------------------------------------------------------------------------
| Description : Unlock block
|
| Parameters  : addr     - Address within block
|
| Returns     : void
+-----------------------------------------------------------------------------*/
static void sibley_unlock_block(U32 addr)
{
    FLASH_WRITE(addr, LOCK_SETUP);
    FLASH_WRITE(addr, BLOCK_UNLOCK);
    FLASH_WRITE(addr, READ_ARRAY);
    return;
}

/*-----------------------------------------------------------------------------
| Function    : sibley_lock_block
+------------------------------------------------------------------------------
| Description : Lock block
|
| Parameters  : addr     - Addresss within block
|
| Returns     : void
+-----------------------------------------------------------------------------*/
static void sibley_lock_block(U32 addr)
{
    FLASH_WRITE(addr, LOCK_SETUP);
    FLASH_WRITE(addr, BLOCK_LOCK);
    FLASH_WRITE(addr, READ_ARRAY);
    return;
}

/*-----------------------------------------------------------------------------
| Function    : sibley_wait_ready
+------------------------------------------------------------------------------
| Description : Wait until device is ready.
|
| Parameters  : addr     - Address within partition
|
| Returns     : status
+-----------------------------------------------------------------------------*/
static U16 sibley_wait_ready(U32 addr)
{
    U16 status;

    /* Wait until device is ready */
    do
        status = FLASH_READ(addr);
    while ((status & DEVICE_READY) == 0);

    return status;
}

/*-----------------------------------------------------------------------------
| Function    : nor_write_page
+------------------------------------------------------------------------------
| Description : Sibley NOR write one page (region)
|
| Parameters  : addr     - Start write address
|               buffer   - Pointer to U16 buffer containing the input data
|               size     - Number of bytes to write
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
static U8 sibley_write_page(U32 addr, U16 *buffer, U32 size)
{
    U16 i, status;

    U8 ret_val = FLASH_DRV_SUCCESS;

    U16 sizeWords;
    U8 oddByteCount = 0;

    if (size & 1 || size >= 2<<17)
    {
        ret_val = FLASH_DRV_ERR_BAD_SIZE;
    }
    else
    {
	    sizeWords = (U16)(size >> 1);

	    oddByteCount = size & 0x0001;

        /* Unlock block */
        sibley_unlock_block(addr);

        /* Prepare buffered write.  */
        FLASH_WRITE(addr, WRITE_BUFFER);

        /* Write "word count" minus one. */
        if(oddByteCount)
            FLASH_WRITE(addr, sizeWords);
        else
            FLASH_WRITE(addr, sizeWords - 1);

        /* Fill buffer using word writes. */
        for (i = 0; i < sizeWords; ++i)
        {
            FLASH_WRITE(addr + 2 * i, *buffer);
            buffer++;
        }

        /* Write odd byte to buffer. */
        if(oddByteCount)
        {
            FLASH_WRITE(addr + 2 * i, 0xFF00 + ((*buffer) & 0xFF));
        }

        /* Confirm "write buffer" command and wait until it completes. */
        FLASH_WRITE(addr, CONFIRM);

        status = sibley_wait_ready(addr);

        if(status & VPP_ERROR)              /* Vpp range error */
        {
            ret_val =  FLASH_DRV_ERR_VOLTAGE;
        }
        else if(status & PROGRAM_ERROR)         /* Programming failed */
        {
            /* Following errors are Sibley specific*/
            if((status & OBJ_MODE_VIOLATION) && (status & CTRL_MODE_VIOLATION))
                ret_val = FLASH_DRV_ERR_ILLEGAL_WRITE_CMD;
            else if(status & OBJ_MODE_VIOLATION)
                ret_val =  FLASH_DRV_ERR_REWRITE_OBJ_DATA;
            else if(status & CTRL_MODE_VIOLATION)
                ret_val =  FLASH_DRV_ERR_WRITE_OBJ_DATA_TO_CTRL_REGION;
            else ret_val = FLASH_DRV_ERR_WRITE;
        }
        else if(status & LOCK_ERROR)         /* Device protection error */
        {
            ret_val =  FLASH_DRV_ERR_PROTECTION;
        }

        if(ret_val != FLASH_DRV_SUCCESS)
        {
            /* Reset status.*/
            FLASH_WRITE(addr, CLEAR_STATUS);
        }

        /* Return to "read array" mode. */
        FLASH_WRITE(addr, READ_ARRAY);
    }

    return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    : fill_data_buffer
+------------------------------------------------------------------------------
| Description : The function copies input data into write_buffer.
|
| Parameters  : flash    - Flash parms
|               dst_addr - Start write address
|               src_ptr  - Pointer to input data
|               size     - Number of bytes to write
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
static void fill_data_buffer(T_driver_config * config, T_SIBLEY_PARAMS *params, U32 dst_addr, U8 *src_ptr,
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

/*==== PUBLIC FUNCTIONS ======================================================*/

/*-----------------------------------------------------------------------------
| Function    : nor_intel_sibley_init
+------------------------------------------------------------------------------
| Description : Sibley NOR Flash initialization (queries flash buffer size)
|
| Parameters  : flash    - Flash parms
|               addr     - Address (don't care)
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_intel_sibley_init(T_driver_config * config, char ** result)
{
  T_driver_setup_var *setup = get_setup_var();  

  T_SIBLEY_PARAMS *params;
  *result = NULL;

  /* Parse configuration string */

  if(drv_parse_config(get_setup_const(), setup, config->cstring, config) == FLASH_DRV_ERROR)
  {
    *result = "UNABLE TO FIND CONFIGURATION PARAMETER DURING INITIALIZATION";
    return FLASH_DRV_ERROR;
  }

  /* Update callback functions pointers and check that the necessary functions are there */

  config->data = config->drv_malloc(sizeof(T_SIBLEY_PARAMS));

  params = (T_SIBLEY_PARAMS *)(config->data);
  
  if(!params)
  {
    *result = "DRIVER DATA MEMORY ALLOCATION ERROR DURING INITIALIZATION";
    return FLASH_DRV_ERROR;
  }

  params->device_base_address = setup[C_ADDRESS].value;
  params->device_size         = 0;
  params->buf_valid           = FALSE;

  /* Find the size parameters */ 
  
  if(start_query_nor_flash(config))
  {
    U8 number_block_regions = query_nor_flash_number_block_regions(config);
    U8 i;
  
    config->send_info("SIBLEY DRV: CFI compliant memory detected");
    config->send_info("SIBLEY DRV: %d region(s)", number_block_regions);

    for(i = 0; i < number_block_regions; i++) 
    {
      U32 erase_block_info = query_nor_flash_erase_block_info(config, i);
      U32 block_size = (erase_block_info & 0xFFFF0000) >> 8;
      U32 nb_blocks = (erase_block_info & 0xFFFF) + 1;
      U32 region_size = nb_blocks * block_size;
      config->send_info("SIBLEY DRV: region %d - %d blocks of %d bytes (%d bytes)", i, nb_blocks, block_size, region_size);
      params->device_size += region_size;
    }
    config->send_info("SIBLEY DRV: total size %d bytes", params->device_size);
    sibley_soft_reset(params->device_base_address);
  }

  config->send_info("SIBLEY DRV: init complete");

  return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : nor_intel_sibley_deinit
+------------------------------------------------------------------------------
| Description : Sibley NOR Flash deinitialization
|
| Parameters  : flash    - Flash parms
|               result   - Never returned
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_intel_sibley_deinit(T_driver_config * config, char ** result)
{
  config->drv_free(config->data);
  config->data = NULL;
  config->send_info("NOR INTEL SIBLEY DRIVER DEINIT COMPLETE");
  return FLASH_DRV_SUCCESS;
}


/*-----------------------------------------------------------------------------
| Function    : nor_intel_sibley_get_info
+------------------------------------------------------------------------------
| Description : Sibley NOR Flash info
|
| Parameters  : |
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_intel_sibley_get_info(T_driver_config * config, T_driver_info * info)
{
  T_SIBLEY_PARAMS * params  = (T_SIBLEY_PARAMS *)config->data;
  info->device_base_address = params->device_base_address;
  info->device_size         = params->device_size;
  return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : nor_intel_sibley_erase
+------------------------------------------------------------------------------
| Description : Sibley NOR Flash erase
|
| Parameters  : flash    - Flash parms
|               address  - Block address
|               length   - Number of bytes to erase (don't care)
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_intel_sibley_erase_block(T_driver_config * config, char ** result, U32 address, U32 length)
{
  U16 status;
  U8 result_code = FLASH_DRV_SUCCESS;

  /* Unlock block */
  sibley_unlock_block(address);

  /* Send "block erase" command sequence. */
  FLASH_WRITE(address, BLOCK_ERASE);
  FLASH_WRITE(address, CONFIRM);

  /* Wait for erase operation to complete. */
  status = sibley_wait_ready(address);

  /* Return to "read array" mode. */
  FLASH_WRITE(address, READ_ARRAY);


  if(status & VPP_ERROR)                          /* Vpp range error */
  {
    *result = "FLASH PROGRAMMING VOLTAGE ERROR";
    result_code = FLASH_DRV_ERR_VOLTAGE;
  }
  else if(status & ERASE_ERROR)
  {
    if(status & PROGRAM_ERROR)                  /* Sequence error -> program error bit set as well */
    {
      *result = "BAD FLASH PROGRAMMING SEQUENCE";
      result_code = FLASH_DRV_ERR_BAD_SEQUENCE;
    }
    else                                        /* Erase error */
    {
      *result = "FLASH BLOCK ERASE ERROR";
      result_code = FLASH_DRV_ERR_ERASE;
    }
  }
  else if (status & LOCK_ERROR)                 /* Device protection error */
  {
    *result = "FAILED TO ERASE PROTECTED BLOCK";
    result_code = FLASH_DRV_ERR_PROTECTION;
  }

  if(result_code != FLASH_DRV_SUCCESS)
  {
    FLASH_WRITE(address, CLEAR_STATUS);
    /* Put back into Read Array mode.*/
    FLASH_WRITE(address, 0x00FF);        /* INTEL_READ_MODE;*/
  }
  else
  {
    /* Lock block */
    sibley_lock_block(address);
  }

  return result_code;
}

/** Start a NOR flash queary acording to JEDEC Standard No. 68.01
 * @see http://www.jedec.org/download/search/jesd68-01.pdf for details
 */
BOOLEAN start_query_nor_flash(T_driver_config * config)
{
    T_SIBLEY_PARAMS * params = (T_SIBLEY_PARAMS *)(config->data);
    U32 addr = params->device_base_address;
    U16 parms[3];

    /* Query CFI parms */
    FLASH_WRITE (addr + 0xAA,   0xF0); /* AMD and SST reset command */
    FLASH_WRITE (addr, 0xFF);          /* Intel read-array/reset command */

    FLASH_WRITE (addr + 0xAA,   0x98); // Set CFI query mode

    /* Check if x16 CFI-compatible device in 16-bit mode */
    parms[0] = FLASH_READ(addr + 0x20);
    parms[1] = FLASH_READ(addr + 0x22);
    parms[2] = FLASH_READ(addr + 0x24);

    /* Read CFI parameters if data == 'QRY' */
    if((U8)parms[0]=='Q' && (U8)parms[1]=='R' && (U8)parms[2]=='Y') 
    {
        return TRUE;
    }
    else 
    {
        config->send_info("flash is non CFI (found: %#04X %#04X %#04X)", parms[0], parms[1], parms[2]);
        return FALSE;
    }
}


/** Query number of blocks in NOR flash (acording to JEDEC Standard No. 68.01)
 * assume start_query_nor_flash has already been called
 * @see http://www.jedec.org/download/search/jesd68-01.pdf for details
 */
U8 query_nor_flash_number_block_regions(T_driver_config * config) ///<@todo make common file with shared code in 2nd for query_nor_flash_number_block_regions
{
    T_SIBLEY_PARAMS * params = (T_SIBLEY_PARAMS *)(config->data);
    U32 addr = params->device_base_address;
    U8 number_block_regions = (U8)FLASH_READ(addr + 0x58);
    return number_block_regions;
}

/** Query NOR flash acording to JEDEC Standard No. 68.01
 * assume start_query_nor_flash has already been called
 * @see http://www.jedec.org/download/search/jesd68-01.pdf for details
 * @param block_index must be <= value returned by query_nor_flash_number_block_regions
 */
U32 query_nor_flash_erase_block_info(T_driver_config * config, U8 block_index)
{
    T_SIBLEY_PARAMS * params = (T_SIBLEY_PARAMS *)(config->data);
    U32 addr = params->device_base_address;
    U32 erase_block_info 
		= FLASH_READ(addr + 0x5A + 8*block_index) 
		+ (FLASH_READ(addr + 0x5C + 8*block_index)<<8)
        + (FLASH_READ(addr + 0x5E + 8*block_index)<<16) 
		+ (FLASH_READ(addr + 0x60 + 8*block_index)<<24);
    return erase_block_info;
}

U32 nor_intel_sibley_erase(T_driver_config * config, char ** result, U64 address, U64 length)
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
        *result = "start_query_nor_flash drv first";
        return FLASH_DRV_ERROR;
    }
    else {
        T_SIBLEY_PARAMS * params = (T_SIBLEY_PARAMS *)(config->data);
        U32 offset = address - params->device_base_address;
        U32 size = length;
        U8 number_block_regions = query_nor_flash_number_block_regions(config);
        U32 current_region = 0;
        U32 current_offset = offset;
        U32 target_size = size;
        U8 i;
        for(i=0; i < number_block_regions; i++) {
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
                        config->send_info("Erase start address does not match block boundary at %#08X", params->device_base_address + offset);
                    else
                        config->send_info("Internal Error: Block start address does not match block boundary at %#08X", params->device_base_address + offset);
                    return FLASH_DRV_INVALID_PARAMETER;
                }
                else
                {
                    if (current_offset + size >= next_region)
                        config->send_info("Erasing region of %#02X blocks of %#02X bytes at %#08X", size / block_size, block_size, params->device_base_address + current_offset);
                    else
                        config->send_info("Erasing %#02X blocks in region of %#02X blocks of %#02X bytes at %#08X", size / block_size, nb_blocks, block_size, params->device_base_address + current_offset);

                    while (current_offset < next_region)
                    {
                        U32 block_base_addr = params->device_base_address + current_offset;
                        U32 result_code;
                        if (size < block_size)
                        {
                            *result = "Erase end address don't match block boundary"; 
                            config->send_info("%s for %#02X bytes at %#08X", *result, size, block_base_addr);
                            return FLASH_DRV_INVALID_PARAMETER;
                        }
                        actual_size += block_size;
                        config->send_status("Erase progress", actual_size, target_size);
                        result_code = nor_intel_sibley_erase_block(config, result, block_base_addr, block_size);
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
                    *result = "start_query_nor_flash drv next";
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
| Function    : nor_intel_sibley_write
+------------------------------------------------------------------------------
| Description : Sibley NOR Flash write
|
| Parameters  : flash     - Flash parms
|               dst_addr  - Destination address
|               src_addr  - Source address (pointer to input data)
|               size      - Number of bytes to write
|               flags_ptr - Driver flags
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_intel_sibley_write(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more)
{
  U8 buffer_write_flag=FALSE, write_flag=FALSE;
  U32 offset = 0, start_offset = 0, write_size = 0;
  T_SIBLEY_PARAMS *params = (T_SIBLEY_PARAMS *)config->data;

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
            fill_data_buffer(config, params, dst_addr + offset, (U8 *)(src_addr + offset), write_size);
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
          fill_data_buffer(config, params, dst_addr + offset, (U8 *)(src_addr + offset), write_size);
        }
      }
      else
      { /* Flash region is full: write data */
        write_size = PAGE_SIZE - start_offset;

        if((start_offset % 2) || ((src_addr + offset) % 2))
        { /* Fill data buffer if start write addr or src_addr
          not word-aligned. */
          fill_data_buffer(config, params, dst_addr + offset, (U8 *)(src_addr + offset), write_size);
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
      result_code = sibley_write_page(params->buf_write_addr, params->data_buffer,
                                      params->buf_write_size);

      /* Clean-up */
      params->buf_valid = FALSE;
      buffer_write_flag = FALSE;
    }
    else if(write_flag)
    {
      /* Write data from write_req*/
      result_code = sibley_write_page(dst_addr + offset, (U16 *)(src_addr + offset), write_size);

      /* Clean-up */
      write_flag = FALSE;
    }

    offset += write_size;
  } while((result_code==FLASH_DRV_SUCCESS) && (offset<size));

  if(result_code != FLASH_DRV_SUCCESS)
  {
    config->send_info("WRITE FAILED AT ADDRESS %#08x", dst_addr + offset);
    *result = "FAILED TO WRITE DATA FOR BLOCK";
  }

  return result_code;
}

/*-----------------------------------------------------------------------------
| Function    : nor_intel_sibley_read
+------------------------------------------------------------------------------
| Description : Sibley NOR Flash read
|
| Parameters  : flash     - Flash parms
|               dst_addr  - Destination address (pointer to buffer for data)
|               src_addr  - Source address (start read address)
|               size      - Number of bytes to read
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_intel_sibley_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size)
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

/*-----------------------------------------------------------------------------*/
