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
 * AMD NOR Flash driver
 */

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES =============================================================*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "types.h"
#include "flash_drv.h"
#include "nor_numonyx_drv.h"

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
    "NOR NUMONYX",
    CTRL_Z
  },
  {
    &driver_if,
    nor_numonyx_init,
    nor_numonyx_read,
    nor_numonyx_write,
    nor_numonyx_erase,
    nor_numonyx_deinit,
    nor_numonyx_get_info
  }
};

typedef enum
{
  C_ADDRESS,
  C_ECHECK,
  C_WMODE,
  C_LOG
} T_driver_setup_index;

const T_driver_setup_const setup_const[] =
{
  { "address", DEFAULT, TRUE  },
  { "echeck",  DEFAULT, FALSE },
  { "wmode",   DEFAULT, FALSE },
  { "log",     DEFAULT, FALSE },
  { "",        DEFAULT, FALSE }
};

T_driver_setup_var setup_var[sizeof(setup_const) / sizeof(T_driver_setup_const)];

/*==== TYPES ================================================================*/

/*==== LOCALS ===============================================================*/
/*==== PRIVATE FUNCTIONS ====================================================*/
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

  /* Make sure we are in a good state */
  FLASH_WRITE(pNOR->cs_base_addr, CMD_READ_ARRAY);

  /* Query CFI parameters */
  FLASH_WRITE(pNOR->cs_base_addr, CMD_READ_CFI_QUERY); 

  /* Check if x16 CFI-compatible device in 16-bit mode */
  parms[0] = FLASH_READ(pNOR->cs_base_addr + CFI_Q);
  parms[1] = FLASH_READ(pNOR->cs_base_addr + CFI_R);
  parms[2] = FLASH_READ(pNOR->cs_base_addr + CFI_Y);

  /* Read CFI parameters if data == "QRY" */
  if (parms[0] == 'Q' && parms[1] == 'R' && parms[2] == 'Y')
  {
    return TRUE;
  }
  else
  {
    config->send_info("Flash is non CFI (found: %#04X %#04X %#04X)", parms[0], parms[1], parms[2]);
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
  U8 number_block_regions = (U8)FLASH_READ(pNOR->cs_base_addr + CFI_BLOCK_REGIONS);
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
  U32 erase_block_info; 
    
  /* Make sure we are in a good state */
  FLASH_WRITE(pNOR->cs_base_addr, CMD_READ_ARRAY);

  /* Query CFI parameters */
  FLASH_WRITE(pNOR->cs_base_addr, CMD_READ_CFI_QUERY); 
  
  erase_block_info = (FLASH_READ(pNOR->cs_base_addr + CFI_NUM_ERASE_BLOCKS_LOW(block_index))) + 
                     (FLASH_READ(pNOR->cs_base_addr + CFI_NUM_ERASE_BLOCKS_HIGH(block_index)) << 8) + 
                     (FLASH_READ(pNOR->cs_base_addr + CFI_SIZE_ERASE_BLOCKS_LOW(block_index)) << 16) + 
                     (FLASH_READ(pNOR->cs_base_addr + CFI_SIZE_ERASE_BLOCKS_HIGH(block_index)) << 24);

  FLASH_WRITE(pNOR->cs_base_addr, CMD_READ_ARRAY);
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
| Function    : nor_numonyx_init
+------------------------------------------------------------------------------
| Description : Flash initialization (queries write buffer size)
|
| Parameters  : p_flash [OUT] - flash parameters
|               addr     [IN] - address
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
U32 nor_numonyx_init(T_driver_config * config, char ** result)
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
  pNOR->echeck       = setup[C_ECHECK].valid ? setup[C_ECHECK].value : FALSE;
  pNOR->wmode        = setup[C_WMODE].valid ? (T_write_mode)setup[C_WMODE].value : write_single_word; 
  pNOR->device_size  = 0;
  pNOR->buf_valid    = FALSE;

  if(start_query_nor_flash(config))
  {
    pNOR->device_size = 1 << FLASH_READ(pNOR->cs_base_addr + CFI_DEVICE_SIZE);
  }
  else
  {
    *result = "CFI QUERY FAILED - NOT A CFI DEVICE";
    return FLASH_DRV_ERROR;
  }

  config->send_info("- Manufacturer: %#04X", FLASH_READ(pNOR->cs_base_addr + CFI_MANUFACTURER_CODE));
  config->send_info("- Device code : %#04X", FLASH_READ(pNOR->cs_base_addr + CFI_DEVICE_CODE));
  config->send_info("- Device size : %d", pNOR->device_size);
  
  config->send_info("Resetting device...");

  FLASH_WRITE (pNOR->cs_base_addr, CMD_READ_ARRAY);
  FLASH_WRITE(pNOR->cs_base_addr, CMD_CLEAR_STATUS_REG);

  config->send_info("Initialization complete...");
  
  return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : nor_numonyx_deinit
+------------------------------------------------------------------------------
| Description : AMD NOR Flash deinitialization
|
| Parameters  : config   - Flash parms
|               result   - Returned status
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_numonyx_deinit(T_driver_config * config, char ** result)
{
  config->drv_free(config->data);
  config->data = NULL;
  config->send_info("NOR AMD DRIVER DEINIT COMPLETE");
  return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : nor_numonyx_get_info
+------------------------------------------------------------------------------
| Description : AMD NOR Flash info
|
| Parameters  : config   - Flash parms
|               info     - Returned information
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 nor_numonyx_get_info(T_driver_config * config, T_driver_info * info)
{
  T_NOR_PARAMS * pNOR  = (T_NOR_PARAMS *)config->data;
  info->device_base_address = pNOR->cs_base_addr;
  info->device_size         = pNOR->device_size;
  return FLASH_DRV_SUCCESS;
}

U32 nor_numonyx_unlock_block(T_driver_config * config, U32 address)
{
  T_NOR_PARAMS * pNOR  = (T_NOR_PARAMS *)config->data;
  U8 result_code = FLASH_DRV_SUCCESS;

  FLASH_WRITE(address, CMD_READ_ARRAY);
  LOG_EVENT("S: Unlock block");
  FLASH_WRITE(address, CMD_BLOCK_UNLOCK_SETUP);
  FLASH_WRITE(address, CMD_BLOCK_UNLOCK_CONFIRM);
  FLASH_WRITE(address, CMD_READ_ELECTRONIC_SIGNATURE);
  if(FLASH_READ(address) & 0x0001)
  {
    LOG_EVENT("Unlock failed");
    result_code = FLASH_DRV_ERROR;
  }
  FLASH_WRITE(address, CMD_READ_ARRAY);
  LOG_EVENT("E: Unlock block");
  
  return(result_code);
}


U8 numonyx_check_sr_value(U32 address, U16 value, char ** result, U8 check_seq)
{
  U8 result_code = FLASH_DRV_SUCCESS;

  if(STATUS_QCHECK_NO_SEQ(value) || (STATUS_QCHECK_SEQ(value) && check_seq))
  {
    if(STATUS_VPP(value))
    {
      result_code = FLASH_DRV_ERROR;
      *result = "VPP STATUS ERROR";
    }
    else if(STATUS_PROGRAM(value) && STATUS_ERASE(value) && check_seq)
    {
      result_code = FLASH_DRV_ERROR;
      *result = "COMMAND SEQUENCE ERROR";
    }
    else if(STATUS_PROGRAM(value))
    {
      result_code = FLASH_DRV_ERROR;
      *result = "PROGRAM ERROR";
    }
    else if(STATUS_BLOCK_PROTECTION(value))
    {
      result_code = FLASH_DRV_ERROR;
      *result = "BLOCK PROTECTION ERROR";
    }

    FLASH_WRITE(address, CMD_CLEAR_STATUS_REG);
  }

  return(result_code);
}

/*-----------------------------------------------------------------------------
| Function    : nor_numonyx_erase
+------------------------------------------------------------------------------
| Description : Erase flash sector
|
| Parameters  : p_flash [IN] - flash parameters (not used)
|               addr    [IN] - block address
|               length  [IN] - length (not used)
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
U32 nor_numonyx_erase_block(T_driver_config * config, char ** result, U32 address, U32 length)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *) config->data;
  volatile U16 status;

  U16 * pw = (U16 *)(address);
  U16 * pe = (U16 *)(address + length);

  if(pNOR->echeck)
  {
    while(pw < pe)
    {
      if(*pw != 0xFFFF) break;
      pw++;
    }
  }

  if((*pw != 0xFFFF) || (pNOR->echeck == FALSE))
  {
    LOG_EVENT("ERASE BLOCK");
    
    LOG_EVENT("S: Erase block");

    if(nor_numonyx_unlock_block(config, address) == FLASH_DRV_SUCCESS)
    {

      FLASH_WRITE(address, CMD_BLOCK_ERASE_SETUP);
      FLASH_WRITE(address, CMD_BLOCK_ERASE_CONFIRM);
      
      do
      {
        status = FLASH_READ(address);
      } while(!STATUS_P_EC(status));

      if(STATUS_QCHECK_SEQ(status))
      {
        result_code = numonyx_check_sr_value(address, status, result, 1);
      }
   }
    else
    {
        result_code = FLASH_DRV_ERROR;
        *result = "FAILED TO UNLOCK BLOCK";
    }
    LOG_EVENT("E: Erase block");
  }
  FLASH_WRITE(address, CMD_READ_ARRAY);
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
U32 nor_numonyx_erase(T_driver_config * config, char ** result, U64 address, U64 length)
{
  /* driver supports only 32-bit addr. U64 address is just for 2nd API compliance */
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
    *result = "QUERY ERROR - ERASE";
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

      if(length == 0)
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

            result_code = nor_numonyx_erase_block(config, result, block_base_addr, block_size);

            if (result_code != FLASH_DRV_SUCCESS)
            {
              return result_code;
            }
            current_offset += block_size;
            size -= block_size;
            if (size == 0 && i == (number_block_regions - 1))
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
| Function    : numonyx_write_word
+------------------------------------------------------------------------------
| Description : Function for writing a word of data to the device
|
| Parameters  : result - result string for driver
|               addr   - base address
|               dst    - pointer to destination
|               srv    - pointer to source
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 numonyx_write_word(char **result, U32 addr, U16 * dst, U16 * src)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  U16 status;

  FLASH_WRITE((U32)addr, CMD_PROGRAM_SETUP);
  FLASH_WRITE((U32)dst, *src);

  do
  {
    status = FLASH_READ((U32)dst);
  } while(!STATUS_P_EC(status));

  if(STATUS_QCHECK_NO_SEQ(status))
  {
    result_code = numonyx_check_sr_value(addr, status, result, 0);
  }
  return(result_code);
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_write_dword
+------------------------------------------------------------------------------
| Description : Function for writing a double word of data to the device
|
| Parameters  : result - result string for driver
|               addr   - base address
|               dst    - pointer to destination
|               srv    - pointer to source
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 numonyx_write_dword(char **result, U32 addr, U16 * dst, U16 * src)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  U16 status;

  FLASH_WRITE((U32)addr, CMD_DW_PROGRAM_SETUP);
  FLASH_WRITE((U32)dst, *src);
  FLASH_WRITE((U32)(dst + 1), *(src + 1));

  do
  {
    status = FLASH_READ(addr);
  } while(!STATUS_P_EC(status));

  if(STATUS_QCHECK_NO_SEQ(status))
  {
    result_code = numonyx_check_sr_value(addr, status, result, 0);
  }

  return(result_code);
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_write_qword
+------------------------------------------------------------------------------
| Description : Function for writing a quad word of data to the device
|
| Parameters  : result - result string for driver
|               addr   - base address
|               dst    - pointer to destination
|               srv    - pointer to source
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 numonyx_write_qword(char **result, U32 addr, U16 * dst, U16 * src)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  U16 status;

  FLASH_WRITE((U32)addr, CMD_QW_PROGRAM_SETUP);
  FLASH_WRITE((U32)dst, *src);
  FLASH_WRITE((U32)(dst + 1), *(src + 1));
  FLASH_WRITE((U32)(dst + 2), *(src + 2));
  FLASH_WRITE((U32)(dst + 3), *(src + 3));

  do
  {
    status = FLASH_READ((U32)dst);
  } while(!STATUS_P_EC(status));

  if(STATUS_QCHECK_NO_SEQ(status))
  {
    result_code = numonyx_check_sr_value(addr, status, result, 0);
  }

  return(result_code);
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_setup_efact_write
+------------------------------------------------------------------------------
| Description : Function for setting up enhanced factury write sequence
|
| Parameters  : result - result string for driver
|               addr   - base address
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 numonyx_setup_efact_write(char **result, U32 addr)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  U16 status;

  FLASH_WRITE(addr, CMD_READ_ARRAY);
  FLASH_WRITE(addr, CMD_ENH_PROGRAM_SETUP);
  FLASH_WRITE(addr, CMD_E_FACT_PROGRAM_CONFIRM);

  status = FLASH_READ(addr);

  if(STATUS_P_EC(status))
  {
    *result = "P_EC SET AFTER SETUP OF E-FACT PROGRAM";
    result_code = numonyx_check_sr_value(addr, status, result, 0);
    result_code = FLASH_DRV_ERROR;
  }
  else
  {
    while(STATUS_BANK_WRITE(status))
    {
      status = FLASH_READ((U32)addr);
    }
  }

  return(result_code);
}


/*-----------------------------------------------------------------------------
| Function    : numonyx_efact_write_verify
+------------------------------------------------------------------------------
| Description : Function for writing or verifying enhanced factury write data
|
| Parameters  : result - result string for driver
|               addr   - start address
|               src    - pointer to data
|               length - amount of data (in words)
|
| Returns     : Nothing
+----------------------------------------------------------------------------*/
static void numonyx_efact_write_verify(T_driver_config * config, U32 addr, U16 * src, U32 length)
{
  volatile U16 status;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *)config->data;

  while(length)
  {
    FLASH_WRITE((U32)addr, *src);

    do
    {
      status = FLASH_READ((U32)addr);
    } while(STATUS_BANK_WRITE(status));

    src++;
    length--;
  }
  
  if(addr < (pNOR->cs_base_addr + BLOCK_SIZE))
  {
    FLASH_WRITE(addr + BLOCK_SIZE, 0xFFFF);
  }
  else
  {
    FLASH_WRITE(addr - BLOCK_SIZE, 0xFFFF);
  }
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_complete_efact_write
+------------------------------------------------------------------------------
| Description : Function for completing enhanced factury write sequence
|
| Parameters  : result - result string for driver
|               addr   - base address
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
static U8 numonyx_complete_efact_write(char ** result, U32 addr)
{
  U16 status;
  U8 result_code = FLASH_DRV_SUCCESS;

  do
  {
    status = FLASH_READ(addr);
  } while(!STATUS_P_EC(status));

  if(STATUS_QCHECK_NO_SEQ(status))
  {
    result_code = numonyx_check_sr_value(addr, status, result, 0);
  }
  return(result_code);
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_write_page
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
static U8 numonyx_write_page(T_driver_config * config, char ** result, U32 addr, U16 *buffer, U32 size)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *) config->data;
  U16 *p_dst = (U16 *) addr;
  U16 *p_src = (U16 *) buffer;
  U32 word_size = size >> 1;

  if(nor_numonyx_unlock_block(config, addr) == FLASH_DRV_SUCCESS)
  {
    LOG_EVENT("S: Write Page");

    while ((result_code == FLASH_DRV_SUCCESS) && (word_size > 0))
    {
      result_code = numonyx_write_word(result, addr, p_dst, p_src);
      p_dst++;
      p_src++;
      word_size--;
    }

    LOG_EVENT("E: Write Page");
  }

  FLASH_WRITE((U32)addr, CMD_READ_ARRAY);
  return result_code;
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_write_page_double
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
static U8 numonyx_write_page_double(T_driver_config * config, char ** result, U32 addr, U16 *buffer, U32 size)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *) config->data;
  U16 *p_dst = (U16 *) addr;
  U16 *p_src = (U16 *) buffer;
  U32 word_size = size >> 1;

  if(nor_numonyx_unlock_block(config, addr) == FLASH_DRV_SUCCESS)
  {
    LOG_EVENT("S: DWrite Page");

    while ((result_code == FLASH_DRV_SUCCESS) && (word_size > 1))
    {
      result_code = numonyx_write_dword(result, addr, p_dst, p_src);      
      p_dst += 2;
      p_src += 2;
      word_size -= 2;
    }

    LOG_EVENT("E: DWrite Page");
  }

  if((word_size > 0) && (result_code == FLASH_DRV_SUCCESS))
  {
    LOG_EVENT("S: Write 1");
    result_code = numonyx_write_word(result, addr, p_dst, p_src);
    LOG_EVENT("E: Write 1");
  }
  
  FLASH_WRITE((U32)addr, CMD_READ_ARRAY);
  return result_code;
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_write_page_quad
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
static U8 numonyx_write_page_quad(T_driver_config * config, char ** result, U32 addr, U16 *buffer, U32 size)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *) config->data;
  U16 *p_dst = (U16 *) addr;
  U16 *p_src = (U16 *) buffer;
  U32 word_size = size >> 1;

  if(nor_numonyx_unlock_block(config, addr) == FLASH_DRV_SUCCESS)
  {
    LOG_EVENT("S: QWrite Page");

    while ((result_code == FLASH_DRV_SUCCESS) && (word_size > 3))
    {
      result_code = numonyx_write_qword(result, addr, p_dst, p_src);      
      p_dst += 4;
      p_src += 4;
      word_size -= 4;
    }

    LOG_EVENT("E: QWrite Page");
  }

  if((word_size > 1) && (result_code == FLASH_DRV_SUCCESS))
  {
    LOG_EVENT("S: DWrite 1");

    while ((result_code == FLASH_DRV_SUCCESS) && (word_size > 1))
    {
      result_code = numonyx_write_dword(result, addr, p_dst, p_src);      
      p_dst += 2;
      p_src += 2;
      word_size -= 2;
    }

    LOG_EVENT("E: DWrite 1");
  }

  if((word_size > 0) && (result_code == FLASH_DRV_SUCCESS))
  {
    LOG_EVENT("S: Write 1");
    result_code = numonyx_write_word(result, addr, p_dst, p_src);
    LOG_EVENT("E: Write 1");
  }

  FLASH_WRITE((U32)addr, CMD_READ_ARRAY);
  return result_code;
}

/*-----------------------------------------------------------------------------
| Function    : numonyx_write_page_efact
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
static U8 numonyx_write_page_efact(T_driver_config * config, char ** result, U32 addr, U16 *buffer, U32 size)
{
  U8 result_code = FLASH_DRV_SUCCESS;
  T_NOR_PARAMS *pNOR = (T_NOR_PARAMS *) config->data;
  U32 word_size = size >> 1;

  if(nor_numonyx_unlock_block(config, addr) == FLASH_DRV_SUCCESS)
  {
    LOG_EVENT("S: EFWrite Page");
    
    result_code = numonyx_setup_efact_write(result, addr);

    if(result_code == FLASH_DRV_SUCCESS)
    {
      numonyx_efact_write_verify(config, addr, buffer, word_size);
      result_code = numonyx_complete_efact_write(result, addr);
    }

    LOG_EVENT("E: EFWrite Page");
  }
  FLASH_WRITE((U32)addr, CMD_READ_ARRAY);
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
U32 nor_numonyx_write(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more)
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
#if 0
        config->drv_memset(data_ptr + params->buf_write_size, 0xFF, dst_addr - params->buf_write_addr - params->buf_write_size);
#else
        //config->send_info("data_ptr       = %#08X", data_ptr);
        //config->send_info("buf_write_size = %#08X", params->buf_write_size);
        //config->send_info("buf_write_addr = %#08X", params->buf_write_addr);
        //config->send_info("dst_addr       = %#08X", dst_addr);
        //return FLASH_DRV_SUCCESS;
#endif
        /* Copy data into write buffer. */
        config->drv_memcpy(data_ptr + dst_addr - params->buf_write_addr, (U8 *)src_addr, write_size);
        params->buf_write_size = dst_addr - params->buf_write_addr + write_size;
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
      switch(params->wmode)
      {
        case write_single_word:
          result_code = numonyx_write_page(config, result, params->buf_write_addr, params->data_buffer, params->buf_write_size);
          break;

        case write_double_word:
          result_code = numonyx_write_page_double(config, result, params->buf_write_addr, params->data_buffer, params->buf_write_size);
          break;

        case write_quad_word:
          result_code = numonyx_write_page_quad(config, result, params->buf_write_addr, params->data_buffer, params->buf_write_size);
          break;

        case write_single_efact:
          result_code = numonyx_write_page_efact(config, result, params->buf_write_addr, params->data_buffer, params->buf_write_size);
          break;

        default:
          config->send_info("WRITE ACCESS MODE %#08X UNKNOWN", params->wmode);
          result_code = FLASH_DRV_ERROR;
          break;
      }

      /* Clean-up */
      params->buf_valid = FALSE;
      buffer_write_flag = FALSE;
    }
    else if(write_flag)
    {
      /* Write data from write_req*/
      switch(params->wmode)
      {
        case write_single_word:
          result_code = numonyx_write_page(config, result, dst_addr + offset, (U16 *)(src_addr + offset), write_size);
          break;

        case write_double_word:
          result_code = numonyx_write_page_double(config, result, dst_addr + offset, (U16 *)(src_addr + offset), write_size);
          break;

        case write_quad_word:
          result_code = numonyx_write_page_quad(config, result, dst_addr + offset, (U16 *)(src_addr + offset), write_size);
          break;

        case write_single_efact:
          result_code = numonyx_write_page_efact(config, result, dst_addr + offset, (U16 *)(src_addr + offset), write_size);
          break;

        default:
          config->send_info("WRITE ACCESS MODE %#08X UNKNOWN", params->wmode);
          result_code = FLASH_DRV_ERROR;
          break;
      }

      /* Clean-up */
      write_flag = FALSE;
    }

    offset += write_size;
  } while((result_code==FLASH_DRV_SUCCESS) && (offset<size));

  if(result_code != FLASH_DRV_SUCCESS)
  {
    config->send_info("WRITE FAILED AT ADDRESS %#08X", dst_addr + offset);
  }

  return result_code;
}

/*-----------------------------------------------------------------------------
| Function    : nor_numonyx_read
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
U32 nor_numonyx_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size)
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
