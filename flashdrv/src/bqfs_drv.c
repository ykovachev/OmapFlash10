/**
 * @file bqfs_drv.c
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
 */

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES ==============================================================*/
#include "types.h"
#include "flash_drv.h"
//#include "i2cnew.h"
//#include "simulation.h"

#include "romapi_types.h"

#ifdef OMAP3
#include "romapi_3430.h"
#endif

#ifdef OMAP4
#include "romapi_4430.h"
#endif

#ifdef OMAP5
#include "romapi_5430.h"
#endif

#undef RPAPI
#define RPAPI(offset) ((((T_BQFS_PARAMS *)config->data)->rpapi_base) + offset)

/*= === MACROS ================================================================*/

U32 bqfs_init(T_driver_config * config, char ** result);
U32 bqfs_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
U32 bqfs_write(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more);
U32 bqfs_erase(T_driver_config * config, char ** result, U64 addr, U64 length);
U32 bqfs_deinit(T_driver_config * config, char ** result);
U32 bqfs_get_info(T_driver_config * config, T_driver_info * info);

/*==== TYPES =================================================================*/

typedef struct
{
  U32            i2c_id;
  U32            rpapi_base;
  volatile U32 * clk32;
  U32            length;
  U8 *           data;
} T_BQFS_PARAMS;

#define NL 0x0A
#define CR 0x0D
#define MAX_LENGTH 200
#define MAX_IMAGE_SIZE (1024 * 1024)
typedef enum
{
  BQFS_WRITE,
  BQFS_WAIT,
  BQFS_CHECK,
  BQFS_CHMOD
} T_bqfs_command;

typedef struct  
{
  T_bqfs_command command;
  union
  {
    struct  
    {
      U8 device;
      U8 subaddr;
      U8 length;
      U8 data[MAX_LENGTH];
    } write;
    struct  
    {
      U32 period;
    } wait;
    struct  
    {
      U8 device;
      U8 subaddr;
      U8 length;
      U8 data[MAX_LENGTH];
    } check;
    struct  
    {
      U32 time;
      U32 delay;
      U8 device1;
      U8 subaddr1;
      U8 device2;
      U8 subaddr2;
      U8 length1;
      U8 data1[MAX_LENGTH];
    } chmod;
  } content;
} T_element;

/*==== CONSTS ================================================================*/
#ifndef _MSC_VER
#pragma DATA_SECTION(driver_if,".consttbl");
#endif
const T_driver_if driver_if = 
{
  {
    OMAPFLASH_DRIVER_HEADER_STRING_V7,
    "TI BQFS",
    CTRL_Z
  },
  {
    &driver_if,
    bqfs_init,
    bqfs_read,
    bqfs_write,
    bqfs_erase,
    bqfs_deinit,
    bqfs_get_info
  }
};

typedef enum
{
  C_I2C_ID,
  C_RPAPI_BASE,
  C_CLK32
} T_driver_setup_index;

const T_driver_setup_const setup_const[] =
{
  { "i2c_id",      DEFAULT, TRUE  },
  { "rpapi_base",  DEFAULT, TRUE  },
  { "clk32",       DEFAULT, TRUE  },
  { "",            DEFAULT, FALSE }
};

T_driver_setup_var setup_var[sizeof(setup_const) / sizeof(T_driver_setup_const)];

/*==== PRIVATE FUNCTIONS =====================================================*/
/*==== PUBLIC FUNCTIONS ======================================================*/
/*-----------------------------------------------------------------------------
| Function    : bqfs_init
+------------------------------------------------------------------------------
| Description : BQFS driver initialization
|
| Parameters  : config   - Pointer to configuration parameters
|               result   - Pointer-Pointer to return result string
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 bqfs_init(T_driver_config * config, char ** result)
{
    T_driver_setup_var *setup = get_setup_var();  
  
    T_BQFS_PARAMS * db;

    /* Parse configuration string */

    if(drv_parse_config(get_setup_const(), setup, config->cstring, config) == FLASH_DRV_ERROR)
    {
      *result = "UNABLE TO FIND CONFIGURATION PARAMETER DURING INITIALIZATION";
      return FLASH_DRV_ERROR;
    }

    /* Initialize */    
    
    *result          = NULL;
    config->data     = config->drv_malloc(sizeof(T_BQFS_PARAMS));
    db               = (T_BQFS_PARAMS *)config->data;
    db->i2c_id       = setup[C_I2C_ID].value;
    db->rpapi_base   = setup[C_RPAPI_BASE].value;
    db->clk32        = (volatile U32 *)setup[C_CLK32].value;

    config->dbg_printf("BQFS driver init - I2C%d", db->i2c_id);

    /* Initialize the I2C driver */

    config->dbg_printf("HAL_CTRL_ConfigurePads 0x%08X", RPAPI(0xA8));


    if(HAL_CTRL_ConfigurePads(HAL_MODULE_I2C, db->i2c_id) != NO_ERROR)
    {
      config->dbg_printf("pads");
    }
    else if (HAL_CM_EnableModuleClocks(HAL_MODULE_I2C, db->i2c_id) != NO_ERROR)
    {
      config->dbg_printf("clocks");
    }
    else if (IRQ_Initialize() != NO_ERROR)
    {
      config->dbg_printf("IRQ");
    }
    else if (HAL_I2C_Initialize(db->i2c_id) != NO_ERROR)
    {
      config->dbg_printf("Init");
    }
    else
    {
      config->dbg_printf("I2C init done");
    }
    
    
    // i2c_init(config, db->i2c_id);

    db->length = 0;
    db->data   = config->drv_malloc(MAX_IMAGE_SIZE);

    return FLASH_DRV_SUCCESS;
}


/*-----------------------------------------------------------------------------
| Function    : bqfs_read
+------------------------------------------------------------------------------
| Description : Read function for BQFS
|
| Parameters  : config   - Pointer to configuration parameters
|               result   - Pointer-Pointer to return result string
|               dst_addr - Address to write to
|               src_addr - Address to get data from
|               size     - Number of bytes to read
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 bqfs_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size)
{
    config->dbg_printf("Call to driver read function - not supported!");
    config->send_info("Attempt to use unsupported driver function");
    *result = "BQFS read not supported";
    return FLASH_DRV_ERROR;
}

/*------------------------------------------------------------------------------
| Function    : bqfs_decode
+------------------------------------------------------------------------------
| Description : Decode a line in a BQFS formatted file
|
| Parameters  : config   - Pointer to configuration parameters
|               src_addr - Address to read from
|               element  - Pointer to element of decoded data
|               size     - Size of remaining data to decode
|
| Returns     : result_code
+----------------------------------------------------------------------------*/
U32 bqfs_decode(T_driver_config * config, U32 src_addr, T_element * element, U32 size)
{
  /* Support for download of BQFS update script as a binary. The content of the binary will have to be parsed as text. The
     Following format is supported:
     
     W: 16 00 01 02                    \\ I2C write to device 0x16, offset 0x00, two byte sequence 0x01 0x02
     X: 200                            \\ Delay for 200ms
     C: 16 66 00                       \\ I2C read from device 0x16, offset 0x66, validate against 0x00
    */

  char * data   = (char *)src_addr;
  char * decode = data;

  config->dbg_printf("Decoding: %s", data);
  
  switch(*data)
  {
    case 'W':
    case 'w':
      decode += 2;
      element->command = BQFS_WRITE;
      element->content.write.device  = (U8)config->get_value(&decode, "", NOALIGN) >> 1;
      element->content.write.subaddr = (U8)config->get_value(&decode, "", NOALIGN);
      element->content.write.length  = 0;

      while((*decode != 0x00) && (size > (U32)(decode - data)))
      {
        while((*decode == ' ') && (size > (U32)(decode - data)))
        {
          decode++;
        }
        if(*decode && (size > (U32)(decode - data)))
        {
          element->content.write.data[element->content.write.length++] = (U8)config->get_value(&decode, "", NOALIGN);
        }
      }
      
      while((*decode == 0x00) && (size > (U32)(decode - data)))
      {
        decode++;
      }
      break;

    case 'X':
    case 'x':
      decode += 2;
      element->command = BQFS_WAIT;
      element->content.wait.period  = (U32)config->get_value(&decode, "", DECIMAL | NOALIGN);

      while((*decode == 0x00) && (size > (U32)(decode - data)))
      {
        decode++;
      }
      break;

    case 'C':
    case 'c':
      decode += 2;
      element->command = BQFS_CHECK;
      element->content.check.device  = (U8)config->get_value(&decode, "", NOALIGN) >> 1;
      element->content.check.subaddr = (U8)config->get_value(&decode, "", NOALIGN);
      element->content.check.length  = 0;

      while((*decode != 0x00) && (size > (U32)(decode - data)))
      {
        while((*decode == ' ') && (size > (U32)(decode - data)))
        {
          decode++;
        }
        if(*decode && (size > (U32)(decode - data)))
        {
          element->content.check.data[element->content.check.length++] = (U8)config->get_value(&decode, "", NOALIGN);
        }
      }

      while((*decode == 0x00) && (size > (U32)(decode - data)))
      {
        decode++;
      }
      break;

    case 'M':
    case 'm':
      decode += 2;
      element->command = BQFS_CHMOD;
      element->content.chmod.time     = (U32)config->get_value(&decode, "", DECIMAL | NOALIGN);
      element->content.chmod.delay    = (U32)config->get_value(&decode, "", DECIMAL | NOALIGN);
      element->content.chmod.device1  = (U8)config->get_value(&decode, "", NOALIGN) >> 1;
      element->content.chmod.subaddr1 = 0x00;
      element->content.chmod.device2  = (U8)config->get_value(&decode, "", NOALIGN) >> 1;
      element->content.chmod.subaddr2 = 0x00;
      element->content.chmod.length1  = 0;

      while((*decode != 0x00) && (size > (U32)(decode - data)))
      {
        while((*decode == ' ') && (size > (U32)(decode - data)))
        {
          decode++;
        }
        if(*decode && (size > (U32)(decode - data)))
        {
          element->content.chmod.data1[element->content.chmod.length1++] = (U8)config->get_value(&decode, "", NOALIGN);
        }
      }

      while((*decode == 0x00) && (size > (U32)(decode - data)))
      {
        decode++;
      }
      break;

    default:
      config->dbg_printf("Unrecognized command 0x%02X - unable to decode", *data);
      break;
  }

  return decode - data;
}

U32 bqfs_i2c_read(T_driver_config * config, U8 device, U8 subaddr, U32 length, U8 * data)
{
  T_BQFS_PARAMS * db  = (T_BQFS_PARAMS *)config->data;
  U32 ret;

  ret = HAL_I2C_Write(db->i2c_id, device, 1, &subaddr, *db->clk32, 0xFF);

  if(!ret)
  {
    ret = HAL_I2C_Read(db->i2c_id, device, length, data, *db->clk32, 0xFF);
  }

  if(ret)
  {
    HAL_I2C_Initialize(db->i2c_id);
  }
  return ret;
}

/*------------------------------------------------------------------------------
| Function    : bqfs_i2c_write
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 bqfs_i2c_write(T_driver_config * config, U8 device, U8 subaddr, U32 length, U8 * data)
{
  T_BQFS_PARAMS * db  = (T_BQFS_PARAMS *)config->data;
  U32 ret;
  U8 * cmd = config->drv_malloc(MAX_LENGTH + 1);
  cmd[0] = subaddr;
  config->drv_memcpy(cmd + 1, data, length);

  ret = HAL_I2C_Write(db->i2c_id, device, length + 1, cmd, *db->clk32, 0xFF);

  if(ret)
  {
    HAL_I2C_Initialize(db->i2c_id);
  }
  config->drv_free(cmd);
  return ret;
}

/*-----------------------------------------------------------------------------
| Function    : bqfs_write
+------------------------------------------------------------------------------
| Description : Write function for BQFS
|
| Parameters  : config   - Pointer to configuration parameters
|               result   - Pointer-Pointer to return result string
|               dst_addr - Address to write to
|               src_addr - Address to get data from
|               size     - Number of bytes to write
|               more     - More to come?
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 bqfs_write(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more)
{
  /* Support for download of BQFS update script as a binary. The content of the binary will have to be parsed as text. The
     Following format is supported:
     
     W: 16 00 01 02                    \\ I2C write to device 0x16, offset 0x00, two byte sequence 0x01 0x02
     X: 200                            \\ Delay for 200ms
     C: 16 66 00                       \\ I2C read from device 0x16, offset 0x66, validate against 0x00
    */
    T_BQFS_PARAMS * db  = (T_BQFS_PARAMS *)config->data;
    S32 ret_val         = FLASH_DRV_SUCCESS;
    U32 decoded         = 0;
    char * p            = (char *)db->data;
    T_element * element = (T_element *)config->drv_malloc(sizeof(T_element));
    U8 * check          = (U8 *)config->drv_malloc(MAX_LENGTH);
    U32 chmod_attempts  = 0;
    U32 temp;
    *result = NULL;

    if(db->length + size < MAX_IMAGE_SIZE)
    {
      config->drv_memcpy(db->data + db->length, (void *)src_addr, size);
      db->length += size;
    }
    else
    {
      config->dbg_printf("BQFS file size error -  File too large (%d)?", size + db->length);
      *result = "Driver unable to handle multiple packet - file too large?";
      db->length = 0;
      return FLASH_DRV_ERROR;
    }

    if((more == FALSE) && (p[db->length - 1] != CR) && (p[db->length - 1] != NL))
    {
      config->dbg_printf("BQFS file format error");
      *result = "BQFS data does not end in new line";
      ret_val = FLASH_DRV_ERROR;
    }
    else if(more == FALSE)
    {
      config->dbg_printf("BQFS write started");
      config->dbg_printf("Size: %d", db->length);

      size = db->length;

      while(decoded < size)
      {
        if((*p == CR) || (*p == NL))
        {
          *p = 0;
        }
        decoded++;
        p++;
      }

      p = (char *)db->data;
      decoded = 0;

      do
      {
        decoded = bqfs_decode(config, (U32)p, element, size);
      
        if(decoded)
        {
          switch(element->command)
          {
            case BQFS_WRITE:

              temp = bqfs_i2c_write(config, element->content.write.device, element->content.write.subaddr, element->content.write.length, element->content.write.data);

              if(temp)
              {
                config->dbg_printf("Failed to write I2C data to device 0x%02X (%d)", element->content.write.device, temp);
                *result = "Failed to write data to device";
                ret_val = FLASH_DRV_ERROR;
              }
              else
              {
                config->dbg_printf("OK");
              }
              break;

            case BQFS_WAIT:
              config->wait_microsec(element->content.wait.period * 1000);
              config->dbg_printf("OK");
              break;

            case BQFS_CHECK:

              temp = bqfs_i2c_read(config, element->content.check.device, element->content.check.subaddr, element->content.check.length, check);

              if(temp)
              {
                config->dbg_printf("Failed to read I2C data from device 0x%02X (%d)", element->content.check.device, temp);
                *result = "Failed to read data from device";
                ret_val = FLASH_DRV_ERROR;
              }
              else
              {
                U32 count;
                for(count = 0; count < element->content.check.length; count++)
                {
                  if(check[count] != element->content.check.data[count])
                  {
                    config->dbg_printf("Check of byte %d failed: read 0x%02X != check 0x%02X", count, check[count], element->content.check.data[count]);
                    *result = "Check of value failed";
                    ret_val = FLASH_DRV_ERROR;
                    break;
                  }
                }

                if(ret_val == FLASH_DRV_SUCCESS)
                {
                  config->dbg_printf("OK");
                }
              }
              break;

            case BQFS_CHMOD:
              chmod_attempts = 0;
              config->loop_timeout_ms(1, 0);
              ret_val = FLASH_DRV_ERROR;
              if(bqfs_i2c_read(config, element->content.chmod.device2, element->content.chmod.subaddr2, 1, check))
              {
                do 
                {
                  chmod_attempts++;
                  if(!bqfs_i2c_write(config, element->content.chmod.device1, element->content.chmod.subaddr1, element->content.chmod.length1, element->content.chmod.data1))
                  {
                    break;
                  }
                  config->dbg_printf("Write of mode command failed [%d]", chmod_attempts);
                  config->wait_microsec(element->content.chmod.delay * 1000);
                } while(config->loop_timeout_ms(1, element->content.chmod.time) != TRUE);
              
                chmod_attempts = 0;
              
                while(config->loop_timeout_ms(1, element->content.chmod.time) != TRUE)
                {
                  chmod_attempts++;
                  config->wait_microsec(element->content.chmod.delay * 1000);
                  if(!bqfs_i2c_read(config, element->content.chmod.device2, element->content.chmod.subaddr2, 1, check))
                  {
                    ret_val = FLASH_DRV_SUCCESS;
                    break;
                  }
                  config->dbg_printf("Read from new device ID failed [%d]", chmod_attempts);
                }
              }
              else
              {
                ret_val = FLASH_DRV_SUCCESS;
              }

              if(ret_val == FLASH_DRV_ERROR)
              {
                config->dbg_printf("Unable to change mode");
                *result = "Change of mode failed";
              }
              else
              {
                config->dbg_printf("OK");
              }
              break;  
          }
        }
        else
        {
          config->dbg_printf("Failed to decode instruction");
          *result = "Failed to decode instruction";
          ret_val = FLASH_DRV_ERROR;
        }
        p    += decoded;
        size -= decoded;
      } while(size && (ret_val == FLASH_DRV_SUCCESS));
    }
    
    config->drv_free(element);
    config->drv_free(check);

    if(more == FALSE)
    {
      if(ret_val == FLASH_DRV_SUCCESS)
      {
        config->dbg_printf("BQFS decoding done");
      }
      db->length = 0;
    }

    return ret_val;
}


/*-----------------------------------------------------------------------------
| Function    : bqfs_erase
+------------------------------------------------------------------------------
| Description : BQFS erase function 
|
| Parameters  : config - Pointer to configuration parameters
|               result - Pointer-Pointer to return result string
|               addr   - Address for start of erase
|               length - Number of bytes to erase
|
| Returns     : Will always fail
+-----------------------------------------------------------------------------*/
U32 bqfs_erase(T_driver_config * config, char ** result, U64 addr, U64 length)
{
  config->dbg_printf("Call to driver erase function - not supported!");
  config->send_info("Attempt to use unsupported driver function");
  *result = "BQFS erase not supported";
  return FLASH_DRV_ERROR;
}

/*------------------------------------------------------------------------------
| Function    : bqfs_deinit
+------------------------------------------------------------------------------
| Description : De-initialization function for the driver
|
| Parameters  : config - Pointer to configuration parameters
|               result - Pointer-Pointer to return result string
|
| Returns     : result code
+----------------------------------------------------------------------------*/
U32 bqfs_deinit(T_driver_config * config, char ** result)
{
  T_BQFS_PARAMS * db  = (T_BQFS_PARAMS *)config->data;
  *result = NULL;
  db->length = 0;
  config->drv_free(db->data);
  return(FLASH_DRV_SUCCESS);
}

/*------------------------------------------------------------------------------
| Function    : bqfs_get_info
+------------------------------------------------------------------------------
| Description : Get information about the driver 
|
| Parameters  : config - Pointer to configuration parameters
|               info   - Pointer to return structure
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 bqfs_get_info(T_driver_config * config, T_driver_info * info)
{
  info->device_base_address = 0;
  info->device_size         = MAX_IMAGE_SIZE;
  return FLASH_DRV_SUCCESS;
}


/* End of File */

