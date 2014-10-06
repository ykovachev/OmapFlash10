/**
 * @file triton2.c
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
 * This file contains platform/OMAP dependant Triton2 lower layer
 * 
 */

/*==== DECLARATION CONTROL ===================================================*/

/*==== INCLUDES ==============================================================*/
#define PRIVATE_TRITON2_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "csst_tgt.h"
#include "triton2.h"
#include "i2c_dis.h"
#include "triton2_drv.h"
#include "interrupt.h"
//#include "dl_pfwork_ex.h"
#include "dl_ifwork.h"
#include "padconfig.h"
#include "dbg_ex.h"
#ifdef EMMC_DRV
#include "i2c_drv.h"
#endif //EMMC_DRV

U8 triton2_slave_addr = 0;
U8 prev_slave_addr = 0;

/*============ FUNCTIONS =====================================================*/

/*-----------------------------------------------------------------------------
| Function    : get_omap_i2c_speed
+------------------------------------------------------------------------------
| Description : Return the I2C interface speed used for interfacing to the Triton2
| on 2430 SDP 1.1.
|
| Parameters  : None
|
| Returns     : U32 (returns I2C speed)
+-----------------------------------------------------------------------------*/
U32 get_omap_i2c_speed(void)
{
	/* On OMAP2430 SDP, Triton2 is connected to HS I2C interface working at 2.6Mbits/sec */
	return (I2C_2P6M_CLK);
}

/*-----------------------------------------------------------------------------
| Function    : get_omap_i2c_interface
+------------------------------------------------------------------------------
| Description : Returns which 2430 I2C interface is connected to Triton2
|
| Parameters  : None
|
| Returns     : U32 (returns I2C instance number)
+-----------------------------------------------------------------------------*/
U32 get_omap_i2c_interface(void)
{
	/* On OMAP3430 SDP 1.1, Triton2 is connected to I2C1 of OMAP 3430 */
	return (SID_I2C1);
}

/*-----------------------------------------------------------------------------
| Function    : get_processor_group
+------------------------------------------------------------------------------
| Description : Returns processor resource groups
|
| Parameters  : None
|
| Returns     : U32 (returns Processor group number)
+-----------------------------------------------------------------------------*/
U32 get_processor_group(void)
{
	return (PROCESSOR_GRP1);  /* OMAP 2430 is application processor group */
}
/*-----------------------------------------------------------------------------
| Function    : store_i2c_id_for_triton2
+------------------------------------------------------------------------------
| Description : This function stores the present value of I2c slave address.
|
| Parameters  : U32 i2c_device_handle
|
| Returns     : S32 (OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
S32 store_i2c_id_for_triton2(U32 i2c_device_handle)
{
    S32 ret_status ;
	U32 size;
	size = 1;

	ret_status = dal_read(i2c_device_handle, I2C_SLAVE_ADDR, &size, &triton2_slave_addr);
	if(ret_status != OMAPFLASH_SUCCESS)
	{
		dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Slave Address change failed");
	}
	prev_slave_addr = triton2_slave_addr; /* added */
	return (ret_status);

}
/*-----------------------------------------------------------------------------
| Function    : restore_i2c_id_for_triton2
+------------------------------------------------------------------------------
| Description : This function restores the value of i2c slave address.
|
| Parameters  : U32 i2c_device_handle
|
| Returns     : S32 (OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
S32 restore_i2c_id_for_triton2(U32 i2c_device_handle)
{
    S32 ret_status ;
	U32 size;
	size = 1;

	ret_status = dal_write(i2c_device_handle, I2C_SLAVE_ADDR, &size, &triton2_slave_addr);
	if(ret_status != OMAPFLASH_SUCCESS)
	{
		dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Slave Address change failed");
	}

	return (ret_status);

}
/*-----------------------------------------------------------------------------
| Function    : configure_i2c_slave_id_for_triton2
+------------------------------------------------------------------------------
| Description : This function configures the i2c slave address.
|
| Parameters  : U32 i2c_device_handle
|				U8 i2c_adr
|
| Returns     : S32 (OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
S32 configure_i2c_slave_id_for_triton2(U32 i2c_device_handle, U8 i2c_addr)
{
	S32 ret_val;
	U32 length = 1;
	ret_val = dal_write(i2c_device_handle, I2C_SLAVE_ADDR, &length, &i2c_addr);
	if(ret_val != OMAPFLASH_SUCCESS)
	{
		dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Slave Address change failed");		      
	}
	 return ret_val;	 
}


/*-----------------------------------------------------------------------------
| Function    : t2_reg_read
+------------------------------------------------------------------------------
| Description : reads the triton2 register mentioned.
|
| Parameters  : U32 i2c_device_handle
|               U8 i2c_addr - slave addr
|               U8 subaddress-
|				U32 length- Number of bytes to read.
|				U8 *data-Pointer to the buffer where data read is stored.   
|
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
S32 t2_reg_read(U32 i2c_device_handle, U8 i2c_addr,U8 subaddress,U32 *len, U8 *buff)
{
    U32 length;
    S32 ret_val;
	
    if(i2c_addr != prev_slave_addr)
	{
	    length=*len; /*number of bytes to be read */
	    ret_val = store_i2c_id_for_triton2(i2c_device_handle);
		if(ret_val!=OMAPFLASH_SUCCESS)
	   	{
	         dbg_print(DBG_LEVEL_CRITICAL, "store_i2c_id_for_triton2 failed");
	         return ret_val;
	    }
		ret_val = configure_i2c_slave_id_for_triton2(i2c_device_handle, i2c_addr);
	    if(ret_val!=OMAPFLASH_SUCCESS)
	   	{
	         dbg_print(DBG_LEVEL_CRITICAL, "configure_i2c_slave_id_for_triton2 failed");
	         return ret_val;
	    }
	}
	/* read the content of Triton2 registers through I2C  */
    ret_val = dal_read(i2c_device_handle,I2C_OFFSET(subaddress), &length, buff);
    if(ret_val!=OMAPFLASH_SUCCESS)
   	{
         dbg_print(DBG_LEVEL_CRITICAL, "I2C read failed");
         return ret_val;
    }
	
    if(i2c_addr != prev_slave_addr)
	{
		/* prev_slave_addr = i2c_addr; */  
		ret_val = restore_i2c_id_for_triton2(i2c_device_handle);	 
	    if(ret_val!=OMAPFLASH_SUCCESS)
	   	{
	         dbg_print(DBG_LEVEL_CRITICAL, "restore_i2c_id_for_triton2 failed");
	         return ret_val;
	    }
  	}  
    return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    : t2_reg_write()
+------------------------------------------------------------------------------
| Description : handles the RTC write register .
|
| Parameters  : U8 subaddress-
|				U32 length- Number of bytes to write.
|				U8 *data-Pointer to the buffer where data is stored.   
|
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
S32 t2_reg_write(U32 i2c_device_handle, U8 i2c_addr,U8 subaddress,U32 *len, U8 *buff)
{
    U32 length;
    S32 ret_val;
    
    if(i2c_addr != prev_slave_addr)
	{

	    length=*len; /*number of bytes to be read */
	    ret_val = store_i2c_id_for_triton2(i2c_device_handle);
		if(ret_val!=OMAPFLASH_SUCCESS)
	   	{
	         dbg_print(DBG_LEVEL_CRITICAL, "store_i2c_id_for_triton2 failed");
	         return ret_val;
	    }
		ret_val = configure_i2c_slave_id_for_triton2(i2c_device_handle, i2c_addr);
	    if(ret_val!=OMAPFLASH_SUCCESS)
	   	{
	         dbg_print(DBG_LEVEL_CRITICAL, "configure_i2c_slave_id_for_triton2 failed");
	         return ret_val;
	    }
	}

    /* write the content of RTC registers through I2C  */
    ret_val = dal_write(i2c_device_handle,I2C_OFFSET(subaddress), &length, buff);
    if(ret_val!=OMAPFLASH_SUCCESS)
   	{
         dbg_print(DBG_LEVEL_CRITICAL, "I2C read failed");
         return ret_val;
    }
	
	if(i2c_addr != prev_slave_addr)
	{
		prev_slave_addr = i2c_addr;	
		ret_val = restore_i2c_id_for_triton2(i2c_device_handle);	 
	    if(ret_val!=OMAPFLASH_SUCCESS)
	   	{
	         dbg_print(DBG_LEVEL_CRITICAL, "restore_i2c_id_for_triton2 failed");
	         return ret_val;
	    }
	}   
    return ret_val;
   
}
#ifdef INTERRUPT
/*-----------------------------------------------------------------------------
| Function    : triton2_interrupt_init
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : void 
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void triton2_interrupt_init(void)
{
	 /* Configure the sys.nirq pin in mode0. */
     mux_setup_sysnirq();

	 memset(triton2_int_handler_table,NULL,sizeof(triton2_int_handler_table));
  
}
/*-----------------------------------------------------------------------------
| Function    : triton2_add_int_handler
+------------------------------------------------------------------------------
| Description : add a callback function to the triton2 vector table
|
| Parameters  : None
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void triton2_add_int_handler(U8 int_type, void (*trn2_callback)(void *), void* data)
{
  if(triton2_int_handler_table[int_type].int_handler_func == NULL)
  {

    triton2_int_handler_table[int_type].int_handler_func = trn2_callback;
    triton2_int_handler_table[int_type].data = data;
	dal_interrupt_set_sensitivity(INT_SYSIRQ, INTH_LEVEL);
	dal_interrupt_add_handler(INT_SYSIRQ,triton2_isr,NULL);
  }
}

/*-----------------------------------------------------------------------------
| Function    : triton2_remove_int_handler
+------------------------------------------------------------------------------
| Description : Remove a callback function from the Triton2 vector table
|
| Parameters  : None
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void triton2_remove_int_handler(U8 int_type)
{
    triton2_int_handler_table[int_type].int_handler_func = NULL;
    triton2_int_handler_table[int_type].data = 0;
	dal_interrupt_remove_handler(INT_SYSIRQ); 
}  
/*-----------------------------------------------------------------------------
| Function    : call_isr_func_trtn2
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : U8 trtn2_int_num
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void call_isr_func_trtn2(U8 trtn2_int_num)
{
  if(triton2_int_handler_table[trtn2_int_num].int_handler_func != NULL)
  {
    triton2_int_handler_table[trtn2_int_num].int_handler_func(triton2_int_handler_table[trtn2_int_num].data);
  }

}
#endif //INTERRUPT
/*==== END OF FILE ==========================================================*/
