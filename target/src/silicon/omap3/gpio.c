/**
 * @file gpio.c
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
 * This File contains the GPIO API functions 
 * 
 */

/*========================== DECLARATION CONTROL ============================*/

#ifndef GPIO_C
#define GPIO_C

/*=============================== INCLUDES ==================================*/
#include "csst_tgt.h"

#define GPIO_PRIVATE
#include "gpio.h"

#include "interrupt.h"
#include "dl_ifwork.h"
#include <string.h>
#include "dbg_ex.h"

/*============================= GLOBALS ====================================*/
#ifdef INTERRUPT
U8 gpio_module_num = 0;
#endif //INTERRUPT

/*=========================== PUBLIC FUNCTIONS ==============================*/
/*-----------------------------------------------------------------------------
| Function    :S32 gpio_init(void) 
+------------------------------------------------------------------------------
| Description :	This routine is called to initialize the GPIO subsystem. 
|               GPIO interrupt handlers for the 4 modules are added.
| Parameters  : none
|
| Returns     : DAL_SUCCESS, if the initialization is successful.
|				a negative value if the initialization is failure.
|				
+-----------------------------------------------------------------------------*/

S32 gpio_init(void)
{
#ifdef INTERRUPT
    /*  Add the general GPIO interrupt handlers for the 5 gpio modules */
    /* if unable to add interrupt handler return from this function */
  
  /* send the gpio module number as the data to the callback function*/
   gpio_module_num = GPIO_MODULE_1;
  if(dal_interrupt_add_handler(INT_GPIO1_MPU_IRQ, gpio_int_handler,
                               (void *)gpio_module_num) != OMAPFLASH_SUCCESS)
  {
    dbg_print(DBG_LEVEL_MAJOR,
              "couldn't add interrupt handler for gpio module 1 \r\n");
    return(OMAPFLASH_DAL_ERROR);
  }

  /* send the gpio module number as the data to the callback function*/
  gpio_module_num = GPIO_MODULE_2;  
  if(dal_interrupt_add_handler( INT_GPIO2_MPU_IRQ, gpio_int_handler,
                               (void *)gpio_module_num) != OMAPFLASH_SUCCESS)
  {
    dbg_print(DBG_LEVEL_MAJOR,
              "couldn't add interrupt handler for gpio module 2 \r\n");
    return(OMAPFLASH_DAL_ERROR);
  }   
  
  /* send the gpio module number as the data to the callback function*/
  gpio_module_num = GPIO_MODULE_3;
  if(dal_interrupt_add_handler( INT_GPIO3_MPU_IRQ, gpio_int_handler,
                               (void *)gpio_module_num) != OMAPFLASH_SUCCESS)
  {
    dbg_print(DBG_LEVEL_MAJOR,
              "couldn't add interrupt handler for gpio module 3 \r\n");
    return(OMAPFLASH_DAL_ERROR);
  }
  
  /* send the gpio module number as the data to the callback function*/
  gpio_module_num = GPIO_MODULE_4;
  if(dal_interrupt_add_handler(INT_GPIO4_MPU_IRQ, gpio_int_handler,
                               (void *)gpio_module_num) != OMAPFLASH_SUCCESS)
  {
    dbg_print(DBG_LEVEL_MAJOR,
              "couldn't add interrupt handler for gpio module 4 \r\n");
    return(OMAPFLASH_DAL_ERROR);
  }    
  
  gpio_module_num = GPIO_MODULE_5;
  if(dal_interrupt_add_handler(INT_GPIO5_MPU_IRQ, gpio_int_handler,
                               (void *)gpio_module_num) != OMAPFLASH_SUCCESS)
  {
    dbg_print(DBG_LEVEL_MAJOR,
              "couldn't add interrupt handler for gpio module 5 \r\n");
    return(OMAPFLASH_DAL_ERROR);
  } 
  gpio_module_num = GPIO_MODULE_6;
  if(dal_interrupt_add_handler(INT_GPIO6_MPU_IRQ, gpio_int_handler,
                               (void *)gpio_module_num) != OMAPFLASH_SUCCESS)
  {
    dbg_print(DBG_LEVEL_MAJOR,
              "couldn't add interrupt handler for gpio module 6 \r\n");
    return(OMAPFLASH_DAL_ERROR);
  } 
#endif //INTERRUPT
  /* initialize the gpio pin's interrupt vector table */  
  gpio_intr_init(); 
  
#ifdef INTERRUPT
  /*return from this function */
  dbg_print(DBG_LEVEL_INFO,
                         "hooked ISRs for gpio module \r\n");
#endif //INTERRUPT
  return (OMAPFLASH_SUCCESS);
	
}

/*-----------------------------------------------------------------------------
| Function    :S32 check_gpio_pin_num(U16 pin_num) 
+------------------------------------------------------------------------------
| Description :This function checks the validity of the module number.
|              	
| Parameters  :pin_num
|                 - gpio pin number to be checked.
|                             
| Returns     : DAL_SUCCESS, if the initialization is successful.
|				else DAL_ERROR.
+-----------------------------------------------------------------------------*/
S32 check_gpio_pin_num(U32 pin_num)
{
    /* check whether the pin number is within 0 and 128 */
    /* if yes return DAL_SUCCESS, else return DAL_ERROR */
  if(pin_num > MAX_GPIO_PINS)
  {
	dbg_print(DBG_LEVEL_CRITICAL, "wrong gpio-pin number \n");
  	return(OMAPFLASH_DAL_ERROR);
  }
  
   /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "gpio-pin number is valid \n");
  return (OMAPFLASH_SUCCESS);
}

/*-----------------------------------------------------------------------------
| Function    :void set_gpio_in_out(U8 module, U32 pin_mask, U32 io_mask ) 
+------------------------------------------------------------------------------
| Description :	This function is to initialize the gpio pins. 
|               For eg, as input or output.
|                       if input and an interrupt pin, add callback function
|                       to the gpio interrupt handlers array and set its 
|                       interrupt type.
|				
| Parameters  : 
|               module
|                       -   gpio module number.
|				pin_mask
|						-   specifies which all pins have to be initialized
|               io_mask 
|                       -   This specifies the input/output direction of
|                           pins
|               
| Returns     : DAL_SUCCESS, if the initialization is successful.
|				else DAL_ERROR.
+-----------------------------------------------------------------------------*/

void set_gpio_in_out( U8 module_num, U32 pin_mask, U32 io_mask)
{
  U32 temp_oe;
  U32 gpio_pin_output_en_reg;
  
      /*get the GPIO_OE register address */
  gpio_pin_output_en_reg = (g_gpio_module_base_address[module_num-1]+GPIO_OE);
  
    /*get GPIO_OE register contents */
  temp_oe = in_regl(gpio_pin_output_en_reg); 
  temp_oe = temp_oe & ~pin_mask;     /*manipulate only the pins that needs
                                       configuration */
  temp_oe = temp_oe | io_mask;       /*set the pins as input  or output as
                                       specified */
  
    /*write back */
  out_regl(gpio_pin_output_en_reg, temp_oe);
  
  /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "gpio in/out set \n");
  return;
}

/*-----------------------------------------------------------------------------
| Function    : void gpio_write_output_pin(U8 module_num,U32 pin_mask,U32 data) 
+------------------------------------------------------------------------------
| Description : this function writes to the DATA_OUT register of the GPIOs
|	
| Parameters  : pin_mask
|					-  provides which all pins have to be written
|				data
|					-  has the value that has to be written to the pins
|					   specified by the pin_mask
|				module_num
|					-  gpio module number
|
| Returns     :none
+-----------------------------------------------------------------------------*/

void gpio_write_output_pin(U8 module_num, U32 pin_mask, U32 data)
{
  U32 temp;                    /*temporary variable*/
  U32 gpio_data_out_reg;
    
    /*get the GPIO_DATAOUT register address */
  gpio_data_out_reg = (g_gpio_module_base_address[module_num-1]+GPIO_DATAOUT);
  
    /*  Get the current data for all pins other than the ones, */
    /*  we are programming with new values.  */
    /* temo will have the current data */
  temp = in_regl(gpio_data_out_reg); 
  temp = temp & ~pin_mask;

  /*  Now OR in the new values and write it out!  */
  out_regl(gpio_data_out_reg, (temp | (data & pin_mask)));
  /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "gpio write successful \n");
  
  return;

}

/*-----------------------------------------------------------------------------
| Function    :void gpio_read_input_pin(U8 module_num,U32 pin_mask, U32 *data) 
+------------------------------------------------------------------------------
| Description : this function reads the value from the DATA_IN register
|				of the GPIOs
|	
| Parameters  : pin_mask
|					-  provides which all pins have to be read
|				data
|					-  to hold the value read.
|				module_num
|					-  gpio module number
|
| Returns     : none
+-----------------------------------------------------------------------------*/
void gpio_read_input_pin(U8 module_num,U32 pin_mask, U32 *data)
{
  U32 gpio_data_in_reg;
  U32 temp;                    /*temporary variable*/
  
   /*get the GPIO_DATAIN register address */
  gpio_data_in_reg = (g_gpio_module_base_address[module_num-1]+GPIO_DATAIN);
    	
  /* get the DATA_IN register value */
  temp = in_regl(gpio_data_in_reg); 
   /* get the data relevant to the pin mask*/
  *data = temp & pin_mask;
   
   /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "gpio read success \n");
   return;
}

/*-----------------------------------------------------------------------------
| Function    :void gpio_clear_interrupt_status(U8 module_num, U32 pin_mask)
+------------------------------------------------------------------------------
| Description :	This function clears the required interrupt status bit.
| 
| Parameters  : pin_mask
|                  -  specifies which bit has to be cleared
|               num
|                  -  gpio module number
|
| Returns     : none
|
+-----------------------------------------------------------------------------*/
void gpio_clear_interrupt_status(U8 module_num, U32 pin_mask)
{
  U32 gpio_int_status_reg;
      
     /*get the IRQ_STATUS register address */
  gpio_int_status_reg =(g_gpio_module_base_address[module_num-1]
                                                +GPIO_IRQSTATUS1);
  /*  Clearing the status register */
  /*  you must write a '1' to the bit you want to clear!  */
  /*  Writing a '0' has no effect, so just write the pin_mask!  */
  out_regl(gpio_int_status_reg, pin_mask);
  
  /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "cleared the status bit \n");
  return;
  
}

/*-----------------------------------------------------------------------------
| Function    :S32 gpio_int_handler(U8 module_num)
+------------------------------------------------------------------------------
| Description :	This function is the common ISR for the GPIOs.
|				
| Parameters  : module_num
|                       -   gpio module number.
|
| Returns     :  status returned by the handler or DAL_ERROR value 
|               if interrupt is not serviced.
|				
+-----------------------------------------------------------------------------*/
S32 gpio_int_handler(void * data)
{
  S8 interrupted_pin = -1, i;
  U8 module_num = (U8) (((U32)data) & 0xFF);
  U32 temp_stat, cur_gpio;
  U32 gpio_irqstatus1_reg;
  S32 ret_val = OMAPFLASH_DAL_ERROR;
    
  /* read IRQ_STATUS1 register to identify on which pin there was an interrupt*/
  gpio_irqstatus1_reg = (g_gpio_module_base_address[module_num-1]
                									+GPIO_IRQSTATUS1);
    
  /* read the value to temp_stat */
  temp_stat = in_regl(gpio_irqstatus1_reg);
    
  /*  Fin_regl the first one due to which an  interrupt was caused   */
  for (i = 0, cur_gpio = GPIO_FIRST_PIN; 
                i < NUM_PINS_PER_GPIO_MODULE; i++, cur_gpio <<= 1)
  {
    interrupted_pin++; /*assign the pin number to interrupted_pin*/
    if (cur_gpio & temp_stat)
    {
                /*  Reset the interrupt status  */
      gpio_clear_interrupt_status(module_num, cur_gpio);
            /*  
             * We have a winner! check if a handler is registered, 
             * call the respective GPIO pin interrupt handler.
             * the pin number is sent as the input to the handler.
             */
      if(gpio_handler_table[module_num-1][interrupted_pin].pin_handler != NULL)
      {
        ret_val = 
        gpio_handler_table[module_num-1][interrupted_pin].pin_handler
                                                           (interrupted_pin);
      }  
            /*  Done, for this one. Check for more. */
    }
    
    temp_stat = (temp_stat & ~cur_gpio);
    if(temp_stat == 0)
    {
      break;
    }  
  }
  
  /* return from this function */
  //dbg_print(DBG_LEVEL_INFO," out of ISR \n");
  return( ret_val );
 }
 
/*-----------------------------------------------------------------------------
| Function    :S32 gpio_mask_interrupt(U8 module_num, U32 gpio_pin_mask)
+------------------------------------------------------------------------------
| Description :	This function masks the interrupt generated by one of the GPIOs 
|				
| Parameters  : gpio_pin_mask
|						-  position of the pin in the register/module
|               module_num
|						-  module number. 
| Returns     : none
|
+-----------------------------------------------------------------------------*/

S32  gpio_mask_interrupt(U8 module_num, U32 gpio_pin_mask)
{
  U32 gpio_pin_irq_en_reg;
  U32 temp;           /*temporary variable */
  S32 ret_val = OMAPFLASH_DAL_ERROR;
  
    /*get the address of the CLEARIRQENABLE1 register */
  gpio_pin_irq_en_reg = (g_gpio_module_base_address[module_num-1]
  												+GPIO_CLEARIRQENABLE1);
   /*get the contents of this register */
  temp = in_regl(gpio_pin_irq_en_reg); /*take a copy of it */
  temp |= gpio_pin_mask; /* OR it with the new pin mask */
 
  out_regl(gpio_pin_irq_en_reg, temp); /*write the clear mask */ 
  
  /*return from this function */
  dbg_print(DBG_LEVEL_INFO,"masked the gpio pins specified \n");
  return(ret_val);	
}

/*-----------------------------------------------------------------------------
| Function    :void  gpio_mask_all_intr( void )
+------------------------------------------------------------------------------
| Description :	This function masks all the interrupt generated by GPIOs. 
|				
| Parameters  : none
|
| Returns     : none
|
+-----------------------------------------------------------------------------*/
void gpio_mask_all_intr(void)
{
      /*mask all the gpio interrupts belonging to all the 4 modules */
  gpio_mask_interrupt(GPIO_MODULE_1, GPIO_MASK_ALL);
  gpio_mask_interrupt(GPIO_MODULE_2, GPIO_MASK_ALL);
  gpio_mask_interrupt(GPIO_MODULE_3, GPIO_MASK_ALL);
  gpio_mask_interrupt(GPIO_MODULE_4, GPIO_MASK_ALL);	
  gpio_mask_interrupt(GPIO_MODULE_5, GPIO_MASK_ALL);	
  gpio_mask_interrupt(GPIO_MODULE_6, GPIO_MASK_ALL);
  /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "masked all gpio pins \n");
  return;
}

/*-----------------------------------------------------------------------------
| Function    :S32  gpio_unmask_interrupt( U8 module_num, U32 gpio_pin_mask)
+------------------------------------------------------------------------------
| Description :	This function unmasks the interrupt generated by one of the 
				GPIOs. 
|				
| Parameters  : gpio_pin_mask
|						- pins to be masked.
|               mod_num
|						-  module number. 
| Returns     : none
|
+-----------------------------------------------------------------------------*/
S32  gpio_unmask_interrupt( U8 module_num, U32 gpio_pin_mask)
{
  U32 gpio_pin_irq_en_reg;
  S32 ret_val = OMAPFLASH_DAL_ERROR;
  U32 temp;   /*temporary variable*/
   
     /*get the address of the SETIRQENABLE1 register */
  gpio_pin_irq_en_reg = (g_gpio_module_base_address[module_num-1]
  												+GPIO_SETIRQENABLE1);
   /*get the contents of this register */
  temp = in_regl(gpio_pin_irq_en_reg); /*take a copy of it */
  temp |= gpio_pin_mask; /* OR it with the new pin mask */
  
  out_regl(gpio_pin_irq_en_reg, temp); /*write the set mask */ 
  
  /*return from this function */
  dbg_print(DBG_LEVEL_INFO,"unmasked the gpio pins specified \n");
  return(ret_val);
}

/*-----------------------------------------------------------------------------
| Function    :S32 gpio_add_callback_handler(U8 module_num,
|                                   U32 pin_mask,S32 (*handler)(U8),U8 data)
+------------------------------------------------------------------------------
| Description :	This function hooks the gpio interrupt handler function for the 
|               pin, specified by the application program.
|				
| Parameters  : module_num
|						-  module number. 
|               pin_mask
|						-  pins in the module to which callback function 
|                          is to be hooked
|               S32 (*handler)(U8)
|                       -  callback function specified by the application 
|                          program.
|               
| Returns     : DAL_SUCCESS
|                       - on successful addition of the handler
|               else DAL_ERROR.
|
+-----------------------------------------------------------------------------*/
S32 gpio_add_callback_handler(U8 module_num, U32 pin_mask,S32(*handler)(U8),
                                                         U8 data)
{
  S32 ret_val = OMAPFLASH_DAL_ERROR;
  U8  i; 
  U32 intr_mask;
  intr_mask = pin_mask;
     
     /*  disbale the interrupt before adding a handler.  */
  gpio_mask_interrupt( module_num, intr_mask);
    
  for(i=0;i<NUM_PINS_PER_GPIO_MODULE;i++,pin_mask>>=1)
  {
     /*if the pin is an input, then add a handler */
    if(pin_mask & GPIO_INTR)
    {
      if(gpio_handler_table[module_num-1][i].pin_handler == NULL)
      {
        gpio_handler_table[module_num-1][i].pin_handler = handler;
        gpio_handler_table[module_num-1][i].data = data;
        ret_val = OMAPFLASH_SUCCESS;
      }
    }
  }
      /*  Unmask the interrupt now that it has a handler...  */
  gpio_unmask_interrupt( module_num, intr_mask);
  
  /*return from this function */
  dbg_print(DBG_LEVEL_INFO,"added callback handler \n");
  return(ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : S32 gpio_set_interrupt_type(U8 module_no,
|                                T_GPIO_INTR_TYPE int_type,U32 pin_msk) 
+------------------------------------------------------------------------------
| Description : this function configures the interrupt type on the GPIO pin.
|
| Parameters  : pin_msk
|					-  specifies which all pins have to be specified 
|                      with the interrupt type
|				int_type
|                   -  specifies the interrupt type
|				mod_no
|					-  gpio module number
|
| Returns     : DAL_SUCCESS, if the setting the interrupt type
|               is successful.
|				else DAL_ERROR.
+-----------------------------------------------------------------------------*/
S32 gpio_set_interrupt_type(U8 module_num,T_GPIO_INTR_TYPE int_type,
                                                        U32 pin_msk)
{
  U32 gpio_int_type_reg;
  S32 ret_val = OMAPFLASH_DAL_ERROR;
  U32 temp;   /*temporary variable*/
   
   /*depending on the interrupt type mentioned and the module number,
     get the corresponding register address */
  switch (int_type)
  {
    case GPIO_RISING_EDGE_DETECT:
      gpio_int_type_reg = (g_gpio_module_base_address[module_num-1]
                                                    +GPIO_RISINGDETECT);
      break;
	
	  case GPIO_FALLING_EDGE_DETECT: 
	    gpio_int_type_reg = (g_gpio_module_base_address[module_num-1]
	                                                +GPIO_FALLINGDETECT);
	    break;

	  case GPIO_LOW_LEVEL_DETECT:
      gpio_int_type_reg = (g_gpio_module_base_address[module_num-1]
                                                    +GPIO_LEVELDETECT0);
      break;

	  case GPIO_HIGH_LEVEL_DETECT:
      gpio_int_type_reg = (g_gpio_module_base_address[module_num-1]
                                                    +GPIO_LEVELDETECT1);
      break;
    	
    default:
 //    dbg_print(" Invalid Interrupt type  'type' passed as parameter\n\r"); 	 
    return(OMAPFLASH_DAL_ERROR);
   
  }
  
  temp = in_regl(gpio_int_type_reg);/*read the register */
  temp |= pin_msk;  /* OR it with the new pin mask */
  out_regl(gpio_int_type_reg,temp); /*write to the register */
  
  /*return from this function */
  dbg_print(DBG_LEVEL_INFO,"the interrupt type is set \n");
  return(ret_val);

}

/*-----------------------------------------------------------------------------
| Function    :void gpio_remove_callback_handler(U8 module_num, U32 pin_mask)
+------------------------------------------------------------------------------
| Description :	This function hooks the gpio interrupt handler function for the 
|               pin, specified by the application program.
|				
| Parameters  : pin_mask
|						- pins in the module of which the handler has to be 
|                         removed 
|               module_num
|						-  module number. 
| Returns     : DAL_SUCCESS
|                       - on successful addition of the handler
|               else DAL_ERROR.
|
+-----------------------------------------------------------------------------*/
void gpio_remove_callback_handler(U8 module_num, U32 pin_mask)
{
 
        /*if yes remove handler function */
  gpio_handler_table[module_num-1][pin_mask].pin_handler = NULL;

   /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "removed gpio callback handler %d\n",pin_mask);  
  return;

}

/*-----------------------------------------------------------------------------
| Function    :void gpio_deinit(void) 
+------------------------------------------------------------------------------
| Description :	This routine is called to deinitialize the GPIO subsystem. 
|               GPIO interrupt handlers for the 4 modules are removed.
| Parameters  : none
|
| Returns     : none
|				
+-----------------------------------------------------------------------------*/
void gpio_deinit(void) 
{
      /*de initialize the gpio handler table */
  gpio_intr_deinit();
    
#ifdef INTERRUPT
   	/*remove all the gpio related interrupt handler functions */
  dal_interrupt_remove_handler(INT_GPIO1_MPU_IRQ);
  dal_interrupt_remove_handler(INT_GPIO2_MPU_IRQ);
  dal_interrupt_remove_handler(INT_GPIO3_MPU_IRQ);
  dal_interrupt_remove_handler(INT_GPIO4_MPU_IRQ);
  dal_interrupt_remove_handler(INT_GPIO5_MPU_IRQ);
  dal_interrupt_remove_handler(INT_GPIO6_MPU_IRQ);
#endif //INTERRUPT
  
  /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "gpio module de-initialized \n"); 
  return;
  
}  
/*-----------------------------------------------------------------------------
| Function    :void gpio_intr_init(void)
+------------------------------------------------------------------------------
| Description :	This function initialize the interrupt feature of the 
|               GPIO pins.This function sets up all interrupts properly
|               and disables them.
| Parameters  : none
| 
| Returns     : none
|
+-----------------------------------------------------------------------------*/

void gpio_intr_init(void)
{
		
	/*  ... initialize the gpio handler'jump table' */

  memset(gpio_handler_table,0,sizeof(gpio_handler_table));
	
	/*  ... mask all GPIO interrupts  */
  gpio_mask_all_intr();

  /* return from this function */
  dbg_print(DBG_LEVEL_INFO, "gpio handler table initialized \n");
  return;
	
}

/*-----------------------------------------------------------------------------
| Function    :void gpio_intr_deinit(void)
+------------------------------------------------------------------------------
| Description :	This function de initializes the interrupt feature of the 
|               GPIO pins. 
|	        	
| Parameters  : none
| 
| Returns     : none
|
+-----------------------------------------------------------------------------*/

void gpio_intr_deinit(void)
{
	/*  ...de initialize the gpio handler 'jump table' */
  memset(gpio_handler_table,0,sizeof(gpio_handler_table));
  /* return from this function */
  dbg_print(DBG_LEVEL_INFO,"gpio handler table de-initialized \n");   	
  return;

}

#endif  /* GPIO_C */ 

 
 
 
 
 
