/**
 * @file dl_gpio.c
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
 * This File contains the GPIO framework API functions.
 * 
 */

/*========================== DECLARATION CONTROL ============================*/

#ifndef DL_GPIO_C
#define DL_GPIO_C

/*=============================== INCLUDES ==================================*/
#include "csst_tgt.h"
#include "dl_gpio_ex.h"
#include "gpio.h"
#include "dbg_ex.h"

/*=========================== PUBLIC FUNCTIONS ==============================*/

/*-----------------------------------------------------------------------------
| Function    :S32 gpio_pin_init(U16 pin_num, U8 in_out)
+------------------------------------------------------------------------------
| Description :This function is to initialize the gpio pin as input or output.
|
| Parameters  :pin_num
|                 - gpio pin that needs to be configured.
|
|              in_out
|                 - specifies whether the pin is to be configured as an
|                   input or output.
|
| Returns     : DAL_SUCCESS, if the initialization is successful.
|				else DAL_ERROR.
+-----------------------------------------------------------------------------*/
S32 gpio_pin_init(U16 pin_num, U8 in_out)
{

    U8 module_num = pin_num/NUM_OF_BITS_IN_REG + 1;
    /* GPIO module to which the GPIO number belongs to */
    U32 snum = (module_num-1)*NUM_OF_BITS_IN_REG;
    U32 offset = pin_num - snum;
    U32 pinmask;
    S32 ret_val = OMAPFLASH_SUCCESS;

    /* check whether the gpio number is valid */
    /* if yes continue else return*/
    ret_val = check_gpio_pin_num(pin_num);
    if(ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "wrong gpio-pin number \n");
        return(ret_val);
    }

    /* this variable will have the position of the pin in the corresponding
    gpio module */

    pinmask = GPIO_PIN<<offset;

    /* call this function to configure the pin as input or output */

    set_gpio_in_out(module_num, pinmask, in_out<<offset);

    /*return from this function*/
    dbg_print(DBG_LEVEL_INFO, "gpio pin %d initialized \n",pin_num);

    return (ret_val);

}

/*-----------------------------------------------------------------------------
| Function    :S32 gpio_add_int_handler(U16 pin_num, S32(*gpio_callback)(U8),
|                                         T_GPIO_INTR_TYPE intr_type)
+------------------------------------------------------------------------------
| Description :	This function adds the gpio interrupt handler
| Parameters  : pin_num
|                       -   gpio pin number
|
|                intr_type
|                       -   specifies the interrupt type
|
|               S32 (* gpio_callback)(U8)
|                       -   callback function specified by the application
|                           program.
|
| Returns     : DAL_SUCCESS, if the initialization is successful.
|				else DAL_ERROR.
+-----------------------------------------------------------------------------*/
S32 gpio_add_int_handler(U16 pin_num, S32(*gpio_callback)(U8),U8 data,
                         T_GPIO_INTR_TYPE intr_type)
{
    U8 module_num = pin_num/NUM_OF_BITS_IN_REG + 1;
    /* GPIO module to which the GPIO pin number belongs to */
    U32 snum = (module_num-1)*NUM_OF_BITS_IN_REG;
    U32 offset = pin_num - snum; /* position of the pin in the respective
                                 gpio module */
    U32 pinmask;
    S32 ret_val = OMAPFLASH_DAL_ERROR;

    /* check whether the gpio number is valid */
    /* if yes continue else return*/
    ret_val = check_gpio_pin_num(pin_num);
    if(ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL,"wrong gpio-pin number \n");
        return(ret_val);
    }

    /* this variable will have the position of the pin in the corresponding
    gpio module */

    pinmask = GPIO_PIN<<offset;

    /* add gpio interrupt handler */
    /* first check whether there is a callback function that can be hooked to
    the interrupt */
    /* if unable to add gpio callback function return from this function */
    if(gpio_callback != NULL)
    {
        if(gpio_add_callback_handler(module_num, pinmask, gpio_callback, data)
           != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_INFO, "could not add a callback handler \n");
            return OMAPFLASH_DAL_ERROR;
        }
        /* call this function to set the interrupt type */
        gpio_set_interrupt_type(module_num, intr_type, pinmask);
        ret_val = OMAPFLASH_SUCCESS;
    }
    /* return fromt this function*/
    dbg_print(DBG_LEVEL_INFO, "hooked a ISR for gpio pin %d \n",pin_num);
    return(ret_val);

}

/*-----------------------------------------------------------------------------
| Function    : S32 get_gpio_input(U32 pin_num)
+------------------------------------------------------------------------------
| Description : for getting data from GPIO pins
|
| Parameters  : pin_num -
|                 gpio pin number
|
| Returns     : DAL_ERROR is returned if failure
|               else the DAL_SUCCESS is returned.
+-----------------------------------------------------------------------------*/

S32 get_gpio_input(U32 pin_num)
{
    U32 module_num = pin_num/NUM_OF_BITS_IN_REG + 1;
    /* GPIO module to which the GPIO pin number belongs to */
    U32 snum = (module_num-1)*NUM_OF_BITS_IN_REG;
    U32 offset = pin_num - snum;
    /* position of the pin in the respective gpio module */
    U32 data = 0;
    S32 ret_val = OMAPFLASH_DAL_ERROR;

    /* check whether the gpio number is valid */
    /* if yes continue else return*/
    ret_val = check_gpio_pin_num(pin_num);
    if(ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "wrong gpio-pin number \r\n");
        return(ret_val);
    }

    /* call this function to read the value at the particular pin
    of the corresponding module */
    gpio_read_input_pin(module_num,(1<<offset), &data);
    data = data >> offset;
    /* return from this function*/
    // Commented out in order for CPLD flashing to be speeded up
    //    dbg_print(DBG_LEVEL_INFO,
    //              "read the data at gpio pin %d \n",pin_num);
    return(data);
}

/*-----------------------------------------------------------------------------
| Function    : S32 set_gpio_output(U16 pin_num, U8 set)
+------------------------------------------------------------------------------
| Description : for writing data to the GPIO pins
|
| Parameters  : pin_num -
|                 gpio pin number
|               set -
|                 set/clear information
| Returns     : DAL_ERROR is returned if failure else the value returned
|               by the lower layer function is passed on.
+-----------------------------------------------------------------------------*/
S32 set_gpio_output(U16 pin_num, U8 set)
{
    S32                      ret_val = OMAPFLASH_DAL_ERROR;
    U8                       module_num = pin_num/NUM_OF_BITS_IN_REG + 1;
    /* GPIO module to which the GPIO pin number belongs to */
    U32                      snum = (module_num-1)*NUM_OF_BITS_IN_REG;
    U32                      offset = pin_num - snum;
    /* position of the GPIO pin in respective module register*/

    /* check whether the gpio number is valid */
    /* if yes continue else return*/
    ret_val = check_gpio_pin_num(pin_num);
    if(ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "wrong gpio-pin number \n");
        return(ret_val);
    }

    /* if the output pin value has to be made 1,then the data to be
    written will be - the bit corresponding to that pin is set
    else that bit is cleared and that function is called  */
    if (set)
    {
        gpio_write_output_pin(module_num, (1<<offset), (1<<offset));
    }
    else
    {
        gpio_write_output_pin(module_num, (1<<offset), (0<<offset));
    }
    /* return from this function*/
    // Commented out in order for CPLD flashing to be speeded up
    //    dbg_print(DBG_LEVEL_INFO,
    //              "wrote the data to gpio pin %d \n",pin_num);
    return(ret_val);
}

/*-----------------------------------------------------------------------------
| Function    :S32 gpio_remove_int_handler(U16 pin_num)
+------------------------------------------------------------------------------
| Description :	This function adds the gpio interrupt handler
| Parameters  : pin_num
|                       -   gpio pin number
|
| Returns     : DAL_SUCCESS, if the initialization is successful.
|				else DAL_ERROR.
+-----------------------------------------------------------------------------*/
S32 gpio_remove_int_handler(U16 pin_num)
{
    U8 module_num = pin_num/NUM_OF_BITS_IN_REG + 1;
    /* GPIO module to which the GPIO pin number belongs to */
    U32 snum = (module_num-1)*NUM_OF_BITS_IN_REG;
    U32 pin_offset = pin_num - snum; /* position of the pin in the respective
                                     gpio module */
    S32 ret_val = OMAPFLASH_DAL_ERROR;

    /* check whether the gpio number is valid */
    /* if yes continue else return*/
    ret_val = check_gpio_pin_num(pin_num);
    if(ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL,"wrong gpio-pin number \n");
        return(ret_val);
    }

    /* remove the call back function added to the gpio pin */
    gpio_remove_callback_handler(module_num, pin_offset);

    /* return from this function*/
    dbg_print(DBG_LEVEL_INFO,
              "removed callback function for gpio pin %d \n",pin_num);
    return(ret_val);
}

/*-----------------------------------------------------------------------------
| Function    :S32 gpio_pin_deinit(U16 pin_num)
+------------------------------------------------------------------------------
| Description :	This function de initializes all the features associated with
|               the pins and restores its default/reset state.
|
| Parameters  : pin_num
|						-   this specifies the gpio pin number that has to be
|                           deinitialized.
|
| Returns     : DAL_SUCCESS, a positive value if the de initialization is
|				successful.
|               else DAL_ERROR.
|
+-----------------------------------------------------------------------------*/

S32 gpio_pin_deinit(U16 pin_num)
{
    U8 module_num = pin_num/NUM_OF_BITS_IN_REG + 1;
    /* GPIO module to which the GPIO number belongs to */
    U32 snum = (module_num-1)*NUM_OF_BITS_IN_REG;
    U32 offset = pin_num - snum;
    U32 pinmask;
    S32 ret_val = OMAPFLASH_SUCCESS;

    /* check whether the gpio number is valid */
    /* if yes continue else return*/
    ret_val = check_gpio_pin_num(pin_num);
    if(ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "wrong gpio-pin number \n");
        return(ret_val);
    }

    /* this variable will have the position of the pin in the corresponding
    gpio module */

    pinmask = GPIO_PIN<<offset;

    /* call this function to configure the pin as input - reset state */
    set_gpio_in_out(module_num, pinmask, pinmask);
    /* return from this function*/
    dbg_print(DBG_LEVEL_INFO,
              "de-initialized gpio pin %d \n",pin_num);
    return (ret_val);
}

#endif

