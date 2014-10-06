/**
 * @file dl_gpio_ex.h
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
 * Header file with general GPIO definitions
 * 
 */

/*========================= DECLARATION CONTROL ==========================*/

#ifndef DL_GPIO_EX_H
#define DL_GPIO_EX_H

/*============================== INCLUDES ================================*/

#include "csst_tgt.h"

/*=============================== MACROS =================================*/

#define GPIO_PIN                           1
#define NUM_OF_BITS_IN_REG                 32

#define GPIO_DIRECTION_INPUT               1
#define GPIO_DIRECTION_OUTPUT              0

#define GPIO_DATAOUT_LEVEL_LOW             0
#define GPIO_DATAOUT_LEVEL_HIGH            1
/*=============================== CONSTS =================================*/

/*TBD*/

/*============================= TYPEDEFINES ==============================*/
typedef enum
{
    GPIO_LOW_LEVEL_DETECT = 1,
    GPIO_HIGH_LEVEL_DETECT,
    GPIO_RISING_EDGE_DETECT,
    GPIO_FALLING_EDGE_DETECT
} T_GPIO_INTR_TYPE;

/*=========================== STATIC & GLOBAL ============================*/

/*========================= FUNCTION PROTOTYPES ==========================*/
S32 gpio_pin_init(U16 pin_num, U8 in_out);
S32 set_gpio_output(U16 pin_num, U8 set);
S32 get_gpio_input(U32 pin_num);

void gpio_write_output_pin(U8 module_num, U32 pin_mask, U32 data);
void gpio_read_input_pin(U8 module_num, U32 pin_mask, U32 * data);

S32 gpio_pin_deinit(U16 pin_num);

S32 gpio_add_int_handler(U16 pin_num, S32(*gpio_callback) (U8), U8 data,
                         T_GPIO_INTR_TYPE intr_type);

S32 gpio_remove_int_handler(U16 pin_num);
S32 set_gpio_output(U16 pin_num, U8 set);
S32 get_gpio_input(U32 pin_num);

#endif /* DL_GPIO_EX_H */
