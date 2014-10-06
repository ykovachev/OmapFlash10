/**
 * @file gpio.h
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


/**
 * @file silicon.h
 * @author 
 * @brief 
 * @details 
 * @todo file details
 * @see <link>
 * @todo update link
 */

/*========================= DECLARATION CONTROL ==========================*/

#ifndef GPIO_H
#define GPIO_H

/*============================== INCLUDES ================================*/

#include "csst_tgt.h"
#include "dl_gpio_ex.h"
#include "silicon.h"

/*=============================== MACROS =================================*/
 
#define NUM_PINS_PER_GPIO_MODULE           32

#define GPIO_MAX_MODULES			       6

#define MAX_GPIO_PINS                      192        
                           
#define GPIO_FIRST_PIN                     0x00000001
#define GPIO_INTR                          0x00000001
#define GPIO_MASK_ALL                      0xffffffff

#define GPIO_MODULE_1                      1
#define GPIO_MODULE_2                      2
#define GPIO_MODULE_3                      3
#define GPIO_MODULE_4                      4
 /* for 2430 only */
#define GPIO_MODULE_5                      5
#define GPIO_MODULE_6                      6

/*GPIO Offsets*/
#define GPIO_REVISION					((U32)0x000)
#define GPIO_SYSCONFIG					((U32)0x010)
#define GPIO_SYSSTATUS					((U32)0x014)
#define GPIO_IRQSTATUS1					((U32)0x018)
#define GPIO_IRQENABLE1					((U32)0x01C)
#define GPIO_WAKEUPENABLE               ((U32)0x020)
#define GPIO_IRQSTATUS2					((U32)0x028)
#define GPIO_IRQENABLE2				 	((U32)0x02C)
#define GPIO_CTRL						((U32)0x030)
#define GPIO_OE							((U32)0x034)
#define GPIO_DATAIN						((U32)0x038)
#define GPIO_DATAOUT					((U32)0x03C)
#define GPIO_LEVELDETECT0				((U32)0x040)
#define GPIO_LEVELDETECT1 			    ((U32)0x044)
#define GPIO_RISINGDETECT				((U32)0x048)
#define GPIO_FALLINGDETECT			    ((U32)0x04C)
#define GPIO_DEBOUNCENABLE			    ((U32)0x050)
#define GPIO_DEBONCINGTIME			    ((U32)0x054)
#define GPIO_CLEARIRQENABLE1		    ((U32)0x060)
#define GPIO_SETIRQENABLE1			    ((U32)0x064)
#define GPIO_CLEARIRQENABLE2		    ((U32)0x070)
#define GPIO_SETIRQENABLE2			    ((U32)0x080)
#define GPIO_CLEARWKUPENA				((U32)0x03C)
#define GPIO_SETWKUPENABLE			    ((U32)0x084)
#define GPIO_CLEARDATAOUT				((U32)0x090)
#define GPIO_SETDATAOUT					((U32)0x094)


/*=============================== CONSTS =================================*/

/*TBD*/
 
/*============================= TYPEDEFINES ==============================*/     


typedef struct T_GPIO_PIN_INT_HANDLER
{
    S32          (*pin_handler)(U8 data);
    U8           data;
} T_GPIO_PIN_INT_HANDLER;
		

/*=========================== STATIC & GLOBAL ============================*/
#ifdef GPIO_PRIVATE
static T_GPIO_PIN_INT_HANDLER 
			gpio_handler_table[GPIO_MAX_MODULES]
								[NUM_PINS_PER_GPIO_MODULE];

static U32 g_gpio_module_base_address[GPIO_MAX_MODULES]
				= {GPIO1_MODULE_BA, GPIO2_MODULE_BA, GPIO3_MODULE_BA,
					 GPIO4_MODULE_BA, GPIO5_MODULE_BA, GPIO6_MODULE_BA};
#endif
/*========================= FUNCTION PROTOTYPES ==========================*/
S32 gpio_add_callback_handler(U8 module_num, U32 pin_mask,S32(*handler)(U8),
                                                         U8 data);

S32 gpio_set_interrupt_type(U8 module_num,T_GPIO_INTR_TYPE int_type,
                                                        U32 pin_msk);
S32 gpio_int_handler(void * data);

void gpio_intr_deinit(void);
void gpio_intr_init(void);

S32 gpio_init(void);
S32 check_gpio_pin_num(U32 pin_num);
void set_gpio_in_out( U8 module_num, U32 pin_mask, U32 io_mask);
void gpio_remove_callback_handler(U8 module_num, U32 pin_mask);
S32 gpio_set_interrupt_type(U8 module_num,T_GPIO_INTR_TYPE int_type,
                                                        U32 pin_msk);

#endif /* GPIO_H */ 



