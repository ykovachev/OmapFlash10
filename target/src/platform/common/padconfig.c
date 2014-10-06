/**
 * @file padconfig.c
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
#ifndef PADCONFIG_C
#define PADCONFIG_C

#include "csst_tgt.h"
#include "pinmux_fwrk.h"
#include "uart.h"
#include "padconfig.h"

/*==== PRIVATE DATA =========================================================*/

#if 0 // THIS STUFF SHOULD BE IN THE BOARD CONFIGURATION FILE
#if defined DEBUG_UART && defined OMAP3XXX
const PIN_CONFIG uart1_pin_mux[] = {
    {CONTROL_PADCONF_uart1_tx, MODE0, PUPD_DIS, PULL_DOWN},     //  uart1_tx
    {CONTROL_PADCONF_uart1_tx_HI, MODE0, PUPD_DIS, PULL_DOWN},  //  uart1_rts
    {CONTROL_PADCONF_uart1_cts, MODE0, PUPD_DIS, PULL_DOWN},    // uart1_cts
    {CONTROL_PADCONF_uart1_cts_HI, MODE0, PUPD_DIS, PULL_DOWN}  //  uart1_rx
};
#endif 


#if defined DEBUG_UART && defined OMAP3XXX
const PIN_CONFIG uart3_gpio_pin_mux[] = {
    {CONTROL_PADCONF_mcbsp1_clkx_HI, MODE0, PUPD_EN, PULL_DOWN},    // uart3_cts/rctx Pull Low to prevent irda RC mode
    {CONTROL_PADCONF_uart3_rts_sd, MODE0, PUPD_DIS, PULL_DOWN},     // uart3_rts/sd
    {CONTROL_PADCONF_uart3_rts_sd_HI, MODE0, PUPD_DIS, PULL_DOWN},  // uart3_rx/irrx
    {CONTROL_PADCONF_uart3_tx_irtx, MODE0, PUPD_DIS, PULL_DOWN}     // uart3_tx/irtx
};
#endif 

/*==== PUBLIC FUNCTIONS ======================================================*/

#if defined DEBUG_UART && defined OMAP3XXX
void mux_setup_uart1(void)
{
  do_pin_mux(uart1_pin_mux, ((sizeof(uart1_pin_mux)) / (sizeof(PIN_CONFIG))));
}

void enable_gpio_uart3_pinmux(void)
{
  do_pin_mux(uart3_gpio_pin_mux, (sizeof(uart3_gpio_pin_mux)) / (sizeof(PIN_CONFIG)));
}
#endif 
#endif
#endif

