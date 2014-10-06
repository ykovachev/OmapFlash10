/**
 * @file pinmux_fwrk.c
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

#ifndef PINMUX_FWRK_C
#define PINMUX_FWRK_C


/*==== INCLUDES =============================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "csst_tgt.h"
#include "pinmux_fwrk.h"
#include "dbg_ex.h" 

#if 0
/*==== DEFINES ===============================================================*/

/*==== PRIVATE FUNCTION ======================================================*/

#if defined DEBUG_UART && defined OMAP3
static void config_pad(U32 reg,U8 mode,U8 pull_en,U8 pull_type)
{
	U16 tmp = in_regs(reg);
	tmp = tmp & 0xFFE0;
	tmp |= ((mode & 0x7) |
		      ((pull_en & 0x1) << 3) |
		      ((pull_type & 0x01)  << 4) );
	out_regs(reg, tmp);
}
#endif

/*==== GLOBAL FUNCTIONS ======================================================*/

#if defined DEBUG_UART && defined OMAP3
void do_pin_mux(const PIN_CONFIG pin_mux[],const U32 no_of_pins)
{
	U32 i;
	for(i=0;i<no_of_pins;i++)
	{
		config_pad(pin_mux[i].reg_addr,pin_mux[i].mode,pin_mux[i].pull_en,pin_mux[i].pull_type);
	}
}
#endif

#if defined DEBUG_UART && defined OMAP3
void pin_mux_safe_mode(const PIN_CONFIG pin_mux[],const U32 no_of_pins)
{
	U32 i;
	for(i=0;i<no_of_pins;i++)
	{
		config_pad(pin_mux[i].reg_addr,MODE_SAFE,pin_mux[i].pull_en,pin_mux[i].pull_type);
	}
}
#endif

#endif

#endif 

