/**
 * @file csst_tgt.h
 * @author 
 *
 * @section LICENSE
 *
 * Copyright (c) 2009, Texas Instruments, Inc.
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
/*==== DECLARATION CONTROL =================================================*/

#ifndef CSST_CSST_TGT_H
#define CSST_CSST_TGT_H

#include "types.h"
#include "error.h"

/*==== INCLUDES ============================================================*/

/*==== CONSTS ==============================================================*/

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif 

/* These macros in lower case (exception) since these will be changed to function
   once mmu comes into picture  */

#ifdef SIMULATION
#ifdef __cplusplus
extern "C" {
#endif
U8    simulation_in_regb(char* offset_text, U32 offset);
U16   simulation_in_regs(char* offset_text, U32 offset);
U32   simulation_in_regl(char* offset_text, U32 offset);
void  simulation_out_regb(char* offset_text, char* value_text, U32 offset, U8 value);
void  simulation_out_regs(char* offset_text, char* value_text, U32 offset, U16 value);
void  simulation_out_regl(char* offset_text, char* value_text, U32 offset, U32 value);
#ifdef __cplusplus
}
#endif
#define in_regb(offset)			      simulation_in_regb(#offset, offset)
#define in_regs(offset)			      simulation_in_regs(#offset, offset)			
#define in_regl(offset)			      simulation_in_regl(#offset, offset)			
#define out_regb(offset, value)   simulation_out_regb(#offset, #value, offset, value) 
#define out_regs(offset, value)   simulation_out_regs(#offset, #value, offset, value) 
#define out_regl(offset, value)   simulation_out_regl(#offset, #value, offset, value) 
#define lout_regl(offset, value)  simulation_out_regl(#offset, #value, offset, value) 
#else
#define in_regb(offset)			      (*(PREG_U8)(offset))
#define in_regs(offset)			      (*(PREG_U16)(offset))
#define in_regl(offset)			      (*(PREG_U32)(offset))
#define out_regb(offset, value)   (*(PREG_U8)(offset)   = (U8)(value))
#define out_regs(offset, value)   (*(PREG_U16)(offset)  = (U16)(value))
#define out_regl(offset, value)   (*(PREG_U32)(offset)  = (U32)(value))
#define lout_regl(offset, value)  (*(PREG_U32)(offset)  = (U32)(value))
#endif

// Microsecond interval definitions

#define ONE_MICROSEC          1
#define HUNDRED_MICROSEC	    100
#define FIVEHUNDRED_MICROSEC  500
#define ONE_MILLISEC	        1000
#define TEN_MILLISEC		      10000
#define FIFTY_MILLISEC		    50000
#define HUNDRED_MILLISEC	    100000
#define FIVEHUNDRED_MILLISEC  500000
#define ONE_SECOND			      1000000

#endif /* CSST_CSST_TGT_H */
