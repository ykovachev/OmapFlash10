/**
 * @file pinmux_fwrk.h
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

/*==== DECLARATION CONTROL ==================================================*/

#ifndef PINMUX_FWRK_H
#define PINMUX_FWRK_H

/*==== CONSTS ==============================================================*/

#define  PULL_ENABLE_MASK               0x8
#define  PULL_UP_MASK                   0x10

#define  MODE0                          00
#define  MODE1                          01
#define  MODE2                          02
#define  MODE3                          03
#define  MODE4                          04
#define  MODE5                          05
#define  MODE6                          06
#define  MODE_SAFE                      07

#define  PUPD_EN                        01
#define  PUPD_DIS                       00

#define  PULL_UP                        01
#define  PULL_DOWN                      00


typedef struct 	PIN_CONFIG
{                       ///@todo potential optimize size /= 4
  U32  reg_addr;  //:11 /values 0x48002XXX where XXX is even
  U8   mode;      //:3 //values {0..7}
  U8   pull_en;   //:1 //values {0,1}
  U8   pull_type; //:1 //values {0,1}
} PIN_CONFIG;




/*=====PROTOTYPES============================================================*/

void do_pin_mux(const PIN_CONFIG pin_mux[], const U32 no_of_pins);
void pin_mux_safe_mode(const PIN_CONFIG pin_mux[], const U32 no_of_pins);

#endif

