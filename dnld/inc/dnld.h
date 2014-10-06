/**
 * @file dnld.h
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

/*==== DECLARATION CONTROL =================================================*/
#ifndef DNLD_H
#define DNLD_H

/*==== INCLUDES ============================================================*/
#include "types.h"
#include "csst_tgt.h"

/*==== CONSTS ==============================================================*/

/*==== TYPES ===============================================================*/

typedef enum T_device_type
{
  BUILT_IN_DRIVER    = 0x00,
  EXTERNAL_DRIVER    = 0x01,
  IS_UNKNOWN         = 0xFF
} T_device_type;

typedef U32 T_dl_lazy_delay_callback(U32 remaining_time_ms, void * data);

/*==== EXPORTS =============================================================*/

void dnld_main_handler(char *cmdline);
S32  dnld_init(void);
void dnld_check_abort_prim(U32 sdu_handle);
U8   dnld_check_valid_write_address_range(U32 destination_address, U32 size);
S32  disp_change_ll_baudrate(U8 new_baudrate);
U32  branchandreturn(U32 runcode);
void dl_lazy_delay(unsigned long time_delay);
U32  dl_lazy_delay_ex(unsigned long time_delay_ms, T_dl_lazy_delay_callback *callback, void *data);

#endif
/*==== END OF FILE ===========================================================*/

