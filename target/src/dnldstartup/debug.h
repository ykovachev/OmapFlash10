/**
 * @file debug.h
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

#ifndef DEBUG_H
#define DEBUG_H

/*==== INCLUDES =============================================================*/
/*==== CONSTS ==============================================================*/

typedef enum
{
  debug_reg_id_cfg,
  debug_reg_id_ll,
  debug_reg_id_cmd,
  debug_reg_id_data,
  debug_reg_id_drv_1,
  debug_reg_id_drv_2,
  debug_reg_id_drv_3,
  debug_reg_id_temp,
  /* Insert new members before this point */
  debug_reg_id_count
} T_debug_reg_id;

/*==== EXPORTS =============================================================*/

extern void log_event (char * text);
extern void send_log(void);

extern void debug_reg_init(void);
extern void debug_reg_setup(T_debug_reg_id reg_id, U32 address, U32 mask, U32 value);
extern void debug_reg_set(T_debug_reg_id reg_id);
extern void debug_reg_clear(T_debug_reg_id reg_id);
extern void debug_reg_toggle(T_debug_reg_id reg_id, U32 set_time, U32 clear_time);

#endif /* DEBUG_H */

/*==== END OF FILE ==========================================================*/
