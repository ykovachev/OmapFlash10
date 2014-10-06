/**
 * @file dl_ifwork.h
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

#ifndef CSST_DAL_IFWORK_H
#define CSST_DAL_IFWORK_H

/*==== INCLUDES ============================================================*/

#include "csst_tgt.h"
#include "dl_ifwork_ex.h"

/*==== MACROS ============================================================*/

/*==== CONSTS ==============================================================*/

/*==== TYPEDEFINES ===============================================================*/

/*==== STATIC and GLOBAL ======================================================*/

#define MASK_CLEAR		0
#define MASK_SET		1

extern const U8 g_max_interrupts;
extern const U8 g_max_exceptions;
#ifdef DL_IFWORK_PRIVATE
#ifdef INTERRUPT
static T_DAL_INT_USER_HANDLER * dal_int_user_handlers;
static T_DAL_INT_USER_HANDLER * dal_int_user_exc_handlers;
static T_DAL_INT_USER_HANDLER dal_int_swi_handler;
#endif
#endif
/*====== FUNCTION PROTOTYPES==================================================*/
#ifdef __cplusplus
extern "C" {
#endif
void dal_interrupt_mask_all(void);
void dal_irq_disable (void);
void dal_irq_enable (void);
void dal_fiq_disable (void);
void dal_fiq_enable (void);
U32 dal_cpsr (void);

int dal_interrupt_remove_handler(U32 int_num);
S32 dal_interrupt_add_handler(U32 int_num, S32 (*handler) (void *), void *data);
void dal_lock(void);
void dal_unlock(void);
U32 dal_get_exception_vectors(T_EXCEPTION_VECTORS *vectors);
U32 dal_set_exception_vectors(T_EXCEPTION_VECTORS *vectors);

#ifdef __cplusplus
}
#endif
#endif /* CSST_DAL_IFWORK_H */
