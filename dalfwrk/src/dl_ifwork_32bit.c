/**
 * @file dl_ifwork_32bit.c
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

#ifdef SIMULATION
#include <stdio.h>
#endif
#include "dl_ifwork.h"

/*-----------------------------------------------------------------------------
| Function    : dal_lock
+------------------------------------------------------------------------------
| Description : Function to lock for mutual exclusion during C library calls
|
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
void dal_lock(void)
{
#ifdef SIMULATION
    fprintf(stderr,"dal_lock\n");
    dal_disable_interrupts();
#else
    asm	("  .ref     _dal_disable_interrupts");
    asm ("  stmfd    sp!, {r0, r4-r5, lr} ");
    asm ("  mrs      r0, cpsr ");
    asm ("  and      r0, r0, #0x1F ");
    asm ("  cmp      r0, #0x12 ");
    asm ("  beq      dal_lock_return ");
    asm ("  cmp      r0, #0x11 ");
    asm ("  beq      dal_lock_return ");
    asm ("  bl       _dal_disable_interrupts ");
    asm ("dal_lock_return:");
    asm ("  ldmfd    sp!, {r0, r4-r5, lr} ");
#endif
}

/*-----------------------------------------------------------------------------
| Function    : dal_unlock
+------------------------------------------------------------------------------
| Description : Function to unlock for mutual exclusion during C library calls
|
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
void dal_unlock(void)
{
#ifdef SIMULATION
    fprintf(stderr,"dal_unlock\n");
    dal_enable_interrupts();
#else
    asm	(" .ref     _dal_enable_interrupts");
    asm ("  stmfd    sp!, {r0, r4-r5, lr} ");
    asm ("  mrs      r0, cpsr ");
    asm ("  and      r0, r0, #0x1F ");
    asm ("  cmp      r0, #0x12 ");
    asm ("  beq      dal_unlock_return ");
    asm ("  cmp      r0, #0x11 ");
    asm ("  beq      dal_unlock_return ");
    asm ("  bl       _dal_enable_interrupts ");
    asm ("dal_unlock_return:");
    asm ("  ldmfd    sp!, {r0, r4-r5, lr} ");
#endif
}

