; *
; * Copyright (c) 2010, Texas Instruments, Inc.
; * All rights reserved.
; * 
; * Redistribution and use in source and binary forms, with or without modification, 
; * are permitted provided that the following conditions are met:
; *   
; *  - Redistributions of source code must retain the above copyright notice, 
; *    this list of conditions and the following disclaimer.
; *  - Redistributions in binary form must reproduce the above copyright notice, 
; *    this list of conditions and the following disclaimer in the documentation 
; *    and/or other materials provided with the distribution.
; *  - Neither the name of Texas Instruments nor the names of its contributors 
; *    may be used to endorse or promote products derived from this software  
; *    without specific prior written permission.
; * 
; * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
; * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
; * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
; * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
; * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
; * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
; * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
; * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
; * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
; * OF THE POSSIBILITY OF SUCH DAMAGE.
; *

; DESCRIPTION
;
; Architectural changes were made to the ARM926 core TLB. The function affected
; by these changes is _dal_mmuArmFlushTLBs. The ARM925 command, Invalidate TLB,
; maps to the ARM926 Invalidate set-associative TLB. Since DAL does not lock
; down any table entries this function suffices in the ARM926 core and no 
; changes were made. For further reference, see ARM926EJS_TRM.pdf.
;

	; Include the compiler defn variables
	.include "compiler.h"
; constants
MMU_ENABLE      .EQU  1
MMU_ACCESS      .EQU  0x3



  .text

  .global _dal_mmu_arm_disable
	.if	ARM9ABI
	.armfunc  _dal_mmu_arm_disable
	.endif
_dal_mmu_arm_disable
    STMFD   sp!,    {r0}                        ; save r0 on stack
    MRC     p15,    #0,    r0,   c1,  c0,   #0  ; read control register
    BIC     r0,     r0,   #MMU_ENABLE           ; clear the MMU bit
    MCR     p15,    #0,    r0,   c1,  c0,   #0  ; write to control register
    NOP
    NOP
    LDMFD   sp!,    {r0}                        ; restore r0 from stack
    MOV     r15,    r14                         ; return

  .global _dal_mmu_arm_enable
	.if	ARM9ABI
	.armfunc  _dal_mmu_arm_enable
	.endif
_dal_mmu_arm_enable
    STMFD   sp!,    {r0}                        ; save r0 on stack
    MRC     p15,    #0,    r0,   c1,   c0,  #0  ; read control register
    ORR     r0,     r0,   #MMU_ENABLE           ; set the MMU bit
    MCR     p15,    #0,    r0,   c1,   c0,  #0  ; write to control register
    NOP
    NOP
    LDMFD   sp!,    {r0}                        ; restore r0 from stack
    MOV     r15,    r14                         ; return

  .global _dal_mmu_arm_flush_tlbs
	.if	ARM9ABI
	.armfunc  _dal_mmu_arm_flush_tlbs
	.endif
_dal_mmu_arm_flush_tlbs
    MCR     p15,    #0,    r0,   c8,   c7,  #0  ; write to TLB register
    MOV     r15,    r14                         ; return

  .global _dal_mmu_arm_set_trans_table
	.if	ARM9ABI
	.armfunc  _dal_mmu_arm_set_trans_table
	.endif
_dal_mmu_arm_set_trans_table
    MCR     p15,    #0,    r0,   c2,   c0,  #0  ; write to Tr Table Base reg
    MOV     r15,    r14                         ; return

  .global _dal_mmu_arm_set_domain_access
	.if	ARM9ABI
	.armfunc  _dal_mmu_arm_set_domain_access
	.endif
_dal_mmu_arm_set_domain_access
    STMFD   sp!,    {r2,r3}                     ; save on stack
    MRC     p15,    #0,   r2,    c3,  c0,   #0  ; read domain register

    MOV     r1,     r1,   LSL r0                ; shift left the access bits
    MOV     r3,     #MMU_ACCESS
    MOV     r3,     r3,   LSL r0                ; use this to clear old bits
    BIC     r2,     r2,   r3                    ; clear old ones
    ORR     r2,     r2,   r1                    ; set new ones
    MCR     p15,    #0,   r2,    c3,  c0,   #0  ; write to control register
    LDMFD   sp!,    {r2,r3}                     ; restore from stack
    MOV     r15,    r14                         ; return

	;/*
	; * CP15 access functions.. need this to control the D/I Cache.
	; */

	;/* C Equivalent = unsigned int read_cp15_c1(void); */
	.global _read_cp15_c1
	.if	ARM9ABI
	.armfunc  _read_cp15_c1
	.endif
_read_cp15_c1:
	MRC     p15,    #0,    r0,   c1,  c0,   #0 ; Read to Reg 0 from CP15 C1
	nop                                        ; Give a bit of settling time
	nop
	nop
	MOV     pc,   lr                           ; back to caller

	;/* C Equivalent = void write_cp15_c1(unsigned int reg_value); */
	.global _write_cp15_c1
	.if	ARM9ABI
	.armfunc  _write_cp15_c1
	.endif
_write_cp15_c1:
	MCR     p15,    #0,    r0,   c1,  c0,   #0 ; Write Reg 0 to CP15 C1
	nop                                        ; Give a bit of settling time
	nop
	nop
	MOV     pc,   lr                           ; back to caller

	;/* C Equivalent = void arm_icache_flush(void); */
	.global _arm_icache_flush
	.if	ARM9ABI
	.armfunc  _arm_icache_flush
	.endif
_arm_icache_flush:
	MCR     p15, #0, r1, c7, c5, #0   ; invalidate I cache
	mov     pc, lr                    ; back to caller
.END

; end of mmu.s 


