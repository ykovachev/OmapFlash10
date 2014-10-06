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

    ; Include the compiler defn variables
    .include "compiler.h"
    
    .cdecls
    %{
    #include "config.h"
    %}
    
;/*****************************************************************************
; * Global defines and refernces
; ****************************************************************************/
    .ref    _dal_IRQ_Hndl

    .ref    VEC_TABLE_RELOC_ADDR
    .ref    VEC_TABLE_SIZE
    .ref    cinit
    .ref    ___data__
    .ref    ___end__
    .ref    __STACK_SIZE
 .if !$$defined(MINI_STARTUP) ; TODO figure out where __SYSMEM_SIZE should come from
    .ref    __SYSMEM_SIZE
 .endif
    .ref    _hw_init
    .ref    _PRCMConfiguration
    .def    _reset

;/*****************************************************************************
; * Switch to 32 bit instruction assembly
; ****************************************************************************/
    .state32

;/*****************************************************************************
; * This is the boot section
; ****************************************************************************/
    .sect ".boot"

;/*****************************************************************************
; * This is the start routine
; ****************************************************************************/
_reset:
    ;b _reset


	  ldr     r0,     _data
	  ldr     r2,     _end
	  sub		r4,     r4,     r4

_clear:
	  str     r4,     [r0], #4
	  cmp     r0,     r2
	  blt		_clear

    ;/**********************************
    ; * Intialize the C-environment
    ; *********************************/

    ldr     r0, CINIT_START
    cmn     r0, #1
    blne    c_initialization

    ldr     sp, C_STACK_START
    ldr     r0, C_STACK_SIZE
    add     sp, sp, r0
    
    ldr     r1, C_STACK_START
    ldr     r2, STACK_FILL_PATTERN

_fill_stack:
    str     r2, [r1], #4
    cmp     r1, sp
    blt     _fill_stack
    

;_icache_enable:
    ;/* Set up for MCR */
    ;mov     r0, #0                            
    ;/* Invalidate icache */
    ;mcr     p15, #0, r0, c7, c5, #0             
    ;/* Invalidate BP array */
    ;mcr     p15, #0, r0, c7, c5, #6             
    ;/* Get current control into R0 */
    ;mrc     p15, #0, r0, c1, c0, #0             
    ;/* ORR in i-cache enable bit */
    ;orr     r0, r0, #0x00001000               
    ;/* Enable i-cache */
    ;mcr     p15, #0, r0, c1, c0, #0             

    
    ;/**********************************
    ; * Go to C hw_init
    ; *********************************/
    add     lr, pc, #1
    bx      lr
    .state16
    bl      _hw_init
    .state32

;/*****************************************************************************
; * Definition of variables
; ****************************************************************************/
CINIT_START         .word   cinit
_data               .word   ___data__
_end                .word   ___end__

STACK_FILL_PATTERN  .word   0xAFFEAFFE

__stack:            .usect  ".stack", 0, 4

C_STACK_START       .long    __stack
C_STACK_SIZE        .long    __STACK_SIZE

;---------------------------------------------------------------------
;  PROCESS INITIALIZATION TABLE.
;
;  THE TABLE CONSISTS OF A SEQUENCE OF RECORDS OF THE FOLLOWING FORMAT:
;
;       .word  <length of data (bytes)>
;       .word  <address of variable to initialize>
;       .word  <data>
;
;  THE INITIALIZATION TABLE IS TERMINATED WITH A ZERO LENGTH RECORD.
;
;----------------------------------------------------------------------
tbl_addr: .set    R0
var_addr: .set    R1
length:   .set    R2
data:     .set    R3

c_initialization:
    B       rec_chk
    ;*------------------------------------------------------
    ;* PROCESS AN INITIALIZATION RECORD
    ;*------------------------------------------------------
record:
    ldr     var_addr, [tbl_addr], #4   ;
    ;*------------------------------------------------------
    ;* COPY THE INITIALIZATION DATA
    ;*------------------------------------------------------
    tst     var_addr, #3                ;SEE IF DEST IS ALIGNED
    bne     _bcopy                      ;IF NOT, COPY BYTES
    subs    length, length, #4          ;IF length <= 3, ALSO
    bmi     _bcont                      ;COPY BYTES

_wcopy:
    ldr     data, [tbl_addr], #4
    str     data, [var_addr], #4        ; COPY A WORD OF DATA
    subs    length, length, #4
    bpl     _wcopy
_bcont:
    adds    length, length, #4
    beq     _cont

_bcopy:
    ldrb    data, [tbl_addr], #1
    strb    data, [var_addr], #1        ; COPY A BYTE OF DATA
    subs    length, length, #1
    bne     _bcopy

_cont:
    ands    length, tbl_addr, #0x3      ; MAKE SURE THE ADDRESS
    rsbne   length, length, #0x4        ; IS WORD ALIGNED
    addne   tbl_addr, tbl_addr, length

rec_chk:
    ldr     length, [tbl_addr], #4      ; PROCESS NEXT
    cmp     length, #0                  ; RECORD IF LENGTH IS
    bne     record                      ; NONZERO

    mov     pc, lr                      ;return

;/*****************************************************************************
; * IRQ enable
; *
; ****************************************************************************/
    .state32
    .global _irq_enable
    .armfunc _irq_enable

_irq_enable:
    stmfd   sp!, {r5}
    mrs     r5, cpsr
    bic     r5, r5, #0x80
    msr     cpsr_cxsf, r5
    ldmfd   sp!, {r5}
    mov     pc, lr

;/*****************************************************************************
; * IRQ disable
; *
; ****************************************************************************/

    .state32
    .global _irq_disable
    .armfunc _irq_disable

_irq_disable:
    stmfd   sp!, {r5}
    mrs     r5, cpsr
    orr     r5, r5, #0x80
    msr     cpsr_cxsf, r5
    ldmfd   sp!, {r5}
    mov     pc, lr

.state16
    .global $asm_branch_to_runcode

.state32
    .global _branch_to_runcode

.text

.END
