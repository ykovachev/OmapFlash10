/**
 * @file types.h
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

/*
DESCRIPTION
-----------
The linker command file for the building the application for GP silicon, which
run from RAM. This is required for the use with the TI CCS tools.
*/

-f 0
-c                                         /* LINK USING C CONVENTIONS */
-heap   0x00000020                         /* HEAP AREA SIZE (not used) */

/* _INTVECTS_PHY_LOC = 0x403070D0; */

/*==== DEFINES ==============================================================*/

/* VEC_TABLE_RELOC_ADDR   = 0x4030fFE4;*/
/* VEC_TABLE_SIZE         = 0x0000001C;*/

/* For HS Devices */

-stack  0x00000D00
MEMORY
{
    /* secure ram         0x40300000 len = 0x4000 */
    /* HS start           0x40304000 */
    /* TOC etc.           0x40304000 len ~ 0x1400 */
    /* todo make room for CH */
    BOOT_MEM (RIX)        : org = 0x40306000 len = 0x0120
    /* INTVECS_MEM (RIX)  : org = 0x40306100 len = 0x0050 */
    SD_CONST_MEM (RIX)    : org = 0x40306120 len = 0x0630
    SD_P_MEM (RIX)        : org = 0x40306750 len = 0x509C
    SD_CONFIG_ANCHOR (RIX): org = 0x4030B7EC len = 0x0010
    SD_CONFIG_DATA(RIX)   : org = 0x4030B7FC len = 0x0800
    /* total <= BFFC */
    SD_STACK_MEM          : org = 0x40304000 len = 0x0D00
    SD_D_MEM              : org = 0x40304D00 len = 0x1200
    SD_HEAP_MEM (RIX)     : org = 0x40305F00 len = 0x20
}

SECTIONS
{

    .boot           : {} > BOOT_MEM
    /* .intvecs        : {} > INTVECS_MEM */         /* interrupt vectors */
    .cinit          : {} > SD_P_MEM             /* INITIALIZATION TABLES */
    .text           :
    {
    } > SD_P_MEM
    .clock          : {} > SD_P_MEM

    GROUP
    {
        .const  : {
            _CONST_SECTION_START = .;
        }
        /* we cant figure out the end of const pretty decently considering padding required etc..
         * So creating a dummy section to handle that */
        dummy: { _CONST_SECTION_SIZE = . - _CONST_SECTION_START; }
    } > SD_CONST_MEM

    .config_anchor : {} > SD_CONFIG_ANCHOR
    .config : {} > SD_CONFIG_DATA

    /*  Globals and statics */
    .bss            : {} > SD_D_MEM

    /*  Data (for assembly) */
    .data           : {} > SD_D_MEM

    /*  Stack */
    .stack          : {} > SD_STACK_MEM

    /*  C-standard heap space goes in SRAM  */
    .sysmem         : {} > SD_HEAP_MEM

}
