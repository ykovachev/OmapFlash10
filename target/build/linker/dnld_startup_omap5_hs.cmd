/**
 * @file dnld_startup_omap5_hs.cmd
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

/*==== DEFINES ==============================================================*/

/* For HS Devices */

-stack  0x00002000

MEMORY
{
    SD_STACK_MEM          : org = 0x40300000 len = 0x2000
    BOOT_MEM (RIX)        : org = 0x40308000 len = 0x200
    SD_CONST_MEM (RIX)    : org = 0x40308200 len = 0x700
    SD_P_MEM (RIX)        : org = 0x40308900 len = 0x84F0
    SD_CONFIG_ANCHOR (RIX): org = 0x40310DF0 len = 0x00010
    SD_CONFIG_DATA(RIX)   : org = 0x40310E00 len = 0x00800
    SD_HEAP_MEM (RIX)     : org = 0x40311DE0 len = 0x20
    SD_D_MEM              : org = 0x40311E00 len = 0x2000
    /* total <= 1DFFF */
}
/*
MEMORY
{
    SD_STACK_MEM          : org = 0x40300000 len = 0x2000
    BOOT_MEM (RIX)        : org = 0x40303000 len = 0x200
    SD_CONST_MEM (RIX)    : org = 0x40303200 len = 0x500
    SD_P_MEM (RIX)        : org = 0x40303700 len = 0x56F0
    SD_CONFIG_ANCHOR (RIX): org = 0x40308DF0 len = 0x00010
    SD_CONFIG_DATA(RIX)   : org = 0x40308E00 len = 0x00800
    SD_HEAP_MEM (RIX)     : org = 0x40309DE0 len = 0x20
    SD_D_MEM              : org = 0x40309E00 len = 0x2000
    /* total <= 1DFFF */
/*}*/

SECTIONS
{

    .boot           : {} > BOOT_MEM
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
