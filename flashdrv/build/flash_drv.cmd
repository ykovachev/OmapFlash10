/*-----------------------------------------------------------------------------
| Copyright (c) 2010, Texas Instruments, Inc.
| All rights reserved.
| 
| Redistribution and use in source and binary forms, with or without modification, 
| are permitted provided that the following conditions are met:
|   
|  - Redistributions of source code must retain the above copyright notice, 
|    this list of conditions and the following disclaimer.
|  - Redistributions in binary form must reproduce the above copyright notice, 
|    this list of conditions and the following disclaimer in the documentation 
|    and/or other materials provided with the distribution.
|  - Neither the name of Texas Instruments nor the names of its contributors 
|    may be used to endorse or promote products derived from this software  
|    without specific prior written permission.
| 
| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
| ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
| WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
| IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
| INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
| BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
| LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
| OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
| OF THE POSSIBILITY OF SUCH DAMAGE.
|
+------------------------------------------------------------------------------
| Filename:   flash_drv.cmd
| Purpose:    Linker command file for flash drivers
+----------------------------------------------------------------------------*/

/*==== Linker options =======================================================*/
-f 0
-heap 100

/*-----------------------------------------------------------------------------
| Memory map definition
+----------------------------------------------------------------------------*/
MEMORY
{
    D_MEM    (RXI): org = 0x9DF00000 len = 0x00100
    P_MEM    (RXI): org = 0x9DF00100 len = 0xF0000
}

/*-----------------------------------------------------------------------------
| Mapping of COFF sections into memory:
+----------------------------------------------------------------------------*/
SECTIONS
{
    .consttbl: palign(4) {} > D_MEM     /* CONSTANT DATA         */
    .text    : palign(4) {} > P_MEM     /* CODE                  */
    .const   : palign(4) {} > P_MEM     /* CONSTANT DATA         */
    .mtext   : palign(4) {} > P_MEM     /* CODE                  */
    .cinit   : palign(4) {} > P_MEM     /* INITIALIZATION TABLES */
    .bss     : palign(4) {} > P_MEM
    .sysmem  : palign(4) {} > P_MEM
}