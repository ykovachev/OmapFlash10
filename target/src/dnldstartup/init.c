/**
 * @file init.c
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

/*==== INCLUDES ==============================================================*/
#include <string.h>
#include "types.h"
#include "mem.h"
#include "init.h"
#include "uart.h"
#include "ll.h"
#include "csst_tgt.h"
#include "silicon.h"
#include "romapi.h"
#ifdef __cplusplus
#error
#endif

#include "omapconfig.h"

extern void * _sys_memory;

/*==== External defines ======================================================*/

extern void main(void);

/*-----------------------------------------------------------------------------
| Function    : watchdog_disable
+------------------------------------------------------------------------------
| Description : Disable watchdogs
|
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
void watchdog_disable(void)
{
  /* Disable ARM11 Watchdog Timer
   * Write WSPR register with start/stop value
   */
  out_regl((WD_BASE + WSPR_WDT2), WD_AAAA);

  /* Read WWPS for pending write */
  while (((in_regl(WD_BASE + WWPS_WDT2)) & 0x10) == 0x10)
  {
      /* Do nothing */
  }

  out_regl((WD_BASE + WSPR_WDT2), WD_5555);
  while (((in_regl(WD_BASE + 0x034)) & 0x10) == 0x10)
  {
      /* Do nothing */
  }
}

/*------------------------------------------------------------------------------
| Function    : relocate_heap
+------------------------------------------------------------------------------
| Description : Relocate the heap to the memory space defined by the board
|               configuration file parameter HEAP_ADDR
|
| Parameters  : None
|
| Returns     : void
+----------------------------------------------------------------------------*/
void relocate_heap(void)
{
  mem_init(get_heap_address()); //clear need_mem_init
}

/*------------------------------------------------------------------------------
| Function    : returned_from_main
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void returned_from_main()
{
  /// list label in debugger
    for(;;);
}

/*------------------------------------------------------------------------------
 | Function    : hw_init
 +------------------------------------------------------------------------------
 | Description : Hardware specific init for 2nd
 |
 | Parameters  : None
 |
 | Returns     : None
 +----------------------------------------------------------------------------*/
void hw_init(void)
{
  watchdog_disable();
  omap_config();

  DEBUG_LOGF_START("\n\n\nOMAPFLASH  OMAPFLASH  OMAPFLASH  OMAPFLASH");
  DEBUG_LOGF("OMAP configured");

  relocate_heap();

  main();
  returned_from_main();
}

/*==== END OF FILE ===========================================================*/
