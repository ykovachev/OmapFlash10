/**
 * @file debug.c
 *
 * @section LICENSE
 *
 * Copyright (c) 2009, Texas Instruments, Inc.
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
 * This file contains target side debug functionality 
 */

/*==== DECLARATION CONTROL ==================================================*/
/*==== INCLUDES ==============================================================*/

#include <stdlib.h>
#include <string.h>
#include "silicon.h"
#include "disp.h"
#include "types.h"
#include "mem.h"
#include "debug.h"

/*==== COMPILER CONTROL ======================================================*/
/*==== MACROS ================================================================*/
/*==== CONSTS ================================================================*/

#define LOG_SIZE 1000
#define DES_SIZE 20

/*==== TYPES =================================================================*/

typedef struct
{
  U32 time;
  char description[DES_SIZE + 1];
} T_log_entry;

typedef struct
{
  U32         index;
  BOOLEAN     wrapped;
  T_log_entry entries[1];
} T_log;

typedef struct
{
  U32 address;
  U32 mask;
  U32 value;
} T_debug_reg;

/*==== EXTERNALS ================================================================*/
/*==== GLOBALS ==================================================================*/

T_log * event_log = NULL;
T_debug_reg debug_reg[debug_reg_id_count];

/*==== PUBLIC FUNCTIONS ======================================================*/
/*-----------------------------------------------------------------------------
| Function    : log_event
+------------------------------------------------------------------------------
| Description : Event log handler - call this function to log an event
|
| Parameters  : text - a text string to log
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void log_event(char * text)
{
  if(event_log == NULL)
  {
    event_log = mem_alloc(sizeof(T_log) + LOG_SIZE * sizeof(T_log_entry));
    event_log->index = 0;
    event_log->wrapped = FALSE;
  }

  event_log->entries[event_log->index].time = *CLK32K_COUNTER_REGISTER;
  
  memcpy(event_log->entries[event_log->index].description, text, DES_SIZE);
  
  event_log->entries[event_log->index].description[DES_SIZE] = 0;
  
  event_log->index++;

  if(event_log->index == LOG_SIZE)
  {
    event_log->index = 0;
    event_log->wrapped = TRUE;
  }
}

/*-----------------------------------------------------------------------------
| Function    : send_log
+------------------------------------------------------------------------------
| Description : Sends the content of the log to the host application. Oldest
|               event will be sent first.
|
| Parameters  : void
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void send_log(void)
{
  if(event_log == NULL)
  {
    send_info("Log empty");
  }
  else
  {
    U32 i;
    
    send_info("Entries: %d %s", event_log->index, event_log->wrapped ? "(wr)" : "");

    if(event_log->wrapped)
    {
      for(i = event_log->index; i < LOG_SIZE; i++)
      {
        send_info("%#08X - %s", event_log->entries[i].time, event_log->entries[i].description);
      }
    }

    for(i = 0; i < event_log->index; i++)
    {
      send_info("%#08X - %s", event_log->entries[i].time, event_log->entries[i].description);
    }
  }
}

/*-----------------------------------------------------------------------------
| Function    : debug_reg_init
+------------------------------------------------------------------------------
| Description : Initialize the register configuration for debug
|
| Parameters  : void
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void debug_reg_init(void)
{
  memset(debug_reg, debug_reg_id_count * sizeof(T_debug_reg), 0x00);
}

/*-----------------------------------------------------------------------------
| Function    : debug_reg_setup
+------------------------------------------------------------------------------
| Description : Configure a register for debug (basically define the register
|               address, register mask and value to use for debug)
|
| Parameters  : reg_id  - ID of the register (predefined)
|               address - Register address to modify
|               mask    - Register mask to use (marking the bits that will be
|                         modified)
|               value   - Value to apply when register is 'set' and to reverse
|                         when register is 'cleared'
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void debug_reg_setup(T_debug_reg_id reg_id, U32 address, U32 mask, U32 value)
{
  if(reg_id < debug_reg_id_count)
  {
    debug_reg[reg_id].address = address;
    debug_reg[reg_id].mask    = mask;
    debug_reg[reg_id].value   = value;
  }
}

/*-----------------------------------------------------------------------------
| Function    : debug_reg_set
+------------------------------------------------------------------------------
| Description : Sets the specified value with the specified mask in a defined
|               register for the predefined register ID
|
| Parameters  : reg_id  - ID of the register (predefined)
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void debug_reg_set(T_debug_reg_id reg_id)
{
  if(reg_id < debug_reg_id_count)
  {
    if(debug_reg[reg_id].address)
    {
      *((PREG_U32)debug_reg[reg_id].address) = (*((PREG_U32)debug_reg[reg_id].address) & ~debug_reg[reg_id].mask) | (debug_reg[reg_id].mask & debug_reg[reg_id].value);
    }
  }
}

/*-----------------------------------------------------------------------------
| Function    : debug_reg_clear
+------------------------------------------------------------------------------
| Description : Clears the specified value with the specified mask in a defined
|               register for the predefined register ID
|
| Parameters  : reg_id  - ID of the register (predefined)
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void debug_reg_clear(T_debug_reg_id reg_id)
{
  if(reg_id < debug_reg_id_count)
  {
    if(debug_reg[reg_id].address)
    {
      *((PREG_U32)debug_reg[reg_id].address) = (*((PREG_U32)debug_reg[reg_id].address) & ~debug_reg[reg_id].mask) | (debug_reg[reg_id].mask & ~debug_reg[reg_id].value);
    }
  }
}

/*-----------------------------------------------------------------------------
| Function    : debug_reg_toggle
+------------------------------------------------------------------------------
| Description : Sets and the clears the specified register with a set and clear
|               time specified as ticks on the 32k clock. Total execution time
|               will be the sum of the set and clear time (approximately).
|
| Parameters  : reg_id     - ID of the register (predefined)
|               set_time   - Time (in 1/32k s) the register will be set
|               clear_time - Time (in 1/32k s) the register will be cleared
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void debug_reg_toggle(T_debug_reg_id reg_id, U32 set_time, U32 clear_time)
{
  if(reg_id < debug_reg_id_count)
  {
    if(debug_reg[reg_id].address)
    {
      U32 ctime = *CLK32K_COUNTER_REGISTER;
      debug_reg_set(reg_id);
      while(set_time)
      {
        if(ctime != *CLK32K_COUNTER_REGISTER)
        {
          set_time--;
          ctime = *CLK32K_COUNTER_REGISTER;
        }
      }
      debug_reg_clear(reg_id);
      while(clear_time)
      {
        if(ctime != *CLK32K_COUNTER_REGISTER)
        {
          clear_time--;
          ctime = *CLK32K_COUNTER_REGISTER;
        }
      }
    }
  }
}

/*==== PRIVATE FUNCTIONS =====================================================*/
/*==== END OF FILE ===========================================================*/
