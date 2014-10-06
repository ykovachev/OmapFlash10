/**
 * @file omapconfig.c
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
 * This file contains the target side download functionality for handling memories
 * and drivers in connection with download of binaries to an OMAP based platform.
 */

/*==== DECLARATION CONTROL ==================================================*/
/*==== INCLUDES ==============================================================*/

#define PRIVATE_DNLD_C
#include <ctype.h>
#include <stdlib.h>
#include "types.h"
#include "mem.h"
#include "disp.h"
#include "flash_drv.h"
#include "csst_tgt.h"
#include "silicon.h"
#include "ll.h"
#include "dnld.h"
#include "adler32.h"
#include "uart.h"
#include "romapi.h"
#include "debug.h"

/*==== COMPILER CONTROL ======================================================*/

#ifdef __TMS470__
/// turn off remark generated for va_start
#pragma diag_suppress 238 //controlling expression is constant
#endif

/*==== MACROS ================================================================*/

#define RELATIVE_TO_ABSOLUTE_DRIVER_IF(FUNCTION) \
  { \
  if (!MY_ASSERT(relative_driver_if->functions.FUNCTION != NULL)) \
    { \
    relative_driver_if = NULL; \
    return; \
    } \
    absolute_driver_if.FUNCTION = (T_driver_##FUNCTION)(drv_offset + (U32)relative_driver_if->functions.FUNCTION - (U32)relative_driver_if->functions.self); \
  }

#ifdef MTMARCHX
#define BIT_SET(addr, bit_mask)         *((U32 *)addr) = (*((U32 *)addr) | bit_mask)
#define BIT_CLEAR(addr, bit_mask)       *((U32 *)addr) = (*((U32 *)addr) & ~bit_mask)
#define BIT_IS_SET(addr, bit_mask)      ((*((U32 *)addr) & bit_mask) != 0)
#define BIT_IS_CLEARED(addr, bit_mask)  ((*((U32 *)addr) & bit_mask) == 0)
#endif

/*==== CONSTS ================================================================*/

#define KILO 1024U
#define MEGA (KILO * KILO)
#define GIGA (KILO * MEGA)

#define SDRAM_START_ADR              0x80000000

#define MAX_NESTED_LOOP_TIMEOUTS     3

/*==== TYPES =================================================================*/

#if defined MTADDR || defined MTQUICK || defined MTMARCHX
typedef BOOLEAN (* T_mt_action )(U32 cur_address);
#endif 

#if defined MTADDR || defined MTQUICK || defined MTMARCHX || defined MTIF
const char * progress = "Progress";
#endif 

typedef S8 receive_call_memcpy_t(void *context, U8 *data, U32 size, BOOLEAN last);

/*==== EXTERNALS ================================================================*/

#define MY_ASSERT(CONDITION) my_assert(CONDITION, #CONDITION)

extern void irq_disable(void);

int     my_assert         (int condition, char *message);
void    drv_free          (void *pointer);
void    drv_debug_printf  (const char *format, ...);
void    drv_debug_vxprintf(int flags, const char *format, va_list arg_ptr);
void    drv_debug_signal  (unsigned int id, unsigned int set);
BOOLEAN start_with        (char **text, const char *word);
U64     get_value         (char **text, char *name, T_get_value_option options);
BOOLEAN loop_timeout_ms   (U8 id, U32 time);

void memorydump(U32 start, U32 end, BOOLEAN info);

/*==== GLOBALS ==================================================================*/

BOOLEAN relax_check = FALSE;

#ifdef MTIF
U32 cs_memory_span;
U8  cs_count;
#endif

#ifdef LOG_STATISTICS
BOOLEAN log_stats = FALSE;
#endif

#ifdef LOG_STATISTICS
static struct  
{
  U32 tstart;
  U32 tend;
  U32 await_transfer;
  U32 await_action;
  unsigned long bytes_transfer;
  unsigned long bytes_action;
  char action[9]; 
} statistics =
{
  0, 0, 0, 0, 0, 0, 
  {0, 0, 0, 0, 0, 0, 0, 0, 0} 
};
#endif


T_driver_config driver_config =
{
  mem_alloc,
  mem_free,
  memcmp,
  strcmp,
  strncmp,
  memcpy,
  strcpy,
  strlen,
  memset,
  dl_lazy_delay,
  dl_lazy_delay_ex,
  drv_debug_printf,
  drv_debug_vxprintf,
  send_info,
  vxsend_info,
  send_status,
  log_event,
  loop_timeout_ms,
  drv_debug_signal,
  start_with,
  get_value,
  PROCESSOR,
  NULL,
  NULL
};

/* Flash driver interface structure */

T_driver_if * relative_driver_if = NULL;
T_driver_if_functions absolute_driver_if;

#define DRIVER_DEVICE_NAME_SIZE 10
char driver_device[DRIVER_DEVICE_NAME_SIZE + 1];  // Name of device for which a driver has been downloaded
char command_device[DRIVER_DEVICE_NAME_SIZE + 1]; // Name of device for which a command has been sent latest

/*==== PRIVATE FUNCTIONS =====================================================*/
/*==== PUBLIC FUNCTIONS ======================================================*/
/*------------------------------------------------------------------------------
| Function    : drv_debug_printf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void drv_debug_printf(const char *format, ...)
{
#ifdef DEBUG_UART
  va_list arg_ptr;
  va_start(arg_ptr, format);
  uart_config_terminal(TEXT_CYAN);
  debug_vprintf(format, arg_ptr);
  va_end(arg_ptr);
  uart_config_terminal(TEXT_NORMAL);
#endif
}

/*------------------------------------------------------------------------------
| Function    : drv_debug_vxprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void drv_debug_vxprintf(int flags, const char *format, va_list arg_ptr)
{
#ifdef DEBUG_UART
  uart_config_terminal(TEXT_CYAN);
  debug_vxprintf(flags, format, arg_ptr);
  uart_config_terminal(TEXT_NORMAL);
#endif
}

/*------------------------------------------------------------------------------
| Function    : drv_debug_signal
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void drv_debug_signal(unsigned int id, unsigned int set)
{
  T_debug_reg_id reg_id = (T_debug_reg_id)((unsigned int)debug_reg_id_drv_1 + id);
  if(reg_id <= debug_reg_id_drv_3)
  {
    if(set)
    {
      debug_reg_set(reg_id);
    }
    else
    {
      debug_reg_clear(reg_id);
    }
  }
  //DEBUG_LED(leds_mask, leds_value);
}

/*------------------------------------------------------------------------------
| Function    : dl_lazy_delay_ex
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : time_delay_microsec - time to wait
|               callback - called for every iteration in waiting loop, stop 
|                          looping if non-zero returned
|               data - pointer to optional data for callback
|
| Returns     : zero if waiting specified time or value returned by callback 
|               if not
+----------------------------------------------------------------------------*/
U32 dl_lazy_delay_ex(unsigned long time_delay_microsec, T_dl_lazy_delay_callback *callback, void *data)
{
    U32 count;
    volatile U32 *sync32khzclk = CLK32K_COUNTER_REGISTER;
    /// monitor last knwon good time
    U32 fin_timeval;
    U32 clk_div;
    U32 count_microsec;
    U32 result = 0;
    U32 init_timeval = *sync32khzclk;
    clk_div = 1;
    count_microsec = (1000 * clk_div) / 32;
    count = time_delay_microsec * 32 / (1000 * clk_div); // sync32clk is 260 Hz
    fin_timeval = init_timeval;
    while (count > (fin_timeval - init_timeval))
    {
      U32 remaining_time = (count - (fin_timeval - init_timeval)) * count_microsec;
      result = callback(remaining_time, data);
      if (result)
          break;
      fin_timeval = *sync32khzclk;
      /*loop around of sync32khzclk */
      if (fin_timeval < init_timeval)
      {
        count = count - fin_timeval;
        init_timeval = 0;
      }
    }
    return result;
}

/*------------------------------------------------------------------------------
| Function    : dl_lazy_delay_noop_callback
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 dl_lazy_delay_noop_callback(U32 remaining_time_ms, void *data)
{
  return 0;
}

/*------------------------------------------------------------------------------
| Function    : dl_lazy_delay
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void dl_lazy_delay(unsigned long time_delay_microsec)
{
  dl_lazy_delay_ex(time_delay_microsec, dl_lazy_delay_noop_callback, NULL);
}

/*------------------------------------------------------------------------------
| Function    : loop_timeout_ms
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN loop_timeout_ms(U8 id, U32 time)
{
  static U32 start_time[MAX_NESTED_LOOP_TIMEOUTS];
  BOOLEAN ret = FALSE;

  if(id < MAX_NESTED_LOOP_TIMEOUTS)
  {
    if(time == 0)
    {
      start_time[id] = *CLK32K_COUNTER_REGISTER;
    }
    else
    {
      U32 actual_time = *CLK32K_COUNTER_REGISTER;
      if(start_time[id] <= actual_time)
      {
        actual_time -= start_time[id];
      }
      else
      {
        actual_time += 0xFFFFFFFF - start_time[id];
      }

      if((actual_time / 32) >= time)
      {
        ret = TRUE;
      }
    }
  }

  return ret;
}

/*------------------------------------------------------------------------------
| Function    : branch_setup
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void branch_setup(void)
{
  /* Disable the unwanted clocks */
  // Resetallclocks();
}

/*------------------------------------------------------------------------------
| Function    : skip_space
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void skip_space(char **text)
{
  char *p = *text;
  while (*p && isspace(*p))
    p++;
  *text = p;
}

/*------------------------------------------------------------------------------
| Function    : skip_word
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void skip_word(char **text)
{
  char *p = *text;
  while (*p && !isspace(*p))
    p++;
  *text = p;
}

/*------------------------------------------------------------------------------
| Function    : start_with
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN start_with(char **text, const char *word)
{
  char *p = *text;
  while (*p && *word && *p == *word) 
  {
    p++;
    word++;
  }
  if (*word)
    return FALSE;
  if (*p && !isspace(*p))
    return FALSE;
  while (*p && isspace(*p))
    p++;
  *text = p;
  return TRUE;
}

/*------------------------------------------------------------------------------
| Function    : get_word
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
char * get_word(char **text)
{
  char *p = *text;
  char *result = NULL;
  skip_space (&p);
  if (*p) {
    result = p;
    skip_word(&p);
    if (*p)
      *p++ = 0;
    skip_space (&p);
  }
  else
  {
    result = NULL;
  }
  *text = p;
  return result;
}

/*------------------------------------------------------------------------------
| Function    : send_no_flash_driver
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_no_flash_driver()
{
  DEBUG_LOGF("Device access: %s", command_device);
  DEBUG_LOGF("Device driver: %s", driver_device);
  send_fail("No or wrong driver");
}

/*------------------------------------------------------------------------------
| Function    : flash_driver_dnld_check
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 flash_driver_dnld_check(void)
{
  if(relative_driver_if)
  {
    return OMAPFLASH_SUCCESS;
  }
  return OMAPFLASH_ERROR;
}

/*------------------------------------------------------------------------------
| Function    : get_value
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 get_value_ex(char **text, char *name, U64 * value, T_get_value_option options)
{
  char *p = get_word(text);
  if (!p) 
  {
    if (!(options & OPTIONAL))
      send_fail("%s missing", name);
    return FALSE;
  }
  else 
  {
    char *end;

#ifdef _MSC_VER
    *value = _strtoui64(p, &end, (options & DECIMAL) ? 10 : 16);
#else
    *value = strtoull(p, &end, (options & DECIMAL) ? 10 : 16);
#endif

    if (*end) 
    {								
      send_fail("%s is not %s", (options & DECIMAL) ? "decimal" : "hex", name);
      return FALSE;
    }
    else if (!(options & DECIMAL) && !(options & NOALIGN)) 
    {
      /// relax_check do not apply to ALLOW_ODD since whether an address is executable is well defined
      /// where as if the underlying code do/should support unaligned code is mostly a matter of implementation/taste
      if (options & ALLOW_ODD) 
      {
        if ((*value & 3) == 2) 
        {
          send_fail("%s mod 4 must be 0 or odd (ARM or THUMB)", name);
          return FALSE;
        }
      }
      else if (!relax_check) 
      {
        if (*value & 3) 
        {
          send_fail("%s (%#08X) mod 4 must be 0", name, *value);
          return FALSE;
        }
      }
    }

    return TRUE;
  }
}

/*------------------------------------------------------------------------------
| Function    : get_value
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U64 get_value(char **text, char *name, T_get_value_option options)
{
  U64 ret;
  if(get_value_ex(text, name, &ret, options))
  {
    return ret;
  }
  else
  {
    return (U64)-1LL;
  }
}

/*------------------------------------------------------------------------------
| Function    : get_offset
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U64 get_offset(char **text, const T_device_type device_type, T_get_value_option options)
{
  U64 result = get_value(text, "size", DEFAULT);

  if(device_type == BUILT_IN_DRIVER)
  {
    // TODO Removed check for overflow since the implementation was inconsistent
    return result;
  }
  else if (flash_driver_dnld_check())
  {
    send_no_flash_driver();
    result = (U64)-1LL;
  }
  else
  {
    T_driver_info info;

    absolute_driver_if.get_info(&driver_config, &info);
   
    if(result >= info.device_size)
    {
      send_fail("offset parameter too large - %#llX >= %#llX", result, info.device_size);
      result = (U64)-1LL;
    }
  }
  return result;
}

/*------------------------------------------------------------------------------
| Function    : get_other_value
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 get_other_value(char **text, char *name, U32 * value, U32 other_value, T_get_value_option options)
{
  U64 result;
  U32 ret =  (other_value != (U32)-1) ? get_value_ex(text, name, &result, options) : FALSE;
  *value = (U32)result;
  return ret;
}

/*------------------------------------------------------------------------------
| Function    : get_size
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U64 get_size(char **text, const T_device_type device_type, U64 offset)
{
  if (offset !=  (U64)-1LL)
  {
    U64 result = get_value(text, "size", DEFAULT);

    if(device_type == BUILT_IN_DRIVER)
    {
      // TODO: Removed check for overflow since it was not consistent and correctly implemented
      return result;
    }
    else if (flash_driver_dnld_check())
    {
      send_no_flash_driver();
    }
    else
    {
      T_driver_info info; U64 driver_device_sizeL;
      absolute_driver_if.get_info(&driver_config, &info);
      driver_device_sizeL = info.device_size;

      if((offset + result > driver_device_sizeL) || (offset + result < offset)) /* check for overflow */
      {
        send_fail("offset (%d) + size (%d) over device size (%d)", offset, result, info.device_size);
      }
      else 
      {
        return result;
      }
    }
  }
  return (U64)-1LL;
}

/*------------------------------------------------------------------------------
| Function    : my_toupper
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int my_toupper(int c)
{
  if ('a' <= c && c <= 'z')
    return c - ('a' - 'A');
  else
    return c;
}

#ifndef SIMULATION
///@deprecated use my_stricmp
int stricmp(const char *a, const char *b);
#endif

/*------------------------------------------------------------------------------
| Function    : my_stricmp
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int my_stricmp(char *a, char *b)
{
  while (*a && *b && my_toupper(*a) == my_toupper(*b)) 
  {
    a++;
    b++;
  }
  return my_toupper(*a) - my_toupper(*b);
}

/*------------------------------------------------------------------------------
| Function    : get_device
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
T_device_type get_device(char **text)
{
  char *word = get_word(text);
  if (!word) 
  {
    send_fail("Missing device name");
    return(IS_UNKNOWN);
  }
  else 
  {
    memset(command_device, 0, DRIVER_DEVICE_NAME_SIZE + 1);
    strncpy(command_device, word, DRIVER_DEVICE_NAME_SIZE);

    if (!my_stricmp("SDRAM", word)) 
    {
      return(BUILT_IN_DRIVER);
    }
    else
    {
      return(EXTERNAL_DRIVER);
    }
  }
}

/*------------------------------------------------------------------------------
| Function    : expect_end
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN expect_end(char *text, char *cmd)
{
    skip_space(&text);
    if (*text) 
    {
      send_fail("Too many parameters for: %s", cmd);
      return FALSE;
    }
    else 
    {
      return TRUE;
    }
}

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : request_data
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void request_data(T_device_type device_type, U32 total_size)
{
  U32 package_size;
  U8 package_count;

  /* There is an issue with ROM USB Read unable to read last partial block data
   * if the size is >64K. Full-block reads doesn't have any limitations. So, we
   * split the read request to full-block reads + one partial block read call.
   * This way we dont worry about the size of read.  
   */
//  if(total_size > 512 && total_size & 0x1FF)
//  {
//    total_size = (total_size / 512) * 512;
//  }

  if(total_size >= DATA_BLOCK_SIZE_DOWNLOAD)
  {
    U32 count     = total_size / DATA_BLOCK_SIZE_DOWNLOAD;
    package_size  = DATA_BLOCK_SIZE_DOWNLOAD; 
    package_count = (U8)(count > DATA_BLOCK_COUNT_DOWNLOAD ? DATA_BLOCK_COUNT_DOWNLOAD : count);
  }
  else 
  {
    if(total_size >= 512)
    {
      #ifdef FORCE_NON_MODULUS_64_PACKAGE_SIZE
      if((total_size & 0x000001FF) == 0)
      {
        total_size = total_size - DATA_BLOCK_SIZE_ADJUST;
      }
      #else
      if((total_size & 0x000001FF) != 0)
      {
        total_size = total_size & ~0x000001FF;
      }
      #endif
    }
    package_size  = total_size;
    package_count = 1;
  }

  //DEBUG_LOGF("Requesting data: data%08X", (package_size & 0xFFFFFF) + ((package_count & 0xFF) << 24));
  xsend(NULL, VXPRINTF_NO_REQUIRE_HASH | XSEND_FLAGS_UNACKED, "data%08X", 
        (package_size & 0xFFFFFF) + ((package_count & 0xFF) << 24));
}
#endif


#ifdef LOG_STATISTICS
/*------------------------------------------------------------------------------
| Function    : log_statistics
+------------------------------------------------------------------------------
| Description : Log statistical data based on the 32k clock
|
| Parameters  : await_data - time spent waiting for data
|               await_flashing - time spent waiting for the driver
|
| Returns     : void
+----------------------------------------------------------------------------*/
void log_statistics(void)
{
  if(log_stats)
  {
    unsigned long pct_transfer = statistics.await_transfer;
    unsigned long rate;
    unsigned long rate_multiplier;

    pct_transfer *= 1000;
    pct_transfer /= (statistics.await_transfer + statistics.await_action);

    #define KB(x) (x * 1024)
    #define MB(x) (x * 1024 * 1024)

    rate_multiplier = KB(1);

    rate = (statistics.bytes_transfer < 100 * rate_multiplier) ?
            ((statistics.bytes_transfer * 32000) / (statistics.await_transfer + 1)) / rate_multiplier :
            ((statistics.bytes_transfer / rate_multiplier) * 32000) / (statistics.await_transfer + 1);

    DEBUG_LOGF_STATS("Transfer: %d:%02d.%03d %3d.%d pct - %d bytes (%d kbytes/s)", 
                     (statistics.await_transfer / 32000) / 60,
                     (statistics.await_transfer / 32000) % 60,
                     (statistics.await_transfer / 32) % 1000,
                     (pct_transfer / 10),
                     (pct_transfer % 10),
                     statistics.bytes_transfer,
                     rate);

    rate = (statistics.bytes_action < 100 * rate_multiplier) ?
            ((statistics.bytes_action * 32000) / (statistics.await_action + 1)) / rate_multiplier :
            ((statistics.bytes_action / rate_multiplier) * 32000) / (statistics.await_action + 1);

    DEBUG_LOGF_STATS("%s: %d:%02d.%03d %3d.%d pct - %d bytes (%d kbytes/s)",
                     statistics.action,
                     (statistics.await_action / 32000) / 60,
                     (statistics.await_action / 32000) % 60,
                     (statistics.await_action / 32) % 1000,
                     100 - (pct_transfer / 10) - ((pct_transfer % 10) > 0 ? 1 : 0),
                     ((10 - (pct_transfer % 10)) % 10),
                     statistics.bytes_action,
                     rate);

    rate = (statistics.bytes_transfer < 100 * rate_multiplier) ?
            ((statistics.bytes_transfer * 32000) / (statistics.await_transfer + statistics.await_action + 1)) / rate_multiplier :
            ((statistics.bytes_transfer / rate_multiplier) * 32000) / (statistics.await_transfer + statistics.await_action + 1);

    DEBUG_LOGF_STATS("End2End : %d:%02d.%03d (%d kbytes/s)",
                     ((statistics.await_action + statistics.await_transfer) / 32000) / 60,
                     ((statistics.await_action + statistics.await_transfer) / 32000) % 60,
                     ((statistics.await_action + statistics.await_transfer) / 32) % 1000,
                     rate);
  }
}
#endif 

#ifdef LOG_STATISTICS
/*------------------------------------------------------------------------------
| Function    : void init_statistics(const char * action)
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void init_statistics(const char * action)
{
  if(log_stats)
  {
    statistics.await_transfer  = 0;
    statistics.await_action = 0;
    statistics.bytes_transfer = 0;
    statistics.bytes_action = 0;
    memset(statistics.action, ' ', 8);
    memcpy(statistics.action, action, strlen(action) > 8 ? 8 : strlen(action));
    statistics.tstart = *CLK32K_COUNTER_REGISTER;
  }
}
#endif

#ifdef LOG_STATISTICS
/*------------------------------------------------------------------------------
| Function    : void track_statistics_tend(BOOLEAN action, U32 bytes)
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void track_statistics_tend(BOOLEAN action, U32 bytes)
{
  if(log_stats)
  {
    statistics.tend = *CLK32K_COUNTER_REGISTER;
    if(action)
    {
      statistics.bytes_action += bytes;
      statistics.await_action += (statistics.tend >= statistics.tstart) ? statistics.tend - statistics.tstart : 0xFFFFFFFF - statistics.tstart + statistics.tend;
    }
    else
    {
      statistics.bytes_transfer += bytes;
      statistics.await_transfer += (statistics.tend >= statistics.tstart) ? statistics.tend - statistics.tstart : 0xFFFFFFFF - statistics.tstart + statistics.tend;
    }
    statistics.tstart = *CLK32K_COUNTER_REGISTER;
  }
}
#else
#define init_statistics(action)
#define track_statistics_tend(action, bytes)
#define log_statistics()
#endif

/*------------------------------------------------------------------------------
| Function    : check_device_driver
+------------------------------------------------------------------------------
| Description : Checks the name of the device for which a driver has been
|               downloaded against the name of the device for which a command 
|               has been issued.
|
| Parameters  : None
|
| Returns     : TRUE if devices match, FALSE if not
+----------------------------------------------------------------------------*/
BOOLEAN check_device_driver(void)
{
  return(!strcmp(driver_device, command_device));
}

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : expecting_download
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 expecting_download(T_device_type device_type, receive_call_memcpy_t *receive_call, void *context)
{
  t_message * msg;
  S8 result_code;
  char *cmdline;
  const char download[] = "download:";
  const size_t download_size = sizeof(download) - 1; ///< -1 for zero terminator

  send_ok("expecting download");
  
  msg = ll_receive();
  cmdline = (char *)(msg->content);

  if (!strncmp(cmdline, download, download_size))
  {
    char *end;
    U32 total_size = strtoul(cmdline + download_size, &end, 16);
    if (*end)
      for(;;); ///<@todo recover bad download
    if (!total_size) 
    {
      send_bug("size is 0 bytes");
      return OMAPFLASH_ERROR;
    }

    //TODO check that no device driver is required for given address and that ram is free
    
    init_statistics("Writing");

    request_data(device_type, total_size);

    for (;;) 
    {
      msg = ll_receive();
      
      if(msg->header.fields.ctrl) 
      {
        ///next fifo entry is not part of download
        if (total_size) 
        {
          send_bug("missing %#04X bytes - got control message", total_size);
          return OMAPFLASH_ERROR;
        }
        else 
        {
          log_statistics();          
          return OMAPFLASH_SUCCESS;
        }
      }
      else if (msg->header.fields.length > total_size) 
      {
        send_bug("surplus bytes (%#04X+)", msg->header.fields.length - total_size);
        return OMAPFLASH_ERROR;
      }
      else 
      {
        BOOLEAN last = total_size == msg->header.fields.length;
        // DEBUG_LOGF("Expecting download: Data %d Last %d Total Size %d More %d", msg->header.fields.length, last, total_size, msg->header.fields.more);

        debug_reg_set(debug_reg_id_data);
        /// This function is a receive_call_memcpy_t it should do the equivalent of 
        /// <code>
        ///     memcpy(location, data, package_size);
        ///     context.location += package_size;
        /// <code>

        track_statistics_tend(FALSE, msg->header.fields.length + sizeof(t_message_header));

        result_code = receive_call(context, msg->content, msg->header.fields.length, last);

        track_statistics_tend(TRUE, msg->header.fields.length);

        debug_reg_clear(debug_reg_id_data);

        if (result_code != OMAPFLASH_SUCCESS || last)
        {
          log_statistics();          
          return result_code;
        }

        if (!last)
        {
          total_size -= msg->header.fields.length;
          if (!msg->header.fields.more)
          {
            //DEBUG_LOGF("Expecting download: Requesting more data");
            request_data(device_type, total_size);
          }
        }
      }
    }
  } 
  else
  {
    send_bug("failed to receive download");
    return OMAPFLASH_ERROR;
  }
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : receive_call_memcpy
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 receive_call_memcpy(void *context, U8 *data, U32 size, BOOLEAN last)
{
  U8 **location = (U8**)context;
  memcpy(*location, data, size);
  *location += size;
  return OMAPFLASH_SUCCESS;
}
#endif

/*------------------------------------------------------------------------------
| Function    : my_assert
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int my_assert(int condition, char *message)
{
  if (!condition)
  {
    send_bug("%s is NULL", message);
  }
  return condition;
}

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : driver_dnld_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void driver_dnld_handler(char * device, char * config_string)
{
  static const char omapflash_driver_header_string[OMAPFLASH_DRIVER_HEADER_STRING_SIZE] = OMAPFLASH_DRIVER_HEADER_STRING_V7;
  U32 drv_offset;
  U32 result_code;
  U8 *location;
  char * result = NULL;

  DEBUG_LOGF("Device driver: %s", device);

  memset(driver_device, 0, DRIVER_DEVICE_NAME_SIZE + 1);
  strncpy(driver_device, device, DRIVER_DEVICE_NAME_SIZE);

  /* Deinitialization of previous device driver */

  if(relative_driver_if)
  {
    absolute_driver_if.deinit(&driver_config, &result);
    relative_driver_if = NULL;
  }

  if(driver_config.cstring)
  {
    free(driver_config.cstring);
    driver_config.cstring = NULL;
  }

  if(driver_config.cstring = (char *)mem_alloc(strlen(config_string) + 1))
  {
    strcpy(driver_config.cstring, config_string);
  }
  else
  {
    send_fail("Unable to allocate driver configuration");
    return;
  }

  /* Copy flash driver to specified address */

  drv_offset = get_drv_address();

  location = (U8 *)drv_offset;

  result_code = expecting_download(BUILT_IN_DRIVER, receive_call_memcpy, &location);

  if (result_code != OMAPFLASH_SUCCESS)
  {
    return;
  }

  /* Update function pointers */

  relative_driver_if = (T_driver_if *)drv_offset;

  if(memcmp(relative_driver_if->id.header, omapflash_driver_header_string, OMAPFLASH_DRIVER_HEADER_STRING_SIZE))
  {
    send_fail("Fail: driver version expected %s - got %s", omapflash_driver_header_string, relative_driver_if->id.header);
    relative_driver_if = NULL;
    return;
  }

  DEBUG_LOGF("Interface '%s'", relative_driver_if->id.header);
  DEBUG_LOGF("Driver '%s'", relative_driver_if->id.driver);

  RELATIVE_TO_ABSOLUTE_DRIVER_IF(deinit);
  RELATIVE_TO_ABSOLUTE_DRIVER_IF(erase);
  RELATIVE_TO_ABSOLUTE_DRIVER_IF(init);
  RELATIVE_TO_ABSOLUTE_DRIVER_IF(write);
  RELATIVE_TO_ABSOLUTE_DRIVER_IF(read); 
  RELATIVE_TO_ABSOLUTE_DRIVER_IF(get_info); 

  result_code = absolute_driver_if.init(&driver_config, &result);
  if(result_code != FLASH_DRV_SUCCESS)
  {
    send_fail("Driver init error %#08X - %s", result_code, result);
    relative_driver_if = NULL;
    return;
  }

  send_ok("driver downloaded");
}
#endif

#ifdef DOWNLOAD
/*-----------------------------------------------------------------------------
| Function    : erase_handler
+------------------------------------------------------------------------------
| Description : Erase handler
|
| Parameters  : sdu_handler [IN] - SDU handler
|
| Returns     : void
+-----------------------------------------------------------------------------*/
void erase_handler(T_device_type device_type, U64 offset, U64 size)
{
  char *result = NULL;
  if (device_type == BUILT_IN_DRIVER)
  {
    send_fail("Erase not supported for device");
  }
  else if (flash_driver_dnld_check() || !check_device_driver())
  {
    send_no_flash_driver();
  }
  else
  {
    T_driver_info info;
    absolute_driver_if.get_info(&driver_config, &info);

    if(size)
    {
      send_info("Erasing %llX bytes starting at %llX", size, info.device_base_address + offset);
    }
    else
    {
      send_info("Erasing to end of device starting at %llX", info.device_base_address + offset);
    }

    if (absolute_driver_if.erase(&driver_config, &result, info.device_base_address + offset, size) != FLASH_DRV_SUCCESS)
    {
      send_fail("Error - %s", result);
      return;
    }
    send_ok("%#llX bytes erased at %#llX", size, info.device_base_address + offset);
  }
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : read_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void read_handler(T_device_type device_type, U64 offset, U64 size)
{    
  if(device_type != BUILT_IN_DRIVER && (flash_driver_dnld_check() || !check_device_driver()))
  {
    send_no_flash_driver();
  }
  else
  {
    T_driver_info info;
    int result_code;
    char * result;
    U32 data_block_size = is_uart_connection() ? DATA_BLOCK_SIZE_UPLOAD_UART : DATA_BLOCK_SIZE_UPLOAD_USB;
    U8 *read_buf = NULL;
    U32 s = size > data_block_size ? data_block_size : (U32)size;

    xsend(NULL, VXPRINTF_NO_REQUIRE_HASH, "DATA%08X", size);

    if(device_type != BUILT_IN_DRIVER)
    {
      absolute_driver_if.get_info(&driver_config, &info);
      read_buf = (U8 *)mem_alloc(s);
      if(NULL == read_buf)
      {   
        send_out_of_memory();
        return;
      }
    }
    
    init_statistics("Reading");

    while (size > 0)
    {
      s = size > data_block_size ? data_block_size : (U32)size;

      if(device_type == BUILT_IN_DRIVER)
      {
        if(offset > 0xFFFFFFFF)
        {
          send_fail("Flash read failed - SDRAM offset should be a 32-bit long. Wrong offset - %#llX", offset);
          return;
        }
        read_buf = (U8 *)(SDRAM_START_ADR + (U32)offset);
      }
      else
      {
        result_code = absolute_driver_if.read(&driver_config, &result, (U32)read_buf, info.device_base_address + offset, s);
        if (result_code != FLASH_DRV_SUCCESS)
        {
          free (read_buf);
          send_fail("Flash read failed - %s (error code %#08X)", result, result_code);
          return;
        }
      }

      track_statistics_tend(TRUE, s);

      if(s % 512 == 0)
      {
          send_upload(read_buf, s - 4);
          send_upload(read_buf + (s - 4), 4);
      }
      else
      {
          send_upload(read_buf, s);
      }

      track_statistics_tend(FALSE, s + (s & 0x200 ? 1 : 2) * sizeof(t_message_header));
      
      size -= s;
      offset += s;
    }

    free (read_buf);
    log_statistics();
    send_ok("Upload finished");
  }
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : receive_call_driver
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 receive_call_driver(void *context, U8 *data, U32 size, BOOLEAN last)
{
    T_more_data more;
  U64 *destination = (U64*)context;
    U32 result_code;
    char * result = NULL;
    more = last ? NO_MORE_DATA : MORE_DATA;
    result_code = absolute_driver_if.write(&driver_config, &result, *destination, (U32)data, size, more);
    if (result_code != FLASH_DRV_SUCCESS)
    {
        send_fail("Write error %#08X - %s", result_code, result);
        return OMAPFLASH_ERROR;
    }
    *destination += size;
    return OMAPFLASH_SUCCESS;
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : write_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void write_handler(const T_device_type device_type, U64 offset, U64 total_size)
{
  S8 result_code;

  if(device_type == BUILT_IN_DRIVER)
  {
    U32 destination;
    // Incase of SDRAM, the offset should be max 32-bit long. Perform that check..
    if(offset > 0xFFFFFFFF)
    {
      send_fail("Flash write failed - SDRAM offset should be a 32-bit long. Wrong offset - %#llX", offset);
      return;
    }
    destination = SDRAM_START_ADR + (U32) offset;
    result_code = expecting_download(device_type, receive_call_memcpy, &destination);
  }
  else
  {
    if(flash_driver_dnld_check() || !check_device_driver())
    {
      send_no_flash_driver();
      return;
    }
    else
    {
      T_driver_info info;
      U64 destination;
      absolute_driver_if.get_info(&driver_config, &info);
      destination = info.device_base_address + offset;
      if(total_size > (info.device_size - offset))
      {
        send_fail("Too much data - %#llX > (%#llX - %#llX)", total_size, info.device_size, offset);
        return;
      }
      result_code = expecting_download(device_type, receive_call_driver, &destination);
    }
  }

  if (result_code != OMAPFLASH_SUCCESS)
      return;
  send_ok("Write complete");
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : checksum_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void checksum_handler(const T_device_type device_type, U64 offset, U64 size)
{
  if(flash_driver_dnld_check() || !check_device_driver())
  {
    send_no_flash_driver();
  }
  else
  {
    U8 *read_buf = (U8 *)mem_alloc(DATA_BLOCK_SIZE_CHECKSUM);
    U32 checksum = adler32(read_buf, 0); // init checksum for repeated calls to update_adler32()

    if(NULL != read_buf)
    {
      T_driver_info info;
      absolute_driver_if.get_info(&driver_config, &info);
      while (size > 0) 
      {
        char * result;
        U32 s = size > DATA_BLOCK_SIZE_CHECKSUM ? DATA_BLOCK_SIZE_CHECKSUM : (U32)size;
        int result_code = absolute_driver_if.read(&driver_config, &result, (U32)read_buf, info.device_base_address + offset, s);
        if (result_code != FLASH_DRV_SUCCESS) 
        {
          free(read_buf);
          send_fail("Flash read - %s", result);
          return;
        }
        else 
        {
          checksum = update_adler32(checksum, read_buf, s);
        }
        offset += s;
        size -= s;
      }
      send_ok("Checksum=%#08X", checksum);
      free(read_buf);
    }
    else
    {  
      send_out_of_memory();
    }
  }
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : sw_reset
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void sw_reset(BOOLEAN cold)
{
  send_ok("Reseting");
  /* Wait for last SDU to be sent and deinitialize ll + drv */
  //disp_deinit();
  //dal_disable_interrupts();
  if (cold)
  {
    *(VU32*)PRM_RSTCTRL = RST_GLOBAL_COLD_SW;
  }
  else
  {
    *(VU32*)PRM_RSTCTRL = RST_GLOBAL_WARM_SW;
  }
  dl_lazy_delay(ONE_SECOND);
  send_fail("Not successful");
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : cold_sw_reset_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void cold_sw_reset_handler(void)
{
  sw_reset(TRUE);
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : warm_sw_reset_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void warm_sw_reset_handler(void)
{
  sw_reset(FALSE);
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : branch_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void branch_handler(T_device_type device, U32 offset)
{
    typedef void T_runcode (void);

    if(device == BUILT_IN_DRIVER)
    {
      T_runcode *runcode = (T_runcode *) (SDRAM_START_ADR + offset);
      send_ok("Branching");

      branch_setup();
      (*runcode)();                     /* Branch to branch address */
    }
    else 
    {
      send_fail("Command only for SDRAM.");
    }
}
#endif

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : jump_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void jump_handler(U32 address)
{
  typedef void T_runcode (void);
  T_runcode *runcode = (T_runcode *)(address);
  send_ok("Jumping to absolute address %#08X", address);

  /* We need to Leave the Target in a defined state prior to handing over the
   * control */
  
  branch_setup();
  
  (*runcode)();
}
#endif

#ifdef RECONNECT_HANDLER
/*------------------------------------------------------------------------------
| Function    : reconnect_connect_fail
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void reconnect_connect_fail(void)
{
    for(;;);
}
#endif

#ifdef RECONNECT_HANDLER
/*------------------------------------------------------------------------------
| Function    : reconnect_handler
+------------------------------------------------------------------------------
| Description : Used for romcode validation
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void reconnect_handler(void)
{
    t_connect connect;
    int result;

    send_ok("Reconnecting");
    dl_lazy_delay(2 * ONE_SECOND);
    ll_disconnect(&connect);

    dl_lazy_delay(5 * ONE_SECOND);

    connect = connect_usb;

    result = ll_connect(connect);
    if (result != OMAPFLASH_SUCCESS)
        reconnect_connect_fail();

    send_ok("Reconnected");
}
#endif //RECONNECT_HANDLER

#ifdef DOWNLOAD
/*------------------------------------------------------------------------------
| Function    : relax_check_handler
+------------------------------------------------------------------------------
| Description : only intended for validation purposes, however if you do use 
|               this command it is assumed you know what you are doing
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void relax_check_handler(void)
{
    relax_check = TRUE;
    send_ok(NULL);
}
#endif

#if defined MTMARCHX || defined MTQUICK
/*------------------------------------------------------------------------------
| Function    : mt_action_clear_all_ascending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_action_clear_all_ascending(U32 cur_address)
{
  *((volatile U32 *)cur_address) = 0x00000000;
  return TRUE;
}
#endif

#ifdef MTADDR
/*------------------------------------------------------------------------------
| Function    : mt_action_fill_with_address_ascending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_action_fill_with_address_ascending(U32 cur_address)
{
  *((volatile U32 *)cur_address) = cur_address;
  return TRUE;
}
#endif 

#ifdef MTADDR
/*------------------------------------------------------------------------------
| Function    : mt_action_check_for_address_ascending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_action_check_for_address_ascending(U32 cur_address)
{
  if(*((volatile U32 *)cur_address) != cur_address)
  {
    send_info("Failed at %#08X - read %#08X", cur_address, *((volatile U32 *)cur_address), cur_address);
    return FALSE;
  }
  return TRUE;
}
#endif

#ifdef MTQUICK
/*------------------------------------------------------------------------------
| Function    : mt_action_quick_test
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_action_quick_test(U32 cur_address)
{
  BOOLEAN ret = TRUE;

  if(*((volatile U32 *)cur_address) != 0x00000000)
  {
    send_info("At %#08X: read %#08X not 0x00000000", cur_address, *((volatile U32 *)cur_address));
    ret = FALSE;
  }

  *((volatile U32 *)cur_address) = 0x55555555;

  if(*((volatile U32 *)cur_address) != 0x55555555)
  {
    send_info("At %#08X: read %#08X not 0x55555555", cur_address, *((volatile U32 *)cur_address));
    ret = FALSE;
  }

  *((volatile U32 *)cur_address) = 0xAAAAAAAA;

  if(*((volatile U32 *)cur_address) != 0xAAAAAAAA)
  {
    send_info("At %#08X: read %#08X not 0xAAAAAAAA", cur_address, *((volatile U32 *)cur_address));
    ret = FALSE;
  }
  
  return ret;
}
#endif

#ifdef MTMARCHX
/*------------------------------------------------------------------------------
| Function    : mt_action_marchx_check_cleared_and_set_ascending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_action_marchx_check_cleared_and_set_ascending(U32 cur_address)
{
  U32 bit_mask;
  BOOLEAN ret = TRUE;

  for(bit_mask = 0x00000001; bit_mask != 0; bit_mask = bit_mask << 1)
  {
    if(BIT_IS_CLEARED(cur_address, bit_mask) == FALSE)
    {
      send_info("At %#08X:%#08X - bit not cleared.", cur_address, bit_mask);
      ret = FALSE;
    }
    BIT_SET(cur_address, bit_mask);
  }
  return ret;
}
#endif

#ifdef MTMARCHX
/*------------------------------------------------------------------------------
| Function    : mt_action_marchx_check_set_and_clear_descending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_action_marchx_check_set_and_clear_descending(U32 cur_address)
{
  U32 bit_mask;
  BOOLEAN ret = TRUE;

  for(bit_mask = 0x80000000; bit_mask != 0; bit_mask = bit_mask >> 1)
  {
    if(BIT_IS_SET(cur_address, bit_mask) == FALSE)
    {
      send_info("At %#08X:%#08X - bit not set.", cur_address, bit_mask);
      ret = FALSE;
    }
    BIT_CLEAR(cur_address, bit_mask);
  }
  return ret;
}
#endif

#ifdef MTMARCHX
/*------------------------------------------------------------------------------
| Function    : mt_action_marchx_check_cleared_ascending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_action_marchx_check_cleared_ascending(U32 cur_address)
{
  U32 bit_mask;
  BOOLEAN ret = TRUE;

  for(bit_mask = 0x00000001; bit_mask != 0; bit_mask = bit_mask << 1)
  {
    if(BIT_IS_SET(cur_address, bit_mask))
    {
      send_info("At %#08X:%#08X - bit not cleared.", cur_address, bit_mask);
      ret = FALSE;
    }
  }
  return ret;
}
#endif

#if defined MTADDR || defined MTQUICK || defined MTMARCHX
/*------------------------------------------------------------------------------
| Function    : mt_loop_ascending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_loop_ascending(char * headline, U32 start_address, U32 end_address, T_mt_action mt_action)
{
  U32 cur_address;
  BOOLEAN ret = TRUE;

  send_info(headline);

  for(cur_address = start_address; cur_address <= end_address; cur_address += 4)
  {
    if((((cur_address - start_address) & 0x3FF) == 0))
    {
      send_status(progress, (U64)(cur_address - start_address), (U64)(end_address - start_address));
    }
    
    if(!mt_action(cur_address))
    {
      ret = FALSE;
    }
  }

  send_status(progress, (U64)(end_address - start_address), (U64)(end_address - start_address));
  return ret;
}
#endif

#if defined MTADDR || defined MTQUICK || defined MTMARCHX
/*------------------------------------------------------------------------------
| Function    : mt_loop_descending
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN mt_loop_descending(char * headline, U32 start_address, U32 end_address, T_mt_action mt_action)
{
  U32 cur_address;
  BOOLEAN ret = TRUE;

  send_info(headline);

  for(cur_address = end_address; cur_address >= start_address; cur_address -= 4)
  {
    if((((end_address - cur_address) & 0x3FF) == 0))
    {
      send_status(progress, (U64)(end_address - cur_address), (U64)(end_address - start_address));
    }

    if(!mt_action(cur_address))
    {
      ret = FALSE;
    }
  }
  
  send_status(progress, (U64)(end_address - start_address), (U64)(end_address - start_address));
  return ret;
}
#endif

#ifdef MTIF
#ifdef MTIF_SHOWCFG
/*------------------------------------------------------------------------------
| Function    : MTIF_show_dmm_lisa_map
+------------------------------------------------------------------------------
| Description : Dump out the LISA MAP register configuration
|
| Parameters  : None
|
| Returns     : void
+----------------------------------------------------------------------------*/
void MTIF_show_dmm_lisa_map(void)
{
  U8 instance;
  volatile U32 * reg;
  for(instance = 0; instance < 4; instance++)
  {
    send_info("DMM_LISA_MAP_%d", instance);

    reg = (volatile U32 *)(DMM_LISA_MAP_REGISTER_0 + (4 * instance));

    if(*reg == 0)
    {
      send_info("- Unused\n");
    }
    else
    {
      const char * const sdrc_map[] = {"UNMAPPED", "EMIF1", "EMIF2", "EMIF1&2"};
      const char * const sdrc_intl[] = {"NONE", "128-BYTE", "256-BYTE", "512-BYTE"};

      send_info("- SDRC_ADDR   : %#02X",    DMM_LISA_MAP_SDRC_ADDR(*reg));
      send_info("- SDRC_MAP    : %#02X %s", DMM_LISA_MAP_SDRC_MAP(*reg), DMM_LISA_MAP_SDRC_MAP(*reg) < sizeof(sdrc_map) ? sdrc_map[DMM_LISA_MAP_SDRC_MAP(*reg)] : "?");
      send_info("- SDRC_ADDRSPC: %#02X",    DMM_LISA_MAP_SDRC_ADDRSPC(*reg));
      send_info("- SDRC_INTL   : %#02X %s", DMM_LISA_MAP_SDRC_INTL(*reg), DMM_LISA_MAP_SDRC_INTL(*reg) < sizeof(sdrc_map) ? sdrc_intl[DMM_LISA_MAP_SDRC_INTL(*reg)] : "?");
      send_info("- SYS_SIZE    : %#02X %d-MB SECTION", DMM_LISA_MAP_SYS_SIZE(*reg), 16 << DMM_LISA_MAP_SYS_SIZE(*reg));
      send_info("- SYS_ADDR    : %#02X\n", DMM_LISA_MAP_SYS_ADDR(*reg));
    }
  }
}
#endif

#ifdef MTIF_SHOWCFG
/*------------------------------------------------------------------------------
| Function    : MTIF_show_sdram_config
+------------------------------------------------------------------------------
| Description : Show the configuration of SDRAM setup in EMIF controller
|
| Parameters  : None
|
| Returns     : void
+----------------------------------------------------------------------------*/
void MTIF_show_sdram_config(void)
{
  U8 instance;
  volatile U32 * reg;
  for(instance = 0; instance < 2; instance++)
  {
    const char * const reg_sdram_type[] = {"?", "?", "?", "?", "LPDDR2-S4", "LPDDR2-S2", "?", "?"};

    send_info("EMIF%d_SDRAM_CONFIG", instance + 1);

    reg = (volatile U32 *)(EMIF1_SDRAM_CONFIG + (0x01000000 * instance));

    send_info("- REG_PAGESIZE   : %#02X %d WORDS/PAGE (%d COLUMNS)", EMIF_SDRAM_CONFIG_REG_PAGESIZE(*reg), 256 << EMIF_SDRAM_CONFIG_REG_PAGESIZE(*reg), 8 + EMIF_SDRAM_CONFIG_REG_PAGESIZE(*reg));
    send_info("- REG_EBANK      : %#02X %s", EMIF_SDRAM_CONFIG_REG_EBANK(*reg), EMIF_SDRAM_CONFIG_REG_EBANK(*reg) ? "CS0/CS1" : "CS0");
    send_info("- REG_IBANK      : %#02X %d BANKS", EMIF_SDRAM_CONFIG_REG_IBANK(*reg), 1 << EMIF_SDRAM_CONFIG_REG_IBANK(*reg));
    send_info("- REG_ROWSIZE    : %#02X %d ROW BITS", EMIF_SDRAM_CONFIG_REG_ROWSIZE(*reg), 9 + EMIF_SDRAM_CONFIG_REG_ROWSIZE(*reg));
    send_info("- REG_CL         : %#02X", EMIF_SDRAM_CONFIG_REG_CL(*reg));
    send_info("- REG_NARROW_MODE: %#02X %d BITS", EMIF_SDRAM_CONFIG_REG_NARROW_MODE(*reg), EMIF_SDRAM_CONFIG_REG_NARROW_MODE(*reg) ? 16 : 32);
    send_info("- REG_DISABLE_DLL: %#02X DLL %s", EMIF_SDRAM_CONFIG_REG_DISABLE_DLL(*reg), EMIF_SDRAM_CONFIG_REG_DISABLE_DLL(*reg) ? "DISABLED" : "ENABLED");
    send_info("- REG_DDR2_DDQS  : %#02X %s", EMIF_SDRAM_CONFIG_REG_DDR2_DDQS(*reg), EMIF_SDRAM_CONFIG_REG_DDR2_DDQS(*reg) ? "DIFFERENTIAL" : "SINGLE ENDED");
    send_info("- REG_IBANK_POS  : %#02X", EMIF_SDRAM_CONFIG_REG_IBANK_POS(*reg));
    send_info("- REG_SDRAM_TYPE : %#02X %s\n", EMIF_SDRAM_CONFIG_REG_SDRAM_TYPE(*reg), reg_sdram_type[EMIF_SDRAM_CONFIG_REG_SDRAM_TYPE(*reg)]);

    send_info("EMIF%d_SDRAM_CONFIG_2", instance + 1);

    reg = (volatile U32 *)(EMIF1_SDRAM_CONFIG_2 + (0x01000000 * instance));

    send_info("- RDBSIZE        : %#02X %d BYTES", EMIF_SDRAM_CONFIG_2_REG_RDBSIZE(*reg), 32 << EMIF_SDRAM_CONFIG_2_REG_RDBSIZE(*reg));
    send_info("- RDBNUM         : %#02X %d ROW BUF", EMIF_SDRAM_CONFIG_2_REG_RDBNUM(*reg), 1 << EMIF_SDRAM_CONFIG_2_REG_RDBNUM(*reg));
    send_info("- EBANK_POS      : %#02X %s OCP ADR BITS", EMIF_SDRAM_CONFIG_2_REG_EBANK_POS(*reg), EMIF_SDRAM_CONFIG_2_REG_EBANK_POS(*reg) ? "LOWER" : "HIGHER");
    send_info("- CS1NVMEM       : %#02X CS1 NVM %s\n", EMIF_SDRAM_CONFIG_2_REG_CS1NVMEM(*reg), EMIF_SDRAM_CONFIG_2_REG_CS1NVMEM(*reg) ? "ENABLED" : "DISABLED");

  }
}
#endif

#ifdef MTIF_SHOWCFG
/*------------------------------------------------------------------------------
| Function    : MTIF_show_config
+------------------------------------------------------------------------------
| Description : Logs the memory controller configuration
|
| Parameters  : None
|
| Returns     : void
+----------------------------------------------------------------------------*/
void MTIF_show_config(void)
{
  MTIF_show_dmm_lisa_map();
  MTIF_show_sdram_config();
}
#endif

/*------------------------------------------------------------------------------
| Function    : MTIF_init
+------------------------------------------------------------------------------
| Description : Checks the prerequisites for memory interface testing
|
| Parameters  : None
|
| Returns     : TRUE for successful check, FALSE for fail
+----------------------------------------------------------------------------*/
BOOLEAN MTIF_init(void)
{
  BOOLEAN ret          = TRUE;
  U32 sdram_config_1   = *((volatile U32 *)EMIF1_SDRAM_CONFIG);
  U32 sdram_config_2   = *((volatile U32 *)EMIF2_SDRAM_CONFIG);
  U32 sdram_config_2_1 = *((volatile U32 *)EMIF1_SDRAM_CONFIG_2);
  // U32 sdram_config_2_2 = *((volatile U32 *)EMIF2_SDRAM_CONFIG_2);
  U32 lisa_map_0       = *((volatile U32 *)DMM_LISA_MAP_REGISTER_0);
  // U32 lisa_map_1       = *((volatile U32 *)DMM_LISA_MAP_REGISTER_1);
  // U32 lisa_map_2       = *((volatile U32 *)DMM_LISA_MAP_REGISTER_2);
  // U32 lisa_map_3       = *((volatile U32 *)DMM_LISA_MAP_REGISTER_3);

#ifdef MTIF_SHOWCFG  
  MTIF_show_config();
#endif

  cs_memory_span = ((256 << EMIF_SDRAM_CONFIG_REG_PAGESIZE(sdram_config_1)) * (512 << EMIF_SDRAM_CONFIG_REG_ROWSIZE(sdram_config_1))) 
                   << (EMIF_SDRAM_CONFIG_REG_IBANK(sdram_config_1) + (2 - EMIF_SDRAM_CONFIG_REG_NARROW_MODE(sdram_config_1)));

  cs_count = EMIF_SDRAM_CONFIG_REG_EBANK(sdram_config_1) + 1;

  if(DMM_LISA_MAP_SDRC_INTL(lisa_map_0) != 1)
  {
    send_info("Only 128-byte interleaving supported");
    ret = FALSE;
  }

  if(sdram_config_1 != sdram_config_2)
  {
    send_info("EMIF1 != EMIF2 config not supported");
    ret = FALSE;
  }

  if(EMIF_SDRAM_CONFIG_REG_IBANK_POS(sdram_config_1) != 3)
  {
    send_info("Bank interleaving not supported");
    ret = FALSE;
  }

  if((EMIF_SDRAM_CONFIG_REG_EBANK(sdram_config_1) == 1) &&
    (EMIF_SDRAM_CONFIG_2_REG_EBANK_POS(sdram_config_2_1)))
  {
    send_info("CS interleaving not supported");
    ret = FALSE;
  }

  return ret;
}

#define TCOUNT 100000

/*------------------------------------------------------------------------------
| Function    : MTIF_cs_cke_ca_dqs
+-------------------------------------------------------------------------------
| Description : Check for issues with the CS, CKE, CA and DQS signals
|
| Parameters  : DDR base address, emif controller instance (1/2) and description
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN MTIF_cs_cke_ca_dqs(U32 address, U32 emif, char * channel_descr)
{
  volatile U32 * word = (volatile U32 *)address;
  U32 last_word = *word;
  U32 new_word;
  U32 tcount, bcount;
  U32 error_count = 0;
  U32 error_bit_pos[32];
  U32 error_byte_pos[4];
  U32 failed_bits = 0;

  send_info("Checking CS, CKE, CA, DQS at %#08X (%s)", address, channel_descr);

  memset(error_bit_pos, 0, 32 * sizeof(U32));
  memset(error_byte_pos, 0, 4 * sizeof(U32));
  
  for(tcount = 0; tcount < TCOUNT; tcount++)
  {
    if(!(tcount & 0x0F))
    {
      send_status(progress, tcount + 1, TCOUNT);
    }

    new_word = *word;

    if(new_word != last_word)
    {
      error_count++;
      for(bcount = 0; bcount < 32; bcount++)
      {
        if(((new_word >> bcount) & 0x00000001) != ((last_word >> bcount) & 0x00000001))
        {
          error_byte_pos[bcount / 8] += 1;
          error_bit_pos[bcount] += 1;
          failed_bits |= (0x00000001 << bcount);
        }
      }
      last_word = new_word;
    }
  }

  send_status(progress, TCOUNT, TCOUNT);

  if(!error_count)
  {
    send_info("Done - OK\n");
  }
  else
  {
    send_info("Instability in %d cases of %d", error_count, TCOUNT);
    for(bcount = 0; bcount < 32; bcount++)
    {
      send_info("b%02d : %6d transitions", bcount, error_bit_pos[bcount]);
    }

    if(error_byte_pos[0] && error_byte_pos[1] && error_byte_pos[2] && error_byte_pos[3])
    {
      send_info("All bytes unstable -> CS, CKE or CA[9:0]?");
    }
    else
    {
      U8 nof_unstable_bytes = (error_byte_pos[0] > 0) + (error_byte_pos[1] > 0) + (error_byte_pos[2] > 0) + (error_byte_pos[3] > 0);

      if(nof_unstable_bytes > 1)
      {
        send_info("Multiple bytes unstable -> CA[9:0] or DQS?");
      }

      for(bcount = 0; bcount < 4; bcount++)
      {
        U8 i = (failed_bits >> (bcount * 8)) & 0xFF;
        U8 bytebits = 0;

        while(i) 
        {
          bytebits += i & 0x01;
          i >>= 1;
        }
        
        if(bytebits > 1)
        {
          send_info("Multiple bits in byte %d unstable -> DQS%d_t and/or DQS%d_c?", bcount, bcount, bcount);
        }
        else if(bytebits)
        {
          send_info("One bit in byte %d unstable -> DQ?", bcount);
        }
      }
    }
    send_info("Failed\n");
  }

  return (error_count == 0);
}

/*------------------------------------------------------------------------------
| Function    : MTIF_dm_check
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN MTIF_dm_check(BOOLEAN expect_equal, U8 bytes_skipped, U32 reference_value, U32 actual_value)
{
  U8 bcount;
  U8 expected_byte;
  U8 actual_byte;
  U8 failedcount = 0;

  for(bcount = 0; bcount < 4; bcount++)
  {
    if(((bytes_skipped >> bcount) & 0x01) == 0x00)
    {
      expected_byte = ((reference_value >> (bcount * 8)) & 0x000000FF);
      actual_byte   = ((actual_value >> (bcount * 8)) & 0x000000FF);
      if(((expect_equal == TRUE) && (actual_byte != expected_byte)) || ((expect_equal == FALSE) && (actual_byte == expected_byte)))
      {
        failedcount++;
        send_info("Byte %d is %#02X %s %#02X -> DM%d?", bcount, actual_byte, expect_equal ? "!=" : "==", expected_byte, bcount);
      }
    }
  }

  if(failedcount == 4)
  {
    send_info("All bytes failed - CKE?");
  }

  return (failedcount == 0);
}

/*------------------------------------------------------------------------------
| Function    : MTIF_dm
+------------------------------------------------------------------------------
| Description : Check DM signals
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN MTIF_dm(U32 address, char * channel_descr)
{
  BOOLEAN ret = TRUE;
  volatile U32 * word = (volatile U32 *)address;
  U32 init_value;
  U32 readback_value;
  U8 bcount;
  U8 bytes_skipped = 0x00;

  send_info("Checking DM at %#08X (%s)", address, channel_descr);

  *word = 0x00000000;
  init_value = *word;

  for(bcount = 0; bcount < 4; bcount++)
  {
    if(((init_value >> (bcount * 8)) & 0x000000FF) != 0x00)
    {
      send_info("Init problem - skipping byte %d", bcount);
      bytes_skipped |= 0x01 << bcount;
    }
  }

  if(bytes_skipped == 0x0F)
  {
    send_info("All bytes skipped - CS, CKE, CA or DQS?");
    ret = FALSE;
  }
  else
  {
    send_info("Writing 0xAAAAAAAA to %#08X, checking %#08X", word + 1, word);

    *(word + 1) = 0xAAAAAAAA;
    readback_value = *word;

    ret = MTIF_dm_check(TRUE, bytes_skipped, 0x00000000, readback_value);

    send_info("Writing 0xFFFFFFFF to %#08X, checking %#08X", word, word + 1);

    *word = 0xFFFFFFFF;
    readback_value = *(word + 1);

    ret &= MTIF_dm_check(TRUE, bytes_skipped, 0xAAAAAAAA, readback_value);
  }

  if(ret)
  {
    send_info("Done - OK\n");
  }
  else
  { 
    send_info(bytes_skipped == 0x0F ? "No check\n" : "Failed\n");
  }

  return ret;
}

/*------------------------------------------------------------------------------
| Function    : MTIF_dq_check_bits
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN MTIF_dq_check_bits(U32 address, U32 value)
{
  volatile U32 * word = (volatile U32 *)address;
  U32 check;
  U8 bcount;
  U8 failurecount = 0;

  send_info("Writing %#08X to %#08X and checking result", value, word);

  *word = value;
  check = *word;

  for(bcount = 0; bcount < 32; bcount++)
  {
    if(((check >> bcount) & 0x00000001) != ((value >> bcount) & 0x00000001))
    {
      failurecount++;
      send_info("Bit %02d is %d -> DQ%02d", bcount, ((check >> bcount) & 0x00000001), bcount);
    }
  }

  //if(failurecount == 32)
  //{
  //  send_info("All bits failed - CS or CKE?");
  //}

  return (failurecount == 0);
}

/*------------------------------------------------------------------------------
| Function    : MTIF_dq
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN MTIF_dq(U32 address, char * channel_descr)
{
  BOOLEAN ret;

  send_info("Checking DQ at %#08X (%s)", address, channel_descr);

  ret = MTIF_dq_check_bits(address, 0xFFFFFFFF);

  if(!MTIF_dq_check_bits(address, 0x00000000))
  {
    ret = FALSE;
  }

  if(ret)
  {
    send_info("Done - OK\n");
  }
  else
  {
    send_info("Failed\n");
  }

  return ret;
}
#endif // MTIF

#ifdef MEMDUMP
/*------------------------------------------------------------------------------
| Function    : stringize
+------------------------------------------------------------------------------
| Description : Function for making a string out of binary dump data
|
| Parameters  : in - pointer to dump data
|
| Returns     : string of 16 characters, '.' substitutes unprintable stuff
+----------------------------------------------------------------------------*/
char * stringize(U8 * in)
{
  static char out[17];
  int count;
  for(count = 0; count < 16; count++)
  {
    out[count] = ((in[count] > 31) && (in[count] < 127)) ? in[count] : '.';
  }
  out[16] = 0;
  return out;
}
#endif

#ifdef MEMDUMP
void memorydump(U32 start, U32 end, BOOLEAN info)
{
  if(!info)
  {
    DEBUG_LOGF("DUMP %#08X - %#08X", start, end);
  }
  
  if(start)
  {
    do
    {
      if(info)
      {
        send_info("%08X: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X   %s",
                  start, 
                  *(U8 *)(start), *(U8 *)(start + 1), *(U8 *)(start + 2), *(U8 *)(start + 3), *(U8 *)(start + 4), *(U8 *)(start + 5), *(U8 *)(start + 6), *(U8 *)(start + 7),
                  *(U8 *)(start + 8), *(U8 *)(start + 9), *(U8 *)(start + 10), *(U8 *)(start + 11), *(U8 *)(start + 12), *(U8 *)(start + 13), *(U8 *)(start + 14), *(U8 *)(start + 15),
                  stringize((U8 *)start));
      }
      else
      {
        DEBUG_LOGF("%08X: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X   %s",
                   start, 
                   *(U8 *)(start), *(U8 *)(start + 1), *(U8 *)(start + 2), *(U8 *)(start + 3), *(U8 *)(start + 4), *(U8 *)(start + 5), *(U8 *)(start + 6), *(U8 *)(start + 7),
                   *(U8 *)(start + 8), *(U8 *)(start + 9), *(U8 *)(start + 10), *(U8 *)(start + 11), *(U8 *)(start + 12), *(U8 *)(start + 13), *(U8 *)(start + 14), *(U8 *)(start + 15),
                   stringize((U8 *)start));
      }
      start = start + 16;
    } while(start < end);
  }
}
#endif

typedef enum
{
  DEVICE,
  OFFSET,
  WORD,
  VALUE32,
  VALUE64
} T_argument_type;

typedef enum
{
  CMD_UNKNOWN,
  CMD_LIST_COMMANDS,
  CMD_STATUS,
  CMD_GET_LOG,                   
  CMD_POKE32,                     
  CMD_PEEK32,                      
  CMD_PEEKPOKE32,                    
  CMD_LOGSTATS,                  
  CMD_WORDDUMP,                  
  CMD_MEMDUMP,                   
  CMD_MEMFILL,                   
  CMD_HAL_CM_ENABLEMODULECLOCKS, 
  CMD_HAL_CTRL_CONFIGUREPADS,    
  CMD_DRIVER,                    
  CMD_JUMP,                      
  CMD_BRANCH,                    
  CMD_SHOWMEM,                   
  CMD_MTADDR,                    
  CMD_MTMARCHX,                  
  CMD_MTQUICK,                   
  CMD_MTIF,
  CMD_I2CINIT,                   
  CMD_I2CWRITE,                  
  CMD_I2CREAD,
  CMD_POWER_OFF,                   
  CMD_WRITE,                     
  CMD_ERASE,                     
  CMD_READ,                      
  CMD_CHECKSUM,                  
  CMD_RECONNECT,                 
  CMD_RELAX_CHECK,               
  CMD_COLD_SW_RESET,             
  CMD_WARM_SW_RESET
} T_command;

typedef struct  
{
  const char * name;
  U8           type;    // T_argument_type
  U8           options; // T_get_value_option 
} T_argument;

typedef struct  
{
  U8                 command; // T_command    
  U8                 nof_arguments;
  char *             command_tag;
  const T_argument * arguments;
} T_command_definition;

const T_argument arg_address[]   = { { "<address>",       VALUE32,  DEFAULT   }, 
                                     { "<value>",         VALUE32,  NOALIGN   } };
const T_argument arg_modify[]    = { { "<address>",       VALUE32,  DEFAULT   }, 
                                     { "<mask>",          VALUE32,  NOALIGN   }, 
                                     { "<value>",         VALUE32,  NOALIGN   } };
const T_argument arg_range[]     = { { "<start address>", VALUE32,  DEFAULT   }, 
                                     { "<end address>",   VALUE32,  DEFAULT   }, 
                                     { "<pattern>",       VALUE32,  NOALIGN   } };
const T_argument arg_hal[]       = { { "<module>",        VALUE32,  DECIMAL   }, 
                                     { "<instance>",      VALUE32,  DECIMAL   } };
const T_argument arg_i2c_init[]  = { { "<i2c id>",        VALUE32,  NOALIGN   } };
const T_argument arg_i2c_write[] = { { "<mode>",          VALUE32,  NOALIGN   }, 
                                     { "<i2c id>",        VALUE32,  NOALIGN   }, 
                                     { "<slave id>",      VALUE32,  NOALIGN   }, 
                                     { "<register>",      VALUE32,  NOALIGN   }, 
                                     { "<count>",         VALUE32,  NOALIGN   } };
const T_argument arg_i2c_read[]  = { { "<i2c id>",        VALUE32,  NOALIGN   }, 
                                     { "<slave id>",      VALUE32,  NOALIGN   }, 
                                     { "<register>",      VALUE32,  NOALIGN   } };
const T_argument arg_device[]    = { { "<device>",        DEVICE,   DEFAULT   }, 
                                     { "<offset>",        OFFSET,   ALLOW_ODD }, 
                                     { "<size>",          VALUE64,  DEFAULT   } };
const T_argument arg_driver[]    = { { "<device>",        WORD,     DEFAULT   } };

const T_command_definition commands[] =
{
  #ifdef COMMAND_LIST
  { CMD_LIST_COMMANDS,              0, "list_commands",              NULL           },
  #endif
  { CMD_GET_LOG,                    0, "get_log",                    NULL           },
  { CMD_STATUS,                     0, "status",                     NULL           },
  { CMD_POKE32,                     2, "poke32",                     arg_address    },
  { CMD_PEEK32,                     1, "peek32",                     arg_address    },
  { CMD_PEEKPOKE32,                 3, "peekpoke32",                 arg_modify     },
  { CMD_SHOWMEM,                    0, "showmem",                    NULL           },
  #ifdef LOG_STATISTICS
  { CMD_LOGSTATS,                   0, "logstats",                   NULL           },
  #endif
  #ifdef MEMDUMP
  { CMD_WORDDUMP,                   2, "worddump",                   arg_range      },
  { CMD_MEMDUMP,                    2, "memdump",                    arg_range      },
  { CMD_MEMFILL,                    3, "memfill",                    arg_range      },
  #endif
  #ifdef HALFCT
  { CMD_HAL_CM_ENABLEMODULECLOCKS,  2, "hal_cm_enablemoduleclocks",  arg_hal        },
  { CMD_HAL_CTRL_CONFIGUREPADS,     2, "hal_ctrl_configurepads",     arg_hal        },
  #endif
  #ifdef MTADDR
  { CMD_MTADDR,                     2, "mtaddr",                     arg_range      },
  #endif
  #ifdef MTMARCHX
  { CMD_MTMARCHX,                   2, "mtmarchx",                   arg_range      },
  #endif
  #ifdef MTQUICK
  { CMD_MTQUICK,                    2, "mtquick",                    arg_range      },
  #endif
  #ifdef MTIF
  { CMD_MTIF,                       0, "mtif",                       NULL           },
  #endif
  #ifdef I2CSUPPORT
  { CMD_I2CINIT,                    1, "i2cinit",                    arg_i2c_init   },
  { CMD_I2CWRITE,                   5, "i2cwrite",                   arg_i2c_write  },
  { CMD_I2CREAD,                    3, "i2cread",                    arg_i2c_read   },
  { CMD_POWER_OFF,                  0, "power_off",                  NULL           },
  #endif
  #ifdef RECONNECT_HANDLER
  { CMD_RECONNECT,                  0, "reconnect",                  NULL           },
  #endif
  #ifdef DOWNLOAD
  { CMD_DRIVER,                     1, "driver",                     arg_driver     },
  { CMD_JUMP,                       1, "jump",                       arg_address    },
  { CMD_BRANCH,                     2, "branch",                     arg_device     },
  { CMD_WRITE,                      3, "write",                      arg_device     },
  { CMD_ERASE,                      3, "erase",                      arg_device     },
  { CMD_READ,                       3, "read",                       arg_device     },
  { CMD_CHECKSUM,                   3, "checksum",                   arg_device     },
  { CMD_RELAX_CHECK,                0, "relax_check",                NULL           },
  { CMD_COLD_SW_RESET,              0, "cold_sw_reset",              NULL           },
  { CMD_WARM_SW_RESET,              0, "warm_sw_reset",              NULL           }
  #endif
};

typedef struct  
{
  T_command command;
  U16       nof_arguments;
  union
  {
    U32           val32;
    U64           val64;
    T_device_type dev;
    char *        str;
    VU32 *        adr;
  } arguments[5];
} T_received_command;

/*------------------------------------------------------------------------------
| Function    : list_commands
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
#ifdef COMMAND_LIST
void list_commands(void)
{
  int i;

  send_info("Commands Supported:\n");
  
  for(i = 0; i < (sizeof(commands) / sizeof(T_command_definition)); i++)
  {
    send_info("- %s %s %s %s %s %s", 
              commands[i].command_tag,
              commands[i].nof_arguments > 0 ? commands[i].arguments[0].name : "",
              commands[i].nof_arguments > 1 ? commands[i].arguments[1].name : "",
              commands[i].nof_arguments > 2 ? commands[i].arguments[2].name : "",
              commands[i].nof_arguments > 3 ? commands[i].arguments[3].name : "",
              commands[i].nof_arguments > 4 ? commands[i].arguments[4].name : "",
              commands[i].nof_arguments > 5 ? commands[i].arguments[5].name : "");
  }
}
#endif

/*------------------------------------------------------------------------------
| Function    : dnld_main_handler
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void dnld_main_handler(char *cmdline)
{
    static int call_count = 1;
    char *text = cmdline;
    int i, j;
    T_received_command cmd;

    debug_reg_set(debug_reg_id_cmd);

    DEBUG_LOGF_CMD("\n[%03d] %s", call_count++, cmdline);

    cmd.command = CMD_UNKNOWN;

    for(i = 0; i < (sizeof(commands) / sizeof(T_command_definition)); i++)
    {
      if(start_with(&text, commands[i].command_tag))
      {
        U8 device_index = 0xFF;
        cmd.command = (T_command)commands[i].command;
        cmd.nof_arguments = commands[i].nof_arguments;

        // DEBUG_LOGF("Command: (%02d) %s", cmd.command, commands[i].command_tag);

        for(j = 0; j < cmd.nof_arguments; j++)
        {
          switch(commands[i].arguments[j].type)
          {
            case DEVICE:
              cmd.arguments[j].dev = get_device(&text);
              device_index = j;

              if(cmd.arguments[j].dev == IS_UNKNOWN)
              {
                // DEBUG_LOGF_ERROR("- arg%02d: %d (device type)", j, cmd.arguments[j].dev);
                debug_reg_clear(debug_reg_id_cmd);
                return;
              }
              // DEBUG_LOGF("- arg%02d: %d (device type)", j, cmd.arguments[j].dev);
              break;
            
            case OFFSET:
              cmd.arguments[j].val64 = get_offset(&text, cmd.arguments[device_index].dev, ALLOW_ODD);

              if(cmd.arguments[j].val64 == (U64)-1LL)
              {
                // DEBUG_LOGF_ERROR("- arg%02d: %#08X (offset)", j, cmd.arguments[j].val64);
                debug_reg_clear(debug_reg_id_cmd);
                return;
              }
              // DEBUG_LOGF("- arg%02d: %#08X (offset)", j, cmd.arguments[j].val64);
              break;
            
            case WORD:
              cmd.arguments[j].str = get_word(&text);
              if(cmd.arguments[j].str == NULL)
              {
                // DEBUG_LOGF_ERROR("- arg%02d: %s (word/device)", j, cmd.arguments[j].str);
                send_fail("Argument %s not found", commands[i].arguments[j].name);
                debug_reg_clear(debug_reg_id_cmd);
                return;
              }
              // DEBUG_LOGF("- arg%02d: %s (word/device)", j, cmd.arguments[j].str);
              break;
            
            case VALUE32:
            case VALUE64:
              {
                U64 v;
                
                if(!get_value_ex(&text, (char *)commands[i].arguments[j].name, &v, (T_get_value_option)(commands[i].arguments[j].options)))
                {
                  // DEBUG_LOGF_ERROR("- arg%02d: ??? (value)", j);
                  debug_reg_clear(debug_reg_id_cmd);
                  return;
                }
                
                if(commands[i].arguments[j].type == VALUE32)
                {
                  cmd.arguments[j].val32 = (U32)v;
                  // DEBUG_LOGF("- arg%02d: %#08X (value)", j, cmd.arguments[j].val32);
                }
                else
                {
                  cmd.arguments[j].val64 = v;
                  // DEBUG_LOGF("- arg%02d: %#08X (value)", j, cmd.arguments[j].val64);
                }
              }
              break;
          }
        }
        break;
      }
    }

    switch(cmd.command)
    {
      case CMD_UNKNOWN:
        DEBUG_LOGF_ERROR("Unknown");
        send_fail("Unknown command %s", get_word(&text));
        break;
      
      #ifdef COMMAND_LIST
      case CMD_LIST_COMMANDS:
        list_commands();
        send_ok(NULL);
        break;
      #endif

      case CMD_STATUS:
        send_ok("Ready");
        break;

      case CMD_GET_LOG:
        send_log();
        send_ok(NULL);
        break;

      case CMD_POKE32:
        *cmd.arguments[0].adr = cmd.arguments[1].val32;
        send_ok(NULL);
        break;

      case CMD_PEEK32:
        send_info("value at %#08X is %#08X", cmd.arguments[0].val32, *(VU32*)cmd.arguments[0].val32);
        send_ok("");
        break;

      case CMD_PEEKPOKE32:
        {
          U32 value = *cmd.arguments[0].adr;
          *cmd.arguments[0].adr = (value & cmd.arguments[1].val32) | cmd.arguments[2].val32;
          send_info("value at %#08X changed from %#08X to %#08X", cmd.arguments[0].adr, value, *cmd.arguments[0].adr);
          send_ok("");
        }
        break;

      #ifdef LOG_STATISTICS
      case CMD_LOGSTATS:
        log_stats = !log_stats;
        send_ok("");
        break;
      #endif 

      #ifdef MEMDUMP
      case CMD_WORDDUMP:
        if(cmd.arguments[1].adr >= cmd.arguments[0].adr)
        {
          do
          {
            send_info("%08X: %08X %08X %08X %08X", cmd.arguments[0].adr, cmd.arguments[0].adr[0], cmd.arguments[0].adr[1], cmd.arguments[0].adr[2], cmd.arguments[0].adr[3]);
            cmd.arguments[0].adr += 4;
          } while(cmd.arguments[0].adr < cmd.arguments[1].adr);
          send_ok("");
        }
        else
        {
          send_fail("Invalid address range");
        }
        break;
      #endif

      #ifdef MEMDUMP
      case CMD_MEMDUMP:
        if(cmd.arguments[1].adr >= cmd.arguments[0].adr)
        {
          memorydump(cmd.arguments[0].val32, cmd.arguments[1].val32, TRUE);
          send_ok("");
        }
        else
        {
          send_fail("Invalid address range");
        }
        break;
      #endif

      #ifdef MEMDUMP
      case CMD_MEMFILL:
        if(cmd.arguments[1].adr >= cmd.arguments[0].adr)
        {
          VU32 * current;
          for(current = cmd.arguments[0].adr; current <= cmd.arguments[1].adr; current++)
          {
            if((((current - cmd.arguments[0].adr) & 0x3FF) == 0))
            {
              send_status("Progress", current - cmd.arguments[0].adr, cmd.arguments[1].adr - cmd.arguments[0].adr);
            }
            *current = cmd.arguments[2].val32;
          }
          send_status("Progress", cmd.arguments[1].adr - cmd.arguments[0].adr, cmd.arguments[1].adr - cmd.arguments[0].adr);
          send_ok("");
        }
        else
        {
          send_fail("Invalid address range");
        }
        break;
      #endif

      #ifdef HALFCT
      case CMD_HAL_CM_ENABLEMODULECLOCKS:
        {
          STATUS status = (STATUS)HAL_CM_EnableModuleClocks((HAL_Module_e)cmd.arguments[0].val32, cmd.arguments[1].val32);
          if (status)
          {
            send_fail("status = %d", status);
          }
          else
          {
            send_ok(NULL);
          }
        }
        break;
      #endif

      #ifdef HALFCT
      case CMD_HAL_CTRL_CONFIGUREPADS:
        {
          STATUS status = (STATUS)HAL_CTRL_ConfigurePads((HAL_Module_e)cmd.arguments[0].val32, cmd.arguments[1].val32); 
          if (status)
          {
            send_fail("Status = %d", status);
          }
          else
          {
            send_ok(NULL);
          }
        }
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_DRIVER:
        ll_init(TRUE);
        driver_dnld_handler(cmd.arguments[0].str, text);
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_JUMP:
        jump_handler(cmd.arguments[0].val32);
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_BRANCH:
        branch_handler(cmd.arguments[0].dev, cmd.arguments[1].val32);
        break;
      #endif

      case CMD_SHOWMEM:
        mem_show("Memory");
        send_ok("Heap listed");
        break;

      #ifdef MTADDR
      case CMD_MTADDR:
        {
          BOOLEAN ret = TRUE;

          if(!mt_loop_ascending("Filling range", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_fill_with_address_ascending))
          {
            ret = FALSE;
          }

          if(!mt_loop_ascending("Checking range", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_check_for_address_ascending))
          {
            ret = FALSE;
          }

          if(ret == TRUE)
          {
            send_info("No problems found.");
            send_ok("Test completed");
          }
          else
          {
            send_fail("Failed");
          }
        }
        break;
      #endif

      #ifdef MTMARCHX
      case CMD_MTMARCHX:
        {
          BOOLEAN ret = TRUE;
          if(!mt_loop_ascending("Clearing range", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_clear_all_ascending))
          {
            ret = FALSE;
          }

          if(!mt_loop_ascending("Checking bits (r0, w1)", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_marchx_check_cleared_and_set_ascending))
          {
            ret = FALSE;
          }

          if(!mt_loop_descending("Checking bits (r1, w0)", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_marchx_check_set_and_clear_descending))
          {
            ret = FALSE;
          }

          if(!mt_loop_ascending("Checking bits (r0)", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_marchx_check_cleared_ascending))
          {
            ret = FALSE;
          }

          if(ret == TRUE)
          {
            send_info("No problems found.");
            send_ok("Test completed");
          }
          else
          {
            send_fail("Failed");
          }
        }
        break;
      #endif

      #ifdef MTQUICK
      case CMD_MTQUICK:
        {
          BOOLEAN ret = TRUE;

          if(!mt_loop_ascending("Clearing range", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_clear_all_ascending))
          {
            ret = FALSE;
          }

          if(!mt_loop_ascending("Performing quick bit pattern test", cmd.arguments[0].val32, cmd.arguments[1].val32, mt_action_quick_test))
          {
            ret = FALSE;
          }

          if(ret == TRUE)
          {
            send_info("No problems found.");
            send_ok("Test completed");
          }
          else
          {
            send_fail("Failed");
          }
        }
        break;
      #endif

      #ifdef MTIF
      case CMD_MTIF:
        {
          BOOLEAN ret = TRUE;

          if(!MTIF_init())
          {
            ret = FALSE;
          }
          else
          {
            if(!MTIF_cs_cke_ca_dqs(0x80000000, 1, "CH A (CS0)"))
            {
              ret = FALSE;
            }
            else
            {
              if(cs_count > 1)
              {
                if(!MTIF_cs_cke_ca_dqs(0x80000000 + 2 * cs_memory_span, 1, "CH A (CS1)"))
                {
                  ret = FALSE;
                }
              }
            }

            if(!MTIF_cs_cke_ca_dqs(0x80000080, 2, "CH B (CS0)"))
            {
              ret = FALSE;
            }
            else
            {
              if(cs_count > 1)
              {
                if(!MTIF_cs_cke_ca_dqs(0x80000080 + 2 * cs_memory_span, 1, "CH B (CS1)"))
                {
                  ret = FALSE;
                }
              }
            }

            if(!MTIF_dm(0x80000000, "CH A"))
            {
              ret = FALSE;
            }

            if(!MTIF_dm(0x80000080, "CH B"))
            {
              ret = FALSE;
            }

            if(!MTIF_dq(0x80000000, "CH A"))
            {
              ret = FALSE;
            }

            if(!MTIF_dq(0x80000080, "CH B"))
            {
              ret = FALSE;
            }
          }

          if(ret == TRUE)
          {
            send_info("No problems found.");
            send_ok("Test completed");
          }
          else
          {
            send_fail("Failed");
          }
        }
        break;
      #endif

      #ifdef I2CSUPPORT // Only works for OMAP4/5
      case CMD_I2CINIT:
        send_info("Init I2C%d", cmd.arguments[0].val32);
        if(HAL_CTRL_ConfigurePads(HAL_MODULE_I2C, cmd.arguments[0].val32) != NO_ERROR)
        {
          send_fail("pads");
        }
        else if (HAL_CM_EnableModuleClocks(HAL_MODULE_I2C, cmd.arguments[0].val32) != NO_ERROR)
        {
          send_fail("clocks");
        }
        else if (IRQ_Initialize() != NO_ERROR)
        {
          send_fail("IRQ");
        }
        else if (HAL_I2C_Initialize(cmd.arguments[0].val32) != NO_ERROR)
        {
          send_fail("Init");
        }
        else
        {
          send_ok("i2cinit");
        }
        break;
      #endif
      
      #ifdef I2CSUPPORT
      case CMD_POWER_OFF:
        if(HAL_CTRL_ConfigurePads(HAL_MODULE_I2C, 0x00) == NO_ERROR)
        {
          if (HAL_CM_EnableModuleClocks(HAL_MODULE_I2C, 0x00) == NO_ERROR)
          {
            if (IRQ_Initialize() == NO_ERROR)
            {
              U8 count = 100;
              
              while(count)
              {
                DEBUG_LOGF("Attempt %d", count--);
                if(HAL_I2C_Initialize(0x00) != NO_ERROR)
                {
                  DEBUG_LOGF_ERROR("Could not init");
                }
                else
                {
                  VU32 *  clk32      = CLK32K_COUNTER_REGISTER;
                  U8 address_value[2] = {0x25, 0x07};
                  if(HAL_I2C_Write(0x00, 0x48, 2, address_value, *clk32, 0xFF) != NO_ERROR)
                  {
                    DEBUG_LOGF_ERROR("Could not write");
                  }
                  else
                  {
                    DEBUG_LOGF("Write completed");
                  }
                }
                dl_lazy_delay(50000);
              }
            }
          }
        }
        send_fail("power_off");        
        break;
      #endif

      #ifdef I2CSUPPORT // Only works for OMAP4/5
      case CMD_I2CWRITE:
        {
          VU32 *  clk32      = CLK32K_COUNTER_REGISTER;
          U32     result     = 0;
          U8      cmd_length = 0;
          U8      cmd_content[34];

          cmd_content[cmd_length++] = (U8)(cmd.arguments[3].val32 & 0xFF);
          /* In two-byte register address mode (2) the register address is a U16, in which case the most significant byte must be extracted as well (reverse
             byte order or little endian) */
          if(cmd.arguments[0].val32 == 2) cmd_content[cmd_length++] = (U8)((cmd.arguments[3].val32 >> 8) & 0xFF);

          if(cmd.arguments[4].val32 <= 34)
          {
            while(cmd.arguments[4].val32)
            {
              cmd_content[cmd_length++] = (U8)(get_value(&text, "VALUE", NOALIGN) & 0xFF);
              cmd.arguments[4].val32--;
            }

            result = HAL_I2C_Write(cmd.arguments[1].val32, cmd.arguments[2].val32, cmd_length, cmd_content, *clk32, 0xFF);

            if(result != NO_ERROR)
            {
              HAL_I2C_Initialize(cmd.arguments[1].val32);
              send_fail("Error %d", result);
            }
            else
            {
              send_ok("");
            }
          }
          else
          {
            send_fail("Too many values");
          } 
        }
        break;
      #endif

      #ifdef I2CSUPPORT // Only works for OMAP4/5
      case CMD_I2CREAD:
        {
          VU32 * clk32  = CLK32K_COUNTER_REGISTER;
          U8      tmp   = (cmd.arguments[2].val32 & 0xFF);
          U32 result    = HAL_I2C_Write(cmd.arguments[0].val32, cmd.arguments[1].val32, 1, &tmp, *clk32, 0xFF);

          if(result == NO_ERROR)
          {
            result = HAL_I2C_Read(cmd.arguments[0].val32, cmd.arguments[1].val32, 1, &tmp, *clk32, 0xFF);
          }

          if(result != NO_ERROR)
          {
            HAL_I2C_Initialize(cmd.arguments[0].val32);
            send_fail("Error %d", result);
          }
          else
          {
            send_info("Value: %#02x", tmp);
            send_ok("");
          }
        }
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_WRITE:
        ll_init(TRUE);
        write_handler(cmd.arguments[0].dev, cmd.arguments[1].val64, cmd.arguments[2].val64);
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_ERASE:
        ll_init(TRUE);
        erase_handler(cmd.arguments[0].dev, cmd.arguments[1].val64, cmd.arguments[2].val64);
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_READ:
        ll_init(TRUE);
        read_handler(cmd.arguments[0].dev, cmd.arguments[1].val64, cmd.arguments[2].val64);
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_CHECKSUM:
        ll_init(TRUE);
        checksum_handler(cmd.arguments[0].dev, cmd.arguments[1].val64, cmd.arguments[2].val64);
        break;
      #endif

      #ifdef RECONNECT_HANDLER
      case CMD_RECONNECT:
        reconnect_handler();
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_RELAX_CHECK:
        relax_check_handler();
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_COLD_SW_RESET:
        cold_sw_reset_handler();
        break;
      #endif

      #ifdef DOWNLOAD
      case CMD_WARM_SW_RESET:
        warm_sw_reset_handler();
        break;
      #endif
    }

    debug_reg_clear(debug_reg_id_cmd);
}

/*==== END OF FILE ===========================================================*/
