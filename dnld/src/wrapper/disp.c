/**
* @file disp.c
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

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "types.h"
#include "mem.h"
#include "error.h"
#include "ll.h"
#include "csst_tgt.h"
#include "adler32.h"
#include "dnld.h"
#include "disp.h"
#include "uart_dis.h"
#include "omapconfig.h"
#include "debug.h"

/*==== COMPILER CONTROL ======================================================*/

#ifdef __TMS470__
/// turn off remark generated for va_start
#pragma diag_suppress 238 //controlling expression is constant
#endif

/*==== CONSTS ================================================================*/

#define MY_ULTOA_BUF_SIZE 11 ///< max is 0xFFFFFFFF or 4294967295 + zero terminator
#define MY_ULLTOA_BUF_SIZE 21 // max is 0xFFFFFFFFFFFFFFFF or 18446744073709551615 + zero terminator

/*==== TYPES =================================================================*/

typedef struct dry_vprintf_data
{
  int current;
} dry_vprintf_data;

typedef struct vsprintf_data
{
  char *buf; ///< where to put
  char *current;
  char *end; ///< chars putted so far
} vsprintf_data;

/*==== LOCALS ================================================================*/

extern void irq_enable(void);

T_LL_CONNECT connection;

/// for debugger inspection
char * vxsend_buffer = NULL;

volatile int delay_counter;

/// global for the sake of debugging
#ifdef SIMULATION
char cmdline[100];
#else
char *cmdline = 0;
#endif

/*==== PRIVATE FUNCTIONS =====================================================*/
/*==== PUBLIC FUNCTIONS ======================================================*/
/*------------------------------------------------------------------------------
| Function    : my_ultoa
+------------------------------------------------------------------------------
| Description : Unsigned long to ASCII
|
| Parameters  : val - value to convert
|               buf - pointer to conversion buffer
|               radix - decimal (10) or hex (16) based conversion
|
| Returns     : pointer to conversion buffer
+----------------------------------------------------------------------------*/
char * my_ultoa(unsigned int val, char *buf, int radix)
{
  char *b = buf + MY_ULTOA_BUF_SIZE;
  *--b = 0;

  do
  {
    int v = val % radix;
    val /= radix;
    if (v < 10)
      *--b = '0' + v;
    else
      *--b = 'A' + v - 10;
  } while (val);

  return b;
}

/*------------------------------------------------------------------------------
| Function    : my_ulltoa
+------------------------------------------------------------------------------
| Description : Conversion function from unsigned long long to ascii
|
| Parameters  : lo - low 32 bits of ull
|               hi - high 32 bits of ull
|               buf - pointer to conversion buffer
|               radix - decimal (10) or hex (16) based conversion
|
| Returns     : void
+----------------------------------------------------------------------------*/
char * my_ulltoa(unsigned int lo, unsigned int hi, char *buf, int radix)
{
  char *b = buf + MY_ULLTOA_BUF_SIZE;
  *--b = 0;

  if (hi)
  {
    if (radix != 16) return 0;
    while (hi || lo)
    {
      int v = lo & 0xF;
      lo = ((hi & 0xF) << 28) | (lo >> 4);
      hi >>= 4;
      if (v < 10)
        *--b = '0' + v;
      else
        *--b = 'A' + v - 10;
    }
  }
  else if (lo) 
  {
    return(my_ultoa(lo, buf, radix));
  }
  else
  {
    *--b = '0';
  }
  return b;
}

/*------------------------------------------------------------------------------
| Function    : vxprintf
+------------------------------------------------------------------------------
| Description : Minimal worker function for printf and friends
|
| Parameters  : put - function to store/print char
|               buf - pointer to data for put
|               flags - see T_xprintf_flags
|               format - printf like format string but only %% %d %x %s modifiers 
|                        supported: '-+#0*.*' but only in listed order (where * is 
|                        '*' or number)
|               arg_ptr - value retrieved using arg_start
|
| Returns     : result
+----------------------------------------------------------------------------*/
int vxprintf(vxprintf_put_t *put, void *buf, int flags, const char *format, va_list arg_ptr)
{
  ///@todo support recursive formats e.g. %v for 'const char *format, va_list arg_ptr' argument pair
  char tmp[MY_ULLTOA_BUF_SIZE + 1]; ///< +1 for ')' in %s (NULL)
  const char *s;
  int len;
  int ret;
  int value;
  int valueHi;
  
  while (*format) 
  {
    if (*format == '%') 
    {
      char minus;
      char padding  = ' ';
      int width     = 0;
      int precision = -1;
      char sign     = 0;
      int hash      = 0;
      int size      = 0;
      
      format++; // eat %
      
      minus = (*format == '-');
      if (minus) format++;
 
      sign = (*format == '+');
      if (sign) format++;
      
      hash = (*format == '#');
      if (hash) format++;

      if (*format == '0')
      {
        padding = '0';
        format++;
      }
      
      if(*format == '@')
      {
        padding = '.';
        format++;
      }

      if (*format == '*')
      {
        format++;
        width = va_arg(arg_ptr, int);
      }
      else
      {
        while (*format >= '0' && *format <= '9') width = width * 10 + *format++ - '0';
      }

      if (*format == '.') 
      {
        format++;
      
        if (*format == '*')
        {
          precision = va_arg(arg_ptr, int);
        }
        else if (*format >= '0' && *format <= '9')
        {
          precision = 0;
          while (*format >= '0' && *format <= '9') precision = precision * 10 + *format++ - '0';
        }
      }

      if (*format =='l') 
      {
        size = 4;
        format++;
        
        if (*format =='l') 
        {
          size = 8;
          format++;
        }
        
        if (*format != 'x' && *format != 'X')
        {
          s = "?ERROR: # l requires x";
          goto error_text;
        }
      }
      
      switch (*format++)
      {
        case 's':
          s = va_arg(arg_ptr, char*);

          if (!s)
          {
            s = "(NULL)";
          }
          else if ((int) s <= 999 && (int) s >= -999)
          {
            /// handle near null pointers
            static const char null_text[] = "(NULL+";
            const int null_text_len = sizeof null_text - 1; ///< -1 for exlude '\0'
            char *t; ///<non const temp for s
            value = (int) s;
            t = my_ultoa(value < 0 ? -value : value, tmp, 10);
            len = strlen(tmp);
            t[len] = ')';
            t[len+1] = 0;
            t -= null_text_len;
            if (value < 0)
              t[null_text_len-1] = '-';
            memcpy(t, null_text, null_text_len);
            s = t;
          }
          
          len = strlen(s);

          //if(len > 25)
          //{
          //  s = my_ultoa(s, tmp, 16);
          //  len = strlen(s);
          //}

          if (precision == -1)
          {
            precision = 9999;
          }
          break;

        case 'X':
        case 'x':
          value = va_arg(arg_ptr, int);
          
          valueHi = (size == 8) ? va_arg(arg_ptr, int) : 0;
          
          s = my_ulltoa(value, valueHi, tmp, 16);
          
          if (hash)
          {
            ret = put(buf,'0');
            if (ret < 0) goto error;
            //don't wory about len here 0x don't count against number of digits
            ret = put(buf,'x');
            if (ret < 0) goto error;
            //don't wory about len here 0x don't count against number of digits
          }
          //else if (!(flags & VXPRINTF_NO_REQUIRE_HASH))
          //{
            // This complain is to ensure consistent output to user and may go away in the future
            //s = "?ERROR: # mandatory";
            //goto error_text;
          //}
          goto number;

        case 'd':
          ret = va_arg(arg_ptr, int);
          if (ret < 0) 
          {
            sign = '-';
            s = my_ultoa (-ret, tmp, 10);
          }
          else 
          {
            s = my_ultoa (ret, tmp, 10);
          }

number:
          if (precision == -1)
          {
            precision = 1;
          }
          
          len = strlen(s);
          
          while (len < precision)
          {
            ret = put(buf, '0');
            if (ret <= 0) goto error;
            len += ret;
          }
          
          if (sign)
          {
            ret = put(buf, sign);
          }
          
          precision = 9999;
          break;

        case '%':
          s = "%";
          break;

        default:
          s = "?ERROR:";
error_text:
          ret = put(buf, vxprintf_put_restart);
          
          if (ret < 0) 
          {
            return ret;
          }

          while (*s)
          {
            ret = put(buf, *s++);
            if (ret < 0) return ret;
          }

          while (*--format != '%'); //go back to the % with error

          s = format;
          format = ""; // make sure we exit after printing this
          break;
      }

      if (!minus && width)
      {
        while (len < width)
        {
          ret = put (buf, padding);
          if (ret <= 0) goto error;
          len += ret;
        }
      }

      while (*s)
      {
        if (!precision) break;
        ret = put (buf, *s);
        s++;
        if (ret <= 0) goto error;
        precision-=ret;
      }

      while (len < width)
      {
        ret = put (buf, padding);
        if (ret <= 0) goto error;
        len += ret;
      }
    }
    else
    {
      ret = put (buf, *format++);
      if (ret < 0) return ret;
    }
  }

  return 0;

error:
  if (ret < 0) return ret;
  return -1;
}

/*------------------------------------------------------------------------------
| Function    : dry_vprintf_put
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int dry_vprintf_put(void *data, int c)
{
  dry_vprintf_data *d = (dry_vprintf_data *)data;

  if (c == vxprintf_put_restart) 
  {
    d->current = 0;
    return 0;
  }
  d->current++;
  return 1;
}

/*------------------------------------------------------------------------------
| Function    : dry_vprintf
+------------------------------------------------------------------------------
| Description : Get the needed number of bytes for the given format / parameter 
|               set
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int dry_vprintf(const char *format, va_list arg_ptr)
{
  dry_vprintf_data data;
  int ret;

  data.current = 0;
  ret = vxprintf (dry_vprintf_put, &data, VXPRINTF_STANDARD, format, arg_ptr);
  
  if (ret < 0)
  {
    return ret;
  }
  else
  {
    return data.current;
  }
}

/*------------------------------------------------------------------------------
| Function    : dry_printf
+------------------------------------------------------------------------------
| Description : Get the needed number of bytes for the given format / parameter 
|               set
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int dry_printf(const char *format, ...)
{
  va_list arg_ptr;
  int ret;

  va_start(arg_ptr, format);
  ret = dry_vprintf(format, arg_ptr);
  va_end(arg_ptr);
  return ret;
}

/** put used by snprintf
* @param data where to put
* @param c char to put
*/
/*------------------------------------------------------------------------------
| Function    : vxsprintf_put
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int vxsprintf_put(void *data, int c)
{
  vsprintf_data *d = (vsprintf_data *)data;

  if (c == vxprintf_put_restart) 
  {
    d->current = d->buf;
    return 0;
  }
  
  if (d->current == d->end) 
  {
    return -1;
  }
  
  *d->current++ = c;
  return 1;
}

/*------------------------------------------------------------------------------
| Function    : vxsnprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int vxsnprintf(char *buf, size_t buf_size, int flags, const char *format, va_list arg_ptr)
{
  vsprintf_data data;
  int ret;
  size_t count;

  data.buf = buf;
  data.current = buf;
  data.end = buf + buf_size;
  ret = vxprintf (vxsprintf_put, &data, flags, format, arg_ptr);
  count = data.current - data.buf;
  
  if (ret < 0)
  {
    return ret;
  }

  if (count < buf_size - 1)
  {
    ret = vxsprintf_put(&data, 0);
    
    if (ret < 0) 
    {
      return ret;
    }
  }
  
  return (count < buf_size) ? (int)count : -1;
}

/*------------------------------------------------------------------------------
| Function    : my_vsnprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int my_vsnprintf(char *buf, size_t buf_size, const char *format, va_list arg_ptr)
{
  return vxsnprintf (buf, buf_size, VXPRINTF_STANDARD, format, arg_ptr);
}

#ifndef SIMULATION
///@deprecated use my_vsnprintf
int vsnprintf (char *buf, size_t buf_size, const char *format, va_list arg_ptr)
{
  return my_vsnprintf (buf, buf_size, format, arg_ptr);
}
#endif //SIMULATION

/*------------------------------------------------------------------------------
| Function    : my_snprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int my_snprintf(char *buf, size_t buf_size, const char *format, ...)
{
  va_list arg_ptr;
  int ret;

  va_start(arg_ptr, format);
  ret = my_vsnprintf (buf, buf_size, format, arg_ptr);
  va_end(arg_ptr);
  return ret;
}

#ifndef SIMULATION
///@deprecated use my_snprintf
/*------------------------------------------------------------------------------
| Function    : sprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int sprintf(char *buf, const char *format, ...);

///@deprecated use my_snprintf
int snprintf (char *buf, size_t buf_size, const char *format, ...)
{
  va_list arg_ptr;
  int ret;

  va_start(arg_ptr, format);
  ret = my_vsnprintf (buf, buf_size, format, arg_ptr);
  va_end(arg_ptr);
  return ret;
}
#endif //SIMULATION

/*------------------------------------------------------------------------------
| Function    : vxsend
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void vxsend(char * prefix, int flags, const char *format, va_list arg_ptr)
{
  char * buffer;
  int n = 0;
  int x;
  int max = LL_CTRL_BLOCK_SIZE - 1;
  
  if(vxsend_buffer == NULL)
  {
    get_config_data_area((unsigned char **)&vxsend_buffer);
  }

  buffer = vxsend_buffer;

  memset(buffer, 0, LL_CTRL_BLOCK_SIZE); ///<We rely on this fill if printf fails to figure out how much was actually printed
  
  if (prefix)
  {
    x = my_snprintf (buffer, max, prefix);
    if (x >= 0)
    {
      n = x;
      if (format && *format)
      {
        x = my_snprintf (buffer+n, max - n, ": ");
        if (x >= 0)
        {
          n += x;
        }
      }
    }
  }
  
  if (x >= 0 && flags & XSEND_DEBUG_PREFIX)
  {
    x = my_snprintf (buffer+n, max - n, "#");
    if (x >= 0)
    {
      n += x;
    }
  }
  
  if (x >= 0 && format)
  {
    x = vxsnprintf (buffer+n, max - n, flags, format, arg_ptr);
    if (x >= 0)
    {
      n += x;
    }
  }
  
  if (x < 0 && buffer[max - 2]) // only take this branch if the error is considering size (else buffer already contain error message)
  {
    static const char too_long_msg[] = "FAIL: msg too long '%s'-";
    int len_fail = strlen(buffer); ///< this is safe because of previous memset
    
    n = dry_printf(too_long_msg,format);
  
    if (n >= 0)
    {
      n += 1; ///< +1 for '\0' that we overwrite with '\''
      if (n + len_fail > max)
        len_fail = max - n;
      memmove (buffer + n, buffer, len_fail); ///< make room for too_long_msg
      n = my_snprintf(buffer, n, too_long_msg, format);
    }
    
    if (n > 0)
    {
      buffer[n] = '\''; ///< overwrite '\0' supplied by my_printf
      n+=len_fail; ///< account for what ever amount from failed printf there are room for
      buffer[n] = 0;
    }
    else
    {
      /// hmm ... dry run and actually run should always give same amount
      strcpy(buffer, "FAIL: err in vxsend");
      n = strlen(buffer);
    }
  }
  
  n++; ///< include zero terminator
  
  while (n % 4 == 0) buffer[n++] = 0; ///< align length (needed by romapi)
  
  if (flags & XSEND_FLAGS_UNACKED)
  {
    while(ll_send_ctrl_unacked((U8*)buffer, n) == LL_ERR_BUFFER_FULL);
  }
  else
  {
    while(ll_send_ctrl_acked((U8*)buffer, n) == LL_ERR_BUFFER_FULL);
  }
}

/*------------------------------------------------------------------------------
| Function    : xsend
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void xsend(char * prefix, int flags, char *format, ...)
{
  va_list arg_ptr;
  va_start(arg_ptr, format);
  vxsend (prefix, flags, format, arg_ptr);
  va_end(arg_ptr);
}

/*------------------------------------------------------------------------------
| Function    : send
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send(char * prefix, char *format, ...)
{
  va_list arg_ptr;
  va_start(arg_ptr, format);
  vxsend (prefix, VXPRINTF_STANDARD | XSEND_FLAGS_ACKED, format, arg_ptr);
  va_end(arg_ptr);
}

/*------------------------------------------------------------------------------
| Function    : send_ok
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_ok(char *format, ...)
{
  va_list arg_ptr;
  va_start(arg_ptr, format);
  vxsend ("OKAY", VXPRINTF_STANDARD, format, arg_ptr);
  va_end(arg_ptr);
}

/*------------------------------------------------------------------------------
| Function    : send_fail
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_fail(char *format, ...)
{
  va_list arg_ptr;
  va_start(arg_ptr, format);
  vxsend ("FAIL", VXPRINTF_STANDARD, format, arg_ptr);
  va_end(arg_ptr);
  DEBUG_LOGF_ERROR("FAIL!");
}

/*------------------------------------------------------------------------------
| Function    : send_bug
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_bug(char *format, ...)
{
  va_list arg_ptr;
  va_start(arg_ptr, format);
  vxsend ("FAIL: Internal error", VXPRINTF_STANDARD, format, arg_ptr);
  va_end(arg_ptr);
  DEBUG_LOGF_ERROR("FAIL!");
}

/*------------------------------------------------------------------------------
| Function    : vxsend_info
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void vxsend_info(int flags, const char *format, va_list arg_ptr)
{
  if (format) 
  {
    vxsend ("INFO", flags, format, arg_ptr);
  }
  else 
  {
    send_bug("Missing INFO parameters");
  }
}

/*------------------------------------------------------------------------------
| Function    : send_info
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_info(char *format, ...)
{
  va_list arg_ptr;
  va_start(arg_ptr, format);
  vxsend_info (VXPRINTF_NO_REQUIRE_HASH, format, arg_ptr);
  va_end(arg_ptr);
}

/*------------------------------------------------------------------------------
| Function    : send_status
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_status(const char * description, U64 current, U64 target)
{
  static U64 last_current = (U64)(-1LL);
  
  if((last_current > current) || ((current - last_current) > (target / 100)) || (current == target))
  {
    last_current = current;
    send(NULL, "STAT %#llx %#llx %s", current, target, description);
  }
}

/*------------------------------------------------------------------------------
| Function    : send_out_of_memory
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_out_of_memory(void)
{
  send_fail("Out of memory");
}

/*------------------------------------------------------------------------------
| Function    : send_upload
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_upload(void *data, U32 size)
{
  U32 checksum = adler32((U8*)data, size);
  send(NULL, "READ%#08X %#08X", size, checksum);
  while(ll_send_data((U8*)data, size)==LL_ERR_BUFFER_FULL);
}

/*------------------------------------------------------------------------------
| Function    : disp_deinit
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void disp_deinit(void)
{
  ll_disconnect(NULL);
  /* Only responses from ll_disconnect are LL_OK and LL_ERR_NOT_CONNECTED. We succeed in either case */
  return;
}

/*------------------------------------------------------------------------------
| Function    : is_uart_connection
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN is_uart_connection()
{
  return connection.link_type == connect_uart;
}

/// list label in debugger, if PC end here see dnld_if value for futher trouble shooting
/*------------------------------------------------------------------------------
| Function    : unknown_connect
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void unknown_connect(void)
{
  for(;;);
}

/*------------------------------------------------------------------------------
| Function    : main
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
#ifdef SIMULATION
void main(int argc, char**argv)
#else
void main(void)
#endif
{
#if defined OMAP4 || defined OMAP5
  connection.reuse_romconnection = TRUE;
#else
  connection.reuse_romconnection = FALSE;
#endif

  switch(download_if())
  {
    case DLIF_UART3:
      connection.uart_no       = 3;
      connection.link_type     = connect_uart;
      connection.uart_baudrate = UART_BAUDRATE_115200;
      break;

    case DLIF_USB_OTG:
      connection.link_type = connect_usb;
      break;
    
    default:
      unknown_connect();
      return;
  }

  /* Note: ROMAPI depend on IRQ being enabled but do *NOT* enable it it self
  * and ROMCODE disable IRQ before starting 2nd
  */
  #if 0
  if(IRQ_Initialize() != NO_ERROR)
  {
    DEBUG_LOGF("Failed to init IRQ");
    while(1);
  }
  else
  {
    DEBUG_LOGF("IRQ initialized");
  }
  #else
  irq_enable();
  #endif

  dl_lazy_delay(100 * ONE_MILLISEC);

  /* Initialize LL */

  ll_init(FALSE);

  if (ll_connect((t_connect)(connection.link_type)) != OMAPFLASH_SUCCESS)
  {
    /*if you see PC here in debugger check out global variables:
    when connecting through usb:
    romapi_usb_init_status
    romapi_usb_init.peripheralDesc.Status
    when connecting through uart:
    romapi_uart_init_status
    romapi_uart_init.peripheralDesc.Status
    */
    DEBUG_LOGF("LL connection failure");

    while(1) 
    {
      debug_reg_toggle(debug_reg_id_ll, 32000, 32000);
    }
  }

  if (connection.link_type == connect_uart)
  {
    /// give host a chance to change parity setting
    /// if this timeout is too low UART connection will fail

    dl_lazy_delay(500 * ONE_MILLISEC);
  }

  DEBUG_LOGF("Reporting to host");
  send_ok  ("2nd started");
  DEBUG_LOGF("Ready");

#ifdef SIMULATION
  {
    int i;
    for (i = 1; i < argc; i++)
    {
      if (cmdline[0])
      {
        strcat(cmdline, " ");
      }
      strcat(cmdline, argv[i]);
    }
    dnld_main_handler(cmdline);
  }
#else
  while(1)
  {
    t_message * msg;
    msg = ll_receive();
    if(msg->header.fields.ctrl)
    {
      dnld_main_handler((char *)(msg->content));
    }
    else
    {
      DEBUG_LOGF("Unexpected data");
    }
  }
#endif
}

/*==== END OF FILE ===========================================================*/
