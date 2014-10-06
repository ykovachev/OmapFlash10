/*
 * Copyright (C) 2009-2010 Texas Instruments
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the 
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef CSST_UART_C
#define CSST_UART_C
#endif

#ifdef __TMS470__
/// turn off remark generated for va_start
#pragma diag_suppress 238 //controlling expression is constant
#endif

/*==== INCLUDES =============================================================*/

#include <string.h>
#include <stdio.h>
#include "config.h"
#include "types.h"
#include "error.h"
#include "uart.h"
#include "disp.h"
#include "debug.h"

#if defined DEBUG_UART

/*==== TYPE DECLARATIONS ====================================================*/

typedef struct uprintf_data
{
  U16 uart_no;
  int count;
} uprintf_data;


/*==== GLOBALS ==============================================================*/

static const U32 uart_base_addr[] = 
{
  0,
  UART_MODEM1_BASE,
  UART_MODEM2_BASE,
  UART_MODEM3_BASE,
#if defined OMAP4 || defined OMAP5
  UART_MODEM4_BASE,
#endif
#ifdef OMAP5
  UART_MODEM5_BASE,
  UART_MODEM6_BASE,
#endif
  0
};

#ifdef INTERRUPT
static const U16 uart_irq_no[4] = 
{
  0,
  INT_UART1_IRQ,
  INT_UART2_IRQ,
  INT_UART3_IRQ
};
#endif

static const U16 uart_baudrate[19] = 
{
  0x9C4,
  0x4E2,
  0x271,
  0x138,
  0xD0,
  0x9C,
  0x68,
  0x4E,
  0x34,
  0x1A,
  0x0D,
  0x8,
  0x4,
  0x03,
  0x2,
  0x2,
  0x1,
  0x1,
  0x0
};

U8 debug_uart = UART_NONE;

/*==== PUBLIC FUNCTIONS =====================================================*/
/*------------------------------------------------------------------------------
| Function    : set_debug_uart
+------------------------------------------------------------------------------
| Description : Function for selecting a debug UART 
|
| Parameters  : uart_no - Number of the UART to use for debug
|
| Returns     : void
+----------------------------------------------------------------------------*/
void set_debug_uart(U8 uart_no)
{
  debug_uart = (uart_no <= NUMBER_OF_UARTS) ? uart_no : UART_NONE;

  if((debug_uart != UART_USB_INFO) && (debug_uart != UART_NONE))
  {
    uart_config(debug_uart, UART_115200, UART_DATABITS_8, UART_STOPBITS_1, UART_PARITY_NONE, POLL_MODE);
  }
}

/*-----------------------------------------------------------------------------
| Function    : uart_config
+------------------------------------------------------------------------------
| Description : Configuration and initialization of the UART module.
|
| Parameters  : uart_no  - UART port to be used
|               baudrate - baudrate
|               data     - Number of data bits
|               stop     - Number of stop bits
|               parity   - Number of parity bits
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_config(U16 uart_no, U16 baudrate, U8 data, U8 stop, U8 parity, U8 irq_poll)
{
  U32 b = uart_base_addr[uart_no];

  if(b)
  {
    /* RESET OF UART */
    UART_REG_MDR1(b) = UART_RESET_MODE;

    while (UART_REG_MDR1(b) != UART_RESET_MODE) ;   /* wait for reset */

    /* UART CONFIGURATION */
    UART_REG_LCR(b)  = 0xBF;
    UART_REG_EFR(b) |= (1 << 4);
    UART_REG_LCR(b)  = (data | (stop << 2) | (parity << 3) | 0x80);
    UART_REG_MCR(b)  = (1 << 6);
    UART_REG_TLR(b)  = 0x00;
    UART_REG_FCR(b)  = 0x87;     /* 56 byte RX FIFO and 8 byte TX FIFO */
    UART_REG_DLL(b)  = (uart_baudrate[baudrate] & 0x00FF);
    UART_REG_DLH(b)  = (uart_baudrate[baudrate] & 0xFF00) >> 8;

    UART_REG_LCR(b) &= 0x7F;
    UART_REG_IIR(b);
    UART_REG_IER(b)  = 0x00;

    if (irq_poll)               /*IRQ_MODE is 1 */
    {
      UART_REG_IER(b) = (UART_RX_IRQ_EN | UART_LINE_IRQ_EN);
    }

    UART_REG_MDR1(b) = (baudrate <= UART_230400) ? UART_16X_MODE : UART_13X_MODE;

    return OMAPFLASH_SUCCESS;
  }
  else
  {
    debug_uart = UART_NONE;

    return OMAPFLASH_ERROR;
  }
}

#if 0 
typedef enum 
{
  bin,
  cstr,
  packet
} T_mode;

/*-----------------------------------------------------------------------------
| Function    : uart_read_data()
+------------------------------------------------------------------------------
| Description : Reads data from the UART RHR register.
|
| Parameters  : uart_no - UART port
|               buf     - Pointer to buffer where data must be stored.
|               size    - number of bytes to read and number of bytes
|                         actually read.
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_read_data_ex(U16 uart_no, U8 * buf, U32 * size, T_mode mode)
{
    U32 b = uart_base_addr[uart_no];
    U32 num_bytes_read = 0;

    while ((UART_REG_LSR(b) & UART_DATA_IN_FIFO) && *size)
    {
        char c = *buf++ = UART_REG_RHR(b);
        *size -= 1;
        num_bytes_read++;
        if (mode == cstr && c == 0)
            break;
    }
    *size = num_bytes_read;
    return 0;
}

/*------------------------------------------------------------------------------
| Function    : uart_read_data_cstr
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 uart_read_data_cstr(U16 uart_no, U8 * buf, U32 * size)
{
    return uart_read_data_ex(uart_no, buf, size, cstr);
}

/*------------------------------------------------------------------------------
| Function    : uart_read_data
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 uart_read_data(U16 uart_no, U8 * buf, U32 * size)
{
    return uart_read_data_ex(uart_no, buf, size, bin);
}
#endif

/*-----------------------------------------------------------------------------
| Function    : uart_write_data()
+------------------------------------------------------------------------------
| Description : Writes data to the UART THR register.
|
| Parameters  : uart_no - UART port
|               buf     - Pointer to buffer where data is stored.
|               size    - number of bytes to write and number of bytes
|                         actually written.
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_write_data(U16 uart_no, const U8 * buf, U32 * size)
{
  if(uart_no != UART_NONE)
  {
    U32 b = uart_base_addr[uart_no];
    U32 num = 0;
    U32 count = 0;

    while (*size)
    {
      for (count = 0; (count < 1000) && (UART_REG_SSR(b) & 0x01); count++);

      if(UART_REG_SSR(b) & 0x01)
      {
        // Looks like it will not be possible to send anything
        return -1;
      }

      UART_REG_THR(b) = *buf++;
      *size -= 1;
      num++;
    }
    *size = num;
    return 0;
  }
  else
  {
    return -1;
  }
}

/*------------------------------------------------------------------------------
| Function    : vuprintf_put
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int vuprintf_put(void *data, int c)
{
  uprintf_data *d = (uprintf_data *)data;
  int ret;
  U32 size = 1;
  
  if (d->count >= 1000)
  {
    /*Count limit crossed, not able to transmit */
    return -1;
  }
  
  if (c == vxprintf_put_restart)
  {
      return 0;
  }
  
  ret = (int)uart_write_data(d->uart_no, (const U8*)&c, &size);
  
  if (ret != 0)
  {
      return -1;
  }
  
  d->count++;
  
  return size;
}

/// vprintf like function for sending text over uart (blocks while sending)
/*------------------------------------------------------------------------------
| Function    : vuxprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int vuxprintf(U16 uart_no, int flags, const char *format, va_list arg_ptr)
{
  uprintf_data data;
  int ret = OMAPFLASH_SUCCESS;

  if(debug_uart != UART_NONE)
  {
    data.uart_no = uart_no;
    data.count   = 0;

    ret = vxprintf(vuprintf_put, &data, flags, format, arg_ptr);
  }

  return (ret < 0) ? ret : data.count;
}

/*------------------------------------------------------------------------------
| Function    : vuprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int vuprintf(U16 uart_no, const char *format, va_list arg_ptr)
{
  return vuxprintf(uart_no, VXPRINTF_STANDARD, format, arg_ptr);
}

/// printf like function for sending text over uart (blocks while sending)
/*------------------------------------------------------------------------------
| Function    : uprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
int uprintf(U16 uart_no, const char *format, ...)
{
  va_list arg_ptr;
  int ret = OMAPFLASH_SUCCESS;

  if(debug_uart != UART_NONE)
  {
    va_start(arg_ptr, format);
    ret = vuprintf (uart_no, format, arg_ptr);
    va_end(arg_ptr);
  }

  return ret;
}

/*------------------------------------------------------------------------------
| Function    : debug_vxprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void debug_vxprintf(int flags, const char *format, va_list arg_ptr)
{
  if(debug_uart != UART_NONE)
  {
    if(debug_uart == UART_USB_INFO)
    {
      vxsend_info(flags | XSEND_DEBUG_PREFIX, format, arg_ptr);
    }
    else
    {
      uprintf(debug_uart, "\r\n");
      vuxprintf(debug_uart, flags, format, arg_ptr);
    }
  }
}

/*------------------------------------------------------------------------------
| Function    : debug_vprintf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void debug_vprintf(const char *format, va_list arg_ptr)
{
  if(debug_uart != UART_NONE)
  {
    debug_vxprintf(VXPRINTF_NO_REQUIRE_HASH, format, arg_ptr);
  }
}

/*------------------------------------------------------------------------------
| Function    : debug_printf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void debug_printf(const char *format, ...)
{
  if (debug_uart != (U8)UART_NONE)
  {
    va_list arg_ptr;
    va_start(arg_ptr, format);
    debug_vprintf(format, arg_ptr);
    va_end(arg_ptr);
  }
} 


#if 0
/*-----------------------------------------------------------------------------
| Function    : uart_mask_tx_irq()
+------------------------------------------------------------------------------
| Description : Masks/Un-masks the TX interrupt.
|
| Parameters  : uart_no - UART port
|               mask    - [1] masks the interrupt and [0] unmasks.
|
| Returns     : N/A
+----------------------------------------------------------------------------*/
void uart_mask_tx_irq(U16 uart_no, U8 mask)
{
  U32 b = uart_base_addr[uart_no];

  if (mask)
  {
      UART_REG_IER(b) &= ~UART_TX_IRQ_EN;
  }
  else
  {
      UART_REG_IER(b) |= UART_TX_IRQ_EN;
  }
}

/*-----------------------------------------------------------------------------
| Function    : uart_int_mask_unmask()
+------------------------------------------------------------------------------
| Description : Masks/Un-masks the TX interrupt.
|
| Parameters  : uart_no - UART port
|               mask    - [1] masks the interrupt and [0] unmasks.
|
| Returns     : N/A
+----------------------------------------------------------------------------*/
#ifdef INTERRUPT // and !ROMAPI and !(DEBUG_UART and "debug uart use interrupt")
void uart_int_mask_unmask(U16 uart_no, U8 set_flag)
{
  U32 int_num = (U32) uart_irq_no[uart_no];
  dal_interrupt_mask(int_num, set_flag);
}
#endif //INTERRUPT
#endif

/*------------------------------------------------------------------------------
| Function    : uart_set_attribute
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 uart_config_terminal(char * attribute)
{
  if(debug_uart != UART_NONE)
  {
    uprintf(debug_uart, "\x1b[%s", attribute);
  }
  return 0;
}



#endif //DEBUG_UART


/*==== END OF FILE ==========================================================*/
