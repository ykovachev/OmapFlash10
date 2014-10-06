/*
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
 */

/*==== DECLARATION CONTROL ==================================================*/

#define ROMAPI_USB_C

/*==== INCLUDES =============================================================*/

#include <string.h>
#ifdef DEBUG_UART
#include <stdio.h>
#endif
#include <stdlib.h>
#include "config.h"
#include "types.h"
#include "error.h"
#include "uart.h"
#include "romapi.h"
#include "disp.h"

/*==== CONFIGURATION ===========================================================*/
extern BOOLEAN loop_timeout_ms(U8 id, U32 time);

#ifdef DEBUG_USB
#define LOGF DEBUG_LOGF
#else
#define LOGF NO_DEBUG_LOGF
#endif

#define SLOWDOWN   { loop_timeout_ms(2, 0); while(!loop_timeout_ms(2, 5)); }

typedef enum
{
  rx_state_null,
  rx_state_idle,
  rx_state_pending,
  rx_state_awaiting_header,
  rx_state_awaiting_data
} t_rx_state;

#ifdef DEBUG_USB
const char * const rx_state_names[] =
{
  "null",
  "idle",
  "pending",
  "awaiting header",
  "awaiting data"
};
#endif

typedef enum
{
  rx_event_init,
  rx_event_deinit,
  rx_event_receive,
  rx_event_tx_idle,
  rx_event_header_received,
  rx_event_header_failed,
  rx_event_data_received,
  rx_event_data_failed,
  rx_event_more,
  rx_event_done
} t_rx_event;

#ifdef DEBUG_USB
const char * const rx_events[] =
{
  "init",
  "deinit",
  "receive",
  "tx idle",
  "header received",
  "header failed",
  "data received",
  "data failed",
  "more",
  "done"
};
#endif

typedef enum
{
  tx_state_null,
  tx_state_idle,
  tx_state_pending,
  tx_state_sending,
  tx_state_awaiting_ack
} t_tx_state;

#ifdef DEBUG_USB
const char * const tx_state_names[] = 
{
  "null",
  "idle",
  "pending",
  "sending",
  "awaiting ack"
};
#endif

typedef enum
{
  tx_event_init,
  tx_event_deinit,
  tx_event_transmit,
  tx_event_rx_idle,
  tx_event_expect_ack,
  tx_event_more,
  tx_event_no_more,
  tx_event_ack_received,
  tx_event_done
} t_tx_event;

#ifdef DEBUG_USB
const char * const tx_events[] =
{
  "init",
  "deinit",
  "transmit",
  "rx idle",
  "expect ack",
  "more",
  "no more",
  "ack received",
  "done"
};
#endif

t_rx_state rx_state = rx_state_null;
t_tx_state tx_state = tx_state_null;

t_peripheral peripheral_usb;

t_message_header rx_header, tx_header, ack_header;
t_message *      message = NULL;

static void tx_usb(t_tx_event event);
static void rx_usb(t_rx_event event);
BOOLEAN send_ack(t_ack_type type);

#if defined OMAP4 || defined OMAP5
const SYS_DriverPer_t * romapi_usb_SYS_DriverPer;
#endif

U32 rom_code_read_calls  = 0;
U32 rom_code_write_calls = 0;

/*------------------------------------------------------------------------------
| Function    : peripheral_init_usb
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 peripheral_init_usb(void)
{
  LOGF("peripheral_init_usb");

  #if defined OMAP4
  if(!romapi_usb_SYS_DriverPer)
  {
    SYS_GetDriverPer_2nd_usb(DEVICE_TYPE_USBEXT);
  }
  #endif

  #if defined OMAP5
  if(!romapi_usb_SYS_DriverPer)
  {
    SYS_GetDriverPer_2nd_usb(DEVICE_TYPE_USB);
  }
  #endif

  peripheral_usb.configuration.usb.peripheral_init_options       = 0x000A;
  peripheral_usb.configuration.usb.rom_read_descriptor.pOptions  = &peripheral_usb.configuration.usb.peripheral_init_options;
  peripheral_usb.configuration.usb.rom_read_descriptor.Status    = (STATUS)-1;

  peripheral_usb.configuration.usb.rom_read_descriptor.pCallBackFunction  = NULL;
    
  #if defined OMAP4
  peripheral_usb.configuration.usb.rom_read_descriptor.Device  = DEVICE_TYPE_USBEXT;
  #endif
  
  #if defined OMAP5
  peripheral_usb.configuration.usb.rom_read_descriptor.Device  = DEVICE_TYPE_USB;
  #endif
  
  #ifdef OMAP4
  peripheral_usb.configuration.usb.rom_read_descriptor.Mode    = HAL_TRANSFER_MODE_DMA;
  #endif

  #ifdef OMAP3
  peripheral_usb.configuration.usb.rom_read_descriptor.Device  = DEVICE_TYPE_HS_USB;
  #endif

  #ifdef OMAP5
  peripheral_usb.configuration.usb.rom_read_descriptor.pDeviceData = NULL;
  #endif
  
  // NOTE: FOR OMAP5 IT MAY BE NECESSARY TO CALL CONFIG FOR SETTING POWER OPTIONS!

  #if defined OMAP4 || defined OMAP5
  {
  // Reuse ROM connection
    PER_DeviceDesc_t *oppDeviceDesc;
    STATUS result = SYS_GetDeviceDescPer( &oppDeviceDesc );

    #if defined OMAP5
    peripheral_usb.configuration.usb.rom_read_descriptor.IoConfObj = oppDeviceDesc->IoConfObj;
    #endif
    //romapi_usb_init_options = *oppDeviceDesc->pOptions;
    if(result != NO_ERROR)
    {
      DEBUG_LOGF_ERROR("Failed to get device descriptor");
      return(OMAPFLASH_ERROR);
    } 
    peripheral_usb.configuration.usb.peripheral_init_options = *oppDeviceDesc->pOptions; 
  }
  #endif 
  
  #ifdef OMAP3
  // Open new ROM connection
  if(ROMAPI_USB_Initialize(&peripheral_usb.configuration.usb.rom_read_descriptor) != NO_ERROR)
  {
    return OMAPFLASH_ERROR;
  }
  #endif

  memcpy(&peripheral_usb.configuration.usb.rom_write_descriptor, &peripheral_usb.configuration.usb.rom_read_descriptor, sizeof(PeripheralDesc_t));
  memcpy(&peripheral_usb.configuration.usb.rom_ack_descriptor, &peripheral_usb.configuration.usb.rom_read_descriptor, sizeof(PeripheralDesc_t));

  /* Initialize ack header for TX direction and assign address to descriptor for use */

  ack_header.pattern       = PATTERN_VALUE;
  ack_header.fields.ack    = TRUE;
  ack_header.fields.ctrl   = TRUE;
  ack_header.fields.more   = FALSE;
  ack_header.fields.length = 0;
  peripheral_usb.configuration.usb.rom_ack_descriptor.Address = (U32)&ack_header;
  peripheral_usb.configuration.usb.rom_ack_descriptor.Size  = sizeof(t_message_header);

  /// Set the states correctly
  tx_usb(tx_event_init);
  rx_usb(rx_event_init);

  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : peripheral_deinit_usb
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 peripheral_deinit_usb(void)
{
#ifdef OMAP3
  ROMAPI_USB_Close();
#endif
#if defined OMAP4 || defined OMAP5
  // Note: this is wrong! Should be a structure of a different type! TODO!
  // ROMAPI_USB_Close(&peripheral_usb.configuration.usb.rom_write_descriptor);
#endif
  tx_usb(tx_event_deinit);
  rx_usb(rx_event_deinit);
  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : peripheral_write_usb
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 peripheral_write_usb(void)
{
  tx_usb(tx_event_transmit);
  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : peripheral_read_usb
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 peripheral_read_usb(void)
{
  rx_usb(rx_event_receive);
  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : peripheral_get_descriptor_usb
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 peripheral_get_descriptor_usb(t_peripheral ** peripheral)
{
  LOGF("peripheral_get_descriptor_usb");
  
  memset(&peripheral_usb, 0, sizeof(peripheral_usb));

  /// Set the call functions for the API of the driver
  peripheral_usb.call.init   = peripheral_init_usb;
  peripheral_usb.call.deinit = peripheral_deinit_usb;
  peripheral_usb.call.read   = peripheral_read_usb;
  peripheral_usb.call.write  = peripheral_write_usb;

  *peripheral = &peripheral_usb;

  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : receive_message_length
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
t_rx_event receive_message_header(BOOLEAN start_data)
{
  STATUS result = NO_ERROR;

  if(start_data)
  {
    message = peripheral_usb.callback.read_storage(); 
  }

  peripheral_usb.configuration.usb.rom_read_descriptor.Address           = (U32)&rx_header; 
  peripheral_usb.configuration.usb.rom_read_descriptor.Size              = sizeof(rx_header);
  peripheral_usb.configuration.usb.rom_read_descriptor.Status            = (STATUS)-1; /* initialize to invalid value for debugger inspection */

  //LOGF("USB: Receiving header of size %d", peripheral_usb.configuration.usb.rom_read_descriptor.Size);

  result = (STATUS)ROMAPI_USB_Read(&peripheral_usb.configuration.usb.rom_read_descriptor);
  
  rom_code_read_calls++;

  if(result == NO_ERROR)
  {
    while(peripheral_usb.configuration.usb.rom_read_descriptor.Status == WAITING)
    {
      loop_timeout_ms(2, 0);

      while((peripheral_usb.configuration.usb.rom_read_descriptor.Status == WAITING) && (!loop_timeout_ms(2, 5000)));

      if(peripheral_usb.configuration.usb.rom_read_descriptor.Status == WAITING)
      {
        DEBUG_LOGF("Waited 5 s for USB header read (R:%d W:%d)", rom_code_read_calls, rom_code_write_calls);
      }
    }

    if(peripheral_usb.configuration.usb.rom_read_descriptor.Status != NO_ERROR)
    {
      DEBUG_LOGF_ERROR("USB: Failed to receive message header (%d)", peripheral_usb.configuration.usb.rom_read_descriptor.Status);
      return rx_event_header_failed;
    }
    
    LOGF("USB: Header C%d M%d L%d", rx_header.fields.ctrl, rx_header.fields.more, rx_header.fields.length);

    if(rx_header.fields.ack && rx_header.fields.length) 
    {
      send_ack(header_ack);
    }

    if(start_data && message)
    {
      peripheral_usb.configuration.usb.rom_read_descriptor.Address           = (U32)message->content;
      peripheral_usb.configuration.usb.rom_read_descriptor.Size              = rx_header.fields.length;
      peripheral_usb.configuration.usb.rom_read_descriptor.Status            = (STATUS)-1; /* initialize to invalid value for debugger inspection */

      ROMAPI_USB_Read(&peripheral_usb.configuration.usb.rom_read_descriptor);

      rom_code_read_calls++;
    }

    return rx_event_header_received;
  }
  else
  {
    rx_header.fields.ctrl   = FALSE;
    rx_header.fields.more   = FALSE;
    rx_header.fields.length = 0;
    DEBUG_LOGF_ERROR("USB: Failed to receive message header (%d)", result);
    return rx_event_header_failed;
  }
}

/*------------------------------------------------------------------------------
| Function    : receive_message_ack
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
t_tx_event receive_message_ack(void)
{
  receive_message_header(FALSE);
  if(rx_header.fields.ctrl && rx_header.fields.ack && !rx_header.fields.length) 
  {
    return tx_header.fields.more ? tx_event_more : tx_event_no_more;
  }
  else
  {
    DEBUG_LOGF_ERROR("USB: Expected ACK but received MSG");
    return tx_event_done;
  }
}

/*------------------------------------------------------------------------------
| Function    : receive_message_length
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
t_rx_event receive_message_data(void)
{
  STATUS result = NO_ERROR;

  if(message)
  {
    while(peripheral_usb.configuration.usb.rom_read_descriptor.Status == WAITING)
    {
      loop_timeout_ms(2, 0);

      while((peripheral_usb.configuration.usb.rom_read_descriptor.Status == WAITING) && (!loop_timeout_ms(2, 5000)));

      if(peripheral_usb.configuration.usb.rom_read_descriptor.Status == WAITING)
      {
        #if defined OMAP4_GP || defined OMAP4_HS || defined OMAP4_MT
        DEBUG_LOGF("Waited 5 s for USB data read (R:%d W:%d S:%d M:%d)", rom_code_read_calls, rom_code_write_calls, rx_header.fields.length, USBOTGHS_COUNT0);
        #else
        DEBUG_LOGF("Waited 5 s for USB data read (R:%d W:%d S:%d)", rom_code_read_calls, rom_code_write_calls, rx_header.fields.length);
        #endif
      }
    }
      
    if(peripheral_usb.configuration.usb.rom_read_descriptor.Status != NO_ERROR)
    {
        DEBUG_LOGF_ERROR("USB: Failed to receive message data (%d)", peripheral_usb.configuration.usb.rom_read_descriptor.Status);
        return rx_event_data_failed;
    }

    memcpy(&message->header, &rx_header, sizeof(message->header));

    LOGF("USB: Received %s %s %s %d '%s'", 
          message->header.fields.ctrl ? "CTRL" : "DATA", 
          message->header.fields.ack ? "ACK" : "UNACK", 
          message->header.fields.more ? "MORE" : "NOMORE", 
          message->header.fields.length, 
          message->header.fields.ctrl ? message->content : "");
    
    if(rx_header.fields.ack && rx_header.fields.length) 
    {
      send_ack(data_ack);
    }

    return rx_event_data_received;
  }
  else
  {
    DEBUG_LOGF_ERROR("USB no storage");
    return rx_event_data_failed;
  }
}


/*------------------------------------------------------------------------------
| Function    : check_more_data
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
t_rx_event check_more_data(void)
{
  return rx_header.fields.more ? rx_event_more : rx_event_done;
}


/*------------------------------------------------------------------------------
| Function    : receive_message_data
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void deliver_message_data(void)
{
  peripheral_usb.callback.read_deliver();
  message = NULL;
}


/// State machines

/*------------------------------------------------------------------------------
| Function    : rx_usb
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void rx_usb(t_rx_event event)
{
  do
  {
    LOGF("USB RX: %s - %s", rx_state_names[rx_state], rx_events[event]);

    switch(rx_state)
    {
      case rx_state_null:
        switch(event)
        {
          case rx_event_init:
            rx_state = rx_state_idle;
            event = rx_event_done;
            break;
        }
        break;

      case rx_state_idle:
        switch(event)
        {
          case rx_event_receive:
            if(tx_state == tx_state_idle)
            {
              rx_state = rx_state_awaiting_header;
              event = receive_message_header(TRUE);
            }
            else
            {
              rx_state = rx_state_pending;
              event = rx_event_done;
            }
            break;

          case rx_event_tx_idle:
            event = rx_event_done;
            break;
        }
        break;

      case rx_state_pending:
        switch(event)
        {
          case rx_event_more:
          case rx_event_tx_idle:
            rx_state = rx_state_awaiting_header;
            event = receive_message_header(TRUE);
            break;

          case rx_event_receive:
            event = rx_event_done;
            break;
        }
        break;

      case rx_state_awaiting_header:
        switch(event)
        {
          case rx_event_header_received:
            rx_state = rx_state_awaiting_data;
            event = receive_message_data();
            break;

          case rx_event_header_failed:
            rx_state = rx_state_idle;
            event = rx_event_done;
            break;
        }
        break;

      case rx_state_awaiting_data:
        switch(event)
        {
          case rx_event_data_received:
            event = check_more_data();
            if(event == rx_event_more) 
            {
              rx_state = rx_state_pending;
            }
            else
            {
              rx_state = rx_state_idle;
              tx_usb(tx_event_rx_idle);
            }
            deliver_message_data();
            break;

          case rx_event_data_failed:
            rx_state = rx_state_idle;
            event = rx_event_done;
            break;
        }
        break;
    }
  } while(event != rx_event_done);
}

/*------------------------------------------------------------------------------
| Function    : send_ack
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
BOOLEAN send_ack(t_ack_type type)
{
  STATUS result;
  peripheral_usb.configuration.usb.rom_ack_descriptor.Status = (STATUS)-1;
  
  ack_header.fields.ack_type = type;

  LOGF("USB: Sending ACK (%d)", type);

  result = (STATUS)ROMAPI_USB_Write(&peripheral_usb.configuration.usb.rom_ack_descriptor);

  rom_code_write_calls++;

  if(result == NO_ERROR)
  {
    while(peripheral_usb.configuration.usb.rom_ack_descriptor.Status == WAITING)
    {
      loop_timeout_ms(2, 0);
      while((peripheral_usb.configuration.usb.rom_ack_descriptor.Status == WAITING) && (!loop_timeout_ms(2, 5000)));

      if(peripheral_usb.configuration.usb.rom_ack_descriptor.Status == WAITING)
      {
        DEBUG_LOGF("Waited 5 s for USB ACK write (R:%d W:%d)", rom_code_read_calls, rom_code_write_calls);
      }
    }
  }

  if(peripheral_usb.configuration.usb.rom_ack_descriptor.Status != NO_ERROR)
  {
    DEBUG_LOGF_ERROR("USB: Failed to send ACK (%d)", peripheral_usb.configuration.usb.rom_ack_descriptor.Status);
  }
  
  return (peripheral_usb.configuration.usb.rom_ack_descriptor.Status == NO_ERROR);
}

/*------------------------------------------------------------------------------
| Function    : send_message
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
t_tx_event send_message(void)
{
  t_message * message = NULL;
  t_tx_event ret = tx_event_done;
  // Get message from link layer using write callback function

  peripheral_usb.callback.write(&message);

  if(message != NULL)
  {
    STATUS result;

    peripheral_usb.configuration.usb.rom_write_descriptor.Address           = (U32)message;
    peripheral_usb.configuration.usb.rom_write_descriptor.Size              = message->header.fields.length + sizeof(t_message_header);
    peripheral_usb.configuration.usb.rom_write_descriptor.Status            = (STATUS)-1;
    
    memcpy(&tx_header, &message->header, sizeof(tx_header));

    LOGF("USB: Sending %s %s %s %d '%s'", 
         message->header.fields.ctrl ? "CTRL" : "DATA", 
         message->header.fields.ack ? "ACK" : "UNACK", 
         message->header.fields.more ? "MORE" : "NOMORE", 
         message->header.fields.length, 
         message->content);
    
    result = (STATUS)ROMAPI_USB_Write(&peripheral_usb.configuration.usb.rom_write_descriptor);

    rom_code_write_calls++;

    if(result == NO_ERROR)
    {
      while(peripheral_usb.configuration.usb.rom_write_descriptor.Status == WAITING)
      {
        loop_timeout_ms(2, 0);
        while((peripheral_usb.configuration.usb.rom_write_descriptor.Status == WAITING) && (!loop_timeout_ms(2, 5000)));

        if(peripheral_usb.configuration.usb.rom_write_descriptor.Status == WAITING)
        {
          #if defined OMAP4_GP || defined OMAP4_HS || defined OMAP4_MT
          DEBUG_LOGF("Waited 5 s for USB write (R:%d W:%d S:%d M:%d)", rom_code_read_calls, rom_code_write_calls, message->header.fields.length, USBOTGHS_COUNT0);
          #else
          DEBUG_LOGF("Waited 5 s for USB write (R:%d W:%d S:%d)", rom_code_read_calls, rom_code_write_calls, message->header.fields.length);
          #endif
          DEBUG_LOGF("--> %s %s %s %d '%s'", 
                     message->header.fields.ctrl ? "CTRL" : "DATA", 
                     message->header.fields.ack ? "ACK" : "UNACK", 
                     message->header.fields.more ? "MORE" : "NOMORE", 
                     message->header.fields.length, 
                     message->content);

          #ifdef OMAP4_USB_DMA_DEBUG
          DEBUG_LOGF("--> ADDR0   = %#08X", USBOTGHS_ADDR0);
          DEBUG_LOGF("--> COUNT   = %#08X", USBOTGHS_COUNT0);
          DEBUG_LOGF("--> INTRDMA = %#08X", USBOTGHS_INTRDMA);
          DEBUG_LOGF("--> CNTL0   = %#08X:", USBOTGHS_CNTL0);
          DEBUG_LOGF("    ENABLE...........%d  DIRECTION.......%d",  USBOTGHS_CNTL_ENABLE,  USBOTGHS_CNTL_DIRECTION); 
          DEBUG_LOGF("    DMAMODE..........%d  INTRENABLE......%d",  USBOTGHS_CNTL_DMA_MODE,  USBOTGHS_CNTL_INTR_ENABLE);
          DEBUG_LOGF("    EPNUM............%d  BUSERROR........%d",  USBOTGHS_CNTL_EP_NUM,  USBOTGHS_CNTL_BUS_ERROR);
          DEBUG_LOGF("    BURSTMODE........%d",  USBOTGHS_CNTL_BURST_MODE);
          DEBUG_LOGF("--> TXCSR1  = %#08X:", USBOTGHS_TXCSR1);
          DEBUG_LOGF("    TXPKTRDY.........%d  FIFONOTEMPTY....%d", USBOTGHS_TXCSR_TXPKTRDY, USBOTGHS_TXCSR_FIFONOTEMPTY);
          DEBUG_LOGF("    UNDR/ERR.........%d  FLSHFIFO........%d", USBOTGHS_TXCSR_ERROR, USBOTGHS_TXCSR_FLUSHFIFO); 
          DEBUG_LOGF("    SNDSTLL/SETUPKT..%d  RXSTLL/SNTSTLL..%d", USBOTGHS_TXCSR_SENDSTALL, USBOTGHS_TXCSR_SENTSTALL); 
          DEBUG_LOGF("    CLRDTOG..........%d  NAKTO/INCMPTX...%d", USBOTGHS_TXCSR_CLRDATATOG, USBOTGHS_TXCSR_INCOMPTX); 
          DEBUG_LOGF("    DTOG.............%d  DTOGWREN........%d", USBOTGHS_TXCSR_DATATOGGLE,USBOTGHS_TXCSR_DATATOGGLEWRENABLE); 
          DEBUG_LOGF("    DMAREQMODE.......%d  FRCDTOG.........%d", USBOTGHS_TXCSR_DMAREQMODE, USBOTGHS_TXCSR_FRCDATATOG); 
          DEBUG_LOGF("    DMAREQEN.........%d  MODE............%d", USBOTGHS_TXCSR_DMAREQENAB, USBOTGHS_TXCSR_MODE); 
          DEBUG_LOGF("    ISO..............%d  AUTOSET.........%d", USBOTGHS_TXCSR_ISO, USBOTGHS_TXCSR_AUTOSET); 
          #endif
        }
      }
      
      if(peripheral_usb.configuration.usb.rom_write_descriptor.Status != NO_ERROR)
      {
          DEBUG_LOGF_ERROR("USB: Failed to send message data (%d)", peripheral_usb.configuration.usb.rom_write_descriptor.Status);
          //return tx_event_data_failed;
      }

      //free(message);

      if(tx_header.fields.ack)
      {
        ret = tx_event_expect_ack;
      }
      else if(tx_header.fields.more)
      {
        ret = tx_event_more;
      }
      else
      {
        ret = tx_event_no_more;
      }
    }
    else
    {
      DEBUG_LOGF("USB: Write failed (%d)", result);
    }
  }
  
  return ret;
}

/*------------------------------------------------------------------------------
| Function    : tx_usb
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void tx_usb(t_tx_event event)
{
  do 
  {
    LOGF("USB TX: %s - %s", tx_state_names[tx_state], tx_events[event]);

    switch(tx_state)
    {
      case tx_state_null:
        switch(event)
        {
          case tx_event_init:
            tx_state = tx_state_idle;
            event = tx_event_done;
            break;
        }
        break;

      case tx_state_idle:
        switch(event)
        {
          case tx_event_transmit:
            if(rx_state == rx_state_idle)
            {
              // TODO START TRANSMISSION
              tx_state = tx_state_sending;
              event = send_message();
            }
            else
            {
              tx_state = tx_state_pending;
              event = tx_event_done;
            }
            break;

          case tx_event_rx_idle:
            event = tx_event_done;
            break;
        }
        break;

      case tx_state_pending:
        switch(event)
        {
          case tx_event_rx_idle:
            tx_state = tx_state_sending;
            event = send_message();
            break;

          case tx_event_transmit:
            event = tx_event_done;
            break;
        }
        break;

      case tx_state_sending:
        switch(event)
        {
          case tx_event_expect_ack:
            tx_state = tx_state_awaiting_ack;
            event = receive_message_ack();
            break;

          case tx_event_more:
            event = send_message();
            break;

          case tx_event_no_more:
            tx_state = tx_state_idle;
            event = tx_event_done;
            rx_usb(rx_event_tx_idle);
            break;
        }
        break;

      case tx_state_awaiting_ack:
        switch(event)
        {
          case tx_event_more:
            tx_state = tx_state_sending;
            event = send_message();
            break;

          case tx_event_no_more:
            tx_state = tx_state_idle;
            event = tx_event_done;
            rx_usb(rx_event_tx_idle);
            break;
        }
        break;
    }
  } while (event != tx_event_done);
}

/*==== END OF FILE ==========================================================*/
