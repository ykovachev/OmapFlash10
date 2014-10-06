/**
* @file ll.c
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
* Implementation of the Logical Link (LL) API
*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef CSST_LL_API_C
#define CSST_LL_API_C
#endif

/*==== INCLUDES =============================================================*/
#include <stdlib.h>
#ifdef SIMULATION
#include <stdio.h>
#endif
#include "types.h"
#include "mem.h"
#include "error.h"
#ifdef ROMAPI
#include "romapi_uart.h"
#include "romapi_usb.h"
#endif
#include "ll.h"
#include "disp.h"
#include "debug.h"


#ifdef DEBUG_LL
#define LOGF DEBUG_LOGF
#else
#define LOGF NO_DEBUG_LOGF
#endif

extern S32 peripheral_get_descriptor_usb(t_peripheral ** peripheral);

t_connect         ll_connection = connect_unknown;
T_LL_FIFO         rx_fifo;
T_LL_FIFO         tx_fifo;
t_peripheral  *   peripheral;

/*==== PUBLIC FUNCTIONS =====================================================*/

/*------------------------------------------------------------------------------
| Function    : ll_rx_callback
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
//void ll_rx_callback(t_message * message)
//{
//  LOGF("ll_rx_callback - occupancy %d", rx_fifo.occupancy);
//  
//  if(rx_fifo.occupancy < LL_BUFFER_SIZE)
//  {
//    rx_fifo.data[rx_fifo.head] = message;
//    rx_fifo.head               = LL_BUFFER_MODULO(rx_fifo.head + 1);
//    rx_fifo.occupancy++;
//  }
//  else
//  {
//    LOGF("LL RX FIFO OVERFLOW");
//    while(1); // ERROR - OVERFLOW
//  }
//}

/*------------------------------------------------------------------------------
| Function    : ll_rx_callback_get_storage
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
t_message * ll_rx_callback_get_storage(void)
{
  t_message * ret = NULL;

  // LOGF("ll_rx_callback_get_storage - occupancy %d", rx_fifo.occupancy);

  if(rx_fifo.occupancy < rx_fifo.size)
  {
      ret = rx_fifo.data[rx_fifo.head];
  }

  return ret;
}

/*------------------------------------------------------------------------------
| Function    : ll_rx_callback
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ll_rx_callback_deliver(void)
{
  if(rx_fifo.occupancy < rx_fifo.size)
  {
    rx_fifo.head = LL_FIFO_MODULO(rx_fifo, rx_fifo.head + 1);
    rx_fifo.occupancy++;
  }
  else
  {
    DEBUG_LOGF_ERROR("LL RX FIFO OVERFLOW");
    while(rx_fifo.occupancy >= rx_fifo.size); // ERROR - OVERFLOW
  }
}

/*------------------------------------------------------------------------------
| Function    : ll_tx_callback
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ll_tx_callback(t_message ** message)
{
  if(tx_fifo.occupancy > 0)
  {
    //*message                       = tx_fifo.data[tx_fifo.tail];
    //tx_fifo.data[tx_fifo.tail]     = NULL;
    //(*message)->header.fields.more = tx_fifo.occupancy-- > 1;
    //tx_fifo.tail                   = LL_BUFFER_MODULO(tx_fifo.tail + 1);

    *message                       = tx_fifo.data[tx_fifo.tail];
    (*message)->header.fields.more = tx_fifo.occupancy-- > 1;
    tx_fifo.tail                   = LL_FIFO_MODULO(tx_fifo, tx_fifo.tail + 1);
  }
  else
  {
    DEBUG_LOGF_ERROR("LL TX FIFO UNDERFLOW");
    while(tx_fifo.occupancy == 0); // ERROR - UNDERFLOW
  }
}

/*-----------------------------------------------------------------------------
| Function    : ll_init()
+------------------------------------------------------------------------------
| Description : Initialization of the LL module.
|
| Parameters  : abort_cb - Abort Call-back function called every time an SDU
|                          is received by LL.
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
void ll_init(BOOLEAN heap)
{
  static BOOLEAN heap_used = FALSE;
  int count;

  if(!heap_used)
  {
    memset (&rx_fifo, 0x0, sizeof rx_fifo);
    memset (&tx_fifo, 0x0, sizeof tx_fifo);
  }
  
  if(!heap)
  {
    unsigned char * address;
    U32             size;

    LOGF("ll_init - SRAM");

    size = get_config_data_area(&address);

    rx_fifo.size = tx_fifo.size = (U16)LL_FIFO_SIZE_INT(size);

    LOGF("Size memory = %d - Size FIFO = %d", size, rx_fifo.size);

    for(count = 0; count < rx_fifo.size; count++)
    {
      rx_fifo.data[count] = (t_message *)LL_FIFO_ADDRESS_INT(address, count);
      tx_fifo.data[count] = (t_message *)LL_FIFO_ADDRESS_INT(address, count + rx_fifo.size);
      LOGF("Address RX%d = %#08X - Address TX%d = %#08X", count, rx_fifo.data[count], count, tx_fifo.data[count]);
    }
  }
  else if(!heap_used)
  {
    LOGF("ll_init - Heap");

    rx_fifo.size = tx_fifo.size = LL_FIFO_SIZE_HEAP;

    for(count = 0; count < LL_FIFO_SIZE_HEAP; count++)
    {
      rx_fifo.data[count] = (t_message *)mem_alloc(sizeof(t_message_header) + DATA_BLOCK_SIZE_DOWNLOAD + DATA_BLOCK_SIZE_ADJUST);
      tx_fifo.data[count] = (t_message *)mem_alloc(sizeof(t_message_header) + DATA_BLOCK_SIZE_UPLOAD_USB + DATA_BLOCK_SIZE_ADJUST);

      if(rx_fifo.data[count] == NULL)
      {
        DEBUG_LOGF_ERROR("LL mem_alloc error: rx_fifo.data[%d]", count);
      }

      if(tx_fifo.data[count] == NULL)
      {
        DEBUG_LOGF_ERROR("LL mem_alloc error: tx_fifo.data[%d]", count);
      }
    }

    heap_used = TRUE;
  }
}

/*------------------------------------------------------------------------------
| Function    : ll_connect
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 ll_connect(t_connect connect)
{
  S32 ret = OMAPFLASH_ERROR;

  LOGF("ll_connect");

  if (ll_connection != connect_unknown)
  {
    return LL_ERR_ALREADY_CONNECTED;
  }

#if 0
#ifndef DISABLE_UART
  if(connect->link_type == connect_uart)
  {
    peripheral_get_descriptor_uart(peripheral);
  }
#endif
#endif 

  if(connect == connect_usb)
  {
    peripheral_get_descriptor_usb(&peripheral);
    peripheral->callback.read_storage = ll_rx_callback_get_storage;
    peripheral->callback.read_deliver = ll_rx_callback_deliver;
    peripheral->callback.write = ll_tx_callback;
  }

  ret = peripheral->call.init();

  if (ret != OMAPFLASH_SUCCESS)
  {
    DEBUG_LOGF_ERROR("Failed to open peripheral IF");
    return LL_ERR_PORT_OPEN;
  }
  else
  {
    ll_connection = connect;

    if (tx_fifo.head != tx_fifo.tail)
    {
      LOGF("FIFO pre-populated!");
      peripheral->call.write();
    }

    debug_reg_set(debug_reg_id_ll);
    LOGF("Peripheral IF opened");
    return LL_OK;
  }
}

/*------------------------------------------------------------------------------
| Function    : ll_disconnect
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 ll_disconnect(t_connect * was_connect)
{
  S32 ret;

  LOGF("ll_disconnect");

  if(ll_connection == connect_unknown)
  {
    return LL_ERR_NOT_CONNECTED;
  }

  if(was_connect)
  {
    *was_connect = ll_connection;
  }

  ret = peripheral->call.deinit();

  if (ret == OMAPFLASH_SUCCESS)
  {
    ll_connection = connect_unknown;
  }

  debug_reg_clear(debug_reg_id_ll);

  return ret;
}

/*------------------------------------------------------------------------------
| Function    : ll_receive
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
t_message * ll_receive(void)
{
  t_message * ret = NULL;

  if(rx_fifo.occupancy == 0)
  {
    peripheral->call.read();
  }

  while(ret == NULL) // Blocking call
  {
    if(rx_fifo.occupancy > 0)
    {
      LOGF("ll_receive - occupancy %d", rx_fifo.occupancy);

      ret                        = rx_fifo.data[rx_fifo.tail];
      rx_fifo.tail               = LL_FIFO_MODULO(rx_fifo, rx_fifo.tail + 1);
      rx_fifo.occupancy--;
    }
  }

  return ret;
}

/*------------------------------------------------------------------------------
| Function    : ll_send_ex
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 ll_send_ex(U8 * data, U32 size, BOOLEAN ctrl, BOOLEAN ack)
{
  S8 ret = LL_ERR_BUFFER_FULL;

  if(ll_connection != connect_unknown)
  {
    if(tx_fifo.occupancy < tx_fifo.size)
    {
      tx_fifo.data[tx_fifo.head]->header.pattern         = PATTERN_VALUE;
      tx_fifo.data[tx_fifo.head]->header.fields.ctrl     = ctrl;
      tx_fifo.data[tx_fifo.head]->header.fields.ack      = ack;
      tx_fifo.data[tx_fifo.head]->header.fields.ack_type = ack_request;
      tx_fifo.data[tx_fifo.head]->header.fields.more     = FALSE;
      tx_fifo.data[tx_fifo.head]->header.fields.length   = size;

      memcpy(tx_fifo.data[tx_fifo.head]->content, data, size);

      tx_fifo.head = LL_FIFO_MODULO(tx_fifo, tx_fifo.head + 1);
      tx_fifo.occupancy++;

      ret = LL_OK;

      peripheral->call.write();
    }
  }
  else
  {
    ret = LL_OK;
  }
  
  return ret;
}

/*------------------------------------------------------------------------------
| Function    : ll_send
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  :
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 ll_send_ctrl_acked(U8 *data, U32 size)
{
  LOGF("ll_send_ctrl_acked %s", data);
  return ll_send_ex(data, size, TRUE, TRUE);
}

/*------------------------------------------------------------------------------
| Function    : ll_send_unacked
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  :
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 ll_send_ctrl_unacked(U8 *data, U32 size)
{
  LOGF("ll_send_ctrl_unacked %s", data);
  return ll_send_ex(data, size, TRUE, FALSE);
}

/*------------------------------------------------------------------------------
| Function    : ll_send_data
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
S8 ll_send_data(U8 * data, U32 size)
{
  LOGF("ll_send_data %d", size);
  return ll_send_ex(data, size, FALSE, FALSE);
}

/*==== END OF FILE ==========================================================*/

