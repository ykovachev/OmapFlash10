/**
 * @file ll_rxtx.c
 * @author Jens Odborg
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
 * This file contains the implementation of the Logical Link (LL) layer RX and TX
 * functionality.
 */

/*==== DECLARATION CONTROL ==================================================*/

#ifndef CSST_LL_RXTX_C
#define CSST_LL_RXTX_C
#endif

/*==== INCLUDES =============================================================*/
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include "types.h"
#include "error.h"
#include "uart_dis.h"
#include "usb_dis.h"
#include "ll.h"
#include "ll_rxtx.h"
#include "romapi.h"

/*==== TYPEDEFINES ==========================================================*/
#if 0
/*==== GLOBALS ==============================================================*/

#define RX_DATA_SIZE 64

/*==== PUBLIC FUNCTIONS =====================================================*/

T_LL_MESSAGE tx_message = {0, 0};

/*==== PUBLIC FUNCTIONS =====================================================*/
/*------------------------------------------------------------------------------
| Function    : ll_rx_callback
+------------------------------------------------------------------------------
| Description : LL RX callback function triggered by receiver interrupt (UART 
|               or USB)
|
| Parameters  : None
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ll_rx_callback(void)
{
  U8 *rx_buf = 0;
  U32 rx_len = 0;
  U32 result = connect_type.read_fptr(ll.drv_hndl, UART_USER_DATA, &rx_len , (U8*)&rx_buf);

  rx_fifo.data[rx_fifo.head].data = rx_buf;
  rx_fifo.data[rx_fifo.head].size = rx_len;

  switch(result)
  {
    case ROMAPI_USB_SUCCESS:
      rx_fifo.data[rx_fifo.head].tag = ll_recv_result_normal;
      break;
    case ROMAPI_USB_DOWNLOAD_MORE:
      rx_fifo.data[rx_fifo.head].tag = ll_recv_result_download_more;
      break;
    case ROMAPI_USB_DOWNLOAD_LAST:
      rx_fifo.data[rx_fifo.head].tag = ll_recv_result_download_last;
      break;
    default:
      while (result != OMAPFLASH_SUCCESS); //TODO
      break;
  }

  rx_fifo.head = LL_BUFFER_MODULO(rx_fifo.head + 1);
}

/*------------------------------------------------------------------------------
| Function    : ll_tx_callback
+------------------------------------------------------------------------------
| Description : LL TX callback function triggered by transmitter interrupt (UART 
|               or USB)
|
| Parameters  : None
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ll_tx_callback(void)
{
	if(tx_fifo.head != tx_fifo.tail)
	{
		T_LL_MESSAGE *tx_message = &tx_fifo.data[tx_fifo.tail++];
		tx_fifo.tail = LL_BUFFER_MODULO(tx_fifo.tail);
		connect_type.write_fptr(ll.drv_hndl, tx_message->tag, &tx_message->size, tx_message->data);
		tx_message->data = 0;
		tx_message->size = 0;
	}
	else
	{
    connect_type.write_fptr(ll.drv_hndl, UART_STOP_XMIT_KEY, NULL, NULL);
	}
}

/*------------------------------------------------------------------------------
| Function    : ll_lsr_callback
+------------------------------------------------------------------------------
| Description : LL UART LSR callback function triggered by a UART LSR interrupt
|
| Parameters  : Error code for callback context
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ll_lsr_callback(U8 err_code)
{
    ///@todo proper error handling
    return; 
}

#endif

/*==== END OF FILE ==========================================================*/

