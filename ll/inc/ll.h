/**
 * @file ll.h
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

/**
 * @file silicon.h
 * @author 
 * @brief API definition for logical link layer
 * @details 
 * @todo file details
 * @see <link>
 * @todo update link
 */

/*==== DECLARATION CONTROL ==================================================*/

#ifndef LL_H
#define LL_H

/*==== INCLUDES =============================================================*/
#include "types.h"
#include "uart.h"
#include "romapi.h"

/*==== MACROS ===============================================================*/

#ifdef FORCE_NON_MODULUS_64_PACKAGE_SIZE
#define DATA_BLOCK_SIZE_ADJUST       4 ///< there is a bug in OMAP3 rom api concerning sending data where (size % 64 == 0)
#else
#define DATA_BLOCK_SIZE_ADJUST       0
#endif

#define DATA_BLOCK_SIZE_DOWNLOAD       (4 * 1024 * 1024 - DATA_BLOCK_SIZE_ADJUST)  
#define DATA_BLOCK_COUNT_DOWNLOAD      2                                    
#define DATA_BLOCK_SIZE_UPLOAD_USB     (256 * 1024 - DATA_BLOCK_SIZE_ADJUST)  
#define DATA_BLOCK_SIZE_UPLOAD_UART    64  
#define DATA_BLOCK_SIZE_CHECKSUM       (16 * 512)

#define LL_CTRL_BLOCK_SIZE             256

#define LL_FIFO_BUFFER_SIZE_HEAP_UP    (sizeof(t_message_header) + DATA_BLOCK_SIZE_DOWNLOAD + DATA_BLOCK_SIZE_ADJUST)
#define LL_FIFO_BUFFER_SIZE_HEAP_DOWN  (sizeof(t_message_header) + DATA_BLOCK_SIZE_UPLOAD_USB + DATA_BLOCK_SIZE_ADJUST)
#define LL_FIFO_BUFFER_SIZE_INT_UP     (sizeof(t_message_header) + LL_CTRL_BLOCK_SIZE)
#define LL_FIFO_BUFFER_SIZE_INT_DOWN   (sizeof(t_message_header) + LL_CTRL_BLOCK_SIZE)
#define LL_FIFO_SIZE_HEAP              3 // Must be larger than FIFO size for internal allocation

// Internal LL buffer organization is as follows (until heap allocation is applied):
// [INT_BLOCK_SIZE vxsend buffer]
// [LL_FIFO_SIZE_INT LL FIFO BUFFER 0]
// [LL_FIFO_SIZE_INT LL FIFO BUFFER 1]
// ...
// [LL_FIFO_SIZE_INT LL FIFO BUFFER N]
// The LL internal buffering reuses the configuration data area in SRAM...

#define LL_FIFO_SIZE_INT(is)           ((is - LL_CTRL_BLOCK_SIZE) / (LL_FIFO_BUFFER_SIZE_INT_UP + LL_FIFO_BUFFER_SIZE_INT_DOWN))
#define LL_FIFO_ADDRESS_INT(r, i)      ((U32)r + ((i + 1) * LL_CTRL_BLOCK_SIZE))

#define LL_FIFO_MODULO(name, i)        ((i >= name.size) ? 0 : i)

#define LL_OK                          (0x0)      /* Success                          */
#define LL_ERR_UNSUPP_LINK_TYPE        (0x1)      /* Unsupported Link type            */
#define LL_ERR_PORT_OPEN               (0x2)      /* Port open failure                */
#define LL_ERR_NOT_CONNECTED           (0x3)      /* No link established              */
#define LL_ERR_ALREADY_CONNECTED       (0x4)      /* Link is already established      */
#define LL_ERR_MAX_SESSIONS_EXCEEDED   (0x5)
#define LL_ERR_RX_UNEXPECTED_BYTES     (0x6)      /* Unexpected bytes in input stream */
#define LL_ERR_TIMEOUT                 (0x10)     /* Timeout error                    */
#define LL_ERR_GENERIC                 (0x20)     /* Generic Error code               */

typedef enum t_connect 
{
  connect_uart    = 0,                            /* Select UART                      */
  connect_usb     = 1,                            /* Select USB                       */
  connect_unknown = 2
} t_connect;

/*==== TYPEDEFINES ==========================================================*/

typedef struct T_LL_INSTANCE
{
  U32 drv_hndl;
} T_LL_INSTANCE;

typedef enum T_ll_recv_result
{
  ll_recv_result_none,
  ll_recv_result_normal,
  ll_recv_result_download_last,
  ll_recv_result_download_more
} T_ll_recv_result;

typedef struct T_LL_FIFO 
{
  volatile U16   occupancy;
  volatile U16   head;
  volatile U16   tail;
  volatile U16   size;
  t_message    * data[LL_FIFO_SIZE_HEAP];
} T_LL_FIFO;

typedef struct T_LL_CONNECT
{
  U8 link_type;
  U8 uart_no;
  U8 uart_baudrate;
  U8 usb_transfer_type;
  /// set this to true to have 2nd skip re-enumerate USB
  BOOLEAN reuse_romconnection;
} T_LL_CONNECT;

typedef struct T_LL_CONNECT_FPTR
{
  t_connect connect_type; /* either connect_USB or connect_UART */
  S32 (*write_fptr)(U32 dis_addr, U32 tag, U32 *len, U8 *buf);
  S32 (*read_fptr)(U32 dis_addr, U32 tag, U32 *len, U8 *buf);
  S32 (*deinit_fptr)(U32 dis_addr);
  U32 start_xmit_key;
  U32 mask_intr;
} T_LL_CONNECT_FPTR;

typedef enum T_ll_recv_mode 
{
  ll_recv_mode_peek,      ///< just look at the front of the queue
  ll_recv_mode_try_poll,  ///< peek and if there is nothing start xmit
  ll_recv_mode_poll,      ///< try_get until there is some thing
  ll_recv_mode_try_get,   ///< poll and if there is somthing remove it from queue
  ll_recv_mode_get        ///< try_get until there is some thing
} T_ll_recv_mode;

/*==== GLOBALS ==============================================================*/

extern T_LL_CONNECT_FPTR connect_type; //last type of connection made, either USB or UART

/*====== FUNCTION PROTOTYPES=================================================*/
#ifdef __cplusplus
extern "C" {
#endif

void        ll_init(BOOLEAN heap);
S8          ll_connect(t_connect connect);
U32         ll_disconnect(t_connect * connect);
t_message * ll_receive(void);
S8          ll_send_ctrl_acked(U8 *buf, U32 size);
S8          ll_send_ctrl_unacked(U8 *data, U32 size);
S8          ll_send_data(U8 * data, U32 size);

#ifdef __cplusplus
}
#endif
#endif /* LL_H */
/*==== END OF FILE ===========================================================*/

