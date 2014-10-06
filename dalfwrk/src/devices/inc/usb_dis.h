/**
 * @file usb_dis.h
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
#ifndef USB_DIS_H
#define USB_DIS_H

/*==== INCLUDES =============================================================*/
#include "types.h"
#include "uart_dis.h"

#define USB_LLDEVICE_SID            1
#define USB_FS_ENUMERATIONTEST_SID  2
#define USB_HS_ENUMERATIONTEST_SID  3

/*==== MACROS ===============================================================*/

#define USB_START_XMIT_KEY          UART_START_XMIT_KEY    /* UART Start XMIT */
#define USB_STOP_XMIT_KEY           UART_STOP_XMIT_KEY   /* UART Stop XMIT */
#define USB_USER_DATA               UART_USER_DATA   /* User data */
#define USB_USER_DATA_CSTR          UART_USER_DATA_CSTR   /* User data until null */
#define USB_MASK_INTR               UART_MASK_INTR
#define USB_ENUM_STATUS		          13
#define USB_USER_DATA_ACKED         UART_USER_DATA_ACKED

/*==== TYPEDEFINES ==========================================================*/

typedef struct T_USB_INIT_STRUCTURE
{                       
  U16 pid;			      ///<this is ignored by the driver
  U16 sid;            ///<this is ignored by the driver
  U8 callback_poll;   ///<this reflect the internal working of the driver, if it is Poll or Callback based, the driver should all ways use ll_*_callback to talk to link layer 
  /// set this to true to have 2nd skip re-enumerate USB
  BOOLEAN reuse_romconnection;
} T_USB_INIT_STRUCTURE;

/*====== FUNCTION PROTOTYPES=================================================*/

extern S32 usb_init(const void *init_str, U32 *dis_addr);
extern S32 usb_deinit(U32 dis_addr);
extern S32 usb_read(U32 dis_addr,U32 tag, U32 *len, U8*buf);
extern S32 usb_write(U32 dis_addr, U32 tag, U32 *len ,U8 *buf);

#endif /* USB_DIS_H */
/*==== END OF FILE ==========================================================*/

