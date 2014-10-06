/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  DAL - USB driver
+------------------------------------------------------------------------------
|             Copyright 2005 Texas Instruments.
|             All rights reserved.
|
|             This file is confidential and a trade secret of Texas
|             Instruments .
|             The receipt of or possession of this file does not convey
|             any rights to reproduce or disclose its contents or to
|             manufacture, use, or sell anything it may describe, in
|             whole, or in part, without the specific written consent of
|             Texas Instruments.
+------------------------------------------------------------------------------
| Filename:   usb_drv.h
| Author:     Witold Pietraszek
| Purpose:    Defines the DAL USB functionality
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef CSST_DAL_USB_DRV_H
#define CSST_DAL_USB_DRV_H

/*==== INCLUDES =============================================================*/
#include "types.h"
//#include "uart.h"

/*==== MACROS ===============================================================*/

/*==== TYPEDEFINES ==========================================================*/

/*====== FUNCTION PROTOTYPES=================================================*/
S32 usb_init(const void *init_str, U32 *dis_addr);
S32 usb_deinit(U32 dis_addr);
S32 usb_read(U32 dis_addr, U32 tag, U32 *len, U8 *buf);
S32 usb_write(U32 dis_addr, U32 tag, U32 *len ,U8 *buf);

#endif /* CSST_DAL_USB_DRV_H */
/*==== END OF FILE ==========================================================*/

