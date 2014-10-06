/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  Flash programmer (ROM assisted)
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
| Filename:   usb_drv.c
| Author:     Linda Irish (lirish@ti.com)
| Purpose:    Drivers needed to support USB
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef CSST_DAL_USB_DRV_C
#define CSST_DAL_USB_DRV_C
#endif


/*==== INCLUDES =============================================================*/
#include <stdio.h>
#include "types.h"
#include "usb_dis.h"
#include "interrupt.h"
#include "error.h"
#include "usb_dis.h"
#include "usb.h"

/*==== GLOBALS ==============================================================*/
/* Pointer to hook the enumeration status function */
BOOLEAN (*usb_enum_status)(U32) = NULL;

/*==== PUBLIC FUNCTIONS =====================================================*/
S32 usb_init(const void *init_str, U32 *dis_addr)
{
    // update the dis_addr .
    *dis_addr = (U32) init_str;

    return(usb_initialize((T_USB_INIT_STRUCTURE *) init_str));
}

S32 usb_deinit(U32 dis_addr)
{
    return(usb_deinitialize(dis_addr));
}

S32 usb_read(U32 dis_addr,U32 tag, U32 *len, U8*buf)
{
    switch(tag)
    {
    case USB_START_XMIT_KEY:
        usb_rx_start();
        break;

    case USB_STOP_XMIT_KEY:
        usb_rx_stop();
        break;

    case USB_USER_DATA:
        return(usb_read_hw( len, buf ));

    case USB_USER_DATA_CSTR:
        return(usb_read_hw_cstr( len, buf ));

    case USB_ENUM_STATUS:
        if (usb_enum_status != NULL)
        {
            if (usb_enum_status(dis_addr) == TRUE)
            {
                *buf = TRUE;
            }
            else
            {
                *buf = FALSE;
            }
        }
        return CSST_SUCCESS;

    default:
        break;
    }
    return CSST_ERROR;
}


S32 usb_write(U32 dis_addr, U32 tag, U32 *len ,U8 *buf)
{
    switch(tag)
    {
    case USB_START_XMIT_KEY:
        usb_tx_start();
        break;
    case USB_STOP_XMIT_KEY:
        usb_tx_stop();
        break;
    case USB_MASK_INTR:
        usb_int_mask(*buf);
        break;
    case USB_USER_DATA:
        usb_write_hw(len, buf);
        break;
    default:
        break;
    }
    return(0);
}

/*==== END OF FILE ==========================================================*/

