/**
 * @file peripheral.h
 * @author 
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
 * 
 */
/**
 * @file silicon.h
 * @author Jens Odborg
 * @brief Contains the base addresses of the interfaces or peripherals present in the silicon.
 * @details 
 * @todo file details
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM/OMAP4430_NDA_TRM_MEMSS_EMIF_v3.0.pdf
 * @todo updata link
 */

#ifndef PERIPHERAL_H
#define PERIPHERAL_H

/*====== INCLUDES ===========================================================*/

#include "types.h"

/*====== DEFINES ============================================================*/

#define ROMAPI_USB_SUCCESS       OMAPFLASH_SUCCESS
#define ROMAPI_USB_ERROR         OMAPFLASH_ERROR
#define ROMAPI_USB_DOWNLOAD_MORE 1
#define ROMAPI_USB_DOWNLOAD_LAST 2

/*==== TYPES ================================================================*/

typedef enum t_peripheral_callback_poll
{
  peripheral_callback,
  peripheral_poll
} t_peripheral_callback_poll;

/*==== GLOBALS ==============================================================*/

S32 usb_peripheral_init(const void *init_str, U32 *dis_addr);

S32 peripheral_write(U32 dis_addr, U32 tag, U32 *len, U8 *buf);
S32 peripheral_read(U32 dis_addr,U32 tag, U32 *len, U8*buf);
S32 peripheral_deinit(U32 dis_addr);

#endif //PERIPHERAL_H
