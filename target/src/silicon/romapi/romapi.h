/**
 * @file 
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
 * @author Jens Odborg
 * @brief Contains the base addresses of the interfaces or peripherals present in the silicon.
 * @details 
 * @todo file details
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM/OMAP4430_NDA_TRM_MEMSS_EMIF_v3.0.pdf
 * @todo updata link
 */

#ifndef ROMAPI_H
#define ROMAPI_H

/*====== INCLUDES ===========================================================*/

#include "omapconfig.h"
#include "silicon.h"
#include "romapi_types.h"

#ifdef OMAP3
#include "romapi_3430.h"
#endif

#ifdef OMAP4
#include "romapi_4430.h"
#endif

#ifdef OMAP5
#include "romapi_5430.h"
#endif

/*====== ROM TYPES ==========================================================*/

#define PATTERN_VALUE 0x4F4D4150

typedef enum 
{
  ack_request = 0,
  message_ack = 1,
  header_ack  = 2,
  data_ack    = 3
} t_ack_type;

typedef struct  
{
  U32 pattern;
  struct
  {
    U32 more     : 1;
    U32 ctrl     : 1;
    U32 ack      : 1;
    U32 ack_type : 2;
    U32 unused   : 3;
    U32 length   : 24;
  } fields;
} t_message_header;

typedef struct 
{
  t_message_header header;
  U8               content[1];
} t_message;


typedef void (* t_ll_write_callback  )(t_message ** message);
//typedef void (* t_ll_read_callback   )(t_message * message);
typedef t_message * (* t_ll_read_callback_get_storage   )(void);
typedef void (* t_ll_read_callback_deliver   )(void);

typedef struct  
{
  t_ll_write_callback write;
  t_ll_read_callback_get_storage  read_storage;
  t_ll_read_callback_deliver  read_deliver;
} t_ll_callback_api;

typedef S32  (* t_peripheral_init    )(void);
typedef S32  (* t_peripheral_deinit  )(void);
typedef S32  (* t_peripheral_write   )(void);
typedef S32  (* t_peripheral_read    )(void);

typedef struct  
{
  t_peripheral_init   init;
  t_peripheral_deinit deinit;
  t_peripheral_write  write;
  t_peripheral_read   read;
} t_peripheral_api;

typedef struct  
{
  union
  {
    struct
    {
      int dummy;
    } uart;
    struct
    {
      U16              peripheral_init_options; 
      PeripheralDesc_t rom_write_descriptor;
      PeripheralDesc_t rom_read_descriptor;
      PeripheralDesc_t rom_ack_descriptor;
    } usb;
  } configuration;

  t_peripheral_api  call;
  t_ll_callback_api callback;
} t_peripheral;

//typedef S32 (* t_peripheral_get_descriptor)(t_peripheral * peripheral);

#endif //ROMAPI_H
