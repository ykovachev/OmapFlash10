/**
 * @file dnld_trg_func.h
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

/*==== DECLARATION CONTROL =================================================*/
#ifndef DNLD_TGT_FUNC_H
#define DNLD_TGT_FUNC_H

#if 0

/*==== INCLUDES ============================================================*/
#include "types.h"
#include "dl_pinmux_fwrk.h"
#include "dl_gpio.h"
#include "gfci.h"

/*==== CONSTS ==============================================================*/


/*==== TYPES ===============================================================*/

typedef T_pinmux_ret (*TFNCp_pinmux)(T_pinmux*);
typedef U8 (*TFNCp_gpioSetDirection)(U8 gpio,BOOLEAN direction);
typedef U8 (*TFNCp_gpioSetOutput)(U8 gpio, BOOLEAN value);  //TRUE sets to High , FALSE sets to Low
typedef U8 (*TFNCp_gpioGetInput)(U8 gpio);
typedef void (*TFNCp_wait_ms)(U32 ms); //Wait for specified nr. of microseconds
typedef void (*TFNCp_nfcCalculateEcc)(U32 *data_ptr, U32 size, U8 *ecc_gen);
typedef void (*TFNCp_nfcWaitReady)(void); //Block until NAND device is ready
typedef void* (*TFNCp_malloc)(unsigned int tSize);
typedef void (*TFNCp_free)(void * packet);
typedef S32 (*TFNCp_is_onenand)(U16 *vendorID, U16 *deviceID);
typedef S32 (*TFNCp_configure_device)(U32 device_type, U32 base_address, U32 size, U8 cs);
typedef void (*TFNCp_branch_setup)(void);
typedef S8 (* TFNCp_bind)(U32 dst_addr, U8 *p_data_buf, U32 data_size, U8 encryption, U8 **pp_cert_buf, U8 *p_cert_size);
typedef S8 (* TFNCp_unbind)(U8 *p_data_buf, U8 *p_cert_buf);
typedef void (* TFNCp_cache_enable)(void);
typedef void (* TFNCp_cache_disable)(void);
typedef void (* TFNCp_cache_flush)(void);
typedef void (* TFNCp_mmu_save_config)(void);
typedef void (* TFNCp_mmu_restore_config)(void);
typedef void (* TFNCp_prepareCpldForFlashing)(U16 *tck, U16 *tdi, U16 *tdo, U16 *tms);
typedef U32 (*TFNCp_get32khzTime)(void);
typedef void (*TFNCp_printf)(const char *format, ...);
typedef void (*TFNCp_send_info)(char *format, ...);

struct T_dnld_trg_func_struct
{
    TFNCp_cache_enable cache_enable;
    TFNCp_cache_disable cache_disable;
    TFNCp_cache_flush cache_flush;
    TFNCp_mmu_save_config mmu_save_config;
    TFNCp_mmu_restore_config mmu_restore_config;
};

typedef enum T_dnld_trg2drv_func_order
{
    e_prepareCpldForFlashing,
    e_gpioSetDirection,
    e_gpioSetOutput,
    e_gpioGetInput,
    e_wait_ms,
    e_nfcCalculateEcc,
    e_nfcWaitReady,
    e_gfciPtr,
    e_get32khzTime,
    e_printf
} T_dnld_trg2drv_func_order; // Order must match function order in struct T_dnld_trg2drv_func_struct

typedef enum T_dnld_gen_func_order
{
    e_malloc,
    e_free
} T_dnld_gen_func_order; // Order must match function order in struct T_dnld_gen2drv_func_struct

struct T_dnld_gen2drv_func_struct
{
    TFNCp_malloc malloc;
    TFNCp_free free;
};

struct T_driver_func_offered
{
    U32 sizeof_T_dnld_gen2drv_func_struct;
    struct T_dnld_gen2drv_func_struct* pdlgen2drvfnc;
    U32 sizeof_T_dnld_trg2drv_func_struct;
    struct T_dnld_trg2drv_func_struct*  pdltrg2drvfnc;
};

#ifdef __cplusplus
extern "C" {
#endif

extern struct T_dnld_trg_func_struct     * dnld_trg_func_struct_ptr;
extern struct T_dnld_trg2drv_func_struct * dnld_trg2drv_func_struct_ptr;

#ifdef __cplusplus
}
#endif

#endif

#endif
/*==== END OF FILE ===========================================================*/
