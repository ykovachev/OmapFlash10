/**
 * @file romapi_types.h
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
 * @brief Contains types used by romapi in OMAP3 and OMAP4.
 * @details 
 * Strictly many types are only slightly the same and thus depend on #defines set in silicon.h 
 * for proper working do not include this file directly but include romapi.h
 * @todo file details
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM/OMAP4430_NDA_TRM_MEMSS_EMIF_v3.0.pdf
 * @todo updata link
 */

#ifndef ROMAPI_TYPES_H
#define ROMAPI_TYPES_H

#include "types.h"
#include "config.h"

#ifdef SIMULATION
int simulation_call_romapi(char* type, int address, const struct PeripheralDesc_t *pPeripheralDesc);
int simulation_call_romapi_1(char* type, int address, int);
int simulation_call_romapi_2(char* type, int address, int, int);
int simulation_call_romapi_3(char* type, int address, int, int, int);
int simulation_call_romapi_4(char* type, int address, int, int, int, int);
int simulation_call_romapi_5(char* type, int address, int, int, int, int, int);
int simulation_call_romapi_6(char* type, int address, int, int, int, int, int, int);
int simulation_call_romapi_7(char* type, int address, int, int, int, int, int, int, int);
int simulation_call_romapi_8(char* type, int address, int, int, int, int, int, int, int, int);
#define CALL_ROMAPI_8(return,type,address,a,b,c,d,e,f,g,h) (simulation_call_romapi_8(#type,address,(int)a,(int)b,(int)c,(int)d,(int)e,(int)f,(int)g,(int)h))
#define CALL_ROMAPI_7(return,type,address,a,b,c,d,e,f,g) (simulation_call_romapi_7(#type,address,(int)a,(int)b,(int)c,(int)d,(int)e,(int)f,(int)g))
#define CALL_ROMAPI_6(return,type,address,a,b,c,d,e,f) (simulation_call_romapi_6(#type,address,(int)a,(int)b,(int)c,(int)d,(int)e,(int)f))
#define CALL_ROMAPI_5(return,type,address,a,b,c,d,e) (simulation_call_romapi_5(#type,address,(int)a,(int)b,(int)c,(int)d,(int)e))
#define CALL_ROMAPI_4(return,type,address,a,b,c,d) (simulation_call_romapi_4(#type,address,(int)a,(int)b,(int)c,(int)d))
#define CALL_ROMAPI_3(return,type,address,a,b,c) (simulation_call_romapi_3(#type,address,(int)a,(int)b,(int)c))
#define CALL_ROMAPI_2(return,type,address,a,b) (simulation_call_romapi_2(#type,address,(int)a,(int)b))
#define CALL_ROMAPI_1(return,type,address,a) (simulation_call_romapi_1(#type,address,(int)a))
#define CALL_ROMAPI_0(return,type,address) (simulation_call_romapi(#type,address, NULL))
#else
#define CALL_ROMAPI_8(return,type,address,a,b,c,d,e,f,g,h) ((return)(**(type**)address)(a,b,c,d,e,f,g,h))
#define CALL_ROMAPI_7(return,type,address,a,b,c,d,e,f,g) ((return)(**(type**)address)(a,b,c,d,e,f,g))
#define CALL_ROMAPI_6(return,type,address,a,b,c,d,e,f) ((return)(**(type**)address)(a,b,c,d,e,f))
#define CALL_ROMAPI_5(return,type,address,a,b,c,d,e) ((return)(**(type**)address)(a,b,c,d,e))
#define CALL_ROMAPI_4(return,type,address,a,b,c,d) ((return)(**(type**)address)(a,b,c,d))
#define CALL_ROMAPI_3(return,type,address,a,b,c) ((return)(**(type**)address)(a,b,c))
#define CALL_ROMAPI_2(return,type,address,a,b) ((return)(**(type**)address)(a,b))
#define CALL_ROMAPI_1(return,type,address,a) ((return)(**(type**)address)(a))
#define CALL_ROMAPI_0(return,type,address) ((return)(*(type**)address)())
#endif

#ifdef OMAP3
#define PBOOTDEV_FIRST_DEV (0x10)
// Device types
typedef enum PeriphDeviceType_e
{
  DEVICE_TYPE_UART = PBOOTDEV_FIRST_DEV, //0x10 16
  DEVICE_TYPE_HS_USB, //0x11 17
  DEVICE_TYPE_SSI, //0x12 18
  DEVICE_TYPE_SSI_HANDSHAKE, //0x13 19
  DEVICE_TYPE_NOMOREDEVICE //0x14 20
} PeriphDeviceType_e;
#endif //OMAP3

#if defined OMAP4 || defined OMAP5
// Device types
typedef enum DEVICE_Per_e
{
  DEVICE_TYPE_NULL_PER = (0x40), //64
  DEVICE_TYPE_UART1, //0x41 65
  DEVICE_TYPE_UART2, //0x42 66
  DEVICE_TYPE_UART3, //0x43 67
  DEVICE_TYPE_UART4, //0x44 68
  DEVICE_TYPE_USB,   //0x45 69
  DEVICE_TYPE_USBEXT //0x46 70 
} DEVICE_Per_e;
#endif

#ifdef OMAP4
typedef enum HAL_TRANSFER_MODE_e
{
  HAL_TRANSFER_MODE_CPU = (0x00),
  HAL_TRANSFER_MODE_DMA
} HAL_TRANSFER_MODE_e;
#endif //OMAP4 

typedef enum STATUS
{
  NO_ERROR, //0
  FAILED, //1
  TIMED_OUT, //2
  PARAM_ERROR, //3
  WAITING, //4
  MALLOC_ERROR, //5
  INVALID_POINTER, //6
  STATUS_SIZE //7 must be last
} STATUS;

// Peripheral Descriptor

#ifdef OMAP3
typedef struct PeripheralDesc_t
{
  void *             IoConfObj;                        /* pointer to the IO Configuration    structure */
  void               (*pCallBackFunction)( void * );   /* pointer to the callback function */
  U32                Address;                          /* address of data to send or receive */
  U32                Size;                             /* size of data in bytes */
  U16 *              pOptions;                         /* Pointer to Boot options from topmain, see def in 4.1.1*/
  PeriphDeviceType_e Device;                           /* Type of the Device used */
  volatile STATUS    Status;                           /* Current Status */
} PeripheralDesc_t;
#endif

#ifdef OMAP4
typedef struct PeripheralDesc_t
{
  void *              IoConfObj;                        /* pointer to the IO Configuration    structure */
  void                (*pCallBackFunction)( void * );   /* pointer to the callback function */
  U32                 Address;                          /* address of data to send or receive */
  U32                 Size;                             /* size of data in bytes */
  U16 *               pOptions;                         /* Pointer to Boot options from topmain, see def in 4.1.1*/
  HAL_TRANSFER_MODE_e Mode;                             /* This field is only used by the USB driver. */
  DEVICE_Per_e        Device;                           /* Type of the peripheral device used */
  volatile STATUS     Status;                           /* Current Status */
  U16                 HsTocMask;                        /* Reserved. Should be zero. */
  U16                 GpTocMask;                        /* Reserved. Should be zero. */
} PeripheralDesc_t;
#endif

#ifdef OMAP5
typedef struct PeripheralDesc_t
{
  void *              IoConfObj;                        /* pointer to the IO Configuration    structure */
  void                (*pCallBackFunction)( void * );   /* pointer to the callback function */
  U32                 Address;                          /* address of data to send or receive */
  U32                 Size;                             /* size of data in bytes */
  U16 *               pOptions;                         /* Pointer to Boot options from topmain, see def in 4.1.1*/
  DEVICE_Per_e        Device;                           /* Type of the peripheral device used */
  volatile STATUS     Status;                           /* Current Status */
  U16                 HsTocMask;                        /* Reserved. Should be zero. */
  U16                 GpTocMask;                        /* Reserved. Should be zero. */
  void *              pDeviceData;                      /* Device dependent sub-structure pointer */
} PeripheralDesc_t;
#endif

#endif //ROMAPI_TYPES_H
