/*-----------------------------------------------------------------------------
|  Project :  CSST
+------------------------------------------------------------------------------
|             Copyright 2005 - 2008 Texas Instruments.
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
| Purpose:    This file contains the dal_device_driver_table
, dal_device_handle_table, and other static structures
that the DAL Pheriperal framework uses
+-------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===============================================*/

#ifndef CSST_DAL_PFWORK_H
#define CSST_DAL_PFWORK_H

/*==== INCLUDES ===========================================================*/
/*TBD*/
#include "csst_tgt.h"

/*==== MACROS ===========================================================*/
#define		DAL_MAX_DEVICES						60//30
#define		DAL_MAX_DEVICE_HANDLE				10
#define		DDDT								dal_device_driver_table
#define		DDHT								dal_device_handle_table
#define 	DAL_DIRTY_RECORD					0xFFFFFFFF
/*==== CONSTS ==============================================================*/

/*TBD*/

/*==== TYPEDEFINES =========================================================*/
/*void * is used for init_str, since the initialization structures
for various devices are different
same for the other paramters also*/

typedef struct
{
    S32 (*device_init)  (const void* init_str, U32*  dis_addr);
    S32 (*device_write) (U32 dis_addr, U32 tag, U32 * size, U8 * buffer);
    S32 (*device_read)  (U32 dis_addr, U32 tag, U32 * size, U8 * buffer);
    S32 (*device_deinit)(U32 dis_addr);
} T_DAL_DEVICE_DRIVER_ENTRY;
/*Device Driver Entry contains the driver functions*/

typedef struct
{
    U16 pid;
    U16 sid;
} T_DAL_DEVICE_ID;


typedef struct
{
    T_DAL_DEVICE_ID device_id;
    U32 dis_addr;
} T_DAL_DEVICE_HANDLE_ENTRY;

/*Contains the initialised */


/*==== STATIC & GLOBAL ====================================================*/

static T_DAL_DEVICE_DRIVER_ENTRY dal_device_driver_table[DAL_MAX_DEVICES];

static T_DAL_DEVICE_HANDLE_ENTRY
dal_device_handle_table[DAL_MAX_DEVICE_HANDLE];


/*====== FUNCTION PROTOTYPES================================================*/

#endif /* CSST_DAL_PFWORK_H */
