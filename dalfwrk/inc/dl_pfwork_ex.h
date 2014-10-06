/*-----------------------------------------------------------------------------
|  Project :  CSST
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
| Purpose:    This file contains the function declaration of the
|			  DAL PERIPHERAL FRAMEWORK APIs
|			  The application modules that uses the dal apis need to
|			  include this file
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL =================================================*/

#ifndef CSST_DAL_PFWORK_EX_H
#define CSST_DAL_PFWORK_EX_H

#include "csst_tgt.h"
/*==== INCLUDES ============================================================*/
/*TBD*/

/*==== CONSTS ==============================================================*/

/*TBD*/

/*==== TYPEDEFINES ===============================================================*/
/*TBD*/

/*====== FUNCTION PROTOTYPES==================================================*/

#ifndef EMMC_DRV
S32 dal_initialization();
S32 dal_init(void * init_str, U32 * devicehandle);
S32 dal_write(U32 devicehandle, U32 tag, U32 * length, U8 * buffer);
S32 dal_read(U32 devicehandle, U32 tag, U32 * length, U8 * buffer);
S32 dal_deinit(U32 devicehandle);

S32 dal_register_device_driver_functions
(
 U32 device_pid,
 S32 (*device_init)  (const void* init_str, U32 *  dis_addr),
 S32 (*device_write) (U32 dis_addr, U32 tag, U32 * size, U8 * buffer),
 S32 (*device_read)  (U32 dis_addr, U32 tag, U32 * size, U8 * buffer),
 S32 (*device_deinit)(U32 dis_addr)
);
#endif //EMMC_DRV




#endif /* CSST_DAL_PFWORK_H */


