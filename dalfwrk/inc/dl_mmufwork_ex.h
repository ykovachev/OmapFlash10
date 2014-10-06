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
| Purpose:    This file contains the MMU API Functions.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL =================================================*/

#ifndef CSST_DAL_MMUFWORK_EX_H
#define CSST_DAL_MMUFWORK_EX_H

/*==== INCLUDES ============================================================*/
/*TBD*/

/*==== CONSTS ==============================================================*/
//#define DAL_MMU_ACCESS_MGR  0xc0
/*TBD*/

/*==== TYPEDEFINES ===============================================================*/
/*TBD*/

/*====== STATIC and GLOBAL ==================================================*/


/*====== FUNCTION PROTOTYPES==================================================*/
S32 dal_mmu_init();

S32 dal_mmu_map_section
(
 U32      virtual_base,
 U32      physical_base,
 S32      bufferable,
 S32      cacheable
);
#endif /* CSST_DAL_MMUFWORK_H */

