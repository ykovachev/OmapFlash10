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
| Purpose:    This file contains the dal_device_driver_table
, dal_device_handle_table, and other static structures
that the DAL Pheriperal framework uses
+-------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===============================================*/

#ifndef CSST_DBG_PRINTS_H
#define CSST_DBG_PRINTS_H

/*==== INCLUDES ===========================================================*/
/*TBD*/

/*==== MACROS ===========================================================*/
/*==== CONSTS ==============================================================*/


#define DMPT	debug_module_parameter_table

#define DBGTRACE_MAX_MODULES	10
#define DBG_EGRESS			1

#define DBGTRACE_ENABLE		1
#define DBGTRACE_DISABLE	0


#define DBGTRACE_MAX_LEVELS		    6
#define DBGTRACE_MAX_DESTN			2

/*#define DBGTRACE_FLOW		6*/

#define DBGTRACE_MAX_FILENAME_LENGTH	20
#define DBGTRACE_MAX_FUNCNAME_LENGTH	40

#define DBGTRACE_MAX_MESSAGESIZE		256 //512

#define GET_DBG_HEADER_PTR(sdu_handle,length_addr)   \
        (T_DBGTRACE_REQ_HEADER *)(sdu_get_user_data(sdu_handle,length_addr))
/*==== TYPEDEFINES =========================================================*/

/* DBGTRACE packets can be one of the following
DBGTRACE_SET_DEBUG_PARAMS_REQ (ModuleId, level, destn)
DBGTRACE_SET_FLOW_REQ (ModuleId, on\off)
DBGTRACE_SET_TRACE_REQ(ModuleId, on\off)
*/

/*==== STATIC & GLOBAL ====================================================*/
//static T_DBGTRACE_MODULE_RECORD debug_module_parameter_table[MAX_MODULES];


/*====== FUNCTION PROTOTYPES================================================*/


#endif /* CSST_DAL_PFWORK_H */
