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
|  Filename: dbg_common.h
|  Author  : sHs
| Purpose  : Common header which is used by both the target and the host
|             Debug and Trace Modules
+----------------------------------------------------------------------------*/
/*==== DECLARATION CONTROL =================================================*/

#ifndef CSST_DBG_COMM_H
#define CSST_DBG_COMM_H
/*==== INCLUDES =============================================================*/

#include "types.h"

/*==== CONSTS ==============================================================*/

#define TRACER_TRACE_OFF		0x0
#define TRACER_TRACE_ON			0x1

#define TRACER_LEVEL_OFF		0x0
//#define TRACER_LEVEL_INFO		0x5 //VISHNU
#define TRACER_LEVEL_MINOR		0x4
#define TRACER_LEVEL_MAJOR		0x3
#define TRACER_LEVEL_ALERT		0x2

/* VISHNU*/
#define TRACER_LEVEL_CRITICAL	 0x1
#define TRACER_LEVEL_INFO		 0x2
#define TRACER_LEVEL_MINOR_INFO	 0x3
#define TRACER_LEVEL_HEADER_INFO 0x4
#define TRACER_LEVEL_MISC_INFO	 0x5

#define TRACER_DEST_DBGCONSOLE	0x0
#define TRACER_DEST_CSST		0x1

/*MESSAGE TYPES */
#define	DBGTRACE_SET_DEBUG_PARAMS_REQ	1
#define DBGTRACE_SET_FLOW_REQ			2
#define DBGTRACE_SET_TRACE_REQ			3

#define DBGTRACE_MESSAGES				4

#define DBGTRACE_SET_DEBUG_PARAMS_CNF	5
#define DBGTRACE_SET_FLOW_CNF			6
#define DBGTRACE_SET_TRACE_CNF			7

#define DBGTRACE_TARGET_ERROR_IND		8


/*DIFFERENT TYPES IN THE TLV STRUCTURE */

#define DBGTRACE_ALLMODULES	0xFF


/*==== TYPES ===============================================================*/
typedef void (*DbgCallback)(char* rDbgPrint);

//#define DBGTRACE_DBGPRINT_HEADER			0x0003
//#define DBGTRACE_TRACEPRINT_HEADER			0x0004

#define DBGTRACE_REQ_HEADER 					0x02000106
#define DBGTRACE_CNF_HEADER 					0x02000206
#define DBGTRACE_DBGPRINT_HEADER    			0x02000306
#define DBGTRACE_TRACEPRINT_HEADER  			0x02000406

#define MAX_MODULENAME_LENGTH	255

typedef struct
{
    char mModuleName[255];
    char* mDbgPrint;
}T_DBGTRACE_DBGPRINT_HEADER;

typedef struct
{
    char mModuleName[255];
    char* mTracePrint;
    void* mBuffer;
    U32 mBufferLength;
}T_DBGTRACE_TRACEPRINT_HEADER;

#if 0
typedef struct
{
    U8 mMessageType;
    U8 mModuleid;
    U8 mDbginfo; /*level_No or trace_enable/disable
                 or flow_enable/disable*/
    U8 mDestn;   /*only in the case of DBGTRACE_SET_DEBUG_PARAMS_REQ*/
}T_DBGTRACE_REQ_HEADER;

typedef struct
{
    U8 mMessageType;
    U32 mStatusCode;
}T_DBGTRACE_CNF_HEADER;
#endif

typedef struct
{
    U8 debug_level;/*Critical(1), Info(5), off(0)*/
    U8 debug_msg_destination; /* Debug Console / Host (1) */
    U8 debug_pkt_trace_enable; /*0->disable, else enable*/
    U8 debug_flow_enable; /*0->disable, else enable*/
} T_DBGTRACE_MODULE_RECORD;


/* DBGTRACE packets can be one of the following
DBGTRACE_SET_DEBUG_PARAMS_REQ (ModuleId, level, destn)
DBGTRACE_SET_FLOW_REQ (ModuleId, on\off)
DBGTRACE_SET_TRACE_REQ(ModuleId, on\off)
*/
typedef struct
{
    U8 message_type;
    U8 module_id;
    U8 dbg_info1; /*level_No or trace_enable/disable
                  or flow_enable/disable*/
    U8 destn;     /*only in the case of DBGTRACE_SET_DEBUG_PARAMS_REQ*/
}T_DBGTRACE_REQ_HEADER;

typedef struct
{
    U8 message_type;
    U32 status_code;
}T_DBGTRACE_CNF_HEADER;

#endif /*CSST_DIAG_COMM_H*/
/*==== END OF FILE ==========================================================*/


