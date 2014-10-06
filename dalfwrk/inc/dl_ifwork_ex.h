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
| Purpose:    <description of purpose of this file>
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL =================================================*/

#ifndef CSST_DAL_IFWORK_EX_H
#define CSST_DAL_IFWORK_EX_H

/*==== INCLUDES ============================================================*/
/*TBD*/

#define DAL_INT_EDGE				 0
#define DAL_INT_LEVEL				 1
#define DAL_INT_ROUTE_FIQ			 0
#define DAL_INT_ROUTE_IRQ			 1

/*Exports*/



/*==== CONSTS ==============================================================*/

/*TBD*/

/*==== TYPEDEFINES ===============================================================*/
/*TBD*/
typedef struct
{
    S32         (*handler) (void *data);
    void        *data;
} T_DAL_INT_USER_HANDLER;


typedef struct exception_vectors
{
    U32 undef_instr;
    U32 swi;
    U32 prefech_abort;
    U32 data_abort;
    U32 unused;
    U32 IRQ;
    U32 FIQ;

    U32 undef_instr_addr;
    U32 swi_addr;
    U32 prefech_abort_addr;
    U32 data_abort_addr;
    U32 unused_addr;
    U32 IRQ_addr;
    U32 FIQ_addr;
} T_EXCEPTION_VECTORS;

/*====== FUNCTION PROTOTYPES==================================================*/

#ifdef __cplusplus
extern "C" {
#endif

S32 dal_interrupt_init (void);
S32 dal_interrupt_dispatch ( U32 int_number);

#ifdef __cplusplus
}
#endif
#endif /* CSST_DAL_IFWORK_EX_H */
