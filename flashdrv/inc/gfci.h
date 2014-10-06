/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  Download Module (ROM assisted)
+------------------------------------------------------------------------------
|             Copyright 2003 Texas Instruments.
|             All rights reserved.
|
|             This file is confidential and a trade secret of Texas
|             Instruments.
|             The receipt of or possession of this file does not convey
|             any rights to reproduce or disclose its contents or to
|             manufacture, use, or sell anything it may describe, in
|             whole, or in part, without the specific written consent of
|             Texas Instruments.
+------------------------------------------------------------------------------
| Filename:    gfci.h
| Author:      Thomas Soehus (tls@ti.com)
| Purpose:     Generic Flash Controller Interface definition.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef __GFCI_H__
#define __GFCI_H__

/*==== INCLUDES =============================================================*/
#include "types.h"
/*==== MACROS ===============================================================*/

/*==== CONSTS ===============================================================*/

/*==== TYPES ================================================================*/

typedef enum
{
    GFCI_SUCCESS = 0,
    GFCI_ECC_ERROR = -1,
    GFCI_TIMEOUT = -2,
    GFCI_NOT_IMPLEMENTED = -0x80
} GFCISTATUS;

#ifdef __cplusplus //SIMULATION
inline GFCISTATUS operator |= (GFCISTATUS &a, GFCISTATUS b) 
{
    a = (GFCISTATUS)(a|b);
    return a;
}
#endif


typedef enum
{
    NO_ECC = 0,
    ONE_BIT_ECC = 1,
    FOUR_BIT_ECC = 4
} T_ECC;

typedef enum
{
    WIDTH_8  = 0,
    WIDTH_16 = 1
} T_BUS_WIDTH;

typedef struct
{
    U32 cs;
    T_ECC ecc_mode;
    T_BUS_WIDTH width;
} T_GFCI_PARMS;


/* An opaque platform-specific structure holding information relevant to the
specific memory controller.*/

typedef struct T_MEM_CTRL* PMEMCTRL;

typedef struct
{
    PMEMCTRL    (*create)             (T_GFCI_PARMS *parms);
    GFCISTATUS  (*destroy)            (PMEMCTRL nc);
    GFCISTATUS 	(*nand_set_command)   (PMEMCTRL nc, U8 command);
    GFCISTATUS	(*nand_set_address)   (PMEMCTRL nc, U8 address);
    GFCISTATUS 	(*nand_data_write)    (PMEMCTRL nc, U16 *data, U32 length);
    GFCISTATUS 	(*nand_data_read)     (PMEMCTRL nc, U16 *data, U32 length);
    GFCISTATUS 	(*nand_wait_ready)    (PMEMCTRL nc);
    GFCISTATUS 	(*nand_ecc_calc_start)(PMEMCTRL nc, U8 *data, U32 length, U32 *ecc);
    GFCISTATUS 	(*nand_ecc_calc_read) (PMEMCTRL nc, U8 *data, U32 length, U32 *ecc);
} T_GFCI;


/*==== LOCALS ===============================================================*/

/*==== PRIVATE FUNCTIONS ====================================================*/

#endif  /* __GFCI_H__ */

/*==== END OF FILE gfci.c ==========================================================*/

