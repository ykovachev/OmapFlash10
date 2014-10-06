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
| Filename: dl_pinmux_fwork.c
| Author: Frank Jankowski
| Purpose: This file contains generic pinmuxing functions
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef __CSST_DL_PINMUX_FWRK_H
#define __CSST_DL_PINMUX_FWRK_H

/*==== CONSTS ==============================================================*/

#define  PULL_ENABLE_MASK              0x8
#define  PULL_UP_MASK                  0x10


//To be utilized in connection with function void * getcnfgtbl(T_pincnfg_dev cnfg_dev, TRDP_rev RDP_rev)
//in file confiuration_rdp.c
typedef enum {
    e_pincnfg_cpld,
    e_OPO_cpld,
    NO_OF_pincnfg_dev
}T_pincnfg_dev;

#define PMX_SUCCESS ((0))
#define PMX_ERROR ((-1))
#define PMX_BALL_NOT_SUPPORTED ((-2))
#define PMX_PACKET_NOT_SUPPORTED ((-10))

typedef enum{
    packet_BGA=0,   //default packet implying ball numbering A1,A2, A.. , B1,B2,..,B13,..,H23... etc
    packet_QP=1,    //alternate packet implying pin numbering  1,2,3,4,.. etc
    NO_OF_PACKET
} t_packet_type;


typedef enum
{
    bA, bB,bC,bD,bE,bF,bG,bH,bI,bJ,bK,bL,bM,bN,bO,bP,bQ,bR,bS,bT,bU,bV,bX,bY,bZ
}T_BALL_X ;

typedef struct {
    t_packet_type packet_type;
}T_config_ext;

typedef struct{
    U32 ball;
    U8 mode;
    U8 configuration;
    void * p_config_ext; //if NULL the default packet type packet_BGA is chosen
} T_pinmux;

typedef S16 T_pinmux_ret;

/*=====PROTOTYPES============================================================*/
T_pinmux_ret  pinmux(T_pinmux * p_pinmux);

#endif /*csst_pmux_c.c*/
/*=============END OF FILE=====================*/



