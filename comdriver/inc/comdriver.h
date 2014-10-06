/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  Communication driver
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
| Filename:   ComDriver.h
|
| Purpose:    This header file defines the Com Driver class
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL =================================================*/
#ifndef __CSST_COMDRIVER_H__
#define __CSST_COMDRIVER_H__

/*==== INCLUDES ============================================================*/
#include "stdafx.h"
#include "types.h"

/*==== MACROS ==============================================================*/
#ifdef API
#undef API
#endif

#ifdef COMDRIVER_EXPORTS
#define API __declspec(dllexport)
#else
#define API __declspec(dllimport)
#endif

#define COM_DRIVER_DEBUG_ENV_VARIABLE "CSST_COMDRIVER_DEBUG_LEVEL"

#define COM_DRIVER_DEFAULT_TIMEOUT      2000

#define COM_DRIVER_RET_OK               (0)
#define COM_DRIVER_RET_ERR              (-1)

/*==== CONSTS ==============================================================*/

/*==== TYPES ===============================================================*/
typedef S8 (*PCOM_DRV_CALL_BACK) (U16 sid, U8 *p_buf, U32 size);
typedef PCOM_DRV_CALL_BACK T_COM_DRV_CALL_BACK;

typedef S8 (*PCOM_DRV_CONNECTION_LOST_CALL_BACK) (U16 sid);
typedef PCOM_DRV_CONNECTION_LOST_CALL_BACK T_COM_DRV_CONNECTION_LOST_CALL_BACK;

/*==== EXPORTS =============================================================*/
class API ComDriver
{
public:
    virtual ~ComDriver() { };
    virtual S8 open(U8 port, U32 baudRate, U8 parity, U8 stopBits, U8 data) = 0;
    virtual S8 close(void) = 0;
    virtual S8 reset(void) = 0;
    virtual S8 configure(U32 baudRate, U8 parity, U8 stopBits, U8 data) = 0;
    virtual S32 write(U8 *buffer, U32 size) = 0;
    virtual S32 read(U8 *buffer, U32 size, U32 timeout) = 0;
    virtual void registerCallback(T_COM_DRV_CALL_BACK callback) = 0;
    virtual void unregisterCallback() = 0;
    virtual void registerConnectionLostCallback(T_COM_DRV_CONNECTION_LOST_CALL_BACK callback) = 0;
	virtual U16 sid() = 0;
	virtual int getDrvErr() = 0;
protected:
    U8 debugLevel;
    virtual U8 initDebugLevel();
    virtual void debugPrint(U8 level, const char *format, ...);
};

#endif /* __CSST_COMDRIVER_H__ */
/*==== END OF FILE ===========================================================*/

