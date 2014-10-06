/**
 * @file SerialDriver.h
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
 *  This header file defines the Serial (UART) Driver class
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef __CSST_SERIALDRIVER_H__
#define __CSST_SERIALDRIVER_H__

/*==== INCLUDES =============================================================*/
#include "stdafx.h"
#include "types.h"
#include "ComDriver.h"

/*==== MACROS ===============================================================*/

/*==== CONSTS ===============================================================*/
static U16		uniqueCounter=100;

/*==== TYPES ================================================================*/

/*==== EXPORTS ==============================================================*/

class API SerialDrv : public ComDriver
{
public:
    SerialDrv(U16 sid = 0xFFFF);
    ~SerialDrv();
    S8 open(U8 port, U32 baudRate, U8 parity, U8 stopBits, U8 data);
    S8 close(void);
    S8 reset(void);
    S8 configure(U32 baudRate, U8 parity, U8 stopBits, U8 data);
    S32 write(U8 *p_buffer, U32 size);
    S32 read(U8 *p_buffer, U32 size, U32 timeout);
    void registerCallback(T_COM_DRV_CALL_BACK callback);
    void unregisterCallback();
    void registerConnectionLostCallback(T_COM_DRV_CONNECTION_LOST_CALL_BACK callback);
	U16 sid() { return m_sid; }
	int getDrvErr() { return 0; }

private:
    U16                 m_sid;
    U8					m_portNo;
    U8                  m_isOpen;
    U8                  m_isClosing;
    U8					m_constructionError;
    HANDLE              m_hReadyEvent;
    HANDLE              m_hStopEvent;
    HANDLE              m_hSerial;
    HANDLE              m_hThread;
    HANDLE				m_hBeginListening;
    HANDLE				m_hListenerDone;
    OVERLAPPED          m_readOverlapped;
    OVERLAPPED          m_readOverlappedStatus;
    OVERLAPPED          m_writeOverlapped;
    U32                 m_threadId;
    T_COM_DRV_CALL_BACK m_callBack;
    U8					m_callBackRegistered;
    U32					m_sizeToRead;
    U32					m_baudRate;
    U8*					m_pBlockingReadBuffer;
    U8*					m_pBlockingReadBufferActualPosition;
    U32					m_blockingReadBytesAvailable;
    U32					m_lastCommTimeout;
    S32 				read_hw(U8 *p_buffer, U32 size, U32 timeout);
    S32 				ResetCommError(DWORD* bytesRxLeft = NULL, DWORD* bytesTxLeft = NULL);
    friend unsigned int __stdcall serialListener(void *p_drvObj);
protected:
};

#endif /* __CSST_SERIALDRIVER_H__ */

/*==== END OF FILE ==========================================================*/

