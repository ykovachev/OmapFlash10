/**
 * @file SerialDriver.cpp
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
 *  This file contains the serial (USRT) driver for CSST host side.
 */

#include "stdafx.h"
#include "stdlib.h"

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES =============================================================*/
#include "dbg_common.h"
#include "serialdriver.h"

/*==== MACROS ===============================================================*/
/*==== CONSTS ===============================================================*/
#define SERIALDRIVER_TIMEOUT 2000
#define MAX_NUM_EVENT_HANDLES 2
#define BUF_SIZE 1024 /*1024*/
#define RX_HW_BUF_SIZE /*4096*/BUF_SIZE
#define TX_HW_BUF_SIZE /*4096*/BUF_SIZE
#define EVENT_MASK EV_RXCHAR | EV_ERR
#define BLOCKING_READ_BUF_SIZE 1024

//#define VERBOSE_DEBUGGING
#define USE_BUFFERED_BLOCKING_READ

/*==== TYPES ================================================================*/
/*==== GLOBALS ==============================================================*/
/*==== PUBLIC FUNCTIONS =====================================================*/


/*-----------------------------------------------------------------------------
| Function    : SerialDrv()
+------------------------------------------------------------------------------
| Description : Serial Driver Constructor
|
| Parameters  : U16 sid
|
| Returns     : None
+----------------------------------------------------------------------------*/
SerialDrv::SerialDrv(U16 sid) : m_sid(sid)
{
    m_hReadyEvent = NULL;
    m_hStopEvent = NULL;
    m_hBeginListening = NULL;
    m_hListenerDone = NULL;
    m_hSerial = INVALID_HANDLE_VALUE;
    m_hThread = INVALID_HANDLE_VALUE;
    m_callBack = NULL;
    m_pBlockingReadBuffer = NULL;
    m_constructionError = 0;
    m_callBackRegistered = 0;
    m_portNo = 0xFF;
    memset(&m_readOverlapped, 0, sizeof(m_readOverlapped));
    memset(&m_readOverlappedStatus, 0, sizeof(m_readOverlappedStatus));
    memset(&m_writeOverlapped, 0, sizeof(m_writeOverlapped));

    /* Default state is not opened. */
    m_isOpen = 0;
    m_isClosing = 0;
    m_lastCommTimeout = 0xFFFFFFFF;

    if(m_sid == 0xFFFF)
    {
        m_sid = uniqueCounter++;
        if(uniqueCounter == 0xFFFF) uniqueCounter = 100;
    }

    char eventName[100] = { 0 };
    char timeBuf[9] = { 0 };
    _strtime(timeBuf);
    sprintf(eventName, "%s SerialDrv::m_hBeginListening (Session: %d)", timeBuf, m_sid);
    m_hBeginListening = CreateEvent(NULL, FALSE, FALSE, eventName);
    sprintf(eventName, "%s SerialDrv::m_hListenerDone (Session: %d)", timeBuf, m_sid);
    m_hListenerDone = CreateEvent(NULL, FALSE, FALSE, eventName);

    if (m_hBeginListening == NULL || m_hListenerDone == NULL)
        m_constructionError++;

    //Get debug level from registry.
    initDebugLevel();

}


/*-----------------------------------------------------------------------------
| Function    : ~SerialDrv()
+------------------------------------------------------------------------------
| Description : Serial Driver Decontructor
|
| Parameters  : void
|
| Returns     : None
+----------------------------------------------------------------------------*/
SerialDrv::~SerialDrv()
{
    close();
    if (m_hBeginListening != NULL)
    {
        CloseHandle(m_hBeginListening);
        m_hBeginListening = NULL;
    }
    if (m_hListenerDone != NULL)
    {
        CloseHandle(m_hListenerDone);
        m_hListenerDone = NULL;
    }
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::open()
+------------------------------------------------------------------------------
| Description : Open function for the serial driver.
|
| Parameters  : [IN] port - The serial COM port.
|               [IN] baudRate - The serial communication speed [bps].
|               [IN] parity - Even or ord perity bit can be set.
|               [IN] stopBits - 1, 1½, or 2 stop bits can be configured.
|               [IN] data - The data bits which can be either 5, 6, 7 or 8 bits
|
| Returns     : None
+----------------------------------------------------------------------------*/
S8 SerialDrv::open(U8 port, U32 baudRate, U8 parity, U8 stopBits, U8 data)
{
    /**************** TEMPORARY SOLUTION ******************/
    /* This is a temporary solution for solving an issue  */
    /* where an ACCESS_DENIED is returned when opening    */
    /* the UART after ROM-code communication. The issue   */
    /* is only present when executing csst.exe directly,  */
    /* and not when executing from MS Visual or using DL  */
    /* with the test application. With this in mind, it is*/
    /* suspected that the issue is due to timing of the   */
    /* threads.                                           */
    /* The correct solution is to verify that the UART is */
    /* properly released before sending the connect req.  */
    //Sleep(1000); // TODO: Optimize and remove this delay

    COMSTAT comStat;
    DWORD dwErrors;

    if(m_constructionError>0)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%05.5i]: Object construction not successful - return -1\n", m_sid);
        return -1;
    }

    if (m_isOpen)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%i:%05.5i]: Already opened - doing nothing - return -1\n", m_portNo, m_sid);
        return -1; /* PORT_IS_ALREADY_OPEN */
    }

    debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::open[%i:%05.5i]: Port: %i, Baud: %i\n", port, m_sid, port, baudRate);

    char eventName[100] = { 0 };
    char timeBuf[9] = { 0 };
    _strtime(timeBuf);

    sprintf(eventName, "%s SerialDrv::m_readOverlapped.hEvent (Session: %d)", timeBuf, m_sid);
    m_readOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, eventName);
    sprintf(eventName, "%s SerialDrv::m_readOverlappedStatus.hEvent (Session: %d)", timeBuf, m_sid);
    m_readOverlappedStatus.hEvent = CreateEvent(NULL, TRUE, FALSE, eventName);
    sprintf(eventName, "%s SerialDrv::m_writeOverlapped.hEvent (Session: %d)", timeBuf, m_sid);
    m_writeOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, eventName);

    char comPort[20] = { 0 };
    sprintf(comPort, "%s%d", "\\\\.\\COM", port);

    m_hSerial = CreateFile(comPort,                      // Specify port device.
                           GENERIC_READ | GENERIC_WRITE, // Specify mode that open device.
                           0,                            // The devide isn't shared.
                           NULL,                         // The object gets a default security.
                           OPEN_EXISTING,                // Specify which action to take on file.
                           FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, // File attributes
                           0);                           // Default
    DWORD ret = GetLastError();

    if (m_hSerial == INVALID_HANDLE_VALUE)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%i:%05.5i]: Could not open %s. GetLastError() = %i. Closing...\n", m_portNo, m_sid, comPort, ret);

        close();
        return -1;
    }

    m_portNo = port;

    if (configure(baudRate, parity, stopBits, data) != 0)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%i:%05.5i]: Could not configure UART. Closing...\n", m_portNo, m_sid);

        close();
        return -1;
    }
    ClearCommError(m_hSerial, &dwErrors, &comStat); //Clear any errors which may be pending on the port
    PurgeComm(m_hSerial, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR); //Clean out pending data
    SetCommMask(m_hSerial, EVENT_MASK);

    sprintf(eventName, "%s SerialDrv::m_hReadyEvent (Session: %d)", timeBuf, m_sid);
    m_hReadyEvent = CreateEvent(NULL, FALSE, FALSE, eventName);
    sprintf(eventName, "%s SerialDrv::m_hStopEvent (Session: %d)", timeBuf, m_sid);
    m_hStopEvent = CreateEvent(NULL, FALSE, FALSE, eventName);

    if (m_hReadyEvent == NULL || m_hStopEvent == NULL)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%i:%05.5i]: m_hReadyEvent or m_hStopEvent could not be created...\n", m_portNo, m_sid);

        close();
        return -1;
    }

    m_hThread = (HANDLE) _beginthreadex(NULL,
                                        0,
                                        serialListener,
                                        (void *) this,
                                        0,
                                        (unsigned int *) &m_threadId);
    if (m_hThread == INVALID_HANDLE_VALUE)
    {
        close();
        return -1;
    }

    debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::open[%i:%05.5i]: Thread 0x%8.8lX created with thread function serialListener()\n", m_portNo, m_sid, m_threadId);

    //SetThreadPriority(m_hThread, THREAD_PRIORITY_ABOVE_NORMAL);
    SetThreadPriority(m_hThread, THREAD_PRIORITY_HIGHEST);

    m_sizeToRead = BUF_SIZE;

    m_pBlockingReadBuffer = (U8*) malloc(BLOCKING_READ_BUF_SIZE);
    if(m_pBlockingReadBuffer == NULL)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%i:%05.5i]: Blocking read buffer could not be allocated. Closing...\n", m_portNo, m_sid);
        close();
        return -1;
    }

    m_pBlockingReadBufferActualPosition = m_pBlockingReadBuffer;
    m_blockingReadBytesAvailable = 0;

    switch(WaitForSingleObject(m_hReadyEvent, SERIALDRIVER_TIMEOUT))
    {
    case WAIT_OBJECT_0:
        break;
    case WAIT_TIMEOUT:
    default:
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%i:%05.5i]: Waiting for listener thread failed or timed out. Closing...\n", m_portNo, m_sid);
        close();
        return -1;
    }

    m_isOpen = 1;
    debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::open[%i:%05.5i]: Succeeded...\n", m_portNo, m_sid);
    return 0;
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::close()
+------------------------------------------------------------------------------
| Description : Closing function for the serial driver.
|
| Parameters  : void
|
| Returns     : S8
+----------------------------------------------------------------------------*/
S8 SerialDrv::close(void)
{
    U32 exitCode, ret;

    debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::close[%i:%05.5i]\n", m_portNo, m_sid);

    m_isClosing = 1;
    m_isOpen = 0;

    // Signal serialListener() thread to exit
    if (m_hSerial != INVALID_HANDLE_VALUE)
    {
        SetCommMask(m_hSerial, EVENT_MASK);
        CancelIo(m_hSerial);
        PurgeComm(m_hSerial, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR); //Clean out pending data
    }
    if(m_hStopEvent!=NULL)
    {
        SetEvent(m_hStopEvent);
    }

    ResetEvent(m_hBeginListening);

    if(m_pBlockingReadBuffer!=NULL)
    {
        free(m_pBlockingReadBuffer);
        m_pBlockingReadBuffer = NULL;
    }

    m_blockingReadBytesAvailable = 0;

    if (m_hThread != INVALID_HANDLE_VALUE)
    {
        //Wait for COMM event thread to exit
        WaitForSingleObject(m_hThread, SERIALDRIVER_TIMEOUT); // TODO: Get result

        GetExitCodeThread(m_hThread, &exitCode);
        if (exitCode == STILL_ACTIVE)
        {
            debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::close[%i:%05.5i]: Thread 0x%8.8lX *terminated*\n", m_portNo, m_sid, m_threadId);
            TerminateThread(m_hThread, exitCode);
        }
        CloseHandle(m_hThread);
        m_hThread = INVALID_HANDLE_VALUE;


    }
    if (m_hSerial != INVALID_HANDLE_VALUE)
    {
        ret = CloseHandle(m_hSerial);
        if( ret == 0){
            debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::close[%i:%05.5i]: Could not close serial handle - GetLastError = %i\n", m_portNo, m_sid, GetLastError());
        }
        m_hSerial = INVALID_HANDLE_VALUE;
    }
    if (m_hReadyEvent != NULL)
    {
        CloseHandle(m_hReadyEvent);
        m_hReadyEvent = NULL;
    }
    if (m_hStopEvent != NULL)
    {
        CloseHandle(m_hStopEvent);
        m_hStopEvent = NULL;
    }

    if (m_readOverlapped.hEvent != NULL)
    {
        CloseHandle(m_readOverlapped.hEvent);
        memset(&m_readOverlapped, 0, sizeof(m_readOverlapped));
    }
    if (m_readOverlappedStatus.hEvent != NULL)
    {
        CloseHandle(m_readOverlappedStatus.hEvent);
        memset(&m_readOverlappedStatus, 0, sizeof(m_readOverlappedStatus));
    }
    if (m_writeOverlapped.hEvent != NULL)
    {
        CloseHandle(m_writeOverlapped.hEvent);
        memset(&m_writeOverlapped, 0, sizeof(m_writeOverlapped));
    }

    m_isClosing = 0;
    debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::close[%i:%05.5i] - suceeded\n", m_portNo, m_sid);
    return 0;
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::reset()
+------------------------------------------------------------------------------
| Description : Reset function for the serial driver.
|
| Parameters  : void
|
| Returns     : S8
+----------------------------------------------------------------------------*/
S8 SerialDrv::reset(void)
{
    if(m_isOpen==1)
    {
        m_blockingReadBytesAvailable = 0; //"Empty" local buffer for blocking read
        return PurgeComm(m_hSerial, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR);
    }
    else
    {
        return -1;
    }
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::configure()
+------------------------------------------------------------------------------
| Description : Function for configuring the serial driver.
|
| Parameters  : [IN] baudRate - Configuring the baud rate for the serial driver.
|               [IN] parity - Configuring pariy bit.
|               [IN] stopBits - 1, 1½, or 2 stop bits can be configured.
|               [IN] data - Data bits which can be either 5, 6, 7, or 8 bits.
|
| Returns     : S8
+----------------------------------------------------------------------------*/
S8 SerialDrv::configure(U32 baudRate, U8 parity, U8 stopBits, U8 data)
{
    DCB dcb = { 0 };

    dcb.DCBlength = sizeof(DCB);

    debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::configure[%i:%05.5i]: Baud:%i\n", m_portNo, m_sid, baudRate);

    // Get current configuration of serial communication port.
    if (!GetCommState(m_hSerial, &dcb))
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::configure[%i:%05.5i]: GetCommState() problem. GetLastError() = %i\n", m_portNo, m_sid, GetLastError());
        return -1;
    }

    dcb.fAbortOnError = /*TRUE;*/FALSE;
    dcb.BaudRate = baudRate; // Specify baud rate of communicaiton.
    dcb.ByteSize = data; // Specify byte of size of communication.
    dcb.Parity = parity; // Specify parity of communication.
    dcb.StopBits = stopBits; // Specify stop bits of communication.
    dcb.fOutxCtsFlow = 0;
    dcb.fOutxDsrFlow = 0;
    dcb.fDsrSensitivity = 0;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    // Store the new configuration.
    if (!SetCommState(m_hSerial, &dcb))
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::configure[%i:%05.5i]: SetCommState() problem. GetLastError() = %i\n", m_portNo, m_sid, GetLastError());
        return -1;
    }

    if (!SetupComm(m_hSerial, RX_HW_BUF_SIZE, TX_HW_BUF_SIZE))
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::configure[%i:%05.5i]: SetupComm() problem. GetLastError() = %i\n", m_portNo, m_sid, GetLastError());
        return -1;
    }

    m_baudRate = baudRate;

    return 0;
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::write()
+------------------------------------------------------------------------------
| Description : Write function for the serial driver.
|
| Parameters  :
|
| Returns     : S32
+----------------------------------------------------------------------------*/
S32 SerialDrv::write(U8 *p_buffer, U32 size)
{
    S32 ret,err;
    U32 written = 0;

    //debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::write[%i:%05.5i]: Size: %i\n", m_portNo, m_sid, size);

    if(!m_isOpen){
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::write[%i:%05.5i]: Error - SerialDrv::open not called or not completed successfully\n", m_portNo, m_sid);
        return -1;
    }

    ret = WriteFile(m_hSerial, p_buffer, size, &written, &m_writeOverlapped);
    if(ret == 0)
    {

        err = GetLastError();
        if (err == ERROR_IO_PENDING){
            WaitForSingleObject(m_writeOverlapped.hEvent, INFINITE);
            ResetEvent(m_writeOverlapped.hEvent);
            written = size;
        }
        else
        {
            // Writing has problem.
            debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::write[%i:%05.5i]: Error GetLastError() != ERROR_IO_PENDING - err = %i\n", m_portNo, m_sid, err);
        }
    }

    debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::write[%i:%05.5i]: Wrote: %i\n", m_portNo, m_sid, written);
    return written;
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::registerCallback()
+------------------------------------------------------------------------------
| Description : Register the callback function
|
| Parameters  :
|
| Returns     : void
+----------------------------------------------------------------------------*/
void SerialDrv::registerCallback(T_COM_DRV_CALL_BACK callback)
{
    if(m_sid==0xFFFF)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::registerCallback[%i:%05.5i] makes no sense with no sid supplied - abort\n", m_portNo, m_sid);
        return;
    }
    debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::registerCallback[%i:%05.5i]\n", m_portNo, m_sid);

    m_callBack = callback;
    m_sizeToRead = BUF_SIZE;
    m_callBackRegistered = 1;
    SetEvent(m_hBeginListening); //Tell listener thread to begin reading by itself
    return;
}

/*-----------------------------------------------------------------------------
| Function    : SerialDrv::unregisterCallback()
+------------------------------------------------------------------------------
| Description : Unregister the callback function
|
| Parameters  :
|
| Returns     : S8
+----------------------------------------------------------------------------*/
void SerialDrv::unregisterCallback()
{

    if(m_callBackRegistered == 0)
    {
        return;
    }
    else
    {
        m_callBackRegistered = 0; //This will cause listener thread to fall out of the loop and wait for another begin event

        if(!m_isOpen)
        {
            m_callBack = NULL;
        }
        else
        {
            if(WaitForSingleObject(m_hListenerDone, SERIALDRIVER_TIMEOUT) == WAIT_OBJECT_0){
                //Listener loop has fallen out and it is safe to remove callback
                m_callBack = NULL;
            }
            else
            {
                debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::unregisterCallback[%i:%05.5i]: Error waiting for listener thread to complete\n", m_portNo, m_sid);
            }
        }
    }
    return;
}

/*-----------------------------------------------------------------------------
| Function    : SerialDrv::registerConnectionLostCallback()
+------------------------------------------------------------------------------
| Description : Register the callback function for connection lost
|
| Parameters  :
|
| Returns     : void
+----------------------------------------------------------------------------*/
void SerialDrv::registerConnectionLostCallback(T_COM_DRV_CONNECTION_LOST_CALL_BACK callback)
{
    // This function isn't supported for a serial link, but needs to be here
    // to fullfill the interface, since it's needed for the USB driver
    return;
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::read()
+------------------------------------------------------------------------------
| Description : Serial Read Function
|
| Parameters  :
|
| Returns     : S32
+----------------------------------------------------------------------------*/
S32 SerialDrv::read(U8 *p_buffer, U32 size, U32 timeout)
{
#ifndef USE_BUFFERED_BLOCKING_READ
    //Blocking read only makes sense if we are running without a registered callback
    if(m_callBackRegistered)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read[%i:%05.5i]: Error - read() called while callback is registered\n", m_portNo, m_sid);
        return -1;
    }

    //debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::read[%i:%05.5i]: Size: %i, Timeout %i\n", m_portNo, m_sid, size, timeout);

    if(!m_isOpen){
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read[%i:%05.5i]: Error - SerialDrv::open not called or not completed successfully\n", m_portNo, m_sid);
        return -1;
    }
    if (size == 0)
    {
        Sleep(timeout);
        return 0;
    }
    else return read_hw(p_buffer,size,timeout);
#else
    S32 bytesCopied,ret,bytesToCopy,tmpret;
    U8* pBufferCopy;
    //Blocking read only makes sense if we are running without a registered callback
    if(m_callBackRegistered)
    {
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read[%i:%05.5i]: Error - read() called while callback is registered\n", m_portNo, m_sid);
        return -1;
    }

    //debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::read[%i:%05.5i]: Size: %i, Timeout %i\n", m_portNo, m_sid, size, timeout);

    if(!m_isOpen){
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read[%i:%05.5i]: Error - SerialDrv::open not called or not completed successfully\n", m_portNo, m_sid);
        return -1;
    }

    if (size == 0)
    {
        Sleep(timeout);
        ret = 0;
    }
    else
    {
        pBufferCopy = p_buffer;
        bytesCopied = 0;
        if(size<=m_blockingReadBytesAvailable)
        {
            //We can satisfy the read with the contents of our local buffer
            memcpy(pBufferCopy, m_pBlockingReadBufferActualPosition, size);
            m_blockingReadBytesAvailable-=size;
            m_pBlockingReadBufferActualPosition+=size;
            bytesCopied += size;
            ret = bytesCopied;
        }
        else //We don't have all of the requested bytes.
        {
            if(m_blockingReadBytesAvailable>0){
                //Copy the ones we have.
                memcpy(pBufferCopy, m_pBlockingReadBufferActualPosition, m_blockingReadBytesAvailable);
                m_pBlockingReadBufferActualPosition += m_blockingReadBytesAvailable;
                pBufferCopy += m_blockingReadBytesAvailable;
                bytesCopied += m_blockingReadBytesAvailable;
                m_blockingReadBytesAvailable=0; //We emptied the buffer
            }
            //Now we are out of bytes - issue a read
            //If timeout is specified, we should not wait to read bytes not requested. If more bytes are requested
            //than will fit in local buffer, we should not attempt to fill local buffer. Instead read rest of request
            //directly to supplied buffer and do not update local bytesavailable counter.
            if((timeout>0)||((size-bytesCopied)>=BLOCKING_READ_BUF_SIZE))
            {
                tmpret = read_hw(pBufferCopy,(size-bytesCopied),timeout);
                if(tmpret != -1)
                {
                    bytesCopied += tmpret;
                }
            }
            //With a timeout of 0 and less bytes than BLOCKING_READ_BUF_SIZE remaing to read we can get all that
            //is there without a severe timing penalty (ie. we won't wait).
            else
            {
                tmpret = read_hw(m_pBlockingReadBuffer,BLOCKING_READ_BUF_SIZE,0);
                if(tmpret != -1)
                {
                    m_blockingReadBytesAvailable = tmpret;
                    m_pBlockingReadBufferActualPosition = m_pBlockingReadBuffer;
                }
            }

            if(tmpret == -1)
            {
                //This is a tricky situation. We may already have copied something, but read fails.
                debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read[%i:%05.5i]: Error - read_hw returned -1\n", m_portNo, m_sid);
                if(bytesCopied) ret = bytesCopied; //Return what we already copied, ignore that we had an error
                else ret = -1;                     //Return error
            }
            // Find numbers of bytes to copy to finish operation - then do it
            bytesToCopy=((size-bytesCopied) < m_blockingReadBytesAvailable) ? (size-bytesCopied):m_blockingReadBytesAvailable;

            memcpy(pBufferCopy, m_pBlockingReadBufferActualPosition, bytesToCopy);
            m_blockingReadBytesAvailable-=bytesToCopy;
            m_pBlockingReadBufferActualPosition+=bytesToCopy;
            bytesCopied += bytesToCopy; //We completed and read all
            ret = bytesCopied;
        }
    }
    return ret;
#endif
}


/*-----------------------------------------------------------------------------
| Function    : SerialDrv::read_hw()
+------------------------------------------------------------------------------
| Description : Serial Read Function
|
| Parameters  :
|
| Returns     : S32
+----------------------------------------------------------------------------*/
S32 SerialDrv::read_hw(U8 *p_buffer, U32 size, U32 timeout)
{
    S32 ret = 0;
    S32 err = 0;
    COMMTIMEOUTS timeouts;
    HANDLE hEvents[MAX_NUM_EVENT_HANDLES];
    S32 try_again = 1;

    if(m_hSerial == INVALID_HANDLE_VALUE){
        debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read_hw[%i:%05.5i]: Invalid handle\n", m_portNo, m_sid);
        return -1;
    }

    if(timeout!=m_lastCommTimeout){
        /* Set time-outs for communication. */
        if(timeout == 0)
        { //Return immediately - special configuration according to MSDN
            timeouts.ReadIntervalTimeout = MAXDWORD;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;
        }
        else
        { //Configure reading with supplied total timeout
            timeouts.ReadIntervalTimeout = 0;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = timeout;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;
        }
        do
        {
            try_again--;
            if(!SetCommTimeouts(m_hSerial, &timeouts))
            {
                err = GetLastError();
                debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read_hw[%i:%05.5i]: Setting timeouts failed. Error = %i\n", m_portNo, m_sid, err);
                if(err == ERROR_OPERATION_ABORTED)
                {
                    ResetCommError();
                    try_again++;
                }
                else
                {
                    return -1;
                }
            }

        }while((try_again > 0) && (try_again < 5));
        m_lastCommTimeout = timeout;
    }

    try_again = 1;
    m_sizeToRead = size;
    hEvents[0] = m_hStopEvent;
    hEvents[1] = m_readOverlapped.hEvent;

    do
    {
        try_again--;
        ResetEvent(m_readOverlapped.hEvent);
        if (!ReadFile(m_hSerial, p_buffer, m_sizeToRead, &size, &m_readOverlapped))
        {
            if (GetLastError() == ERROR_IO_PENDING)
            {
                switch(WaitForMultipleObjects(MAX_NUM_EVENT_HANDLES, hEvents, FALSE, INFINITE))
                {
                case WAIT_OBJECT_0:
                    {
                        debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::read_hw[%i:%05.5i]: Stop event received (Thread: 0x%8.8lX)..\n", m_portNo, m_sid, m_threadId);
                        return 0;
                    }
                case WAIT_OBJECT_0 + 1:
                    {
                        if (!GetOverlappedResult(m_hSerial, &m_readOverlapped, &size, FALSE))
                        {
                            // TODO: Check if (GetLastError() == ERROR_OPERATION_ABORTED)
                            err = GetLastError();
                            debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read_hw[%i:%05.5i]: GetOverlappedResult: error = 0x%8.8lX; size = %d (Thread: 0x%8.8lX)\n", m_portNo, m_sid, err, size, m_threadId);
                            if((err == ERROR_OPERATION_ABORTED) || (err = ERROR_IO_INCOMPLETE))
                            {
                                CancelIo(m_hSerial);
                                DWORD bytesLeftToRead;
                                ResetCommError(&bytesLeftToRead);
                                if(bytesLeftToRead) try_again++;
                                //while(!HasOverlappedIoCompleted(&m_readOverlapped));
                            }
                        }

                        ret=size;
                        break;
                    }
                default:
                    {
                        debugPrint(TRACER_LEVEL_CRITICAL,"SerialDrv::read_hw[%i:%05.5i]: Problem with event list..\n", m_portNo, m_sid);
                        break;
                    }

                }
            }
            else
            {
                // TODO: Error handling
                err = GetLastError();
                debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::read_hw[%i:%05.5i]: Readfile error: %i\n", m_portNo, m_sid, err);
                ResetCommError();
                reset();
            }
        }
        else
        {
            //Reading succeeded - update size
            ret=size;
        }
    }while((try_again > 0) && (try_again < 5));

    if(ret==1)
        debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::read_hw[%i:%05.5i]: Req: %i Got: %i, Timeout:%i, Char: 0x%x\n", m_portNo, m_sid, m_sizeToRead, ret, timeout, *p_buffer);
    else
        debugPrint(TRACER_LEVEL_MINOR_INFO, "SerialDrv::read_hw[%i:%05.5i]: Req: %i Got: %i, Timeout:%i\n", m_portNo, m_sid, m_sizeToRead, ret, timeout);
    return ret;
}



S32 SerialDrv::ResetCommError(DWORD* bytesRxLeft, DWORD* bytesTxLeft)
{
    char errStr[200] = {0};
    COMSTAT comStat;
    DWORD dwErrors;
    ClearCommError(m_hSerial, &dwErrors, &comStat);
    strcat(errStr, dwErrors & CE_DNS ? "DNS " : "");
    strcat(errStr, dwErrors & CE_IOE ? "IOE " : "");
    strcat(errStr, dwErrors & CE_OOP ? "OOP " : "");
    strcat(errStr, dwErrors & CE_PTO ? "PTO " : "");
    strcat(errStr, dwErrors & CE_MODE ? "MODE " : "");
    strcat(errStr, dwErrors & CE_BREAK ? "BREAK " : "");
    strcat(errStr, dwErrors & CE_FRAME ? "FRAME " : "");
    strcat(errStr, dwErrors & CE_RXOVER ? "RXOVER " : "");
    strcat(errStr, dwErrors & CE_TXFULL ? "TXFULL " : "");
    strcat(errStr, dwErrors & CE_OVERRUN ? "OVERRUN " : "");
    strcat(errStr, dwErrors & CE_RXPARITY ? "RXPARITY " : "");
    debugPrint(TRACER_LEVEL_CRITICAL, "SerialDrv::ResetCommError[%i:%05.5i] error = 0x%8.8lX (Thread: 0x%8.8lX); errorname = %s; %i bytes waiting for read...\n", m_portNo, m_sid, dwErrors, m_threadId, errStr, comStat.cbInQue);
    if(bytesRxLeft) *bytesRxLeft = comStat.cbInQue;
    if(bytesTxLeft) *bytesTxLeft = comStat.cbOutQue;
    return dwErrors;
}

/*-----------------------------------------------------------------------------
| Function    : serialListener()
+------------------------------------------------------------------------------
| Description : Serial Listener Function
|
| Parameters  : void *p_drvObj
|
| Returns     : unsigned int
+----------------------------------------------------------------------------*/
unsigned int __stdcall serialListener(void *p_drvObj)
{
    U32 eventMask;
    DWORD bytesLeftToRead;
    DWORD ret;
    U8 buf[BUF_SIZE];
    COMMTIMEOUTS timeouts;
    U32 size;
    BOOL retry = FALSE;
    SerialDrv *p_drv = (SerialDrv *) p_drvObj;
    HANDLE hEvents[MAX_NUM_EVENT_HANDLES];
    hEvents[0] = p_drv->m_hStopEvent;
    SetEvent(p_drv->m_hReadyEvent);

    while (p_drv->m_isClosing == 0)
    {
        hEvents[1] = p_drv->m_hBeginListening;
        switch(WaitForMultipleObjects(MAX_NUM_EVENT_HANDLES, hEvents, FALSE, INFINITE))
        {
        case WAIT_OBJECT_0:
            {
                p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Stop event received 1(Thread: 0x%8.8lX)...\n", p_drv->m_portNo,  p_drv->m_sid, p_drv->m_threadId);
                return 0;
            }
        case WAIT_OBJECT_0 + 1:
            {
                p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: m_hBeginListening event triggered - beginning read loop...\n", p_drv->m_portNo,  p_drv->m_sid);

                /* Set time-outs for communication. */

                // Specify time-out between characters for receiving.
                // Just tell it to wait the shortest possible time. Will be enough for it to keep latching bytes in a continous stream and stop
                // when the stream is interrupted. At 921600 baud 1 ms is approximately 90 bytes...
                timeouts.ReadIntervalTimeout = 1;
                // Specify value that is multiplied
                // by the requested number of bytes to be read.
                timeouts.ReadTotalTimeoutMultiplier = 0;
                // Specify value is added to the product of the
                // ReadTotalTimeoutMultiplier member
                timeouts.ReadTotalTimeoutConstant = 1000;
                // Specify value that is multiplied
                // by the requested number of bytes to be sent.
                timeouts.WriteTotalTimeoutMultiplier = 0;
                // Specify value is added to the product of the
                // WriteTotalTimeoutMultiplier member
                timeouts.WriteTotalTimeoutConstant = 0;

                if (!SetCommTimeouts(p_drv->m_hSerial, &timeouts))
                {
                    p_drv->debugPrint(TRACER_LEVEL_CRITICAL,"serialListener[%i:%05.5i]: Setting timeouts failed\n", p_drv->m_portNo, p_drv->m_sid);
                    return -1;
                }
                break;
            }
        default:
            {
                p_drv->debugPrint(TRACER_LEVEL_CRITICAL,"serialListener[%i:%05.5i]: Problem with event list 0..\n", p_drv->m_portNo, p_drv->m_sid);
                return -1;
                break;
            }
        }
        while(p_drv->m_callBackRegistered)  //Reading in this thread makes sense only if a callback is registered
        {
            hEvents[1] = p_drv->m_readOverlappedStatus.hEvent;
            ResetEvent(p_drv->m_readOverlappedStatus.hEvent);
            //SetCommMask(p_drv->m_hSerial, EVENT_MASK);
            eventMask = 0;
            p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: WaitCommEvent about to begin..\n", p_drv->m_portNo,  p_drv->m_sid);
            if (!WaitCommEvent(p_drv->m_hSerial, &eventMask, &p_drv->m_readOverlappedStatus))
            {
                switch(GetLastError ()){
                case ERROR_IO_PENDING:
                    p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Wait1..\n", p_drv->m_portNo,  p_drv->m_sid);

                    switch(WaitForMultipleObjects(MAX_NUM_EVENT_HANDLES, hEvents, FALSE, INFINITE))
                    {
                    case WAIT_OBJECT_0:
                        {
                            CancelIo(p_drv->m_hSerial);
                            p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Stop event received 1(Thread: 0x%8.8lX)..\n", p_drv->m_portNo,  p_drv->m_sid, p_drv->m_threadId);
                            return 0;
                        }
                    case WAIT_OBJECT_0 + 1:
                        {
                            if (!GetOverlappedResult(p_drv->m_hSerial, &p_drv->m_readOverlappedStatus, &size, FALSE))
                            {
                                // TODO: Check if (GetLastError() == ERROR_OPERATION_ABORTED)
                                p_drv->debugPrint(TRACER_LEVEL_CRITICAL, "serialListener[%i:%05.5i]: GetOverlappedResult(1): error = 0x%8.8lX; eventMask = 0x%8.8lX; size = %d (Thread: 0x%8.8lX)\n", p_drv->m_portNo,  p_drv->m_sid, GetLastError(), eventMask, size, p_drv->m_threadId);
                            }
                            p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO,"serialListener[%i:%05.5i]: Done wait1..EvMask: 0x%x\n", p_drv->m_portNo, p_drv->m_sid, eventMask);
                            break;
                        }
                    default:
                        {
                            p_drv->debugPrint(TRACER_LEVEL_CRITICAL,"serialListener[%i:%05.5i]: Problem with event list 1..\n", p_drv->m_portNo, p_drv->m_sid);
                            break;
                        }
                    }
                    break;

                case ERROR_OPERATION_ABORTED:
                    {
                        p_drv->reset();
                        p_drv->ResetCommError();
                        p_drv->debugPrint(TRACER_LEVEL_CRITICAL,"serialListener[%i:%05.5i]: WaitCommEvent ERROR_OPERATION_ABORTED\n", p_drv->m_portNo,  p_drv->m_sid);
                        break;
                    }
                default:
                    {
                        // TODO: Error handling
                        p_drv->debugPrint(TRACER_LEVEL_CRITICAL,"serialListener[%i:%05.5i]: CommEv err: %i - exiting thread", p_drv->m_portNo,  p_drv->m_sid, GetLastError());
                        return -1;
                    }
                }
            }
            p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: WaitCommEvent done..EvMask: 0x%x\n", p_drv->m_portNo,  p_drv->m_sid, eventMask);

            if (p_drv->m_isClosing == 1)
            {
                p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Closing (Thread: 0x%8.8lX)..\n", p_drv->m_portNo,  p_drv->m_sid, p_drv->m_threadId);
                return 0;
            }

            if(eventMask & EV_ERR)
            {
                //p_drv->reset();
                p_drv->ResetCommError();

            }

            hEvents[1] = p_drv->m_readOverlapped.hEvent;
            do
            {
                ResetEvent(p_drv->m_readOverlapped.hEvent);
                SetLastError(0);
                if (!ReadFile(p_drv->m_hSerial, buf, p_drv->m_sizeToRead, &size, &p_drv->m_readOverlapped))
                {
                    if (GetLastError() == ERROR_IO_PENDING)
                    {
                        //OutputDebugString("serialListener: Waiting 2..");
                        p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Wait2..\n", p_drv->m_portNo,  p_drv->m_sid);
                        switch(WaitForMultipleObjects(MAX_NUM_EVENT_HANDLES, hEvents, FALSE, INFINITE))
                        {
                        case WAIT_OBJECT_0:
                            {
                                CancelIo(p_drv->m_hSerial);
                                p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Stop event received 2(Thread: 0x%8.8lX)..\n", p_drv->m_portNo,  p_drv->m_sid, p_drv->m_threadId);
                                return 0;
                            }
                        case WAIT_OBJECT_0 + 1:
                            {
                                if (!GetOverlappedResult(p_drv->m_hSerial, &p_drv->m_readOverlapped, &size, FALSE))
                                {
                                    ret = GetLastError();
                                    if(ret==ERROR_OPERATION_ABORTED)
                                    {
                                        //Someone aborted the read (probably a call to close()) - fall out of read loop (do not retry)
                                        bytesLeftToRead = 0;
                                    }
                                    else
                                    {
                                        //We saw another error - this could be an error condition on the serial port.
                                        //Cancel current operation, reset error and retry
                                        p_drv->debugPrint(TRACER_LEVEL_CRITICAL, "serialListener[%i:%05.5i]: GetOverlappedResult(2): error = 0x%8.8lX; eventMask = 0x%8.8lX; size = %d (Thread: 0x%8.8lX)\n", p_drv->m_portNo,  p_drv->m_sid, ret, eventMask, size, p_drv->m_threadId);
                                        CancelIo(p_drv->m_hSerial); //Reset operation and hope it is OK to read the rest in the next iteration
                                        p_drv->ResetCommError(&bytesLeftToRead);
                                    }
                                    if(bytesLeftToRead) retry = TRUE;
                                }
                                else //Everything went fine during our wait for the overlapped operation
                                {
                                    retry = FALSE;
                                }
                                //OutputDebugString("..done ");
                                break;
                            }
                        default:
                            {
                                p_drv->debugPrint(TRACER_LEVEL_CRITICAL,"serialListener[%i:%05.5i]: Problem with event list 2..\n", p_drv->m_portNo, p_drv->m_sid);
                                retry = TRUE;
                                break;
                            }

                        }
                    }
                    else
                    {
                        // TODO: Error handling
                        p_drv->debugPrint(TRACER_LEVEL_CRITICAL, "serialListener[%i:%05.5i]: Readfile error: %i\n", p_drv->m_portNo,  p_drv->m_sid, GetLastError());
                        p_drv->ResetCommError(&bytesLeftToRead);
                        if(bytesLeftToRead) retry = TRUE;
                        break;
                    }
                }
                else //Readfile() completed with no errors
                {
                    retry = FALSE;
                }
                p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Got:%i\n", p_drv->m_portNo,  p_drv->m_sid, size);
                if (size > 0)
                {
                    p_drv->m_callBack(p_drv->m_sid, buf, size);
                }
                // Poll for stop event (non blocking)
                if(WaitForSingleObject(p_drv->m_hStopEvent, 0) == WAIT_OBJECT_0)
                {
                    p_drv->debugPrint(TRACER_LEVEL_MINOR_INFO, "serialListener[%i:%05.5i]: Stop event set. Exiting thread 0x%8.8lX\n", p_drv->m_portNo,  p_drv->m_sid, p_drv->m_threadId);
                    return 0;
                }
            } while (size > 0 || retry);
        }
        //Signal unregisterCallback function that it is safe to remove callback function
        SetEvent(p_drv->m_hListenerDone);
    }

    return 0;
}


/*==== END OF FILE ===========================================================*/

