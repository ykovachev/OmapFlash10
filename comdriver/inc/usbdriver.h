/**
 * @file usbdriver.h
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
 * This header file defines the USB user space Driver class it uses WinUSB
 * the interface was not modified so it can be compatible with existing code 
 * for CSST USB
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef __USBDRIVER_H__
#define __USBDRIVER_H__

/*==== INCLUDES =============================================================*/
#include "stdafx.h"
#include "types.h"
#include "ComDriver.h"

// Include WinUSB headers
#include <winusb.h>
#include <Usb100.h>
#include <Setupapi.h>

/*==== MACROS ===============================================================*/
#define USB_RX_BUF_SIZE                 512
#define USB_BULK_PACKET_SIZE            512
#define USB_MINIMUM_TIMEOUT_VALUE       10
#define DRIVER_KEY_BASE	"SYSTEM\\CurrentControlSet\\Enum\\USB\\"
#define OLD_DRIVER "CSSTUSB"

#define DRV_ERR_EXPECTED		0
#define DRV_ERR_NO_DRIVER		1
#define DRV_ERR_WRONG_DRIVER	2
#define	DRV_ERR_OLD_DRIVER		3

// Linked libraries
#pragma comment (lib , "setupapi.lib" )
#pragma comment (lib , "winusb.lib" )

/*==== CONSTS ===============================================================*/

/*==== TYPES ================================================================*/

/*==== EXPORTS ==============================================================*/

class API UsbDriver : public ComDriver
{
public:
    UsbDriver(U16 sid = 0);
    ~UsbDriver();
    S8 open(U8 port, U32 baudRate, U8 parity, U8 stopBits, U8 data);
    S8 close(void);
    S8 reset(void);
    S8 configure(U32 baudRate, U8 parity, U8 stopBits, U8 data);
    S32 write(U8 *buffer, U32 size);
    S32 read(U8 *buffer, U32 size, U32 timeout);
    void registerCallback(T_COM_DRV_CALL_BACK callback);
    void unregisterCallback();
    void registerConnectionLostCallback(T_COM_DRV_CONNECTION_LOST_CALL_BACK callback);
	U16 sid() { return sessionId; }
	int getDrvErr() { return driverErr; }
private:	
    U8 portNum;
    U16 sessionId, threadId;
    bool isOpen;
    bool isClosing;
    bool rxComplete;
    U8 *readFifo;
    U32 readFifoPtr;
    U32 readFifoBytesAvailable;
    HANDLE usbDevice;
    HANDLE listener;
    HANDLE readyEvent;
    HANDLE shutdownEvent;
	char driverName[50];
	int driverErr;
    unsigned char bulkInPipe;
	unsigned char bulkOutPipe;

	WINUSB_INTERFACE_HANDLE winUsbHandle;
   
	T_COM_DRV_CALL_BACK rxCallback;
    T_COM_DRV_CONNECTION_LOST_CALL_BACK connectionLostCallback;  
	HANDLE getDeviceHandle(U8 port);
	void getDriverName(char deviceKeyName[]);
	void getDeviceKeyName(char usbPath[], char deviceKeyName[]);
    S32 readHw(U8 *buffer, U32 size, U32 timeout);

	// to use for WinUSB
	bool initializeDevice();

    friend unsigned int __stdcall usbListener(void *drvObj);
protected:

};

#endif /* __USBDRIVER_H__ */

/*==== END OF FILE ==========================================================*/

