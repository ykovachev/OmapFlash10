/**
 * @file UsbDriver.cpp
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
 * This file defines a user interface for the WinUSB driver.
 */


/*==== INCLUDES ============================================================*/
#include "stdafx.h"
#include "types.h"
#include "PortEnumerator.h"
#include "UsbDriver.h"


/*==== MACROS ===============================================================*/

/*==== CONSTS ===============================================================*/

/*==== TYPES ================================================================*/

/*==== GLOBALS ==============================================================*/

/*==== PUBLIC FUNCTIONS =====================================================*/

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::UsbDriver()
+------------------------------------------------------------------------------
| Description : USB Driver constructor
|
| Parameters  : sid - Session Id number
|
| Returns     : N/A
+----------------------------------------------------------------------------*/
UsbDriver::UsbDriver(U16 sid)
: sessionId(sid), isOpen(false), isClosing(false),
rxComplete(false), rxCallback(NULL), readyEvent(INVALID_HANDLE_VALUE),
shutdownEvent(INVALID_HANDLE_VALUE), listener(INVALID_HANDLE_VALUE), usbDevice(INVALID_HANDLE_VALUE),
winUsbHandle(INVALID_HANDLE_VALUE), readFifoPtr(0), readFifoBytesAvailable(0), connectionLostCallback(NULL)
{
    readFifo = new U8[2*USB_BULK_PACKET_SIZE];
	driverErr = DRV_ERR_EXPECTED;
    initDebugLevel();
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::~UsbDriver()
+------------------------------------------------------------------------------
| Description : USB Driver destructor
|
| Parameters  : N/A
|
| Returns     : N/A
+----------------------------------------------------------------------------*/
UsbDriver::~UsbDriver()
{
    if (isOpen)
        close();
    delete [] readFifo;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::open()
+------------------------------------------------------------------------------
| Description : Opens the USB driver
|
| Parameters  : port     - Not used. Only for Interface Compliance
|               baudrate - Not used. Only for Interface Compliance
|               parity   - Not used. Only for Interface Compliance
|               stopBits - Not used. Only for Interface Compliance
|               data     - Not used. Only for Interface Compliance
|
| Returns     : return code
+----------------------------------------------------------------------------*/
S8 UsbDriver::open(U8 port, U32 baudRate, U8 parity, U8 stopBits, U8 data)
{
    debugPrint(5, "UsbDriver::open[%05.5i]: Port:%i, baud:%i\n", sessionId, port, baudRate);

	

    if (isOpen)
    {
        debugPrint(1, "UsbDriver::open[%05.5i]: Already opened - doing nothing (port:%i)\n", sessionId, port);
        return COM_DRIVER_RET_ERR;
    }

    portNum = port;
    usbDevice = getDeviceHandle(port);
    
    if (INVALID_HANDLE_VALUE == usbDevice)
    {
        debugPrint(5, "UsbDriver::open[%05.5i]: Failed to open usb device (port:%i)\n", sessionId, port);
        close();
        return COM_DRIVER_RET_ERR;
    }	

	bool res = initializeDevice();

    if (res == false)
    {
        debugPrint(5, "UsbDriver::open[%05.5i]: Failed to initialize WinUsb (port:%i)\n", sessionId, port);
        close();
        return COM_DRIVER_RET_ERR;
    }	  

    if (NULL != rxCallback)
    {
        readyEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
        shutdownEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

        if ((INVALID_HANDLE_VALUE == readyEvent) || (INVALID_HANDLE_VALUE == shutdownEvent))
        {
            debugPrint(1, "UsbDriver::open[%05.5i]: Failed to create necessary event objects (port:%i)\n", sessionId, port);
            close();
            return COM_DRIVER_RET_ERR;
        }

        listener = (HANDLE) _beginthreadex(NULL,
                                           0,
                                           usbListener,
                                           (void *) this,
                                           0,
                                           (unsigned int *) &threadId);

        if (INVALID_HANDLE_VALUE == listener)
        {
            debugPrint(1, "UsbDriver::open[%05.5i]: Failed to create listener thread (port:%i)\n", sessionId, port);
            close();
            return COM_DRIVER_RET_ERR;
        }
		DWORD res =	WaitForSingleObject(readyEvent, COM_DRIVER_DEFAULT_TIMEOUT);
        switch(res)
        {
        case WAIT_OBJECT_0:
            break;
        case WAIT_TIMEOUT:
        default:
            debugPrint(1, "UsbDriver::open[%05.5i]: Waiting for listener thread failed or timed out (port:%i)\n", sessionId, port);
            close();
            return COM_DRIVER_RET_ERR;
        }
    }

    isOpen = true;	

    return COM_DRIVER_RET_OK;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::initializeDevice()
+------------------------------------------------------------------------------
| Description : Initiliza WinUSB device
|
| Returns     : return true on success, false otherwise
+----------------------------------------------------------------------------*/
bool UsbDriver::initializeDevice()
{
	unsigned long timeoutIn = 400;	
	unsigned long timeoutOut = 500;	
	unsigned long length = 0, timeOut = 0;

	unsigned long opt = 0, err;
	

	if (usbDevice == INVALID_HANDLE_VALUE)
    {
        return false;
    }
	bool res = WinUsb_Initialize(usbDevice, &winUsbHandle);
	
	if (!res)
    {
		if (strcmp(driverName, OLD_DRIVER) == 0)
		{
			//strcpy_s(driverError, 200, "OmapFlash driver was not found, please install OmapFlash driver");
			driverErr = DRV_ERR_OLD_DRIVER;
		}
		else
		{
			//strcpy_s(driverError, 200, "Previous driver CSSTUSB was detected, OmapFlash driver needs to be updated");
			driverErr = DRV_ERR_WRONG_DRIVER;
		}
		debugPrint(1, "UsbDriver::initializeDevice[%05.5i]: WinUSB could not be initialized\n", sessionId);
		return false;
	}
	
	//  Get the transfer type, endpoint number, and direction for the interface's
	//  bulk and interrupt endpoints.
	USB_INTERFACE_DESCRIPTOR interfaceDescriptor;
	WINUSB_PIPE_INFORMATION  pipe;

	ZeroMemory(&interfaceDescriptor, sizeof(USB_INTERFACE_DESCRIPTOR));
	ZeroMemory(&pipe, sizeof(WINUSB_PIPE_INFORMATION));

	res = WinUsb_QueryInterfaceSettings(winUsbHandle, 0, &interfaceDescriptor);
	if (res)
    {
        for (int index = 0; index < interfaceDescriptor.bNumEndpoints; index++)
        {
            res = WinUsb_QueryPipe(winUsbHandle, 0, index, &pipe);

            if (res)
            {
                if (pipe.PipeType == UsbdPipeTypeControl)
                {
                    debugPrint(3, "UsbDriver::initializeDevice[%05.5i]: Endpoint index: %d Pipe type: Control Pipe ID: %d\n", 
						sessionId, index, pipe.PipeType, pipe.PipeId);					
                }
                if (pipe.PipeType == UsbdPipeTypeIsochronous)
                {
                    debugPrint(3, "UsbDriver::initializeDevice[%05.5i]: Endpoint index: %d Pipe type: Isochronous Pipe ID: %d\n", 
						sessionId, index, pipe.PipeType, pipe.PipeId);
                }
                if (pipe.PipeType == UsbdPipeTypeBulk)
                {
                    if (USB_ENDPOINT_DIRECTION_IN(pipe.PipeId))
                    {
                        debugPrint(3, "UsbDriver::initializeDevice[%05.5i]: Endpoint index: %d Pipe type: Bulk Pipe ID: %d\n",
							sessionId, index, pipe.PipeType, pipe.PipeId);
                        bulkInPipe = pipe.PipeId;
						res = WinUsb_SetPipePolicy (winUsbHandle, bulkInPipe, PIPE_TRANSFER_TIMEOUT, sizeof(timeoutIn), &timeoutIn);
						if (!res)
						{	
							err = GetLastError();		
						}							
						
                    }
                    if (USB_ENDPOINT_DIRECTION_OUT(pipe.PipeId))
                    {
                        debugPrint(3, "UsbDriver::initializeDevice[%05.5i]: Endpoint index: %d Pipe type: Bulk Pipe ID: %d\n",
							sessionId, index, pipe.PipeType, pipe.PipeId);
                        bulkOutPipe = pipe.PipeId;
						res = WinUsb_SetPipePolicy (winUsbHandle, bulkOutPipe, PIPE_TRANSFER_TIMEOUT,
								sizeof (timeoutOut), &timeoutOut);
						res = WinUsb_SetPipePolicy(winUsbHandle, bulkOutPipe,
                              IGNORE_SHORT_PACKETS, sizeof(opt), &opt);						
                    }
                }
                if (pipe.PipeType == UsbdPipeTypeInterrupt)
                {
                    debugPrint(3, "UsbDriver::initializeDevice[%05.5i]: Endpoint index: %d Pipe type: Interrupt Pipe ID: %d\n",
							sessionId, index, pipe.PipeType, pipe.PipeId);					
                }
            }
            else
            {
                continue;
            }
        }
    }
	else
	{
		return false;
	}	
	
	
	return true;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::close()
+------------------------------------------------------------------------------
| Description : Closing function for the USB driver.
|
| Parameters  : void
|
| Returns     : S8
+----------------------------------------------------------------------------*/
S8 UsbDriver::close(void)
{
    debugPrint(5, "UsbDriver::close[%05.5i] (port:%i)\n", sessionId, portNum);

    if (!isOpen)
    {
        debugPrint(5, "UsbDriver::close[%05.5i]: Close called but driver is not open....! (port:%i)\n", sessionId, portNum);
        return COM_DRIVER_RET_OK;
    }	
	
    if (INVALID_HANDLE_VALUE != listener)
    {
        isClosing = true;

        SetEvent(shutdownEvent);
       
		if (INVALID_HANDLE_VALUE != usbDevice)
        {            
            reset();
        }

        WaitForSingleObject(listener, 2*COM_DRIVER_DEFAULT_TIMEOUT);

        DWORD exitCode;
        GetExitCodeThread(listener, &exitCode);
        if (STILL_ACTIVE == exitCode)
        {
            debugPrint(1, "UsbDriver::close[%05.5i]: Failed to close thread properly - Terminating it...! (port:%i)\n", sessionId, portNum);
            TerminateThread(listener, exitCode);
        }
        CloseHandle(listener);
        isClosing = false;
    }
    if (INVALID_HANDLE_VALUE != readyEvent)
        CloseHandle(readyEvent);
    if (INVALID_HANDLE_VALUE != shutdownEvent)
        CloseHandle(shutdownEvent); 	
	if (INVALID_HANDLE_VALUE != winUsbHandle)
	{
		WinUsb_Free(winUsbHandle);
	}
	if (INVALID_HANDLE_VALUE != usbDevice)
        CloseHandle(usbDevice);
    isOpen = false;   
	
    return COM_DRIVER_RET_OK;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::reset()
+------------------------------------------------------------------------------
| Description : USB Reset Function
|
| Parameters  : None
|
| Returns     : S8
+----------------------------------------------------------------------------*/
S8 UsbDriver::reset()
{
    bool res;
    	
	WinUsb_AbortPipe(winUsbHandle, bulkInPipe);
	WinUsb_AbortPipe(winUsbHandle, bulkOutPipe);	
	res = true;

    readFifoPtr = 0;
    readFifoBytesAvailable = 0;
    
	return res;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::configure()
+------------------------------------------------------------------------------
| Description : USB Reset Function
|
| Parameters  :
|
| Returns     : S8
+----------------------------------------------------------------------------*/
S8 UsbDriver::configure(U32 baudRate, U8 parity, U8 stopBits, U8 data)
{
    return COM_DRIVER_RET_OK;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::write()
+------------------------------------------------------------------------------
| Description : USB write Function
|
| Parameters  :
|
| Returns     : S8
+----------------------------------------------------------------------------*/
S32 UsbDriver::write(U8 *buffer, U32 size)
{
    unsigned long bytesWritten = 0;
	DWORD error;
    
    if (!isOpen)
    {
        debugPrint(1, "UsbDriver::write[%05.5i]: Cannot issue a write when driver is not opened.\n", sessionId);
        return COM_DRIVER_RET_ERR;
    }

	bool res = true;

	res = WinUsb_WritePipe(winUsbHandle, bulkOutPipe, buffer, size, &bytesWritten, 0);
	if(!res)
	{
		error = GetLastError();        
		debugPrint(1, "UsbDriver::write[%05.5i]: WriteFile failed. Err %i (port:%i)\n", sessionId, error, portNum);
		return 0;
	}	
	    
	debugPrint(3, "UsbDriver::write[%05.5i]: Wrote: %i (port:%i)\n", sessionId, bytesWritten, portNum);
    return bytesWritten;
}


/*-----------------------------------------------------------------------------
| Function    : UsbDriver::read()
+------------------------------------------------------------------------------
| Description : USB Read Function
|
| Parameters  :
|
| Returns     : S32
+----------------------------------------------------------------------------*/
S32 UsbDriver::read(U8 *buffer, U32 size, U32 timeout)
{
    if (rxCallback != NULL)
    {
        debugPrint(1, "UsbDriver::read[%05.5i]: Cannot issue a read while running in Callback mode. (port:%i)\n", sessionId, portNum);
        return COM_DRIVER_RET_ERR;
    }

    if (!isOpen)
    {
        debugPrint(1, "UsbDriver::read[%05.5i]: Cannot issue a read when driver is not opened (port:%i).\n", sessionId, portNum);
        return COM_DRIVER_RET_ERR;
    }

    if (0 == size)
    {
        return 0;
    }

    if (0 == timeout)
        timeout = USB_MINIMUM_TIMEOUT_VALUE;

    return readHw(buffer, size, timeout);
}


/*-----------------------------------------------------------------------------
| Function    : UsbDriver::registerCallback()
+------------------------------------------------------------------------------
| Description : Register Callback Function
|
| Parameters  :
|
| Returns     : void
+----------------------------------------------------------------------------*/
void UsbDriver::registerCallback(T_COM_DRV_CALL_BACK callback)
{
    if (isOpen)
    {
        debugPrint(1, "UsbDriver::registerCallback[%05.5i]: Failed to register callback because driver is open (port:%i).\n", sessionId, portNum);
        return;
    }
    rxCallback = callback;
    return;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::unregisterCallback()
+------------------------------------------------------------------------------
| Description : Unregister function
|
| Parameters  : None
|
| Returns     : void
+----------------------------------------------------------------------------*/
void UsbDriver::unregisterCallback()
{
    if (isOpen)
    {
        debugPrint(1, "UsbDriver::unregisterCallback[%05.5i]: Failed to unregister callback because driver is open (port:%i).\n", sessionId, portNum);
        return;
    }
    rxCallback = NULL;
    return;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::registerConnectionLostCallback()
+------------------------------------------------------------------------------
| Description : Register Callback Function for Connection Lost
|
| Parameters  :
|
| Returns     : void
+----------------------------------------------------------------------------*/
void UsbDriver::registerConnectionLostCallback(T_COM_DRV_CONNECTION_LOST_CALL_BACK callback)
{
    if (isOpen)
    {
        debugPrint(1, "UsbDriver::registerConnectionLostCallback[%05.5i]: Failed to register callback because driver is open (port:%i).\n", sessionId, portNum);
        return;
    }
    connectionLostCallback = callback;
    return;
}


/*==== PRIVATE FUNCTIONS ====================================================*/

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::getDeviceHandle()
+------------------------------------------------------------------------------
| Description : get the device handle for an USB device.
|
| Parameters  : port, only for interface compliance with PortEnumerator
|
| Returns     : S32
+----------------------------------------------------------------------------*/
HANDLE UsbDriver::getDeviceHandle(U8 port)
{
    HANDLE h;
    char usbDesc[256];
    PortEnumerator* enumerator = new PortEnumerator();

    if (0 != enumerator->UsbMapPortToDescriptor(usbDesc, port))
    {
        delete enumerator;
        return INVALID_HANDLE_VALUE;
    }    

    h = CreateFile(usbDesc,
                   GENERIC_WRITE | GENERIC_READ,
                   FILE_SHARE_WRITE | FILE_SHARE_READ,
                   NULL,
                   OPEN_EXISTING,
                   FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                   //FILE_ATTRIBUTE_NORMAL,
                   NULL);
	
	char deviceKeyName[200];
	
	getDeviceKeyName(usbDesc, deviceKeyName);
	getDriverName(deviceKeyName);

    delete enumerator;
    return h;
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::getDeviceKeyName()
+------------------------------------------------------------------------------
| Description : Gets the registry key associated with the attached device.
|
| Parameters  : char usbPath[] - String with the full system path for the attached device
|				char deviceKeyName[] - A string to save the name of the registry key
|										for the device
|
| Returns     : 
+----------------------------------------------------------------------------*/
void UsbDriver::getDeviceKeyName(char usbPath[], char deviceKeyName[])
{
	char *aux;

	// first we copied the root directories for the key
	strcpy_s(deviceKeyName, 200, DRIVER_KEY_BASE);
	aux = strtok(usbPath, "#");
	// the first part is discarded because is not useful
	aux = strtok(NULL, "#");
	// the next part should be used
	strcat_s(deviceKeyName, 200, aux);
	strcat_s(deviceKeyName, 200, "\\");
	aux = strtok(NULL, "#");
	strcat_s(deviceKeyName, 200, aux);
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::getDriverName()
+------------------------------------------------------------------------------
| Description : Gets the name of the driver associated with the attached device.
|
| Parameters  : char deviceKeyName[] - A string to save the name of the registry key
|										for the device
|
| Returns     : 
+----------------------------------------------------------------------------*/
void UsbDriver::getDriverName(char deviceKeyName[])
{
	long result;
	HKEY key;
	char buf[255] = {0};
	unsigned long type = 0;
	unsigned long bufSize = sizeof(buf);

	strcpy(driverName, "UKNOWN");

	result = RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT(deviceKeyName), 0, KEY_READ, &key);
	if (result == ERROR_SUCCESS)
	{
		if (RegQueryValueEx(key, "Service", 0, &type, (BYTE *)buf, &bufSize) == ERROR_SUCCESS)
		{
			strcpy(driverName, buf);
		}
	}
}

/*-----------------------------------------------------------------------------
| Function    : UsbDriver::readHw()
+------------------------------------------------------------------------------
| Description : USB Read from Hw wrapper function.
|
| Parameters  :
|
| Returns     : S32
+----------------------------------------------------------------------------*/
S32 UsbDriver::readHw(U8 *buffer, U32 size, U32 timeout)
{    
    bool success;
	unsigned long bytesRead = 0;

	success = WinUsb_ReadPipe(winUsbHandle, bulkInPipe, buffer, size, &bytesRead, 0);
	if (!success)
	{
		debugPrint(1, "UsbDriver::read[%05.5i]: ReadFile failed (port:%i)\n", sessionId, portNum);
		return 0;
	}	
    debugPrint(3, "UsbDriver::readHW[%05.5i]: Read: %i (port:%i)\n", sessionId, bytesRead, portNum);
    return bytesRead;
}


/*-----------------------------------------------------------------------------
| Function    : usbListener()
+------------------------------------------------------------------------------
| Description : USB Listener Function
|
| Parameters  : void *lpParam - Pointer to a UsbDriver object.
|
| Returns     : unsigned int
+----------------------------------------------------------------------------*/
unsigned int __stdcall usbListener(void *drvObj)
{
    unsigned long bytesRead;
	UsbDriver *drv = (UsbDriver *) drvObj;
	
    drv->debugPrint(3, "UsbDriver::usbListener[%05.5i]: Listener thread started (port:%i)\n", drv->sessionId, drv->portNum);
    SetEvent(drv->readyEvent);

    U8 *rxBuf = new U8[USB_RX_BUF_SIZE];
    while (!drv->isClosing)
    {
        bytesRead = 0;
				
		if (!WinUsb_ReadPipe(drv->winUsbHandle, drv->bulkInPipe, rxBuf, USB_RX_BUF_SIZE, &bytesRead, 0))
        {
            DWORD error = GetLastError();
			
			drv->debugPrint(1, "UsbDriver::usbListener[%05.5i]: Read from device failed. Err: %i (port:%i)\n", drv->sessionId, error, drv->portNum);			
			if (ERROR_IO_PENDING == error)
            {
				int res = WaitForSingleObject(drv->shutdownEvent, INFINITE);
				if (res == WAIT_OBJECT_0)
				{
					// a shutdown event was signaled
					drv->debugPrint(1, "UsbDriver::usbListener[%05.5i]: shutdownEvent received. (port:%i)\n", drv->sessionId, drv->portNum);			
					bytesRead = 0;
				}
				break;
			}			
            // In case of a error code of ERROR_BAD_COMMAND, the connection is lost and we need to signal the calling layer
            if ( (ERROR_BAD_COMMAND == error) && (drv->connectionLostCallback) )
			{
				drv->connectionLostCallback(drv->sessionId);
				break; 
			}
			/*FILE* file;
			fopen_s( &file, "omapflashlog.log", "a+" );
			fprintf( file, "Error while reading %d\n", error);
			fclose( file );*/
			 						
        }
        if (bytesRead)
        {			
            drv->debugPrint(3, "UsbDriver::usbListener[%05.5i]: Read: %i (port:%i)\n", drv->sessionId, bytesRead, drv->portNum);
            drv->rxCallback(drv->sessionId, rxBuf, bytesRead);
        }
		
    }
	

    delete [] rxBuf;	
	
    drv->debugPrint(3, "UsbDriver::usbListener[%05.5i]: Listener thread ending (port:%i).\n", drv->sessionId, drv->portNum);
    return COM_DRIVER_RET_OK;
}

/*==== END OF FILE ===========================================================*/

