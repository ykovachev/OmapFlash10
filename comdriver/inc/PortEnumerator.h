/**
 * @file PortEnumerator.h
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
 *  This header file defines the PortEnumerator class
 */

/*==== DECLARATION CONTROL =================================================*/
#ifndef __CSST_PORT_ENUMERATOR_H__
#define __CSST_PORT_ENUMERATOR_H__

/*==== INCLUDES ============================================================*/
#include "types.h"
#include <vector>
#ifndef _MANAGED // setupapi.h cannot be included in managed code???
    #include <setupapi.h>
#else
    #if (!defined GUID)
        #pragma message("Define of GUID")
        typedef struct
        {
            long    Data1;
            short   Data2;
            short   Data3;
            char    Data4[8];
        }GUID;
    #endif
#endif

#if (defined _MANAGED) && (!defined BOOL) // windef.h (where BOOL is defined) cannot be included in managed code???
#define BOOL int
#endif


/*==== MACROS ==============================================================*/
#ifdef COMDRIVER_EXPORTS
#define API __declspec(dllexport)
#else
#define API __declspec(dllimport)
#endif

/*==== CONSTS ==============================================================*/

/*==== TYPES ===============================================================*/

/*==== EXPORTS =============================================================*/

class API PortEnumerator
{
public:
    PortEnumerator();
    ~PortEnumerator();
    S16 GetComPorts(std::vector<U16> *comPorts);
    S16 GetUsbPorts(std::vector<U16> *usbPorts);
    void FreePortList(std::vector<U16> *portList);
    S16 UsbEnumeratePort(U16 port, char *usbDescriptor);
    S16 ClearUsbEnumeration();
    S16 UsbMapDescriptorToPort(char *usbDescriptor);
    S16 UsbMapPortToDescriptor(char *usbDescriptor, U16 port);
    BOOL GetUsbDeviceFileName(char *outNameBuf);
    S16 UsbAddTarget(U16 *port, char *usbHubPortId);
    S16 UsbRemoveTarget(U16 port);
    S16 GetUsbPortsByPtr(std::vector<U16> **usbPorts);
    S16 GetComPortsByPtr(std::vector<U16> **comPorts);
protected:
private:
    void restoreTokenizedString(char* strPtr, int length, char seperatorToRestore);
    void mapFromUSBDescriptorToHubPortId(char *usbDescriptor, char *usbHubPortId);
	S16 UsbRemoveFriendlyName(U16 port);
	S16 UsbRemovePortFromRegistry(U16 port);
#ifndef _MANAGED
    HANDLE OpenOneDevice (HDEVINFO HardwareDeviceInfo,
                          PSP_DEVICE_INTERFACE_DATA DeviceInfoData,
                          char *devName);
    HANDLE OpenUsbDevice(LPGUID pGuid, char *outNameBuf);
    void EnumerateKeyspanPorts(std::vector<U16> *comPorts);

        S16 AllowAdminFullAccess(HKEY hKey);
        void mapFromUSBDescriptorToDeviceRegistryKey(char *usbDescriptor, char *usb_device_key);
#endif
    GUID USBGuid;

};

#endif /* __CSST_COMDRIVER_H__ */
/*==== END OF FILE ===========================================================*/

