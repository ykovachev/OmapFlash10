/**
 * @file PortEnumerator.cpp
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
 * This file contains the implementation of the PortEnumerator Class
 */

/*==== INCLUDES =============================================================*/
#include "stdafx.h"
#include <conio.h>
#include <assert.h>
#include "devioctl.h"
#include <setupapi.h>
#include <basetyps.h>
#include "usbdi.h"
#include <algorithm>
#include "PortEnumerator.h"
#include <initguid.h> // Used by Keyspan enumerator routine
#include "windows.h"

/*==== MACROS ================================================================*/
#define ENUM_REGISTRY_SUBKEY    L"Enumerations"
#define CSST_REGISTRY_KEY       L"SOFTWARE\\Texas Instruments\\CSST"
#define ENUM_REGISTRY_KEY       L"SOFTWARE\\Texas Instruments\\CSST\\Enumerations"
#define USB_ENUM_REGISTRY_KEY    "SYSTEM\\CurrentControlSet\\Enum\\USB"

/*==== CONSTS ================================================================*/
static const GUID GuidClassTISample = {0x873fdf,0x61a8,0x11d1,0xaa,0x5e,0x0,0xc0,0x4f,0xb1,0x72,0x8b};

// Used by Keyspan Enumerator routine
DEFINE_GUID(GUID_PORTSDEVS, 0x4d36e978, 0xe325, 0x11ce, 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18);

/*==== TYPES =================================================================*/

/*==== GLOBALS ===============================================================*/

/*==== PUBLIC FUNCTIONS ======================================================*/

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::PortEnumerator()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
PortEnumerator::PortEnumerator() : USBGuid(GuidClassTISample)
{
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::~PortEnumerator()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
PortEnumerator::~PortEnumerator()
{
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::GetComPortsByPtr()
+------------------------------------------------------------------------------
| Description : A wrapper for GetComPorts() to allow call from an other DLL.
|               IMPORTANT: remenber to free the vector by using FreePortList().
|
| Parameters  : a pointer to a std::vector<U16> pointer.
|
| Returns     : number of com ports (the size of the vector).
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::GetComPortsByPtr(std::vector<U16> **comPorts)
{
    *comPorts = new std::vector<U16>();
    return GetComPorts(*comPorts);
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::GetComPorts()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::GetComPorts(std::vector<U16> *comPorts)
{
    GUID *ClassGuidList = NULL;
    DWORD requiredSize;
    BOOL res;
    S16 devCount = 0;

    /* Get the number of required class GUIDS and allocate an array */
    SetupDiClassGuidsFromName("PORTS", ClassGuidList, 0, &requiredSize);
    ClassGuidList = new GUID[requiredSize];

    /* Retrieve the class GUID list */
    res = SetupDiClassGuidsFromName("PORTS", ClassGuidList, requiredSize, &requiredSize);
    if (!res)
    {
        return -1;
    }

    for (DWORD count=0; count < requiredSize; count++)
    {
        GUID *guidDev = &ClassGuidList[count];
        HDEVINFO hwDevInfo = INVALID_HANDLE_VALUE;

        hwDevInfo = SetupDiGetClassDevs(guidDev,
                                        NULL,
                                        NULL,
                                        DIGCF_PRESENT |
                                        DIGCF_DEVICEINTERFACE);

        if (INVALID_HANDLE_VALUE == hwDevInfo)
        {
            return -1;
        }

        SP_DEVICE_INTERFACE_DATA deviceInterfaceData;
        deviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        BOOL done = FALSE;
        TCHAR friendlyName[256];
        while (!done)
        {
            BOOL res = SetupDiEnumDeviceInterfaces(hwDevInfo,
                                                   NULL,
                                                   guidDev,
                                                   devCount,
                                                   &deviceInterfaceData);

            if (res)
            {
                DWORD detailedInterfaceDataSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA) + 256;

                SP_DEVICE_INTERFACE_DETAIL_DATA *detailedInterfaceData;
                detailedInterfaceData = (SP_DEVICE_INTERFACE_DETAIL_DATA*) new char[detailedInterfaceDataSize];
                detailedInterfaceData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

                SP_DEVINFO_DATA deviceInfoData;
                deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

                res = SetupDiGetDeviceInterfaceDetail(hwDevInfo,
                                                      &deviceInterfaceData,
                                                      detailedInterfaceData,
                                                      detailedInterfaceDataSize,
                                                      NULL,
                                                      &deviceInfoData);
                delete [] detailedInterfaceData;
                if (res)
                {
                    res = SetupDiGetDeviceRegistryProperty(hwDevInfo,
                                                           &deviceInfoData,
                                                           SPDRP_FRIENDLYNAME,
                                                           NULL,
                                                           (PBYTE)friendlyName,
                                                           sizeof(friendlyName),
                                                           NULL);
                }
            }
            if (0 != res)
            {
                devCount++;
                /* Extract port number and add it to the vector */
                std::string nameString(friendlyName);
                int startPos = nameString.find("(COM") + 4;
                int endPos = nameString.find(")");
                /*Detecting printer port(ECP PRINTER PORT) and enumerating as 0 in the GUI of Windows 2000 machine but not in XP.Hence added
                check for printer port*/
                if(nameString.find("COM") != 0xffffffff)
                {
                    std::string portNumber = nameString.substr(startPos, endPos-startPos);
                    comPorts->push_back(atoi(portNumber.c_str()));
                }

            }
            else
            {
                done = TRUE;
            }
        }

        /* Destroy the Device Info list and free any allocated memory */
        SetupDiDestroyDeviceInfoList(hwDevInfo);

    }

    // Adding Keyspan Serial ports as described in Keyspan Developer Documentation - USB to Serial Programming
    // Windows 98/Me/2000/XP - DevDocsUSBSerial.pdf
    EnumerateKeyspanPorts(comPorts);

    // Adding Virtio virtual Serial ports - The have the property, that (caDevName == caPortName)
    // which we use to sortout other virtual comports like modems, BT devices etc...
    HKEY hKey;
    TCHAR caDevName[40], caPortName[20];
    DWORD dwDevNameSize, dwPortNameSize, dwType;
    int i;

    {
        if( RegOpenKeyEx( HKEY_LOCAL_MACHINE, "Hardware\\DeviceMap\\SerialComm", 0, KEY_READ,
                         &hKey) == ERROR_SUCCESS)
        {
            for( i = 0; TRUE; i++ )
            {
                dwDevNameSize = sizeof( caDevName );
                dwPortNameSize = sizeof( caPortName );
                if( RegEnumValue( hKey, i, caDevName, &dwDevNameSize, NULL,
                                 &dwType, (LPBYTE)caPortName, &dwPortNameSize ) != ERROR_SUCCESS )
                {
                    // no more entries found - quit the loop!
                    break;
                }

                if (!strcmp(caDevName, caPortName))
                {
                    // entry found add it to the array
                    devCount++;
                    /* Extract port number and add it to the vector */
                    std::string nameString(caPortName);
                    int startPos = nameString.find("COM")+3;
                    int endPos = nameString.length();
                    std::string portNumber = nameString.substr(startPos, endPos-startPos);
                    comPorts->push_back(atoi(portNumber.c_str()));
                }
            }
            RegCloseKey( hKey );
        }
    }

    /* Sort the vector before returning it */
    std::sort(comPorts->begin(), comPorts->end());

    /* Make the vector of COM ports unique, in case a COM port is found more than once */
    std::vector<U16>::iterator last = std::unique(comPorts->begin(), comPorts->end());
    comPorts->erase(last, comPorts->end());
    devCount = comPorts->size();

    delete [] ClassGuidList;

    return devCount;
}



/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::GetUsbPortsByPtr()
+------------------------------------------------------------------------------
| Description : A wrapper for GetUsbPorts() to allow call from an other DLL.
|               IMPORTANT: remenber to free the vector by using FreePortList().
|
| Parameters  : a pointer to a std::vector<U16> pointer.
|
| Returns     : number of usb ports (the size of the vector).
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::GetUsbPortsByPtr(std::vector<U16> **usbPorts)
{
    *usbPorts = new std::vector<U16>();
    return GetUsbPorts(*usbPorts);
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::GetUsbPorts()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::GetUsbPorts(std::vector<U16> *usbPorts)
{
    HKEY hkey;
    U32 res;
    U32 index = 0, size = 20;
    U32 numUsbPorts = 0;
    wchar_t name[20];

    res = RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                        ENUM_REGISTRY_KEY,
                        0,
                        KEY_READ,
                        &hkey);

    while(ERROR_SUCCESS == RegEnumValueW(hkey,
                                         index,
                                         name,
                                         &size,
                                         NULL,
                                         NULL,
                                         NULL,
                                         NULL))
    {
        if (wcscmp(name, L"USB"))
        {
            usbPorts->push_back(_wtoi(&name[3]));
            numUsbPorts++;
        }
        index++;
        size = 20;
    }
    std::sort(usbPorts->begin(), usbPorts->end());

    RegCloseKey(hkey);
    return numUsbPorts;
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::FreePortList()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
void PortEnumerator::FreePortList(std::vector<U16> *portList)
{
    portList->clear();
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::UsbRemoveFriendlyName()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::UsbRemoveFriendlyName(U16 port)
{
    U32 result, result_sec;
    HKEY hkey;
    char desc[1024]; desc[0] = '\0'; // Initialize a null terminated string to be able to use str-functions
    char usb_device_key[1024]; usb_device_key[0] = '\0'; // Initialize a null terminated string to be able to use str-functions
    result = UsbMapPortToDescriptor(desc, port);
    if(0 == result)
    {
        mapFromUSBDescriptorToDeviceRegistryKey(desc, usb_device_key);

        // Open registry key
        result = RegOpenKeyEx(HKEY_LOCAL_MACHINE,
                              usb_device_key,
                              0,
                              KEY_READ | WRITE_DAC,
                              &hkey);

        // Get security information for USB driver key
        char sd[1024]; // 0x44 bytes needed on my computer - Assuming 1024 bytes enough
        DWORD size = sizeof(sd);

        if (ERROR_SUCCESS == result)
        {
            result_sec = RegGetKeySecurity(hkey,
                                           (SECURITY_INFORMATION)DACL_SECURITY_INFORMATION,
                                           sd, &size );

            // Allow administrator full access to modify the USB key
            if (ERROR_SUCCESS == result_sec)
                result = AllowAdminFullAccess(hkey);

            RegCloseKey(hkey);
        }


        // Reopen registry key - To delete Value and restore security settings
        result = RegOpenKeyEx(HKEY_LOCAL_MACHINE,
                              usb_device_key,
                              0,
                              KEY_SET_VALUE | WRITE_DAC,
                              &hkey);
        if (ERROR_SUCCESS == result)
        {
            result = RegDeleteValue(hkey, "FriendlyName");

            // Restore security information for USB driver key
            if (ERROR_SUCCESS == result_sec)
                result_sec = RegSetKeySecurity(hkey,
                                               (SECURITY_INFORMATION)DACL_SECURITY_INFORMATION,
                                               sd );

            // Close registry key and end
            RegCloseKey(hkey);
        }
    }
    return result;

}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::UsbPortPortFromRegistry()
+------------------------------------------------------------------------------
| Description : Removes a single USB port from the CSST\Enumerations key.
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::UsbRemovePortFromRegistry(U16 port)
{
    HKEY hkey;
    U32 result;
    // Open registry key
    result = RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                           ENUM_REGISTRY_KEY,
                           0,
                           KEY_SET_VALUE,
                           &hkey);
    // After the reg. key is open the value for the USB port can be deleted.
    if(ERROR_SUCCESS == result)
    {
        // Generate the value name. (eg. USB1)
        char valueName[10];
        sprintf(valueName, "USB%i", port);

        result = RegDeleteValue(hkey,        // handle to key
                                valueName);  // address of value name
    }
    RegCloseKey(hkey);
    return result;
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::UsbRemoveTarget()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::UsbRemoveTarget(U16 port)
{
    U32 res1,res2;
    // Call the two remove functions to remove both friendly name and USB port
    // mapping path from the registry.
    res1 = UsbRemoveFriendlyName(port);
    res2 = UsbRemovePortFromRegistry(port);

    // Return the error code if any og the calls failed, else the success code
    // from the second call.
    if( res1 != ERROR_SUCCESS)
    {
        return res1;
    }
    else
    {
        return res2;
    }

}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::UsbAddTarget()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     : -1 if no target is found, 0 for all OK
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::UsbAddTarget(U16 *port, char *usbHubPortId)
{
    S16 retval = -1;
    std::vector<U16> ports;
    U16 nextPort;
    char usbDescriptor[1024]; usbDescriptor[0] = '\0'; // Initialize a null terminated string
    int num;

    /* 1) validate that a board is connected.
    * 2) find the first available USB port number, to be able to perform step 3).
    * 3) call UsbEnumeratePort() with the found port number.
    */

    // 1) validate that a board is connected.
    if (GetUsbDeviceFileName(usbDescriptor))
    {
        // Check if the board is already mapped, if so, just return the port number
        retval = UsbMapDescriptorToPort(usbDescriptor);
        if(retval != -1)
        {
            nextPort = retval;
            retval = 0;
        }
        else
        {
            // 2) find the first available USB port number.
            num = GetUsbPorts(&ports);

            if(num>0)
            {
                // The list supplied by GetUsbPorts is sorted, hence looping through the
                // values, comparing with the index+1 will reveal the first nonused port.
                // (the first USB port is 1 and the first index is 0, hence index+1)
                for (int i = 0; i < num; i++)
                {
                    nextPort = i+1;
                    if(ports[i] != nextPort)
                    {
                        // If a gab in the port list is found, break the loop.
                        break;
                    }
                }
                // If the nextPort equals the last port in the list, increment the nextPort
                // value to be sure an unused port number will be used.
                if (ports[num-1] == nextPort)
                {
                    nextPort++;
                }
            }
            else
            {
                nextPort = 1;
            }
            FreePortList(&ports);

            // 3) call UsbEnumeratePort() with the found port number to make the actual port mapping.
            retval = UsbEnumeratePort(nextPort, usbDescriptor);

        }
    }
    if (retval == 0)
    {
        // Set valid output values.
        *port = nextPort;
        mapFromUSBDescriptorToHubPortId(usbDescriptor, usbHubPortId);
    }
    else
    {
        // Set invalid output values.
        *port = 0;
        usbHubPortId[0] = '\0';
    }
    return retval;
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::UsbEnumeratePort()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::UsbEnumeratePort(U16 port, char *usbDescriptor)
{
    S16 ret = -1;

    if (!GetUsbDeviceFileName(usbDescriptor))
        return ret;

    /*
    * If we found a device and the port is different from zero we add it to the
    * registry. Port zero is special and should not target a specific USB port,
    * but rather use any port on the machine. This is why port zero is not added
    * to the registry port mapping.
    */
    if (0 != port)
    {
        /* Add the port mapping to the registry */
        U32 result, result_sec;
        HKEY hkey;
        result = RegCreateKeyExW(HKEY_LOCAL_MACHINE,
                                 ENUM_REGISTRY_KEY,
                                 NULL,
                                 NULL,
                                 REG_OPTION_NON_VOLATILE,
                                 KEY_ALL_ACCESS,
                                 NULL,
                                 &hkey,
                                 NULL);

        if (ERROR_SUCCESS == result)
        {
            U32 size = strlen(usbDescriptor)+1;
            //wchar_t *value = (wchar_t*) malloc(size*sizeof(wchar_t));
            //size = mbstowcs(value, usbDescriptor, size);
            char valueName[10];
            sprintf(valueName, "USB%i", port);
            result = RegSetValueEx(hkey,
                                   valueName,
                                   NULL,
                                   REG_SZ,
                                   (unsigned char*) usbDescriptor,
                                   size + sizeof(wchar_t));
            RegCloseKey(hkey);
        }


        // Add the Friendly name to the registry - To be used with Device Manager
        char usb_device_key[1024];
        mapFromUSBDescriptorToDeviceRegistryKey(usbDescriptor, usb_device_key);

        // Open registry key
        result = RegOpenKeyEx(HKEY_LOCAL_MACHINE,
                              usb_device_key,
                              0,
                              KEY_READ | WRITE_DAC,
                              &hkey);

        // Get security information for USB driver key
        char sd[1024]; // 0x44 bytes needed on my computer - Assuming 1024 bytes enough
        DWORD size = sizeof(sd);

        if (ERROR_SUCCESS == result)
        {
            result_sec = RegGetKeySecurity(hkey,
                                           (SECURITY_INFORMATION)DACL_SECURITY_INFORMATION,
                                           sd, &size );
        }


        // Allow administrator full access to modify the USB key
        if (ERROR_SUCCESS == result_sec)
            result = AllowAdminFullAccess(hkey);

        result = RegCloseKey(hkey);

        // Reopen registry key
        result = RegOpenKeyEx(HKEY_LOCAL_MACHINE,
                              usb_device_key,
                              0,
                              KEY_SET_VALUE | KEY_QUERY_VALUE | KEY_READ | WRITE_DAC,
                              &hkey);


        // Create Friendly name for USB device
        if (ERROR_SUCCESS == result)
        {
            unsigned char buffer[256];
            DWORD size = sizeof(buffer);
            result = RegQueryValueEx(hkey, "DeviceDesc", NULL, NULL,
                                     (unsigned char *)&buffer,
                                     &size);

            if (ERROR_SUCCESS == result)
            {
                char friendlyName[256];
                sprintf(friendlyName,"%s (USB%i)", buffer, port);
                U32 size = strlen(friendlyName)+1;
                char valueName[20];
                strcpy(valueName, "FriendlyName");
                result = RegSetValueEx(hkey,
                                       valueName,
                                       NULL,
                                       REG_SZ,
                                       (unsigned char*) friendlyName,
                                       size + sizeof(wchar_t));
            }


            // Restore security information for USB driver key
            if (ERROR_SUCCESS == result_sec)
            {
                result_sec = RegSetKeySecurity(hkey,
                                               (SECURITY_INFORMATION)DACL_SECURITY_INFORMATION,
                                               sd );
            }
        }

        // Close registry key and end
        if (ERROR_SUCCESS == result)
        {
            RegCloseKey(hkey);
            ret = 0;
        }

    }
    else
    {
        ret = 0;
    }

    return ret;
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::ClearUsbEnumeration()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::ClearUsbEnumeration()
{
    HKEY hkey;
    U32 result;

    // Remove all currently enumerated Friendly Names
    std::vector<U16> ports;
    int num = GetUsbPorts(&ports);

    for (int i = 0; i < num; i++)
    {
        result = UsbRemoveFriendlyName(ports[i]);
    }
    FreePortList(&ports);



    // Remove CSST USB port mapping key in CSST key tree
    result = RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                           CSST_REGISTRY_KEY,
                           0,
                           KEY_SET_VALUE,
                           &hkey);

    if (ERROR_SUCCESS == result)
    {
        result = RegDeleteKeyW(hkey, ENUM_REGISTRY_SUBKEY);
        RegCloseKey(hkey);
    }

    return 0;
}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::UsbMapDescriptorToPort()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::UsbMapDescriptorToPort(char *usbDescriptor)
{
    HKEY hkey;
    U32 res;
    U32 index = 0, nameSize = 10, valueSize = 1024;
    char valueName[10];
    unsigned char value[1024];
    S16 port = -1;

    res = RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                        ENUM_REGISTRY_KEY,
                        0,
                        KEY_READ,
                        &hkey);

    while(ERROR_SUCCESS == RegEnumValue(hkey,
                                        index,
                                        valueName,
                                        &nameSize,
                                        NULL,
                                        NULL,
                                        value,
                                        &valueSize))
    {
        if (nameSize && valueSize)
        {
            if (0 == memcmp("USB", valueName, 3) &&
                0 == memcmp(usbDescriptor, (char*)value, strlen(usbDescriptor)))
            {
                port = atoi(&valueName[3]);
                break;
            }
            index++;
            nameSize = 10;
            valueSize = 1024;
        }
    }

    RegCloseKey(hkey);
    return port; /* change return value */
}


/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::UsbMapPortToDescriptor()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :
|
| Returns     :
+-----------------------------------------------------------------------------*/
S16 PortEnumerator::UsbMapPortToDescriptor(char *usbDescriptor, U16 port)
{
    HKEY hkey;
    U32 res;
    U32 index = 0, nameSize = 10, valueSize = 1024;
    char valueName[10];
    char portName[10];
    unsigned char value[1024];
    S16 ret = -1;

    if (0 == port)
    {
        ret = UsbEnumeratePort(port, usbDescriptor);
    }
    else
    {
        res = RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                            ENUM_REGISTRY_KEY,
                            0,
                            KEY_READ,
                            &hkey);

        sprintf(portName, "USB%d", port);

        while(ERROR_SUCCESS == RegEnumValue(hkey,
                                            index,
                                            valueName,
                                            &nameSize,
                                            NULL,
                                            NULL,
                                            value,
                                            &valueSize))
        {
            if (nameSize && valueSize)
            {
                if (0 == memcmp(portName, valueName, strlen(portName)))
                {
                    ret = 0;
                    memcpy(usbDescriptor, value, (valueSize+1));
                    break;
                }
                index++;
                nameSize = 10;
                valueSize = 1024;
            }
        }

        RegCloseKey(hkey);
    }
    return ret;
}

/*==== PROTECTED FUNCTIONS ===================================================*/

/*==== PRIVATE FUNCTIONS =====================================================*/

void PortEnumerator::mapFromUSBDescriptorToDeviceRegistryKey(char *usbDescriptor, char *usb_device_key)
{
    char seps[]   = "#";
    char *token;
    int length = strlen(usbDescriptor);

    sprintf(usb_device_key,"%s\\",USB_ENUM_REGISTRY_KEY);

    // Get first token, which we don't use
    token = strtok( usbDescriptor, seps );

    // Get next token - Vid and Pid part of name
    token = strtok( NULL, seps );

    strcat(usb_device_key,token);

    strcat(usb_device_key,"\\");

    // Get next token - HUB port number
    token = strtok( NULL, seps );

    strcat(usb_device_key,token);

    restoreTokenizedString(usbDescriptor, length, '#'); // Restore the original string to be able to use it later
}

void PortEnumerator::restoreTokenizedString(char* strPtr, int length, char seperatorToRestore)
{
    for(int i = 0; i<length; i++)
    {
        if(strPtr[i] == '\0')
        {
            strPtr[i] = seperatorToRestore;
        }
    }
}

void PortEnumerator::mapFromUSBDescriptorToHubPortId(char *usbDescriptor, char *usbHubPortId)
{
    char seps[]   = "#";
    char *token;
    int length = strlen(usbDescriptor);

    // Get first token, which we don't use
    token = strtok( usbDescriptor, seps );

    // Get next token - Vid and Pid part of name
    token = strtok( NULL, seps );

    // Get next token - HUB port number
    token = strtok( NULL, seps );
    strcpy(usbHubPortId,token);
    restoreTokenizedString(usbDescriptor, length, '#'); // Restore the original string to be able to use it later
}

S16 PortEnumerator::AllowAdminFullAccess(HKEY hKey)
{
    SID_IDENTIFIER_AUTHORITY sia = SECURITY_NT_AUTHORITY;
    PSID pAdministratorsSid = NULL;
    SECURITY_DESCRIPTOR sd;
    PACL pDacl = NULL;
    DWORD dwAclSize;
    LONG lRetCode;
    BOOL bSuccess = FALSE; // assume this function fails

    // preprate a Sid representing the well-known admin group
    if(!AllocateAndInitializeSid(
                                 &sia,
                                 2,
                                 SECURITY_BUILTIN_DOMAIN_RID,
                                 DOMAIN_ALIAS_RID_ADMINS,
                                 0, 0, 0, 0, 0, 0,
                                 &pAdministratorsSid
                                )) {
        goto cleanup;
    }

    // compute size of new acl
    dwAclSize = sizeof(ACL) +
    2 * ( sizeof(ACCESS_ALLOWED_ACE) - sizeof(DWORD) ) +
    GetLengthSid(pAdministratorsSid) ;

    // allocate storage for Acl
    pDacl = (PACL)HeapAlloc(GetProcessHeap(), 0, dwAclSize);
    if(pDacl == NULL) goto cleanup;

    if(!InitializeAcl(pDacl, dwAclSize, ACL_REVISION)) {
        goto cleanup;
    }

    // grant the Administrators Sid KEY_ALL_ACCESS access to the perf key
    if(!AddAccessAllowedAce(
                            pDacl,
                            ACL_REVISION,
                            KEY_ALL_ACCESS,
                            pAdministratorsSid
                           )) {
        goto cleanup;
    }

    if(!InitializeSecurityDescriptor(&sd, SECURITY_DESCRIPTOR_REVISION)) {
        goto cleanup;
    }

    if(!SetSecurityDescriptorDacl(&sd, TRUE, pDacl, FALSE)) {
        goto cleanup;
    }

    // apply the security descriptor to the registry key
    lRetCode = RegSetKeySecurity(
                                 hKey,
                                 (SECURITY_INFORMATION)DACL_SECURITY_INFORMATION,
                                 &sd
                                );

    if(lRetCode != ERROR_SUCCESS) {
        goto cleanup;
    }

    bSuccess = TRUE; // indicate success

    // free allocated resources
cleanup:
    if(pDacl != NULL)
        HeapFree(GetProcessHeap(), 0, pDacl);

    if(pAdministratorsSid != NULL)
        FreeSid(pAdministratorsSid);

    if(bSuccess) {
        return ERROR_SUCCESS;
    } else {
        return ERROR_GEN_FAILURE;
    }
}



/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::EnumerateKeyspanPorts()
+------------------------------------------------------------------------------
| Description : Adding Keyspan Serial ports as described in Keyspan Developer
|               Documentation - USB to Serial Programming
|               Windows 98/Me/2000/XP - DevDocsUSBSerial.pdf
|
| Parameters  : std::vector<U16> *comPorts
|                                   Vector containing the list of found COM ports
|
| Returns     : none - The comport list is updated in case any Keyspan Serial
|                      adapters are found.
+----------------------------------------------------------------------------*/
void PortEnumerator::EnumerateKeyspanPorts(std::vector<U16> *comPorts)
{
    HDEVINFO hDevInfo = NULL;
    SP_DEVINFO_DATA DeviceInterfaceData;
    int i = 0;
    DWORD dataType, actualSize = 0;
    unsigned char dataBuf[MAX_PATH + 1];
    // Search device set
    hDevInfo = SetupDiGetClassDevs((struct _GUID *)&GUID_PORTSDEVS,0,0,DIGCF_PRESENT);
    if ( hDevInfo )
    {
        while (TRUE)
        {
            ZeroMemory(&DeviceInterfaceData, sizeof(DeviceInterfaceData));
            DeviceInterfaceData.cbSize = sizeof(DeviceInterfaceData);
            if (!SetupDiEnumDeviceInfo(hDevInfo, i, &DeviceInterfaceData))
            {
                // SetupDiEnumDeviceInfo failed
                break;
            }
            if (SetupDiGetDeviceRegistryProperty(hDevInfo,
                                                 &DeviceInterfaceData,
                                                 SPDRP_FRIENDLYNAME,
                                                 &dataType,
                                                 dataBuf,
                                                 sizeof(dataBuf),
                                                 &actualSize))
            {
                // IF IT'S A KEYSPAN DEVICE
                if (strstr((char *)dataBuf, "Keyspan"))
                {
                    char *begin, *end;
                    if (begin = strstr((char *)dataBuf, "COM"))
                    {
                        if (end = strstr(begin, ")"))
                        {
                            *end = 0;

                            std::string portNumber = begin+3;
                            comPorts->push_back(atoi(portNumber.c_str()));

                        }
                    }
                }
            }
            i++;
        }
    }
    SetupDiDestroyDeviceInfoList(hDevInfo);
}


/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::OpenOneDevice()
+------------------------------------------------------------------------------
| Description : Given the HardwareDeviceInfo, representing a handle to the
|               plug and play information, and deviceInfoData, representing a
|               specific usb device, open that device and fill in all the
|               relevant information in the given USB_DEVICE_DESCRIPTOR structure.
|
| Parameters  : HardwareDeviceInfo: handle to info obtained from Pnp mgr via
|                                   SetupDiGetClassDevs()
|               DeviceInfoData:     ptr to info obtained via
|                                   SetupDiEnumDeviceInterfaces()
|
| Returns     : return HANDLE if the open and initialization was successfull,
|               else INVLAID_HANDLE_VALUE.
+----------------------------------------------------------------------------*/
HANDLE PortEnumerator::OpenOneDevice (HDEVINFO HardwareDeviceInfo,
                                      PSP_DEVICE_INTERFACE_DATA DeviceInfoData,
                                      char *devName)
{
    PSP_DEVICE_INTERFACE_DETAIL_DATA     functionClassDeviceData = NULL;
    ULONG                                predictedLength = 0;
    ULONG                                requiredLength = 0;
    HANDLE                               hOut = INVALID_HANDLE_VALUE;

    //
    // allocate a function class device data structure to receive the
    // goods about this particular device.
    //
    SetupDiGetDeviceInterfaceDetail (
                                     HardwareDeviceInfo,
                                     DeviceInfoData,
                                     NULL, // probing so no output buffer yet
                                     0,    // probing so output buffer length of zero
                                     &requiredLength,
                                     NULL); // not interested in the specific dev-node


    predictedLength = requiredLength;
    // sizeof (SP_FNCLASS_DEVICE_DATA) + 512;

    functionClassDeviceData = (PSP_DEVICE_INTERFACE_DETAIL_DATA) malloc (predictedLength);
    if(NULL == functionClassDeviceData)
    {
        return INVALID_HANDLE_VALUE;
    }
    functionClassDeviceData->cbSize = sizeof (SP_DEVICE_INTERFACE_DETAIL_DATA);

    //
    // Retrieve the information from Plug and Play.
    //
    if (! SetupDiGetDeviceInterfaceDetail (
                                           HardwareDeviceInfo,
                                           DeviceInfoData,
                                           functionClassDeviceData,
                                           predictedLength,
                                           &requiredLength,
                                           NULL))
    {
        free(functionClassDeviceData);
        return INVALID_HANDLE_VALUE;
    }

    strcpy( devName,functionClassDeviceData->DevicePath) ;

    hOut = CreateFile (
                       functionClassDeviceData->DevicePath,
                       GENERIC_READ | GENERIC_WRITE,
                       FILE_SHARE_READ | FILE_SHARE_WRITE,
                       NULL, // no SECURITY_ATTRIBUTES structure
                       OPEN_EXISTING, // No special create flags
                       0,    // No special attributes
                       NULL); // No template file

    free( functionClassDeviceData );

    return hOut;

}

/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::OpenUsbDevice()
+------------------------------------------------------------------------------
| Description : Do the required PnP things in order to find the next available
|               proper device in the system at this time.
|
| Parameters  : pGuid: ptr to GUID registered by the driver itself
|               outNameBuf: the generated zero-terminated name for this device
|
| Returns     : return HANDLE if the open and initialization was successful,
|               else INVLAID_HANDLE_VALUE.
+----------------------------------------------------------------------------*/
HANDLE PortEnumerator::OpenUsbDevice(LPGUID pGuid, char *outNameBuf)
{
    ULONG NumberDevices;
    HANDLE hOut = INVALID_HANDLE_VALUE;
    HDEVINFO                 hardwareDeviceInfo;
    SP_DEVICE_INTERFACE_DATA deviceInfoData;
    ULONG                    i;
    BOOLEAN                  done;
    PUSB_DEVICE_DESCRIPTOR   usbDeviceInst;
    PUSB_DEVICE_DESCRIPTOR   *UsbDevices = &usbDeviceInst;
    PUSB_DEVICE_DESCRIPTOR   tempDevDesc;

    *UsbDevices = NULL;
    tempDevDesc = NULL;
    NumberDevices = 0;

    //
    // Open a handle to the plug and play dev node.
    // SetupDiGetClassDevs() returns a device information set that contains info on all
    // installed devices of a specified class.
    //
    hardwareDeviceInfo = SetupDiGetClassDevs (
                                              pGuid,
                                              NULL, // Define no enumerator (global)
                                              NULL, // Define no
                                              (DIGCF_PRESENT // Only Devices present
                                               | DIGCF_DEVICEINTERFACE // Function class devices.
                                               ));

    //
    // Take a wild guess at the number of devices we have;
    // Be prepared to realloc and retry if there are more than we guessed
    //
    NumberDevices = 4;
    done = FALSE;
    deviceInfoData.cbSize = sizeof (SP_DEVICE_INTERFACE_DATA);

    i=0;
    while (!done)
    {
        NumberDevices *= 2;

        if (*UsbDevices)
        {
            tempDevDesc = (PUSB_DEVICE_DESCRIPTOR) realloc (*UsbDevices, (NumberDevices * sizeof (USB_DEVICE_DESCRIPTOR)));
            if(tempDevDesc)
            {
                *UsbDevices = tempDevDesc;
                tempDevDesc = NULL;
            }
            else
            {
                free(*UsbDevices);
                *UsbDevices = NULL;
            }
        }
        else
        {
            *UsbDevices = (PUSB_DEVICE_DESCRIPTOR) calloc (NumberDevices, sizeof (USB_DEVICE_DESCRIPTOR));
        }

        if (NULL == *UsbDevices)
        {
            // SetupDiDestroyDeviceInfoList destroys a device information set
            // and frees all associated memory.
            SetupDiDestroyDeviceInfoList (hardwareDeviceInfo);
            return INVALID_HANDLE_VALUE;
        }

        usbDeviceInst = *UsbDevices + i;

        for (; i < NumberDevices; i++)
        {
            // SetupDiEnumDeviceInterfaces() returns information about device interfaces
            // exposed by one or more devices. Each call returns information about one interface;
            // the routine can be called repeatedly to get information about several interfaces
            // exposed by one or more devices.
            if (SetupDiEnumDeviceInterfaces (hardwareDeviceInfo,
                                             0, // We don't care about specific PDOs
                                             pGuid,
                                             i,
                                             &deviceInfoData))
            {

                hOut = OpenOneDevice (hardwareDeviceInfo, &deviceInfoData, outNameBuf);
                if ( hOut != INVALID_HANDLE_VALUE )
                {
                    done = TRUE;
                    break;
                }
            }
            else
            {
                if (ERROR_NO_MORE_ITEMS == GetLastError())
                {
                    done = TRUE;
                    break;
                }
            }
        }
    }

    NumberDevices = i;

    // SetupDiDestroyDeviceInfoList() destroys a device information set
    // and frees all associated memory.
    SetupDiDestroyDeviceInfoList(hardwareDeviceInfo);
    free(*UsbDevices);

    return hOut;
}


/*-----------------------------------------------------------------------------
| Function    : PortEnumerator::GetUsbDeviceFileName()
+------------------------------------------------------------------------------
| Description : Given a ptr to a driver-registered GUID, give us a string with
|               the device name that can be used in a CreateFile() call.
|               Actually briefly opens and closes the device and sets outBuf
|               if successfull; returns FALSE if not
|
| Parameters  : pGuid: ptr to GUID registered by the driver itself
|               outNameBuf: the generated zero-terminated name for this device
|
| Returns     : TRUE on success else FALSE
+----------------------------------------------------------------------------*/
BOOL PortEnumerator::GetUsbDeviceFileName(char *outNameBuf)
{
    HANDLE hDev = OpenUsbDevice(&USBGuid, outNameBuf);
    if (hDev != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hDev);
        return TRUE;
    }
    return FALSE;
}

/*==== END OF FILE ===========================================================*/

