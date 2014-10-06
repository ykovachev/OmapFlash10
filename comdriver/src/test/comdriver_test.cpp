/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  Communication driver - test application
+------------------------------------------------------------------------------
|             Copyright 2003 Texas Instruments.
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
| Filename:   comdriver_test.cpp
|
| Purpose:    This file contains a test app. for the CSST Communication Driver.
+----------------------------------------------------------------------------*/

/*==== INCLUDES =============================================================*/
//#include <windows.h>
//#include <process.h>
#include <iostream.h>
//#include <stdlib.h>

#include "ComDriver.h"
#include "SerialDriver.h"
#include "UsbDriver.h"
#include "PortEnumerator.h"

/*==== MACROS ===============================================================*/
#define MAX_PORTS   20

/*==== CONSTS ===============================================================*/

/*==== TYPES ================================================================*/

/*==== GLOBALS ==============================================================*/

/*==== PUBLIC FUNCTIONS =====================================================*/

/*-----------------------------------------------------------------------------
| Function    : print_usage()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  : argc - Number of parameters
|               argv - Array of arguments
|
| Returns     : None
+----------------------------------------------------------------------------*/
void print_usage()
{
    // TODO: Implement......
    return;
}

/*-----------------------------------------------------------------------------
| Function    : parse_arguments()
+------------------------------------------------------------------------------
| Description :
|
| Parameters  : argc - Number of parameters
|               argv - Array of arguments
|
| Returns     : TRUE for success and FALSE for failure.
+----------------------------------------------------------------------------*/
bool parse_arguments(int argc, char *argv[])
{
    // TODO: Implement commandline argument passing.....
    return TRUE;
}

/*-----------------------------------------------------------------------------
| Function    : usb_tests()
+------------------------------------------------------------------------------
| Description : Main test function
|
| Parameters  : void
|
| Returns     : None
+----------------------------------------------------------------------------*/
int usb_tests()
{
    UsbDriver* dev = new UsbDriver();

    dev->open(0, 0, 0, 0, 0);
    dev->configure(0, 0, 0, 0);
    dev->close();

    delete dev;
    return 0;
}

/*-----------------------------------------------------------------------------
| Function    : enumeration_tests()
+------------------------------------------------------------------------------
| Description : Main test function
|
| Parameters  : void
|
| Returns     : None
+----------------------------------------------------------------------------*/
int enumeration_tests()
{
    PortEnumerator*enumerator = new PortEnumerator();

    /* Read out the port mappings one the machine*/
    std::vector<U16> ports;
    S16 num = enumerator->GetUsbPorts(&ports);
    cout << "Ports found: " << endl;
    while(num)
    {
        num--;
        cout << "Port: " << ports[num] << endl;
    }
    enumerator->FreePortList(&ports);


    /* Try find an attached target within 20 seconds */
    char usbDesc[1024];
    int count = 0;
    cout << "Attach target to USB port '1': " << endl;
    while (0 != enumerator->UsbEnumeratePort(1, usbDesc) && count < 200)
    {
        Sleep(100);
        count++;
    }
    if (200 > count)
    {
        cout << "Usb target found on port '1':" << usbDesc << endl;
    }
    else
    {
        cout << "Usb port:" << *usbDesc << endl;
    }

    /* Map port to descriptor */
    S16 port;
    port = enumerator->UsbMapDescriptorToPort(usbDesc);
    cout << "Port from Descriptor: " << port << endl;
    char usbDesc2[1024];
    usbDesc2[0] = 0;
    enumerator->UsbMapPortToDescriptor(usbDesc2, 3);
    cout << "Descriptor from port: " << usbDesc2 << endl;

    /* Delete all current USB enumeration entries */
    //enumerator->ClearUsbEnumeration();

    delete enumerator;
    return 0;
}

/*-----------------------------------------------------------------------------
| Function    : print_enumerations()
+------------------------------------------------------------------------------
| Description : Print enumerations
|
| Parameters  : enumerator
|
| Returns     : None
+----------------------------------------------------------------------------*/
void print_enumerations(PortEnumerator* enumerator)
{
    std::vector<U16> ports;
    int num = enumerator->GetUsbPorts(&ports);

    cout << "Results of enumeration: " << endl;
    for (int i = 0; i < num; i++)
    {
        char desc[1024];
        cout << "Port: " << "USB" << ports[i] << " -> " << endl;
        enumerator->UsbMapPortToDescriptor(desc, ports[i]);
        cout << desc << endl << endl;
    }
    enumerator->FreePortList(&ports);
}

/*-----------------------------------------------------------------------------
| Function    : port_enumerator()
+------------------------------------------------------------------------------
| Description : Port enumeration
|
| Parameters  : void
|
| Returns     : None
+----------------------------------------------------------------------------*/
int port_enumerator()
{
    PortEnumerator* enumerator = new PortEnumerator();

    /* Clear previous enumeration before starting */
    enumerator->ClearUsbEnumeration();

    bool quit = false;
    U16 port = 1;
    while (!quit)
    {
        bool found = false;
        char usbDescriptor[1024];
        cout << "Attach target to USB port: USB" << port << endl << endl;

        while (0 != enumerator->UsbEnumeratePort(port, usbDescriptor) )
            Sleep(500);

        char reply;
        cout << "Port enumerated successfully!!" << endl;
        cout << endl;
        cout << "IMPORTANT: Unplug target from port USB" << port << " before answering" << endl;
        cout << "Continue to enumerate another port (y/n)?" << endl;
        cin >> reply;
        if ('n' == reply || 'N' == reply)
        {
            quit = true;
        }
        port++;

    }
    print_enumerations(enumerator);
    delete enumerator;
    return 0;
}


/*-----------------------------------------------------------------------------
| Function    : comport_test()
+------------------------------------------------------------------------------
| Description : Comport test
|
| Parameters  : void
|
| Returns     : None
+----------------------------------------------------------------------------*/
int comport_test()
{
    std::vector<U16> ports;
    PortEnumerator* enumerator = new PortEnumerator();

    S16 num = enumerator->GetComPorts(&ports);
    cout << "Results of Com Port enumeration: " << endl;
    for (int i=0; i < num; i++)
    {
        cout << "COM" << ports[i] << endl;
    }
    enumerator->FreePortList(&ports);

    delete enumerator;
    return 0;
}

/*-----------------------------------------------------------------------------
| Function    : main()
+------------------------------------------------------------------------------
| Description : Main test function
|
| Parameters  : void
|
| Returns     : None
+----------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    if (!parse_arguments(argc, argv))
    {
        print_usage();
        return 0;
    }

    //usb_tests();
    //enumeration_tests();
    port_enumerator();
    //comport_test();
    return 0;
}

/*==== END OF FILE ===========================================================*/

