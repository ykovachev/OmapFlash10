/*
 * Copyright (c) 2009, Texas Instruments, Inc.
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
 */
/**
 * @file romapi_uart.c
 * @author Jens Odborg
 * @brief Implements UART driver using ROM API
 *
 * @details 
 * This file contains the implementation of a UART driver based on the use of the
 * Texas Instruments OMAP processor ROM code API for access to the UART interface. 
 * This version of the implementation is compatible with the ROM code API offered
 * for the OMAP3 series of processors, including the OMAP3430 and OMAP3630.
 */

///@todo refactor common functionality from romapi_usb.c and romapi_uart.c to romapi_common.c 

/*==== DECLARATION CONTROL ==================================================*/

#ifndef DISABLE_UART

#define ROMAPI_UART_C
//#define DEBUG_LOG_UART_STATES
//#define ROMAPI_UART_POLL

/*==== INCLUDES =============================================================*/
#include <string.h>
#ifdef DEBUG_UART
#include <stdio.h>
#include <disp.h>
#endif
#include <stdlib.h>
#include "types.h"
#include "error.h"
#include "uart.h"
#include "romapi.h"
#include "romapi_uart.h"

/*==== GLOBALS ==============================================================*/

#if 0

#define ROMAPI_UART_RX_DATA_DEFAULT_LENGTH 60

#ifdef OMAP4
const SYS_DriverPer_t *romapi_uart_SYS_DriverPer;
#endif

T_UART_INIT_STRUCTURE *romapi_uart_descriptor;
t_peripheral_init romapi_uart_init;

STATUS romapi_uart_init_status = NO_ERROR;
//U16 romapi_uart_init_options = 0xA; /* Magic value with no name in documentation */
UartConfObj_t ioConfObj;

/*==== PRIVATE FUNCTIONS ====================================================*/

void uart_call_deinit(void)
{
#ifdef OMAP3
    ROMAPI_UART_Reset();
#endif
#ifdef OMAP4430
    ROMAPI_UART_Close(&romapi_uart_init.peripheralDesc);
#endif
}

STATUS uart_call_read(PeripheralDesc_t *peripheralDesc)
{
    return ROMAPI_UART_Read(&peripheral_read_descriptor.peripheralDesc);
}

STATUS uart_call_write(PeripheralDesc_t *peripheralDesc)
{
    return ROMAPI_UART_Write(&peripheral_write_descriptor.peripheralDesc);
}

t_call_romapi uart_call_romapi = { uart_call_deinit, uart_call_read, uart_call_write };

/*==== PUBLIC FUNCTIONS =====================================================*/
/**
 * Initialization of the UART interface API.
 *
 * @param  init_str - Input Pointer to initialization structure.
 * @param  dis_addr - Output pointer to descriptor structure.
 * @return Status code
 */
S32 uart_peripheral_init(const void *init_str, U32 * dis_addr)
{
    T_UART_INIT_STRUCTURE * uart_init = (T_UART_INIT_STRUCTURE*)init_str;

#ifdef OMAP4
    if (!romapi_uart_SYS_DriverPer)
        SYS_GetDriverPer_2nd_uart(DEVICE_TYPE_UART3);
#endif

    memset(&romapi_uart_init, 0, sizeof romapi_uart_init);

    switch (uart_init->baud_rate)
    {
#define CASE_BAUD(baud) \
        case UART_BAUDRATE_##baud: \
            ioConfObj.baudrate 					= baud; \
            break
        ///only 115200, 230400 and 921600 supported by ROMAPI
        //CASE_BAUD(300);
        //CASE_BAUD(600);
        //CASE_BAUD(1200);
        //CASE_BAUD(2400);
        //CASE_BAUD(4800);
        //CASE_BAUD(9600);
        //CASE_BAUD(14400);
        //CASE_BAUD(19200);
        //CASE_BAUD(28800);
        //CASE_BAUD(38400);
        //CASE_BAUD(57600);
        CASE_BAUD(115200);
        CASE_BAUD(230400);
        //CASE_BAUD(460800);
        CASE_BAUD(921600);
        //CASE_BAUD(3686400);
        default:
            return OMAPFLASH_ERROR;
    }
    switch (uart_init->data_bits)
    {
        /*
        case UART_DATABITS_5:
            ioConfObj.n_bits 					= 5;
            break;
        case UART_DATABITS_6:
            ioConfObj.n_bits 					= 6;
            break;
        case UART_DATABITS_7:
            ioConfObj.n_bits 					= 7;
            break;
        */
        case UART_DATABITS_8:
            ioConfObj.n_bits 					= 8;
            break;
        default:
            return OMAPFLASH_ERROR;
    }
    switch (uart_init->parity_bits)
    {
        case UART_PARITY_NONE:
            ioConfObj.parity 		    = PARITY_NONE;
            break;
        case UART_PARITY_EVEN:
            ioConfObj.parity 		    = PARITY_EVEN;
            break;
        case UART_PARITY_ODD:
            ioConfObj.parity 		    = PARITY_ODD;
            break;
        default:
            return OMAPFLASH_ERROR;
    }
    switch (uart_init->stop_bits)
    {
        case UART_STOPBITS_1:
            ioConfObj.stop_bits					= STOP1;
            break;
#if UART_STOPBIT_1_5 != UART_STOPBIT_2
        case UART_STOPBIT_1_5:
            ioConfObj.stop_bits					= STOP15;
            break;
#endif
        case UART_STOPBIT_2:
            ioConfObj.stop_bits					= STOP2;
            break;
        default:
            return OMAPFLASH_ERROR;
    }
    //TODO these seem incorrect
    ioConfObj.RxTrig                                = 1;
    /*
        Symptom:
            If using UART and transmittin more than 0x74 bytes there will be a data abort in rom (seen on OMAP4)
        What I think is wrong:
            If enough characters send UART_Write will store 60 bytes in buffer, this is OK
            Succesive while still enough character uart_ISR will store 60 bytes in buffer on each interrupt this is NOT OK
            Things will go wrong if the interrupt is service rather quickly and TxTrig was configured to 56 as there will then not be enough space in tx buffer for 60 characters
    */
    //ioConfObj.TxTrig                                = 56; //60;
    ioConfObj.TxTrig                                = 60;

    *dis_addr = (U32) init_str;
    romapi_uart_descriptor = (T_UART_INIT_STRUCTURE *) init_str;

    /// We need to set up this structure even for reuse_romconnection as it is used by deinit incase of branch

    romapi_uart_init.peripheralDesc.IoConfObj            = &ioConfObj;
    //romapi_uart_init.peripheralDesc.pOptions             = &romapi_uart_init_options; /* Pointer to Boot options from topmain, see def in 4.1.1*/
#ifdef OMAP3
    romapi_uart_init.peripheralDesc.Device               = DEVICE_TYPE_UART; /* Type of the Device used */
#endif
#ifdef OMAP4
    romapi_uart_init.peripheralDesc.Mode                 = HAL_TRANSFER_MODE_DMA; /* This field is only used by the USB driver */
    romapi_uart_init.peripheralDesc.Device               = DEVICE_TYPE_UART3; /* Type of the Device used */
#endif
    peripheral_init(&romapi_uart_init, &uart_call_romapi);
    romapi_uart_init.peripheralDesc.Status               = (STATUS)-1; /* initialize to invalid value for debugger inspection */

    *dis_addr = (U32) init_str;
    romapi_uart_descriptor = (T_UART_INIT_STRUCTURE *) init_str;

    DEBUG_LOGF("uart_init");
#ifndef ROMAPI_UART_Init
#ifdef _MSC_VER
#pragma MESSAGE(PLATFORM)
#endif
#endif
    if (!romapi_uart_descriptor->reuse_romconnection)
    {
    romapi_uart_init_status = ROMAPI_UART_Init( &romapi_uart_init.peripheralDesc );

    switch (romapi_uart_init_status)
    {
        case NO_ERROR:
            return OMAPFLASH_SUCCESS;
        case FAILED:
        default:
            DEBUG_LOGF("uart_init fail %d", status_text(romapi_uart_init_status));
            //TODO
            return OMAPFLASH_ERROR;
        }
    }
    else
    {
        /// Note initial romapi called 2nd with an open UART connection! 
#ifdef OMAP3
        DEBUG_LOGF("usb_init reuse_romconnection not implemented for OMAP3");
        return OMAPFLASH_ERROR;
#endif
#ifdef OMAP4
        PER_DeviceDesc_t *oppDeviceDesc;

        SYS_GetDeviceDescPer( &oppDeviceDesc );
        //romapi_uart_init_options = *oppDeviceDesc->pOptions; 
        peripheral_init_options = *oppDeviceDesc->pOptions; 
        //memcpy(&ioConfObj, oppDeviceDesc->IoConfObj, sizeof ioConfObj);
        return OMAPFLASH_SUCCESS;
#endif
    }
}

#endif 
#endif
/*==== END OF FILE ==========================================================*/
