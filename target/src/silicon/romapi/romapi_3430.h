/**
 * @file romapi_3430.h
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
 * ROM API constants
 */

#ifndef ROMAPI_3430_H
#define ROMAPI_3430_H

/*==== GLOBALS ==============================================================*/

#include "romapi_types.h"

/*==== PUBLIC FUNCTIONS =====================================================*/

struct PeripheralDesc_t;

//#define ROMAPI_SEC_ENTRY_pub2sec_dispatcher         RPAPI(0x00)
//#define ROMAPI_main                                 RPAPI(0x04)
//#define ROMAPI_CLKDET_DetectClock                   RPAPI(0x08)
//#define ROMAPI_CLK_OrderOmapClocks                  RPAPI(0x0C)
//#define ROMAPI_CLK_SetUpClocks                      RPAPI(0x10)
//#define ROMAPI_OMAP_GetSysBootPins                  RPAPI(0x14)
//#define ROMAPI_OMAP_GetDevType                      RPAPI(0x18)
//#define ROMAPI_OMAP_GetResetType                    RPAPI(0x1C)
//#define ROMAPI_SWCFG_GetSection                     RPAPI(0x20)
//#define ROMAPI_TRITON2_Configure                    RPAPI(0x24)
//#define ROMAPI_triton2_Detect                       RPAPI(0x28)
//#define ROMAPI_triton2_ConfigureUSB                 RPAPI(0x2C)
//#define ROMAPI_triton2_EnablePowerConfigRegWriting  RPAPI(0x30)
//#define ROMAPI_triton2_ConfigureVMMCRegulator       RPAPI(0x34)
//#define ROMAPI_triton2_ConfigureClock               RPAPI(0x38)
//#define ROMAPI_triton2_ConfigureVDACRegulator       RPAPI(0x3C)
//#define ROMAPI_triton2_WaitStablePLL                RPAPI(0x40)
//#define ROMAPI_triton2_EnableUSBTransceiver         RPAPI(0x44)
//#define ROMAPI_triton2_Write                        RPAPI(0x48)
typedef STATUS T_ROMAPI_triton2_Write(U16 iSlaveAddress, U32 iCount, U8  * ipData, U32 iStartTime, U32 iTimeOut);
#define ROMAPI_triton2_Write(iSlaveAddress, iCount, ipData, iStartTime, iTimeOut) CALL_ROMAPI_5(STATUS,T_ROMAPI_triton2_Write,RPAPI(0x48),iSlaveAddress,iCount,ipData,iStartTime,iTimeOut)

typedef STATUS T_ROMAPI_UART_Init ( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_UART_Init(a)                         CALL_ROMAPI_1(STATUS,T_ROMAPI_UART_Init,RPAPI(0x4C),a)
typedef STATUS T_ROMAPI_UART_Write ( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_UART_Write(a)                         CALL_ROMAPI_1(STATUS,T_ROMAPI_UART_Write,RPAPI(0x50),a)
typedef STATUS T_ROMAPI_UART_Read ( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_UART_Read(a)                         CALL_ROMAPI_1(STATUS,T_ROMAPI_UART_Read,RPAPI(0x54),a)
typedef void T_ROMAPI_UART_Reset ( void );
#define ROMAPI_UART_Reset()                         CALL_ROMAPI_0(void,T_ROMAPI_UART_Reset,RPAPI(0x58))

//#define ROMAPI_XIP_Initialize                       RPAPI(0x5C)
//#define ROMAPI_XIP_ReadSectors                      RPAPI(0x60)
//#define ROMAPI_BOOT_Booting                         RPAPI(0x64)
//#define ROMAPI_HWCFG_ConfigureHardware              RPAPI(0x68)
//#define ROMAPI_MBOOT_MemoryBooting                  RPAPI(0x6C)
//#define ROMAPI_PBOOT_PeripheralBooting              RPAPI(0x70)
//#define ROMAPI_TOC_FindTocItems                     RPAPI(0x74)
//#define ROMAPI_GPMC_Reset                           RPAPI(0x78)
//#define ROMAPI_GPMC_SetConfig                       RPAPI(0x7C)
//#define ROMAPI_GPMC_EnableWaitMonitoring            RPAPI(0x80)
//#define ROMAPI_GPMC_Set16Bits                       RPAPI(0x84)
//#define ROMAPI_GPMC_ConfigureECC                    RPAPI(0x88)
//#define ROMAPI_GPMC_ResetECC                        RPAPI(0x8C)
//#define ROMAPI_GPMC_SetECCPointer                   RPAPI(0x90)

//#define ROMAPI_I2C_Initialize                       RPAPI(0x94)
typedef STATUS T_HAL_I2C_Initialize(U32 iI2C);
#define HAL_I2C_Initialize(iI2C) CALL_ROMAPI_1(STATUS,T_HAL_I2C_Initialize,RPAPI(0x94),iI2C)

//#define ROMAPI_I2C_Write                            RPAPI(0x98)
typedef STATUS T_HAL_I2C_Write(U32 iI2C, U16 iSlaveAddress, U32 iCount, U8  * ipData, U32 iStartTime, U32 iTimeOut);
#define HAL_I2C_Write(iI2C, iSlaveAddress, iCount, ipData, iStartTime, iTimeOut) CALL_ROMAPI_6(STATUS,T_HAL_I2C_Write,RPAPI(0x98),iI2C,iSlaveAddress,iCount,ipData,iStartTime,iTimeOut)

//#define ROMAPI_I2C_Read                             RPAPI(0x9C)
typedef STATUS T_HAL_I2C_Read(U32 iI2C, U16 iSlaveAddress, U32 iCount, U8  * ipData, U32 iStartTime, U32 iTimeOut);
#define HAL_I2C_Read(iI2C, iSlaveAddress, iCount, ipData, iStartTime, iTimeOut) CALL_ROMAPI_6(STATUS,T_HAL_I2C_Read,RPAPI(0x9C),iI2C,iSlaveAddress,iCount,ipData,iStartTime,iTimeOut)

//#define ROMAPI_I2C_Close                            RPAPI(0xA0)
//#define ROMAPI_MDOC_Initialize                      RPAPI(0xA4)
//#define ROMAPI_MDOC_ReadSectors                     RPAPI(0xA8)
//#define ROMAPI_ECC_CompareEcc                       RPAPI(0xAC)
//#define ROMAPI_ECC_GenEccBytes                      RPAPI(0xB0)
//#define ROMAPI_NAND_Initialize                      RPAPI(0xB4)
//#define ROMAPI_NAND_ReadSectors                     RPAPI(0xB8)
//#define ROMAPI_NANDLL_ReadId                        RPAPI(0xBC)
//#define ROMAPI_NANDLL_ReadId2                       RPAPI(0xC0)
//#define ROMAPI_NANDLL_NandRead                      RPAPI(0xC4)
//#define ROMAPI_NANDPAR_Parameters                   RPAPI(0xC8)
//#define ROMAPI_ONAND_Initialize                     RPAPI(0xCC)
//#define ROMAPI_ONAND_ReadSectors                    RPAPI(0xD0)
//#define ROMAPI_ONLL_ReadIdFromRegisters             RPAPI(0xD4)
//#define ROMAPI_ONLL_ReadIdCommand                   RPAPI(0xD8)
//#define ROMAPI_ONLL_ReadSector                      RPAPI(0xDC)
//#define ROMAPI_onll_WaitOnBusy                      RPAPI(0xE0)
//#define ROMAPI_onll_LoadSector                      RPAPI(0xE4)
//#define ROMAPI_SDRC_SetConfig                       RPAPI(0xE8)
//#define ROMAPI_SSI_Init                             RPAPI(0xEC)
//#define ROMAPI_SSI_LowInit                          RPAPI(0xF0)
//#define ROMAPI_SSI_Write                            RPAPI(0xF4)
//#define ROMAPI_SSI_Read                             RPAPI(0xF8)
//#define ROMAPI_SSI_Close                            RPAPI(0xFC)
//#define ROMAPI_SSIHS_Init                           RPAPI(0x100)
//#define ROMAPI_SSIHS_Handshake                      RPAPI(0x104)
//#define ROMAPI_SSIHS_Write                          RPAPI(0x108)
//#define ROMAPI_FAT_ReadFat                          RPAPI(0x10C)
//#define ROMAPI_MMC_Initialize                       RPAPI(0x110)
//#define ROMAPI_MMC_ReadSectors                      RPAPI(0x114)
//#define ROMAPI_MMC_SetClock                         RPAPI(0x118)
//#define ROMAPI_MMC_SetBuswidth                      RPAPI(0x11C)
//#define ROMAPI_MMC_Close                            RPAPI(0x120)

typedef STATUS T_ROMAPI_USB_Initialize( const struct PeripheralDesc_t *pPeripheralDesc );
#define ROMAPI_USB_Initialize(a)                     CALL_ROMAPI_1(STATUS,T_ROMAPI_USB_Initialize,RPAPI(0x124),a)
typedef STATUS T_ROMAPI_USB_Write( const struct PeripheralDesc_t * ipPeripheralDesc );
#define ROMAPI_USB_Write(a)                         CALL_ROMAPI_1(STATUS,T_ROMAPI_USB_Write,RPAPI(0x128),a)
typedef STATUS T_ROMAPI_USB_Read( const struct PeripheralDesc_t * ipPeripheralDesc );
#define ROMAPI_USB_Read(a)                             CALL_ROMAPI_1(STATUS,T_ROMAPI_USB_Read,RPAPI(0x12C),a)
typedef void T_ROMAPI_USB_Close( void );
#define ROMAPI_USB_Close()                             CALL_ROMAPI_0(void,T_ROMAPI_USB_Close,RPAPI(0x130))

//#define ROMAPI_FAT_BufferFATEntries                 RPAPI(0x134)
//#define ROMAPI_MMC_LL_Initialize                    RPAPI(0x138)
//#define ROMAPI_MMC_LL_SendCommand                   RPAPI(0x13C)
//#define ROMAPI_MMC_LL_SetClock                      RPAPI(0x140)
//#define ROMAPI_0x00000000                           RPAPI(0x144)
//#define ROMAPI_mmc_Identify                         RPAPI(0x148)
//#define ROMAPI_mmc_GetCardSize                      RPAPI(0x14C)
//#define ROMAPI_mmc_GetCardBusWidthCaps              RPAPI(0x150)
//#define ROMAPI_mmc_ReadSectors                      RPAPI(0x154)
//#define ROMAPI_NANDMLC_TestMlcImage                 RPAPI(0x158)
//#define ROMAPI_NANDMLC_NandRead                     RPAPI(0x15C)


/* IRQ Errors */
#define ERROR_IRQ ( ((U32)'I'<<24) | ((U32)'R'<<16) | ((U32)'Q'<<8) )
#define ERROR_IRQ_NO_MORE_ENTRIES (ERROR_IRQ | 0x00)
#define ERROR_IRQ_ENTRY_NOT_FOUND (ERROR_IRQ | 0x01)
/* Timers Errors */
#define ERROR_TIMERS ( ((U32)'T'<<24) | ((U32)'I'<<16) | ((U32)'M'<<8) )
#define ERROR_TIMERS_REGISTERING_TIMER_HANDLER (ERROR_TIMERS | 0x00)
#define ERROR_TIMERS_START_BAD_PARAMETERS (ERROR_TIMERS | 0x01)
#define ERROR_TIMERS_START_NO_MORE_TIMERS (ERROR_TIMERS | 0x02)
#define ERROR_TIMERS_STOP_INVALID_TIMER_ID (ERROR_TIMERS | 0x03)
#define ERROR_TIMERS_HANDLER_INVALID_IRQ_NUMBER (ERROR_TIMERS | 0x04)
#define ERROR_TIMERS_HANDLER_NULL_CALLBACK_FUNCTION (ERROR_TIMERS | 0x05)
/* XIP Errors */
#define ERROR_XIP ( ((U32)'X'<<24) | ((U32)'I'<<16) | ((U32)'P'<<8) )
#define ERROR_XIP_INIT_GPMC (ERROR_XIP | 0x00)
#define ERROR_XIP_INIT_ENABLE_WAIT (ERROR_XIP | 0x01)
/* I2C Errors */
#define ERROR_I2C ( ((U32)'I'<<24) | ((U32)'2'<<16) | ((U32)'C'<<8) )
#define ERROR_I2C_REGISTERING_IRQ_HANDLER (ERROR_I2C | 0x00)
#define ERROR_I2C_HANDLER_INVALID_IRQ_NUMBER (ERROR_I2C | 0x01)
#define ERROR_I2C_UNREGISTERING_IRQ_HANDLER (ERROR_I2C | 0x02)
#define ERROR_I2C_PENDING_READ (ERROR_I2C | 0x03)
#define ERROR_I2C_READ_TIMEOUT (ERROR_I2C | 0x04)
#define ERROR_I2C_READ_FAILED (ERROR_I2C | 0x05)
#define ERROR_I2C_PENDING_WRITE (ERROR_I2C | 0x06)
#define ERROR_I2C_WRITE_TIMEOUT (ERROR_I2C | 0x07)
#define ERROR_I2C_WRITE_FAILED (ERROR_I2C | 0x08)
/* Triton 2 Errors */
#define ERROR_TRITON2 ( ((U32)'T'<<24) | ((U32)'R'<<16) | ((U32)'2'<<8) )
#define ERROR_TRITON2_INIT_I2C (ERROR_TRITON2 | 0x00)
#define ERROR_TRITON2_NOT_DETECTED (ERROR_TRITON2 | 0x01)
#define ERROR_TRITON2_CONFIGURE_CLOCK (ERROR_TRITON2 | 0x02)
#define ERROR_TRITON2_CONFIGURE_USB_ERROR (ERROR_TRITON2 | 0x03)
#define ERROR_TRITON2_CONFIGURE_VMMC1_ERROR (ERROR_TRITON2 | 0x04)
#define ERROR_TRITON2_CONFIGURE_VDAC_ERROR (ERROR_TRITON2 | 0x05)
#define ERROR_TRITON2_CONFIGURATION_ERROR (ERROR_TRITON2 | 0x06)
#define ERROR_TRITON2_CLOSE_I2C (ERROR_TRITON2 | 0x07)
#define ERROR_TRITON2_WRITING_I2C (ERROR_TRITON2 | 0x08)
#define ERROR_TRITON2_READING_I2C (ERROR_TRITON2 | 0x09)
#define ERROR_TRITON2_WRONG_ID (ERROR_TRITON2 | 0x0A)
#define ERROR_TRITON2_CONFIGURE_USB_REGULATORS (ERROR_TRITON2 | 0x0B)
#define ERROR_TRITON2_CONFIGURE_USB_WAIT_STABLE_PLL (ERROR_TRITON2 | 0x0C)
#define ERROR_TRITON2_CONFIGURE_USB_ENABLE_TRANSCEIVER (ERROR_TRITON2 | 0x0D)
#define ERROR_TRITON2_ENABLE_POWER_CONF (ERROR_TRITON2 | 0x0E)
#define ERROR_TRITON2_CONFIGURE_CLOCKS (ERROR_TRITON2 | 0x0F)
#define ERROR_TRITON2_DISABLE_POWER_CONF (ERROR_TRITON2 | 0x10)
#define ERROR_TRITON2_DETECTION_TIMEOUT (ERROR_TRITON2 | 0x11)
#define ERROR_TRITON2_CONFIGURE_VMMC2_ERROR (ERROR_TRITON2 | 0x12)
#define ERROR_TRITON2_CONFIGURE_VSIM_ERROR (ERROR_TRITON2 | 0x13)
/* USB Errors */
#define ERROR_USB ( ((U32)'U'<<24) | ((U32)'S'<<16) | ((U32)'B'<<8) )
#define ERROR_USB_REGISTERING_IRQ_HANDLER (ERROR_USB | 0x00)
#define ERROR_USB_UNREGISTERING_IRQ_HANDLER (ERROR_USB | 0x01)
#define ERROR_USB_WRITE_TRANSFER_NOT_READY (ERROR_USB | 0x02)
#define ERROR_USB_READ_TRANSFER_NOT_READY (ERROR_USB | 0x03)
#define ERROR_USB_CORE_INIT (ERROR_USB | 0x04)
#define ERROR_USB_CONFIGURATION_TIMEOUT (ERROR_USB | 0x05)
#define ERROR_USB_AS_HOST (ERROR_USB | 0x06)
#define ERROR_USB_VBUS_VALID_TIMEOUT (ERROR_USB | 0x07)
/* FAT Extended Errors Definitions */
#define ERROR_FAT ( ((U32)'F'<<24) | ((U32)'A'<<16) | ((U32)'T'<<8) )
#define ERROR_FAT_NO_SIGNATURE (ERROR_FAT | 0x00)
#define ERROR_FAT_READ_ERROR (ERROR_FAT | 0x01)
#define ERROR_FAT_NOT_FAT12_16_32 (ERROR_FAT | 0x02)
#define ERROR_FAT_NO_FAT (ERROR_FAT | 0x03)
#define ERROR_FAT_NOT_VALID_BYTSPERSEC (ERROR_FAT | 0x04)
#define ERROR_FAT_NOT_VALID_SECPERCLUS (ERROR_FAT | 0x05)
#define ERROR_FAT_NOT_VALID_RSVDSECCNT (ERROR_FAT | 0x06)
#define ERROR_FAT_NOT_VALID_NUMFATS (ERROR_FAT | 0x07)
#define ERROR_FAT_NOT_VALID_ROOTENTCNT (ERROR_FAT | 0x08)
#define ERROR_FAT_NOT_VALID_TOTSEC (ERROR_FAT | 0x09)
#define ERROR_FAT_NO_BOOTING_FILE (ERROR_FAT | 0x0A)
#define ERROR_FAT_FILE_VOL_DIR_LFN (ERROR_FAT | 0x0B)
#define ERROR_FAT_FILE_SIZE_0 (ERROR_FAT | 0x0C)
#define ERROR_FAT_REACHED_ROOT_DIR_END (ERROR_FAT | 0x0D)
#define ERROR_FAT_FAT_CHAIN_INIT (ERROR_FAT | 0x0E)
#define ERROR_FAT_CLUSTER_OUT_OF_BOUNDS (ERROR_FAT | 0x0F)
#define ERROR_FAT_ACCESSING_FAT (ERROR_FAT | 0x10)
#define ERROR_FAT_UNKNOWN_FAT_TYPE (ERROR_FAT | 0x11)
#define ERROR_FAT_BUFFERING_FAT (ERROR_FAT | 0x12)
#define ERROR_FAT_BUFFERS_FILESIZE_ZERO (ERROR_FAT | 0x13)
#define ERROR_FAT_BUFFERS_FILE_TOOBIG (ERROR_FAT | 0x14)
#define ERROR_FAT_BUFFERS_OVERRUN (ERROR_FAT | 0x15)
#define ERROR_FAT_BUFFERS_UNDERRUN (ERROR_FAT | 0x16)
#define ERROR_FAT_BUFFERS_FAT_COPIES_ERR (ERROR_FAT | 0x17)
#define ERROR_FAT_RESERVED_BAD_CLUSTER (ERROR_FAT | 0x18)
#define ERROR_FAT_ROOT_DIR_CHAIN_INIT (ERROR_FAT | 0x19)
#define ERROR_FAT_NOT_FAT12_16_32_NOT_MBR (ERROR_FAT | 0x1A)
/* MBR Extended Errors Definitions */
#define ERROR_MBR ( ((U32)'M'<<24) | ((U32)'B'<<16) | ((U32)'R'<<8) )
#define ERROR_MBR_NO_SIGNATURE (ERROR_MBR | 0x00)
#define ERROR_MBR_MULTIPLE_ACTIVE (ERROR_MBR | 0x01)
#define ERROR_MBR_NOT_FOUND (ERROR_MBR | 0x02)
#define ERROR_MBR_EMPTY_ENTRY_CORRUPTED (ERROR_MBR | 0x03)
#define ERROR_MBR_CALCULATED_LBA_CORRUPTED (ERROR_MBR | 0x04)
#define ERROR_MBR_READ_ERROR (ERROR_MBR | 0x05)
#define ERROR_MBR_OUT_OF_BOUNDS (ERROR_MBR | 0x06)
#define ERROR_MBR_NO_ACTIVE (ERROR_MBR | 0x07)
#define ERROR_MBR_NOT_SUPPORTED_OR_ACTIVE (ERROR_MBR | 0x08)
#define ERROR_MBR_ENTRY_CORRUPTED (ERROR_MBR | 0x09)
/* MMC Extended Errors Definitions */
#define ERROR_MMC ( ((U32)'M'<<24) | ((U32)'M'<<16) | ((U32)'C'<<8) )
#define ERROR_MMC_INIT_MMC (ERROR_MMC | 0x00)
#define ERROR_MMC_IDENTIFICATION (ERROR_MMC | 0x01)
#define ERROR_MMC_SENDING_CMD (ERROR_MMC | 0x02)
#define ERROR_MMC_BUFFERING_FILE (ERROR_MMC | 0x03)
#define ERROR_MMC_OUT_OF_BOUNDS (ERROR_MMC | 0x04)
#define ERROR_MMC_READING_SECTOR (ERROR_MMC | 0x05)
#define ERROR_MMC_CARD_BUSY (ERROR_MMC | 0x06)
#define ERROR_MMC_VDDRANGE_NOT_SUPPORTED (ERROR_MMC | 0x07)
#define ERROR_MMC_OUT_OF_RANGE (ERROR_MMC | 0x08)
#define ERROR_MMC_SET_CLOCKS (ERROR_MMC | 0x09)
#define ERROR_MMC_SET_CLOCKS_WRONG_TRAN_SPEED (ERROR_MMC | 0x0A)
#define ERROR_MMC_INIT_MMC_WRONG_MODULE (ERROR_MMC | 0x0B)
#define ERROR_MMC_SIZE_IS_ZERO (ERROR_MMC | 0x0C)
#define ERROR_MMC_UNSUPPORTED_BUS_WIDTH (ERROR_MMC | 0x0D)
#define ERROR_MMC_CONFIGURING_BUS_WIDTH (ERROR_MMC | 0x0E)
#define ERROR_MMC_GET_BUS_WIDTH_CAPS (ERROR_MMC | 0x0F)
#define ERROR_MMC_GETTING_CARD_SIZE (ERROR_MMC | 0x10)
#define ERROR_MMC_SIZE_INVALID_C_SIZE_MULT (ERROR_MMC | 0x11)
#define ERROR_MMC_SIZE_INVALID_READ_BL_LEN (ERROR_MMC | 0x12)
#define ERROR_MMC_UNSUPPORTED_SPEC_VERSION (ERROR_MMC | 0x13)
#define ERROR_MMC_READING_DATA (ERROR_MMC | 0x14)
//#ifdef MMC_HANDLE_WRITING
#define ERROR_MMC_WRITING_DATA (ERROR_MMC | 0x15)


#endif //ROMAPI_3430_H
