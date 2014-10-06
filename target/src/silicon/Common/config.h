/**
 * @file config.h
 * @author 
 *
 * @section LICENSE
 *
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
 *
 * @section DESCRIPTION
 *
 * 
 */
/**
 * @file config.h
 * @author Jens Odborg
 * @brief set feature defines based on board defines
 * @details 
 *  This file is included by assembler as well as c files and should for that reason only contain # lines
 *
 * @todo file details
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM/OMAP4430_NDA_TRM_MEMSS_EMIF_v3.0.pdf
 * @todo updata link
 */

#ifndef CONFIG_H
#define CONFIG_H
/*====== CONFIG =============================================================*/

#ifdef _MSC_VER
#define MESSAGE_2(file,line,msg) message (file "(" #line ") : message: " #msg)
#define MESSAGE_1(file,line,msg) MESSAGE_2(file,line,msg) 
#define MESSAGE(msg) MESSAGE_1(__FILE__,__LINE__,msg) 
//#pragma MESSAGE("silicon.h")
#endif

#if defined OMAP3_GP || defined OMAP3_HS
#define PROCESSOR "OMAP3"
#define OMAP3
#define FORCE_NON_MODULUS_64_PACKAGE_SIZE
#define DEBUG_UART
#define noDEBUG_LL
#define MEMDUMP
#define MTADDR
#define MTQUICK
#define MTMARCHX
#define DOWNLOAD
#define COMMAND_LIST
#endif

#ifdef OMAP4_GP
#define PROCESSOR "OMAP4"
#define OMAP4
#define DEBUG_UART
#define noDEBUG_USB
#define I2CSUPPORT
#define noDEBUG_LL
#define MEMDUMP
#define MTADDR
#define MTQUICK
#define MTMARCHX
#define MTIF
#define MTIF_SHOWCFG
#define noRECONNECT_HANDLER
#define HALFCT
#define LOG_STATISTICS
#define DOWNLOAD
#define COMMAND_LIST
#define OMAP4_USB_DMA_DEBUG
#endif

#ifdef OMAP4_HS
#define PROCESSOR "OMAP4"
#define OMAP4
#define DEBUG_UART
#define noDEBUG_USB
#define I2CSUPPORT
#define noDEBUG_LL
#define MEMDUMP
#define MTADDR
#define MTQUICK
#define MTMARCHX
#define noMTIF
#define noMTIF_SHOWCFG
#define noRECONNECT_HANDLER
#define HALFCT
#define noLOG_STATISTICS
#define DOWNLOAD
#define COMMAND_LIST
#define noOMAP4_USB_DMA_DEBUG
#endif

#ifdef OMAP4_MT
#define PROCESSOR "OMAP4"
#define OMAP4
#define DEBUG_UART
#define noDEBUG_USB
#define noI2CSUPPORT
#define noDEBUG_LL
#define MEMDUMP
#define MTADDR
#define MTQUICK
#define MTMARCHX
#define MTIF
#define MTIF_SHOWCFG
#define noRECONNECT_HANDLER
#define noHALFCT
#define noLOG_STATISTICS
#define noDOWNLOAD
#define COMMAND_LIST
#endif

#if defined OMAP5_GP || defined OMAP5_HS
#define PROCESSOR "OMAP5"
#define OMAP5
#define FORCE_NON_MODULUS_64_PACKAGE_SIZE
#define DEBUG_UART
#define noDEBUG_USB
#define I2CSUPPORT
#define noDEBUG_LL
#define MEMDUMP
#define MTADDR
#define MTQUICK
#define MTMARCHX
#define MTIF
#define MTIF_SHOWCFG
#define noRECONNECT_HANDLER
#define HALFCT
#define LOG_STATISTICS
#define DOWNLOAD
#define COMMAND_LIST
#endif

#ifdef OMAP5_MT
#define PROCESSOR "OMAP5"
#define OMAP5
#define FORCE_NON_MODULUS_64_PACKAGE_SIZE
#define DEBUG_UART
#define noDEBUG_USB
#define noI2CSUPPORT
#define noDEBUG_LL
#define MEMDUMP
#define MTADDR
#define MTQUICK
#define MTMARCHX
#define MTIF
#define MTIF_SHOWCFG
#define noRECONNECT_HANDLER
#define noHALFCT
#define noLOG_STATISTICS
#define noDOWNLOAD
#define COMMAND_LIST
#endif

#ifdef OMAPX
#ifndef SIMULATION
#error "OMAPX" only supported for SIMULATION
#endif
#endif

#ifndef PROCESSOR 
#error "No processor defined"
#endif

#endif //CONFIG_H
