/**
 * @file error.h
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
 * 
 */


/**
 * @file silicon.h
 * @author 
 * @brief Error code declarations.
 * @details 
 * @todo file details
 * @see <link>
 * @todo update link
 */


/*==== DECLARATION CONTROL =================================================*/

#ifndef ERROR_H
#define ERROR_H

/*==== INCLUDES ============================================================*/

/*==== TYPES ==============================================================*/

/*==== MACROS ==============================================================*/

#define OMAPFLASH_SUCCESS                             0
#define OMAPFLASH_ERROR                              (U32)-1

/* Common error codes (OEC = 0xF0) */

#define OMAPFLASH_MALLOC_FAILED						            0xF0000001

/* Error code specific to Host Dispatcher */

#define OMAPFLASH_DISP_INVALID_PARAMETER              0xF1000010

#define OMAPFLASH_DEVICE_ALREADY_INITIALIZED		      0xF2050002
#define OMAPFLASH_DAL_ERROR                           0xF205000C
#define OMAPFLASH_INVALID_SID					                0xF205000E
#define OMAPFLASH_INVALID_TAG					                0xF2050011

/*I2C error codes*/
#define OMAPFLASH_I2C_ER_AL						                0xF2052000
#define OMAPFLASH_I2C_ER_NACK						              0xF2052001
#define OMAPFLASH_I2C_ER_XUDF						              0xF2052002
#define OMAPFLASH_I2C_ER_ROVR						              0xF2052003
#define OMAPFLASH_I2C_ABORT							              0xF2052006

/* MMC error codes*/
#define NO_MMC_CARD                                   0xF2055000
#define MMC_WRITE_ER 				                          0xF2055001
#define MMC_PARAM_ER 						                      0xF2055002
#define MMC_WR_PR 							                      0xF2055003
#define MMC_INIT_ER 						                      0xF2055004
#define MMC_CARD_ER							                      0xF2055005
#define MMC_CMD_TO_ER						                      0xF2055006
#define MMC_CMD_FAIL						                      0xF2055007
#define MMC_BUSY_ER							                      0xF2055008
#define MMC_DATA_TO_ER					                      0xF2055009
#define MMC_CRC_ER							                      0xF205500A

#define LL_ERR_ID                                     (100)
#define LL_ERR_BUFFER_FULL                            (-8 - LL_ERR_ID)

/*==== GLOBALS ==============================================================*/

#endif /* ERROR_H */

