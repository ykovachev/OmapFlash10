/**
 * @file init.h
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
 * @brief Target initialization
 * @details 
 * @todo file details
 * @see <link>
 * @todo update link
 */

/*==== DECLARATION CONTROL =================================================*/
#ifndef INIT_H
#define INIT_H

/*==== CONFIGURATION =========================================================*/

/*==== INCLUDES ============================================================*/

#include "types.h"
#include "silicon.h"

/*==== CONSTS ==============================================================*/

/** @group Watchdog constants */
///@{
#ifdef OMAP3
#define WD_BASE                               0x48314000
#endif

#ifdef OMAP4
#define WD_BASE                               0x4A314000
#endif

#ifdef OMAP5
#define WD_BASE                               0x4AE14000
#endif

#define WD_AAAA                               0xAAAA
#define WSPR_WDT2                             0x0048
#define WWPS_WDT2                             0x0034
#define WD_5555                               0x5555
///@}


///*==== TYPES ===============================================================*/

/*====== FUNCTION PROTOTYPES=================================================*/

void watchdog_disable(void);

#endif
/*==== END OF FILE ===========================================================*/
