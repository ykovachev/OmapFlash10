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
 * @file silicon.h
 * @author Jens Odborg
 * @brief Contains the base addresses of the interfaces or peripherals present in the silicon.
 * @details 
 * @todo file details
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM/OMAP4430_NDA_TRM_MEMSS_EMIF_v3.0.pdf
 * @todo updata link
 */

#ifndef SILICON_H
#define SILICON_H

/*====== CONFIG =============================================================*/

#include "config.h"

/*====== INCLUDES ===========================================================*/

#ifdef OMAP3
#include "silicon_3430.h" ///<@todo rename silicon_3xxx
#include "silicon_sdrc.h"
#include "silicon_gpmc.h"
#endif

#ifdef OMAP4
#include "silicon_4430.h"
#include "silicon_emif.h"
#endif

#ifdef OMAP5
#include "silicon_5430.h"
#include "silicon_emif.h"
#endif

#ifdef OMAPX
#ifndef SIMULATION
#error "OMAPX" only supported for SIMULATION
#endif
#define SDRC_BASE_ADDRESS 0
#define GPMC_BASE_ADDR    0
#define PAD_CONFIG_BASE   0
extern volatile U32 clk32k_counter_register;
#define CLK32K_COUNTER_REGISTER &clk32k_counter_register
#endif

#endif //SILICON_H
