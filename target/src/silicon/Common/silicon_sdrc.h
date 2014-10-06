/**
 * @file silicon_sdrc.h
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
 * Contains the base address declarations for SDRC
 */

/*======= DECLARATION CONTROL ===============================================*/

#ifndef SILICON_SDRC_H

#define SILICON_SDRC_H

/*====== CONFIG =============================================================*/


/*====== INCLUDES ===========================================================*/


/*===== CONSTS ==============================================================*/

/* SDRC Registers */
#if 0

#define SDRC_SYSCONFIG                      (SDRC_BASE_ADDRESS + 0x010)
#define SDRC_STATUS                         (SDRC_BASE_ADDRESS + 0x014)
#define SDRC_CS_CFG                         (SDRC_BASE_ADDRESS + 0x040)
#define SDRC_ACTIM_CTRLB_0                  (SDRC_BASE_ADDRESS + 0x0A0)
#define SDRC_MANUAL_0                       (SDRC_BASE_ADDRESS + 0x0A8)
#define SDRC_MANUAL_1                       (SDRC_BASE_ADDRESS + 0x0D8)
#define SDRC_ACTIM_CTRLB_1                  (SDRC_BASE_ADDRESS + 0x0C8)
#define SDRC_SHARING                        (SDRC_BASE_ADDRESS + 0x044)
#define SDRC_ERR_TYPE                       (SDRC_BASE_ADDRESS + 0x04C)
#define SDRC_DLLB_CTRL                      (SDRC_BASE_ADDRESS + 0x068)
#define SDRC_DLLA_CTRL                      (SDRC_BASE_ADDRESS + 0x060)
#define SDRC_DLLA_STATUS                    (SDRC_BASE_ADDRESS + 0x064)
#define SDRC_MCFG_0                         (SDRC_BASE_ADDRESS + 0x080)
#define SDRC_MCFG_1                         (SDRC_BASE_ADDRESS + 0x0B0)
#define SDRC_ACTIM_CTRLA_0                  (SDRC_BASE_ADDRESS + 0x09C)
#define SDRC_ACTIM_CTRLA_1                  (SDRC_BASE_ADDRESS + 0x0C4)
#define SDRC_DCDL1_CTRL                     (SDRC_BASE_ADDRESS + 0x094)
#define SDRC_DCDL2_CTRL                     (SDRC_BASE_ADDRESS + 0x098)
#define SDRC_POWER                          (SDRC_BASE_ADDRESS + 0x070)
#define SDRC_RFR_CTRL_0                     (SDRC_BASE_ADDRESS + 0x0A4)
#define SDRC_RFR_CTRL_1                     (SDRC_BASE_ADDRESS + 0x0D4)
#define SDRC_MR_0                           (SDRC_BASE_ADDRESS + 0x084)
#define SDRC_MR_1                           (SDRC_BASE_ADDRESS + 0x0B4)
/* Manual regs */
#define SDRC_MANUAL_0                       (SDRC_BASE_ADDRESS + 0x0A8)
#define SDRC_MANUAL_1                       (SDRC_BASE_ADDRESS + 0x0D8)

#define  CONTROL_PADCONF_sdrc_d0            (PAD_CONFIG_BASE + 0x0030)
#define  CONTROL_PADCONF_sdrc_d0_HI         (PAD_CONFIG_BASE + 2 + 0x0032)
#define  CONTROL_PADCONF_sdrc_d2            (PAD_CONFIG_BASE + 0x0034)
#define  CONTROL_PADCONF_sdrc_d2_HI         (PAD_CONFIG_BASE + 2+ 0x0036)
#define  CONTROL_PADCONF_sdrc_d4            (PAD_CONFIG_BASE + 0x0038)
#define  CONTROL_PADCONF_sdrc_d4_HI         (PAD_CONFIG_BASE + 2 + 0x0038)
#define  CONTROL_PADCONF_sdrc_d6            (PAD_CONFIG_BASE + 0x003C)
#define  CONTROL_PADCONF_sdrc_d6_HI         (PAD_CONFIG_BASE + 2 + 0x003C)
#define  CONTROL_PADCONF_sdrc_d8            (PAD_CONFIG_BASE + 0x0040)
#define  CONTROL_PADCONF_sdrc_d8_HI         (PAD_CONFIG_BASE + 2 + 0x0040)
#define  CONTROL_PADCONF_sdrc_d10           (PAD_CONFIG_BASE + 0x0044)
#define  CONTROL_PADCONF_sdrc_d10_HI        (PAD_CONFIG_BASE + 2 + 0x0044)
#define  CONTROL_PADCONF_sdrc_d12           (PAD_CONFIG_BASE + 0x0048)
#define  CONTROL_PADCONF_sdrc_d12_HI        (PAD_CONFIG_BASE + 2 + 0x0048)
#define  CONTROL_PADCONF_sdrc_d14           (PAD_CONFIG_BASE + 0x004C)
#define  CONTROL_PADCONF_sdrc_d14_HI        (PAD_CONFIG_BASE + 2 + 0x004C)
#define  CONTROL_PADCONF_sdrc_d16           (PAD_CONFIG_BASE + 0x0050)
#define  CONTROL_PADCONF_sdrc_d16_HI        (PAD_CONFIG_BASE + 2 + 0x0050)
#define  CONTROL_PADCONF_sdrc_d18           (PAD_CONFIG_BASE + 0x0054)
#define  CONTROL_PADCONF_sdrc_d18_HI        (PAD_CONFIG_BASE + 2 + 0x0054)
#define  CONTROL_PADCONF_sdrc_d20           (PAD_CONFIG_BASE + 0x0058)
#define  CONTROL_PADCONF_sdrc_d20_HI        (PAD_CONFIG_BASE + 2 + 0x0058)
#define  CONTROL_PADCONF_sdrc_d22           (PAD_CONFIG_BASE + 0x005C)
#define  CONTROL_PADCONF_sdrc_d22_HI        (PAD_CONFIG_BASE + 2 + 0x005C)
#define  CONTROL_PADCONF_sdrc_d24           (PAD_CONFIG_BASE + 0x0060)
#define  CONTROL_PADCONF_sdrc_d24_HI        (PAD_CONFIG_BASE + 2 + 0x0060)
#define  CONTROL_PADCONF_sdrc_d26           (PAD_CONFIG_BASE + 0x0064)
#define  CONTROL_PADCONF_sdrc_d26_HI        (PAD_CONFIG_BASE + 2 + 0x0064)
#define  CONTROL_PADCONF_sdrc_d28           (PAD_CONFIG_BASE + 0x0068)
#define  CONTROL_PADCONF_sdrc_d28_HI        (PAD_CONFIG_BASE + 2 + 0x0068)
#define  CONTROL_PADCONF_sdrc_d30           (PAD_CONFIG_BASE + 0x006C)
#define  CONTROL_PADCONF_sdrc_d30_HI        (PAD_CONFIG_BASE + 2 + 0x006C)
#define  CONTROL_PADCONF_sdrc_clk           (PAD_CONFIG_BASE + 0x0070)
#define  CONTROL_PADCONF_sdrc_clk_HI        (PAD_CONFIG_BASE + 2 + 0x0070)
#define  CONTROL_PADCONF_sdrc_dqs1          (PAD_CONFIG_BASE + 0x0074)
#define  CONTROL_PADCONF_sdrc_dqs1_HI       (PAD_CONFIG_BASE + 2 + 0x0074)
#define  CONTROL_PADCONF_sdrc_dqs3          (PAD_CONFIG_BASE + 0x0078)
#define  CONTROL_PADCONF_sdrc_dqs3_HI       (PAD_CONFIG_BASE + 2 + 0x0078)

#endif

/*==== TYPEDEFINES =========================================================*/

/*==== STATIC & GLOBAL ====================================================*/

/*====== FUNCTION PROTOTYPES================================================*/

#endif /* SILICON_SDRC_H */
