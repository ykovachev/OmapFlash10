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
 * @brief Contains various register addresses special to 3430
 * @details 
 * @todo file details
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM
 * @todo updata link
 */

#ifndef SILICON_5430_H
#define SILICON_5430_H

#define CONTROL_PBIAS_LITE                      0x4A002E00

#define CLK32K_COUNTER_REGISTER                 ((volatile U32 *)0x4AE04030)

#define PRM_RSTCTRL                             0x4AE07B00
#define RST_GLOBAL_COLD_SW                      0x00000002
#define RST_GLOBAL_WARM_SW                      0x00000001

/* The following is needed for EMIF access in connection with testing of the memory interface */

#define EMIF1_SDRAM_CONFIG_2                    0x4C00000C
#define EMIF2_SDRAM_CONFIG_2                    0x4D00000C

#define EMIF_SDRAM_CONFIG_2_REG_RDBSIZE(reg)    ((reg >>  0) & 0x07)
#define EMIF_SDRAM_CONFIG_2_REG_RDBNUM(reg)     ((reg >>  4) & 0x03)
#define EMIF_SDRAM_CONFIG_2_REG_EBANK_POS(reg)  ((reg >> 27) & 0x01)
#define EMIF_SDRAM_CONFIG_2_REG_CS1NVMEM(reg)   ((reg >> 30) & 0x01)


#define EMIF1_SDRAM_CONFIG                      0x4C000008
#define EMIF2_SDRAM_CONFIG                      0x4D000008

#define EMIF_SDRAM_CONFIG_REG_PAGESIZE(reg)     ((reg >>  0) & 0x03)
#define EMIF_SDRAM_CONFIG_REG_EBANK(reg)        ((reg >>  3) & 0x01)
#define EMIF_SDRAM_CONFIG_REG_IBANK(reg)        ((reg >>  4) & 0x07)
#define EMIF_SDRAM_CONFIG_REG_ROWSIZE(reg)      ((reg >>  7) & 0x07)
#define EMIF_SDRAM_CONFIG_REG_CL(reg)           ((reg >> 10) & 0x0F)
#define EMIF_SDRAM_CONFIG_REG_NARROW_MODE(reg)  ((reg >> 14) & 0x03)
#define EMIF_SDRAM_CONFIG_REG_DISABLE_DLL(reg)  ((reg >> 20) & 0x01)
#define EMIF_SDRAM_CONFIG_REG_DDR2_DDQS(reg)    ((reg >> 23) & 0x01)
#define EMIF_SDRAM_CONFIG_REG_IBANK_POS(reg)    ((reg >> 27) & 0x03)
#define EMIF_SDRAM_CONFIG_REG_SDRAM_TYPE(reg)   ((reg >> 29) & 0x07)

#define DMM_LISA_MAP_REGISTER_0                 0x4E000040
#define DMM_LISA_MAP_REGISTER_1                 0x4E000044
#define DMM_LISA_MAP_REGISTER_2                 0x4E000048
#define DMM_LISA_MAP_REGISTER_3                 0x4E00004C

#define DMM_LISA_MAP_SDRC_ADDR(reg)             ((reg >>  0) & 0xFF)
#define DMM_LISA_MAP_SDRC_MAP(reg)              ((reg >>  8) & 0x03)
#define DMM_LISA_MAP_SDRC_ADDRSPC(reg)          ((reg >> 16) & 0x03)
#define DMM_LISA_MAP_SDRC_INTL(reg)             ((reg >> 18) & 0x03)
#define DMM_LISA_MAP_SYS_SIZE(reg)              ((reg >> 20) & 0x07)
#define DMM_LISA_MAP_SYS_ADDR(reg)              ((reg >> 24) & 0xFF)

#endif /* SILICON_5430_H */

