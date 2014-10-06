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
 * @file silicon_emif.h
 * @author Jens Odborg
 * @brief Contains the register addresses of the EMIF interfaces present in the silicon.
 *
 * @details 
 * The values are taken from the EMIF Reegisters Summary of the 4430 TRM
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM/OMAP4430_NDA_TRM_MEMSS_EMIF_v3.0.pdf
 * @todo updata link
 */

#ifndef SILICON_EMIF_H
#define SILICON_EMIF_H

#if 0
/* EMIF Register Sets */
#define EMIF_BASE_ADDR(emif)               (EMIF##emif##_BASE_ADDR)

/* EMIF Registers */
#define EMIF_MOD_ID_REV(emif)              (EMIF_BASE_ADDR(emif) + 0x0000)
#define EMIF_STATUS(emif)                  (EMIF_BASE_ADDR(emif) + 0x0004)
#define EMIF_SDRAM_CONFIG(emif)            (EMIF_BASE_ADDR(emif) + 0x0008)
#define EMIF_LPDDR2_NVM_CONFIG(emif)       (EMIF_BASE_ADDR(emif) + 0x000C)
#define EMIF_SDRAM_REF_CTRL(emif)          (EMIF_BASE_ADDR(emif) + 0x0010)
#define EMIF_SDRAM_REF_CTRL_SHDW(emif)     (EMIF_BASE_ADDR(emif) + 0x0014)
#define EMIF_SDRAM_TIM_1(emif)             (EMIF_BASE_ADDR(emif) + 0x0018)
#define EMIF_SDRAM_TIM_1_SHDW(emif)        (EMIF_BASE_ADDR(emif) + 0x001C)
#define EMIF_SDRAM_TIM_2(emif)             (EMIF_BASE_ADDR(emif) + 0x0020)
#define EMIF_SDRAM_TIM_2_SHDW(emif)        (EMIF_BASE_ADDR(emif) + 0x0024)
#define EMIF_SDRAM_TIM_3(emif)             (EMIF_BASE_ADDR(emif) + 0x0028)
#define EMIF_SDRAM_TIM_3_SHDW(emif)        (EMIF_BASE_ADDR(emif) + 0x002C)
#define EMIF_LPDDR2_NVM_TIM(emif)          (EMIF_BASE_ADDR(emif) + 0x0030)
#define EMIF_LPDDR2_NVM_TIM_SHDW(emif)     (EMIF_BASE_ADDR(emif) + 0x0034)
#define EMIF_PWR_MGMT_CTRL(emif)           (EMIF_BASE_ADDR(emif) + 0x0038)
#define EMIF_PWR_MGMT_CTRL_SHDW(emif)      (EMIF_BASE_ADDR(emif) + 0x003C)
#define EMIF_LPDDR2_MODE_REG_DATA(emif)    (EMIF_BASE_ADDR(emif) + 0x0040)
#define EMIF_LPDDR2_MODE_REG_CFG(emif)     (EMIF_BASE_ADDR(emif) + 0x0050)
#define EMIF_L3_CONFIG(emif)               (EMIF_BASE_ADDR(emif) + 0x0054)
#define EMIF_L3_CFG_VAL_1(emif)            (EMIF_BASE_ADDR(emif) + 0x0058)
#define EMIF_L3_CFG_VAL_2(emif)            (EMIF_BASE_ADDR(emif) + 0x005C)
#define EMIF_PERF_CNT_1(emif)              (EMIF_BASE_ADDR(emif) + 0x0080)
#define EMIF_PERF_CNT_2(emif)              (EMIF_BASE_ADDR(emif) + 0x0084)
#define EMIF_PERF_CNT_CFG(emif)            (EMIF_BASE_ADDR(emif) + 0x0088)
#define EMIF_PERF_CNT_SEL(emif)            (EMIF_BASE_ADDR(emif) + 0x008C)
#define EMIF_PERF_CNT_TIM(emif)            (EMIF_BASE_ADDR(emif) + 0x0090)
#define EMIF_READ_IDLE_CTRL(emif)          (EMIF_BASE_ADDR(emif) + 0x0090)
#define EMIF_READ_IDLE_CTRL_SHDW(emif)     (EMIF_BASE_ADDR(emif) + 0x0090)
#define EMIF_IRQSTATUS_RAW_SYS(emif)       (EMIF_BASE_ADDR(emif) + 0x00A4)
#define EMIF_IRQSTATUS_RAW_LL(emif)        (EMIF_BASE_ADDR(emif) + 0x00A8)
#define EMIF_IRQSTATUS_SYS(emif)           (EMIF_BASE_ADDR(emif) + 0x00AC)
#define EMIF_IRQSTATUS_LL(emif)            (EMIF_BASE_ADDR(emif) + 0x00B0)
#define EMIF_IRQENABLE_SET_SYS(emif)       (EMIF_BASE_ADDR(emif) + 0x00B4)
#define EMIF_IRQENABLE_SET_LL(emif)        (EMIF_BASE_ADDR(emif) + 0x00B8)
#define EMIF_IRQENABLE_CLR_SYS(emif)       (EMIF_BASE_ADDR(emif) + 0x00BC)
#define EMIF_IRQENABLE_CLR_LL(emif)        (EMIF_BASE_ADDR(emif) + 0x00C0)
#define EMIF_ZQ_CONFIG(emif)               (EMIF_BASE_ADDR(emif) + 0x00C8)
#define EMIF_TEMP_ALERT_CONFIG(emif)       (EMIF_BASE_ADDR(emif) + 0x00CC)
#define EMIF_L3_ERR_LOG(emif)              (EMIF_BASE_ADDR(emif) + 0x00D0)
#define EMIF_DDR_PHY_CTRL_1(emif)          (EMIF_BASE_ADDR(emif) + 0x00E4)
#define EMIF_DDR_PHY_CTRL_1_SHDW(emif)     (EMIF_BASE_ADDR(emif) + 0x00E8)
#define EMIF_DDR_PHY_CTRL_2(emif)          (EMIF_BASE_ADDR(emif) + 0x00EC)

/*EMIF_SDRAM_CONFIG*/
// 31:29 REG_SDRAM_TYPE
#define EMIF_REG_SDRAM_TYPE_SHIFT                    29 
#define EMIF_REG_SDRAM_TYPE_LPDDR2_S4                (4 << EMIF_REG_SDRAM_TYPE_SHIFT)
#define EMIF_REG_SDRAM_TYPE_LPDDR2_S2                (5 << EMIF_REG_SDRAM_TYPE_SHIFT)
// 28:27 REG_IBANK_POS
#define EMIF_REG_IBANK_POS_SHIFT                     27 
#define EMIF_REG_IBANK_POS_TYPE_MASK                 (0x3 << EMIF_REG_IBANK_POS_SHIFT)
// 26:16 RESERVED 
// 15:14 REG_NARROW_MODE
#define EMIF_REG_NARROW_MODE_SHIFT                   14
#define EMIF_REG_NARROW_MODE_32_BIT_DATA_BUS         (0 << EMIF_REG_NARROW_MODE_SHIFT)
#define EMIF_REG_NARROW_MODE_16_BIT_DATA_BUS         (1 << EMIF_REG_NARROW_MODE_SHIFT)
// 13:10 REG_CL (CAS Latency)
#define EMIF_REG_CL_SHIFT                            10
#define EMIF_REG_CL_3                                (3 << EMIF_REG_CL_SHIFT)
#define EMIF_REG_CL_4                                (4 << EMIF_REG_CL_SHIFT)
#define EMIF_REG_CL_5                                (5 << EMIF_REG_CL_SHIFT)
#define EMIF_REG_CL_6                                (6 << EMIF_REG_CL_SHIFT)
#define EMIF_REG_CL_7                                (7 << EMIF_REG_CL_SHIFT)
#define EMIF_REG_CL_8                                (8 << EMIF_REG_CL_SHIFT)
//  9:7  REG_ROWSIZE
#define EMIF_REG_ROWSIZE_SHIFT                       7
#define EMIF_REG_ROWSIZE_9_ROW_ADDRESS_BITS          (0 << EMIF_REG_ROWSIZE_SHIFT)
#define EMIF_REG_ROWSIZE_10_ROW_ADDRESS_BITS         (1 << EMIF_REG_ROWSIZE_SHIFT)
#define EMIF_REG_ROWSIZE_11_ROW_ADDRESS_BITS         (2 << EMIF_REG_ROWSIZE_SHIFT)
#define EMIF_REG_ROWSIZE_12_ROW_ADDRESS_BITS         (3 << EMIF_REG_ROWSIZE_SHIFT)
#define EMIF_REG_ROWSIZE_13_ROW_ADDRESS_BITS         (4 << EMIF_REG_ROWSIZE_SHIFT)
#define EMIF_REG_ROWSIZE_14_ROW_ADDRESS_BITS         (5 << EMIF_REG_ROWSIZE_SHIFT)
#define EMIF_REG_ROWSIZE_15_ROW_ADDRESS_BITS         (6 << EMIF_REG_ROWSIZE_SHIFT)
#define EMIF_REG_ROWSIZE_16_ROW_ADDRESS_BITS         (7 << EMIF_REG_ROWSIZE_SHIFT)
//  6:4  REG_IBANK
#define EMIF_REG_IBANK_SHIFT                         4
#define EMIF_REG_IBANK_1_BANK                        (0 << EMIF_REG_IBANK_SHIFT)
#define EMIF_REG_IBANK_2_BANK                        (1 << EMIF_REG_IBANK_SHIFT)
#define EMIF_REG_IBANK_4_BANK                        (2 << EMIF_REG_IBANK_SHIFT)
#define EMIF_REG_IBANK_8_BANK                        (3 << EMIF_REG_IBANK_SHIFT)
//  3    REG_EBANK
#define EMIF_REG_EBANK_SHIFT                         3  
#define EMIF_REG_EBANK_1_CS                          (0 << EMIF_REG_EBANK_SHIFT)
#define EMIF_REG_EBANK_2_CS                          (1 << EMIF_REG_EBANK_SHIFT)
//  2:0  REG_PAGESIZE
#define EMIF_REG_PAGESIZE_SHIFT                      0
#define EMIF_REG_PAGESIZE_256                        (0 << EMIF_REG_PAGESIZE_SHIFT)
#define EMIF_REG_PAGESIZE_512                        (1 << EMIF_REG_PAGESIZE_SHIFT)    
#define EMIF_REG_PAGESIZE_1024                       (2 << EMIF_REG_PAGESIZE_SHIFT)
#define EMIF_REG_PAGESIZE_2048                       (3 << EMIF_REG_PAGESIZE_SHIFT)

/*EMIF_SDRAM_REF_CTRL*/
// 31    REG_INTREF_DIS
#define EMIF_REG_INTREF_DIS                          (1 << 31)
// 30:16 RESERVED
// 15:00 REG_REFRESH_RATE

/*EMIF_DDR_PHY_CTRL_1*/
// 31:30 RESERVED
// 29:26 REG_PHY_FREEZE_DELAY_CODE_POSTAMBLE
// 25:22 REG_PHY_FREEZE_DELAY_CODE_PREAMBLE
// 21:12 REG_DLL_MASTER_SW_CODE_CTRL
// 11:4  REG_DL_SLAVE_DLY_CTRL
//  3:0  REG_READ_LATENCY
#define EMIF_REG_READ_LATENCY_SHIFT                  0
#define EMIF_REG_READ_LATENCY_MASK                   (0xF << EMIF_REG_READ_LATENCY_SHIFT)
#endif

#endif /* SILICON_EMIF_H */
