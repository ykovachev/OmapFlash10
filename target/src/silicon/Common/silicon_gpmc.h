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
 * @file silicon_gpmc.h
 * @author Jens Odborg
 * @brief Contains the base addresses of the interfaces or peripherals present in the silicon.
 * @details 
 * @todo file details
 * @see https://dncsps.itg.ti.com/sites/CS-IC-doc/trm_and_dm_repository/OMAP4430%20TRM/OMAP4430_NDA_TRM_GPMC_v3.2.pdf
 * @todo updata link
 */

#ifndef SILICON_GPMC_H
#define SILICON_GPMC_H

#define GPMC_SYSCONFIG                              (GPMC_BASE_ADDR + 0x010)
#define GPMC_IRQENABLE                              (GPMC_BASE_ADDR + 0x01C)
#define GPMC_TIMEOUT_CONTROL                        (GPMC_BASE_ADDR + 0x040)

#define GPMC_CONFIG1_0                              (GPMC_BASE_ADDR + 0x060)
#define GPMC_CONFIG2_0                              (GPMC_BASE_ADDR + 0x064)
#define GPMC_CONFIG3_0                              (GPMC_BASE_ADDR + 0x068)
#define GPMC_CONFIG4_0                              (GPMC_BASE_ADDR + 0x06C)
#define GPMC_CONFIG5_0                              (GPMC_BASE_ADDR + 0x070)
#define GPMC_CONFIG6_0                              (GPMC_BASE_ADDR + 0x074)
#define GPMC_CONFIG7_0                              (GPMC_BASE_ADDR + 0x078)

#define GPMC_NAND_COMMAND(baseaddress,chipselect)   (baseaddress + 0x07C + (GPMC_CONFIG1_1 - GPMC_CONFIG1_0) * chipselect)
#define GPMC_NAND_ADDRESS(baseaddress,chipselect)   (baseaddress + 0x080 + (GPMC_CONFIG1_1 - GPMC_CONFIG1_0) * chipselect)
#define GPMC_NAND_DATA(baseaddress,chipselect)      (baseaddress + 0x084 + (GPMC_CONFIG1_1 - GPMC_CONFIG1_0) * chipselect)

#define GPMC_CONFIG1_1                              (GPMC_BASE_ADDR + 0x090)
#define GPMC_CONFIG2_1                              (GPMC_BASE_ADDR + 0x094)
#define GPMC_CONFIG3_1                              (GPMC_BASE_ADDR + 0x098)
#define GPMC_CONFIG4_1                              (GPMC_BASE_ADDR + 0x09C)
#define GPMC_CONFIG5_1                              (GPMC_BASE_ADDR + 0x0A0)
#define GPMC_CONFIG6_1                              (GPMC_BASE_ADDR + 0x0A4)
#define GPMC_CONFIG7_1                              (GPMC_BASE_ADDR + 0x0A8)

#define GPMC_CONFIG1_2                              (GPMC_BASE_ADDR + 0x0C0)
#define GPMC_CONFIG2_2                              (GPMC_BASE_ADDR + 0x0C4)
#define GPMC_CONFIG3_2                              (GPMC_BASE_ADDR + 0x0C8)
#define GPMC_CONFIG4_2                              (GPMC_BASE_ADDR + 0x0CC)
#define GPMC_CONFIG5_2                              (GPMC_BASE_ADDR + 0x0D0)
#define GPMC_CONFIG6_2                              (GPMC_BASE_ADDR + 0x0D4)
#define GPMC_CONFIG7_2                              (GPMC_BASE_ADDR + 0x0D8)

#define GPMC_CONFIG1_3                              (GPMC_BASE_ADDR + 0x0F0)
#define GPMC_CONFIG2_3                              (GPMC_BASE_ADDR + 0x0F4)
#define GPMC_CONFIG3_3                              (GPMC_BASE_ADDR + 0x0F8)
#define GPMC_CONFIG4_3                              (GPMC_BASE_ADDR + 0x0FC)
#define GPMC_CONFIG5_3                              (GPMC_BASE_ADDR + 0x100)
#define GPMC_CONFIG6_3                              (GPMC_BASE_ADDR + 0x104)
#define GPMC_CONFIG7_3                              (GPMC_BASE_ADDR + 0x108)

#define GPMC_CONFIG1_4                              (GPMC_BASE_ADDR + 0x120)
#define GPMC_CONFIG2_4                              (GPMC_BASE_ADDR + 0x124)
#define GPMC_CONFIG3_4                              (GPMC_BASE_ADDR + 0x128)
#define GPMC_CONFIG4_4                              (GPMC_BASE_ADDR + 0x12C)
#define GPMC_CONFIG5_4                              (GPMC_BASE_ADDR + 0x130)
#define GPMC_CONFIG6_4                              (GPMC_BASE_ADDR + 0x134)
#define GPMC_CONFIG7_4                              (GPMC_BASE_ADDR + 0x138)

#define GPMC_CONFIG1_5                              (GPMC_BASE_ADDR + 0x150)
#define GPMC_CONFIG2_5                              (GPMC_BASE_ADDR + 0x154)
#define GPMC_CONFIG3_5                              (GPMC_BASE_ADDR + 0x158)
#define GPMC_CONFIG4_5                              (GPMC_BASE_ADDR + 0x15C)
#define GPMC_CONFIG5_5                              (GPMC_BASE_ADDR + 0x160)
#define GPMC_CONFIG6_5                              (GPMC_BASE_ADDR + 0x164)
#define GPMC_CONFIG7_5                              (GPMC_BASE_ADDR + 0x168)

#define GPMC_CONFIG1_6                              (GPMC_BASE_ADDR + 0x180)
#define GPMC_CONFIG2_6                              (GPMC_BASE_ADDR + 0x184)
#define GPMC_CONFIG3_6                              (GPMC_BASE_ADDR + 0x188)
#define GPMC_CONFIG4_6                              (GPMC_BASE_ADDR + 0x18C)
#define GPMC_CONFIG5_6                              (GPMC_BASE_ADDR + 0x190)
#define GPMC_CONFIG6_6                              (GPMC_BASE_ADDR + 0x194)
#define GPMC_CONFIG7_6                              (GPMC_BASE_ADDR + 0x198)

#define GPMC_CONFIG1_7                              (GPMC_BASE_ADDR + 0x1B0)
#define GPMC_CONFIG2_7                              (GPMC_BASE_ADDR + 0x1B4)
#define GPMC_CONFIG3_7                              (GPMC_BASE_ADDR + 0x1B8)
#define GPMC_CONFIG4_7                              (GPMC_BASE_ADDR + 0x1BC)
#define GPMC_CONFIG5_7                              (GPMC_BASE_ADDR + 0x1C0)
#define GPMC_CONFIG6_7                              (GPMC_BASE_ADDR + 0x1C4)
#define GPMC_CONFIG7_7                              (GPMC_BASE_ADDR + 0x1C8)

#define NAND_GPMC_CS0_COMMAND                       (GPMC_BASE_ADDR + 0x07C)
#define NAND_GPMC_CS0_ADDRESS                       (GPMC_BASE_ADDR + 0x080)
#define NAND_GPMC_CS0_DATA                          (GPMC_BASE_ADDR + 0x084)

/*#define GPMC_CONFIG                                 (GPMC_BASE_ADDR + 0x050) */
#define GPMC_ECC_CONFIG                             *(volatile U32 *) (GPMC_BASE_ADDR + 0x1F4)
#define GPMC_ECC_CONTROL                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x1F8)
#define GPMC_ECC_SIZE_CONFIG                        *(volatile U32 *) (GPMC_BASE_ADDR + 0x1FC)
#define GPMC_ECC1_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x200)
#define GPMC_ECC2_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x204)
#define GPMC_ECC3_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x208)
#define GPMC_ECC4_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x20C)
#define GPMC_ECC5_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x210)
#define GPMC_ECC6_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x214)
#define GPMC_ECC7_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x218)
#define GPMC_ECC8_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x21C)
#define GPMC_ECC9_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x220)
#define GPMC_ECC_RESULT_BASE                                          (GPMC_BASE_ADDR + 0x200)

#define GPMC_CONFIG                                 (GPMC_BASE_ADDR + 0x050)
#define GPMC_ECC_CONFIG                             *(volatile U32 *) (GPMC_BASE_ADDR + 0x1F4)
#define GPMC_ECC_CONTROL                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x1F8)
#define GPMC_ECC_SIZE_CONFIG                        *(volatile U32 *) (GPMC_BASE_ADDR + 0x1FC)
#define GPMC_ECC1_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x200)
#define GPMC_ECC2_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x204)
#define GPMC_ECC3_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x208)
#define GPMC_ECC4_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x20C)
#define GPMC_ECC5_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x210)
#define GPMC_ECC6_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x214)
#define GPMC_ECC7_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x218)
#define GPMC_ECC8_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x21C)
#define GPMC_ECC9_RESULT                            *(volatile U32 *) (GPMC_BASE_ADDR + 0x220)
#define GPMC_ECC_RESULT_BASE                                          (GPMC_BASE_ADDR + 0x200)

#endif //SILICON_GPMC_H
