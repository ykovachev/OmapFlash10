/**
 * @file interrupt.h
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
 * @brief 
 * @details 
 * @todo file details
 * @see <link>
 * @todo update link
 */

/*==== DECLARATION CONTROL =================================================*/

#ifndef INTERRUPT_H
#define INTERRUPT_H

/*==== INCLUDES ============================================================*/
 /*TBD*/
/*==== MACROS ============================================================*/
#define MAX_NUMBER_OF_INTERRUPTS			          96
#define MAX_NUMBER_OF_BANKS                     3
#define MAX_NUMBER_OF_EXCEPTIONS			          8
#define DAL_INT_MAX_EXCEPTIONS						      8
#define DAL_INT_MASK_ALL                        0xFFFFFFFF
#define DAL_INT_ILR_PRIORITY_BIT_POSITION       0x02
#define DAL_INT_PRIORITY_BITS_MASK              0x03
#define DAL_INT_ROUTING_BIT_POSITION            0x01
#define DAL_INT_ROUTING_MASK                    0xFFFFFFFE
#define MASK_CLEAR		                          0
#define MASK_SET		                            1
/* Interrupt Controller Register Addresses */
/* Group 0 registers - OCP socket system regs, only offsets from the base */
#define INTC_REVISION_OFFSET		                (0x00)
#define INTC_SYSCONFIG_OFFSET		                (0x10)
#define INTC_SYSSTATUS_OFFSET                   (0x14)
/* Group 1 registers - Control and status		*/
#define INTC_SIR_IRQ_OFFSET		                  (0x40)
#define INTC_SIR_FIQ_OFFSET		                  (0x44)
#define INTC_CONTROL_OFFSET		                  (0x48)
#define INTC_PROTECTION_OFFSET	                (0x4C)
#define INTC_IDLE_OFFSET		                    (0x50)
/* Group 2 registers - Control and status bank 0 */
#define INTC_BANK0_BASE_OFFSET                  (0x80)
#define INTC_BANK1_BASE_OFFSET                  (0xA0)
#define INTC_BANK2_BASE_OFFSET                  (0xC0)
/*Registers inside the bank, offsets specified related to Bank Base*/
#define INTC_ITR_OFFSET		                      (0x00)
#define INTC_MIR_OFFSET			                    (0x04)
#define INTC_MIR_CLEAR_OFFSET		                (0x08)
#define INTC_MIR_SET_OFFSET		                  (0x0C)
#define INTC_ISR_SET_OFFSET		                  (0x10)
#define INTC_ISR_CLEAR_OFFSET		                (0x14)
#define INTC_PENDING_IRQ_OFFSET	                (0x18)
#define INTC_PENDING_FIQ_OFFSET	                (0x1C)
/*LLR offsets from the interrupt controller base*/
/* Bank 0 ILR reg base */
#define INTC_BANK0_ILR		                      (0x100)
/* Bank 1 ILR reg base */
#define INTC_BANK1_ILR		                      (0x180)
/* Bank 2 ILR reg base */
#define INTC_BANK2_ILR		                      (0x200)
#define INTH_LEVEL                              0
#define EXCEPTION_DAL_UNDEF                     1
#define EXCEPTION_DAL_SWI                       2
#define EXCEPTION_DAL_DATA_ABORT                3
#define EXCEPTION_DAL_PREFETCH                  4
#define EXCEPTION_COUNT                         4
#ifndef __ASM_HEADER__
enum E_343x_INTRRUPTS
{
	INT_EMUINT = 0,
	INT_COMMRX = 1,
	INT_COMMTX = 2,
	INT_BENCH = 3,
	INT_XTI_IRQ = 4,
	INT_XTI_WKUP_IRQ = 5,
	INT_FREE1 = 6,
	INT_SYSIRQ = 7,
	INT_FREE2 = 8,
	INT_FREE3 = 9,
	INT_L3_IRQ = 10,
	INT_PRCM_MPU_IRQ = 11,
	INT_SDMA_IRQ0 = 12,
	INT_SDMA_IRQ1 = 13,
	INT_SDMA_IRQ2 = 14,
	SDMA_IRQ3 = 15,
	INT_FREE4 = 16,
	INT_FREE5 = 17,
	INT_FREE6 = 18,
	INT_FREE7 = 19,
	INT_GPMC_IRQ = 20,
	INT_GFX_IRQ = 21,
	INT_IVA_IRQ = 22,
	INT_EAC_IRQ = 23,
	INT_CAM_MPU_IRQ = 24,
	INT_DSS_IRQ = 25,
	INT_MAIL_MPU_IRQ = 26,
	INT_FREE8 = 27,
	INT_FREE9 = 28,
	INT_GPIO1_MPU_IRQ = 29,
	INT_GPIO2_MPU_IRQ = 30,
	INT_GPIO3_MPU_IRQ = 31,
	INT_GPIO4_MPU_IRQ = 32,
	INT_GPIO5_MPU_IRQ = 33,
	INT_GPIO6_MPU_IRQ = 34,
	INT_USIM_IRQ = 35,
	INT_WDT3_OVF = 36,
	INT_GPT1_IRQ = 37,
	INT_GPT2_IRQ = 38,
	INT_GPT3_IRQ = 39,
	INT_GPT4_IRQ = 40,
	INT_GPT5_IRQ = 41,
	INT_GPT6_IRQ = 42,
	INT_GPT7_IRQ = 43,
	INT_GPT8_IRQ = 44,
	INT_GPT9_IRQ = 45,
	INT_GPT10_IRQ = 46,
	INT_GPT11_IRQ = 47,
	INT_GPT12_IRQ = 48,
	INT_FREE12 = 49,
	INT_PKA_IRQ = 50,
	INT_SHA1MD5_IRQ = 51,
	INT_RNG_IRQ = 52,
	INT_MG_IRQ = 53,
	INT_FREE13 = 54,
	INT_FREE14 = 55,
	INT_I2C1_IRQ = 56,
	INT_I2C2_IRQ = 57,
	INT_HDQ_IRQ = 58,
	INT_MCBSP1_IRQ_TX = 59,
	INT_MCBSP1_IRQ_RX = 60,
	INT_MCBSP1_IRQ_OV = 61,
	INT_MCBSP2_IRQ_TX = 62,
	INT_MCBSP2_IRQ_RX = 63,
	INT_MCBSP2_IRQ_OV = 64,
	INT_SPI1_IRQ = 65,
	INT_SPI2_IRQ = 66,
	INT_SSI_P1_MPU_IRQ0 = 67,
	INT_SSI_P1_MPU_IRQ1 = 68,
	INT_SSI_P2_MPU_IRQ0 = 69,
	INT_SSI_P2_MPU_IRQ1 = 70,
	INT_SSI_GDD_MPU_IRQ = 71,
	INT_UART1_IRQ = 72,
	INT_UART2_IRQ = 73,
	INT_UART3_IRQ = 74,
	INT_USB_IRQ_GEN = 75,
	INT_USB_IRQ_NISO = 76,
	INT_USB_IRQ_ISO = 77,
	INT_USB_IRQ_HGEN = 78,
	INT_USB_IRQ_HSOF = 79,
	INT_USB_IRQ_OTG = 80,
	INT_VLYNQ_IRQ = 81,
	INT_FREE15 = 82,
	INT_MMC_IRQ = 83,
	INT_MS_IRQ = 84,
	INT_FAC_IRQ = 85,
	INT_MMC2_IRQ = 86,
	INT_ARM11_ICR_IRQ = 87,
	INT_D2DFRINT = 88,
	INT_MCBSP3_IRQ_TX = 89,
	INT_MCBSP3_IRQ_RX = 90,
	INT_SPI3_IRQ = 91,
	INT_HS_USB_MC_NINT = 92,
	INT_HS_USB_DMA_NINT = 93,
	INT_Carkit_IRQ = 94,
	INT_FREE25 = 95
};

#ifdef __cplusplus
extern "C" {
#endif

S32 interrupt_init (void);

S32 dal_interrupt_set_sensitivity(U32 int_num, U32 sensitivity);
U32 dal_interrupt_get_sensitivity(U32 int_num, U8 * sensitivity);

S32 dal_interrupt_set_priority(U32 int_num, U32 priority);
S32 dal_interrupt_get_priority(U32 int_num, U32 * priority);

S32 dal_interrupt_get_routing(U32 int_num, U32 * routing);
S32 dal_interrupt_set_routing(U32 int_num, U32 routing);

void dal_enable_interrupts(void);
void dal_disable_interrupts(void);

S32 dal_interrupt_mask(U32 int_num, U8 set_flag);
void dal_interrupt_mask_all(void);

#ifdef __cplusplus
}
#endif
#endif /* #ifndef __ASM_HEADER__ */

#endif /* INTERRUPT_H */
