/**
 * @file triton2_utils.h
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
 * This is the h-file for the Triton2 platform utility functions (triton2_utils.c)
 * 
 */

/*==== DECLARATION CONTROL =================================================*/
#ifndef TRITON2_UTILS_H
#define TRITON2_UTILS_H

/*==== INCLUDES ============================================================*/

/*==== DEFINES ============================================================*/
#define TRITON2_GPIO_INPUT  0
#define TRITON2_GPIO_OUTPUT 1

#define TRITON2_GPIO_SET_DATA   1
#define TRITON2_GPIO_CLEAR_DATA 0

#define TRITON2_AUDIO_MUTE_ENABLE  1
#define TRITON2_AUDIO_MUTE_DISABLE 0

#define TRITON2_SUB_LCD_BKLIGHT_ENABLE  1
#define TRITON2_SUB_LCD_BKLIGHT_DISABLE 0

#define VMMC1_V1P8 0
#define VMMC1_V3P0 1

#define VPLL2_V1P8	1
#define VPLL2_V1P3	2

#define VSIM_V1P8 0
#define VSIM_V2P8 1
#define VSIM_V3P0 2

#define TRITON2_GPIO_0  0
#define TRITON2_GPIO_1  1
#define TRITON2_GPIO_2  2
#define TRITON2_GPIO_3  3
#define TRITON2_GPIO_4  4
#define TRITON2_GPIO_5  5
#define TRITON2_GPIO_6  6
#define TRITON2_GPIO_7  7
#define TRITON2_GPIO_8  8
#define TRITON2_GPIO_9  9
#define TRITON2_GPIO_10 10
#define TRITON2_GPIO_11 11
#define TRITON2_GPIO_12 12
#define TRITON2_GPIO_13 13
#define TRITON2_GPIO_14 14
#define TRITON2_GPIO_15 15
#define TRITON2_GPIO_16 16
#define TRITON2_GPIO_17 17
/*==== TYPES ===============================================================*/

/* Prototype Functions */

U32	enable_triton2_vdac();
U32	disable_triton2_vdac();

U32	enable_triton2_vpll1();
U32	disable_triton2_vpll1();

U32	enable_triton2_vpll2(U8 voltage);
U32	disable_triton2_vpll2();

U32	enable_triton2_vmmc1(U8 voltage);
U32	disable_triton2_vmmc1(U8 voltage);

U32	enable_triton2_vmmc2();
U32	disable_triton2_vmmc2();

U32	enable_triton2_vaux1();
U32	disable_triton2_vaux1();

U32	enable_triton2_vaux2();
U32	disable_triton2_vaux2();

U32	enable_triton2_vaux3();
U32	disable_triton2_vaux3();

U32	enable_triton2_vaux4();
U32	disable_triton2_vaux4();

U32 enable_triton2_vsim(U8 voltage);
U32 disable_triton2_vsim(U8 voltage);

S32 triton2_gpio_pin_init(U8 pin_num, U8 in_out);
S32 triton2_get_gpio_input(U32 pin_num, U8 *gpio_data);
S32 triton2_set_gpio_output(U16 pin_num, U8 set);
S32 triton2_gpio_pin_deinit(U16 pin_num);
S32 simcard_detect();
S32 headset_detect();
S32 audio_mute_control(U8 enable_disable);
S32 mmc_card_detect_t2(U8 mmc_slot_num);
S32 triton2_clear_startup_interrupts();
S32 spi_lcd_bklight(U8 enable_disable);
S32 pri_lcd_bklight(U8 enable_disable);
S32 triton2_bootconfig();
S32 mmc_write_protect_t2(U8 mmc_slot_num);
S32 disable_triton2_i2s_interface(void);
#ifdef OMAP3430MDK
S32 mux_dac_to_stereojack(void);
S32 demux_dac_to_stereojack(void);
#endif /* OMAP3430MDK */
#ifdef HDMI_MCLK_FROM_T2_256FS
S32 triton2_clk256fs_enable(void);
#endif /*HDMI_MCLK_FROM_T2_256FS*/
#endif /* TRITON2_UTILS_H */

