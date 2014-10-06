/**
 * @file padconfig.c
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
 * This file contains pinmuxing functions
 * 
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef PADCONFIG_MMC_C
#define PADCONFIG_MMC_C

#include "csst_tgt.h"
#include "silicon.h"
#include "types.h"
#include "pinmux_fwrk.h"
#include "padconfig_mmc.h"

#ifndef OMAP4
const PIN_CONFIG mmcsd_pin_mux[] = {
    {CONTROL_PADCONF_mmc1_clk, MODE0, PUPD_DIS, PULL_DOWN}, // mmc1_clk
    {CONTROL_PADCONF_mmc1_clk_HI, MODE0, PUPD_DIS, PULL_DOWN},  // mmc1_cmd
    {CONTROL_PADCONF_mmc1_dat0, MODE0, PUPD_DIS, PULL_DOWN},    // mmc1_dat0
    {CONTROL_PADCONF_mmc1_dat0_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc1_dat1
    {CONTROL_PADCONF_mmc1_dat2, MODE0, PUPD_DIS, PULL_DOWN},    // mmc1_dat2
    {CONTROL_PADCONF_mmc1_dat2_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc1_dat3
    {CONTROL_PADCONF_mmc1_dat4, MODE0, PUPD_DIS, PULL_DOWN},    // mmc1_dat4
    {CONTROL_PADCONF_mmc1_dat4_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc1_dat5
    {CONTROL_PADCONF_mmc1_dat6, MODE0, PUPD_DIS, PULL_DOWN},    // mmc1_dat6
    {CONTROL_PADCONF_mmc1_dat6_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc1_dat7
    {CONTROL_PADCONF_mmc2_clk, MODE0, PUPD_DIS, PULL_DOWN}, // mmc2_clk
    {CONTROL_PADCONF_mmc2_clk_HI, MODE0, PUPD_DIS, PULL_DOWN},  // mmc2_cmd
    {CONTROL_PADCONF_mmc2_dat0, MODE0, PUPD_DIS, PULL_DOWN},    // mmc2_dat0
    {CONTROL_PADCONF_mmc2_dat0_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc2_dat1
    {CONTROL_PADCONF_mmc2_dat2, MODE0, PUPD_DIS, PULL_DOWN},    // mmc2_dat2
    {CONTROL_PADCONF_mmc2_dat2_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc2_dat3
    {CONTROL_PADCONF_mmc2_dat4, MODE0, PUPD_DIS, PULL_DOWN},    // mmc2_dat4
    {CONTROL_PADCONF_mmc2_dat4_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc2_dat5
    {CONTROL_PADCONF_mmc2_dat6, MODE0, PUPD_DIS, PULL_DOWN},    // mmc2_dat6
    {CONTROL_PADCONF_mmc2_dat6_HI, MODE0, PUPD_DIS, PULL_DOWN}, // mmc2_dat7
    {CONTROL_PADCONF_etk_clk_HI, MODE4, PUPD_DIS, PULL_DOWN},   // gpio_13    ->MMC1_WP
    {CONTROL_PADCONF_mcspi2_cs0_HI, MODE4, PUPD_DIS, PULL_DOWN} // gpio_182->MMC2_WP
};

const PIN_CONFIG i2c1_pin_mux[] = {
    {CONTROL_PADCONF_hsusb0_data7_HI, MODE0, PUPD_DIS, PULL_DOWN},  // i2c1_scl
    {CONTROL_PADCONF_i2c1_sda, MODE0, PUPD_DIS, PULL_DOWN}  // i2c1_sda
};

const PIN_CONFIG i2c2_pin_mux[] = {
    {CONTROL_PADCONF_i2c1_sda_HI, MODE0, PUPD_DIS, PULL_DOWN},  // i2c2_scl
    {CONTROL_PADCONF_i2c2_sda, MODE0, PUPD_DIS, PULL_DOWN}  // i2c2_sda
};

const PIN_CONFIG i2c3_pin_mux[] = {
    {CONTROL_PADCONF_i2c2_sda_HI, MODE0, PUPD_DIS, PULL_DOWN},  // i2c3_scl
    {CONTROL_PADCONF_i2c3_sda, MODE0, PUPD_DIS, PULL_DOWN}  // i2c3_sda
};

void mux_setup_mmcsd(void)
{
    U32 mmcsd_num = ((sizeof(mmcsd_pin_mux)) / (sizeof(PIN_CONFIG)));
    //DBG_START();
    do_pin_mux(mmcsd_pin_mux, mmcsd_num);
    //DBG_END();
}

void mux_setup_i2c1(void)
{
    U32 i2c1_num = ((sizeof(i2c1_pin_mux)) / (sizeof(PIN_CONFIG)));
    //DBG_START();
    do_pin_mux(i2c1_pin_mux, i2c1_num);
    //DBG_END();
}

void mux_setup_i2c2(void)
{
    U32 i2c2_num = ((sizeof(i2c2_pin_mux)) / (sizeof(PIN_CONFIG)));
    //DBG_START();
    do_pin_mux(i2c2_pin_mux, i2c2_num);
    //DBG_END();
}

void mux_setup_i2c3(void)
{
    U32 i2c3_num = ((sizeof(i2c3_pin_mux)) / (sizeof(PIN_CONFIG)));
    //DBG_START();
    do_pin_mux(i2c3_pin_mux, i2c3_num);
    //DBG_END();
}
#endif //OMAP4

#endif //PADCONFIG_MMC_C
