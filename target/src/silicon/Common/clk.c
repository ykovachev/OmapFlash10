/**
 * @file 
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
 * This File contains the GPIO framework API functions.
 */

/*========================== DECLARATION CONTROL ============================*/

#ifndef CLK_C
#define CLK_C

//#if 0 // NONE OF THIS STUFF SHOULD BE IN THE SECOND - USE BOARD CONFIG!!!!


/*=============================== INCLUDES ==================================*/

#include "csst_tgt.h"
#include "silicon.h"
#include "clk.h"
#include "uart.h"

/*=========================== PUBLIC FUNCTIONS ==============================*/
#if 0
/*-----------------------------------------------------------------------------
| Function    :void Resetallclocks
+------------------------------------------------------------------------------
| Description :This function is to Reset clocks to defaults
|              We leave UART1,3 and GPT1 enabled for our testcases to run
|
| Parameters  :void
|
| Returns     :void
+-----------------------------------------------------------------------------*/
void Resetallclocks()
{
#if 0 ///@todo may need a revisit later, but for now, removing this enables branch to u-boot after SDRAM download
#ifdef OMAP3XXX ///@todo revisit Resetallclocks
    /* CORE Domain */
    out_regl(CM_FCLKEN1_CORE, 0x00000000); 
    out_regl(CM_ICLKEN1_CORE, 0x00000000 | 0x1 << 1); /*SDRC interface clock is enabled */
    out_regl(CM_FCLKEN3_CORE, 0x00000000 );
    out_regl(CM_ICLKEN3_CORE, 0x00000000 );
    /* Wakeup Domain */
    out_regl(CM_FCLKEN_WKUP, 0x00000000 | 0x1 << 5); /* WDTIMER 2 functional clock is enabled */
    out_regl(CM_ICLKEN_WKUP, 0x00000000 | 0x1 << 5); /* WDTIMER 2 interface clock is enabled */
    /* Peripheral Domain */
    out_regl(CM_FCLKEN_PER, 0x00000000); /* Note: UART3 disabled */
    out_regl(CM_ICLKEN_PER, 0x00000000); /* Note: UART3 disabled */
#endif //OMAP3XXX
#endif
}

/*-----------------------------------------------------------------------------
| Function    :void uart_func_clk_en(void)
+------------------------------------------------------------------------------
| Description :This function is to initialize the gpio pin as input or output.
|
| Parameters  :void
|
| Returns     :void
+-----------------------------------------------------------------------------*/
void uart_func_clk_en(U8 uart_no)
{
#ifdef OMAP4430
#ifdef DEBUG_UART
    /*
     * Acording to OMAP4430_NDA_TRM_UART_v3.2 > UART/IrDA/CIR > UART/IrDA/CIR Functional Description > Clock Configuration (1.4.2)
     * Refers:
        OMAP4430_NDA_TRM_PRCM_v3.0 > Device Power Management Introduction > Clock Management Functional Description > CD_L4_CFG Clock Domain 
            > Clock Domain Module Attributes (1.6.14.4)
        OMAP4430_NDA_TRM_PRCM_v3.0 > Device Power Management Introduction > Device Power Management Architecture Building Blocks > Clock Management 
            > Module Level Clock Managment (1.1.1.1.2)

        OMAP4430_NDA_TRM_PRCM_v3.0 > Table 1-25 and Table 1-31
            list UARTi in Power Domain PD_L4_PER and Reset Domain L4_PER_RET_RST

            PM_L4PER_UARTi_WKDEP do not need initialization since we dont do powerdown nothing has to depend on UART wakeup
            RM_L4PER_UART1_CONTEXT nothing here we do not detect softreset
            CM_L4PER_UART1_CLKCTRL enable module
        OMAP4430_NDA_TRM_UART_v3.2
            UART_REG_SYSC enable clock
     */
    ///@todo do we need to power on the PD_L4_PER domain?
    U32 cm_l4per_uart_clkctrl = CM_L4PER_UART1_CLKCTRL + uart_no * 8;
    U32 templ;
    templ = in_regl (cm_l4per_uart_clkctrl);
    templ |= 0x00000002;
    out_regl (cm_l4per_uart_clkctrl, templ);
    out_regl (UART_REG_SYSC(uart_no), UART_SYSC_IDLEMODE_NEVER);
#endif //DEBUG_UART
#endif //OMAP4430
#ifdef OMAP3XXX
    U32 templ;
    /*enable the function clock for UART1 and UART2 */
    templ = in_regl(CM_FCLKEN1_CORE);
    templ |= 0x00006000;
    out_regl(CM_FCLKEN1_CORE, templ);
    /*enable the function clock for UART3 */
    templ = in_regl(CM_FCLKEN_PER);
    templ |= 0x00000800;
    out_regl(CM_FCLKEN_PER, templ);
#endif //OMAP3XXX
}

#endif

#endif /* CLK_C */
