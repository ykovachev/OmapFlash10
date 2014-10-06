/**
 * @file i2c.c
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
 * This file contains the low level driver function for i2c
 * 
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef I2C_C
#define I2C_C

/*=======INCLUDES======================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "csst_tgt.h"
#include "i2c.h"
#include "i2c_dis.h"
#include "pinmux_fwrk.h"
#include "silicon.h"
#include "padconfig.h"
#include "dbg_ex.h" 
#include "emmc_drv.h"
//#include "romapi.h"
#include "romapi_types.h"
#ifdef OMAP3
#include "romapi_3430.h" ///<@todo rename silicon_3xxx
#endif
#ifdef OMAP4
#include "romapi_4430.h"
#endif
#ifdef OMAP5
#include "romapi_5430.h"
#endif
#include "dbg_ex.h"

/*=======EXTERNS======================================================*/
//extern volatile U32 i2c_reg_base;
#define i2c_reg_base get_driver_data()->i2c_reg_base
volatile U16 *regtmp;
static U8 i2c_hs = 0;

S32 read_i2c_payload_using_fifo(U16 device,
    U16 size,
    U8 * data);

/* Try just once. if this fails.. return immediately. */
#define RETRY_COUNT 1

#ifdef TRACE_I2C
#define MY_DBG_PRINT DO_DBG_PRINT
#else
#define MY_DBG_PRINT NO_DBG_PRINT
#endif

#define I2C_MSG(CRIT,LVL,PRINT_ARGS) \
    MY_DBG_PRINT(CRIT, CSST_DRV_ID,"I2C:%s:%d:" LVL ": ",__FUNCTION__,__LINE__); \
    PRINT_ARGS; \
    MY_DBG_PRINT(CRIT, CSST_DRV_ID,"\r\n")

#ifdef _MSC_VER
#define i2c_warn(...) I2C_MSG(DBG_LEVEL_CRITICAL,"CRIT", MY_DBG_PRINT(DBG_LEVEL_CRITICAL,__VA_ARGS__))
#define i2c_info(...) I2C_MSG(DBG_LEVEL_MINOR,   "INFO", MY_DBG_PRINT(DBG_LEVEL_MINOR,__VA_ARGS__))
#else
#define i2c_warn(ARGS...) I2C_MSG(DBG_LEVEL_CRITICAL,"CRIT", MY_DBG_PRINT(DBG_LEVEL_CRITICAL,ARGS))
#define i2c_info(ARGS...) I2C_MSG(DBG_LEVEL_MINOR,   "INFO",MY_DBG_PRINT(DBG_LEVEL_MINOR,ARGS))
#endif

#if !defined SECOND_DNLD_BUILD && !defined EMMC_DRV
U8 g_allow_abort_i2c = 0;
static U8 i2c_abort_check(void)
{
    if (g_allow_abort_i2c)
    {
        return disp_check_abort_flag();
    }
    return FALSE;

}

#define I2C_IS_ABORT i2c_abort_check()
#else
/* 2nd downloader */
static U8 i2c_abort_check(void)
{
    return FALSE;
}

/* never abort in Second dnld build mode */
#define I2C_IS_ABORT i2c_abort_check()
#endif

#define DO_NOT_IGNORE_NACK_IRQS
#define DO_NOT_IGNORE_AL_IRQS

/* Enable me to allow NAKs ->DO NOT TRIGGER ERROR ON NAKS */
U8 i2c_allow_nak = 0;

/*-----------------------------------------------------------------------------
| Function    :void configure_i2c(U16 clock_speed)
+------------------------------------------------------------------------------
| Description : This function configures the i2c by setting the own address.
|
| Parameters  :Clock value
|
| Returns     :void
+-----------------------------------------------------------------------------*/
void configure_i2c(U16 i2c_number,
    U16 clock_speed)
{
#ifdef OMAP3
    /*Choose the proper I2C module based on the sid */
    if (i2c_number == 0)
    {
        i2c_reg_base = (U32) I2C1_REG_BASE; /*I2C1 module */
        /*Call mux setup for I2C */
        mux_setup_i2c1();
    }
    else if (i2c_number == 1)
    {
        i2c_reg_base = (U32) I2C2_REG_BASE; /*I2C2 module */
        /*Call mux setup for I2C */
        mux_setup_i2c2();
    }
    else if (i2c_number == 2)
    {
        i2c_reg_base = (U32) I2C3_REG_BASE; /*I2C2 module */
        /*Call mux setup for I2C */
        mux_setup_i2c3();
    }

    if (clock_speed > I2C_400K_CLK)
    {
        i2c_hs = 1;
    }
    else
    {
        i2c_hs = 0;
    }
    clear_i2c();
    set_i2c_clocks(clock_speed);
#endif
#if defined OMAP4 || defined OMAP5
    ///@todo verify this code
    HAL_CTRL_ConfigurePads( HAL_MODULE_I2C, HAL_I2C1 );
    HAL_CM_EnableModuleClocks( HAL_MODULE_I2C, HAL_I2C1 );  
#endif

    /* configure own address and master code */
    out_regs((i2c_reg_base + I2C_OA_OFFSET), (I2C_OA_VAL | I2C_MASTER_CODE));

    /* take the I2C module out of reset */
    en_i2c(I2C_CON_EN, 1);
}

/*-----------------------------------------------------------------------------
| Function    :set_i2c_clocks(U16 clock_speed)
+------------------------------------------------------------------------------
| Description : This routine configures clock for I2C host module. pre scalar
|               value is  configured.
|
| Parameters  :Clock value
|
| Returns     :void
+-----------------------------------------------------------------------------*/
void set_i2c_clocks(U16 clock_speed)
{
    U16 hssclh, hsscll;

    if (clock_speed == I2C_100K_CLK)
    {
        /*This is clock setting calculated for 100kbps */
        /* set prescalar register */
        out_regs((i2c_reg_base + I2C_PSC_OFFSET), I2C_PSC_VAL);
        /* set i2c_scll and i2c_sclh registers */
        out_regs((i2c_reg_base + I2C_SCLL_OFFSET), I2C_SCLL_VAL);
        out_regs((i2c_reg_base + I2C_SCLH_OFFSET), I2C_SCLH_VAL);
        /*
         * Current configuration ends here, remaining part is kept for further
         * improvements
         */
    }
    else if (clock_speed == I2C_400K_CLK)
    {
        /*This is clock setting calculated for 400kbps */
        /* set prescalar register */
        out_regs((i2c_reg_base + I2C_PSC_OFFSET), I2C_FS_PSC_VAL);
        /* set i2c_scll and i2c_sclh registers */
        out_regs((i2c_reg_base + I2C_SCLL_OFFSET), I2C_FS_SCLL_VAL);
        out_regs((i2c_reg_base + I2C_SCLH_OFFSET), I2C_FS_SCLH_VAL);
    }
    else if (clock_speed > I2C_400K_CLK)
    {
        /* HS operation, configure the SCLL and SCLH registers according to
           the HS clock speed */
        hssclh = get_hssclh_val(clock_speed);
        hsscll = get_hsscll_val(clock_speed);
        /* set prescalar register */
        out_regs((i2c_reg_base + I2C_PSC_OFFSET), I2C_HSPSC_VAL);
        /* set i2c_scll and i2c_sclh registers */
        out_regs((i2c_reg_base + I2C_SCLL_OFFSET),
                 ((hsscll << 8) | I2C_FS_SCLL_VAL));
        out_regs((i2c_reg_base + I2C_SCLH_OFFSET),
                 ((hssclh << 8) | I2C_FS_SCLH_VAL));
        /*
         * Current configuration ends here, remaining part is kept for further
         * improvements
         */
    }
}

/*-----------------------------------------------------------------------------
| Function    :get_hssclh_val(U16 clock_speed)
+------------------------------------------------------------------------------
| Description : This routine returns the value to be configured in the HSSCLH
|			    register for HS I2C operation.
|
| Parameters  :Clock value
|
| Returns     : value to be configured
+-----------------------------------------------------------------------------*/
U16 get_hssclh_val(U16 clock_speed)
{
    /* The below values are calculated assuming I2C clock duty cycle is 50%*/
    if (clock_speed == I2C_1P95M_CLK)
    {
        return (0x0013);        /* HSSCLH = ((48/1.95) - 5) */
    }
    else if (clock_speed == I2C_2P6M_CLK)
    {
        return (0x000D);        /* HSSCLH = ((48/2.6) - 5) */
    }
    else if (clock_speed == I2C_3P4M_CLK)
    {
        return (0x000C);        /* HSSCLH = ((48/3.4) - 5) */
    }
    else
    {
        return (0x000C);        /* Assuming 3.4 Mbit/s  */
    }
}

/*-----------------------------------------------------------------------------
| Function    :get_hsscll_val(U16 clock_speed)
+------------------------------------------------------------------------------
| Description : This routine returns the value to be configured in the HSSCLL
|			    register for HS I2C operation.
|
| Parameters  :Clock value
|
| Returns     : value to be configured
+-----------------------------------------------------------------------------*/
U16 get_hsscll_val(U16 clock_speed)
{
    /* All the below values are calculated assuming I2C clock duty cycle is 50%*/
    if (clock_speed == I2C_1P95M_CLK)
    {
        return (0x0011);        /* HSSCLL = ((48/1.95) - 7) */
    }
    else if (clock_speed == I2C_2P6M_CLK)
    {
        return (0x000B);        /* HSSCLL = ((48/2.6) - 7) */
    }
    else if (clock_speed == I2C_3P4M_CLK)
    {
        return (0x0005);        /* HSSCLL = ((48/3.4) - 7) */
    }
    else
    {
        return (0x0005);        /* Assuming 3.4 Mbit/s  */
    }
}

/*-----------------------------------------------------------------------------
| Function    :S32 check_i2c_status (U16 status)
+------------------------------------------------------------------------------
| Description : This routine will check the status,clears it and returns the
|               status.
|
| Parameters  :
|
| Returns     :DAL_I2C_SUCCESS to indicate status is ok.
|              I2C_ERROR to indicate status has error.
+-----------------------------------------------------------------------------*/
S32 check_i2c_status(U16 status)
{
    S32 i2c_ret = DAL_I2C_SUCCESS;

    /*  Arbitration lost. bit 0 */
    if (status & I2C_STAT_AL)
    {
        /*  Clear the bit. */
        i2c_warn("error - Arbitration lost  0x%x", status);
        out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_AL);
#ifdef DO_NOT_IGNORE_AL_IRQS
        i2c_ret = OMAPFLASH_I2C_ER_AL;
#endif
    }

    /*  No-acknowledge.  bit 1 */
    if (status & I2C_STAT_NACK)
    {
        /* Clear the bit.  */
        i2c_warn("error - No-acknowledge  0x%x", status);
        out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_NACK);
#ifdef DO_NOT_IGNORE_NACK_IRQS
        if (!i2c_allow_nak)
            i2c_ret = OMAPFLASH_I2C_ER_NACK;
#endif
    }

    /*  General call. bit 5 */
    if (status & I2C_STAT_GC)
    {
        i2c_info("status - General call  0x%x", status);
    }

    /*  Address as a slave   bit 9 */
    if (status & I2C_STAT_AAS)
    {
        i2c_info("as a slave");
    }

    /* Transmit underrun.  bit 10  */
    if (status & I2C_STAT_XUDF)
    {
        i2c_warn("error - underrun  0x%x", status);
        out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_XUDF);
        i2c_ret = OMAPFLASH_I2C_ER_XUDF;
    }

    /*  Receiving overrun. bit 11 */
    if (status & I2C_STAT_ROVR)
    {
        i2c_warn("error - overrun  0x%x", status);
        out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_ROVR);
        i2c_ret = OMAPFLASH_I2C_ER_ROVR;
    }

    return i2c_ret;
}

/*-----------------------------------------------------------------------------
| Function    :S32 en_i2c()
+------------------------------------------------------------------------------
| Description : This routine enables the I2C bit and ensures completion
|
| Parameters  :None
|
| Returns     :Success / Error
+-----------------------------------------------------------------------------*/
S32 en_i2c(U16 reg, U8 chk)
{
    U8 count = 100;
    /*Enable I2C Module to see the bring the i2c module out of reset */
    out_regs((i2c_reg_base + I2C_CON_OFFSET), I2C_CON_EN | reg);
    while (chk
           && ((in_regs(i2c_reg_base + I2C_SYSS_OFFSET) & I2C_SYSS_RDONE) == 0))
    {
        dl_lazy_delay(10);      /* Cause a 1000 uSec delay for timeout */
        count--;
        if (!count)
        {
            break;
        }
    }
    return (count) ? OMAPFLASH_SUCCESS : OMAPFLASH_I2C_ABORT;
}

/*-----------------------------------------------------------------------------
| Function    :S32 clear_i2c()
+------------------------------------------------------------------------------
| Description : This routine clears the i2c status register
|
| Parameters  :None
|
| Returns     :Success / Error
+-----------------------------------------------------------------------------*/
S32 clear_i2c()
{
    U16 i2c_status;
    S32 i2c_ret;

    i2c_status = in_regs((i2c_reg_base + I2C_STAT_OFFSET)); /*read the status */
    i2c_ret = check_i2c_status(i2c_status);
    i2c_info("Clearing I2C status...");

    /*Disable I2C module */
    out_regs((i2c_reg_base + I2C_CON_OFFSET), I2C_CON_DISABLE); /*I2C_Disable */
    return i2c_ret;
}

/*-----------------------------------------------------------------------------
| Function    :S32 reset_i2c()
+------------------------------------------------------------------------------
| Description : This routine clears the i2c status register
|
| Parameters  :None
|
| Returns     :Success / Error
+-----------------------------------------------------------------------------*/
S32 reset_i2c()
{
    /*reset the module */
    out_regs((i2c_reg_base + I2C_SYSC_OFFSET), I2C_SYSC_SRST);

    /* I2c En should be disabled at this point */
    return en_i2c(I2C_CON_EN, 1);
}

/*-----------------------------------------------------------------------------
| Function    :S32 write_i2c_data(U16 device, U8 subaddr, U8 data)
+------------------------------------------------------------------------------
| Description : This function writes a byte of data to the addressed I2C device.

| Parameters  :device - Device identification code for the I2C slave device
|              subaddr - I2C slave device internal register address
|              data - Data to write on I2C bus
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
S32 write_i2c_data(U16 device,
    U8 subaddr,
    U8 data)
{
    U8 dataptr[2];

    dataptr[0] = subaddr;
    dataptr[1] = data;
    return write_i2c_with_dataptr(device, 2, dataptr);
}

/*+-----------------------------------------------------------------------------
| Description : This function writes multiple bytes of data to the addressed I2C
|               device. It utilizes the FIFO buffer in the I2C peripheral.
|               This is needed to access I2C slave devices with 2byte
|               subaddresses
|
| Parameters  :device  - Device identification code for the I2C slave device
|              length  - Length of data to write
|              dataptr - Data to write on I2C bus, subaddress is prepended to
|                        ordinary data
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
S32 write_i2c_with_dataptr(U16 device, U16 length, U8 * dataptr)
{
    U16 i2c_con_val = 0, i2c_stat_val = 0, txstat = 0, op_mode;
    S32 i2c_ret = DAL_I2C_SUCCESS, tmpcnt = 0;
    U32 timeout = 0, attempts = 0;
    U16 fifodepth, transmission_finished = 0;
    U8 index = 0;
    U8 subindex = 0;

    while ((attempts++ < (RETRY_COUNT)) && !transmission_finished)
    {

        out_regs((i2c_reg_base + I2C_CON_OFFSET), 0);

        /*I2C_BUFSTAT_OFFSET[15:14] contains fifodepth */
        /*I2C_BUFSTAT_FIFODEPTH:
         * 0x0=8byte, 0x1=16 bytes,
         * 0x2=32bytes, 0x3=64bytes -> Conversion to number of bytes in line
         * below
         */
        fifodepth =
            (in_regs(i2c_reg_base + I2C_BUFSTAT_OFFSET) & I2C_BUFSTAT_FIFODEPTH)
            >> 14;
        /*The same as 8*pow(2,fifodepth), but without the mathlib included */
        fifodepth = 8 << fifodepth;
        out_regs((i2c_reg_base + I2C_BUF_OFFSET),
                 (fifodepth - 1) | I2C_BUF_TXFIFO_CLR);

        /* Disable all interrupts */
        out_regs((i2c_reg_base + I2C_IE_OFFSET), 0x0);
        i2c_stat_val = in_regs(i2c_reg_base + I2C_STAT_OFFSET); /* Dummy read */
        out_regs((i2c_reg_base + I2C_SA_OFFSET), i2c_stat_val); /* pending sta*/
        /*configure slave address */
        out_regs((i2c_reg_base + I2C_SA_OFFSET), (I2C_SLAVE_MASK & device));

        /*configure data count register */
        out_regs((i2c_reg_base + I2C_CNT_OFFSET), length);
        if (i2c_hs)
        {
            op_mode = I2C_CON_HS_MODE;
        }
        else
        {
            op_mode = I2C_CON_SD_MODE;
        }

        /* poll bus busy bit in status register */
        while (in_regs((i2c_reg_base + I2C_STAT_OFFSET)) & I2C_STAT_BB)
        {
            if (timeout++ > TENTHOUSAND)
            {
                i2c_warn("timeout on BUS BUSY..reseting i2c");
                goto dead_mode;
            }
            if (I2C_IS_ABORT == TRUE)
                return OMAPFLASH_I2C_ABORT;
            dl_lazy_delay(ONE_MICROSEC);
        }

        /*set master mode, transmit mode,Enable start,stop conditions */
#if 1
        /* This is a suggested work around.. */
        i2c_con_val =
            (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_TRX | I2C_CON_STT);
        en_i2c(i2c_con_val, 0);
        while (in_regs((i2c_reg_base + I2C_CON_OFFSET)) & I2C_CON_STT)
        {
            if (timeout++ > TENTHOUSAND)
            {
                clear_i2c();
                i2c_warn("timeout on STT..reseting i2c");
                goto dead_mode;
            }
            if (I2C_IS_ABORT == TRUE)
                return OMAPFLASH_I2C_ABORT;
        }
        i2c_con_val =
            (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_TRX | I2C_CON_STP);
        en_i2c(i2c_con_val, 1);
#else
        i2c_con_val =
            (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_TRX | I2C_CON_STT |
             I2C_CON_STP);
        en_i2c(i2c_con_val, 1);
#endif
        /* Implementation is due to figure 19.20 in the TRM for
         * OMAP2430 Multimediaprocessor. Please find this figure for easy
         * understanding of implementation
         */
        tmpcnt = 0;
        i2c_stat_val = 0;
        while (tmpcnt++ < TENTHOUSAND)
        {

            i2c_stat_val = in_regs(i2c_reg_base + I2C_STAT_OFFSET);
            /* in case of our CPU running at a high speed and i2c is slow,
             * we'd see this */
            if (0 == i2c_stat_val)
            {
                /* retry again-this is less than 3uSec or so for the first bit*/
                dl_lazy_delay(ONE_MICROSEC);
                continue;
            }
            else
                /* Considered to be a transmission error in this design -
                 * error condition returned in "check_i2c_status(status)" below
                 */
            if ((i2c_stat_val & I2C_STAT_NACK))
            {
#ifndef DO_NOT_IGNORE_NACK_IRQS
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_NACK);
#else
                if (i2c_allow_nak)
                {
                    out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_NACK);
                }
                else
                {
                    break;
                }
#endif /* DO_NOT_IGNORE_NACK_IRQS */
            }
            else if (i2c_stat_val & I2C_STAT_AL)
            { /*Not relevant - not a Multimaster device */
#ifdef DO_NOT_IGNORE_AL_IRQS
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_AL);
#else
                /*Reprogram the registers; */
                /*Check for new start; */
                break;
#endif
            }
            else if (i2c_stat_val & I2C_STAT_ARDY)
            {
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_ARDY);
                transmission_finished = 1;
                break;
            }
            else if (i2c_stat_val & (I2C_STAT_XDR | I2C_STAT_XRDY))
            {
                /* Note: "index" is equal to "xtrsh" multiplied with a positive
                 * whole number Read I2C_BUFSTAT[5:0] TXSTAT to check the amount
                 * of data left to be transmitted
                 */
#if 1
                txstat = (i2c_stat_val & I2C_STAT_XRDY) ? fifodepth :
                    I2C_BUFSTAT_TXSTAT & in_regs(i2c_reg_base +
                                                 I2C_BUFSTAT_OFFSET);
#else
                /* HACK ALERT: Hw limitations cause us to move to 1 byte size */
                txstat = 1;
#endif
                /*Write I2C_DATA register for TXSTAT times ; */
                for (subindex = 0; (subindex < txstat) && (index < length);
                     index++)
                {
                    ++subindex;
                    out_regb((i2c_reg_base + I2C_DATA_OFFSET), dataptr[index]);

                }

                out_regs((i2c_reg_base + I2C_STAT_OFFSET),
                        (i2c_stat_val & (I2C_STAT_XDR | I2C_STAT_XRDY)));
                tmpcnt = 0;
                continue;
            }
            else if (i2c_stat_val & (I2C_STAT_BF | I2C_STAT_BB))
            {                   /* yeah.. sure.. */
                out_regs((i2c_reg_base + I2C_STAT_OFFSET),
                        (i2c_stat_val & (I2C_STAT_BF | I2C_STAT_BB)));
                continue;
            }
            else
            {
                /*(NACK + no AL +) no ARDY + no XDR + no XRDY */
                if (I2C_IS_ABORT == TRUE)
                    return OMAPFLASH_I2C_ABORT;
                return OMAPFLASH_ERROR;
            }
        }                       /* while(tmpcnt++ < TENTHOUSAND) */
        if (tmpcnt >= TENTHOUSAND)
        {
            clear_i2c();
            i2c_warn("Transmission error-No response seen");
            continue;           /*Start new attempt */
        }
      dead_mode:
        /*status of transmit operation */
        i2c_stat_val = in_regs((i2c_reg_base + I2C_STAT_OFFSET));
        i2c_ret = check_i2c_status(i2c_stat_val);
        if (i2c_ret != DAL_I2C_SUCCESS)
        {
            /*Reset I2C module */
            i2c_warn("Write Operation fail..Resetting the I2C..");
            clear_i2c();
        }
        timeout = 0;
    }                           /*end of while */

    if ((attempts > RETRY_COUNT) && (!transmission_finished))
    {
        i2c_ret = DAL_I2C_FAIL;
        /*Reset I2C module */
        i2c_warn("Too Many I2c attempts! failed..");
        i2c_warn("wr_dptr:device: %x addr %x", device, dataptr);
    }
    clear_i2c();
    i2c_info("return from i2c_write ");
    return i2c_ret;
}

/*-----------------------------------------------------------------------------
| Function    :void write_i2c_onlydata(U16 device, U8 * buf, U32 * len)
+------------------------------------------------------------------------------
| Description : This function writes with no data, used to set internal address
|               pointer of i2c device in order to read.
|
| Parameters  :device  - Device identification code for the I2C slave device
|              subaddr - I2C slave device internal register address
|              data    - Data to write on I2C bus
|
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
void write_i2c_onlydata(U16 device,
    U8 * buf,
    U32 * len)
{
    U32 i;
    U32 temp = *len;

    for (i = 0; i < temp; i++)
    {
        write_i2c_nodata(device, buf[i]);
    }

}

/*-----------------------------------------------------------------------------
| Function    :S32 write_i2c_nodata(U16 device, U8 data)
+------------------------------------------------------------------------------
| Description :
|
| Parameters  :device  - Device identification code for the I2C slave device
|              data    - Data to write on I2C bus
|
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
S32 write_i2c_nodata(U16 device,
    U8 data)
{
    U16 i2c_data = 0, i2c_status = 0, regval, op_mode, start_stop;
    S32 i2c_ret = DAL_I2C_SUCCESS, tmpcnt = 0;
    U32 timeout = 0, attempts = 0;
    U8 transmission_finished = 0;
    /*get the reg base */

    /*configure slave address */
    out_regs((i2c_reg_base + I2C_SA_OFFSET), (I2C_SLAVE_MASK & device));

    /*configure data count register */
    out_regs((i2c_reg_base + I2C_CNT_OFFSET), I2C_DATA_CNT_VAL1);

    i2c_data = data;            /*address offset */

    while ((attempts++ < RETRY_COUNT) && !transmission_finished)
    {
        /*poll bus busy bit in status register */
        while (in_regs((i2c_reg_base + I2C_STAT_OFFSET)) & I2C_STAT_BB)
        {
            timeout = 0;
            if (timeout++ > TENTHOUSAND)
            {
                clear_i2c();
                i2c_warn("Timeout on BUS BUSY..reseting i2c");
                return OMAPFLASH_DAL_ERROR;
            }
            if (I2C_IS_ABORT == TRUE)
                return OMAPFLASH_I2C_ABORT;
            dl_lazy_delay(ONE_MICROSEC);
        }

        if (i2c_hs)
        {
            op_mode = I2C_CON_HS_MODE;
            /* for HS mode no stop condition has to be given,
               if done so it switches to fs mode(repeat start has to be used) */

            start_stop = I2C_CON_STT;
        }
        else
        {
            op_mode = I2C_CON_SD_MODE;
            /* for std mode we can stop */
            start_stop = I2C_CON_STT | I2C_CON_STP;

        }

        /*set master mode, transmit mode,Enable start,stop conditions */

#if 1
        /* This is a suggested work around.. */
        if (start_stop & (I2C_CON_STP | I2C_CON_STT))
        {
            U32 timeout = 0x0;
            regval =
                (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_TRX |
                 I2C_CON_STT);
            en_i2c(regval, 0);
            timeout = 0;
            while (in_regs((i2c_reg_base + I2C_CON_OFFSET)) & I2C_CON_STT)
            {
                if (timeout++ > TENTHOUSAND)
                {
                    clear_i2c();
                    i2c_warn("timeout on STT..reseting i2c");
                    return DAL_I2C_FAIL;
                }
                if (I2C_IS_ABORT == TRUE)
                    return OMAPFLASH_I2C_ABORT;
            }

            regval =
                (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_TRX |
                 I2C_CON_STP);
            en_i2c(regval, 1);
        }
        else
        {
            regval =
                (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_TRX | start_stop);
            en_i2c(regval, 1);
        }
#else

        regval =
            (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_TRX | start_stop);
        en_i2c(regval, 1);
#endif
        /*wait for the reset to complete. */
        tmpcnt = 0;
        while (!(in_regs((i2c_reg_base + I2C_SYSS_OFFSET)) & I2C_SYSS_RDONE))
        {
            tmpcnt++;
            if (tmpcnt == TENTHOUSAND)
            {
                i2c_warn("Reset timeout");
                break;
            }
            else
            {
                dl_lazy_delay(ONE_MICROSEC);
            }
        }

        if (tmpcnt >= TENTHOUSAND)
        {
            clear_i2c();
            continue;           /*Start new attempt */
        }

        tmpcnt = 0;
        while (tmpcnt++ < TENTHOUSAND)
        {
            /*Poll XRDY bit to check for transmit ready */
            if (in_regs((i2c_reg_base + I2C_STAT_OFFSET)) & I2C_STAT_XRDY)
            {
                /*load the FIFO */
                out_regs((i2c_reg_base + I2C_DATA_OFFSET), i2c_data);
                break;
            }
            else
            {
                /*XRDY is not set */
                i2c_warn("XRDY not set");
                dl_lazy_delay(ONE_MICROSEC);
            }
            if (I2C_IS_ABORT == TRUE)
                return OMAPFLASH_I2C_ABORT;
        }

        if (tmpcnt >= TENTHOUSAND)
        {
            clear_i2c();
            continue;           /*Start new attempt */
        }

        tmpcnt = 0;
        while (tmpcnt++ < TENTHOUSAND)
        {
            /*Poll the ARDY bit to confirm that transmition is over */

            if (in_regs((i2c_reg_base + I2C_STAT_OFFSET)) & I2C_STAT_ARDY)
            {
                /*ardy is set */
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_ARDY);
                transmission_finished = 1;
                break;
            }
            else
            {
                if (I2C_IS_ABORT == TRUE)
                    return OMAPFLASH_I2C_ABORT;
                dl_lazy_delay(HUNDRED_MICROSEC);    /*Interval for polling */
            }
        }
        if (tmpcnt >= TENTHOUSAND)
        {
            /*ardy not set within 100 millisecs. */
            i2c_warn("ARDY not set");
            clear_i2c();
            continue;
        }
        /*status of transmit operation */
        i2c_status = in_regs((i2c_reg_base + I2C_STAT_OFFSET));
        i2c_ret = check_i2c_status(i2c_status);
        if (i2c_ret != DAL_I2C_SUCCESS)
        {
            /*Reset I2C module */
            clear_i2c();
        }
        else
        {
            /*clear XRDY bit */
            out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_XRDY);

            transmission_finished = 1;
            break;
        }
        /*clear XRDY bit    */
        out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_XRDY);
        timeout = 0;
    }                           /*end of while(attempts++ < (RETRY_COUNT)) */

    if ((attempts > RETRY_COUNT) && (!transmission_finished))
    {
        i2c_warn("i2c address write failed");
        i2c_warn("wr_nodat:device: %x addr %x", device, data);
        return DAL_I2C_FAIL;
    }

    return i2c_ret;
}

/*-----------------------------------------------------------------------------
| Function    :read_i2c_data
+------------------------------------------------------------------------------
| Description :This function reads a byte of data from the addressed I2C device.
|
| Parameters  :device  - Device identification code for the I2C slave device
|              subaddr - I2C slave device internal register address
|              data    - Data to write on I2C bus
|
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
S32 read_i2c_data(U16 device,
    U8 subaddr,
    U8 * data)
{
    return read_i2c_with_dataptr_2byte_subaddr(device, subaddr, 1, 1, data);
}

/*-----------------------------------------------------------------------------
| Function    :read_i2c_with_dataptr_2byte_subaddr
+------------------------------------------------------------------------------
| Description :This function reads a byte of data from the addressed I2C device.
|
| Parameters  :device  - Device identification code for the I2C slave device
|              subaddr - I2C slave device internal register address
|              address_mode -     1: 1-byte subaddress
|                                           2: 2-byte subaddress
|              size     - Size of data buffer
|              data    - Data to read from remote device via I2C bus
|
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
S32 read_i2c_with_dataptr_2byte_subaddr(U16 device,
    U16 subaddr,
    U8 address_mode,
    U16 size,
    U8 * data)
{
    S32 i2c_ret = DAL_I2C_SUCCESS;
    U8 *subaddr_ptr = NULL;

    /*write subaddress(offset) to remote device */
    if (I2C_1_BYTE_ADDRESS == address_mode)
    {
        subaddr_ptr = (U8 *) calloc(sizeof(U8), 0x1);
        if (NULL == subaddr_ptr)
        {
            i2c_warn("Error in malloc");
            return -1;
        }
        subaddr_ptr[0] = (U8) subaddr;
        i2c_ret = write_i2c_with_dataptr(device, address_mode, subaddr_ptr);
        free(subaddr_ptr);
    }
    else if (I2C_2_BYTE_ADDRESS == address_mode)
    {
        subaddr_ptr = (U8 *) calloc(2 * sizeof(U8), 0x1);
        if (NULL == subaddr_ptr)
        {
            i2c_warn("Error in malloc");
            return -1;
        }
        subaddr_ptr[0] = (U8) ((subaddr & 0xFF00) >> 8);
        subaddr_ptr[1] = (U8) (subaddr & 0x00FF);
        i2c_ret = write_i2c_with_dataptr(device, address_mode, subaddr_ptr);
        free(subaddr_ptr);
    }
    else
    {
        i2c_ret = DAL_I2C_FAIL;
    }

    if (DAL_I2C_SUCCESS == i2c_ret)
        i2c_ret = read_i2c_payload_using_fifo(device, size, data);

    return i2c_ret;
}

/*-----------------------------------------------------------------------------
| Function    :S32 read_i2c_onlydata(U16 device, U8 * buf, U32 * len)
+------------------------------------------------------------------------------
| CONVERTED - function is not necessary anymore - the more general function
| "read_i2c_payload_using_fifo" can be used directly.
+-----------------------------------------------------------------------------*/
S32 read_i2c_onlydata(U16 device,
    U8 * buf,
    U32 * len)
{
    return read_i2c_payload_using_fifo(device, (U16) * len, buf);
}

/*-----------------------------------------------------------------------------
| Function    : read_i2c_payload_using_fifo
+------------------------------------------------------------------------------
| Description : This routine reads the payload of the given size from the remote
|               device.
|
| Parameters  :device - Device identification code for the I2C slave device
|                     size  - Size of
|                     data - Pointer to Data-buffer. This is assumed allocated
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
S32 read_i2c_payload_using_fifo(U16 device,
    U16 size,
    U8 * data)
{
    U16 i2c_con_val = 0, i2c_stat_val = 0, rxstat = 0, op_mode;
    S32 i2c_ret = DAL_I2C_SUCCESS, tmpcnt = 0;
    U32 timeout = 0, attempts = 0;
    U16 fifodepth, transmission_finished = 0;
    U8 index = 0, subindex = 0;

    while ((attempts++ < (RETRY_COUNT)) && !transmission_finished)
    {

        timeout = 0;
        /*poll bus busy bit in status register */
        while (in_regs((i2c_reg_base + I2C_STAT_OFFSET)) & I2C_STAT_BB)
        {
            if (timeout++ > TENTHOUSAND)
            {
                clear_i2c();
                i2c_warn("timeout on BUS BUSY..reseting i2c");
                return OMAPFLASH_DAL_ERROR;
            }
            if (I2C_IS_ABORT == TRUE)
                return OMAPFLASH_I2C_ABORT;
            dl_lazy_delay(ONE_MICROSEC);
        }

        /*write the slave address */
        out_regs((i2c_reg_base + I2C_SA_OFFSET), (I2C_SLAVE_MASK & device));
        /*write the payload size */
        out_regs((i2c_reg_base + I2C_CNT_OFFSET), size);

        /* Disable all interrupts */
        out_regs((i2c_reg_base + I2C_IE_OFFSET), 0x0);

        /*I2C_BUFSTAT_OFFSET[15:14] contains fifodepth */
        fifodepth =
            (in_regs(i2c_reg_base + I2C_BUFSTAT_OFFSET) & I2C_BUFSTAT_FIFODEPTH)
            >> 14;
        /*I2C_BUFSTAT_FIFODEPTH:   0x0=8byte, 0x1=16 bytes, 0x2=32bytes,
         * 0x3=64bytes -> Conversion to number of bytes in line below
         *The same as 8*pow(2,fifodepth), but without the mathlib included
         * get notified when 1/2 the buffer is full */
        fifodepth = 8 << fifodepth;
        out_regs((i2c_reg_base + I2C_BUF_OFFSET),
                (((fifodepth / 2 - 1) << 8) & I2C_BUF_RTRSH) |
                    I2C_BUF_RXFIFO_CLR);

        if (i2c_hs)
        {
            op_mode = I2C_CON_HS_MODE;
        }
        else
        {
            op_mode = I2C_CON_SD_MODE;
        }
#if 1
        /* This is a suggested work around.. */
        i2c_con_val = (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_STT);
        en_i2c(i2c_con_val, 0);
        timeout = 0;
        while (in_regs((i2c_reg_base + I2C_CON_OFFSET)) & I2C_CON_STT)
        {
            if (timeout++ > TENTHOUSAND)
            {
                clear_i2c();
                i2c_warn("Timeout on BUS BUSY..reseting i2c");
                goto dead_i2c;
            }
            if (I2C_IS_ABORT == TRUE)
            {
                i2c_warn("Abort in i2c ops");
                return OMAPFLASH_I2C_ABORT;
            }

        }
        i2c_con_val = (I2C_CON_EN | op_mode | I2C_CON_MST | I2C_CON_STP);
        en_i2c(i2c_con_val, 1);
#else
        /*set master mode, recieve mode,Enable start,stop bits. */
        i2c_con_val = (I2C_CON_EN |
                op_mode |
                I2C_CON_MST |
                I2C_CON_STT |
                I2C_CON_STP); //8403 BIT15 Master, 10 Rec, 0 start, 1 stop
        en_i2c(i2c_con_val, 1);
#endif
        /* Implementation is due to figure 19.21 in the TRM for
         * OMAP2430 Multimediaprocessor. Please find this figure for
         * easy understanding of implementation */
        tmpcnt = 0;
        while (tmpcnt++ < TENTHOUSAND)
        {
            i2c_stat_val = in_regs(i2c_reg_base + I2C_STAT_OFFSET);
            /* in case of our CPU running at a high speed and i2c is slow,
             * we'd see this */
            if (0 == i2c_stat_val)
            {  /* this is less than 3uSec or so for the first bit */
                dl_lazy_delay(ONE_MICROSEC);
                /* retry again */
                continue;
            }
            else
            if ((i2c_stat_val & I2C_STAT_NACK))
            {
                /*Considered to be a transmission error in this design */
#ifndef DO_NOT_IGNORE_NACK_IRQS
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_NACK);
#else
                /*Reprogram registers; */
                /*Check for new start; */
                break;
#endif
            }
            else if (i2c_stat_val & I2C_STAT_AL)
            {  /*Not relevant - not a Multimaster device */
#ifndef DO_NOT_IGNORE_AL_IRQS
                /*Also considered an error condition */
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_AL);
#else
                /*Reprogram the registers; */
                /*Check for new start; */
                break;
#endif
            }
            else if (i2c_stat_val & I2C_STAT_ARDY)
            {
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), I2C_STAT_ARDY);
                transmission_finished = 1;
                i2c_info("Transfer Complete!");
                break;
            }
            else if (i2c_stat_val & (I2C_STAT_RDR | I2C_STAT_RRDY))
            {
                if (index < size)
                {
                    /* Note: "index" equals "rtrsh" multiplied with a positive
                     * whole number Read I2C_BUFSTAT[13:8] RXSTAT to check the
                     * amount of data left to be transmitted; */
#if 1
                    rxstat = i2c_stat_val & (I2C_STAT_RDR) ?
                        (I2C_BUFSTAT_RXSTAT &
                         in_regs(i2c_reg_base +
                                 I2C_BUFSTAT_OFFSET)) >> 8 : fifodepth / 2;
#else
                    /* HACK ALERT: Due to hw limitation..
                     * Read only a single byte */
                    rxstat = 1;
#endif
                    /*Read from I2C_DATA register for RXSTAT times ; */
                    for (subindex = 0; (subindex < rxstat) && (index < size);
                         index++)
                    {
                        ++subindex;
                        data[index] = in_regb(i2c_reg_base + I2C_DATA_OFFSET);
                    }
                    out_regs((i2c_reg_base + I2C_STAT_OFFSET), (i2c_stat_val &
                                (I2C_STAT_RDR | I2C_STAT_RRDY)));
                    tmpcnt = 0;
                }
                continue;
            }
            else if (i2c_stat_val & (I2C_STAT_BF | I2C_STAT_BB))
            {                   /* yeah.. sure.. */
                out_regs((i2c_reg_base + I2C_STAT_OFFSET), (i2c_stat_val &
                            (I2C_STAT_BF | I2C_STAT_BB)));
                continue;
            }
            else
            {
                i2c_warn("Unknown Status.. quitting");
                /*(NACK + no AL +) no ARDY + no RDR + no RRDY */
                if (I2C_IS_ABORT == TRUE)
                    return OMAPFLASH_I2C_ABORT;
                return OMAPFLASH_ERROR;
            }
        }
        if (tmpcnt >= TENTHOUSAND)
        {
            clear_i2c();
            i2c_warn("Transmission error-No response seen");
            continue;           /*Start new attempt */
        }
      dead_i2c:
        /*status of receive operation */
        i2c_stat_val = in_regs((i2c_reg_base + I2C_STAT_OFFSET));
        i2c_ret = check_i2c_status(i2c_stat_val);
        if (i2c_ret != DAL_I2C_SUCCESS)
        {
            /*Reset I2C module */
            i2c_warn("Read Operation fail..Resetting the I2C..");
            clear_i2c();
        }
        timeout = 0;
    } /* End of while((attempts++ < (RETRY_COUNT)) && !transmission_finished) */

    if ((attempts > RETRY_COUNT) && (!transmission_finished))
    {
        i2c_ret = DAL_I2C_FAIL;
        /*Reset I2C module */
        i2c_warn("Too Many I2c attempts! failed..");
        i2c_warn("READ: device: %x addr %x", device, data);
    }
    clear_i2c();
    i2c_info("return from i2c_read ");
    return i2c_ret;
}

/*-----------------------------------------------------------------------------
| Function    :S32 deinit_i2c(void)
+------------------------------------------------------------------------------
| Description :
| Parameters  : none
|
| Returns     :0=success, ER Code on failure
+-----------------------------------------------------------------------------*/
S32 deinit_i2c(void)
{
    clear_i2c();
    i2c_hs = 0;
    return 0;

}

#endif /* i2c.c */
