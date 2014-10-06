/**
 * @file triton2_drv.c
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
 * The purpose of this file is to provide the Triton2 functions
 * 
 */

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES ==============================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "csst_tgt.h"

#include "triton2_drv.h"
#include "triton2.h"
#include "i2c_dis.h"
#ifndef EMMC_DRV
#include "triton2bci.h"
#include "t2_audio.h"
#endif //EMMC_DRV

#include "dl_pfwork_ex.h"
#include "dbg_ex.h"
#ifdef EMMC_DRV
#include "i2c_drv.h"
#endif //EMMC_DRV

U32 debug_temp = 0;             /* For debug purpose only */
U32 triton2_i2c_handle = 0;     /* U8 triton2_i2c_handle before */

U8 active_int = 0;              /* active interrupt */

/*-----------------------------------------------------------------------------
| Function    : triton2_init()
+------------------------------------------------------------------------------
| Description : Initialize the triton2
|
| Parameters  : <description of in/out parameters>
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
S32 triton2_init(const void *init_str,
    U32 * dis_addr)
{
    S32 ret_status;
    T_I2C_INIT_STRUCTURE i2c_init_str;

    /* malloc the triton2 DIS structure */

    T_TRITON2_DIS *triton2_dis;
    triton2_dis = (T_TRITON2_DIS *) malloc(sizeof(T_TRITON2_DIS));

    if (triton2_dis == 0)
    {
        // error
        ret_status = OMAPFLASH_MALLOC_FAILED;
        return ret_status;
    }

    /* Input param dis_addr is to be filled with is malloced mem and returned to DAL */
    *dis_addr = (U32) triton2_dis;
    memcpy(triton2_dis, init_str, sizeof(T_TRITON2_INIT_STRUCTURE));

#ifndef EMMC_DRV
    /*
       Initialize the I2C control interface to Triton2
     */
    i2c_init_str.pid = DAL_I2C_PID;
    i2c_init_str.sid = get_omap_i2c_interface();

    /* Triton2's I2C interface supported HS,FS,and LS */
    i2c_init_str.clock_speed = get_omap_i2c_speed();

    ret_status = dal_init((void *)&i2c_init_str, &(triton2_dis->i2c_handle));

    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C init failed");
        return ret_status;      // Need to check whether the return value is as expected
    }
    triton2_i2c_handle = triton2_i2c_handle;
#else //EMMC_DRV
    i2c_init_str.pid = 2; //DAL_I2C_PID;
    i2c_init_str.sid = 0;

    /* Triton2's I2C interface supported HS,FS,and LS */
    i2c_init_str.clock_speed = get_omap_i2c_speed();

    ret_status = dal_init((void *)&i2c_init_str, &triton2_i2c_handle);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C init failed");
        return ret_status;      // Need to check whether the return value is as expected
    }
#endif //EMMC_DRV

#ifndef EMMC_DRV
    if (triton2_dis->triton2_initstr.t2pwron_irq_callback != NULL)
    {
        confgure_t2_pwrbutton_int((U32) triton2_dis);
    }
#endif //EMMC_DRV

    /*
       Update the Triton2 group ids
     */
    update_triton2_groupids(triton2_dis);
#ifndef EMMC_DRV
    if (triton2_dis->triton2_initstr.sampling_rate)
    {
        configure_i2c_id_for_audio_reg(triton2_dis);
        reset_triton2_codec(triton2_dis);
        triton2_codec_init(triton2_dis);
    }
#endif //EMMC_DRV

    return ret_status;

}                               /* end triton2_init */

/*-----------------------------------------------------------------------------
| Function    : triton2_read()
+------------------------------------------------------------------------------
| Description : triton2 read is used to read a member of triton2 DIS.
|
| Parameters  : <description of in/out parameters>
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
S32 triton2_read(U32 dis_addr,
    U32 tag,
    U32 * size,
    U8 * buffer)
{
    S32 ret_status = OMAPFLASH_SUCCESS;
    T_TRITON2_DIS *triton2_dis = (T_TRITON2_DIS *) dis_addr;
    U8 module_num;
    T_TRITON2_GPIO_STRUCTURE *trtn2_gpio;
    U8 offset;                  /* position of the pin in the respective
                                   gpio module */
    U8 pin_mask, snum;
    U8 gpio_temp_data = 0;

    /* Check if anything to be read from DIS */
    switch (tag)
    {
        case TRITON2_VDAC_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VDAC_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vdac = *(U8 *) buffer;
            }
            break;

        case TRITON2_VPLL1_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VPLL1_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vpll1 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VPLL2_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VPLL2_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vpll2 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VMMC1_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VMMC1_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vmmc1 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VMMC2_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VMMC2_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vmmc2 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VSIM_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VSIM_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vsim = *(U8 *) buffer;
            }
            break;

        case TRITON2_VAUX1_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VAUX1_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vaux1 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VAUX2_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VAUX2_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vaux2 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VAUX3_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VAUX3_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vaux3 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VAUX4_VOLTAGE_TAG:

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 VDD1 register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_VAUX4_DEDICATED_REG), size, buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the dal_read is success */
                triton2_dis->triton2_power.vaux4 = *(U8 *) buffer;
            }
            break;

        case TRITON2_PIH_ISR1_TAG:

            configure_i2c_id_for_triton2_addr_group2(triton2_dis);
            *size = 1;

            /* Read the Triton2 PIH_ISR1 register and call the respective ISR */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_PIH_ISR1_REG), size, buffer);

            break;

        case TRITON2_PIH_ISR2_TAG:

            configure_i2c_id_for_triton2_addr_group2(triton2_dis);
            *size = 1;

            /* Read the Triton2 PIH_ISR1 register and call the respective ISR */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_PIH_ISR2_REG), size, buffer);

            break;

        case TRITON2_GPIO_DATA_TAG:

            store_i2c_id_for_triton2(triton2_i2c_handle);
            configure_i2c_id_for_triton2_addr_group2(triton2_dis);
            trtn2_gpio =
                (T_TRITON2_GPIO_STRUCTURE *)
                malloc(sizeof(T_TRITON2_GPIO_STRUCTURE));

            memcpy(trtn2_gpio, (T_TRITON2_GPIO_STRUCTURE *) buffer,
                   sizeof(T_TRITON2_GPIO_STRUCTURE));

            *size = 1;
            module_num = trtn2_gpio->gpio_pin_num / NUM_OF_BITS_IN_TRITON2_REG;
            snum = module_num * NUM_OF_BITS_IN_TRITON2_REG;
            offset = trtn2_gpio->gpio_pin_num - snum;

            pin_mask = 1 << offset;

            /* Read the Triton2 GPIODATAIN register */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_GPIODATAIN1_REG + module_num), size,
                         &gpio_temp_data);
            if (ret_status != OMAPFLASH_SUCCESS)
            {

                return ret_status;
            }

            /* get the data relevant to the pin mask */
            trtn2_gpio->gpio_data = gpio_temp_data & pin_mask;
            trtn2_gpio->gpio_data = trtn2_gpio->gpio_data >> offset;

            memcpy((T_TRITON2_GPIO_STRUCTURE *) buffer, trtn2_gpio,
                   sizeof(T_TRITON2_GPIO_STRUCTURE));

            restore_i2c_id_for_triton2(triton2_i2c_handle);

            break;
        case TRITON2_HW_STS_TAG:   /* tag to read the PWON button status */

            configure_i2c_id_for_power_reg(triton2_dis);
            *size = 1;
            /* Read the Triton2 STS_HW_CONDITIONS register and give the value to the upper layer */
            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_STS_HW_CONDITIONS_REG), size,
                         buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            break;

#ifndef EMMC_DRV
        case TRITON2_BCI_MONITOR_TAG:  /* Tag to collect the BCI charging status */

            ret_status = configure_i2c_id_for_bci_reg(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            /* Get the Precharge FSM states and monitor status */
            ret_status = update_precharge_status(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            /* Get the Maincharge FSM states and monitor status */
            ret_status = update_maincharge_status(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            /* Get the BCI monitor measurements */
            ret_status = update_bat_monitor_info(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            /* Get the BCI charging modes */
            ret_status = update_bci_chaging_mode(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            /* Get the BCI charging devices */
            ret_status = update_mbat_charging_device(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            /* Update the return buffer with the BCI charger status */
            (*(T_TRITON2_DIS *) buffer) = *triton2_dis;
            break;
#endif //EMMC_DRV

        default:
            /* nothing from DIS */
            ret_status = OMAPFLASH_INVALID_TAG;
            break;

    }

    return ret_status;

}                               /* end triton2_read */

/*-----------------------------------------------------------------------------
| Function    : triton2_write()
+------------------------------------------------------------------------------
| Description : triton2 write used to write data in the triton2
|
| Parameters  : <description of in/out parameters>
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
S32 triton2_write(U32 dis_addr,
    U32 tag,
    U32 * size,
    U8 * buffer)
{

    S32 ret_status = OMAPFLASH_SUCCESS;
    U8 data;
    tOutputs outdev;
    tMics indevice;
    T_TRITON2_GPIO_STRUCTURE *trtn2_gpio;
    T_TRITON2_DIS *triton2_dis = (T_TRITON2_DIS *) dis_addr;
    U8 offset;                  /* position of the pin in the respective
                                   gpio module */
    U8 pin_mask, snum, io_mask;
    U8 gpio_temp_data = 0;
    U8 module_num;
    U8 in_volume = 0;

    /* Check if anything to be written to DIS */
    switch (tag)
    {
        case TRITON2_VDAC_VOLTAGE_TAG:

            /* Select the VDAC output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VDAC_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vdac = *(U8 *) buffer;
            }
            break;

        case TRITON2_VDAC_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VDAC_DEV_GRP_REG, *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vdac_stat = *buffer;
            }
            break;

        case TRITON2_VPLL1_VOLTAGE_TAG:

            /* Select the VPLL1 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VPLL1_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vpll1 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VPLL1_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VPLL1_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vpll1_stat = *buffer;
            }
            break;

        case TRITON2_VPLL2_VOLTAGE_TAG:

            /* Select the VPLL2 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VPLL2_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vpll2 = *(U8 *) buffer;
            }

            break;

        case TRITON2_VPLL2_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VPLL2_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vpll2_stat = *buffer;
            }

            break;

        case TRITON2_VMMC1_VOLTAGE_TAG:

            /* Select the VMMC1 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VMMC1_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vmmc1 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VMMC1_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VMMC1_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vmmc1_stat = *buffer;
            }

            break;

        case TRITON2_VMMC2_VOLTAGE_TAG:

            /* Select the VMMC2 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VMMC2_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vmmc2 = *(U8 *) buffer;
            }

            break;

        case TRITON2_VMMC2_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VMMC2_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vmmc2_stat = *buffer;
            }

            break;

        case TRITON2_VSIM_VOLTAGE_TAG:

            /* Select the VSIM output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VSIM_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vsim = *(U8 *) buffer;
            }
            break;

        case TRITON2_VSIM_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VSIM_DEV_GRP_REG, *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vsim_stat = *buffer;
            }
            break;

        case TRITON2_VAUX1_VOLTAGE_TAG:

            /* Select the VAUX1 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VAUX1_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux1 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VAUX1_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VAUX1_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux1_stat = *buffer;
            }

            break;

        case TRITON2_VAUX2_VOLTAGE_TAG:

            /* Select the VAUX2 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VAUX2_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux2 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VAUX2_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VAUX2_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux2_stat = *buffer;
            }

            break;

        case TRITON2_VAUX3_VOLTAGE_TAG:

            /* Select the VAUX3 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VAUX3_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux3 = *(U8 *) buffer;
            }
            break;

        case TRITON2_VAUX3_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VAUX3_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux3_stat = *buffer;
            }
            break;

        case TRITON2_VAUX4_VOLTAGE_TAG:

            /* Select the VAUX4 output voltage */
            ret_status =
                select_power_res_volt(triton2_dis, TRITON2_VAUX4_DEDICATED_REG,
                                      *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux4 = *(U8 *) buffer;
            }

            break;

        case TRITON2_VAUX4_ON_OFF_TAG:

            /* Enable/ Disable the power resource  */
            ret_status =
                control_triton2_pwr_resource(triton2_dis,
                                             TRITON2_VAUX4_DEV_GRP_REG,
                                             *buffer);
            if (ret_status == OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                triton2_dis->triton2_power.vaux4_stat = *buffer;
            }

            break;

#ifndef EMMC_DRV
        case TRITON2_VOLUME_TAG:

            /* Does the gain control for adjusting volume */

            configure_i2c_id_for_audio_reg(triton2_dis);
            ret_status = triton2_SetVolume(triton2_dis, *(U32 *) buffer);
            triton2_dis->triton2_audio.volume = *(U32 *) buffer;

            break;

        case TRITON2_LOUDSPEAKER_TAG:

            /* Does the gain control for adjusting volume */

            configure_i2c_id_for_audio_reg(triton2_dis);
            triton2_dis->triton2_audio.loudspeaker = *buffer;
            if (triton2_dis->triton2_audio.loudspeaker == 0)
            {
                outdev = tLeftLS;
            }
            else if (triton2_dis->triton2_audio.loudspeaker == 1)
            {
                outdev = tRightLS;
            }
            else if (triton2_dis->triton2_audio.loudspeaker == 2)
            {
                outdev = tHeadSet;
            }
            else if (triton2_dis->triton2_audio.loudspeaker == 3)
            {
                outdev = tSterioJack;
            }
            else if (triton2_dis->triton2_audio.loudspeaker == 4)
            {
                outdev = tEarphone;
            }
            else
            {
                outdev = tHeadSet;
            }

            /* Do the settings for particular output device */
            ret_status = outdev_settings(triton2_dis, outdev);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec o/p device setting failed");
                return ret_status;
            }

            break;

        case TRITON2_MIC_TAG:

            /* Update the DIS member */
            /* By defult the settings are for headset mic */
            configure_i2c_id_for_audio_reg(triton2_dis);
            triton2_dis->triton2_audio.mic = *buffer++;
            in_volume = *buffer;
            if (triton2_dis->triton2_audio.mic == 0)
            {
                indevice = tMainMic;
            }
            else if (triton2_dis->triton2_audio.mic == 1)
            {
                indevice = tSubMicMainMic;
            }
            else if (triton2_dis->triton2_audio.mic == 2)
            {
                indevice = tHeadsetMic;
            }
            else if (triton2_dis->triton2_audio.mic == 3)
            {
                indevice = tAUX_FM;
            }
            else if (triton2_dis->triton2_audio.mic == 4)
            {
                indevice = tDIG_MIC_0;
            }
            else if (triton2_dis->triton2_audio.mic == 5)
            {
                indevice = tDIG_MIC_1;
            }
            else
            {
                indevice = tHeadsetMic;
            }

            /* Configure Triton audio code for user selected mic */
            /* by default the headset mic is enabled. */
            ret_status = input_device_setting(triton2_dis, indevice, in_volume);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec i/p device setting failed");
                return ret_status;
            }
            break;
#endif //EMMC_DRV

        case TRITON2_DAC_TAG:

            /* Does the gain control for adjusting volume */
            configure_i2c_id_for_audio_reg(triton2_dis);
            data = 0x12;
            ret_status =
                dal_write(triton2_i2c_handle,
                          I2C_OFFSET(TRITON2_ARXL1_APGA_CTL_OFFSET), size,
                          &data);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec init failed");
            }
            data = 0x12;
            ret_status =
                dal_write(triton2_i2c_handle,
                          I2C_OFFSET(TRITON2_ARXL2_APGA_CTL_OFFSET), size,
                          &data);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec init failed");
            }
            data = 0x12;
            ret_status =
                dal_write(triton2_i2c_handle,
                          I2C_OFFSET(TRITON2_ARXR1_APGA_CTL_OFFSET), size,
                          &data);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec init failed");
            }
            data = 0x12;
            ret_status =
                dal_write(triton2_i2c_handle,
                          I2C_OFFSET(TRITON2_ARXR2_APGA_CTL_OFFSET), size,
                          &data);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* Update the DIS member only if the value is written to the device */
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec init failed");
            }

            break;

        case TRITON2_GPIO_DATA_DIR_TAG:

            store_i2c_id_for_triton2(triton2_i2c_handle);
            configure_i2c_id_for_triton2_addr_group2(triton2_dis);
            trtn2_gpio =
                (T_TRITON2_GPIO_STRUCTURE *)
                malloc(sizeof(T_TRITON2_GPIO_STRUCTURE));

            memcpy(trtn2_gpio, (T_TRITON2_GPIO_STRUCTURE *) buffer,
                   sizeof(T_TRITON2_GPIO_STRUCTURE));

            *size = 1;
            module_num = trtn2_gpio->gpio_pin_num / NUM_OF_BITS_IN_TRITON2_REG;
            snum = module_num * NUM_OF_BITS_IN_TRITON2_REG;
            offset = trtn2_gpio->gpio_pin_num - snum;

            pin_mask = 1 << offset;
            io_mask = trtn2_gpio->gpio_data_dir << offset;

            ret_status =
                dal_read(triton2_i2c_handle,
                         I2C_OFFSET(TRITON2_GPIODATADIR1_REG + module_num),
                         size, &gpio_temp_data);

            gpio_temp_data = gpio_temp_data & ~pin_mask;    /*manipulate only the pins that needs
                                                               configuration */
            gpio_temp_data = gpio_temp_data | io_mask;  /*set the pins as input  or output as
                                                           specified */

            /* Write to the Triton2 GPIODATAIN register */
            ret_status =
                dal_write(triton2_i2c_handle,
                          I2C_OFFSET(TRITON2_GPIODATADIR1_REG + module_num),
                          size, &gpio_temp_data);

            /* Enable the internal pullup for MMC Card Detect GPIOs (GPIO1 and GPIO2) */
            if (trtn2_gpio->gpio_pin_num == 0)
            {
                *size = 1;
                data = 0x16;    /* For enabling the internal pullup for GPIO1 used as MMC Slot1 Card Detect */
                /* write to the Triton2 TRITON2_GPIOPUPDCTR1_REG register */
                ret_status =
                    dal_write(triton2_i2c_handle,
                              I2C_OFFSET(TRITON2_GPIOPUPDCTR1_REG), size,
                              &data);

            }
            else if (trtn2_gpio->gpio_pin_num == 1)
            {
                *size = 1;
                data = 0x19;    /* For enabling the internal pullup for GPIO2 used as MMC Slot2 Card Detect */
                /* write to the Triton2 TRITON2_GPIOPUPDCTR1_REG register */
                ret_status =
                    dal_write(triton2_i2c_handle,
                              I2C_OFFSET(TRITON2_GPIOPUPDCTR1_REG), size,
                              &data);
            }
            else if (trtn2_gpio->gpio_pin_num == 2)
            {
                *size = 1;
                data = 0x05;    /* For disabling internal pull down for headset detect */
                /* write to the Triton2 TRITON2_GPIOPUPDCTR1_REG register */
                ret_status =
                    dal_write(triton2_i2c_handle,
                              I2C_OFFSET(TRITON2_GPIOPUPDCTR1_REG), size,
                              &data);
            }

            restore_i2c_id_for_triton2(triton2_i2c_handle);

            break;
        case TRITON2_GPIO_DATA_TAG:

            /*Disable the pull down on the GPIO2 */
            store_i2c_id_for_triton2(triton2_i2c_handle);
            configure_i2c_id_for_triton2_addr_group2(triton2_dis);
            trtn2_gpio =
                (T_TRITON2_GPIO_STRUCTURE *)
                malloc(sizeof(T_TRITON2_GPIO_STRUCTURE));

            memcpy(trtn2_gpio, (T_TRITON2_GPIO_STRUCTURE *) buffer,
                   sizeof(T_TRITON2_GPIO_STRUCTURE));

            *size = 1;
            module_num = trtn2_gpio->gpio_pin_num / NUM_OF_BITS_IN_TRITON2_REG;
            snum = module_num * NUM_OF_BITS_IN_TRITON2_REG;
            offset = trtn2_gpio->gpio_pin_num - snum;

            /* GPIO module to which the GPIO pin number belongs to */
            pin_mask = 1 << offset;

            gpio_temp_data = pin_mask;

            /* write to the Triton2 GPIODATAOUT register */
            if (trtn2_gpio->gpio_data)
            {
                ret_status =
                    dal_write(triton2_i2c_handle,
                              I2C_OFFSET(TRITON2_SETGPIODATAOUT1_REG +
                                         module_num), size, &gpio_temp_data);
            }
            else
            {
                ret_status =
                    dal_write(triton2_i2c_handle,
                              I2C_OFFSET(TRITON2_CLEARGPIODATAOUT1_REG +
                                         module_num), size, &gpio_temp_data);
            }
            restore_i2c_id_for_triton2(triton2_i2c_handle);

            break;

#ifndef EMMC_DRV
        case TRITON2_BCI_CHARGE_DISABLE:

            /* configure the i2c slave address for the BCI */
            ret_status = configure_i2c_id_for_bci_reg(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }

            /* Stop the current mode of charging */
            ret_status =
                bci_charge_mode_off((triton2_dis->triton2_initstr.triton2_bci.
                                     charger_sel), triton2_dis);

            break;

        case TRITON2_BCI_CHARGE_ENABLE:

            /* configure the i2c slave address for the BCI */
            ret_status = configure_i2c_id_for_bci_reg(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }

            /* enable the BCI charging */
            ret_status =
                bci_charge_mode_set(triton2_dis->triton2_initstr.triton2_bci.
                                    charger_sel, triton2_dis);
            break;

        case TRITON2_BCI_CONFIG:
            /* configure the bci charging */
            ret_status = bci_config(dis_addr);
            break;

        case TRITON2_BCI_CHGMODE:
            /* configure the i2c slave address for the BCI */
            ret_status = configure_i2c_id_for_bci_reg(triton2_dis);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                return ret_status;
            }
            /* set the BCI charging mode */
            ret_status =
                bci_charge_mode_set(triton2_dis->triton2_initstr.triton2_bci.
                                    charger_sel, triton2_dis);
            break;
#endif //EMMC_DRV

#ifndef EMMC_DRV
        case TRITON2_PWRON_INT_MASK_TAG:   /* This tag is mask or unmask the power on
                                               interrupt */
            /* configure the i2c slave address for the Power regsiters */
            configure_i2c_id_for_power_reg(triton2_dis);
            /* write to PWR_IMR1 register */
            ret_status =
                dal_write(triton2_i2c_handle,
                          I2C_OFFSET(TRITON2_PWR_IMR1_REG), size, buffer);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* debug print */
                dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_IMR1 reg failed");
            }
            break;

        case TRITON2_LED_NUMBER_TAG:

            triton2_dis->triton2_led.led_no = *(U8 *) buffer;

            break;

        case TRITON2_LED_BRIGHTNESS_TAG:

            triton2_dis->triton2_led.led_brightness = *(U8 *) buffer;

            break;

        case TRITON2_LED_ENABLE_DISABLE_TAG:

            triton2_dis->triton2_led.led_enable_disable_flag = *(U8 *) buffer;
            configure_triton2_leds((U32) triton2_dis);

            break;

        case T2_HOTDIEDECT_CONFIG_TAG:

            ret_status =
                configure_hotdiedetector((U32) triton2_dis, *buffer,
                                         *(buffer + 1));
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* debug print */
                dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_IMR1 reg failed");
                return ret_status;
            }

            break;

        case TRITON2_HOTDIE_INT_MASK_TAG:  /* This tag is mask or unmask the hotdie
                                               interrupt */
            /* configure the i2c slave address for the Power regsiters */
            configure_i2c_id_for_power_reg(triton2_dis);

            /* masking the interrupt PWR_IMR1 register */
            ret_status =
                dal_write(triton2_i2c_handle, I2C_OFFSET(PWR_IMR1), size,
                          buffer);
            if (ret_status != OMAPFLASH_SUCCESS)
            {
                /* debug print */
                dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_IMR1 reg failed");
            }
            break;
#endif //EMMC_DRV

        default:
            /* nothing from DIS */
            ret_status = OMAPFLASH_INVALID_TAG;
            break;
    }

    return ret_status;

}                               /* end triton2_write */

/*-----------------------------------------------------------------------------
| Function    : triton2_deinit()
+------------------------------------------------------------------------------
| Description : De-initializes triton2
|
| Parameters  : <description of in/out parameters>
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
S32 triton2_deinit(U32 dis_addr)
{
    S32 ret_status = 0;

    T_TRITON2_DIS *triton2_dis = (T_TRITON2_DIS *) dis_addr;

    if (triton2_dis != 0)
    {
#ifndef EMMC_DRV
        if (triton2_dis->triton2_initstr.sampling_rate)
        {
            ret_status = triton2_codec_deinit(triton2_dis);
        }
#endif //EMMC_DRV

#ifdef INTERRUPT
        if (triton2_dis->triton2_initstr.t2pwron_irq_callback != NULL)
        {
            triton2_remove_int_handler(active_int);

        }
#endif //INTERRUPT

        /* free the allocated I2C DIS handle */
        ret_status = dal_deinit(triton2_i2c_handle);

        /* free the allocated Triton2 DIS */
        free((void *)dis_addr);
    }

    return ret_status;
}                               /* end triton2_deinit */

/*-----------------------------------------------------------------------------
| Function    : configure_hotdiedetector
+------------------------------------------------------------------------------
| Description :  The Triton2 hotdie detector is configured
|
| Parameters  : U32 t2hotdie_dis
|
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/

S32 configure_hotdiedetector(U32 t2hotdie_dis,
    U8 hot_die_f_r,
    U8 hot_die_thresh)
{

    S32 ret_val = OMAPFLASH_SUCCESS; /* default */
    U32 length = 1;
    T_TRITON2_DIS *triton2_dis = (T_TRITON2_DIS *) t2hotdie_dis;
    U8 buffer = 0;

    /* configure the i2c slave address for the Power regsiters */
    configure_i2c_slave_id_for_triton2(triton2_i2c_handle,
                                       T2_HOTDIE_ADDR_GROUP);

    /*STEP 1 */

    /*Configure the hot-die detector for operation
       in its lowest temperature range (111 to 120 degrees (C)): */
    /*The TEMP_SEL[7:6] bits are set to in the MISC_CFG reg */
    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(MISC_CFG), &length,
                 &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error reading hotdie register value\r\n");
        return ret_val;
    }
    buffer &= 0x3F;
    buffer |= (hot_die_thresh & 0x3) << 6;  /* Set the threshold bits */

    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(MISC_CFG), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to MISC_CFG reg failed");
        return ret_val;
    }

#if 0
    /*STEP 2 */

    /*Configure the hot-die detector so that it generates
       an interrupt on interrupt status line 1 */
    /*Set the HOT_DIE bit to 1 in the PWR_ISR1[4] */
    buffer = 0;
    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(PWR_ISR1), &length,
                 &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error reading hotdie register value\r\n");
        return ret_val;
    }
    buffer |= 0x10;

    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(PWR_ISR1), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_ISR1 reg failed");
        return ret_val;
    }
#endif
    /*STEP 3 */

    /*Ensure that the interrupt is not masked:
       umask  the HOT_DIE bit in the PWR_IMR1[4] register */
    buffer = 0;
    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(PWR_IMR1), &length,
                 &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error reading hotdie register value\r\n");
        return ret_val;
    }

    buffer = 0xEF;              /*write 0 to unmask the bit */
    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(PWR_IMR1), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_IMR1 reg failed");
        return ret_val;
    }

    /*STEP 4 */

    /*Ensure that the hot-die detector rising threshold and the hot-die
       detector falling threshold are active (enabled):
       Set the HOT_DIE_RISING and HOT_DIE_FALLING bits in the PWR_EDR2[1:0] register */
    buffer = 0;
    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(PWR_EDR2), &length,
                 &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error reading hotdie register value\r\n");
        return ret_val;
    }
    buffer |= hot_die_f_r;

    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(PWR_EDR2), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_EDR2 reg failed");
        return ret_val;
    }
    return ret_val;

}

/*-----------------------------------------------------------------------------
| Function    : update_triton2_groupids()
+------------------------------------------------------------------------------
| Description : To update the I2C group Ids of Triton2.
|
| Parameters  : Triton2 DIS pointer
|
| Returns     : none
+-----------------------------------------------------------------------------*/
void update_triton2_groupids(T_TRITON2_DIS * triton2_dis)
{
    /* Set the I2C group ID for accessing the Triton2 resources */
    triton2_dis->triton2_power.i2caddr = 0x4B;  /* For power resources */
    triton2_dis->triton2_audio.i2caddr = 0x49;  /* For audio resources */
}

/*-----------------------------------------------------------------------------
| Function    : configure_i2c_id_for_power_reg()
+------------------------------------------------------------------------------
| Description : Configure I2C driver to use the I2C group ID to access the Power
|               resource registers.
|
| Parameters  : Triton2 DIS pointer
|
| Returns     : DAL Status
+-----------------------------------------------------------------------------*/
S32 configure_i2c_id_for_power_reg(T_TRITON2_DIS * triton2_dis)
{
    S32 ret_status;
    U32 size;
    U8 data;

    data = triton2_dis->triton2_power.i2caddr;
    size = 1;

    /* Select the I2C group IDs before accessing the respective Triton2 modules */
    ret_status =
        dal_write(triton2_i2c_handle, I2C_SLAVE_ADDR, &size, &data);

    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Slave Address change failed");
    }

    return (ret_status);
}

/*-----------------------------------------------------------------------------
| Function    : configure_i2c_id_for_audio_reg()
+------------------------------------------------------------------------------
| Description : Configure I2C driver to use the I2C group ID to access the audio
|               resource registers.
|
| Parameters  : Triton2 DIS pointer
|
| Returns     : DAL Status
+-----------------------------------------------------------------------------*/
S32 configure_i2c_id_for_audio_reg(T_TRITON2_DIS * triton2_dis)
{
    S32 ret_status;
    U32 size;
    U8 data;

    data = triton2_dis->triton2_audio.i2caddr;
    size = 1;

    ret_status =
        dal_write(triton2_i2c_handle, I2C_SLAVE_ADDR, &size, &data);

    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Slave Address change failed");
    }

    return (ret_status);
}

/*-----------------------------------------------------------------------------
| Function    : form_singular_power_bus_message()
+------------------------------------------------------------------------------
| Description : Configure I2C driver to use the I2C group ID to access the Power
|               resource registers.
|
| Parameters  : dev_grp 	- Processor Group of the resource
|				res_id		- ID of the power resource
|				res_state 	- State of the Power resource (OFF, ACTIVE, or SLEEP)
|
| Returns     : U16 - singular message
+-----------------------------------------------------------------------------*/
U16 form_singular_power_bus_message(U8 dev_grp,
    U8 res_id,
    U8 res_state)
{
    U16 singular_msg;
    /*  |DEV_GRP|0|ID|STATE| */
    /*   DEV_GRP[13:15],0,ID[4:11],and STATE[0:3] */
    singular_msg = ((dev_grp) << 13);   /* Resource group */
    singular_msg |= ((0) << 12);    /* Message type - Singular */
    singular_msg |= ((res_id) << 4);    /* Resource ID */
    singular_msg |= (res_state);    /* Resource State */
    return (singular_msg);
}

/*-----------------------------------------------------------------------------
| Function    : select_power_res_volt()
+------------------------------------------------------------------------------
| Description : To configure the Power Resource VSEL values.
|
| Parameters  : Triton2 DIS pointer,Offset to the power resource SEL register,resource register data
|
| Returns     : DAL Status
+-----------------------------------------------------------------------------*/
S32 select_power_res_volt(T_TRITON2_DIS * triton2_dis,
    U8 res_vsel_reg_offset,
    U8 reg_data)
{
    S32 ret_status;
    U32 size;

    ret_status = configure_i2c_id_for_power_reg(triton2_dis);

    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
        return (ret_status);
    }

    size = 1;
    ret_status =
        dal_write(triton2_i2c_handle, I2C_OFFSET(res_vsel_reg_offset),
                  &size, &reg_data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
    }
    return (ret_status);
}

/*-----------------------------------------------------------------------------
| Function    : send_singular_pb_message()
+------------------------------------------------------------------------------
| Description : Function for sending the Power Bus singular message.
|
| Parameters  : Triton2 DIS pointer,Power resource ID,Power resource state
|
| Returns     : DAL Status
+-----------------------------------------------------------------------------*/
S32 send_singular_pb_message(T_TRITON2_DIS * triton2_dis,
    U8 power_res_id,
    U8 res_state)
{

    S32 ret_status;
    U32 size;
    U8 data;
    U16 pb_message;

    ret_status = configure_i2c_id_for_power_reg(triton2_dis);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
        return (ret_status);
    }

    data = 0x02;                /* Enable I2C access to the Power Bus */
    size = 1;

    ret_status =
        dal_write(triton2_i2c_handle, I2C_OFFSET(TRITON2_PB_CFG), &size,
                  &data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        /* Power bus access configuration failed */
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
        return (ret_status);
    }

    /* Form the message for VDAC */
    pb_message =
        form_singular_power_bus_message(get_processor_group(), power_res_id,
                                        res_state);

    /* Extract the Message MSB */
    data = (pb_message >> 8);
    size = 1;

    ret_status =
        dal_write(triton2_i2c_handle, I2C_OFFSET(TRITON2_PB_MSB), &size,
                  &data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        /* Power bus access configuration failed */
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
        return (ret_status);
    }

    /* Extract the Message LSB */
    data = (pb_message & 0x00FF);
    size = 1;

    ret_status =
        dal_write(triton2_i2c_handle, I2C_OFFSET(TRITON2_PB_LSB), &size,
                  &data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
    }

    return (ret_status);
}

/*-----------------------------------------------------------------------------
| Function    : triton2_i2c_write()
+------------------------------------------------------------------------------
| Description : Function for writing to Triton2 registers
|
| Parameters  : Pointer to Triton2 DIS, register offset, size, and data
|
| Returns     : Status
+-----------------------------------------------------------------------------*/
S32 triton2_i2c_write(T_TRITON2_DIS * triton2_dis,
    U32 offset,
    U32 size,
    U8 data)
{
    S32 ret_status;
    ret_status =
        dal_write(triton2_i2c_handle, I2C_OFFSET(offset), &size, &data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec init failed");
    }
    return ret_status;
}

/*-----------------------------------------------------------------------------
| Function    : configure_i2c_id_for_triton2_addr_group2()
+------------------------------------------------------------------------------
| Description : Configure I2C driver to use the I2C group ID to access the address
|				group 2 registers.
|
| Parameters  : Triton2 DIS pointer
|
| Returns     : DAL Status
+-----------------------------------------------------------------------------*/
S32 configure_i2c_id_for_triton2_addr_group2(T_TRITON2_DIS * triton2_dis)
{
    S32 ret_status;
    U32 size;
    U8 data;

    data = TRITON2_ID2;
    size = 1;

    ret_status =
        dal_write(triton2_i2c_handle, I2C_SLAVE_ADDR, &size, &data);

    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Slave Address change failed");
    }

    return (ret_status);
}

/*-----------------------------------------------------------------------------
| Function    :  control_triton2_pwr_resource(T_TRITON2_DIS* triton2_dis,U8 res_grp_reg_offset,U8 resource_state)
+------------------------------------------------------------------------------
| Description : Function for enabling or disabling T2 power resource.
|
| Parameters  : triton2_dis - Pointer to T2 DIS
|                     res_grp_reg_offset - T2 register offset for that particular resource
|                     resource_status - State of the T2 resource state - TRITON2_RES_ACTIVE/TRITON2_RES_OFF
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
S32 control_triton2_pwr_resource(T_TRITON2_DIS * triton2_dis,
    U8 res_grp_reg_offset,
    U8 resource_state)
{
    S32 ret_status;
    U32 size = 1;
    U8 dev_grp, data, retry = 20;

    ret_status = configure_i2c_id_for_power_reg(triton2_dis);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C write function call failed ");
        return (ret_status);
    }

  CONTROL_T2_PWR_RES_LABEL:
    if (resource_state == TRITON2_RES_ACTIVE)
    {
        dev_grp = (get_processor_group() << 5); /* Selecting device group for making the power resource to ACTIVE state */
        ret_status =
            dal_write(triton2_i2c_handle, I2C_OFFSET(res_grp_reg_offset),
                      &size, &dev_grp);
        if (ret_status != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
        }
    }
    else
    {
        dev_grp = 0;            /* Selecting "no device" device group for making the power resource to OFF state */
        ret_status =
            dal_write(triton2_i2c_handle, I2C_OFFSET(res_grp_reg_offset),
                      &size, &dev_grp);
        if (ret_status != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Write failed ");
        }
    }

    /* Quick workaround for handling T2 power setting issue
     * Check if the resource is active, else, retry enabling the same
     */
    while (--retry)
    {
        size = 1;
        data = 0;
        ret_status =
            dal_read(triton2_i2c_handle, I2C_OFFSET(res_grp_reg_offset),
                     &size, &data);
        if (ret_status != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:I2C Read failed ");
            return (ret_status);
        }

        if (resource_state == TRITON2_RES_ACTIVE)
        {
            if ((data & 0x0F) == 0x0E)
            {
                break;
            }
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:PWR Fail");

            /* else */
            goto CONTROL_T2_PWR_RES_LABEL;  /*Retry */
        }
        else
        {
            if ((data & 0x0F) == 0x00)
            {
                break;
            }

            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:PWR Fail");
            /* else */
            goto CONTROL_T2_PWR_RES_LABEL;  /*Retry */
        }
    }
    if (retry != 0)
    {
        return (ret_status);
    }
    /* else */
    return (!ret_status);
}

#ifdef INTERRUPT
/*-----------------------------------------------------------------------------
| Function    : triton2_isr()
+------------------------------------------------------------------------------
| Description : Interrupt hanlder for Triton2 to interrupts
|
| Parameters  : Data
|
| Returns     : DAL Status
+-----------------------------------------------------------------------------*/
S32 triton2_isr(void *data_in)
{
    U32 size = 1;

    U8 i2c_addr = TRITON2_ID2;

    /* Read the PIH regsiter to check the interrupt source */
    /* this wil not clear any bits ,but inform the host on which subsytem sih_ register
       to be read */
    U8 pih_isr_reg;

    /* get the interrupt source */
    t2_reg_read(triton2_i2c_handle, i2c_addr, TRITON2_PIH_ISR1_REG, &size,
                &pih_isr_reg);

    if ((pih_isr_reg & POWER_MANAGEMENT_DEDICATED_INT) ==
        POWER_MANAGEMENT_DEDICATED_INT)
    {
        active_int = 5;
    }

    if ((pih_isr_reg & USB_DEDICATED_INT) == USB_DEDICATED_INT)
    {
        active_int = 4;
    }
    if ((pih_isr_reg & MADC_DEDICATED_INT) == MADC_DEDICATED_INT)
    {
        active_int = 3;
    }
    if ((pih_isr_reg & BCI_DEDICATED_INT) == BCI_DEDICATED_INT)
    {
        active_int = 2;
    }
    if ((pih_isr_reg & KEYPAD_DEDICATED_INT) == KEYPAD_DEDICATED_INT)
    {
        active_int = 1;
    }
    if ((pih_isr_reg & GPIO_DEDICATED_INT) == GPIO_DEDICATED_INT)
    {
        active_int = 0;
    }

    if (active_int != 0)
    {
        call_isr_func_trtn2(active_int);
    }

    return 0;
}

/*-----------------------------------------------------------------------------
| Function    : confgure_t2_pwrbutton_int
+------------------------------------------------------------------------------
| Description : This function add the interrupt handler to handle Power interrupts.
|
| Parameters  : U32 triton2_dis
|
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
S32 confgure_t2_pwrbutton_int(U32 triton2_dis)
{

    triton2_interrupt_init();

    /* add interrupt handler for the power on interrupt */
    triton2_add_int_handler(5, t2_pwr_isr, (void *)triton2_dis);

    return OMAPFLASH_SUCCESS;
}
#endif //INTERRUPT

/*-----------------------------------------------------------------------------
| Function    : t2_pwr_isr
+------------------------------------------------------------------------------
| Description : This function clears the interrupt status on getting pwron interrut
|				and handle the interrupt.
|
| Parameters  :
|
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/
void t2_pwr_isr(void *dis_addr)
{
    T_TRITON2_DIS *triton2_dis = (T_TRITON2_DIS *) dis_addr;
    S32 ret_val;
    U8 temp_data = 0x00;
    U32 length = 1;

    temp_data = 0x4B;
    /* configure i2c for power registers */
    ret_val =
        dal_write(triton2_i2c_handle, I2C_SLAVE_ADDR, &length, &temp_data);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C slave adress write failed");
    }

    /* Read the PWR_ISR1  regsiter through i2c */
    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(TRITON2_PWR_ISR1_REG),
                 &length, &temp_data);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Read to PWR_ISR1 reg failed");
        return;
    }

    /* if the interrupt is power on interrupt ,clear the interrupt status */
    if ((temp_data & PWRON_INT) == PWRON_INT)
    {
        temp_data = PWRON_INT;
        /*write to PWR_ISR1 reg - to clear the interrupt status */
        ret_val =
            dal_write(triton2_i2c_handle, I2C_OFFSET(TRITON2_PWR_ISR1_REG),
                      &length, &temp_data);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            /* debug print */
            dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_ISR1 reg failed");
        }

        /* call the call back function to handle the interrupt */
        (triton2_dis->triton2_initstr.t2pwron_irq_callback) ();
    }
    /* if the interrupt is hot die  interrupt ,clear the interrupt status */
    else if ((temp_data & HOT_DIE_INT) == HOT_DIE_INT)
    {
        temp_data = HOT_DIE_INT;
        /*write to PWR_ISR1 reg - to clear the interrupt status */
        ret_val =
            dal_write(triton2_i2c_handle, I2C_OFFSET(TRITON2_PWR_ISR1_REG),
                      &length, &temp_data);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            /* debug print */
            dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to PWR_ISR1 reg failed");
        }
        /* call the call back function to handle the interrupt */
        (triton2_dis->triton2_initstr.t2pwron_irq_callback) ();

    }
    else
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "unknown interrupt source ");

    }
    return;
}

/*-----------------------------------------------------------------------------
| Function    : configure_triton2_leds
+------------------------------------------------------------------------------
| Description :  The Triton2 led is configured
|
| Parameters  : U32 triton2led_dis
|
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/

#ifdef OMAP3430SDP
S32 configure_triton2_leds(U32 triton2led_dis)
{
    U8 buffer = 0, ledno, data = 0, brightness_value, ledpwmoff =
        0, pwmduty_cycle = 0;
    S32 ret_val = OMAPFLASH_SUCCESS; /* default */
    U32 length = 1;
    T_TRITON2_DIS *triton2_dis = (T_TRITON2_DIS *) triton2led_dis;

    /*Set the led pwm off value */
    ledpwmoff = T2LED_PWROFF_VALUE;

    /* configure the i2c slave address for the Power regsiters */
    configure_i2c_slave_id_for_triton2(triton2_i2c_handle,
                                       T2_I2C_LED_ADDR_GROUP);

    /* Read T2_LEDEN register */
    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(T2_LEDEN), &length,
                 &buffer);
    /* Report if error */
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error reading LED register value\r\n");
        return ret_val;
    }

    data = triton2_dis->triton2_led.led_enable_disable_flag;
    ledno = triton2_dis->triton2_led.led_no;

    if (data == LED_ENABLE)
    {
        if (ledno == LEDA)
        {
            buffer |= (LEDAON | LEDAPWM);
        }
        else if (ledno == LEDB)
        {
            buffer |= (LEDBON | LEDBPWM);
        }
    }
    else if (data == LED_DISABLE)
    {
        if (ledno == LEDA)
        {
            buffer &= (~(LEDAON | LEDAPWM));
        }
        else if (ledno == LEDB)
        {
            buffer &= (~(LEDBON | LEDBPWM));
        }

    }

    /* write to T2_LEDEN register */
    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(T2_LEDEN), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to T2_LEDEN reg failed");
        return ret_val;
    }

    if (data == LED_ENABLE)
    {
        brightness_value = triton2_dis->triton2_led.led_brightness;
        pwmduty_cycle = calculate_pwmontime(brightness_value);
        if (ledno == LEDA)
        {
            /* write to T2_PWMAONregister */
            ret_val =
                dal_write(triton2_i2c_handle, I2C_OFFSET(T2_PWMAON),
                          &length, &pwmduty_cycle);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                /* debug print */
                dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to T2_PWMAON reg failed");
                return ret_val;
            }

            /* write to T2_PWMAOFF register */
            ret_val =
                dal_write(triton2_i2c_handle, I2C_OFFSET(T2_PWMAOFF),
                          &length, &ledpwmoff);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                /* debug print */
                dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to T2_PWMAOFF reg failed");
                return ret_val;
            }
        }
        else if (ledno == LEDB)
        {
            /* write to T2_PWMAONregister */
            ret_val =
                dal_write(triton2_i2c_handle, I2C_OFFSET(T2_PWMBON),
                          &length, &pwmduty_cycle);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                /* debug print */
                dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to T2_PWMBON reg failed");
                return ret_val;
            }

            /* write to T2_PWMBOFF register */
            ret_val =
                dal_write(triton2_i2c_handle, I2C_OFFSET(T2_PWMBOFF),
                          &length, &ledpwmoff);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                /* debug print */
                dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to T2_PWMBOFF reg failed");
                return ret_val;
            }

        }
    }
    /* Setup pin mux */
    /* configure the i2c slave address for the Pin mux reg */
    configure_i2c_slave_id_for_triton2(triton2_i2c_handle,
                                       T2_PMBR1_ADDR_GROUP);
    length = 1;

    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(PMBR1_REG),
                 (U32 *) & length, &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Read from T2_PMBR1_REG reg failed");
        return ret_val;
    }
    /* Clear the field */
    buffer &=
        ((ledno ==
          LEDA) ? (~PMBR1_GPIO6_CLKOK_PWM0_MUTE)
         : (~PMBR1_GPIO7_VIBRASYNC_PWM1_MASK));
    /* Set the field */
    if (data == LED_ENABLE)
    {
        buffer |= ((ledno == LEDA) ? PMBR1_PWM0_ENABLE : PMBR1_PWM1_ENABLE);
    }
    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(PMBR1_REG),
                  (U32 *) & length, &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to T2_PMBR1_REG reg failed");
        /* Fall thru */
    }

    return ret_val;

}
#else
S32 configure_triton2_leds(U32 triton2led_dis)
{
    U8 buffer = 0, ledno, data = 0, brightness_value, on_reg, off_reg;
    S32 ret_val = OMAPFLASH_SUCCESS; /* default */
    U32 length = 1;
    T_TRITON2_DIS *triton2_dis = (T_TRITON2_DIS *) triton2led_dis;

    data = triton2_dis->triton2_led.led_enable_disable_flag;
    ledno = triton2_dis->triton2_led.led_no;
    brightness_value = triton2_dis->triton2_led.led_brightness;


    /* STEP1:
     * Configure the time when the PWM will go ON and when it should
     * switch back off. Both on and off regs are based from clock 0
     * PWM0,1 get clocks from 32Khz clock. each cycle is 1/256 of a sec
     */
    /* configure the i2c slave address for the Power regsiters */
    configure_i2c_slave_id_for_triton2(triton2_i2c_handle,
                                       T2_I2C_LED_ADDR_GROUP);
    on_reg = (ledno == LEDA) ? PWM0_ON_REG : PWM1_ON_REG;
    off_reg = (ledno == LEDA) ? PWM0_OFF_REG : PWM1_OFF_REG;
    /* Set up when to switch off PWM */
    buffer = T2LED_PWROFF_VALUE;
    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(off_reg), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error writing off register\r\n");
        return ret_val;
    }
    /* Setup when to switch on PWM */
    if (data == LED_ENABLE)
    {
        buffer = calculate_pwmontime(brightness_value);
    }
    else
    {
        buffer = T2LED_PWROFF_VALUE;
    }
    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(on_reg), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error reading LED register value\r\n");
        return ret_val;
    }

    /* Step 2 - we need to enable two things:
     * Enable 32Khz input clock to PWM module and enable the
     * PWM output */
    configure_i2c_slave_id_for_triton2(triton2_i2c_handle,
                                       T2_GPBR1_ADDR_GROUP);
    /* Read the GPBR1 register */
    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(GPBR1_REG), &length,
                 &buffer);
                 
    /* Report if error */
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error reading GPBR1_REG \r\n");
        return ret_val;
    }
    buffer &= ((ledno == LEDA) ? ~GPBR1_PWM0_MASK : ~GPBR1_PWM1_MASK);
    if (data == LED_ENABLE)
    {
        buffer |= ((ledno == LEDA) ? GPBR1_PWM0_MASK : GPBR1_PWM1_MASK);
    }
    /* Set the GPBR1 register */
    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(GPBR1_REG), &length,
                  &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_INFO, "Error writing GPBR1_REG\r\n");
        return ret_val;
    }

    /* Step 3: Setup pin mux */
    /* configure the i2c slave address for the Pin mux reg */
    configure_i2c_slave_id_for_triton2(triton2_i2c_handle,
                                       T2_PMBR1_ADDR_GROUP);

    ret_val =
        dal_read(triton2_i2c_handle, I2C_OFFSET(PMBR1_REG),
                 (U32 *) & length, &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Read from T2_PMBR1_REG reg failed");
        return ret_val;
    }
    /* Clear the field */
    buffer &=
        ((ledno ==
          LEDA) ? (~PMBR1_GPIO6_CLKOK_PWM0_MUTE)
         : (~PMBR1_GPIO7_VIBRASYNC_PWM1_MASK));
    /* Set the field */
    if (data == LED_ENABLE)
    {
        buffer |= ((ledno == LEDA) ? PMBR1_PWM0_ENABLE : PMBR1_PWM1_ENABLE);
    }
    ret_val =
        dal_write(triton2_i2c_handle, I2C_OFFSET(PMBR1_REG),
                  (U32 *) & length, &buffer);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        /* debug print */
        dbg_print(DBG_LEVEL_CRITICAL, "Triton2 : I2C Write to T2_PMBR1_REG reg failed");
        /* Fall thru */
    }

    return ret_val;

}
#endif /* #ifndef OMAP3430SDP_CHMN */

/*-----------------------------------------------------------------------------
| Function    :  calculate_pwmdutycycle
+------------------------------------------------------------------------------
| Description :  The function calculates the dutycycle of the pwm
|
| Parameters  : U8 brightness_value
|
| Returns     : S32(OMAPFLASH_SUCCESS OR error code of the error type.)
+-----------------------------------------------------------------------------*/

U8 calculate_pwmontime(U8 dutycycle)
{
    U8 pwmontime;

    if (dutycycle != 0)
    {
        /* To calculate the pwm on time from the dutycycle given by the user */
        pwmontime = 127 - (((dutycycle) * 128) / 100);
    }
    else if (dutycycle == 0)
    {
        pwmontime = 0;
    }
    return pwmontime;
}

/*==== END OF FILE ===========================================================*/
