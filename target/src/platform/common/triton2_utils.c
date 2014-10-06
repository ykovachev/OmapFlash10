/**
 * @file triton2_utils.c
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
 * This file contains the Triton2 power resource configuration functions
 * 
 */

/*==== DECLARATION CONTROL ===================================================*/

/*==== INCLUDES ==============================================================*/

#include "csst_tgt.h"
#include "triton2_drv.h"
#include "triton2_utils.h"
#include "triton2_usb.h"
#include "dl_pfwork_ex.h"
#include "i2c_drv.h"
#include "triton2.h"
#include "dbg_ex.h"

#define ES_2P1			2

U8 omap_silicon_revision()
{
    return (0); //@todo get real silicon revision
}

/* Prototype Functions */
U32 configure_triton2_power_resources(U32 power_res_tag, U8 res_voltage,
                                      U32 power_res_on_off_tag,
                                      U8 power_res_state);

/*============ FUNCTIONS =====================================================*/
/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vdac
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VDAC power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vdac()
{
    U32 ret_val;
    /* Configure and enable VDAC for 1.8v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VDAC_VOLTAGE_TAG, VDAC_1P8,
                                          TRITON2_VDAC_ON_OFF_TAG,
                                          TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VDAC configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vdac
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VDAC power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vdac()
{
    U32 ret_val;
    /* Disable the VDAC */
    ret_val =
        configure_triton2_power_resources(TRITON2_VDAC_VOLTAGE_TAG, VDAC_1P8,
                                          TRITON2_VDAC_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VDAC configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vpll1
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VPLL1 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vpll1()
{
    U32 ret_val;
    /* Configure and enable VPLL1 for 1.3v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VPLL1_VOLTAGE_TAG, VPLL1_1P3,
                                          TRITON2_VPLL1_ON_OFF_TAG,
                                          TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VPLL1 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vpll1
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VDAC power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vpll1()
{
    U32 ret_val;
    /* Disable VPLL1 */
    ret_val =
        configure_triton2_power_resources(TRITON2_VPLL1_VOLTAGE_TAG, VPLL1_1P3,
                                          TRITON2_VPLL1_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VPLL1 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vpll2(U8 voltage)
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VPLL2 power resources for 3430 SDP
|
| Parameters  : voltage - parameter to select different voltages
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vpll2(U8 voltage)
{
    U32 ret_val;

    /* return 0;  *//* debugging */
    if (VPLL2_V1P8 == voltage)
    {
        /* Configure and enable VPLL2 for 1.8v */
        ret_val =
            configure_triton2_power_resources(TRITON2_VPLL2_VOLTAGE_TAG,
                                              VPLL2_1P8,
                                              TRITON2_VPLL2_ON_OFF_TAG,
                                              TRITON2_RES_ACTIVE);
    }
    else if (VPLL2_V1P3 == voltage)
    {
        /* Configure and enable VPLL2 for 1.3v */
        ret_val =
            configure_triton2_power_resources(TRITON2_VPLL2_VOLTAGE_TAG,
                                              VPLL2_1P3,
                                              TRITON2_VPLL2_ON_OFF_TAG,
                                              TRITON2_RES_ACTIVE);
    }

    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VPLL2 configuration (enabling) failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vpll2
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VPLL2 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vpll2()
{
    U32 ret_val;
    /* Disable VPLL2 */
    ret_val =
        configure_triton2_power_resources(TRITON2_VPLL2_VOLTAGE_TAG, VPLL2_1P8,
                                          TRITON2_VPLL2_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VPLL2 disabling  failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vmmc1
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VMMC1 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vmmc1(U8 voltage)
{
    U32 ret_val;
    /* Configure and enable VMMC1 for 3.0v */
    if (VMMC1_V3P0 == voltage)
    {
        ret_val =
            configure_triton2_power_resources(TRITON2_VMMC1_VOLTAGE_TAG,
                                              VMMC1_3P0,
                                              TRITON2_VMMC1_ON_OFF_TAG,
                                              TRITON2_RES_ACTIVE);
    }
    else
    {
        /* Configure and enable VMMC1 for 1.85v */
        ret_val =
            configure_triton2_power_resources(TRITON2_VMMC1_VOLTAGE_TAG,
                                              VMMC1_1P85,
                                              TRITON2_VMMC1_ON_OFF_TAG,
                                              TRITON2_RES_ACTIVE);
    }
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VMMC1 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vmmc1
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VMMC1 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vmmc1(U8 voltage)
{
    U32 ret_val;
    /* Disable VMMC1 */
    if (VMMC1_V3P0 == voltage)
    {
        ret_val =
            configure_triton2_power_resources(TRITON2_VMMC1_VOLTAGE_TAG,
                                              VMMC1_3P0,
                                              TRITON2_VMMC1_ON_OFF_TAG,
                                              TRITON2_RES_OFF);
    }
    else
    {
        ret_val =
            configure_triton2_power_resources(TRITON2_VMMC1_VOLTAGE_TAG,
                                              VMMC1_1P85,
                                              TRITON2_VMMC1_ON_OFF_TAG,
                                              TRITON2_RES_OFF);
    }
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VMMC1 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vmmc2
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VMMC2 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vmmc2()
{
    U32 ret_val;
    /* Configure and enable VMMC2 for 1.85v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VMMC2_VOLTAGE_TAG, VMMC2_1P85,
                                          TRITON2_VMMC2_ON_OFF_TAG,
                                          TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VMMC2 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vmmc2
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VMMC1 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vmmc2()
{
    U32 ret_val;
    /* Disable VMMC2 */
    ret_val =
        configure_triton2_power_resources(TRITON2_VMMC2_VOLTAGE_TAG, VMMC2_1P85,
                                          TRITON2_VMMC2_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VMMC2 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vaux1
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VAUX1 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vaux1()
{
    U32 ret_val;
    /* Configure and enable VAUX1 for 2.8v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX1_VOLTAGE_TAG, VAUX1_2P8,
                                          TRITON2_VAUX1_ON_OFF_TAG,
                                          TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX1 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vaux1
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VAUX1 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vaux1()
{
    U32 ret_val;
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX1_VOLTAGE_TAG, VAUX1_2P8,
                                          TRITON2_VAUX1_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX1 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vaux2
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VAUX2 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vaux2()
{
    U32 ret_val;
    /* Configure and enable VAUX2 for 2.8v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX2_VOLTAGE_TAG, VAUX2_2P8,
                                          TRITON2_VAUX2_ON_OFF_TAG,
                                          TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX2 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vaux2
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VAUX2 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vaux2()
{
    U32 ret_val;
    /* Disable VAUX2 */
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX2_VOLTAGE_TAG, VAUX2_2P8,
                                          TRITON2_VAUX2_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX2 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vaux3
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VAUX3 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vaux3()
{
    U32 ret_val;
    /* Configure and enable VAUX3 for 2.8v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX3_VOLTAGE_TAG, VAUX3_2P8,
                                          TRITON2_VAUX3_ON_OFF_TAG,
                                          TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX3 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vaux3
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VAUX1 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vaux3()
{
    U32 ret_val;
    /* Disable VAUX3 */
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX3_VOLTAGE_TAG, VAUX3_2P8,
                                          TRITON2_VAUX3_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX3 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vaux4
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VAUX4 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vaux4()
{
    U32 ret_val;
    /* Configure and enable VAUX4 for 1.8v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX4_VOLTAGE_TAG, VAUX4_1P8,
                                          TRITON2_VAUX4_ON_OFF_TAG,
                                          TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX4 configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vaux4
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VAUX4 power resources for 3430 SDP
|
| Parameters  : none
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vaux4()
{
    U32 ret_val;
    /* Disable VAUX4 */
    /* Configure and disable VAUX4 for 1.8v */
    ret_val =
        configure_triton2_power_resources(TRITON2_VAUX4_VOLTAGE_TAG, VAUX4_1P8,
                                          TRITON2_VAUX4_ON_OFF_TAG,
                                          TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VAUX4 configuration failed ");
    }
    return (ret_val);
}

/*============ FUNCTIONS =====================================================*/
/*-----------------------------------------------------------------------------
| Function    : enable_triton2_vsim
+------------------------------------------------------------------------------
| Description : Enable the Triton2 VSIM power resources for 3430 SDP
|
| Parameters  : voltage to set it to
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 enable_triton2_vsim(U8 voltage)
{
    U32 ret_val;
    U8 volt = 0;
    switch (voltage)
    {
        case VSIM_V1P8:
            volt = VSIM_1P8;
            break;
        case VSIM_V2P8:
            volt = VSIM_2P8;
            break;
        case VSIM_V3P0:
            volt = VSIM_3P0;
            break;
        default:
            return OMAPFLASH_DAL_ERROR;
    }
    /* Configure and enable VSIM for requested voltage */
    ret_val = configure_triton2_power_resources(TRITON2_VSIM_VOLTAGE_TAG,
                                                volt,
                                                TRITON2_VSIM_ON_OFF_TAG,
                                                TRITON2_RES_ACTIVE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VSIM setup configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_vsim
+------------------------------------------------------------------------------
| Description : Disable the Triton2 VSIM power resources for 3430 SDP
|
| Parameters  : voltage to set it to
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 disable_triton2_vsim(U8 voltage)
{

    U32 ret_val;
    U8 volt = 0;
    switch (voltage)
    {
        case VSIM_V1P8:
            volt = VSIM_1P8;
            break;
        case VSIM_V2P8:
            volt = VSIM_2P8;
            break;
        case VSIM_V3P0:
            volt = VSIM_3P0;
            break;
        default:
            return OMAPFLASH_DAL_ERROR;
    }
    /* Configure and enable VSIM for requested voltage */
    ret_val = configure_triton2_power_resources(TRITON2_VSIM_VOLTAGE_TAG,
                                                volt,
                                                TRITON2_VSIM_ON_OFF_TAG,
                                                TRITON2_RES_OFF);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2: VSIM shutdown configuration failed ");
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : configure_triton2_power_resources
+------------------------------------------------------------------------------
| Description : Common function for disabling and enabling the power resources
|
| Parameters  : Power Resource TAG, Resource Voltage,Power resource ON/OFF tag, Resource State
|
| Returns     : U32 (error code)
+-----------------------------------------------------------------------------*/
U32 configure_triton2_power_resources(U32 power_res_tag, U8 res_voltage,
                                      U32 power_res_on_off_tag,
                                      U8 power_res_state)
{
    U32 ret_val;
    U32 triton2_handle;
    U32 size;

    T_TRITON2_INIT_STRUCTURE triton2_init_str;

    /* Initialize the init structure */
    triton2_init_str.pid = 38; //DAL_TRITON2_PID;
    triton2_init_str.sid = DAL_TRITON2_SID;
    triton2_init_str.sampling_rate = 0;
    triton2_init_str.t2pwron_irq_callback = NULL;

#ifdef _MSC_VER
#define S1(s) #s
#define S2(s) S1(s)
#pragma message (S2(dal_init((void *)&triton2_init_str, &triton2_handle);))
#endif //_MSC_VER
    /* Initialize the Triton2 DIS */
    ret_val = dal_init((void *)&triton2_init_str, &triton2_handle);

    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL init failed (configure_triton2_power_resources)");
        return (ret_val);
    }

    if (power_res_state == TRITON2_RES_ACTIVE)  /* No need to set the voltage if we want to turn the power off */
    {
        size = 1;
        /* Configure the power resource voltage */
        ret_val = dal_write(triton2_handle, power_res_tag, &size, &res_voltage);

        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL write failed");
            /* Deinitialize the Triton2 DIS */
            ret_val = dal_deinit(triton2_handle);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
                return (ret_val);
            }
            return (ret_val);
        }
    }

    size = 1;
    /* Enable or Disable the power source */
    ret_val =
        dal_write(triton2_handle, power_res_on_off_tag, &size,
                  &power_res_state);

    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL write failed");
        /* Deinitialize the Triton2 DIS */
        ret_val = dal_deinit(triton2_handle);

        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
            return (ret_val);
        }
        return (ret_val);
    }

    /* Deinitialize the Triton2 DIS */
    ret_val = dal_deinit(triton2_handle);

    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
        return (ret_val);
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    :S32 triton2_gpio_pin_init(U16 pin_num, U8 in_out)
+------------------------------------------------------------------------------
| Description :This function is to initialize the gpio pin as input or output.
|
| Parameters  :pin_num
|                 - gpio pin that needs to be configured.
|
|              in_out
|                 - specifies whether the pin is to be configured as an
|                   input or output.
|  				TRITON2_GPIO_OUTPUT  1
|  				TRITON2_GPIO_INPUT   0
|
| Returns     : DAL_SUCCESS, if the initialization is successful.
|				else DAL_ERROR.
+-----------------------------------------------------------------------------*/
S32 triton2_gpio_pin_init(U8 pin_num, U8 in_out)
{
    S32 ret_val = OMAPFLASH_SUCCESS;
    U32 triton2_handle, size = 0;
    U8 deinit = 1;
    T_TRITON2_INIT_STRUCTURE triton2_init_str;
    T_TRITON2_GPIO_STRUCTURE trtn2_gpio;

    /* Initialize the init structure */
    triton2_init_str.pid = 38; //DAL_TRITON2_PID;
    triton2_init_str.sid = DAL_TRITON2_SID;
    triton2_init_str.sampling_rate = 0;
    triton2_init_str.t2pwron_irq_callback = NULL;

    /* there are 18 gpio pins.. (0-17) */
    if (pin_num > 17)
    {
        dbg_print(DBG_LEVEL_INFO, "max number of gpio's = 18 \n");
    }

    /* Initialize the Triton2 DIS */
    ret_val = dal_init((void *)&triton2_init_str, &triton2_handle);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        if (ret_val != OMAPFLASH_DEVICE_ALREADY_INITIALIZED)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL init failed (triton2_gpio_pin_init)");
            return (ret_val);
        }
        else
        {
            deinit = 0;
        }
    }

    /* set the data direction of the triton2 gpio pin specified */
    size = 1;
    trtn2_gpio.gpio_pin_num = pin_num;
    trtn2_gpio.gpio_data_dir = in_out;

    ret_val =
        dal_write(triton2_handle, TRITON2_GPIO_DATA_DIR_TAG, &size,
                  (U8 *) & trtn2_gpio);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL write to GPIO failed");
        /* Deinitialize the Triton2 DIS and then return */
        if (deinit == 1)
        {
            ret_val = dal_deinit(triton2_handle);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
                return (ret_val);
            }
        }
        return (ret_val);
    }

    /* Deinitialize the Triton2 DIS */
    if (deinit == 1)
    {
        ret_val = dal_deinit(triton2_handle);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
            return (ret_val);
        }
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : S32 triton2_get_gpio_input(U32 pin_num, U8 *gpio_data)
+------------------------------------------------------------------------------
| Description : for getting data from GPIO pins
|
| Parameters  : pin_num -
|                 gpio pin number
|				gpio_data -
|				  pointer where the gpio data will be stored.
|
| Returns     : ERROR is returned if failure
|               else SUCCESS is returned.
+-----------------------------------------------------------------------------*/
S32 triton2_get_gpio_input(U32 pin_num, U8 * gpio_data)
{
    S32 ret_val = OMAPFLASH_DAL_ERROR;
    T_TRITON2_GPIO_STRUCTURE trtn2_gpio;
    U32 triton2_handle, size = 0;
    U8 deinit = 1;
    T_TRITON2_INIT_STRUCTURE triton2_init_str;

    /* Initialize the init structure */
    triton2_init_str.pid = 38; //DAL_TRITON2_PID;
    triton2_init_str.sid = DAL_TRITON2_SID;
    triton2_init_str.sampling_rate = 0;
    triton2_init_str.t2pwron_irq_callback = NULL;

    /* there are 18 gpio pins.. (0-17) */
    if (pin_num > 17)
    {
        dbg_print(DBG_LEVEL_INFO, "max number of gpio's = 18 \n");
    }

    /* Initialize the Triton2 DIS */
    ret_val = dal_init((void *)&triton2_init_str, &triton2_handle);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        if (ret_val != OMAPFLASH_DEVICE_ALREADY_INITIALIZED)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL init failed (triton2_get_gpio_input)");
            return (ret_val);
        }
        else
        {
            deinit = 0;
        }
    }

    /* read the data at the gpio pin specified */
    size = 1;
    trtn2_gpio.gpio_pin_num = pin_num;
    trtn2_gpio.gpio_data = 0;   /* data */
    ret_val =
        dal_read(triton2_handle, TRITON2_GPIO_DATA_TAG, &size,
                 (U8 *) & trtn2_gpio);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL GPIO read failed");
        /* Deinitialize the Triton2 DIS and then return */
        if (deinit == 1)
        {
            ret_val = dal_deinit(triton2_handle);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
                return (ret_val);
            }
        }
        return (ret_val);
    }
    /* store the value in the variable specified by the user of this function */
    *gpio_data = trtn2_gpio.gpio_data;

    /* Deinitialize the Triton2 DIS */
    if (deinit == 1)
    {
        ret_val = dal_deinit(triton2_handle);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
            return (ret_val);
        }
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    : S32 triton2_set_gpio_output(U16 pin_num, U8 set)
+------------------------------------------------------------------------------
| Description : for writing data to the GPIO pins
|
| Parameters  : pin_num -
|                 gpio pin number
|               set -
|                 set/clear information
|				TRITON2_GPIO_SET_DATA   - 1
|				TRITON2_GPIO_CLEAR_DATA - 0
| Returns     : ERROR is returned if failure else the value returned
|               by the lower layer function is passed on.
+-----------------------------------------------------------------------------*/
S32 triton2_set_gpio_output(U16 pin_num, U8 set)
{
    S32 ret_val = OMAPFLASH_DAL_ERROR;
    U32 triton2_handle, size = 0;
    U8 deinit = 1;
    T_TRITON2_INIT_STRUCTURE triton2_init_str;
    T_TRITON2_GPIO_STRUCTURE trtn2_gpio;
    /* Initialize the init structure */
    triton2_init_str.pid = 38; //DAL_TRITON2_PID;
    triton2_init_str.sid = DAL_TRITON2_SID;
    triton2_init_str.sampling_rate = 0;
    triton2_init_str.t2pwron_irq_callback = NULL;
    /* there are 18 gpio pins.. (0-17) */
    if (pin_num > 17)
    {
        dbg_print(DBG_LEVEL_INFO, "max number of gpio's = 18 \n");
    }

    /* Initialize the Triton2 DIS */
    ret_val = dal_init((void *)&triton2_init_str, &triton2_handle);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        if (ret_val != OMAPFLASH_DEVICE_ALREADY_INITIALIZED)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL init failed (triton2_set_gpio_output)");
            return (ret_val);
        }
        else
        {
            deinit = 0;
        }
    }

    /* write the data to the gpio pin specified by the user of this function */
    size = 1;
    trtn2_gpio.gpio_pin_num = pin_num;
    trtn2_gpio.gpio_data = set; /* data */

    ret_val =
        dal_write(triton2_handle, TRITON2_GPIO_DATA_TAG, &size,
                  (U8 *) & trtn2_gpio);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL write to GPIO failed");
        /* Deinitialize the Triton2 DIS and then return */
        if (deinit == 1)
        {
            ret_val = dal_deinit(triton2_handle);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
                return (ret_val);
            }
        }
        return (ret_val);
    }

    /* Deinitialize the Triton2 DIS */
    if (deinit == 1)
    {
        ret_val = dal_deinit(triton2_handle);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
            return (ret_val);
        }
    }
    /* return from this function */
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    :S32 triton2_gpio_pin_deinit(U16 pin_num)
+------------------------------------------------------------------------------
| Description :	This function de initializes all the features associated with
|               the pins and restores its default/reset state.
|
| Parameters  : pin_num
|						-   this specifies the gpio pin number that has to be
|                           deinitialized.
|
| Returns     : SUCCESS, if deinitialisation is successful
|               else ERROR.
|
+-----------------------------------------------------------------------------*/
S32 triton2_gpio_pin_deinit(U16 pin_num)
{
    S32 ret_val = OMAPFLASH_SUCCESS;
    U32 triton2_handle, size = 0;
    U8 deinit = 1;
    T_TRITON2_INIT_STRUCTURE triton2_init_str;
    T_TRITON2_GPIO_STRUCTURE trtn2_gpio;

    /* Initialize the init structure */
    triton2_init_str.pid = 38; //DAL_TRITON2_PID;
    triton2_init_str.sid = DAL_TRITON2_SID;
    triton2_init_str.sampling_rate = 0;
    triton2_init_str.t2pwron_irq_callback = NULL;
    /* there are 18 gpio pins.. (0-17) */
    if (pin_num > 17)
    {
        dbg_print(DBG_LEVEL_INFO, "max number of gpio's = 18 \n");
    }

    /* Initialize the Triton2 DIS */
    ret_val = dal_init((void *)&triton2_init_str, &triton2_handle);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        if (ret_val != OMAPFLASH_DEVICE_ALREADY_INITIALIZED)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL init failed (triton2_gpio_pin_deinit)");
            return (ret_val);
        }
        else
        {
            deinit = 0;
        }
    }

    /* set the data direction back to the default value(input) */
    size = 1;
    trtn2_gpio.gpio_pin_num = pin_num;
    trtn2_gpio.gpio_data_dir = TRITON2_GPIO_INPUT;

    ret_val =
        dal_write(triton2_handle, TRITON2_GPIO_DATA_DIR_TAG, &size,
                  (U8 *) & trtn2_gpio);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL write to GPIO failed");
        /* Deinitialize the Triton2 DIS and then return */
        if (deinit == 1)
        {
            ret_val = dal_deinit(triton2_handle);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
                return (ret_val);
            }
        }
        return (ret_val);
    }

    /* Deinitialize the Triton2 DIS */
    if (deinit == 1)
    {
        ret_val = dal_deinit(triton2_handle);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:DAL deinit failed");
            return (ret_val);
        }
    }
    return (ret_val);
}

/*-----------------------------------------------------------------------------
| Function    :S32 simcard_detect()
+------------------------------------------------------------------------------
| Description :	This function detects the presence of the sim card
| Parameters  : none
| Returns     : TRUE if sim card is inserted
|               FALSE if sim card is not inserted or for any other error
|
+-----------------------------------------------------------------------------*/
S32 simcard_detect()
{
    U8 sim_card_detector = TRITON2_GPIO_15;
    U8 detect_val;
    S32 ret_val = OMAPFLASH_SUCCESS;

    /* initialize the gpio pin as an input */
    ret_val = triton2_gpio_pin_init(sim_card_detector, TRITON2_GPIO_INPUT);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio init failed");
        return FALSE;
    }
    /* get the gpio data */
    ret_val = triton2_get_gpio_input(sim_card_detector, &detect_val);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:get gpio input failed");
        /* Deinitialize the Triton2 DIS and then return */
        ret_val = triton2_gpio_pin_deinit(sim_card_detector);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
            return FALSE;
        }
        return FALSE;
    }
    /* deinitialize the gpio pin settings */
    ret_val = triton2_gpio_pin_deinit(sim_card_detector);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
        return FALSE;
    }

    if (detect_val)
    {
        /* GPIO input(detect_val) is one means, headset is not present */
        return FALSE;
    }
    else
    {
        /* GPIO input(detect_val) is zero means, headset is present */
        return TRUE;
    }

}

/*-----------------------------------------------------------------------------
| Function    :S32 headset_detect()
+------------------------------------------------------------------------------
| Description :	This function is used to detect the presence of headset
| Parameters  : none
| Returns     : TRUE if headset is attached
|               FALSE if headset is not attached or for any other error
|
+-----------------------------------------------------------------------------*/
S32 headset_detect()
{
    U8 headset_detector = TRITON2_GPIO_2;
    U8 detect_val;
    S32 ret_val = OMAPFLASH_SUCCESS;

    /* Initialize the T2 GPIO2 as input for headset detect */
    ret_val = triton2_gpio_pin_init(headset_detector, TRITON2_GPIO_INPUT);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio init failed");
        return FALSE;
    }

    /* Drive the headset mute GPIO */
    ret_val = audio_mute_control(TRITON2_AUDIO_MUTE_ENABLE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:Headset mute on failed");
        return FALSE;
    }

    /* Get the headset detect GPIO input value to see whether headset is present */
    ret_val = triton2_get_gpio_input(headset_detector, &detect_val);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:get gpio input failed");
        /* Deinitialize the GPIO and then return */
        ret_val = triton2_gpio_pin_deinit(headset_detector);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
            return FALSE;
        }
        return FALSE;
    }

    /* Deinitialize the headset detect gpio settings */
    ret_val = audio_mute_control(TRITON2_AUDIO_MUTE_DISABLE);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:Headset mute on failed");
        return FALSE;
    }

    /* Deinitialize the headset mute GPIO6 settings */
    ret_val = triton2_gpio_pin_deinit(headset_detector);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
        return FALSE;
    }
#ifdef OMAP3430MDK
/* TODO : Headset detect is not reliable on Pilot version of Zoom2 
  due to a lack of gnd connection to set T2_HDSET_DET (T2’s GPIO_2) signal low 
  for no headset case. Hacked till next revision */
  detect_val = 1;
#endif

    if (detect_val)
    {
        /* GPIO input(detect_val) is one means, headset is present */
        return TRUE;
    }
    else
    {
        /* GPIO input(detect_val) is zero means, headset is not present */
        return FALSE;
    }
}

/*-----------------------------------------------------------------------------
| Function    :S32 lcd_bklight(U8 enable_disable)
+------------------------------------------------------------------------------
| Description :	This function is used to enable/disable sublcd backlight
| Parameters  : gpio_num enable_disable -
|					TRITON2_SUB_LCD_BKLIGHT_ENABLE to enable backlight
|					TRITON2_SUB_LCD_BKLIGHT_DISABLE to disable backlight
|
| Returns     : SUCCESS,if write is successful
|               else ERROR.
|
+-----------------------------------------------------------------------------*/
static S32 lcd_bklight(U8 init, U8 sublcd_bklight_control_pin, U8 enable_disable)
{
    S32 ret_val = OMAPFLASH_SUCCESS;

    if (init == TRITON2_SUB_LCD_BKLIGHT_ENABLE)
    {
        /* initialize the gpio pin as output */
        ret_val =
            triton2_gpio_pin_init(sublcd_bklight_control_pin,
                                  TRITON2_GPIO_OUTPUT);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio init failed");
            return (ret_val);
        }
    }

    /* configure the enable_disable value to the gpio pin */
    ret_val =
            triton2_set_gpio_output(sublcd_bklight_control_pin, enable_disable);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio set output failed");

            /* deinitialize the gpio pin settings and then return */
            ret_val = triton2_gpio_pin_deinit(sublcd_bklight_control_pin);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
                return (ret_val);
            }
            return (ret_val);
    }

    return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    :S32 spi_lcd_bklight(U8 enable_disable)
+------------------------------------------------------------------------------
| Description :	This function is used to enable/disable sublcd backlight
| Parameters  : enable_disable -
|					TRITON2_SUB_LCD_BKLIGHT_ENABLE to enable backlight
|					TRITON2_SUB_LCD_BKLIGHT_DISABLE to disable backlight
|
| Returns     : SUCCESS,if write is successful
|               else ERROR.
|
+-----------------------------------------------------------------------------*/
S32 spi_lcd_bklight(U8 enable_disable)
{
    static U8 init = 1;
    S32 ret_val;
    ret_val = lcd_bklight(init, TRITON2_GPIO_7,enable_disable);
    /* Do init once only */
    if ((ret_val == OMAPFLASH_SUCCESS) && (init))
    {
        init = 0;
    }
    return ret_val;
}
/*-----------------------------------------------------------------------------
| Function    :S32 pri_lcd_bklight(U8 enable_disable)
+------------------------------------------------------------------------------
| Description :	This function is used to enable/disable sublcd backlight
| Parameters  : enable_disable -
|					TRITON2_SUB_LCD_BKLIGHT_ENABLE to enable backlight
|					TRITON2_SUB_LCD_BKLIGHT_DISABLE to disable backlight
|
| Returns     : SUCCESS,if write is successful
|               else ERROR.
|
+-----------------------------------------------------------------------------*/
S32 pri_lcd_bklight(U8 enable_disable)
{
    static U8 init = 1;
    S32 ret_val;
    ret_val = lcd_bklight(init, TRITON2_GPIO_6,enable_disable);
    /* Do init once only */
    if ((ret_val == OMAPFLASH_SUCCESS) && (init))
    {
        init = 0;
    }
    return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    :S32 audio_mute_control(U8 enable_disable)
+------------------------------------------------------------------------------
| Description :	This function is used to enable/disable audio mute feature
| Parameters  : enable_disable -
|					TRITON2_AUDIO_MUTE_ENABLE to enable the mute feature
|					TRITON2_AUDIO_MUTE_DISABLE to disable the mute feature
|
| Returns     : SUCCESS,if write is successful
|               else ERROR.
|
+-----------------------------------------------------------------------------*/
S32 audio_mute_control(U8 enable_disable)
{
    U8 audio_mute_control_pin =
#ifdef OMAP3430SDP_CHMN
    TRITON2_GPIO_15;   /* On Chameleon Platforms */
#else
    TRITON2_GPIO_6;    /* On 3430 SDP Platforms */
#endif	/* OMAP3430SDP_CHMN */
    S32 ret_val = OMAPFLASH_SUCCESS;
    if (enable_disable == TRITON2_AUDIO_MUTE_ENABLE)
    {
        /* initialize the gpio pin as output */
        ret_val =
            triton2_gpio_pin_init(audio_mute_control_pin, TRITON2_GPIO_OUTPUT);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio init failed");
            return (ret_val);
        }

        /* configure the enable_disable value to the gpio pin */
        ret_val =
            triton2_set_gpio_output(audio_mute_control_pin, enable_disable);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio set output failed");

            /* deinitialize the gpio pin settings and then return */
            ret_val = triton2_gpio_pin_deinit(audio_mute_control_pin);
            if (ret_val != OMAPFLASH_SUCCESS)
            {
                dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
                return (ret_val);
            }
            return (ret_val);
        }

    }
    else                        /* To keep high state on the GPIO6 dont deinitialize when mute is enabled - Satish */
    {
        ret_val = triton2_gpio_pin_deinit(audio_mute_control_pin);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
            return (ret_val);
        }
    }
    return ret_val;
}

/*****************************************************************************
*
* mmc_card_detect_t2- This function checks if card is inserted
*
* RETURNS: 0 - If card is not detected
*          1 - If card is detected
*
*******************************************************************************/
S32 mmc_card_detect_t2(U8 mmc_slot_num)
{
    U8 mmc_card_detector = TRITON2_GPIO_0;
    U8 detect_val;
    S32 ret_val = OMAPFLASH_SUCCESS;

    /* gpio 0 is used for slot num 1 */
    if (mmc_slot_num == 0)
    {
        mmc_card_detector = TRITON2_GPIO_0;
    }
    /* gpio 1 is used for slot num 2 */
    else if (mmc_slot_num == 1)
    {
        mmc_card_detector = TRITON2_GPIO_1;
    }
    /* initialize the gpio pin as input */
    ret_val = triton2_gpio_pin_init(mmc_card_detector, TRITON2_GPIO_INPUT);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio init failed");
        return FALSE;
    }
    /* read the gpio data */
    ret_val = triton2_get_gpio_input(mmc_card_detector, &detect_val);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio get input failed");
        /* deinitialize the gpio pin settings and then return */
        ret_val = triton2_gpio_pin_deinit(mmc_card_detector);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
            return FALSE;
        }
        return FALSE;
    }

    /* deinitialize the gpio pin settings */
    ret_val = triton2_gpio_pin_deinit(mmc_card_detector);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
        return FALSE;
    }

    if (detect_val)
    {
        /* GPIO input(detect_val) is one means, card (CD) is not present */
        return FALSE;
    }
    else
    {
        /* GPIO input(detect_val) is zero means, card (CD) is present */
        return TRUE;
    }

}                               /* end of mmc_card_detect */

//#if ( defined(OMAP3430SDP_CHMN) || defined(OMAP3430MDK) ) /* #ifdef OMAP3430SDP_CHMN */

/******************************************************************************
*
* mmc_write_protect_t2- This function checks if card is write protected or not.
*
* RETURNS: 0 - If card is write protected
*          1 - If card is not write protected.
*
*******************************************************************************/
S32 mmc_write_protect_t2(U8 mmc_slot_num)
{
	U8 mmc_write_protect = TRITON2_GPIO_17;
	U8 detect_val;
	S32 ret_val = OMAPFLASH_SUCCESS;

	if (mmc_slot_num == 0)
	{
		mmc_write_protect = TRITON2_GPIO_17;
	}
	if (mmc_slot_num == 1)
	{
		return TRUE;
	}

	/* initialize the gpio pin as input */
	ret_val = triton2_gpio_pin_init(mmc_write_protect, TRITON2_GPIO_INPUT);
	if (ret_val != OMAPFLASH_SUCCESS)
	{
		dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio init failed");
		return FALSE;
	}
	/* read the gpio data */
	ret_val = triton2_get_gpio_input(mmc_write_protect, &detect_val);
	if (ret_val != OMAPFLASH_SUCCESS)
	{
		dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio get input failed");
		/* deinitialize the gpio pin settings and then return */
		ret_val = triton2_gpio_pin_deinit(mmc_write_protect);
		if (ret_val != OMAPFLASH_SUCCESS)
		{
			dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
			return FALSE;
		}
		return FALSE;
	}

	/* deinitialize the gpio pin settings */
	ret_val = triton2_gpio_pin_deinit(mmc_write_protect);
	if (ret_val != OMAPFLASH_SUCCESS)
	{
		dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
		return FALSE;
	}

	if (detect_val)
	{
		/* GPIO input(detect_val) is one means, card is  write protected */
		return TRUE;
	}
	else
	{
		/* GPIO input(detect_val) is zero means, card  is not write protected  */
		return FALSE;
	}

}								/* end of mmc_write_protect */

//#endif


/*-----------------------------------------------------------------------------
| Function    : triton2_bootconfig()
+------------------------------------------------------------------------------
| Description : Enable the HFCLK for Internal use and for SMPS
|
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 triton2_bootconfig()
{
    S32 ret_val = 0;
    U32 length = 1;
    U8 data, addr;

    // Init the i2c interface
    U32 i2c_device_handle_local;
    T_I2C_INIT_STRUCTURE init_str;

    init_str.pid = 2; //DAL_I2C_PID;
    init_str.sid = 0;           /* 0 */
    init_str.slave_addr = T2PWR_DEVICE_ADDR;
    init_str.clock_speed = I2C_2P6M_CLK;

    //////////////////////////////////////////
    /*Initialise i2c and get the device handle */
    //////////////////////////////////////////
    ret_val = i2c_init((void *)&init_str, &i2c_device_handle_local);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        //dbg_print(DBG_LEVEL_CRITICAL, "I2C init failed");
        return ret_val;
    }

    /* Enable writing to Power Configuration registers */
    data = 0xC0;
    addr = T2PROTECT_KEY_REG;
    i2c_write(i2c_device_handle_local, I2C_OFFSET(addr), &length, &data);

    data = 0x0C;
    addr = T2PROTECT_KEY_REG;
    i2c_write(i2c_device_handle_local, I2C_OFFSET(addr), &length, &data);

    /* CFG_BOOT bit HFCLK_FREQ set to 2 for 26M crystal clk
       Slicer is disabled becasue we are giving digital clock */
    data = 0x0E;
    addr = T2CFG_BOOT_REG;
    i2c_write(i2c_device_handle_local, I2C_OFFSET(addr), &length, &data);

    if (omap_silicon_revision() >= ES_2P1)
    {
        data = T2AUDIO_DEVICE_ADDR;
        ret_val =
            i2c_write(i2c_device_handle_local, I2C_SLAVE_ADDR, &length, &data);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "I2C write failed");
            return ret_val;
        }

        /* MADC clock is disabled by ROM code in OMAP3430 ES2.1, below code enable the MADC clock (used by BCI) */
        data = 0x90;
        addr = TRITON2_GPBR1_REG;
        ret_val =
            i2c_write(i2c_device_handle_local, I2C_OFFSET(addr), &length,
                      &data);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "I2C write failed");
            return ret_val;
        }
    }

    /////////////////////
    /* deinit the i2c */
    /////////////////////
    ret_val = i2c_deinit(i2c_device_handle_local);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        return ret_val;
    }

    return ret_val;
}                               /* end of triton2_bootconfig */

/*-----------------------------------------------------------------------------
| Function    : triton2_clear_startup_interrupts()
+------------------------------------------------------------------------------
| Description : Clear the interrupts seen when the Triton2 is powered up
|
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
S32 triton2_clear_startup_interrupts()
{
    S32 ret_val = 0;
    U32 length = 1;
    U8 data, addr;

    // Init the i2c interface
    U32 i2c_device_handle_local;
    T_I2C_INIT_STRUCTURE init_str;

    init_str.pid = 2; //DAL_I2C_PID;
    init_str.sid = 0;           /* 0 */
    init_str.slave_addr = T2PWR_DEVICE_ADDR;
    init_str.clock_speed = I2C_2P6M_CLK;

    //////////////////////////////////////////
    /*Initialise i2c and get the device handle */
    //////////////////////////////////////////
    ret_val = i2c_init((void *)&init_str, &i2c_device_handle_local);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "I2C init failed");
        return ret_val;
    }

    /* Clear the interrupt by writing to the Power module ISR register  */
    data = 0xFF;
    addr = TRITON2_PWR_ISR1_REG;
    i2c_write(i2c_device_handle_local, I2C_OFFSET(addr), &length, &data);

    /////////////////////
    /* deinit the i2c */
    /////////////////////
    ret_val = i2c_deinit(i2c_device_handle_local);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        return ret_val;
    }

    return ret_val;

}/* end of triton2_clear_startup_interrupts */

/*-----------------------------------------------------------------------------
| Function    : disable_triton2_i2s_interface()
+------------------------------------------------------------------------------
| Description : Disable Triton I2S interface
|
| Parameters  : void
|
| Returns     : Status
+----------------------------------------------------------------------------*/
S32 disable_triton2_i2s_interface(void)
{
	S32 ret_status = OMAPFLASH_ERROR ;
	U32 i2c_device_handle = 0,size = 0x1;
	U8 data = 0;
	T_I2C_INIT_STRUCTURE i2c_init_str;
    i2c_init_str.pid = 2; //DAL_I2C_PID;
    i2c_init_str.sid = SID_I2C1;
    /* T2 I2C interface supported HS */
    i2c_init_str.clock_speed = I2C_2P6M_CLK;
	/* T2 Audio Slave Address */
	i2c_init_str.slave_addr = T2AUDIO_DEVICE_ADDR;
	ret_status = dal_init((void *)&i2c_init_str, &i2c_device_handle);
	if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\n T2 I2S disable - I2C init  failed");
		return OMAPFLASH_ERROR;
	}
	data = 4;
	ret_status = dal_write(i2c_device_handle,I2C_OFFSET(TRITON2_AUDIO_IF_OFFSET),&size,&data);
	if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\n T2 I2S disable - I2C Write  failed");
		return OMAPFLASH_ERROR;
	}
	ret_status = dal_deinit(i2c_device_handle);
	if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\n T2 I2S disable - I2C deinit  failedd");
		return OMAPFLASH_ERROR;
	}
	return OMAPFLASH_SUCCESS;
}

#ifdef OMAP3430MDK
#define ENABLE_T2_DAC_SEL   1
/*-----------------------------------------------------------------------------
| Function    :void route_dac_to_stereojack()
+------------------------------------------------------------------------------
| Description :This function enables routhing from AIC3254 to stereo Jack
| Parameters  : none
| Returns     : success/failure
|
|
+-----------------------------------------------------------------------------*/
S32 mux_dac_to_stereojack()
{
    U8 t2_aic3254_sel = TRITON2_GPIO_15;
    S32 ret_val = OMAPFLASH_SUCCESS;

    /* Initialize the T2 GPIO15 as o/p for driving MUX  */
    ret_val = triton2_gpio_pin_init(t2_aic3254_sel, TRITON2_GPIO_OUTPUT);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio init failed");
        return ret_val;
    }

   /* Set GPIO 15 to high */
    ret_val = triton2_set_gpio_output(t2_aic3254_sel, ENABLE_T2_DAC_SEL);
    if (ret_val != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:get gpio output failed");
        /* Deinitialize the GPIO and then return */
        ret_val = triton2_gpio_pin_deinit(t2_aic3254_sel);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:gpio deinit failed");
            return ret_val;
        }
        return ret_val;
    }
    return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    :void demux_dac_to_stereojack()
+------------------------------------------------------------------------------
| Description :this line makes t2_3254_sel line to default state.
| Parameters  : none
| Returns     : success/failure
|
|
+-----------------------------------------------------------------------------*/
S32 demux_dac_to_stereojack()
{
	/* Pull GPIO15 to LOW */
	return(triton2_gpio_pin_deinit(TRITON2_GPIO_15));
}
#endif

#ifdef HDMI_MCLK_FROM_T2_256FS
/*-----------------------------------------------------------------------------
| Function    :void triton2_clk256fs_enable()
+------------------------------------------------------------------------------
| Description :Enables 256FS clock  .
| Parameters  : none
| Returns     : success/failure
|
+-----------------------------------------------------------------------------*/
S32 triton2_clk256fs_enable(void)
{
    S32 ret_status = OMAPFLASH_ERROR ;
    U32 i2c_device_handle = 0,size = 0x1;
    U8 data = 0;
    T_I2C_INIT_STRUCTURE i2c_init_str;
    i2c_init_str.pid = DAL_I2C_PID;
    i2c_init_str.sid = SID_I2C1;
    /* T2 I2C interface supported HS */
    i2c_init_str.clock_speed = I2C_2P6M_CLK;
    /* T2 Audio Slave Address */
    i2c_init_str.slave_addr = T2AUDIO_DEVICE_ADDR;
    ret_status = dal_init((void *)&i2c_init_str, &i2c_device_handle);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\n T2 I2S disable - I2C init  failed");
        return OMAPFLASH_ERROR;
	}
    /* Power on codec  */
    data =  (0x1 << 1) | (0xA << 4) ; /* Power on CODEC ; Fs = 48 KHz , o/p = 256 * 48 KHz */
    ret_status = dal_write(i2c_device_handle,I2C_OFFSET(TRITON2_CODEC_MODE_OFFSET),&size,&data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\nTriton2:codec init failed");
    }
    /* Enable APLL */
    data = 0x16; /* Input clock freq - 26Mhz and APLL is enabled */
    ret_status = dal_write(i2c_device_handle,I2C_OFFSET(TRITON2_APLL_CTL_OFFSET),&size,&data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\n T2 I2S disable - I2C init  failed");
        return OMAPFLASH_ERROR;
    }
    /* Enable  CLK256FS */
    data = 0x87;
    ret_status = dal_write(i2c_device_handle,I2C_OFFSET(TRITON2_AUDIO_IF_OFFSET),&size,&data);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\n T2 I2S disable - I2C init  failed");
        return OMAPFLASH_ERROR;
    }
    ret_status = dal_deinit(i2c_device_handle);
    if (ret_status != OMAPFLASH_SUCCESS)
    {
        dbg_print(DBG_LEVEL_CRITICAL, "\r\n T2 I2S disable - I2C deinit  failedd");
        return OMAPFLASH_ERROR;
    }
    return OMAPFLASH_SUCCESS;
}
#endif    /* HDMI_MCLK_FROM_T2_256FS */


/*==== END OF FILE ===========================================================*/
