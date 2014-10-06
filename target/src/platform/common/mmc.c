/**
* @file  mmc.c
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
* File for mmc driver functions based on the MMC spec 4.2
* 
*/

/*==== DECLARATION CONTROL ===================================================*/
#ifndef MMC_C
#define MMC_C
#endif

///@todo move mmc.h private stuff to seperate file
#define MMC_C_PRIVATE			/*for mmc specific info */

#if 0
#define MMC_DEBUG_SIGNAL(reg_id, state) (void)drv_debug_signal(reg_id, state)
#else
#define MMC_DEBUG_SIGNAL(reg_id, state) ((void)0)
#endif

/*=======INCLUDES=============================================================*/
//#include <memory.h>
#include "types.h"
#include "error.h"
#include "csst_tgt.h"
#include "silicon.h"

#include "sparse_format.h"
#include "emmc_drv.h"
#include "flash_drv.h"
#include "mmc.h"


/*==== MACROS ================================================================*/

#define MMC1_WP                      4	/* 8 */
#define MMC2_WP                      7	/* 5 */

#define PBIAS_MAX_RETRY 3

#define MMC_HIGHDENSITY_MASK         (3 << 29)
#define MMC_HIGHDENSITY_EN           (2 << 29)
#define MMC_HIGHSPEED_BIT_EN         (0x1)

#define RESET_TIMOUT ONE_SECOND

/// The value of DEFAUL_DELAY_COUNT was experimentally determined (smallest value with predictable success seems to be 8)
#define DEFAULT_DELAY_COUNT_EXPONENT                    (8 + 1) ///< +1 to give safty slack

//#define ENABLE_TRACE

#ifdef ENABLE_TRACE
#define DBG_PRINTF dbg_printf
#else
#define DBG_PRINTF dummy_printf
#endif


/*==== CONSTS ================================================================*/

///@todo make non global variable: write_protected_card, mmc_high_density_card 
#ifndef RELOCATEABLE
static U32 write_protected_card = 0;	/* to identify the write protected cards */
static U8 mmc_high_density_card = 0;	/*flag used for the High density cards */
#else
#define write_protected_card    get_driver_data()->local_dis.write_protected_card  /* to identify the write protected cards */
#define mmc_high_density_card   get_driver_data()->local_dis.mmc_high_density_card /*flag used for the High density cards */
#endif

void dummy_printf(char * format, ...) { return; }


///@todo expand inline
void dl_lazy_delay(unsigned long time_delay_ms)
{
  wait_microsec(time_delay_ms);
}

/**
* data used to wait for the bit in an mmc register to read 1
*/
typedef struct T_mmchs_fgetw_r_data
{
  U16 mmc_slot;
  U16 reg;
  U32 mask;
  U32 val;
} T_mmchs_fgetw_r_data;

/**
* callback used to wait for the bit in an mmc register to read 1
*/
U32 mmchs_fgetw_r_callback(U32 time_left_microsec, void *d)
{
  T_mmchs_fgetw_r_data *data = (T_mmchs_fgetw_r_data*)d;
  return ((MMCHS_GET_R(data->mmc_slot, data->reg) & data->mask) != data->val) ? 0 : time_left_microsec;
}

/// first check the value to avoid any initial overhead in mmchs_fwaitset_r
#define MMCHS_FWAITSET_R(mmc_slot,reg,val,timeout_microsec) \
  ((MMCHS_GET_R(mmc_slot, reg) & val) == val ? timeout_microsec : mmchs_fwaitset_r(mmc_slot,reg,val,timeout_microsec))

/**
* wait for the bit(s) in an mmc register to read 1
* @return time left in microsec
*/
U32 mmchs_fwaitset_r(U16 mmc_slot, U16 reg, U32 val, U32 timeout_microsec)
{
  /// first check the value to avoid any initial overhead in dl_lazy_delay
  if ((MMCHS_GET_R(mmc_slot, reg) & val) == val)
  {
    return timeout_microsec;
  }
  else 
  {
    T_mmchs_fgetw_r_data data;

    data.mmc_slot = mmc_slot;
    data.reg = reg;
    data.mask = val;
    data.val = val;

    return wait_microsec_ex(timeout_microsec, mmchs_fgetw_r_callback, &data);
  }
}

#define MMCHS_FWAITCLR_R(mmc_slot,reg,val,timeout_microsec) \
  ((MMCHS_GET_R(mmc_slot, reg) & val) == 0 ? timeout_microsec : mmchs_fwaitclr_r(mmc_slot, reg, val, timeout_microsec))

/**
* wait for the bit(s) in an mmc register to read 0
* @return time left in microsec
*/
U32 mmchs_fwaitclr_r(U16 mmc_slot, U16 reg, U32 val, U32 timeout_microsec)
{
  /// first check the value to avoid any initial overhead in dl_lazy_delay
  if ((MMCHS_GET_R(mmc_slot, reg) & val) == 0)
  {
    return timeout_microsec;
  }
  else 
  {
    T_mmchs_fgetw_r_data data;

    data.mmc_slot = mmc_slot;
    data.reg = reg;
    data.mask = val;
    data.val = 0;

    return wait_microsec_ex(timeout_microsec, mmchs_fgetw_r_callback, &data);
  }
}

/**
* set the bit of an mmc regist to 1 and wait for it to read back 1 too
* @return time left in microsec
*/
U32 mmchs_fsetwaitset_r(U16 mmc_slot, U16 reg, U32 val, U32 timeout_microsec)
{
  MMCHS_FSET_R(mmc_slot, reg, val);
  return MMCHS_FWAITSET_R(mmc_slot, reg, val, timeout_microsec);
}

/**
* set the bit of an mmc regist to 1 and wait for it to read back 0 too
* @return time left in microsec
*/
U32 mmchs_fsetwaitclr_r(U16 mmc_slot, U16 reg, U32 val, U32 timeout_microsec)
{
  MMCHS_FSET_R(mmc_slot, reg, val);
  return MMCHS_FWAITCLR_R(mmc_slot, reg, val, timeout_microsec);
}

/*-----------------------------------------------------------------------------
| Function    : mmc_config
+------------------------------------------------------------------------------
|
| Description : intialize the omap mmc controller and the mmc card
|
| Parameters  : U16 sid, U16 mmc_volt,U16 *rca, U8 *type_card,
|               U8* data_width_max, U32 *trans_clk_max, U32 *cardsize
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_config(U16 sid, U16 mmc_volt, U8 data_width, U16 * rca, U8 * type_card, U8 * data_width_max, U32 * trans_clk_max, U64 * cardsize)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U8 mmc_slot;				/* hold Slot number info from SID info */
  U16 card_rca;				/* Hold the relative card address */
  U8 card_type;				/* get the card type and update the type_card */
  U8 max_data_width;			/* Maximum data with that card and host can communicate */
  U32 max_trans_clk;			/* Maximum clk freq that card can work */
  U64 size_of_card = 0;

#ifdef OMAP3
#ifdef RELOCATEABLE
#define CONTROL_PBIAS_LITE_FLAG get_driver_data()->control_pbias_lite_flag
#else
  static U32 control_pbias_lite_flag = 0;	/*For 52Mhz enable for only once */
#endif
#endif

  /*get the mmc slot number from the sid */
  mmc_slot = (U8)sid;

  /* Check the mmc card is present unless emmc slot */

  //if (control_pbias_lite_flag == 0)
  //if (get_driver_data()->control_pbias_lite_flag == 0)
  ///@todo is this needed for OMAP4430?
#ifdef OMAP3
  if (CONTROL_PBIAS_LITE_FLAG == 0)
  {
    out_regl(CONTROL_PBIAS_LITE,
      (in_regl(CONTROL_PBIAS_LITE) | PBIAS_LITE_VMMC1_52MHZ));
    CONTROL_PBIAS_LITE_FLAG = 1;
  }
#endif


  if(mmc_slot != MMC_SLOT_2)
  {
    *(volatile U32 *) CONTROL_PBIAS_LITE = 0x0C400000;
  }

  /* Internal clock selection for MMC1 and MMC2 */
#ifdef OMAP3
  out_regl(DEV_CONFIG_1, (in_regl(DEV_CONFIG_1) | 0x00000040));
  out_regl(DEV_CONFIG_0, (in_regl(DEV_CONFIG_0) | 0x01000000));
#endif

  /*pinmux setup */
#ifdef OMAP3
  mux_setup_mmcsd();
#endif
#if defined OMAP4 || defined OMAP5
  HAL_CTRL_ConfigurePads(HAL_MODULE_MMC, mmc_slot - 1);  
  HAL_CM_EnableModuleClocks(HAL_MODULE_MMC, mmc_slot - 1);                     
#endif 

  write_protected_card = 0;	/*No write protect */
  mmc_high_density_card = 0;	/*No high density support */

  /*initializes the OMAP MMC controller */
  ret_value = mmc_controller_config(mmc_slot, mmc_volt);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    /*mmc controller configuration fail */
    drv_send_info("mmc controller initialzation failed %#02X", ret_value);
    return ret_value;
  }


#ifdef OMAP3
  //#ifdef OMAP3430MDK  
  //		if(mmc_slot == MMC_SLOT_1)
  if (omap3430mdk() && mmc_slot == MMC_SLOT_1)  
  {			
    *(volatile U32 *)CONTROL_PBIAS_LITE |= MMC_PWR_STABLE;
  }
  //#endif
#endif

  /*initializes the MMC or SD card and put it in the STAND-BY-STATE */
  ret_value = sd_mmc_card_init(mmc_slot, data_width, &card_type, &card_rca, &max_data_width, &max_trans_clk, &size_of_card);

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    /*Card initialzation fail */
    drv_send_info("card initialzation failed %#02X", ret_value);
    return ret_value;
  }

  *rca = card_rca;
  *type_card = card_type;
  *data_width_max = max_data_width;
  *trans_clk_max = max_trans_clk;
  *cardsize = size_of_card;

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_controller_config
+------------------------------------------------------------------------------
|
| Description : initializes the OMAP MMC controller.
|
| Parameters  : U8 mmc_slot, U16 mmc_volt
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_controller_config(U8 mmc_slot, U16 mmc_volt)
{
  /* Software reset of the MMC/SD/SDIO host controller */
  U32 ret_value = mmc_controller_reset(mmc_slot);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    /*controller reset failed. */
    return ret_value;
  }

#ifdef OMAP3
  /* Check for debounce stable */
  if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DBOUNCE, ONE_SECOND))
  {
    /* CLK Unstable */
		dbg_printf("MMC PSTAT Debounce stable failed.");
    return OMAPFLASH_DAL_ERROR;
  }
#endif //OMAP3

  /* Set module’s hardware capabilitiesc */
  if (MMC_VOLT_1_8 == mmc_volt)
  {
    MMCHS_FSET_R(mmc_slot, MMCHS_CAPA, MMCHS_1_8V);
  }
  else if (MMC_VOLT_3_0 == mmc_volt)
  {
    MMCHS_FSET_R(mmc_slot, MMCHS_CAPA, (MMCHS_1_8V | MMCHS_3_0V));
  }

  /* Write MMCi.MMCHS_HCTL register (SDVS, SDBP, DTW) to configure the card
  voltage value and power mode and dat bus width */
  if (MMC_VOLT_1_8 == mmc_volt)
  {
    MMCHS_FSET_R(mmc_slot, MMCHS_HCTL, MMCHS_SDVS_1_8V);
  }
  else if (MMC_VOLT_3_0 == mmc_volt)
  {
    MMCHS_FSET_R(mmc_slot, MMCHS_HCTL, MMCHS_SDVS_3_0V);
  }

  /// power up the card 
  if (!mmchs_fsetwaitset_r(mmc_slot, MMCHS_HCTL, MMCHS_HCTL_SDBP, ONE_SECOND))
  {
		dbg_printf("MMC Power up fail message.");
    return OMAPFLASH_DAL_ERROR;
  }
  MMCHS_FSET_R(mmc_slot, MMCHS_SYSC, SYSC_AUTOIDLE);	
  MMCHS_FSET_R(mmc_slot, MMCHS_SYSC, SYSC_CLKACT_OCP_FCLK);	
  MMCHS_FSET_R(mmc_slot, MMCHS_SYSC, SYSC_CLKACT_NOSTDBY);	

  MMCHS_FSET_R(mmc_slot, MMCHS_SYSCTL, MMCHS_SYSCTL_ICE);	///< enable internal clock
  MMCHS_ASET_R(mmc_slot, MMCHS_SYSCTL, MMCHS_SYSCTL_CLK_DIS);	/*clear the clock freq field */
  MMCHS_FSET_R(mmc_slot, MMCHS_SYSCTL, 0xF000);	/*selct clk freq (96Mhz/960 = 100Khz) */

  /// wait for clk freq stable
  if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_SYSCTL, MMCHS_SYSCTL_ICS, ONE_SECOND))
  {
    /* CLK Unstable */
		dbg_printf("MMC clk freq stable failed.");
    return OMAPFLASH_DAL_ERROR;
  }

  MMCHS_FSET_R(mmc_slot, MMCHS_SYSCTL, MMCHS_SYSCTL_CEN);

  return OMAPFLASH_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_controller_reset
+------------------------------------------------------------------------------
|
| Description : Resetting the OMAP MMC controller
|
| Parameters  : void
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_controller_reset(U8 mmc_slot)
{
  /*give soft reset to the controller */
  MMCHS_FSET_R(mmc_slot, MMCHS_SYSC, SYSC_SOFTRESET);

  /// check reset state
  if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_SYSS, MMCHS_SYSS_RESETDONE, ONE_SECOND))
  {
		dbg_printf("MMC Controller reset stable failed.");
    return OMAPFLASH_DAL_ERROR;
  }

  if (reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRA) == OMAPFLASH_DAL_ERROR)
  {
    /*terminate the loop if SRA is not cleared */
		dbg_printf("Controller soft reset all failed..\r\n");
    return OMAPFLASH_DAL_ERROR;		/*Send the soft reset success */
  }

  return OMAPFLASH_SUCCESS;			/*Send the soft reset success */
}

/*-----------------------------------------------------------------------------
| Function    : set_mmc_sdvs_voltage
+------------------------------------------------------------------------------
|
| Description : sets the controller capa and SDVS as per the setup voltage
|
| Parameters  : U8 mmc_slot, U16 mmc_volt
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 set_mmc_sdvs_voltage(U8 mmc_slot, U16 mmc_volt)
{
  U32 ret_value = OMAPFLASH_SUCCESS;

  if (MMC_VOLT_1_8 == mmc_volt)
  {
    MMCHS_FSET_R(mmc_slot, MMCHS_CAPA, MMCHS_1_8V);
    MMCHS_FSET_R(mmc_slot, MMCHS_HCTL, MMCHS_SDVS_1_8V);
  }
  else if (MMC_VOLT_3_0 == mmc_volt)
  {
    MMCHS_FSET_R(mmc_slot, MMCHS_CAPA, (MMCHS_1_8V | MMCHS_3_0V));
    MMCHS_FSET_R(mmc_slot, MMCHS_HCTL, MMCHS_SDVS_3_0V);
  }
  else
  {
    /*sent the no voltage support */
    return OMAPFLASH_DAL_ERROR;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : set_mmc_controller_clk
+------------------------------------------------------------------------------
|
| Description : sets the clock divisior as per the given input clock
|
| Parameters  : U32 mmc_clk
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 set_mmc_controller_clk(U8 mmc_slot, U32 mmc_clk)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 divisor = 0;
  U32 temp_data;

  MMCHS_ASET_R(mmc_slot, MMCHS_SYSCTL, (~MMCHS_SYSCTL_CEN));	/*Disable the Clock */
  temp_data = MMCHS_GET_R(mmc_slot, MMCHS_SYSCTL);
  temp_data &= MMCHS_SYSCTL_CLK_DIS;
  divisor = 96000 / mmc_clk;	/* 96 Meg clk */
  divisor = divisor << 6;
  temp_data |= (divisor);
  MMCHS_SET_R(mmc_slot, MMCHS_SYSCTL, temp_data);	/*selct clk freq */

  /// wait for clk freq stable
  if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_SYSCTL, MMCHS_SYSCTL_ICS, ONE_SECOND))
  {
    /* CLK Unstable */
		dbg_printf("MMC clk freq stable failed.");
    return OMAPFLASH_DAL_ERROR;
  }

  /// enable clk to card
  if (!mmchs_fsetwaitset_r(mmc_slot, MMCHS_SYSCTL, MMCHS_SYSCTL_CEN, ONE_SECOND))
  {
    /* CLK Unstable */
		dbg_printf("MMC enable clk to card failed.");
    return OMAPFLASH_DAL_ERROR;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : sd_mmc_card_init
+------------------------------------------------------------------------------
|
| Description : Initialize the MMC or SD card and put it in the STAND-BY-STATE.
|
| Parameters  : U8 mmc_slot, U8 *card_type, U16 *card_rca,
|               U8 *max_data_width, U32 *max_trans_clk
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 sd_mmc_card_init(U8 mmc_slot, U8 data_width, U8 * card_type, U16 * card_rca, U8 * max_data_width, U32 * max_trans_clk, U64 * size_of_card)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 bootup = 0;
  U32 mmc_init_timeout = get_driver_data()->init_timeout;
  U32 cmd01delay = get_driver_data()->cmd01delay * 1000;
  U8 mmc_sd_card;
  U16 rca;
  U32 ocr_val = (get_driver_data()->hd) ? 0x40FF8080 : 0x00FF8080;
  U32 high_speed = 3;
  U32 *buf = (U32*) get_driver_data()->block_data;
  BOOLEAN enable_trace = TRUE;

  /* Send the Init sequence to the card for 80 clk cycles */

  //ret_value = send_mmc_init_sequence(mmc_slot);

  //if (OMAPFLASH_SUCCESS != ret_value)
  //{
  //  return ret_value;
  //}

  /*Set the Clk with 400Khz (MAX Opendrain CLK) */

  ret_value = set_mmc_controller_clk(mmc_slot, 400);

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  /* Detection of MMC or SD card */

  get_card_type(mmc_slot, &mmc_sd_card, &rca);

  /*Card type: MMC/HSMMC/SD */

  *card_type = mmc_sd_card;

  /*Card relative address for future accessing of the card */

  *card_rca = rca;

  /*CMD flow as per the MMC spec... */

  if (MMC_CARD == mmc_sd_card)
  {
    /* put the card to idle state -> send CMD0 */

    ret_value = send_mmc_cmd(mmc_slot, MMCHS_GO_IDLE_STATE, 0, 0);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      drv_send_info("CMD0 failed");
      return ret_value;
    }


    /* if specified - wait a bit between CMD0 and CMD1 */

    if(cmd01delay > 0)
    {
      dbg_printf("CMD0 to CMD1 delay active");
      wait_microsec(cmd01delay);
    }
    
    /* put the card to ready state */

    check_timeout_ms(0, 0);

    do
    {
      /* Go to ready state (CMD1) wait until bit 15 of RSP7 is reset */

      ret_value =	send_mmc_cmd_ex(mmc_slot, MMCHS_SEND_OP_COND, (U16)((ocr_val >> 16) & MMC_STUFF_BITS), ((U16) (ocr_val & MMC_STUFF_BITS)), &enable_trace);

      if (OMAPFLASH_SUCCESS == ret_value)
      {
        bootup = MMCHS_GET_R(mmc_slot, MMCHS_RSP10);
        ret_value = bootup;
        bootup = bootup >> 31;
        bootup &= 1;
      }
    }while ((bootup == 0) && (check_timeout_ms(0, mmc_init_timeout) != TRUE));  

    /*Card is not in Ready */

    if (bootup == 0)
    {
      drv_send_info("CMD1 failed");
      return ret_value;
    }
    else	/*Card is ready */
    {
      /* check the OCR[30,29] bits for High density support */

      if ((ret_value & MMC_HIGHDENSITY_MASK) == MMC_HIGHDENSITY_EN)
      {
        /* high density support enabled */
        mmc_high_density_card = 1;
      }

      ret_value = OMAPFLASH_SUCCESS;
    }

    /*put the card to identification state */

    ret_value = send_mmc_cmd(mmc_slot, MMCHS_ALL_SEND_CID, MMCHS_STUFF_BITS, MMCHS_STUFF_BITS);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      drv_send_info("SEND_CID failed");
      return ret_value;
    }

    /*put the card to stand-by state */

    ret_value =	send_mmc_cmd(mmc_slot, MMCHS_SET_RELATIVE_ADDR, rca, MMCHS_STUFF_BITS);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      drv_send_info("SET_RELATIVE_ADDR failed");
      return ret_value;
    }

    /* Read the CSD and get the card properties */
    ret_value = get_card_details(mmc_slot, *card_rca, card_type, max_data_width, max_trans_clk, size_of_card);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      drv_send_info("READ CSD failed");
      return ret_value;
    }

    /* Disable the OpenDrain Operation */
    ret_value = mmc_opendrain_enable_disable(mmc_slot, MMC_OPENDRAIN_DISABLE);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

#if 1 ///@todo make mmc_buswidth_detect work
    /* For High speed MMC card need to find supported data width and speed */
    if (HS_MMC_CARD == *card_type)
    {
#if 0 ///@todo make mmc_buswidth_detect work
      ret_value = mmc_buswidth_detect(*card_rca, max_data_width);
      if (OMAPFLASH_SUCCESS != ret_value)
      {
        /*MMC card is not inserted */
        return ret_value;
      }
#endif
      get_driver_data()->block_id = NO_MMC_BLOCK_ID; ///<what ever was ther before will be over written by EXT CSD 

      ret_value = mmc_read_ext_csd(mmc_slot, data_width, *card_rca, *card_type, (U8 *) buf);

      if (OMAPFLASH_SUCCESS != ret_value)
      {
        /*MMC card is not inserted */
        drv_send_info("Read Ext_CSD failed");
        return ret_value;
      }

      /*High speed bit enabled or not */
      high_speed = buf[49] & MMC_HIGHSPEED_BIT_EN;

      if (high_speed == MMC_HIGHSPEED_BIT_EN)
      {
        /*High speed bit enabled, can support 52MHz,
        *by considering the OMAP divisor can set upto 48MHz*/
        *max_trans_clk = 48000;	/*clk in KHz */
      }
      else
      {
        /*High speed bit Disabled, can support 26MHz, */
        *max_trans_clk = 26000;	/*clk in KHz */
      }

      /* High Density card Size */

      if (mmc_high_density_card == 1)
      {
        /*high density card size read from the EXT_CSD[215-212] */
        *size_of_card = ((U64)buf[53] * 512);
      }
    }
#endif
  }
  else if (SD_CARD == mmc_sd_card)
  {
    ret_value =	send_mmc_cmd(mmc_slot, MMCHS_GO_IDLE_STATE, 0, 0);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    check_timeout_ms(0, 0);  
    do
    {
      /* Send SD application command - needed for SD */
      ret_value = send_mmc_cmd(mmc_slot, SDHS_APP_CMD, MMCHS_STUFF_BITS, MMCHS_STUFF_BITS);

      if (OMAPFLASH_SUCCESS == ret_value)
      {
        /* Go to ready state, wait until bit 15 of RSP7 is reset */
        ret_value =	send_mmc_cmd(mmc_slot, SDHS_APP_OP_COND, (U16) ((MMC_HIGH_VOLTAGE_CARD >> 16) & MMC_STUFF_BITS), ((U16) (MMC_HIGH_VOLTAGE_CARD & MMC_STUFF_BITS)));

        if (OMAPFLASH_SUCCESS == ret_value)
        {
          bootup = MMCHS_GET_R(mmc_slot, MMCHS_RSP10);
          ret_value = bootup;
          bootup = bootup >> 31;
          bootup &= 1;
        }
      }
    }while ((bootup == 0) &&  (check_timeout_ms(0, mmc_init_timeout) != TRUE));  

    if (bootup == 0)		/* maximum number of tries exceeded */
    {
      return ret_value;
    }
    else
    {
      ret_value = OMAPFLASH_SUCCESS;
    }

    /* Go to identification state (CMD2) */

    ret_value = send_mmc_cmd(mmc_slot, SDHS_ALL_SEND_CID, MMC_STUFF_BITS, MMC_STUFF_BITS);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    /* Go to stand-by-state (CMD3) */
    ret_value = send_mmc_cmd(mmc_slot, SDHS_SEND_RELATIVE_ADDR, MMC_STUFF_BITS, MMC_STUFF_BITS);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    *card_rca = MMCHS_GET_R(mmc_slot, MMCHS_RSP10) >> 16;

    get_card_details(mmc_slot, *card_rca, card_type, max_data_width, max_trans_clk, size_of_card);
  }
  else
  {
    return OMAPFLASH_DAL_ERROR;
  }
  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : get_card_type
+------------------------------------------------------------------------------
|
| Description : finds type of the card inserted in the slot.
|
| Parameters  : U8 mmc_slot,U8 *card_type, U16 *card_rca
|
| Returns     : void
|
+-----------------------------------------------------------------------------*/
void get_card_type(U8 mmc_slot, U8 * card_type, U16 * card_rca)
{
  U32 ret_value = OMAPFLASH_SUCCESS;

  /* Send an CMD55 command */
  ret_value = send_mmc_cmd(mmc_slot, SDHS_APP_CMD, MMCHS_STUFF_BITS, MMCHS_STUFF_BITS);
  if (ret_value != OMAPFLASH_SUCCESS)
  {
    if (reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRC) == OMAPFLASH_DAL_ERROR)
    {
      /*terminate the loop if SRC is not cleared */
			dbg_printf("soft reset command line failed..\r\n");
    }

    /*MMC Card is present */
    *card_type = MMC_CARD;
    *card_rca = 4;

    /*For MMC cards need to enable the Open drain */
    mmc_opendrain_enable_disable(mmc_slot, MMC_OPENDRAIN_ENABLE);
  }
  else
  {
    if ((MMCHS_GET_R(mmc_slot, MMCHS_RSP10) & 0x0000FFFF) == 0x0120)	/*supports app cmd ? */
    {
      //SD Card is present
      *card_type = SD_CARD;
    }
    else
    {
      if (reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRC) == OMAPFLASH_DAL_ERROR)
      {
        /*terminate the loop if SRC is not cleared */
				dbg_printf("soft reset command line failed..\r\n");
      }

      /*MMC Card is present */
      *card_type = MMC_CARD;
      *card_rca = 4;

      /*For MMC cards need to enable the Open drain */
      mmc_opendrain_enable_disable(mmc_slot, MMC_OPENDRAIN_ENABLE);
    }
  }
}

/*-----------------------------------------------------------------------------
| Function    : get_card_details
+------------------------------------------------------------------------------
|
| Description : finds type of the card inserted in the slot.
|
| Parameters  : U8 mmc_slot,U16 card_rca,U8 *card_type,
|               U8 *max_data_width,U32 *max_trans_clk
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32
  get_card_details(U8 mmc_slot, U16 card_rca, U8 * card_type,
  U8 * max_data_width, U32 * max_trans_clk, U64 * size_of_card)
{
  T_MMC_CSD csd;
  U16 len = sizeof(T_MMC_CSD);
  U16 sid = mmc_slot;
  U16 mmc_ver;
  U32 *buf = (U32*) get_driver_data()->block_data;
  U32 ret_value = OMAPFLASH_SUCCESS;

  /* check CSD data */
  ret_value = get_sd_mmc_csd(sid, card_rca, *card_type, len, (U32 *) & csd);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  /* check the mmc_version >= 4 */
  if (MMC_CARD == *card_type)
  {
    mmc_ver = csd.mmc_prot;
    if (mmc_ver >= 4)		/*HS MMC check */
    {
      *card_type = HS_MMC_CARD;
    }
  }

  /* Need verify the EXT_CSD data */
  if (HS_MMC_CARD == *card_type)
  {
    /* default selecting 8-bit and 48MHz and
    later these values can changed as per card's
    EXT_CSD and buswidth detection */
    *max_data_width = MMC_DATAWIDTH_8_BITS;
    *max_trans_clk = 48000;	/*clk in the KHz */
  }
  else
  {
    /*Default support-> 1-bit and 20MHz */
    *max_data_width = MMC_DATAWIDTH_1_BITS;
    *max_trans_clk = 20000;	/*clk in the KHz */
  }

  /*card registers indicating write protect enabled or not */
  if ((csd.tmp_write_protect) || (csd.tmp_write_protect))
  {
    /*write protect enabled for the card */
    write_protected_card = 1;
  }


  /* High Density cards (>2GB) capcaity = SEC_COUNT (from Ext_CSD) * 512B
     Low Denisty cards memory capacity = BLOCKNR * BLOCK_LEN, where
            BLOCKNR = (C_SIZE+1) * MULT
            MULT = 2 ^ (C_SIZE_MULT+2) ; (C_SIZE_MULT < 8)
            BLOCK_LEN = 2 ^ READ_BL_LEN ; (READ_BL_LEN < 12)
   */
  if (mmc_high_density_card == 0)
  {
    // high density support not enabled,
    U32 mult = 1 << (csd.csize_mult + 2);
    U32 blk_len = 1 << csd.read_blk_len;
    *size_of_card = ((U64)(csd.csize + 1) * mult * blk_len);
  }
  else
  {
    //*size_of_card = buf[53];
    *size_of_card = 0;
  }

  return OMAPFLASH_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : send_mmc_cmd
+------------------------------------------------------------------------------
|
| Description : sends a command to the card.
|
| Parameters  : U32 cmd, U16 argh, U16 argl
|
| Returns     : S32
|
+-----------------------------------------------------------------------------*/
S32 send_mmc_cmd(U8 mmc_slot, U32 cmd, U16 argh, U16 argl)
{
  BOOLEAN enable_trace = TRUE;
  return send_mmc_cmd_ex(mmc_slot, cmd, argh, argl, &enable_trace);
}

S32 send_mmc_cmd_ex(U8 mmc_slot, U32 cmd, U16 argh, U16 argl, BOOLEAN *enable_trace)
{
  U32 status;
  U32 arg;
  U32 mmc_timeout = get_driver_data()->timeout;               // in msec
  U32 mmc_timeout_tick = get_driver_data()->timeout_tick;     // in msec
  U8 c = (cmd & MMCHS_CMD63) >> 24; 

  // dbg_printf("Sending Command: Slot %d Cmd %#02x Arg %#08x", mmc_slot, c, ((argh << 16) + argl));
  
  /* clear the MMCSTAT register of its previous value */
  MMCHS_ASET_R(mmc_slot, MMCHS_STAT, 0xFFFFFFFF);

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_CMDI, ONE_SECOND))
  {
    drv_send_info("CMD %#02x: %#02x, MMCHS_PSTAT not out from the CMDI state!", c, cmd);
    return (S32)OMAPFLASH_ERROR;
  }
  if(cmd == MMCHS_GO_IRQ_STATE)
    MMCHS_FSET_R(mmc_slot, MMCHS_CON, (MMCHS_CON_MIT));
  else
    MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~(MMCHS_CON_MIT));

  if(cmd == MMCHS_READ_DAT_UNTIL_STOP || cmd == MMCHS_WRITE_DAT_UNTIL_STOP)
    MMCHS_FSET_R(mmc_slot, MMCHS_CON, (MMCHS_CON_STR));
  else
    MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~(MMCHS_CON_STR));

  if(cmd != MMCHS_WRITE_MULTIPLE_BLOCK && cmd != MMCHS_READ_MULTIPLE_BLOCK && 
    cmd != MMCHS_WRITE_MULTIPLE_BLOCK_ADMA && cmd != MMCHS_READ_MULTIPLE_BLOCK_ADMA)
  {
    MMCHS_SET_R(mmc_slot, MMCHS_BLK, (MMCHS_NBLK_ONE | MMCHS_BLKSZ_512));
  }

  MMCHS_FSET_R(mmc_slot, MMCHS_SYSCTL, (MMCHS_DTO_MAX));

  /*generate a single arg */
  arg = argh;
  arg = (arg << 16);
  arg = (arg | argl);
  /*wait for CMD line idle */

  MMCHS_SET_R(mmc_slot, MMCHS_IE, 0xFFFFFEFF);	/* IE status enable */
  MMCHS_SET_R(mmc_slot, MMCHS_ISE, 0x00);	        /* ISE  */

  MMCHS_SET_R(mmc_slot, MMCHS_ARG, arg);	/*send the argumant value */

  MMCHS_SET_R(mmc_slot, MMCHS_CMD, cmd);	/*send the command data */

  check_timeout_ms(1, 0);
  /* Wait for response done */
  do
  {
    /* Get status */
    status = MMCHS_GET_R(mmc_slot, MMCHS_STAT);

    /* Status is Eod of command phase - OK */
    if ((((status & MMCHS_MMCSTAT_CCRC) == MMCHS_MMCSTAT_CCRC) &&
      ((status & MMCHS_MMCSTAT_CTO) == MMCHS_MMCSTAT_CTO)))
    {
      if (reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRC) == OMAPFLASH_DAL_ERROR)
      {
        /*terminate the loop if SRC is not cleared */
        //if (enable_trace)
                dbg_printf("soft reset command line failed..");
      }
      break;
    }
    else if ((status & MMCHS_MMCSTAT_CTO) == MMCHS_MMCSTAT_CTO)
    {
      if (reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRC) == OMAPFLASH_DAL_ERROR)
      {
        /*terminate the loop if SRC is not cleared */
        //if (enable_trace)
                dbg_printf("soft reset command line failed..");
      }

      break;
    }
    else if ((status & MMCHS_MMCSTAT_CC) == MMCHS_MMCSTAT_CC)
    {
      break;
    }
    wait_microsec(mmc_timeout_tick * 1000);  // mmc_timeout_tick is in msec
  }
  while(check_timeout_ms(1, mmc_timeout) != TRUE);

  status = MMCHS_GET_R(mmc_slot, MMCHS_STAT);

  if ((status & MMCHS_MMCSTAT_CCRC) != 0)
  {
    drv_send_info("MMCSTAT_CRC error happened during cmd %#x", cmd);
    return MMC_CMD_FAIL;
  }

  if ((status & MMCHS_MMCSTAT_DCRC) != 0)
  {
    drv_send_info("MMCSTAT_DCRC error happened during cmd %#x", cmd);
    return MMC_CMD_FAIL;
  }

  if ((status & MMCHS_MMCSTAT_CERR) != 0)
  {
    drv_send_info("MMCSTAT_CERR occured status: %#02x, rsp: %#02x", status, (MMCHS_GET_R(mmc_slot, MMCHS_RSP10)));
    return MMC_CMD_FAIL;
  }

  if ((status & MMCHS_MMCSTAT_CTO) != 0)
  {
    if (*enable_trace) 
    {
      if (cmd == SDHS_APP_CMD)
      {
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#0x, MMCHS_MMCSTAT_CTO!", c, cmd, status, arg);
      }
      else
      {
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#0x, MMCHS_MMCSTAT_CTO!", c, cmd, status, arg);
      }
      *enable_trace = FALSE;
    }

    return MMC_CMD_TO_ER;
  }
  else 
  { 
    *enable_trace = TRUE;
    if ((status & MMCHS_MMCSTAT_CC) != 0)
    {
      U32 rsp10;
      U32 rsp32;
      U32 rsp54;
      U32 rsp76;
      char *rsp = NULL;
      //if (enable_trace)
      switch (cmd & MMCHS_CMD63) 
      {
      case MMCHS_CMD0:
      case MMCHS_CMD4:
      case MMCHS_CMD15:
        //no RSP
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#0x", c, cmd, status, arg);
        break;
      case MMCHS_CMD21:
      case MMCHS_CMD22:
      case MMCHS_CMD31:
      case MMCHS_CMD41: //IO?
      case MMCHS_CMD43: //lock?
      case MMCHS_CMD44: //lock?
      case MMCHS_CMD45: //lock?
      case MMCHS_CMD46: //lock?
      case MMCHS_CMD47: //lock?
      case MMCHS_CMD48: //lock?
      case MMCHS_CMD49: //lock?
      case MMCHS_CMD50: //lock?
      case MMCHS_CMD51: //lock?
      case MMCHS_CMD52: //lock?
      case MMCHS_CMD53: //lock?
      case MMCHS_CMD54: //lock?
      case MMCHS_CMD57: //app?
      case MMCHS_CMD58: //app?
      case MMCHS_CMD59: //app?
        //reseved
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#0x, RESERVED!", c, cmd, status, arg);
        break;
      case MMCHS_CMD60: //erase?
      case MMCHS_CMD61: //erase?
      case MMCHS_CMD62: //erase?
      case MMCHS_CMD63: //erase?
        //reseved
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#0x, Reserved Manufacturer!", c, cmd, status, arg);
        break;
      case MMCHS_CMD34: //erase?
      case MMCHS_CMD37: //erase?
        //reseved
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#0x, OBSOLETE!", c, cmd, status, arg);
        break;
      case MMCHS_CMD12:
        //R1b (Auto CMD12 responce)
        rsp76 = (MMCHS_GET_R(mmc_slot, MMCHS_RSP76));
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#x, R1b: %#x", c, cmd, status, arg, rsp76);
        break;
      case MMCHS_CMD2:
      case MMCHS_CMD9:
      case MMCHS_CMD10:
        rsp10 = (MMCHS_GET_R(mmc_slot, MMCHS_RSP10));
        rsp32 = (MMCHS_GET_R(mmc_slot, MMCHS_RSP32));
        rsp54 = (MMCHS_GET_R(mmc_slot, MMCHS_RSP54));
        rsp76 = (MMCHS_GET_R(mmc_slot, MMCHS_RSP76));
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#x, R2: %#08x.%#08x.%#08x.%#08x", c, cmd, status, arg, rsp76, rsp54, rsp32, rsp10);
        break;
      case MMCHS_CMD3:
      case MMCHS_CMD11:
      case MMCHS_CMD13:
      case MMCHS_CMD14:
      case MMCHS_CMD16: //block read
      case MMCHS_CMD17: //block read
      case MMCHS_CMD18: //block read
      case MMCHS_CMD19:
      case MMCHS_CMD20: //stream write
      case MMCHS_CMD23: //block write
      case MMCHS_CMD24: //block write
      case MMCHS_CMD25: //block write
      case MMCHS_CMD26: //block write
      case MMCHS_CMD27: //block write
      case MMCHS_CMD28: //block write protection
      case MMCHS_CMD29: //block write protection
      case MMCHS_CMD30: //block write protection
      case MMCHS_CMD32: //erase start (sd)
      case MMCHS_CMD33: //erase end (sd)
      case MMCHS_CMD35: //erase start (mmc)
      case MMCHS_CMD36: //erase end (mmc)
      case MMCHS_CMD42: //lock
      case MMCHS_CMD55: //app
      case MMCHS_CMD56: //app
        rsp = "R1";
        goto short_rsp;
      case MMCHS_CMD5:
      case MMCHS_CMD6:
      case MMCHS_CMD8:
      case MMCHS_CMD38: //erase execute
        rsp = "R1b";
        goto short_rsp;
      case MMCHS_CMD7:
        rsp = "R1/R1b";
        goto short_rsp;
      case MMCHS_CMD1:
        rsp = "R3";
        goto short_rsp;
      case MMCHS_CMD39: //IO
        rsp = "R4";
        goto short_rsp;
      case MMCHS_CMD40: //IO
        rsp = "R4";
        goto short_rsp;
      default:
        rsp = "R?";
short_rsp:
        //including R1b from non CMD12 
        rsp10 = (MMCHS_GET_R(mmc_slot, MMCHS_RSP10));
                DBG_PRINTF("CMD%d: %#x, Status: %#x, Arg: %#x, %s: %#x", c, cmd, status, arg, rsp, rsp10);
        break;
      }
      { ///@todo only delay MMC access if sufficient time has not already passed (includes move this code up front)
        volatile U32 *delay_variable = &get_driver_data()->delay_variable;
        U32 delay_count = 1 << (get_driver_data()->delay);
        if (delay_count == 0)
        {
          delay_count = DEFAULT_DELAY_COUNT_EXPONENT;
        }

        for (*delay_variable = 0; *delay_variable < delay_count; ++*delay_variable);
      }
      return OMAPFLASH_SUCCESS;
    }
    else
    {
      return MMC_CMD_FAIL;
    }
  }
}								/* end of send_mmc_cmd */

/*-----------------------------------------------------------------------------
| Function    : get_mmc_cmd_response
+------------------------------------------------------------------------------
|
| Description : gets the response for the send command.
|
| Parameters  : U32 *resp, U8 response_type
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 get_mmc_cmd_response(U8 mmc_slot, U32 * resp, U8 response_type)
{
  U32 ret_value = OMAPFLASH_SUCCESS;

  if (MMC_RSP2 == response_type)
  {
    *resp = MMCHS_GET_R(mmc_slot, MMCHS_RSP10);
    resp++;
    *resp = MMCHS_GET_R(mmc_slot, MMCHS_RSP32);
    resp++;
    *resp = MMCHS_GET_R(mmc_slot, MMCHS_RSP54);
    resp++;
    *resp = MMCHS_GET_R(mmc_slot, MMCHS_RSP76);
  }
  else
  {
    *resp = MMCHS_GET_R(mmc_slot, MMCHS_RSP10);
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_cmd_status
+------------------------------------------------------------------------------
|
| Description : gets the response for the send command.
|
| Parameters  : void
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_cmd_status(U8 mmc_slot)
{
  U32 status;
  U32 mmc_timeout = get_driver_data()->timeout;               // in msec
  U32 mmc_timeout_tick = get_driver_data()->timeout_tick;     // in msec
  check_timeout_ms(2, 0);
  do
  {
    /* Get status */
    status = MMCHS_GET_R(mmc_slot, MMCHS_STAT);

    /* Status is Eod of command phase - OK */
    if (((status & MMCHS_MMCSTAT_CC) == MMCHS_MMCSTAT_CC)
      || ((status & MMCHS_MMCSTAT_CTO) == MMCHS_MMCSTAT_CTO))
    {
      break;
    }
    else if (status & MMCHS_MMCSTAT_CERR)
    {
      break;
    }
    wait_microsec(mmc_timeout_tick * 1000);	// mmc_timeout_tick is in msec
  }
  while(check_timeout_ms(2, mmc_timeout) != TRUE);

  if ((status & MMCHS_MMCSTAT_CC) != 0)
  {
    return OMAPFLASH_SUCCESS;
  }
  else if ((status & MMCHS_MMCSTAT_CERR) != 0)
  {
    return MMC_CARD_ER;
  }
  else if ((status & MMCHS_MMCSTAT_CTO) != 0)
  {
    return MMC_CMD_TO_ER;
  }
  else
    return MMC_CMD_FAIL;

}								/* end of send_mmc_cmd */

/*-----------------------------------------------------------------------------
| Function    : mmc_datatransfer_enable_disable
+------------------------------------------------------------------------------
|
| Description : Put the card in the transfer state
|
| Parameters  : U8 enable, U16 card_rca
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_datatransfer_enable_disable(U8 mmc_slot, U8 flag, U16 card_rca)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 i;
  U32 res_data = 0;

#if 1
  if (MMC_TRANFER_ENABLE == flag)
  {
    ret_value =
      send_mmc_cmd(mmc_slot, MMCHS_SEND_STATUS, (card_rca & MMC_STUFF_BITS),
      MMCHS_STUFF_BITS);
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    ret_value = get_mmc_cmd_response(mmc_slot, &res_data, MMC_RSP1);
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

		DBG_PRINTF("Card state %#x", res_data); // i < 10

    res_data = (res_data & MMC_CARD_STATE_TRAN);

    for (i = 0; (i < 10) && (res_data != MMC_CARD_STATE_TRAN); i++)
    {
      /* send the select cmd with RCA to select the card for data tranfer */
      ret_value =
        send_mmc_cmd(mmc_slot, MMCHS_SELECT_CARD, (card_rca & MMC_STUFF_BITS),
        MMCHS_STUFF_BITS);
      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }

      ret_value =
        send_mmc_cmd(mmc_slot, MMCHS_SEND_STATUS, (card_rca & MMC_STUFF_BITS),
        MMCHS_STUFF_BITS);
      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }

      ret_value = get_mmc_cmd_response(mmc_slot, &res_data, MMC_RSP1);
      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }

			DBG_PRINTF("Card state %#x, i= %d", res_data, i); // i < 10

      res_data = (res_data & MMC_CARD_STATE_TRAN);
    }
    if (i >= 10)
    {
      drv_send_info("Card state %#02x, i= %d, busy", res_data, i); // i >= 10
      ret_value = MMC_BUSY_ER;
    }

  }
  else if (MMC_TRANFER_DISABLE == flag)
  {
    card_rca = 0;
    /* send the deselect cmd with RCA=0 to unselct the card */
    ret_value =
      send_mmc_cmd(mmc_slot, MMCHS_DESELECT_CARD, (card_rca & MMC_STUFF_BITS),
      MMCHS_STUFF_BITS);
  }
  else
  {
    ret_value = OMAPFLASH_DAL_ERROR;
  }

  /*sending the CMD7 to change state between tanfer state and */
  return ret_value;
#else
  U32 card_state;

  if(get_card_state(mmc_slot, card_rca, &card_state) != OMAPFLASH_SUCCESS)
  {
    drv_send_info("mmc_datatransfer_enable_disable: Error reading card status");
    return OMAPFLASH_ERROR;
  }

  if (MMC_TRANFER_ENABLE == flag)
  {
    // Sending SELECT_CARD when already in TRANS_SATE can be illegal cmd. So check it out first.
    if((card_state & MMC_CARD_STATE_TRAN) == MMC_CARD_STATE_TRAN)
      return OMAPFLASH_SUCCESS;

    // Card should be in StandBy state to put it to Transfer State for data access
    if((card_state & MMC_CARD_STATE_STBY) != MMC_CARD_STATE_STBY)
    {
      drv_send_info("Card is not right state to put in transfer state. Card State:%#02X", card_state);
      return OMAPFLASH_ERROR;
    }

    /* send the select cmd with RCA to select the card for data tranfer */
    ret_value = send_mmc_cmd(mmc_slot, MMCHS_SELECT_CARD, (card_rca & MMC_STUFF_BITS), MMCHS_STUFF_BITS);
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    if(get_card_state(mmc_slot, card_rca, &card_state) != OMAPFLASH_SUCCESS)
    {
      drv_send_info("mmc_datatransfer_enable_disable: Error reading card status");
      return OMAPFLASH_ERROR;
    }

    return ((card_state & MMC_CARD_STATE_TRAN) == MMC_CARD_STATE_TRAN)? OMAPFLASH_SUCCESS: OMAPFLASH_ERROR;        
  }
  else if (MMC_TRANFER_DISABLE == flag)
  {
    card_rca = 0;

    // Sending DESELECT_CARD when already in STBY can be illegal cmd. So check it out first.
    if((card_state & MMC_CARD_STATE_STBY) == MMC_CARD_STATE_STBY)
      return OMAPFLASH_SUCCESS;

    // Card should be in Transfer or Data state to put it to Standby State 
    if(((card_state & MMC_CARD_STATE_TRAN) != MMC_CARD_STATE_TRAN) && ((card_state & MMC_CARD_STATE_DATA) != MMC_CARD_STATE_DATA))
    {
      drv_send_info("Card is not right state to put in standby state. Card State:%#02X", card_state);
      return OMAPFLASH_ERROR;
    }

    /* send the deselct cmd */
    ret_value = send_mmc_cmd(mmc_slot, MMCHS_DESELECT_CARD, (card_rca & MMC_STUFF_BITS), MMCHS_STUFF_BITS);
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    if(get_card_state(mmc_slot, card_rca, &card_state) != OMAPFLASH_SUCCESS)
    {
      drv_send_info("mmc_datatransfer_enable_disable: Error reading card status");
      return OMAPFLASH_ERROR;
    }

    return ((card_state & MMC_CARD_STATE_STBY) == MMC_CARD_STATE_STBY)? OMAPFLASH_SUCCESS: OMAPFLASH_ERROR;        
  }
  else
  {
    return OMAPFLASH_DAL_ERROR;
  }

#endif
}

/*-----------------------------------------------------------------------------
| Function    : mmc_opendrain_enable_disable
+------------------------------------------------------------------------------
|
| Description : puts in or out, the controller to operate in Opendrain mode.
|
| Parameters  : U8 enable
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_opendrain_enable_disable(U8 mmc_slot, U8 enable)
{
  U32 ret_value = OMAPFLASH_SUCCESS;

  if (MMC_OPENDRAIN_ENABLE == enable)
  {
    /* Enable the Opendrain mode */
    MMCHS_FSET_R(mmc_slot, MMCHS_CON, MMCHS_ODRN);
  }
  else if (MMC_OPENDRAIN_DISABLE == enable)
  {
    /* Disable the Opendrain mode */
    MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~MMCHS_ODRN);
  }
  else
  {
    ret_value = OMAPFLASH_DAL_ERROR;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_set_data_width
+------------------------------------------------------------------------------
|
| Description : sets transfer data width in controller and card
|
| Parameters  : U8 data_bits
|
| Returns     : U8
|
+-----------------------------------------------------------------------------*/
U32 mmc_set_data_width(U8 mmc_slot, U8 data_bits, U32 ddr)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U8 ddr_set = (U8)ddr;

  if (MMC_DATAWIDTH_8_BITS == data_bits)
  {
    /* HS MMC 8-bit mode */
    /* Sending Switch command to change the mode to 8 bit */

    ret_value = mmc_send_switch_command(SWITCH_MODE_WR_BYTE, mmc_slot, EXT_CSD_OFFSET(bus_width), ddr_set ? 0x06 : 0x02);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    /* Setting the controller for 8-bit mode */
    if(ddr_set)
      MMCHS_FSET_R(mmc_slot, MMCHS_CON, (MMCHS_CON_DDR));
    MMCHS_FSET_R(mmc_slot, MMCHS_CON, (MMCHS_CON_DW8));

  }
  else if (MMC_DATAWIDTH_4_BITS == data_bits)
  {
    ret_value = mmc_send_switch_command(SWITCH_MODE_WR_BYTE, mmc_slot, EXT_CSD_OFFSET(bus_width), ddr_set ? 0x05 : 0x01);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    /* Setting the controller for 4-bit mode */
    if(ddr_set)
      MMCHS_FSET_R(mmc_slot, MMCHS_CON, (MMCHS_CON_DDR));
    MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~(MMCHS_CON_DW8));
    MMCHS_FSET_R(mmc_slot, MMCHS_HCTL, (MMCHS_HCTL_DTW));

  }
  else if (MMC_DATAWIDTH_1_BITS == data_bits)
  {
    if(ddr_set)
    {
      drv_send_info("Error: DDR mode not supported with 1-bit data width ");
      return OMAPFLASH_ERROR;
    }

    ret_value = mmc_send_switch_command(SWITCH_MODE_WR_BYTE, mmc_slot, EXT_CSD_OFFSET(bus_width), 0x00);

    /* Setting the controller for 1-bit mode */
    MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~(MMCHS_CON_DW8));
    MMCHS_ASET_R(mmc_slot, MMCHS_HCTL, ~(MMCHS_HCTL_DTW));
  }
  else
  {
    /* Default : Setting the controller for 1-bit mode */
    //MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~(MMCHS_CON_DW8));
    //MMCHS_ASET_R(mmc_slot, MMCHS_HCTL, ~(MMCHS_HCTL_DTW));
    drv_send_info("Error: %#02x-bit data width not supported ", data_bits);
    return OMAPFLASH_ERROR;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_erase_block_data
+------------------------------------------------------------------------------
|
| Description : erases one block of data or less
|
| Parameters  : U8 card_type,U32 start_addr,U32 end_addr
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_erase_block_data(U8 mmc_slot, U8 card_type, U32 start_addr, U32 end_addr)
{
  U32 ret_value = OMAPFLASH_SUCCESS;

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    /* CLK Unstable */
    drv_send_info("MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }

  if (SD_CARD == card_type)
  {
    /*SD*/
    /* CMD 32 - give start address for erasing */

    ret_value = send_mmc_cmd(mmc_slot, SDHS_ERASE_WR_BLK_START, ((start_addr >> 16) & MMC_STUFF_BITS), (start_addr & MMC_STUFF_BITS));

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }
    /* CMD 33 - give end address for erasing */
    ret_value =
      send_mmc_cmd(mmc_slot, SDHS_ERASE_WR_BLK_END,
      ((end_addr >> 16) & MMC_STUFF_BITS),
      (end_addr & MMC_STUFF_BITS));
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }
  }
  else
  {
    /*MMC*/
    /* CMD 35 - give start address for erasing */
    ret_value =
      send_mmc_cmd(mmc_slot, MMCHS_ERASE_GROUP_START,
      ((start_addr >> 16) & MMC_STUFF_BITS),
      (start_addr & MMC_STUFF_BITS));
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }
    /* CMD 36 - give end address for erasing */
    ret_value =
      send_mmc_cmd(mmc_slot, MMCHS_ERASE_GROUP_END,
      ((end_addr >> 16) & MMC_STUFF_BITS),
      (end_addr & MMC_STUFF_BITS));
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }
  }

  /* Erase command (CMD38) */
  ret_value = send_mmc_cmd(mmc_slot, MMCHS_ERASE, MMC_STUFF_BITS, MMC_STUFF_BITS);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_TC, 2 * HUNDRED_MILLISEC))
  {
    drv_send_info("MMCHS_MMCSTAT_TC not set!");
    return MMC_BUSY_ER;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_write_block_data()
+------------------------------------------------------------------------------
|
| Description : writes given data pattern to one block memory of the card from
|               the given card's address
|
| Parameters  : U32 card_addr, void *buffer
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_write_block_data(U8 mmc_slot, U32 card_addr, void *buffer)
{
  U32 ret_value = OMAPFLASH_SUCCESS;	
  U32 j;
  U32 *buf = (U32 *) buffer; U32 status;
  U8 dma_enabled = (mmc_slot == MMC_SLOT_1 || mmc_slot == MMC_SLOT_2)? TRUE : FALSE;
  U32 cmd;

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    /* CLK Unstable */
    drv_send_info("MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }

  if(dma_enabled)
  {
    if(mmc_adma_config(mmc_slot, (U32)buf, MMC_BLOCK_SIZE) != OMAPFLASH_SUCCESS)
      return OMAPFLASH_ERROR;
  }

  cmd = (dma_enabled)? MMCHS_WRITE_SINGLE_BLOCK_ADMA : MMCHS_WRITE_SINGLE_BLOCK;
  ret_value = send_mmc_cmd(mmc_slot, cmd, (U16) ((card_addr >> 16) & MMC_STUFF_BITS), (U16) (card_addr & MMC_STUFF_BITS));	

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if(!dma_enabled)
  {
    for (j = 0; j < (MMC_BLOCK_SIZE / sizeof(U32)); j++)	// write data to DATA buffer
    {
      /*Wait for Buffer Write Enable bit to be set */
      if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BWR, ONE_SECOND))
      {
        drv_send_info("Card could not Set for BWR state: Read status: %#02x",
          MMCHS_GET_R(mmc_slot, MMCHS_STAT));
        return MMC_BUSY_ER;
      }
      MMCHS_SET_R(mmc_slot, MMCHS_DATA, buf[j]);
    }
  }

#if 0
  ret_value = data_transfer_complete(mmc_slot);
#else
  while(1)
  {	
    status = (MMCHS_GET_R(mmc_slot, MMCHS_STAT));
    if(status & MMCHS_MMCSTAT_TC)
    {
      MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_TC);
      ret_value = OMAPFLASH_SUCCESS;
      break;
    }

    if(status & (MMCHS_MMCSTAT_DEB | MMCHS_MMCSTAT_DCRC | MMCHS_MMCSTAT_DTO))
    {
      MMCHS_SET_R(mmc_slot, MMCHS_STAT, 0xFFFFFFFF);
      ret_value = OMAPFLASH_ERROR;
      reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRD);
      break;
    }       

    if((status = MMCHS_GET_R(mmc_slot, MMCHS_ADMAES)))
    {
      drv_send_info("Error with ADMA transfer. status : %#02X", status);
      ret_value = OMAPFLASH_ERROR;
      break;
    }
  }
#endif

  if (OMAPFLASH_SUCCESS != ret_value)
  {
		DBG_PRINTF("WRITE BLOCK: TC state not set -status: %#x\r\n",
      MMCHS_GET_R(mmc_slot, MMCHS_STAT));
    return ret_value;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_write_multi_block_data()
+------------------------------------------------------------------------------
|
| Description : writes given data pattern to one block memory of the card from
|               the given card's address
|
| Parameters  : U32 card_addr, void *buffer
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_write_multi_block_data(U8 mmc_slot, U32 card_addr, void *buffer, U32 num_blks)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 j,status;
  U32 *buf = (U32 *) buffer; 
  U8 dma_enabled = (mmc_slot == MMC_SLOT_1 || mmc_slot == MMC_SLOT_2)? TRUE : FALSE;
  U32 cmd;

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    /* CLK Unstable */
		dbg_printf("MMCHS_PSTAT not out from the DATI busy state!");
    drv_send_info("####mmc_write_multi_block_data MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }

  if(dma_enabled)
  {
    if(mmc_adma_config(mmc_slot, (U32)buf, MMC_BLOCK_SIZE * num_blks) != OMAPFLASH_SUCCESS)
      return OMAPFLASH_ERROR;
  }

  ret_value = send_mmc_cmd(mmc_slot, MMCHS_SET_BLOCK_COUNT, 0 , (U16) (num_blks & 0xFFFF));

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    drv_send_info("####mmc_write_multi_block_data MMCHS_WRITE_MULTIPLE_BLOCK! %#02x", ret_value);
    return ret_value;
  }

  MMCHS_SET_R(mmc_slot, MMCHS_BLK, (num_blks<<16) | 0x200 );

  cmd = (dma_enabled)? MMCHS_WRITE_MULTIPLE_BLOCK_ADMA : MMCHS_WRITE_MULTIPLE_BLOCK;
  ret_value =	send_mmc_cmd(mmc_slot, cmd, (U16)((card_addr >> 16) & MMC_STUFF_BITS), (U16)(card_addr & MMC_STUFF_BITS));

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if(!dma_enabled)
  {
    while(1)
    {
      do
      {	
        status = (MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      } while(status == 0);

      if (status & MMCHS_MMCSTAT_BWR)
      {	
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BWR);
        for(j=0; j < (MMC_BLOCK_SIZE / 4); j++)
        {
          MMCHS_SET_R(mmc_slot, MMCHS_DATA, *buf);
          buf++;
        }
      }

      if(status & MMCHS_MMCSTAT_BRR) 
      {
        MMCHS_SET_R(mmc_slot,MMCHS_STAT,MMCHS_MMCSTAT_BRR);
      }

      if(status & MMCHS_MMCSTAT_TC)
      {
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_TC);
        break;
      }
    }
  }
  else
  {
    while(1)
    {	
      status = (MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      if(status & MMCHS_MMCSTAT_TC)
      {
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_TC);
        ret_value = OMAPFLASH_SUCCESS;
        break;
      }

      if(status & (MMCHS_MMCSTAT_DEB | MMCHS_MMCSTAT_DCRC | MMCHS_MMCSTAT_DTO))
      {
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, 0xFFFFFFFF);
        ret_value = OMAPFLASH_ERROR;
        reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRD);
        break;
      }

      if((status = MMCHS_GET_R(mmc_slot, MMCHS_ADMAES)))
      {
        drv_send_info("Error with ADMA transfer. status : %#02X", status);
        ret_value = OMAPFLASH_ERROR;
        break;
      }
    } 
  }
  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_read_single_block()
+------------------------------------------------------------------------------
|
| Description : reads single block of data and puts into the buffer
|
| Parameters  : U32 card_addr, void *buffer
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_read_single_block(U8 mmc_slot, U32 card_addr, void *buffer)
{
  U32 ret_value = OMAPFLASH_SUCCESS, status;
  U32 i;
  U32 *buf = (U32 *) buffer;
  U8 dma_enabled = (mmc_slot == MMC_SLOT_1 || mmc_slot == MMC_SLOT_2)? TRUE : FALSE;
  U32 cmd;

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    drv_send_info("MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }

  if(dma_enabled)
  {
    mmc_adma_config(mmc_slot, (U32)buf, MMC_BLOCK_SIZE);
  }

  /* CMD17 R1 (READ_SINGLE_BLOCK) Arguments are address specified by caller */
  cmd = (dma_enabled)? MMCHS_READ_SINGLE_BLOCK_ADMA : MMCHS_READ_SINGLE_BLOCK;
  ret_value =
    send_mmc_cmd(mmc_slot, cmd,
    (U16) ((card_addr >> 16) & MMC_STUFF_BITS),
    (U16) (card_addr & MMC_STUFF_BITS));

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    drv_send_info(" Send CMD failed. ret_value: %302X", ret_value);
    return ret_value;
  }

  if(!dma_enabled)
  {
    /*Wait for Block ready for read */
    if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BRR, ONE_SECOND))
    {
      drv_send_info("Card could not Set for BRR state: Read status: %#02x",
        MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      return MMC_BUSY_ER;
    }

    /* Get all data from DATA register and write in user buffer */
    for (i = 0; i < (MMC_BLOCK_SIZE / sizeof(U32)); i++)
    {
      *buf = MMCHS_GET_R(mmc_slot, MMCHS_DATA);
      buf++;
    }
  }

#if 0
  ret_value = data_transfer_complete(mmc_slot);
#else
  while(1)
  {	
    status = (MMCHS_GET_R(mmc_slot, MMCHS_STAT));

    if(status & MMCHS_MMCSTAT_TC)
    {
      MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_TC);
      ret_value = OMAPFLASH_SUCCESS;
      break;
    }

    if(status & (MMCHS_MMCSTAT_DEB | MMCHS_MMCSTAT_DCRC | MMCHS_MMCSTAT_DTO))
    {
      MMCHS_SET_R(mmc_slot, MMCHS_STAT, 0xFFFFFFFF);
      ret_value = OMAPFLASH_ERROR;
      reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRD);
      break;
    }

    if((status = MMCHS_GET_R(mmc_slot, MMCHS_ADMAES)))
    {
      drv_send_info("Error with ADMA transfer. status : %#02X", status);
      ret_value = OMAPFLASH_ERROR;
      break;
    }
  } 
#endif
  if (OMAPFLASH_SUCCESS != ret_value)
  {
		dbg_printf("READ BLOCK: TC state not set -status: %#x\r\n",
      MMCHS_GET_R(mmc_slot, MMCHS_STAT));
    return ret_value;
  }
  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_read_multi_block_data()
+------------------------------------------------------------------------------
|
| Description : read given data pattern to one block memory of the card from
|               the given card's address
|
| Parameters  : U32 card_addr, void *buffer
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_read_multi_block_data(U8 mmc_slot, U32 card_addr, void *buffer, U32 num_blks)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 j,status;
  U32 *buf = (U32 *) buffer; 
  U8 dma_enabled = (mmc_slot == MMC_SLOT_1 || mmc_slot == MMC_SLOT_2)? TRUE : FALSE;
  U32 cmd;

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    /* CLK Unstable */
		dbg_printf("MMCHS_PSTAT not out from the DATI busy state!");
    drv_send_info("####mmc_read_multi_block_data MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }

  if(dma_enabled)
  {
    if(mmc_adma_config(mmc_slot, (U32)buf, MMC_BLOCK_SIZE * num_blks) != OMAPFLASH_SUCCESS)
      return OMAPFLASH_ERROR;
  }

  ret_value = send_mmc_cmd(mmc_slot, MMCHS_SET_BLOCK_COUNT, 0 , (U16) (num_blks & 0xFFFF));

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    drv_send_info("####mmc_read_multi_block_data MMCHS_READ_MULTIPLE_BLOCK! %#02x", ret_value);
    return ret_value;
  }

  MMCHS_SET_R(mmc_slot, MMCHS_BLK, (num_blks<<16) | 0x200 );

  cmd = (dma_enabled)? MMCHS_READ_MULTIPLE_BLOCK_ADMA : MMCHS_READ_MULTIPLE_BLOCK;
  ret_value =	send_mmc_cmd(mmc_slot, cmd, (U16)((card_addr >> 16) & MMC_STUFF_BITS), (U16)(card_addr & MMC_STUFF_BITS));

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if(!dma_enabled)
  {
    while(1)
    {
      do
      {	
        status = (MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      } while(status == 0);

      if (status & MMCHS_MMCSTAT_BRR)
      {	
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BRR);
        for(j=0; j < (MMC_BLOCK_SIZE / 4); j++)
        {
          *buf = MMCHS_GET_R(mmc_slot, MMCHS_DATA);
          buf++;
        }
      }

      /*
      if(status & MMCHS_MMCSTAT_BWR) 
      {
      MMCHS_SET_R(mmc_slot,MMCHS_STAT,MMCHS_MMCSTAT_BWR);
      }*/

      if(status & MMCHS_MMCSTAT_TC)
      {
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_TC);
        break;
      }    

      if(status & (MMCHS_MMCSTAT_DEB | MMCHS_MMCSTAT_DCRC | MMCHS_MMCSTAT_DTO))
      {
        drv_send_info("mmc_read: Error reading data. status - %#02x", status);
        return OMAPFLASH_ERROR;
      }
    }
  }
  else
  {
    while(1)
    {	
      status = (MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      if(status & MMCHS_MMCSTAT_TC)
      {
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_TC);
        ret_value = OMAPFLASH_SUCCESS;
        break;
      }

      if(status & (MMCHS_MMCSTAT_DEB | MMCHS_MMCSTAT_DCRC | MMCHS_MMCSTAT_DTO))
      {
        MMCHS_SET_R(mmc_slot, MMCHS_STAT, 0xFFFFFFFF);
        ret_value = OMAPFLASH_ERROR;
        reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRD);
        break;
      }

      if((status = MMCHS_GET_R(mmc_slot, MMCHS_ADMAES)))
      {
        drv_send_info("Error with ADMA transfer. status : %#02X", status);
        ret_value = OMAPFLASH_ERROR;
        break;
      }
    } 
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : get_sd_mmc_csd()
+------------------------------------------------------------------------------
|
| Description : reads the csd data
|
| Parameters  : U16 sid, U16 card_rca, U8 card_type, U16 len, const void *dat
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 get_sd_mmc_csd_ex(U16 sid, U16 card_rca, U8 card_type, U16 regs[8])
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 card_state = 0;
  U8 mmc_slot;

  /*get the mmc slot number from the sid */
  mmc_slot = (U8)sid;

  /*Check the card is in Stanby state or not..? */
  ret_value = get_card_state(mmc_slot, card_rca, &card_state);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }
  if (card_state != MMC_CARD_STATE_STBY)
  {
    drv_send_info("Card state %#02x\r\n : StandBy State Failure !", card_state);

    return (MMC_BUSY_ER);	/*Busy in other state */
  }

  /*Get the CSD data */
  ret_value =
    send_mmc_cmd(mmc_slot, MMCHS_SEND_CSD, (card_rca & MMC_STUFF_BITS),
    MMCHS_STUFF_BITS);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
		dbg_printf("Reading CSD Data Failure!");
    return ret_value;
  }

  /* Read data from MMC response registers */
  regs[0] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP10) & 0x0000ffff);	/*read RSP0,RSP1 from RSP10 */
  regs[1] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP10) >> 16);
  regs[2] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP32) & 0x0000ffff);	/*read RSP1,RSP2 from RSP32 */
  regs[3] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP32) >> 16);
  regs[4] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP54) & 0x0000ffff);	/*read RSP3,RSP4 from RSP54 */
  regs[5] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP54) >> 16);
  regs[6] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP76) & 0x0000ffff);	/*read RSP5,RSP6 from RSP76 */
  regs[7] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP76) >> 16);

  return ret_value;
}

U32
  get_sd_mmc_csd(U16 sid, U16 card_rca, U8 card_type, U16 len, const void *dat)
{
  U16 regs[8];				/* registers array */
  U16 lo;						/* lower bits of data returned in different registers */
  T_MMC_CSD *csd = (T_MMC_CSD *) dat;
  U32 ret_value = get_sd_mmc_csd_ex(sid, card_rca, card_type, regs);

  if (ret_value != OMAPFLASH_SUCCESS) 
    return ret_value;

  /* refer to SanDisk MultiMediaCard Product Manual */
  csd->crc = regs[0] & 0xFE;	//7:1
  csd->tmp_write_protect = (regs[0] >> 12) & 1;	// 12
  csd->perm_write_protect = (regs[0] >> 13) & 1;	// 13
  csd->copy = (regs[0] >> 14) & 1;	// 14

  csd->write_bl_partial = (regs[1] >> 5) & 1;	// 21
  csd->write_bl_len = (regs[1] >> 6) & 0xF;	// 25:22
  csd->r2w_factor = (regs[1] >> 10) & 0x7;	// 28:26
  csd->default_ecc = (regs[1] >> 13) & 0x3;	// 30:29
  csd->wp_grp_enable = (regs[1] >> 15) & 1;	// 31

  csd->wp_grp_size = (regs[2] & 0x1F);	// 36:32
  csd->erase_grp_mult = (regs[2] >> 5) & 0x1f;	// 41:37
  csd->erase_grp_size = (regs[2] >> 10) & 0x1f;	// 46:42
  lo = (regs[2] >> 15) & 1;	// 49:47
  csd->csize_mult = ((regs[3] & 3) << 1) | lo;

  csd->vdd_wcurr_max = (regs[3] >> 2) & 0x7;	// 52:50
  csd->vdd_wcurr_min = (regs[3] >> 5) & 0x7;	// 55:53
  csd->vdd_rcurr_max = (regs[3] >> 8) & 0x7;	// 58:56
  csd->vdd_rcurr_min = (regs[3] >> 11) & 0x7;	// 61:59
  lo = (regs[3] >> 14) & 0x3;	// 73:62
  csd->csize = ((regs[4] & 0x3FF) << 2) | lo;
  csd->dsr_imp = (regs[4] >> 12) & 1;	// 76
  csd->read_blk_misalign = (regs[4] >> 13) & 1;	// 77
  csd->write_blk_misalign = (regs[4] >> 14) & 1;	// 78
  csd->read_blk_partial = (regs[4] >> 15) & 1;	// 79
  csd->read_blk_len = (regs[5]) & 0xf;	// 83:80
  csd->ccc = (regs[5] >> 4) & 0xfff;	// 95:84
  csd->tran_speed = (regs[6]) & 0xff;	// 103:96
  csd->nsac = (regs[6] >> 8) & 0xff;	// 111:104

  csd->taac = (regs[7]) & 0xff;	// 119:112
  csd->mmc_prot = (regs[7] >> 10) & 0xf;	// 125:122
  csd->csd_structure = (regs[7] >> 14) & 0x3;	// 127:126

  /* extra fields for SD card,make no sense for MMC card */
  csd->erase_blk_len = (regs[2] >> 14) & 0x1;	//46
  csd->erase_sector_size = (regs[2] >> 7) & 0x7F;	//39:45

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : get_sd_mmc_cid()
+------------------------------------------------------------------------------
|
| Description : reads the CID data from the card and puts in data buffer
|
| Parameters  : U16 sid, U16 card_rca, U8 card_type, U16 len, const void *data
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32
  get_sd_mmc_cid_ex(U16 sid, U16 card_rca, U8 card_type, U16 regs[8])
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 card_state = 0;
  U8 mmc_slot;

  /*get the mmc slot number from the sid */
  mmc_slot = (U8)sid;

  ret_value = get_card_state(mmc_slot, card_rca, &card_state);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if (card_state != MMC_CARD_STATE_STBY)
  {
    drv_send_info("Card state %#02x\r\n : StandBy State Failure !", card_state);

    return (MMC_BUSY_ER);	/*Busy in other state */
  }

  ret_value =
    send_mmc_cmd(mmc_slot, MMCHS_SEND_CID, (card_rca & MMC_STUFF_BITS),
    MMC_STUFF_BITS);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  /*  Read data from MMC response registers */
  regs[0] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP10) & 0x0000ffff);	/*read RSP0,RSP1 from RSP10 */
  regs[1] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP10) >> 16);
  regs[2] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP32) & 0x0000ffff);	/*read RSP1,RSP2 from RSP32 */
  regs[3] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP32) >> 16);
  regs[4] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP54) & 0x0000ffff);	/*read RSP3,RSP4 from RSP54 */
  regs[5] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP54) >> 16);
  regs[6] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP76) & 0x0000ffff);	/*read RSP5,RSP6 from RSP76 */
  regs[7] = (MMCHS_GET_R(mmc_slot, MMCHS_RSP76) >> 16);
  return ret_value;
}

#if 0
U32
  get_sd_mmc_cid(U16 sid, U16 card_rca, U8 card_type, U16 len, const void *data)
{
  U16 regs[8];				/* registers array */
  T_MMC_CID *cid = (T_MMC_CID *) data;
  U32 ret_value = get_sd_mmc_cid_ex(sid, card_rca, card_type, regs);

  /* refer to SanDisk MultiMediaCard Product Manual */
  cid->crc = regs[0] & 0xFE;	//7:1
  /* MDT fields */
  cid->year_code = (regs[0] >> 8) & 0xF;	// 11:8
  cid->month_code = (regs[0] >> 12) & 0xF;	// 12:15
  /* PSN */
  cid->mmc_v3unique_id = (U32) ((regs[2] << 16) | regs[1]);	//47:16
  cid->sd_unique_id = (U32) (((regs[3] & 0xFF) << 16) | ((regs[2] << 8) & MMC_STUFF_BITS) | ((regs[1] >> 8) & 0xFF));	//55:24
  cid->serial_number[3] = regs[1] & 0xFF;
  cid->serial_number[2] = (regs[1] >> 8) & 0xFF;
  cid->serial_number[1] = regs[2] & 0xFF;
  cid->serial_number[0] = (regs[2] >> 8) & 0xFF;
  cid->fw_rev = regs[3] & 0xF;	// 51:48
  cid->hw_rev = (regs[3] >> 4) & 0xF;	// 55:52
  /* PNM 103:56 */
  cid->product_name[5] = ((regs[3]) >> 8) & 0xFF;
  cid->product_name[4] = ((regs[4]) & 0xFF);
  cid->product_name[3] = ((regs[4]) >> 8) & 0xFF;
  cid->product_name[2] = ((regs[5]) & 0xFF);
  cid->product_name[1] = ((regs[5]) >> 8) & 0xFF;
  cid->product_name[0] = ((regs[6]) & 0xFF);
  /* OID 119:104 */
  cid->app_id = ((regs[7] & 0xFF) << 8) | ((regs[6] >> 8) & 0xFF);	//119:104
  /* MID */
  cid->mfg_id = (regs[7] >> 8) & 0xFF;

  return ret_value;
}
#endif

/*-----------------------------------------------------------------------------
| Function    : send_mmc_init_sequence()
+------------------------------------------------------------------------------
|
| Description : Send the initialization sequence to the MMC card
|
| Parameters  : void
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
//U32 send_mmc_init_sequence(U8 mmc_slot)
//{
//  U8 count;
//  U32 ret_value = OMAPFLASH_SUCCESS;
//
//  /*enable the init bit in the controller */
//  MMCHS_FSET_R(mmc_slot, MMCHS_CON, MMCHS_INIT);
//
//  for (count = 0; count < 10; count++)
//  {
//    send_mmc_cmd(mmc_slot, MMCHS_GO_IDLE_STATE, 0, 0);
//    dl_lazy_delay(ONE_MILLISEC);
//  }
//
//  /*disable the init bit in the controller */
//  MMCHS_ASET_R(mmc_slot, MMCHS_CON, (~MMCHS_INIT));
//
//  return ret_value;
//}

/*-----------------------------------------------------------------------------
| Function    : sd_mmc_card_read
+------------------------------------------------------------------------------
|
| Description : Read one block from the card..
|
| Parameters  : U16 sid,U16 card_rca, U8 card_type,U8 data_width,
|               U32 transfer_clk,U32 len,void *buf
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32
  sd_mmc_card_read(U16 sid, U16 card_rca, U8 card_type, U8 data_width, U32 transfer_clk, U32 mblock, U32 ddr, U32 start_block, U32 nu_blocks, U32 len, void *buf)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  char *data = (char *) buf;
  U32 card_addr;
  U8 mmc_slot;
  U32 i; 

  mmc_slot = (U8)sid;

  // Put Card in Transfer State
  //if(mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_ENABLE, card_rca) != OMAPFLASH_SUCCESS)
  //{
  //  drv_send_info("sd_mmc_card_read: Error putting card in transfer state");
  //  return OMAPFLASH_ERROR;
  //}

  //ret_value = set_hsmmc_clk_data_width(mmc_slot, card_type, card_rca, data_width, transfer_clk, ddr);

  //if (OMAPFLASH_SUCCESS != ret_value)
  //{
  //  return ret_value;
  //}

  if(!mblock || nu_blocks == 1)
  {
    for (i = 0; i < nu_blocks; i++)
    {
      ret_value = is_card_tranfer_state(mmc_slot, card_rca);
      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }

      if (mmc_high_density_card == 1)
      {
        /* High Density card - Sectorwise addressing */
        card_addr = (start_block + i);
      }
      else
      {
        /* Normal card - Bytewise addressing */
        card_addr = (start_block + i) * MMC_BLOCK_SIZE;
      }

      ret_value = mmc_read_single_block(mmc_slot, card_addr, data + i * MMC_BLOCK_SIZE);
      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }
    }
  }
  else
  {
    ret_value = mmc_read_multi_block_data(mmc_slot, start_block, data, nu_blocks);
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : sd_mmc_card_write
+------------------------------------------------------------------------------
|
| Description : writes the given data to card with given tranfer speed and data
|               width
|
| Parameters  : U16 sid,U16 card_rca, U8 card_type,U8 data_width,
|               U32 transfer_clk,U32 len,void *buf
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 sd_mmc_card_write(U16 sid, U16 card_rca, U8 card_type, U8 data_width, U32 transfer_clk, U32 mblock, U32 ddr, U32 start_block, U32 nu_blocks, U32 len, void *buf)
{
  static int flag = 0;
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 *data = (U32 *) buf;
  U32 card_addr;
  U32 i;

  U8 mmc_slot;
  mmc_slot = (U8)sid;

  if (write_protected_card)
  {
    drv_send_info("####sd_mmc_card_write: write_protected_card\n");
    /*card is in write protect mode, can not write */
    return MMC_WR_PR;
  }

  if(!flag)
  {
    ret_value = mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_ENABLE, card_rca);

    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    flag = 1;
  }

  if(!mblock || nu_blocks == 1)
  {
    for (i = 0; i < nu_blocks; i++)
    {
      ret_value = is_card_tranfer_state(mmc_slot, card_rca);
      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }

      if (mmc_high_density_card == 1)
      {
        /* High Density card - Sectorwise addressing */
        card_addr = (start_block + i);
      }
      else
      {
        /* Normal card - Bytewise addressing */
        card_addr = (start_block + i) * MMC_BLOCK_SIZE;
      }

      ret_value = mmc_write_block_data(mmc_slot, card_addr, data + i * MMC_BLOCK_SIZE);

      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }
    }
  }
  else
  {
    ret_value = mmc_write_multi_block_data(mmc_slot, start_block, data, nu_blocks);
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : sd_mmc_card_read
+------------------------------------------------------------------------------
|
| Description : reads data to buffer from the card with given tranfer speed
|               and data width
|
| Parameters  : U16 sid,U16 card_rca, U8 card_type,U8 data_width,
|               U32 transfer_clk,U32 len,void *buf
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32
  sd_mmc_card_erase(U16 sid, U16 card_rca, U8 card_type, U8 data_width,
  U32 transfer_clk, 
  U32 ddr,
  U32 start_block, U32 nu_blocks,
  void *buf)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 start_addr;
  U32 end_addr;
	U32 i = 0;
  U8 mmc_slot;

  mmc_slot = (U8)sid;

  ret_value = mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_ENABLE, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = set_hsmmc_clk_data_width(mmc_slot, card_type, card_rca, data_width, transfer_clk, ddr);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if (mmc_high_density_card == 1)
  {
    /* High Density card - Sectorwise addressing */
    start_addr = start_block;
  }
  else
  {
    /* Normal card - Bytewise addressing */
    start_addr = start_block * MMC_BLOCK_SIZE;
  }

  for (i = 0; i < nu_blocks; i++)
  {
    if (mmc_high_density_card == 1)
    {
      /* High Density card - Sectorwise addressing */
      end_addr = (start_addr) + (i);
    }
    else
    {
      /* Normal card - Bytewise addressing */
      end_addr = (start_addr) + (MMC_BLOCK_SIZE);
    }

    ret_value = mmc_erase_block_data(mmc_slot, card_type, start_addr, end_addr);
    if (OMAPFLASH_SUCCESS != ret_value)
    {
      return ret_value;
    }

    start_addr = end_addr;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_read_ext_csd
+------------------------------------------------------------------------------
|
| Description : reads the EXT_CSD data from the card finds HS clks field to
|               detect the support for the 52MHz/26Mhz speed
|
| Parameters  : U8 slot,U16 card_rca, U8 card_type
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_read_ext_csd(U8 mmc_slot, U8 data_width, U16 card_rca, U8 card_type, U8 * buff)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 *buf = (U32 *) buff;
  U32 i;
  U32 transfer_clk = 400;		/*Select Clk frq 400KHz to read EXT_CSD */

  ret_value = mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_ENABLE, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = is_card_tranfer_state(mmc_slot, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = mmc_set_data_width(mmc_slot, data_width, 0);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = is_card_tranfer_state(mmc_slot, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = set_mmc_controller_clk(mmc_slot, transfer_clk);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    /* CLK Unstable */
    drv_send_info("MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }

  ret_value =
    send_mmc_cmd(mmc_slot, MMCHS_SEND_EXT_CSD, MMC_STUFF_BITS, MMC_STUFF_BITS);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BRR, ONE_SECOND))
  {
    drv_send_info("Card could not Set for BRR state for read EXT_CSD. Read status: %#02x",
      MMCHS_GET_R(mmc_slot, MMCHS_STAT));
    return MMC_BUSY_ER;
  }

  /* Get all data from DATA register and write in user buffer */
  for (i = 0; i < (MMC_BLOCK_SIZE / sizeof(U32)); i++)
  {
    if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BRR, ONE_SECOND))
    {
      drv_send_info("Card could not Set for BRR state: Read status: %#02x",
        MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      return MMC_BUSY_ER;
    }

    buf[i] = MMCHS_GET_R(mmc_slot, MMCHS_DATA);
  }

  ret_value = data_transfer_complete(mmc_slot);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_DISABLE, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : is_card_tranfer_state
+------------------------------------------------------------------------------
|
| Description : Checks whether the card is in transferstate or not
|
| Parameters  : U16 card_rca
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 is_card_tranfer_state(U8 mmc_slot, U16 card_rca)
{
  U32 res_data = 0;
  U32 ret_value = OMAPFLASH_SUCCESS;

  ret_value = send_mmc_cmd(mmc_slot, MMCHS_SEND_STATUS, (card_rca & MMC_STUFF_BITS), MMCHS_STUFF_BITS);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    drv_send_info("Send Status CMD failed. %#02x", ret_value);
    return OMAPFLASH_ERROR;
  }
  res_data = MMCHS_GET_R(mmc_slot, MMCHS_RSP10);    
  if ((res_data & MMC_CARD_STATE_TRAN) != MMC_CARD_STATE_TRAN)
  {
    drv_send_info("Card not in Trans State. Card state %#02x", res_data);
    return (MMC_BUSY_ER);	/*Busy in other state */
  }

  return OMAPFLASH_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : get_card_state
+------------------------------------------------------------------------------
|
| Description : Get the state of the card
|
| Parameters  : U16 card_rca, U32 card state
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 get_card_state(U8 mmc_slot, U16 card_rca, U32 * state)
{
  U32 res_data = 0;
  U32 ret_value = OMAPFLASH_SUCCESS;

  ret_value = send_mmc_cmd(mmc_slot, MMCHS_SEND_STATUS, (card_rca & MMC_STUFF_BITS), MMCHS_STUFF_BITS);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    drv_send_info("Send Status CMD failed. %#02x", ret_value);
    return OMAPFLASH_ERROR;
  }
  res_data = MMCHS_GET_R(mmc_slot, MMCHS_RSP10);    
  *state = (res_data & MMC_CARD_STATE_MASK);

  return OMAPFLASH_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_buswidth_detect
+------------------------------------------------------------------------------
|
| Description : Detects the functional pins for data transfer
|
| Parameters  : U16 card_rca, void *max_bus_width
|
| Returns     : U32 (fail or success)
+-----------------------------------------------------------------------------*/
U32 mmc_buswidth_detect(U8 mmc_slot, U16 card_rca, void *max_bus_width)
{
  U8 i = 0;
  U8 *ptr = (U8 *)max_bus_width;
  U8 bus_width[3] = { MMC_DATAWIDTH_8_BITS,
    MMC_DATAWIDTH_4_BITS,
    MMC_DATAWIDTH_1_BITS
  };
  U32 ret_val = OMAPFLASH_SUCCESS;
  U32 card_state;
  U32 retry = 0;

  ret_val = mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_ENABLE, card_rca);
  if (OMAPFLASH_SUCCESS != ret_val)
  {
    return ret_val;
  }

  for (i = 0; i < 3; i++)
  {
    ret_val = mmc_bustest_w(mmc_slot, bus_width[i]);
    if (OMAPFLASH_SUCCESS != ret_val)
    {
			dbg_printf("Card Data width failed: mmc_bustest_w");
      return ret_val;
    }

    retry = 10;
    do
    {
      /*Check the card is in bus test state or not..? */
      ret_val = get_card_state(mmc_slot, card_rca, &card_state);
      if (OMAPFLASH_SUCCESS != ret_val)
      {
        return ret_val;
      }
      retry--;

    }
    while ((card_state != MMC_CARD_STATE_BTST) && retry != 0);
    if (retry == 0)
    {
			dbg_printf("Card state %#x\r\n : bustest State Failure !",
        card_state);

      return OMAPFLASH_DAL_ERROR;	/*Busy in other state */
    }

    ret_val = mmc_bustest_r(mmc_slot, bus_width[i]);
    if (OMAPFLASH_SUCCESS == ret_val)
    {
      ret_val = is_card_tranfer_state(mmc_slot, card_rca);
      if (OMAPFLASH_SUCCESS != ret_val)
      {
        return ret_val;
      }

      *ptr = bus_width[i];

			dbg_printf("Card Data width detected, data_width = %d\r\n", bus_width[i]);

			ret_val = mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_DISABLE, card_rca);

      if (OMAPFLASH_SUCCESS != ret_val)
      {
        return ret_val;
      }
    }

    ret_val = is_card_tranfer_state(mmc_slot, card_rca);
    if (OMAPFLASH_SUCCESS != ret_val)
    {
      return ret_val;
    }
  }							/* for(i=0; i<3; i++) */
  if (i == 3)
  {
		dbg_printf("Card Data width detect failed: tried with 8, 4, 1 ");
    ret_val = OMAPFLASH_DAL_ERROR;
  }

  ret_val = mmc_datatransfer_enable_disable(mmc_slot, MMC_TRANFER_DISABLE, card_rca);
  if (OMAPFLASH_SUCCESS != ret_val)
  {
    return ret_val;
  }

  return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_bustest_w()
+------------------------------------------------------------------------------
|
| Description : writes standard data pattern to card
|
| Parameters  : U8 data_width
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_bustest_w(U8 mmc_slot, U8 data_width)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 j = 1000;
  U8 buf[4] = { 0 };
  U32 temp_data;

  if (data_width == MMC_DATAWIDTH_8_BITS)
  {
    /* Setting the controller for 8-bit mode */
    MMCHS_FSET_R(mmc_slot, MMCHS_CON, (MMCHS_CON_DW8));

    buf[0] = 0x55;
    buf[1] = 0xAA;


  }
  else if (data_width == MMC_DATAWIDTH_4_BITS)
  {
    /* Setting the controller for 4-bit mode */
    MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~(MMCHS_CON_DW8));
    MMCHS_FSET_R(mmc_slot, MMCHS_HCTL, (MMCHS_HCTL_DTW));

    buf[0] = 0x5A;
  }
  else
  {
    /* Setting the controller for 1-bit mode */
    MMCHS_ASET_R(mmc_slot, MMCHS_CON, ~(MMCHS_CON_DW8));
    MMCHS_ASET_R(mmc_slot, MMCHS_HCTL, ~(MMCHS_HCTL_DTW));

    buf[0] = 0x40;
  }

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    drv_send_info("MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }


  ret_value = send_mmc_cmd(mmc_slot, MMCHS_BUSTEST_W, MMCHS_STUFF_BITS, MMCHS_STUFF_BITS);
  //ret_value = send_mmc_cmd(MMCHS_BUSTEST_W, 0xaaaa, 0xaaaa);

  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  memcpy(&temp_data, buf, 4);

  for (j = 0; j < (MMC_BLOCK_SIZE / sizeof(U32)); j++)	// write data to DATA buffer
  {
    if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BWR, ONE_SECOND))
    {
      drv_send_info("Card could not Set for BWR state: Read status: %#02x",
        MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      return MMC_BUSY_ER;
    }

    MMCHS_SET_R(mmc_slot, MMCHS_DATA, temp_data);
  }


  data_transfer_complete(mmc_slot);	/*Ignore DTO for Bustest */

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_bustest_r()
+------------------------------------------------------------------------------
|
| Description : reads standard data pattern from card
|
| Parameters  : U8 data_width
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/
U32 mmc_bustest_r(U8 mmc_slot, U8 data_width)
{
  U32 ret_value = OMAPFLASH_SUCCESS;
  U32 i = 1000;
  U8 buf[512] = { 0 };
  U32 *ptr = (U32 *) buf;

  if (!MMCHS_FWAITCLR_R(mmc_slot, MMCHS_PSTAT, MMCHS_PSTAT_DATI, FIVEHUNDRED_MILLISEC))
  {
    drv_send_info("MMCHS_PSTAT not out from the DATI busy state!");
    return MMC_BUSY_ER;
  }

  ret_value = send_mmc_cmd(mmc_slot, MMCHS_BUSTEST_R, MMCHS_STUFF_BITS,
    MMCHS_STUFF_BITS);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  for (i = 0; i < (MMC_BLOCK_SIZE / sizeof(U32)); i++)
  {
    if (!MMCHS_FWAITSET_R(mmc_slot, MMCHS_STAT, MMCHS_MMCSTAT_BWR, ONE_SECOND))
    {
      drv_send_info("Card could not Set for BWR state: Read status: %#02x",
        MMCHS_GET_R(mmc_slot, MMCHS_STAT));
      return MMC_BUSY_ER;
    }

    *ptr = MMCHS_GET_R(mmc_slot, MMCHS_DATA);
    ptr++;
  }

  data_transfer_complete(mmc_slot);	/*Ignore the DTO for Bustest */

  switch (data_width)
  {
  case MMC_DATAWIDTH_8_BITS:
    if (buf[0] != 0xAA || (buf[1] != 0x55))
    {
      /*Bus test failed with 8-bit data */
      return OMAPFLASH_DAL_ERROR;
    }
    break;

  case MMC_DATAWIDTH_4_BITS:
    if (buf[0] != 0xA5)
    {
      /*Bus test failed with 8-bit data */
      return OMAPFLASH_DAL_ERROR;
    }
    break;

  case MMC_DATAWIDTH_1_BITS:
    if ((buf[0] & 0xc0) != 0x80)
    {
      /*Bus test failed with 8-bit data */
      return OMAPFLASH_DAL_ERROR;
    }
    break;

  default:
    return OMAPFLASH_DAL_ERROR;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : data_transfer_complete
+------------------------------------------------------------------------------
|
| Description : Check the data transfers complete or not
|
| Parameters  : void
|
| Returns     : U32
|
+-----------------------------------------------------------------------------*/

/**
* callback used by data_transfer_complete
*/
U32 data_transfer_complete_callback(U32 time_left_microsec, void *d)
{
  U8 mmc_slot = (U8)(int)d;
  if ((MMCHS_GET_R(mmc_slot, MMCHS_STAT) & MMCHS_MMCSTAT_TC) != MMCHS_MMCSTAT_TC)
  {
    if ((MMCHS_GET_R(mmc_slot, MMCHS_STAT) &
      (MMCHS_MMCSTAT_DEB | MMCHS_MMCSTAT_DCRC | MMCHS_MMCSTAT_DTO)))
    {
      /*
      dbg_print(DBG_LEVEL_CRITICAL, "Data Tranfer error : status: %x\r\n",
      MMCHS_GET_R(MMCHS_STAT));
      */ 
      if (reset_mmc_lines(mmc_slot, MMCHS_SYSCTL_SRD) == OMAPFLASH_DAL_ERROR)
      {
        /*terminate the loop if SRD is not cleared */
				dbg_printf("Controller soft reset data failed..");
      }

      return OMAPFLASH_DAL_ERROR;	/*In Bustest Procedure no need to send error for DTO */
    }
    else
    {
      return 0;
    }
  }
  else
  {
    return time_left_microsec;
  }
}

U32 data_transfer_complete(U8 mmc_slot)
{
  U32 result = wait_microsec_ex(ONE_SECOND, data_transfer_complete_callback, (void*)mmc_slot);
  if (result == OMAPFLASH_DAL_ERROR)
  {
    return result;
  }

  if (result == 0)
  {
		dbg_printf("DataTransComplete: TC state not set -status: %#x",
      MMCHS_GET_R(mmc_slot, MMCHS_STAT));
    return OMAPFLASH_DAL_ERROR;
  }

  return OMAPFLASH_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : reset_mmc_lines
+------------------------------------------------------------------------------
|
| Description : soft resets the cmd line or data lines
|
| Parameters  : U32 reset_val - (cmd line or data line reset val)
|
| Returns     : S32
+-----------------------------------------------------------------------------*/
S32 reset_mmc_lines(U8 mmc_slot, U32 reset_val)
{
  if(!mmchs_fsetwaitclr_r(mmc_slot, MMCHS_SYSCTL, reset_val, ONE_SECOND))
    return OMAPFLASH_DAL_ERROR;

  return OMAPFLASH_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : set_hsmmc_clk_data_width
+------------------------------------------------------------------------------
|
| Description : Sets the tranfer clk and tranfer data width
|
| Parameters  : U16 card_rca, U8 data_width, U32 transfer_clk
|
| Returns     : S32
+-----------------------------------------------------------------------------*/
S32 set_hsmmc_clk_data_width(U8 mmc_slot, U8 card_type, U16 card_rca, U8 data_width, U32 transfer_clk, U32 ddr)
{
  S32 ret_value = OMAPFLASH_SUCCESS;

  ret_value = is_card_tranfer_state(mmc_slot, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  if (HS_MMC_CARD == card_type)
  {
    if (26000 < transfer_clk)
    {
      ret_value = mmc_send_switch_command(SWITCH_MODE_WR_BYTE, mmc_slot, EXT_CSD_OFFSET(hs_timing), 0x01);

      if (OMAPFLASH_SUCCESS != ret_value)
      {
        return ret_value;
      }
    }
  }

  ret_value = is_card_tranfer_state(mmc_slot, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = mmc_set_data_width(mmc_slot, data_width, ddr);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  ret_value = is_card_tranfer_state(mmc_slot, card_rca);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  /*send clock command */
  ret_value = set_mmc_controller_clk(mmc_slot, transfer_clk);
  if (OMAPFLASH_SUCCESS != ret_value)
  {
    return ret_value;
  }

  return ret_value;
}

/*-----------------------------------------------------------------------------
| Function    : mmc_adma_config
+------------------------------------------------------------------------------
|
| Description : Configure ADMA 
|
| Parameters  : U8 mmc_slot, U32 add, U32 size
|
| Returns     : S32
+-----------------------------------------------------------------------------*/
U32 mmc_adma_config(U8 mmc_slot, U32 addr, U32 size)
{
  T_db *db = (T_db *)(get_config()->data);
  U32 num_desc = 0;
  U32 adma_len; 
  omap_adma2_desc *adma_desc, *first_desc;

  // pointer to ADMA descriptor mem location
  adma_desc = db->adma_desc;
  // current size of ADMA descripto
  adma_len = db->adma_desc_num;

  if(!(MMCHS_GET_R(mmc_slot, MMCHS_CAPA) & MMCHS_CAPA_AD2S)) 
  {
    drv_send_info("ADMA support not available. Use non-DMA mode");
    return OMAPFLASH_ERROR;
  }

  if(size % 512)
  {
    drv_send_info("length should be multiple of Block Length (512 bytes)");
    return OMAPFLASH_ERROR;
  }

  // no. of descriptors needed
  num_desc = (size/MAX_ADMA_DESC_LEN) + 1;
  if(num_desc > adma_len)
  {
    free(adma_desc);
    db->adma_desc = adma_desc = (omap_adma2_desc *)malloc(sizeof(omap_adma2_desc) * num_desc);
    if(adma_desc == NULL)
    {
      drv_send_info("Cannot allocate memory to create ADMA2 descriptor");
      return OMAPFLASH_ERROR;
    }    
    db->adma_desc_num = num_desc;
  }
  first_desc = adma_desc;

  // create ADMA descriptors..
  while(size > 0)
  {
    adma_len = (size > MAX_ADMA_DESC_LEN)? MAX_ADMA_DESC_LEN : size;
    adma_desc->addr_32 = addr;
    adma_desc->attr = (adma_len & 0xFFFF) << 16;
    adma_desc->attr |= (ADMA_ATTR_ACT_TRAN | ADMA_ATTR_VALID);
    size -= adma_len;

    if(size == 0)
    {
      adma_desc->attr |= ADMA_ATTR_END;
      break;
    }
    addr += adma_len;
    adma_desc++;
  }

  MMCHS_SET_R(mmc_slot, MMCHS_ADMASAL, first_desc);

  // Enable ADMA mode
  MMCHS_FSET_R(mmc_slot, MMCHS_CON, MMCHS_CON_DMA_MNS);
  MMCHS_FSET_R(mmc_slot, MMCHS_HCTL, MMCHS_HCTL_DMAS_ADMA2);    

  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : mmc_send_switch_command
+------------------------------------------------------------------------------
| Description : Send a switch command to overwrite a field in EXT_CSD
|
| Parameters  : mmc_slot
|               ext_csd_offset
|               data
|
| Returns     : Result
+----------------------------------------------------------------------------*/
U32 mmc_send_switch_command(U8 mode, U8 mmc_slot, U8 ext_csd_offset, U8 data)
{
  U32 ret;
  U16 mode_offset = mode << 8 | ext_csd_offset;
  U16 data_stuff  = data << 8;
  
  //dbg_printf("Starting switch command: mode %d mmc_slot %d offset %d data %#02X", mode, mmc_slot, ext_csd_offset, data);

  ret = send_mmc_cmd(mmc_slot, MMCHS_SEND_SWITCH, mode_offset, data_stuff);

  if(ret == OMAPFLASH_SUCCESS)  
  {
    U32 response = 0;

    if(send_mmc_cmd(mmc_slot, MMCHS_SEND_STATUS, 0x0004, MMCHS_STUFF_BITS))
    {
      dbg_printf("Status command failed after switch command");
    }
    
    get_mmc_cmd_response(mmc_slot, &response, MMC_RSP1);

    if((response & MMC_CARD_SWITCH_ERROR) == MMC_CARD_SWITCH_ERROR)
    {
      ret = OMAPFLASH_ERROR;
    }
  }

  //dbg_printf("Switch command %s", ret ? "Failed" : "OK");

  return ret;
}
