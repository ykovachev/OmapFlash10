/**
 * @file triton2_drv.h
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
 * 	This is the h-file for the Triton2 Driver (triton2_drv.c)
 * 
 */

/*==== DECLARATION CONTROL =================================================*/

#ifndef TRITON2_DRV_H
#define TRITON2_DRV_H

/*==== INCLUDES ============================================================*/

#include "triton2_dis.h"
#include "emmc_drv.h"

/*==== DEFINES ============================================================*/

/* Power Resource values */

/* VDD1 voltage values */

#define VDD1_1P3		56	/* VSEL = 56 to get the 1.3v (VDD1 = VSEL*12.5mV + 600mV) */

/* VDD2 voltage values */

#define VDD2_1P3		56	/* VSEL = 56 to get the 1.3v (VDD2 = VSEL*12.5mV + 600mV) */

/* VIO possible voltage values */

#define VIO_1P8		0x00
#define VIO_1P85	0x01

/* VDAC possible voltage values */

#define VDAC_1P2	0x00
#define VDAC_1P3	0x01
#define VDAC_1P8	0x02	/* or 0x03 */

/* VPLL1 possible voltage values */

#define VPLL1_1P0	0x00
#define VPLL1_1P2	0x01
#define VPLL1_1P3	0x02
#define VPLL1_1P8	0x03
#define VPLL1_2P8	0x04
#define VPLL1_3P0	0x05

/* VPLL2 possible voltage values */

#define VPLL2_0P7	0x00
#define VPLL2_1P0	0x01
#define VPLL2_1P2	0x02
#define VPLL2_1P3	0x03
#define VPLL2_1P5	0x04
#define VPLL2_1P8	0x05
#define VPLL2_1P85	0x06
#define VPLL2_2P5	0x07
#define VPLL2_2P6	0x08
#define VPLL2_2P8	0x09
#define VPLL2_2P85	0x0A
#define VPLL2_3P0	0x0B
#define VPLL2_3P15	0x0C

/* VMMC1 possible voltage values */

#define VMMC1_1P85	0x00
#define VMMC1_2P85	0x01
#define VMMC1_3P0	0x02
#define VMMC1_3P15	0x03

/* VMMC2 possible voltage values */

#define VMMC2_1P0	0x01
#define VMMC2_1P2	0x02
#define VMMC2_1P3	0x03
#define VMMC2_1P5	0x04
#define VMMC2_1P8	0x05
#define VMMC2_1P85	0x06
#define VMMC2_2P5	0x07
#define VMMC2_2P6	0x08
#define VMMC2_2P8	0x09
#define VMMC2_2P85	0x0A
#define VMMC2_3P0	0x0B
#define VMMC2_3P15	0x0C	/* or 0x0D or 0x0E or 0x0F */

/* VSIM possible voltage values */

#define VSIM_1P0	0x00
#define VSIM_1P2	0x01
#define VSIM_1P3	0x02
#define VSIM_1P8	0x03
#define VSIM_2P8	0x04
#define VSIM_3P0	0x05	/*  or 0x06 or 0x07 */

/* VAUX1 possible voltage values */

#define VAUX1_1P5	0x00
#define VAUX1_1P8	0x01
#define VAUX1_2P5	0x02
#define VAUX1_2P8	0x03
#define VAUX1_3P0	0x04	/* or 0x05 or 0x06 or 0x07 */

/* VAUX2 possible voltage values */

/* VAUX2 values for TWL5030 */
#define VAUX2_1P7	0x01
#define VAUX2_1P9	0x02
#define VAUX2_1P3	0x03
#define VAUX2_1P5	0x04
#define VAUX2_1P8	0x05
#define VAUX2_2P0	0x06
#define VAUX2_2P5	0x07
#define VAUX2_2P1	0x08
#define VAUX2_2P8	0x09
#define VAUX2_2P2	0x0A
#define VAUX2_2P3	0x0B
#define VAUX2_2P4	0x0C

/* VAUX2 values for TWL4030 */
#define VAUX2_1P0	0x01
#define VAUX2_1P2	0x02
#define VAUX2_1P3	0x03
#define VAUX2_1P5	0x04
#define VAUX2_1P8	0x05
#define VAUX2_1P85	0x06
#define VAUX2_2P5	0x07
#define VAUX2_2P6	0x08
#define VAUX2_2P8	0x09
#define VAUX2_2P85	0x0A
#define VAUX2_3P0	0x0B
#define VAUX2_3P15	0x0C



/* VAUX3 possible voltage values */

#define VAUX3_1P5	0x00
#define VAUX3_1P8	0x01
#define VAUX3_2P5	0x02
#define VAUX3_2P8	0x03
#define VAUX3_3P0	0x04

/* VAUX4 possible voltage values */

#define VAUX4_0P7	0x00
#define VAUX4_1P0	0x01
#define VAUX4_1P2	0x02
#define VAUX4_1P3	0x03
#define VAUX4_1P5	0x04
#define VAUX4_1P8	0x05
#define VAUX4_1P85	0x06
#define VAUX4_2P5	0x07
#define VAUX4_2P6	0x08
#define VAUX4_2P8	0x09
#define VAUX4_2P85	0x0A
#define VAUX4_3P0	0x0B
#define VAUX4_3P15	0x0C

/* Triton2 Power Source IDs */

#define VAUX1_RES_ID 1
#define VAUX2_RES_ID 2
#define VAUX3_RES_ID 3
#define VAUX4_RES_ID 4
#define VMMC1_RES_ID 5
#define VMMC2_RES_ID 6
#define VPLL1_RES_ID 7
#define VPLL2_RES_ID 8
#define VSIM_RES_ID  9
#define VDAC_RES_ID  10
#define VIO_RES_ID   14
#define VDD1_RES_ID  15
#define VDD2_RES_ID  16

/* power on interrupt */
#define PWRON_INT  0x01
/* hot die  interrupt */
#define HOT_DIE_INT 0x10




#define POWER_MANAGEMENT_DEDICATED_INT  0x20
#define USB_DEDICATED_INT				0x10
#define MADC_DEDICATED_INT			    0x08
#define BCI_DEDICATED_INT 				0x04
#define KEYPAD_DEDICATED_INT            0x02
#define GPIO_DEDICATED_INT				0x01

#define NUM_OF_BITS_IN_TRITON2_REG      0x8


/* Precharge FSM states (Reg - BCIPSTATE) */

#define NO_CHARGING_DEV     0x0
#define STANDBY_MODE        0x1
#define BATTERY_DETECT      0x2
#define BATTERY_OPEN        0x3
#define PRECHARGE_PROTECTION 0x4
#define PRECHARGE_OFF       0x5
#define AC_CHARGER_OV       0x6
#define USB_CHARGER_OV      0x7
#define AC_SLOW_CHARGE      0x9
#define AC_FAST_CHARGE      0xA
#define AC_CONSTANT_VOLT    0xB
#define USB_SLOW_PRECHARGE  0xD
#define USB_FAST_PRECHARGE  0xE


#define PRECH_CURRENT_STATE_MASK	0x0F
#define PRECH_PREV_STATE_MASK		0xF0
#define PRECH_MONI_INFO_MASK1       0x3F
#define PRECH_MONI_INFO_MASK2       0x60

#define MAINC_MONI_INFO_MASK       0x000000FF


#define MADC_10BIT_MSB_MASK         0x03
#define MADC_10BIT_MSB_SHIFT        0x08

/* Precharge Monitoring information (Regs - BCIMFSTS1, BCITRIM5)*/

#define AC_CHARGER_PRESENCE_IN_PRECH    0x20
#define USB_CHARGER_PRESENCE_IN_PRECH   0x10
#define BAT_PRESENCE_IN_PRECH           0x08
#define VBUS_OVER_VOLT                  0x04
#define FAST_PRECHARGE                  0x02
#define SLOW_PRECHARGE                  0x01
#define AC_OVER_VOLT                    0x40	/* This value is shifted by one bit */
#define USB_OVER_VOLT                   0x80	/* This value is shifted by one bit */

/* Main Charge Monitoring information (Regs - BCIMFSTS2,BCIMFSTS3,BCIMFSTS4) */

#define BAT_PRESENCE_STAT			0x40
#define BAT_VOLT_DET_STAT4			0x20
#define BAT_VOLT_DET_STAT3			0x10
#define BAT_VOLT_DET_STAT2			0x08
#define BAT_VOLT_DET_STAT1			0x04
#define VBUS_OV_STAT    			0x02
#define ACC_OV_STAT    			    0x01

#define BAT_PRESENCE_STAT			0x40
#define ACC_PRESENCE_STAT		    0x20
#define BAT_TEMP_OUT_RANGE1_STAT	0x10
#define BAT_TEMP_OUT_RANGE2_STAT	0x08
#define CHARGE_CURRENT_END_STAT    	0x04
#define CHARGE_CURRENT_LOW_STAT     0x02
#define CHARGE_CURRENT_HIGH_STAT    0x01

#define SYSACTIV_STAT   			0x10
#define USB_SUSPND_ACTIV_STAT  		0x08
#define USBC_CARKT_PRESENCE_STAT	0x04
#define USBH_MCPC_PRESENCE_STAT	    0x02
#define HEAT_SAVE_DETECT_STAT		0x01

/* BCI Charger modes */

#define	BCI_MODE_AUTOMATIC			        0x01
#define	BCI_MODE_SW_CONTROLLED_LINEAR       0x02
#define	BCI_MODE_SW_CONTROLLED_PULSED       0x03
#define	BCI_MODE_SW_CONTROLLED_ACCESSORY    0x04
#define	BCI_MODE_CONSTANT_VOLT              0x05

/* BCI charging modes (Reg - BCIMDEN) */

#define	BCIMDEN_LINCHEN                     0x10
#define	BCIMDEN_PWMEN                       0x08
#define	BCIMDEN_ACCSUPEN                    0x04
#define	BCIMDEN_ACPATHEN                    0x02
#define	BCIMDEN_USBPATHEN                   0x01

#define	BOOT_BCIAUTOUSB 			        0x02
#define	BOOT_BCIAUTOAC   			        0x01
#define	BOOT_CVENAC        			        0x04

/* Battery Chargers */

#define	USB_CHARGER	0x01
#define	AC_CHARGER	0x02

/* BCI <=> MADC Monitor status */

#define BCI_MONITORING_ENABLE   0x1
#define BCI_MONITORING_DISABLE  0x0

/* BCI battery monitoring enable bits. (Regs - BCIMFEN1, BCIMFEN2, BCIMFEN3, BCIMFEN4) */

#define VBAT_VOLT_LVL_DET1_EN           0x80
#define VBAT_VOLT_LVL_DET1_CF           0x40
#define VBAT_VOLT_LVL_DET2_EN           0x20
#define VBAT_VOLT_LVL_DET2_CF           0x10
#define VBAT_VOLT_LVL_DET3_EN           0x08
#define VBAT_VOLT_LVL_DET3_CF           0x04
#define VBAT_VOLT_LVL_DET4_EN           0x02
#define VBAT_VOLT_LVL_DET4_CF           0x01

#define AC_CHARGER_OV_MONITOR_EN        0x80
#define AC_CHARGER_OV_MONITOR_CF        0x40
#define VBUS_OV_MONITOR_EN              0x20
#define VBUS_OV_MONITOR_CF              0x10
#define BAT_TEMP_OV1_MONITOR_EN         0x08
#define BAT_TEMP_OV1_MONITOR_CF         0x04
#define BAT_TEMP_OV2_MONITOR_EN         0x02
#define BAT_TEMP_OV2_MONITOR_CF         0x01

#define BAT_EOC_MONITOR_EN              0x80
#define BAT_EOC_MONITOR_CF              0x40
#define BAT_CHARGE_CURRENT_MONITOR_EN   0x20
#define BAT_CHARGE_CURRENT_MONITOR_CF   0x10
#define BAT_OC_MONITOR_EN               0x08
#define BAT_OC_MONITOR_CF               0x04

#define BAT_STSMCHG_EN					0x04
#define BAT_OV_MONITOR_EN               0x02
#define BAT_OV_MONITOR_CF               0x01

/* BCI main charger FSM states (Regs - BCISTATEC,BCISTATEP) */

#define	MC_NO_CHARGING_DEV      0x00
#define	OFF_MODE                0x01
#define	MC_STANDBY_MODE         0x02
#define	OPEN_BATTERY_MODE       0x03

#define	CONSTANT_VOLT_MODE		0x21

#define	QUICK_CHARGE_AC1		0x22
#define	QUICK_CHARGE_AC2		0x23
#define	QUICK_CHARGE_AC3		0x24
#define	QUICK_CHARGE_AC4		0x25
#define	QUICK_CHARGE_AC5		0x26
#define	QUICK_CHARGE_AC6		0x27

#define	CHARGE_STOP_AC1			0x28
#define	CHARGE_STOP_AC2			0x29
#define	CHARGE_STOP_AC3			0x2A

#define	CHARGE_AC_COMP1			0x2B
#define	CHARGE_AC_COMP2			0x2C
#define	CHARGE_AC_COMP3			0x2D
#define	CHARGE_AC_COMP4			0x2E

#define	AC_ADAPTER_OV			0x2F

#define	QUICK_CHARGE_USB1		0x12
#define	QUICK_CHARGE_USB2		0x13
#define	QUICK_CHARGE_USB3		0x14
#define	QUICK_CHARGE_USB4		0x15
#define	QUICK_CHARGE_USB5		0x16
#define	QUICK_CHARGE_USB6		0x17

#define	CHARGE_STOP_USB1		0x18
#define	CHARGE_STOP_USB2		0x19
#define	CHARGE_STOP_USB3		0x1A

#define	CHARGE_USB_COMP1		0x1B
#define	CHARGE_USB_COMP2		0x1C
#define	CHARGE_USB_COMP3		0x1D
#define	CHARGE_USB_COMP4		0x1E

#define	USB_ADAPTER_OV			0x1F


#define TRITON2_PWR_IMR1_REG		(0x2F)	/* register to mask the Power interrupts */
#define TRITON2_STS_HW_CONDITIONS_REG   (0x45)

/*Triton2 LED interface*/


/* LED registers are located in this slave address */

#define T2_I2C_LED_ADDR_GROUP	0x4A	/* This should be checked */
#define T2LED_PWROFF_VALUE 127

#define LEDA 1
#define LEDB 2

#define LED_ENABLE 1
#define LED_DISABLE 0

/* Physical address of LED registers */

#define	T2_LEDEN				0xEE
#define	T2_PWMAON				0xEF
#define	T2_PWMAOFF				0xF0
#define T2_PWMBON				0xF1
#define T2_PWMBOFF				0xF2


/* LEDEN Register Bits */
#define PWM_LENGTHB 			0x80
#define PWM_LENGTHA 			0x40
#define LEDBPWM					0x20
#define LEDAPWM 				0x10
#define LEDBEXT 				0x08
#define LEDAEXT 				0x04
#define LEDBON 					0x02
#define LEDAON					0x01
/* clear LEDEN register vale */
#define LEDOFF					0x00

typedef enum
{
  SUB_LED_BRIGHTNESS_0_PERCNTAGE = 0,
  SUB_LED_BRIGHTNESS_25_PERCNTAGE = 25,
  SUB_LED_BRIGHTNESS_50_PERCNTAGE = 30,
  SUB_LED_BRIGHTNESS_75_PERCNTAGE = 75,
  SUB_LED_BRIGHTNESS_100_PERCNTAGE = 100
} T_SUB_LED_BRIGHTNESS;



/*Triton2 hotdie detector interface*/

#define T2_HOTDIE_ADDR_GROUP 0x4B

#define PWR_ISR1 			0x2E
#define PWR_IMR1 	 		0x2F
#define PWR_ISR2  			0x30
#define PWR_IMR2 			0x31
#define PWR_SIR 			0x32
#define PWR_EDR1 			0x33
#define PWR_EDR2 			0x34
#define PWR_SIH_CTRL 		0x35
#define CFG_P123_TRANSITION	0x39
#define MISC_CFG            0x68

#define T2_UNLOCK_ADDR_GROUP 0x49
#define   PIH_SIR 		 	0x83
#define  UNLOCK_TEST_REG 	0x97

#define T2_PMBR1_ADDR_GROUP                  0x49
#define PMBR1_REG                            0x92
#define PMBR1_GPIO7_VIBRASYNC_PWM1_MASK		(0x3<<4)
#define PMBR1_PWM1_ENABLE                   (0x3<<4)
#define PMBR1_GPIO6_CLKOK_PWM0_MUTE			(0x3<<2)
#define PMBR1_PWM0_ENABLE                   (0x1<<2)

#define T2_GPBR1_ADDR_GROUP                  0x49
#define GPBR1_REG                            0x91
#define GPBR1_PWM0_MASK                      0x05
#define GPBR1_PWM1_MASK                      0x0A

#define T2_PWMi_ADDR_GROUP                   0x4A
#define PWM0_ON_REG                          0xF8
#define PWM0_OFF_REG                         0xF9
#define PWM1_ON_REG                          0xFB
#define PWM1_OFF_REG                         0xFC
/*==== TYPES ===============================================================*/

/* Prototype Functions */
/* These four will be registered with DAL */

S32 triton2_init (const void *init_str, U32 * dis_addr);
S32 triton2_read (U32 dis_addr, U32 tag, U32 * size, U8 * buffer);
S32 triton2_write (U32 dis_addr, U32 tag, U32 * size, U8 * buffer);
S32 triton2_deinit (U32 dis_addr);

/* Prototype Functions */

typedef struct
{
  U8 i2caddr;			/* Triton2 I2C group ID */
  U8 vdac;
  U8 vdac_stat;
  U8 vpll1;
  U8 vpll1_stat;
  U8 vpll2;
  U8 vpll2_stat;
  U8 vmmc1;
  U8 vmmc1_stat;
  U8 vmmc2;
  U8 vmmc2_stat;
  U8 vsim;
  U8 vsim_stat;
  U8 vmic1;
  U8 vmic1_stat;
  U8 vmic2;
  U8 vmic2_stat;
  U8 vhsmic;
  U8 vhsmic_stat;
  U8 vaux1;
  U8 vaux1_stat;
  U8 vaux2;
  U8 vaux2_stat;
  U8 vaux3;
  U8 vaux3_stat;
  U8 vaux4;
  U8 vaux4_stat;
} T_TRITON2_POWER_RESOURCES;

/* Triton2 audio DIS */
typedef struct
{

  U8 i2caddr;			/* Triton2 I2C group ID     */
  U32 volume;			/* In DB                                        */
  U8 mic;			/* 0=headset mic;
				   1=handset mic                        */
  U8 loudspeaker;		/* 0=off, 1=on                          */
  U8 agc;			/* 0=disabled,1=enabled         */
  U8 sidetone;			/* 0=disabled,1=enabled         */

  U8 keyclick_freq;

  U8 dac;

} T_TRITON2_AUDIO_DIS;

typedef enum
{
  TRITON2_ID1 = 0x48,		/* Genenarl purpose I2C bus addressing */
  TRITON2_ID2 = 0x49,
  TRITON2_ID3 = 0x4A,
  TRITON2_ID4 = 0x4B
} TRITON2_IDS;

typedef struct
{
  U8 gpio_pin_num;
  U8 gpio_data_dir;
  U8 gpio_data;
} T_TRITON2_GPIO_STRUCTURE;

typedef struct
{
  U8 led_no;
  U8 led_enable_disable_flag;
  U8 led_brightness;
} T_TRITON2_LED_STRUCTURE;

typedef struct
{
  U8 cur_state;			/* Current state of Precharge FSM */
  U8 prev_state;		/* Previous state of Precharge FSM */
  U8 charging_monitor_status;	/* BCI Precharge Monitor status information */

} T_BAT_PRECHARGE;

typedef struct
{
  U8 cur_state;			/* Current state of Main charge FSM */
  U8 prev_state;		/* Previous state of Main charge FSM */
  U8 charging_monitor_status1;	/* BCI Main charge Monitor status information1 */
  U8 charging_monitor_status2;	/* BCI Main charge Monitor status information2 */
  U8 charging_monitor_status3;	/* BCI Main charge Monitor status information3 */
} T_BAT_MAIN_CHARGE;

typedef struct
{
  F32 battery_voltage;		/* Battery voltage monitored by MADC */
  F32 charging_current;		/* Battery charging current monitored by MADC */
  F32 battery_temperature1;	/* Battery temperature1 monitored by MADC */
  F32 battery_temperature2;	/* Battery temperature2 monitored by MADC */
  F32 battery_type;		/* Battery type monitored by MADC */
  F32 usb_charger_voltage;	/* USB charger voltage monitored by MADC */
  F32 ac_charger_voltage;	/* AC charger voltage monitored by MADC */

} T_BAT_MONITOR;

/* T2 BCI device information structure */

typedef struct
{
  U8 bci_charging_mode;		/* Charging mode of BCI (SW controlled,Automatic,constant voltage etc) */
  U8 charging_dev;		/* Charger device */
  T_BAT_PRECHARGE pcharge;
  T_BAT_MAIN_CHARGE mcharge;
  T_BAT_MONITOR monitor;

} T_TRITON2_MBAT_CHARGER;

typedef struct
{
  F32 voltage;
} T_TRITON2_BACKUPBAT_MONITOR;

typedef struct
{
  U8 charging_status;
  T_TRITON2_BACKUPBAT_MONITOR monitor;
} T_TRITON2_BACKUPBAT_CHARGER;

/* Triton2 DIS structure */

typedef struct
{
  T_TRITON2_INIT_STRUCTURE triton2_initstr;

  T_TRITON2_POWER_RESOURCES triton2_power;

  T_TRITON2_AUDIO_DIS triton2_audio;

  T_TRITON2_MBAT_CHARGER mbat_charger;

  T_TRITON2_BACKUPBAT_CHARGER backupbat_charger;

  U32 i2c_handle;

  T_TRITON2_LED_STRUCTURE triton2_led;

} T_TRITON2_DIS;

typedef enum
{
  tLeftLS = 0,
  tRightLS,
  tHeadSet,
  tSterioJack,
  tEarphone
} tOutputs;

typedef enum
{
  tMainMic = 0,			// Mono
  tSubMicMainMic,		// Stereo -- The main mic used in conjunction with the sub mic to form a stereo record source.
  tHeadsetMic,			// Mono
  tAUX_FM,			// Stereo
  tDIG_MIC_0,			// Stereo
  tDIG_MIC_1			// Stereo
} tMics;

S32 triton2_i2c_write (T_TRITON2_DIS * triton2_dis, U32 offset, U32 size,
		       U8 data);
S32 triton2_isr (void *data);	/* Triton2 interrupt service routine */
void t2_pwr_isr (void *data);	/* Interrupt servce routine to handle power on interrupt */
S32 confgure_t2_pwrbutton_int (U32 triton2_dis);

S32 configure_triton2_leds (U32 triton2led_dis);
U8 calculate_pwmontime (U8 brightness_value);

S32 configure_hotdiedetector (U32 t2hotdie_dis, U8 hot_die_f_r,
			      U8 hot_die_thresh);
S32 configure_i2c_id_for_power_reg (T_TRITON2_DIS * triton2_dis);
S32 configure_i2c_id_for_audio_reg (T_TRITON2_DIS * triton2_dis);
void update_triton2_groupids (T_TRITON2_DIS * triton2_dis);
U16 form_singular_power_bus_message (U8 dev_grp, U8 res_id, U8 res_state);
S32 send_singular_pb_message (T_TRITON2_DIS * triton2_dis, U8 power_res_id,
			      U8 res_state);
S32 select_power_res_volt (T_TRITON2_DIS * triton2_dis, U8 res_grp_reg_offset,
			   U8 reg_data);
S32 configure_i2c_id_for_triton2_addr_group2 (T_TRITON2_DIS * triton2_dis);
void triton2_interrupt_init (void);
S32 control_triton2_pwr_resource (T_TRITON2_DIS * triton2_dis,
				  U8 res_grp_reg_offset, U8 resource_state);


/* Globals */
#ifdef PRIVATE_TRITON_2_DRV_H
static U8 volume_set[21] = {
  0x93, 0x93, 0x93, 0x8B, 0x83, 0x7B, 0x73, 0x6B, 0x63, 0x5B,
  0x53, 0x4B, 0x43, 0x3B, 0x33, 0x2B, 0x23, 0x1B, 0x13, 0x0B, 0x03
};
#endif

#ifdef EMMC_DRV
//extern U32 triton2_dis_addr;
//extern U32 i2c_dis_addr;

#define triton2_dis_addr get_driver_data()->triton2_handle
#define i2c_dis_addr get_driver_data()->i2c_handle

#define triton2_handle_ triton2 //triton2_handle -> triton2_init
#define i2c_device_handle_ i2c //i2c_device_handle -> i2c_init
#define triton2_i2c_handle_ i2c //triton2_i2c_handle -> i2c_init

#define concat2a(a,b) (a##_##b)
#define concat2(a,b) concat2a(a,b)
#define concat(a,b) concat2(a##_,b)
#define dal_init(init_str,handle) concat(*handle,init)(init_str,concat(handle,dis_addr))
#define dal_deinit(handle) concat(handle,deinit)(concat(handle,dis_addr))
#define dal_read(handle,tag,size,buffer) concat(handle,read)(concat(handle,dis_addr),tag,size,buffer)
#define dal_write(handle,tag,size,buffer) concat(handle,write)(concat(handle,dis_addr),tag,size,buffer)
#endif //EMMC_DRV

#endif /* TRITON2_DRV_H */
