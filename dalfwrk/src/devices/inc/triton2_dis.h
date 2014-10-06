/*-----------------------------------------------------------------------------
|  Project :  CSST
+------------------------------------------------------------------------------
|             Copyright 2005 Texas Instruments.
|             All rights reserved.
|
|             This file is confidential and a trade secret of Texas
|             Instruments .
|             The receipt of or possession of this file does not convey
|             any rights to reproduce or disclose its contents or to
|             manufacture, use, or sell anything it may describe, in
|             whole, or in part, without the specific written consent of
|             Texas Instruments.
+------------------------------------------------------------------------------
| Filename: triton2_dis.h
| Author  : TII
| Purpose : Header file with general Triton2 definitions for the
|           Device Information Structure (DIS) and Triton2 initialisation
|           structure
+----------------------------------------------------------------------------*/

/*=========================== DECLARATION CONTROL ===========================*/
#ifndef CSST_TRITON2_DIS_H
#define CSST_TRITON2_DIS_H

/*================================ INCLUDES =================================*/
#include "types.h"

/*================================= MACROS ==================================*/

#define DAL_TRITON2_SID 		  0

/* Triton2 Power Resources TAG values */

#define TRITON2_VDD1_VOLTAGE_TAG 	0xA000
#define TRITON2_VDD1_ON_OFF_TAG 	0xA001

#define TRITON2_VDD2_VOLTAGE_TAG 	0xA002
#define TRITON2_VDD2_ON_OFF_TAG 	0xA003

#define TRITON2_VIO_VOLTAGE_TAG 	0xA004
#define TRITON2_VIO_ON_OFF_TAG 		0xA005

#define TRITON2_USB_CHARGE_PUMP_TAG 		0xA006
#define TRITON2_USB_CHARGE_PUMP_ON_OFF_TAG 	0xA007

#define TRITON2_VDAC_VOLTAGE_TAG	0xA008
#define TRITON2_VDAC_ON_OFF_TAG 	0xA009

#define TRITON2_VPLL1_VOLTAGE_TAG 	0xA00A
#define TRITON2_VPLL1_ON_OFF_TAG 	0xA00B

#define TRITON2_VPLL2_VOLTAGE_TAG 	0xA00C
#define TRITON2_VPLL2_ON_OFF_TAG 	0xA00D

#define TRITON2_VMMC1_VOLTAGE_TAG	0xA00E
#define TRITON2_VMMC1_ON_OFF_TAG 	0xA00F

#define TRITON2_VMMC2_VOLTAGE_TAG	0xA010
#define TRITON2_VMMC2_ON_OFF_TAG 	0xA011

#define TRITON2_VSIM_VOLTAGE_TAG	0xA012
#define TRITON2_VSIM_ON_OFF_TAG 	0xA013

#define TRITON2_VAUX1_VOLTAGE_TAG	0xA014
#define TRITON2_VAUX1_ON_OFF_TAG 	0xA015

#define TRITON2_VAUX2_VOLTAGE_TAG	0xA016
#define TRITON2_VAUX2_ON_OFF_TAG 	0xA017

#define TRITON2_VAUX3_VOLTAGE_TAG	0xA018
#define TRITON2_VAUX3_ON_OFF_TAG 	0xA019

#define TRITON2_VAUX4_VOLTAGE_TAG	0xA01A
#define TRITON2_VAUX4_ON_OFF_TAG 	0xA01B

#define TRITON2_HW_STS_TAG          0xA01C

#define TRITON2_PIH_ISR1_TAG        0xB000
#define TRITON2_PIH_ISR2_TAG        0xB001
#define TRITON2_GPIO_DATA_DIR_TAG   0xB002
#define TRITON2_GPIO_DATA_TAG       0xB003
#define TRITON2_PWRON_INT_MASK_TAG  0xB004

#define TRITON2_RES_OFF			0x00	/* Resource in OFF */
#define TRITON2_RES_SLEEP		0x08    /* Resource in SLEEP */
#define TRITON2_RES_ACTIVE		0x0D	/* Resource in ACTIVE */

/* Triton2 Audio Resources TAG values */
/* The TAG offset defined here is same as TSC2101 driver
TAG values. This way we can do dynamic configuation
without changing the driver */

#define TRITON2_TAG_BASE          0

#define TRITON2_MASTER_TAG 					(TRITON2_TAG_BASE + 4)
#define TRITON2_SAMPLINGRATE_TAG	 		(TRITON2_TAG_BASE + 5)
#define TRITON2_CODEC_TS_TAG			    (TRITON2_TAG_BASE + 9)

#define TRITON2_VOLUME_TAG					(TRITON2_TAG_BASE + 10)
#define TRITON2_MIC_TAG						(TRITON2_TAG_BASE + 14)
#define TRITON2_AGC_TAG						(TRITON2_TAG_BASE + 15)
#define TRITON2_SIDETONE_TAG				(TRITON2_TAG_BASE + 16)
#define TRITON2_LOUDSPEAKER_TAG				(TRITON2_TAG_BASE + 17)
#define TRITON2_DAC_TAG						(TRITON2_TAG_BASE + 19)

/* Defines for DIS members values */

/* Values for DIS member master */
#define TRITON2_MASTER			1
#define TRITON2_SLAVE			0

/* Values for DIS member mic */
#define TRITON2_HEADSET_MIC		1
#define TRITON2_HANDSET_MIC		0

/* Values for DIS member loudspeaker */
#define TRITON2_LOUDSPEAKER_ENABLE		1
#define TRITON2_LOUDSPEAKER_DISBLE		0

/* BCI tags */
#define TRITON2_BCI_CHARGER_SEL		0xC001
#define TRITON2_BCI_CHGMODE			0xC002   /* BCI charging mode*/
#define TRITON2_BCI_BATTERY_TYPE	0xC003
#define TRITON2_BCI_CONFIG			0xC004
#define TRITON2_BCI_CHARGE_DISABLE  0xC005
#define TRITON2_BCI_CHARGE_ENABLE   0xC006
#define TRITON2_BCI_VOLTAGE			0xC007
#define TRITON2_BCI_CURRENT			0xC008
#define TRITON2_BCI_TEMPERATURE		0xC009
#define TRITON2_BCI_MONITOR_TAG		0xC00A

/*T2 LED Tags*/

#define TRITON2_LED_NUMBER_TAG 0xD001
#define TRITON2_LED_ENABLE_DISABLE_TAG 0xD002
#define TRITON2_LED_BRIGHTNESS_TAG 0xD003

/*hotdie detector tags*/

#define T2_HOTDIEDECT_CONFIG_TAG 0xE001
#define TRITON2_HOTDIE_INT_MASK_TAG  0xE002
#define TRITON2_HOTDIE_SOFT_INT_TAG 0xE003

/*================================= CONSTS ==================================*/
/*================================= TYPES ===================================*/
/*================================ EXPORTS ==================================*/

/*Initialization structure*/
/* Triton 2 Battery Charger Interface */
typedef struct
{

    U8 charger_sel;    /* AC Charger or USB charger */
    U8 mode;           /* Automatic or Software Controlled*/
    U8 battery_type;

}T_TRITON2_BCI_RESOURCES;

typedef struct
{
    U16 pid;							/* Primary ID */
    U16 sid;                            /* Second ID */
    U32 sampling_rate;                  /* 8 kHz, 11.025 kHz, 12 kHz, 16 kHz,22.05 kHz,
                                        24 kHz, 32 kHz, 44.1 kHz, and 48 kHz*/
	U8  mode;                           /* Voice (PCM) or Audio (I2S)*/
    void (*t2pwron_irq_callback)(void); /* Callback function */
    T_TRITON2_BCI_RESOURCES triton2_bci;
    /* Here fill the required fields */

} T_TRITON2_INIT_STRUCTURE;


#endif /* CSST_TRITON2_DIS_H */

