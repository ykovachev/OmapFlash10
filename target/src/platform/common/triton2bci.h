/*-----------------------------------------------------------------------------
|  Project :  CSST
+------------------------------------------------------------------------------
|             Copyright 2003 Texas Instruments.
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
| Filename: triton2bci.h
| Author : 	
| Purpose: 	This is the h-file for the Triton2 BCI platform dependant low
|           level functions (triton2bci.c)
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL =================================================*/
#ifndef CSST_TRITON2BCI_H
#define CSST_TRITON2BCI_H

/*==== INCLUDES ============================================================*/
#include "triton2_drv.h"
/*==== DEFINES ============================================================*/

/* Error codes specific to BCI tests */
#define BCI_CHARGER_NOTCONNECTED	0xFF
#define BCI_ADC_READ_FAILED			0xF0

/* Backup battery charging control register */
#define BB_CFG			0x6D

/* Current and Voltage threshold settings */
#define BBSEL_25	0x00
#define BBSEL_30	0x04
#define BBSEL_31	0x08
#define BBSEL_32	0x0C
#define BBISEL_25	0x00
#define BBISEL_150	0x01
#define BBISEL_500	0x02
#define BBISEL_1000	0x03

#define BBCH_ENABLE		0x10
#define BBCH_DISABLE	0x00

/* BCI Interrupt registers */
#define BCISIHCTRL	0xC6
#define BCIISR1A	0xB9
#define BCIISR1B	0xBA
#define BCIISR2A	0xBD
#define BCIISR2B	0xBE

/*Main Charge Register offsets(IC2 group - 0x4A) */

#define BCIMDEN			0x74
#define BCIMDKEY		0x75
#define BCIMSTATEC		0x76
#define BCIMSTATEP		0x77
#define BCIVBAT1		0x78
#define BCIVBAT2		0x79
#define BCITBAT1		0x7A
#define BCITBAT2		0x7B
#define BCIICHG1		0x7C
#define BCIICHG2		0x7D
#define BCIVAC1			0x7E
#define BCIVAC2			0x7F
#define BCIVBUS1		0x80
#define BCIVBUS2		0x81

#define BCIMFSTS2		0x82
#define BCIMFSTS3		0x83
#define BCIMFSTS4		0x84


#define BCIMFKEY	0x85
#define BCIMFEN1	0x86
#define BCIMFEN2	0x87
#define BCIMFEN3	0x88
#define BCIMFEN4	0x89

#define BCIMFTH1	 0x8A
#define BCIMFTH2	 0x8B
#define BCIMFTH3	 0x8C
#define BCIMFTH4	 0x8D
#define BCIMFTH5 	 0x8E
#define BCIMFTH6	 0x8F
#define BCIMFTH7	 0x90
#define BCIMFTH8	 0x91
#define BCIMFTH9	 0x92

#define BCITIMER1		0x93
#define BCITIMER2		0x94
#define BCIWDKEY		0x95
#define BCIWD			0x96
#define BCICTL1			0x97
#define BCICTL2			0x98
#define BCIVREF1		0x99
#define BCIVREF2		0x9A
#define BCIIREF1		0x9B
#define BCIIREF2		0x9C
#define BCIPWM2			0x9D
#define BCIPWM1			0x9E
#define BCITRIM1		0x9F
#define BCITRIM2		0xA0
#define BCITRIM3		0xA1
#define BCITRIM4		0xA2
#define BCIVREFCOMB1	0xA3
#define BCIVREFCOMB2	0xA4
#define BCIIREFCOMB1	0xA5
#define BCIIREFCOMB2	0xA6
#define BCIMNTEST1		0xA7
#define BCIMNTEST2		0xA8
#define BCIMNTEST3		0xA9
#define BCIPSTATE		0xAA
#define BCIMFSTS1		0xAB
#define BCITRIM5		0xAC

/* Key codes for the Monitoring functions regsiter acess.*/
#define MFKEY1		0x57
#define MFKEY2		0x73
#define MFKEY3		0x9C
#define MFKEY4		0x3E
#define MFKEY5		0xD2
#define MFKEY6		0x7F
#define MFKEY7		0x6D
#define MFKEY8		0xEA
#define MFKEY9		0xC4
#define MFKEY10		0xBC
#define MFKEY11		0xC3
#define MFKEY12		0xF4
#define MFKEY13		0xE7

/* key code values for the Watch dog timer*/
#define WDKEY1	0xAA
#define WDKEY2	0x55
#define WDKEY3	0xDB
#define WDKEY4	0xBD
#define WDKEY5	0xF3
#define WDKEY6	0x33
/* Register bit values */

/* BCIMFEN2 */
#define ACCHGOVCF  0x80

/* BCIMFEN1 */
#define VBATOV1EN	0x80	
#define VBATOV2EN	0x20
#define VBATOV3EN	0x08
#define VBATOV4EN	0x02

#define VBATOV1CF	0x40	
#define VBATOV2CF	0x20
#define VBATOV3CF	0x80
#define VBATOV4CF	0x02
/* BCIMFEN4 */
#define VBATOVEN    0x2

/* BCIMDKEY */
#define AC_SWCTRL_CHG   0x25  /* Software controlled linear charger with AC */
#define USB_SWCTRL_CHG  0x26  /* Software controlled linear charger with USB */
#define CHG_OFF_MODE    0x2A

/*BCIMFTH3*/
#define ACCHGOVTH_VAC_5P5  0x0C /* reg bits 2:3-ACCHGOVTH[1:0]=00 correspond to 
									VAC=5.5*/
/* BCICTL1 */									
#define TYPEN  0x10
#define ITHEN  0x08

/* Register which is part of the T2 power domain */
#define BOOT_BCI	0x3D

/* BCI charging mode deciding bits */

#define BCIAUTOWEN_SET          0x20
#define CONFIG_DONE_SET         0x10
#define CVENAC_SET              0x04
#define BCIAUTOAC_RESET         0x00
#define BCIAUTOUSB_RESET        0x00

/* To enable the monitoring functions */

#define MEASURE_VBUS_EN         0x04 
#define MEASURE_VBAT_EN         0x02 
#define MEASURE_VAC_EN          0x01 

/* Mask and shift values used for getting CGAIN value from BCICTL1 */
#define BCICTL_CGAIN_MASK   0x20
#define BCICTL_CGAIN_SHIFT  0x05

/* MADC channels */
                             
#define ADC_CHANNEL_0    0
#define ADC_CHANNEL_1    1
#define ADC_CHANNEL_2    2
#define ADC_CHANNEL_3    3                             
#define ADC_CHANNEL_4    4
#define ADC_CHANNEL_5    5
#define ADC_CHANNEL_6    6
#define ADC_CHANNEL_7    7                             
#define ADC_CHANNEL_8    8
#define ADC_CHANNEL_9    9
#define ADC_CHANNEL_10   10
#define ADC_CHANNEL_11   11                             
#define ADC_CHANNEL_12   12
#define ADC_CHANNEL_13   13                             
#define ADC_CHANNEL_14   14
#define ADC_CHANNEL_15   15

/* MADC registers */ 

#define CTRL1 			0x00
#define SW1SELECT_LSB	0x06
#define SW1SELECT_MSB	0x07
#define SW1AVERAGE_LSB	0x08
#define SW1AVERAGE_MSB	0x09
#define CTRL_SW1		0x12
#define GPCH_BASE_ADDR	0x37	/* Channel-0 address */

/* MADC Register fields */

#define MADCON			0x01	/*  Bit to power on the MADC module for SW conversion */
#define	MADC_SW1		0x20	/*  Bit to start the MADC conversion */
#define MADC_EOC_SW1	0x02	/*  MADC conversion completion bit */

/* To convert MADC 10-bit values to corrsponding voltages */

#define CONV_TO_VOLTAGE(value, channel)    ( (((value) * 1.5) / (adc_prescalar_ratio[channel]))/1023)

/*==== TYPES ===============================================================*/

/*==== FUNCTION PROTOTYPES ==================================================*/
S32 bci_reg_read(U8 subaddress,U32 *len, U8 *buff);
S32 bci_reg_write(U8 subaddress,U32 *len, U8 *buff);

S32 battery_voltage_level_detect();
S32 battery_over_voltage_protect();

S32 ac_over_voltage_detect();
S32 ac_vol_threshold_set();
S32 ac_charging_config();


S32 configure_bcimfkey_bci(U8 * mkey_code);

S32 bci_config(U32 dis_addr);
F32 convert_to_current(U16 value,U8 cgain);

S32 enable_mbat_monitor();
S32 disable_mbat_monitor();

S32 get_madc_result(U16 channel_number, U16 *data);
U16 read_madc_conv_result(U16 channel_number);

S32 start_madc_conversion();
void select_madc_channel(U16 channel_number);
void madc_poweron();
S32 configure_i2c_id_for_bci_reg(T_TRITON2_DIS* triton2_dis);
S32 update_precharge_status(T_TRITON2_DIS* triton2_dis);
S32 update_maincharge_status(T_TRITON2_DIS* triton2_dis);
S32 update_bat_monitor_info(T_TRITON2_DIS* triton2_dis);
S32 update_bci_chaging_mode(T_TRITON2_DIS* triton2_dis);
S32 update_mbat_charging_device(T_TRITON2_DIS* triton2_dis);
S32 bci_charge_mode_set(U8 ,T_TRITON2_DIS *);
S32 bci_charge_mode_off(U8 ,T_TRITON2_DIS *);
extern void dl_lazy_delay(unsigned long time_delay);

#endif /* CSST_TRITON2BCI_H */
