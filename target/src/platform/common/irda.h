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
| Filename: irda.h
| Author :
| Purpose: This is the h-file for the irda Driver 
+----------------------------------------------------------------------------*/
/*==== DECLARATION CONTROL =================================================*/
#ifndef CSST_IRDA_H
#define CSST_IRDA_H

/*==== INCLUDES ============================================================*/

/*==== DEFINES ============================================================*/
#define LSR_RX_FIFO_EMPTY		                 0x01

#define ACREG_EOT_EN			0x01
#define ACREG_SD_MODE			0x40  /* SD pin is set to low */
#define MDR1_SET_TXIR           0x10  /* TXIR output pin is forced high */
#define EBLR_IR_BOF				0x03
#define BLR_XBOF_TYPE			0x40
#define IER_INTERRUPT_DIS       0x00
#define LSR_RX_FIFO_E			0x01
#define LSR_RX_LB				0x20
/*function prototypes*/
S32 read_rhr_irda(U16 uart_no,U8 * buf, U32*  size);
S32 irda_write_data(U16 uart_no,U8* buf, U32* size);
S32 irda_config(U16 uart_no, U16 baudrate, U8 data, U8 stop, U8 parity, U8 poll_irq);

#endif /*irda.h*/
