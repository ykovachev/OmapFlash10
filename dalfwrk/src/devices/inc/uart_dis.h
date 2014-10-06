/**
 * @file uart_dis.h
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
 * 
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef UART_DIS_H
#define UART_DIS_H

/*==== INCLUDES =============================================================*/
#include "types.h"

/*==== MACROS ===============================================================*/
#define UART_TAG_BASE                 (0)
#define UART_CALLBACK_RX              (UART_TAG_BASE+1)    /* Callback RX */
#define UART_CALLBACK_TX              (UART_TAG_BASE+2)    /* Callback TX */
#define UART_CALLBACK_LSR             (UART_TAG_BASE+3)    /* Callback LSR */
#define UART_IRQ_POLL                 (UART_TAG_BASE+4)    /* IRQ Polling */
#define UART_BAUD_RATE                (UART_TAG_BASE+5)    /* Baud Rate */
#define UART_DATA_BITS                (UART_TAG_BASE+6)    /* Data Bits */
#define UART_PARITY_BITS              (UART_TAG_BASE+7)    /* Parity Bits */
#define UART_STOP_BITS                (UART_TAG_BASE+8)    /* UART Stop Bits */
#define UART_START_XMIT_KEY           (UART_TAG_BASE+9)    /* UART Start XMIT */
#define UART_STOP_XMIT_KEY            (UART_TAG_BASE+10)   /* UART Stop XMIT */
#define UART_USER_DATA                (UART_TAG_BASE+11)   /* User data */
#define UART_MASK_INTR                (UART_TAG_BASE+12)
//#define USB_ENUM_STATUS		          (UART_TAG_BASE+13)
#define UART_USER_DATA_CSTR           (UART_TAG_BASE+14)   /* User data until null */
#define UART_USER_DATA_ACKED          (UART_TAG_BASE+15)   /* User data expect ack after tx */
#define UART_IN_XMIT                  (UART_TAG_BASE+16)   /* state machine is not idle */   
#define UART_INIT_START_XMIT_KEY      (UART_TAG_BASE+17) 
#define UART_CONTINUE_START_XMIT_KEY  (UART_TAG_BASE+18)  


#define UART_USER_DATA_MAX		        (56)

#define POLL_MODE                     (0)
#define UART_IRQ_MODE                 (1)

#define MASK_INTR                     (1)
#define UNMASK_INTR                   (0)

//TODO these are different from those in uart.h
#define UART_BAUDRATE_300             0
#define UART_BAUDRATE_600             1
#define UART_BAUDRATE_1200            2
#define UART_BAUDRATE_2400            3
#define UART_BAUDRATE_4800            4
#define UART_BAUDRATE_9600            5
#define UART_BAUDRATE_14400           6
#define UART_BAUDRATE_19200           7
#define UART_BAUDRATE_28800           8
#define UART_BAUDRATE_38400           9
#define UART_BAUDRATE_57600           10
#define UART_BAUDRATE_115200          11
#define UART_BAUDRATE_230400          12
#define UART_BAUDRATE_460800          13
#define UART_BAUDRATE_921600          14
#define UART_BAUDRATE_3686400         15 //For connecting to ZeBu platform

typedef struct T_uart_baudrate_info{
    U32 value;
    U8 id;
}T_uart_baudrate_info;

#ifdef PRIVATE_DNLD_C
#ifdef ENABLE_CHANGE_BAUDRATE
#define BAUDRATE(baudrate) {baudrate, UART_BAUDRATE_##baudrate}
static const T_uart_baudrate_info baudrates[] = 
{
    BAUDRATE(300),
    BAUDRATE(600),
    BAUDRATE(1200),
    BAUDRATE(2400),
    BAUDRATE(4800),
    BAUDRATE(9600),
    BAUDRATE(14400),
    BAUDRATE(19200),
    BAUDRATE(28800),
    BAUDRATE(38400),
    BAUDRATE(57600),
    BAUDRATE(115200),
    BAUDRATE(230400),
    BAUDRATE(460800),
    BAUDRATE(921600),
    //BAUDRATE(1000000),
    //BAUDRATE(1500000),
    //BAUDRATE(1843200),
    //BAUDRATE(3000000),
    BAUDRATE(3686400),
    //{(U32)-1,UART_NO_BAUD_CHANGE}, 
    //{0,UART_UNSUPORTE_SPEED}
    {0,0}
};
#endif //ENABLE_CHANGE_BAUDRATE
#endif //PRIVATE_DNLD_C


#define UART_DATABITS_5               0x00
#define UART_DATABITS_6               0x01
#define UART_DATABITS_7               0x02
#define UART_DATABITS_8               0x03

#define UART_PARITY_NONE              0x00
#define UART_PARITY_ODD               0x01
#define UART_PARITY_EVEN              0x03
#define UART_PARITY_MARK              0x05
#define UART_PARITY_SPACE             0x07

#define UART_STOPBITS_1               0x00
#define UART_STOPBIT_1_5              0x01
#define UART_STOPBIT_2                0x01


#define UART1_SID                     0
#define UART2_SID                     1
#define UART3_SID                     2

/* Irda modes*/

#define IRDA_MODE                     1
#define NON_IRDA_MODE                 0

#define MDR1_UART_MODE_SIR						((U8)0x01)
#define MDR1_UART_MODE_MIR						((U8)0x04)
#define MDR1_UART_MODE_FIR						((U8)0x05)

/* Baud rate for each mode*/
#define IRDA_SIR_HIGHEST_BUADRATE			((U32) 576000)
#define IRDA_MIR_HIGHEST_BAUDRATE			((U32) 1152000)
#define IRDA_FIR_HIGHEST_BAUDRATE			((U32) 4000000)

#define MAX_NUM                       9
#ifdef PRIVATE_UART_DIS_H
static U32 irda_baud_rate_arr[MAX_NUM] =
{
  2400,
  9600,
  19200,
  38400,
  57600,
  115200,
  576000,
  1152000,
  4000000
};
#endif

/*==== TYPEDEFINES ==========================================================*/
#define UART_CIRC_BUFFER_SIZE	100
typedef struct T_UART_INIT_STRUCTURE
{
    U16 pid;            ///<Primary ID
    U16 sid;            ///<Secondary ID
    U8 callback_poll;   ///<this reflect the internal working of the driver, if it is Poll or Callback based, the driver should all ways use ll_*_callback to talk to link layer 
    U16 baud_rate;      ///<9600, 19200.. 921600bps
    U8 data_bits;       ///<5, 6, 7, or 8 bits (word length), only 8 bit suported by romapi driver 
    U8 parity_bits;     ///<None, even, odd
    U8 stop_bits;       ///<1, 1 1/2, or 2 stop bits, only 1 and 2 stop bits supported by romapi driver
    U8 irda_mode;       ///<irda mode for uart3
    /// set this to true to have 2nd skip re-initialization of UART (in which case all of the above is ignored)
    BOOLEAN reuse_romconnection;
} T_UART_INIT_STRUCTURE;

typedef struct T_UART_DIS
{
    U16 pid;                            /* Primary ID */
    U16 sid;                            /* Secondary ID */
    U8 callback_poll;                        /* Poll or Interrupt based */
    U16 baud_rate;
    U8 irda_mode;						/* irda mode for uart3 */

    /*sHs Newly added*/
    U8 * uart_circ_buffer;
    U8 fifo_head;
    U8 fifo_tail;
    /*sHs add end*/
} T_UART_DIS;

/*====== FUNCTION PROTOTYPES=================================================*/
S32 uart_init(const void *init_str, U32 *dis_addr);
S32 uart_deinit(U32 dis_addr);
S32 uart_read(U32 dis_addr, U32 tag, U32 *len, U8 *buf);
S32 uart_write(U32 dis_addr, U32 tag, U32 *len ,U8 *buf);


#endif /* UART_DIS_H */
/*==== END OF FILE ==========================================================*/

