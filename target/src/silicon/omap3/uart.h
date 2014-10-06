/**
 * @file  uart.h
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

#ifndef UART_H
#define UART_H

/*==== INCLUDES =============================================================*/

#include <stdarg.h>
#include "types.h"
#include "silicon.h"

/*==== MACROS ===============================================================*/

#ifdef SIMULATION
#ifdef __cplusplus
extern "C" 
#endif
volatile U8 *uart_reg(U32 b,U8 offset);
#define UART_REG(b,offset) (*uart_reg(b,offset))
#else
#define UART_REG(b,offset) (*(volatile U8*)(((U32)(b)) + offset))
#endif

/* The base address for the UART instances are the same on OMAP3/4/5 - only the number of UARTs varies */

#define UART_MODEM1_BASE                  0x4806A000
#define UART_MODEM2_BASE                  0x4806C000
#define UART_MODEM3_BASE                  0x48020000
#define UART_MODEM4_BASE                  0x4806E000
#define UART_MODEM5_BASE                  0x48066000
#define UART_MODEM6_BASE                  0x48068000

#define UART_MODEM_BASE(uart)             UART_MODEM##uart##_BASE

#define UART_IRDA_BASE                    UART_MODEM3_BASE

#define UART_REG_RHR(b)                   (UART_REG(b,0x00))
#define UART_REG_THR(b)                   (UART_REG(b,0x00))
#define UART_REG_DLL(b)                   (UART_REG(b,0x00))
#define UART_REG_DLH(b)                   (UART_REG(b,0x04))
#define UART_REG_IER(b)                   (UART_REG(b,0x04))
#define UART_REG_EFR(b)                   (UART_REG(b,0x08))
#define UART_REG_IIR(b)                   (UART_REG(b,0x08))
#define UART_REG_FCR(b)                   (UART_REG(b,0x08))
#define UART_REG_LCR(b)                   (UART_REG(b,0x0C))
#define UART_REG_MCR(b)                   (UART_REG(b,0x10))
#define UART_REG_XON1(b)                  (UART_REG(b,0x10))
#define UART_REG_XON2(b)                  (UART_REG(b,0x14))
#define UART_REG_LSR(b)                   (UART_REG(b,0x14))
#define UART_REG_MSR(b)                   (UART_REG(b,0x18))
#define UART_REG_TCR(b)                   (UART_REG(b,0x18))
#define UART_REG_XOFF1(b)                 (UART_REG(b,0x18))
#define UART_REG_SPR(b)                   (UART_REG(b,0x1C))
#define UART_REG_TLR(b)                   (UART_REG(b,0x1C))
#define UART_REG_XOFF2(b)                 (UART_REG(b,0x1C))
#define UART_REG_MDR1(b)                  (UART_REG(b,0x20))
#define UART_REG_MDR2(b)                  (UART_REG(b,0x24))
#define UART_REG_SFLSR(b)                 (UART_REG(b,0x28))
#define UART_REG_TXFLL(b)                 (UART_REG(b,0x28))
#define UART_REG_TXFLH(b)                 (UART_REG(b,0x2C))
#define UART_REG_RESUME(b)                (UART_REG(b,0x2C))
#define UART_REG_SFREGL(b)                (UART_REG(b,0x30))
#define UART_REG_RXFLL(b)                 (UART_REG(b,0x30))
#define UART_REG_SFREGH(b)                (UART_REG(b,0x34))
#define UART_REG_RXFLH(b)                 (UART_REG(b,0x34))
#define UART_REG_BLR(b)                   (UART_REG(b,0x38))
#define UART_REG_ACREG(b)                 (UART_REG(b,0x3C))
#define UART_REG_SCR(b)                   (UART_REG(b,0x40))
#define UART_REG_SSR(b)                   (UART_REG(b,0x44))
#define UART_REG_EBLR(b)                  (UART_REG(b,0x48))
#if defined OMAP4 || defined OMAP5
#define UART_REG_UASR(b)                  (UART_REG(b,0x38))
#define UART_DIV_1_6(b)                   (UART_REG(b,0x3C))
#define UART_REG_MVR(b)                   (UART_REG(b,0x50))
#define UART_REG_SYSC(b)                  (UART_REG(b,0x54))
#define UART_REG_SYSS(b)                  (UART_REG(b,0x58))
#define UART_REG_WER(b)                   (UART_REG(b,0x5C))
#define UART_REG_CFPS(b)                  (UART_REG(b,0x60))
#define UART_REG_RXFIFO_LVL(b)            (UART_REG(b,0x64))
#define UART_REG_TXFIFO_LVL(b)            (UART_REG(b,0x68))
#define UART_REG_IER2(b)                  (UART_REG(b,0x6C))
#define UART_REG_ISR2(b)                  (UART_REG(b,0x70))
#define UART_REG_FREQ_SEL(b)              (UART_REG(b,0x74))
#define UART_REG_MDR3(b)                  (UART_REG(b,0x80)) 
#endif
#ifdef OMAP5
#define UART_TX_DMA_THRESHOLD             (UART_REG(b,0x84))
#endif

#define UART_RX_IRQ_EN                    0x01
#define UART_TX_IRQ_EN                    0x02
#define UART_LINE_IRQ_EN                  0x04

#define UART_16X_MODE                     0x00
#define UART_16X_AUTO_BAUD                0x02
#define UART_13X_MODE 			              0x03
#define UART_IRDA_MODE                    0x01
#define UART_RESET_MODE                   0x07

#define UART_IRQ_MASK                     0x3F
#define UART_MODEM_IRQ                    0x00
#define UART_TX_IRQ                       0x02
#define UART_RX_IRQ                       0x04
#define UART_LINE_STAT_IRQ                0x06
#define UART_RX_TIMEOUT_IRQ               0x0C
#define UART_XOFF_IRQ                     0x10
#define UART_CTS_RTS_DSR_RIQ              0x20

#define UART_LSR_ERROR_MASK               0x1C

#define UART_DATA_IN_FIFO                 0x01

#define MDR1_FRAME_EOT                    0x80 /*EOT bit method*/
#define MDR1_SCT                          0x20 /*uart-irda-CIR mode selection*/

#define MCR_LOOPBACK_EN                   0x10 /*loopback enable*/
#define MCR_TCR_TLR_ACCESS                0x40

#define UPPER_BYTE_MASK                   0xFF00
#define LOWER_BYTE_MASK                   0x00FF
#define UART_FCR_FIFO_EN                  0x01 /*enables the transmit and receive
											 fifo*/
#define UART_FCR_DMA_MODE_EN              0x08 /* dma mode */

#define FCR_TX_FIFO_CLR                   0x04
#define FCR_RX_FIFO_CLR                   0x02
#define UART_LCR_CHAR_LENGTH8             0x03 
#define UART_LCR_DIV1		                  0x80    /* Divisor Latch Enable */
#define UARTMOD_LCR_STOP2                 0x04
#define MDR1_UART_MODE_DISABLED           ((U8)0x07)
#define	SCTX_EN                           0x04

#define UART_NONE                         ((U8)-1)
#define UART_USB_INFO                     0
#define UART1                             1
#define UART2                             2
#define UART3                             3
#define UART4                             4
#define UART5                             5

#ifdef OMAP3
#define NUMBER_OF_UARTS                   3
#endif //OMAP3
#ifdef OMAP4
#define NUMBER_OF_UARTS                   4
#endif //OMAP4
#ifdef OMAP5
#define NUMBER_OF_UARTS                   6
#endif //OMAP5

/**@def DEBUG_LOGF a printf like function only active in debug build
 * @see vxprintf for details
 */

#ifdef _MSC_VER
#ifdef DEBUG_UART
#define DEBUG_LOGF_START(format, ...) { uart_config_terminal(TEXT_BOLD_BLUE); uart_config_terminal(TEXT_UNDERLINE); debug_printf(format, __VA_ARGS__); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF_ERROR(format, ...) { uart_config_terminal(TEXT_BOLD_RED); debug_printf(format, __VA_ARGS__); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF_CMD(format, ...)   { uart_config_terminal(TEXT_GREEN); debug_printf(format, __VA_ARGS__); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF_STATS(format, ...) { uart_config_terminal(TEXT_YELLOW); debug_printf(format, __VA_ARGS__); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF(format, ...)       debug_printf(format, __VA_ARGS__)
#else
#define DEBUG_LOGF_START(format, ...) ((void)0)
#define DEBUG_LOGF_ERROR(format, ...) ((void)0)
#define DEBUG_LOGF_CMD(format, ...)   ((void)0)
#define DEBUG_LOGF_STATS(format, ...) ((void)0)
#define DEBUG_LOGF(format, ...)       ((void)0)
#endif
#define NO_DEBUG_LOGF(format, ...)    ((void)0)
#else
#ifdef DEBUG_UART
#define DEBUG_LOGF_START(format...)   { uart_config_terminal(TEXT_BOLD_BLUE); uart_config_terminal(TEXT_UNDERLINE); debug_printf(format); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF_ERROR(format...)   { uart_config_terminal(TEXT_BOLD_RED); debug_printf(format); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF_CMD(format...)     { uart_config_terminal(TEXT_BOLD_GREEN); debug_printf(format); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF_STATS(format...)   { uart_config_terminal(TEXT_BOLD_YELLOW); debug_printf(format); uart_config_terminal(TEXT_NORMAL); } 
#define DEBUG_LOGF(format...)         debug_printf(format)
#else
#define DEBUG_LOGF_START(format...)   ((void)0)
#define DEBUG_LOGF_ERROR(format...)   ((void)0)
#define DEBUG_LOGF_CMD(format...)     ((void)0)
#define DEBUG_LOGF_STATS(format...)   ((void)0)
#define DEBUG_LOGF(format...)         ((void)0)
#endif
#define NO_DEBUG_LOGF(format...)      ((void)0)
#endif

/*==== TYPEDEFINES ==========================================================*/

//TODO these are different from those in uart_dis.h
typedef enum T_uart_baudrate
{
  UART_NO_BAUD_CHANGE          = (0xf0),      /* Don't change the UART baudrate */
  UART_1200                    = (0x0),      	/* UART Baudrate 1200 */
  UART_2400                    = (0x1),      	/* UART Baudrate 2400 */
  UART_4800                    = (0x2),      	/* UART Baudrate 4800 */
  UART_9600                    = (0x3),      	/* UART Baudrate 9600 */
  UART_14400                   = (0x4),      	/* UART Baudrate 14400 */
  UART_19200                   = (0x5),      	/* UART Baudrate 19200 */
  UART_28800                   = (0x6),      	/* UART Baudrate 28800 */
  UART_38400                   = (0x7),      	/* UART Baudrate 38400 */ 
  UART_57600                   = (0x8),      	/* UART Baudrate 57600 */
  UART_115200                  = (0x9),       /* UART Baudrate 115200 */
  UART_230400                  = (0xa),       /* UART Baudrate 230400 */
  UART_460800                  = (0xb),       /* UART Baudrate 460800 */
  UART_921600                  = (0xc),       /* UART Baudrate 921600 */
  UART_1000000                 = (0xd),       /* UART Baudrate 1000000 */
  UART_1500000                 = (0xe),       /* UART Baudrate 1500000 */
  UART_1843200                 = (0xf),       /* UART Baudrate 1843200 */
  UART_3000000                 = (0x10),      /* UART Baudrate 3000000 */
  UART_3686400                 = (0x11),      /* UART Baudrate 3686400 */
  UART_UNSUPORTE_SPEED         = (0xf1)       /* UART Baudrate 3686400 */
} T_uart_baudrate;

#define UART_DATABITS_5               0x00
#define UART_DATABITS_6               0x01
#define UART_DATABITS_7               0x02
#define UART_DATABITS_8               0x03

#define UART_STOPBITS_1               0x00
#define UART_STOPBIT_1_5              0x01
#define UART_STOPBIT_2                0x01

#define UART_PARITY_NONE              0x00
#define UART_PARITY_ODD               0x01
#define UART_PARITY_EVEN              0x03
#define UART_PARITY_MARK              0x05
#define UART_PARITY_SPACE             0x07

#define POLL_MODE                     (0)
#define UART_IRQ_MODE                 (1)

/*====== FUNCTION PROTOTYPES=================================================*/
#ifdef __cplusplus
extern "C" {
#endif

//S32 uart_read_data(U16 uart_no, U8 *buf, U32 *size);
//S32 uart_read_data_cstr(U16 uart_no, U8 * buf, U32 * size);
//S32 uart_write_data(U16 uart_no, const U8 *buf, U32 *size);

//int vuprintf(U16 uart_no, const char *format, va_list arg_ptr);
//int uprintf(U16 uart_no, const char *format, ...);

#define TEXT_NORMAL             "0m"
#define TEXT_BOLD               "1m"
#define TEXT_UNDERLINE          "4m"
#define TEXT_INVERSE            "7m"
#define TEXT_BLACK              "30m"
#define TEXT_RED                "31m"
#define TEXT_BOLD_RED           "31;1m"
#define TEXT_GREEN              "32m"
#define TEXT_BOLD_GREEN         "32;1m"
#define TEXT_YELLOW             "33m"
#define TEXT_BOLD_YELLOW        "33;1m"
#define TEXT_BLUE               "34m"
#define TEXT_BOLD_BLUE          "34;1m"
#define TEXT_MAGENTA            "35m"
#define TEXT_BOLD_MAGENTA       "35;1m"
#define TEXT_CYAN               "36m"
#define TEXT_BOLD_CYAN          "36;1m"
#define TEXT_WHITE              "37m"
#define TEXT_BOLD_WHITE         "37;1m"
#define TEXT_CLEAR              "2J"


#ifdef DEBUG_UART
S32  uart_config(U16 uart_no, U16 baudrate, U8 data, U8 stop, U8 parity, U8 poll_irq);
void debug_vxprintf(int flags, const char *format, va_list arg_ptr);
void debug_vprintf(const char *format, va_list arg_ptr);
void debug_printf(const char *format, ...);
void open_debug_uart(U8 uart_no, U8 baudrate);
void close_debug_uart(U8 uart_no);
void set_debug_uart(U8 uart_no);
S32 uart_config_terminal(char * attribute);
#endif //DEBUG_UART

#if 0
void uart_mask_tx_irq(U16 uart_no, U8 mask);
void uart_int_mask_unmask(U16 uart_no, U8 set_flag);
#endif

#ifdef __cplusplus
}
#endif
#endif /* UART_H */
/*==== END OF FILE ==========================================================*/

