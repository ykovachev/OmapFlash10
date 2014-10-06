/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  DAL - UART driver
+------------------------------------------------------------------------------
|             Copyright 2005-2008 Texas Instruments.
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
| Filename:   uart_drv.c
| Author:     Thomas Lund Soehus (tls@ti.com)
| Purpose:    Defines the DAL UART driver
+----------------------------------------------------------------------------*/

#define PRIVATE_UART_H
#define PRIVATE_UART1_H
/*==== INCLUDES =============================================================*/
#include "types.h"
#include "error.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "uart_dis.h"
#include "uart_drv.h"
#include "dl_ifwork.h"
#include "interrupt.h"
#include "irda.h"

#if !defined(SECOND_DNLD_BUILD)   && !defined(OMAP3430MDK) && !defined(USB_DOWNLOAD)
#define IRDA_ENABLE
#endif

void uart_read_to_circ_fifo(T_UART_DIS * uart_dis, U32 * len);
void uart_read_from_circ_fifo(T_UART_DIS * uart_dis, U32 * len, U8 * buf);
void uart_read_from_circ_fifo_cstr(T_UART_DIS * uart_dis, U32 * len, U8 * buf);


/*==== GLOBALS ==============================================================*/

/*==== PUBLIC FUNCTIONS =====================================================*/

/*-----------------------------------------------------------------------------
| Function    : uart_isr()
+------------------------------------------------------------------------------
| Description : UART interrupt service routine.
|
| Parameters  : dis_addr - Pointer to the UART device Information structure.
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_isr(void *dis_addr)
{
    T_UART_DIS *uart_dis = (T_UART_DIS *) dis_addr;
    U32 b;
    U8 irq, err;
    U32 len = 64;

    b = uart_base_addr[uart_dis->sid];

    while (!(UART_REG_IIR(b) & 0x01))
    {
        irq = (UART_REG_IIR(b) & UART_IRQ_MASK);
        switch (irq)
        {
            case UART_LINE_STAT_IRQ:
                if ((uart_dis->uart_err_cb) != NULL)
                {
                    err = (UART_REG_LSR(b));
                    /* OMAPS00067828 - Port replicator workaround
                     * Check for break indication error
                     *
                     * CLO & HEU:
                     * Bit 7 is RX_FIFO_STS if this bit is set it means:
                     * At least one parity error, framing error or break indication in the receiver FIFO.
                     * Bit 7 is cleared when no more errors are present in the FIFO.
                     * In any of these cases the best we can do is to clear the error and continue.
                     */

                    /* The line below is commented because of OMAPS00164590 */

                    if (err & (1 << 7)) /* <-- bit 7 of LHR register is UART error indication flag */
                    {
                        /* Clear interrupt by reading RHR register */
                        UART_REG_RHR(b);
                    }
                    else
                    {
                        uart_dis->uart_err_cb(err);
                    }
                }
                break;
            case UART_RX_IRQ:
            case UART_RX_TIMEOUT_IRQ:

                uart_read_to_circ_fifo(uart_dis, &len);
                if ((uart_dis->uart_rx_cb) != NULL)
                {
                    while (uart_dis->fifo_head != uart_dis->fifo_tail)
                    {
                        uart_dis->uart_rx_cb();
                    }
                }
                break;
            case UART_TX_IRQ:
                if ((uart_dis->uart_tx_cb) != NULL)
                {
                    uart_dis->uart_tx_cb();
                }
                break;
            default:
                break;
        }
    }

    return (CSST_DAL_SUCCESS);
}

/*-----------------------------------------------------------------------------
| Function    : uart_init()
+------------------------------------------------------------------------------
| Description : Initializes the given UART.
|
| Parameters  : init_str - Input Pointer to initialization structure.
|               dis_addr - Output pointer to DIS structure.
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_init(const void *init_str,
    U32 * dis_addr)
{
    S32 ret_val = CSST_DAL_SUCCESS;
    T_UART_INIT_STRUCTURE *uart_init_str;
    T_UART_DIS *uart_dis;

    uart_init_str = (T_UART_INIT_STRUCTURE *) init_str;

    if (uart_init_str->sid > NUM_UARTS)
    {
        return (CSST_DAL_INVALID_SID);
    }

    if ((uart_dis = (T_UART_DIS *) malloc(sizeof(T_UART_DIS))) == NULL)
    {
        *dis_addr = 0;
        return -1;
    }

    uart_dis->pid = uart_init_str->pid;
    uart_dis->sid = uart_init_str->sid;

    /*sHs uart_fif0..... */
    if ((uart_dis->uart_circ_buffer =
         (U8 *) malloc(UART_CIRC_BUFFER_SIZE)) == NULL)
    {
        *dis_addr = 0;
        free(uart_dis);
        return -1;
    }
    uart_dis->fifo_head = 0;
    uart_dis->fifo_tail = 0;
    /*.....sHs uart_fif0 */

    uart_dis->irq_poll = uart_init_str->irq_poll;
    uart_dis->baud_rate = uart_init_str->baud_rate;
    uart_dis->irda_mode = uart_init_str->irda_mode;

    if (uart_init_str->irda_mode != IRDA_MODE)
    {
        uart_config(uart_dis->sid,
                    uart_dis->baud_rate,
                    uart_init_str->data_bits,
                    uart_init_str->stop_bits,
                    uart_init_str->parity_bits, uart_init_str->irq_poll);
    }

    /* The UART module must be initialized before interrupts are enabled */
    if (uart_init_str->irq_poll == UART_IRQ_MODE)
    {
        uart_dis->uart_rx_cb = uart_init_str->uart_rx_cb;
        uart_dis->uart_tx_cb = uart_init_str->uart_tx_cb;
        uart_dis->uart_err_cb = uart_init_str->uart_err_cb;
        dal_interrupt_set_sensitivity(uart_irq_no[uart_dis->sid], INTH_LEVEL);
        /* OMAPS00067828 - Port replicator workaround
         * Need to postpone interrupt enabling till after UART is completely setup
         */
        /*dal_interrupt_add_handler(uart_irq_no[uart_dis->sid], uart_isr, (void *) uart_dis); */
        dal_disable_interrupts();
    }

#if defined(IRDA_ENABLE)
    else if (uart_init_str->irda_mode == IRDA_MODE)
    {
        /* Intialization for the irda mode */
        ret_val = irda_config(uart_dis->sid,
                              uart_dis->baud_rate,
                              uart_init_str->data_bits,
                              uart_init_str->stop_bits,
                              uart_init_str->parity_bits,
                              uart_init_str->irq_poll);

    }
#endif
    /* OMAPS00067828 - Port replicator workaround
     * Need to postpone interrupt enabling till after UART is completely setup
     */
    if (uart_init_str->irq_poll == UART_IRQ_MODE)
    {
        /* Note: This call also enables interrupts */
        dal_interrupt_add_handler(uart_irq_no[uart_dis->sid], uart_isr,
                                  (void *)uart_dis);
    }

    *dis_addr = (U32) uart_dis;

    return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    : uart_deinit()
+------------------------------------------------------------------------------
| Description : De-initializes the given UART.
|
| Parameters  : dis_addr - DIS address
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_deinit(U32 dis_addr)
{
    T_UART_DIS *uart_dis = (T_UART_DIS *) dis_addr;
    S32 ret = CSST_DAL_INVALID_PARAMETERS;
    U32 b;

    b = uart_base_addr[uart_dis->sid];

    switch (UART_REG_MDR1(b) & 0x7) /* Get MDR1[2:0] */
    {
        case UART_16X_MODE:
        case UART_16X_AUTO_BAUD:
        case UART_13X_MODE:
            while ((UART_REG_LSR(b) & (1 << 6) /*TX_SR_E */ ) == 0) ;
            break;
        default:
            break;
    }

    if (uart_dis->irq_poll == UART_IRQ_MODE)
    {
        if (dal_interrupt_remove_handler(uart_irq_no[uart_dis->sid]) !=
            CSST_DAL_SUCCESS)
            return ret;
    }

    /*sHs Changed this for UART FIFO */
    if (uart_dis->uart_circ_buffer != NULL)
    {
        free(uart_dis->uart_circ_buffer);

        if (uart_dis != NULL)
        {
            free(uart_dis);
            ret = CSST_DAL_SUCCESS;
        }
    }
    return ret;
}

/*-----------------------------------------------------------------------------
| Function    : uart_read()
+------------------------------------------------------------------------------
| Description : Reads data from the UART FIFO
|
| Parameters  : dis_addr - DIS address
|               tag      - TAG that specifiec what should be read.
|               len      - Number of bytes to read and actual number of bytes
|                          read
|               buf      - Pointer to buffer where data must be stored.
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_read(U32 dis_addr,
    U32 tag,
    U32 * len,
    U8 * buf)
{
    T_UART_DIS *uart_dis = (T_UART_DIS *) dis_addr;
    S32 ret;

    if (dis_addr == 0)
        return CSST_DAL_INVALID_PARAMETERS;

    switch (tag)
    {
        case UART_USER_DATA:
            if (uart_dis->irda_mode != IRDA_MODE)
            {
                if (uart_dis->irq_poll == UART_IRQ_MODE)
                {
                    uart_read_from_circ_fifo(uart_dis, len, buf);
                }
                else
                {
                    uart_read_data(uart_dis->sid, buf, len);
                }
                ret = CSST_DAL_SUCCESS;
            }
#if defined(IRDA_ENABLE)
            else if (uart_dis->irda_mode == IRDA_MODE)
            {
                read_rhr_irda(uart_dis->sid, buf, len);
            }
#endif
            break;
        case UART_USER_DATA_CSTR:
            if (uart_dis->irda_mode != IRDA_MODE)
            {
                if (uart_dis->irq_poll == UART_IRQ_MODE)
                {
                    uart_read_from_circ_fifo_cstr(uart_dis, len, buf);
                }
                else
                {
                    uart_read_data_cstr(uart_dis->sid, buf, len);
                }
                ret = CSST_DAL_SUCCESS;
            }
#if defined(IRDA_ENABLE)
            else if (uart_dis->irda_mode == IRDA_MODE)
            {
                read_rhr_irda_cstr(uart_dis->sid, buf, len);
            }
#endif
            break;
        default:
            ret = CSST_DAL_INVALID_PARAMETERS;
            break;
    }
    return ret;
}

/*-----------------------------------------------------------------------------
| Function    : uart_write()
+------------------------------------------------------------------------------
| Description : Writes data to the UART FIFO
|
| Parameters  : dis_addr - DIS address
|               tag      - TAG that specifiec what should be read.
|               len      - Number of bytes to read and actual number fo bytes
|                          read
|               buf      - Pointer to buffer where data must be stored.
|
| Returns     : Status code
+----------------------------------------------------------------------------*/
S32 uart_write(U32 dis_addr,
    U32 tag,
    U32 * len,
    U8 * buf)
{
    T_UART_DIS *uart_dis = (T_UART_DIS *) dis_addr;
    S32 ret;

    if (dis_addr == 0)
        return CSST_DAL_INVALID_PARAMETERS;

    switch (tag)
    {
        case UART_USER_DATA:
            if (uart_dis->irda_mode == IRDA_MODE)
            {
#if defined(IRDA_ENABLE)
                irda_write_data(uart_dis->sid, buf, len);
#endif
            }
            else
            {
                uart_write_data(uart_dis->sid, buf, len);
            }

            ret = CSST_DAL_SUCCESS;
            break;
        case UART_START_XMIT_KEY:
            uart_mask_tx_irq(uart_dis->sid, 0);
            ret = CSST_DAL_SUCCESS;
            break;
        case UART_STOP_XMIT_KEY:
            uart_mask_tx_irq(uart_dis->sid, 1);
            ret = CSST_DAL_SUCCESS;
            break;
        case UART_MASK_INTR:
            uart_int_mask_unmask(uart_dis->sid, *buf);
            ret = CSST_DAL_SUCCESS;
            break;
        default:
            ret = CSST_DAL_INVALID_PARAMETERS;
            break;
    }
    return ret;
}

/*This function will be used by the uart_isr Handler*/
void uart_read_to_circ_fifo(T_UART_DIS * uart_dis,
    U32 * len)
{
    U32 templen = *len;
    U32 remlen = UART_CIRC_BUFFER_SIZE - uart_dis->fifo_tail;
    U32 readlen = remlen;
    /*We are not checking for the overflow condition
     */
    if (templen < remlen)
        readlen = templen;

    templen = readlen;
    uart_read_data(uart_dis->sid,
                   (uart_dis->uart_circ_buffer + uart_dis->fifo_tail),
                   &readlen);
    uart_dis->fifo_tail = uart_dis->fifo_tail + (U8)readlen;
    /*Check if we have read the required amount of data
       If we have not read, then there is no point in
       reading further */
    if ((readlen < templen) || (*len <= remlen))
    {
        /*We have not got the required amount of data, so
           no point in proceeding */
        /*The second check is to check if the circular buffer
           comes back to initial position */
        return;
    }
    else
    {
        /*Control reaching here means that
           1. The tail needs to be 0
           2. Still more bytes to be read */
        uart_dis->fifo_tail = 0;
        readlen = *len - remlen;
        uart_read_data(uart_dis->sid,
                       (uart_dis->uart_circ_buffer + uart_dis->fifo_tail),
                       &readlen);
        uart_dis->fifo_tail = uart_dis->fifo_tail + (U8)readlen;
        return;
    }

}

/*This function is used by the Uart_read function*/

static BOOLEAN find_null(T_UART_DIS * uart_dis, U32 * len) 
{
    U32 i = 0;
    while (i < *len) {
        if (uart_dis->uart_circ_buffer[uart_dis->fifo_head+i++]) {
            *len = i;
            return TRUE;
        }
    }
    return FALSE;
}


typedef enum T_mode {bin,cstr,packet} T_mode;
void uart_read_from_circ_fifo_ex(T_UART_DIS * uart_dis, U32 * len, U8 * buf, T_mode mode)
{
    U32 templen = *len;
    U32 remlen = UART_CIRC_BUFFER_SIZE - uart_dis->fifo_head;
    if (uart_dis->fifo_head <= uart_dis->fifo_tail)
    {
        remlen = uart_dis->fifo_tail - uart_dis->fifo_head;
        if (remlen < *len)
            *len = remlen;
        if (mode == cstr) 
            find_null(uart_dis, len);
        memcpy(buf, (uart_dis->uart_circ_buffer + uart_dis->fifo_head), (*len));
        uart_dis->fifo_head = uart_dis->fifo_head + (U8)(*len);
    }
    else
    {
        if (*len < remlen)
        {
            if (mode == cstr) 
                find_null(uart_dis, len);
            memcpy(buf, (uart_dis->uart_circ_buffer + uart_dis->fifo_head), (*len));
            uart_dis->fifo_head = uart_dis->fifo_head + (U8)(*len);
        }
        else
        {
            BOOLEAN truncate = mode == cstr && find_null(uart_dis, &remlen);

            memcpy(buf,
                   (uart_dis->uart_circ_buffer + uart_dis->fifo_head), remlen);
            if (truncate) 
            {
                *len = remlen;
                uart_dis->fifo_head = uart_dis->fifo_head + (U8)(*len);
            }
            else
            {
                uart_dis->fifo_head = 0;
                templen = *len - remlen;
                *len = remlen;

                remlen = uart_dis->fifo_tail - uart_dis->fifo_head;

                if (remlen < templen)
                    templen = remlen;
                truncate = mode == cstr && find_null(uart_dis, &templen);

                memcpy((buf + (*len)),
                       (uart_dis->uart_circ_buffer + uart_dis->fifo_head), templen);

                uart_dis->fifo_head += (U8)templen;

                *len = *len + templen;
            }
        }
    }
}

void uart_read_from_circ_fifo_cstr(T_UART_DIS * uart_dis, U32 * len, U8 * buf)
{
    uart_read_from_circ_fifo_ex(uart_dis, len, buf, cstr);
}

void uart_read_from_circ_fifo(T_UART_DIS * uart_dis, U32 * len, U8 * buf)
{
    uart_read_from_circ_fifo_ex(uart_dis, len, buf, bin);
}

/*==== END OF FILE ==========================================================*/

