/**
 * @file i2c_drv.c
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
 * i2c functions used by the dal layer
 * 
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef I2C_DRV_C
#define I2C_DRV_C

/*==== INCLUDES ==============================================================*/
#include "csst_tgt.h"
#include "dbg_ex.h"
#include <stdlib.h>
#include "i2c.h"
#include "i2c_dis.h"
#include "i2c_drv.h"

#ifndef TRACE_I2C
#define I2C_DBG_PRINT DO_DBG_PRINT
#else
#define I2C_DBG_PRINT NO_DBG_PRINT
#endif

/*=====GLOBALS================================================================*/
/*reg base for i2c module*/
//volatile U32 i2c_reg_base;

/*==== LOCALS ================================================================*/
/*local DIS to store slave address,clock speed,sid etc */
static T_I2C_DIS *i2c_dis;

/*------------------------------------------------------------------------------
 | Function    : i2c_init
 +------------------------------------------------------------------------------
 | Description : This routine will initialize the I2C bus.
 |               This routine will initialize the I2C bus,
 |               Clock settings, own address and enables I2C host controller.
 |
 | Parameters  :
 |   init_str  - init string
 |   dis_addr  - returned dis ptr
 |
 | Returns     : status of operation
 +----------------------------------------------------------------------------*/
S32 i2c_init(const void *init_str,
    U32 * dis_addr)
{
    S32 i2c_ret = 0;            /*return value */
    T_I2C_INIT_STRUCTURE *local_init_str;   /*initialization structure */
    local_init_str = (T_I2C_INIT_STRUCTURE *) init_str;
    /*check for the validity of the sid */
    if (local_init_str->sid >= I2C_MAX_INSTANCES)
    {
        return (OMAPFLASH_INVALID_SID);
    }

    /*allocate the memory for the DIS */
    i2c_dis = (T_I2C_DIS *) malloc(sizeof(T_I2C_DIS));
    if (i2c_dis == NULL)
    {
        I2C_DBG_PRINT(DBG_LEVEL_CRITICAL, "\r\nError in malloc\n\r");
        return -1;
    }

    /*Initialize the DIS */
    memcpy(i2c_dis, init_str, sizeof(T_I2C_INIT_STRUCTURE));
    *dis_addr = (U32) i2c_dis;

    configure_i2c(local_init_str->sid, local_init_str->clock_speed);
    /*return */
    I2C_DBG_PRINT(DBG_LEVEL_MINOR, "\r\nI2C: Returning from i2c init");
    return i2c_ret;
}

/*------------------------------------------------------------------------------
 | Function    : i2c_write
 +------------------------------------------------------------------------------
 | Description : I2C Write function
 |
 | Parameters  :
 |   dis_addr  - dis address
 |   key       - key -> this says what type of write it is
 |   len       - len to write
 |   buf       - buffer to write
 |
 | Returns     : status of op
 +----------------------------------------------------------------------------*/
S32 i2c_write(U32 dis_addr,
    U32 key,
    U32 * len,
    U8 * buf)
{
    U8 data;                    /*data buffer for the device */
    U16 offset;                 /*Subaddress can be 8 or 16 bit */
    U32 count, length;          /*local variables for storing size */
    T_I2C_DIS *local_dis;       /*pointer of type T_I2C_DIS */
    S32 ret_val = DAL_I2C_SUCCESS;  /*return value */

    U8 sub_address_mode = I2C_1_BYTE_ADDRESS;
    U8 *dataptr;

    /*use a local pointer to the dis */
    local_dis = (T_I2C_DIS *) dis_addr;
    length = *len;

    /* Check if it is a 2 byte subaddress - bit 24 in key is SET */
    if (key & I2C_USING_2BYTE_SUBADDRESS)
    {
        sub_address_mode = I2C_2_BYTE_ADDRESS;
        key &= ~I2C_USING_2BYTE_SUBADDRESS; /* Clear bit */
    }

    /*Check if it's a data transfer */
    /* Calculate the offset by taking the difference between DATA tag and
     * the current tag value */
    if (key & I2C_DATA)
    {
        offset = (U16) key;     /*Remove flags - bit 16-31 */
        /* make key as I2C_DATA so that it passes the following
         * switch statement*/
        key = I2C_DATA;
    }
    if (key & I2C_DATA_USING_FIFO)
    {
        offset = (U16) key;
        /* make key as I2C_DATA_USING_FIFO so that it passes
         * the following switch statement */
        key = I2C_DATA_USING_FIFO;
    }

    switch (key)
    {
        case I2C_DATA:         /*write the data */
            /*loop till written byte is less than actual write size */
            for (count = 0; count < length; count++)
            {
                data = buf[count];  /* next data */
                /*call the i2c write function to write a byte */
                ret_val = write_i2c_data(local_dis->i2c_initstr.slave_addr,
                        offset + count, data);  /* write a byte of data */
                if (ret_val != OMAPFLASH_SUCCESS)
                    break;
            }
            break;

        case I2C_DATA_USING_FIFO:
            if (I2C_2_BYTE_ADDRESS == sub_address_mode) /* 2-byte subaddress */
            {
                dataptr = (U8 *) malloc(length + 2);
                if (NULL == dataptr)
                {
                    I2C_DBG_PRINT(DBG_LEVEL_CRITICAL, 
                              "Error in malloc\n\r");
                    return -1;
                }
                /* prepend data with subaddress */
                dataptr[0] = (U8) ((offset & 0xFF00) >> 8);
                dataptr[1] = (U8) (offset & 0x00FF);
                for (count = 0; count < length; count++)
                    dataptr[count + 2] = buf[count];

                /*prepare for call to "write_i2c_with_dataptr" */
                length += 2;
            }
            else                /* 1-byte subaddress */
            {
                dataptr = (U8 *) malloc(length + 1);
                if (NULL == dataptr)
                {
                    I2C_DBG_PRINT(DBG_LEVEL_CRITICAL, 
                              "Error in malloc\n\r");
                    return -1;
                }
                /* prepend data with subaddress */
                dataptr[0] = (U8) (offset & 0x00FF);
                for (count = 0; count < length; count++)
                    dataptr[count + 1] = buf[count];

                /*prepare for call to "write_i2c_with_dataptr" */
                length += 1;
            }
            ret_val =
                write_i2c_with_dataptr(local_dis->i2c_initstr.slave_addr,
                                       length, dataptr);
            free(dataptr);
            break;

        case I2C_NOADDRESS_DATA:
            write_i2c_onlydata(local_dis->i2c_initstr.slave_addr, buf, len);
            break;

        case I2C_CLOCK_SPEED:
            set_i2c_clocks((U16) (*buf));   /* set the baud rate */
            I2C_DBG_PRINT(DBG_LEVEL_INFO, 
                      "I2C: Setting the clock speed\n");
            break;

        case I2C_SLAVE_ADDR:
            local_dis->i2c_initstr.slave_addr = (*buf);
            break;
    }
    /*return the status */
    return ret_val;
}

/*------------------------------------------------------------------------------
 | Function    : i2c_read
 +------------------------------------------------------------------------------
 | Description : i2c read op
 |
 | Parameters  :
 |   dis_addr  - dis
 |   tag       - type of read
 |   len       - length to read
 |   buf       - buffer to read to
 |
 | Returns     : status of op
 +----------------------------------------------------------------------------*/
S32 i2c_read(U32 dis_addr,
    U32 tag,
    U32 * len,
    U8 * buf)
{
    U8 *data;                   /*data buffer of the device */
    U16 offset;                 /*Subaddress can be 8 or 16 bit */
    U16 count, length;          /*local variables for storing size */
    T_I2C_DIS *local_dis;       /*pointer of type T_I2C_DIS */
    S32 ret_val = DAL_I2C_SUCCESS;  /*return value */
    U8 sub_address_mode = I2C_1_BYTE_ADDRESS;

    /*size of read */
    length = *len;
    /*use a local pointer to the dis */
    local_dis = (T_I2C_DIS *) dis_addr;

    if (I2C_USING_2BYTE_SUBADDRESS & tag)
    {
        sub_address_mode = I2C_2_BYTE_ADDRESS;
        tag &= ~I2C_USING_2BYTE_SUBADDRESS; /*Clear bit */
    }

    /*Calculate the offset in case of a I2C read operation */
    if (I2C_DATA & tag)
    {
        offset = (U16) tag;
        /* make tag equal I2C_DATA so that it passes the following switch
         * statement */
        tag = I2C_DATA;
    }
    if (I2C_DATA_USING_FIFO & tag)
    {
        offset = (U16) tag;
        /* make tag equal I2C_DATA_USING_FIFO so that it passes the following
         * switch statement */
        tag = I2C_DATA_USING_FIFO;
    }

    /*Do the read based on the tag value */
    switch (tag)
    {
        case I2C_DATA:         /*read the data */
            /*read till the no. of bytes read is less than the read size */
            for (count = 0; count < length; count++)
            {
                data = ((U8 *) buf + count);    /*increment the data pointer */
                /*call the i2c read function to read a byte */
                ret_val = read_i2c_data(local_dis->i2c_initstr.slave_addr,
                        offset + count, data);
                if (ret_val != OMAPFLASH_SUCCESS)
                    break;
            }
            (*len) = count;
            /*to print the function number and line number. */
            break;

        case I2C_DATA_USING_FIFO:
            /*variable "address_mode_is_2byte_offset" is used as a bool -
             *hence 1 must be added to get real address_mode as in defines */
            ret_val =
                read_i2c_with_dataptr_2byte_subaddr(local_dis->i2c_initstr.
                                                    slave_addr, offset,
                                                    sub_address_mode, length,
                                                    buf);
            break;

        case I2C_NOADDRESS_DATA:
            /*read till the no. of bytes read is less than the read size */
            /*call the i2c read function to read a byte */
            ret_val =
                read_i2c_onlydata(local_dis->i2c_initstr.slave_addr, buf, len);
            /*to print the function number and line number. */
            break;

        case I2C_CLOCK_SPEED:  /*read the clock speed */
            (*buf) = local_dis->i2c_initstr.clock_speed;
            (*len) = 2;         /*return the size of the parameter */
            I2C_DBG_PRINT(DBG_LEVEL_INFO, 
                      "I2C: Reading the clock speed\n");
            break;

        case I2C_SLAVE_ADDR:   /*read the slave address */
            (*buf) = local_dis->i2c_initstr.slave_addr;
            (*len) = 1;         /*return the size of the parameter */
            I2C_DBG_PRINT(DBG_LEVEL_INFO, 
                      "I2C: Reading the slave address\n");
            break;
    }
    /*return the status */
    return ret_val;
}

/*------------------------------------------------------------------------------
 | Function    : i2c_deinit
 +------------------------------------------------------------------------------
 | Description : de-init the same
 |
 | Parameters  :
 |   dis_addr  - dis address to free
 |
 | Returns     : status - success in our case
 +----------------------------------------------------------------------------*/
S32 i2c_deinit(U32 dis_addr)
{
    deinit_i2c();
    /* free the allocated dis */
    free((T_I2C_DIS *) dis_addr);
    /*return */
    return DAL_I2C_SUCCESS;
}

#endif /* END I2C_DRV_C */
