/*
 * Copyright (C) 2008 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the 
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _USB_H_
#define _USB_H_

#ifdef _WIN32
#ifdef __cplusplus
typedef class ComDriver omap_usb_handle;
#else
typedef struct ComDriver omap_usb_handle;
#endif
#else
typedef struct fb_usb_handle omap_usb_handle;
#endif
typedef struct fb_usb_handle fb_usb_handle;

typedef struct usb_ifc_info usb_ifc_info;

struct usb_ifc_info
{
        /* from device descriptor */
    unsigned short dev_vendor;
    unsigned short dev_product;

    unsigned char dev_class;
    unsigned char dev_subclass;
    unsigned char dev_protocol;
    
    unsigned char ifc_class;
    unsigned char ifc_subclass;
    unsigned char ifc_protocol;

    unsigned char has_bulk_in;
    unsigned char has_bulk_out;
    
    unsigned char writable;
    char serial_number[256];
};
  
typedef int (*ifc_match_func)(usb_ifc_info *ifc);
typedef int t_wait_cb(int timeleft_millisec, void *data);

fb_usb_handle *fb_usb_open(ifc_match_func callback);
omap_usb_handle *open_omap_device(int port, int baud, int parity);

int usb_read_cstr(omap_usb_handle *h, void *_data, int len);
int usb_read_raw(omap_usb_handle* drv, void* data, int len);
long total_wait_rx_data(long additional);
int wait_millisec(int timeout_millisec, t_wait_cb *callback, void *data);
void wait_rx_data_ms(int timeout);
void wait_rx_data(int timeout);
int usb_write_raw(omap_usb_handle* drv, const void* data, int len);
int usb_write_bin(omap_usb_handle* drv, const void* data, int len, unsigned count);
int usb_write_cstr(omap_usb_handle* drv, const void* data, int len);
int omap_usb_close(omap_usb_handle *h);
int omap_usb_read(omap_usb_handle *h, void *_data, int len);

int fb_usb_close(fb_usb_handle *h);
int fb_usb_read(fb_usb_handle *h, void *_data, int len);
int fb_usb_write(fb_usb_handle *h, const void *_data, int len);


#endif
