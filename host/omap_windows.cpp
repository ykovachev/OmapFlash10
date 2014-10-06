/**
 * @file 
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
 * 
 */
///@todo header should mention why this is a cpp when every thing else is c

#include <assert.h>
#include <time.h>
#include "comdriver.h"
#include "usbdriver.h"
#include "serialdriver.h"
extern "C" {
#include "usb.h"
#include "fastboot.h"
#include "output.h"
#include "omap_protocol.h"
}

#define USB 0
#define SID 0

// #define USE_CALLBACK
int driverErr;

#define PATTERN_VALUE 0x4F4D4150

typedef enum 
{
  ack_request = 0,
  message_ack = 1,
  header_ack  = 2,
  data_ack    = 3
} t_ack_type;

typedef struct  
{
  U32 pattern;
  struct
  {
    U32 more     : 1;
    U32 ctrl     : 1;
    U32 ack      : 1;
    U32 ack_type : 2;
    U32 unused   : 3;
    U32 length   : 24;
  } fields;
} t_message_header;

//TODO for some unknown reason we can not write without callback
#ifdef USE_CALLBACK
struct buf_t {
	struct buf_t *next;
    int offset;
    U32 size;
    U8 data[1];
};

struct queue {
	struct buf_t *first;
    int error;
};

struct queue queues[1];
int rx_callback_count;
int in_rx_callback;

S8 rx_callback (U16 sid, U8 *p_buf, U32 size) {

    static unsigned debug1 = 0;
    assert(!in_rx_callback); ///< once got a strange invalid pointer access on '*p=...' below assume that one rx_callback was interrupted by another need confirmation on that
    in_rx_callback = 1;

	rx_callback_count++;
	struct queue *q = &queues[sid];
	struct buf_t **p = &q->first;
	struct buf_t *b = (struct buf_t *)malloc (sizeof(struct buf_t) + size);
	if(b == NULL)
		return -1;

    b->offset = 0;
    b->size = size;
    memcpy(b->data, p_buf, size);
    b->next = 0;

    while(*p) p = &((*p)->next);

    *p = b;
    in_rx_callback = 0;

    #ifdef LOGGING
    if(debug1++) printf("  [USB CB: %d %s]\n", q->first->next > 0, p_buf);
    #endif
    return 0;
}

S8 connection_lost_callback (U16 sid) {
	host_logf("Error - connection lost");
	struct queue *q = &queues[sid];
	q->error = -1;
	return -1;
}
#endif

#ifndef USE_CALLBACK
int wait_rx_data_cb(int timeleft_millisec, void *data)
{
	return timeleft_millisec;
}
#else
extern "C" int wait_rx_data_cb(int timeleft_millisec, void *data)
{
	if(queues[0].first)
	{
		return timeleft_millisec;
	}
	else
	{
		return 0;
	}
}
#endif

extern "C" long total_wait_rx_data(long additional)
{
	static clock_t total_time = 0;
	total_time += additional;
	return total_time;
}

extern "C" int wait_millisec(int timeout_millisec, t_wait_cb *callback, void *data)
{
	clock_t start_time = clock();
	clock_t current_time;
	int timepassed = 0;
	int result;
	//TODO this is workaround we need to understand what is really going on and do a proper fix
	while(start_time == clock()); // To fix issue with host check for data being too fast for multiple INFOs:( 1 ms minimum...

	for(;;)
	{
		current_time  = clock();
		if (current_time  < start_time)
		{
			timeout_millisec -= timepassed;
			start_time = 0;
		}

		timepassed = (current_time - start_time) * 1000 / CLOCKS_PER_SEC;
		if(timeout_millisec < timepassed)
		{
			result = 0;
			break;
		}

		result = callback(timeout_millisec - timepassed, data);
		if (result || driverErr != DRV_ERR_EXPECTED)
		{
			break;
		}
	}

	result = total_wait_rx_data(current_time - start_time);
	return result;
}

extern "C" void wait_rx_data_ms(int timeout_millisec)
{
	wait_millisec(timeout_millisec, wait_rx_data_cb, 0);
}

extern "C" void wait_rx_data(int timeout)
{
	wait_rx_data_ms(timeout * 1000);
}

extern int match_fastboot(usb_ifc_info *info);

extern "C" omap_usb_handle *open_omap_device(int port, int baud, int parity)
{
	ComDriver *drv;

    if (port == USB)
        drv = new UsbDriver(SID);
    else
        drv = new SerialDrv(SID);

#ifdef USE_CALLBACK
    queue *q = &queues[drv->sid()];
    q->first = 0;
    q->error = 0;
    drv->registerCallback(rx_callback);
    drv->registerConnectionLostCallback(connection_lost_callback);
#endif

    int ret = drv->open(port,baud, parity, ONESTOPBIT, 8);
    if (ret < 0)
	{
		// gets the error str
		driverErr = drv->getDrvErr();
        return 0;
	}
    //binary_mode = binary_mode_;
    //omap_usb_handle *usb = new omap_usb_handle;
    //rx_state = RX_LEN_1;
    //rx_inv_size = 0;
    //error_handled = false;
    return drv;
}

enum T_mode {bin,cstr,raw,ack};
extern "C" int usb_write_ex(omap_usb_handle* drv, const void* data, int len, T_mode mode, unsigned bin_count) 
{ 
    int result, r;
    U8* ptr = (U8*) data;

#ifdef USE_CALLBACK
    queue *q = &queues[drv->sid()];

    if (q->error)
    {
        ///@todo output some details on what went wrong e.g. error code
        host_logf("USB TX Failed - mode %d", mode);
        omap_usb_close(drv);
        return q->error;
    }
#endif

    //TODO only start with length when romapi involved
    if (!no_force_package_alignment)
    {
      len = (len + 3) & ~3; // romapi can only receive whole U32
    }

    result = 0;

    sleepms(rxtx_delay);
    
	if(bin_count == 0)
		bin_count = 1;

    for(; bin_count > 0; bin_count--)
    {
      // For anything other than RAW mode we need to send a header first
      
      if(data_delay)
      {
        sleepms(data_delay);
      }

      if (mode != raw)
      {
        t_message_header message_header;

        message_header.pattern         = PATTERN_VALUE;
        message_header.fields.ctrl     = (mode == cstr) || (mode == ack);
        message_header.fields.ack      = (mode == ack) || force_ack;
        message_header.fields.ack_type = (mode == ack) ? message_ack : ack_request;
        message_header.fields.more     = (bin_count > 1);
        message_header.fields.length   = len;
        r = drv->write ((U8 *)&message_header, sizeof(message_header));
        
        if (r != sizeof(message_header))
        {
          host_logf("Failed to send message header");
          return -1;
        }
      }

      if (mode == ack)
      {
        return result;
      }

      if ((mode != raw) && force_ack)
      {
        t_message_header message_header;
        
        wait_rx_data(timeout);
        
        r = drv->read((U8 *)&message_header, sizeof(message_header), 1000);

        if(r != sizeof(message_header))
        {
          host_logf("Failed to receive header ACK (%d)", r);
          return -1;
        }

        if(message_header.pattern != 0x4F4D4150)
        {
          host_logf("Unexpected message format (%08X)", message_header.pattern);
          return -1;
        }

        if(!message_header.fields.ctrl || !message_header.fields.ack || message_header.fields.length)
        {
          host_logf("Bogus header ACK received (CTRL %d ACK %d LENGTH %d)", message_header.fields.ctrl, message_header.fields.ack, message_header.fields.length);
          return -1;
        }

        RXTX_TRACE("RX ACK (Type %d)", message_header.fields.ack_type);
      }

      if(data_delay)
      {
        sleepms(data_delay);
      }
    
      r = drv->write (ptr, len);		

      //sleepms(5);

      if (r != len)
      {
        host_logf("Failed to send message data");
        return -1;
      }

      ptr += len;
      result += r;


      if ((mode != raw) && force_ack)
      {
        t_message_header message_header;
        
        wait_rx_data(timeout);

        r = drv->read((U8 *)&message_header, sizeof(message_header), 1000);

        if(r != sizeof(message_header))
        {
          host_logf("Failed to receive data ACK (%d)", r);
          return -1;
        }

        if(message_header.pattern != 0x4F4D4150)
        {
          host_logf("Unexpected message format (%08X)", message_header.pattern);
          return -1;
        }

        if(!message_header.fields.ctrl || !message_header.fields.ack || message_header.fields.length)
        {
          host_logf("Bogus data ACK received (CTRL %d ACK %d LENGTH %d)", message_header.fields.ctrl, message_header.fields.ack, message_header.fields.length);
          return -1;
        }
        
        RXTX_TRACE("RX ACK (Type %d)", message_header.fields.ack_type);
      }
    }
 #ifdef LOGGING
  if(mode == cstr) fprintf(stderr,"  [USB WR(CHR): %s]\n", (char *)data); 
  if(mode == bin) fprintf(stderr,"  [USB WR(BIN): %X bytes]\n", len);
  if(mode == raw) fprintf(stderr,"  [USB WR(RAW): %X bytes]\n", len);
  #endif
  return result;
}

extern "C" int usb_write_cstr(omap_usb_handle* drv, const void* data, int len) { 
    return usb_write_ex(drv, data, len, cstr, 0);
}

extern "C" int usb_write_bin(omap_usb_handle* drv, const void* data, int len, unsigned count) { 
    if (!no_force_package_alignment)
        assert((len & 3) == 0);
    return usb_write_ex(drv, data, len, bin, count);
}

extern "C" int usb_write_raw(omap_usb_handle* drv, const void* data, int len) { 
    return usb_write_ex(drv, data, len, raw, 0);
}

extern "C" int usb_write_ack(omap_usb_handle* drv) { 
  sleepms(50);
  return usb_write_ex(drv, 0, 0, ack, 0);
}

extern "C" int usb_read_ex(omap_usb_handle* drv, void* data, int len, T_mode mode) { 
	static t_message_header message_header;

#ifdef USE_CALLBACK
    queue *q = &queues[drv->sid()];

    if (q->error) {
        printf("  [USB READ: ERROR]\n");
        omap_usb_close(drv);
        return q->error;
    }

    int actual_len = 0;
    
    memset(&message_header, 0, sizeof(message_header));
    
    while (q->first && len > actual_len) 
    {
		struct buf_t * first = q->first;
        int count = first->size - first->offset;
        U8 *queue_data = first->data + first->offset;
        
        U32 pattern = (*(queue_data)) | (*(queue_data + 1) << 8) | (*(queue_data + 2) << 16) | (*(queue_data + 3) << 24);
        
        if(pattern == 0x4F4D4150)
        {
          memcpy(&message_header, queue_data, sizeof(message_header));
          queue_data += 8;
          count -= 8;
          first->offset += 8;
        }
        
        if (count > len - actual_len)
            count = len - actual_len;
    
        if (mode == cstr)
        {
            int j;
            for (j = 0;  j < count; j++) 
            {
                if (!queue_data[j]) 
                {
                    j++;
                    break;
                }
            }
            // eat fill bytes
            while (j < count && !queue_data[j] && j % 3 != 0) j++;
            count = j;
        }
        memcpy((char*)data + actual_len, queue_data, count);

        actual_len += count;
        first->offset += count;
        if (first->offset == first->size) 
        {
            q->first = first->next;
            free (first);
        }
        
        if (mode == cstr || mode == bin) 
        {
            #ifdef LOGGING
            printf("  [USB READ: %X %s %s]\n", actual_len, mode == cstr ? "CSTR" : "RAW", mode == cstr ? data : "");
            #endif
            if (mode == cstr ? ((char*)data)[actual_len - 1] : len > actual_len)
            {
                wait_rx_data_ms(1000);
                if (q->first)
                    continue;
            }
           
            if((message_header.fields.ack) && message_header.fields.length)
            {
              RXTX_TRACE("TX ACK");
              usb_write_ack(drv);
            }
            return actual_len;
        }
    }

    if((message_header.fields.ack) && message_header.fields.length)
    {
      RXTX_TRACE("TX ACK");
      usb_write_ack(drv);
    }

    #ifdef LOGGING
    printf("  [USB READ: %X BIN]\n", actual_len);
    #endif
    return actual_len;
#else //USE_CALLBACK
    int ret; U8 *buf, *bufptr;
    
    len = (mode == raw)? len : len + sizeof(message_header);
    bufptr = buf = (U8 *)malloc(len);
    
    ret = drv->read (buf, len, 1000);		
    // we should always get message_header size data except when raw mode
	if(ret >= sizeof(message_header))
	{
		unsigned int *pattern = (unsigned int *)buf; 
		if(*pattern == 0x4F4D4150)
		{
            memset(&message_header, 0, sizeof(message_header));
            memcpy(&message_header, buf, sizeof(message_header));
            ret -= sizeof(message_header);
            buf += sizeof(message_header);
            if((message_header.fields.ack) && message_header.fields.length)
            {
              RXTX_TRACE("TX ACK");
              usb_write_ack(drv);
            }
		}
	}

    if(ret > 0)
    {
        memcpy(data, buf, ret);
        free((void *)bufptr);
    }

    return ret;
#endif //USE_CALLBACK
}

extern "C" int usb_read_cstr(omap_usb_handle* drv, void* data, int len) { 
    return usb_read_ex(drv, data, len, cstr);
}

extern "C" int usb_read_raw(omap_usb_handle* drv, void* data, int len) { 
    return usb_read_ex(drv, data, len, raw);
}

extern "C" int omap_usb_read(omap_usb_handle* drv, void* data, int len) { 
    return usb_read_ex(drv, data, len, bin);
}

extern "C" int omap_usb_close(omap_usb_handle* drv) {
    assert(drv);
    if (drv)
        delete drv;
    drv = 0;
    return 0;
}

