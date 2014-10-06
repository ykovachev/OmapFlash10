/*
* Copyright (C) 2009-2010 Texas Instruments
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

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <ctype.h>

#include "fastboot.h"
#include "adler32.h"
#include "output.h"
#include "omap_protocol.h"
#include "memory_partitions.h"

#ifdef _WIN32
#include "util_windows.h"
#endif

#define TIMEOUT_POLL_TIME	10	// msec

#define DATA_RESPONSE_PACKAGE_SIZE(rsp) (rsp & 0xFFFFFF)
#define DATA_RESPONSE_PACKAGE_COUNT(rsp) ((rsp >> 24) & 0xFF) 

static int do_check_data_response(omap_usb_handle *usb, unsigned expected_size);
static int do_send_data(omap_usb_handle *usb, const void *data, unsigned size, unsigned count);
static int do_send_command(omap_usb_handle *usb, const char *cmd);
static int do_check_control_response(omap_usb_handle *usb, char *response);
static int get_response(omap_usb_handle *usb, char *response);
static unsigned do_check_read_response(omap_usb_handle *usb, unsigned expected_size);
//static int do_receive_data(omap_usb_handle *usb, const unsigned * data, unsigned expected_size);
static int do_receive_data(omap_usb_handle *usb, FILE *fp, unsigned expected_size);

extern char *chunkSizeMem;

#ifdef DO_RXTX_TRACE
void do_rxtx_trace(int line, char *format, ...)
{
  va_list ap;

  if (rxtx_trace) 
  {
    va_start(ap, format);
    trace_vlogf(line, format, ap);
    va_end(ap);
  }
}

void do_rxtx_raw_data_trace(int line, const void *data, int size, char *format, ...)
{
  va_list ap;
  int i;
  char buf[300];
  int maxN = sizeof buf - 1;
  int n;

  if (rxtx_trace) {
    va_start(ap, format);
    fprintf(stderr,"\n");
    n = vsnprintf(buf, maxN, format, ap);
    va_end(ap);

    ///if no error and enough space for hex dump in buf add a hexdump
    /// RXTX_TRACE_HEX_SZ * 4 is for: 1 space seperator, 2 hex digit, 1 ascii
    /// 50 is for text, quotes, etc. 
    /// should be enough that we don't need to check for negative return from snprintf in the following
    if (n > 0 && n < maxN - RXTX_TRACE_HEX_SZ * 4 - 50) 
    {
      n += SNPRINTF(buf+n, maxN-n, " %d raw bytes:", size);
      for (i = 0; i < RXTX_TRACE_HEX_SZ && i < size; i++) {
        n += SNPRINTF(buf+n, maxN-n, " %02X", ((char*)data)[i] & 0xFF);
      }

      n += SNPRINTF(buf+n, maxN-n, " '");

      for (i = 0; i < RXTX_TRACE_HEX_SZ && i < size; i++) {
        if (isprint(((unsigned char*)data)[i]))
          n += SNPRINTF(buf+n, maxN-n, "%c", ((char*)data)[i]);
        else
          n += SNPRINTF(buf+n, maxN-n, "?");
      }
      n += SNPRINTF(buf+n, maxN-n, "'");
      if (i < size) {
        n += SNPRINTF(buf+n, maxN-n, " ...");
      }
      trace_logf(line, "%s", buf);
    }
  }
}
#endif //DO_RXTX_TRACE

static char ERROR[128];

static int read_hex_dump_width = 16;

void get_command_element(char * src, unsigned char element, char * dst)
{
  // "write EMMC 0 20000"
  // "write xloader 100 200"
  char * start = src, * end;
  unsigned char i;

  for(i = 0; i < element; i++)
  {
    start = strchr(start, ' ') + 1;  
  }

  end = start;
  end = strchr(end, ' ');

  if(!end) end = src + strlen(src);

  memcpy(dst, start, end - start);

  dst[end - start] = 0;
} 


char *fb_get_error(void)
{
  return ERROR;
}

int fb_data_receive(omap_usb_handle *usb,void *data, unsigned size)
{
  int r = usb_read_raw(usb, data, size);
  if(r < 0) {
    sprintf(ERROR, "fb_data_receive: Status read failed (%s)", strerror(errno));
    omap_usb_close(usb);
    return -1;
  }
  return r;
}

int fb_data_send_raw(omap_usb_handle *usb,const void *data, unsigned size)
{
  int r;
  RXTX_RAW_DATA_TRACE(data, size, "TX");
  r = usb_write_raw(usb, data, size);
  if(r < 0) {
    sprintf(ERROR, "Raw data transfer failure (%s)", strerror(errno));
    omap_usb_close(usb);
    return -1;
  }
  if(r != ((int) size)) {
    sprintf(ERROR, "Raw data transfer failure (short transfer)");
    omap_usb_close(usb);
    return -1;
  }
  return r;
}

int fb_data_send(omap_usb_handle *usb, const void *data, unsigned size, unsigned count)
{
  int r;
  RXTX_TRACE("TX %d bin bytes",size);
  r = usb_write_bin(usb, data, size, count);
  if(r < 0) {
    sprintf(ERROR, "Data transfer failure (code %d, %s)", r, strerror(errno));
    omap_usb_close(usb);
    return -1;
  }
  if(r != ((int) size * count)) {
    sprintf(ERROR, "Data transfer failure (short transfer tried %d*%d=%d act %d)", size, count, size * count, r);
    omap_usb_close(usb);
    return -1;
  }
  return r;
}

int fb_response(omap_usb_handle *usb, char *response)
{
  return do_check_control_response(usb, response);
}

static int do_download_data_packages(omap_usb_handle *usb, const void *data, unsigned size, const char *file)
{
  char cmd[64];
  int r, ret;
  unsigned fileSz;

  if((ret = getFileSize(file)) == -1)
  {
    host_logf("Error: Cannot determine file size. Download failed");
    return -1;
  }

  if((fileSz = (unsigned) ret) < size)
  {
    host_logf("Error: File size is less than download size.");
    return -1;
  }

  sprintf(cmd, "download:%08x", size);

  if(do_send_command(usb, cmd) >= 0) 
  {
    unsigned package_size = 0, package_count = 0;
    unsigned s = size, tempSize = 0, bytesRead = 0;
    unsigned int offset = 0;
    unsigned char *d = (char *)data;
    unsigned char *aux;
    unsigned byteToRead = 0;

    do 
    {
      int rsp = do_check_data_response(usb, s);
#ifndef _WIN32
      sleepms(20);
#endif
      if(rsp > 0)
      {
        package_size = DATA_RESPONSE_PACKAGE_SIZE(rsp);
        package_count = DATA_RESPONSE_PACKAGE_COUNT(rsp);

        assert(package_size * package_count <= s);

        // check to see if we have enough data to download, else load another chunk from file
        if(package_size * package_count > tempSize)
        {
          // if there is stil some more data left from previous chunk, discard it and
          // read starting from the start of the discarded data in file. so adjust offset accordingly.
          // ** this should never happen unless the chunkSize is not exact multiple of target package size.
          if(tempSize != 0)
          {
            offset -= tempSize;
            fileSz += tempSize;
          }
          byteToRead = (fileSz > chunkSize)? chunkSize : fileSz;
          bytesRead = load_file_chunk(file, d, byteToRead, offset);	
          if(bytesRead == -1 || bytesRead == 0)
          {
            host_logf("Download failed(read file error)");
            return -1;
          }
          aux = d;
          offset += bytesRead;
          tempSize = bytesRead;
          fileSz -= bytesRead;
          if((bytesRead != byteToRead) && (bytesRead < s))
          {
            host_logf("Download failed (read file error): file ended");
            assert(!assert_on_error);
            return -1;
          }
        }

        assert(package_size * package_count <= tempSize);

        host_progress_logf(s == size, size - s + package_size, size, "Sending data (%d bytes)", size);

        r = do_send_data(usb, aux, package_size, package_count);

        if (r < 0)
        {
          char *sep = ERROR[0] == 0 ? ", ": "";
          host_logf("Download failed (data transmission error): %s%s", sep, ERROR);
          assert(!assert_on_error);
          return -1;
        }

        aux += r;
        s -= r;
        tempSize -= r;
      }
      else
      {
        host_logf("Download failed (data response error): %s", ERROR);
        assert(!assert_on_error);
        return -1;
      }
    } while(s);


    host_progress_logf(0, size - s, size, "Sending data (%d bytes)", size);
    if(do_check_control_response(usb, 0) < 0)
    {
      host_logf("Download failed (final data response error): %s", ERROR);
      assert(!assert_on_error);
      return -1;
    }

    host_logf_ex(VERBOSE, "Downloading complete");
    return 0;
  }

  return(-1);
}

int fb_download_data_packages(omap_usb_handle *usb, const void *data, unsigned size, const char *file)
{
  int result;
  clock_handle clk1, clk2;
  getCurrentTime(&clk1);
  result = do_download_data_packages(usb, data, size, file);
  getCurrentTime(&clk2);
  report_elapsed_time_rate(clk1, clk2, size);
  return result;
}

static int do_upload_data(omap_usb_handle *usb, const char *cmd, char *fname, unsigned *size)
{
  unsigned dsize; int ret;
  if(do_send_command(usb, cmd) > 0) {
    sleepms(50);
    dsize = do_check_read_response(usb, *size);

    if(dsize > 0) 
    {
      //*data = (char *)malloc(*size);
      //if(*chunkSizeMem) 
      //{
      FILE *fp = fopen(fname, "wb+");
      if(fp == NULL)
      {
        strcpy(ERROR, "Cannot createread protocol error 1");
        SNPRINTF(ERROR, sizeof(ERROR), "Cannot create file %s for saving data", fname);
        omap_usb_close(usb);
        return -1;
      }

      ret = do_receive_data(usb, fp, dsize);
      fclose(fp);
      if(ret < 0) 
      {
        assert(!assert_on_error);
        return -1;
      }
      else 
      {
        return(do_check_control_response(usb, 0));
      }
      //}
      //sprintf(ERROR, "Cannot alloc memory for upload data");
    }
  }
  assert(!assert_on_error);
  return -1;
}

int fb_upload_data(omap_usb_handle *usb, char *cmd, char *fname, unsigned *size)
{
  int result;
  clock_handle clk1, clk2;
  getCurrentTime(&clk1);

  if(!memcmp(cmd, "read", 4))
  {
    char memory[21];
    char offset[21];
    char size[21];
    char * partition_memory;

    get_command_element(cmd, 1, memory);
    get_command_element(cmd, 2, offset);
    get_command_element(cmd, 3, size);

    partition_memory = partition_get_memory(memory);

    if(partition_memory)
    {
      char modified_command[256];
      unsigned long partition_size = partition_get_size(memory) * 1024;
      unsigned long partition_offset = partition_get_offset(memory) * 1024;
      unsigned long image_size = strtoul(size, NULL, 16);
      unsigned long image_offset = strtoul(offset, NULL, 16);
      if(image_size > partition_size)
      {
        die("Unable to read from %s - requested size larger than partition", memory);
      }
      if(image_size + image_offset > partition_size + partition_offset)
      {
        die("Unable to read from %s - requested data beyond end of partition", memory);
      }

      host_logf_ex(VERBOSE, "Partition %s@%s --> Memory %s@%0X", memory, offset, partition_memory, partition_offset + image_offset);
      sprintf(modified_command, "read %s %0X %0X", partition_memory, partition_offset + image_offset, size);

      cmd = modified_command;
    }
  }

  result = do_upload_data(usb, cmd, fname, size);
  getCurrentTime(&clk2);
  if(result == -1)
    report_elapsed_time_rate(clk1, clk2, 0);
  else
    report_elapsed_time_rate(clk1, clk2, *size);
  return result;
}

int omap_send_command(omap_usb_handle *usb, char *cmd)
{

  if(!memcmp(cmd, "write", 5))
  {
    char memory[21];
    char offset[21];
    char size[21];
    char * partition_memory;

    get_command_element(cmd, 1, memory);
    get_command_element(cmd, 2, offset);
    get_command_element(cmd, 3, size);

    partition_memory = partition_get_memory(memory);

    if(partition_memory)
    {
      char modified_command[256];
      unsigned long long partition_size = partition_get_size(memory) * 1024;  // TODO: Fix these length fields to long long for large devices!!!!
      unsigned long partition_offset = partition_get_offset(memory) * 1024;
      unsigned long image_size = strtoul(size, NULL, 16);
      unsigned long image_offset = strtoul(offset, NULL, 16);
      if(image_size > partition_size)
      {
        die("Unable to write to %s - too much data for partition", memory);
      }

      if(image_size + image_offset > partition_size + partition_offset)
      {
        die("Unable to write to %s - data beyond end of partition", memory);
      }

      host_logf_ex(VERBOSE, "Partition %s@%s --> Memory %s@%0X", memory, offset, partition_memory, partition_offset + image_offset);
      sprintf(modified_command, "write %s %0X %0X", partition_memory, partition_offset + image_offset, image_size);

      cmd = modified_command;
    }
  }

  if(do_send_command(usb, cmd) > 0) 
  {
    return do_check_control_response(usb, 0);
  }
  return -1;
}

int omap_send_command_response(omap_usb_handle *usb, char *cmd, char *response)
{
  if(!memcmp(cmd, "erase", 5))
  {
    char memory[21];
    char offset[21];
    char size[21];
    char * partition_memory;

    get_command_element(cmd, 1, memory);
    get_command_element(cmd, 2, offset);
    get_command_element(cmd, 3, size);

    partition_memory = partition_get_memory(memory);

    if(partition_memory)
    {
      char modified_command[256];
      unsigned long long partition_erase_size;
      unsigned long long partition_size = partition_get_size(memory); 
      unsigned long long partition_offset = partition_get_offset(memory); 
      #ifdef _MSC_VER
      unsigned long long erase_size = _strtoui64(size, NULL, 16);
      unsigned long long erase_offset = _strtoui64(offset, NULL, 16);
      #else
      unsigned long long erase_size = strtoull(size, NULL, 16);
      unsigned long long erase_offset = strtoull(offset, NULL, 16);
      #endif
      partition_size *= 1024;
      partition_offset *= 1024;
      partition_erase_size = (erase_size == 0) ? partition_size - erase_offset : erase_size;
      if(erase_size > partition_size)
      {
        die("Unable to erase 0x%I64X bytes in %s - would erase more than partition size of 0x%I64X", erase_size, memory, partition_size);
      }
      if(erase_size + erase_offset > partition_size + partition_offset)
      {
        die("Unable to erase 0x%I64X bytes in %s - would erase beyond end of partition", erase_size, memory);
      }

      host_logf_ex(VERBOSE, "Partition %s@%s %s --> Memory %s@%I64X %I64X", memory, offset, size, partition_memory, (partition_offset + erase_offset), partition_erase_size);
      sprintf(modified_command, "erase %s %I64X %I64X", partition_memory, (partition_offset + erase_offset), partition_erase_size);

      cmd = modified_command;
    }
  }

  if(do_send_command(usb, cmd) > 0) 
  {
    return do_check_control_response(usb, response);
  }

  return -1;
}

//int omap_send_download_data(omap_usb_handle *usb, const void *data, unsigned size)
//{
//    char cmd[64];
//
//	sprintf(cmd, "download:%08x", size);
//
//	if(omap_send_command(usb, cmd) > 0)
//    {
//        signed package_size = do_check_data_response(usb, 0);
//        if(package_size == size)
//        {
//            return do_send_data(usb, data, package_size);
//        }
//    }
//    
//    host_logf("Downloading failed");
//    return -1;
//}

/**********************************************************************************************************/

static int do_send_command(omap_usb_handle *usb, const char *cmd)
{
  int result;
  int cmdsize = cmd ? strlen(cmd) + 1: 0;

  if(cmdsize > 256) {
    sprintf(ERROR,"Command too large");
    return -1;
  }
#ifndef _WIN32
  sleepms(50);
#endif
  if(cmdsize) {
    RXTX_TRACE("TX:%s",cmd);
    result = usb_write_cstr(usb, cmd, cmdsize);
    if (result < 0) {
      sprintf(ERROR,"Command write failed (%s)", strerror(errno));
      omap_usb_close(usb);
      return -1;
    }
  }

  return cmdsize;
}

static int do_send_data(omap_usb_handle *usb, const void *data, unsigned size, unsigned count)
{
  int r;

  assert (data);
  assert (size);
  r = fb_data_send(usb, data, size, count);
  if(r < 0) {
    return -1;
  }

  return r;
}

void get_response_break() {}
int debug_count_get_response = 0;
static int get_response(omap_usb_handle *usb, char *response)
{
  unsigned char status[FB_RESPONSE_SZ + 1];

  int r = 0; int timeout_loop =  (timeout * 1000) / TIMEOUT_POLL_TIME;
  static int init = 1;

  for(;;) 
  {
    memset(status, 0, sizeof status); //LOE

    //if (++debug_count_get_response == 9)
    //    get_response_break();
    do {
      //wait_rx_data(timeout);
      r = usb_read_cstr(usb, status, sizeof status - 1);
      if(r != 0)
        break;
      sleepms(TIMEOUT_POLL_TIME);
    } while (timeout_loop--); // skip fill bytes

    if (r == 0) 
    {
      sprintf(ERROR, "Reception failed - timeout after %d s", timeout);
      omap_usb_close(usb);
      assert(!assert_on_error);
      return -1;
    }

    if (r < 0) 
    {
      sprintf(ERROR, "get_response: Status read failed (%s)", strerror(errno));
      omap_usb_close(usb);
      assert(!assert_on_error);
      return -1;
    }

    status[r] = 0;

    if (r < 4) 
    {
      RXTX_RAW_DATA_TRACE(status, r, "RX:");
      sprintf(ERROR, "Status malformed response (%d bytes)", r);
      omap_usb_close(usb);
      assert(!assert_on_error);
      return -1;
    }

    //sleepms(100); ///<@todo fix bug in target concerning ack not received
    if (memcmp(status, "data", 4))
    {
      //RXTX_TRACE("TX ACK");
      //usb_write_ack(usb); 
    }

    if(!memcmp(status, "STAT", 4))
    {
      char * end = status + 5;
      unsigned long long current, target;
      RXTX_TRACE("RX:%s\n", status);
      //current = _atoi64((char*) end, &end, 10);
      //target  = _atoi64((char*) end + 1, &end, 10);
#ifdef _MSC_VER
      current = _strtoi64((char*) end, &end, 16);
      target  = _strtoi64((char*) end + 1, &end, 16);
#else
      current = strtoull((char*) end, &end, 16);
      target  = strtoull((char*) end + 1, &end, 16);
#endif
      status[FB_RESPONSE_SZ] = 0;
      target_progress_logf(init, current, target, end);
      init = 0;
      continue;
    }

    init = 1;

    if(!memcmp(status, "INFO", 4)) {
      RXTX_TRACE("RX:%s", status);
      //send a pseudo ACK, protocol is ping-pong since the capability of communication chanel sucks
      //1) romapi do not support duplex and
      //2) Windows-USB drivers read will only return latest message (if more than one arrive between two reads the first ones is lost)
      //Note: TODO? we might be able to fix 2 but then there is the issue of getting all the esisting CSST installations updated
      target_logf_ex(VERBOSE, "%s", status + 5);
      continue;
    }

    if(!memcmp(status, "FAIL", 4)) 
    {
      RXTX_TRACE("RX:%s", status);
      if(r > 4) 
      {
        sprintf(ERROR, "Remote: %s", status + 4);
      } 
      else 
      {
        strcpy(ERROR, "Remote failure");
      }
      omap_usb_close(usb);
      assert(!assert_on_error);
      if(response)
      {
        strcpy(response, status + 5);
      }
      return -1;
    }

    if(response)
    {
      strcpy(response, status);
    }

    return(r);
  }
}

static int do_check_data_response(omap_usb_handle *usb, unsigned expected_size)
{
  unsigned char status[FB_RESPONSE_SZ + 1];

  int r = get_response(usb, status);

  if((r > 0)  && (!memcmp(status, "DATA", 4) || !memcmp(status, "data", 4))) 
  {
    unsigned rsp = strtoul((char*) status + 4, 0, 16);
    unsigned dsize = DATA_RESPONSE_PACKAGE_COUNT(rsp) * DATA_RESPONSE_PACKAGE_SIZE(rsp);
    RXTX_TRACE("RX:%s", status);
    if((dsize > expected_size) && expected_size) 
    {
      strcpy(ERROR, "data size too large");
      omap_usb_close(usb);
      return -1;
    }
    else 
    {
      return rsp;
    }
  }
  else 
  {
    host_logf("Error in do_check_data_response %d %s", r, status);
    strcpy(ERROR, "response");
    return -1;
  }
}

static int do_check_control_response(omap_usb_handle *usb, char *response)
{
  unsigned char status[FB_RESPONSE_SZ + 1];

  int r = get_response(usb, status);

  if(r > 0 && !memcmp(status, "OKAY", 4)) {
    RXTX_TRACE("RX:%s", status);
    if(response) {
      strcpy(response, (char*) status + 4);
    }
    return 0;
  }

  if (r > 0) {
    unsigned char text[FB_RESPONSE_SZ + 1];
    int i;
    for (i = 0; i < r; i++) {
      if (isprint(status[i]))
        text[i] = status[i];
      else
        text[i] = '?';
    }
    text[i] = 0;
    RXTX_RAW_DATA_TRACE(status, r, "RX:");
    sprintf(ERROR,"Unknown status message '%.*s'", r, text);
    //sprintf(ERROR,"Unknown status message");
  }
  assert(!assert_on_error);
  return -1;
}

static unsigned do_check_read_response(omap_usb_handle *usb, unsigned expected_size)
{
  unsigned char status[FB_RESPONSE_SZ + 1];

  int r = get_response(usb, status);
  if (r > 0)
  {
    if (!memcmp(status, "DATA", 4) || !memcmp(status, "data", 4))
    {
      char *end;
      unsigned dsize = strtoul((char*) status + 4, &end, 16);
      RXTX_TRACE("RX:%s", status);
      if((dsize != expected_size) && expected_size) 
      {
        strcpy(ERROR, "data size too large");
        omap_usb_close(usb);
        assert(!assert_on_error);
        return 0;
      }
      else
      {
        return(dsize);
      }
    }
    else
    {
      sprintf(ERROR, "unexpected data response (%s)", status);
      omap_usb_close(usb);
      assert(!assert_on_error);
      return 0;
    }
  }
  else
  {
    assert(!assert_on_error);
    return 0;
  }
}

static int do_receive_data(omap_usb_handle *usb, FILE *fd, unsigned expected_size)
{

  int r;
  unsigned char status[FB_RESPONSE_SZ + 1];
  unsigned remDataBufSz = chunkSize;
  char * read_data = chunkSizeMem;
  unsigned target_size = expected_size;

  while (expected_size > 0) {
#ifndef _WIN32
    sleepms(50);
#endif

    r = get_response(usb, status);

    if(r < 0) {
      return -1;
    }

    if(!memcmp(status, "READ", 4))
    {
      char *end;
      unsigned dsize = strtoul((char*) status + 4, &end, 16);
      RXTX_TRACE("RX:%s", status);
      if (*end != ' ') {
        strcpy(ERROR, "read protocol error 1");
        omap_usb_close(usb);
        return -1;
      }
      else {
        unsigned checksum_expected = strtoul((char*) end + 1, &end, 16);
        unsigned checksum_calculated = adler32((unsigned char*)NULL, 0);;
        if (*end) {
          strcpy(ERROR, "read protocol error 2");
          omap_usb_close(usb);
          return -1;
        }
        else {
          // this will happen only if the target packet_size * packet_num > chunckSize
          if(dsize > chunkSize)
          {
            strcpy(ERROR, "Read buff too small. Increase chunksize;");
            return -1;
          }
          if(dsize > remDataBufSz)
          {
            if(fwrite(chunkSizeMem, sizeof(char), chunkSize - remDataBufSz, fd) != (chunkSize -remDataBufSz))
            {
              strcpy(ERROR, "Write to file failed");
              omap_usb_close(usb);
              return -1;
            }
            read_data = chunkSizeMem;
            remDataBufSz = chunkSize;
          }
          wait_rx_data(timeout);
          r = omap_usb_read(usb, read_data, dsize);
          if (r == 0) {
            sprintf(ERROR, "Reception failed - timeout after %d s", timeout);
            omap_usb_close(usb);
            return -1;
          }

          if ((unsigned)r < dsize){
            sprintf(ERROR, "Received less data than expected (%08X < %08X)", r, dsize);
            omap_usb_close(usb);
            return -1;
          }

#ifndef _WIN32
          sleepms(100);
#endif
          checksum_calculated = update_adler32(checksum_calculated, (unsigned char*)read_data, r);
          read_data += r;
          expected_size -= r;
          remDataBufSz -= r;

          target_progress_logf(expected_size == (target_size - r), target_size - expected_size, target_size, " Receiving data (%u bytes)", target_size);

          if (checksum_expected != checksum_calculated){
            sprintf(ERROR, "Data read checksum error %08X != %08X", checksum_expected, checksum_calculated);
            omap_usb_close(usb);
            return -1;
          }
        }
      }
    }
  }

  if(fwrite(chunkSizeMem, sizeof(char), chunkSize - remDataBufSz, fd) != (chunkSize - remDataBufSz))
  {
    strcpy(ERROR, "Write to file failed");
    omap_usb_close(usb);
    return -1;
  }

  return 0;
}


