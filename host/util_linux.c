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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <sys/time.h>

#include "fastboot.h"
#include "output.h"

/// some machines can't cope with the whole file at once for big files
#if 0 //def _DEBUG
#define CHUNKREAD
#define MB(x) (x * 1024 * 1024)
#endif


int GetLastError()
{
  return errno;
}

void get_my_path(char *path)
{
  char proc[64];
  char *x;

  sprintf(proc, "/proc/%d/exe", getpid());
  int err = readlink(proc, path, PATH_MAX - 1);

  if(err <= 0) {
    path[0] = 0;
  } else {
    path[err] = 0;
    x = strrchr(path,'/');
    if(x) x[1] = 0;
  }
}

void strrep(char *str)
{
  while(*str)
  {
    if(*str == '\\')
      *str = '/';
    str++;
  }
}

void *load_file_ex(const char *fn, unsigned *_sz, T_load_file_mode mode)
{
  char *data;
  int file_size, out_bytes;
  int fd;
  unsigned  do_add;

  strrep(fn);
  data = 0;
  fd = open(fn, O_RDONLY);
  if(fd < 0){
    host_logf("load_file: could not open '%s'", fn);
    return 0;
  }

  file_size = lseek(fd, 0, SEEK_END);
  if(file_size < 0){
    host_logf("load_file: could not determine size of '%s'", fn);
    goto oops;
  }

  if(lseek(fd, 0, SEEK_SET) < 0){
    host_logf("load_file: could not seek to start of file '%s'", fn);
    goto oops;
  }

  if (file_size > 0) {
    //  SSC/HEU: HACK, since BulkUSB.sys (CsstUSB.sys) can't handle to send packages of a multiplum of 64 bytes.
    //  In case the length we want to send is a multiplum of 64 bytes, we need to add an extra byte at the end.
    if (mode == add_nl || mode == add_null || mode == add_mod64_null && file_size % 64 == 0)
      do_add = 4;
    else if(mode == add_to_mod4)
      do_add = file_size % 4 ? (4 - file_size % 4) : 0;
    else
      do_add = 0;

    data = (char*) malloc(file_size + do_add);
    if (data == NULL) {
      host_logf("load_file: could not allocate %ld bytes", file_size );
      file_size = 0;
      goto oops;
    } else {
#ifdef CHUNKREAD
      int index = 0;
      int read_sz = 0;
      while(index != file_size)
      {
        read_sz = (file_size - index) > MB(10) ? MB(10) : file_size - index;
        if ( read(fd, data, read_sz) != read_sz )
        {
          host_logf("load_file: could not read %ld bytes from '%s'", file_size, fn);
          host_logf("last_error: %d", GetLastError()); ///<@todo handle for all error locations and print a text message in addition to error code
          file_size = 0;
          goto oops;
        }
        else
        {
          index += read_sz;
        }
      }
      if (do_add && file_size)
#else
      out_bytes = read(fd, data, file_size);
      if ( out_bytes <= 0 )
      {
        host_logf("load_file: could not read %ld bytes from '%s'", file_size, fn);
        host_logf("last_error: %d", GetLastError()); ///<@todo handle for all error locations and print a text message in addition to error code
        file_size = 0;
        goto oops;
      }
      else if (do_add)
#endif
      {
        if (mode == add_nl)
          data[file_size] = '\n';
        while (do_add--)
        {
          data[file_size] = 0;
          ++file_size;
        }
      }
    }
  }

  close(fd);

  if (out_bytes < file_size)
  {
    file_size = out_bytes;
  }

  if(_sz) *_sz = file_size;
  return data;

oops:
  close(fd);
  if(data != 0) free(data);
  return 0;
}


int getFileSize(const char *fn)
{
  int fd, file_size = 0;
  fd = open(fn, O_RDONLY);
  if(fd < 0){
    host_logf("load_file: could not open '%s'", fn);
    return -1;
  }
  file_size = lseek(fd, 0, SEEK_END);
  if(file_size < 0){
    host_logf("load_file: could not determine size of '%s'", fn);
  }
  close(fd);
  return file_size;
}

int load_file_ex2(const char *fn, char *data, unsigned sz, unsigned int offset, T_load_file_mode mode)

{
  int file_size, out_bytes;
  int fd, ret;
  unsigned  do_add;

  strrep(fn);
  fd = open(fn, O_RDONLY);
  if(fd < 0){
    host_logf("load_file: could not open '%s'", fn);
    return -1;
  }

  file_size = chunkSize;

  if (file_size > 0) {
    //  SSC/HEU: HACK, since BulkUSB.sys (CsstUSB.sys) can't handle to send packages of a multiplum of 64 bytes.
    //  In case the length we want to send is a multiplum of 64 bytes, we need to add an extra byte at the end.
    if (mode == add_nl || mode == add_null || mode == add_mod64_null && file_size % 64 == 0)
      do_add = 4;
    else if(mode == add_to_mod4)
      do_add = file_size % 4 ? (4 - file_size % 4) : 0;
    else
      do_add = 0;

    if((ret = lseek(fd, offset, SEEK_SET)) == -1){
      host_logf("load_file: could not seek to offset:%d of file '%s'", offset, fn);
      host_logf("last_error: %d %s. ret is %d", GetLastError(), strerror (errno), ret); ///<@todo handle for all error locations and print a text message in addition to error code
      file_size = -1;
      goto oops;
    }

    out_bytes = read(fd, data, file_size);
    if ( out_bytes < 0 )
    {
      host_logf("load_file: could not read %ld bytes from '%s'", file_size, fn);
      host_logf("last_error: %d", GetLastError()); ///<@todo handle for all error locations and print a text message in addition to error code
      file_size = -1;
      goto oops;
    }
    else if (do_add)
    {
      if (mode == add_nl)
        data[file_size] = '\n';
      while (do_add--)
      {
        data[file_size] = 0;
        ++file_size;
      }
    }
  }

  close(fd);

  if(out_bytes < file_size)
  {
    file_size = out_bytes;
  }

oops:
  close(fd);
  return file_size;
}

void *load_file(const char *fn, unsigned *_sz)
{
  return load_file_ex(fn, _sz, add_to_mod4);
}

int load_file_chunk(const char *fn, char *data, unsigned sz, unsigned int offset)
{
  return load_file_ex2(fn, data, sz, offset, add_to_mod4);
}

int save_file(const char *fn, const char *data, unsigned size)
{
  int fd;

  fd = open(fn, O_WRONLY | O_CREAT | O_TRUNC);
  if(fd < 0){
    host_logf("save_file: could not open for writing '%s'", fn);
    host_logf("last_error: %d", GetLastError());
    return 0;
  }

  if(lseek(fd, 0, SEEK_SET) != 0){
    host_logf("save_file: could not seek to start of file '%s'", fn);
    host_logf("last_error: %d", GetLastError());
    goto oops;
  }

  if(data == NULL || size == 0) goto oops;

  if(write(fd, data, size) != size)
  {
    host_logf("save_file: could not write %ld bytes to '%s'", size, fn);
    host_logf("last_error: %d", GetLastError()); ///<@todo handle for all error locations and print a text message in addition to error code
    goto oops;
  }

  return size;

oops:
  close(fd);
  return 0;
}

void sleepms(int mseconds)
{
  usleep(mseconds * 1000);
}

void getCurrentTime(struct timeval *clk)
{
  gettimeofday(clk, NULL);
}

void report_elapsed_time(struct timeval start, struct timeval end)
{
  long diff_sec, diff_usec;
  unsigned long total_msec, seconds, minutes;
  diff_sec  = end.tv_sec  - start.tv_sec;
  diff_usec = end.tv_usec - start.tv_usec;
  total_msec = (diff_sec * 1000) + (diff_usec/1000);
  seconds = total_msec / 1000;
  minutes = seconds / 60;

  host_logf("Elapsed time: %u:%02u.%03u", minutes, seconds % 60, total_msec % 1000);
}

void report_elapsed_time_rate(struct timeval start, struct timeval end, unsigned bytes)
{
  long diff_sec, diff_usec;
  unsigned long total_msec, seconds, minutes;
  diff_sec  = end.tv_sec  - start.tv_sec;
  diff_usec = end.tv_usec - start.tv_usec;
  total_msec = (diff_sec * 1000) + (diff_usec/1000);
  seconds = total_msec / 1000;
  minutes = seconds / 60;

  unsigned bps = total_msec ? ((bytes * 1000.0) / total_msec) : 0;

  if(bps) {
    host_logf("Elapsed time: %u:%02u.%03u (%u bytes/s)",
      minutes,
      seconds % 60, total_msec % 1000,
      bps);
  }
  else {
    host_logf("Elapsed time: %u:%02u.%03u (- bytes/s)",
      minutes,
      seconds % 60, total_msec % 1000);
  }
}
