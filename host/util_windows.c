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
#include <time.h>
#include <limits.h>
#include <windows.h>
#include "fastboot.h"
#include "util_windows.h"
#include "output.h"

/// some machines can't cope with the whole file at once for big files
#if 0 //def _DEBUG
#define CHUNKREAD
#define MB(x) (x * 1024 * 1024)
#endif

void get_my_path(char exe[PATH_MAX])
{
  char*  r;

  GetModuleFileName( NULL, exe, PATH_MAX-1 );
  exe[PATH_MAX-1] = 0;
  r = strrchr( exe, '\\' );
  if (r)
    *r = 0;
}

void strrep(char *str)
{
  while(*str)
  {
    if(*str == '/')
      *str = '\\';
    str++;
  }
}

void *load_file_ex(const char *fn, unsigned *_sz, T_load_file_mode mode)
{
  HANDLE    file;
  char     *data;
  DWORD     file_size;
  unsigned  do_add;

  strrep(fn);
  file = CreateFile( fn,
    GENERIC_READ,
    FILE_SHARE_READ,
    NULL,
    OPEN_EXISTING,
    0,
    NULL );

  if (file == INVALID_HANDLE_VALUE) {
    host_logf("load_file: could not open '%s'", fn);
    return NULL;
  }

  file_size = GetFileSize( file, NULL );
  data      = NULL;

  if (file_size > 0) {
    //  SSC/HEU: HACK, since BulkUSB.sys (CsstUSB.sys) can't handle to send packages of a multiplum of 64 bytes.
    //  In case the length we want to send is a multiplum of 64 bytes, we need to add an extra byte at the end.
    if (mode == add_nl || mode == add_null || mode == add_mod64_null && file_size % 64 == 0)
      do_add = 4;
    else if(mode == add_to_mod4)
      do_add = file_size % 4 ? (4 - file_size % 4) : 0;
    else
      do_add = 0;

    data = (char*) malloc( file_size + do_add);
    if (data == NULL) {
      host_logf("load_file: could not allocate %ld bytes", file_size );
      file_size = 0;
    } else {
      DWORD  out_bytes;
#ifdef CHUNKREAD
      DWORD  index = 0;

      while(index != file_size)
      {
        SetFilePointer(file, index, NULL, FILE_BEGIN);
        if(!ReadFile(file, data + index, file_size - index > MB(10) ? MB(10) : file_size - index, &out_bytes, NULL))
        {
          host_logf("load_file: could not read %ld bytes from '%s'", file_size, fn);
          host_logf("last_error: %d", GetLastError()); ///<@todo handle for all error locations and print a text message in addition to error code
          free(data);
          data      = NULL;
          file_size = 0;
          break;
        }
        else
        {
          index += out_bytes;
        }

      }
      if (do_add && file_size)
#else            
      if ( !ReadFile( file, data, file_size, &out_bytes, NULL ) ||
        out_bytes != file_size )
      {
        host_logf("load_file: could not read %ld bytes from '%s'", file_size, fn);
        host_logf("last_error: %d", GetLastError()); ///<@todo handle for all error locations and print a text message in addition to error code
        free(data);
        data      = NULL;
        file_size = 0;
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
  CloseHandle( file );

  if (_sz)
    *_sz = (unsigned) file_size;
  return  data;
}


int getFileSize(const char *fn)
{
  HANDLE    file;
  DWORD     file_size;

  strrep(fn);
  file = CreateFile( fn,
    GENERIC_READ,
    FILE_SHARE_READ,
    NULL,
    OPEN_EXISTING,
    0,
    NULL );

  if (file == INVALID_HANDLE_VALUE) {
    host_logf("load_file: could not open '%s'", fn);
    return -1;
  }

  file_size = GetFileSize( file, NULL );
  if(file_size == INVALID_FILE_SIZE)
  {
    host_logf("load_file: GetFileSize failed. Error: %d", GetLastError());
    CloseHandle( file );
    return -1;
  }

  CloseHandle( file );
  return file_size;
}

// Function that reads from a specific offset and chunkSize.
//void load_file_ex2(const char *fn, char *data, unsigned *_sz, unsigned int offset, T_load_file_mode mode)
int load_file_ex2(const char *fn, char *data, unsigned sz, unsigned int offset, T_load_file_mode mode)
{
  HANDLE    file;
  //DWORD     file_size = chunkSize;
  DWORD     file_size = sz;
  unsigned  do_add;
  DWORD  out_bytes = 0, res;
  
  LARGE_INTEGER loffset;

  strrep(fn);
  file = CreateFile( fn,
    GENERIC_READ,
    FILE_SHARE_READ,
    NULL,
    OPEN_EXISTING,
    0,
    NULL );

  if (file == INVALID_HANDLE_VALUE) {
    host_logf("load_file: could not open '%s'", fn);   
    //*_sz = 0;
    return -1;
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

    loffset.QuadPart = offset;
    loffset.LowPart = SetFilePointer(file, loffset.LowPart, &loffset.HighPart, FILE_BEGIN);
    if(loffset.LowPart == INVALID_SET_FILE_POINTER && GetLastError() != NO_ERROR)
    {
      host_logf("load_file_ex2: could not set file pointer at offset %u", offset);
      host_logf("last_error: %d", GetLastError()); 
      CloseHandle( file );
      //*_sz = 0;
      return -1;
    }

    if (!ReadFile( file, data, file_size, &out_bytes, NULL ) || (out_bytes <= 0))
    {
      res = GetLastError();
      host_logf("load_file: could not read %u bytes from '%s'", file_size, fn);
      host_logf("last_error: %d", GetLastError()); ///<@todo handle for all error locations and print a text message in addition to error code
      CloseHandle( file );
      //*_sz = 0;
      return -1;
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
  CloseHandle( file );
  if (out_bytes < file_size)
  {
    file_size = out_bytes;
  }

  return file_size;
}         

void *load_file(const char *fn, unsigned *_sz)
{
  return load_file_ex(fn, _sz, add_to_mod4);
}

int load_file_chunk(const char *fn, char *data, unsigned sz, unsigned offset)
{
  return load_file_ex2(fn, data, sz, offset, add_to_mod4);
}

int save_file(const char *fn, const char *data, unsigned size)
{
  HANDLE    file;
  DWORD  out_bytes;

  file = CreateFile( fn,
    GENERIC_WRITE,
    FILE_SHARE_WRITE,
    NULL,
    CREATE_ALWAYS,
    0,
    NULL );

  if (file == INVALID_HANDLE_VALUE) {
    LPVOID lpMsgBuf;
    //LPVOID lpDisplayBuf;
    DWORD dw = GetLastError(); 

    FormatMessage(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | 
      FORMAT_MESSAGE_FROM_SYSTEM |
      FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      dw,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPTSTR) &lpMsgBuf,
      0, NULL );

    host_logf("save_file: could not open '%s'", lpMsgBuf);
    return 0;
  }
  SetLastError(0); ///<@ GetLastError() might return 183 when reopening an existing file

  if ( !WriteFile( file, data, size, &out_bytes, NULL ) ||
    out_bytes != size )
  {
    host_logf("save_file: could not write %ld bytes to '%s'", size, fn);
    return 0;
  }

  CloseHandle( file );
  return size;
}

void sleep(int seconds)
{
  Sleep(seconds * 1000);
}
void sleepms(int mseconds)
{
  Sleep(mseconds);
}
void getCurrentTime(clock_t *clk)
{
  *clk = clock();
}

void report_elapsed_time(clock_t start, clock_t end)
{
  ///@todo there is some wrong here when times wrap around
  /*
  got following output for something running a few hours but
  if all 32 bits are used in clock_t wraparound should ocur
  first time after 25 days and then every 50 days
  (the change is warp around time is due clock_t beeing signed)
  � Elapsed time: -9:-58.-717 (283 bytes/s)
  � Elapsed time: -11:-14.-973
  */
  clock_t delta = end - start;
  clock_t mili_seconds = (delta * 1000) / CLOCKS_PER_SEC;
  clock_t seconds = mili_seconds / 1000;
  clock_t minutes = seconds / 60;
  host_logf_ex(NONVERBOSE, "Elapsed time: %u:%02u.%03u", minutes, seconds % 60, mili_seconds % 1000);
}

void report_elapsed_time_rate(clock_t start, clock_t end, unsigned bytes)
{
  clock_t delta = end - start;
  clock_t mili_seconds = (delta * 1000) / CLOCKS_PER_SEC;
  clock_t seconds = mili_seconds / 1000;
  clock_t minutes = seconds / 60;

    unsigned bps = mili_seconds ? ((bytes * 1000.0) / mili_seconds) : 0;
    if(bps) {
        host_logf_ex(VERBOSE, "Elapsed time: %u:%02u.%03u (%u bytes/s)",
                     minutes,
                     seconds % 60, mili_seconds % 1000,
                     bps);
    }
    else {
        host_logf_ex(VERBOSE, "Elapsed time: %u:%02u.%03u (- bytes/s)",
                     minutes,
                     seconds % 60, mili_seconds % 1000);
    }
}
