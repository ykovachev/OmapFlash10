/*
* Copyright (C) 2009 Texas Instruments
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

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#ifdef _WIN32
#include <memory.h>
#include <windows.h>
#endif
#include "fastboot.h"
#include "output.h"

#define HOST_START_CHAR 175 ///< '>>'
#define TARGET_START_CHAR ((stdout_verbose) ? '\t' : 174)
#define TRACE_START_CHAR ' '

static BOOLEAN need_nl = FALSE;

void debug_vlogf(char const *format, va_list arg_ptr)
{
#ifndef NDEBUG
    if (show_debug_string) {
        char buf[1000];
        int result = VSNPRINTF(buf, sizeof buf - 1, format, arg_ptr);
        if (result > 0){
            strcat(buf, "\r\n");
#ifdef _WIN32
            OutputDebugString(buf);
#else
            printf(buf);
#endif
        }
    }
#endif
}

void debug_logf(char const *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    debug_vlogf(format, arg_ptr);
    va_end(arg_ptr);
}

void end_log()
{
    fprintf(output,"\n");
}

static void vlogf(int is_verbose, const char *prefix, const char *format, va_list arg_ptr, char start_char)
{
    if (stdout_verbose || !is_verbose)
    {
        fprintf(output,"\n%c %s", start_char, prefix);
        vfprintf(output, format, arg_ptr);
    }
    if (logfile) 
    {
        fprintf(logfile,"%c %s", start_char, prefix);
        vfprintf(logfile, format, arg_ptr);
        fprintf(logfile,"\n");
    }
    debug_vlogf(format, arg_ptr);
    need_nl = TRUE;
}

void host_vlogf_ex(int is_verbose, const char *prefix, const char *format, va_list arg_ptr)
{
    vlogf(is_verbose, prefix, format, arg_ptr, HOST_START_CHAR);
}

void host_vlogf(const char *prefix, const char *format, va_list arg_ptr)
{
    vlogf(NONVERBOSE, prefix, format, arg_ptr, HOST_START_CHAR);
}

void host_logf(const char *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    host_vlogf("", format, arg_ptr);
    va_end(arg_ptr);
}

void host_logf_ex(int is_verbose, const char *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    host_vlogf_ex(is_verbose, "", format, arg_ptr);
    va_end(arg_ptr);
}

void target_logf(const char *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    vlogf(NONVERBOSE, "", format, arg_ptr, TARGET_START_CHAR);
    va_end(arg_ptr);
}

void target_logf_ex(int is_verbose, const char *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    vlogf(is_verbose, "", format, arg_ptr, TARGET_START_CHAR);
    va_end(arg_ptr);
}

#define STAT_SIZE 20

static void progress_vlogf(unsigned init, unsigned long long current, unsigned long long target, const char *format, va_list arg_ptr, char start_char)
{
    char stat[STAT_SIZE + 1];
    int len;
    unsigned c;
    stat[STAT_SIZE] = 0;

    if(target)
    {
      if(target < STAT_SIZE)
      {
        c = (STAT_SIZE * current) / target;
      }
      else
      {
        c = current  / (target / STAT_SIZE);
      }
      if (c > STAT_SIZE)
          c = STAT_SIZE;

      memset(stat, ':', c);
      memset(stat + c, '.', STAT_SIZE - c);

      len  = fprintf(output, "%s\r%c ", init || need_nl ? "\n" : "", start_char);
      len += vfprintf(output, format, arg_ptr);
      len += fprintf(output, " %s [%I64u]", stat, current);
      if (logfile) 
      {
    	  fprintf(logfile,"%c ", start_char);
          vfprintf(logfile, format, arg_ptr);
          fprintf(logfile," %s [%I64u]\n", stat, current);
      }
      need_nl = FALSE;
    }
}

void host_progress_logf(unsigned init, unsigned long long current, unsigned long long target, const char *format, ...)
{
    static U64 last_current = 0;
    va_list arg_ptr;

    /* No more than 500 updates of status during a download */
    if(init || (last_current > current) || ((current - last_current) >= (target / 500)) || (current == target))
    { 
        va_start(arg_ptr, format);
        progress_vlogf(init, current, target, format, arg_ptr, HOST_START_CHAR);
        va_end(arg_ptr);
        last_current = current;
    }
}

void target_progress_logf(unsigned init, unsigned long long current, unsigned long long target, const char *format, ...)
{
    va_list arg_ptr;

    va_start(arg_ptr, format);
    progress_vlogf(init, current, target, format, arg_ptr, TARGET_START_CHAR);
    va_end(arg_ptr);
}

int current_trace_id;
void trace_vlogf(int line, char *format, va_list arg_ptr)
{
    char buf[20];
    SNPRINTF(buf,sizeof buf,"%05d-%03d-", line, current_trace_id);
    vlogf(VERBOSE, buf, format, arg_ptr, TRACE_START_CHAR);
    if (current_trace_id++ == 149) 
    {
        current_trace_id = current_trace_id;
    }
}

void trace_logf(int line, char *format, ...)
{
    va_list arg_ptr;

    va_start(arg_ptr, format);
    trace_vlogf(line, format, arg_ptr);
    va_end(arg_ptr);
}
