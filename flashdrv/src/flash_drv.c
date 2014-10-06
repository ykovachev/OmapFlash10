/**
 * @file flash_drv.c
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


/*==== INCLUDES ==============================================================*/
#include <string.h>
#include "types.h"
#include "flash_drv.h"
#include "disp.h"

#undef malloc
#undef free

void *drv_malloc(unsigned int tSize)
{
    return get_config()->drv_malloc(tSize);
}

void *drv_calloc(size_t num, size_t size)
{
    size_t s = num * size;
    void *result = drv_malloc(s);
    if (result) 
    {
        drv_memset(result, 0, s);
    }
    return result;
}

void drv_free(void *packet)
{
    get_config()->drv_free(packet);
}
S32 drv_memcmp(const void * b1, const void * b2, size_t size)
{
  return get_config()->drv_memcmp(b1, b2, size);
}

S32 drv_strcmp(const char * s1, const char * s2)
{
  return get_config()->drv_strcmp(s1, s2);
}

S32 drv_strncmp(const char * s1, const char * s2, size_t size)
{
  return get_config()->drv_strncmp(s1, s2, size);
}

void * drv_memcpy(void * tgt, const void * src, size_t size)
{
  return get_config()->drv_memcpy(tgt, src, size);
}

char * drv_strcpy(char * tgt, const char * src)
{
  return get_config()->drv_strcpy(tgt, src);
}

size_t drv_strlen(const char * str)
{
  return get_config()->drv_strlen(str);
}

void * drv_memset(void * tgt, int value, size_t size)
{
  return get_config()->drv_memset(tgt, value, size);
}

#ifndef SIMULATION
void *malloc(unsigned int tSize)
{
    return drv_malloc(tSize);
}

void *calloc(size_t num, size_t size)
{
    return drv_calloc(num, size);
}

void free(void *packet)
{
    drv_free(packet);
}
#endif //SIMULATION

void wait_microsec(U32 microsec)
{
    get_config()->wait_microsec(microsec);
}

U32 wait_microsec_ex(U32 timeout_microsec, T_wait_microsec_ex_callback *callback, void *data)
{
    #ifndef SIMULATION
    U32 address = (U32)get_setup_const();
    callback = (T_wait_microsec_ex_callback *)(((U32)callback & 0x00FFFFFF) | (address & 0xFF000000));
    #endif
    return get_config()->wait_microsec_ex(timeout_microsec, callback, data);
}

BOOLEAN check_timeout_ms(U8 id, U32 millisec)
{
    return get_config()->loop_timeout_ms(id, millisec);
}


void dbg_vxprintf(int flags, const char *format, va_list arg_ptr)
{
    get_config()->dbg_vxprintf(flags, format, arg_ptr);
}

void dbg_printf(const char *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    dbg_vxprintf(VXPRINTF_STANDARD, format, arg_ptr);
    va_end(arg_ptr);
}

void drv_vxsend_info(int flags, char *format, va_list arg_ptr)
{
    get_config()->vxsend_info(flags, format, arg_ptr);
}

void drv_send_info(char *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    drv_vxsend_info(VXPRINTF_STANDARD, format, arg_ptr);
    va_end(arg_ptr);
}

//void drv_xsend_info(int flags, char *format, ...)
//{
//    va_list arg_ptr;
//    va_start(arg_ptr, format);
//    drv_vxsend_info(flags, format, arg_ptr);
//    va_end(arg_ptr);
//}

void drv_send_status(char * description, U64 current, U64 target)
{
    get_config()->send_status(description, current, target);
}

void drv_debug_signal(unsigned int id, unsigned int set)
{
    get_config()->drv_debug_signal(id, set);
}

void drv_log_event(char * text)
{
    get_config()->log_event(text);
}

BOOLEAN omap3(void)
{
    return get_config()->processor[4] == '3';
}

BOOLEAN omap4(void)
{
  return get_config()->processor[4] == '4';
}

BOOLEAN omap5(void)
{
  return get_config()->processor[4] == '5';
}

