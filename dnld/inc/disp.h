/**
 * @file disp.h
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

/*==== DECLARATION CONTROL =================================================*/
#ifndef TGT_DISP_H
#define TGT_DISP_H

/*==== INCLUDES ============================================================*/
#include <stdarg.h>
#include <string.h>
#include "types.h"
#include "config.h"

/*==== CONSTS ==============================================================*/

/*==== TYPES ===============================================================*/
/*==== EXPORTS =============================================================*/
#ifdef __cplusplus
extern "C" {
#endif
/** put used by snprintf
  * @param data where to put
  * @param c char to put
  */
enum vxprintf_put_command_t { vxprintf_put_restart = 0x8000000 };
typedef int vxprintf_put_t(void *data, int c);

enum T_xprintf_flags
{
    VXPRINTF_STANDARD = 0,
    VXPRINTF_NO_REQUIRE_HASH = 1
};

#define XSEND_FLAGS_SHIFT 8
enum T_xsend_flags
{
    XSEND_FLAGS_ACKED = 0,
    XSEND_FLAGS_UNACKED = (1 << XSEND_FLAGS_SHIFT),
    XSEND_DEBUG_PREFIX = (2 << XSEND_FLAGS_SHIFT) //prefix line with # for easy indentification
};

int vxprintf (vxprintf_put_t *put, void *buf, int flags, const char *format, va_list arg_ptr);
int vxsnprintf (char *buf, size_t buf_size, int flags, const char *format, va_list arg_ptr);
int my_vsnprintf (char *buf, size_t buf_size, const char *format, va_list arg_ptr);
int my_snprintf (char *buf, size_t buf_size, const char *format, ...);
int dry_vprintf (const char *format, va_list arg_ptr);
int dry_printf (const char *format, ...);

//not intended for public consumption
void xsend (char * prefix, int flags, char *format, ...);
void send_ok (char *format, ...);
void send_fail (char *format, ...);
void send_bug (char *format, ...);
void vxsend_info (int flags, const char *format, va_list arg_ptr);
void send_info (char *format, ...);
void send_status(const char * description, U64 current, U64 target);
void send_out_of_memory ();
void send_upload(void *data, U32 size);


BOOLEAN is_uart_connection();

void disp_deinit(void);

#ifdef __cplusplus
}
#endif
#endif

/*==== END OF FILE ===========================================================*/

