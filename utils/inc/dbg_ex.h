/**
 * @file dbg_ex.h
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

/*==== DECLARATION CONTROL ==================================================*/

#ifndef CSST_DBG_EX_H
#define CSST_DBG_EX_H

/*==== INCLUDES =============================================================*/

#include "dbg_common.h"

/*==== CONSTS ==============================================================*/
#define DBGTRACE_CRITICAL 5

#ifdef LOCATION_TRACE
#define LOCATION_VALUES __FILE__,__FUNCTION__,__LINE__, 
#define LOCATION_PARAM_DECL const char * file_name, const char * function_name, U32 line_num
#define LOCATION_PARAM_USE file_name, function_name, line_num,
#else
#define LOCATION_VALUES
#define LOCATION_PARAM_DECL
#define LOCATION_PARAM_use
#endif

S32 dbg_print_msg(LOCATION_PARAM_DECL U8 level, const char * format,...);

#ifdef _MSC_VER
///@deprecated use DBG_PRINT
#define DBG_PRINT(...) dbg_print_msg(LOCATION_VALUES __VA_ARGS__)
#else 
//#ifdef __TMS470__
#define DBG_PRINT(args...) dbg_print_msg(LOCATION_VALUES args)
//#else 
//S32 dbg_print_msg(U8 level,U8 Moduleid,const char * format,...);
//#ifdef LOCATION_TRACE
//void dbg_update_flow_params(const char * file_name, const char * function_name, U32 line_num);
//#define DBG_PRINT	(dbg_update_flow_params(__FILE__,__FUNCTION__,__LINE__),dbg_print_msg)
//#else //LOCATION_TRACE
//#define DBG_PRINT	dbg_print_msg
//#endif //LOCATION_TRACE
//#endif
#endif

///@deprecated use DBG_PRINT
#define dbg_print DBG_PRINT


/* vishnu */
S32 tgt_con_printf(char * fmt,...);
S32 host_con_printf(char * fmt,...);

#define DBG_LEVEL_OFF      TRACER_LEVEL_OFF //0
#define DBG_LEVEL_INFO	   5 //TRACER_LEVEL_INFO //2
#define DBG_LEVEL_MINOR    TRACER_LEVEL_MINOR//4
#define DBG_LEVEL_MAJOR    TRACER_LEVEL_MAJOR//3
#define DBG_LEVEL_ALERT    TRACER_LEVEL_ALERT//2
#define DBG_LEVEL_CRITICAL TRACER_LEVEL_CRITICAL//1

#define DBG_FLOW		6
#define DBG_TRACE		7

#ifdef _MSC_VER
#define DO_DBG_PRINT(...) DBG_PRINT(__VA_ARGS__)
#define NO_DBG_PRINT(...) ((void)0)
#else
#define DO_DBG_PRINT(ARGS...) DBG_PRINT(ARGS)
#define NO_DBG_PRINT(ARGS...) ((void)0)
#endif



/*==== EXPORTS =============================================================*/

#endif /* CSST_DIAG_H */

/*==== END OF FILE ==========================================================*/
