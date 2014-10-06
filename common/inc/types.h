/**
 * @file types.h
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

#ifndef TYPES_H
#define TYPES_H

/*==== INCLUDES ============================================================*/
/*TBD*/

/*==== CONSTS ==============================================================*/

/*TBD*/

/*==== TYPEDEFINES ===============================================================*/

typedef unsigned char U8;
typedef signed char S8;
typedef char C8;
typedef U8 UWORD8;
typedef S8 WORD8;
typedef U8 BOOLEAN;
typedef volatile U8 VU8;
//#ifndef _MSC_VER
//typedef U8 BOOL;
//#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

typedef unsigned short int        U16;
typedef signed short int          S16;
typedef U16                       UWORD16;
typedef S16                       WORD16;
typedef volatile U16              VU16;

typedef float                     F32;

typedef volatile unsigned char *  PREG_U8;
typedef volatile signed char *    PREG_S8;
typedef volatile unsigned short * PREG_U16;
typedef volatile short *          PREG_S16;
typedef volatile unsigned long *  PREG_U32;
typedef volatile long *           PREG_S32;

typedef unsigned long             U32;
typedef signed long               S32;
typedef volatile U32              VU32;

typedef unsigned long long        U64;
typedef signed long long          S64;
typedef volatile U64              VU64;

#ifndef _MANAGED
#ifndef NULL
#ifdef __cplusplus
#define NULL                      0
#else
#define NULL                      ((void *)0)
#endif
#endif
#endif


#ifdef _MSC_VER
#define DEPRECATED(text)          __declspec(deprecated(text))
#else
#define DEPRECATED(text)
#endif


/*====== FUNCTION PROTOTYPES==================================================*/

#endif /* TYPES_H */

