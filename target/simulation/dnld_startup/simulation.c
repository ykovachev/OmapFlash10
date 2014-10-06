/**
 * @file simulation.c
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

#include <stdio.h>
#include "simulation.h"
#include "csst_tgt.h"
#ifdef SECOND_DNLD_BUILD
#include "uart.h"
#include "dnld.h"
#include "disp.h"
#include "romapi.h"
#endif

void _register_lock (void (*dal_lock)(void))
{
    fprintf(stderr,"_register_lock\n");
}

void _register_unlock (void (*dal_unlock)(void))
{
    fprintf(stderr,"_register_unlock\n");
}

void simulation_out_regb (char* offset_text, char* value_text, U32 offset, U8 value)
{
    fprintf(stderr,"out_regb(%s, %s): *(U8*)%#08X = %#02X\n",offset_text,value_text,offset,value);
}

void simulation_out_regs (char* offset_text, char* value_text, U32 offset, U16 value)
{
    fprintf(stderr,"out_regs(%s, %s): *(U16*)%#08X = %#04X\n",offset_text,value_text,offset,value);
}

void simulation_out_regl (char* offset_text, char* value_text, U32 offset, U32 value)
{
    fprintf(stderr,"out_regl(%s, %s): *(U32*)%#08X = %#08X\n",offset_text,value_text,offset,value);
}

U8 simulation_in_regb (char* offset_text, U32 offset)
{
    fprintf(stderr,"in_regb(%s): *(U8*)%#08X == XX\n",offset_text,offset);
    return 0;
}

U16 simulation_in_regs (char* offset_text, U32 offset)
{
    fprintf(stderr,"in_regs(%s): *(U16*)%#08X == XXXX\n",offset_text,offset);
    return 0;
}

U32 simulation_in_regl (char* offset_text, U32 offset)
{
    fprintf(stderr,"in_regl(%s): *(U32*)%#08X == XXXXXXXX\n",offset_text,offset);
    return 0;
}

#ifndef EMMC_DRV
#define SPLIT_WORD(val) (val & 0xFF), ((val >> 8) & 0xFF)
#define SPLIT_DWORD(val) SPLIT_WORD(val), SPLIT_WORD(val >> 16)

#ifdef SECOND_DNLD_BUILD
//TODO values set to 0 should be given proper value 
// (if zero is the proper value 0x00 is used) 
U16 sibley_QRY[] = {
    //0x00
    0x0089,0x88B1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
    //0x20
    'Q','R','Y',SPLIT_WORD(0x0200),0,0,0, 0,0,0,0, 0,0,0,0,
    //0x40
    0,0,0,0, 0,0,0,0, 0,0,0,0, 0x01,0xFF,0x01,0x00,
    //0x60
    0x04,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
};

volatile U16 *do_flash_read(U32 addr)
{
    static U16 i = 0x0;
    U32 a = addr;
    if (a >= 0x10000000 && a < 0x20000000 /*&& QRY*/) {
        a -= 0x10000000; // remove base part
        a &= ~0x08000000; // return same values for either sibley1 or sibley2
        a /= 2; // convert to word address
        i = sibley_QRY[a];
    }
    fprintf(stderr,"flash_read(%#08X) == %#04X\n",addr,i);
    return &i;    
}
#endif 

void do_flash_write(U32 addr, U16 data)
{
	//TODO need logic to swap between QRY data and user data (and possible also different query modes)
    fprintf(stderr,"flash_write(%#08X,%#04X)\n",addr,data);
}
#endif //EMMC_DRV

volatile U8 *do_nand8_read(U32 addr)
{
    static U8 i;
    fprintf(stderr,"b@%#08X\n",addr);
    return &i;    
}

void do_nand8_write(U32 addr, U8 data)
{
    fprintf(stderr,"@%#08X = %#02X\n",addr,data);
}

void irq_disable (void)
{
    fprintf(stderr,__FUNCTION__"\n");
}

void irq_enable (void)
{
    fprintf(stderr,__FUNCTION__"\n");
}

void dal_fiq_disable (void)
{
    fprintf(stderr,__FUNCTION__"\n");
}

void dal_fiq_enable (void)
{
    fprintf(stderr,__FUNCTION__"\n");
}

volatile U8 uart_register[256]; 
volatile U8 *uart_reg(U32 b,U8 offset)
{
    fprintf(stderr,"uart_reg(%#08X,%#02X)\n", b, offset);
    return &uart_register[offset];
}

void write_cp15_c1(unsigned int c1)
{
    fprintf(stderr,__FUNCTION__"(%d)\n", c1);
}

unsigned int read_cp15_c1(void)
{
    fprintf(stderr,__FUNCTION__" return 42\n");
	return 42;
}

void arm_icache_flush(void)
{
    fprintf(stderr,__FUNCTION__"\n");
}

#ifdef SECOND_DNLD_BUILD
enum call_romapi_state_t {other, rx, tx} call_romapi_state;
int simulation_call_romapi(char* type, int address, const PeripheralDesc_t *pPeripheralDesc)
{
	if (address == 0x14328 || address == 0x14250) {
		call_romapi_state = tx;
		fprintf(stderr,__FUNCTION__ ": %s(%s)\n", type, (char*)(pPeripheralDesc->Address));
	}
	else {
		if (address == 0x1432C || address == 0x14254) {
			if (call_romapi_state == tx) {
				memset((void*)pPeripheralDesc->Address, 0, pPeripheralDesc->Size);
				fprintf(stderr,__FUNCTION__ ": %s ack\n", type);
			}
			else {
				fprintf(stderr,__FUNCTION__ ": %s\n", type);
			}
			call_romapi_state = rx;
		}
		else {
			call_romapi_state = other;
			fprintf(stderr,__FUNCTION__ ": %s\n", type);
		}
	}
	if (pPeripheralDesc->pCallBackFunction)
		pPeripheralDesc->pCallBackFunction((void*)pPeripheralDesc);
	return 0;
}
#else
int simulation_call_romapi(char* type, int address, const void *pPeripheralDesc)
{
  return 0;
}
#endif SECOND_DNLD_BUILD

int simulation_call_romapi_1(char* type, int address, int a)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

int simulation_call_romapi_2(char* type, int address, int a, int b)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

int simulation_call_romapi_3(char* type, int address, int a, int b, int c)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

int simulation_call_romapi_4(char* type, int address, int a, int b, int c, int d)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

int simulation_call_romapi_5(char* type, int address, int a, int b, int c, int d, int e)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

int simulation_call_romapi_6(char* type, int address, int a, int b, int c, int d, int e, int f)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

int simulation_call_romapi_7(char* type, int address, int a, int b, int c, int d, int e, int f, int g)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

int simulation_call_romapi_8(char* type, int address, int a, int b, int c, int d, int e, int f, int g, int h)
{
	fprintf(stderr,__FUNCTION__ ": %s\n", type);
    return 0;
}

U32 HEAP_SIZE;
U32 FLASH_DRV_LOCATION;

