/**
 * @file flash_drv.h
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
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "flash_drv.h"


unsigned int drv_pow(unsigned int x, unsigned int y)
{
  unsigned int ret = 1;
  if(y > 0)
  {
    while(y--) ret *= x;
  }
  return(ret);
}

static unsigned int drv_get_value(char * s, T_driver_config * config)
{
  char * p = s;
  char * e;
  unsigned int ret = 0;
  int base = 10;
  int exp;

  if(p[0] == '0' && p[1] == 'x')
  {
    base = 16;
    p += 2;
  }

  e = p;

  while(*e != ' ' && *e)
  {
    e++;
  }

  exp = (e - p) - 1;
  
  while(*p && *p != ' ')
  {
    if((*p >= '0') && (*p <= '9'))
    {
      ret += (*p - '0') * drv_pow(base, exp--);
    }
    else if((((*p >= 'A') && (*p <= 'F')) || ((*p >= 'a') && (*p <= 'f'))) && (base == 16))
    {
      ret += (*p - (((*p >= 'A') && (*p <= 'F')) ? 'A' : 'a') + 10) * drv_pow(base, exp--);
    }
    else
    {
      config->send_info("Driver configuration: ERROR in value %s", s);
    }
    p++;
  }

  return(ret);
}

static int drv_find_element(char * string, char ** result)
{
  int length = 0;
  *result = NULL;

  while(*string)
  {
    if(((*string >= 0x41) && (*string <= 0x5a)) || 
       ((*string >= 0x61) && (*string <= 0x7a)) ||
       ((*string >= 0x30) && (*string <= 0x39)) ||
       (*string == '_'))
    {
      if(*result == NULL)
      {
        *result = string;
      }
      length++;
      string++;
    }
    else
    {
      if(*result == NULL)
      {
        string++;
      }
      else
      {
        break;
      }
    }
  }

  return(length);
}

int drv_parse_config(const T_driver_setup_const * setup_const, T_driver_setup_var * setup_var, char * cstring, T_driver_config * config)
{
  int index;
  char * element;
  int element_length;
  
  index = 0;

  while(setup_const[index].tag[0])
  {  
    setup_var[index].value = 0xFFFFFFFF;
    setup_var[index].valid = FALSE;
    index++;
  }

  /* Parse configuration string */

  element = cstring;
  
  while(1)
  {
    index = 0;
    
    element_length = drv_find_element(element, &element);
    
    if(!element_length)
    {
      break;
    }

    while(setup_const[index].tag[0])
    {  
      char * sw = element;
      if(config->starts_with(&sw, setup_const[index].tag))
      {
        element += element_length;
        element_length = drv_find_element(element, &element);
        setup_var[index].value = drv_get_value(element, config);
        setup_var[index].valid = TRUE;
        break;
      }
      index++;
    }

    if(!setup_const[index].tag[0])
    {
      config->send_info("Driver configuration: Unrecognized content at '%s'", element);
    }

    element += element_length;
  }

  index = 0;

  while(setup_const[index].tag[0])
  {
    if((setup_const[index].mandatory == TRUE) && (setup_var[index].valid == FALSE))
    {
      config->send_info("Driver configuration: %-@15s NOT FOUND", setup_const[index].tag);
      return FLASH_DRV_ERROR;
    }

    if(setup_var[index].valid == TRUE)
    {
      config->dbg_printf("Driver configuration: %-@15s %#0*x", setup_const[index].tag, 2 * (1 + setup_var[index].value && 0xFF00 + setup_var[index].value && 0xFF0000 + setup_var[index].value && 0xFF000000), setup_var[index].value);
    }
    index++;
  }

  return FLASH_DRV_SUCCESS;
}

T_driver_config *config_storage;

#ifdef SIMULATION
static T_driver_config **ref_config(void)
{
	return &config_storage;
}

const T_driver_setup_const *get_setup_const(void)
{
    return setup_const;
}

T_driver_setup_var *get_setup_var(void)
{
    return setup_var;
}
#else
int pc()
{
    asm ("  mov       r0, pc ");
	asm ("  sub       r0, #4 ");
    asm ("  bx        r14    ");

	///unused but required by compiler to avoid warning
    return 0;
}

static T_driver_config **ref_config(void)
{
#if 1
    // find running location
    char *p = (char*)pc();

    // find load address of image
    p -= ((int)pc & ~ 1) - (int)&driver_if;

    // add offset within image
    p += (int)&config_storage - (int)&driver_if; 

    // make it proper type
    return (T_driver_config **)p;
#else
    return (T_driver_config **)(((int)pc() & ~ 1) + (int)&config_storage);
#endif
}

const T_driver_setup_const *get_setup_const(void)
{
#if 1
    // find running location
    char *p = (char*)pc();

    // find load address of image
    p -= ((int)pc & ~ 1) - (int)&driver_if;

    // add offset within image
    p += (int)&setup_const - (int)&driver_if; 

    // make it proper type
    return (T_driver_setup_const *)p;
#else
    return (T_driver_setup_const *)(((int)pc() & ~ 1) + (int)&setup_const);
#endif
}

T_driver_setup_var *get_setup_var(void)
{
#if 1
    // find running location
    char *p = (char*)pc();

    // find load address of image
    p -= ((int)pc & ~ 1) - (int)&driver_if;

    // add offset within image
    p += (int)&setup_var - (int)&driver_if; 

    // make it proper type
    return (T_driver_setup_var *)p;
#else
    return (T_driver_setup_var *)(((int)pc() & ~ 1) + (int)&setup_var);
#endif
}
#endif

///get config_storage in an image location independ way
T_driver_config *get_config(void)
{
    return *ref_config();
}

///set config_storage in an image location independ way
void set_config(T_driver_config * config)
{
    *ref_config() = config;
}

T_driver_data *get_driver_data(void)
{
    return (struct T_driver_data*)get_config()->data;
}
