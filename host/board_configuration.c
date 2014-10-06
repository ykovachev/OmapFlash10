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
#include <memory.h>
#include <malloc.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "fastboot.h"
#include "output.h"
#include "memory_partitions.h"
#include "board_configuration.h"


#define MAX_LINE_LENGTH  1000
#define MAX_DEFINITIONS 2000
#define MAX_CONFIGURATION_ELEMENTS 5000

typedef enum
{
  FILEPATH,
  COMMAND_VALUE,
  PARAMETER_STRING,
  PARTITION_NAME
} T_element;

/* Operation codes known by the target source code */

#define OPCODE_MASK 0xFFFF0000
#define N_MASK      0x0000FFFF

typedef enum
{
  OPC_NOP          = 0x00000000, // no opcode
  OPC_WRITE        = 0x00010000, // write a value to a register
  OPC_WRITE_N      = 0x00020000, // write a value to n registers
  OPC_MODIFY       = 0x00100000, // modify the value of a register
  OPC_MODIFY_N     = 0x00200000, // modify the value of n registers
  OPC_MODIFY_N_M   = 0x00300000, // modify the value of n registers with fixed mask
  OPC_MODIFY_N_M_V = 0x00400000, // modify the value of n registers with fixed mask and value
  OPC_POLL_ZERO    = 0x01000000, // poll a register value until zero
  OPC_POLL_NZERO   = 0x02000000, // poll a register value until not zero
  OPC_POLL_VALUE   = 0x03000000, // poll a register until value
  OPC_WAIT_N       = 0x10000000, // loop n
  OPC_SPIN         = 0x20000000, // spin forever
  OPC_RPAPI_BASE   = 0x40000000, // set ROM code public API base address
  OPC_INTERFACE    = 0x80000000, // peripheral boot interface (set by host)
  OPC_MODE_16      = 0x00160000, // 16 bit register access mode
  OPC_MODE_32      = 0x00320000, // 32 bit register access mode
  OPC_DEBUG_REG    = 0x00110000, // Configure register based tracing and status
  OPC_COPY         = 0x00120000, // Copy the content at one address to another
  OPC_DEBUG_UART   = 0x00130000, // Set the debug UART for traces (0/1/2/3)
  OPC_HEAP_ADDR    = 0x00140000, // Set the base address of the heap in DDR (and inderectly the driver load address)
  OPC_SKIP         = 0x00150000, // Do not execute commands from this point forward
  OPC_UNSKIP       = 0x00170000, // Start executing commands from this point forward
  OPC_SKIP_COND    = 0x00180000  // Do not execute command from this point forward if condition is met
} T_init_opcode;

/* Definition elements from the definition file used for writing the configuration script */

char * definitions[MAX_DEFINITIONS][2];

/* Board configuration data to be sent to the platform */

unsigned int omap_configuration[MAX_CONFIGURATIONS];

/* Memory configuration data describing the memories available on the board */

unsigned char * memory_configuration[MAX_MEMORIES][3];

/* Store the peripheral boot mode 0=USB, 1=UART */

extern unsigned int peripheral_boot_if; //peripheral boot mode


/* Status for board configuration file */

unsigned int valid_board_config = FALSE;

/* Mapping data for interpretation of the operations in the configuration file */
#define CMD_WRITE       "WRITE"
#define CMD_MODIFY      "MODIFY"
#define CMD_POLL_ZERO   "POLL_ZERO"
#define CMD_POLL_NZERO  "POLL_NZERO"
#define CMD_POLL_VALUE  "POLL_VALUE"
#define CMD_WAIT_N      "WAIT_N"
#define CMD_SPIN        "SPIN"
#define CMD_RPAPI_BASE  "RPAPI_BASE"
#define CMD_MODE_16     "MODE_16"
#define CMD_MODE_32     "MODE_32"
#define CMD_DEBUG_REG   "DEBUG_REG"
#define CMD_COPY        "COPY"
#define CMD_DEBUG_UART  "DEBUG_UART"
#define CMD_HEAP_ADDR   "HEAP_ADDR"
#define CMD_SKIP        "SKIP"
#define CMD_UNSKIP      "UNSKIP"
#define CMD_SKIP_COND   "SKIP_COND"

const struct
{
  const char * command;
  T_init_opcode opcode;
} command_map[] =
{
  { CMD_WRITE,      OPC_WRITE      },
  { CMD_MODIFY,     OPC_MODIFY     },
  { CMD_POLL_ZERO,  OPC_POLL_ZERO  },
  { CMD_POLL_NZERO, OPC_POLL_NZERO },
  { CMD_POLL_VALUE, OPC_POLL_VALUE },
  { CMD_WAIT_N,     OPC_WAIT_N     },
  { CMD_SPIN,       OPC_SPIN       },
  { CMD_RPAPI_BASE, OPC_RPAPI_BASE },
  { CMD_MODE_16,    OPC_MODE_16    },
  { CMD_MODE_32,    OPC_MODE_32    },
  { CMD_DEBUG_REG,  OPC_DEBUG_REG  },
  { CMD_COPY,       OPC_COPY       },
  { CMD_DEBUG_UART, OPC_DEBUG_UART },
  { CMD_HEAP_ADDR,  OPC_HEAP_ADDR  },
  { CMD_SKIP,       OPC_SKIP       },
  { CMD_UNSKIP,     OPC_UNSKIP     },
  { CMD_SKIP_COND,  OPC_SKIP_COND  },
  { NULL,           OPC_NOP        }
};

static struct  
{
  BOOLEAN rpapi_base;
  BOOLEAN heap_addr;
} mandatory_parameters =
{
  FALSE,
  FALSE
};

extern void strrep(char *);
static T_init_opcode get_command_opcode(char * s)
{
  int i = 0; 
  while(command_map[i].command != NULL)
  {
    if(!strcmp(command_map[i].command, s))
    {
      return(command_map[i].opcode);
    }
    i++;
  }
  return(OPC_NOP);
}

/* Calculation of the power of x to the y */

static unsigned int board_pow(unsigned int x, unsigned int y)
{
  unsigned int ret = 1;
  if(y > 0)
  {
    while(y--) ret *= x;
  }
  return(ret);
}

/* Convert the value element to an unsigned integer */

static unsigned int get_value(char * s, T_init_opcode mode)
{
  char * p = s;
  unsigned int ret = 0;
  int base = 10;
  int exp;

  if(!strncmp(p, "0x", 2))
  {
    base = 16;
    p += 2;
  }

  exp = strlen(p) - 1;
  
  while(*p)
  {
    if((*p >= '0') && (*p <= '9'))
    {
      ret += (*p - '0') * board_pow(base, exp--);
    }
    else if((((*p >= 'A') && (*p <= 'F')) || ((*p >= 'a') && (*p <= 'f'))) && (base == 16))
    {
      ret += (*p - (((*p >= 'A') && (*p <= 'F')) ? 'A' : 'a') + 10) * board_pow(base, exp--);
    }
    else
    {
      die("Unknown number format encountered in conversion of %s to integer.", s);
    }
    p++;
  }

  if((mode == OPC_MODE_16) && (ret & 0xFFFF0000))
  {
    die("Value field %s contains a value incompatible with the register access mode chosen (MODE_16).", s);
  }

  return(ret);
}

/* Read a line from one of the configuration files */

static int read_line(FILE * f, char * l, int max_length)
{
  int i = 0;
  int length;

  memset(l, 0, max_length);

  while(fread(l + i, sizeof(char), 1, f))
  {
    if(*(l + i) == '\n')
    {
      *(l + i) = 0;
      break;
    }

    i++;

    if(i > max_length)
    {
      die("Line length of %d characters exceeded at:\n%s", max_length, l);
    }
  }
  
  length = strlen(l);
  
  return(length ? length : feof(f) ? -1 : 0);
}

/* Extract an element from a line in the file (label or value) */

static int find_element(char * string, char ** result, T_element type)
{
  int length = 0;
  *result = NULL;
  while(*string)
  {
    if(*string == '#')
    {
      break;
    }

    if(((*string >= 0x41) && (*string <= 0x5a)) || 
       ((*string >= 0x61) && (*string <= 0x7a)) ||
       ((*string >= 0x30) && (*string <= 0x39)) ||
       (*string == '_') ||
       ((*string == '-') && (type == PARTITION_NAME)) ||
       (((*string == '/') || (*string == '"') || (*string == '\\') || (*string == '-') || (*string == ':') || (*string == '.')) && (type == FILEPATH)) ||
       (((*string == ' ') || (*string == '-') || (*string == '=')) && ((*result != NULL) && (type == PARAMETER_STRING))))
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

/* Parse the definition file and store the definitions in the definition aray */

static int parse_definitions(char * file_name, char * definitions[][2], int max_definitions)
{
  FILE * f;
  char * line; 
  char * psearch;
  int lsearch;
  int length;
  int index = 0;
  int index_def, index_check;

  strrep(file_name);
  if((f = fopen(file_name, "r")) == NULL)
  {
    die("ERROR: Unable to open definition file %s\n", file_name);
    return(-1);
  }

  host_logf_ex(VERBOSE, "Reading definition file %s", file_name);

  line = (char *)malloc(MAX_LINE_LENGTH + 1); line[MAX_LINE_LENGTH] = 0;
  

  while((length = read_line(f, line, MAX_LINE_LENGTH)) >= 0)
  {
    if(length)
    {
      if(index == max_definitions)
      {
        die("Too many definitions in definition file %s", file_name);
      }

      lsearch = find_element(line, &psearch, COMMAND_VALUE);
      if(lsearch)
      {
        definitions[index][0] = (char *)malloc(lsearch + 1);
        definitions[index][0][lsearch] = 0;
        memcpy(definitions[index][0], psearch, lsearch);

        lsearch = find_element(psearch + lsearch, &psearch, COMMAND_VALUE);
        if(lsearch)
        {
          definitions[index][1] = (char *)malloc(lsearch + 1);
          definitions[index][1][lsearch] = 0;
          memcpy(definitions[index][1], psearch, lsearch);
        }
        else
        {
          die("Syntax error in definition file %s at:\n%s", file_name, line);
        }

        lsearch = find_element(psearch + lsearch, &psearch, COMMAND_VALUE);
        if(lsearch)
        {
          die("Syntax error in definition file %s at:\n%s", file_name, line);
        }
        
        index++;
      }
      
    }
  }

  free(line);

  // Check that the definition file does not contain multiple definitions with the same
  // label

  for(index_def = 0; index_def < (index - 1); index_def++)
  {
    for(index_check = index_def + 1; index_check < index; index_check++)
    {
      if(!strcmp(definitions[index_def][0], definitions[index_check][0]))
      {
        die("Defintion of %s is defined more than once in definition file %s", definitions[index_def][0], file_name);
      }
    }
  }

  return(index);
}

/* Helper for checking command parameter requirements */

static void check_required_arguments(char ** command, int c_required)
{
  int i;
  for(i = 1; i <= c_required; i++)
  {
    if(strncmp(command[i], "0x", 2))
    {
      die("Command parameter format error for %s. Parameter #%d not a value (%s).", command[0], i, command[i]);
    }
  }
}

/* Parse the configuration file and return the configuration data */

int parse_configuration(char * file_name)
{
  FILE * f;
  char * line; 
  char * psearch;
  int lsearch;
  int index_a;
  int index_b = 0;
  int length;
  unsigned int * popc, * pins;
  int nof_definitions = 0;
  T_init_opcode mode = OPC_MODE_32;
  char ** elements = (char **)malloc(sizeof(char *) * MAX_CONFIGURATION_ELEMENTS);

  memset(elements, 0, sizeof(char *) * MAX_CONFIGURATION_ELEMENTS);
  memset(omap_configuration, 0, MAX_CONFIGURATIONS * sizeof(unsigned int));
  
  for(index_a = 0; index_a < MAX_MEMORIES; index_a++)
  {
    memory_configuration[index_a][MEMORY_NAME]   = NULL;
    memory_configuration[index_a][MEMORY_DRIVER] = NULL;
    memory_configuration[index_a][MEMORY_CONFIG] = NULL;
  }

  strrep(file_name);
  if((f = fopen(file_name, "r")) == NULL)
  {
    die("Unable to open configuration file %s\n", file_name);
    return(-1);
  }

  host_logf_ex(VERBOSE, "Reading board configuration file %s", file_name);

  line = (char *)malloc(MAX_LINE_LENGTH + 1); line[MAX_LINE_LENGTH] = 0;

  while((length = read_line(f, line, MAX_LINE_LENGTH)) >= 0)
  {
    if(length)
    {
      psearch = line;

      while(lsearch = find_element(psearch, &psearch, COMMAND_VALUE))
      {
        if(index_b == MAX_CONFIGURATION_ELEMENTS)
        {
          die("Too many configuration elements in configuration file %s. Maximum is %d.", file_name, MAX_CONFIGURATION_ELEMENTS);
        }

        if(!strncmp(psearch, "partition", lsearch))
        {
          psearch += lsearch;

          if(lsearch = find_element(psearch, &psearch, COMMAND_VALUE))
          {
            char memory[21];
            memcpy(memory, psearch, 20); memory[lsearch > 20 ? 20 : lsearch] = 0;
            
            psearch += lsearch;
             
            if(lsearch = find_element(psearch, &psearch, COMMAND_VALUE))
            {
              char size_kb[11];
              memcpy(size_kb, psearch, 10); size_kb[lsearch > 10 ? 10 : lsearch] = 0;

              psearch += lsearch;

              if(lsearch = find_element(psearch, &psearch, PARTITION_NAME))
              {
                char name[41];
                memcpy(name, psearch, 40); name[lsearch > 40 ? 40 : lsearch] = 0;
                
                partition_add(memory, name, get_value(size_kb, OPC_MODE_32));
                break;
              }
              else
              {
                die("Missing parameter 'name' in partition definition in %s:\n%s", file_name, line);
              }
            }
            else
            {
              die("Missing parameter 'size (kB)' in partition definition in %s:\n%s", file_name, line);
            }
          }
          else
          {
            die("Missing parameter 'memory' in partition definition in %s:\n%s", file_name, line);
          }          
        }

        if(!strncmp(psearch, "use", lsearch))
        {
          if(!nof_definitions)
          {
            psearch += lsearch;
            if(lsearch = find_element(psearch, &psearch, FILEPATH))
            {
              *(psearch + lsearch) = 0;
              nof_definitions = parse_definitions(psearch, definitions, MAX_DEFINITIONS);
              break;
            }
            else
            {
              die("Syntax error in 'use' directive in file %s:\n- %s", file_name, line);
            }
          }
          else
          {
            die("Multiple 'use' directives in file %s not allowed:\n- %s", file_name, line);
          }
        }

        if(!strncmp(psearch, "memory", lsearch))
        {
          int m;

          for(m = 0; (m < MAX_MEMORIES) && (memory_configuration[m][MEMORY_NAME] != NULL); m++);

          if(m >= MAX_MEMORIES)
          {
            die("Too many memories defined in configuration file %s (maximum is %d)", file_name, MAX_MEMORIES);
          }

          psearch += lsearch;

          if(lsearch = find_element(psearch, &psearch, COMMAND_VALUE))
          {
            memory_configuration[m][MEMORY_NAME] = (char *)malloc(lsearch + 1);
            memcpy(memory_configuration[m][MEMORY_NAME], psearch, lsearch);
            memory_configuration[m][MEMORY_NAME][lsearch] = 0;
          }
          else
          {
            die("Missing parameter 'name' in memory definition in %s:\n%s", file_name, line);
          }

          psearch += lsearch;

          if(lsearch = find_element(psearch, &psearch, COMMAND_VALUE))
          {
            if(!strncmp(psearch, "driver", lsearch))
            {
              psearch += lsearch;

              if(lsearch = find_element(psearch, &psearch, FILEPATH))
              {
                memory_configuration[m][MEMORY_DRIVER] = (char *)malloc(lsearch + 1);
                memcpy(memory_configuration[m][MEMORY_DRIVER], psearch, lsearch);
                memory_configuration[m][MEMORY_DRIVER][lsearch] = 0;

                psearch += lsearch;
                lsearch = find_element(psearch, &psearch, FILEPATH);
              }
            }

            if(lsearch)
            {
              if(!strncmp(psearch, "parameters", lsearch))
              {
                psearch += lsearch;
    
                if(lsearch = find_element(psearch, &psearch, PARAMETER_STRING))
                {
                  memory_configuration[m][MEMORY_CONFIG] = (char *)malloc(lsearch + 1);
                  memcpy(memory_configuration[m][MEMORY_CONFIG], psearch, lsearch);
                  memory_configuration[m][MEMORY_CONFIG][lsearch] = 0;
                }
              }
            }
          }
          break;
        }

        if(nof_definitions)
        {
          for(index_a = 0; index_a < nof_definitions; index_a++)
          {
            if(!strncmp(psearch, definitions[index_a][0], lsearch) && (lsearch == strlen(definitions[index_a][0])))
            {
              elements[index_b] = (char *)malloc(strlen(definitions[index_a][1]) + 1);
              strcpy(elements[index_b], definitions[index_a][1]);
              break;
            }
          }
        }

        if(elements[index_b] == NULL)
        {
          elements[index_b] = (char *)malloc(lsearch + 1);
          elements[index_b][lsearch] = 0;
          memcpy(elements[index_b], psearch, lsearch);
        }

        psearch += lsearch;
        index_b++;
      }
    }
  }

  // Check content of elements specifying what to do. At this point everyting should be Hex
  // except for the commands.

  for(index_a = 0; index_a < index_b; index_a++)
  {
    if(strncmp(elements[index_a], "0x", 2))
    {
      T_init_opcode opc = get_command_opcode(elements[index_a]);
      switch(opc)
      {
        case OPC_NOP:
          die("Error in configuration file %s.\n- Definition of %s unknown or not properly converted to value.\n", file_name, elements[index_a]);
          break;

        case OPC_SPIN:
        case OPC_MODE_16:
        case OPC_MODE_32:
        case OPC_SKIP:
        case OPC_UNSKIP:
          break;

        case OPC_RPAPI_BASE:
        case OPC_WAIT_N:
        case OPC_DEBUG_UART:
        case OPC_HEAP_ADDR:
          check_required_arguments(&elements[index_a], 1);
          break;

        case OPC_WRITE:
        case OPC_POLL_ZERO:
        case OPC_POLL_NZERO:
        case OPC_COPY:
          check_required_arguments(&elements[index_a], 2);
          break;
      
        case OPC_MODIFY:
        case OPC_POLL_VALUE:
        case OPC_SKIP_COND:
          check_required_arguments(&elements[index_a], 3);
          break;

        case OPC_DEBUG_REG:
          check_required_arguments(&elements[index_a], 4);
          break;
      }

    }
  }

  // Run through the elements and optimize the sequence

  index_a = 0;
  popc = omap_configuration;
  pins = omap_configuration;

  while((index_a < index_b) && ((pins - omap_configuration) < (MAX_CONFIGURATIONS - 6)))
  {
    T_init_opcode opc = get_command_opcode(elements[index_a++]);
    
    switch(opc)
    {
      case OPC_WRITE:
        switch(*popc & OPCODE_MASK)
        {
          case OPC_WRITE:
            *popc = OPC_WRITE_N + 2;
            break;
          case OPC_WRITE_N:
            *popc += 1;
            break;
          default:
            popc = pins;
            *(pins++) = opc;
            break;
        }
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // Register
        *(pins++) += get_value(elements[index_a++], mode);        // Value
        break;

      case OPC_MODIFY:
        {
          unsigned int reg  = get_value(elements[index_a++], OPC_MODE_32); // Register
          unsigned int mask = get_value(elements[index_a++], mode);        // Mask
          unsigned int val  = get_value(elements[index_a++], mode);        // Value

          switch(*popc & OPCODE_MASK)
          {
            case OPC_MODIFY:
              *popc = OPC_MODIFY_N;
              if(mask == *(popc + 2))
              {
                *popc = OPC_MODIFY_N_M;
                if(val == *(popc + 3))
                {
                  *popc = OPC_MODIFY_N_M_V;
                }
              }
              switch(*popc)
              {
                case OPC_MODIFY_N:
                  *(pins++) += reg;
                  *(pins++) += mask;
                  *(pins++) += val;
                  break;
              
                case OPC_MODIFY_N_M:
                  *(popc + 2) = *(popc + 1);
                  *(popc + 1) = mask;
                  *(pins++)   = reg;
                  *(pins++)   = val;
                  break;

                case OPC_MODIFY_N_M_V:
                  *(popc + 3) = *(popc + 1);
                  *(popc + 1) = mask;
                  *(popc + 2) = val;
                  *(pins++)   = reg;
                  break;
              }
              *popc += 2;
              break;

            case OPC_MODIFY_N:
              *popc += 1;
              *(pins++) += reg;
              *(pins++) += mask;
              *(pins++) += val;
              break;

            case OPC_MODIFY_N_M:
              if(*(popc + 1) == mask)
              {
                *popc += 1;
                *(pins++) += reg;
                *(pins++) += val;
              }
              else
              {
                popc = pins;
                *(pins++) = opc;
                *(pins++) += reg;
                *(pins++) += mask;
                *(pins++) += val;
              }
              break;

            case OPC_MODIFY_N_M_V:
              if((*(popc + 1) == mask) && (*(popc + 2) == val))
              {
                *popc += 1;
                *(pins++) += reg;
              }
              else
              {
                popc = pins;
                *(pins++) = opc;
                *(pins++) += reg;
                *(pins++) += mask;
                *(pins++) += val;
              }
              break;

            default:
              popc = pins;
              *(pins++) = opc;
              *(pins++) += reg;
              *(pins++) += mask;
              *(pins++) += val;
              break;
          }
        }
        break;

      case OPC_POLL_ZERO:
      case OPC_POLL_NZERO:
        popc = pins;
        *(pins++) = opc;
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // Register
        *(pins++) += get_value(elements[index_a++], mode);        // Mask
        break;

      case OPC_POLL_VALUE:
      case OPC_SKIP_COND:
        popc = pins;
        *(pins++) = opc;
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // Register
        *(pins++) += get_value(elements[index_a++], mode);        // Mask
        *(pins++) += get_value(elements[index_a++], mode);        // Value
        break;

      case OPC_WAIT_N:
        popc = pins;
        *(pins) = opc;
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32);
        if((*popc & OPCODE_MASK) != OPC_WAIT_N)
        {
          die("Value of N for command WAIT_N too large in configuration file %s.\n- N must be in the range 0x0000 - 0xFFFF.", file_name);
        }
        break;

      case OPC_RPAPI_BASE:
      case OPC_HEAP_ADDR:
        mandatory_parameters.rpapi_base |= (opc == OPC_RPAPI_BASE);
        mandatory_parameters.heap_addr  |= (opc == OPC_HEAP_ADDR);
        popc = pins;
        *(pins++) = opc;
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // Address
        break;

      case OPC_SPIN:
      case OPC_MODE_16:
      case OPC_MODE_32:
      case OPC_SKIP:
      case OPC_UNSKIP:
        popc = pins;
        *(pins++) = opc;
        mode = opc;
        break;

      case OPC_DEBUG_REG:
        popc = pins;
        *(pins) = opc;
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // ID - stored with the OPC
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // Register
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // Mask
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // Value
        break;

      case OPC_COPY:
        popc = pins;
        *(pins++) = opc;
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // From
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32); // To
        break;

      case OPC_DEBUG_UART:
        popc = pins;
        *(pins) = opc;
        *(pins++) += get_value(elements[index_a++], OPC_MODE_32);
        if((*popc & OPCODE_MASK) != OPC_DEBUG_UART)
        {
          die("Value of N for command DEBUG_UART too large in configuration file %s.\n- N must be in the range 0x0000 - 0xFFFF.", file_name);
        }
        break;

      default:
        die("Unrecognized command %s in configuration file %s.", elements[index_a], file_name);
        break;
    }
  }

  *(pins++) = OPC_INTERFACE + peripheral_boot_if;

  if((pins - omap_configuration) >= MAX_CONFIGURATIONS)
  {
    die("Too many configuration commands. The number of commands exceeds the maximum after compression.");
  }

  *(pins++) = OPC_NOP;

  if(!mandatory_parameters.rpapi_base)
  {
    die("Mandatory configuration parameter RPAPI_BASE missing from board configuration file.");
  }

  if(!mandatory_parameters.heap_addr)
  {
    die("Mandatory configuration parameter HEAP_ADDR missing from board configuration file.");
  }

  for(index_a = 0; index_a < index_b; index_a++)
  {
    free(elements[index_a]);
  }

  free(elements);
  free(line);

  valid_board_config = TRUE;

  return(pins - omap_configuration);
}


BOOLEAN board_config_read(void)
{
  return(valid_board_config);
}