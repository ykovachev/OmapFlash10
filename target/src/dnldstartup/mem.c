/*
 * Copyright (c) 2009, Texas Instruments, Inc.
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
 */

#include "types.h"
#include "mem.h"
#include "omapconfig.h"
#include "uart.h"

typedef struct  
{
  U32 reserved;
  U32 allocated;
  U32 size;
} t_header;

typedef struct t_element 
{
  t_header           header;
  struct t_element * next;
} t_element;

#ifdef SIMULATION
U32 simulation_heap[HEAP_SPACE / sizeof(U32)];
#endif

t_element * heap = NULL;
BOOLEAN     heap_initialized = FALSE;

/*------------------------------------------------------------------------------
| Function    : show_element
+------------------------------------------------------------------------------
| Description : Function for showing the content of a heap header
|
| Parameters  : description - Description of content
|               e - pointer to element that is to be shown
|
| Returns     : void
+----------------------------------------------------------------------------*/
void mem_show_element(char * description, t_element * e)
{
  U32 count;
  DEBUG_LOGF(description);
  for(count = 0; count < sizeof(t_element); count += 4)
  {
    DEBUG_LOGF("%08x: %02x %02x %02x %02x", ((U32)e) + count, ((unsigned char *)e)[count], ((unsigned char *)e)[count + 1], ((unsigned char *)e)[count + 2], ((unsigned char *)e)[count + 3]);
  }
}

/*------------------------------------------------------------------------------
| Function    : show_heap
+------------------------------------------------------------------------------
| Description : Function for showing the heap content
|
| Parameters  : description - Description of content
|
| Returns     : void
+----------------------------------------------------------------------------*/
void mem_show(char * description)
{
  t_element * e = heap;
  U32 size_a = 0;
  U32 size_u = 0;
  U32 num_a = 0;
  U32 num_u = 0;

  DEBUG_LOGF("%s:", description);

  if(heap == NULL)
  {
    DEBUG_LOGF("Heap address not set");
    return;
  }

  if(heap_initialized == FALSE)
  {
    DEBUG_LOGF("Heap unused");
    return;
  }

  while(e)
  {
    if(e->header.allocated)
    {
      size_a += sizeof(t_element) + e->header.size;
      num_a++;
      DEBUG_LOGF("H: %08x +: %08x [%8d]", e, e + 1, e->header.size);
    }
    else
    {
      size_u += sizeof(t_element) + e->header.size;
      num_u++;
      DEBUG_LOGF("H: %08x -: %08x [%8d]", e, e + 1, e->header.size);
    }

    if((((U32)(e->next) > (U32)(e)) && ((U32)(e->next) < ((U32)heap) + HEAP_SPACE - 1)) || (e->next == NULL))
    {
      e = e->next;
    }
    else
    {
      DEBUG_LOGF_ERROR("Link ptr corrupted!");
      mem_show_element("Element", e);
      if((U32)(e->next) <= (U32)(e))
      {
        DEBUG_LOGF_ERROR("Next ptr backwards");
      }
      else if((U32)(e->next) >= (((U32)heap) + HEAP_SPACE))
      {
        DEBUG_LOGF_ERROR("Next ptr out of bounds");
      }
      else if(e->next != NULL)
      {
        DEBUG_LOGF_ERROR("Next ptr not NULL");
      }
      e = NULL;
    }
  }

  DEBUG_LOGF("+: %8d B in %4d bufs", size_a, num_a);
  DEBUG_LOGF("-: %8d B in %4d bufs", size_u, num_u);
}

/*------------------------------------------------------------------------------
| Function    : mem_init
+------------------------------------------------------------------------------
| Description : Function that will initialize the heap address before it can 
|               be used. Initialization of the heap structure happens on the
|               first call to mem_alloc after mem_init (in order to avoid any
|               SDRAM access before the heap is actually used).
|
| Parameters  : address - location of heap memory area
|
| Returns     : void
+----------------------------------------------------------------------------*/
void mem_init(U32 address)
{
  if(heap == NULL)
  {
    DEBUG_LOGF("Heap at %08x", address);

    #ifdef SIMULATION
    heap = (t_element *)simulation_heap;
    #else
    heap = (t_element *)address;
    #endif
  }
}

/*------------------------------------------------------------------------------
| Function    : mem_alloc
+------------------------------------------------------------------------------
| Description : Function for dynamic allocation of memory. The function will
|               allocate the requested amount of memory rounded up to the 
|               nearest chunk of 128 bytes in order to minimize fragmentation
|
| Parameters  : size - amount of memory to be allocated
|
| Returns     : Pointer to allocated memory buffer
+----------------------------------------------------------------------------*/
void * mem_alloc(U32 size)
{
  t_element * e = heap;
  t_element * b = NULL;

  size = (size + 127) & ~0x7F;

  if(heap == NULL)
  {
    DEBUG_LOGF_ERROR("Error: no heap");
    return NULL;
  }

  if(heap_initialized == FALSE)
  {
    DEBUG_LOGF("Heap initialized");
    heap_initialized       = TRUE;
    heap->header.allocated = FALSE;
    heap->header.size      = HEAP_SPACE - sizeof(t_element);
    heap->next             = NULL;
  }

  while(e)
  {
    if(!e->header.allocated)
    {
      if(e->header.size >= size)
      {
        if(b)
        {
          if(b->header.size > e->header.size)
          {
            b = e;
          }
        }
        else
        {
          b = e;
        }
      }
    }

    if((((U32)(e->next) > (U32)(e)) && ((U32)(e->next) < (((U32)heap) + HEAP_SPACE - 1)) || (e->next == NULL)))
    {
      e = e->next;
    }
    else
    {
      DEBUG_LOGF_ERROR("Failed to allocate memory");
      mem_show_element("alloc() - overwrite", e);
      mem_show("Heap content");
      return(NULL);
    }
  }

  if(b)
  {
    if(b->next)
    {
      // Found previously allocated chunk that was of at least size bytes
      b->header.allocated = TRUE;
      return &b[1];
    }

    // Found no previously allocated chunk of the right size - make a new chunk
    if((((U32)b + sizeof(t_element) + size) - (U32)heap) < HEAP_SPACE)
    {
      b->next = (t_element *)((U32)b + sizeof(t_element) + size);
      b->next->header.allocated = FALSE;
      b->next->header.size = HEAP_SPACE - ((U32)b - (U32)heap) - 2 * sizeof(t_element) - size;
      b->next->next = NULL;
      b->header.allocated = TRUE;
      b->header.size = size;
      return &b[1];
    }
  }

  DEBUG_LOGF_ERROR("Heap error!");
  return NULL;
}

/*------------------------------------------------------------------------------
| Function    : mem_free
+------------------------------------------------------------------------------
| Description : Function for freeing a dynamically allocated buffer in memory
|
| Parameters  : address - the address of the buffer to be released
|
| Returns     : void
+----------------------------------------------------------------------------*/
void mem_free(void * address)
{
  if(address)
  {
    t_element * e = (t_element *)((U32)address - sizeof(t_element));

    if((U32)(e->next) != ((U32)e) + sizeof(t_element) + e->header.size)
    {
      DEBUG_LOGF_ERROR("Overwrite detected on free() of 0x%08X", address);
      mem_show_element("free() - overwrite", e);
      mem_show("Heap content");
    }

    e->header.allocated = FALSE;
  }
}
