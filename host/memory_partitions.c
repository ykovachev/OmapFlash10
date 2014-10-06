/*
 * Copyright (c) 2010, The Android Open Source Project.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Neither the name of The Android Open Source Project nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
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
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "output.h"
#include "memory_partitions.h"
#include "fastboot.h"

#define BLOCKS_PER_KB 2

#define EFI_VERSION 0x00010000
#define EFI_ENTRIES 128
#define EFI_NAMELEN 36

struct partition 
{
  char     memory[21];
  char     name[41];
  unsigned size_kb;
};

struct efi_entry 
{
  unsigned char      type_uuid[16];
  unsigned char      uniq_uuid[16];
  unsigned long long first_lba;
  unsigned long long last_lba;
  unsigned long long attr;
  unsigned short     name[EFI_NAMELEN];
};

struct efi_header 
{
  unsigned char      magic[8];

  unsigned int       version;
  unsigned int       header_sz;

  unsigned int       crc32;
  unsigned int       reserved;

  unsigned long long header_lba;
  unsigned long long backup_lba;
  unsigned long long first_lba;
  unsigned long long last_lba;

  unsigned char      volume_uuid[16];

  unsigned long long entries_lba;

  unsigned int       entries_count;
  unsigned int       entries_size;
  unsigned int       entries_crc32;
};

struct ptable 
{
  unsigned char mbr[512];
  union 
  {
    struct        efi_header header;
    unsigned char block[512];
  };
  struct efi_entry entry[EFI_ENTRIES];    
};

static struct partition partitions[21] = 
{
  {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 },
  {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 },
  {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 },
  {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 }, {{0}, {0}, 0 },
  {{0}, {0}, 0 }
};

static unsigned nof_partitions = 0;

static struct ptable the_ptable;

static const unsigned char partition_type[16] = 
{
  0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44,
  0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7,
};

static const unsigned char random_uuid[16] = 
{
  0xff, 0x1f, 0xf2, 0xf9, 0xd4, 0xa8, 0x0e, 0x5f,
  0x97, 0x46, 0x59, 0x48, 0x69, 0xae, 0xc3, 0x4e,
};
        


static void init_mbr(unsigned char *mbr, unsigned int blocks)
{
  mbr[0x1be] = 0x00; // nonbootable
  mbr[0x1bf] = 0xFF; // bogus CHS
  mbr[0x1c0] = 0xFF;
  mbr[0x1c1] = 0xFF;

  mbr[0x1c2] = 0xEE; // GPT partition
  mbr[0x1c3] = 0xFF; // bogus CHS
  mbr[0x1c4] = 0xFF;
  mbr[0x1c5] = 0xFF;

  mbr[0x1c6] = 0x01; // start
  mbr[0x1c7] = 0x00;
  mbr[0x1c8] = 0x00;
  mbr[0x1c9] = 0x00;

  memcpy(mbr + 0x1ca, &blocks, sizeof(unsigned int));

  mbr[0x1fe] = 0x55;
  mbr[0x1ff] = 0xaa;
}

static void start_ptbl(struct ptable *ptbl, unsigned blocks)
{
  struct efi_header *hdr = &ptbl->header;

  memset(ptbl, 0, sizeof(*ptbl));

  init_mbr(ptbl->mbr, blocks - 1);

  memcpy(hdr->magic, "EFI PART", 8);
  hdr->version = EFI_VERSION;
  hdr->header_sz = sizeof(struct efi_header);
  hdr->header_lba = 1;
  hdr->backup_lba = blocks - 1;
  hdr->first_lba = 34;
  hdr->last_lba = blocks - 1;
  memcpy(hdr->volume_uuid, random_uuid, 16);
  hdr->entries_lba = 2;
  hdr->entries_count = EFI_ENTRIES;
  hdr->entries_size = sizeof(struct efi_entry);
}

static void end_ptbl(struct ptable *ptbl)
{
  struct efi_header *hdr = &ptbl->header;
  unsigned int n;

  n = crc32(0, 0, 0);
  n = crc32(n, (void*) ptbl->entry, sizeof(ptbl->entry));
  hdr->entries_crc32 = n;

  n = crc32(0, 0, 0);
  n = crc32(0, (void*) &ptbl->header, sizeof(ptbl->header));
  hdr->crc32 = n;
}

int add_ptn(struct ptable *ptbl, unsigned long long first, unsigned long long last, const char *name)
{
  struct efi_header *hdr = &ptbl->header;
  struct efi_entry *entry = ptbl->entry;
  unsigned n;

  if (first < 34) 
  {
    printf("partition '%s' overlaps partition table\n", name);
    return -1;
  }

  if (last > hdr->last_lba) 
  {
    printf("partition '%s' does not fit\n", name);
    return -1;
  }
  for (n = 0; n < EFI_ENTRIES; n++, entry++) 
  {
    if (entry->last_lba)
    {
      continue;
    }
    memcpy(entry->type_uuid, partition_type, 16);
    memcpy(entry->uniq_uuid, random_uuid, 16);
    entry->uniq_uuid[0] = n;
    entry->first_lba = first;
    entry->last_lba = last;
    for (n = 0; (n < EFI_NAMELEN) && *name; n++)
    {
      entry->name[n] = *name++;
    }
    return 0;
  }
  printf("out of partition table entries\n");
  return -1;
}


unsigned int partition_get_offset(char * name)
{
  unsigned int i;
  unsigned int offset;
  for(i = 0, offset = 0; i < nof_partitions; i++)
  {
    if(!strcmp(partitions[i].name, name))
    {
      return offset;
    }
    offset += partitions[i].size_kb;
  }
  return 0;
}

unsigned int partition_get_size(char * name)
{
  unsigned int i;
  for(i = 0; i < nof_partitions; i++)
  {
    if(!strcmp(partitions[i].name, name))
    {
      return partitions[i].size_kb;
    }
  }
  return 0;
}

char * partition_get_memory(char * name)
{
  unsigned int i;
  for(i = 0; i < nof_partitions; i++)
  {
    if(!strcmp(partitions[i].name, name))
    {
      return partitions[i].memory;
    }
  }
  return 0;
}

static unsigned int get_total_size()
{
  unsigned int i;
  unsigned int size;
  for(i = 0, size = 0; i < nof_partitions; i++)
  {
    size += partitions[i].size_kb;
  }
  return size;
}

int partition_get_android_table(char * memory, unsigned char * data, char * outfile)
{
  struct ptable *ptbl = &the_ptable;
  unsigned blocks;
  unsigned next;
  int n;

  //if (mmc_init(1)) {
  //        printf("mmc init failed?\n");
  //        return -1;
  //}

  //mmc_info(1, &sector_sz, &blocks);
  //printf("blocks %d\n", blocks);


  blocks = get_total_size() * BLOCKS_PER_KB;

  start_ptbl(ptbl, blocks);
  n = 0;
  next = 0;
  for (n = 0, next = 0; partitions[n].name[0]; n++) 
  {
    if(!strcmp(memory, partitions[n].memory))
    {
      unsigned sz = partitions[n].size_kb * BLOCKS_PER_KB;
      if (!strcmp(partitions[n].name,"-")) 
      {
        next += sz;
        continue;
      }
      if (sz == 0)
      {
        sz = blocks - next;
      }
      if (add_ptn(ptbl, next, next + sz - 1, partitions[n].name))
      {
        return -1;
      }
      next += sz;
    }
  }
  end_ptbl(ptbl);

  //fastboot_flash_reset_ptn();
  //if (mmc_write(1, (void*) ptbl, 0, sizeof(struct ptable)) != 1)
  //        return -1;

  //printf("\nnew partition table:\n");
  //load_ptbl();

  data = (unsigned char *)ptbl;

  if(outfile)
  {
    FILE *f;
    int i;

    if((f = fopen(outfile, "wb")) == NULL)
    {
      die("ERROR: Unable to open output file %s for OEM partition table\n", outfile);
      return(-1);
    }
    for(i = 0; i < sizeof(struct ptable); i++) 
    {
      fputc(data[i], f);
    }
    fclose(f);
  }

  return sizeof(struct ptable);
}


unsigned partition_add(char * memory, char * name, unsigned size_kb)
{
  if(nof_partitions >= 20)
  {
    return (unsigned)-1;
  }
  strncpy(partitions[nof_partitions].memory, memory, 20);
  partitions[nof_partitions].memory[20] = 0;
  strncpy(partitions[nof_partitions].name, name, 40);
  partitions[nof_partitions].name[40] = 0;
  partitions[nof_partitions].size_kb = size_kb;
  nof_partitions++;
  return 0;
}

