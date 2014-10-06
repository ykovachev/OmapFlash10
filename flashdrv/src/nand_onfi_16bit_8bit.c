/**
 * @file nand_onfi_16bit_8bit.c
 * @author Lars Oernbo
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
 * ONFI compliant driver for NAND devices
 */

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES ==============================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "flash_drv.h"
#include "csst_tgt.h"

/*==== CONFIGURATION OPTIONS =================================================*/

// Enable the following for detailed ecc corrected downloads
#define SANE_ECC_READ

/*==== PRIVATE MACROS ================================================================*/

//#define LOG_INFO
#define LOG_ERRORS

#define ONFI_COMMAND_READ_CYCLE1              0x00
#define ONFI_COMMAND_READ_CYCLE2              0x30
#define ONFI_COMMAND_PAGE_PROGRAM_CYCLE1      0x80
#define ONFI_COMMAND_PAGE_PROGRAM_CYCLE2      0x10
#define	ONFI_COMMAND_BLOCK_ERASE_CYCLE1	      0x60
#define	ONFI_COMMAND_BLOCK_ERASE_CYCLE2	      0xD0
#define ONFI_COMMAND_READ_STATUS              0x70
#define ONFI_COMMAND_READ_ID                  0x90
#define ONFI_COMMAND_READ_PARAMETER_PAGE      0xEC
#define ONFI_COMMAND_RESET                    0xFF

#ifdef FF_DEVICE_LOCKED_ON_BOOT
#define ONFI_COMMAND_BLOCK_UNLOCK_CYCLE1       0x23
#define	ONFI_COMMAND_BLOCK_UNLOCK_CYCLE2       0x24
/* define the following to swap the upper and lower bounds */
/*#undef FF_INVERTED_DEVICE_LOCK_COMMAND */
#endif /* FF_DEVICE_LOCKED_ON_BOOT */

#define	ONFI_STATUS_ERROR                     0x01
#define	ONFI_STATUS_READY                     0x40
#define	ONFI_STATUS_WRITE_PROTECT             0x80

#define NAND16BIT                   (db->device.features.options.data_16bit)

#define GPMC_NAND_COMMAND(baseaddress,chipselect)   (baseaddress + 0x07C + 0x30 * chipselect)
#define GPMC_NAND_ADDRESS(baseaddress,chipselect)   (baseaddress + 0x080 + 0x30 * chipselect)
#define GPMC_NAND_DATA(baseaddress,chipselect)      (baseaddress + 0x084 + 0x30 * chipselect)


// These macros are compensated for 16 and 8 bit operation
#define NAND_COMMAND(COMMAND)       NAND_WRITE_ADDR(db->gpmc.registers.command, COMMAND)
#define NAND_ADDRESS(ADDRESS)       NAND_WRITE_ADDR(db->gpmc.registers.address, ADDRESS)
#define NAND_WRITE_ADDR(ADDR, DATA) { if(NAND16BIT) (*(volatile U16 *) (ADDR)) = (DATA); else (*(volatile U8 *) (ADDR)) = (DATA); }
#define NAND_WRITE(DATA)            { if(NAND16BIT) (*(volatile U16 *) (db->gpmc.registers.data)) = (U16)(DATA); else (*(volatile U8 *) (db->gpmc.registers.data)) = (U8)(DATA); }
#define NAND_WRITE_PTR(DATA)        { if(NAND16BIT) (*(volatile U16 *) (db->gpmc.registers.data)) = *((U16 *)(DATA)); else (*(volatile U8 *) (db->gpmc.registers.data)) = *((U8 *)(DATA)); }
#define NAND_READ()                 (NAND16BIT ? ((*((volatile U16 *) (db->gpmc.registers.data))) & 0xFFFF) : ((*((volatile U8 *) (db->gpmc.registers.data))) & 0xFF))
#define NAND_READ_PTR(DATA)         { if(NAND16BIT) *((U16 *)(DATA)) = ((*((volatile U16 *) (db->gpmc.registers.data))) & 0xFFFF); else *((U8 *)(DATA)) = ((*((volatile U8 *) (db->gpmc.registers.data))) & 0xFF)); }

#define WAIT_MICROSEC(MICROSEC)                  (db->access.wait_microsec(MICROSEC))

#define PSKIP(BYTES)                { int idx; for(idx = 0; idx < BYTES; idx++) NAND_READ(); }
#define PREAD(PARAM)                { int idx; for(idx = 0, PARAM = 0; idx < sizeof(PARAM); idx++) PARAM |= NAND_READ() << (idx * 8) ; }
#define PSREAD(PARAM, BYTES)        { int idx; for(idx = 0, PARAM[BYTES] = 0; idx < BYTES; idx++) PARAM[idx] = NAND_READ(); }

#ifdef LOG_INFO
//#define INFO(FORMAT)                 { if(db->access.dbg_printf) db->access.dbg_printf("DRV INFO: "FORMAT); }
//#define INFO1(FORMAT, P1)            { if(db->access.dbg_printf) db->access.dbg_printf("DRV INFO: "FORMAT, P1); }
//#define INFO2(FORMAT, P1, P2)        { if(db->access.dbg_printf) db->access.dbg_printf("DRV INFO: "FORMAT, P1, P2); }
//#define INFO3(FORMAT, P1, P2, P3)    { if(db->access.dbg_printf) db->access.dbg_printf("DRV INFO: "FORMAT, P1, P2, P3); }
#define INFO(FORMAT)                  SEND(FORMAT)
#define INFO1(FORMAT, P1)             SEND1(FORMAT, P1)
#define INFO2(FORMAT, P1, P2)         SEND2(FORMAT, P1, P2)
#define INFO3(FORMAT, P1, P2, P3)     SEND3(FORMAT, P1, P2, P3)
#define INFO4(FORMAT, P1, P2, P3, P4) SEND4(FORMAT, P1, P2, P3, P4)
#else
#define INFO(FORMAT)                 
#define INFO1(FORMAT, P1)            
#define INFO2(FORMAT, P1, P2)        
#define INFO3(FORMAT, P1, P2, P3)    
#endif

#ifdef LOG_ERRORS
#define ERROR(FORMAT)                { if(db->access.dbg_printf) db->access.dbg_printf("DRV ERROR: "FORMAT); }
#define ERROR1(FORMAT, P1)           { if(db->access.dbg_printf) db->access.dbg_printf("DRV ERROR: "FORMAT, P1); }
#define ERROR2(FORMAT, P1, P2)       { if(db->access.dbg_printf) db->access.dbg_printf("DRV ERROR: "FORMAT, P1, P2); }
#define ERROR3(FORMAT, P1, P2, P3)   { if(db->access.dbg_printf) db->access.dbg_printf("DRV ERROR: "FORMAT, P1, P2, P3); }
#else
#define ERROR(FORMAT)                
#define ERROR1(FORMAT, P1)           
#define ERROR2(FORMAT, P1, P2)       
#define ERROR3(FORMAT, P1, P2, P3)   
#endif

#define SEND(FORMAT)                  { if(db->access.send_info) db->access.send_info("NAND "FORMAT); }
#define SEND1(FORMAT, P1)             { if(db->access.send_info) db->access.send_info("NAND "FORMAT, P1); }
#define SEND2(FORMAT, P1, P2)         { if(db->access.send_info) db->access.send_info("NAND "FORMAT, P1, P2); }
#define SEND3(FORMAT, P1, P2, P3)     { if(db->access.send_info) db->access.send_info("NAND "FORMAT, P1, P2, P3); }
#define SEND4(FORMAT, P1, P2, P3, P4) { if(db->access.send_info) db->access.send_info("NAND "FORMAT, P1, P2, P3, P4); }

#define DEVICE_DEFAULT_TIME         600 
#define DEVICE_RESET_TIME           501 /* TRst+TWb */
#define DEVICE_STATUS_READ_TIME		ONE_MILLISEC
#define DEVICE_ERASE_WAIT_TIME      (ONE_MILLISEC * 3)
/* Typical max is around 700 ns or so.. we wait a bit more.. */
#define DEVICE_ERASE_TIMEOUT        TEN_MILLISEC
#define DEVICE_WRITE_TIMEOUT        TEN_MILLISEC
#define DEVICE_READ_TIMEOUT         TEN_MILLISEC
/* The least delay in loop after which we read status again */
#define DEVICE_LEAST_WAIT           10
#define DEVICE_WAIT_UNLOCK2         10
/* TWp is 25 ns */
#define DEVICE_WRITE_PROTECT_DELAY TEN_MILLISEC


#define DEVICE_REAL_ADDR(base_addr,addr) (((addr)<(base_addr))?(addr):((addr) - (base_addr)))
#define DEVICE_WORD_SIZE       (1 + NAND16BIT)
#define DEVICE_WORD_ADDR(ADDR) ((ADDR) >> NAND16BIT)
#define DEVICE_COLUMN_ADDR     (0xFFF >> NAND16BIT)

#define DEVICE_BLOCK_SIZE_FIVEPHASE             0x800
#define DEVICE_ADDTRANS_FIRST(COL_ADDR)         ((COL_ADDR) & 0xFF)
#define DEVICE_ADDTRANS_SECOND(COL_ADDR)        (((COL_ADDR) >> 8) & (0x0F >> NAND16BIT))
#define DEVICE_ADDTRANS_THIRD(PG_ADDR,BLK_ADDR) (((BLK_ADDR) & 0x3) << 6 | ((PG_ADDR) & 0x3F) & 0xFF)
#define DEVICE_ADDTRANS_FOURTH(BLK_ADDR)        (((BLK_ADDR)>>2) & 0xFF)
#define DEVICE_ADDTRANS_FIFTH(BLK_ADDR)         (((BLK_ADDR)>>10) & 0x07)

#define DEVICE_PAGE_ADDRESS_CYCLE     1
#define DEVICE_BLOCK_ADDRESS_CYCLE    2

#define DEVICE_OPERATION_SPARE        1
#define DEVICE_OPERATION_DATA         0

#define DEVICE_OPERATION_ECC_OP       1
#define DEVICE_OPERATION_NO_ECC       0

#define DEVICE_BLOCK_IS_GOOD          0
#define DEVICE_BLOCK_IS_BAD           1

#define DEVICE_WRITE_PROTECT_OFF      0
#define DEVICE_WRITE_PROTECT_ON       1

#define OMAP_GPMC_CONFIG				      0x050
#define OMAP_GPMC_STATUS				      0x054
#define OMAP_GPMC_ECC_CONFIG			    0x1F4
#define OMAP_GPMC_ECC_CONTROL			    0x1F8
#define OMAP_GPMC_ECC_SIZE_CONFIG		  0x1FC
#define OMAP_GPMC_ECC1_RESULT         0x200
#define OMAP_GPMC_ECC2_RESULT			    0x204
#define OMAP_GPMC_ECC3_RESULT			    0x208
#define OMAP_GPMC_ECC4_RESULT			    0x20C
#define OMAP_GPMC_ECC5_RESULT			    0x210
#define OMAP_GPMC_ECC6_RESULT			    0x214
#define OMAP_GPMC_ECC7_RESULT			    0x218
#define OMAP_GPMC_ECC8_RESULT			    0x21C
#define OMAP_GPMC_ECC9_RESULT			    0x220
#define OMAP_GPMC_ECC_RESULT_BASE		  OMAP_GPMC_ECC1_RESULT

///* ECC Readjust Macros */
#define ECC_P1_128_E(val)    ((U8)((val)  & 0x000000FF))      /* Bit 0 to 7 */
#define ECC_P512_2048_E(val) ((U8)(((val) & 0x00000F00)>>8))  /* Bit 8 to 11 */
#define ECC_P1_128_O(val)    ((U8)(((val) & 0x00FF0000)>>16)) /* Bit 16 to Bit 23 */
#define ECC_P512_2048_O(val) ((U8)(((val) & 0x0F000000)>>24)) /* Bit 24 to Bit 27 */

// This macro changes the ECC from the three byte TI standard flash layout:
// 
// MSB -> LSB
// 
// [P2048 P1024 P512 P256 P'2048 P'1024 P'512 P'256] [P128 P64 P32 P16 P8 P4 P2 P1] [P'128 P'64 P'32 P'16 P'8 P'4 P'2 P'1]
// 
// ..into an U32 having the following format:
// 
// [0 0 0 0 P2048 P1024 P512 P256 P128 P64 P32 P16 P8 P4 P2 P1 0 0 0 0 P'2048 P'1024 P'512 P'256 P'128 P'64 P'32 P'16 P'8 P'4 P'2 P'1]
//
// This way, it is easier XOR'ing and counting the ECC bits, and P and P' parities can be deducted from the
// two halves of the U32.

#define ROM_CODE_ECC_2_GPMC_ECC(x) (x[0] | x[1]<<16 | (x[2] & 0xF0)<<20 | (x[2] & 0x0F)<<8)

#define DEVICE_NO_CHECK 0xFFFFFFFF

/*==== PRIVATE DATA STRUCTURES ===============================================*/

typedef struct
{
  U8 chipselect;
  struct
  {
    U32 command;
    U32 address;
    U32 data;
  } registers;
  U32 base_address;
  U32 device_address;
} T_gpmc;

typedef struct
{
  U8 vendor_id;
  U8 device_id;
} T_ids;

typedef struct
{
  char onfi_signature[5];
  U16  onfi_revision;
  union
  {
    U16 value;
    struct
    {
      int data_16bit : 1;
      int multiple_lun_operations : 1;
      int get_set_features : 1;
      int read_stats_enhanced : 1;
      int interleaved_operations : 1;
      int odd_even_page_copyback : 1;
      int reserved : 11;
    } options;
  } features;
  char manufacturer[13];
  char model[21];
  U8   manufacturer_id;
  U16  date_code;
  U32  bytes_per_page;
  U16  spare_bytes_per_page;
  U32  pages_per_block;
  U32  blocks_per_lun;
  U8   luns;
  union
  {
    U8 value;
    struct
    {
      int row : 4;
      int column : 4;
    } options;
  } address_cycles;
  // Calculated
  U32 bytes_per_block;
  U32 bytes_per_lun;
  U32 size_device;
} T_device;

typedef struct 
{
  T_driver_malloc           malloc;
  T_driver_free             free;
  T_driver_wait_microsec    wait_microsec;
  T_driver_dbg_printf       dbg_printf;
  T_driver_send_info        send_info;
} T_access;

typedef struct
{
  U32 word;
  U32 page;
  U32 block;
} T_masks;

typedef struct
{
  U32  address;
  U32  offset;
  U8   data[1];
} T_page;

typedef struct
{
  struct
  {
    U32 bad_block_count;
    U32 last_checked;
  } read;
  struct
  {
    U32 bad_block_count;
    U32 last_checked;
  } write;
  struct
  {
    U32     bad_block_count;
    U32     last_checked;
    U32     last_erased;
    U8      valid;
    BOOLEAN bberase;
  } erase;
	struct
	{
	  /* max bytes for handling rom code requirements */
	  U8 spare[64];
	  /* max bytes for handling rom code requirements */
	  U8 ecc[64];
	} buffers;
} T_parameters;

typedef struct T_db
{
  T_gpmc         gpmc;
  T_ids          ids;
  T_device       device;
  T_access       access;
  T_masks        masks;
  T_parameters   parameters;
  T_page       * page;
} T_db;

/* 16 bit access, large page nand */
typedef struct
{
  U16 bad_block1;
  U16 ecc_a0_a1;
  U16 ecc_a2_b0;
  U16 ecc_b1_b2;
  U16 ecc_c0_c1;
  U16 ecc_c2_d0;
  U16 ecc_d1_d2;
} T_spare_area_info_16;

/* 8 bit access, large page nand */
typedef struct
{
  U8 bad_block1;
  U8 ecc_a0;
  U8 ecc_a1;
  U8 ecc_a2;
  U8 ecc_b0;
  U8 ecc_b1;
  U8 ecc_b2;
  U8 ecc_c0;
  U8 ecc_c1;
  U8 ecc_c2;
  U8 ecc_d0;
  U8 ecc_d1;
  U8 ecc_d2;
} T_spare_area_info_8;

#define DEVICE_GOOD_BLOCK_MARKER (0xFFFF >> (!NAND16BIT * 8))
#define DEVICE_BAD_BLOCK_MARKER  (0xDEAD >> (!NAND16BIT * 8))
#define DEVICE_BLOCK_IS_GOOD     0
#define DEVICE_BLOCK_IS_BAD      1

typedef union
{
  T_spare_area_info_8 data8;
  T_spare_area_info_16 data16;
} T_device_spare_area;

#define DEVICE_OPERATION_SPARE    1
#define DEVICE_OPERATION_DATA     0

/*==== HELPER FUNCTION FORWARD DECLARATION ===================================*/
static S32 drv_add_cycle(T_db * db, U8 mode, U32 address, U8 spare_op);
static S32 drv_read_status(T_db * db, U32 timeout);
S32 drv_is_badblock(T_db * db, U32 address, U8 * is_badblock);
static void drv_status(T_db * db, U16 * status);
static void drv_init_ecc(T_db * db);
static S32 drv_end_ecc(T_db * db, U8 ecc_len, U8 * ecc_data);
void drv_writeprotect(T_db * db, U8 protect_en);
S32 drv_read_data(T_driver_config * config, T_db * db, U32 address, U16 * data_len, U8 * data, U8 ecc_enable, U8 ecc_len, U8 * ecc_data);
S32 drv_read_spare(T_db * db, U32 address, U16 * data_len, U8 * data);
static U8 drv_ecc_compare_correct(U32 src_addr, T_db * db, U32 spare_buffer, U32 ecc_buffer, U32 data_buffer, U32 read_size);
S32 drv_write(T_db * db, U32 address, U8 spare_op, U16 data_len, U8 * data, U8 ecc_enable, U8 ecc_len, U8 * ecc_data);
S32 drv_erase(T_db * db, U32 address);
S32 drv_mark_badblock(T_db * db, U32 address);
static S32 drv_wrstatus(T_db * db, U32 timeout);

//#ifdef DOWNLOAD_DRIVER

U32 drv_dnld_init(T_driver_config * config, char ** result);
U32 drv_dnld_erase(T_driver_config * config, char ** result, U64 address, U64 length);
U32 drv_dnld_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more);
U32 drv_dnld_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
U32 drv_dnld_deinit(T_driver_config * config, char ** result);
U32 drv_dnld_get_info(T_driver_config * config, T_driver_info * info);

void drv_reset(T_db * db);

#ifndef _MSC_VER
#pragma DATA_SECTION(driver_if,".consttbl");
#endif
const T_driver_if driver_if = 
{
  {
    OMAPFLASH_DRIVER_HEADER_STRING_V7,
    "NAND ONFI 16/8 BIT",
    CTRL_Z
  },
  {
    &driver_if,
    drv_dnld_init,
    drv_dnld_read,
    drv_dnld_write,
    drv_dnld_erase,
    drv_dnld_deinit,
    drv_dnld_get_info
  }
};

typedef enum
{
  C_GPMC,
  C_CS,
  C_ADDRESS,
  C_BBERASE,
  C_ONFI,
  C_BPP,
  C_SBPP,
  C_PPB,
  C_BPL,
  C_L,
  C_ACV,
  C_F
} T_driver_setup_index;

const T_driver_setup_const setup_const[] =
{
  /* 00 */ { "gpmc",    DEFAULT,  TRUE  },
  /* 01 */ { "cs",      DECIMAL,  TRUE  },
  /* 02 */ { "address", DEFAULT,  TRUE  },
  /* 03 */ { "bberase", DEFAULT,  TRUE  },
  /* 04 */ { "onfi",    DEFAULT,  FALSE },
  /* 05 */ { "bpp",     DEFAULT,  FALSE },
  /* 06 */ { "sbpp",    DEFAULT,  FALSE },
  /* 07 */ { "ppb",     DEFAULT,  FALSE },
  /* 08 */ { "bpl",     DEFAULT,  FALSE },
  /* 09 */ { "l",       DEFAULT,  FALSE },
  /* 10 */ { "acv",     DEFAULT,  FALSE },
  /* 11 */ { "f",       DEFAULT,  FALSE },
           { "",        0,        FALSE }
};

T_driver_setup_var setup_var[sizeof(setup_const) / sizeof(T_driver_setup_const)];

/*==== DOWNLOAD DRIVER FUNCTION IMPLEMENTATION ===============================*/

void set_gpmc(T_db * db, U8 chipselect, U32 base_address, U32 device_base_address)
{
  INFO1("GMPC CHIPSELECT %d", chipselect);
  db->gpmc.chipselect        = chipselect;
  db->gpmc.registers.command = GPMC_NAND_COMMAND(base_address, chipselect);
  db->gpmc.registers.address = GPMC_NAND_ADDRESS(base_address, chipselect);
  db->gpmc.registers.data    = GPMC_NAND_DATA(base_address, chipselect);
  db->gpmc.base_address      = base_address;
  db->gpmc.device_address    = device_base_address; 
}

U8 get_nand_onfi_info(T_db * db, BOOLEAN reset)
{
  // Get the NAND device specific parameters

  if(reset)
  {
    drv_reset(db);
  }

  /* Read Vendor ID and Device ID */

  NAND_COMMAND(ONFI_COMMAND_READ_ID);
  NAND_ADDRESS(0x00);
  /* Wait for NAND device ready, give read id to the NAND Device */
  WAIT_MICROSEC(DEVICE_RESET_TIME);
  db->ids.vendor_id = NAND_READ();
  db->ids.device_id = NAND_READ();

  /* Read ONFI record components */

  NAND_COMMAND(ONFI_COMMAND_READ_PARAMETER_PAGE);
  NAND_ADDRESS(0x00);
  /*wait for NAND device ready, after giving read id to the NAND Device */
  WAIT_MICROSEC(DEVICE_DEFAULT_TIME);

  PSREAD(db->device.onfi_signature, 4);
  PREAD(db->device.onfi_revision);
  PREAD(db->device.features.value);
  PSKIP(24);
  PSREAD(db->device.manufacturer, 12);
  PSREAD(db->device.model, 20);
  PREAD(db->device.manufacturer_id);
  PREAD(db->device.date_code);
  PSKIP(13);
  PREAD(db->device.bytes_per_page);
  PREAD(db->device.spare_bytes_per_page);
  PSKIP(6);
  PREAD(db->device.pages_per_block);
  PREAD(db->device.blocks_per_lun);
  PREAD(db->device.luns);
  PREAD(db->device.address_cycles.value);

  /* Calculate derived parameters */

  db->device.bytes_per_block = db->device.bytes_per_page * db->device.pages_per_block;
  db->device.bytes_per_lun   = db->device.bytes_per_block * db->device.blocks_per_lun;
  db->device.size_device     = db->device.bytes_per_lun * db->device.luns;

  /* Check and return */

  return(db->device.onfi_signature[0] == 'O' &&
         db->device.onfi_signature[1] == 'N' &&
         db->device.onfi_signature[2] == 'F' &&
         db->device.onfi_signature[3] == 'I');
}

void output_device_info(T_db * db)
{
  INFO2("NAND %sv%X", db->device.onfi_signature, db->device.onfi_revision);
  INFO2("NAND VENDOR %#02X %s", db->device.manufacturer_id, db->device.manufacturer);
  INFO3("NAND %s DEVICE %#02X %s", db->device.features.options.data_16bit ? "16 BIT" : "8 BIT", db->ids.device_id, db->device.model);
  INFO3("NAND CYCLES %#02x (%d ROW, %d COLUMN)", db->device.address_cycles.value, db->device.address_cycles.options.row, db->device.address_cycles.options.column);
  INFO2("NAND %d BYTES/PAGE (SPARE %d)", db->device.bytes_per_page, db->device.spare_bytes_per_page);
  INFO1("NAND %d PAGES/BLOCK", db->device.pages_per_block);
  INFO1("NAND %d BYTES/BLOCK", db->device.bytes_per_block);
  INFO1("NAND %d BLOCKS/LOGICAL UNIT", db->device.blocks_per_lun);
  INFO1("NAND %d LOGICAL UNIT(s)", db->device.luns);
  INFO1("NAND %d BYTES PER LOGICAL UNIT", db->device.bytes_per_lun);
  INFO1("NAND %d MB TOTAL SIZE", db->device.size_device / 1024 / 1024);
  INFO3("GPMC CS%d BASEADR %#08x DEVICE %#08x", db->gpmc.chipselect, db->gpmc.base_address, db->gpmc.device_address);
  INFO3("GPMC CMD%#08x ADR%#08x DAT%#08x", db->gpmc.registers.command, db->gpmc.registers.address, db->gpmc.registers.data);

  SEND4("%sv%d VENDOR %#02X %s", db->device.onfi_signature, db->device.onfi_revision, db->device.manufacturer_id, db->device.manufacturer);
  SEND3("%s DEVICE %#02X %s", db->device.features.options.data_16bit ? "16 BIT" : "8 BIT", db->ids.device_id, db->device.model);
  SEND2("%d BYTES/PAGE (SPARE %d)", db->device.bytes_per_page, db->device.spare_bytes_per_page);
  SEND2("%d PAGES/BLOCK (%d BYTES/BLOCK)", db->device.pages_per_block, db->device.bytes_per_block);
  SEND2("%d BLOCKS/UNIT (%d BYTES/UNIT)", db->device.blocks_per_lun, db->device.bytes_per_lun);
  if(db->device.luns > 1) SEND1("DRIVER ONLY SUPPORTS 1 OF %d UNITS", db->device.luns);
  SEND1("%d MB TOTAL SIZE", db->device.size_device / 1024 / 1024);
  // SEND3("CS%d BASEADR %#08x DEVICE %#08x", db->gpmc.chipselect, db->gpmc.base_address, db->gpmc.device_address);
  // SEND3("CMD %#08x ADR %#08x DAT %#08x", db->gpmc.registers.command, db->gpmc.registers.address, db->gpmc.registers.data);
}


/*------------------------------------------------------------------------------
| Function    : dnld_init
+------------------------------------------------------------------------------
| Description : Initialization for nand Flash
|
| Parameters  :
|   flash     - [IN/OUT] Flash parameters
|   addr      - [IN] Address
|
| Returns     : result code
+----------------------------------------------------------------------------*/
U32 drv_dnld_init(T_driver_config * config, char ** result)
{
  T_driver_setup_var *setup = get_setup_var();  

  U32 ret = FLASH_DRV_SUCCESS;

  U32 num_pages = 0;
  U32 num_bytes = 0;
  U32 num_blocks = 0;
  U32 mask_bytes = 0;
  U32 mask_pages = 0;
  U32 mask_blocks = 0;

  T_db * db;

  *result = NULL;

  /* Update callback functions pointers and check that the necessary functions are there */

  if(!config->drv_malloc)
  {
    *result = "2ND DID NOT PROVIDE MALLOC";
    return FLASH_DRV_ERROR;
  }

  if(!config->drv_free)
  {
    *result = "2ND DID NOT PROVIDE FREE";
    return FLASH_DRV_ERROR;
  }

  if(!config->wait_microsec)
  {
    *result = "2ND DID NOT PROVIDE WAIT_MICROSEC";
    return FLASH_DRV_ERROR;
  }

  if(!config->starts_with)
  {
    *result = "2ND DID NOT PROVIDE FUNCTION FOR PARSING DATA (STARTS WITH)";
    return FLASH_DRV_ERROR;
  }

  if(!config->get_value)
  {
    *result = "2ND DID NOT PROVIDE FUNCTION FOR PARSING DATA (GET VALUE)";
    return FLASH_DRV_ERROR;
  }

  if(!config->drv_memcpy)
  {
    *result = "2ND DID NOT PROVIDE MEMCPY";
    return FLASH_DRV_ERROR;
  }

  if(!config->drv_memset)
  {
    *result = "2ND DID NOT PROVIDE MEMSET";
    return FLASH_DRV_ERROR;
  }

  config->data = config->malloc(sizeof(T_db));

  db = (T_db *)(config->data);
  
  if(!db)
  {
    *result = "DRIVER DATA MEMORY ALLOCATION ERROR DURING INITIALIZATION";
    return FLASH_DRV_ERROR;
  }

  config->drv_memset(db, 0, sizeof(T_db));

  // TODO: Keeping these duplicate pointers in order not to have to mess around too much with the structure of the code

  db->access.malloc         = config->malloc;
  db->access.free           = config->free;
  db->access.dbg_printf     = config->dbg_printf;
  db->access.send_info      = config->send_info;
  db->access.wait_microsec  = config->wait_microsec;

  /* Parse configuration string */

  if(drv_parse_config(get_setup_const(), setup, config->cstring, config) == FLASH_DRV_ERROR)
  {
    *result = "UNABLE TO FIND CONFIGURATION PARAMETER DURING INITIALIZATION";
    return FLASH_DRV_ERROR;
  }

  set_gpmc(db, (U8)setup[C_CS].value, setup[C_GPMC].value, setup[C_ADDRESS].value);

  db->parameters.erase.bberase = setup[C_BBERASE].value ? TRUE : FALSE;

  if(!setup[C_ONFI].valid || setup[C_ONFI].value)
  {
    if(!get_nand_onfi_info(db, TRUE))
    {
      db->access.free(db);
      *result = "DRIVER UNABLE TO READ ONFI INFORMATION FROM DEVICE";
      return FLASH_DRV_ERROR;
    }
  }
  else
  {
    drv_reset(db);
    
    if((setup[C_BPP].valid == FALSE)  || (setup[C_SBPP].valid == FALSE) || (setup[C_PPB].valid == FALSE)  ||
       (setup[C_BPL].valid == FALSE)  || (setup[C_L].valid == FALSE)    || (setup[C_ACV].valid == FALSE)  ||
       (setup[C_F].valid == FALSE))
    {
      *result = "NOT ALL PARAMETERS SPECICIED FOR NON-ONFI OPERATION";
      return FLASH_DRV_ERROR;
    }

    db->ids.vendor_id = 0;
    db->ids.device_id = 0;
    db->device.features.value = (U16)setup[C_F].value;
    config->drv_memcpy(db->device.manufacturer, "UNKNOWN", 8);
    config->drv_memcpy(db->device.model, "UNKNOWN", 8);
    db->device.manufacturer_id = 0;
    db->device.date_code = 0;
    db->device.bytes_per_page = setup[C_BPP].value;
    db->device.spare_bytes_per_page = (U16)setup[C_SBPP].value;
    db->device.pages_per_block = setup[C_PPB].value;
    db->device.blocks_per_lun = setup[C_BPL].value;
    db->device.luns = (U8)setup[C_L].value;
    db->device.address_cycles.value = (U8)setup[C_ACV].value;

    /* Calculate derived parameters */

    db->device.bytes_per_block = db->device.bytes_per_page * db->device.pages_per_block;
    db->device.bytes_per_lun   = db->device.bytes_per_block * db->device.blocks_per_lun;
    db->device.size_device     = db->device.bytes_per_lun * db->device.luns;

    config->drv_memcpy(db->device.onfi_signature, "NONE", 5);
  }
  output_device_info(db);

  /* Set bad block counter to 0 & init the basic params */
  db->parameters.write.bad_block_count = 0;
  db->parameters.read.bad_block_count  = 0;
  db->parameters.read.last_checked     = DEVICE_NO_CHECK;
  db->parameters.write.last_checked    = DEVICE_NO_CHECK;
  db->parameters.read.bad_block_count  = 0;

  num_pages  = db->device.pages_per_block;
  num_bytes  = db->device.bytes_per_page;
  num_blocks = db->device.blocks_per_lun;
  
  /* The following "algorithm" will work ONLY if the sizes are aligned such that only a single 
     bit (highest)is set in the value */
  
  /* Create the initial masks */
  mask_bytes  = num_bytes - 1;
  mask_pages  = num_pages - 1;
  mask_blocks = num_blocks - 1;

  /* Shift the page masks and block masks to the left */
  mask_pages  *= num_bytes;
  mask_blocks *= num_bytes;
  mask_blocks *= num_pages;

  db->masks.word  = mask_bytes;
  db->masks.page  = mask_pages;
  db->masks.block = mask_blocks;

  //INFO3("MASKS BLOCK%#08x PAGE%#08x WORD%#08x", db->masks.block, db->masks.page, db->masks.word);

  /* Allocate buffer for page data */

  db->page = db->access.malloc(sizeof(T_page) + db->device.bytes_per_page);
  
  if(!db->page)
  {
    db->access.free(db);
    *result = "DRIVER PAGE MEMORY ALLOCATION ERROR DURING INITIALIZATION";
    return FLASH_DRV_ERROR;
  }

  db->page->address = DEVICE_NO_CHECK;
  db->page->offset  = DEVICE_NO_CHECK;

  SEND("ONFI DRIVER INIT COMPLETE");
  return ret;
}

/*------------------------------------------------------------------------------
 | Function    : drv_dnld_deinit
 +------------------------------------------------------------------------------
 | Description : NAND Deinit Function
 |
 | Parameters  :
 |   flash     - [IN] - Flash parms
 |   address   - [IN] - Block address
 |   length    - [IN] - length to erase
 |
 | Returns     :
 +----------------------------------------------------------------------------*/
U32 drv_dnld_deinit(T_driver_config * config, char ** result)
{
  T_db * db = (T_db *)config->data;

  *result = NULL;

  if(db->page)
  {
    db->access.free(db->page);
  }

  if(db)
  {
    db->access.free(db);
  }

  SEND("ONFI DRIVER DEINIT COMPLETE");
  return FLASH_DRV_SUCCESS;
}

/*------------------------------------------------------------------------------
 | Function    : drv_dnld_get_info
 +------------------------------------------------------------------------------
 | Description : NAND get info function
 |
 | Parameters  :
 |
 | Returns     :
 +----------------------------------------------------------------------------*/
U32 drv_dnld_get_info(T_driver_config * config, T_driver_info * info)
{
  T_db * db = (T_db *)config->data;
  info->device_base_address = db->gpmc.device_address;
  info->device_size         = db->device.size_device;
  return FLASH_DRV_SUCCESS;
}



/*------------------------------------------------------------------------------
 | Function    : drv_dnld_erase
 +------------------------------------------------------------------------------
 | Description : NAND Erase Function to erase a block in the nand device
 |
 | Parameters  :
 |   flash     - [IN] - Flash parms
 |   address   - [IN] - Block address
 |   length    - [IN] - length to erase
 |
 | Returns     :
 +----------------------------------------------------------------------------*/
U32 drv_dnld_erase(T_driver_config * config, char ** result, U64 address, U64 length)
{
  U32 status = 0;
  U32 addr = 0;
  U32 original_length = length;
  U32 target_length;
  U16 bad_blocks = 0;
  U16 nof_erased_blocks = 0;
  T_db * db = (T_db *)config->data;
  U8 skip_bb_chk = db->parameters.erase.bberase;
  
  *result = NULL;

  /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
  if(address > 0xFFFFFFFF || length > 0xFFFFFFFF)
  {
    if(length > 0xFFFFFFFF)
    {
      SEND1("64-bit LENGTH (%#08x) NOT SUPPORTED.", length);
    }
    else
    {
      SEND1("64-bit ADDRESS (%#08x) NOT SUPPORTED.", address);
    }
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }

  /* Translate address to proper offset 0 address */
  
  addr = DEVICE_REAL_ADDR(db->gpmc.device_address, address);
  
  if(addr >= db->device.size_device)
  {
    SEND2("ADDRESS %#08x OUT OF RANGE (BOUNDARY %#08x)", addr, db->device.size_device);
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }

  /* Check for block alignment */
  
  if((addr & db->masks.block) != addr)
  {
    SEND2("ADDRESS %#08x NOT ALIGNED TO BLOCK SIZE %#x", address, db->device.bytes_per_block);
    *result = "ADDRESS ALIGNMENT CHECK FAILED";
    return FLASH_DRV_ERROR;
  }
  
  /* Zero length means "ALL" */

  if(length == 0)
  {
    length = db->device.bytes_per_lun;
  } 

  target_length = length;

  INFO("RESET");
  drv_reset(db); /* Reset the flash */ // NEEDED?????
  
  INFO("WRITE PROTECT OFF");
  drv_writeprotect(db, DEVICE_WRITE_PROTECT_OFF);

  while (length)
  {
    if (addr >= db->device.bytes_per_lun) // NOTE: NO SUPPORT FOR MULTIPLE LUNs IN THIS VERSION OF THE DRIVER
    {
      if(!original_length)
      {
        config->send_status("Erase progress", target_length, target_length);
        SEND3("ERASED %d BYTES FROM ADDRESS %#08x (%d BAD BLOCKS)", 
              db->device.bytes_per_lun - (bad_blocks * db->device.bytes_per_block) - DEVICE_REAL_ADDR(db->gpmc.device_address, address), 
              address,
              bad_blocks);
        return FLASH_DRV_SUCCESS;
      }
      else
      {
        /* bad chip.. we ran out of good blocks here!! */
        SEND2("ERASE OF %d BYTES FROM ADDRESS %#08x - NOT ENOUGH GOOD BLOCKS", original_length, address);
        *result = "ERASE FAILED - RAN OUT OF GOOD BLOCKS";
        return FLASH_DRV_ERROR;
      }
    }

    if(!skip_bb_chk)
    {
      U8 bb = 0;
      S32 ret = drv_is_badblock(db, addr & db->masks.block, &bb);
      /* handle already marked bad blocks.. */
      if((ret != OMAPFLASH_SUCCESS) || (bb != DEVICE_BLOCK_IS_GOOD))
      {
        bad_blocks++;
        INFO1("SKIPPING BAD BLOCK AT %#08x", addr & db->masks.block);
        //SEND2("SKIPPING BAD BLOCK #%d AT ADDRESS %#08x", bad_blocks, addr); TODO... Create a list and send out after erase complete
        db->parameters.erase.bad_block_count++;
        /* Go to the next available block.. */
        addr += db->device.bytes_per_block;
        continue;
      }
    }
  
    /* Erase block if not already erased */

    status = drv_erase(db, addr);
   
    if (status != OMAPFLASH_SUCCESS)
    {
      bad_blocks++;
      INFO1("ERASE FAILED - MARKING BLOCK AT %#08x AS BAD", addr & db->masks.block);
      /* Bad block.. could not erase it..go to next after marking this one.. */
      SEND2("BAD BLOCK #%d DETECTED AT ADDRESS %#08x", bad_blocks, addr);
      drv_mark_badblock(db, addr);
      addr += db->device.bytes_per_block;
      db->parameters.erase.bad_block_count++;
      continue;
    }

    nof_erased_blocks++;

    /* Updated last_erased_block if erase succeeded. */
    db->parameters.erase.valid = TRUE;
    /* End of Erased block */
    db->parameters.erase.last_erased = addr + db->device.bytes_per_block - 1;
  
    length = (length < db->device.bytes_per_block) ? 0 : length - db->device.bytes_per_block;
    addr += db->device.bytes_per_block;
    config->send_status("Erase progress", target_length - length, target_length);
  }

  SEND3("ERASED %d BYTES FROM ADDRESS %#08x (%d BAD BLOCKS)", 
        nof_erased_blocks * db->device.bytes_per_block, 
        address,
        bad_blocks);
  return FLASH_DRV_SUCCESS;      
}

/*------------------------------------------------------------------------------
 | Function    : drv_dnld_write
 +------------------------------------------------------------------------------
 | Description : Write function for NAND flash
 |
 | Parameters  :
 |   flash     - [IN] - Flash parms
 |   dest_add  - [IN] - Destination address
 |   src_addr  - [IN] - Source address (pointer to data)
 |   size      - [IN] - Size of data
 |   flags_ptr - [IN] - write flags
 |
 | Returns     :
 +----------------------------------------------------------------------------*/
U32 drv_dnld_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more)
{
  /* ASSUMPTION: Erase has already been called and the requisite blocks have
     been checked for bad blocks */

  T_db * db = (T_db *)config->data;

  /* Dont erase the blocks ever if skip_erase is set.. */
  
  U8 erase_en = 0; // (flash->session.drv_flag & DRVFLAG_SKIP_ERASE) ? 0 : 1;
  U32 remaining_size = size;
  U32 status = 0;
  U32 dst_addr;
  
  *result = NULL;

  /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
  if(dest_addr > 0xFFFFFFFF)
  {
    SEND1("64-bit ADDRESS (%#08x) NOT SUPPORTED.", dest_addr);
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }

  /* Translate address to proper offset 0 address */
  
  dst_addr = DEVICE_REAL_ADDR(db->gpmc.device_address, dest_addr);
  
  if (dst_addr >= db->device.bytes_per_lun) // NO SUPPORT FOR MULTIPLE LUNs IN THIS VERSION
  {
    SEND2("ADDRESS OVERFLOW (%#08x >= %#08x)", dst_addr, db->device.bytes_per_lun);
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }
  
  /* Skip BB if we see any */
  
  dst_addr += db->parameters.write.bad_block_count * db->device.bytes_per_block;
  
  if(erase_en && (dst_addr > db->parameters.erase.last_erased))
  {
    *result = "INTERNAL ERROR - WRITE FAILED, AHEAD OF ERASE";
    return FLASH_DRV_ERROR;
  }
  
  if(db->page->offset != DEVICE_NO_CHECK)
  {
    if(dst_addr != (db->page->address + db->page->offset))
    {
      SEND3("PAGE BUFFER ADDRESS MISMATCH (%#08x != %#08x + %#x)", dst_addr, db->page->address, db->page->offset);
      *result = "INTERNAL ERROR - ADDRESSES MUST BE SEQUENTIAL";
      return FLASH_DRV_ERROR;
    }
  }
  else
  {
    drv_reset(db);        /* Reset the flash */
    drv_writeprotect(db, DEVICE_WRITE_PROTECT_OFF);

    db->page->offset = 0;

    /* Page aligned writes */

    if ((dst_addr & (db->masks.block | db->masks.page)) != dst_addr)
    {
      SEND1("UNALIGNED INITIAL WRITE ADDRESS %#08x", dst_addr);
      *result = "FAILED - START ADDRESS MUST BE PAGE ALIGNED";
      return FLASH_DRV_ERROR;
    }
  }

  while(remaining_size)
  {
    U32 chunk_size = remaining_size > (db->device.bytes_per_page - db->page->offset) ? 
                     db->device.bytes_per_page - db->page->offset : 
                     remaining_size;
    
    db->page->address = dst_addr & (db->masks.block | db->masks.page);

    if(db->page->address >= db->device.bytes_per_lun) // NO SUPPORT FOR MULTIPLE LUNs IN THIS VERSION
    {
      SEND2("ADDRESS OVERFLOW (%#08x >= %#08x)", db->page->address, db->device.bytes_per_lun);
      *result = "RAN OUT OF ADDRESS SPACE";
      return FLASH_DRV_ERROR;
    }
    
    /* Check if this block is erased (only if skip erase is not set) */
  
    if (erase_en && (dst_addr > db->parameters.erase.last_erased))
    {
      /* Possible that we had some bad blocks generated during writes and failed to erase required 
         number of extra blocks */
  
      INFO1("WRITE AHEAD OF ERASE", dst_addr);

      /* Try erasing this block.. if this failed.. try next block */
      
      status = drv_dnld_erase(config, result, dst_addr & db->masks.block, db->device.bytes_per_block);

      if (status != FLASH_DRV_SUCCESS)
      {
          INFO1("ERASE OF BLOCK FAILED (%#08x)", dst_addr & db->masks.block);
          *result = NULL;
          db->parameters.write.bad_block_count++;
          dst_addr += db->device.bytes_per_block;
          status = FLASH_DRV_SUCCESS;
          continue;       /* recheck address */
      }
    }

    /* Check for existing and erase generated bad blocks */
  
    if ((db->parameters.write.last_checked == DEVICE_NO_CHECK) || 
        ((dst_addr & db->masks.block) != db->parameters.write.last_checked))
    {
      U8 bb = 0;
      db->parameters.write.last_checked = dst_addr & db->masks.block;
      status = drv_is_badblock(db, dst_addr & db->masks.block, &bb);
      if ((status != OMAPFLASH_SUCCESS) || (bb != DEVICE_BLOCK_IS_GOOD))
      {
        INFO1("BAD BLOCK DURING WRITE AT %#08x", dst_addr & db->masks.block);
        /* try this loop again with nxt avail block address.. */
        dst_addr += db->device.bytes_per_block;
        db->parameters.write.bad_block_count++;
        status = FLASH_DRV_SUCCESS;
        continue;
      }
    }

    /* Create a page to write */

    INFO3("STORING PAGE DATA FOR ADDRESS %#08x (OFFSET %4d, LENGTH %4d)", db->page->address, db->page->offset, chunk_size);

    config->drv_memcpy((void *)(db->page->data + db->page->offset), (void *)src_addr, chunk_size);

    db->page->offset += chunk_size;

    /* Fill last page */

    if((more == NO_MORE_DATA) && (db->page->offset != db->device.bytes_per_page))
    {
      INFO1("WRITING LAST PAGE", db->page->address);
      config->drv_memset(db->page->data + db->page->offset, 0xFF, db->device.bytes_per_page - db->page->offset);
      db->page->offset = db->device.bytes_per_page;
    }

    /* Check if we have a full page to write */

    if(db->page->offset == db->device.bytes_per_page)
    {
      INFO1("WRITING PAGE TO ADDRESS %#08x", db->page->address);

      /* BUSINESS END GOES HERE ************************************************************************/

      /* write data to nand flash with ECC generation */

      status = drv_write(db,
                         db->page->address,
                         DEVICE_OPERATION_DATA,
                         (U16)db->device.bytes_per_page,
                         db->page->data,
                         DEVICE_OPERATION_ECC_OP,
                         16, 
                         db->parameters.buffers.ecc);

      /* Create the spare area */

      if (status == OMAPFLASH_SUCCESS)
      {
        config->drv_memset(db->parameters.buffers.spare, 0xFF, DEVICE_WORD_SIZE);

        /* Byte offset 0 is bad block marker */
  
        config->drv_memcpy(db->parameters.buffers.spare + DEVICE_WORD_SIZE,
                           db->parameters.buffers.ecc,
                           sizeof(T_device_spare_area) - DEVICE_WORD_SIZE);
        
  //db->access.dbg_printf("DRV INFO: SPARES FOR %#08x : %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x",
  //                       db->page->address, 
  //                       db->parameters.buffers.spare[0] ,
  //                       db->parameters.buffers.spare[1] ,
  //                       db->parameters.buffers.spare[2] ,
  //                       db->parameters.buffers.spare[3] ,
  //                       db->parameters.buffers.spare[4] ,
  //                       db->parameters.buffers.spare[5] ,
  //                       db->parameters.buffers.spare[6] ,
  //                       db->parameters.buffers.spare[7] ,
  //                       db->parameters.buffers.spare[8] ,
  //                       db->parameters.buffers.spare[9] ,
  //                       db->parameters.buffers.spare[10] ,
  //                       db->parameters.buffers.spare[11] ,
  //                       db->parameters.buffers.spare[12] ,
  //                       db->parameters.buffers.spare[13] ,
  //                       db->parameters.buffers.spare[14] ,
  //                       db->parameters.buffers.spare[15]);        

        status = drv_write(db,
                           db->page->address,
                           DEVICE_OPERATION_SPARE, 
                           16,
                           db->parameters.buffers.spare,
                           DEVICE_OPERATION_NO_ECC, 
                           0, 
                           NULL);
      }

      /* In case of any errors, move source pointer to nearest block boundary OR 0 and 
         continue Also, reset the remaining_size parameter */
  
      if (status == FLASH_DRV_INVALID_PARAMETER)
      {
          *result = "INTERNAL ERROR!";
          return status;
      }
      else if (status != OMAPFLASH_SUCCESS)
      {
        SEND1("WRITE OPERATION FAILED AT %#08x - NEW BAD BLOCK", db->page->address);
  
        /* There is no way to easily recover from a new bad block during the flashing
           process. The procedure would have to be to copy the data of the already
           written good pages from the bad block to the next good block, then mark
           the block as bad and continue the flashing process after that. For now,
           fail the process - the user will have to try again and in the next try
           the newly marked bad block will be skipped... */
        
        status = drv_mark_badblock(db, db->page->address);

        if(status != OMAPFLASH_SUCCESS)
        {
          SEND1("FATAL - FAILED TO MARK NEW BAD BLOCK AS BAD AT %#08x", db->page->address | db->masks.block);
        }
        *result = "NEW BAD BLOCK ENCOUNTERED - WRITE ABORTED, RETRY!";
        return FLASH_DRV_ERROR;
      }

      /* BUSINESS END ENDS HERE */

      db->page->address += db->device.bytes_per_page;
      db->page->offset = 0;
    }

    remaining_size -= chunk_size;
    dst_addr       += chunk_size;
    src_addr       += chunk_size;
    
  }
  
  if(more == NO_MORE_DATA)
  {
    db->page->address = DEVICE_NO_CHECK;
    db->page->offset  = DEVICE_NO_CHECK;
  }

  return FLASH_DRV_SUCCESS;
}

/*------------------------------------------------------------------------------
 | Function    : drv_dnld_read
 +------------------------------------------------------------------------------
 | Description : Read function for nand flash
 |
 | Parameters  :
 |   flash     - [IN]  - Flash parms
 |   dest_addr - [OUT] - Destination address (pointer to data)
 |   src_addr  - [IN]  - Source address
 |   size      - [IN]  - Size of data
 |
 | Returns     : return result
 +----------------------------------------------------------------------------*/
U32 drv_dnld_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size)
{
  T_db * db = (T_db *)config->data;
  U32 remaining_size; 
  U32 status = 0;

  remaining_size = size;
  *result = NULL;

  /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
  if(src_addr > 0xFFFFFFFF)
  {
    SEND1("64-bit ADDRESS (%#08x) NOT SUPPORTED.", src_addr);
    *result = "ADDRESS BOUNDARY CHECK FAILED";
    return FLASH_DRV_ERROR;
  }

  config->drv_memset((U8 *)dst_addr, 0x00, size);

  /* Translate address to proper offset 0 address */

  src_addr = DEVICE_REAL_ADDR(db->gpmc.device_address, src_addr);

  if (src_addr >= db->device.size_device)
  {
    SEND1("ADDRESS OVERFLOW %#x", src_addr);
    *result = "FAILED - ADDRESS OUT OF RANGE DURING READ";
    return FLASH_DRV_ERROR;
  }

    /* Skip BB if we see any */

    if (db->parameters.read.bad_block_count)
    {
      src_addr += db->parameters.read.bad_block_count * db->device.bytes_per_block;
    }

    /* Page aligned reads */

    //if (src_addr & (db->masks.block | db->masks.page) != src_addr)
    //{
    //  SEND1("UNALIGNED READ ADDRESS %#08x", src_addr);
    //  *result = "FAILED - START ADDRESS FOR READ NOT PAGE ALIGNED";
    //  return(FLASH_DRV_ERROR);
    //}

    drv_writeprotect(db, DEVICE_WRITE_PROTECT_ON);

    drv_reset(db);        /* Reset the flash */

  while (remaining_size)
  {
    /* Choose the size of data to read - set it to difference first */
    // ** any size >= 0xFFFF fails. need to investigate...
    U16 read_size = (remaining_size >= 0xFFF0)? 0xFFF0 : remaining_size;

    if (src_addr >= db->device.size_device)
    {
      /* Bad chip.. we ran out of good blocks here!! */
  
      *result = "END OF DEVICE MEMORY";
      return FLASH_DRV_ERROR;
    }

    /* Check for existing and erase generated bad blocks */
  
    if ((db->parameters.read.last_checked == DEVICE_NO_CHECK) || ((src_addr & db->masks.block) > db->parameters.read.last_checked))
    {
      U8 bb;
      db->parameters.read.last_checked = src_addr & db->masks.block;
      status = drv_is_badblock(db, src_addr & db->masks.block, &bb);
      if ((status == FLASH_DRV_ERROR) || (bb != DEVICE_BLOCK_IS_GOOD))
      {
        INFO1("SKIPPING BAD BLOCK AT %#08x", src_addr & db->masks.block);        
        src_addr += db->device.bytes_per_block;
        db->parameters.read.bad_block_count++;
        continue;
      }
    }

    /* Read the data */
    status = drv_read_data(config,
                           db,
                           src_addr,
                           &read_size,
                           (U8 *) (dst_addr + (size - remaining_size)),
                           DEVICE_OPERATION_ECC_OP, 
                           16, 
                           db->parameters.buffers.ecc);
  
    if (status != OMAPFLASH_SUCCESS)
    {
      SEND1("UNABLE TO READ DATA AT ADDRESS %#08x", src_addr);
      *result = "FAILED TO COMPLETE DATA READ";
      return FLASH_DRV_ERROR;
    }

    /* All is well.. move on.. */
    remaining_size -= read_size;
    src_addr += read_size;
  }                           /* End of while */

  return FLASH_DRV_SUCCESS;
}

//#endif /* #ifdef DOWNLOAD_DRIVER */

/*==== FIRST LEVEL HELPER FUNCTIONS ==========================================*/

/*------------------------------------------------------------------------------
 | Function    : drv_reset
 +------------------------------------------------------------------------------
 | Description : Reset the flash by sending reset command
 |
 | Parameters  :
 |   nand      - nand pointer
 |
 | Returns     : None
 +----------------------------------------------------------------------------*/
void drv_reset(T_db * db)
{
    NAND_COMMAND(ONFI_COMMAND_RESET);
    /* Wait for Trst+TWb time */
//#ifdef FF_MICRON_WAIT_MONITOR
//    /* Ignore result */
//    mt29f1g_wait_ready(nand, MICRON_RESET_TIME);
//#else
    WAIT_MICROSEC(DEVICE_RESET_TIME);
//#endif
}

#ifdef FF_DEVICE_LOCKED_ON_BOOT
/*------------------------------------------------------------------------------
 | Function    : drv_unlock_chip
 +------------------------------------------------------------------------------
 | Description : Send the Unlock command to the device (ONLY for some device conf)
 |
 | Parameters  :
 |   nand      - nand pointer
 |   s_address - Start Address
 |   e_address - End Address
 |
 | Returns     : return result
 +----------------------------------------------------------------------------*/
static S32 drv_unlock_chip(T_db * db, U32 s_address, U32 e_address)
{
#ifndef FF_INVERTED_DEVICE_LOCK_COMMAND
  U32 start_address = s_address;
  U32 end_address = e_address;
#else
  U32 start_address = e_address;
  U32 end_address = s_address;
#endif
  S32 ret_val;

  /* mask out the address */
  start_address &= db->masks.block | db->masks.page;
  end_address &= db->masks.block | db->masks.page;
  
  NAND_COMMAND(ONFI_COMMAND_BLOCK_UNLOCK_CYCLE1);
  
  ret_val = drv_add_cycle(db, DEVICE_BLOCK_ADDRESS_CYCLE, start_address, DEVICE_OPERATION_DATA);

  if (ret_val != OMAPFLASH_SUCCESS)
  {
    return ret_val;
  }

  NAND_COMMAND(ONFI_COMMAND_BLOCK_UNLOCK_CYCLE2);

  ret_val = drv_add_cycle(db, DEVICE_BLOCK_ADDRESS_CYCLE, end_address, DEVICE_OPERATION_DATA);
  /* Spec does not say about a delay -> it just waits for ALE to go down. I am being paranoid here */
  if (ret_val == OMAPFLASH_SUCCESS)
  {
//#ifdef FF_MICRON_WAIT_MONITOR
//        ret_val = mt29f1g_wait_ready(nand, MICRON_WAIT_UNLOCK2);
//#else
    WAIT_MICROSEC(DEVICE_WAIT_UNLOCK2);
//#endif
  }
  return (ret_val);
}
#endif /* FF_DEVICE_LOCKED_ON_BOOT */

/*------------------------------------------------------------------------------
 | Function    : drv_erase
 +------------------------------------------------------------------------------
 | Description : erase a block of data
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   address   - block aligned address
 |
 | Returns     : operation result
 +----------------------------------------------------------------------------*/
S32 drv_erase(T_db * db, U32 address)
{
  S32 ret_val = OMAPFLASH_SUCCESS;

  INFO1("ERASING BLOCK AT ADDRESS %#08x", address);

  NAND_COMMAND(ONFI_COMMAND_BLOCK_ERASE_CYCLE1);

  /* 2 cycle addressing identifying the block address,page and byte offset */

  ret_val = drv_add_cycle(db, DEVICE_BLOCK_ADDRESS_CYCLE, address, DEVICE_OPERATION_DATA);

  if (ret_val != OMAPFLASH_SUCCESS)
  {
    ERROR1("BAD ADDRESS %#08x FOR ERASE", address);
    drv_reset(db);
    return ret_val;
  }

  NAND_COMMAND(ONFI_COMMAND_BLOCK_ERASE_CYCLE2);
//#ifdef FF_MICRON_WAIT_MONITOR
//    ret_val = mt29f1g_wait_ready(nand, MICRON_ERASE_WAIT_TIME);
//    if (ret_val != OMAPFLASH_SUCCESS)
//    {
//        DBG_MICRON_ERROR("Erase status r/b timeout ", address);
//        DBG_MICRON_HANGERR("Erase quits", address);
//        mt29f1g_reset(nand);
//        return ret_val;
//    }
//#else
  WAIT_MICROSEC(DEVICE_ERASE_WAIT_TIME);
//#endif
  /* Read the status and ensure that erase operation did not fail due to either 
     a  write protect OR a failure.. or even a timeout! */

  ret_val = drv_wrstatus(db, DEVICE_ERASE_TIMEOUT);
  if (ret_val != OMAPFLASH_SUCCESS)
  {
    ERROR1("ERASE BAD BLOCK STATUS %#08x", address);
    drv_reset(db);
  }
  return ret_val;
}

/*------------------------------------------------------------------------------
 | Function    : drv_write
 +------------------------------------------------------------------------------
 | Description : write to a nand page
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   address   - dest address where the data should be written to
 |   spare_op  - is this a spare area operation
 |   data_len  - lendth of data in bytes
 |   data      - pointer to data buffer
 |   ecc_enable - enable ecc for data operation?
 |   ecc_len   - length of ecc buffer
 |   ecc_data  - ecc data to be returned back
 |
 | Returns     : result of operation
 +----------------------------------------------------------------------------*/
S32 drv_write(T_db * db, U32 address, U8 spare_op, U16 data_len, U8 * data, U8 ecc_enable, U8 ecc_len, U8 * ecc_data)
{
  S32 ret_val = OMAPFLASH_SUCCESS;

  /* Take only the page aligned address */
  address &= db->masks.block | db->masks.page;

  /* Programming sequence is as follows: prog_start command,address,data,prog_end,read status
     Enable ecc if required for data write path ONLY */

  NAND_COMMAND(ONFI_COMMAND_PAGE_PROGRAM_CYCLE1);

  ret_val = drv_add_cycle(db, DEVICE_PAGE_ADDRESS_CYCLE, address, spare_op);

  if (ret_val != OMAPFLASH_SUCCESS)
  {
     ERROR1("BAD ADDRESS %#08x IN WRITE", address);
     drv_reset(db);
     return ret_val;
  }

  if (ecc_enable) drv_init_ecc(db);

  /* pump in the data.. */

  while (data_len)
  {
    NAND_WRITE_PTR(data);
    data += DEVICE_WORD_SIZE;
    data_len -= DEVICE_WORD_SIZE;
  }

  if (ecc_enable) 
  {
      ret_val = drv_end_ecc(db, ecc_len, ecc_data);
      if (ret_val != OMAPFLASH_SUCCESS)
      {
        ERROR1("Internal error at %#08x", address);
        drv_reset(db);
        return ret_val;
      }
  }

  NAND_COMMAND(ONFI_COMMAND_PAGE_PROGRAM_CYCLE2);

  /* Read the status and ensure that write operation did not fail due to either a write 
     protect OR a failure.. or even a timeout! */

  ret_val = drv_wrstatus(db, DEVICE_WRITE_TIMEOUT);

  if (ret_val != OMAPFLASH_SUCCESS)
  {
    ERROR1("BAD WRITE STATUS/TIMEOUT AT %#08x", address);
    drv_reset(db);
  }

  return ret_val;
}

/*------------------------------------------------------------------------------
 | Function    : drv_read_data
 +------------------------------------------------------------------------------
 | Description :
 |
 | Parameters  :
 |   nand      - nand flash pointer
 |   address   - address to read from
 |   data_len  - length of data to read
 |   data      - pointer to data buffer
 |   ecc_enable - do ecc operation for the data?
 |   ecc_len   - length of ecc buffer
 |   ecc_data  - ecc buffer
 |
 | Returns     : return the result of operation
 +----------------------------------------------------------------------------*/
S32 drv_read_data(T_driver_config * config, T_db * db, U32 address, U16 * data_len, U8 * data, U8 ecc_enable, U8 ecc_len, U8 * ecc_data)
{
  S32 ret_val = OMAPFLASH_SUCCESS;
  U32 page_address = address & (db->masks.block | db->masks.page);
  U32 page_offset  = address & db->masks.word;
  U32 max_return   = db->device.bytes_per_page - page_offset;

  if (*data_len > max_return)
  {
    *data_len = (U16)max_return;
  }

  if((NAND16BIT) && ((U32)data & 0x00000001))
  {
    ERROR1("16 BIT ACCESS BASED ON ODD TARGET ADDRESS %#08x", data);
    return(OMAPFLASH_DAL_ERROR);
  }

  // LOE ASSUMPTION: We need to always start from a page aligned address in 
  // order to have ECC working properly and to read a whole page at a time
  // to be able to correct problems

  if(page_address != db->page->address)
  {
    U32 data_count;
  
    // New page read necessary

    db->page->address = page_address;
    db->page->offset  = DEVICE_NO_CHECK;

    INFO2("PAGE READ AT ADDRESS %#08x (%d BYTES)", page_address, db->device.bytes_per_page);

    /* Read sequence is as follows:
     * read command,address,read_conf, wait till Rb  line is gone,
     * Normally with wait_on_busy line is setup, things are good..
     * else, we read status, till we go into ready status, set the
     * read mode again and  read data
     * enable ecc if required for data read path ONLY
     */

    NAND_COMMAND(ONFI_COMMAND_READ_CYCLE1);

    ret_val = drv_add_cycle(db, DEVICE_PAGE_ADDRESS_CYCLE, page_address, DEVICE_OPERATION_DATA);

    if (ret_val != OMAPFLASH_SUCCESS)
    {
      ERROR1("READ BAD ADDRESS %#x", address);
      drv_reset(db);
      return ret_val;
    }

    NAND_COMMAND(ONFI_COMMAND_READ_CYCLE2);

    /* Data sheet recommends using wait ready(R/B). we have an alternate workaround strategy too. */

    //#ifdef FF_MICRON_WAIT_MONITOR
    //    ret_val = mt29f1g_wait_ready(nand, MICRON_READ_TIMEOUT_MS);
    //    if (ret_val != OMAPFLASH_SUCCESS)
    //    {
    //        DBG_MICRON_ERROR("Read bad status/timedout", address);
    //        DBG_MICRON_HANGERR("Read quits", address);
    //        mt29f1g_reset(nand);
    //        return ret_val;
    //    }
    //#else
      
    /* Wait at least tR time = max 25 ns */

    WAIT_MICROSEC(DEVICE_LEAST_WAIT);

    /* Read the status and wait till data is available due to either a failure.. or even a timeout! */
    
    ret_val = drv_read_status(db, DEVICE_READ_TIMEOUT);
    
    if (ret_val != OMAPFLASH_SUCCESS)
    {
      ERROR1("READ BAD ADDRESS %#x", address);
      drv_reset(db);
      return ret_val;
    }
    
    NAND_COMMAND(ONFI_COMMAND_READ_CYCLE1);

    //#endif /* FF_MICRON_WAIT_MONITOR */

    if (ecc_enable) drv_init_ecc(db);

    /* pump out the data.. */

    for(data_count = 0; data_count < (db->device.bytes_per_page / DEVICE_WORD_SIZE); data_count++)
    {
      if(NAND16BIT)
      {
        ((U16 *)(db->page->data))[data_count] = NAND_READ();
      }
      else
      {
        ((U8 *)(db->page->data))[data_count] = NAND_READ();

      }
    }

    if (ecc_enable) 
    {
        ret_val = drv_end_ecc(db, ecc_len, ecc_data);
        if (ret_val != OMAPFLASH_SUCCESS)
        {
            ERROR1("Internal error at %#08x", address);
            drv_reset(db);
            return ret_val;
        }
    }

    #ifdef SANE_ECC_READ
    if(ecc_enable)
    {
      U16 spare_size = 16;

      /* Read the spare for ECC */

      if(drv_read_spare(db,
                        page_address & (db->masks.page | db->masks.block),
                        &spare_size,
                        (U8 *) db->parameters.buffers.spare) == 
         FLASH_DRV_SUCCESS)
      {
        /* Skip the bad block region by adding DEVICE_WORD_SIZE */

        if(drv_ecc_compare_correct((U32)db->page->data, 
                                   db,
                                   ((U32)db->parameters.buffers.spare) + DEVICE_WORD_SIZE,
                                   (U32)db->parameters.buffers.ecc,
                                   (U32)db->page->data,
                                   db->device.bytes_per_page) !=
           FLASH_DRV_SUCCESS)
        {
          /* Check for spare area being all 0xFF.. empty flash is ok to give back */

          U32 spare_count = 0;
          U8 net_res      = 0xFF;
  
          while (spare_count < spare_size)
          {
              net_res &= db->parameters.buffers.spare[spare_count];
              spare_count++;
          }
  
          if (net_res != 0xFF)
          {
            ERROR1("ECC CORRECTION FAILED FOR PAGE AT ADDRESS %#08x", page_address);
            
            /* Commenting out following return will skip this page and go to the next.. */
            return OMAPFLASH_DAL_ERROR;
          }
        }
      }
      else
      {
        ERROR("FAILED TO READ SPARE AREA FOR ECC CORRECTION");
        return OMAPFLASH_DAL_ERROR;
      }
    }
    #endif

  }

//  INFO3("READING DATA %#08x - %#08x (%03d)", address, address + *data_len - 1, *data_len);

  config->drv_memcpy(data, db->page->data + page_offset, *data_len);

  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
 | Function    : drv_read_spare
 +------------------------------------------------------------------------------
 | Description :
 |
 | Parameters  :
 |   nand      - nand flash pointer
 |   address   - address to read from
 |   data_len  - length of data to read
 |   data      - pointer to data buffer
 |
 | Returns     : return the result of operation
 +----------------------------------------------------------------------------*/
S32 drv_read_spare(T_db * db, U32 address, U16 * data_len, U8 * data)
{
  S32 ret_val = OMAPFLASH_SUCCESS;
  U16 data_count;
  U32 page_address = address & (db->masks.block | db->masks.page);
  U32 page_offset  = address & db->masks.word;
  U32 max_return   = db->device.spare_bytes_per_page - page_offset;
                   
  if (max_return >= 1 << 8 * sizeof *data_len)
  {
      return FLASH_DRV_INVALID_PARAMETER;
  }

  if (*data_len > max_return)
  {
    *data_len = (U16)max_return;
  }

  data_count = *data_len;

  if((NAND16BIT) && ((U32)data & 0x00000001))
  {
    ERROR1("16 BIT ACCESS BASED ON ODD TARGET ADDRESS %#08x", data);
    return(OMAPFLASH_DAL_ERROR);
  }

  if((NAND16BIT) && (*data_len & 0x00000001))
  {
    ERROR1("16 BIT ACCESS FOR READ OF ODD BYTE COUNT %d", *data_len);
    return(OMAPFLASH_DAL_ERROR);
  }

  /* Read sequence is as follows:
   * read command,address,read_conf, wait till Rb  line is gone,
   * Normally with wait_on_busy line is setup, things are good..
   * else, we read status, till we go into ready status, set the
   * read mode again and  read data
   * enable ecc if required for data read path ONLY
   */

  NAND_COMMAND(ONFI_COMMAND_READ_CYCLE1);

  ret_val = drv_add_cycle(db, DEVICE_PAGE_ADDRESS_CYCLE, page_address, DEVICE_OPERATION_SPARE);

  if (ret_val != OMAPFLASH_SUCCESS)
  {
    ERROR1("READ BAD ADDRESS %#x", address);
    drv_reset(db);
    return ret_val;
  }

  NAND_COMMAND(ONFI_COMMAND_READ_CYCLE2);

  /* Data sheet recommends using wait ready(R/B). we have an alternate workaround strategy too. */

  //#ifdef FF_MICRON_WAIT_MONITOR
  //    ret_val = mt29f1g_wait_ready(nand, MICRON_READ_TIMEOUT);
  //    if (ret_val != OMAPFLASH_SUCCESS)
  //    {
  //        DBG_MICRON_ERROR("Read bad status/timedout", address);
  //        DBG_MICRON_HANGERR("Read quits", address);
  //        mt29f1g_reset(nand);
  //        return ret_val;
  //    }
  //#else
    
  /* Wait at least tR time = max 25 ns */

  WAIT_MICROSEC(DEVICE_LEAST_WAIT);

  /* Read the status and wait till data is available due to either a failure.. or even a timeout! */
  
  ret_val = drv_read_status(db, DEVICE_READ_TIMEOUT);
  
  if (ret_val != OMAPFLASH_SUCCESS)
  {
      ERROR1("READ BAD ADDRESS %#x", address);
      drv_reset(db);
      return ret_val;
  }
  
  NAND_COMMAND(ONFI_COMMAND_READ_CYCLE1);

  //#endif /* FF_MICRON_WAIT_MONITOR */

  /* pump out the data.. */

  while(data_count)
  {
    if(NAND16BIT)
    {
      *((U16 *)(data)) = NAND_READ();
    }
    else
    {
      *((U8 *)(data)) = NAND_READ();

    }
    data += DEVICE_WORD_SIZE;
    data_count -= DEVICE_WORD_SIZE;
  }

  //db->access.dbg_printf("DRV INFO: %02d SPARES AT %#08x : %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x",
  //                       *data_len,
  //                       address, 
  //                       ((U8 *)data - *data_len)[0] ,
  //                       ((U8 *)data - *data_len)[1] ,
  //                       ((U8 *)data - *data_len)[2] ,
  //                       ((U8 *)data - *data_len)[3] ,
  //                       ((U8 *)data - *data_len)[4] ,
  //                       ((U8 *)data - *data_len)[5] ,
  //                       ((U8 *)data - *data_len)[6] ,
  //                       ((U8 *)data - *data_len)[7] ,
  //                       ((U8 *)data - *data_len)[8] ,
  //                       ((U8 *)data - *data_len)[9] ,
  //                       ((U8 *)data - *data_len)[10] ,
  //                       ((U8 *)data - *data_len)[11] ,
  //                       ((U8 *)data - *data_len)[12] ,
  //                       ((U8 *)data - *data_len)[13] ,
  //                       ((U8 *)data - *data_len)[14] ,
  //                       ((U8 *)data - *data_len)[15]);

  //INFO3("READING SPARES %#08x - %#08x (%03d)", address , address + *data_len - 1, *data_len);

  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
 | Function    : drv_is_badblock
 +------------------------------------------------------------------------------
 | Description : checks if the block is bad
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   address   - address whose block needs to be checked
 |   is_badblock - pointer to variable which contains good/bad block info
 |
 | Returns     : the spare area read operation result
 +----------------------------------------------------------------------------*/
S32 drv_is_badblock(T_db * db, U32 address, U8 * is_badblock)
{
    T_device_spare_area spare_data;
    U8 page_count;
    S32 ret_val;

    /* I dont care which address u give me, i will sync to nearest Block boundary */

    address &= db->masks.block;
  
    INFO1("CHECK FOR BB @ %#08x", address);

    for (page_count = 0; page_count < 2; page_count++)
    {
        U16 length = sizeof(spare_data);
        ret_val = drv_read_spare(db, 
                                 address, 
                                 &length, //sizeof(spare_data), 
                                 (U8 *)&spare_data);

        if(ret_val != OMAPFLASH_SUCCESS)
        {
            ERROR2("BAD BLOCK FAILED TO READ DATA %#08x SIZE %d", address, sizeof(spare_data));
            return ret_val;
        }

        //INFO2("SPARE %#08x %#02x", address, spare_data.data8.bad_block1);
        
        if(NAND16BIT)
        {
          if(spare_data.data16.bad_block1 != DEVICE_GOOD_BLOCK_MARKER)
          {
            *is_badblock = DEVICE_BLOCK_IS_BAD;
            INFO1("BAD BLOCK AT ADDRESS %#08x", address);
            return OMAPFLASH_SUCCESS;
          }
        }
        else
        {
          if(spare_data.data8.bad_block1 != DEVICE_GOOD_BLOCK_MARKER)
          {
            *is_badblock = DEVICE_BLOCK_IS_BAD;
            INFO1("BAD BLOCK AT ADDRESS %#08x", address);
            return OMAPFLASH_SUCCESS;
          }
        }

        if(!page_count) // Just to get the right info value out
        {
          address += db->device.bytes_per_page * (db->device.pages_per_block - 1); // FIRST AND LAST
        }
    }
    INFO1("GOOD BLOCK AT ADDRESS %#08x", address & db->masks.block);
    *is_badblock = DEVICE_BLOCK_IS_GOOD;
    return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
 | Function    : drv_mark_badblock
 +------------------------------------------------------------------------------
 | Description : Mark a block as bad
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   address   - address which block needs to be marked as bad
 |
 | Returns     : failure or success of operation
 +----------------------------------------------------------------------------*/
S32 drv_mark_badblock(T_db * db, U32 address)
{
  T_device_spare_area spare_data;
  U8 page_count;
  S32 ret_val[2];
  
  /* I dont care which address u give me, i will sync to nearest block boundary */

  address &= db->masks.block;

  INFO1("MARKING BLOCK AT %#08x AS BAD", address);

  /* Set up the marker as bad.. dont care about rest of data.. */

  if(NAND16BIT)
  {
    spare_data.data16.bad_block1  = DEVICE_BAD_BLOCK_MARKER;
  }
  else
  {
    spare_data.data8.bad_block1  = DEVICE_BAD_BLOCK_MARKER;
  }

  for (page_count = 0; page_count < 2; page_count++)
  {
    ret_val[page_count] = drv_write(db, 
                                    address, 
                                    DEVICE_OPERATION_SPARE, 
                                    DEVICE_WORD_SIZE, //sizeof(spare_data), 
                                    (U8 *)&spare_data, 
                                    DEVICE_OPERATION_NO_ECC, 
                                    0, 
                                    NULL);

    address += db->device.bytes_per_page * (db->device.pages_per_block - 1); // FIRST AND LAST
  }

  return ((ret_val[0] == OMAPFLASH_SUCCESS) || (ret_val[1] == OMAPFLASH_SUCCESS)) ? 
         OMAPFLASH_SUCCESS : 
         OMAPFLASH_DAL_ERROR;
}
//
/*------------------------------------------------------------------------------
 | Function    : mt29f1g_writeprotect
 +------------------------------------------------------------------------------
 | Description : Write protect enable/disable to the device
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   protect_en - enable/disable write protect
 |
 | Returns     : None
 +----------------------------------------------------------------------------*/
void drv_writeprotect(T_db * db, U8 protect_en)
{
  U32 config_reg = 0;
  U32 reg_addr = db->gpmc.base_address + OMAP_GPMC_CONFIG;

  config_reg = in_regl(reg_addr);
  
  /* Write protect line is assumed to be active low for these devices */

  if (protect_en == DEVICE_WRITE_PROTECT_ON)
  {
    config_reg &= ~0x00000010;  /* GPMC WP line is low */
  }
  else
  {
    config_reg |= 0x00000010;   /* GPMC Wp line is high */
  }

  out_regl(reg_addr, config_reg);
#ifdef FF_DEVICE_LOCKED_ON_BOOT
  /* Dont bother about a specific region, unlock the entire flash
     Weird config.. adds cycles to every erase/write op! */
  if (protect_en == DEVICE_WRITE_PROTECT_OFF)
  {
    /* Till the last block */
    drv_unlock_chip(db, 0, db->masks.block);
  }
#else
  WAIT_MICROSEC(DEVICE_WRITE_PROTECT_DELAY);
#endif
}

/*==== LOWLEVEL HELPER FUNCTIONS ==========================================*/

/*------------------------------------------------------------------------------
 | Function    : drv_add_cycle
 +------------------------------------------------------------------------------
 | Description : Do Address cycle operation - translate provided address to
 |               required address cycles
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   mode      - what mode -> block address/page address?
 |   address   - address i want to put on the bus
 |   spare_op  - is this a spare area operation?
 |
 | Returns     : success, failure in case of check failure in debug mode
 +----------------------------------------------------------------------------*/
static S32 drv_add_cycle(T_db * db, U8 mode, U32 address, U8 spare_op)
{
  U32 col_add;
  U32 page_add;
  U32 block_add;

  col_add = DEVICE_WORD_ADDR(address) % DEVICE_WORD_ADDR(db->device.bytes_per_page);

  /* if this is a spare area operation, add additional addressing */

  col_add += (spare_op == DEVICE_OPERATION_SPARE) ? DEVICE_WORD_ADDR(db->device.bytes_per_page) : 0;

  page_add = DEVICE_WORD_ADDR(address) / DEVICE_WORD_ADDR(db->device.bytes_per_page);
  block_add = page_add / (db->device.pages_per_block);
  page_add = page_add % (db->device.pages_per_block);

  //INFO3("DRV CYCLE: COL %#08x PAGE %#08x BLOCK %#08x", col_add, page_add, block_add);
  //INFO1("CYCLE1 = %#02x", DEVICE_ADDTRANS_FIRST(col_add));
  //INFO1("CYCLE2 = %#02x", DEVICE_ADDTRANS_SECOND(col_add));

  if(mode == DEVICE_PAGE_ADDRESS_CYCLE)
  {
    NAND_ADDRESS(DEVICE_ADDTRANS_FIRST(col_add));
    NAND_ADDRESS(DEVICE_ADDTRANS_SECOND(col_add));
  }

  //INFO1("CYCLE3 = %#02x", DEVICE_ADDTRANS_THIRD(page_add, block_add));
  //INFO1("CYCLE4 = %#02x", DEVICE_ADDTRANS_FOURTH(block_add));
  
  NAND_ADDRESS(DEVICE_ADDTRANS_THIRD(page_add, block_add));
  NAND_ADDRESS(DEVICE_ADDTRANS_FOURTH(block_add));

  if(db->device.address_cycles.options.row > 2)
  {
    NAND_ADDRESS(DEVICE_ADDTRANS_FIFTH(block_add));
    //INFO1("CYCLE5 = %#02x", DEVICE_ADDTRANS_FIFTH(block_add));
  }

  return OMAPFLASH_SUCCESS;
}
//
/*------------------------------------------------------------------------------
 | Function    : drv_wrstatus
 +------------------------------------------------------------------------------
 | Description : Translate the status read for write operations
 |
 | Parameters  :
 |   nand      - nand pointer
 |   timeout   - timeout in microsecs
 |
 | Returns     : success or failure for timeout/error conditions
 +----------------------------------------------------------------------------*/
static S32 drv_wrstatus(T_db * db, U32 timeout)
{
  U16 status;

  /* We shall now poll on status till we see error/success */
  while(timeout)
  {
    drv_status(db, &status);
 
    if (status & (ONFI_STATUS_ERROR | ONFI_STATUS_READY))
    {
      break;
    }
    /* if device is protected.. we'd see that bit as 0 */
    if ((status & ONFI_STATUS_WRITE_PROTECT) == 0)
    {
      ERROR1("DEVICE IS WRITEPROTECTED (STATUS %#08x)", status);
      return OMAPFLASH_DAL_ERROR;
    }
    /* reduce time by at least least wait time */
    timeout -= (timeout < DEVICE_LEAST_WAIT) ? timeout : DEVICE_LEAST_WAIT;
    WAIT_MICROSEC(DEVICE_LEAST_WAIT);
  }                           /* End of timeout */

  if ((status & ONFI_STATUS_ERROR) || (timeout == 0))
  {
    ERROR2("WRITE STATUS %#08x AT TIMEOUT %d", status, timeout);
    return OMAPFLASH_DAL_ERROR;
  }
  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
 | Function    : drv_rdstatus
 +------------------------------------------------------------------------------
 | Description : translate the status reg for read operation
 |
 | Parameters  :
 |   nand      - nand pointer
 |   timeout   - timeout in microsec
 |
 | Returns     : returns success or failure if status of error/timeout
 +----------------------------------------------------------------------------*/
//#ifndef FF_MICRON_WAIT_MONITOR
static S32 drv_read_status(T_db * db, U32 timeout)
{
  U16 status;

  /* We shall now poll on status till we see error/success */
  while (timeout)
  {
    drv_status(db, &status);
    if (status & (ONFI_STATUS_ERROR | ONFI_STATUS_READY))
    {
      break;
    }
    /* Dont care about the write protect status - reduce time by at least least wait time */
    timeout -= (timeout < DEVICE_LEAST_WAIT) ? timeout : DEVICE_LEAST_WAIT;
    WAIT_MICROSEC(DEVICE_LEAST_WAIT);
  }
  if ((status & (ONFI_STATUS_ERROR)) || (timeout == 0))
  {
    ERROR2("TIMEOUT %d / STATUS %#04x", timeout, status);
    return OMAPFLASH_DAL_ERROR;
  }
  return OMAPFLASH_SUCCESS;
}
//#endif

/*------------------------------------------------------------------------------
 | Function    : drv_status
 +------------------------------------------------------------------------------
 | Description : basic status read operation
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   status    - return status of the operation
 |
 | Returns     : None
 +----------------------------------------------------------------------------*/
static void drv_status(T_db * db, U16 * status)
{
    NAND_COMMAND(ONFI_COMMAND_READ_STATUS);
    /* typical wait duration is 45ns */
    WAIT_MICROSEC(DEVICE_STATUS_READ_TIME);
    *status = NAND_READ();
}

/*------------------------------------------------------------------------------
 | Function    : drv_init_ecc
 +------------------------------------------------------------------------------
 | Description : Initialize the ECC operation
 |
 | Parameters  :
 |   nand      - pointer to nand
 |
 | Returns     : None
 +----------------------------------------------------------------------------*/
static void drv_init_ecc(T_db * db)
{
  /* ECC engine is configured for CS1, for each CS, the corresponding value
     must be loaded to ECC config register */

  /* Clear ECC + use ecc reg1 */
  out_regl(db->gpmc.base_address + OMAP_GPMC_ECC_CONTROL, 0x00000101);
  /* Set size 0 and 1 to 0xFF (512 bytes) and point all regs to size 0 */
  out_regl(db->gpmc.base_address + OMAP_GPMC_ECC_SIZE_CONFIG, 0x3fcff000); 

  out_regl(db->gpmc.base_address + OMAP_GPMC_ECC_CONFIG, 
           0x1 |                      /* ECC Enable */ 
           db->gpmc.chipselect << 1 | /* Which CS to enable */ 
           NAND16BIT << 7);           /* run ecc on all 16/8 columns, depending on device type */
}

//#ifdef FF_MICRON_WAIT_MONITOR
///*------------------------------------------------------------------------------
// | Function    : mt29f1g_wait_ready
// +------------------------------------------------------------------------------
// | Description : Poll the GPMC Wait ready bit to figure out device readiness state
// |
// | Parameters  :
// |   nand      - nand structure
// |   timeout   - timeout period
// |
// | Returns     : Success if device is ready, else returns error on timeout
// +----------------------------------------------------------------------------*/
//static S32 mt29f1g_wait_ready(T_MICRON_CONFIG_PARAMS * nand,
//    U32 timeout)
//{
//    while (timeout)
//    {
//        U32 reg_val = in_regl(nand->controller_gpmc_base + MICRON_GPMC_STATUS);
//        if ((reg_val & MICRON_WAIT_READY_MASK) == MICRON_WAIT_READY_MASK)
//        {
//            break;
//        }
//
//        /* reduce time by at least least wait time */
//        timeout -= (timeout < MICRON_LEAST_WAIT) ? timeout : MICRON_LEAST_WAIT;
//        nand->wait_microsec(MICRON_LEAST_WAIT);
//    }                           /* End of timeout */
//    if (timeout == 0)
//    {
//        return OMAPFLASH_DAL_ERROR;
//    }
//    return OMAPFLASH_SUCCESS;
//}
//#endif

/*------------------------------------------------------------------------------
 | Function    : drv_end_ecc
 +------------------------------------------------------------------------------
 | Description : End and grab the ecc data
 |
 | Parameters  :
 |   nand      - pointer to nand
 |   ecc_len   - length of ecc buffer
 |   ecc_data  - pointer to ecc buffer
 |
 | Returns     : None
 +----------------------------------------------------------------------------*/
static S32 drv_end_ecc(T_db * db, U8 ecc_len, U8 * ecc_data)
{
  U32 num_regs_ex = db->device.bytes_per_page >> 9; // Size config for ECC is 0xFF => 512 bytes per register
  U8 num_regs;
  U8 reg_num  = 0;
  U8 ecc_idx  = 0;
  U32 reg_val = 0;

  if (num_regs_ex >= 1 << 8 * sizeof num_regs)
  {
      return FLASH_DRV_INVALID_PARAMETER;
  }
  num_regs = (U8)num_regs_ex;

 while(num_regs)
 {
    reg_val = in_regl(db->gpmc.base_address + OMAP_GPMC_ECC_RESULT_BASE + (reg_num * 4));

    /* ECC-x[0] where x is from  A-D */
    ecc_data[ecc_idx] = ECC_P1_128_E(reg_val);
    /* ECC-x[1] where x is  from A-D */
    ecc_data[ecc_idx + 1] = ECC_P1_128_O(reg_val);
    /* ECC-x[2]  where x  is from  A-D */
    ecc_data[ecc_idx + 2] = ECC_P512_2048_E(reg_val) | ECC_P512_2048_O(reg_val) << 4;

    num_regs--;
    reg_num++;
    ecc_idx += 3;
  }
  /* no more ecc */
  out_regl(db->gpmc.base_address + OMAP_GPMC_ECC_CONFIG, 0x0);
  return FLASH_DRV_SUCCESS;
}

#ifdef SANE_ECC_READ
/*------------------------------------------------------------------------------
 | Function    : drv_check_num_bits
 +------------------------------------------------------------------------------
 | Description : Count the number of 1-bits in a value
 |
 | Parameters  :
 |   n         - the value to count
 |
 | Returns     : count of bits
 +----------------------------------------------------------------------------*/
static U8 drv_check_num_bits(U32 n)
{
    U8 count = 0;
    while (n)
    {
        count += n & 0x01;
        n >>= 1;
    }
    return count;
}

/*------------------------------------------------------------------------------
 | Function    : drv_ecc_compare_correct
 +------------------------------------------------------------------------------
 | Description : Compute and correct the ECC in the spare area.
 | We use a macro to change the ECC from the three byte TI standard flash layout:
 | MSB -> LSB
 | [P2048 P1024 P512 P256 P'2048 P'1024 P'512 P'256]
 | [P128 P64 P32 P16 P8 P4 P2 P1] [P'128 P'64 P'32 P'16 P'8 P'4 P'2 P'1]
 |
 | into an U32(GPMC register compatible) having the following format:
 | [0 0 0 0 P2048 P1024 P512 P256 P128 P64 P32 P16 P8 P4 P2 P1 0 0 0 0 \
 |   P'2048 P'1024 P'512 P'256 P'128 P'64 P'32 P'16 P'8 P'4 P'2 P'1]
 |
 | This way, it is easier XOR'ing and counting the ECC bits, and P and P' parities
 | can be deducted from the two halves of the U32.
 | b) XOR the original(from spare area) and read-back converted ECC values.
 | c) If the XOR result contains only 1 bit set as 1, the ECC value we had stored
 |     has error. We ignore the ecc result and return data back to host.
 |     comparison there will verify the data.
 | d) If the XOR contains parity bits (12) number of bits set as 1, then there is
 |      a single bit error, and we can correct it using the following logic:
 |     i) bit_location = bits 16 to 18 of the xor result.
 |     ii)  byte_location = bits 19 to 28 of the xor result.
 |     iii) Flip the bit_location in offset byte_location of the 512 byte chunk of data.
 | e) In all other cases, return a failure result. It could a 2 bit error we detected.
 |
 | Parameters  :
 |   src_addr  - source address of the data that needs checking/correcting
 |   nand      - pointer to nand struct
 |   spare_buffer - spare area read from flash (pointer to first byte in ecc)
 |                  we need to skip bad blocks when we give to the function
 |   ecc_buffer - ECC computed when we read the data
 |   data_buffer - Size of the data buffer (src_address)
 |   read_size - what is the chunk size?
 |
 | Returns     : SUCCESS if corrected/no error/ECC itself has error (host corr)
 |               Failure if ECC error was detected
 +----------------------------------------------------------------------------*/
static U8 drv_ecc_compare_correct(U32 src_addr, T_db * db, U32 spare_buffer, U32 ecc_buffer, U32 data_buffer, U32 read_size)
{
  U32 data_idx    = 0;
  U8 *ecc         = (U8 *)ecc_buffer;
  U8 *spare       = (U8 *)spare_buffer;

  /* Setup the ECC configuration params */

  U8 num_regs     = 3;
  U16 chunk_size  = 512;
  U8 parity_pairs = 12;

  while (data_idx < read_size)
  {
    U32 original_ecc = 0;
    U32 new_ecc      = 0;
    U32 xor          = 0;

    /* Put it back the original GPMC way.. */

    original_ecc = ROM_CODE_ECC_2_GPMC_ECC(spare);
    new_ecc      = ROM_CODE_ECC_2_GPMC_ECC(ecc);
    xor          = new_ecc ^ original_ecc;

    if (xor)
    {
      U8 num_bits = drv_check_num_bits(xor);

      /* See if we can recover or not.. */

      if (num_bits == 1)
      {
        /* Stored ECC got a bit error.. ignore is right?? */

        ERROR("ECC ERROR (IGNORED)");
      }
      else if (num_bits == parity_pairs)
      {
        /* Correct the data here - 1 bit correction */

        U16 p_bits      = xor >> 16;
        U8 bit_location = (p_bits & 0x7);

        /* bytelocations are from 0 to 512 need U16 to store */

        U16 byte_location = (p_bits >> 3) & (chunk_size - 1);
        U8 *error_data    = (U8 *) (data_buffer + data_idx + byte_location);
        U8 byte;

        if (bit_location > 8)
        {
          ERROR("ECC BAD BIT LOCATION");
        }
        else if (byte_location > chunk_size)
        {
          ERROR("ECC BAD BYTE LOCATION");
        }

        /* Flip the bit.. */

        byte        = *error_data;
        byte       ^= (0x1 << bit_location);
        *error_data = byte;
      }
      else
      {
        /* Check Sum failed!!! */
        ERROR("ECC CHECK SUM FAILURE");
        return FLASH_DRV_ERROR;
      }
    }
    ecc      += num_regs;
    spare    += num_regs;
    data_idx += chunk_size;
  }
  return FLASH_DRV_SUCCESS;
}
#endif


