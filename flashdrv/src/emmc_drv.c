/**
* @file emmc_drv.c
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
*/


/*==== INCLUDES ==============================================================*/

#include <stdio.h>
#include <string.h>
#include "types.h"
#include "error.h"

#include "platform.h"
#include "csst_tgt.h"

#include "dbg_ex.h"
#include "flash_drv.h" 
#include "sparse_format.h"
#include "emmc_drv.h"

/*==== CONFIGURATION OPTIONS =================================================*/

/*==== PRIVATE MACROS ================================================================*/

//#define SPIN { volatile int i = 1; while(i); }

#define LOG_ERRORS

//@todo for other drivers depercated void drv_send_info(const char *format, ...);
#define SEND(FORMAT)                  { drv_send_info("EMMC "FORMAT); }
//#define SEND1(FORMAT, P1)             { drv_send_info("EMMC "FORMAT, P1); }
//#define SEND2(FORMAT, P1, P2)         { drv_send_info("EMMC "FORMAT, P1, P2); }
//#define SEND3(FORMAT, P1, P2, P3)     { drv_send_info("EMMC "FORMAT, P1, P2, P3); }
//#define SEND4(FORMAT, P1, P2, P3, P4) { drv_send_info("EMMC "FORMAT, P1, P2, P3, P4); }

#define DEFAULT_TIMEOUT                 512         //msec
#define DEFAULT_TIMEOUT_TICK            1			      //msec
#define DEFAULT_INIT_TIMEOUT            5000        //msec
#define DEFAULT_DDR_DISABLED            0           
#define DEFAULT_PARTITION               USER_DATA_AREA

#define SWICTH_DELAY                    100000

/*==== PRIVATE DATA STRUCTURES ===============================================*/

U32 emmc_drv_dnld_init(T_driver_config * config, char ** result);
U32 emmc_drv_dnld_erase(T_driver_config * config, char ** result, U64 address, U64 length);
U32 emmc_drv_dnld_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more);
U32 emmc_drv_dnld_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
U32 emmc_drv_dnld_deinit(T_driver_config * config, char ** result);
U32 emmc_drv_dnld_get_info(T_driver_config * config, T_driver_info * info);

//static S32 mmc_dip_switch_setting_validate(U32 sdu_handle, U8 mmc_slot_number);

U32 mmc_read(T_db * db, U32 start_block, U32 block_count, void *destination);

//static void * drv_memcpy(void *dest, const void *src, U32 n);
//static void * drv_memset(void *s, U8 c, U32 n);

#ifndef SIMULATION
#pragma DATA_SECTION(driver_if,".consttbl");
#endif
const T_driver_if driver_if = 
{
  {
    OMAPFLASH_DRIVER_HEADER_STRING_V7,
      "eMMC JESD84-A43", //JEDEC MMCA v4.3
      CTRL_Z
  },
  {
    &driver_if,
      emmc_drv_dnld_init,
      emmc_drv_dnld_read,
      emmc_drv_dnld_write,
      emmc_drv_dnld_erase,
      emmc_drv_dnld_deinit,
      emmc_drv_dnld_get_info
    }
};

typedef enum
{
  C_SID,
  C_WIDTH,
  C_DELAY,
  C_RPAPI_BASE,
  C_DEVINFO,
  C_MBLOCK,
  C_HD,
  C_MAXCLK,
  C_TO,
  C_TOT,
  C_INITTO,
  C_C01D,
  C_DDR,
  C_PT,
  C_BOOT,
  C_RST_N,
  C_SPARSE
} T_driver_setup_index;

const T_driver_setup_const setup_const[] =
{
  { "sid",          DEFAULT, TRUE  },
  { "width",        DEFAULT, TRUE  },
  { "delay",        DEFAULT, TRUE  },
  { "rpapi_base",   DEFAULT, TRUE  },
  { "devinfo",      DEFAULT, FALSE },
  { "mblock",       DEFAULT, FALSE },
  { "hd",           DEFAULT, FALSE },
  { "maxclk",       DEFAULT, FALSE },
  { "to",			      DEFAULT, FALSE },
  { "tot",			    DEFAULT, FALSE },
  { "initto",		    DEFAULT, FALSE },
  { "c01d",         DEFAULT, FALSE },
  { "ddr",  		    DEFAULT, FALSE },
  { "pt",           DEFAULT, FALSE },
  { "boot",         DEFAULT, FALSE },
  { "rst_n",        DEFAULT, FALSE },
  { "sparse",       DEFAULT, FALSE },
  { "",             DEFAULT, FALSE }
};

T_driver_setup_var setup_var[sizeof(setup_const) / sizeof(T_driver_setup_const)];

/*==== DOWNLOAD DRIVER FUNCTION IMPLEMENTATION ===============================*/

#define BIT_FIELD_INFO(comment, name, bitwidth, type, additional_fct) {comment, #name, bitwidth, type, additional_fct}
#define BYTE_FIELD_INFO(comment, name, bitwidth, type, additional_fct) {comment, #name, bitwidth, type, additional_fct}

typedef void (* T_additional_field_info)(U64 field);

typedef struct T_field_info 
{
  const char *            comment;
  const char *            name;
  U16                     width;
  U16                     type;
  T_additional_field_info additional_info;
} T_field_info;

enum T_type {R=1,W=2,E=4};

#define AIS_INDENT 9 // 38

/*------------------------------------------------------------------------------
| Function    : ai_csd_pnm
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_csd_pnm(U64 field)
{
  int i;
  char ais[7];
  for(i = 0; i < 6; i++)
  {
    ais[i] = field & 0xFF;
    field >>= 8;
  }
  ais[6] = 0;
  dbg_printf("%*s%s", AIS_INDENT,"", ais);
}

/*------------------------------------------------------------------------------
| Function    : ai_csd_mdt
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_csd_mdt(U64 field)
{
  int month = (field >> 4) & 0x0F;
  int year  = (field & 0x0F) + 1997;
  dbg_printf("%*s%02d-%04d", AIS_INDENT, "", month, year);
}

///From JESD84-A43.pdf Table 32 page 79-80
const T_field_info cid_info[] = {
  BIT_FIELD_INFO("Manufacturer ID",                   MID,                  8, R, NULL),            // [127:120]
  BIT_FIELD_INFO("Reserved",                          RESERVED0,            6, R, NULL),            // [119:114]
  BIT_FIELD_INFO("Card/BGA",                          CBX,                  2, R, NULL),            // [113:112]
  BIT_FIELD_INFO("OEM/Application ID",                OID,                  8, R, NULL),            // [111:104]
  BIT_FIELD_INFO("Product name",                      PNM,                 48, R, ai_csd_pnm),      // [103:56]
  BIT_FIELD_INFO("Product revision",                  PRV,                  8, R, NULL),            // [55:48]
  BIT_FIELD_INFO("Product serial number",             PSN,                 32, R, NULL),            // [47:16]
  BIT_FIELD_INFO("Manufacturing date",                MDT,                  8, R, ai_csd_mdt),      // [15:8]
  BIT_FIELD_INFO("CRC7 checksum",                     CRC,                  7, R, NULL),            // [7:1]
  BIT_FIELD_INFO("not used, always '1'",              RESERVED1,            1, 0, NULL),            // [0:0]
  {0,0,0,0,0}
};

///From JESD84-A43.pdf Table 34 page 79-80
const T_field_info csd_info[] = {
  BIT_FIELD_INFO("CSD structure",                     CSD_STRUCTURE,        2, R, NULL),      // [127:126]
  BIT_FIELD_INFO("System specification version",      SPEC_VERS,            4, R, NULL),      // [125:122]
  BIT_FIELD_INFO("Reserved",                          RESERVED2,            2, R, NULL),      // [121:120]
  BIT_FIELD_INFO("Data read access-time 1",           TAAC,                 8, R, NULL),      // [119:112]
  BIT_FIELD_INFO("Data read access-time 2 in CLK cycles (NSAC*100)", NSAC,  8, R, NULL),      // [111:104]
  BIT_FIELD_INFO("Max. bus clock frequency",          TRAN_SPEED,           8, R, NULL),      // [103:96]
  BIT_FIELD_INFO("Card command classes",              CCC,                 12, R, NULL),      // [95:84]
  BIT_FIELD_INFO("Max. read data block length",       READ_BL_LEN,          4, R, NULL),      // [83:80]
  BIT_FIELD_INFO("Partial blocks for read allowed",   READ_BL_PARTIAL,      1, R, NULL),      // [79:79]
  BIT_FIELD_INFO("Write block misalignment",          WRITE_BLK_MISALIGN,   1, R, NULL),      // [78:78]
  BIT_FIELD_INFO("Read block misalignment",           READ_BLK_MISALIGN,    1, R, NULL),      // [77:77]
  BIT_FIELD_INFO("DSR implemented",                   DSR_IMP,              1, R, NULL),      // [76:76]
  BIT_FIELD_INFO("Reserved",                          RESERVED,             2, R, NULL),      // [75:74]
  BIT_FIELD_INFO("Device size",                       C_SIZE,              12, R, NULL),      // [73:62]
  BIT_FIELD_INFO("Max. read current @ VDD min",       VDD_R_CURR_MIN,       3, R, NULL),      // [61:59]
  BIT_FIELD_INFO("Max. read current @ VDD max",       VDD_R_CURR_MAX,       3, R, NULL),      // [58:56]
  BIT_FIELD_INFO("Max. write current @ VDD min",      VDD_W_CURR_MIN,       3, R, NULL),      // [55:53]
  BIT_FIELD_INFO("Max. write current @ VDD max",      VDD_W_CURR_MAX,       3, R, NULL),      // [52:50]
  BIT_FIELD_INFO("Device size multiplier",            C_SIZE_MULT,          3, R, NULL),      // [49:47]
  BIT_FIELD_INFO("Erase group size",                  ERASE_GRP_SIZE,       5, R, NULL),      // [46:42]
  BIT_FIELD_INFO("Erase group size multiplier",       ERASE_GRP_MULT,       5, R, NULL),      // [41:37]
  BIT_FIELD_INFO("Write protect group size",          WP_GRP_SIZE,          5, R, NULL),      // [36:32]
  BIT_FIELD_INFO("Write protect group enable",        WP_GRP_ENABLE,        1, R, NULL),      // [31:31]
  BIT_FIELD_INFO("Manufacturer default ECC",          DEFAULT_ECC,          2, R, NULL),      // [30:29]
  BIT_FIELD_INFO("Write speed factor",                R2W_FACTOR,           3, R, NULL),      // [28:26]
  BIT_FIELD_INFO("Max. write data block length",      WRITE_BL_LEN,         4, R, NULL),      // [25:22]
  BIT_FIELD_INFO("Partial blocks for write allowed",  WRITE_BL_PARTIAL,     1, R, NULL),      // [21:21]
  BIT_FIELD_INFO("Reserved",                          RESERVED3,            4, R, NULL),      // [20:17]
  BIT_FIELD_INFO("Content protection application",    CONTENT_PROT_APP,     1, R, NULL),      // [16:16]
  BIT_FIELD_INFO("File format group",                 FILE_FORMAT_GRP,      1, R|W, NULL),    // [15:15]
  BIT_FIELD_INFO("Copy flag (OTP)",                   COPY,                 1, R|W, NULL),    // [14:14]
  BIT_FIELD_INFO("Permanent write protection",        PERM_WRITE_PROTECT,   1, R|W, NULL),    // [13:13]
  BIT_FIELD_INFO("Temporary write protection",        TMP_WRITE_PROTECT,    1, R|W|E, NULL),  // [12:12]
  BIT_FIELD_INFO("File format",                       FILE_FORMAT,          2, R|W, NULL),    // [11:10]
  BIT_FIELD_INFO("ECC code",                          ECC,                  2, R|W|E, NULL),  // [9:8]
  BIT_FIELD_INFO("CRC",                               CRC,                  7, R|W|E, NULL),  // [7:1]
  BIT_FIELD_INFO("Not used, always '1'",              RESERVED4,            1, 0, NULL),      // [0:0]
  {0,0,0,0,0}
};

/*------------------------------------------------------------------------------
| Function    : ai_ext_boot_size_mult
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_ext_boot_size_mult(U64 field)
{
  int bp_size = (field & 0xFF) * 128;
  dbg_printf("%*s%d kB", AIS_INDENT, "", bp_size);
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_sec_count
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_ext_sec_count(U64 field)
{
  U32 mb;
  field *= 512;
  mb = (U32)(field / (1024 * 1024));
  dbg_printf("%*s%d MB", AIS_INDENT, "", mb);
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_min_perf
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_ext_min_perf(U64 field)
{
  int val_sdr = (field & 0xFF) << 1;
  int val_ddr = val_sdr;
  val_sdr *= 150;
  val_ddr *= 300;
  if(val_sdr)
  {
    dbg_printf("%*s%d kB/s SDR mode\n\r%*s%d kB/s DDR mode", AIS_INDENT, "", val_sdr, AIS_INDENT, "", val_ddr);
  }
  else
  {
    dbg_printf("%*s<2400 kB/s SDR mode\n\r%*s<4800 kB/s DDR mode", AIS_INDENT, "", AIS_INDENT, "");
  }
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_timeout
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_ext_timeout(U64 field)
{
  int val = (field & 0xFF) * 10;
  dbg_printf("%*s%d ms", AIS_INDENT, "", val);
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_card_type
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_ext_card_type(U64 field)
{
  int val = (field & 0xFF);

  if(val & 0x01)
  {
    dbg_printf("%*sHS MMC @ 26 MHz", AIS_INDENT, "");
  }

  if(val & 0x02)
  {
    dbg_printf("%*sHS MMC @ 52 MHz", AIS_INDENT, "");
  }

  if(val & 0x04)
  {
    dbg_printf("%*sHS DDR MMC @ 52 MHz 1.8/3V", AIS_INDENT, "");
  }

  if(val & 0x08)
  {
    dbg_printf("%*sHS DDR MMC @ 52 MHz 1.2V", AIS_INDENT, "");
  }
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_partition_config
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/

const char * const access[]    = { "NONE", "BP1", "BP2", "RPMB", "GPP1", "GPP2", "GPP3", "GPP4" };
const char * const boot_part[] = { "NONE", "BP1", "BP2", "UDA" };
const char * const boot_ack[]  = { "DISABLED", "ENABLED" };

void ai_ext_partition_config(U64 field)
{
  char ** p;
  U8 val = field & 0xFF;
  U8 acc = PARTITION_CONFIG__PARTITION_ACCESS(val);
  U8 ben = PARTITION_CONFIG__BOOT_PARTITION(val) >> 3;
  U8 ack = PARTITION_CONFIG__BOOT_ACK(val) >> 6;
  p = (char **)RELOCATED(access);
  dbg_printf("%*sACCESS: %s", AIS_INDENT, "", RELOCATED(p[acc]));
  p = (char **)RELOCATED(boot_part);
  dbg_printf("%*sBOOT  : %s", AIS_INDENT, "", RELOCATED(p[ben]));
  p = (char **)RELOCATED(boot_ack);
  dbg_printf("%*sACK   : %s", AIS_INDENT, "", RELOCATED(p[ack]));
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_rst_n_function
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/

const char * const rst_n_enable[] = { "TEMPORARILY DISABLED", "PERMANENTLY ENABLED", "PERMANENTLY DISABLED", "RESERVED" };

void ai_ext_rst_n_function(U64 field)
{
  U8 value = field & 0x03;
  char ** p = (char **)RELOCATED(rst_n_enable);
  dbg_printf("%*s%s", AIS_INDENT, "", RELOCATED(p[value]));
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_boot_wp
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
const char * const b_wp_en[] = { "OFF", "ON" };
const char * const b_wp_dis[] = { "ENABLED", "DISABLED" };

void ai_ext_boot_wp(U64 field)
{
  char indent = 0;
  U8 value = field & 0xFF;
  char ** p_en = (char **)RELOCATED(b_wp_en);
  char ** p_dis = (char **)RELOCATED(b_wp_dis);
  dbg_printf("%*sPower-On Write Protect : %s", AIS_INDENT, &indent, RELOCATED(p_en[value & 0x01]));
  dbg_printf("%*sPermanent Write Protect: %s", AIS_INDENT, &indent, RELOCATED(p_en[(value >> 2) & 0x01]));
  dbg_printf("%*sPower-On Write Protect : %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 4) & 0x01]));
  dbg_printf("%*sPermanent Write Protect: %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 6) & 0x01]));
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_user_wp
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void ai_ext_user_wp(U64 field)
{
  char indent = 0;
  U8 value = field & 0xFF;
  char ** p_en = (char **)RELOCATED(b_wp_en);
  char ** p_dis = (char **)RELOCATED(b_wp_dis);
  dbg_printf("%*sPower-On Write Protect : %s", AIS_INDENT, &indent, RELOCATED(p_en[value & 0x01]));
  dbg_printf("%*sPermanent Write Protect: %s", AIS_INDENT, &indent, RELOCATED(p_en[(value >> 2) & 0x01]));
  dbg_printf("%*sPower-On Write Protect : %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 3) & 0x01]));
  dbg_printf("%*sPermanent Write Protect: %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 4) & 0x01]));
  dbg_printf("%*sCSD Perm Write Protect : %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 6) & 0x01]));
  dbg_printf("%*sPassword Protection    : %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 7) & 0x01]));
}

/*------------------------------------------------------------------------------
| Function    : ai_ext_boot_bus_width
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/

const char * const boot_mode[] = { "SDR + Compatible", "SDR + High Speed", "DDR", "Reserved" };

void ai_ext_boot_bus_width(U64 field)
{
  char indent = 0;
  U8 value = field & 0xFF;
  char ** p_dis = (char **)RELOCATED(b_wp_dis);
  dbg_printf("%*sBOOT BUS WIDTH      : x%d", AIS_INDENT, &indent, 1 + (value & 0x01) * 3 + ((value & 0x02) >> 1) * 7);
  dbg_printf("%*sRESET BOOT BUS WIDTH: %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 2) & 0x01]));
  dbg_printf("%*sBOOT MODE           : %s", AIS_INDENT, &indent, RELOCATED(p_dis[(value >> 3) & 0x03]));
}

///From JESD84-A43.pdf Table 34 page 79-80
const T_field_info ext_csd_info[] = {
  //Properties Segment
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   7,   0, NULL), // [511:505]
  BYTE_FIELD_INFO("Supported Command Sets",                                         S_CMD_SET,                  1,   R, NULL), // [504]
  BYTE_FIELD_INFO("HPI features",                                                   HPI_FEATURES,               1,   R, NULL), // [503]
  BYTE_FIELD_INFO("Background operations support",                                  BKOPS_SUPPORT,              1,   R, NULL), // [502]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                 255,   0, NULL), // [501:247]
  BYTE_FIELD_INFO("Background operations status",                                   BKOPS_STATUS,               1,   R, NULL), // [246]
  BYTE_FIELD_INFO("Number of correctly programmed sectors",                         CRRCTLY_PRG_SCTRS_NM,       4,   R, NULL), // [245:242]
  BYTE_FIELD_INFO("1st initialization time after partitioning",                     INI_TIMEOUT_AP,             1,   R, NULL), // [241]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [240]
  BYTE_FIELD_INFO("Power class for 52MHz, DDR at 3.6V",                             PWR_CL_DDR_52_360,          1,   R, NULL), // [239]
  BYTE_FIELD_INFO("Power class for 52MHz, DDR at 1.95V",                            PWR_CL_DDR_52_195,          1,   R, NULL), // [238]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   2,   0, NULL), // [237:236]
  BYTE_FIELD_INFO("Minimum Write Performance for 8bit at 52MHz in DDR mode",        MIN_PERF_DDR_W_8_52,        1,   R, NULL), // [235]
  BYTE_FIELD_INFO("Minimum Read Performance for 8bit at  52MHz in DDR mode",        MIN_PERF_DDR_R_8_52,        1,   R, NULL), // [234]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [233]
  BYTE_FIELD_INFO("TRIM Multiplier",                                                TRIM_MULT,                  1,   R, NULL), // [232]
  BYTE_FIELD_INFO("Secure Feature support",                                         SEC_FEATURE_SUPPORT,        1,   R, NULL), // [231]
  BYTE_FIELD_INFO("Secure Erase Multiplier",                                        SEC_ERASE_MULT,             1,   R, NULL), // [230]
  BYTE_FIELD_INFO("Secure TRIM Multiplier",                                         SEC_TRIM_MULT,              1,   R, NULL), // [229]
  BYTE_FIELD_INFO("Boot information",                                               BOOT_INFO,                  1,   R, NULL), // [228]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [227]
  BYTE_FIELD_INFO("Boot partition size",                                            BOOT_SIZE_MULTI,            1,   R, ai_ext_boot_size_mult), // [226]
  BYTE_FIELD_INFO("Access size",                                                    ACC_SIZE,                   1,   R, NULL), // [225]
  BYTE_FIELD_INFO("High-capacity erase unit size",                                  HC_ERASE_GRP_SIZE,          1,   R, NULL), // [224]
  BYTE_FIELD_INFO("High-capacity erase timeout",                                    ERASE_TIMEOUT_MULT,         1,   R, NULL), // [223]
  BYTE_FIELD_INFO("Reliable write sector count",                                    REL_WR_SEC_C,               1,   R, NULL), // [222]
  BYTE_FIELD_INFO("High-capacity write protect group size",                         HC_WP_GRP_SIZE,             1,   R, NULL), // [221]
  BYTE_FIELD_INFO("Sleep current (VCC)",                                            S_C_VCC,                    1,   R, NULL), // [220]
  BYTE_FIELD_INFO("Sleep current (VCCQ)",                                           S_C_VCCQ,                   1,   R, NULL), // [219]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [218]
  BYTE_FIELD_INFO("Sleep/awake timeout",                                            S_A_TIMEOUT,                1,   R, NULL), // [217]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [216]
  BYTE_FIELD_INFO("Sector Count",                                                   SEC_COUNT,                  4,   R, ai_ext_sec_count), // [215:212]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [211]
  BYTE_FIELD_INFO("Minimum Write Performance for 8bit at 52MHz",                    MIN_PERF_W_8_52,            1,   R, ai_ext_min_perf), // [210]
  BYTE_FIELD_INFO("Minimum Read Performance for 8bit at 52MHz",                     MIN_PERF_R_8_52,            1,   R, ai_ext_min_perf), // [209]
  BYTE_FIELD_INFO("Minimum Write Performance for 8bit at 26MHz, for 4bit at 52MHz", MIN_PERF_W_8_26_4_52,       1,   R, ai_ext_min_perf), // [208]
  BYTE_FIELD_INFO("Minimum Read Performance for 8bit at 26MHz, for 4bit at 52MHz",  MIN_PERF_R_8_26_4_52,       1,   R, ai_ext_min_perf), // [207]
  BYTE_FIELD_INFO("Minimum Write Performance for 4bit at 26MHz",                    MIN_PERF_W_4_26,            1,   R, ai_ext_min_perf), // [206]
  BYTE_FIELD_INFO("Minimum Read Performance for 4bit at 26MHz",                     MIN_PERF_R_4_26,            1,   R, ai_ext_min_perf), // [205]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [204]
  BYTE_FIELD_INFO("Power class for 26MHz at 3.6V",                                  PWR_CL_26_360,              1,   R, NULL), // [203]
  BYTE_FIELD_INFO("Power class for 52MHz at 3.6V",                                  PWR_CL_52_360,              1,   R, NULL), // [202]
  BYTE_FIELD_INFO("Power class for 26MHz at 1.95V",                                 PWR_CL_26_195,              1,   R, NULL), // [201]
  BYTE_FIELD_INFO("Power class for 52MHz at 1.95V",                                 PWR_CL_52_195,              1,   R, NULL), // [200]
  BYTE_FIELD_INFO("Partition switching timing",                                     PARTITION_SWCH_TIME,        1,   R, ai_ext_timeout), // [199]
  BYTE_FIELD_INFO("Out-of-interrupt busy timing",                                   OUT_OF_INTRPT_TIME,         1,   R, ai_ext_timeout), // [198]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [197]
  BYTE_FIELD_INFO("Card type",                                                      CARD_TYPE,                  1,   R, ai_ext_card_type), // [196]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [195]
  BYTE_FIELD_INFO("CSD structure version",                                          CSD_STRUCTURE,              1,   R, NULL), // [194]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [193]
  BYTE_FIELD_INFO("Extended CSD revision",                                          EXT_CSD_REV,                1,   R, NULL), // [192]
  //Modes Segment
  BYTE_FIELD_INFO("Command set",                                                    CMD_SET,                    1, R|W, NULL), // [191]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [190]
  BYTE_FIELD_INFO("Command set revision",                                           CMD_SET_REV,                1,   R, NULL), // [189]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [188]
  BYTE_FIELD_INFO("Power class",                                                    POWER_CLASS,                1, R|W, NULL), // [187]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [186]
  BYTE_FIELD_INFO("High-speed interface timing",                                    HS_TIMING,                  1, R|W, NULL), // [185]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [184]
  BYTE_FIELD_INFO("Bus width mode",                                                 BUS_WIDTH,                  1,   W, NULL), // [183]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [182]
  BYTE_FIELD_INFO("Erased memory content",                                          ERASED_MEM_CONT,            1,   R, NULL), // [181]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [180]
  BYTE_FIELD_INFO("Partition configuration",                                        PARTITION_CONFIG,           1, R|W, ai_ext_partition_config), // [179]
  BYTE_FIELD_INFO("Boot config protection",                                         BOOT_CONFIG_PROT,           1, R|W, NULL), // [178]
  BYTE_FIELD_INFO("Boot bus width1",                                                BOOT_BUS_WIDTH,             1, R|W, ai_ext_boot_bus_width), // [177]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [176]
  BYTE_FIELD_INFO("High-density erase group definition",                            ERASE_GROUP_DEF,            1, R|W, NULL), // [175]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [174]
  BYTE_FIELD_INFO("Boot area write protection register",                            BOOT_WP,                    1, R|W, ai_ext_boot_wp), // [173]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [172]
  BYTE_FIELD_INFO("User area write protection register",                            USER_WP,                    1, R|W, ai_ext_user_wp), // [171]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [170]
  BYTE_FIELD_INFO("FW configuration",                                               FW_CONFIG,                  1, R|W, NULL), // [169]
  BYTE_FIELD_INFO("RPMB Size",                                                      RPMB_SIZE_MULT,             1,   R, NULL), // [168]
  BYTE_FIELD_INFO("Write reliability setting register",                             WR_REL_SET,                 1, R|W, NULL), // [167]
  BYTE_FIELD_INFO("Write reliability parameter register",                           WR_REL_PARAM,               1,   R, NULL), // [166]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [165]
  BYTE_FIELD_INFO("Manually start background operations",                           BKOPS_START,                1,   W, NULL), // [164]
  BYTE_FIELD_INFO("Enable background operations handshake",                         BKOPS_EN,                   1, R|W, NULL), // [163]
  BYTE_FIELD_INFO("H/W reset function",                                             RST_n_FUNCTION,             1, R|W, ai_ext_rst_n_function), // [162]
  BYTE_FIELD_INFO("HPI management",                                                 HPI_MGMT,                   1, R|W, NULL), // [161]
  BYTE_FIELD_INFO("Partitioning Support",                                           PARTITIONING_SUPPORT,       1,   R, NULL), // [160]
  BYTE_FIELD_INFO("Max Enhanced Area Size",                                         MAX_ENH_SIZE_MULT,          3,   R, NULL), // [159:157]
  BYTE_FIELD_INFO("Partitions attribute",                                           PARTITIONS_ATTRIBUTE,       1, R|W, NULL), // [156]
  BYTE_FIELD_INFO("Paritioning Setting",                                            PARTITION_SETNG_CMPLTD,     1, R|W, NULL), // [155]
  BYTE_FIELD_INFO("General Purpose Partition Size",                                 GP_SIZE_MULT,              12, R|W, NULL), // [154:143]
  BYTE_FIELD_INFO("Enhanced User Data Area Size",                                   ENH_SIZE_MULT,              3, R|W, NULL), // [142:140]
  BYTE_FIELD_INFO("Enhanced User Data Start Address",                               ENH_START_ADDR,             4, R|W, NULL), // [139:136]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                   1,   0, NULL), // [135]
  BYTE_FIELD_INFO("Bad Block Management mode",                                      SEC_BAD_BLK_MGMNT,          1, R|W, NULL), // [134]
  BYTE_FIELD_INFO("Reserved",                                                       RESERVED,                 134,   0, NULL), // [133:0]
  {0,0,0,0,0}
};

/*------------------------------------------------------------------------------
| Function    : get_bits
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U64 get_bits(U8 * source, U16 offset, U16 width)
{
  U64 value = 0;
  U8 startbyte = offset >> 3;
  U8 startbit  = offset - (startbyte << 3);

  while(width)
  {
    U8 bytebits = width > (8 - startbit) ? 8 - startbit : width;
    U64 v   = source[startbyte++];
    v      &= (0xFF >> startbit);
    v     >>= (8 - startbit - bytebits);
    v     <<= width > bytebits ? (width - (8 - startbit)) : 0;
    value  |= v;
    width  -= bytebits;
    startbit = 0;
  }

  return value;
}

/*------------------------------------------------------------------------------
| Function    : send_field
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void send_field(const char *name, const T_field_info *info, U32 width, U64 value)
{
  char es = 0;
  width *= 2; //2 digits per byte
  dbg_printf("eMMC %s %-@28s %#0*llX %*s// %s",name,RELOCATED(info->name),width,value,16-width,&es,RELOCATED(info->comment));
  if(info->additional_info)
  {
    ((T_additional_field_info)RELOCATED(info->additional_info))(value);
  }
}

#if 0
/*------------------------------------------------------------------------------
| Function    : stringize
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
char * stringize(U8 * in)
{
  static char out[17];
  int count;
  for(count = 0; count < 16; count++)
  {
    out[count] = ((in[count] > 31) && (in[count] < 127)) ? in[count] : '.';
  }
  out[16] = 0;
  return out;
}

/*------------------------------------------------------------------------------
| Function    : memorydump
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
void memorydump(U32 start, U32 end)
{
  if(start)
  {
    do
    {
      dbg_printf("%#08X: %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X   %s",
                start, 
                *(U8 *)(start), *(U8 *)(start + 1), *(U8 *)(start + 2), *(U8 *)(start + 3), *(U8 *)(start + 4), *(U8 *)(start + 5), *(U8 *)(start + 6), *(U8 *)(start + 7),
                *(U8 *)(start + 8), *(U8 *)(start + 9), *(U8 *)(start + 10), *(U8 *)(start + 11), *(U8 *)(start + 12), *(U8 *)(start + 13), *(U8 *)(start + 14), *(U8 *)(start + 15),
                stringize((U8 *)start));
      start = start + 16;
    } while(start < end);
  }
}
#endif

/*------------------------------------------------------------------------------
| Function    : send_bit_field
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 send_bit_field(U8 * regs, const char *name, const T_field_info *info)
{
  const T_field_info *p;
  U16 offset = 128;
  
  for (p = (const T_field_info *)RELOCATED(info); p->width; p++)
  {
    offset -= p->width;
    
    if (strncmp((const char *)RELOCATED(p->name), "RESERVED", 8)) 
    {
      U64 value = get_bits(regs, offset, p->width);
      send_field(name, p, (p->width+7)/8, value);
    }

  }
  return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : send_byte_field
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 send_byte_field(U8 * data, const char *name, const T_field_info *info)
{
  const T_field_info *p;
  U32 offset  = 512;

  for (p = (const T_field_info *)RELOCATED(info); p->width; p++)
  {
    if (offset < p->width)
    {
        return OMAPFLASH_DAL_ERROR;
    }
    else
    {
      offset  -= p->width;
      if (strncmp((const char *)RELOCATED(p->name), "RESERVED", 8)) 
      {
        int i;
        U64 value = 0;

        if(p->width <= 8)
        {
          for (i = 0; i < p->width; i++) 
          {
            value = value << 8 | data[offset + p->width - i - 1];
          }
          send_field(name, p, p->width, value);
        }
      }
    }
  }
  if (offset > 0)
      return OMAPFLASH_DAL_ERROR;
  else
      return OMAPFLASH_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : get_mmc_info
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 get_mmc_info(T_db * db)
{
  U32 ret_value;
  U16 regs[8];				/* registers array */

  ret_value = get_sd_mmc_cid_ex(db->local_dis.sid, db->local_dis.card_rca, db->local_dis.card_type, regs);
  if (ret_value != OMAPFLASH_SUCCESS) 
    return ret_value;
  if(db->devinfo)
  {
    ret_value = send_bit_field((U8 *)regs, "CID", cid_info);
  }

  ret_value = get_sd_mmc_csd_ex(db->local_dis.sid, db->local_dis.card_rca, db->local_dis.card_type, regs);
  if (ret_value != OMAPFLASH_SUCCESS) 
    return ret_value;
  if(db->devinfo)
  {
    ret_value = send_bit_field((U8 *)regs, "CSD", csd_info);
  }

  db->block_id = NO_MMC_BLOCK_ID;
    ret_value = mmc_read_ext_csd((U8)db->local_dis.sid, db->local_dis.data_width, db->local_dis.card_rca, db->local_dis.card_type, (U8 *)db->ext_csd);
  if (ret_value != OMAPFLASH_SUCCESS) 
    return ret_value;
  if(db->devinfo)
  {
    ret_value = send_byte_field((U8 *)db->ext_csd, "EXT", ext_csd_info);
  }

  return OMAPFLASH_SUCCESS;
}

U32 get_rpapi_base(void)
{
  T_driver_config * config = get_config();
  T_db * db = (T_db *)config->data;
  return db->rpapi_base;
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
U32 emmc_drv_dnld_init(T_driver_config * config, char ** result)
{
  T_driver_setup_var *setup = get_setup_var();  

  U32 ret_val = FLASH_DRV_SUCCESS;
  T_MMC_DIS *mmc_dis;
  T_db * db;

  *result = NULL;

  /* Parse configuration string */

  if(drv_parse_config(get_setup_const(), setup, config->cstring, config) == FLASH_DRV_ERROR)
  {
    *result = (char *)RELOCATED("UNABLE TO FIND CONFIGURATION PARAMETER DURING INITIALIZATION");
    return FLASH_DRV_ERROR;
  }

  /* Update callback functions pointers and check that the necessary functions are there */

  if(!config->drv_malloc)
  {
    *result = (char *)RELOCATED("2ND DID NOT PROVIDE MALLOC");
    return FLASH_DRV_ERROR;
  }

  if(!config->drv_free)
  {
    *result = (char *)RELOCATED("2ND DID NOT PROVIDE FREE");
    return FLASH_DRV_ERROR;
  }

  set_config(config);

  config->data = drv_malloc(sizeof(T_db));

  db = (T_db *)(config->data);

  if(!db)
  {
    *result = (char *)RELOCATED("DRIVER DATA MEMORY ALLOCATION ERROR DURING INITIALIZATION");
    return FLASH_DRV_ERROR;
  }

  db->rpapi_base    = setup[C_RPAPI_BASE].value;
  db->delay         = setup[C_DELAY].value;
  db->devinfo       = setup[C_DEVINFO].valid ? setup[C_DEVINFO].value : 0;
  db->mblock        = setup[C_MBLOCK].valid ? setup[C_MBLOCK].value : 1;
  db->hd            = setup[C_HD].valid ? setup[C_HD].value : 1;
  db->maxclk        = setup[C_MAXCLK].valid ? setup[C_MAXCLK].value : 0;
  db->timeout       = setup[C_TO].valid ? setup[C_TO].value : DEFAULT_TIMEOUT;
  db->timeout_tick  = setup[C_TOT].valid ? setup[C_TOT].value : DEFAULT_TIMEOUT_TICK;
  db->init_timeout  = setup[C_INITTO].valid ? setup[C_INITTO].value : DEFAULT_INIT_TIMEOUT;
  db->cmd01delay    = setup[C_C01D].valid ? setup[C_C01D].value : 0;
  db->ddr           = setup[C_DDR].valid ? setup[C_DDR].value : DEFAULT_DDR_DISABLED;
  db->partition     = (T_emmc_partition)(setup[C_PT].valid ? setup[C_PT].value : DEFAULT_PARTITION);
  db->make_bootable = (setup[C_BOOT].valid)? (BOOLEAN)setup[C_BOOT].value : FALSE;
  db->rst_n         = (setup[C_RST_N].valid)? setup[C_RST_N].value : 0;
  db->sparse.state  = setup[C_SPARSE].valid? setup[C_SPARSE].value? sparse_check_needed : sparse_disabled : sparse_disabled;

  db->ext_csd       = (T_ext_csd *)drv_malloc(sizeof(T_ext_csd));

  if(!db->ext_csd)
  {
    *result = (char *)RELOCATED("EXT CSD MEMORY ALLOCATION ERROR DURING INITIALIZATION");
    return FLASH_DRV_ERROR;
  }

  mmc_dis      = &db->local_dis;
  mmc_dis->sid = (U16)setup[C_SID].value + 1; // OMAPFlash SID starts from 0. Add 1 to match MMC ID from TRM.

#ifdef OMAP3
  mmc_dis->mmc_volt = MMC_VOLT_3_0V;
#endif
#if defined OMAP4 || defined OMAP5
  mmc_dis->mmc_volt = MMC_VOLT_1_8V;
#endif
  mmc_dis->data_width = (U8)setup[C_WIDTH].value; // MMC_GET(MMC_WIDTH,config->device_specific_configuration); //@todo see CSST MMC_MAX_DATA_WIDTH for determin data bus width

  /* Allocate buffer for page data */
  db->block_id           = NO_MMC_BLOCK_ID;
  db->block_data         = (U8 *)drv_malloc(MMC_BLOCK_SIZE * 2); //@todo refactor reader to eliminate *2 here
  db->block_id_partial   = NO_MMC_BLOCK_ID;
  db->block_data_partial = (U8 *)drv_malloc(MMC_BLOCK_SIZE * 2); //@todo refactor reader to eliminate *2 here
  
  if(!db->block_data)
  {
    drv_free(db);
    *result = (char *)RELOCATED("DRIVER PAGE MEMORY ALLOCATION ERROR DURING INITIALIZATION");
    return FLASH_DRV_ERROR;
  }

  ret_val = mmc_config(mmc_dis->sid, 
                       mmc_dis->mmc_volt,
                       mmc_dis->data_width,
                       &(mmc_dis->card_rca),
                       &(mmc_dis->card_type),
                       &(mmc_dis->data_width_support),
                       &(mmc_dis->transfer_clk_max),
                       &(mmc_dis->card_size));

  //if(db->devinfo)
  //{
    dbg_printf("eMMC sid............................. %#02X", mmc_dis->sid);
    dbg_printf("eMMC mmc_volt........................ %#02X", mmc_dis->mmc_volt);
    dbg_printf("eMMC data_width...................... %#02X", mmc_dis->data_width);
    dbg_printf("eMMC card_rca........................ %#02X", mmc_dis->card_rca);
    dbg_printf("eMMC card_type....................... %#02X", mmc_dis->card_type);
    dbg_printf("eMMC data_width_support.............. %#02X", mmc_dis->data_width_support);
    dbg_printf("eMMC transfer_clk_max................ %#02X (%d)", mmc_dis->transfer_clk_max, mmc_dis->transfer_clk_max);
    dbg_printf("eMMC card_size....................... %#llX (%d MB)", mmc_dis->card_size, mmc_dis->card_size >> 20);
    dbg_printf("eMMC mblock.......................... %#02X", db->mblock);
    dbg_printf("eMMC ddr............................. %#02X", db->ddr);
  //}

  if(ret_val != OMAPFLASH_SUCCESS)
  {
    drv_send_info("MMC mmc_config return %#08X", ret_val);
    *result = (char *)RELOCATED("MMC CONFIG FAILURE");
    emmc_drv_dnld_deinit(config, result);
    return ret_val;
  }

  if (mmc_dis->data_width > mmc_dis->data_width_support)
  {
    // mmc_dis->data_width = mmc_dis->data_width_support;
    drv_send_info("EMMC init failed. Cannot support %#02d-bit data width", mmc_dis->data_width);
      return (U32)OMAPFLASH_ERROR;
  }

  if(db->maxclk)
  {
    if (db->maxclk > mmc_dis->transfer_clk_max)
    {
      drv_send_info("EMMC init failed. Cannot support %#02d clock speed", db->maxclk);
          return (U32)OMAPFLASH_ERROR;
    }
    mmc_dis->transfer_clk = db->maxclk; 
  }
  else
  {
    mmc_dis->transfer_clk = mmc_dis->transfer_clk_max; 
  }

  ret_val = get_mmc_info(db);

  if(ret_val != OMAPFLASH_SUCCESS)
  {
    drv_send_info("MMC get_mmc_info failed");
    *result = (char *)RELOCATED("MMC CONFIG FAILURE");
    emmc_drv_dnld_deinit(config, result);
    return ret_val;
  }

  if(mmc_datatransfer_enable_disable((U8)mmc_dis->sid, MMC_TRANFER_ENABLE, mmc_dis->card_rca) != OMAPFLASH_SUCCESS)
  {
    drv_send_info("sd_mmc_card_read: Error putting card in transfer state");
      return (U32)OMAPFLASH_ERROR;
  }

  {
    U8 cfg;

    switch(db->partition)
    {
      case USER_DATA_AREA:
        if(db->make_bootable)
        {
          // If the partition if to be made boot-able, configure as per ROM code requirements
          cfg = BOOT_ACK_ENABLE | BOOT_PARTITION_UDA | PARTITION_ACCESS_NONE;
        }
        else
        {
          if(PARTITION_CONFIG__BOOT_PARTITION(db->ext_csd->partition_config) == BOOT_PARTITION_UDA)
          {
            // Partition is not to be boot-able and the partition was already selected for boot in the configuration
            cfg = BOOT_ACK_DISABLE | BOOT_PARTITION_NONE | PARTITION_ACCESS_NONE;
          }
          else
          {
            // Partition is not to be boot-able and another partition was selected for boot in the configuration
            cfg = PARTITION_CONFIG__BOOT_ACK(db->ext_csd->partition_config) | PARTITION_CONFIG__BOOT_PARTITION(db->ext_csd->partition_config) | PARTITION_ACCESS_NONE;
          }
        }
        break;

      case BOOT_AREA_PARTITION_1:
        if(db->make_bootable)
        {
          // If the partition if to be made boot-able, configure as per ROM code requirements
          cfg = BOOT_ACK_ENABLE | BOOT_PARTITION_BP1 | PARTITION_ACCESS_BP1;
        }
        else
        {
          if(PARTITION_CONFIG__BOOT_PARTITION(db->ext_csd->partition_config) == BOOT_PARTITION_BP1)
          {
            // Partition is not to be boot-able and the partition was already selected for boot in the configuration
            cfg = BOOT_ACK_DISABLE | BOOT_PARTITION_NONE | PARTITION_ACCESS_BP1;
          }
          else
          {
            // Partition is not to be boot-able and another partition was selected for boot in the configuration
            cfg = PARTITION_CONFIG__BOOT_ACK(db->ext_csd->partition_config) | PARTITION_CONFIG__BOOT_PARTITION(db->ext_csd->partition_config) | PARTITION_ACCESS_BP1;
          }
        }
        break;

      case BOOT_AREA_PARTITION_2:
        if(db->make_bootable)
        {
          // If the partition if to be made boot-able, configure as per ROM code requirements
          cfg = BOOT_ACK_ENABLE | BOOT_PARTITION_BP2 | PARTITION_ACCESS_BP2;
        }
        else
        {
          if(PARTITION_CONFIG__BOOT_PARTITION(db->ext_csd->partition_config) == BOOT_PARTITION_BP2)
          {
            // Partition is not to be boot-able and the partition was already selected for boot in the configuration
            cfg = BOOT_ACK_DISABLE | BOOT_PARTITION_NONE | PARTITION_ACCESS_BP2;
          }
          else
          {
            // Partition is not to be boot-able and another partition was selected for boot in the configuration
            cfg = PARTITION_CONFIG__BOOT_ACK(db->ext_csd->partition_config) | PARTITION_CONFIG__BOOT_PARTITION(db->ext_csd->partition_config) | PARTITION_ACCESS_BP2;
          }
        }
        break;
    
      default:
        drv_send_info("Unknown partition %d specified for operation", db->partition);
        return OMAPFLASH_ERROR;
    }

    if(cfg != db->ext_csd->partition_config)
    {
      dbg_printf("Changing PARTITION_CONFIG from %#02X to %#02X", db->ext_csd->partition_config, cfg);
      ret_val = mmc_send_switch_command(SWITCH_MODE_WR_BYTE, (U8)mmc_dis->sid, EXT_CSD_OFFSET(partition_config), cfg);
    }

    if (OMAPFLASH_SUCCESS != ret_val)
    {
      drv_send_info("Unable to switch to partition %d", db->partition);
      return ret_val;
    }

    wait_microsec(50000 + db->ext_csd->partition_switch_time * 10000); // Add an additional 50 ms to be sure...
  }

  if(RST_N_FUNCTION__RST_N_ENABLE(db->ext_csd->rst_n_function) == RST_N_ENABLE_TEMP_DISABLED) 
  {
    /* If RST_N is not already reconfigured, check whether there is a need to permanently change this parameter... If configured
       RST_N for the driver has been set to something other than what is already there, change it */

    if(RST_N_FUNCTION__RST_N_ENABLE(db->ext_csd->rst_n_function) != db->rst_n)
    {
      dbg_printf("Permanently changing RST_N_FUNCTION to %#02X", db->ext_csd->partition_config, db->rst_n);
      ret_val = mmc_send_switch_command(SWITCH_MODE_WR_BYTE, (U8)mmc_dis->sid, EXT_CSD_OFFSET(rst_n_function), db->rst_n);

      if (OMAPFLASH_SUCCESS != ret_val)
      {
        drv_send_info("Unable to change reset function");
        return ret_val;
      }

      wait_microsec(10000);
    }
  }
  else
  {
    /* If RST_N has already been reconfigured check whether there is a configuration of the parameter for the driver and if so, 
       check whether this configuration matches the one applied already... */

    if(setup[C_RST_N].valid)
    {
      if(RST_N_FUNCTION__RST_N_ENABLE(db->ext_csd->rst_n_function) != db->rst_n)
      {
        drv_send_info("Reset function already set (%#02X) does not match requested configuration (%#02X)", db->ext_csd->rst_n_function & 0x03, db->rst_n);
        return OMAPFLASH_ERROR;
      }
    }
  }

  ret_val = set_hsmmc_clk_data_width((U8)mmc_dis->sid, mmc_dis->card_type, mmc_dis->card_rca, mmc_dis->data_width, mmc_dis->transfer_clk, db->ddr);

  if (OMAPFLASH_SUCCESS != ret_val)
  {
    drv_send_info("MMC set_hsmmc_clk_data_width failed. ret_val:%#02X", ret_val);
    return ret_val;
  }

  db->adma_desc_num = MAX_ADMA_DESC_NUM;    
  db->adma_desc = (omap_adma2_desc *)malloc(sizeof(omap_adma2_desc) * db->adma_desc_num);

  if(db->adma_desc == NULL)
  {
      drv_send_info("Cannot allocate memory to create ADMA2 descriptor");
      return (U32)OMAPFLASH_ERROR;
  }

  dbg_printf("eMMC driver init complete");
  return ret_val;
}

/*------------------------------------------------------------------------------
| Function    : drv_dnld_deinit
+------------------------------------------------------------------------------
| Description : eMMC Deinit Function
|
| Parameters  :
|   flash     - [IN] - Flash parms
|   address   - [IN] - Block address
|   length    - [IN] - length to erase
|
| Returns     :
+----------------------------------------------------------------------------*/
U32 emmc_drv_dnld_deinit(T_driver_config * config, char ** result)
{
  T_db * db = (T_db *)config->data;
  
  if(db->ext_csd)
  {
    drv_free(db->ext_csd);
  }

  if(db->block_data)
  {
    drv_free(db->block_data);
  }

  if(db->block_data_partial)
  {
    drv_free(db->block_data_partial);
  }

  if(db)
  {
    drv_free(db);
    config->data = NULL;
  }

  dbg_printf("eMMC driver deinit complete");
  return FLASH_DRV_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : emmc_drv_dnld_get_info
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 emmc_drv_dnld_get_info(T_driver_config * config, T_driver_info * info)
{
  T_db * db = (T_db *)config->data;
  info->device_base_address = 0;
  if(db->partition == USER_DATA_AREA)
  {
    info->device_size = db->local_dis.card_size;
  }
  else
  {
    info->device_size = db->ext_csd->boot_size_mult * 128 * 1024;
  }
  return FLASH_DRV_SUCCESS;
}

/*------------------------------------------------------------------------------
| Function    : mmc_read
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 mmc_read(T_db * db, U32 start_block, U32 block_count, void *destination)
{
  T_MMC_DIS *local_dis = &db->local_dis;
  S32 ret_val;
  ret_val = sd_mmc_card_read(local_dis->sid,
                             local_dis->card_rca,
                             local_dis->card_type,
                             local_dis->data_width,
                             local_dis->transfer_clk,
                             db->mblock,
                             db->ddr,
                             start_block, block_count,
                             block_count * MMC_BLOCK_SIZE, destination);
  return ret_val;
}

/*------------------------------------------------------------------------------
| Function    : mmc_write
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 mmc_write(T_db * db, U32 start_block, U32 block_count, void *source)
{
  T_MMC_DIS *local_dis = &db->local_dis;
  S32 ret_val;
  ret_val = sd_mmc_card_write(local_dis->sid,
                              local_dis->card_rca,
                              local_dis->card_type,
                              local_dis->data_width,
                              local_dis->transfer_clk,
                              db->mblock,
                              db->ddr,
                              start_block, block_count,
                              block_count * MMC_BLOCK_SIZE, 
                              source);
  return ret_val;
}

//U32 mmc_erase(T_db * db, U32 start_block, U32 block_count, U32 status_offset, U32 status_length)
/*------------------------------------------------------------------------------
| Function    : mmc_erase
+------------------------------------------------------------------------------
| Description : 
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 mmc_erase(T_db * db, U32 start_block, U32 block_count)
{
  T_MMC_DIS *local_dis = &db->local_dis;
  S32 ret_val = sd_mmc_card_erase(local_dis->sid,
                                  local_dis->card_rca,
                                  local_dis->card_type,
                                  local_dis->data_width,
                                  local_dis->transfer_clk,
                                  db->ddr,
                                  start_block, block_count,
                                  //status_offset, status_length, NULL);
                                  NULL);
  return ret_val;
}

/*------------------------------------------------------------------------------
 | Function    : emmc_write
 +------------------------------------------------------------------------------
 | Description : Write function for eMMC
 |
 | Parameters  : config, 
 |               result
 |               dest_addr
 |               src_addr
 |               size
 |               more
 |
 | Returns     : Result code
 +----------------------------------------------------------------------------*/
U32 emmc_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more)
{
  T_db * db = (T_db *)config->data;
  S32 ret_val = OMAPFLASH_SUCCESS;
  U32 first_block = (U32)(dest_addr / MMC_BLOCK_SIZE);
  U32 first_offset = dest_addr % MMC_BLOCK_SIZE;
  U32 first_size = first_offset ? MMC_BLOCK_SIZE - first_offset : 0;
  U32 whole_block_count = 0;
  U32 last_offset = 0;
  U32 last_size = 0;
  U32 last_block = 0;
  U8 *src = (U8*)src_addr;
  U32 block = first_block;

  if (first_size > size)
  {
    first_size = size;
  }
  else 
  {
    whole_block_count = (size - first_size) / MMC_BLOCK_SIZE;
    last_offset = first_size + whole_block_count * MMC_BLOCK_SIZE;
    last_block = first_block + (first_offset ? 1 : 0) + whole_block_count;
    last_size = size - last_offset;
  }

  if (first_size != 0) 
  {
    if (db->block_id != first_block || db->block_id == NO_MMC_BLOCK_ID) 
    {
  		//dbg_printf("Read first partial block to do partial write");
      ret_val = mmc_read(db, first_block, 1, db->block_data);
    }
    else if (!more && first_offset + first_size < MMC_BLOCK_SIZE) 
    {
      ///@todo refactor reader to support reading partial block and eliminate +MMC_BLOCK_SIZE here
      U8 *block_data = db->block_data + MMC_BLOCK_SIZE;
		  //dbg_printf("Read last partial block to do partial write");
      ret_val = mmc_read(db, first_block, 1, block_data);
      if (ret_val == OMAPFLASH_SUCCESS) 
      {
        if(db->block_id_partial == first_block)
        {
          memcpy(db->block_data, db->block_data_partial, first_offset);
        }
        memcpy(db->block_data + first_offset, block_data + first_offset, first_size);
      }
    }

    if (ret_val == OMAPFLASH_SUCCESS)
    {
		    // dbg_printf("Write first partial block");
        if(db->block_id_partial == first_block)
        {
          memcpy(db->block_data, db->block_data_partial, first_offset);
        }
        memcpy(db->block_data + first_offset, src, first_size);
        ret_val = mmc_write(db, first_block, 1, db->block_data);
        if (ret_val == OMAPFLASH_SUCCESS)
        {
            db->block_id = first_block;
            block++;
        }
        else
        {
            db->block_id = NO_MMC_BLOCK_ID;
        }
    }
    else 
    {
      db->block_id = NO_MMC_BLOCK_ID;
    }
  }

  if (ret_val == OMAPFLASH_SUCCESS && whole_block_count != 0) 
  {
	  //dbg_printf("Write %d whole blocks", whole_block_count);
    ret_val = mmc_write(db, block, whole_block_count, src + first_size);
  }

  if (ret_val == OMAPFLASH_SUCCESS) 
  {
    if (last_size != 0) 
    {
      if (more) 
      {    
		    //dbg_printf("Save last partial block");
        memcpy(db->block_data_partial, src + last_offset, last_size);
        db->block_id_partial = last_block;
      }
      else 
      {
		    //dbg_printf("Read/write last partial block");
        ret_val = mmc_read(db, last_block, 1, db->block_data);
        if (ret_val == OMAPFLASH_SUCCESS) 
        {
          memcpy(db->block_data, src+last_offset, last_size);
          if (!more) 
          {
            ret_val = mmc_write(db, last_block, 1, db->block_data);
            if (ret_val == OMAPFLASH_SUCCESS)
            {
              db->block_id = last_block;
            }
          }
        }
        else
        {
          db->block_id = NO_MMC_BLOCK_ID;
        }
      }
    }
    //else
    //{
    // dbg_printf("No last partial block");
    //}
  }

  if(!more)
  {
    db->block_id_partial = NO_MMC_BLOCK_ID;
  }

  return ret_val;
}

/*------------------------------------------------------------------------------
| Function    : emmc_memset
+------------------------------------------------------------------------------
| Description : Function for memsetting the content of the eMMC device to a
|               certain value. Value is a U32. Function uses a 1 MB buffer for 
|               the erase data and will overwrite the content using the value.
|
| Parameters  : 
|
| Returns     : void
+----------------------------------------------------------------------------*/
U32 emmc_memset(T_driver_config * config, char ** result, U64 address, U64 length, U32 value, BOOLEAN send_progress)
{
  #define MEMSET_BUFFER_SIZE (1024 * 1024)
  S32 ret_val = OMAPFLASH_SUCCESS;
  U32 * memset_buffer = config->drv_malloc(MEMSET_BUFFER_SIZE);
  U64 write_buffer_calls_full = length / MEMSET_BUFFER_SIZE;
  U64 count;
  U64 target_length = length;

  //dbg_printf("emmc_memset address: %#llX length %#llX value %#08X %s", address, length, value, send_progress ? "Progress" : "No progress");

  for(count = 0; count < MEMSET_BUFFER_SIZE / sizeof(U32); count ++)
  {
    memset_buffer[count] = value;
  }

  for(count = 0; count < write_buffer_calls_full; count++)
  {
    emmc_write(config, result, address, (U32)memset_buffer, MEMSET_BUFFER_SIZE, (T_more_data)(length > MEMSET_BUFFER_SIZE));
    if(send_progress)
    {
      config->send_status("Erase progress", target_length - length, target_length);
    }
    length -= MEMSET_BUFFER_SIZE;
    address += MEMSET_BUFFER_SIZE;
  }

  if(length)
  {
    emmc_write(config, result, address, (U32)memset_buffer, (U32)length, NO_MORE_DATA);
  }

  if(send_progress)
  {
    config->send_status("Erase progress", target_length, target_length);
  }

  config->drv_free(memset_buffer);
  *result = NULL;
  return ret_val;      
}

/*------------------------------------------------------------------------------
 | Function    : drv_dnld_erase
 +------------------------------------------------------------------------------
 | Description : Erase Function to erase a block in the eMMC device
 |
 | Parameters  :
 |   flash     - [IN] - Flash parms
 |   address   - [IN] - Block address
 |   length    - [IN] - length to erase
 |
 | Returns     :
 +----------------------------------------------------------------------------*/
U32 emmc_drv_dnld_erase(T_driver_config * config, char ** result, U64 address, U64 length)
{
  T_db * db = (T_db *)config->data;
  U32 value = db->ext_csd->erased_mem_cont ? 0xFFFFFFFF : 0x00000000;
  
  if(length == 0) 
  {
    length = db->local_dis.card_size - address;
  }

  dbg_printf("Erasing from %#llX to %#llX (%#llX bytes, erased content %#02X)", address, address + length, length, (value & 0xFF));

  return emmc_memset(config, result, address, length, value, TRUE);
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
U32 emmc_drv_dnld_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more)
{
  T_db * db = (T_db *)config->data;
  U32 ret_val = OMAPFLASH_SUCCESS;

  /* First, check whether Sparse format is supported and a check for the format is needed. This will be for the first
     packet only - after that we should end up either in the sparse state active (the format is Sparse) or inactive
     (the format is raw). */
  
  if(db->sparse.state == sparse_check_needed)
  {
    sparse_header_t * header = (sparse_header_t *)src_addr;

    if(header->magic == SPARSE_HEADER_MAGIC)
    {
      /* The format is apparently Sparse - now do a lot of misc checks of the content of the header of the Sparse
         format */

      dbg_printf("\nSparse image detected:");
      dbg_printf("- version             = %d.%d", header->major_version, header->minor_version);
      dbg_printf("- file header size    = %d", header->file_hdr_sz);
      dbg_printf("- chunk header size   = %d", header->chunk_hdr_sz);
      dbg_printf("- block size          = %d", header->blk_sz);
      dbg_printf("- total output blocks = %d", header->total_blks);
      dbg_printf("- total input chunks  = %d", header->total_chunks);
      dbg_printf("- image checksum      = %#08X\n", header->image_checksum);	
      
      /* Check that there is actually enough data for the header itself */

      if(size < sizeof(sparse_header_t))
      {
        dbg_printf("Size of received buffer smaller than header size");
        config->send_info("Image size %d incorrect", size);
        *result = (char *)RELOCATED("INCORRECT FILE SIZE");
        return (U32)OMAPFLASH_ERROR;
      }

      /* Check the major version number of the format in the Sparse encoded data */
      
      if(header->major_version > 0x01)
      {
        dbg_printf("Image version %d not supported", header->major_version);
        config->send_info("Image version %d not supported", header->major_version);
        *result = (char *)RELOCATED("MAJOR VERSION OUT OF RANGE");
        return (U32)OMAPFLASH_ERROR;
      }

      /* Check that the size of the header is as expected */

      if(header->file_hdr_sz != sizeof(sparse_header_t))
      {
        dbg_printf("File header size %d incorrect", header->file_hdr_sz);
        config->send_info("File header size %d incorrect", header->file_hdr_sz);
        *result = (char *)RELOCATED("INCORRECT FILE HEADER");
        return (U32)OMAPFLASH_ERROR;
      }

      /* Check that the chunk header size is as expected in this version of the format */

      if(header->chunk_hdr_sz != sizeof(chunk_header_t))
      {
        dbg_printf("Chunk header size %d incorrect", header->chunk_hdr_sz);
        config->send_info("Chunk header size %d incorrect", header->chunk_hdr_sz);
        *result = (char *)RELOCATED("INCORRECT FILE HEADER");
        return (U32)OMAPFLASH_ERROR;
      }

      /* Check that the block size for chunks is aligned to a 4-byte boundary as per the standard */

      if((header->blk_sz & 0x0000003) > 0)
      {
        dbg_printf("Block size %d incorrect - not multiple of 4", header->blk_sz);
        config->send_info("Block size %d incorrect - not multiple of 4", header->blk_sz);
        *result = (char *)RELOCATED("INCORRECT FILE HEADER");
        return (U32)OMAPFLASH_ERROR;
      }

      /* Copy the header to local storage so that we have it on hand for later - then move the src pointer
         and adjust the size to go for the remaining part of the content below the header */

      drv_memcpy(&(db->sparse.file_header), header, sizeof(sparse_header_t));
      src_addr += sizeof(sparse_header_t);
      size -= sizeof(sparse_header_t);

      /* Check that we have at least enough data remaining for one chunk - otherwise there must be a problem
         with the encoding of the image */

      if(size < sizeof(chunk_header_t))
      {
        dbg_printf("Size of image does not include valid first chunk");
        config->send_info("Image size %d incorrect (no first chunk)", size + sizeof(sparse_header_t));
        *result = (char *)RELOCATED("INCORRECT FILE SIZE");
        return (U32)OMAPFLASH_ERROR;
      }

      /* Initialize tracking variable and change the state to active sparse format processing */

      db->sparse.chunk_header_bytes = 0;
      db->sparse.chunk_index        = 0;
      db->sparse.uncompressed_size  = 0;
      db->sparse.output_address     = dest_addr;
      db->sparse.start_address      = dest_addr;
      drv_memset(&db->sparse.chunk_stats, 0, sizeof(db->sparse.chunk_stats));
      db->sparse.state = sparse_active;
    }
    else
    {
      /* The format of the data being downloaded seems to be raw - set the Sparse state to inactive */
      
      dbg_printf("Raw image detected");
      db->sparse.state = sparse_inactive;
    }
  }
  
  /* Check whether we need to write the data as raw format - if so, just write the buffer to eMMC directly */

  if((db->sparse.state == sparse_disabled) || (db->sparse.state == sparse_inactive))
  {
    dbg_printf("Writing %d raw bytes to %#llX (%s)", size, dest_addr, more ? "more" : "last");
    ret_val = emmc_write(config, result, dest_addr, src_addr, size, more);
  }

  /* Check whether we need to write the data as sparse encoded format - if so, process the content accordingly */

  if(db->sparse.state == sparse_active)
  {
    BOOLEAN done = FALSE;

    /* While we have more data, an active chunk header and we are not done with the processing of the active chunk header,
       decode and write to the eMMC memory as specified... */

    while((size > 0) || ((db->sparse.chunk_header_bytes == sizeof(chunk_header_t)) && (done == FALSE)))
    {
      if(db->sparse.chunk_header_bytes == sizeof(chunk_header_t))
      {
        /* We have a full chunk header and will need to have a look at it to figure out what to do - if it is a new chunk (we have
           not kept it around due to data being distributed across multiple data packets), trace out a bit of tracking information... */
        
        if(db->sparse.chunk_new)
        {
          dbg_printf("Chunk #%4d of %4d: %s - blocks %5d", 
                     db->sparse.chunk_index, 
                     db->sparse.file_header.total_chunks, 
                     db->sparse.chunk_header.chunk_type == CHUNK_TYPE_RAW ? "RAW " : 
                     db->sparse.chunk_header.chunk_type == CHUNK_TYPE_FILL ? "FILL" :
                     db->sparse.chunk_header.chunk_type == CHUNK_TYPE_DONT_CARE ? "DC  " :
                     db->sparse.chunk_header.chunk_type == CHUNK_TYPE_CRC ? "CRC " :
                     "UNKNOWN",
                     db->sparse.chunk_header.chunk_sz);
        }
        else
        {
          dbg_printf("(Chunk continued)");
        }

        /* Check the chunk header for the type of chunk to process */

        switch(db->sparse.chunk_header.chunk_type)
        {
          case CHUNK_TYPE_RAW:
            {
              U32 size_to_write = db->sparse.chunk_header.total_sz - sizeof(chunk_header_t);

              if(db->sparse.chunk_new)
              {
                db->sparse.chunk_stats.raw++;
                db->sparse.chunk_stats.raw_size += size_to_write;
              }

              if((size_to_write != (db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz)) && (db->sparse.chunk_new == TRUE))
              {
                dbg_printf("Size error for RAW chunk (output size %d != chunk total size %d)", 
                           db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz,
                           size_to_write);
                config->send_info("Size error for RAW chunk (output size %d != chunk total size %d)", 
                                  db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz,
                                  size_to_write);
                *result = (char *)RELOCATED("FILE FORMAT ERROR");
                return (U32)OMAPFLASH_ERROR;
              }

              if(size < size_to_write)
              {
                size_to_write = size;
              }
              else
              {
                db->sparse.chunk_header_bytes = 0;
              }

              db->sparse.chunk_header.total_sz -= size_to_write;

              ret_val = emmc_write(config, result, db->sparse.output_address, src_addr, size_to_write, NO_MORE_DATA);
                
              if(ret_val != OMAPFLASH_SUCCESS)
              {
                dbg_printf("eMMC write error");
                config->send_info("eMMC write error");
                return (U32)OMAPFLASH_ERROR;
              }

              db->sparse.output_address    += size_to_write;
              db->sparse.uncompressed_size += size_to_write;

              size     -= size_to_write;
              src_addr += size_to_write;

              /* If there is still data to write in RAW mode according to the chunk header, and there is no more data coming, there must
                 be a problem with the format of the image */
              
              if((db->sparse.chunk_header.total_sz > sizeof(chunk_header_t)) && (more == NO_MORE_DATA))
              {
                dbg_printf("RAW chunk incomplete at end of data (%d bytes missing)", db->sparse.chunk_header.total_sz - sizeof(chunk_header_t));
                config->send_info("RAW chunk incomplete at end of data (%d bytes missing)", db->sparse.chunk_header.total_sz - sizeof(chunk_header_t));
                *result = (char *)RELOCATED("FILE FORMAT ERROR");
                return (U32)OMAPFLASH_ERROR;
              }

              /* If the size of the remaining data in the current packet is zero, we must be done, even if the chunk header is still valid... */

              if(size == 0)
              {
                done = TRUE;
              }
            }
            break;

          case CHUNK_TYPE_FILL:
            if(db->sparse.chunk_header.total_sz != (sizeof(chunk_header_t) + 4))
            {
              dbg_printf("Input size error for FILL chunk (total size %d != %d)", db->sparse.chunk_header.total_sz, (sizeof(chunk_header_t) + 4));
              config->send_info("Input size error for FILL chunk (total size %d != %d)", db->sparse.chunk_header.total_sz, (sizeof(chunk_header_t) + 4));
              *result = (char *)RELOCATED("FILE FORMAT ERROR");
              return (U32)OMAPFLASH_ERROR;
            }
            
            if(db->sparse.chunk_new)
            {
              db->sparse.chunk_stats.fill++;
              db->sparse.chunk_stats.fill_size += db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz;
            }

            if(size >= 4)
            {
              /* It looks like there is enough data to process the chunk */
              
              U32 fill_value = *(U32 *)src_addr;

              db->sparse.uncompressed_size  += db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz;

              if(db->sparse.uncompressed_size > (db->sparse.file_header.total_blks * db->sparse.file_header.blk_sz))
              {
                dbg_printf("Too much data in uncompressed image (%d)", db->sparse.uncompressed_size);
                config->send_info("Too much data in uncompressed image (%d)", db->sparse.uncompressed_size);
                *result = (char *)RELOCATED("FILE FORMAT ERROR");
                return (U32)OMAPFLASH_ERROR;
              }

              ret_val = emmc_memset(config, result, db->sparse.output_address, db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz, fill_value, FALSE);

              if(ret_val != OMAPFLASH_SUCCESS)
              {
                dbg_printf("Failed to fill memory for chunk %d", db->sparse.chunk_header_bytes);
                config->send_info("Failed to fill memory for chunk %d", db->sparse.chunk_header_bytes);
                *result = (char *)RELOCATED("DEVICE ACCESS ERROR");
                return (U32)OMAPFLASH_ERROR;
              }

              src_addr += 4;
              size     -= 4;
              db->sparse.chunk_header_bytes = 0;

              db->sparse.output_address += db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz;
            }
            else if(size)
            {
              /* There is not enough data to process the chunk, which means that the packet content has somehow ended up
                 not being aligned to a word boundary in the packet - that would seem to be a problem... */

              dbg_printf("Alignment problem at buffer boundary - %d bytes", size);
              config->send_info("Alignment problem at buffer boundary - %d bytes", size);
              *result = (char *)RELOCATED("PROCESSING ERROR (INTERNAL)");
              return (U32)OMAPFLASH_ERROR;
            }
            else if(more == NO_MORE_DATA)
            {
              /* There is not enough data to complete the chunk decoding and we are not supposed to receive any more
                 data from the second - so something is off... */

              dbg_printf("Incomplete chunk at end of data - fill data missing");
              config->send_info("Incomplete chunk at end of data - fill data missing");
              *result = (char *)RELOCATED("File format error");
              return (U32)OMAPFLASH_ERROR;
            }
            else
            {
              /* There is not enough data and it would seem that the chunk ends on the packet boundary - we need to keep the
                 chunk header and move on to the next packet... */
              
              done = TRUE;
            }
            break;

          case CHUNK_TYPE_DONT_CARE:
          case CHUNK_TYPE_CRC:
            /* These two chunk types are not dependent on additional data - so can always be fully processed... */
            
            if(db->sparse.chunk_header.total_sz != sizeof(chunk_header_t))
            {
              dbg_printf("Input size error for DC or CRC chunk (total size %d != %d)", db->sparse.chunk_header.total_sz, sizeof(chunk_header_t));
              config->send_info("Input size error for DC or CRC chunk (total size %d != %d)", db->sparse.chunk_header.total_sz, sizeof(chunk_header_t));
              *result = (char *)RELOCATED("FILE FORMAT ERROR");
              return (U32)OMAPFLASH_ERROR;
            }

            if(db->sparse.chunk_new)
            {
              if(db->sparse.chunk_header.chunk_type == CHUNK_TYPE_DONT_CARE)
              {
                db->sparse.chunk_stats.dc++;
                db->sparse.chunk_stats.dc_size += db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz;
              }
              else
              {
                db->sparse.chunk_stats.crc++;
              }
            }

            db->sparse.chunk_header_bytes = 0;
            db->sparse.output_address += db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz;
            db->sparse.uncompressed_size += db->sparse.chunk_header.chunk_sz * db->sparse.file_header.blk_sz;
            break;

          default:
            dbg_printf("Unknown chunk type in header (%#04X) - don't know how to proceed", db->sparse.chunk_header.chunk_type);
            config->send_info("Unknown chunk type in header (%#04X) - don't know how to proceed", db->sparse.chunk_header.chunk_type);
            *result = (char *)RELOCATED("FILE FORMAT ERROR");
            return (U32)OMAPFLASH_ERROR;
        }

        db->sparse.chunk_new = FALSE;
      }
      else
      {
        /* We are looking for a new chunk header or to complete a partial chunk header from the boundary area of the previous packet... */
        
        U32 bytes_to_copy = (size >= (sizeof(chunk_header_t) - db->sparse.chunk_header_bytes)) ? sizeof(chunk_header_t) - db->sparse.chunk_header_bytes : size;
        
        drv_memcpy(((U8 *)&(db->sparse.chunk_header)) + db->sparse.chunk_header_bytes, (void *)src_addr, bytes_to_copy);

        db->sparse.chunk_new = TRUE;
        
        db->sparse.chunk_header_bytes += bytes_to_copy;
        size     -= bytes_to_copy;
        src_addr += bytes_to_copy;

        if(db->sparse.chunk_header_bytes < sizeof(chunk_header_t))
        {
          dbg_printf("Copied partial chunk header of %d bytes at end-of-buffer", db->sparse.chunk_header_bytes);
        }
        else if(bytes_to_copy < sizeof(chunk_header_t))
        {
          dbg_printf("Partial chunk header completed with %d bytes", bytes_to_copy);
        }
        
        /* If we have no more data and did not complete a chunk header, something must be wrong... */

        if((size == 0) && (db->sparse.chunk_header_bytes != sizeof(chunk_header_t)) && (more == NO_MORE_DATA))
        {
          dbg_printf("Incomplete chunk header of %d bytes at end-of-data", db->sparse.chunk_header_bytes);
          config->send_info("Incomplete chunk header of %d bytes at end of data", db->sparse.chunk_header_bytes);
          *result = (char *)RELOCATED("FILE FORMAT ERROR");
          return (U32)OMAPFLASH_ERROR;
        }

        /* Check the chunk index against the sparse file header information to verify that we are not processing
           more headers than expected */

        if(db->sparse.chunk_header_bytes == sizeof(chunk_header_t))
        {
          db->sparse.chunk_index++;

          if(db->sparse.chunk_index > db->sparse.file_header.total_chunks)
          {
            dbg_printf("Too many chunks detected in file - only expected %d but found at least one more", db->sparse.file_header.total_chunks);
            config->send_info("Too many chunks detected in file - only expected %d but found at least one more", db->sparse.file_header.total_chunks);
            *result = (char *)RELOCATED("FILE FORMAT ERROR");
            return (U32)OMAPFLASH_ERROR;
          }
        }
      }
    }
  }

  /* If we are at the end of the file and sparse is enabled in the driver, make sure we can process another file
     if new data shows up */

  if((more == FALSE) && (db->sparse.state != sparse_disabled))
  {
    if(db->sparse.state == sparse_active)
    {
      //U64 expected_output = db->sparse.file_header.total_blks * db->sparse.file_header.blk_sz; - Note: Compiler issue cause 32-bit truncation of calculated result
      U64 expected_output = db->sparse.file_header.total_blks; 
      expected_output = expected_output * db->sparse.file_header.blk_sz;

      db->sparse.state = sparse_check_needed;

      dbg_printf("\nSparse image completed:");
      dbg_printf("- Start address       = %#llX", db->sparse.start_address);
      dbg_printf("- End address         = %#llX", db->sparse.start_address + (db->sparse.uncompressed_size ? db->sparse.uncompressed_size - 1 : 0));
      dbg_printf("- Expected output     = %#llX bytes", expected_output);
      dbg_printf("- Actual output       = %#llX bytes", db->sparse.uncompressed_size);
      dbg_printf("- Raw chunks          = %d (%#llX bytes)", db->sparse.chunk_stats.raw, db->sparse.chunk_stats.raw_size);
      dbg_printf("- Fill chunks         = %d (%#llX bytes)", db->sparse.chunk_stats.fill, db->sparse.chunk_stats.fill_size);
      dbg_printf("- Don't care chunks   = %d (%#llX bytes)", db->sparse.chunk_stats.dc, db->sparse.chunk_stats.dc_size);
      dbg_printf("- CRC chunks          = %d", db->sparse.chunk_stats.crc);

      if(db->sparse.file_header.total_chunks !=  db->sparse.chunk_index)
      {
        dbg_printf("Number of chunks in image not as indicated by file header!");
        config->send_info("Number of chunks in image not as indicated by file header");
        *result = (char *)RELOCATED("FILE FORMAT ERROR");
        ret_val = (U32)OMAPFLASH_ERROR;
      }

      if(expected_output != db->sparse.uncompressed_size)
      {
        dbg_printf("Expected output does not match actual output!");
        config->send_info("Uncompressed size not as indicated by file header");
        *result = (char *)RELOCATED("FILE FORMAT ERROR");
        ret_val = (U32)OMAPFLASH_ERROR;
      }
    }
    else
    {
      db->sparse.state = sparse_check_needed;
    }
  }

  return ret_val;
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
U32 emmc_drv_dnld_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size)
{
  T_db * db = (T_db *)config->data;
  S32 ret_val = OMAPFLASH_SUCCESS;
  U32 start_block = (U32)(src_addr / MMC_BLOCK_SIZE);
  U32 first_offset = src_addr % MMC_BLOCK_SIZE;
  U32 first_size = first_offset ? MMC_BLOCK_SIZE - first_offset : 0;
  U32 whole_block_count = 0;
  U32 last_offset = 0;
  U32 last_size = 0;
  U8 *dst = (U8*)dst_addr;

  if (first_size > size)
  {
    first_size = size;
  }
  else 
  {
    whole_block_count = (size - first_size) / MMC_BLOCK_SIZE;
    last_offset = first_size + whole_block_count * MMC_BLOCK_SIZE;
    last_size = size - last_offset;
  }

  if (first_size != 0) 
  {
    if (db->block_id != start_block || db->block_id == NO_MMC_BLOCK_ID) 
    {
	  	dbg_printf("Read first partial block");
      ret_val = mmc_read(db, start_block, 1, db->block_data);
    }
    if (ret_val == OMAPFLASH_SUCCESS)
    {
      db->block_id = start_block;
      memcpy(dst, db->block_data + first_offset, first_size);
      start_block++;
    }
    else
    {
      db->block_id = NO_MMC_BLOCK_ID;
    }
  }

  if (ret_val == OMAPFLASH_SUCCESS && whole_block_count != 0) 
  {
	dbg_printf("Read %d whole blocks", whole_block_count);
    ret_val = mmc_read(db, start_block, whole_block_count, dst + first_size);
  }

  if (ret_val == OMAPFLASH_SUCCESS) 
  {
    if (last_size != 0) 
    {
	    dbg_printf("Read last partial block");
      ret_val = mmc_read(db, start_block + whole_block_count, 1, db->block_data);
      if (ret_val == OMAPFLASH_SUCCESS) 
      {
        db->block_id = start_block + whole_block_count;
        memcpy(dst+last_offset, db->block_data, last_size);
      }
      else
      {
        db->block_id = NO_MMC_BLOCK_ID;
      }
    }
  }

  *result = NULL;

  return ret_val;
}

