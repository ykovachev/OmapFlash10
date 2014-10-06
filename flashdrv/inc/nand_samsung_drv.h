/*-----------------------------------------------------------------------------
|  Project :  CSST
+------------------------------------------------------------------------------
|             Copyright 2005 - 2009 Texas Instruments.
|             All rights reserved.
|
|             This file is confidential and a trade secret of Texas
|             Instruments .
|             The receipt of or possession of this file does not convey
|             any rights to reproduce or disclose its contents or to
|             manufacture, use, or sell anything it may describe, in
|             whole, or in part, without the specific written consent of
|             Texas Instruments.
+------------------------------------------------------------------------------
| Filename:   nand_samsung_drv.h
| Author:     TII
| Purpose:    NAND samsung flash driver for 8 bit and 16 bit samsung
|             nand flashes)
+----------------------------------------------------------------------------*/
/*==== DECLARATION CONTROL =================================================*/
#ifndef NAND_SAMSUNG_DRV_H
#define NAND_SAMSUNG_DRV_H

/*==== INCLUDES ============================================================*/
#include "types.h"
#include "flash_drv.h"
#include "dnld_trg_func.h"

/*==== CONSTS ==============================================================*/

#define	READ_A                0x00
#define	READ_B                0x01
#define	READ_C                0x50
#define	READ_ID               0x90
#define	RESET                 0xFF
#define	PAGE_PROGRAM_START    0x80
#define	PAGE_PROGRAM_END      0x10
#define	COPY_BACK_PROGRAM     0x00
#define	COPY_BACK_PROGRAM_2	  0x8A
#define	BLOCK_ERASE           0x60
#define	BLOCK_ERASE_2         0xD0
#define	READ_STATUS           0x70

/* Samsung 8 Bit 1Gb device command set */
#define	READ_1					 0x00
#define	READ_2					 0x30

#define NAND_INVALID_BLOCK        (1)
#define NAND_VALID_BLOCK          (0)


#define	STATUS_ERROR             0x01
#define	STATUS_READY             0x40
#define	STATUS_WRITE_PROTECT     0x80



#define BYTE                     (8)
#define WORD                     (16)
#define SPARE_AREA_SIZE 16


#define NAND_STATUS_OK                                      0x00
#define NAND_STATUS_ERROR_INVALID_ADDR                      0xFF
#define NAND_STATUS_ERROR_INVALID_PARM                      0xFE
#define NAND_STATUS_NO_MEMORY                               0xFD


#if defined(SDP_2430)

#define ECC_CS 0x01 /**ECC computed on CS1*/
#define GPMC_BASE_ADDR          0x6E000000
#define NAND_DATA 		         ((volatile U16 *)NAND_GPMC_CS1_DATA)
#define NAND_COMMAND	         ((volatile U16 *)NAND_GPMC_CS1_COMMAND)
#define NAND_ADDRESS	         ((volatile U16 *)NAND_GPMC_CS1_ADDRESS)

#elif defined(OMAP3430SDP) || defined(OMAP3430SDP_CHMN)

#define ECC_CS 0x01 /**ECC computed on CS1*/
#define GPMC_BASE_ADDR          0x6E000000
#define NAND_DATA 		         ((volatile U16 *)NAND_GPMC_CS1_DATA)
#define NAND_COMMAND	         ((volatile U16 *)NAND_GPMC_CS1_COMMAND)
#define NAND_ADDRESS	         ((volatile U16 *)NAND_GPMC_CS1_ADDRESS)


#elif defined(SDP_2420)

/*Specific to 2420*/
#define ECC_CS 0x00 /**ECC computed on CS0*/
#define GPMC_BASE_ADDR          0x6800A000

#define NAND_DATA 		 ((volatile U16 *)NAND_GPMC_CS0_DATA)
#define NAND_COMMAND	         ((volatile U16 *)NAND_GPMC_CS0_COMMAND)
#define NAND_ADDRESS	         ((volatile U16 *)NAND_GPMC_CS0_ADDRESS)

#endif


#define GPMC_CONFIG             (GPMC_BASE_ADDR + 0x050)
#define GPMC_ECC_CONFIG			*(volatile U32 *) (GPMC_BASE_ADDR + 0x1F4)
#define GPMC_ECC_CONTROL		*(volatile U32 *) (GPMC_BASE_ADDR + 0x1F8)
#define GPMC_ECC_SIZE_CONFIG	*(volatile U32 *) (GPMC_BASE_ADDR + 0x1FC)
#define GPMC_ECC1_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x200)
#define GPMC_ECC2_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x204)
#define GPMC_ECC3_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x208)
#define GPMC_ECC4_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x20C)
#define GPMC_ECC5_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x210)
#define GPMC_ECC6_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x214)
#define GPMC_ECC7_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x218)
#define GPMC_ECC8_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x21C)
#define GPMC_ECC9_RESULT		*(volatile U32 *) (GPMC_BASE_ADDR + 0x220)
#define GPMC_ECC_RESULT_BASE	                  (GPMC_BASE_ADDR + 0x200)





#define ECC_ENABLE 0x01
#define ECC16B 0x01
#define ECCSIZE1 0xFF
#define ECCSIZE0 0xFF
#define ECC9RESULTSIZE  0x00
#define ECC8RESULTSIZE  0x00
#define ECC7RESULTSIZE  0x00
#define ECC6RESULTSIZE  0x00
#define ECC5RESULTSIZE  0x00
#define ECC4RESULTSIZE  0x00
#define ECC3RESULTSIZE  0x00
#define ECC2RESULTSIZE  0x00
#define ECC1RESULTSIZE  0x00
#define ECCCLEAR 0x01
#define ECCPOINTER 0x01

#define NAND_GPMC_CS0_COMMAND   (GPMC_BASE_ADDR + 0x07C)
#define NAND_GPMC_CS0_ADDRESS   (GPMC_BASE_ADDR + 0x080)
#define NAND_GPMC_CS0_DATA      (GPMC_BASE_ADDR + 0x084)

#define NAND_GPMC_CS1_COMMAND   (NAND_GPMC_CS0_COMMAND+(0x30*1))
#define NAND_GPMC_CS1_ADDRESS   (NAND_GPMC_CS0_ADDRESS+(0x30*1))
#define NAND_GPMC_CS1_DATA      (NAND_GPMC_CS0_DATA+(0x30*1))

#define NAND_GPMC_CS2_COMMAND   (NAND_GPMC_CS0_COMMAND+(0x30*2))
#define NAND_GPMC_CS2_ADDRESS   (NAND_GPMC_CS0_ADDRESS+(0x30*2))
#define NAND_GPMC_CS2_DATA      (NAND_GPMC_CS0_DATA+(0x30*2))

#define NAND_GPMC_CS7_COMMAND   (NAND_GPMC_CS0_COMMAND+(0x30*7))
#define NAND_GPMC_CS7_ADDRESS   (NAND_GPMC_CS0_ADDRESS+(0x30*7))
#define NAND_GPMC_CS7_DATA      (NAND_GPMC_CS0_DATA+(0x30*7))

typedef struct
{
    U16 vendor_code;
    U16 device_code;
    U32 spare_size;
    U32 block_size;
    U32 page_size;
    U32 num_pages;
    U32 num_blocks;
    U8  device_size;
    U8  bad_blk_idx;
    U32 cs_base_addr;
    U32 no_bit_errs_corrected;
    TFNCp_malloc malloc;
    TFNCp_free free;

} T_NAND_CONFIG_PARMS;


#define NAND_BLOCKSIZE					(512*32)
#define NAND_PAGESIZE				  512

#define MAXBLOCKNUM_NAND8         2048
#define MAXBLOCKNUM_DISCRETE      2048
#define MAXBLOCKNUM_COMBO         4096

#define VENDOR_ID_SAMSUNG         0xEC
#define DEVICE_ID_K9F1G08R0A      0xA1

U16 nand8_get_status ();
U16 nand16_get_status ();

/*==== EXPORTS =============================================================*/
S32 nand16_init(T_NAND_CONFIG_PARMS *flash, U32 cs_addr);
S32 nand16_erase(T_NAND_CONFIG_PARMS *flash, U32 addr, U32 length);
S32 nand16_write(T_NAND_CONFIG_PARMS *flash, U32 dst_addr, U32 src_addr, U32 size);
S32 nand16_read(T_NAND_CONFIG_PARMS *flash, U32 dst_addr, U32 src_addr, U32 size);

S32 nand8_init(T_NAND_CONFIG_PARMS *flash, U32 cs_addr);
S32 nand8_erase(T_NAND_CONFIG_PARMS *flash, U32 addr, U32 length);
S32 nand8_write(T_NAND_CONFIG_PARMS *flash, U32 dst_addr, U32 src_addr, U32 size);
S32 nand8_read(T_NAND_CONFIG_PARMS *flash, U32 dst_addr, U32 src_addr, U32 size);

U32 nand_samsung_init(T_FLASH_CONFIG_PARMS *flash, U32 cs_addr);
U32 nand_samsung_erase(T_FLASH_CONFIG_PARMS *flash, U32 addr, U32 length);
U32 nand_samsung_write(T_FLASH_CONFIG_PARMS *flash, U32 dst_addr, U32 src_addr, U32 size, T_write_req_flags* flags_ptr);
U32 nand_samsung_read(T_FLASH_CONFIG_PARMS *flash, U32 dst_addr, U32 src_addr, U32 size);

/* DRV helpers for diagnostics */
void nand_set_addr_cs_8(U8 cs);
void nand_set_addr(U8 cs);
void nand8_get_id (U16 * pMaker, U16 * pId);
void nand16_get_id (U16 * p_maker, U16 * p_id);
S32 nand8_flash_reset();
S32 nand16_flash_reset();
int nand16_is_ready ();
#endif /* NAND_SAMSUNG_DRV_H */
/*==== END OF FILE ===========================================================*/

