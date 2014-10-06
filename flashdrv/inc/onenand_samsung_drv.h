
/*-----------------------------------------------------------------------------
|  Project :  CSST
+------------------------------------------------------------------------------
|             Copyright 2005 Texas Instruments.
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
| Filename:   onenand_samsung_drv.h
| Author:     PSI
| Purpose:    header file for ONENAND samsung flash driver
|
+----------------------------------------------------------------------------*/
/*==== DECLARATION CONTROL =================================================*/
#ifndef ONENAND_SAMSUNG_DRV_H
#define ONENAND_SAMSUNG_DRV_H

/*==== INCLUDES ============================================================*/
/*=====================================================================*/
/*
BSA - BufferRAM Sector Address
BSC - BufferRAM Sector Count

FBA - Flash Block Address
FSA - NANDflash Sector Address
FPA - Flash Page Address

FCBA - Flash Copyback Block Address
FCPA - Flash Copyback Page Address
FCSA - Flash Copyback Sector Address

DFS - Device Flash core Select
DBS - Device BufferRAM Select
*/

/*==== CONSTS ==============================================================*/

#define ONLD_MAIN_SIZE              512
#define ONLD_SPARE_SIZE             16
#define ONLD_WINCE_SPARE_SIZE       8

#define DATA_RAM_ADDR_LINES         12
#define DATA_RAM_BASE_ADDR          0x00000400

#define MAIN_AREA                   0
#define SPARE_AREA                  1

#define ONENAND_SECTOR_SIZE         512
#define ONENAND_MAX_PAGES           64
#define ONENAND_MAX_SECTORS         4
#define ONENAND_PAGE_SIZE           (ONENAND_SECTOR_SIZE * ONENAND_MAX_SECTORS)

#define BLOCKSIZE_ONENAND           (ONENAND_SECTOR_SIZE * ONENAND_MAX_SECTORS * ONENAND_MAX_PAGES)
#define ONENAND_1GB_BLKCNT          0x0400
#define ONENAND_2GB_BLKCNT          0x0800

#define ONENAND_BLOCK_ADDRESS_MASK  0xFFFE0000
#define ONENAND_PAGE_ADDRESS_MASK   0xFFFFF800
#define ONENAND_SECTOR_ADDRESS_MASK 0xFFFFFE00
#define ONENAND_BLOCK_OFFSET_MASK   0x0001FFFF
#define ONENAND_PAGE_OFFSET_MASK    0x000007FF
#define ONENAND_SECTOR_OFFSET_MASK  0x000001FF

#define DEVICE_16BIT                0x10
#define SAMSUNG_VENDOR_CODE         0xEC
#define KFM1G16Q2M                  0x30
#define KFM2G16Q2M                  0x40
#define BLOCK_COUNT_ONENAND         0x400

#define BOOT_RAM0_BSA               0x000
#define BOOT_RAM1_BSA               0x100
#define DATA_RAM0_BSA               0x800
#define DATA_RAM1_BSA               0xC00

#define DATA_RAM0_SEL               0x000
#define DATA_RAM1_SEL               0x001

#define VALID_BLK_MARK              0xFFFF
#define VALID_BLK_MARK_WINCE        0xFF

/*****************************************************************************/
/* OneNAND Base Address Definitions                                          */
/*****************************************************************************/
/* the byte order addressing is used */
#define ON_REG_BASE_ADDR(x)            ((x) + 0x0001E000)

/*****************************************************************************/
/* OneNAND Register Address Definitions                                      */
/*****************************************************************************/
#define ONLD_REG_MANUF_ID(x)        (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0000))
#define ONLD_REG_DEV_ID(x)          (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0002))
#define ONLD_REG_VER_ID(x)          (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0004))
#define ONLD_REG_DATABUF_SIZE(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0006))
#define ONLD_REG_BOOTBUF_SIZE(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0008))
#define ONLD_REG_BUF_AMOUNT(x)      (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x000A))
#define ONLD_REG_TECH(x)            (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x000C))

#define ONLD_REG_START_ADDR1(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0200))
#define ONLD_REG_START_ADDR2(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0202))
#define ONLD_REG_START_ADDR3(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0204))
#define ONLD_REG_START_ADDR4(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0206))
#define ONLD_REG_START_ADDR5(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0208))
#define ONLD_REG_START_ADDR6(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x020A))
#define ONLD_REG_START_ADDR7(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x020C))
#define ONLD_REG_START_ADDR8(x)     (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x020E))

#define ONLD_REG_START_BUF(x)       (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0400))

#define ONLD_REG_CMD(x)             (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0440))
#define ONLD_REG_SYS_CONF1(x)       (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0442))
#define ONLD_REG_SYS_CONF2(x)       (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0444))

#define ONLD_REG_CTRL_STAT(x)       (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0480))
#define ONLD_REG_INT(x)             (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0482))

#define ONLD_REG_ULOCK_START_BA(x)  (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x0498))
#define ONLD_REG_ULOCK_END_BA(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x049A))
#define ONLD_REG_WR_PROTECT_STAT(x) (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x049C))

#define ONLD_REG_ECC_STAT(x)        (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E00))
#define ONLD_REG_ECC_RSLT_MB0(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E02))
#define ONLD_REG_ECC_RSLT_SB0(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E04))
#define ONLD_REG_ECC_RSLT_MB1(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E06))
#define ONLD_REG_ECC_RSLT_SB1(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E08))
#define ONLD_REG_ECC_RSLT_MB2(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E0A))
#define ONLD_REG_ECC_RSLT_SB2(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E0C))
#define ONLD_REG_ECC_RSLT_MB3(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E0E))
#define ONLD_REG_ECC_RSLT_SB3(x)    (*(volatile U16*)(ON_REG_BASE_ADDR(x) + 0x1E10))

/*****************************************************************************/
/* OneNAND Register Masking values                                           */
/*****************************************************************************/
#define MASK_DFS                    0x8000
#define MASK_FBA                    0x7FFF
#define MASK_DBS                    0x8000
#define MASK_FCBA                   0x0EFF
#define MASK_FCPA                   0x00FC
#define MASK_FCSA                   0x0003
#define MASK_FPA                    0x00FC
#define MASK_FSA                    0x0003
#define MASK_BSA                    0x0F00
#define MASK_BSC                    0x0003

#define MASK_SBA                    0x7FFF
#define MASK_EBA                    0x7FFF

/*****************************************************************************/
/* OneNAND Command Set                                                       */
/*****************************************************************************/
#define ONLD_CMD_READ_PAGE          0x0000
#define ONLD_CMD_READ_SPARE         0x0013
#define ONLD_CMD_WRITE_PAGE         0x0080
#define ONLD_CMD_WRITE_SPARE        0x001A
#define ONLD_CMD_CPBACK_PRGM        0x001B
#define ONLD_CMD_ULOCK_NAND         0x0023
#define ONLD_CMD_LOCK_BLK           0x002A
#define ONLD_CMD_LOCK_TIGHT_BLK     0x002C
#define ONLD_CMD_ERASE_VERIFY		    0x0071
#define ONLD_CMD_ERASE_BLK          0x0094
#define ONLD_CMD_ERASE_MBLK			    0x0095
#define ONLD_CMD_ERASE_SUSPEND	  	0x00B0
#define ONLD_CMD_ERASE_RESUME		    0x0030
#define ONLD_CMD_RST_NF_CORE        0x00F0
#define ONLD_CMD_RESET              0x00F3
#define ONLD_CMD_OPT_ACCESS         0x0065

/*****************************************************************************/
/* OneNAND System Configuration1 Register Values                            */
/*****************************************************************************/
#define SYNC_READ_MODE              0x8000
#define ASYNC_READ_MODE             0x0000

#define BST_RD_LATENCY_8            0x0000      /*   N/A   */
#define BST_RD_LATENCY_9            0x1000      /*   N/A   */
#define BST_RD_LATENCY_10           0x2000      /*   N/A   */
#define BST_RD_LATENCY_3            0x3000      /*   min   */
#define BST_RD_LATENCY_4            0x4000      /* default */
#define BST_RD_LATENCY_5            0x5000
#define BST_RD_LATENCY_6            0x6000
#define BST_RD_LATENCY_7            0x7000

#define BST_LENGTH_CONT             0x0000      /* default */
#define BST_LENGTH_4WD              0x0200
#define BST_LENGTH_8WD              0x0400
#define BST_LENGTH_16WD             0x0600
#define BST_LENGTH_32WD             0x0800      /* N/A on spare */

#define CONF1_ECC_ON                0xFEFF
#define CONF1_ECC_OFF               0x0100      //(~CONF1_ECC_ON)   //0x0100

#define RDY_POLAR                   0x0080
#define INT_POLAR                   0x0040
#define IOBE_ENABLE                 0x0020

#define BWPS_UNLOCKED               0x0001

/*****************************************************************************/
/* OneNAND Controller Status Register Values                                 */
/*****************************************************************************/
#define CTRL_ONGO                   0x8000
#define LOCK_STATE                  0x4000
#define LOAD_STATE                  0x2000
#define PROG_STATE                  0x1000
#define ERASE_STATE			            0x0800
#define ERROR_STATE			            0x0400
#define FAULT_CHECK                 0x4000
#define SUSPEND_STATE		            0x0200
#define RESET_STATE			            0x0080
#define OTPL_STATE			            0x0040
#define TIME_OUT                    0x0001

#define PROG_LOCK			              (LOCK_STATE | PROG_STATE | ERROR_STATE)
#define ERASE_LOCK			            (LOCK_STATE | ERASE_STATE | ERROR_STATE)
#define PROG_FAIL		  	            (PROG_STATE | ERROR_STATE)
#define ERASE_FAIL			            (ERASE_STATE | ERROR_STATE)

/*****************************************************************************/
/* OneNAND Interrupt Status Register Values                                  */
/*****************************************************************************/
#define INT_MASTER                  0x8000
#define INT_CLEAR                   0x0000

#define INT_READ                    0x0080
#define INT_WRITE                   0x0040
#define INT_ERASE                   0x0020
#define INT_RESET                   0x0010

#define PEND_INT                    (INT_MASTER)
#define PEND_READ                   (INT_MASTER | INT_READ)
#define PEND_WRITE                  (INT_MASTER | INT_WRITE)
#define PEND_ERASE                  (INT_MASTER | INT_ERASE)
#define PEND_RESET                  (INT_MASTER | INT_RESET)


/*****************************************************************************/
/* OneNAND Write Protection Status Register Values                           */
/*****************************************************************************/
#define UNLOCKED_STAT                0x0004
#define LOCKED_STAT                  0x0002
#define LOCK_TIGHTEN_STAT            0x0001

/*****************************************************************************/
/* OneNAND Main Buffer Address                                               */
/*****************************************************************************/
#define ONLD_BT_MB0_ADDR(x)         ((x) + 0x00000)
#define ONLD_BT_MB1_ADDR(x)         ((x) + 0x00200)
#define ONLD_DT_MB00_ADDR(x)        ((x) + 0x00400)
#define ONLD_DT_MB01_ADDR(x)        ((x) + 0x00600)
#define ONLD_DT_MB02_ADDR(x)        ((x) + 0x00800)
#define ONLD_DT_MB03_ADDR(x)        ((x) + 0x00A00)
#define ONLD_DT_MB10_ADDR(x)        ((x) + 0x00C00)
#define ONLD_DT_MB11_ADDR(x)        ((x) + 0x00E00)
#define ONLD_DT_MB12_ADDR(x)        ((x) + 0x01000)
#define ONLD_DT_MB13_ADDR(x)        ((x) + 0x01200)

#define ONLD_BT_SB0_ADDR(x)         ((x) + 0x10000)
#define ONLD_BT_SB1_ADDR(x)         ((x) + 0x10010)
#define ONLD_DT_SB00_ADDR(x)        ((x) + 0x10020)
#define ONLD_DT_SB01_ADDR(x)        ((x) + 0x10030)
#define ONLD_DT_SB02_ADDR(x)        ((x) + 0x10040)
#define ONLD_DT_SB03_ADDR(x)        ((x) + 0x10050)
#define ONLD_DT_SB10_ADDR(x)        ((x) + 0x10060)
#define ONLD_DT_SB11_ADDR(x)        ((x) + 0x10070)
#define ONLD_DT_SB12_ADDR(x)        ((x) + 0x10080)
#define ONLD_DT_SB13_ADDR(x)        ((x) + 0x10090)


/*****************************************************************************/
/*ONENAND Erase Structure */
/*****************************************************************************/

typedef struct
{
    U32 last_erased_block;
    U8 erased_valid;
} T_NAND_PARAMS;

/*---------------------- Functions in the Driver---------------*/
S32 one_nandWrite(U32 bs_addr, U16 blk_num, U16 pg_num, U16 sector_num,
                  U16 nscts, U16 *buffer, U8 mn_sp, U32 access_size);
S32 OneNAND_eraseBlock(U32 nBAddr, U16 blk_num);
S32 unlock_OneNAND(U32 ba, U16 start_addr, U16 end_addr);
S32 one_nandRead(U32 bs_addr, U16 blk_num, U16 pg_num, U16 sector_num,
                 U16 nscts, U16 *buffer, U8 mn_sp, U32 access_size);
S32 OneNAND_invalidBlock(U32 ba, U16 blk_num);
//U32  flash_check_bad_blocks(T_FLASH_CONFIG_PARMS *flash,
//                            T_BAD_BLOCK_ELEMENT **bad_blocks,
//                            U32 src_addr,
//                            U32 size);


/*==== EXPORTS =============================================================*/

extern U32 onenand_flash_init(T_driver_config * config, char ** result);
extern U32 onenand_flash_erase(T_driver_config * config, char ** result, U64 addr, U64 length);
extern U32 onenand_flash_write(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more);
extern U32 onenand_flash_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
extern U32 onenand_flash_deinit(T_driver_config * config, char ** result);
extern U32 onenand_flash_get_info(T_driver_config * config, T_driver_info * info);

#endif
/*==== END OF FILE ===========================================================*/

