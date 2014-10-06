/**
 * @file onenand_drv.c
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

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES ==============================================================*/
#include "types.h"
#include "flash_drv.h"
#include "onenand_samsung_drv.h"
#if 0
#include "wince.h"
#endif

/*= === MACROS ================================================================*/
//#define LOG_INFO
#define LOG_ERRORS

#ifdef LOG_INFO
#define INFO(FORMAT)                  { onenand->dbg_printf("DRV INFO: "FORMAT); }
#define INFO1(FORMAT, P1)             { onenand->dbg_printf("DRV INFO: "FORMAT, P1); }
#define INFO2(FORMAT, P1, P2)         { onenand->dbg_printf("DRV INFO: "FORMAT, P1, P2); }
#define INFO3(FORMAT, P1, P2, P3)     { onenand->dbg_printf("DRV INFO: "FORMAT, P1, P2, P3); }
#define INFO4(FORMAT, P1, P2, P3, P4) { onenand->dbg_printf("DRV INFO: "FORMAT, P1, P2, P3, P4); }
#else
#define INFO(FORMAT)                 
#define INFO1(FORMAT, P1)            
#define INFO2(FORMAT, P1, P2)        
#define INFO3(FORMAT, P1, P2, P3)    
#define INFO4(FORMAT, P1, P2, P3, P4)    
#endif

#ifdef LOG_ERRORS
#define ERROR(FORMAT)                { onenand->dbg_printf("DRV ERROR: "FORMAT); }
#define ERROR1(FORMAT, P1)           { onenand->dbg_printf("DRV ERROR: "FORMAT, P1); }
#define ERROR2(FORMAT, P1, P2)       { onenand->dbg_printf("DRV ERROR: "FORMAT, P1, P2); }
#define ERROR3(FORMAT, P1, P2, P3)   { onenand->dbg_printf("DRV ERROR: "FORMAT, P1, P2, P3); }
#else
#define ERROR(FORMAT)                
#define ERROR1(FORMAT, P1)           
#define ERROR2(FORMAT, P1, P2)       
#define ERROR3(FORMAT, P1, P2, P3)   
#endif


#define ONENAND_FLASH_READ(addr)        (*(volatile U16 *) (addr))
#define ONENAND_FLASH_WRITE(addr, data) (*(volatile U16 *) (addr)) = (data)

#define ENABLE 1
#define DISABLE 0

#define DRV_FILE_FORMATS(ARG) ((ARG) & (DRVFLAG_NAND_ROM_CODE_FORMAT | \
                                        DRVFLAG_NAND_POSTPROC_FORMAT | \
                                        DRVFLAG_NAND_NONPOSTPROC_FORMAT))

#define GET_MAIN_BUF_ADDR(bs_addr,sc_nm,buf_sel)\
        (ONLD_DT_MB00_ADDR(bs_addr) + (buf_sel << 11) + ((sc_nm) << 9))

#define GET_SPARE_BUF_ADDR(bs_addr,sc_nm,buf_sel)\
        (ONLD_DT_SB00_ADDR(bs_addr) + (buf_sel << 6) + ((sc_nm) << 4))

#define GET_ONLD_INT_STAT(x,a)  ((U16)(ONLD_REG_INT(x) & (U16)(a)))
#define GET_ONLD_CTRL_STAT(x,a) ((U16)(ONLD_REG_CTRL_STAT(x) & (U16)(a)))

#define GET_ONLD_REG_WR_PROTECT_STAT(x,a) ((U16)(ONLD_REG_WR_PROTECT_STAT(x) & (U16)(a)))

/* These defines MUST be aligned with the values in param_map.xml. Theses
values are used to interpret the driver configuration parameters. */

#define PAGE_SIZE_H                      0x00000004
#define BLOCK_SIZE                       0x00000005
#define BLOCK_COUNT                      0x00000006
#define BUS_WIDTH                        0x00000007

#define ONENAND_BLOCK_MASK               0x03FFC000
#define ONENAND_PAGE_MASK                0x00003E00
#define ONENAND_WORD_MASK                0x000001FE
#define ONENAND_PAGE_OFF_MASK            (NAND_WORD_MASK|NAND_PAGE_MASK)

/* The define to enable checking of Bad blocks in skipped areas */
#define SKIP_BAD_BLOCKS_IN_EMPTY 1

/* Should we check for ECC correctable data and attempt to fix it? */
#define SANE_ECC_READ 1

/* Should the reads for WINCE POST PROC Return with Spare area info also? */
#undef READ_WITH_RETURN_SPARE

/* Should Read Skip freshly erased blocks? */
#undef READ_SKIP_EMPTY_BLOCKS

/* Should Read mark failed reads as bad blocks? */
#undef MARK_BAD_BLOCKS_DURING_READS

/*==== TYPES =================================================================*/

typedef struct
{
  U32 address;
  U32 offset;
  U8  data[1]; // Allocate to real size
} T_rw; /* Struct to make sure it is aligned for 16 bit access */

typedef struct
{
    /* NOTE: cs_address provides two functionality:
    * a) provide us with base address
    * b) host always gives data w.r.t cs_addr
    */
    U32 cs_addr;
    U32 sector_size;
    U32 num_sector;
    U16 block_num;
    U16 page_num;
    U16 sec_num;
    U32 block_size;
    U32 block_count;
    U32 page_count;
    U32 page_size;
    U32 sectors_per_page;
    U32 total_size;
    U32 bus_width;
    U32 w_bad_block_count;
    U32 r_bad_block_count;
    U32 e_bad_block_count;
    U32 last_erased_block;
    U32 last_write_block;
    U16 device_code;
    U16 vendor_code;
    U8 spare_type;

    /*the Flash count are the actual sizes of the flash */
    U32 flash_blkcnt;

    /* callback functions */
    T_driver_wait_microsec  wait_microsec;
    T_driver_malloc         malloc;
    T_driver_free           free;
    T_driver_dbg_printf     dbg_printf;    
    T_driver_send_info      send_info;     
    T_driver_send_status    send_status; 

    U16 spare_buffer[8];  /* max 16 bytes for handling rom code requirements */
    U32 store_onenand[8]; /* max 16 bytes for handling rom code requirements */
    BOOLEAN bberase;
    T_rw * rw;
} T_ONENAND_PARAMS;

#if 0
typedef struct
{
    U8  reserved1[4];
    U8  oemReserved;
    U8  badBlock;
    U8  reserved2[2];
} WINCE_SECTOR_INFO;
#endif

/*bad block rewind logic*/
typedef struct bad_block_rewind
{
    U16 bb_block_num;
    U32 valid_block_num;
    U16 page_num;
    U16 sector_num;
    U32 size;
    U32 src_addr;
    U32 base_addr;
} BB_STORE_ONENAND_DATA;

/*==== CONSTS ================================================================*/
#ifndef _MSC_VER
#pragma DATA_SECTION(driver_if,".consttbl");
#endif
const T_driver_if driver_if = 
{
  {
    OMAPFLASH_DRIVER_HEADER_STRING_V7,
    "SAMSUNG ONENAND",
    CTRL_Z
  },
  {
    &driver_if,
    onenand_flash_init,
    onenand_flash_read,
    onenand_flash_write,
    onenand_flash_erase,
    onenand_flash_deinit,
    onenand_flash_get_info
  }
};

typedef enum
{
  C_ADDRESS,
  C_BBERASE,
  C_WIDTH,
  C_BLOCKS,
  C_PAGESPBLOCK,
  C_SECTORSPPAGE,
  C_SECTORSIZE
} T_driver_setup_index;

const T_driver_setup_const setup_const[] =
{
  { "address", DEFAULT, TRUE  },
  { "bberase", DEFAULT, TRUE  },
  { "w",       DEFAULT, FALSE },
  { "b",       DEFAULT, FALSE },
  { "ppb",     DEFAULT, FALSE },
  { "spp",     DEFAULT, FALSE },
  { "ssize",   DEFAULT, FALSE },
  { "",        DEFAULT, FALSE }
};

T_driver_setup_var setup_var[sizeof(setup_const) / sizeof(T_driver_setup_const)];

/*==== PRIVATE FUNCTIONS =====================================================*/

void resolve_flashaddress(T_ONENAND_PARAMS *onenand, U32 flash_address);

/*==== PUBLIC FUNCTIONS ======================================================*/
/**************************************************************************
* OneNAND_init - enables the interrupt mode
* RETURNS:  N/A.
*/
S32 OneNAND_flash_reset(U32 ba)
{
    /* IO Buffer Enable for NAND interrupt enable/disable */
    ONLD_REG_SYS_CONF1(ba) |= IOBE_ENABLE;

    /* INT Stat Reg Clear */
    ONLD_REG_INT(ba) = (U16)INT_CLEAR;

    ONLD_REG_CMD(ba) = (U16)ONLD_CMD_RESET;

    if(GET_ONLD_INT_STAT(ba, PEND_RESET) != (U16)PEND_RESET)
    {
        return FLASH_DRV_ERROR;
    }

    return FLASH_DRV_SUCCESS;
}

/**************************************************************************
* unlock_OneNAND - unlocks the blocks from the start block address to the
*                   end block address
* takes in the start block address and end block address
* RETURNS:  N/A.
*/

S32 unlock_OneNAND(U32 ba, U16 start_addr, U16 end_addr)
{

    /* check if the block number is less than 512, if yes then there is
    only one flash core */

    /* configure the start address */
    ONLD_REG_ULOCK_START_BA(ba) = (U16)(start_addr & MASK_SBA);

    /* configure the end address */
    ONLD_REG_ULOCK_END_BA(ba) = (U16)(end_addr & MASK_EBA);

    /* INT Stat Reg Clear */
    ONLD_REG_INT(ba) = (U16)INT_CLEAR;

    /* issue unlock command */
    ONLD_REG_CMD(ba) = (U16)ONLD_CMD_ULOCK_NAND;

    while (GET_ONLD_INT_STAT(ba, INT_MASTER) != (U16)INT_MASTER)
    {
        /* Wait until device ready */
    }

    return FLASH_DRV_SUCCESS;

}



/**************************************************************************
* OneNAND_get_id - reads the OneNAND Flash Device manufacturer id and
*                  device id.
*
* RETURNS:  N/A
*/
void OneNAND_get_id(U16 *manufacturer, U16 *device_id, U32 base_addr)
{
    *manufacturer = ONLD_REG_MANUF_ID(base_addr);
    *device_id = ONLD_REG_DEV_ID(base_addr);
}

/**************************************************************************
* _ReadMain - reads main data from the OneNAND buffer and puts into the SDRAM buffer
*             this takes in
*             sdram buffer pointer
*             OneNAND buffer pointer
*             number of sectors to be read.
* RETURNS: N/A
*/
/* implemented to read even the odd number of bytes, the access size
should be the bytes to be read...*/

void _ReadMain(U16 *pBuf, U16 *pTgt, U32 nScts, U32 access_size)
{
    U32  nCnt;
    U16  temp_data;
    U16  read_ram;

    U32 read_count = (access_size/2);

    /* copy the data from the buffer to the sdram buffer */
    for (nCnt = 0; nCnt < (read_count) * nScts; nCnt++)
    {
        temp_data = ONENAND_FLASH_READ(pTgt);

        *pBuf = temp_data;
        pTgt++;
        pBuf++;
    }

    /*are we suppose to read odd number of bytes?*/
    if( (access_size%2) != 0 )
    {
        temp_data = ONENAND_FLASH_READ(pTgt);

        /*Mask the MSB bytes, the valid bytes are only LSB*/
        temp_data &= 0x00FF;

        read_ram = *pBuf;
        read_ram &= 0xFF00;

        *pBuf = temp_data;
        *pBuf |= read_ram;

        pTgt++;
        pBuf++;

    }

}


/**************************************************************************
* _ReadSpare - reads spare data from the OneNAND buffer and puts into the SDRAM buffer
*             this takes in
*             sdram buffer pointer
*             OneNAND buffer pointer
*             number of sectors to be read.
* RETURNS: N/A
*/
void _ReadSpare(U16 *pBuf, U16 *pTgt, U32 nScts)
{
    U32  nCnt;
    U16  temp_data;
    /* copy spare data to sdram buffer*/

    for (nCnt = 0; nCnt < (ONLD_SPARE_SIZE/2) * nScts; nCnt++)
    {
        temp_data = ONENAND_FLASH_READ(pTgt);

        *pBuf = temp_data;
        pTgt++;
        pBuf++;
    }

}

/**************************************************************************
* one_nandCopyback - performs the OneNAND copyback opertaion
*
* Following are the steps to implement the copyback operation.
* 1.Data is read from the NAND Array using Flash Block Address (FBA), Flash Page Address (FPA)
*   and Flash Sector Address (FSA). FBA, FPA, and FSA identify the source address to read data
*   from NAND Flash array.
* 2.The BufferRAM Sector Count (BSC) and BufferRAM Sector Address (BSA) identifies how many
*   sectors and the location of the sectors in DataRAM that are used.
* 3.The destination address in the NAND Array is written using the Flash Copy-Back Block Address
*   (FCBA),Flash Copy-Back Page Address (FCPA), and Flash Copy-Back Sector Address (FCSA).
* 4.The Copy-Back Program command is issued to start programming.
*/
S32 one_nandCopyback(U32 bs_addr, U16 src_blk_num, U16 src_pg_num, U16 src_sec_num,
                     U16 nscts, U16 dest_blk_num, U16 dest_pg_num, U16 dest_sec_num)
{

    U16 sector_always_zero = 0;
    U16 nBSA;

    /* block address */
    ONLD_REG_START_ADDR1(bs_addr) = (U16)((src_blk_num & MASK_FBA));

    /* flash page address, flash sector address*/
    ONLD_REG_START_ADDR8(bs_addr) = (U16)(((src_pg_num << 2) & MASK_FPA) | (src_sec_num & MASK_FSA));

    /* block address */
    ONLD_REG_START_ADDR3(bs_addr) = (U16)((dest_blk_num & MASK_FCBA));

    /* flash page address, flash sector address*/
    ONLD_REG_START_ADDR4(bs_addr) = (U16)(((dest_pg_num << 2) & MASK_FCPA) | (dest_sec_num & MASK_FCSA));

    /* buffer start address, buffer sector count */
    nBSA = ((DATA_RAM0_BSA) | ((sector_always_zero) << 8));

    ONLD_REG_START_BUF(bs_addr) = (U16)((nBSA & MASK_BSA) | (nscts & MASK_BSC));

    /* INT Stat Reg Clear */
    ONLD_REG_INT(bs_addr) = (U16)INT_CLEAR;

    /* Main Write Command issue */
    ONLD_REG_CMD(bs_addr) = (U16)ONLD_CMD_CPBACK_PRGM;


    while(GET_ONLD_INT_STAT(bs_addr, PEND_WRITE) != (U16)PEND_WRITE)
    {
        /* Wait until device ready */
        /* Write Protection Error Check */
        if(GET_ONLD_CTRL_STAT(bs_addr, LOCK_STATE) == LOCK_STATE)
        {
            return FLASH_DRV_ERROR;
        }
    }

    /* Write Operation Error Check */
    if(GET_ONLD_CTRL_STAT(bs_addr, ERROR_STATE) == ERROR_STATE)
    {
        return FLASH_DRV_ERROR;
    }

    /* write success */
    return FLASH_DRV_SUCCESS;

}



/**************************************************************************
* one_nandRead - performs the OneNAND read operation
*
* this function carries out the necessary steps for a read operation
*              parameters:
*                       block number Eg:0-511
*                       page number  Eg:0-63
*                       sector number  Eg:0-3
*                       number of sectors to be read
*                       pointer to the SDRAM buffer
*                       flag which specifies whether the read is from main
*                           or spare area
* RETURNS:  FLASH_DRV_SUCCESS - Data Read success
*           ONLD_READ_ERROR | ECC result code
*                      - Data intigrity fault. (2bit ECC error)
*
*/

S32 one_nandRead(U32 bs_addr, U16 blk_num, U16 pg_num, U16 sector_num,
                 U16 nscts, U16 *buffer, U8 mn_sp, U32 access_size)
{
    U16 nBSA;
    U16 *dt_buf;

    /* fill in the block address */
    ONLD_REG_START_ADDR1(bs_addr) = (U16)((blk_num & MASK_FBA));

    /* fill in the flash page address and flash sector address */

    ONLD_REG_START_ADDR8(bs_addr) = (U16)(((pg_num << 2) & MASK_FPA) | (sector_num & MASK_FSA));

    /* buffer start address and buffer sector count */
    nBSA = ((DATA_RAM0_BSA) | ((sector_num) << 8));

    ONLD_REG_START_BUF(bs_addr) = (U16)((nBSA & MASK_BSA) | (nscts & MASK_BSC));

    /* INT Stat Reg Clear */
    ONLD_REG_INT(bs_addr) = (U16)INT_CLEAR;

    /* OneNAND Read CMD is issued */
    if(mn_sp == MAIN_AREA)
    {
        /* Page Read Command issue */
        ONLD_REG_CMD(bs_addr) = (U16)ONLD_CMD_READ_PAGE;
    }
    else if(mn_sp == SPARE_AREA)
    {
        /* Spare Read Command issue */
        ONLD_REG_CMD(bs_addr) = (U16)ONLD_CMD_READ_SPARE;
    }
    else
    {
        return FLASH_DRV_ERROR;
    }

    while(GET_ONLD_INT_STAT(bs_addr, PEND_READ) !=  (U16)PEND_READ)
    {
        /* Wait until device ready */
    }

    /*
    * Read the Control and Status Register to find out whether Read Operation
    * has gone into Error State */

    if (GET_ONLD_CTRL_STAT(bs_addr, FAULT_CHECK) == FAULT_CHECK)
    {
        return FLASH_DRV_ERROR;
    }

    if(mn_sp == MAIN_AREA)
    {
        /* get main buffer address */
        dt_buf = (U16 *)GET_MAIN_BUF_ADDR(bs_addr,sector_num,DATA_RAM0_SEL);
        /* Memcopy for main data */
        _ReadMain(buffer, dt_buf, nscts, access_size);

    }
    else if(mn_sp == SPARE_AREA)
    {
        /* get spare buffer address */
        dt_buf = (U16 *)GET_SPARE_BUF_ADDR(bs_addr,sector_num,DATA_RAM0_SEL);
        /* Memcopy for spare data */
        _ReadSpare(buffer, dt_buf, nscts);
    }

    /* read successful*/
    return FLASH_DRV_SUCCESS;

}

/**************************************************************************
* _WriteMain - writes main data from the SDRAM buffer to the OneNAND buffer
*             this takes in
*             sdram buffer pointer
*             OneNAND buffer pointer
*             number of sectors to be written.
* RETURNS: N/A
***************************************************************************/
void _WriteMain(U16 *pBuf, U16 *pTgt, U16 nScts, U32 access_size)
{
    U32  nCnt;
    U16 temp_data;

    U32 read_count = (access_size/2);

    for (nCnt = 0; nCnt < (read_count) * nScts; nCnt++)
    {
        temp_data = *pBuf;

        ONENAND_FLASH_WRITE(pTgt, temp_data);

        pBuf++;
        pTgt++;
    }

    /*are we suppose to write odd number of bytes?*/
    if( (access_size%2) != 0 )
    {
        temp_data = *pBuf;

        /*mask the msb and only write the lsb data*/
        temp_data |= 0xFF00;
        ONENAND_FLASH_WRITE(pTgt, temp_data);

        pBuf++;
        pTgt++;
    }

}

/**************************************************************************
* _WriteSpare - writes spare data from the SDRAM buffer to the OneNAND buffer
*             this takes in
*             sdram buffer pointer
*             OneNAND buffer pointer
*             number of sectors to be written.
* RETURNS: N/A
***************************************************************************/
void _WriteSpare(U16 *pBuf, U16 *pTgt, U16 nScts)
{
    U32 nCnt;
    U16 temp_data;

    for (nCnt = 0; nCnt < (U32)((ONLD_SPARE_SIZE/2) * nScts); nCnt++)
    {
        temp_data = *pBuf;

        ONENAND_FLASH_WRITE(pTgt, temp_data);

        pBuf++;
        pTgt++;
    }

}


/**************************************************************************
* one_nandWrite - This function write data into OneNAND flash
*              parameters:
*                       block number
*                       page number
*                       sector number
*                       number of sectors to be written
*                       pointer to the SDRAM buffer which has the data to be written
*                       flag which specifies whether the write is to main
*                           or spare area
* RETURNS:  FLASH_DRV_SUCCESS - Data write success
*           ONLD_WR_PROTECT_ERROR - Attempt to write at Locked Area
*           ONLD_WRITE_ERROR - Data write failure
*
***************************************************************************/

S32 one_nandWrite(U32 bs_addr, U16 blk_num, U16 pg_num, U16 sector_num,
                  U16 nscts, U16 *buffer, U8 mn_sp, U32 access_size)
{
    U16 nBSA;
    U16 *dt_buf;
    U16 sector_always_zero = 0; //always zero


    if(mn_sp == MAIN_AREA)
    {
        /* get main buffer address */
        dt_buf = (U16*)GET_MAIN_BUF_ADDR(bs_addr,sector_always_zero,DATA_RAM0_SEL);
        /* Memcopy for main data */
        _WriteMain(buffer, dt_buf, nscts, access_size);
    }
    else if(mn_sp == SPARE_AREA)
    {
        /* get spare buffer address */
        dt_buf = (U16*)GET_SPARE_BUF_ADDR(bs_addr,sector_always_zero,DATA_RAM0_SEL);
        /* Memcopy for spare data */
        _WriteSpare(buffer, dt_buf, nscts);
    }
    else
    {
        return FLASH_DRV_ERROR;
    }

    /* block address */
    ONLD_REG_START_ADDR1(bs_addr) = (U16)((blk_num & MASK_FBA));

    /* flash page address, flash sector address*/
    ONLD_REG_START_ADDR8(bs_addr) = (U16)(((pg_num << 2) & MASK_FPA) | (sector_num & MASK_FSA));

    /* buffer start address, buffer sector count */
    nBSA = ((DATA_RAM0_BSA) | ((sector_always_zero) << 8));

    ONLD_REG_START_BUF(bs_addr) = (U16)((nBSA & MASK_BSA) | (nscts & MASK_BSC));

    /* INT Stat Reg Clear */
    ONLD_REG_INT(bs_addr) = (U16)INT_CLEAR;

    /* ONLD Write CMD is issued */
    if(mn_sp == MAIN_AREA)
    {
        /* Main Write Command issue */
        ONLD_REG_CMD(bs_addr) = (U16)ONLD_CMD_WRITE_PAGE;
    }
    else
    {
        /* Spare Write Command issue */
        ONLD_REG_CMD(bs_addr) = (U16)ONLD_CMD_WRITE_SPARE;
    }

    while(GET_ONLD_INT_STAT(bs_addr, PEND_WRITE) != (U16)PEND_WRITE)
    {
        /* Wait until device ready */
        /* Write Protection Error Check */
        if(GET_ONLD_CTRL_STAT(bs_addr, LOCK_STATE) == LOCK_STATE)
        {
            return FLASH_DRV_ERROR;
        }
    }

    /* Write Operation Error Check */
    if (GET_ONLD_CTRL_STAT(bs_addr, ERROR_STATE) == ERROR_STATE)
    {
        return FLASH_DRV_ERROR;
    }

    /* write success */
    return FLASH_DRV_SUCCESS;
}

/**************************************************************************
* OneNAND_eraseBlock - This function erases the block specified in OneNAND flash
*              parameters:
*                       block number
* RETURNS:  FLASH_DRV_SUCCESS - Data erase success
*           ONLD_WR_PROTECT_ERROR - Attempt to erase a Locked Area
*           ONLD_ERASE_ERROR - Data erase failure
*
*/
S32 OneNAND_eraseBlock(U32 nBAddr, U16 blk_num)
{

    /* Block Number Set */
    ONLD_REG_START_ADDR1(nBAddr) = (U16)((blk_num & MASK_FBA));

    /* INT Stat Reg Clear */
    ONLD_REG_INT(nBAddr) = (U16)INT_CLEAR;

    /* ONLD Erase CMD is issued */

    ONLD_REG_CMD(nBAddr)         = (U16)ONLD_CMD_ERASE_BLK;

    while (GET_ONLD_INT_STAT(nBAddr, PEND_ERASE) != (U16)PEND_ERASE)
    {
        /* Wait until device ready */
        /* Write Protection Error Check */
        if (GET_ONLD_CTRL_STAT(nBAddr, LOCK_STATE) == LOCK_STATE)
        {
            return FLASH_DRV_ERROR;
        }
    }

    /* Erase Operation Error Check */
    if (GET_ONLD_CTRL_STAT(nBAddr, ERROR_STATE) == ERROR_STATE)
    {
        return FLASH_DRV_ERROR;
    }

    /* erase suucess */
    return FLASH_DRV_SUCCESS;
}

/**************************************************************************
* OneNAND_invalidBlock - This function checks whether the block specified
*                        in OneNAND flash is a valid/invalid block.
*              parameters:
*                       block number
* RETURNS: ONLD_READ_ERROR: when there is error while reading
*          ONLD_INIT_BADBLOCK: when the block is identified as invalid
*          ONLD_INIT_GOODBLOCK:when the block is identified as good.
*/
S32 OneNAND_invalidBlock(U32 ba, U16 blk_num)
{

    U16 aSpare[ONLD_SPARE_SIZE / 2];
    U8 i;

    /* read the spare data of this block */
    for(i = 0; i<2; i++)
    {
        if(one_nandRead(ba,blk_num,i,0,1, aSpare, SPARE_AREA,0) != FLASH_DRV_SUCCESS)
        {
            return FLASH_DRV_ERROR;
        }
        /* check for the presence of data ather than 0xffff in the first word*/
        /* if it is, then the block is invalid */
        if(aSpare[0] != (U16)VALID_BLK_MARK)
        {
            /* return this is an invalid block */
            return FLASH_DRV_ERROR;
        }
    }
    /* this is a good block */
    return (FLASH_DRV_SUCCESS);

}

#if 0
/**************************************************************************
* OneNAND_invalidBlock_Wince - This function checks whether the block specified
*                        in OneNAND flash is a valid/invalid block.
*              parameters:
*                       block number
* RETURNS: ONLD_READ_ERROR: when there is error while reading
*          ONLD_INIT_BADBLOCK: when the block is identified as invalid
*          ONLD_INIT_GOODBLOCK:when the block is identified as good.
*/
S32 OneNAND_invalidBlock_Wince(U32 ba, U16 blk_num)
{

    U8 aSpare[ONLD_SPARE_SIZE];
    U8 i;
    /* read the spare data of this block */
    for(i = 0; i<2; i++)
    {
        if(one_nandRead(ba,blk_num,i,0,1, (U16*)aSpare, SPARE_AREA,0) != FLASH_DRV_SUCCESS)
        {
            return FLASH_DRV_ERROR;
        }
        /* check for the presence of data ather than 0xffff in the first word*/
        /* if it is, then the block is invalid */
        if(aSpare[0] != VALID_BLK_MARK_WINCE)
        {
            /* return this is an invalid block */
            return (FLASH_DRV_ERROR);
        }
    }
    /* this is a good block */
    return (FLASH_DRV_SUCCESS);

}
#endif

/**************************************************************************
* resolve_flashaddress: resolves the flash address to give the blocknumber, page number
*  and the sector number
*/
void resolve_flashaddress(T_ONENAND_PARAMS *onenand, U32 flash_address)
{
    //U32 temp;

    //onenand->block_num = (U16)(flash_address/BLOCKSIZE_ONENAND);

    //temp = (flash_address % BLOCKSIZE_ONENAND);

    //if(temp % (ONENAND_MAX_SECTORS * ONENAND_SECTOR_SIZE) == 0)
    //{
    //    onenand->page_num = (U16)(temp/(ONENAND_MAX_SECTORS * ONENAND_SECTOR_SIZE));
    //    onenand->sec_num = 0;
    //}
    //else
    //{
    //    onenand->page_num = (U16)(temp/(ONENAND_MAX_SECTORS * ONENAND_SECTOR_SIZE));
    //    onenand->sec_num = temp % (ONENAND_MAX_SECTORS * ONENAND_SECTOR_SIZE);
    //}


    onenand->block_num = (U16)(flash_address / onenand->block_size);
    onenand->page_num  = (U16)((flash_address & ONENAND_BLOCK_OFFSET_MASK) / onenand->page_size);
    onenand->sec_num   = (U16)((flash_address & ONENAND_PAGE_OFFSET_MASK) / onenand->sector_size);


}


/**************************************************************************
* mark_bad_block: marks the bad block of ONE NAND flash
*
* Parameters  : onenand_addr [IN] - Address to check the bad block
*
* Returns     : result_code
*/
static S32 mark_bad_block(T_ONENAND_PARAMS *onenand, U32 base_addr, U32 onenand_addr)
{
    U16 onld_spare[ONLD_SPARE_SIZE/2];
    S32 ret_val = FLASH_DRV_SUCCESS;
    U8  i;
    U32 block_num;

    onld_spare[0]=0xdead;

    block_num = (onenand_addr - base_addr) / onenand->block_size;

    /* Erase the block first.. */
    ret_val = OneNAND_eraseBlock(base_addr, (U16)block_num);

    /* we try to put in data now */
    for(i=0;i<2;i++)
    {
        ret_val = one_nandWrite(base_addr, (U16)block_num, i,0,1, onld_spare, SPARE_AREA,0);
        if(ret_val != FLASH_DRV_SUCCESS)
        {
            /*not able to write to the block*/
            break;
        }
    }
    return ret_val;
}

/*-----------------------------------------------------------------------------
| Function    : badblock_Copyback
+------------------------------------------------------------------------------
|*/
S32 badblock_Copyback(T_ONENAND_PARAMS *onenand, BB_STORE_ONENAND_DATA * store_onenand_data)
{
    U32 mark_bad;
    U16 temp_sec_num = 0;
    U16 temp_page_num = 0;
    S32 ret_val = FLASH_DRV_SUCCESS;

    /*goto the beginning of the block and start copying data to
    the current good block*/
    do
    {
        ret_val = one_nandCopyback(store_onenand_data->base_addr,
                                   store_onenand_data->bb_block_num,
                                   temp_page_num,
                                   temp_sec_num,
                                   1,
                                   onenand->block_num,
                                   temp_page_num,
                                   temp_sec_num);
        if(ret_val != FLASH_DRV_SUCCESS)
        {
            /*really a bad block. return saying error*/
            return FLASH_DRV_ERROR;
        }

        temp_sec_num++;
        if(temp_sec_num == 4)
        {
            temp_sec_num = 0;
            temp_page_num++;
        }

    } while( (temp_page_num  == store_onenand_data->page_num) && (temp_sec_num == store_onenand_data->sector_num) );

    mark_bad = store_onenand_data->bb_block_num * onenand->block_size;
    mark_bad += store_onenand_data->base_addr;
    /*if possible mark it as a bad block*/
    if(mark_bad_block(onenand, store_onenand_data->base_addr, mark_bad) != FLASH_DRV_SUCCESS)
    {
      onenand->send_info("FAILED TO MARK BLOCK AT %#08x AS BAD", mark_bad);
      //hang_line(__LINE__);
    }

    return ret_val;
} /*end of rewind logic*/

#if 0
/*-----------------------------------------------------------------------------
| Function    : Retrive_Wince_Spare_Info
+------------------------------------------------------------------------------
| Description :Extracts the Wince spare fileds from the image and fills
|               the Wince structure, to be written to the ONLD spare area
|
| Parameters  : source_addr - address from where the wince image spare
|                              bytes are taken
|               onld_spareinfo - Wince structure, the fields are filled
|                                from the image
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/

U8 Retrive_Wince_Spare_Info( U8* source_addr, T_ONLD_WINCE_SPARE_AREA_INFO* onld_spareinfo)
{

    T_WINCE_SECTOR_INFO* wince_sectorinfo;

    U16 rev2;
    U32 rev1;

    /*filling the wince sector info from memory (image file)*/
    wince_sectorinfo = ((T_WINCE_SECTOR_INFO*)(source_addr));

    /*Filling 16 byte Spare area info for ONENAND*/

    onld_spareinfo->badBlock = wince_sectorinfo->badBlock;

    rev1 = wince_sectorinfo->reserved1;
    onld_spareinfo->reserved1_1_0[0] = (U8)(rev1 & 0xFF);
    onld_spareinfo->reserved1_1_0[1] = (U8)((rev1 >> 8) & 0xFF);
    onld_spareinfo->reserved1_2      = (U8)((rev1 >> 16) & 0xFF);
    onld_spareinfo->reserved1_3      = (U8)((rev1 >> 24) & 0xFF);

    onld_spareinfo->int_ecc_sector[0] = 0xFF;
    onld_spareinfo->int_ecc_sector[1] = 0xFF;
    onld_spareinfo->int_ecc_sector[2] = 0xFF;

    onld_spareinfo->int_ecc_main[0] = 0xFF;
    onld_spareinfo->int_ecc_main[1] = 0xFF;
    onld_spareinfo->int_ecc_main[2] = 0xFF;

    onld_spareinfo->int_ecc_spare[0] = 0xFF;
    onld_spareinfo->int_ecc_spare[1] = 0xFF;

    onld_spareinfo->oemReserved = wince_sectorinfo->oemReserved;

    rev2 =    wince_sectorinfo->reserved2;
    onld_spareinfo->reserved2[0] = (U8)(rev2 & 0xFF);
    onld_spareinfo->reserved2[1] = (U8)((rev2 >> 8) & 0xFF);

    return (FLASH_DRV_SUCCESS);

}



U8 Retrieve_WinceImage_spare( T_ONLD_WINCE_SPARE_AREA_INFO* spare_data, U16* dest_address)
{

    WINCE_SECTOR_INFO* wince_sectorinfo;

    /* filling the wince sector info from memory (image file) */
    wince_sectorinfo = ((WINCE_SECTOR_INFO*)(dest_address));


    wince_sectorinfo->reserved1[0] = spare_data->reserved1_1_0[0];
    wince_sectorinfo->reserved1[1] = spare_data->reserved1_1_0[1];
    wince_sectorinfo->reserved1[2] = spare_data->reserved1_2;
    wince_sectorinfo->reserved1[3] = spare_data->reserved1_3;

    wince_sectorinfo->oemReserved = spare_data->oemReserved;

    wince_sectorinfo->badBlock = spare_data->badBlock;

    wince_sectorinfo->reserved2[0] = spare_data->reserved2[0];
    wince_sectorinfo->reserved2[1] = spare_data->reserved2[1];

    return (FLASH_DRV_SUCCESS);

}
#endif

/*-----------------------------------------------------------------------------
| Function    : onenand_init
+------------------------------------------------------------------------------
| Description : onenand Flash initialization
|
| Parameters  : flash [OUT] - Flash parms
|               addr  [IN]  - Address
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 onenand_flash_init(T_driver_config * config, char ** result)
{
    T_driver_setup_var *setup = get_setup_var();  
  
    T_ONENAND_PARAMS *onenand;

    /* Parse configuration string */

    if(drv_parse_config(get_setup_const(), setup, config->cstring, config) == FLASH_DRV_ERROR)
    {
      *result = "UNABLE TO FIND CONFIGURATION PARAMETER DURING INITIALIZATION";
      return FLASH_DRV_ERROR;
    }

    if(setup[C_WIDTH].valid || setup[C_BLOCKS].valid || setup[C_PAGESPBLOCK].valid || setup[C_SECTORSPPAGE].valid || setup[C_SECTORSIZE].valid)
    {
      if(!setup[C_WIDTH].valid || !setup[C_BLOCKS].valid || !setup[C_PAGESPBLOCK].valid || !setup[C_SECTORSPPAGE].valid || !setup[C_SECTORSIZE].valid)
      {
        *result = "CONFIGURATION PARAMETER MISSING - ALL PARAMETERS SPECIFIED?";
        return FLASH_DRV_ERROR;
      }
    }
    
    /* Initialize */    
    
    *result          = NULL;
    config->data     = config->drv_malloc(sizeof(T_ONENAND_PARAMS));
    onenand          = (T_ONENAND_PARAMS *)config->data;
    onenand->rw      = NULL;
    onenand->cs_addr = setup[C_ADDRESS].value;
    onenand->bberase = setup[C_BBERASE].value ? TRUE : FALSE;


    /* Update callback functions pointers */

    onenand->malloc         = config->drv_malloc;
    onenand->free           = config->drv_free;
    onenand->wait_microsec  = config->wait_microsec;
    onenand->dbg_printf     = config->dbg_printf;
    onenand->send_info      = config->send_info;     
    onenand->send_status    = config->send_status; 

    OneNAND_get_id(&onenand->vendor_code, &onenand->device_code, onenand->cs_addr);

    onenand->send_info("ONENAND VENDOR %#04x DEVICE %#04x", onenand->vendor_code, onenand->device_code);

    /* Check for supported chips */
    
    switch (onenand->device_code)
    {
      case KFM1G16Q2M:/*buswidth to be changed in the driver.xml*/
          onenand->bus_width        = DEVICE_16BIT;
          onenand->flash_blkcnt     = ONENAND_1GB_BLKCNT;
          onenand->block_count      = ONENAND_1GB_BLKCNT;
          onenand->page_count       = ONENAND_MAX_PAGES;
          onenand->sector_size      = ONENAND_SECTOR_SIZE;
          onenand->sectors_per_page = ONENAND_MAX_SECTORS;
          break;
      case KFM2G16Q2M:/*buswidth to be changed in the driver.xml*/
          onenand->bus_width        = DEVICE_16BIT;
          onenand->flash_blkcnt     = ONENAND_2GB_BLKCNT;
          onenand->block_count      = ONENAND_2GB_BLKCNT;
          onenand->page_count       = ONENAND_MAX_PAGES;
          onenand->sector_size      = ONENAND_SECTOR_SIZE;
          onenand->sectors_per_page = ONENAND_MAX_SECTORS;
          break;

      default:
          if(!setup[C_WIDTH].valid)
          {
            *result = "UNKNOWN VENDOR CODE AND NO CONFIGURATION";
            return FLASH_DRV_ERROR;
          }
          onenand->bus_width        = setup[C_WIDTH].value;
          onenand->flash_blkcnt     = setup[C_BLOCKS].value;
          onenand->block_count      = setup[C_BLOCKS].value;
          onenand->page_count       = setup[C_PAGESPBLOCK].value;
          onenand->sector_size      = setup[C_SECTORSIZE].value;
          onenand->sectors_per_page = setup[C_SECTORSPPAGE].value;
          break;
    }


    onenand->block_size = onenand->sector_size * onenand->sectors_per_page * onenand->page_count;
    onenand->page_size  = onenand->sector_size * onenand->sectors_per_page;

    /* Set bad block counter to 0 */

    onenand->w_bad_block_count = 0;
    onenand->r_bad_block_count = 0;
    onenand->e_bad_block_count = 0;
    onenand->last_erased_block = 0;
    onenand->last_write_block = 0;
    onenand->num_sector = onenand->sectors_per_page * onenand->page_count;
    onenand->total_size = onenand->block_size * onenand->flash_blkcnt;
    onenand->rw = onenand->malloc(sizeof(T_rw) + onenand->block_size);
    onenand->rw->address = 0xFFFFFFFF;
    onenand->rw->offset  = 0;

    onenand->send_info("ONENAND PAGE SIZE %d BYTES", onenand->page_size);
    onenand->send_info("ONENAND BLOCK SIZE %d BYTES", onenand->block_size);
    onenand->send_info("ONENAND TOTAL SIZE %d BYTES", onenand->total_size);

    OneNAND_flash_reset(onenand->cs_addr);

    return FLASH_DRV_SUCCESS;
}


/*-----------------------------------------------------------------------------
| Function    : onenand_flash_read
+------------------------------------------------------------------------------
| Description : Read function for onenand flash
|
| Parameters  : flash    [IN]  - Flash parms
|               dest_add [OUT] - Destination address
|               src_addr [IN]  - Source address (pointer to data)
|               size     [IN]  - Size of data
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 onenand_flash_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size)
{
    S32 ret_val = FLASH_DRV_SUCCESS;
    T_ONENAND_PARAMS *onenand = (T_ONENAND_PARAMS *)config->data;
    U32 base_addr = 0;
    U32 current_flash_address = 0;
    U32 read_size = size;

    *result = NULL;
    /* driver supports only 32-bit addr. U64 address is just for 2nd API compliance */
    if(src_addr > 0xFFFFFFFF)
    {
      config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", src_addr);
      *result = "ADDRESS BOUNDARY CHECK FAILED";
      return FLASH_DRV_ERROR;
    }

    /* driver supports only 32-bit addr. U64 address is just for 2nd API compliance */
    if(src_addr > 0xFFFFFFFF)
    {
      config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", src_addr);
      *result = "ADDRESS BOUNDARY CHECK FAILED";
      return FLASH_DRV_ERROR;
    }

    INFO2("STARTING READ OF %d BYTES FROM %#08x", size, src_addr);

    if(src_addr < onenand->cs_addr)
    {
        /* If the host did not send the base address,
        add the OneNand base address */
        src_addr += onenand->cs_addr;
    }

    if(onenand->r_bad_block_count)
    {
        /* Skip the bad blocks already detected in previous reads */
        src_addr += onenand->r_bad_block_count * onenand->block_size;
    }

    /* I dont like non-page aligned reads */
    //if (src_addr & ONENAND_WORD_MASK != 0)
    //{
    //    show_data(src_addr,2);
    //    hang_line(__LINE__);
    //    return FLASH_DRV_ERROR;
    //}

    ///* I dont like non-block aligned reads.. */
    //if (src_addr & ONENAND_PAGE_MASK != 0)
    //{
    //    show_data(src_addr,2);
    //    hang_line(__LINE__);
    //    return FLASH_DRV_ERROR;
    //}

    base_addr = onenand->cs_addr;

    while(read_size > 0)
    {
        current_flash_address = (src_addr - base_addr);

        resolve_flashaddress(onenand, current_flash_address);
        INFO4("NEW BLOCK LOOP AT %#08x (B%d, P%d, S%d)", src_addr & ONENAND_SECTOR_ADDRESS_MASK, onenand->block_num, onenand->page_num, onenand->sec_num);
        
        /* Check if we are overflowing*/
        if(current_flash_address >= onenand->total_size)
        {
            //hang_line(__LINE__);
            /* Ran out of chip space.. return error */
            config->send_info("ADDRESS RANGE CHECK FAILED AT %#08x. BOUNDARY IS %#08x.", current_flash_address, onenand->total_size); 
            *result = "READ BEYOND SIZE OF DEVICE";
            return FLASH_DRV_ERROR;
        }

#if 0
        switch(DRV_FILE_FORMATS(flash->session.drv_flag))
        {
          case(DRVFLAG_NAND_ROM_CODE_FORMAT):
#endif
            ret_val = OneNAND_invalidBlock(base_addr, onenand->block_num);
#if 0
            break;

          case(DRVFLAG_NAND_POSTPROC_FORMAT):
          case(DRVFLAG_NAND_NONPOSTPROC_FORMAT):
              ret_val = OneNAND_invalidBlock_Wince(base_addr, onenand->block_num);
              break;

          default:
              return FLASH_DRV_ERROR;

        }
#endif

        if(ret_val != FLASH_DRV_SUCCESS)
        {
            /*the block is bad*/
            if( (current_flash_address % onenand->block_size) == 0 )
            {
                /*The BB count is incremented only once for a block */
                onenand->r_bad_block_count++;
            }
            onenand->send_info("SKIPPING BAD BLOCK AT %#08x", src_addr);
            src_addr += onenand->block_size;
            continue;
        }

        do
        {
            INFO4("READING DATA AT %#08x (B%d, P%d, S%d)", src_addr, onenand->block_num, onenand->page_num, onenand->sec_num);
            ret_val = one_nandRead(base_addr,
                                   onenand->block_num,
                                   onenand->page_num,
                                   onenand->sec_num,
                                   1,
                                   (U16*)onenand->rw->data,
                                   MAIN_AREA,
                                   onenand->sector_size);

            if(ret_val != FLASH_DRV_SUCCESS)
            {
                return FLASH_DRV_ERROR;
            }

  

            if((onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK)) > read_size)
            {
              INFO4("COPYING %d BYTES TO TARGET ADDRESS %#08x (%d > %d)", read_size, dst_addr, onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK), read_size);
              config->drv_memcpy((void *)dst_addr, onenand->rw->data + (src_addr & ONENAND_SECTOR_OFFSET_MASK), read_size);
              src_addr += read_size;
              read_size = 0;
            }
            else
            {
              INFO4("COPYING %d BYTES TO TARGET ADDRESS %#08x (%d <= %d)", onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK), dst_addr, onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK), read_size);
              config->drv_memcpy((void *)dst_addr, onenand->rw->data + (src_addr & ONENAND_SECTOR_OFFSET_MASK), onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK));
              dst_addr  += onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK);
              read_size -= onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK);
              src_addr  += onenand->sector_size - (src_addr & ONENAND_SECTOR_OFFSET_MASK);
            }


            /*increment the destination address by the number of bytes read*/
#if 0
#ifdef ONLD_READ_SPARE
            if( (DRV_FILE_FORMATS(flash->drv_flag) == DRVFLAG_NAND_POSTPROC_FORMAT) ||
               (DRV_FILE_FORMATS(flash->drv_flag) == DRVFLAG_NAND_NONPOSTPROC_FORMAT) )
            {

                ret_val = one_nandRead(base_addr,
                                       onenand->block_num,
                                       onenand->page_num,
                                       onenand->sec_num,
                                       1,
                                       onenand->spare_buffer ,
                                       SPARE_AREA,
                                       ONLD_SPARE_SIZE/2);
                if(ret_val != FLASH_DRV_SUCCESS)
                {
                    return FLASH_DRV_ERROR;
                }
                Retrieve_WinceImage_spare( (T_ONLD_WINCE_SPARE_AREA_INFO*)onenand->spare_buffer,
                                          (U16*)dest_add);
                dest_add = dest_add + ONLD_WINCE_SPARE_SIZE;
            }
#endif
#endif
            onenand->sec_num++;
            if(onenand->sec_num == onenand->sectors_per_page)
            {
                onenand->page_num++;
                onenand->sec_num= 0;
            }

            if(onenand->page_num == onenand->page_count)
            {
                break;
            }

        } while(read_size);


    }/*while(read_size)*/

    INFO("COMPLETED READING DATA");

    return FLASH_DRV_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : onenand_flash_write
+------------------------------------------------------------------------------
| Description : Write function for ONENAND flash
|
| Parameters  : flash    [IN] - Flash parms
|               dst_addr [IN] - Destination address
|               src_addr [IN] - Source address (pointer to data)
|               size     [IN] - Size of data
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 onenand_flash_write(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more)
{
    S32 ret_val= FLASH_DRV_SUCCESS;

    U32 base_addr = 0;
    U32 access_size;
    U32 real_size   = size;

    T_ONENAND_PARAMS *onenand = (T_ONENAND_PARAMS *)config->data;
    BB_STORE_ONENAND_DATA * store_onenand_data;
    U8  badblock_rewindlogic = DISABLE;
#if 0
    U16 *dta_buf;
    U16 *wince_buf;
    U32 spare_src_addr;
    U16 sector_always_zero = 0;
#endif
    U32 current_flash_address =0;

    *result = NULL;
    /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
    if(dst_addr > 0xFFFFFFFF)
    {
      config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", dst_addr);
      *result = "ADDRESS BOUNDARY CHECK FAILED";
      return FLASH_DRV_ERROR;
    }

    /* driver support only 32-bit addr. U64 address is just for 2nd API compliance */
    if(dst_addr > 0xFFFFFFFF)
    {
      config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", dst_addr);
      *result = "ADDRESS BOUNDARY CHECK FAILED";
      return FLASH_DRV_ERROR;
    }

#if 0
    T_ONLD_WINCE_SPARE_AREA_INFO* onld_wince_spareptr;
    onld_wince_spareptr =  (T_ONLD_WINCE_SPARE_AREA_INFO*) onenand->spare_buffer;
#endif
    store_onenand_data = (BB_STORE_ONENAND_DATA*)onenand->store_onenand;

    if(dst_addr < onenand->cs_addr)
    {
        /* If the host did not send the base address, add the OneNand base address */
        dst_addr += onenand->cs_addr;
    }

    if(onenand->w_bad_block_count)
    {
        /* Skip the bad blocks already detected in previous writes */
        dst_addr += onenand->w_bad_block_count * onenand->block_size;
    }

    ///* I dont like non-page aligned writes */
    //if(dest_add & ONENAND_WORD_MASK != 0)
    //{
    //    show_data(dest_add,2);
    //    hang_line(__LINE__);
    //    return FLASH_DRV_ERROR;
    //}

    ///* I Dont like non-block aligned writes.. */
    //if(dest_add & ONENAND_PAGE_MASK != 0)
    //{
    //    show_data(dest_add,2);
    //    hang_line(__LINE__);
    //    return FLASH_DRV_ERROR;
    //}


    if(onenand->rw->address == 0xFFFFFFFF)
    {
      if((dst_addr & ONENAND_BLOCK_ADDRESS_MASK) != dst_addr)
      {
        config->send_info("START ADDRESS %#08x NOT ALIGNED TO BLOCK SIZE %#x)", 
                          dst_addr,
                          onenand->page_size);
        *result = "ILLEGAL START ADDRESS";
        return FLASH_DRV_ERROR;

      }
      
      onenand->rw->address = dst_addr;
      onenand->rw->offset  = 0;
    }
    else
    {
      if(dst_addr != (onenand->rw->address + onenand->rw->offset))
      {
        config->send_info("ADDRESS MISMATCH FOR BUFFER DATA STORAGE (%#08x != %#08x)", 
                          dst_addr,
                          onenand->rw->address + onenand->rw->offset);
        *result = "MISALIGNMENT OF CONSECUTIVE WRITES";
        return FLASH_DRV_ERROR;
      }
    }

    while(real_size)
    {
      U16 chunk_size;

      chunk_size = (U16)(real_size > onenand->block_size - onenand->rw->offset ? onenand->block_size - onenand->rw->offset : real_size);
      config->drv_memcpy(onenand->rw->data + onenand->rw->offset, (void *)src_addr, chunk_size);
      
      onenand->rw->offset += chunk_size;
      real_size           -= chunk_size;
      src_addr            += chunk_size;

      if((onenand->rw->offset == onenand->block_size) || ((more == NO_MORE_DATA) && !real_size))
      {
          // BUSINESS END GOES HERE... PER BLOCK...        

          U32 write_size = onenand->rw->offset;
          U8 * source    = onenand->rw->data;
          base_addr      = onenand->cs_addr;
          access_size    = onenand->sector_size;

          while(write_size)
          {
            current_flash_address = (onenand->rw->address - base_addr);

            resolve_flashaddress(onenand, current_flash_address);

            /* Check if we are overflowing... */

            if(current_flash_address >= onenand->total_size)
            {
              config->send_info("ADDRESS RANGE CHECK FAILED AT %#08x. BOUNDARY IS %#08x.", current_flash_address, onenand->total_size); 
              *result = "WRITE BEYOND SIZE OF DEVICE";
              ret_val = FLASH_DRV_ERROR;
              return ret_val;
            }

            /* we have the current block.. */
#if 0
            if(DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_ROM_CODE_FORMAT)
            {
#endif
              if(OneNAND_invalidBlock(base_addr, onenand->block_num) != FLASH_DRV_SUCCESS)
              {
                if( (current_flash_address % onenand->block_size) == 0 )
                {
                  /*The BB count is incremented only once for a block */
                  onenand->w_bad_block_count++;
                  onenand->send_info("SKIPPING BLOCK %#04x - BLOCK IS BAD", onenand->block_num);
                }
                onenand->block_num++;
                dst_addr += onenand->block_size;
                continue;
              }
#if 0
            }
            else if((DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_POSTPROC_FORMAT) ||
                    (DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_NONPOSTPROC_FORMAT))
            {
              if(OneNAND_invalidBlock_Wince(base_addr, onenand->block_num) != FLASH_DRV_SUCCESS)
              {
                if( (current_flash_address % BLOCKSIZE_ONENAND) == 0 )
                {
                  /*The BB count is incremented only once for a block */
                  onenand->w_bad_block_count++;
                }
                onenand->block_num++;
                dest_add +=  BLOCKSIZE_ONENAND;
                show_data(dest_add,2);
                continue;
              }
            }
#endif

            if(badblock_rewindlogic)
            {
              ERROR("ACTIVATING SUSPECT BADBLOCK REWIND LOGIC");
              badblock_rewindlogic = DISABLE;
              store_onenand_data->valid_block_num = onenand->block_num;
              ret_val = badblock_Copyback(onenand, store_onenand_data);
              if(ret_val != FLASH_DRV_SUCCESS)
              {
                /*unable to rewind the data*/
                ret_val = FLASH_DRV_ERROR;
                return ret_val;
              }
              //src_addr = store_onenand_data->src_addr;  // TODO - LOOKS SUSPECT
              //real_size =  store_onenand_data->size;
              onenand->page_num =	store_onenand_data->page_num;
              onenand->sec_num = store_onenand_data->sector_num;
            }

            /* If this is still protected.. we cannot do a thing.. board issue */
            INFO1("UNLOCKING BLOCK %#04x", onenand->block_num);
            unlock_OneNAND(base_addr, onenand->block_num, onenand->block_num);

            /* Downloading the Non Wince Format images */
#if 0
            if(DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_ROM_CODE_FORMAT)
            {
#endif
              do
              {
                /* At this point.. the block is good.. ready to write */
                access_size = (write_size > onenand->sector_size) ? onenand->sector_size : write_size;

                INFO4("WRITING %d BYTES TO SECTOR %d OF PAGE %d OF BLOCK %#04x", access_size, onenand->sec_num, onenand->page_num, onenand->block_num);
                
                ret_val = one_nandWrite(base_addr,
                                        onenand->block_num,
                                        onenand->page_num,
                                        onenand->sec_num,
                                        1,
                                        (U16*)source,
                                        MAIN_AREA,
                                        access_size);
              
                if(ret_val != FLASH_DRV_SUCCESS)
                {
                  /*store the current values before rewinding*/
                  store_onenand_data->bb_block_num = onenand->block_num;
                  store_onenand_data->page_num = onenand->page_num;
                  store_onenand_data->sector_num = onenand->sec_num;
                  store_onenand_data->src_addr = src_addr;
                  store_onenand_data->size = real_size;
                  store_onenand_data->base_addr = base_addr;

                  if( (current_flash_address % onenand->block_size) == 0 )
                  {
                    /*The BB count is incremented only once for a block */
                    onenand->w_bad_block_count++;
                  }

                  onenand->block_num++;
                  dst_addr += onenand->block_size;
                  badblock_rewindlogic = ENABLE;

                  /*check for next valid block to copy back the data
                  and then write the current data*/
                  break;
                }

                /*if write is successful store the block address*/
                onenand->last_write_block = onenand->block_num * onenand->block_size;

                if(write_size > onenand->sector_size)
                {
                  write_size -= onenand->sector_size;
                  source     += onenand->sector_size;
                }
                else
                {
                  source     += write_size;
                  write_size -= write_size;
                }

                onenand->sec_num++;
                if(onenand->sec_num == onenand->sectors_per_page)
                {
                  onenand->sec_num = 0;
                  onenand->page_num++;
                }
                
                if((onenand->page_num == onenand->page_count) && write_size)
                {
                  /* Since we operate the write loop on a block sized buffer, this should NOT happen! */
                  *result = "INTERNAL ERROR - WRITE CROSSING BLOCK BOUNDARY";
                  return FLASH_DRV_ERROR;
                }

              } while(write_size);/* a block is complete*/

              if(!write_size)
              {
                onenand->rw->address = (onenand->rw->address + onenand->rw->offset) & ONENAND_BLOCK_ADDRESS_MASK;
                onenand->rw->offset  = 0;
              }
#if 0
              continue;
            }
            // STUFF BELOW HERE IS SUSPECT!!!
            else if((DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_POSTPROC_FORMAT) ||
                    (DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_NONPOSTPROC_FORMAT))
            {
              /*Downloading the Wince Format images */
              while(real_size > 0)
              {
                /*First Update the Spare area data to the ONENAND DATA BUFFER */
                spare_src_addr = src_addr + onenand->sector_size;
                Retrive_Wince_Spare_Info((U8*)spare_src_addr, onld_wince_spareptr);

                wince_buf = (U16*)onld_wince_spareptr;

                /* get spare buffer address */
                dta_buf = (U16*)GET_SPARE_BUF_ADDR(base_addr,sector_always_zero,DATA_RAM0_SEL);
                /* Memcopy for spare data */
                _WriteSpare(wince_buf, dta_buf, 1);

                /* Write Main AREA of the ONE NAND */
                ret_val = one_nandWrite(base_addr,onenand->block_num, onenand->page_num, onenand->sec_num, 1, (U16*)src_addr, MAIN_AREA, access_size);
                if(ret_val != FLASH_DRV_SUCCESS)
                {
                  store_onenand_data->bb_block_num = onenand->block_num;
                  store_onenand_data->page_num = onenand->page_num;
                  store_onenand_data->sector_num = onenand->sec_num;
                  store_onenand_data->src_addr = src_addr;
                  store_onenand_data->size = real_size;
                  store_onenand_data->base_addr = base_addr;
                  if( (current_flash_address % BLOCKSIZE_ONENAND) == 0 )
                  {
                    /*The BB count is incremented only once for a block */
                    onenand->w_bad_block_count++;
                  }

                  onenand->block_num++;
                  dest_add += BLOCKSIZE_ONENAND;
                  badblock_rewindlogic = DISABLE;

                  /*check for next valid block to copy back the data
                  and then write the current data*/
                  break;
                }

                /*if write is successful store the block address*/
                onenand->last_write_block = onenand->block_num * BLOCKSIZE_ONENAND;

                real_size -= (real_size > (onenand->sector_size+ ONLD_WINCE_SPARE_SIZE)) ? (onenand->sector_size + ONLD_WINCE_SPARE_SIZE) : real_size;
                src_addr = src_addr + ONENAND_SECTOR_SIZE + ONLD_WINCE_SPARE_SIZE;
                onenand->sec_num++;
                if(onenand->sec_num == 4)
                {
                  onenand->sec_num = 0;
                  onenand->page_num++;
                  if(onenand->page_num == ONENAND_MAX_PAGES)
                  {
                    onenand->page_num = 0;
                    onenand->block_num++;
                    dest_add += BLOCKSIZE_ONENAND;
                    break;
                  }
                }
              }/*while*/
              continue;
            }
            else
            {
              return FLASH_DRV_ERROR;
            }
#endif

          }/* while (write_size) */


          // END OF BUSINESS END

      }
    }

    return ret_val;
}


/*-----------------------------------------------------------------------------
| Function    : onenand_flash_erase
+------------------------------------------------------------------------------
| Description : onenand erase function to erase a block in the onenand device
|
| Parameters  : flash [IN] - Flash parms
|               addr  [IN] - Block address
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/
U32 onenand_flash_erase(T_driver_config * config, char ** result, U64 addr, U64 length)
{
    S32 ret_val = FLASH_DRV_SUCCESS;

    //U32 block_num =0;
    //U32 blk_aligned_address = 0;
    U32 base_addr = 0;
    T_ONENAND_PARAMS *onenand = (T_ONENAND_PARAMS *)config->data;
    U8 skip_bb_chk            = onenand->bberase; 
    U32 current_flash_address = 0;
    U32 targetlength          = length ? length : onenand->total_size;
    U32 eraselength           = targetlength;
    U32 address               = addr;

    *result = NULL;

    /* driver supports only 32-bit addr. U64 address is just for 2nd API compliance */
    if(addr > 0xFFFFFFFF || length > 0xFFFFFFFF)
    {
      if(length > 0xFFFFFFFF)
        config->send_info("64-bit LENGTH (%#08x) NOT SUPPORTED.", length);
      else
        config->send_info("64-bit ADDRESS (%#08x) NOT SUPPORTED.", addr);
      *result = "ADDRESS BOUNDARY CHECK FAILED";
      return FLASH_DRV_ERROR;
    }

    if(address < onenand->cs_addr)
    {
        /* If the host did not send the base address, add the OneNand base address */
        address += onenand->cs_addr;
    }

    base_addr = onenand->cs_addr;

    if(onenand->e_bad_block_count)
    {
        /* Skip the bad blocks already detected in previous writes */
        address += onenand->e_bad_block_count * onenand->block_size;
    }

    current_flash_address = (address - base_addr);

    /*if the end of the flash is reached and all the blocks are erased
    then return success, this condition may occur if there are bad blocks
    in the flash*/
    if( (current_flash_address >= onenand->total_size) &
       (onenand->last_erased_block == (onenand->total_size - onenand->block_size)))
    {
        return FLASH_DRV_SUCCESS;
    }


    onenand->block_num = (U16)(current_flash_address/onenand->block_size);
//    blk_aligned_address = current_flash_address & ONENAND_BLOCK_ADDRESS_MASK;

    /* erase first block of the flash always */
    //if( address != onenand->cs_addr )
    //{
    //    /*we dont want to erase the block multiple times*/
    //    if(blk_aligned_address <= onenand->last_erased_block)
    //    {
    //        /*the block has already been erased*/
    //        return FLASH_DRV_SUCCESS;
    //    }

    //    /*we dont want to erase the blocks which are already written*/
    //    if(blk_aligned_address <= onenand->last_write_block)
    //    {
    //        /*the block has already been erased*/
    //        return FLASH_DRV_SUCCESS;
    //    }

    //}


    /* now we will erase; eraseing is done block wise only */
    while(eraselength > 0)
    {
        /*  are we out of the chip boundary */
        if(current_flash_address >= onenand->total_size)
        {
          if(length)
          {
            config->send_info("FAILED TO ERASE %#x BYTES FROM %#08x (%#x BYTES ERASED)", targetlength, addr, targetlength - eraselength);
            *result = "ERASE BEYOND SIZE OF DEVICE";
            return FLASH_DRV_ERROR;
          }
          else
          {
            config->send_status("Erase progress", targetlength, targetlength); 
            config->send_info("%d bytes erased", targetlength - eraselength);
            return FLASH_DRV_SUCCESS;
          }
        }

        config->send_status("Erase progress", targetlength - eraselength, targetlength); 

        /* Check if this is a bad block */
        if (skip_bb_chk==0)
        {
            /* Erase block if not already erased. */
#if 0
            if(DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_ROM_CODE_FORMAT)
            {
#endif
                while(current_flash_address < onenand->total_size)
                {
                    if(OneNAND_invalidBlock(base_addr, onenand->block_num) != FLASH_DRV_SUCCESS)
                    {
                        if( (current_flash_address % onenand->block_size) == 0 )
                        {
                            /*The BB count is incremented only once for a block */
                            INFO1("SKIPPING BLOCK %#04x - BLOCK IS BAD", onenand->block_num);
                            onenand->e_bad_block_count++;
                        }

                        /* skip to the next block address */
                        onenand->block_num++;
                        current_flash_address += onenand->block_size;
                    }
                    else
                    {
                        break;
                    }
                }
#if 0
            }
            else if((DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_POSTPROC_FORMAT) ||
                    (DRV_FILE_FORMATS(flash->session.drv_flag) == DRVFLAG_NAND_NONPOSTPROC_FORMAT))
            {
                while(current_flash_address < onenand->total_size)
                {
                    if(OneNAND_invalidBlock_Wince(base_addr, onenand->block_num) != FLASH_DRV_SUCCESS)
                    {
                        if( (current_flash_address % BLOCKSIZE_ONENAND) == 0 )
                        {
                            /*The BB count is incremented only once for a block */
                            onenand->e_bad_block_count++;
                        }

                        /* skip to the next block address */
                        onenand->block_num++;
                        current_flash_address += BLOCKSIZE_ONENAND;

                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                return FLASH_DRV_ERROR;
            }
#endif
            /* check if we went beyond chip boundary - nothing we can do now.. */
            if (current_flash_address > onenand->total_size)
            {
              if(length)
              {
                config->send_info("FAILED TO ERASE %#x BYTES FROM %#08x (%#x BYTES ERASED)", targetlength, addr, targetlength - eraselength);
                *result = "ERASE BEYOND SIZE OF DEVICE";
                return FLASH_DRV_ERROR;
              }
              else
              {
                config->send_status("Erase progress", targetlength, targetlength); 
                config->send_info("%d bytes erased", targetlength - eraselength);
                return FLASH_DRV_SUCCESS;
              }
            }

        } /*skip bad block*/

        /* If this is still protected.. we cannot do a thing.. board issue */
        INFO1("UNLOCKING BLOCK %#04x", onenand->block_num);
        unlock_OneNAND(base_addr, onenand->block_num, onenand->block_num);

        INFO1("ERASING BLOCK %#04x", onenand->block_num);
        ret_val = OneNAND_eraseBlock(base_addr, onenand->block_num);
        /* check if there was an error in operation */
        if (ret_val != FLASH_DRV_SUCCESS)
        {
            /* Attempt to mark.. ignore if failed.. */
            //if(mark_bad_block(base_addr,current_flash_address) != FLASH_DRV_SUCCESS)
            //{
            //	show_data(current_flash_address,1);
            //	hang_line(__LINE__);
            //}
            INFO1("FAILED TO ERASE BLOCK %#04x", onenand->block_num);
            if( (current_flash_address % onenand->block_size) == 0 )
            {
                /*The BB count is incremented only once for a block */
                onenand->e_bad_block_count++;
            }

            /* retry again with the next block address */
            onenand->block_num++;
            current_flash_address += onenand->block_size;
            continue;
        }
        else
        {
            /*store only the block aligned address*/
            INFO1("BLOCK %#04x ERASED", onenand->block_num);
            onenand->last_erased_block = onenand->block_num * onenand->block_size;
        }
        /* go to next block to erase */
        eraselength -= (eraselength > onenand->block_size) ? onenand->block_size : eraselength;
        onenand->block_num++;
        current_flash_address += onenand->block_size;

    } /* End of while loop */

    config->send_status("Erase progress", targetlength - eraselength, targetlength); 
    config->send_info("%d bytes erased", targetlength);
    return ret_val;
}

U32 onenand_flash_deinit(T_driver_config * config, char ** result)
{
  *result = NULL;
  if(config->data)
  {
    if(((T_ONENAND_PARAMS *)config->data)->rw) config->drv_free(((T_ONENAND_PARAMS *)config->data)->rw);
    config->drv_free(config->data);
  }
  return(FLASH_DRV_SUCCESS);
}

U32 onenand_flash_get_info(T_driver_config * config, T_driver_info * info)
{
  T_ONENAND_PARAMS *onenand = (T_ONENAND_PARAMS *)config->data;
  info->device_base_address = onenand->cs_addr;
  info->device_size         = onenand->total_size;
  return FLASH_DRV_SUCCESS;
}


#if 0

/*-----------------------------------------------------------------------------
| Function    : flash_check_bad_blocks
+------------------------------------------------------------------------------
| Description : Checks if the requested block is good.
|
| Parameters  : flash    [IN]  - Flash parms
|				bad_blocks [OUT] - Flash bad block list
|               src_addr [IN]  - Source address (pointer to data)
|               size     [IN]  - Size to check
|
| Returns     : result_code
+-----------------------------------------------------------------------------*/

U32 flash_check_bad_blocks(T_FLASH_CONFIG_PARMS *flash,
                           T_BAD_BLOCK_ELEMENT **bad_blocks,
                           U32 src_addr,
                           U32 size)
{
    T_ONENAND_PARAMS *onenand = (T_ONENAND_PARAMS *)flash->drv_cache_ptr;

    U8 status = FLASH_DRV_SUCCESS;
    U32 offset = 0;
    T_BAD_BLOCK_ELEMENT *pNew=NULL,*pLast=NULL;
    U32 temp = 0, base_addr =0;

    *bad_blocks = NULL;

    base_addr = onenand->cs_addr;

    temp = (src_addr - base_addr);
    onenand->block_num = temp/(ONENAND_MAX_PAGES * ONENAND_MAX_SECTORS * ONENAND_SECTOR_SIZE);

    while(offset < size)
    {
        /* Check if we are jumping out..*/
        if(temp >= (onenand->total_size))
        {
            status = FLASH_DRV_INVALID_PARAMETER;
            break;
        }

        /* Check if current block is bad */
        if(OneNAND_invalidBlock(base_addr, onenand->block_num) != FLASH_DRV_SUCCESS)
        {
            pNew = (T_BAD_BLOCK_ELEMENT *) onenand->drv_malloc(sizeof(T_BAD_BLOCK_ELEMENT));
            if(pNew == NULL)
            {
                status = FLASH_DRV_ERROR;
                break;
            }

            /* Reset status here .. in case we dont have any more bad blocks */
            status = FLASH_DRV_SUCCESS;

            if(pLast == NULL)
                *bad_blocks = pNew;
            else
                pLast->next = pNew;

            pNew->next = NULL;
            pNew->blockNr = (onenand->block_num*BLOCKSIZE_ONENAND);
            pLast = pNew;
        }
        temp += BLOCKSIZE_ONENAND;
        onenand->block_num++;
        offset += BLOCKSIZE_ONENAND;
    }

    /* If Failed.. traverse the list and free prev allocated items */
    if ((*bad_blocks != NULL) && (status != FLASH_DRV_SUCCESS))
    {
        pLast = *bad_blocks;
        while(pLast != NULL)
        {
            pNew = pLast->next;
            onenand->free(pLast);
            pLast = pNew;
        }
        *bad_blocks = NULL;
    }

    /* if successful.. the caller has the responsibility of freeing the chain */
    return status;
}

#endif

/* End of File */

