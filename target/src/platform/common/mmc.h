/**
 * @file mmc.h
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
 * Provide Command information to mmc
 * 
 */

/*==== DECLARATION CONTROL =================================================*/
#ifndef MMC_H
#define MMC_H

#include "types.h"
#include "config.h"

//#ifdef MMC_C_PRIVATE

/*==== INCLUDES ============================================================*/

#include "mmc_dis.h"

/*=================== GLOBALS ===============================================*/

/*=================== MACROS=================================================*/

/*Macros to get and set*/
#define MMCHS_REG(mmc_slot,reg)            (volatile U32 *)(MMCHS_BASE(mmc_slot) + reg)
#define MMCHS_GET_R(mmc_slot,reg)            (*MMCHS_REG(mmc_slot,reg))
#define MMCHS_SET_R(mmc_slot,reg,val)        (*MMCHS_REG(mmc_slot,reg) =(U32)val)
#define MMCHS_FSET_R(mmc_slot,reg,val)       (*MMCHS_REG(mmc_slot,reg)|=(U32)val)
#define MMCHS_ASET_R(mmc_slot,reg,val)       (*MMCHS_REG(mmc_slot,reg)&=(U32)val)

#define MMCHS_STUFF_BITS            (0x0000)
#define MMC_STUFF_BITS              (0xFFFF)

#define MMC_OCR_ACCESS_MODE_BYTE             (0x0  << 29)
#define MMC_OCR_ACCESS_MODE_SECTOR           (0x2  << 29)

#define MMC_PWR_STABLE              (0x202)
#define PBIAS_LITE_VMMC1_3V         (0x101)
#define PBIAS_LITE_VMMC1_52MHZ      (0x404)
#define PBIAS_LITE_MMC1_ERROR       (0x808)

/*CARD status Bits 12:9*/
#define MMC_CARD_STATE_IDLE         (0x0 << 9)
#define MMC_CARD_STATE_READY        (0x1 << 9)
#define MMC_CARD_STATE_IDENT        (0x2 << 9)
#define MMC_CARD_STATE_STBY         (0x3 << 9)
#define MMC_CARD_STATE_TRAN         (0x4 << 9)
#define MMC_CARD_STATE_DATA         (0x5 << 9)
#define MMC_CARD_STATE_RCV          (0x6 << 9)
#define MMC_CARD_STATE_PRG          (0x7 << 9)
#define MMC_CARD_STATE_DIS          (0x8 << 9)
#define MMC_CARD_STATE_BTST         (0x9 << 9)
#define MMC_CARD_STATE_MASK         (0xF << 9)

#define MMC_CARD_SWITCH_ERROR       (0x1 << 7)


#define MMC_HIGH_VOLTAGE_CARD       (0x00FF8000)
/*===================MMC MACROS=============================================*/
///This only handles HSMMC1 and HSMMC2 (not MMC/SD3, MMC/SD4, MMC/SD5)
#ifdef OMAP3
#define MMCHS_BASE(mmc_slot_no)     ((0x4809C000) + ((0x18000)*(mmc_slot_no-1)))
#endif
#ifdef OMAP4
#define MMCHS_BASE(mmc_slot_no)     ((0x4809C100) + ((0x18000)*(mmc_slot_no-1)))
#endif
#ifdef OMAP5
#define MMCHS_BASE(mmc_slot_no)     ((0x4809C100) + ((0x18000)*(mmc_slot_no-1)))
#endif


/*OMAP HSMMC Controller Registers*/
#define MMCHS_HL_REV                0x000
#define MMCHS_HL_HWINFO             0x004
#define MMCHS_HL_SYSONFIG           0x004
#define MMCHS_SYSC                  0x010
#define MMCHS_SYSS                  0x014
#define MMCHS_CSRE                  0x024
#define MMCHS_SYST                  0x028
#define MMCHS_CON                   0x02C
#define MMCHS_BLK                   0x104
#define MMCHS_ARG                   0x108
#define MMCHS_CMD                   0x10C
#define MMCHS_RSP10                 0x110
#define MMCHS_RSP32                 0x114
#define MMCHS_RSP54                 0x118
#define MMCHS_RSP76                 0x11C
#define MMCHS_DATA                  0x120
#define MMCHS_PSTAT                 0x124
#define MMCHS_HCTL                  0x128
#define MMCHS_SYSCTL                0x12C
#define MMCHS_STAT                  0x130
#define MMCHS_IE                    0x134
#define MMCHS_ISE                   0x138
#define MMCHS_AC12                  0x13C
#define MMCHS_CAPA                  0x140
#define MMCHS_CURCAPA               0x148
#define MMCHS_FE                    0x150
#define MMCHS_ADMAES                0x154
#define MMCHS_ADMASAL               0x158
#define MMCHS_REV                   0x1FC

/* MMCHS_SYSC (sysconfig) register fields*/
#define SYSC_AUTOIDLE               ((0x1)<<0)  /*auto clock gating */
#define SYSC_SOFTRESET              ((0x1)<<1)  /*module reset */
#define SYSC_ENWKUP                 ((0x1)<<2)  /*wakeup feature */
//#define SYSC_SIDLEMODE  /*4:3 BITS  Have to set */
#define SYSC_CLKACT_CLKOFF          ((0x0)<<8)  /*OCP FCLK switch off */
#define SYSC_CLKACT_OCP             ((0x1)<<8)
#define SYSC_CLKACT_FCLK            ((0x2)<<8)
#define SYSC_CLKACT_OCP_FCLK        ((0x3)<<8)
#define SYSC_CLKACT_NOSTDBY         ((0x1)<<12)

/* MMCHS_SYSS (sysstaus) register */
#define MMCHS_SYSS_RESETDONE        ((0x1)<<0)

/* MMCHS_CSRE register fields*/

/* MMCHS_CON bit fields*/
#define MMCHS_ODRN                  ((0x1)<<0) /*open drain*/   /*for broad cast cmds 1,2,3 and 40 must set */
#define MMCHS_INIT                  ((0x1)<<1)  /*init sequence */
#define MMCHS_CON_HR                ((0x1)<<2)  /* HOST generates 48bit resp */
#define MMCHS_CON_STR               ((0x1)<<3)  /* enable stream orented data transfer */
//#define MMCHS_CON_MODE
#define MMCHS_CON_DW8               ((0x1)<<5)  /* enable 8 bit data width */
#define MMCHS_CON_MIT               ((0x1)<<6)  /* command timeout disable */
#define MMCHS_CON_CDP               ((0x1)<<7)  /*card detect polarity */
//#define MMCHS_CON_WPP
#define MMCHS_CTO                   (0x0)<<6    /*command timeout enable */
#define MMCHS_CON_CTPL              ((0x1)<<11)
#define MMCHS_CON_PADEN             ((0x1)<<15)
#define MMCHS_CON_DDR               ((0x1)<<19)
#define MMCHS_CON_DMA_MNS           ((0x1) << 20)

/* MMCHS_SYST (SYETEM TEST) register*/
#define MMCHS_SYST_MCKD             ((0x1)<<0)  /*OUTPUT CLOCK */
#define MMCHS_SYST_CDIR_H2C         ((0x0)<<1)  /* cmd direction host to card) */
#define MMCHS_SYST_CDIR_C2H         ((0x1)<<1)  /* cmd direction card to host) */
//#define MMCHS_SYST_CDAT              <<<NEED TO ADD< !!!!!!!!
#define MMCHS_SYST_DDIR_H2C         ((0X0)<<3)  /*DATA direction host to card */
#define MMCHS_SYST_DDIR_C2H         ((0X1)<<3)  /*data direction card to host */
#define MMCHS_SYST_SSB              ((0x1)<<12) /*set status bit clear statusreg */
#define MMCHS_SYST_WAKD_LOW         ((0x0)<<13)
#define MMCHS_SYST_WAKD_HIGH        ((0x1)<<13)
#define MMCHS_SYST_SDWP             ((0x1)<<14) /*card write protect */
#define MMCHS_SYST_SDCD             ((0X1)<<15) /* card detect */
#define MMCHS_HCTL_DTW              (0x1<<1)
#define MMCHS_SYSCTL_CLK_DIS        (0xFFFF003F)

/* MMCHS_BLK reg values*/
#define MMCHS_NBLK_ONE              (0x00010000)
#define MMCHS_BLKSZ_512             (0x00000200)

/* MMCHS_PSTAT */
#define MMCHS_PSTAT_CLEV            (0x1<<24)
#define MMCHS_PSTAT_DLEV            (0xF<<20)
#define MMCHS_PSTAT_DBOUNCE         (0x3<<16)
#define MMCHS_PSTAT_BRE             (0x1<<11)
#define MMCHS_PSTAT_BWE             (0x1<<10)
#define MMCHS_PSTAT_RTA             (0x1<<9)
#define MMCHS_PSTAT_WTA             (0x1<<8)
#define MMCHS_PSTAT_DLA             (0x1<<2)
#define MMCHS_PSTAT_DATI            (0x1<<1)
#define MMCHS_PSTAT_CMDI            (0x1<<0)

/*SYSCTL REG*/
#define MMCHS_DTO_MAX               ((0xE)<<16) /*data timeout */
#define MMCHS_SYSCTL_ICE            (0x1<<0)    /*internal clock enable */
#define MMCHS_SYSCTL_ICS            (0x1<<1)
#define MMCHS_SYSCTL_CEN            (0x1<<2)    /*clock enable */
#define CLK_DIV                     (0x4)   /*this value should be changed based on the fclk value */
#define MMCHS_SYSCTL_CLKD_1         (0x1<<6)
#define MMCHS_SYSCTL_CLKD_2         (0x2<<6)
#define MMCHS_SYSCTL_CLKD_4         (0x4<<6)
#define MMCHS_SYSCTL_CLKD_5         (0x5<<6)
#define MMCHS_SYSCTL_CLKD_8         (0x8<<6)
#define MMCHS_SYSCTL_CLKD_48        (0x30<<6)
#define MMCHS_SYSCTL_RESET          (0x2)   /*reset */
#define CLK_DIV0                    ((0x0)<<6)
#define MMCHS_SYSCTL_SRA            (0x1<<24)   /*Soft reset all */
#define MMCHS_SYSCTL_SRC            (0x1<<25)   /*Soft reset command line */
#define MMCHS_SYSCTL_SRD            (0x1<<26)   /*Soft reset data lines */

/*HCTL reg*/
#define MMCHS_SDVS_1_8V             (0x5<<9)    /*select 1.8v */
#define MMCHS_SDVS_3_0V             (0x6<<9)    /*select 3.0v */
#define MMCHS_SDVS_3_3V             (0x7<<9)    /*select 3.3v */
#define MMCHS_HCTL_SDBP             (0x0100)    /*enable sd bus pwr */
#define MMCHS_HCTL_IBG              (0x1<<19)
#define MMCHS_HCTL_RWC              (0x1<<18)
#define MMCHS_HCTL_CR               (0x1<<17)
#define MMCHS_HCTL_SBGR				(0x1<<16)
#define MMCHS_HCTL_SBGR_CLR			(0x0<<16)
#define MMCHS_HCTL_DMAS_ADMA2       (0x2<<3)    /* select ADMA2 DMA mode */
/*CAPA Reg*/
#define MMCHS_1_8V                  (0x1<<26)
#define MMCHS_3_0V                  (0x1<<25)
#define MMCHS_3_3V                  (0x1<<24)
#define MMCHS_CAPA_AD2S             (0x01<<19)

/*  MMCHS STAT field values  */
/*Error interrupt status*/
#define MMCHS_MMCSTAT_BADA          (0x20000000)    /*Bad access to data space */
#define MMCHS_MMCSTAT_CERR          (0x10000000)    /*card error */
#define MMCHS_MMCSTAT_ACE           (0x01000000)    /*Auto cmd12 er */
#define MMCHS_MMCSTAT_DEB           (0x00400000)    /*Data end bit er */
#define MMCHS_MMCSTAT_DCRC          (0x00200000)    /*Data crc er */
#define MMCHS_MMCSTAT_DTO           (0x00100000)    /*Data timeout */
#define MMCHS_MMCSTAT_CIE           (0x00080000)    /*command index er */
#define MMCHS_MMCSTAT_CEB           (0x00040000)    /*command endbit er */
#define MMCHS_MMCSTAT_CCRC          (0x00020000)    /*command crc error */
#define MMCHS_MMCSTAT_CTO           (0x00010000)    /*command timeout */
/*normal interrupt status*/
#define MMCHS_MMCSTAT_ERRI          (0x8000)    /*error interrupt */
#define MMCHS_MMCSTAT_CIRQ          (0x0100)    /*card interrupt, only for SD */
#define MMCHS_MMCSTAT_CREM          (0x0080)    /*card removed */
#define MMCHS_MMCSTAT_CINS          (0x0040)    /*card inserted */
#define MMCHS_MMCSTAT_BRR           (0x0020)    /*buffer read ready */
#define MMCHS_MMCSTAT_BWR           (0x0010)    /*buffer write ready */
#define MMCHS_MMCSTAT_DMA           (0x0008)    /*DMA interrupt, not supported */
#define MMCHS_MMCSTAT_BGE           (0x0004)    /*block gap event */
#define MMCHS_MMCSTAT_TC            (0x0002)    /*transfer complete */
#define MMCHS_MMCSTAT_CC            (0x0001)    /*command complete */

/*  MMCHS CMD field values  */
/*Response lengths*/
#define MMCHS_RSP_LEN48             (0x00020000)
#define MMCHS_RSP_LEN48B            (0x00030000)
#define MMCHS_RSP_LEN136            (0x00010000)
#define MMCHS_RSP_NONE              (0x00000000)
#define MMCHS_RSP_MASK              (0x00030000)

/*command index*/
#define MMCHS_CMD0                  (0x00000000)
#define MMCHS_CMD1                  (0x01000000)
#define MMCHS_CMD2                  (0x02000000)
#define MMCHS_CMD3                  (0x03000000)
#define MMCHS_CMD4                  (0x04000000)
#define MMCHS_CMD5                  (0x05000000)
#define MMCHS_CMD6                  (0x06000000)
#define MMCHS_CMD7                  (0x07000000)
#define MMCHS_CMD8                  (0x08000000)
#define MMCHS_CMD9                  (0x09000000)
#define MMCHS_CMD10                 (0x0A000000)
#define MMCHS_CMD11                 (0x0B000000)
#define MMCHS_CMD12                 (0x0C000000)
#define MMCHS_CMD13                 (0x0D000000)
#define MMCHS_CMD14                 (0x0E000000)
#define MMCHS_CMD15                 (0x0F000000)
#define MMCHS_CMD16                 (0x10000000)
#define MMCHS_CMD17                 (0x11000000)
#define MMCHS_CMD18                 (0x12000000)
#define MMCHS_CMD19                 (0x13000000)
#define MMCHS_CMD20                 (0x14000000)
#define MMCHS_CMD21                 (0x15000000)
#define MMCHS_CMD22                 (0x16000000)
#define MMCHS_CMD23                 (0x17000000)
#define MMCHS_CMD24                 (0x18000000)
#define MMCHS_CMD25                 (0x19000000)
#define MMCHS_CMD26                 (0x1A000000)
#define MMCHS_CMD27                 (0x1B000000)
#define MMCHS_CMD28                 (0x1C000000)
#define MMCHS_CMD29                 (0x1D000000)
#define MMCHS_CMD30                 (0x1E000000)
#define MMCHS_CMD31                 (0x1F000000)
#define MMCHS_CMD32                 (0x20000000)
#define MMCHS_CMD33                 (0x21000000)
#define MMCHS_CMD34                 (0x22000000)
#define MMCHS_CMD35                 (0x23000000)
#define MMCHS_CMD36                 (0x24000000)
#define MMCHS_CMD37                 (0x25000000)
#define MMCHS_CMD38                 (0x26000000)
#define MMCHS_CMD39                 (0x27000000)
#define MMCHS_CMD40                 (0x28000000)
#define MMCHS_CMD41                 (0x29000000)
#define MMCHS_CMD42                 (0x2A000000)
#define MMCHS_CMD43                 (0x2B000000)
#define MMCHS_CMD44                 (0x2C000000)
#define MMCHS_CMD45                 (0x2D000000)
#define MMCHS_CMD46                 (0x2E000000)
#define MMCHS_CMD47                 (0x2F000000)
#define MMCHS_CMD48                 (0x30000000)
#define MMCHS_CMD49                 (0x31000000)
#define MMCHS_CMD50                 (0x32000000)
#define MMCHS_CMD51                 (0x33000000)
#define MMCHS_CMD52                 (0x34000000)
#define MMCHS_CMD53                 (0x35000000)
#define MMCHS_CMD54                 (0x36000000)
#define MMCHS_CMD55                 (0x37000000)
#define MMCHS_CMD56                 (0x38000000)
#define MMCHS_CMD57                 (0x39000000)
#define MMCHS_CMD58                 (0x3A000000)
#define MMCHS_CMD59                 (0x3B000000)
#define MMCHS_CMD60                 (0x3C000000)
#define MMCHS_CMD61                 (0x3D000000)
#define MMCHS_CMD62                 (0x3E000000)
#define MMCHS_CMD63                 (0x3F000000)

/*command type*/
#define MMCHS_CMD_NORMAL            (0x00000000)
#define MMCHS_CMD_SUSP              (0x1<<22)
#define MMCHS_FUNC_SEL              (0x2<<22)
#define MMCHS_CMD_ABORT             (0x3<<22) //selwin
/*other fileds*/
#define MMCHS_CMD_DMA               (0x00000001)    /*DMA mode */
#define MMCHS_CMD_BCE               (0x00000002)    /*block count enable */
#define MMCHS_CMD_ACEN              (0x00000004)    /*auto cmd12 enable */
#define MMCHS_CMD_DDIR              (0x00000010)    /*data direction card to host */
#define MMCHS_CMD_READ              (0x00000010)    /*read, card to host */
#define MMCHS_CMD_WRITE             (0x00000000)    /*write, host to card */
#define MMCHS_CMD_MSBS              (0x00000020)    /*multiblockselect */
#define MMCHS_CMD_CCCE              (0x00080000)    /*cmd crc enable */
#define MMCHS_CMD_CICE              (0x00100000)    /*cmd index check enable */
#define MMCHS_CMD_DP                (0x00200000)    /*data present */

/*================== COMMANDS =================================================*/

#define MMCHS_GO_IDLE_STATE           (MMCHS_CMD0  | MMCHS_RSP_NONE)
#define MMCHS_SEND_OP_COND            (MMCHS_CMD1  | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_ALL_SEND_CID            (MMCHS_CMD2  | MMCHS_RSP_LEN136 |MMCHS_CMD_CCCE | MMCHS_CMD_NORMAL)
#define MMCHS_SET_RELATIVE_ADDR       (MMCHS_CMD3  | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_SET_DSR                 (MMCHS_CMD4  | MMCHS_RSP_NONE | MMCHS_CMD_NORMAL)
#define MMCHS_SELECT_CARD             (MMCHS_CMD7  | MMCHS_RSP_LEN48B | MMCHS_CMD_NORMAL)
#define MMCHS_DESELECT_CARD           (MMCHS_CMD7  | MMCHS_RSP_NONE | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_CSD                (MMCHS_CMD9  | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_CID                (MMCHS_CMD10 | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_CID_SPI            (MMCHS_CMD9  | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
/*ALONG WITH CMD11, ENABLE STREAM MODE IN MMCHS_CON REG*/
#define MMCHS_READ_DAT_UNTIL_STOP     (MMCHS_CMD11 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
#define MMCHS_STOP_TRANSMISSION       (MMCHS_CMD12 | MMCHS_RSP_LEN48B | MMCHS_CMD_ABORT |MMCHS_CMD_DDIR)


#define MMCHS_SEND_STATUS             (MMCHS_CMD13 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_GO_INACTIVE_STATE       (MMCHS_CMD15 | MMCHS_RSP_NONE | MMCHS_CMD_NORMAL)
#define MMCHS_SET_BLOCKLEN            (MMCHS_CMD16 | MMCHS_CMD_CCCE | MMCHS_CMD_CICE | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_READ_SINGLE_BLOCK       (MMCHS_CMD17 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
#define MMCHS_READ_MULTIPLE_BLOCK     (MMCHS_CMD18 | MMCHS_RSP_LEN48 | MMCHS_CMD_MSBS | MMCHS_CMD_BCE | MMCHS_CMD_READ | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
// ADMA READ CMD
#define MMCHS_READ_SINGLE_BLOCK_ADMA   (MMCHS_READ_SINGLE_BLOCK | MMCHS_CMD_DMA | MMCHS_CMD_BCE)
#define MMCHS_READ_MULTIPLE_BLOCK_ADMA (MMCHS_READ_MULTIPLE_BLOCK | MMCHS_CMD_DMA)
/*ALONG WITH CMD 20, ENABLE STREAM MODE IN MMCHS_CON REG*/
#define MMCHS_WRITE_DAT_UNTIL_STOP    (MMCHS_CMD20 | MMCHS_RSP_LEN48B | MMCHS_CMD_DP | MMCHS_CMD_NORMAL | MMCHS_CMD_WRITE)

#define MMCHS_SET_BLOCK_COUNT      	  (MMCHS_CMD23 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL | MMCHS_CMD_WRITE | MMCHS_CMD_BCE)

#define MMCHS_WRITE_SINGLE_BLOCK      (MMCHS_CMD24 | MMCHS_RSP_LEN48 | MMCHS_CMD_DP | MMCHS_CMD_NORMAL | MMCHS_CMD_WRITE)
#define MMCHS_WRITE_MULTIPLE_BLOCK    (MMCHS_CMD25 | MMCHS_RSP_LEN48 | MMCHS_CMD_DP | MMCHS_CMD_MSBS | MMCHS_CMD_BCE | MMCHS_CMD_WRITE | MMCHS_CMD_NORMAL) // MMCHS_CMD_ACEN)
// ADMA WRITE CMD
#define MMCHS_WRITE_SINGLE_BLOCK_ADMA    (MMCHS_WRITE_SINGLE_BLOCK | MMCHS_CMD_DMA | MMCHS_CMD_BCE)
#define MMCHS_WRITE_MULTIPLE_BLOCK_ADMA  (MMCHS_WRITE_MULTIPLE_BLOCK | MMCHS_CMD_DMA) // MMCHS_CMD_ACEN)

#define MMCHS_PROGRAM_CID             (MMCHS_CMD26 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_PROGRAM_CSD             (MMCHS_CMD27 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_SET_WRITE_PROT          (MMCHS_CMD28 | MMCHS_RSP_LEN48B | MMCHS_CMD_NORMAL)
#define MMCHS_CLR_WRITE_PROT          (MMCHS_CMD29 | MMCHS_RSP_LEN48B | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_WRITE_PROT         (MMCHS_CMD30 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_NORMAL)
#define MMCHS_ERASE_GROUP_START       (MMCHS_CMD35 |MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_ERASE_GROUP_END         (MMCHS_CMD36 |MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_ERASE                   (MMCHS_CMD38 | MMCHS_RSP_LEN48B | MMCHS_CMD_NORMAL)
#define MMCHS_FAST_IO                 (MMCHS_CMD39 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_GO_IRQ_STATE            (MMCHS_CMD40 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_CRC_ON_OFF_SPI          (MMCHS_CMD59 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
//SD ACMD Commands
#define SDHS_SET_BUS_WIDTH            (MMCHS_CMD6  | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_STATUS                   (MMCHS_CMD13 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_SEND_NUM_WR_BLOCKS       (MMCHS_CMD22 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_NORMAL)
#define SDHS_SET_WR_BLK_ERASE_COUNT   (MMCHS_CMD23 | MMCHS_RSP_LEN48 | MMCHS_CMD_WRITE | MMCHS_CMD_NORMAL)
#define SDHS_APP_OP_COND              (MMCHS_CMD41 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_SET_CLR_CARD_DETECT      (MMCHS_CMD42 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_SEND_SCR                 (MMCHS_CMD51 | MMCHS_RSP_LEN48 | MMCHS_CMD_DP| MMCHS_CMD_NORMAL)

//SD commands
#define SDHS_ALL_SEND_CID              (MMCHS_CMD2 | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
#define SDHS_SEND_RELATIVE_ADDR       (MMCHS_CMD3 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_APP_CMD                  (MMCHS_CMD55| MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_LOCK_UNLOCK              (MMCHS_CMD42| MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_NORMAL)
#define SDHS_GEN_CMD_WR               (MMCHS_CMD56| MMCHS_RSP_LEN48 | MMCHS_CMD_WRITE | MMCHS_CMD_NORMAL)
#define SDHS_GEN_CMD_RD               (MMCHS_CMD56| MMCHS_RSP_LEN48B| MMCHS_CMD_READ | MMCHS_CMD_NORMAL)
#define SDHS_ERASE_WR_BLK_START       (MMCHS_CMD32| MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_ERASE_WR_BLK_END         (MMCHS_CMD33| MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_EXT_CSD            (MMCHS_CMD8 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_SWITCH             (MMCHS_CMD6 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_BUSTEST_R               (MMCHS_CMD14 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
#define MMCHS_BUSTEST_W               (MMCHS_CMD19 | MMCHS_RSP_LEN48 | MMCHS_CMD_WRITE| MMCHS_CMD_DP | MMCHS_CMD_NORMAL)

//#endif //MMC_C_PRIVATE
/*================== END MMC PRIVATE DATA=======================================*/

#define MMC_CARD               1    // MMC card
#define SD_CARD                2    // SD card
#define HS_MMC_CARD            3    // MMC card

#define    MMC_SLOT_1          1
#define    MMC_SLOT_2          2
#define    MMC_SLOT_3          3
#define    MMC_SLOT_4          4
#define    MMC_SLOT_5          5

#define    MMC_VOLT_1_8        1
#define    MMC_VOLT_3_0        2

#define    MMC_DATAWIDTH_8_BITS     8
#define    MMC_DATAWIDTH_4_BITS     4
#define    MMC_DATAWIDTH_1_BITS     1

#define    MMC_TRANFER_ENABLE       1
#define    MMC_TRANFER_DISABLE      0

#define    MMC_OPENDRAIN_ENABLE     1
#define    MMC_OPENDRAIN_DISABLE    0

#define    MMC_BLOCK_SIZE           512
#define    ERASE_GRP_SIZE           (512 * 1024)

#define    MMC_RSP1    1
#define    MMC_RSP2    2
#define    MMC_RSP3    3
#define    MMC_RSP4    4
#define    MMC_RSP5    5
#define    MMC_RSP6    6
#define    MMC_RSP7    7

/*================== FUNCTIONS =================================================*/

U32 mmc_config(U16 sid, U16 mmc_volt, U8 data_width, U16 * card_rca, U8 * card_type,
               U8 * max_data_width, U32 * max_trans_clk, U64 * card_size);
U32 mmc_controller_config(U8 mmc_slot, U16 mmc_volt);
U32 check_mmc_inserted(U8 mmc_slot);
U32 sd_mmc_card_init(U8 mmc_slot, U8 data_width, U8 * card_type, U16 * card_rca,
                     U8 * max_data_width, U32 * max_trans_clk,
                     U64 * size_of_card);
void get_card_type(U8 mmc_slot, U8 * card_type, U16 * card_rca);
U32 get_card_details(U8 mmc_slot, U16 card_rca, U8 * card_type,
                     U8 * max_data_width, U32 * max_trans_clk,
                     U64 * size_of_card);
U32 mmc_controller_reset(U8 mmc_slot);
U32 set_mmc_sdvs_voltage(U8 mmc_slot, U16 mmc_volt);
U32 set_mmc_controller_clk(U8 mmc_slot, U32 mmc_clk);
S32 send_mmc_cmd_ex(U8 mmc_slot, U32 cmd, U16 argh, U16 argl, BOOLEAN *enable_trace);
S32 send_mmc_cmd(U8 mmc_slot, U32 cmd, U16 argh, U16 argl);
U32 get_mmc_cmd_response(U8 mmc_slot, U32 * resp, U8 response_type);
U32 mmc_cmd_status(U8 mmc_slot);
U32 mmc_disable_pwr(U8 slot, U16 volt);
U32 mmc_enable_pwr(U8 slot, U16 volt);
U32 get_sd_mmc_csd(U16 sid, U16 card_rca, U8 card_type, U16 len,
                   const void *dat);
U32 get_sd_mmc_cid(U16 sid, U16 card_rca, U8 card_type, U16 len,
                   const void *dat);
U32 get_sd_mmc_csd_ex(U16 sid, U16 card_rca, U8 card_type, U16 regs[8]);
U32 get_sd_mmc_cid_ex(U16 sid, U16 card_rca, U8 card_type, U16 regs[8]);
U32 send_mmc_init_sequence(U8 mmc_slot);
U32 mmc_opendrain_enable_disable(U8 mmc_slot, U8 enable);
U32 mmc_read_ext_csd(U8 slot, U8 data_width, U16 card_rca, U8 card_type, U8 * buff);
U32 sd_mmc_card_read(U16 sid, U16 card_rca, U8 card_type, U8 data_width, U32 transfer_clk, U32 mblock, U32 ddr, U32 start_block, U32 nu_blocks, U32 len, void *buf);
U32 sd_mmc_card_write(U16 sid, U16 card_rca, U8 card_type, U8 data_width, U32 transfer_clk, U32 mblock, U32 ddr, U32 start_block, U32 nu_blocks, U32 len, void *buf);
U32 sd_mmc_card_erase(U16 sid, U16 card_rca, U8 card_type, U8 data_width,
                      U32 transfer_clk, U32 ddr,
                      U32 start_block, U32 nu_blocks,
                      //U32 status_offset, U32 status_length, void *buf);
                      void *buf);
S32 mmc_disable(U16 slot, U8 volt);
U32 mmc_datatransfer_enable_disable(U8 mmc_slot, U8 enable, U16 card_rca);
U32 is_card_tranfer_state(U8 mmc_slot, U16 card_rca);
U32 get_card_state(U8 mmc_slot, U16 card_rca, U32 * state);
U32 mmc_bustest_w(U8 mmc_slot, U8 data_width);
U32 mmc_bustest_r(U8 mmc_slot, U8 data_width);
U32 mmc_buswidth_detect(U8 mmc_slot, U16 card_rca, void *max_bus_width);
U32 data_transfer_complete(U8 mmc_slot);
S32 reset_mmc_lines(U8 mmc_slot, U32 reset_val);
S32 set_hsmmc_clk_data_width(U8 mmc_slot, U8 card_type, U16 card_rca, U8 data_width,
                             U32 transfer_clk, U32 ddr);
U32 is_write_protect_card(U16 mmc_slot);
U32 mmc_send_switch_command(U8 mode, U8 mmc_slot, U8 ext_csd_offset, U8 data);

/*=================== DMA SECTION ===========================================*/

/*=================== MACROS=================================================*/
#define ADMA_ATTR_VALID         (0x1 << 0)
#define ADMA_ATTR_END           (0x1 << 1)
#define ADMA_ATTR_INTR          (0x1 << 2)
#define ADMA_ATTR_ACT_NOOP      (0x0 << 4)    
#define ADMA_ATTR_ACT_TRAN      (0x2 << 4)    
#define ADMA_ATTR_ACT_LINK      (0x3 << 4)    

#define MAX_ADMA_DESC_LEN   0xFFF0
#define MAX_ADMA_DESC_NUM   0x20

/*=================== DMA FUNCTIONS =================================================*/

typedef struct 
{
  U32 attr;       
  U32 addr_32;
} omap_adma2_desc;

U32 mmc_adma_config(U8 mmc_slot, U32 addr, U32 len);

/* EXT CSD Structure */

typedef struct
{
  //Modes segment
  U8 undefined1[134];              //  [133:0]    0      Reserved
  U8 sec_bad_blk_mgmnt;            //  [134]      R|W    Bad Block Management mode
  U8 undefined2;                   //  [135]      0      Reserved
  U8 enh_start_addr[4];            //  [139:136]  R|W    Enhanced User Data Start Address
  U8 enh_size_mult[3];             //  [142:140]  R|W    Enhanced User Data Area Size
  U8 gp_size_mult[12];             //  [154:143]  R|W    General Purpose Partition Size
  U8 partition_setting_cp;         //  [155]      R|W    Paritioning Setting
  U8 partitions_attribute;         //  [156]      R|W    Partitions attribute
  U8 max_enh_size_mult[3];         //  [159:157]  R      Max Enhanced Area Size
  U8 partitioning_support;         //  [160]      R      Partitioning Support
  U8 hpi_mgmt;                     //  [161]      R|W    HPI management
  U8 rst_n_function;               //  [162]      R|W    H/W reset function
  U8 bkops_en;                     //  [163]      R|W    Enable background operations handshake
  U8 bkops_start;                  //  [164]      W      Manually start background operations
  U8 undefined3;                   //  [165]      0      Reserved
  U8 wr_rel_param;                 //  [166]      R      Write reliability parameter register
  U8 wr_rel_set;                   //  [167]      R|W    Write reliability setting register
  U8 rpmb_size_mult;               //  [168]      R      RPMB Size
  U8 fw_config;                    //  [169]      R|W    FW configuration
  U8 undefined4;                   //  [170]      0      Reserved
  U8 user_wp;                      //  [171]      R|W    User area write protection register
  U8 undefined5;                   //  [172]      0      Reserved
  U8 boot_wp;                      //  [173]      R|W    Boot area write protection register
  U8 undefined6;                   //  [174]      0      Reserved
  U8 erase_group_def;              //  [175]      R|W    High-density erase group definition
  U8 undefined7;                   //  [176]      0      Reserved
  U8 boot_bus_width;               //  [177]      R|W    Boot bus width1
  U8 boot_config_prot;             //  [178]      R|W    Defines boot configuration protection
  U8 partition_config;             //  [179]      R|W    Partition configuration
  U8 undefined8;                   //  [180]      0      Reserved
  U8 erased_mem_cont;              //  [181]      R      Erased memory content
  U8 undefined9;                   //  [182]      0      Reserved
  U8 bus_width;                    //  [183]      W      Bus width mode
  U8 undefined10;                  //  [184]      0      Reserved
  U8 hs_timing;                    //  [185]      R|W    High-speed interface timing
  U8 undefined11;                  //  [186]      0      Reserved
  U8 power_class;                  //  [187]      R|W    Power class
  U8 undefined12;                  //  [188]      0      Reserved
  U8 cmd_set_rev;                  //  [189]      R      Command set revision
  U8 undefined13;                  //  [190]      0      Reserved
  U8 cmd_set;                      //  [191]      R|W    Command set
  //Properties Segment
  U8 ext_csd_rev;                  //  [192]      R      Extended CSD revision
  U8 undefined14;                  //  [193]      0      Reserved
  U8 csd_structure;                //  [194]      R      CSD structure version
  U8 undefined15;                  //  [195]      0      Reserved
  U8 card_type;                    //  [196]      R      Card type
  U8 undefined16[2];               //  [198:197]  0      Reserved
  U8 partition_switch_time;        //  [199]      R      Maximum partition switch time in 10 ms steps
  U8 pwr_cl_52_195;                //  [200]      R      Power class for 52MHz at 1.95V
  U8 pwr_cl_26_195;                //  [201]      R      Power class for 26MHz at 1.95V
  U8 pwr_cl_52_360;                //  [202]      R      Power class for 52MHz at 3.6V
  U8 pwr_cl_26_360;                //  [203]      R      Power class for 26MHz at 3.6V
  U8 undefined17;                  //  [204]      0      Reserved
  U8 min_perf_r_4_26;              //  [205]      R      Minimum Read Performance for 4bit at 26MHz
  U8 min_perf_w_4_26;              //  [206]      R      Minimum Write Performance for 4bit at 26MHz
  U8 min_perf_r_8_26_4_52;         //  [207]      R      Minimum Read Performance for 8bit at 26MHz, for 4bit at 52MHz
  U8 min_perf_w_8_26_4_52;         //  [208]      R      Minimum Write Performance for 8bit at 26MHz, for 4bit at 52MHz
  U8 min_perf_r_8_52;              //  [209]      R      Minimum Read Performance for 8bit at 52MHz
  U8 min_perf_w_8_52;              //  [210]      R      Minimum Write Performance for 8bit at 52MHz
  U8 sec_count[4];                 //  [215:212]  R      Sector Count
  U8 undefined18;                  //  [216]      0      Reserved
  U8 undefined19;                  //  [211]      0      Reserved
  U8 s_a_timeout;                  //  [217]      R      Sleep/awake timeout
  U8 undefined20;                  //  [218]      0      Reserved
  U8 s_c_vccq;                     //  [219]      R      Sleep current (VCCQ)
  U8 s_c_vcc;                      //  [220]      R      Sleep current (VCC)
  U8 hc_wp_grp_size;               //  [221]      R      High-capacity write protect group size
  U8 rel_wr_sec_c;                 //  [222]      R      Reliable write sector count
  U8 erase_timeout_mult;           //  [223]      R      High-capacity erase timeout
  U8 hc_erase_grp_size;            //  [224]      R      High-capacity erase unit size
  U8 acc_size;                     //  [225]      R      Access size
  U8 boot_size_mult;               //  [226]      R      Boot partition size
  U8 undefined21;                  //  [227]      0      Reserved
  U8 boot_info;                    //  [228]      R      Boot information
  U8 undefined22[275];             //  [503:229]  0      Reserved
  U8 s_cmd_set;                    //  [504]      R      Supported Command Sets
  U8 undefined23[7];               //  [511:505]  0      Reserved
} T_ext_csd;

#define EXT_CSD_OFFSET(x)           ((U32)(&(((T_ext_csd *)0)->x)))

// EXT CSD PARTITION_CONFIG (before BOOT_CONFIG) [179] - This register defines the configuration for partitions.

// Bit 7    Bit 6     Bit 5 Bit 4 Bit 3       Bit 2 Bit 1 Bit 0
// Reserved BOOT_ACK  BOOT_PARTITION_ENABLE   PARTITION_ACCESS
//          R/W/E     R/W/E                   R/W/E_P

#define BOOT_ACK_DISABLE                      (0x0 << 6)  // No boot acknowledge sent (default)
#define BOOT_ACK_ENABLE                       (0x1 << 6)  // Boot acknowledge sent during boot operation
#define BOOT_PARTITION_NONE                   (0x0 << 3)  // Device not boot enabled (default)
#define BOOT_PARTITION_BP1                    (0x1 << 3)  // Boot partition 1 enabled for boot
#define BOOT_PARTITION_BP2                    (0x2 << 3)  // Boot partition 2 enabled for boot
#define BOOT_PARTITION_UDA                    (0x7 << 3)  // User area enabled for boot
#define PARTITION_ACCESS_NONE                 (0x0 << 0)  // No access to boot partition (default)
#define PARTITION_ACCESS_BP1                  (0x1 << 0)  // R/W boot partition 1
#define PARTITION_ACCESS_BP2                  (0x2 << 0)  // R/W boot partition 2
#define PARTITION_ACCESS_RPMB                 (0x3 << 0)  // R/W Replay Protected Memory Block (RPMB)
#define PARTITION_ACCESS_GPP1                 (0x4 << 0)  // Access to General Purpose partition 1
#define PARTITION_ACCESS_GPP2                 (0x5 << 0)  // Access to General Purpose partition 2
#define PARTITION_ACCESS_GPP3                 (0x6 << 0)  // Access to General Purpose partition 3
#define PARTITION_ACCESS_GPP4                 (0x7 << 0)  // Access to General Purpose partition 4
#define PARTITION_ACCESS_MASK                 0x07
#define BOOT_PARTITION_MASK                   0x38
#define BOOT_ACK_MASK                         0x40
#define PARTITION_CONFIG__PARTITION_ACCESS(x) (x & PARTITION_ACCESS_MASK)
#define PARTITION_CONFIG__BOOT_PARTITION(x)   (x & BOOT_PARTITION_MASK)
#define PARTITION_CONFIG__BOOT_ACK(x)         (x & BOOT_ACK_MASK)

#define RST_N_ENABLE_TEMP_DISABLED            0x00
#define RST_N_ENABLE_PERM_ENABLED             0x01
#define RST_N_ENABLE_PERM_DISABLED            0x02
#define RST_N_ENABLE_MASK                     0x03
#define RST_N_FUNCTION__RST_N_ENABLE(x)       (x & RST_N_ENABLE_MASK)

#define SWITCH_MODE_CMD_SET   0x00
#define SWITCH_MODE_SET_BITS  0x01
#define SWITCH_MODE_CLR_BITS  0x02
#define SWITCH_MODE_WR_BYTE   0x03

/*======================= EOF =================================================*/
#endif /*MMC_H */
