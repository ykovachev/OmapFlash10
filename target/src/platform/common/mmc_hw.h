/* mmc_hw.h - header file for sample code demonstrating the MMC/SD interface */

/*
 * Copyright (c) Texas Instruments 2004. 
 */


/*
DESCRIPTION
-----------
This is an header file for sample code demonstrating the interface the MMC/SD
with necessary HW definitions.
The code in this example will perform interface to the following
specifications:
  OMAP2430 Technical Reference Manual
  MMC/SD Card Specifications
  H4 Sample Register Map
*/


/* includes */
#ifndef CSST_INC_MMC_HW 
#define CSST_INC_MMC_HW


/* defines */
#ifdef REG8
#undef REG8
#endif
#ifdef REG16
#undef REG16
#endif
#ifdef REG32
#undef REG32
#endif


#define REG16(addr)		(*(volatile U16*)(addr))
#define REG32(addr)		(*(volatile U32*)(addr))

#define PREG32(addr)	REG32(addr)
#define PREG16(addr)	REG16(addr)

#define MMC_STUFF_BITS 0xFFFF
/*MMC HS offsets*/
#define MMCHS_BASE(mmcSlot)  	((0x4809C000) + ((0x18000)*(mmcSlot-1)))
/*This is the slot 2 base address*/
//#define MMCHS_BASE()   (0x480B4000)


#define MMCHS_SYSC		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x010)
#define MMCHS_SYSS		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x014)
#define MMCHS_CSRE		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x024)
#define MMCHS_SYST		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x028)
#define MMCHS_CON 		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x02C)
#define MMCHS_BLK 		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x104)
#define MMCHS_ARG 		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x108)
#define MMCHS_CMD 		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x10C)
#define MMCHS_RSP10 	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x110)
#define MMCHS_RSP32 	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x114)
#define MMCHS_RSP54 	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x118)
#define MMCHS_RSP76 	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x11C)
#define MMCHS_DATA 	  	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x120)
#define MMCHS_PSTAT 	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x124)
#define MMCHS_HCTL 	  	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x128)
#define MMCHS_SYSCTL 	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x12C)
#define MMCHS_STAT	 	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x130)
#define MMCHS_IE 		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x134)
#define MMCHS_ISE		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x138)
#define MMCHS_AC12 		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x13C)
#define MMCHS_CAPA  	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x140)
#define MMCHS_CURCAPA	(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x148)
#define MMCHS_REV 		(volatile U32 *)(MMCHS_BASE(mmcSlot) + 0x1FC)

/*to evaluate the base at runtime*/
#ifdef MMC_SELECTSLOT
U32 mmcSlot; //default slot number 1
#else
extern U32 mmcSlot;
#endif

/*Macros to get and set*/
#define MMCHS_GET_R(reg)			(*reg)
#define MMCHS_SET_R(reg,val)	    (*reg = val)
#define MMCHS_FSET_R(reg,val)       ((*reg)|=val)
#define MMCHS_ASET_R(reg,val)       ((*reg)&=val)
#define MMCHS_STUFF_BITS		    (0x0000)


/*MMCHS_SYSC (sysconfig) register fields*/
#define SYSC_AUTOIDLE           ((0x1)<<0)  /*auto clock gating */
#define SYSC_SOFTRESET          ((0x1)<<1)  /*module reset*/
#define SYSC_ENWKUP             ((0x1)<<2)  /*wakeup feature*/
#define SYSC_SIDLEMODE  /*4:3 BITS  Have to set */
#define SYSC_CLKACT_CLKOFF      ((0x0)<<8) /*OCP FCLK switch off*/
#define SYSC_CLKACT_OCP         ((0x1)<<8)
#define SYSC_CLKACT_FCLK        ((0x2)<<8)
#define SYSC_CLKACT_OCP_FCLK    ((0x3)<<8)


/* MMCHS_SYSS (sysstaus) register */
#define MMCHS_SYSS_RESETDONE   ((0x1)<<0)

/*MMCHS_CSRE register fields*/

/*MMCHS_SYST (SYETEM TEST) register*/
#define MMCHS_SYST_MCKD        ((0x1)<<0) /*OUTPUT CLOCK*/
#define MMCHS_SYST_CDIR_H2C    ((0x0)<<1) /* cmd direction host to card)*/
#define MMCHS_SYST_CDIR_C2H    ((0x1)<<1) /* cmd direction card to host)*/
#define MMCHS_SYST_CDAT         <<<NEED TO ADD< !!!!!!!!
#define MMCHS_SYST_DDIR_H2C    ((0X0)<<3) /*DATA direction host to card*/
#define MMCHS_SYST_DDIR_C2H    ((0X1)<<3) /*data direction card to host*/
#define MMCHS_SYST_SSB         ((0x1)<<12) /*set status bit clear statusreg*/
#define MMCHS_SYST_WAKD_LOW    ((0x0)<<13)    
#define MMCHS_SYST_WAKD_HIGH   ((0x1)<<13) 
#define MMCHS_SYST_SDWP        ((0x1)<<14) /*card write protect*/    
#define MMCHS_SYST_SDCD        ((0X1)<<15) /* card detect */
#define MMCHS_HCTL_DTW         (0x1<<1)

/*CON REG bit fields*/
#define MMCHS_ODRN          ((0x1)<<0) /*open drain*//*for broad cast cmds 1,2,3 and 40 must set*/
#define MMCHS_INIT          ((0x1)<<1) /*init sequence*/
#define MMCHS_CON_HR        ((0x1)<<2) /* HOST generates 48bit resp*/
#define MMCHS_CON_STR       ((0x1)<<3) /* enable stream orented data transfer*/
//#define MMCHS_CON_MODE 
#define MMCHS_CON_DW8       ((0x1)<<5) /* enable 8 bit data width*/
#define MMCHS_CON_MIT       ((0x1)<<6) /* command timeout disable*/
#define MMCHS_CON_CDP       ((0x1)<<7) /*card detect polarity*/
//#define MMCHS_CON_WPP
#define MMCHS_CTO	        (0x0)<<6 /*command timeout enable*/

/*MMCHS NBLK reg values*/
#define   MMCHS_NBLK_ONE	0x00010000
#define   MMCHS_BLKSZ_512	0x00000200

/*SYSCTL REG*/
#define MMCHS_DTO_MAX    ((0xE)<<16) /*data timeout*/
#define MMCHS_SYSCTL_ICE        (0x1<<0)  /*internal clock enable*/
#define MMCHS_SYSCTL_ICS        (0x1<<1)
#define MMCHS_SYSCTL_CEN	    (0x1<<2) /*clock enable*/
#define CLK_DIV		            (0x4)       /*this value should be changed based on the fclk value*/
#define MMCHS_SYSCTL_CLKD_1     (0x1<<6)
#define MMCHS_SYSCTL_CLKD_2     (0x2<<6)
#define MMCHS_SYSCTL_CLKD_4     (0x4<<6)
#define MMCHS_SYSCTL_CLKD_5     (0x5<<6)
#define MMCHS_SYSCTL_CLKD_8     (0x8<<6)
#define MMCHS_SYSCTL_CLKD_48    (0x30<<6)
#define MMCHS_SYSCTL_RESET	    (0x2) /*reset*/
#define CLK_DIV0	   ((0x0)<<6)

/*HCTL reg*/
#define MMCHS_SDVS_1_8V (0x5<<9) /*select 1.8v*/
#define MMCHS_SDVS_3_0V (0x6<<9) /*select 3.0v*/ 
#define MMCHS_SDVS_3_3V (0x7<<9) /*select 3.3v*/
#define MMCHS_SDBP      (0x0100) /*enable sd bus pwr*/
#define MMCHS_HCTL_IBG      (0x1<<19)
#define MMCHS_HCTL_RWC      (0x1<<18)
#define MMCHS_HCTL_CR       (0x1<<17)
/*CAPA Reg*/
#define MMCHS_1_8V      (0x1<<26)
#define MMCHS_3_0V		(0x1<<25)
#define MMCHS_3_3V		(0x1<<24)


/*  MMCHS STAT field values  */
/*Error interrupt status*/
#define MMCHS_MMCSTAT_BADA 					(0x20000000) /*Bad access to data space*/
#define MMCHS_MMCSTAT_CERR 					(0x10000000) /*card error*/
#define MMCHS_MMCSTAT_ACE 					(0x01000000) /*Auto cmd12 er*/
#define MMCHS_MMCSTAT_DEB 					(0x00400000) /*Data end bit er*/
#define MMCHS_MMCSTAT_DCRC 					(0x00200000) /*Data crc er*/
#define MMCHS_MMCSTAT_DTO 					(0x00100000) /*Data timeout*/
#define MMCHS_MMCSTAT_CIE 					(0x00080000) /*command index er*/
#define MMCHS_MMCSTAT_CEB	  				(0x00040000) /*command endbit er*/
#define MMCHS_MMCSTAT_CCRC					(0x00020000) /*command crc error*/
#define MMCHS_MMCSTAT_CTO 					(0x00010000) /*command timeout*/
/*normal interrupt status*/
#define MMCHS_MMCSTAT_ERRI					(0x8000) /*error interrupt*/
#define MMCHS_MMCSTAT_CIRQ                  (0x0100) /*card interrupt, only for SD*/
#define MMCHS_MMCSTAT_CREM	                (0x0080) /*card removed*/
#define MMCHS_MMCSTAT_CINS                  (0x0040) /*card inserted*/
#define MMCHS_MMCSTAT_BRR                   (0x0020) /*buffer read ready*/
#define MMCHS_MMCSTAT_BWR                   (0x0010) /*buffer write ready*/
#define MMCHS_MMCSTAT_DMA                   (0x0008) /*DMA interrupt, not supported*/
#define MMCHS_MMCSTAT_BGE			        (0x0004) /*block gap event*/
#define MMCHS_MMCSTAT_TC			        (0x0002) /*transfer complete*/
#define MMCHS_MMCSTAT_CC                    (0x0001) /*command complete*/

/*  MMCHS CMD field values  */
/*Response lengths*/
#define MMCHS_RSP_LEN48				  (0x00020000)
#define MMCHS_RSP_LEN48B			  (0x00030000)
#define MMCHS_RSP_LEN136			  (0x00010000)
#define MMCHS_RSP_NONE				  (0x00000000)

/*command index*/
#define MMCHS_CMD0					  (0x00000000)
#define MMCHS_CMD1					  (0x01000000)
#define MMCHS_CMD2					  (0x02000000)
#define MMCHS_CMD3					  (0x03000000)
#define MMCHS_CMD4					  (0x04000000)
#define MMCHS_CMD5					  (0x05000000)
#define MMCHS_CMD6					  (0x06000000)
#define MMCHS_CMD7					  (0x07000000)
#define MMCHS_CMD8					  (0x08000000)
#define MMCHS_CMD9	     			  (0x09000000)
#define MMCHS_CMD10                   (0x0A000000)
#define MMCHS_CMD11                   (0x0B000000)
#define MMCHS_CMD12                   (0x0C000000)
#define MMCHS_CMD13                   (0x0D000000)
#define MMCHS_CMD14                   (0x0E000000)
#define MMCHS_CMD15                   (0x0F000000)
#define MMCHS_CMD16                   (0x10000000)
#define MMCHS_CMD17                   (0x11000000)
#define MMCHS_CMD18                   (0x12000000)
#define MMCHS_CMD19                   (0x13000000)
#define MMCHS_CMD20                   (0x14000000)
#define MMCHS_CMD21                   (0x15000000)
#define MMCHS_CMD22                   (0x16000000)
#define MMCHS_CMD23                   (0x17000000)
#define MMCHS_CMD24                   (0x18000000)
#define MMCHS_CMD25                   (0x19000000)
#define MMCHS_CMD26                   (0x1A000000)
#define MMCHS_CMD27                   (0x1B000000)
#define MMCHS_CMD28                   (0x1C000000)
#define MMCHS_CMD29                   (0x1D000000)
#define MMCHS_CMD30                   (0x1E000000)
#define MMCHS_CMD31                   (0x1F000000)
#define MMCHS_CMD32                   (0x20000000)
#define MMCHS_CMD33                   (0x21000000)
#define MMCHS_CMD34                   (0x22000000)
#define MMCHS_CMD35                   (0x23000000)
#define MMCHS_CMD36                   (0x24000000)
#define MMCHS_CMD37                   (0x25000000)
#define MMCHS_CMD38                   (0x26000000)
#define MMCHS_CMD39                   (0x27000000)
#define MMCHS_CMD40                   (0x28000000)
#define MMCHS_CMD41                   (0x29000000)
#define MMCHS_CMD42                   (0x2A000000)
#define MMCHS_CMD43                   (0x2B000000)
#define MMCHS_CMD44                   (0x2C000000)
#define MMCHS_CMD45                   (0x2D000000)
#define MMCHS_CMD46                   (0x2E000000)
#define MMCHS_CMD47                   (0x2F000000)
#define MMCHS_CMD48                   (0x30000000)
#define MMCHS_CMD49                   (0x31000000)
#define MMCHS_CMD50                   (0x32000000)
#define MMCHS_CMD51                   (0x33000000)
#define MMCHS_CMD52                   (0x34000000)
#define MMCHS_CMD53                   (0x35000000)
#define MMCHS_CMD54                   (0x36000000)
#define MMCHS_CMD55                   (0x37000000)
#define MMCHS_CMD56                   (0x38000000)
#define MMCHS_CMD57                   (0x39000000)
#define MMCHS_CMD58                   (0x3A000000)
#define MMCHS_CMD59                   (0x3B000000)
#define MMCHS_CMD60                   (0x3C000000)
#define MMCHS_CMD61                   (0x3D000000)
#define MMCHS_CMD62                   (0x3E000000)
#define MMCHS_CMD63                   (0x3F000000)

/*command type*/
#define MMCHS_CMD_NORMAL			  (0x00000000)		
#define MMCHS_CMD_SUSP				  (0x1<<22)		
#define MMCHS_FUNC_SEL	              (0x2<<22)

/*other fileds*/
#define MMCHS_CMD_DMA				  (0x00000001) /*DMA mode*/
#define MMCHS_CMD_BCE				  (0x00000002) /*block count enable*/
#define MMCHS_CMD_ACEN				  (0x00000004) /*auto cmd12 enable*/
#define MMCHS_CMD_DDIR				  (0x00000010) /*data direction card to host*/
#define MMCHS_CMD_READ				  (0x00000010) /*read, card to host*/
#define MMCHS_CMD_WRITE				  (0x00000000) /*write, host to card*/
#define MMCHS_CMD_MSBS				  (0x00000020) /*multiblockselect*/
#define MMCHS_CMD_CCCE				  (0x00080000) /*cmd crc enable*/
#define MMCHS_CMD_CICE				  (0x00100000) /*cmd index check enable*/
#define MMCHS_CMD_DP				  (0x00200000) /*data present*/

/*COMMANDS****************************************************************************/

#define MMCHS_GO_IDLE_STATE           (MMCHS_CMD0  | MMCHS_RSP_NONE)
#define MMCHS_SEND_OP_COND            (MMCHS_CMD1  |MMCHS_CMD_CICE	| MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_ALL_SEND_CID		      (MMCHS_CMD2  | MMCHS_RSP_LEN136 |MMCHS_CMD_CCCE | MMCHS_CMD_NORMAL)
#define MMCHS_SET_RELATIVE_ADDR       (MMCHS_CMD3  | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_SET_DSR                 (MMCHS_CMD4  | MMCHS_RSP_NONE  | MMCHS_CMD_NORMAL)
#define MMCHS_SELECT_CARD             (MMCHS_CMD7  | MMCHS_RSP_LEN48B | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_CSD                (MMCHS_CMD9  | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_CID                (MMCHS_CMD10 | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_CID_SPI            (MMCHS_CMD9  | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
/*ALONG WITH CMD11, ENABLE STREAM MODE IN MMCHS_CON REG*/
#define MMCHS_READ_DAT_UNTIL_STOP     (MMCHS_CMD11 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_DP | MMCHS_CMD_NORMAL) /*ALONG WITH THIS ENABLE STREAM MODE IN MMCHS_CON REG*/
#define MMCHS_STOP_TRANSMISSION       (MMCHS_CMD12 | MMCHS_RSP_LEN48B | MMCHS_CMD_ABORT)
#define MMCHS_SEND_STATUS             (MMCHS_CMD13 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_GO_INACTIVE_STATE       (MMCHS_CMD15 | MMCHS_RSP_NONE | MMCHS_CMD_NORMAL)
#define MMCHS_SET_BLOCKLEN            (MMCHS_CMD16 | MMCHS_CMD_CCCE | MMCHS_CMD_CICE | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_READ_SINGLE_BLOCK       (MMCHS_CMD17 | MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
#define MMCHS_READ_MULTIPLE_BLOCK     (MMCHS_CMD18 | MMCHS_RSP_LEN48 | MMCHS_CMD_MSBS | MMCHS_CMD_BCE | MMCHS_CMD_READ | |MMCHS_CMD_DP|MMCHS_CMD_NORMAL)
/*ALONG WITH CMD 20, ENABLE STREAM MODE IN MMCHS_CON REG*/
#define MMCHS_WRITE_DAT_UNTIL_STOP    (MMCHS_CMD20 | MMCHS_RSP_LEN48B | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
#define MMCHS_WRITE_SINGLE_BLOCK      (MMCHS_CMD24 | MMCHS_RSP_LEN48 | MMCHS_CMD_DP | MMCHS_CMD_NORMAL)
#define MMCHS_WRITE_MULTIPLE_BLOCK    (MMCHS_CMD25 | MMCHS_RSP_LEN48 | MMCHS_CMD_DP | MMCHS_CMD_MSBS | MMCHS_CMD_BCE | MMCHS_CMD_NORMAL)
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
#define SDHS_ALL_SEND_CID		      (MMCHS_CMD2 | MMCHS_RSP_LEN136 | MMCHS_CMD_NORMAL)
#define SDHS_SEND_RELATIVE_ADDR       (MMCHS_CMD3 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)

#define SDHS_APP_CMD                  (MMCHS_CMD55| MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_LOCK_UNLOCK              (MMCHS_CMD42| MMCHS_RSP_LEN48 | MMCHS_CMD_READ | MMCHS_CMD_NORMAL)
#define SDHS_GEN_CMD_WR               (MMCHS_CMD56| MMCHS_RSP_LEN48 | MMCHS_CMD_WRITE | MMCHS_CMD_NORMAL)
#define SDHS_GEN_CMD_RD               (MMCHS_CMD56| MMCHS_RSP_LEN48B | MMCHS_CMD_READ | MMCHS_CMD_NORMAL)

#define SDHS_ERASE_WR_BLK_START       (MMCHS_CMD32| MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define SDHS_ERASE_WR_BLK_END         (MMCHS_CMD33| MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)

#define MMCHS_SEND_EXT_CSD            (MMCHS_CMD8 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)
#define MMCHS_SEND_SWITCH             (MMCHS_CMD6 | MMCHS_RSP_LEN48 | MMCHS_CMD_NORMAL)

#endif /*CSST_INC_MMC_HW */ 


