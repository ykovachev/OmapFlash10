/**
 * @file romapi_4430.h
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
 *  ROM API constants
 */   

#ifndef ROMAPI_4430_H
#define ROMAPI_4430_H

/*==== INCLUDES =============================================================*/

#include "config.h"
#include "romapi_types.h"

/*==== type =================================================================*/

typedef struct PeripheralDesc_t PER_DeviceDesc_t;

typedef struct SYS_DriverPer_t
{
  STATUS (*Init) ( PER_DeviceDesc_t* ipPeripheralDesc );
  STATUS (*Read) ( PER_DeviceDesc_t* ipPeripheralDesc );
  STATUS (*Write) ( PER_DeviceDesc_t* ipPeripheralDesc );
  STATUS (*Close) ( PER_DeviceDesc_t* ipPeripheralDesc );
  STATUS (*Configure) ( PER_DeviceDesc_t* ipPeripheralDesc, void* ipConfigDesc );
} SYS_DriverPer_t;

#ifdef ROMAPI_DEVICE_MEM
typedef enum MMCSD_Type_e 
{
  MMCSD_TYPE_UNDEFINED = 0,
  MMCSD_TYPE_MMC,
  MMCSD_TYPE_SD
  
} MMCSD_Type_e;

typedef enum MMCSD_Mode_e 
{
  MMCSD_MODE_UNDEFINED = 0,
  MMCSD_MODE_RAW,
  MMCSD_MODE_FAT
} MMCSD_Mode_e;

typedef enum MMCSD_Addressing_e 
{
  MMCSD_ADDRESSING_UNDEFINED = 0,
  MMCSD_ADDRESSING_BYTE,
  MMCSD_ADDRESSING_SECTOR  
} MMCSD_Addressing_e;

#define FAT_MAXIMUM_FAT_ENTRIES                     256

typedef struct FAT_Buffer_t 
{
  U32 SectorEntries[FAT_MAXIMUM_FAT_ENTRIES]; /* Sector entries */
  U32 SectorsTotal;     /* Number of all sector entries. */
  U32 SectorsBuffered;  /* Number of buffered sector entries */
  U32 NextCluster;      /* Next cluster for for chain operations. */
} FAT_Buffer_t;


/* Device Descriptor. Defines basic parameters for accessing the device
 * Used by Read Sectors functions */
typedef struct MEM_DeviceDesc_t
{
  BOOLEAN Initialized;  /* initialization state */
  U8     DeviceType;    /* current device type */
  U8     TrialsCount;   /* number of booting trials */
  BOOLEAN XipDevice;    /* tells if a device is XIP like */
  U16    SearchSize;    /* size of block to search for image */
  U32    BaseAddress;   /* device base address */
  U16    HsTocMask;     /* mask of TOC items to search (HS device only)*/
  U16    GpTocMask;     /* mask of TOC items to search on GP device. */
  void*  pDeviceData;   /* device dependent sub-structure pointer */
  U16*   pBootOptions;  /* Pointer to Boot options from main() */
} MEM_DeviceDesc_t, *MEM_pDeviceDesc_t;

/* Read Descriptor. Defines where to read data from and how many sectors */
typedef struct MEM_ReadDesc_t
{
  U32     StartSector;    /* first sector to copy */
  U32     SectorCount;    /* number of sectors to copy */
  U32     DestAddr;       /* destination address */
} MEM_ReadDesc_t, *MEM_pReadDesc_t;

/* ReadSectors is a common function to all device related drivers */
typedef STATUS (*pReadSectors_fn_t)( MEM_pDeviceDesc_t ipDeviceDesc, MEM_pReadDesc_t ipReadDesc);

/* Partition Entry structure */
typedef struct
{
  U8  State;              /* 0x0000   0x00 Inactive, 0x80 Active */
  U8  StartHead;          /* 0x0001   Hs Partition Start Head */
  U16 StartCylinderSector;
                            /* 0x0002   Cs, Ss Partition Start Cylinder and
                             *          Sector */
  U8  Type;               /* 0x0004   Partition type */
  U8  EndHead;            /* 0x0005   He Partition End Head */
  U16 EndCylinderSector;
                            /* 0x0006   Ce, Se Partition End Cylinder and
                             *          Sector */
                            /* The next 2 fields are 32 bits but need to be
                             * declared as 16 bits as the structure is passed
                             * as a parameter to the MBR functions and have
                             * a 2-bytes offset -> 0x1BE. */
  U16 LBA[2];             /* 0x0008   First sector of partition */
  U16 Size[2];            /* 0x000C   Number of sectors in partition */
  
} FAT_PartitionEntry_t;

/* Enumerates supported FAT systems */
typedef enum
{
  FAT12,
  FAT16,
  FAT32
} FAT_Type_e;

/* Size of a sector in memory. Nands are commonly 512 bytes this size is used
 * for each memory device */
#define SECTOR_SIZE             (512)
#define SECTOR_SIZE_SHIFT       (9)

/* A 512 bytes buffer used by each ReadSectors function */
typedef U8 SectorBuffer_t[SECTOR_SIZE];

/* This structure is used to access the FAT on a partition */
typedef struct
{
  U32 Cluster;                       /* Current Cluster */
                                                 
  SectorBuffer_t FATsSectors[2];     /* Sectors in which the FAT is 
                                      * loaded */
  U32 CurrentSector;                 /* Indicate which sector has been
                                      * loaded in the buffers */
} FAT_ChainInfo_t;

/* This structure is used in all functions to keep track of important parameters
 * of the accessed FAT */
typedef struct
{
  MEM_DeviceDesc_t * pDeviceDesc;         /* pointer to the device descriptor */
  pReadSectors_fn_t pReadSectorsFn;       /* pointer to the read sectors fn */

  FAT_PartitionEntry_t PartitionEntry;    /* Start sector and Size
                                           * of the FAT partition */

  FAT_Type_e  Type;                       /* Type of FAT */

  U32 SectorsPerCluster;                  /* Number of sectors per cluster */
  U32 ReservedSectors;                    /* Size of the reserved area */

  U32 FATSize;                            /* Size in sectors of one FAT */
  U32 FATsInUse;                          /* Mask of FATs used
                                           * bit 0: first FAT
                                           * bit 1: second FAT */
  U32 CountOfClusters;                    /* Maximum number of clusters */

  /* Root Directory Access */
  U32 RootDirectoryCluster;               /* Location of Root Directory ,
                                           * used only for FAT32 */
  U32 RootDirectorySize;                  /* Size in sectors of Root Directory
                                           * used only for FAT12/16 */
  FAT_ChainInfo_t RootDirFATChain;         /* For FAT32, the Root Directory is
                                           * a file, we need to access FAT */
  U32 RootDirectoryCurrentOffset;         /* Used for FAT32 to remember the
                                           * sector offset within the cluster
                                           * in case of >1 sectors per cluster
                                           * for FAT12/16, this is the sector
                                           * offset from the beginning of the
                                           * Root Directory */

  /* Offsets to access different parts of the partition */
  U32 FATsSectorOffsets[2];               /* Offset to the first sector of a
                                           * FAT, there can be 2 FATs */
  U32 RootDirSectorOffset;                /* Offset to the first sector of the
                                           * Root directory, used only for
                                           * FAT12/16, located after the
                                           * 2 FATS */
  U32 DataClusterSectorOffset;            /* Offset to the first data cluster
                                           * sector on the partition,
                                           * for FAT12/16 this points after
                                           * the Root Dir, for FAT32 this
                                           * points after the 2 FATs */
} FAT_Info_t;

#define MMCSD_SD_PARTITIONS     (8)

typedef struct MMCSD_DeviceData_t 
{
    U32                 ModuleID;
    MMCSD_Type_e        Type;      /* memory type (MMC/SD) */
    MMCSD_Mode_e        Mode;      /* opmode (BOOT/RAW/FAT) */
    U32                 Copy;
    
    U32                 SpecificationVersion;
    MMCSD_Addressing_e  AddressingMode; 
    U32                 SupportedBusWidth;
    U32                 Size;
    U32                 RCA;
        
    // eSD(2.1) properties
    U32                 PartitionSize[MMCSD_SD_PARTITIONS];
    U8                  PartitionBoot[MMCSD_SD_PARTITIONS];
    U8                  Partition;
    // FAT
    FAT_Buffer_t        FatBuffer;
    FAT_Info_t          FatInfo;
} MMCSD_DeviceData_t;

typedef struct
{
  STATUS (*Init)      ( MEM_DeviceDesc_t* ipDeviceDesc  );
  STATUS (*Read)      ( MEM_DeviceDesc_t* ipDeviceDesc, MEM_ReadDesc_t*   ipReadDesc    );
  STATUS (*Configure) ( MEM_DeviceDesc_t* ipDeviceDesc,void*             ipConfigDesc  );
} SYS_DriverMem_t;

/*  Memory device types */ 
typedef enum DEVICE_Mem_e
{
  DEVICE_TYPE_NULL      = (0x00),
  DEVICE_TYPE_NULL_MEM  = (0x00),
  
//#ifdef SYS_DRIVER_XIP  
  DEVICE_TYPE_XIP,                      
  DEVICE_TYPE_XIP_WAIT,
//#endif

//#ifdef SYS_DRIVER_NAND    
	DEVICE_TYPE_NAND,    
//#endif
	
//#ifdef SYS_DRIVER_ONENAND	
	DEVICE_TYPE_ONENAND,
//#endif
	
//#ifdef SYS_DRIVER_MMCSD1  	  
	DEVICE_TYPE_MMCSD1,
//#endif

//#ifdef SYS_DRIVER_MMCSD2_BOOT
  DEVICE_TYPE_MMCSD2_MMCBOOT,
//#endif

//#ifdef SYS_DRIVER_MMCSD2_JC64	
	DEVICE_TYPE_MMCSD2_JC64,
//#endif

//#ifdef SYS_DRIVER_MMCSD6_BOOT
  DEVICE_TYPE_MMCSD6_MMCBOOT,	
//#endif
  
//#ifdef SYS_DRIVER_MMCSD6_JC64  
	DEVICE_TYPE_MMCSD6_JC64,
//#endif

//#ifdef SYS_DRIVER_EMIF	
	DEVICE_TYPE_EMIF,
//#endif

  DEVICE_TYPE_NOMORE_MEM
	
}DEVICE_Mem_e;
#endif //ROMAPI_DEVICE_MEM

typedef enum HAL_Module_e
{
  HAL_MODULE_NULL = (0x00),
  HAL_MODULE_MMC,
  HAL_MODULE_UART,
  HAL_MODULE_USB,
  HAL_MODULE_I2C,
  HAL_MODULE_DMTIMER1MS,
  HAL_MODULE_WDTIMER,
  HAL_MODULE_ELM,
  HAL_MODULE_GPMC,
  HAL_MODULE_EMIF
} HAL_Module_e; 

typedef enum HAL_UART_e 
{
  HAL_UART1 = (0x00),
  HAL_UART2,
  HAL_UART3,
  HAL_UART4
} HAL_UART_e; 

typedef enum HAL_MMCHS_e 
{
  HAL_MMCHS1 = (0x00),
  HAL_MMCHS2,
  HAL_MMCHS3,
  HAL_MMCHS4,
  HAL_MMCHS5,
  HAL_MMCHS6  
} HAL_MMCHS_e; 

typedef enum HAL_I2C_e 
{
  HAL_I2C1 = (0x00),
  HAL_I2C2,
  HAL_I2C3,
  HAL_I2C4  
} HAL_I2C_e; 

/* DMTIMER1MS modules */
typedef enum HAL_DMTIMER1MS_e
{
  HAL_DMTIMER1MS1 = (0x00),
  HAL_DMTIMER1MS2,
  HAL_DMTIMER1MS10
} HAL_DMTIMER1MS_e; 

/* WDTIMER modules */
typedef enum HAL_WDTIMER_e
{
  HAL_WDTIMER1 = (0x00),
  HAL_WDTIMER2
} HAL_WDTIMER_e; 

extern const SYS_DriverPer_t *romapi_usb_SYS_DriverPer;
#ifdef ROMAPI_DEVICE_MEM
extern const SYS_DriverMem_t *romapi_SYS_DriverMem;
#endif //ROMAPI_DEVICE_MEM

/*==== GLOBALS ==============================================================*/

//00h SEC_ENTRY_Pub2SecDispatcher
//04h SYS_GetDriverMem
#ifdef ROMAPI_DEVICE_MEM
typedef STATUS T_SYS_GetDriverMem( const SYS_DriverMem_t ** oppDriverPer, DEVICE_Mem_e inDeviceType );
//#define SYS_GetDriverMem_2nd(inDeviceType) CALL_ROMAPI_2(STATUS,T_SYS_GetDriverMem,RPAPI(0x04),&romapi_SYS_DriverMem,inDeviceType)
#define SYS_GetDriverMem(oppDriverPer, inDeviceType) CALL_ROMAPI_2(STATUS,T_SYS_GetDriverMem,RPAPI(0x04),oppDriverPer,inDeviceType)
#endif //ROMAPI_DEVICE_MEM

//08h SYS_GetDriverPer
typedef STATUS T_SYS_GetDriverPer ( const SYS_DriverPer_t ** oppDriverPer, DEVICE_Per_e inDeviceType );
#define SYS_GetDriverPer_2nd_usb(inDeviceType) CALL_ROMAPI_2(STATUS,T_SYS_GetDriverPer,RPAPI(0x08),&romapi_usb_SYS_DriverPer,inDeviceType)
#define SYS_GetDriverPer_2nd_uart(inDeviceType) CALL_ROMAPI_2(STATUS,T_SYS_GetDriverPer,RPAPI(0x08),&romapi_uart_SYS_DriverPer,inDeviceType)
#define SYS_GetDriverPer(oppDriverPer, inDeviceType) CALL_ROMAPI_2(STATUS,T_SYS_GetDriverPer,RPAPI(0x08),oppDriverPer,inDeviceType)

//0Ch Reserved
//10h CLK_DetectSysClock
//14h CLK_SetClocks
//18h HAL_CKGEN_GetDefaultClockSettings
//1Ch HAL_CKGEN_IsDpllLocked
//20h HAL_CTRL_GetSysBootPins
//24h HAL_CTRL_GetDevType
//28h HAL_PRM_GetResetType
//2Ch Reserved
//30h HAL_GPMC_Reset
//34h HAL_GPMC_SetConfig
//38h HAL_GPMC_EnableWaitMonitoring
//3Ch HAL_GPMC_Set16Bits
//40h IRQ_Initialize
typedef STATUS T_IRQ_Initialize( void );
#define IRQ_Initialize() CALL_ROMAPI_0(STATUS,T_IRQ_Initialize, RPAPI(0x40))

//44h IRQ_Register
typedef void (*pInterruptHandler_fn_t)( U32 inIRQNumber );
typedef STATUS T_IRQ_Register(pInterruptHandler_fn_t ipFunction, U32 inIRQNumber, U32 inPriority );

//48h IRQ_UnRegister
typedef STATUS T_IRQ_UnRegister( U32 inIRQNumber );
#define IRQ_UnRegister(inIRQNumber)                   CALL_ROMAPI_1(STATUS,T_IRQ_UnRegister,RPAPI(0x48),inIRQNumber)
#define IRQ_HS_USB_MC_NINT          (92)  /* Module HS USB OTG controller     */
#define IRQ_HS_USB_DMA_NINT         (93)  /* Module HS USB OTG DMA controller */
#define IRQ_I2C1_IRQ                (56)

//4Ch IRQ_HandlerC
//50h HAL_WDTIMER_Enable
//54h HAL_WDTIMER_Disable
typedef void T_HAL_WDTIMER_Disable( void );
#define HAL_WDTIMER_Disable()                         CALL_ROMAPI_0(void,T_HAL_WDTIMER_Disable,RPAPI(0x54))

//58h HAL_WDTIMER_Reset
//5Ch Reserved
//60h HAL_I2C_Initialize
typedef STATUS T_HAL_I2C_Initialize(U32 iI2C);
#define HAL_I2C_Initialize(iI2C) CALL_ROMAPI_1(STATUS,T_HAL_I2C_Initialize,RPAPI(0x60),iI2C)
//64h HAL_I2C_Write
typedef STATUS T_HAL_I2C_Write(U32 iI2C, U16 iSlaveAddress, U32 iCount, U8  * ipData, U32 iStartTime, U32 iTimeOut);
#define HAL_I2C_Write(iI2C, iSlaveAddress, iCount, ipData, iStartTime, iTimeOut) CALL_ROMAPI_6(STATUS,T_HAL_I2C_Write,RPAPI(0x64),iI2C,iSlaveAddress,iCount,ipData,iStartTime,iTimeOut)

//68h HAL_I2C_Read
typedef STATUS T_HAL_I2C_Read(U32 iI2C, U16 iSlaveAddress, U32 iCount, U8  * ipData, U32 iStartTime, U32 iTimeOut);
#define HAL_I2C_Read(iI2C, iSlaveAddress, iCount, ipData, iStartTime, iTimeOut) CALL_ROMAPI_6(STATUS,T_HAL_I2C_Read,RPAPI(0x68),iI2C,iSlaveAddress,iCount,ipData,iStartTime,iTimeOut)

//6Ch HAL_I2C_Close
//70h HAL_EMIF4D_SetConfig
//74h Reserved
//78h Reserved
//7Ch Reserved
//80h SYS_GetDeviceDescMem
#ifdef ROMAPI_DEVICE_MEM
typedef STATUS T_SYS_GetDeviceDescMem( MEM_DeviceDesc_t **oppDeviceDesc );
#define SYS_GetDeviceDescMem(oppDeviceDesc)           CALL_ROMAPI_1(STATUS,T_SYS_GetDeviceDescMem,RPAPI(0x80),oppDeviceDesc)
#endif //ROMAPI_DEVICE_MEM

//84h SYS_GetDeviceDescPer
typedef STATUS T_SYS_GetDeviceDescPer( PER_DeviceDesc_t **oppDeviceDesc );
#define SYS_GetDeviceDescPer(oppDeviceDesc)           CALL_ROMAPI_1(STATUS,T_SYS_GetDeviceDescPer,RPAPI(0x84),oppDeviceDesc)

//88h SYS_GetDeviceData
//8Ch Reserved
//90h FAT_ReadFat
//94h FAT_FindFile
//98h FAT_BufferEntries
//9Ch Reserved
//A0h HAL_CM_EnableModuleClocks
typedef STATUS T_HAL_CM_EnableModuleClocks( HAL_Module_e inModule, U32 inInstance );
#define HAL_CM_EnableModuleClocks(inModule, inInstance)    CALL_ROMAPI_2(STATUS,T_HAL_CM_EnableModuleClocks,RPAPI(0xa0),inModule,inInstance)

//A4h HAL_CM_DisableModuleClocks
//A8h HAL_CTRL_ConfigurePads
typedef STATUS T_HAL_CTRL_ConfigurePads( HAL_Module_e inModule, U32 inInstance );
#define HAL_CTRL_ConfigurePads(inModule,inInstance)        CALL_ROMAPI_2(STATUS,T_HAL_CTRL_ConfigurePads,RPAPI(0xA8),inModule,inInstance)

#ifdef SIMULATION
#define CALL_ROMAPI_USB_DRIVER(return,type,kind,a) (simulation_call_romapi(#type,0,a))
#define CALL_ROMAPI_UART_DRIVER(return,type,kind,a) (simulation_call_romapi(#type,0,a))
#else
#define CALL_ROMAPI_USB_DRIVER(return,type,kind,a) (romapi_usb_SYS_DriverPer->kind(a))
#define CALL_ROMAPI_UART_DRIVER(return,type,kind,a) (romapi_uart_SYS_DriverPer->kind(a))
#endif


typedef STATUS T_ROMAPI_UART_Init ( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_UART_Init(a)                         CALL_ROMAPI_UART_DRIVER(STATUS,T_ROMAPI_UART_Init,Init,a)
typedef STATUS T_ROMAPI_UART_Write ( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_UART_Write(a)                         CALL_ROMAPI_UART_DRIVER(STATUS,T_ROMAPI_UART_Write,Write,a)
typedef STATUS T_ROMAPI_UART_Read ( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_UART_Read(a)                         CALL_ROMAPI_UART_DRIVER(STATUS,T_ROMAPI_UART_Read,Read,a)
typedef void T_ROMAPI_UART_Close ( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_UART_Close(a)                         CALL_ROMAPI_UART_DRIVER(void,T_ROMAPI_UART_Close,Close,a)

typedef STATUS T_ROMAPI_USB_Initialize( const struct PeripheralDesc_t *pPeripheralDesc );
#define ROMAPI_USB_Initialize(a)                     CALL_ROMAPI_USB_DRIVER(STATUS,T_ROMAPI_USB_Initialize,Init,a)
typedef STATUS T_ROMAPI_USB_Write( const struct PeripheralDesc_t * ipPeripheralDesc );
#define ROMAPI_USB_Write(a)                         CALL_ROMAPI_USB_DRIVER(STATUS,T_ROMAPI_USB_Write,Write,a)
typedef STATUS T_ROMAPI_USB_Read( const struct PeripheralDesc_t * ipPeripheralDesc );
#define ROMAPI_USB_Read(a)                             CALL_ROMAPI_USB_DRIVER(STATUS,T_ROMAPI_USB_Read,Read,a)
typedef void T_ROMAPI_USB_Close( const struct PeripheralDesc_t * pIoObj );
#define ROMAPI_USB_Close(a)                             CALL_ROMAPI_USB_DRIVER(void,T_ROMAPI_USB_Close,Close,a)

///////////////////////////////////////////////////////
// from "omap4430 rom code memory and pheripheral booting functional specification.pdf"
// also "omap4430_sdc_nda_trm_initialization_v3.0.pdf"

//Chapter 3.2 Table 7 Tracing Data page 22
//Address Size Description
//4030D040h 4 Current tracing vector, word 1
#define ROM_TRACE1 0x4030D040
//4030D044h 4 Current tracing vector, word 2
#define ROM_TRACE2 0x4030D044
//4030D048h 4 Current tracing vector, word 3
#define ROM_TRACE3 0x4030D048
//4030D04Ch 4 Current copy of the PRM_RSTST register (reset reasons)
//4030D050h 4 Cold reset run tracing vector, word 1
//4030D054h 4 Cold reset run tracing vector, word 2
//4030D058h 4 Cold reset run tracing vector, word 3
//4030D05Ch 4 Reserved
//4030D060h 4 Reserved
//4030D064h 4 Reserved

//Chapter 12 Table 43 Tracing vectors page 88
//Trace vector Bit # Group Meaning
//1 0 (0x00000001) General Passed the public reset vector
//1 1 General Entered main function
//1 2 General Running after the cold reset
//1 3 Boot Main booting routine entered
//1 4 Memory Boot Memory booting started
//1 5 Peripheral Boot Peripheral booting started
//1 6 Boot Booting loop reached last device
//1 7 Boot GP header or HS TOC found
//1 8 (0x00000100) Boot Booting Message “Skip Peripheral Booting” received
//1 9 Boot Booting Message “Change Device” received
//1 10 Peripheral Boot Booting Message “Peripheral booting” received
//1 11 Peripheral Boot Booting Message “Get Asic Id”
//1 12 Peripheral Boot Device initialized
//1 13 Peripheral Boot Asic Id sent
//1 14 Peripheral Boot Image received
//1 15 Peripheral Boot Peripheral booting failed
//1 16 (0x00010000) Peripheral Boot Booting Message not received (timeout)
//1 17 Peripheral Boot Image size not received (timeout)
//1 18 Peripheral Boot Image not received (timeout)
//1 19 Reserved Reserved
//1 20 Configuration Header CHSETTINGS found
//1 21 Configuration Header CHSETTINGS executed
//1 22 Configuration Header CHRAM executed
//1 23 Configuration Header CHFLASH executed
//1 24 (0x01000000) Configuration Header CHMMCSD clocks executed
//1 25 Configuration Header CHMMCSD bus width executed
//1 26 Reserved Reserved
//1 27 Reserved Reserved
//1 28 SW Booting configuration SWCFG general detected
//1 29 SW Booting configuration SWCFG clocks detected
//1 30 SW Booting configuration SWCFG timeout detected
//1 31 Reserved Reserved
//2 0 (0x00000001) Companion chip Phoenix detected
//2 1 Companion chip VBUS detected
//2 2 Companion chip VMMC switched on
//2 3 Companion chip VUSB switched on
//2 4 USB USB connect
//2 5 USB USB configured state
#define TRACE2_USB_CONFIGURE (1 << 5) //0x00000020
//2 6 USB USB VBUS valid
//2 7 USB USB session valid
//2 8 (0x00000100) Reserved Reserved
//2 9 Reserved Reserved
//2 10 Reserved Reserved
//2 11 Reserved Reserved
//2 12 Memory Boot Memory booting trial 0
//2 13 Memory Boot Memory booting trial 1
//2 14 Memory Boot Memory booting trial 2
//2 15 Memory Boot Memory booting trial 3
//2 16 (0x00010000) Memory Boot Execute GP image
//2 17 Peripheral Boot Start authentication of peripheral boot image
//2 18 Memory & Peripheral Boot Jumping to Initial SW
//2 19 Memory & Peripheral Boot High Secure image not executed
//2 20 Memory & Peripheral Boot Start image authentication
//2 21 Memory & Peripheral Boot Image authentication failed
//2 22 Memory & Peripheral Boot Analyzing SpeedUp
//2 23 Memory & Peripheral Boot Speedup failed
//2 24 (0x01000000) Memory & Peripheral Boot PPA authentication success
//2 25 Memory & Peripheral Boot PPA authentication failed
//2 26 Memory & Peripheral Boot R&D authentication success
//2 27 Memory & Peripheral Boot R&D authentication failed
//2 28 Memory & Peripheral Boot Authentication procedure failed
//2 29 Reserved Reserved
//2 30 Reserved Reserved
//2 31 Reserved Reserved
//3 0 (0x00000001) Memory Boot Memory booting device NULL
//3 1 Memory Boot Memory booting device XIP
//3 2 Memory Boot Memory booting device XIPWAIT
//3 3 Memory Boot Memory booting device NAND
//3 4 Memory Boot Memory booting device OneNAND
//3 5 Memory Boot Memory booting device MMCSD1
//3 6 Reserved Reserved
//3 7 Memory Boot Memory booting device MMCSD2
//3 8 (0x00000100) Reserved Reserved
//3 9 Reserved Reserved
//3 10 Memory Boot Memory booting device EMIF LPDDR2-NVM
//3 11 Reserved Reserved
//3 12 Reserved Reserved
//3 13 Reserved Reserved
//3 14 Reserved Reserved
//3 15 Reserved Reserved
//3 16 (0x00010000) Reserved Reserved
//3 17 Reserved Reserved
//3 18 Peripheral Boot Peripheral booting device UART3
#define TRACE3_UART3_BOOT (1 << 18) //0x00040000
//3 19 Reserved Reserved
//3 20 Peripheral Boot Peripheral booting device USB
#define TRACE3_USB_BOOT (1 << 20) //0x00200000
//3 21 Peripheral Boot Peripheral booting device USB ULPI
//3 22 Peripheral Boot Peripheral booting device NULL
//3 23 Reserved Reserved
//3 24 Reserved Reserved
//3 25 Reserved Reserved
//3 26 Reserved Reserved
//3 27 Reserved Reserved
//3 28 Reserved Reserved
//3 29 Reserved Reserved
//3 30 Reserved Reserved
//3 31 Reserved Reserved

/////////////////////////////////
//From: "OMAP4430_SDC_NDA_TRM_Control_Module_v3.1.pdf"

#define CONTROL_STATUS 0x4a0022c4

/////////////////////////////////
//From: "OMAP4430_SDC_NDA_TRM_Initialization_v3.0.pdf"
//Table 1-4. Memory Preffered Booting
//sys_boot[5:0] 1st 2nd 3rd 4th
//0b100000 JC64-MMC2 USB(1)
//0b100001 XIP USB
//0b100010 XIPWAIT USB
//0b100011 NAND USB
//0b100100 USB
//0b100101 MMC1 USB
//0b100110 OneNAND USB
//0b100111 OneNAND JC64-MMC2 USB
//0b101000 JC64-MMC2 UART
//0b101001 XIP UART
//0b101010 XIPWAIT UART
//0b101011 NAND UART
//0b101100 UART
//0b101101 MMC1 UART
//0b101110 OneNAND UART
//0b101111 JC64-MMC2 USB-ULPI(2)
//0b110000 XIP USB-ULPI
//0b110001 XIPWAIT USB-ULPI
//0b110010 NAND USB-ULPI
//0b110011 USB-ULPI
//0b110100 MMC1 USB-ULPI
//0b110101 OneNAND USB-ULPI
//0b110110 JC64-MMC2 USB UART MMC1
//0b110111 XIP USB UART MMC1
//0b111000 XIPWAIT USB UART MMC1
//0b111001 NAND USB UART MMC1
//0b111010 USB UART MMC1
//0b111011 MMC1 USB UART
//0b111100 OneNAND USB UART MMC1
//0b111101 Reserved
//0b111110 Reserved
//0b111111(Only GP) Fast XIP booting. Wait monitoring off USB UART
//Table 1-5. Peripheral Preferred Booting
//sys_boot[5:0] 1st 2nd 3rd 4th
//0b000000 USB JC64-MMC2
//0b000001 USB XIP
//0b000010 USB XIPWAIT
//0b000011 USB NAND
//0b000100 USB
//0b000101 USB MMC1
//0b000110 USB OneNAND
//0b000111 USB OneNAND JC64-MMC2
//0b001000 UART JC64-MMC2
//0b001001 UART XIP
//0b001010 UART XIPWAIT
//0b001011 UART NAND
//0b001100 UART
//0b001101 UART MMC1
//0b001110 UART OneNAND
//0b001111 USB-ULPI JC64-MMC2
//0b010000 USB-ULPI XIP
//0b010001 USB-ULPI XIPWAIT
//0b010010 USB-ULPI NAND
//0b010011 USB-ULPI
//0b010100 USB-ULPI MMC1
//0b010101 USB-ULPI OneNAND
//0b010110 USB UART MMC1 JC64-MMC2
//0b010111 USB UART MMC1 XIP
//0b011000 USB UART MMC1 XIPWAIT
//0b011001 USB UART MMC1 NAND
//0b011010 USB UART MMC1
//0b011011 USB UART MMC1
//0b011100 USB UART MMC1 OneNAND
//0b011101 Reserved
//0b011110 Reserved
//0b011111(Only GP) Fast XIP booting. Wait monitoring ON USB UART

#endif //ROMAPI_4430_H
