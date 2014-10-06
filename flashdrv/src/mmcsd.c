/*==============================================================================
*
*            TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION
*
*   Property of Texas Instruments
*   For Unrestricted Internal Use Only
*   Unauthorized reproduction and/or distribution is strictly prohibited.
*   This product is protected under copyright law and trade secret law as an
*   unpublished work.
*   Created 2009, (C) Copyright 2009 Texas Instruments.  All rights reserved.
*
*   Filename   : mmcsd.c
*
*   Description: MMCSD flashing driver for ATTILA platform.
*_______________________________________________________________________________
*
*   History:
*   10-Aug-09    Martin Zarzycki
*                Creation
*
*=============================================================================*/

//#include "romcode_types.h"
#include <stdarg.h>
#include "types.h"

#define ROMAPI_DEVICE_MEM
typedef U8 BOOL;

#include "romapi.h"
#include "flash_drv.h"
#include "emmc_drv.h"
//
//#include "console.h" 
//#include "drivers.h"
//#include "message.h"
//
//#if (CHIPTYPE==4430)  
//  #include "memory_device.h"
//  #include "mmcsd.h"
//  #include "system.h"
//  #include "hal.h"
//  #include "hal_mmchs.h"
//  
//  /* Enable functions in public api interface. */
//  #define PUBLIC_API_SYS_GETDRIVERMEM
//  #define PUBLIC_API_SYS_GETDEVICEDESCMEM
//  #define PUBLIC_API_SYS_GETDEVICEDATA
//  
//  #include "public_api_func.h"
#include "\cdb\OMAPSW-ROM-ATTILA\src\romcode\shared\include\cred\mmchs.h"
#include "\cdb\OMAPSW-ROM-ATTILA\src\romcode\shared\include\cram_types.h"
#include "\cdb\OMAPSW-ROM-ATTILA\src\romcode\public\include\hal\hal_mmchs.h"
//#endif

#define NOVERB 1
#define VERB 2
#define MSG_RESULT_FAIL 11
#define DRV_DETECT_MEMORY_RESP_NOMEM 12
#define DRV_DETECT_MEMORY_RESP_MEMOK 0
#define DRV_DISPLAY_INFO_RESP_MEMOK 0

/*------------------------ local variables -----------------------------------*/

static SYS_DriverMem_t*     LV_pDriver;
static MEM_DeviceDesc_t*    LV_pDeviceDesc;
static MMCSD_DeviceData_t*  LV_pDeviceData;

static U32 LV_Interface;
static U32 LV_Partition;

/*------------------------ external functions --------------------------------*/

///*------------------------------------------------------------------------------
// * @DEFINITION RD_MEM_32_VOLATILE
// *
// * @BRIEF      Read memory volatile - 32bit; accessing the memory address
// *             This is ok for ARM cores and C64, not ok for C55
// *----------------------------------------------------------------------------*/
//#ifndef RD_MEM_32_VOLATILE
//  #define RD_MEM_32_VOLATILE(addr)     ( (U32)( *((VU32 *)(addr)) ) )
//#endif
//
///*------------------------------------------------------------------------------
// * @DEFINITION RD_FIELD_32  
// *
// * @BRIEF      Read bit-field; takes component instance name, register offset
// *             and bit-field as parameters
// *----------------------------------------------------------------------------*/
//#define RD_FIELD_32(COMP, REG, FIELD) \
//  ( ( RD_REG_32(COMP, REG) & FIELD ) >> FIELD##__POS )
//
///*------------------------------------------------------------------------------
// * @DEFINITION REG_ADDR_32
// *
// * @BRIEF      Register address; takes component instance name and register 
// *             offset as parameters
// *----------------------------------------------------------------------------*/
//#define REG_ADDR_32(COMP, REG) (COMP+(VU32)(REG))
//
///*------------------------------------------------------------------------------
// * @DEFINITION RD_REG_32 
// *
// * @BRIEF      Read register; takes component instance name and register offset
// *             as parameters
// *----------------------------------------------------------------------------*/
//#define RD_REG_32(COMP, REG) \
//  RD_MEM_32_VOLATILE(REG_ADDR_32(COMP, REG))
//
///*------------------------------------------------------------------------------
// * @DEFINITION WR_MEM_32_VOLATILE
// *
// * @BRIEF      Write memory volatile - 32bit; accessing the memory address
// *             This is ok for ARM cores and C64, not ok for C55
// *----------------------------------------------------------------------------*/
//#ifndef WR_MEM_32_VOLATILE
//  #define WR_MEM_32_VOLATILE(addr, data) ( *((VU32 *)(addr)) = (U32)(data) )
//#endif
//
///*------------------------------------------------------------------------------
// * @DEFINITION WR_REG_32 
// *
// * @BRIEF      Write value to register; takes component instance name, 
// *             register offset and value to write as parameters
// *----------------------------------------------------------------------------*/
//#define WR_REG_32(COMP, REG, VAL) \
//  WR_MEM_32_VOLATILE(REG_ADDR_32(COMP, REG), (U32)(VAL))

/*-------------------------------------------------------------------------*//**
 * @DEFINITION   SYNCTIMER32K__CR
 *
 * @BRIEF        This register holds the counter value  
 *                 
 *
 *//*------------------------------------------------------------------------ */
#define SYNCTIMER32K__CR                                   0x10
#define SYNCTIMER32K_CR_ADDR(BASE) \
    (BASE + SYNCTIMER32K__CR)
#define SYNCTIMER32K_CR(BASE) \
    (VREG32(BASE + SYNCTIMER32K__CR))

/*-------------------------------------------------------------------------*//**
 * @DEFINITION   MMCHS__MMCHS_PSTATE
 *
 * @BRIEF        Present state register 
 *               The Host can get status of the Host Controller from this 
 *               32-bit read only register. 
 *
 *//*------------------------------------------------------------------------ */
#define MMCHS__MMCHS_PSTATE                                0x224
#define MMCHS_MMCHS_PSTATE_ADDR(BASE) \
    (BASE + MMCHS__MMCHS_PSTATE)
#define MMCHS_MMCHS_PSTATE(BASE) \
    (VREG32(BASE + MMCHS__MMCHS_PSTATE))

/*-------------------------------------------------------------------------*//**
 * @DEFINITION   MMCHS__MMCHS_STAT
 *
 * @BRIEF        Interrupt status register 
 *               The interrupt status regroups all the status of the module 
 *               internal events that can generate an interrupt. 
 *               MMCHS_STAT[31:16] = Error Interrupt Status 
 *               MMCHS_STAT[15:0] = Normal Interrupt Status 
 *
 *//*------------------------------------------------------------------------ */
#define MMCHS__MMCHS_STAT                                  0x230
#define MMCHS_MMCHS_STAT_ADDR(BASE) \
    (BASE + MMCHS__MMCHS_STAT)
#define MMCHS_MMCHS_STAT(BASE) \
    (VREG32(BASE + MMCHS__MMCHS_STAT))

/*-------------------------------------------------------------------------*//**
 * @DEFINITION   MMCHS__MMCHS_PSTATE__DATI   
 *
 * @BRIEF        Command inhibit(DAT) 
 *               This status bit is generated if either DAT line is active 
 *               (MMCHS_PSTATE[DLA]) or Read transfer is active 
 *               (MMCHS_PSTATE[RTA]) or when a command with busy is issued. 
 *               This bit prevents the local host to issue a command. 
 *               A change of this bit from 1 to 0 generates a transfer 
 *               complete interrupt (MMCHS_STAT[TC]). - (RO) 
 *
 *//*------------------------------------------------------------------------ */
#define MMCHS__MMCHS_PSTATE__DATI                      0x2
#define MMCHS__MMCHS_PSTATE__DATI__POS                1

/*-------------------------------------------------------------------------*//**
 * @DEFINITION   MMCHS__MMCHS_PSTATE__DATI__CMDDIS
 *
 * @BRIEF        Issuing of command using DAT lines is not allowed - (Read) 
 *
 *//*------------------------------------------------------------------------ */
#define MMCHS__MMCHS_PSTATE__DATI__CMDDIS             0x1
#define MMCHS_MMCHS_PSTATE_DATI_CMDDIS \
   SET_FIELD(MMCHS__MMCHS_PSTATE__DATI__CMDDIS, \
             MMCHS__MMCHS_PSTATE__DATI)


//#define GET_BASE_ADDRESS(group, phys)  (##phys##_U_BASE)
#define GRP_L4_PER          VIRT_ADDR__HS_L4_PER
#define GRP_L4_WKUP         VIRT_ADDR__HS_L4_WKUP
#define GET_BASE_ADDRESS(group, phys)   ((##phys##_U_BASE & 0x000FFFFF) + group)

#define VIRT_ADDR__HS_L4_WKUP                                       0xAF400000
#define VIRT_ADDR__HS_L4_PER                                        0xAEF00000

#define SYNCTIMER32K_U_BASE                                 0x4A304000
#define DMTIMER_DMC1MS1_U_BASE                              0x4A318000
#define DMTIMER_DMC1MS2_U_BASE                              0x48032000
#define DMTIMER_DMC1MS10_U_BASE                             0x48086000

#define SYNCTIMER32K                                GET_BASE_ADDRESS(GRP_L4_WKUP, SYNCTIMER32K)

/* CR Register */ 
#define CR \
  ( RD_REG_32(  SYNCTIMER32K,       \
                SYNCTIMER32K__CR ) )

/* Macro to retrieve current time ( in ticks ) */
#define TIME_NOW  ( CR )

/* Macro to check if timed-out ( in ticks ) */
#define TIMED_OUT( inStart, inTimeout ) \
  ( TIME_NOW - (inStart) >= ( inTimeout ) )

/* Macro to convert time in milliseconds into number of cycles
 * The Timers are all clocked with 32.768kHz 
 * This macro limits the maximum possible timeout to 131073 ms 
 * = 2 minutes and 11 seconds... 
 * Warning: Be careful to use a constant as parameter otherwise the linker will
 * include the ARM divide function, which is forbidden in ROM Code */
#define TIME_MSEC2TICKS( inMs )  ( (inMs) * 32768 / 1000 )

/* General 100 ms timeout to prevent deadloops in stall conditions. */
#define LM_TIMEOUT_100MS    ( TIME_MSEC2TICKS( 100 ) )

#define DMTIMER_DMC1MS1                             GET_BASE_ADDRESS(GRP_L4_WKUP, DMTIMER_DMC1MS1)
#define DMTIMER_DMC1MS2                             GET_BASE_ADDRESS(GRP_L4_PER, DMTIMER_DMC1MS2)
#define DMTIMER_DMC1MS10                            GET_BASE_ADDRESS(GRP_L4_PER, DMTIMER_DMC1MS10)

// Collect the base addresses of all DMTIMER1MS modules.
static const U32 LC_pBase[3] =
{
  DMTIMER_DMC1MS1,
  DMTIMER_DMC1MS2,
  DMTIMER_DMC1MS10,
};

#define CHECK_COND( cond, error ) \
            if (cond) \
            { \
                return FAILED; \
            }

#define LM_MMCHS_STAT_ALL   ( MMCHS__MMCHS_STAT__BADA |   \
                              MMCHS__MMCHS_STAT__CERR |   \
                              MMCHS__MMCHS_STAT__ADMAE |  \
                              MMCHS__MMCHS_STAT__ACE |    \
                              MMCHS__MMCHS_STAT__DEB |    \
                              MMCHS__MMCHS_STAT__DCRC |   \
                              MMCHS__MMCHS_STAT__DTO |    \
                              MMCHS__MMCHS_STAT__CIE |    \
                              MMCHS__MMCHS_STAT__CEB |    \
                              MMCHS__MMCHS_STAT__CCRC |   \
                              MMCHS__MMCHS_STAT__CTO |    \
                              MMCHS__MMCHS_STAT__ERRI |   \
                              MMCHS__MMCHS_STAT__BSR |    \
                              MMCHS__MMCHS_STAT__OBI |    \
                              MMCHS__MMCHS_STAT__CIRQ |   \
                              MMCHS__MMCHS_STAT__CREM |   \
                              MMCHS__MMCHS_STAT__CINS |   \
                              MMCHS__MMCHS_STAT__BRR |    \
                              MMCHS__MMCHS_STAT__BWR |    \
                              MMCHS__MMCHS_STAT__DMA |    \
                              MMCHS__MMCHS_STAT__BGE |    \
                              MMCHS__MMCHS_STAT__TC |     \
                              MMCHS__MMCHS_STAT__CC ) 

/*_____________________________________________________________________________
* FUNCTION:     HAL_MMCHS_SendCommand
*
* DESCRIPTION:  Send a command to the card.
*
* PARAMETERS:   inModuleID - MMCHS instance.
*               inCmd - Encoded value containing the command to send.
*               inArg - Argument used for the command.
*               opResponses - Buffer to be filled with response data.
*
* RETURNS:      NO_ERROR if successful.
*               FAILED (ERROR_MMC_SENDING_CMD) if problems sending command.
*____________________________________________________________________________*/
STATUS HAL_MMCHS_SendCommand( U32 inModuleID,
                              U32 inCmd,
                              U32 inArg,
                              U32 *opResponses )
{
  VU32  lv_MmcStat;
  U32   lv_CR;
  
  /* Get current time. */
  lv_CR = TIME_NOW;
    
  /* Wait for the BUSY state on DAT line to end. */
  while( RD_FIELD_32( LC_pBase[inModuleID],
                      MMCHS__MMCHS_PSTATE,
                      MMCHS__MMCHS_PSTATE__DATI )
         == MMCHS__MMCHS_PSTATE__DATI__CMDDIS )
  {
    /* Check for timeout condition (100 ms). */  
    CHECK_COND( TIMED_OUT( lv_CR, LM_TIMEOUT_100MS ), ERROR_MMC_SENDING_CMD );
  }
                       
  /* Number of Blocks: 0, Length: 512 bytes
   * Since this setting never changes, it is set in HAL_MMCHS_Initialize().
   * It is possible to change it anyway, using a wrapper for application
   * specific commands, like CE-ATA. */
            
  /* Clear all status bits */
  WR_REG_32(  LC_pBase[inModuleID],
              MMCHS__MMCHS_STAT,
              LM_MMCHS_STAT_ALL );

  /* Write the argument for the CMD */
  WR_REG_32(  LC_pBase[inModuleID],
              MMCHS__MMCHS_ARG,
              inArg );

  /* Send the Command */
  WR_REG_32(  LC_pBase[inModuleID],
              MMCHS__MMCHS_CMD,
              ( inCmd |
                MMCHS_MMCHS_CMD_MSBS_SGLEBLK    |
                MMCHS_MMCHS_CMD_ACEN_DISABLE    |
                MMCHS_MMCHS_CMD_BCE_DISABLE     |
                MMCHS_MMCHS_CMD_DE_DISABLE ) );

  /* Check all the flags */
  /* WHILE (Not Failed & Command not completed) */
  while ( 1 )
  {
    /* Wait for a flag */
    do {
      lv_MmcStat = RD_REG_32( LC_pBase[inModuleID], MMCHS__MMCHS_STAT );
    } while ( lv_MmcStat == 0 );

    /* Exit if error bit in the status register has been set. */
    CHECK_COND( (lv_MmcStat & MMCHS__MMCHS_STAT__ERRI),
                ERROR_MMC_SENDING_CMD );

    /* IF (Command Complete ?) */
    if ( lv_MmcStat & MMCHS__MMCHS_STAT__CC )
    {
      /* Clear the bit */
      WR_REG_32( LC_pBase[inModuleID],
                 MMCHS__MMCHS_STAT,
                 MMCHS_MMCHS_STAT_CC_ST_RST_W );
      
      /* Copy the Response */
      opResponses[0] = RD_REG_32( LC_pBase[inModuleID], MMCHS__MMCHS_RSP10 );
      
      /* IF( expected response length is 136 bit ) 
       * Note: The define should be LGHT136 instead of LGHT36. */
      if( ( inCmd & MMCHS__MMCHS_CMD__RSP_TYPE )
          == MMCHS_MMCHS_CMD_RSP_TYPE_LGHT36 )
      {
        opResponses[1] = RD_REG_32( LC_pBase[inModuleID], MMCHS__MMCHS_RSP32 );
        opResponses[2] = RD_REG_32( LC_pBase[inModuleID], MMCHS__MMCHS_RSP54 );
        opResponses[3] = RD_REG_32( LC_pBase[inModuleID], MMCHS__MMCHS_RSP76 );
      }

      break;
    } /* ENDIF (Command Complete ?) */

  } /* ENDWHILE (Not Failed & Command not completed) */

  return NO_ERROR;
} /* HAL_MMCHS_SendCommand */

/*_____________________________________________________________________________
* FUNCTION:    HAL_MMCHS_WriteData
*
* DESCRIPTION: Transefer data to MMC/SD card.
*
* PARAMETERS:  inModuleID - MMCHS module instance.
*              ipBuffer - Buffer with data to be send.
*
* RETURNS:     NO_ERROR if successful.
*              FAILED (ERROR_MMC_WRITING_DATA) if writing data failed.
*____________________________________________________________________________*/
STATUS HAL_MMCHS_WriteData( U32 inModuleID, U32 *ipBuffer )
{
  VU32 lv_MmcStat;
  U32  lv_BytesWritten = 0;
  U32  lv_BlockLength;

  /* Get block length */
  lv_BlockLength = RD_FIELD_32( LC_pBase[inModuleID],
                                MMCHS__MMCHS_BLK,
                                MMCHS__MMCHS_BLK__BLEN );
  
  /* WHILE( Transfer not completed ) */
  while(1)
  {
    /* Wait for a flag */
    do {
      lv_MmcStat = RD_REG_32( LC_pBase[inModuleID], MMCHS__MMCHS_STAT );
    } while ( lv_MmcStat == 0 );
   
    /* Exit if Error */
    CHECK_COND( ( lv_MmcStat & MMCHS__MMCHS_STAT__ERRI ),
                ERROR_MMC_WRITING_DATA );          

    /* IF (Buffer Write Ready ?) */
    if ( lv_MmcStat & MMCHS__MMCHS_STAT__BWR )
    {
      U32 lv_Count;
      
      /* Clear the bit */
      WR_REG_32( LC_pBase[inModuleID],
                 MMCHS__MMCHS_STAT,
                 MMCHS_MMCHS_STAT_BWR_ST_RST_W );
      
      /* Write the Data */
      /* BLEN was set to 512 bytes - Buffer is 1024 bytes */
      for ( lv_Count=0; lv_Count < ( lv_BlockLength >> 2 ); lv_Count++ )
      {
        WR_REG_32( LC_pBase[inModuleID],
                   MMCHS__MMCHS_DATA,
                   *ipBuffer );
                   
        ipBuffer ++;
        lv_BytesWritten += 4;
      }
      
    } /* ENDIF (Buffer Write Ready ?) */

    /* IF (Transfer Completed ?) */
    if ( lv_MmcStat & MMCHS__MMCHS_STAT__TC )
    {
      /* Clear the bit */
      WR_REG_32( LC_pBase[inModuleID],
                 MMCHS__MMCHS_STAT,
                 MMCHS_MMCHS_STAT_TC_ST_RST_W );
                 
      break;
    } /* ENDIF (Transfer Completed ?) */

  } /* ENDWHILE( Transfer not completed ) */

  return NO_ERROR;
} /* HAL_MMCHS_WriteData */

void uprintf(int dummy, const char* format, ...) 
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    //dbg_vxprintf(VXPRINTF_STANDARD, format, arg_ptr);
    dbg_vxprintf(0, format, arg_ptr);
    va_end(arg_ptr);
}

/*------------------------ local functions -----------------------------------*/           

U32     FLASHDRV_Verify( U8* ipBuff, U32 inAddr, U32 inSize );
U32     FLASHDRV_DetectMemory( void );

STATUS  flashdrv_DeviceInfo( void );
U32     flashdrv_DetectMemory( void );

/*------------------------------------------------------------------------------
* FUNCTION   : FLASHDRV_Commands
*
* DESCRIPTION: Driver specific commands handler
*
* PARAMETERS :
*
* RETURNS    :
*-----------------------------------------------------------------------------*/
U32 FLASHDRV_Commands( U32 inCmd, U32 inArg1, U32 inArg2, U32 in_Arg3 )
{
  switch ( inCmd )
  {
    //--------------------------------------------
    // Set interface.
    //--------------------------------------------
    case 0x10:
      if( inArg1 == 1 )
        LV_Interface = 2;
      else
        LV_Interface = 1;
        
      uprintf( NOVERB,
        " CMD 0x10 (SetInterface): MMCHS (%d)\n\r", LV_Interface );
      break;
    
    //--------------------------------------------
    // Select partition.
    //--------------------------------------------    
    case 0x11:
      LV_Partition = inArg1;
      uprintf( NOVERB,
        " CMD 0x11 (SelectedPartition): %d\n\r", LV_Partition );
      break;
      
    //--------------------------------------------
    // deafult
    //--------------------------------------------      
    default:
      uprintf( NOVERB,
        " ERROR! CMD 0x%X is not valid for MMCSD driver.\n\r", inCmd );
      break;

    }

   return(0);
}

///*------------------------------------------------------------------------------
//* FUNCTION   : FLASHDRV_Header
//*
//* DESCRIPTION: Returns driver header
//*
//* PARAMETERS :
//*
//* RETURNS    :
//*-----------------------------------------------------------------------------*/
//char* FLASHDRV_Header( void )
//{
//   return("MMC/SD driver");
//}

///*------------------------------------------------------------------------------
//* FUNCTION   : FLASHDRV_SetRawMode
//*
//* DESCRIPTION: Sets (enable/disable) raw mode
//*
//* PARAMETERS : inMode - 0 disable, >0 enable raw mode
//*
//* RETURNS    :
//*-----------------------------------------------------------------------------*/
//U32 FLASHDRV_SetRawMode( U32 inMode )
//{
//  return( DRV_RESP_FAIL );
//}

/*------------------------------------------------------------------------------
* FUNCTION   : FLASHDRV_Erase
*
* DESCRIPTION: Erases space needed for the image
*
* PARAMETERS : inFrom - start address to erase
*              inSize - size to programm
*
* RETURNS    : 0 - ok
*            : block address which failed
*-----------------------------------------------------------------------------*/
U32 FLASHDRV_Erase( U32 inFrom , U32 inSize )
{
  return 0;
}

/*------------------------------------------------------------------------------
* FUNCTION   : FLASHDRV_Program
*
* DESCRIPTION: Programs MMC/SD memory
*
* PARAMETERS : ipBuff - Data pointer
*              inAddr - Address to programm
*              inSize - Size to programm
*
* RETURNS    : Number of programmed bytes.
*-----------------------------------------------------------------------------*/
U32 FLASHDRV_Program( U32* ipBuff, U32 inAddr, U32 inSize )
{
  STATUS lv_Ret;
  
  U32 lv_Argument;
  U32 lv_Responses[4];

  if( ipBuff == NULL )
    return( MSG_RESULT_FAIL );

  if( LV_pDeviceData->AddressingMode == MMCSD_ADDRESSING_SECTOR )
  {
    /* Sector addressing -> The address given to CMD24 is the sector itself. */
     lv_Argument = inAddr >> 9;
  }
  else
  {
    /* Byte addressing -> The address given to CMD24 is in bytes. */
    lv_Argument = inAddr;
  }

  /* Command: write single block */  
  lv_Ret = HAL_MMCHS_SendCommand( LV_pDeviceData->ModuleID,
                                  MMCSD_CMD24,
                                  lv_Argument,
                                  lv_Responses );
  if( lv_Ret != NO_ERROR )
  {
    uprintf( VERB,
      " ERROR! Failed sending CMD to MMC/SD memory.\n\r" );
	  return( MSG_RESULT_FAIL );    
  }                          
  
  /* Write the data. */
  lv_Ret = HAL_MMCHS_WriteData( LV_pDeviceData->ModuleID,
                                (U32*)ipBuff );
  if( lv_Ret != NO_ERROR )
  {
    uprintf( VERB,
      " ERROR! Failed writing data to MMC/SD memory.\n\r" );
	  return( MSG_RESULT_FAIL );    
  }
  else
  {
    /* Verify data. */
    U32 lv_Size;
  
    lv_Size = FLASHDRV_Verify( (U8*)ipBuff, inAddr, inSize );
    if ( lv_Size != inSize) 
	    return( MSG_RESULT_FAIL );
  }   
   
  return( inSize );

}
//ENDFUNCTION FLASHDRV_Program

/*------------------------------------------------------------------------------
* FUNCTION    : FLASHDRV_DetectMemory
*
* DESCRIPTION : initializes MMCSD interface and card
*               detects card type and capabilities 
*
* PARAMETERS  : none
*
* RETURNS     : DRV_DETECT_MEMORY_RESP_MEMOK if succeded
*               DRV_DETECT_MEMORY_RESP_NOMEM if failed
*-----------------------------------------------------------------------------*/
U8 LV_DeviceData[ sizeof( MMCSD_DeviceData_t ) ];

U32 FLASHDRV_DetectMemory( void )
{
  STATUS lv_Ret;
  
  U16 lv_BootOptions = 0;
  
  /* Get memory device descriptor. */
  //PUBLIC_SYS_GetDeviceDescMem( &LV_pDeviceDesc );
  SYS_GetDeviceDescMem( &LV_pDeviceDesc );
  
  /* Get device data. */
  //PUBLIC_SYS_GetDeviceData( (U8**)&LV_pDeviceData );
  //SYS_GetDeviceData( (U8**)&LV_pDeviceData );
  LV_pDeviceData = LV_DeviceData;
  
  /* Check active MMC/SD interface. */
  switch( LV_Interface )
  {
    case 1:
      LV_pDeviceDesc->DeviceType = DEVICE_TYPE_MMCSD1;
      break;
      
    case 2:
      LV_pDeviceDesc->DeviceType = DEVICE_TYPE_MMCSD2_JC64;
      break;
          
    default:
      uprintf(NOVERB,
        " ERROR! Invalid MMC/SD interface: %d!\n\r", LV_Interface );
      return DRV_DETECT_MEMORY_RESP_NOMEM;
  }
  
  /* Get device driver. */
  //PUBLIC_SYS_GetDriverMem( &LV_pDriver,
  SYS_GetDriverMem( &LV_pDriver,
    ( DEVICE_Mem_e )LV_pDeviceDesc->DeviceType );
  
  //------------------------------------------------------- 
  // Call MMC/SD initialization.
  //-------------------------------------------------------
  LV_pDeviceDesc->Initialized  = FALSE;
  LV_pDeviceDesc->pDeviceData  = (U8*)LV_pDeviceData;
  LV_pDeviceDesc->pBootOptions = &lv_BootOptions;
  
  lv_Ret = LV_pDriver->Init( LV_pDeviceDesc );
  if ( lv_Ret != NO_ERROR )
  {
    uprintf( NOVERB,
      " ERROR! MMCHS (%d) could not detect card!\n\r", LV_Interface );
    return(DRV_DETECT_MEMORY_RESP_NOMEM);
  }

  uprintf( NOVERB,
    " MMCHS (%d) and card initialized\n\r", LV_Interface );
  
  uprintf(NOVERB,"\n\r");    
  
  return(DRV_DETECT_MEMORY_RESP_MEMOK);
}

/*------------------------------------------------------------------------------
* FUNCTION   : FLASHDRV_DeviceInfo
*
* DESCRIPTION: Displays device Info
*
* PARAMETERS :
*
* RETURNS    :
*-----------------------------------------------------------------------------*/
U32 FLASHDRV_DeviceInfo(void)
{
  U32 lv_Ret;
  
  lv_Ret = FLASHDRV_DetectMemory();
  
  if( lv_Ret != DRV_DISPLAY_INFO_RESP_MEMOK )
    return( lv_Ret );

  uprintf(NOVERB,
    " Device Info:\n\r");
  
  flashdrv_DeviceInfo();
  
  return DRV_DISPLAY_INFO_RESP_MEMOK;
}

/*------------------------------------------------------------------------------
* FUNCTION    : FLASHDRV_Verify
*
* DESCRIPTION : Verifies up to 512 bytes
*
* PARAMETERS  : ipBuff - data pointer
*               inAddr - address to verify
*               inSize - size to verify
*
* RETURNS     : number of verified bytes (up to last successful byte)
*-----------------------------------------------------------------------------*/
U32 FLASHDRV_Verify( U8* ipBuff, U32 inAddr, U32 inSize )
{
  STATUS lv_Ret;
  
  U32 lv_Offset;
  U8  lv_pBuffer[512];
  
  MEM_ReadDesc_t lv_ReadDesc;

  if( ipBuff == NULL)
    return(0);
  
  lv_ReadDesc.DestAddr    = (U32)lv_pBuffer;
  lv_ReadDesc.StartSector = inAddr >> 9;
  lv_ReadDesc.SectorCount = inSize >> 9;
  
  /* Correct sector count, if verifying data size < 0x200 */
  if( lv_ReadDesc.SectorCount == 0 )
    lv_ReadDesc.SectorCount = 1;
  
  lv_Ret = LV_pDriver->Read( LV_pDeviceDesc, &lv_ReadDesc );
  if( lv_Ret != NO_ERROR )
    return (0);

  lv_Offset = 0;
  while( lv_Offset < inSize)
  {
    if( lv_pBuffer[lv_Offset] != ipBuff[ lv_Offset ] )
    {
      U32 lv_Idx;
	    
	    /* Verification failed! */
	    uprintf( NOVERB,
	      " ERROR! Verification failed @ 0x%X (offset 0x%X / size 0x%X)\n\r",
	      inAddr, lv_Offset, inSize );
	    
	    /* Expected Data. */
	    uprintf( NOVERB, " Expected Data:\n\r" );
      for( lv_Idx=0; lv_Idx < inSize; lv_Idx++ )
        uprintf( NOVERB, "0x%02X ", ipBuff[lv_Idx] );
      
      uprintf( NOVERB, "\n\r" );      
      
      /* Corrupted Data. */
      uprintf( NOVERB, " Corrupted Data:\n\r" );
      for( lv_Idx=0; lv_Idx < inSize; lv_Idx++ )
        uprintf( NOVERB, "0x%02X ", lv_pBuffer[lv_Idx] );
      
      uprintf( NOVERB, "\n\r" );
      
      return( lv_Offset );
    }       
       
    lv_Offset++;
  }
   
  return( lv_Offset );
}

/*------------------------------------------------------------------------------
* FUNCTION    : flashdrv_DeviceInfo
*
* DESCRIPTION : displays device info
*
* PARAMETERS  : none
*
* RETURNS     : none
*-----------------------------------------------------------------------------*/
STATUS flashdrv_DeviceInfo( void )
{  
  if( LV_pDeviceDesc->Initialized == FALSE )
  {
    uprintf( NOVERB,
      " ERROR! MMC/SD device not initilized or not present.\n\r");
  }
  else
  {
    //-----------------------------------------------------------------------
    // module id (1/2)
    //-----------------------------------------------------------------------
    uprintf(NOVERB,
      " - Module ID ....... %d\n\r", LV_pDeviceData->ModuleID);
    
    //-----------------------------------------------------------------------
    // card type (MMC/SD)
    //-----------------------------------------------------------------------
    uprintf( NOVERB, " - Card Type ....... " );
    
    if( LV_pDeviceData->Type == MMCSD_TYPE_MMC )
      uprintf( NOVERB, "MMC\n\r" );
    else
      uprintf( NOVERB,"SD\n\r" );
      
    //-----------------------------------------------------------------------
    // spec version
    //-----------------------------------------------------------------------
/*    
    if( lv_pDeviceData->Type == SD_TYPE )
    {
      switch( lv_pDeviceData->SpecificationVersion )
      {
        case 0:
          sprintf( lv_SpecVersion, "1.00 - 1.01");
          break;
        case 1:
          sprintf( lv_SpecVersion, "1.10");
          break;
        case 2:
          sprintf( lv_SpecVersion, "2.00");
          break;
        case 3:
          sprintf( lv_SpecVersion, "2.10");
          break;
        default:
          sprintf( lv_SpecVersion, "unknown");
      }
    }
    else
    {
      switch( lv_pDeviceData->SpecificationVersion ) 
      {
        case 0:
          sprintf( lv_SpecVersion, "1.0, 1.2");
          break;
        case 1:
          sprintf( lv_SpecVersion, "1.4");
          break;
        case 2:
          sprintf( lv_SpecVersion, "2.0, 2.2");
          break;
        case 3:
          sprintf( lv_SpecVersion, "3.1, 3.2, 3.31");
          break;
        case 4:
          sprintf( lv_SpecVersion, "4.0, 4.1, 4.2, 4.3");
          break;  
        default:
          sprintf( lv_SpecVersion, "unknown");
      }
    }
                 
    uprintf(NOVERB,
      "      Spec.Version ...... %s\n\r", lv_SpecVersion);
*/ 
    //-----------------------------------------------------------------------
    // addressing mode (byte/sector)
    //-----------------------------------------------------------------------
    uprintf( NOVERB," - Addressing Mode . " );
    
    if( LV_pDeviceData->AddressingMode == MMCSD_ADDRESSING_BYTE )
      uprintf( NOVERB, "Byte, < 4GB\n\r" );
    else
      uprintf( NOVERB, "Sector, >= 4GB\n\r" );
    
    //-----------------------------------------------------------------------
    // supported bus width (1/4/8) 
    //-----------------------------------------------------------------------
    uprintf( NOVERB,
      " - Supp.Bus Width .. ");
    
    if( LV_pDeviceData->SupportedBusWidth > 1 )
      uprintf( NOVERB,"%d\n\r", LV_pDeviceData->SupportedBusWidth + 1 );
    else
      uprintf( NOVERB, "1\n\r" );
    
    //-----------------------------------------------------------------------
    // size
    //-----------------------------------------------------------------------
    uprintf( NOVERB,
      " - Size ............ %d MB\n\r", LV_pDeviceData->Size >> 11);
    
    //-----------------------------------------------------------------------
    // rca
    //-----------------------------------------------------------------------
    uprintf(NOVERB,
      " - RCA ............. %08x\n\r", LV_pDeviceData->RCA);
  }
  
  uprintf(NOVERB,"\n\r");
  
  return NO_ERROR;
}

/*---------------------------------- End of File -----------------------------*/

