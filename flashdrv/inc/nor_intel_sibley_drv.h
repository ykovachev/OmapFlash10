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
| Filename:   nor_intel_sibley_drv.h
| Author:     Vishnuprasad S Kodangal
| Purpose:    Intel Sibley flash driver
+----------------------------------------------------------------------------*/


/*==== DECLARATION CONTROL =================================================*/
#ifndef SIBLEY_INTEL_DRV_H
#define SIBLEY_INTEL_DRV_H

/*==== INCLUDES ============================================================*/
#include "types.h"

/*==== CONSTS ==============================================================*/
#define PAGE_SIZE               1024
#define PAGE_MASK               0xFFFFFC00

#define READ_STATUS             0x70

#define LOCK_ERROR              (1 << 1)
#define VPP_ERROR               (1 << 3)
#define PROGRAM_ERROR           (1 << 4)
#define ERASE_ERROR             (1 << 5)
#define DEVICE_READY            (1 << 7)
#define OBJ_MODE_VIOLATION      (1 << 8)
#define CTRL_MODE_VIOLATION     (1 << 9)

#define DEVICE_ERROR            0x30

#define CLEAR_STATUS            0x50
#define READ_ARRAY              0xFF
#define READ_DEVICE_ID          0x90
#define BLOCK_ERASE             0x20
#define CONFIRM                 0xD0
#define WRITE_BUFFER            0xE9
#define WRITE_WORD              0x41

#define LOCK_SETUP              0x60
#define BLOCK_UNLOCK            0xD0
#define BLOCK_LOCK              0x01



/*==== TYPES ===============================================================*/
typedef struct
{
    U32 device_base_address;
    U32 device_size;
    U16 data_buffer[PAGE_SIZE/2];
    U32 buf_write_addr;
    U32 buf_write_size;
    U8 buf_valid;
} T_SIBLEY_PARAMS;

/*==== EXPORTS =============================================================*/
U32 nor_intel_sibley_init(T_driver_config * config, char ** result);
U32 nor_intel_sibley_deinit(T_driver_config * config, char ** result);
U32 nor_intel_sibley_erase(T_driver_config * config, char ** result, U64 address, U64 length);
U32 nor_intel_sibley_write(T_driver_config * config, char ** result, U64 dest_addr, U32 src_addr, U32 size, T_more_data more);
U32 nor_intel_sibley_read(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
U32 nor_intel_sibley_get_info(T_driver_config * config, T_driver_info * info);

#endif
/*==== END OF FILE ===========================================================*/
