/**
 * @file flash_drv.h
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


/**
 * @file silicon.h
 * @author 
 * @brief Flash driver interface declarations
 * @details 
 * @todo file details
 * @see <link>
 * @todo update link
 */


/*==== DECLARATION CONTROL =================================================*/
#ifndef FLASH_DRV_DEFS_H
#define FLASH_DRV_DEFS_H

/*==== INCLUDES ============================================================*/
#include <stdarg.h>
#include <stddef.h>
#include "types.h"
//#if defined EMMC_DRV
//#include "mmc_dis.h"
//#endif


/*==== MACROS ===============================================================*/

#ifdef SIMULATION
volatile U16 *do_flash_read(U32 addr);
void do_flash_write(U32 addr, U16 data);
#define FLASH_READ(addr)        ((U16)*do_flash_read(addr)) //TODO avoid U8 cast, was introduce to kill lot of warnings
#define FLASH_WRITE(addr, data) do_flash_write(addr, data)

volatile U8 *do_nand8_read(U32 addr);
void do_nand8_write(U32 addr, U8 data);
#define NAND8_READ(addr)          (*do_nand8_read(addr))
#define NAND8_WRITE(addr, data)   do_nand8_write(addr, data)
#else
#define FLASH_READ(addr)        (*(volatile U16 *) (addr))
#define FLASH_WRITE(addr, data) ((*(volatile U16 *) (addr)) = (data))
#define NAND8_READ(addr)        (*(volatile U8 *) (addr))
#define NAND8_WRITE(addr, data) ((*(volatile U8 *) (addr)) = (data))
#endif

/*==== CONSTS ==============================================================*/

#define OMAPFLASH_DRIVER_HEADER_STRING_V7               "OMAPFLASH DRIVER v7"
#define CTRL_Z                                          0x1A

/* Error codes used throughout all flash drivers. Must fit into 1 signed byte */

/* Generic flash driver error codes */
#define FLASH_DRV_SUCCESS                               (0x00)

#define FLASH_DRV_ERROR                                 (0x11)
#define FLASH_DRV_NOT_SUPPORTED                         (0x12)
#define FLASH_DRV_INVALID_PARAMETER                     (0x13)
#define FLASH_DRV_NO_MEMORY		                          (0x14)

/* NOR general codes */

#define FLASH_DRV_ERR_WRITE                             (0x20)
#define FLASH_DRV_ERR_ERASE                             (0x21)
#define FLASH_DRV_ERR_UNLOCK                            (0x22)
#define FLASH_DRV_ERR_BUFFERED_WRITE_ABORTED            (0x23)
#define FLASH_DRV_ERR_WRITE_TIMEOUT                     (0x24)
#define FLASH_DRV_ERR_BAD_SEQUENCE                      (0x25)
#define FLASH_DRV_ERR_VOLTAGE                           (0x26)
#define FLASH_DRV_ERR_PROTECTION                        (0x27)
#define FLASH_DRV_ERR_GENERAL_TIMEOUT                   (0x28)
#define FLASH_DRV_ERR_CFI_QRY_FAILED                    (0x29)
#define FLASH_DRV_ERR_BUFFERED_WRITE_NOT_SUPPORTED      (0x2A)
#define FLASH_DRV_ERR_BAD_SIZE                          (0x2B)

/* TBD NAND general codes */

/* NOR-Sibley specific codes */

#define FLASH_DRV_ERR_WRITE_OBJ_DATA_TO_CTRL_REGION     (0x70)
#define FLASH_DRV_ERR_REWRITE_OBJ_DATA                  (0x71)
#define FLASH_DRV_ERR_ILLEGAL_WRITE_CMD                 (0x72)

/* For dealing with pointers in const structures and const variables */

#define RELOCATED(p)                                    (((U32)get_setup_const() & 0xFFFF0000) | ((U32)p & 0x0000FFFF))

/*==== TYPES ===============================================================*/

/// @see T_dl_lazy_delay_callback and the use of dl_lazy_delay_ex 
typedef U32 T_wait_microsec_ex_callback(U32 remaining_time_ms, void *data);

typedef enum 
{ 
  DEFAULT = 0, 
  OPTIONAL = 1, 
  ALLOW_ODD = 2, 
  DECIMAL = 4, 
  NOALIGN = 8
} T_get_value_option;


typedef void *  (* T_driver_malloc           )(U32 tSize);
typedef void    (* T_driver_free             )(void * packet);
typedef int     (* T_driver_memcmp           )(const void * b1, const void * b2, size_t size);
typedef int     (* T_driver_strcmp           )(const char * s1, const char * s2);
typedef int     (* T_driver_strncmp          )(const char * s1, const char * s2, size_t size);
typedef void *  (* T_driver_memcpy           )(void * tgt, const void * src, size_t size);
typedef char *  (* T_driver_strcpy           )(char * tgt, const char * src);
typedef size_t  (* T_driver_strlen           )(const char * str);
typedef void *  (* T_driver_memset           )(void * tgt, int value, size_t size);
typedef void    (* T_driver_wait_microsec    )(U32 microsec);                    ///< Wait for specified nr. of microseconds
typedef U32     (* T_driver_wait_microsec_ex )(U32 microsec, T_wait_microsec_ex_callback *callback, void *data); ///< Wait for specified nr. of microseconds or until callback return non zero
typedef void    (* T_driver_dbg_printf       )(const char *format, ...);
typedef void    (* T_driver_dbg_vxprintf     )(int flags, const char *format, va_list arg_ptr);
typedef void    (* T_driver_send_info        )(char *format, ...);
typedef void    (* T_driver_vxsend_info      )(int flags, const char *format, va_list arg_ptr);
typedef void    (* T_driver_send_status      )(const char * description, U64 current, U64 target);
typedef void    (* T_drv_debug_signal        )(unsigned int id, unsigned int set);
typedef BOOLEAN (* T_driver_starts_with      )(char **text, const char *word);
typedef U64     (* T_driver_get_value        )(char **text, char *name, T_get_value_option options);
typedef void    (* T_driver_log_event        )(char * text);
typedef BOOLEAN (* T_loop_timeout_ms         )(U8 id, U32 time);

typedef struct T_driver_config
{
  // Required access functions for the driver for calls to services in the second to be populated by the second
  T_driver_malloc           drv_malloc;         ///< For allocating memory.
  T_driver_free             drv_free;           ///< For freeing allocated memory.
  T_driver_memcmp           drv_memcmp;         ///< Memory compare
  T_driver_strcmp           drv_strcmp;         ///< String compare
  T_driver_strncmp          drv_strncmp;        ///< String compare (limited)
  T_driver_memcpy           drv_memcpy;         ///< Memory copy
  T_driver_strcpy           drv_strcpy;         ///< String copy
  T_driver_strlen           drv_strlen;         ///< String length
  T_driver_memset           drv_memset;         ///< Memory set
  T_driver_wait_microsec    wait_microsec;      ///< For waiting x micro seconds.
  T_driver_wait_microsec_ex wait_microsec_ex;   ///< For waiting x micro seconds.
  T_driver_dbg_printf       dbg_printf;         ///< Function for debugging purposes.
  T_driver_dbg_vxprintf     dbg_vxprintf;       ///< Function for debugging purposes.
  T_driver_send_info        send_info;          ///< Function for sending back information to the host using the host protocol.
  T_driver_vxsend_info      vxsend_info;        ///< Function for sending back information to the host using the host protocol.
  T_driver_send_status      send_status;        ///< Function for sending back status progress information to the host using the host protocol.
  T_driver_log_event        log_event;          ///< Function for logging an event (time reference and text)
  T_loop_timeout_ms         loop_timeout_ms;    ///< Function for checking loop timeout (id < 3, time = 0 => reset)
  T_drv_debug_signal        drv_debug_signal;   ///< Function for setting (1) or clearing (0) a signal via a register identified by the id (0, 1 or 2)
  T_driver_starts_with      starts_with;        ///< Function for checking whether a string starts with a string
  T_driver_get_value        get_value;          ///< Function for getting a value
  const char *              processor;          ///< String version of OMAP... defined e.g. "OMAP3".

  // Configuration string passed to the driver from the second
  char *                    cstring;

  // Driver specific data allocated, populated and deallocated by the driver
  void *                    data;
} T_driver_config;

typedef struct
{
  char               tag[16];
  T_get_value_option option;
  BOOLEAN            mandatory;
} T_driver_setup_const;

typedef struct
{
  U32                value;
  BOOLEAN            valid;
} T_driver_setup_var;

typedef enum T_more_data
{
  NO_MORE_DATA                     = 0x0,           /* Last data are enclosed in current write call */
  MORE_DATA                        = 0x1            /* More data to come in another write call */
} T_more_data;

typedef struct
{
  U64 device_base_address;
  U64 device_size;
} T_driver_info;

// Dst Addr in case of read & Src Addr in case of write are memory addr in SDRAM
// and on 32-bit OMAP it is a 32-bit Addr.
typedef U32 (* T_driver_init    )(T_driver_config * config, char ** result);
typedef U32 (* T_driver_erase   )(T_driver_config * config, char ** result, U64 addr, U64 length);
typedef U32 (* T_driver_write   )(T_driver_config * config, char ** result, U64 dst_addr, U32 src_addr, U32 size, T_more_data more);
typedef U32 (* T_driver_read    )(T_driver_config * config, char ** result, U32 dst_addr, U64 src_addr, U32 size);
typedef U32 (* T_driver_deinit  )(T_driver_config * config, char ** result);
typedef U32 (* T_driver_get_info)(T_driver_config * config, T_driver_info * info);
  
typedef struct T_driver_if_functions
{
  const struct T_driver_if *self; //for getting located address
  T_driver_init     init;
  T_driver_read     read;
  T_driver_write    write;
  T_driver_erase    erase;
  T_driver_deinit   deinit;
  T_driver_get_info get_info;
} T_driver_if_functions;

#define OMAPFLASH_DRIVER_HEADER_STRING_SIZE 21
typedef struct T_driver_if
{
  // Header information for the application and driver
  struct
  {
    char           header[OMAPFLASH_DRIVER_HEADER_STRING_SIZE];
    char           driver[21];
    char           end;
  } id;
  T_driver_if_functions functions;
} T_driver_if;

///@todo most of these parameters can be auto detected I think
#define MMC_SLOT_MASK   0xF
#define MMC_SLOT_SHIFT  0
//MMC_VOLT currently ignored
#define MMC_VOLT_MASK   0xF
#define MMC_VOLT_SHIFT  4
#define MMC_WIDTH_MASK  0xF
#define MMC_WIDTH_SHIFT 8
//MMC_SPEED currently ignored
#define MMC_SPEED_MASK  0xFF
#define MMC_SPEED_SHIFT 12

#define MMC_SET(name,value) ((name##_MASK & value) << name##_SHIFT)
#define MMC_GET(name,value) (name##_MASK & (value >> name##_SHIFT))

#define MMC_PARAMS(slot,volt,width,speed) (MMC_SET(MMC_SLOT,slot)|MMC_SET(MMC_VOLT,volt)|MMC_SET(MMC_WIDTH,width)|MMC_SET(MMC_SPEED,speed))


#ifndef SECOND_DNLD_BUILD
#ifdef _MSC_VER
#define CDECL __cdecl 
#define DLLIMP __declspec(dllimport)
#else
#define CDECL
#define DLLIMP 
#endif

extern const T_driver_if driver_if;
extern const T_driver_setup_const setup_const[];
extern T_driver_setup_var setup_var[];

T_driver_config *get_config(void);
void set_config(T_driver_config * config);
const T_driver_setup_const *get_setup_const(void);
T_driver_setup_var *get_setup_var(void);

typedef struct T_driver_data T_driver_data;
T_driver_data *get_driver_data(void);

///@deprecated use drv_malloc from driver
DEPRECATED("use drv_malloc from driver")
void DLLIMP * CDECL malloc(size_t tSize);
#define malloc drv_malloc
void *drv_malloc(unsigned int tSize);

///@deprecated use drv_calloc from driver
DEPRECATED("use drv_calloc from driver")
void DLLIMP*CDECL calloc(size_t num, size_t size);
void *drv_calloc(size_t num, size_t size);

///@deprecated use drv_free from driver
DEPRECATED("use drv_free from driver")
void DLLIMP CDECL free(void *packet);
#define free drv_free
void drv_free(void *packet);

#define memcmp drv_memcmp
S32 drv_memcmp(const void * b1, const void * b2, size_t size);

#define strcmp drv_strcmp
S32 drv_strcmp(const char * s1, const char * s2);

#define strncmp drv_strncmp
S32 drv_strncmp(const char * s1, const char * s2, size_t size);

#define memcpy drv_memcpy
void * drv_memcpy(void * tgt, const void * src, size_t size);

#define strcpy drv_strcpy
char * drv_strcpy(char * tgt, const char * src);

#define strlen drv_strlen
size_t drv_strlen(const char * str);

#define memset drv_memset
void * drv_memset(void * tgt, int value, size_t size);
void wait_microsec(U32 microsec);
U32 wait_microsec_ex(U32 microsec, T_wait_microsec_ex_callback *callback, void *data);

BOOLEAN check_timeout_ms(U8 id, U32 millisec);

void drv_debug_led(int leds_mask, int leds_value);

///@deprecated use drv_printf from driver
#define printf dbg_printf
void dbg_printf(const char *format, ...);

///@todo eliminate duplication in disp.h
//#define VXPRINTF_STANDARD 0
//#define VXPRINTF_NO_REQUIRE_HASH 1

void dbg_vxprintf(int flags, const char *format, va_list arg_ptr);
void drv_send_info(char *format, ...);
void drv_vxsend_info(int flags, char *format, va_list arg_ptr);
void drv_send_status(char * description, U64 current, U64 target);
void drv_log_event(char * text);

int drv_parse_config(const T_driver_setup_const * setup_const, T_driver_setup_var * setup_var, char * cstring, T_driver_config * config);

BOOLEAN omap3(void);
BOOLEAN omap4(void);
BOOLEAN omap5(void);

#endif //SECOND_DNLD_BUILD

/*==== EXPORTS =============================================================*/


#endif
/*==== END OF FILE ===========================================================*/
