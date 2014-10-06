/*
 * Copyright (C) 2008 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the 
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _FASTBOOT_H_
#define _FASTBOOT_H_

#define usb_handle void

#include <stdio.h>
#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif
#include "glob.h"
#include "usb.h"

//limits
#define MAX_PACKAGE_COUNT 0x7F ///<@ limited by header format, must be (N**2)-1, @todo share this constant with target
#define ALIGNED_PACKAGE 0x80
#define HOST_MAX_PACKAGE_SIZE 0xFFFC ///<@ apparent restricition in windows usb interface, strictly windows says 0xFFFF but OMAP3 require 4 byte alignment reducing it to 0xFFFC
#define PACKAGE_SIZE_STEPS 4 ///<@ restricition by OMAP3

#define NONVERBOSE 0
#define VERBOSE 1

#define DRV_ERR_EXPECTED		0
#define DRV_ERR_NO_DRIVER		1
#define DRV_ERR_WRONG_DRIVER	2
#define	DRV_ERR_OLD_DRIVER		3

#define MBYTE	1048576L
#define MAX_CHUNK_SIZE	(384 * MBYTE)

//options
extern long comport;
extern long baudrate;
extern FILE *output;
extern FILE *logfile;
extern int stdout_verbose;
extern int rxtx_trace;
extern int show_debug_string;
extern const char *extern_reset;
extern const char *extern_power_on;
extern const char *extern_power_off;
extern int timeout; ///< seconds
extern const char *script_file;
extern int script_line;
extern int no_force_package_alignment;
extern int assert_on_error;
extern unsigned rxtx_delay;
extern unsigned data_delay;
extern unsigned force_ack;
extern int driverErr;
extern unsigned int chunkSize; ///< mbytes

#ifdef _WIN32
typedef clock_t clock_handle;
#else
typedef struct timeval clock_handle;
#endif

/* protocol.c - fastboot protocol */
int fb_command(usb_handle *usb, const char *cmd);
int fb_command_response(usb_handle *usb, const char *cmd, char *response);
int fb_download_data(usb_handle *usb, const void *data, unsigned size);
char *fb_get_error(void);

/* omap_protocol.c functions - make them more generic */
int fb_data_receive(omap_usb_handle *usb,void *data, unsigned size);
//int fb_data_receive(omap_usb_handle *usb,const void *data, unsigned size);
int fb_response(omap_usb_handle *usb, char *response);
int fb_download_data_packages(omap_usb_handle *usb, const void *data, unsigned size, const char *file);
//int fb_upload_data(omap_usb_handle *usb, const char *cmd, const void **data, unsigned *size);
int fb_upload_data(omap_usb_handle *usb, char *cmd, char *fname, unsigned *size);
int fb_data_send_raw(omap_usb_handle *usb,const void *data, unsigned size);

int getFileSize(const char *fn);
void *load_file(const char *fn, unsigned *_sz);
//void *load_file_lite(const char *fn, unsigned *_sz);
int load_file_chunk(const char *fn, char *data, unsigned sz, unsigned int offset);
typedef enum { 
    exact_size, 
    add_nl, 
    add_null,
    add_mod64_null,
    add_to_mod4
} T_load_file_mode;
void *load_file_ex(const char *fn, unsigned *_sz, T_load_file_mode mode);
int load_file_ex2(const char *fn, char *data, unsigned sz, unsigned int offset, T_load_file_mode mode);
int save_file(const char *fn, const char *data, unsigned size);
usb_handle *open_device(void);
int get_comport(void);

#ifdef _WIN32
void sleep(int seconds);
void sleepms(int mseconds);
void getCurrentTime(clock_t *clk);
void report_elapsed_time(clock_t clk1, clock_t clk2);
void report_elapsed_time_rate(clock_t clk1, clock_t clk2, unsigned bytes);
#else
void sleepms(int mseconds);
void getCurrentTime(struct timeval *clk);
void report_elapsed_time(struct timeval clk1, struct timeval clk2);
void report_elapsed_time_rate(struct timeval clk1, struct timeval clk2, unsigned bytes);
#endif


#define FB_COMMAND_SZ 64
//#define FB_RESPONSE_SZ 128
#define FB_RESPONSE_SZ 256 //long buffer needed by INFO_TRACE, @todo common buffer size define between host and target


/* engine.c - high level command queue engine */
void fb_queue_flash(const char *ptn, void *data, unsigned sz);;
void fb_queue_erase(const char *ptn);
void fb_queue_require(const char *var, int invert, unsigned nvalues, const char **value);
void fb_queue_display(const char *var, const char *prettyname);
void fb_queue_reboot(void);
void fb_queue_command(const char *cmd, const char *msg);
void fb_queue_command2(const char *cmd, const char *arg, const char *msg);
void fb_queue_command3(const char *cmd, const char *arg, const char *arg2, const char *msg);
void fb_queue_command4(const char *cmd, const char *arg, const char *arg2, const char *arg3, const char *msg);
void fb_queue_confirmed_command4(const char *cmd, const char *arg, const char *arg2, const char *arg3, const char *msg);
void fb_queue_download(const char *name, void *data, unsigned size);
void fb_queue_download_packages(const char *file, const char * info, void *data, unsigned size);
void fb_queue_reconnect(const char *msg);
//struct Action *fb_queue_upload(char *chip_name, char *offset, unsigned size);
struct Action *fb_queue_upload(char *chip_name, char *offset, unsigned size, char *fname);
void fb_queue_save(char *fname, struct Action *a);
void fb_queue_notice(const char *notice);
int fb_execute_queue(usb_handle *usb);

struct Action *fb_begin_injection(struct Action *a);
void fb_end_injection(struct Action *last);
struct Action *fb_queue_nothing(void);
void fb_queue_chip_driver(char *chip);
void fb_dump_queue(void);
void fb_queue_fastboot(void);
void fb_queue_mmc_erase(const char *cno, const char *ptn);
void fb_queue_mmc_write(const char *cno, const char *ptn, void *data, unsigned sz);
void fb_queue_syscmd(const char *syscmd);
void fb_queue_change_options(const char *options);


/* util stuff */
void die(const char *fmt, ...);

omap_usb_handle *pheriphal_boot(const char *product, const char *config2nd_file, struct Action *begin, char *omap_device);
int save2ndFile(const char *product, const char *config2nd_file, struct Action *begin, char *omap_device);

int arg(int argc, char **argv, char *symbols[][2], const char *file, int line);
int arg_lines(char *data, const char *file, int line, char *symbols[][2]);
extern int peripheralboot_reopen;

/* pheriphalboot.c */
char *pb_get_error(void);
char *pb_search_file(char *config_data, const char *config_file, char* front, char **cmdline, int *config2nd_line);

/* omap_windows.h */
//int usb_write(usb_handle* drv, const void* data, int len); //TODO do we really need three versions? (fastboot only have one version)
int usb_write_raw(omap_usb_handle* drv, const void* data, int len);
int usb_write_bin(omap_usb_handle* drv, const void* data, int len, unsigned count);
int usb_write_cstr(omap_usb_handle* drv, const void* data, int len);
int usb_write_ack(omap_usb_handle* drv);

#endif
