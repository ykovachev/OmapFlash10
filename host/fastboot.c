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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
//#include <direct.h> //TODO remove
#include <limits.h>
#include <ctype.h>
#include "fastboot.h"
#ifdef _WIN32
#include <windows.h>
#include "util_windows.h"
#endif

#include "output.h"

#ifndef _WIN32
#include <sys/time.h>
#endif
#include "bootimg.h"
#ifndef NO_ZIP
#include <zipfile/zipfile.h>
#endif

#include "version.h"
#include "fastboot.h"
#include "output.h"
#include "board_configuration.h"

#define MAX_FILE_ARGV 100
#define SECOND_PARITY_DEFAULT NOPARITY ///<@todo cleanup 
#define SECOND_PARITY_OMAP3 NOPARITY
#define SECOND_PARITY_OMAP4 EVENPARITY
#define SECOND_PARITY_OMAP5 EVENPARITY

#define PER_BOOT_IF_USB    0
#define PER_BOOT_IF_UART   1

// Flag for fastboot Mode
unsigned fastbootMode = 0;

static usb_handle *usb = 0;
static const char *serial = 0;
static const char *product = 0;
static const char *cmdline = 0;
static int wipe_data = 0;
static unsigned short vendor_id = 0;

static unsigned base_addr = 0x10000000;

long comport = 0; ///<comport to use for UART connection or 0 if USB is to be used.
long baudrate = 115200; ///<baudrate to use for UART connection
FILE *output; ///<file descriptor to use for screen output (defult is stderr, -stdout will select stdout)
FILE *logfile; ///<if set print screen output to this file too (always verbosed)
char *save2ndfname;
char custAsicId[100];   // to store Asic ID inputed from the CLI. 
int save2ndFlag = FALSE;    // set this flag if 2nd needs to be saved locally instead of sending it to board.
int custAsicIdFlag = FALSE; // set this flag if custAsicId is provided
int stdout_verbose = 0; ///<print verbosed output on screen
int rxtx_trace = 0; ///<turn on rx/tx low-level trace. Be aware that TX-ACKs are traced before associated RX.
unsigned rxtx_delay = 10;
unsigned data_delay = 0;
unsigned force_ack = 0;
int show_debug_string = 0; ///<@todo set if debugger present
const char *extern_reset; ///<external command to issue a reset of board (reset recommended for omap4 es1.0 wake-up board)
const char *extern_power_on; ///<external command to issue a board power on
const char *extern_power_off; ///<external command to issue a board power off
int timeout = 5; ///< seconds, general timeout when waiting for target
int parity = SECOND_PARITY_DEFAULT; ///< the parity to use for UART (this only affect the host side and only when talking to 2nd i.e. not during boot)
int parity_explicit_defined = 0;
const char *script_file; ///< current file name when reading arg files or associated file name during queue execution
int script_line; ///< current line number when reading arg files or associated line number during queue execution
int no_force_package_alignment;
int assert_on_error = 0; ///<when assert_on_error is non-zero we assert in case of error so developer can click retry to enter debugger

int args_seen = 0; ///<used tofigure if we want to warn about late -v
int verbose_arg_indent = 0; ///<used when printing args to logfile

/* make sure that chunkSize is atleast packet size from target */
unsigned int chunkSize = 265 * MBYTE;	// by default the chunk size is 265 MB

unsigned int peripheral_boot_if = PER_BOOT_IF_USB; 
unsigned int renew_driver = 0;
unsigned int in_fastboot_mode = 0;

char * chunkSizeMem;  // pointer to CHUNKSIZE malloc used for download/upload

#define NAMED_VALUE(name) {#name, name}
typedef struct t_named_value
{
  const char *name;
  int value;
} t_named_value;

const t_named_value parity_names[] = 
{
  NAMED_VALUE(NOPARITY),
  NAMED_VALUE(ODDPARITY),
  NAMED_VALUE(EVENPARITY),
  NAMED_VALUE(MARKPARITY),
  NAMED_VALUE(SPACEPARITY),
  0
};

int find_named_value(const t_named_value *named_value, const char *name, int *value)
{
  int i;
  for (i = 0; named_value[i].name; i++)
  {
    if (!STRICMP(named_value[i].name, name))
    {
      *value = named_value[i].value;
      return 0;
    }
  }
  return -1;
}

/// this is a t_do_command used as parameter to do_command
void set_extern_reset(const char *command)
{
  extern_reset = strdup(command);
}

/// this is a t_do_command used as parameter to do_command
void set_extern_power_on(const char *command)
{
  extern_power_on = strdup(command);
}

/// this is a t_do_command used as parameter to do_command
void set_extern_power_off(const char *command)
{
  extern_power_off = strdup(command);
}

/// this is called when something fatal is happening
void die(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  host_vlogf("ERROR: ", fmt, ap);
  if (script_line)
  {
    host_logf("Related location: line %d of %s", script_line, script_file);
  }
  va_end(ap);
  end_log(); //<generate terminating \n
  assert(!assert_on_error);
  exit(1);
}    

void get_my_path(char *path);

char *find_item(const char *item, const char *product)
{
  char *dir;
  char *fn;
  char path[PATH_MAX + 128];

  if(!strcmp(item,"boot")) {
    fn = "boot.img";
  } else if(!strcmp(item,"recovery")) {
    fn = "recovery.img";
  } else if(!strcmp(item,"system")) {
    fn = "system.img";
  } else if(!strcmp(item,"userdata")) {
    fn = "userdata.img";
  } else if(!strcmp(item,"info")) {
    fn = "android-info.txt";
  } else {
    host_logf("Unknown partition '%s'", item);
    return 0;
  }

  if(product) {
    get_my_path(path);
    sprintf(path + strlen(path),
      "../../../target/product/%s/%s", product, fn);
    return strdup(path);
  }

  dir = getenv("ANDROID_PRODUCT_OUT");
  if((dir == 0) || (dir[0] == 0)) {
    die("Neither -p product specified nor ANDROID_PRODUCT_OUT set");
    return 0;
  }

  sprintf(path, "%s/%s", dir, fn);
  return strdup(path);
}


int match_fastboot(usb_ifc_info *info)
{
  if(fastbootMode)
  {
    if(!(vendor_id && (info->dev_vendor == vendor_id)) &&
      (info->dev_vendor != 0x18d1) &&
      (info->dev_vendor != 0x0451) &&  // TI
      (info->dev_vendor != 0x0502) &&
      (info->dev_vendor != 0x0fce) &&  // Sony Ericsson
      (info->dev_vendor != 0x05c6) &&  // Qualcomm
      (info->dev_vendor != 0x22b8) &&  // Motorola
      (info->dev_vendor != 0x0955) &&  // Nvidia
      (info->dev_vendor != 0x413c) &&  // DELL
      (info->dev_vendor != 0x0bb4))    // HTC
      return -1;

    if(info->ifc_class != 0xff) return -1;
    if(info->ifc_subclass != 0x42) return -1;
    if(info->ifc_protocol != 0x03) return -1;
  }
  else
  {
    if(!(vendor_id && (info->dev_vendor == vendor_id)) &&
      (info->dev_vendor != 0x0451))  // TI
      return -1;

    if(info->ifc_class != 0xff) return -1;
    if(info->ifc_subclass != 0xff) return -1;
    if(info->ifc_protocol != 0xff) return -1;
  }

  // require matching serial number if a serial number is specified
  // at the command line with the -s option.
  if (serial && strcmp(serial, info->serial_number) != 0) return -1;
  return 0;
}

int list_devices_callback(usb_ifc_info *info)
{
  if (match_fastboot(info) == 0) {
    char* serial = info->serial_number;
    if (!info->writable) {
      serial = "no permissions"; // like "adb devices"
    }
    if (!serial[0]) {
      serial = "????????????";
    }
    // output compatible with "adb devices"
    host_logf("%s\tfastboot", serial);
  }

  return -1;
}

typedef struct t_open_device
{
  int announce;
  usb_handle *usb;
} t_open_device;

int open_device_callback(int timeleft, void *data)
{
  t_open_device *info = (t_open_device*)data;

  info->usb = (fastbootMode) ? (usb_handle*)fb_usb_open(match_fastboot) : (usb_handle*)open_omap_device(comport, baudrate, parity);

  if(info->usb) {
    if (fastbootMode)
      host_logf_ex(VERBOSE, "Found device (fastboot)");
    else if (comport)
      host_logf_ex(VERBOSE, "Found device (omap com%d)", comport);
    else
      host_logf_ex(VERBOSE, "Found device (omap usb)");
    return timeleft;
  }
  if(info->announce) {
    info->announce = 0;    
    if (fastbootMode)
      host_logf_ex(VERBOSE, "Waiting for device (fastboot)");
    else if (comport)
      host_logf_ex(VERBOSE, "Waiting for device (omap com%d)", comport);
    else
      host_logf_ex(VERBOSE, "Waiting for device (omap usb)");
  }
  //sleep(1);
  sleepms(100);
  //sleepms(50);
  //sleepms(10);
  //sleepms(150);
  //sleepms(500);
  return 0;
}

usb_handle *open_device(void)
{
  t_open_device info = {1, NULL};
  int timeremaining = wait_millisec(timeout * 1000, open_device_callback, &info);
  return info.usb;
}

int get_comport(void)
{
  return comport;
}

void list_devices(void) {
  if(fastbootMode)
  {
    // We don't actually open a USB device here,
    // just getting our callback called so we can
    // list all the connected devices.
    fb_usb_open(list_devices_callback);	
  } else {
    die ("devices command not supported");
  }
}

void usage(void)
{
  fprintf(stderr,
    // TODO this is way out of date
    /*           1234567890123456789012345678901234567890123456789012345678901234567890123456 */
    "\n"
    "usage: omapflash [ <option> ] <command> or omapflash @<file>\n"
    "\n"
    "OMAP download/upload\n"
    "  chip_download [<device>|<partition>][@offset] <file>       download file to device/partition starting at offset\n"
    "  chip_erase [<device>|<partition>][@offset] <size>          erase size bytes (0 is all) in device/partition from offset\n"
    "  chip_upload [<device>|<partition>][@offset] <size> <file>  upload size bytes to file from device/partiton starting at offset\n"
    "\n"
    "commands (prefixed by command keyword and terminated by newline or comma):\n"
    "  status                                       check whether target responds (returns OK ready)\n"
    "  branch <device> <offset>                     start execution from device and offset address\n"
    "  jump <absolute address>                      start execution absolute address\n"
    "  cold_sw_reset                                issue a cold sw reset\n"
    "  warm_sw_reset                                issue a warm sw reset\n"
    "  peek32 <address>                             read from memory location\n"
    "  poke32 <address> <value>                     write to memory location\n"
    "  peekpoke32 <address> <and value> <or value>  modify memory location\n"
    "  mtquick <start address> <end address>        quick memory test\n"
    "  mtaddr <start address> <end address>         address-write memory test\n"
    "  mtmarchx <start address> <end address>       marchx memory test\n"
    "  mtif                                         memory interface integrity check (LPDDR2)\n"
    "  memdump <start address> <end address>        dump and show memory - byte based\n"
    "  worddump <start address> <end address>       dump and show memory - word based\n"
    "  memfill <start address> <end address> <val>  fill memory (32-bit value based)\n"
    "  list_commands                                list all supported commands in target binary\n"
    "\n"
    "  Note that due to size limitations for the target binary, some of the commands may or may not be supported\n"
    "  by pre-built versions of the target binary. Use list_commands to check for support.\n"
    "\n"
    "options:\n"
    "  -t <timeout>                                 set communication timeout in seconds\n"
    "  -p <platform>                                specify the target platform used\n"
    "  -omap <version>                              specify the OMAP version of the target platform\n"
    "  -v                                           verbose output\n"
    "  -chunk <size>                                set MB size of read buffer for large files\n" 
    "  -rxtx_delay <ms>                             set the rx-to-tx turn around delay in ms (default is 10)\n" 
    "  -data_delay <ms>                             set the header-to-data delay in ms (default is 0)\n" 
    "  -force_ack                                   require target ack of header and data section of all messages"
    "  -renew_driver                                force renewed driver download for each file download\n"
    "  -logfile <filename>                          log output to a file\n" 
    "  -save2nd <filename>                          file name to save 2nd file along with board configuration data\n" 
    "                                               (Note: OMAPFlash will exit after saving 2nd file.)\n" 
    "  -asic_id <AsicId.OmapId> <AsicId.Version> <HS|GP>    asic_id string that matches the format in omapflash2nd.txt \n" 
    "                                               (Note: -asic_id is required for -save2nd option if used offline\n" 
    "  @<file>                                      read options/commands form file\n"
    "                                               use '#' to start comment line\n"
    "                                               parameters for options/commands must be on same line!\n"
    "\n"
    "fastboot specific commands:\n"
    "  fastboot                                     to put OMAPFlash in Fastboot mode\n"
    "  getvar <variable>                            display a bootloader variable\n"
    "  flash <partition> [ <file> ]                 write a file to a flash partition\n"
    "  erase <partition>                            erase a flash partition\n"
    "  mmcwrite <controller_no> <partition> <file>  write a file to a mmc partition\n"
    "  mmcerase <controller_no> <partition>         erase a mmc partition\n"
    "  download <file>                              download an image to memory\n"
    "  boot <kernel> [ <ramdisk> ]                  download and boot kernel\n"
    "  reboot                                       reboot device normally\n"
    "  reboot-bootloader                            reboot device into bootloader\n"
    "\n"
    "fastboot commands supported natively by OMAPFlash:\n"
    "  flash <partition> <file>                     flash a file to a named partition as per board configuration\n"
    "  erase <partition>                            erase a partition as per board configuration\n"
    "  reboot                                       reboot device normally (cold reset)\n"
    "  oem format                                   format and download OEM partition table as per board configuration\n"
    "\n"
    "system <command>                               command to execute <command> on host PC\n"
    );
  assert(!assert_on_error);
  exit(1);
}

void *load_bootable_image(const char *kernel, const char *ramdisk, 
  unsigned *sz, const char *cmdline)
{
  void *kdata = 0, *rdata = 0;
  unsigned ksize = 0, rsize = 0;
  void *bdata;
  unsigned bsize;

  if(kernel == 0) {
    host_logf("No image specified\n");
    return 0;
  }

  kdata = load_file(kernel, &ksize);
  if(kdata == 0) {
    host_logf("Cannot load '%s'", kernel);
    return 0;
  }

  /* is this actually a boot image? */
  if(!memcmp(kdata, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
    if(cmdline) bootimg_set_cmdline((boot_img_hdr*) kdata, cmdline);

    if(ramdisk) {
      host_logf("Cannot boot a boot.img *and* ramdisk");
      return 0;
    }

    *sz = ksize;
    return kdata;
  }

  if(ramdisk) {
    rdata = load_file(ramdisk, &rsize);
    if(rdata == 0) {
      host_logf("Cannot load '%s'", ramdisk);
      return  0;
    }
  }

  host_logf("Creating boot image...");
  bdata = mkbootimg(kdata, ksize, rdata, rsize, 0, 0, 2048, base_addr, &bsize);
  if(bdata == 0) {
    host_logf("Failed to create boot.img");
    return 0;
  }
  if(cmdline) bootimg_set_cmdline((boot_img_hdr*) bdata, cmdline);
  host_logf("Creating boot image - %d bytes", bsize);
  *sz = bsize;

  return bdata;
}

#ifndef NO_ZIP
void *unzip_file(zipfile_t zip, const char *name, unsigned *sz)
{
  void *data;
  zipentry_t entry;
  unsigned datasz;

  entry = lookup_zipentry(zip, name);
  if (entry == NULL) {
    host_logf("archive does not contain '%s'", name);
    return 0;
  }

  *sz = get_zipentry_size(entry);

  datasz = *sz * 1.001;
  data = malloc(datasz);

  if(data == 0) {
    host_logf("failed to allocate %d bytes", *sz);
    return 0;
  }

  if (decompress_zipentry(entry, data, datasz)) {
    host_logf("failed to unzip '%s' from archive", name);
    free(data);
    return 0;
  }

  return data;
}
#endif //NO_ZIP

static char *strip(char *s)
{
  int n;
  while(*s && isspace(*s)) s++;
  n = strlen(s);
  while(n-- > 0) {
    if(!isspace(s[n])) break;
    s[n] = 0;
  }
  return s;
}

#define MAX_OPTIONS 32
static int setup_requirement_line(char *name)
{
  char *val[MAX_OPTIONS];
  const char **out;
  unsigned n, count;
  char *x;
  int invert = 0;

  if (!strncmp(name, "reject ", 7)) {
    name += 7;
    invert = 1;
  } else if (!strncmp(name, "require ", 8)) {
    name += 8;
    invert = 0;
  }

  x = strchr(name, '=');
  if (x == 0) return 0;
  *x = 0;
  val[0] = x + 1;

  for(count = 1; count < MAX_OPTIONS; count++) {
    x = strchr(val[count - 1],'|');
    if (x == 0) break;
    *x = 0;
    val[count] = x + 1;
  }

  name = strip(name);
  for(n = 0; n < count; n++) val[n] = strip(val[n]);

  name = strip(name);
  if (name == 0) return -1;

  /* work around an unfortunate name mismatch */
  if (!strcmp(name,"board")) name = "product";

  out = malloc(sizeof(char*) * count);
  if (out == 0) return -1;

  for(n = 0; n < count; n++) {
    out[n] = strdup(strip(val[n]));
    if (out[n] == 0) return -1;
  }

  fb_queue_require(name, invert, n, out);
  return 0;
}

static void setup_requirements(char *data, unsigned sz)
{
  char *s;

  s = data;
  while (sz-- > 0) {
    if(*s == '\n') {
      *s++ = 0;
      if (setup_requirement_line(data)) {
        die("Out of memory");
      }
      data = s;
    } else {
      s++;
    }
  }
}

void queue_info_dump(void)
{
  fb_queue_notice("--------------------------------------------");
  fb_queue_display("version-bootloader", "Bootloader Version...");
  fb_queue_display("version-baseband",   "Baseband Version.....");
  fb_queue_display("serialno",           "Serial Number........");
  fb_queue_notice("--------------------------------------------");
}

#ifndef NO_ZIP
void do_update_signature(zipfile_t zip, char *fn)
{
  void *data;
  unsigned sz;
  data = unzip_file(zip, fn, &sz);
  if (data == 0) return;
  fb_queue_download("signature", data, sz);
  fb_queue_command("signature", "installing signature");
}
#endif //NO_ZIP

void do_update(char *fn)
{
#ifdef NO_ZIP
  die("Zip files not supported");
#else //NO_ZIP
  void *zdata;
  unsigned zsize;
  void *data;
  unsigned sz;
  zipfile_t zip;

  queue_info_dump();

  zdata = load_file(fn, &zsize);
  if (zdata == 0) die("Failed to load '%s'", fn);

  zip = init_zipfile(zdata, zsize);
  if(zip == 0) die("Failed to access zipdata in '%s'", fn);

  data = unzip_file(zip, "android-info.txt", &sz);
  if (data == 0) {
    char *tmp;
    /* fallback for older zipfiles */
    data = unzip_file(zip, "android-product.txt", &sz);
    if ((data == 0) || (sz < 1)) {
      die("Update package has no android-info.txt or android-product.txt");
    }
    tmp = malloc(sz + 128);
    if (tmp == 0) die("Out of memory");
    sprintf(tmp,"board=%sversion-baseband=0.66.04.19\n",(char*)data);
    data = tmp;
    sz = strlen(tmp);
  }

  setup_requirements(data, sz);

  data = unzip_file(zip, "boot.img", &sz);
  if (data == 0) die("Update package missing boot.img");
  do_update_signature(zip, "boot.sig");
  fb_queue_flash("boot", data, sz);

  data = unzip_file(zip, "recovery.img", &sz);
  if (data != 0) {
    do_update_signature(zip, "recovery.sig");
    fb_queue_flash("recovery", data, sz);
  }

  data = unzip_file(zip, "system.img", &sz);
  if (data == 0) die("Update package missing system.img");
  do_update_signature(zip, "system.sig");
  fb_queue_flash("system", data, sz);
#endif //NO_ZIP
}

void do_send_signature(char *fn)
{
  void *data;
  unsigned sz;
  char *xtn;

  xtn = strrchr(fn, '.');
  if (!xtn) return;
  if (strcmp(xtn, ".img")) return;

  strcpy(xtn,".sig");
  data = load_file(fn, &sz);
  strcpy(xtn,".img");
  if (data == 0) return;
  fb_queue_download("signature", data, sz);
  fb_queue_command("signature", "Installing signature");
}

void do_flashall(void)
{
  char *fname;
  void *data;
  unsigned sz;

  queue_info_dump();

  fname = find_item("info", product);
  if (fname == 0) die("Cannot find android-info.txt");
  data = load_file(fname, &sz);
  if (data == 0) die("Could not load android-info.txt");
  setup_requirements(data, sz);

  fname = find_item("boot", product);
  data = load_file(fname, &sz);
  if (data == 0) die("Could not load boot.img");
  do_send_signature(fname);
  fb_queue_flash("boot", data, sz);

  fname = find_item("recovery", product);
  data = load_file(fname, &sz);
  if (data != 0) {
    do_send_signature(fname);
    fb_queue_flash("recovery", data, sz);
  }

  fname = find_item("system", product);
  data = load_file(fname, &sz);
  if (data == 0) die("Could not load system.img");
  do_send_signature(fname);
  fb_queue_flash("system", data, sz);   
}

#define skip(n) do { int skip_n = (n); argc -= (skip_n); argv += (skip_n); } while (0)
#define require(n) do { if (argc < (n)) usage(); } while (0)

typedef void t_do_command(const char *cmd);

/// this is a t_do_command used as parameter to do_command
void oem_command(const char *command)
{
  fb_queue_command(command, strdup(command));    
}

int do_command(int argc, char **argv, int until_comma, t_do_command *do_command_)
{
  char command[256] = {0};

  command[0] = 0;
  while(1) {
    if (until_comma && !strcmp(*argv,",")) {
      skip(1);
      break;
    }
    if (command[0])
      strcat(command," ");

    strcat(command, *argv);
    skip(1);
    if(argc == 0) break;
  }
  do_command_(command);    
  return argc;
}

int wants_wipe = 0;
int wants_reboot = 0;
int wants_reboot_bootloader = 0;
int wants_2nd = 0;
char *omap_device = 0; 
void *data;
char *config2nd = "omapflash2nd.txt";
int new_argc;
int peripheralboot_reopen;

static int arg_file(const char *file, char *symbols[][2]);

char *offset_and_chip_name(char *chip_name) 
{
  char *offset = strchr(chip_name,'@');
  char *end = 0;
  if (offset) {
    *offset++ = 0;
    //if (!stricmp(offset, "auto")) {
    strtoul(offset + 1, &end, 16);
    if (*end) 
      die("Bad offset format '%s' - expected hex value\n");
    //}
  }
  else {
    offset = "0";
  }
  return offset;
}

int arg_with_digit_value(int *pargc, char ***pargv, char *option_name, char **value)
{
  int argc = *pargc;
  char **argv = *pargv;
  int len = strlen(option_name);
  if(!strcmp(*argv, option_name)) {
    require(2);
    *value = argv[1];
    skip(2);
  } else if(!strncmp(*argv, option_name, len) && isdigit((*argv)[len])) {
    *value = argv[0] + len;
    skip(1);
  } else if(!strncmp(*argv, option_name, len) && (*argv)[len]==':') {
    *value = argv[0] + len + 1; ///< +1 for ':'
    skip(1);
  } else {
    return -1;
  }
  *pargc = argc;
  *pargv = argv;
  return 0;
}

int arg_with_unsigned_value(int *pargc, char ***pargv, char *option_name, unsigned *value)
{
  int argc = *pargc;
  char **argv = *pargv;
  char *end;
  int len = strlen(option_name);
  if(!strcmp(*argv, option_name)) {
    require(2);
    if (argv[1]) {
      //if (!strnicmp(argv[1], "0x", 2))
      //    *value = strtoul(argv[1]+2, &end, 16);
      //else
      //    *value = strtoul(argv[1], &end, 16);
      *value = strtoul(argv[1], &end, 0);
      if (*end) 
        //die("Option %s must be followed by unsigned hex value (found %s)", option_name, argv[1]);
        die("Option %s must be followed by unsigned value (found %s)", option_name, argv[1]);
    }
    else {
      return -1;
    }
    skip(2);
  } else if(!strncmp(*argv, option_name, len) && (*argv)[len]==':') {
    *value = strtoul(argv[0] + len + 1, &end, 0); ///< +1 for ':'
    if (*end) 
      die("Option %s must be followed by unsigned value (found %s)", option_name, argv[1]);
    skip(1);
  } else {
    return -1;
  }
  *pargc = argc;
  *pargv = argv;
  return 0;
}

int arg(int argc, char **argv, char *symbols[][2], const char *file, int line)
{
  const char * save_script_file = script_file;
  int save_script_line = script_line;
  char *end;
  char *temp;
  unsigned sz;

  script_file = file;
  script_line = line;
  if (symbols) {
    int arg;
    for (arg = 0; arg < argc; arg++) {
      if (argv[arg][0] == '%') {
        int symbol;
        for (symbol = 0; symbols[symbol][0]; symbol++) {
          if (!strcmp(argv[arg] + 1, symbols[symbol][0])) {
            argv[arg] = symbols[symbol][1];
            break;
          }
        }
        if (!symbols[symbol][0]) {
          die("Unknown %% symbol '%s'", argv[arg]);
        }                    
      }
    }
  }

  for (;argc > 0; args_seen++) {
    char **prev_argv = argv;

    if(!strcmp(*argv, "-w")) {
      wants_wipe = 1;
      skip(1);
    } else if(!strcmp(*argv, "-b")) {
      require(2);
      base_addr = strtoul(argv[1], 0, 16);
      skip(2);
    } else if(!strcmp(*argv, "-2")) {
      wants_2nd = 1;
      skip(1);
    } else if(!arg_with_digit_value(&argc,&argv, "-omap", &omap_device)) {
      if (!parity_explicit_defined) 
      {
        if (!strcmp(omap_device,"5")) 
        {
          parity = SECOND_PARITY_OMAP5;
        }
        else if (!strcmp(omap_device,"4")) 
        {
          parity = SECOND_PARITY_OMAP4;
        }
        else if (!strcmp(omap_device,"3")) 
        {
          parity = SECOND_PARITY_OMAP3;
        }
      }
    } else if(!strcmp(*argv, "-board_config"))
    {
      require(2);
      parse_configuration(argv[1]);
      skip(2);
    } else if(!strcmp(*argv, "-peripheralboot_reopen")) {
      // This is must for ROM USB API but fatal for CSST USB API set it in omapflash2nd.txt
      // It is ignore for UART since that need parity setting change it make no sense to turn off reopen there
      peripheralboot_reopen = 1;
      skip(1);
    } else if(!strcmp(*argv, "-s")) {
      require(2);
      serial = argv[1];
      skip(2);
    } else if(**argv == '@') {
      if ((*argv)[1]) {
        if (arg_file(*argv + 1, symbols))
          goto error;
        skip(1);
      }
      else {
        /// support '@ filename.txt' for benefit of user that use tab completion on commandline (no tab completion immediately after @ in all shells)
        require(2);
        if (arg_file(argv[1], symbols))
          goto error;
        skip(2);
      }
    } else if(!strcmp(*argv, "-v")) {
      skip(1);
      if (args_seen && !stdout_verbose) host_logf("For best result specify -v as first option");
      stdout_verbose = 1;
    } else if(!strcmp(*argv, "-stdout")) {
      skip(1);
      output = stdout;
      setvbuf(output, 0, _IONBF, 0);
    } else if(!strcmp(*argv, "-renew_driver")) {
      skip(1);
      renew_driver = 1;
    } else if(!strcmp(*argv, "-force_ack")) {
      skip(1);
      force_ack = 1;
    } else if(!strcmp(*argv, "-logfile")) {
      require(2);
      logfile = fopen(argv[1],"w");
      if (!logfile) die("Logfile: Could not open '%s'", argv[1]);
      skip(2);
    } else if(!strcmp(*argv, "-rxtx_trace")) {
      skip(1);
      rxtx_trace = 1;
    } else if(!arg_with_unsigned_value(&argc, &argv, "-rxtx_delay", &rxtx_delay)) {
      if (rxtx_delay > 1000) die ("Invalid slowdown value %s", argv[1]);
    } else if(!arg_with_unsigned_value(&argc, &argv, "-data_delay", &data_delay)) {
      if (data_delay > 1000) die ("Invalid delay value %s", argv[1]);
    } else if(!strcmp(*argv, "-no_force_package_alignment")) {
      skip(1);
      no_force_package_alignment = 1;
    } else if(!strcmp(*argv, "-assert_on_error")) {
      skip(1);
      assert_on_error = 1;
    } else if(!strcmp(*argv, "-show_debug_string")) {
      skip(1);
      show_debug_string = 1;
    } else if(!strcmp(*argv, "-config2nd")) {
      require(2);
      config2nd = argv[1];
      skip(2);
    } else if(!arg_with_digit_value(&argc,&argv, "-com", &temp)) {
      comport = atol(temp);
      peripheral_boot_if = PER_BOOT_IF_UART; //UART
      if (comport < 1 || comport > 64) 
        die("Invalid com port %s", temp);
    } else if(!strcmp(*argv, "-usb")) {
      comport = 0;
      skip(1);
    } else if(!strcmp(*argv, "-parity")) { ///< Warning this option only change parity settings on host side
      require(2);
      if (find_named_value(parity_names, argv[1], &parity))
        die ("Invalid parity value %s", argv[1]);
      else
        host_logf_ex(parity != SECOND_PARITY_DEFAULT, "Parity is set to other than default make sure that 2nd is modified acordingly");
      parity_explicit_defined = 1;
      skip(2);
    } else if(!arg_with_unsigned_value(&argc,&argv, "-t", &timeout)) {
      if (timeout == 0) die ("Invalid timeout %s", argv[1]);
    } else if(!arg_with_unsigned_value(&argc,&argv, "-chunk", &chunkSize)) {
      if (chunkSize <= 10) die ("Invalid chunk size %s must be greater or equal to 10 MB", argv[1]);
      chunkSize *= MBYTE;
    } else if(!strcmp(*argv, "-baudrate")) {
      require(2);
      baudrate = atol(argv[1]);
      if (baudrate < 1) die ("Invalid baudrate %s", argv[1]);
      skip(2);
    } else if(!strcmp(*argv, "-p")) {
      require(2);
      product = argv[1];
      skip(2);
    } else if(!strcmp(*argv, "-c")) {
      require(2);
      cmdline = argv[1];
      skip(2);
    } else if(!strcmp(*argv, "-i")) {
      char *endptr = NULL;
      unsigned long val;

      require(2);
      val = strtoul(argv[1], &endptr, 0);
      if (!endptr || *endptr != '\0' || (val & ~0xffff))
        die("invalid vendor id '%s'", argv[1]);
      vendor_id = (unsigned short)val;
      skip(2);
    } else if(!strcmp(*argv, "getvar")) {
      require(2);
      fb_queue_display(argv[1], argv[1]);
      skip(2);        		
    } else if(!strcmp(*argv, "erase")) {
      if(!in_fastboot_mode)
      {
        require(2);
        {
          char *pname = argv[1];
          fb_queue_chip_driver(pname);
          fb_queue_confirmed_command4("erase", pname, "0", "0", "Erasing"); // LOE
          skip(2);
        }
      }
      else
      {
        require(2);
        fb_queue_erase(argv[1]);
        skip(2);
      }
    } else if(!strcmp(*argv, "chip_erase")) {
      require(3);
      {
        char *chip_name = argv[1];
        char *size = argv[2];
        char *offset = offset_and_chip_name(chip_name);
        fb_queue_chip_driver(chip_name);
        fb_queue_confirmed_command4("erase", chip_name, offset, size, "Erasing"); // LOE
        skip(3);
      }
    } else if(!strcmp(*argv, "signature")) {
      require(2);
      data = load_file(argv[1], &sz);
      if (data == 0) die("Could not load '%s'", argv[1]);
      if (sz != 256) die("Signature must be 256 bytes");
      fb_queue_download("signature", data, sz);
      fb_queue_command("signature", "Installing signature");
      skip(2);
    } else if(!strcmp(*argv, "reboot")) {
      if(!in_fastboot_mode)
      {
        oem_command("cold_sw_reset");
      }
      else
      {
        wants_reboot = 1;
      }
      skip(1);
    } else if(!strcmp(*argv, "reboot-bootloader")) {
      wants_reboot_bootloader = 1;
      skip(1);
    } else if(!strcmp(*argv, "boot")) {
      char *kname = 0;
      char *rname = 0;
      skip(1);
      if (argc > 0) {
        kname = argv[0];
        skip(1);
      }
      if (argc > 0) {
        rname = argv[0];
        skip(1);
      }
      data = load_bootable_image(kname, rname, &sz, cmdline);
      if (data == 0) goto error;
      fb_queue_download("boot.img", data, sz);
      fb_queue_command("boot", "booting");
    } else if(!strcmp(*argv, "chip_download")) {
      require(3);
      {
        char *chip_name = argv[1];
        char *fname = argv[2];
        char *offset = offset_and_chip_name(chip_name);
        char size[20]; int ret;
        char info[60];
        ret = getFileSize(fname);
        if(ret == -1) 
        {
          die("Cannot load '%s' for download\n", fname);
        }
        else if (ret == 0) 
        {
          die("Download file '%s' cannot be empty\n", fname);
        }

        sz = (unsigned) ret;
        sprintf(size, "%08X", sz);
        fb_queue_chip_driver(chip_name);
        fb_queue_command4("write", chip_name, offset, size, "Download file");
        sprintf(info, "%s@%s", chip_name, offset);
        fb_queue_download_packages(fname, info, chunkSizeMem, sz);
        skip(3);
      }
    } else if(!strcmp(*argv, "chip_upload")) {
      require(4);
      {
        char *chip_name = argv[1];
        char *size = argv[2];
        char *fname = argv[3];
        char *offset = offset_and_chip_name(chip_name);
        struct Action *a;
        sz = strtoul(size, &end, 16);
        if (*end) die("Bad size format '%s' expected hex value\n");
        fb_queue_chip_driver(chip_name);
        a = fb_queue_upload(chip_name, offset, sz, fname);
        skip(4);
      }
    } else if(!strcmp(*argv, "autoload_chip_driver")) { // query chip info and load dirver acordingly
      require(2);
      {
        char *chip_name = argv[1];
        fb_queue_chip_driver(chip_name);
        skip(2);
      }
    } else if(!strcmp(*argv, "chip_driver")) { // load specific driver (this command is normally generated by fb_queue_chip_driver/cb_query_chip_driver acordingly to omapflashdriver.txt)
      require(4);
      {
        char *chip_name = argv[1];
        char *fname = argv[2];
        char *config = argv[3];
        data = load_file(fname, &sz);
        if (data == 0) die("Cannot load driver from '%s'\n", fname);
        fb_queue_command3("driver", chip_name, config, "Download driver");
        fb_queue_download_packages(fname, NULL, data, sz);
        skip((config != NULL) ? 4 : 3);
      }
    } else if(!strcmp(*argv, "reconnect")) {
      require(1);
      skip(1);
      fb_queue_command("reconnect", "Reconnect - disconnect");
      skip(argc - do_command(argc, argv, 1, fb_queue_change_options));
      fb_queue_reconnect("Reconnect - connect");
    } else if(!strcmp(*argv, "flash")) {
      if(!in_fastboot_mode)
      {
        require(3);
        {
          char *pname = argv[1];
          char *fname = argv[2];
          char size[20];
          char info[60];
          int ret;

          if(!fname)
          {
            die("No file name given for 'flash' command");
          }

          ret = getFileSize(fname);
          if(ret == -1) 
          {
            die("Cannot load '%s' for download\n", fname);
          }
          else if (ret == 0) 
          {
            die("Download file '%s' cannot be empty\n", fname);
          }
          sz = (unsigned) ret;
          sprintf(size, "%08X", sz);
          fb_queue_chip_driver(pname);
          fb_queue_command4("write", pname, "0", size, "Download file");
          sprintf(info, "%s@0", pname);
          fb_queue_download_packages(fname, info, chunkSizeMem, sz);
          skip(3);
        }
      }
      else
      {
        char *pname = argv[1];
        char *fname = 0;
        require(2);
        if (argc > 2) {
          fname = argv[2];
          skip(3);
        } else {
          fname = find_item(pname, product);
          skip(2);
        }
        if (fname == 0) die("Cannot determine image filename for '%s'", pname);
        data = load_file(fname, &sz);
        if (data == 0) die("Cannot load '%s'\n", fname);
        fb_queue_flash(pname, data, sz);
      }
    } else if(!strcmp(*argv, "flash:raw")) {
      char *pname = argv[1];
      char *kname = argv[2];
      char *rname = 0;
      require(3);
      if(argc > 3) {
        rname = argv[3];
        skip(4);
      } else {
        skip(3);
      }
      data = load_bootable_image(kname, rname, &sz, cmdline);
      if (data == 0) die("Cannot load bootable image");
      fb_queue_flash(pname, data, sz);
    } else if(!strcmp(*argv, "flashall")) {
      skip(1);
      do_flashall();
      wants_reboot = 1;
    } else if(!strcmp(*argv, "update")) {
      if (argc > 1) {
        do_update(argv[1]);
        skip(2);
      } else {
        do_update("update.zip");
        skip(1);
      }
      wants_reboot = 1;
    } else if(!strcmp(*argv, "oem")) {
      if(!in_fastboot_mode)
      {
        require(2);
        if(!strcmp(argv[1], "format"))
        {
          fb_queue_format();
          skip(2);
        }
        else
        {
          die("Unknown native OEM command'%s'", argv[1]);
        }
      }
      else
      {
        require(1);
        argc = do_command(argc, argv, 0, oem_command);
      }
    } else if(!strcmp(*argv, "command")) {
      require(2);
      skip(1);
      skip(argc - do_command(argc, argv, 1, oem_command));
    } else if(!strcmp(*argv, "download")) {
      require(3);	
      {
        char *mem_addr = argv[1];
        char *fname = argv[2]; 	
        skip(3);
        data = load_file(fname, &sz);
        if (data == 0) die("cannot load '%s'\n", fname);
        fb_queue_download(fname, data, sz);
      }
    } else if(!strcmp(*argv, "mmcerase")) {
      require(3);
      {
        char *controller_no = argv[1];
        char *ptname = argv[2];
        skip(3);
        fb_queue_mmc_erase(controller_no, ptname);
      }
    } else if(!strcmp(*argv, "mmcwrite")) {
      require(4);
      {
        char *controller_no = argv[1];
        char *ptname = argv[2];
        char *fname = argv[3];
        data = load_file(fname, &sz);
        if (data == 0) die("cannot load '%s'\n", fname);
        skip(4);
        fb_queue_mmc_write(controller_no, ptname, data, sz);
      }
    } else if(!strcmp(*argv, "fastboot")) {
      in_fastboot_mode = 1;
      fb_queue_fastboot();
      skip(1);
    } else if(!strcmp(*argv, "system")) {
      require(2);
      skip(1);
      skip(argc - do_command(argc, argv, 1, fb_queue_syscmd));
    } else if(!strcmp(*argv, "extern_reset")) {
      require(2);
      skip(1);
      skip(argc - do_command(argc, argv, 1, set_extern_reset));
    } else if(!strcmp(*argv, "extern_power_on")) {
      require(2);
      skip(1);
      skip(argc - do_command(argc, argv, 1, set_extern_power_on));
    } else if(!strcmp(*argv, "extern_power_off")) {
      require(2);
      skip(1);
      skip(argc - do_command(argc, argv, 1, set_extern_power_off));
    } else if(!strcmp(*argv, "change_options")) {
      require(2);
      skip(1);
      skip(argc - do_command(argc, argv, 1, fb_queue_change_options));
    } else if(!strcmp(*argv, "-save2nd")) {
      require(2);
      save2ndfname = argv[1];
      save2ndFlag = TRUE;
      skip(2);
    } else if(!strcmp(*argv, "-asic_id")) {
      require(4);
      SNPRINTF (custAsicId, sizeof custAsicId, "%s %s %s", argv[1], argv[2], argv[3]);
      custAsicIdFlag = TRUE;
      skip(4);
    } else {
      die("Unknown option '%s'", *argv);
    }

    {
      char log_buffer[256];
      int n = 0;

      for (;prev_argv < argv; prev_argv++) {
        int r = SNPRINTF(log_buffer + n, sizeof log_buffer - n - 10, " %s", *prev_argv);
        if (r <= 0) {
          strcat(log_buffer,"...");
          break;
        }
        n += r;
      }
      host_logf_ex(VERBOSE, "%*s%s", verbose_arg_indent * 4, "", log_buffer + 1); ///< +1 for initial space
    }
  }

  script_file = save_script_file;
  script_line = save_script_line;
  return 0;

error:
  script_file = save_script_file;
  script_line = save_script_line;
  return 1;
}

int arg_lines(char *data, const char *file, int line, char *symbols[][2])
{
  if (line)
  {
    host_logf_ex(VERBOSE, "%*s""Entering parameter file:%s at line: %d", verbose_arg_indent * 4, "", file, line);
  }
  else
  {
    host_logf_ex(VERBOSE, "%*s""Entering parameter file:%s", verbose_arg_indent * 4, "", file);
  }
  verbose_arg_indent++;
  while (*data) {
    char *d;
    char *argv[MAX_FILE_ARGV];
    int argc = 0;
    int end_of_file = 0;

    while (*data) {
      while (*data && isspace(*data) && *data != '\n') data++;
      if (*data) {
        if (*data == 0) {
          break;
        }
        else if (*data == '#') {
          while (*data && *data++ != '\n');
          break;
        }
        else if (*data == '\n') {
          data++;
          break;
        }
        else if (*data == '"') {
          d = ++data;
          while (*d && *d != '"') d++;
          if (!*d) {
            host_logf("Error: Quote spans multiple lines on line %d of %s", line, file);
            return 1;
          }
        }
        else {
          d = data;
          while (*d && !isspace(*d)) d++;
        }
        if (*d) {
          char c = *d;
          *d++ = 0;
          if (c == '\n')
            break;
        }
        else {
          end_of_file = 1;
        }
        argv[argc++] = data;
        data = d;
        if (argc == MAX_FILE_ARGV) {
          host_logf("Error: To many arguments on line %d of %s", line, file);
          return 1;
        }
        if (end_of_file)
          break;
      }
      else {
        break;
      }
    }
    argv[argc] = 0;
    arg(argc, argv, symbols, file, line);
    line++;
  }

  verbose_arg_indent--;
  host_logf_ex(VERBOSE, "%*s""Leaving parameter file:%s", verbose_arg_indent * 4, "", file);
  return 0;
}

static int arg_file(const char *file, char *symbols[][2])
{
  unsigned size; 
  char *data = (char*)load_file_ex (file, &size, add_null);
  if (data)
  {
    return arg_lines(data, file, 1, symbols);
  }
  else
  {
    return 1;
  }
}

int main(int argc, char **argv)
{
  ///@todo -nologo or -ver commandline option
  int result; struct Action *begin;
  clock_handle start_time, end_time;
  getCurrentTime(&start_time);
  begin = fb_queue_nothing();

  output = stderr;

  skip(1); ///< skip program name

  //fprintf (stderr, "%s\n", getcwd(NULL, 0));

#ifdef _DEBUG
  host_logf("%s (%s) ENGINEERING DROP\n",OMAPFLASH_VERSION_STRING, __DATE__);
#endif
#ifdef NDEBUG
  host_logf("%s (%s)\n",OMAPFLASH_VERSION_STRING, __DATE__);
#endif

  if (argc == 0) {
    usage();
    end_log(); //<generate terminating \n
    return 0;
  }

  if (!strcmp(*argv, "devices")) {
    list_devices();
    end_log(); //<generate terminating \n
    return 0;
  }

  // allocate chunk size mem used by upload/download
  chunkSizeMem = (char *) malloc(sizeof(char) * chunkSize);

  result = arg(argc, argv, NULL, "<command-line>", 0);
  if (result) {
    end_log(); //<generate terminating \n
    free(chunkSizeMem);
    assert(!assert_on_error);
    return 1;
  }

  // TODO replace wants with priority in queue for proper handling of cmdline in omapflash2nd.txt
  if (wants_wipe) {
    fb_queue_erase("userdata");
    fb_queue_erase("cache");
  }
  if (wants_reboot) {
    fb_queue_reboot();
  } else if (wants_reboot_bootloader) {
    fb_queue_command("reboot-bootloader", "Rebooting into bootloader");
  }

  if(save2ndFlag)
  {
    int ret;
    if(!custAsicIdFlag)
    {
      host_logf("No AsicID info provided to generate 2nd file. Use -asicId option.");
      return -1;
    }
    if(save2ndFile(product, config2nd, begin, omap_device) != 0)
    {
      host_logf("Error generating 2nd file");
      return -1;
    }

    return 0;
  }

  if (wants_2nd) {
    usb = pheriphal_boot(product, config2nd, begin, omap_device);
    if (!usb) {
      end_log(); //<generate terminating \n
      assert(!assert_on_error);
      return 2;
    }
  } else {
    usb = open_device();
  }

  result = fb_execute_queue(usb);
  getCurrentTime(&end_time);
  report_elapsed_time(start_time, end_time);
  end_log(); //<generate terminating \n
  free(chunkSizeMem);
  return result ? 3 : 0;
}
