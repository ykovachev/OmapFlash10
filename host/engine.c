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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "fastboot.h"
#include "board_configuration.h"
#include "memory_partitions.h"
#include "output.h"

extern unsigned int renew_driver;
char last_chip_name[20] = "NO CHIP DRIVER";

char *mkmsg(const char *fmt, ...)
{
  char buf[256];
  char *s;
  va_list ap;

  va_start(ap, fmt);
  vsprintf(buf, fmt, ap);
  va_end(ap);

  s = strdup(buf);
  if (s == 0) die("Out of memory");
  return s;
}

#define OP_DOWNLOAD   1
#define OP_COMMAND    2
#define OP_QUERY      3
#define OP_NOTICE     4
#define OP_NOTHING    5
#define OP_DOWNLOAD_PACKAGES 6
#define OP_UPLOAD     7
#define OP_CONFIRMED_COMMAND 8
#define OP_FASTBOOT   9
#define OP_RECONNECT 10
#define OP_FORMAT    11

typedef struct Action Action;

struct Action 
{
  unsigned op;
  Action *next;

    char cmd[256];    
    void *data;
    unsigned size;
    void *data2;
    unsigned verbose;

    const char *msg;
    int (*func)(Action *a, int status, char *resp);

    const char *script_file;
    int script_line;
};

static Action *action_list = 0;
static Action *action_last = 0;

extern unsigned fastbootMode;

static char * lcase(char * in)
{
  char * c = in;
  while(*c)
  {
    if((*c >= 'A') && (*c <= 'Z'))
    {
      *c += 'a' - 'A';
    }
    c++;
  }
  return in;
}


static int cb_default(Action *a, int status, char *resp)
{
    if (status) {
        host_logf("Operation FAILED (%s)", resp);
    } else {
        if(fastbootMode)
            host_logf_ex(VERBOSE,"OKAY\n");
    }
    return status;
}

static Action *queue_action(unsigned op, const char *fmt, ...)
{
  Action *a;
  va_list ap;

    a = (Action *)calloc(1, sizeof(Action));
    if (a == 0) die("Out of memory");

    a->verbose = VERBOSE;
    va_start(ap, fmt);
    memset(a->cmd, 0, sizeof a->cmd);
    vsprintf(a->cmd, fmt, ap);
    va_end(ap);

    if (action_last) {
        a->next = action_last->next; // This is for steps injection
        action_last->next = a;
    } else {
        action_list = a;
    }
    action_last = a;
    a->op = op;
    a->func = cb_default;

    a->script_file = script_file;
    a->script_line = script_line;
    return a;
}

static int handle_formatting(usb_handle **usb, Action *a)
{
  unsigned char * ptbl = NULL;
  unsigned long   tsize = partition_get_android_table(partition_get_memory("-"), ptbl, "ptbl.bin");

  if(tsize)
  {
    int argc = 3, r;
    char *argv[3] = 
    {
      "flash",
      "-",
      "ptbl.bin"
    };

    Action * last = fb_begin_injection(a);
    fb_queue_chip_driver("-");
    fb_end_injection(last);

    last = fb_begin_injection(a);
    
    r = arg(argc, argv, NULL, NULL, NULL);
    if (r) 
    {
      host_logf("Error executing format command");
      return 1;
    }
   
    fb_queue_notice(mkmsg("Format completed for '%s'", partition_get_memory("-")));
    
    fb_end_injection(last);
    
    return 0;
  }

  return -1;
};


void fb_queue_format(void)
{
  Action *a;
  a = queue_action(OP_FORMAT, "Formatting");
  a->msg = mkmsg("Performing native format for Android");
  a->verbose = VERBOSE;
}


void fb_queue_erase(const char *ptn)
{
    Action *a;
    a = queue_action(OP_COMMAND, "erase:%s", ptn);
    a->msg = mkmsg("Erasing '%s'", ptn);
    a->verbose = VERBOSE;
}

Action *fb_begin_injection(Action *a)
{
  Action *last;
  if (!a) {
    a = action_last;
  }
  if (!a) {
    a = queue_action(OP_NOTHING, "");
  }
  last = action_last;
  if (last == a) {
    last = 0;
  }
  action_last = a;
  return last;
}

void fb_end_injection(Action *last)
{
  if (last) {
    action_last = last;
  }
}

static int cb_query_chip_driver(Action *a, int status, char *resp)
{

  static char *prev_resp;
  char *cmdline;
  char *file;
  char *omapflashdriver = "omapflashdriver.txt";
  char *omapflashdriver_data;
  int line;

  char *symbols[][2] = {
    {"chip_name", (char*)a->data},
    {NULL, NULL}
  };

  ///@TODO should we use previous driver if matching, is it safe to do as listed below?
  /*
  if (prev_resp && !strcmp(prev_resp, resp)) {
  return 0;
  }
  prev_resp = strdup(resp);
  */

  if (status) {
    //TODO error reporting style is not consistent e.g. Die versus here "[Error]" != "FAIL"
    host_logf("%s FAILED (%s)", a->cmd, resp);
    return status;
  }

  if (strncmp (resp, ": ", 2)) {
    host_logf("%s FAILED (%s) - bad response format", a->cmd, resp);
    return status;
  }
  resp += 2;

  host_logf("Searching driver for: %s", resp);
  omapflashdriver_data = (char*)load_file_ex (omapflashdriver, NULL, add_nl);
  file = pb_search_file(omapflashdriver_data, omapflashdriver, resp, &cmdline, &line);

  if (file){
    if (!strcmp(file, ";")) {
      if (cmdline) {
        Action *last = fb_begin_injection(a);
        int r = arg_lines(cmdline, omapflashdriver, line, symbols);

        if (r) {
          host_logf("Error during pheriphal boot executing driver commands");
          return 1;
        }
        fb_queue_notice("End loading driver");
        fb_end_injection(last);
      }
    }
    else {
      host_logf("Bad file format in %s on line %d, missing semicolon", omapflashdriver, line);
      return 1;
    }
  }
  else{
    host_logf("Error no driver for: %s %s", resp, pb_get_error());
    return 1;
  }

  return 0;
}

static int cb_download_driver(Action *a, int status, char *resp)
{
  if(board_config_read())
  {
    int i;

    char * memory = partition_get_memory(a->data);

    if(!memory)
    {
      memory = a->data;
    }

    if(strcmp(last_chip_name, memory) || renew_driver)
    {
      strcpy(last_chip_name, memory);
  
      for(i = 0; i < MAX_MEMORIES; i++)
      {
        if(memory_configuration[i][MEMORY_NAME] == NULL)
        {
          host_logf("Error no driver found in board configuration for '%s' ", a->data);
          return 1;
        }

        if(!strcmp(memory, memory_configuration[i][MEMORY_NAME]))
        {
          if(memory_configuration[i][MEMORY_DRIVER] != NULL)
          {
            int argc = 4;
            char *argv[4] = 
            {
              "chip_driver",
              memory,
              memory_configuration[i][MEMORY_DRIVER],
              memory_configuration[i][MEMORY_CONFIG] ? memory_configuration[i][MEMORY_CONFIG] : ""
            };
            Action * last = fb_begin_injection(a);
            int r = arg(argc, argv, NULL, NULL, NULL);
            if (r) 
            {
                host_logf("Error executing driver download command");
                return 1;
            }
            fb_queue_notice("End loading driver");
            fb_end_injection(last);
          }
          else
          {
            host_logf_ex(VERBOSE, "No driver required for '%s'", memory);
          }
          break;
        }
      }

      if(i == MAX_MEMORIES)
      {
        host_logf("Error no driver found in board configuration for '%s' ", memory);
        return 1;
      }

    }
    return 0;
  }

  host_logf("No board configuration - skipping lookup for driver for '%s' ", a->data);
  return 1;
}

void fb_queue_chip_driver(char *chip)
{
    //Action *a = queue_action(OP_QUERY, "short_query %s", chip);
    //a->func = cb_query_chip_driver;
    //a->data = chip;
    //a->msg = mkmsg("Query chip info for '%s'", chip);
    Action *a = queue_action(OP_NOTHING, "driver search for %s", chip);
    a->func = cb_download_driver;
    a->data = chip;
}

void fb_queue_flash(const char *ptn, void *data, unsigned sz)
{
  Action *a;

  a = queue_action(OP_DOWNLOAD, "");
  a->data = data;
  a->size = sz;
  a->msg = mkmsg("Sending '%s' (%d KB)", ptn, sz / 1024);

  a = queue_action(OP_COMMAND, "flash:%s", ptn);
  a->msg = mkmsg("Writing '%s'", ptn);
}

void fb_queue_mmc_erase(const char *cno, const char *ptn)
{
  Action *a;
  a = queue_action(OP_COMMAND, "mmcerase:%s:%s", cno, ptn);
  a->msg = mkmsg("erasing '%s:%s'", cno, ptn);
}

void fb_queue_mmc_write(const char *cno, const char *ptn, void *data, unsigned sz)
{
  Action *a;

  a = queue_action(OP_DOWNLOAD, "");
  a->data = data;
  a->size = sz;
  a->msg = mkmsg("sending '%s:%s' (%d KB)", cno, ptn, sz / 1024);

  a = queue_action(OP_COMMAND, "mmcwrite:%s:%s", cno, ptn);
  a->msg = mkmsg("writing '%s:%s'", cno, ptn);
}

void fb_queue_fastboot()
{
  Action *a = queue_action(OP_FASTBOOT, "");
}

static int cb_execute_syscmd(Action *a, int status, char *resp)
{
  system(a->data);
  return 0;
}

/// this is a t_do_command used as parameter to do_command
void fb_queue_syscmd(const char *syscmd)
{
  char *cmd;
  Action *a = queue_action(OP_NOTHING, "");
  a->func = cb_execute_syscmd;

  cmd = (char *)calloc(strlen(syscmd) + 2, sizeof(char));
  if (cmd == NULL) die("Out of memory");
  sprintf(cmd, "\"%s\"", syscmd);
  a->data = cmd;

  a->msg = mkmsg("Executing command '%s'", syscmd);
}

static int cb_execute_change_options(Action *a, int status, char *resp)
{
  return arg_lines(a->data, a->script_file, a->script_line, NULL);
}

/// this is a t_do_command used as parameter to do_command
void fb_queue_change_options(const char *options)
{
  if (*options)
  {
    Action *a = queue_action(OP_NOTHING, "");
    a->data = strdup(options);
    a->func = cb_execute_change_options;
    a->msg = mkmsg("Change options: %s", options);
  }
}

static int match(char *str, const char **value, unsigned count)
{
  unsigned n;

  for (n = 0; n < count; n++) {
    const char *val = value[n];
    int len = strlen(val);
    int match;

    if ((len > 1) && (val[len-1] == '*')) {
      len--;
      match = !strncmp(val, str, len);
    } else {
      match = !strcmp(val, str);
    }

    if (match) return 1;
  }

  return 0;
}


static int cb_check(Action *a, int status, char *resp, int invert)
{
  const char **value = a->data;
  unsigned count = a->size;
  unsigned n;
  int yes;

  if (status) {
    host_logf("Operation FAILED (%s)", resp);
    return status;
  }

  yes = match(resp, value, count);
  if (invert) yes = !yes;

  if (yes) {
    if(fastbootMode)
      fprintf(stderr,"OKAY\n");
    return 0;
  }

  host_logf("Operation FAILED");
  host_logf("Device %s is '%s'.", a->cmd + 7, resp);
  host_logf("Update %s '%s'", invert ? "rejects" : "requires", value[0]);
  for (n = 1; n < count; n++) {
    host_logf(" or '%s'", value[n]);
  }
  return -1;
}

static int cb_require(Action *a, int status, char *resp)
{
  return cb_check(a, status, resp, 0);
}

static int cb_reject(Action *a, int status, char *resp)
{
  return cb_check(a, status, resp, 1);
}

void fb_queue_require(const char *var, int invert, unsigned nvalues, const char **value)
{
  Action *a;
  a = queue_action(OP_QUERY, "getvar:%s", var);
  a->data = (void*)value;
  a->size = nvalues;
  a->msg = mkmsg("Checking %s", var);
  a->func = invert ? cb_reject : cb_require;
  if (a->data == 0) die("Out of memory");
}

static int cb_display(Action *a, int status, char *resp)
{
  if (status) {
    host_logf("%s FAILED (%s)", a->cmd, resp);
    return status;
  }
  host_logf("%s: %s", (char*) a->data, resp);
  return 0;
}

void fb_queue_display(const char *var, const char *prettyname)
{
  Action *a;
  a = queue_action(OP_QUERY, "getvar:%s", var);
  a->data = strdup(prettyname);
  if (a->data == 0) die("Out of memory");
  a->func = cb_display;
}

static int cb_do_nothing(Action *a, int status, char *resp)
{
  return 0;
}

void fb_queue_reboot(void)
{
  Action *a = queue_action(OP_COMMAND, "reboot");
  a->func = cb_do_nothing;
  a->msg = "Rebooting";
}

void fb_queue_command(const char *cmd, const char *msg)
{
  Action *a = queue_action(OP_COMMAND, lcase(cmd));
  a->msg = msg;
}

void fb_queue_command2(const char *cmd, const char *arg, const char *msg)
{
  Action *a = queue_action(OP_COMMAND, "%s %s", lcase(cmd), arg);
  a->msg = msg;
}

void fb_queue_command3(const char *cmd, const char *arg, const char *arg2, const char *msg)
{
  Action *a = queue_action(OP_COMMAND, "%s %s %s", lcase(cmd), arg, arg2);
  a->msg = msg;
}

void fb_queue_command4(const char *cmd, const char *arg, const char *arg2, const char *arg3, const char *msg)
{
  Action *a = queue_action(OP_COMMAND, "%s %s %s %s", lcase(cmd), arg, arg2, arg3);
  a->msg = msg;
}

void fb_queue_confirmed_command4(const char *cmd, const char *arg, const char *arg2, const char *arg3, const char *msg)
{
  Action *a = queue_action(OP_CONFIRMED_COMMAND, "%s %s %s %s", lcase(cmd), arg, arg2, arg3);
  a->msg = msg;
    a->verbose = VERBOSE;
}

void fb_queue_download(const char *name, void *data, unsigned size)
{
    Action *a  = queue_action(OP_DOWNLOAD, "");
    a->data    = data;
    a->size    = size;
    a->verbose = VERBOSE;
    a->msg     = mkmsg("Downloading '%s'", name);
}

void fb_queue_download_packages(const char *file, const char * info, void *data, unsigned size)
{
    Action *a  = queue_action(OP_DOWNLOAD_PACKAGES, "");
    a->data    = data;
    a->size    = size;
    a->verbose = VERBOSE;
    a->msg     = mkmsg("Downloading '%s' %c%s%c", file, info ? '(' : ' ', info ? info : "", info ? ')' : ' ');
	  a->script_file = mkmsg("%s", file);
}

int handle_reconnect_cmd(usb_handle **usb, Action *a, char *resp)
{
  //int status = fb_command(*usb, a->cmd);
  //status = a->func(a, status, resp);
  //if (status) 
  //{
  //    return status;
  //}
  //else
  //{
  omap_usb_close(*usb);
  // give target time to disconnect before we connect again.
  sleep(2);
  *usb = open_device();
  if (!*usb)
  {
    return -1;
  }
  else
  {
    return fb_response(usb, 0);
  }
  //}
}

void fb_queue_reconnect(const char *msg)
{
  Action *a = queue_action(OP_RECONNECT, "reconnect");
  a->msg = msg;
}

/*static int cb_query_upload(Action *a, int status, char *resp)
{
if (status > 0) {
Action *last = fb_begin_injection(a);
Action *upload = queue_action(OP_UPLOAD, "");
unsigned size = status;
char *data = (char*)malloc(size);
if (data == 0) die("cannot alloc memory for upload data\n");
upload->data = data;
upload->size = size;

a->data = data;
a->size = size;
a->msg = mkmsg("uploading '%s'", chip_name);
fb_end_injection(last);
return 0;
}*/

Action *fb_queue_upload(char *chip_name, char *offset, unsigned size, char *fname)
{
  Action *a = queue_action(OP_UPLOAD, "read %s %s %08X", chip_name, offset, size);
  a->size = size;
  a->data = fname;
  a->msg = mkmsg("Uploading '%s'", chip_name);
  a->verbose = VERBOSE;
  return a;
}

static int cb_query_save(Action *a, int status, char *resp)
{
    if (status) {
        host_logf("%s FAILED (%s)", a->cmd, resp);
        return status;
    }
    else {
        Action *action = (Action *)a->data;
        int size = save_file((char*)a->data2, (char*)action->data, action->size);
        if (size != action->size) {
            host_logf("%s OKAY (%s)", a->cmd, resp);
            return -1;
        }
        host_logf_ex(VERBOSE, "OKAY");
        return 0;
    }
}

void fb_queue_save(char *fname, Action *action)
{
  Action *a = queue_action(OP_NOTHING, "");
  a->func = cb_query_save;
  a->data2 = fname;
  a->data = action;
  a->msg = mkmsg("Saving '%s'", fname);
}

void fb_queue_notice(const char *notice)
{
  Action *a = queue_action(OP_NOTICE, "");
  a->data = (void*) notice;
}

struct Action *fb_queue_nothing(void)
{
  Action *a = queue_action(OP_NOTHING, "");
  return a;
}

usb_handle* handle_fb_cmd(usb_handle *usb)
{
  if(fastbootMode)
  {
    printf("Already in Fastboot Mode! \n");
    return usb;
  }
  fastbootMode = 1; 
  omap_usb_close(usb);
  // give some time before we connect again. Otherwise USB connection is flaky
  sleep(1);
  usb = open_device();
  printf("In Fastboot Mode.. \n");
  return usb;
}

int fb_execute_queue(usb_handle *usb)
{
  Action *a;
  char resp[FB_RESPONSE_SZ+1];
  int status = 0;

    a = action_list;

    for (a = action_list; a; a = a->next) {
        script_file = a->script_file;
        script_line = a->script_line;
        if (a->msg) {
            host_logf_ex(a->verbose, "%s",a->msg);
        }
        memset(resp, 0 , sizeof(resp));

        if (a->op == OP_DOWNLOAD) {
            status = fb_download_data(usb, a->data, a->size);
            status = a->func(a, status, status ? fb_get_error() : "");
            if (status) break;
        } else if (a->op == OP_DOWNLOAD_PACKAGES) {
			status = fb_download_data_packages(usb, a->data, a->size, a->script_file);
            status = a->func(a, status, status ? fb_get_error() : "");
            if (status) break;
        } else if (a->op == OP_UPLOAD) {
            status = fb_upload_data(usb, a->cmd, a->data, &a->size);
            status = a->func(a, status, status ? fb_get_error() : "");
            if (status) break;
        } else if (a->op == OP_COMMAND) {
            status = fb_command(usb, a->cmd);
            status = a->func(a, status, status ? fb_get_error() : "");
            if (status) break;
        } else if (a->op == OP_QUERY) {
            status = fb_command_response(usb, a->cmd, resp);
            status = a->func(a, status, status ? fb_get_error() : resp);
            if (status) break;
        } else if (a->op == OP_NOTICE) {
            status = a->func(a, status, status ? fb_get_error() : resp);
            host_logf_ex(VERBOSE, "%s",(char*)a->data);
        } else if (a->op == OP_NOTHING) {
            status = a->func(a, status, status ? fb_get_error() : resp);
        } else if (a->op == OP_CONFIRMED_COMMAND) {
            status = fb_command_response(usb, a->cmd, resp);
            status = a->func(a, status, status ? fb_get_error() : resp);
            if (status) break;
        } else if (a->op == OP_FASTBOOT) {
            usb = handle_fb_cmd(usb);
        } else if (a->op == OP_RECONNECT) {
            status = handle_reconnect_cmd(&usb, a, resp);
            if (status) break;
        } else if (a->op == OP_FORMAT) {
          status = handle_formatting(&usb, a);
          if (status) break;
        } else {
            die("Bogus action");
        }
    }
    return status;
}

void fb_dump_queue(void) //TODO add debug option that course this function to be called
{
  Action *a;

  host_logf("-BEGIN-----------------------");
  for (a = action_list; a; a = a->next) {
    host_logf("%s - %s...", a->cmd, a->msg);
  }
  host_logf("-END-------------------------");
}
