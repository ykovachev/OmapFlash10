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
#include <string.h>
#include <assert.h>
#ifdef _WIN32
#include <windows.h>
#include "util_windows.h"
#else
#include <unistd.h>
#include "glob.h"
#endif
#include "fastboot.h"
#include "output.h"
#include "board_configuration.h"

#ifdef _DEBUG
#define LOG_ID
#endif

#define USB_ASIC_ID_SIZE	0x51	// bytes
#define UART_ASIC_ID_SIZE	0x46	// bytes

#define PERIPHAL_BOOT_TIMEOUT (20*60)

/** @group AsicId types defining ASCII Id 
 * These comes from CSST for OMAP3
 * but have also been verified agains "omap4430 rom code memory and perioheral booting functional specification.pdf" Version 0.4
 */
///@{
//#pragma pack (show)
#pragma pack (push, 1)
#define ASIC_ID_ITEM_ID_id 0x01
typedef struct {
    /// 01h
    U8                        id;                       /*<  0:  1>*/
    /// 05h
    U8                        length;                   /*<  1:  1>*/
    /// 01h
    U8                        fixed;                    /*<  2:  1>*/
    /**
     * OMAP3430: {34h, 30h, 07h}
     * OMAP4430-Virtio: {44h, 30h, 0Fh}
    */
    U8                        omap_id[3];               /*<  3:  3>*/
    /// XXh
    U8                        version;                  /*<  6:  1>*/
} T_asic_id_id_sub_block;                               /*<  7    >*/

//typedef struct {
//    U8                        asic_id_id;               /*<  0:  1>*/
//    U8                        asic_id_length;           /*<  1:  1>*/
//    U8                        asic_id_fixed;            /*<  2:  1>*/
//    U8                        asic_id_data;             /*<  3:  1>*/
//} T_asic_id_secure_mode_sub_block;                      /*<  4    >*/
#define ASIC_ID_ITEM_ID_secure_mode 0x13
typedef struct {
    U8                        id;                       /*<  0:  1>*/
    /// 02h
    U8                        length;                   /*<  1:  1>*/
    U8                        fixed;                    /*<  2:  1>*/
    U8                        data[1];                  /*<  3:  1>*/
} T_asic_id_secure_mode_sub_block;                      /*<  4    >*/

#define ASIC_ID_ITEM_ID_public_id 0x12
typedef struct {
    U8                        id;                       /*<  0:  1>*/
    U8                        length;                   /*<  1:  1>*/
    U8                        fixed;                    /*<  2:  1>*/
    U8                        data[20];                 /*<  3: 20>*/
} T_asic_id_public_id_sub_block;                        /*< 23    >*/

#define ASIC_ID_ITEM_ID_root_key_hash 0x14
typedef struct {
    U8                        id;                       /*<  0:  1>*/
    U8                        length;                   /*<  1:  1>*/
    U8                        fixed;                    /*<  2:  1>*/
    U8                        data[32];                 /*<  3: 20/32 (omap3/omap4)> for omap4 last 12 bytes is zero for sha1 device*/
} T_asic_id_root_key_hash_sub_block;                    /*< 23/35    >*/

#define ASIC_ID_ITEM_ID_checksum 0x15
typedef struct {
    U8                        id;                       /*<  0:  1>*/
    U8                        length;                   /*<  1:  1>*/
    U8                        fixed;                    /*<  2:  1>*/
    U32                       public_chksum;            /*<  3:  4>*/
    U32                       secure_chksum;            /*<  7:  4>*/
} T_asic_id_checksum_sub_block;                         /*< 11    >*/

typedef struct
{
    /// 4 (UART) or 5 (USB)
    U8                        items;
    T_asic_id_id_sub_block    id;
    T_asic_id_secure_mode_sub_block secure_mode;
    T_asic_id_public_id_sub_block public_id;
    T_asic_id_root_key_hash_sub_block root_key_hash;
    T_asic_id_checksum_sub_block checksum;
} T_max_asic_id;                               

typedef struct
{
    /// 4 (UART) or 5 (USB)
    U8                        items;
    T_asic_id_id_sub_block    *id;
    T_asic_id_secure_mode_sub_block *secure_mode;
    T_asic_id_public_id_sub_block *public_id;
    T_asic_id_root_key_hash_sub_block *root_key_hash;
    T_asic_id_checksum_sub_block *checksum;
} T_actual_asic_id;                               


#pragma pack (pop)
///@}

static char error[250];

#define ASIC_ID_TUNE_IN_BYTES 32

static char * start_with (char *s, char *start) {
    while (*s != '\n' && *start) {
        if (isspace(*s) && isspace(*start)) {
            while (isspace(*s)) s++;
            while (isspace(*start)) start++;
        } else if (tolower(*s) == tolower(*start)) {
            s++;
            start++;
        } else {
            break;
        }
    }
    while (isspace(*start)) start++;
    if (*start){
        return 0;
    } else {
        while (isspace(*s)) s++;
        return s;
    }
}

char *pb_get_error()
{
    return error;
}

//char *pb_search_file(const char *config_file, char* front, char **cmdline, int *config2nd_line)
char *pb_search_file(char *config_data, const char *config_file, char* front, char **cmdline, int *config2nd_line)
{
    char *s = config_data;
    char *d;
    char *file = 0;
    int line = 0;
    //unsigned size; 
    char *data;

    //s = (char*)load_file_ex (config_file, &size, add_nl);
    if (s) {
        data = s;
        //while (size-- > 0) {
        while (*s) {
            if(*s == '\n') {
                line++;
                *s++ = 0;
                while (*data && *data < 128 && isspace(*data)) data++;
                if (*data != '#') {
                    data = start_with (data, front);
                    if (data) {
                        if (file) {
                            SNPRINTF (error, sizeof error, "Error multiple matches line %d and %d", line, *config2nd_line);
                            break;
                        }
                        d = s-1;
                        while (d > data && isspace(d[-1])) *--d = 0;
                        if (d == data){
                            SNPRINTF (error, sizeof error, "Error missing filename in line %d of %s", line, config_file);
                            break;
                        }
                        if (*data == '"') {
                            d = ++data;
                            while (*d && *d++ != '"');
                        }
                        else {
                            d = data;
                            while (*d && !isspace(*d)) d++;
                        }
                        if (cmdline) {
                            if (*d) {
                                *d++ = 0;
                                while (isspace(*d)) d++;
                                *cmdline = d;
                            }
                            else{
                                *cmdline = 0;
                            }
                        }
                        else {
                            *d = 0;
                        }
                        *config2nd_line = line;
                        file = data;
                    }
                }
                data = s;
            } else {
                s++;
            }
        }
    }    
    return file;
}

extern char custAsicId[100];
static char *find_2nd(const char *product, char *config_data, const char *config2nd_file, T_actual_asic_id *AsicId, char **cmdline, int *config2nd_line)
{
    int pos = 0;
    int quote = 0;
    char buf[250];
    char secure_device = 0;
    char *file;
    int i;

    if(AsicId != NULL)
    {
      for(i = 0; i < sizeof AsicId->public_id->data; i++)
        secure_device |= AsicId->public_id->data[i];
      //product omapid version HS/GP filename.2nd optional_initial_command
      SNPRINTF (buf, sizeof buf, "%s %02X%02X%02X %02X %s ",
        product, 
        AsicId->id->omap_id[0], AsicId->id->omap_id[1], AsicId->id->omap_id[2],
        AsicId->id->version,
        secure_device ? "HS" : "GP");
    }
    else
    {
      //product omapid version HS/GP filename.2nd optional_initial_command
      SNPRINTF (buf, sizeof buf, "%s %s ",
        product, custAsicId);
    }

    host_logf_ex(VERBOSE, "Searching for 2nd for: %s", buf);
    //file = pb_search_file(config2nd_file, buf, cmdline, config2nd_line);
    file = pb_search_file(config_data, config2nd_file, buf, cmdline, config2nd_line);
    if (file){
        return file;
    }
    else{
        host_logf("Error no 2nd for: %s%s", buf, error);
        return 0;
    }
}

void print_asic_id_item(U8 *data, char* name)
{
    U8 id = data[0];
    U8 length = data[1];
    int i;
    int n = 0;
    char buf[200] = {0};
    for (i = 0; i < length && n < sizeof buf - 5; i++)
    {
        if (i == 0 ///<length
            || i == 1 ///<fixed 
            //|| id == ASIC_ID_ITEM_ID_id && i == 5 ///<version
            )
            n += sprintf(buf + n, " ");
        n += sprintf(buf + n, "%02X ", data[i+2]);
    }
    host_logf_ex(VERBOSE, "AsicId %-10s\t%02X %02X %s", name, id, length, buf);
}

typedef struct 
{
    U8 *asic_buffer;
    int length;
    int items;
    int offset;
} check_asic_id_info;

enum check_asic_id_errors
{
    wrong_item_id = -1,
    item_length_to_big = -2
};

int check_asic_id_item(check_asic_id_info *info, void **item, char *name, int id, size_t size, BOOLEAN print)
{
    const size_t header_size = 2; ///< id and length one bytes each
    size_t item_length;
    size_t remaining_bytes = info->length - info->offset;
    if (info->items) { 
        if (remaining_bytes < header_size)
            return header_size - remaining_bytes; ///< missing length part
        else if (info->asic_buffer[info->offset] != id)
            return wrong_item_id;
        item_length = info->asic_buffer[info->offset + 1];
        if (item_length > size)
            return item_length_to_big;
        else if (remaining_bytes < header_size + item_length)
            return header_size + item_length - remaining_bytes;
        else if (print)
            print_asic_id_item(info->asic_buffer + info->offset, name); 
        *item = info->asic_buffer + info->offset; 
        info->offset += header_size + item_length; 
        info->items--;
    }
    return 0;
}

int check_asic_id(U8 *asic_buffer, int length, T_actual_asic_id *asic_id, BOOLEAN print)
{
    int items = asic_buffer[0];
    int missing;
    check_asic_id_info info = {asic_buffer, length, items, 1};
#define CHECK_ASIC_ID_ITEM(name) \
    if ((missing = check_asic_id_item(&info, (void**)&asic_id->name, #name, ASIC_ID_ITEM_ID_##name, sizeof(*asic_id->name), print)) > 0) \
        return missing

    asic_id->items = items;
    if (print)
        host_logf_ex(VERBOSE, "AsicId items %02X", asic_id->items);
    CHECK_ASIC_ID_ITEM(id);
    CHECK_ASIC_ID_ITEM(secure_mode);
    CHECK_ASIC_ID_ITEM(public_id);
    CHECK_ASIC_ID_ITEM(root_key_hash);
    CHECK_ASIC_ID_ITEM(checksum);
    return 0;
}

void do_extern_power_off(void)
{
    host_logf("Executing power_off '%s'", extern_power_off);
    system(extern_power_off);
}

void do_extern_power_on(void)
{
    host_logf("Executing power_on '%s'", extern_power_on);
    system(extern_power_on);
}

void do_extern_reset(void)
{
    host_logf("Executing extern_reset '%s'", extern_reset);
    system(extern_reset);
}

void do_sleepms(int mseconds, char *message)
{
    host_logf("Sleeping %d milliseconds %s", mseconds, message);
    sleepms(mseconds);
}

usb_handle *try_open_boot_device(void)
{
    return open_omap_device(comport,baudrate,EVENPARITY);
}

const int gaurd = 0xDEADBEEF;
U8 asic_buffer[sizeof (T_max_asic_id) + ASIC_ID_TUNE_IN_BYTES + sizeof(gaurd)];
const size_t asic_buffer_size = sizeof(asic_buffer) - sizeof(gaurd);

BOOLEAN empty_uart_buffer(omap_usb_handle *usb, char *message) 
{
    int r = -1;
    while (r!= 0) 
    {
        r = usb_read_raw(usb,asic_buffer,ASIC_ID_TUNE_IN_BYTES);
        if (r < 0) 
        {
            host_logf("Error during %s UART buffer clean-up", message);
            omap_usb_close(usb);
            return FALSE;
        }
    }       
    return TRUE;
}

omap_usb_handle *pheriphal_boot(const char *product, const char *config2nd_file, struct Action *begin, char *omap_device)
{
    int *const gaurded = (int*)(asic_buffer + asic_buffer_size);
    U8 size_command[4];
    T_actual_asic_id AsicId;
    int r = 0;
    int missing;
    size_t asic_id_start = 0;
    size_t asic_id_size = 0;
    size_t size;
    unsigned int asic_id_length = 0;
    char *data;
    char *context = "";
    char *file;
    omap_usb_handle *usb;
    char *cmdline;
    int config2nd_line;
    static const U8 getAsicId_message[4] = {0x03, 0x00, 0x03, 0xF0};
    static const U8 boot_message[4] = {0x02, 0x00, 0x03, 0xF0};
    ///@todo support change_device_mesage
    U8 change_device_message[4] = {0x06, 0x00, 0x03, 0xF0};
    ///@todo support next_device_mesage
    U8 next_device_message[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    ///@todo support memory_boot_mesage
    U8 memory_boot_mesage[4] = {0x12, 0x34, 0x56, 0x78};
    char *config_data = (char*)load_file_ex (config2nd_file, &size, add_nl);
    BOOLEAN retry_asic = TRUE;
    int number_of_tries = 0;
    BOOLEAN reset_message_printed = FALSE;
    BOOLEAN noice_message_printed = FALSE;	

    memset(asic_buffer, 0, asic_buffer_size);
    *gaurded = gaurd;

    if (!extern_power_on != !extern_power_off) 
    {
        die("You must specify either both or none of power_on and power_off");
    }

    if (get_comport())
    {
        host_logf_ex(VERBOSE, "Looking for device (omap com%d)", comport);
    }
    else
    {
        host_logf_ex(VERBOSE, "Looking for device (omap usb)");
    }

    if (extern_power_off) 
    {
        do_extern_power_off();
        do_sleepms(500, "while board powered off");
    }

    usb = try_open_boot_device();
    if (get_comport()) 
    {
        if (usb) 
        {
            if (extern_power_on) 
            {
                do_extern_power_on();
            }
            else 
            {
                host_logf("Please turn off device, then turn it on again");
            }
        }
        else
        {
            host_logf("Error during peripheral boot - can't open COM%d", get_comport());
            assert(!assert_on_error);
            return NULL;
        }
    }
    else {
        if (usb) {
            omap_usb_close (usb);
            if (!extern_power_off) 
            {
                host_logf("Please turn off device");
            }

            while (0 != (usb = try_open_boot_device())) 
            {
                omap_usb_close (usb);
            }
        }
        if (extern_power_on) 
        {
            do_extern_power_on();
        }
        else 
        {
            host_logf("Please turn on device");
        }
        usb = open_device();

        if (!usb)
        {
			switch (driverErr)
			{
				case DRV_ERR_WRONG_DRIVER:
					host_logf("Cannot open device. OMAPFlash driver was not found. Please install OMAPFlash driver");
					break;
				case DRV_ERR_OLD_DRIVER:
					host_logf("Cannot open device, old USB driver (CSSTUSB) detected.\n Please update OmapFlash USB driver "
						        "to the one in C:\\Program Files\\Texas Instruments\\OMAPFlash\\usb_drv_windows");
					break;
				default:
					host_logf("Cannot open device. An error occurs while initializing device, please try again");
			}
            assert(!assert_on_error);
            return NULL;
        }
    }

    for (;;) 
    {
        context = "receiving asic id head";
        if (!omap_device) 
        {
            wait_rx_data_ms(100);
            r = fb_data_receive(usb, asic_buffer, ASIC_ID_TUNE_IN_BYTES);
            if (r == 0) 
            { 
                if (get_comport()) 
                {
                    host_logf("Cannot auto detect device type (OMAP3/OMAP4/OMAP5)");
                    omap_usb_close(usb);
                    assert(!assert_on_error);
                    return NULL;
                }
                else 
                {
                    omap_device = "4";
                }
            }
        }
        
        if (r == 0)
        {
            if (!strcmp(omap_device, "4") || !strcmp(omap_device, "5")) 
            {
                context = "sending getAsicId message";
                //for (timeout_count = 0; timeout_count < PERIPHAL_BOOT_TIMEOUT * 10 && r == 0; timeout_count++)
                //if (number_of_tries != 0 || !get_comport()) 
                {
                    /// empty uart buffer before request
                    if (get_comport() && !empty_uart_buffer(usb, "pre ASIC id request"))
                    {
                        assert(!assert_on_error);
                        return NULL;
                    }
                    else
                    {
                        host_logf_ex(VERBOSE, "Requesting ASIC id");
                        /// for USB we have 3s to send this request
                        /// for UART we have 300ms (but we should have already got an ASIC id so we should not end up here, it is proposed that this is change in future versions of OMAP4 though)
                        r = fb_data_send_raw(usb, getAsicId_message, sizeof boot_message);
                        if (r < 0) 
                        {
                            assert(!assert_on_error);
                            goto error;
                        }
                    }
                }
            }
            else if (!strcmp(omap_device, "3"))
            {
                /// we have 300 ms to answer
                host_logf_ex(VERBOSE, "Awaiting ASIC id");
            }
            else
            {
                host_logf("Unknown OMAP device '%s' (not '3', '4' or '5')", omap_device);
                omap_usb_close(usb);
                assert(!assert_on_error);
                return NULL;
            }

            context = "receiving ASIC id";
            // strictly we don't need to wait this long 
            // but maybe the user is single stepping rom code in a debugger
            //      typical responce time: UART 100us
            sleepms(10);

           //r = fb_data_receive(usb, asic_buffer, ASIC_ID_TUNE_IN_BYTES);
            asic_id_length = (get_comport())? UART_ASIC_ID_SIZE : USB_ASIC_ID_SIZE;
        	  r = fb_data_receive(usb, asic_buffer, asic_id_length);
            if (r < 0) 
            {
                assert(!assert_on_error);
                goto error;
            }
        }

        if ((!strcmp(omap_device, "4") || !strcmp(omap_device, "5")) && r > 0 && r < ASIC_ID_TUNE_IN_BYTES && get_comport()) 
        {
            //int i;
            //for (i = 0; i < r; i++) {
            //    if (asic_buffer[i] != 0 && asic_buffer[i] != 0xff)
            //        retry_asic = FALSE;
            //}
            //if (retry_asic) {
                //host_logf("Wake-up board needing reset detected. Please reset board"); //TODO how to detect this
                //if (!noice_message_printed) {
                    noice_message_printed = TRUE;
                    host_logf_ex(VERBOSE, "UART power-up noise detected."); 
                //}
                //omap_usb_close(usb);
                //sleepms(100);
                //usb = try_open_boot_device();
                //r = 0;
            //}
            goto retry;
        }

        if (r == 0) 
        {
            host_logf("Reception failed");
            omap_usb_close(usb);
            assert(!assert_on_error);
            return NULL;
        }
        //else if (r == asic_id_length) NOTE: Not checking length, since it varies from OMAP generation to OMAP generation
        {
            /* Search for first byte of ASIC ID */
            asic_id_start = 0;
            while(asic_buffer[asic_id_start]!=0x5 && asic_buffer[asic_id_start]!=0x4 || 
                asic_buffer[asic_id_start+1]!=0x1 ||
                asic_buffer[asic_id_start+2]!=0x5) 
            {
                asic_id_start++;
                //if(asic_id_size < ASIC_ID_TUNE_IN_BYTES) {
                //    r = fb_data_receive(usb, asic_buffer + asic_id_size, ASIC_ID_TUNE_IN_BYTES);
                //    if (r < 0)
                //    {
                //        assert(!assert_on_error);
                //        goto error;
                //    }
                //    asic_id_size += r;
                //}
                //else 
                if(asic_id_start > ASIC_ID_TUNE_IN_BYTES) 
                {
                    if ((!strcmp(omap_device, "4") || !strcmp(omap_device, "5")) && get_comport()) 
                    {
                        /// empty uart buffer before reset
                        if (!empty_uart_buffer(usb, "pre-reset"))
                        {
                            assert(!assert_on_error);
                            return NULL;
                        }
                        else if (extern_reset) 
                        {
                            do_extern_reset();
                            do_sleepms(100, "waiting for board to power on after reset");
                            wait_rx_data_ms(100);
                            r = fb_data_receive(usb, asic_buffer, ASIC_ID_TUNE_IN_BYTES);
                            if (r < 0) 
                            {
                                assert(!assert_on_error);
                                goto error;
                            }
                        }
                        else if (!reset_message_printed) {
                            reset_message_printed = TRUE;
                            host_logf("Wake-up board needing reset detected. Please reset board");
                            wait_rx_data(PERIPHAL_BOOT_TIMEOUT);
                            r = fb_data_receive(usb, asic_buffer, ASIC_ID_TUNE_IN_BYTES);
                            if (r < 0) 
                            {
                                assert(!assert_on_error);
                                goto error;
                            }
                        }

                        goto retry;
                    }
                    else 
                    {
                        host_logf("Unable to read ASIC id. Searched %i bytes of data - no ASIC ID start", ASIC_ID_TUNE_IN_BYTES);
            #ifdef LOG_ID
                        {
                          int count;
                          host_logf("\nReceived %d characters", r);
                          for(count = 0; count < r; count++)
                          {
                            host_logf("%s%02x ", count % 16 ? "" : "\n", asic_buffer[count]);
                          }
                        }
            #endif
                        omap_usb_close(usb);
                        assert(!assert_on_error);
                        return NULL;
                    }
                }
            }
            /* End search for first byte of ASIC ID */

            //asic_id_size = ASIC_ID_TUNE_IN_BYTES - asic_id_start;
            asic_id_size = asic_id_length - asic_id_start;

            context = "receiving ASIC id tail";
            while ((missing = check_asic_id(asic_buffer + asic_id_start, asic_id_size, &AsicId, FALSE)) > 0) 
            {
                if(missing < 0 || asic_id_start + asic_id_size + missing > asic_buffer_size)
                {
                    host_logf("Malformed ASIC id", missing);
                    assert(*gaurded == gaurd);
                    omap_usb_close(usb);
                    assert(!assert_on_error);
                    return NULL;
                }

                wait_rx_data_ms(100);
                r = fb_data_receive(usb, asic_buffer + asic_id_start + asic_id_size, missing);
                if (r < 0) 
                {
                    assert(!assert_on_error);
                    goto error;
                }

                if (r == 0) 
                {
                    host_logf("Missing ASIC id tail (at least %d bytes)", missing);
                    assert(*gaurded == gaurd);
                    omap_usb_close(usb);
                    assert(!assert_on_error);
                    return NULL;
                }
                asic_id_size += r;
            }
            break;
        }

retry:
        r = 0;
        number_of_tries++;
        if (number_of_tries++ >= 3) 
        {
            host_logf("Reception failed - timeout after %d retries", number_of_tries);
            omap_usb_close(usb);
            assert(!assert_on_error);
            return NULL;
        }
    } // while (retry_asic)

    context = "printing ASIC id";
    r = check_asic_id(asic_buffer + asic_id_start, asic_id_size, &AsicId, TRUE);
    if (r < 0) 
    {
        assert(!assert_on_error);
        goto error;
    }

    context = "sending boot message";
    r = fb_data_send_raw(usb, boot_message, sizeof boot_message);
    if (r < 0) 
    {
        assert(!assert_on_error);
        goto error;
    }

    /********** load 2nd **********/
    file = find_2nd (product, config_data, config2nd_file, &AsicId, &cmdline, &config2nd_line);
    if (file == 0) {
        omap_usb_close(usb);
        assert(!assert_on_error);
        return NULL;
    }
    host_logf_ex(VERBOSE, "Loading second file %s", file);
    data = load_file_ex (file, &size, exact_size);
    if (!data) {
        omap_usb_close(usb);
        assert(!assert_on_error);
        return NULL;
    }

    if (cmdline) 
    {
        struct Action *last = fb_begin_injection(begin);
        r = arg_lines(cmdline, config2nd_file, config2nd_line, NULL);

        if (r) 
        {
            host_logf("Error during peripheral boot while executing after-2nd commands");
            assert(!assert_on_error);
            return NULL;
        }
        fb_end_injection(last);
    }

    // Create the size command (part of OMAP protocol!)
    // must be little endian for UART - doesn't matter for USB in current devices.
    // but it could matter for future devices.

    if(omap_configuration[0])
    {
      data = realloc(data, size + sizeof(omap_configuration));
      if(!data)
      {
        host_logf("Memory allocation error when appending OMAP configuration data.");
        omap_usb_close(usb);
        assert(!assert_on_error);
        return NULL;
      }
      memcpy(data + size, omap_configuration, sizeof(omap_configuration));
      size = size + sizeof(omap_configuration);
    }

    if((size % 64) == 0)
    {
      /* Fix for problem with modulo 64 size in peripherial boot download */
      data = realloc(data, size + 4);
      if(!data)
      {
        host_logf("Memory allocation error when adjusting second loader image size.");
        omap_usb_close(usb);
        assert(!assert_on_error);
        return NULL;
      }
      memset(data + size, 0, 4);
      size += 4;
    }
    
    size_command [0] = (U8) size;
    size_command [1] = (U8) (size >> 8);
    size_command [2] = (U8) (size >> 16);
    size_command [3] = (U8) (size >> 24);

    context = "sending sizeof 2nd";
    host_logf_ex(VERBOSE, "Sending size of second file (0x%02X%02X%02X%02X bytes)", size_command[3], size_command[2], size_command[1], size_command[0]);
    r = fb_data_send_raw(usb, size_command, sizeof size_command );
    if (r < 0) 
    {
        assert(!assert_on_error);
        goto error;
    }

    context = "sending 2nd";
    host_logf_ex(VERBOSE, "Transferring second file to target (0x%X bytes)", size);

    {
        r = fb_data_send_raw(usb, data, size);
        if (r < 0)
        { 
            assert(!assert_on_error);
            goto error;
        }
    } 

    // pheriphalboot_reopen is ignore for UART since that need parity setting change it make no sense to turn off reopen there
    if (peripheralboot_reopen || get_comport()) {
        //Let target have a chance to start up 2nd code and be ready for connection
        host_logf_ex(VERBOSE, "Closing boot connection");
        omap_usb_close(usb);

        if (!get_comport()) 
        {
            //give target a chance to detect that USB was closed
            //if this timeout is too low USB connection will fail in USB_Init rom api call
            sleep (2);
        }

        usb = open_device();
        //TODO change baudrate if neccesary
        if (!usb)
        {
            host_logf("Timeout reopening connection for USB");
            return NULL;
        }
    }
			
    context = "waiting for 2nd";
    host_logf_ex(VERBOSE, "Waiting for 2nd");
    //TODO verify that we do not need explicit timeout should be handled possible indirectly by fb_response()
    r = fb_response(usb, 0);	
    if (r < 0) 
    {
        assert(!assert_on_error);
        goto error;
    }
    host_logf_ex(VERBOSE, "Found 2nd");
    return usb;

error:
    host_logf("Error: (%s) during peripheral boot (%s)", fb_get_error(), context);
    //usb closed by function detecting error
    assert(!assert_on_error); ///<This should not happen all goto error should be preceeded by their own assert
    return NULL;
}

extern int save2ndFlag;
extern char *save2ndfname;
int save2ndFile(const char *product, const char *config2nd_file, struct Action *begin, char *omap_device)
{
  int *const gaurded = (int*)(asic_buffer + asic_buffer_size);
  int r = 0;
  size_t size;
  char *data;
  char *file;
  char *cmdline;
  int config2nd_line;
  char *config_data = (char*)load_file_ex (config2nd_file, &size, add_nl);

  /********** load 2nd **********/
  file = find_2nd(product, config_data, config2nd_file, NULL, &cmdline, &config2nd_line);
  if (file == 0) {
    host_logf("\nError reading 2nd file\n");
    return -1;
  }
  host_logf_ex(VERBOSE, "Loading second file %s", file);
  data = load_file_ex (file, &size, exact_size);
  if (!data) {
    host_logf("\nError Loading 2nd file\n");
    return -1;
  }

  if (cmdline) 
  { 
    struct Action *last = fb_begin_injection(begin);
    r = arg_lines(cmdline, config2nd_file, config2nd_line, NULL);

    if (r) 
    {
      host_logf("Error during pheriphal boot while executing after-2nd commands");
      return -1;
    }
    fb_end_injection(last);
  }

  // Create the size command (part of OMAP protocol!)
  // must be little endian for UART - doesn't matter for USB in current devices.
  // but it could matter for future devices.

  if(omap_configuration[0])
  {
    data = realloc(data, size + sizeof(omap_configuration));
    if(!data)
    {
      host_logf("Memory allocation error when appending OMAP configuration data.");
      return -1;
    }
    memcpy(data + size, omap_configuration, sizeof(omap_configuration));
    size = size + sizeof(omap_configuration);
  }

  if((size % 64) == 0)
  {
    /* Fix for problem with modulo 64 size in peripherial boot download */
    data = realloc(data, size + 4);
    if(!data)
    {
      host_logf("Memory allocation error when adjusting second loader image size.");
      return -1;
    }
    memset(data + size, 0, 4);
    size += 4;
  }

  if(save2ndFlag)
  {
    host_logf("Saving 2nd binary to a file");
    if(save_file(save2ndfname, data, size) == 0)
    {
      host_logf("\nSaving 2nd file to %s failed \n", save2ndfname);
      return -1;
    }
  }

  return 0;
}
