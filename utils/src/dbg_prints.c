/**
 * @file dbg_prints.c
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
 * This File contains target debug api functions.
 * 
 */

/*==== DECLARATION CONTROL ==================================================*/

#ifndef CSST_DBG_MAIN_C
#define CSST_DBG_MAIN_C


/*==== INCLUDES ==============================================================*/

#include "flash_drv.h"
#include "csst_tgt.h"
#include "dbg_common.h"
#include <stdarg.h>
#include "dbg_prints.h"
#include "dbg_ex.h"
#include "disp.h"

/*==== PUBLIC FUNCTIONS ======================================================*/
#if defined SECOND_DNLD_BUILD
void dbg_update_flow_params(const C8 * file_name, const C8 * function_name,U32 line_num)
{
    //Do nothing for SECOND_DNLD_BUILD
}
S32 dbg_print_msg(U8 level, const char * format,...)
{
    //Do nothing for SECOND_DNLD_BUILD
    return 0;
}
#elif defined EMMC_DRV
void dbg_update_flow_params(const C8 * file_name, const C8 * function_name,U32 line_num)
{
    //Do nothing for SECOND_DNLD_BUILD
}
S32 dbg_print_msg(U8 level, const char * format,...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);
    ///@todo cleanup use of \r\n in dbg_print
    if (*format == '\r') ++format;
    if (*format == '\n') ++format;
    if (level == DBG_LEVEL_CRITICAL)
        //if (level != DBG_LEVEL_OFF) 
            dbg_vxprintf(VXPRINTF_NO_REQUIRE_HASH, format, arg_ptr);
    va_end(arg_ptr);
    return 0;
}
#else
#define MAX_TEMP_BUF_SIZE	255
#define FILE_NAME_SIZE	30
#define FUNC_NAME_SIZE	30

static T_DISP_MODULE_ID_INFO modinfo = {
    CSST_DBG_ID,
    "DEBUG ",
    "1.04",
    __DATE__};

static U8 dbg_initialised = FALSE;

void dbg_recv (U32  sdu_handle);
U32 dbg_send_message_to_host(U8 * message_string, U8 flow_flag);
S32 dbg_send_response(U8 req_message_type,U32 status);

U8 filename[FILE_NAME_SIZE];
U8 functionname[FUNC_NAME_SIZE];
U32 linenum;

/*-----------------------------------------------------------------------------
| Function    :void dbg_init()
+------------------------------------------------------------------------------
| Description :	This function is called by the main()
|				This function initialises the static variables
|				of the debug module, and also registers
the dbg_recv
|
| Parameters  : none
|
| Returns     : none
|
+-----------------------------------------------------------------------------*/

void dbg_init()
{
    U32 i;


    /*Initialise the Debug Global Array*/
    for(i=0;i<DBGTRACE_MAX_MODULES;i++)
    {
        DMPT[i].debug_level = DBG_LEVEL_CRITICAL;
        DMPT[i].debug_flow_enable = DBGTRACE_DISABLE;
        DMPT[i].debug_pkt_trace_enable = DBGTRACE_DISABLE;
        DMPT[i].debug_msg_destination = TRACER_DEST_DBGCONSOLE;
    }
    /*Register the Debug Message Handler*/
    disp_register_callback(&modinfo,dbg_recv);
    dbg_initialised = TRUE;
    /* Exported from dp_ex.h */
    return;
}


/*-----------------------------------------------------------------------------
| Function    :dbg_recv (U32  sdu_handle )
+------------------------------------------------------------------------------
| Description : This function will be registered to the Dispatcher Module.
|				The dispatcher will call this function, when the Module-ID of
|				the CMP-PDU is of CSST_DBG_ID type. This function checks the
|				Diagnostics message and further calls functions depending on
|				the message_type and Info field.
|
|
| Parameters  :sdu_handle a U32 value given to the SDU
|
| Returns     :Void
+----------------------------------------------------------------------------*/

void dbg_recv (U32  sdu_handle)
{

    T_DBGTRACE_REQ_HEADER  *dbg_parser_ptr;

    U16 length;

    if(VALIDATE_SDU_HANDLE(sdu_handle) == FALSE)
    {
        /* If passed SDU handle is not valid return with a debug print*/
        dbg_print(DBG_LEVEL_MAJOR,CSST_DBG_ID,"Invalid SDU Handle\n\r");
        return;
    }

    /* Get the pointer to the diagnostics message */
    if((dbg_parser_ptr = GET_DBG_HEADER_PTR(sdu_handle,&length)) == NULL)
    {
        dbg_print(DBG_LEVEL_INFO,CSST_DIAG_ID,"Invalid SDU Handle\n\r");
        return;
    }

    /* Proceed only if at least message header is present */
    if(length < sizeof(T_DBGTRACE_REQ_HEADER))  /* vishnu changed from >= */
    {
        sdu_dealloc(sdu_handle);
        return;
    }

    if((dbg_parser_ptr->module_id != DBGTRACE_ALLMODULES) && (dbg_parser_ptr->module_id > MAX_MODULES))
    {
        dbg_send_response(sdu_handle,CSST_DBGTRACE_ERROR_MODULE_ID);
        return;
    }

    switch(dbg_parser_ptr->message_type)
    {
    case DBGTRACE_SET_DEBUG_PARAMS_REQ:
        if(dbg_parser_ptr->dbg_info1 > DBGTRACE_MAX_LEVELS)
        {
            dbg_send_response(dbg_parser_ptr->message_type,CSST_DBGTRACE_ERROR_LEVEL_NUMBER);
            break;
        }
        if(dbg_parser_ptr->destn > DBGTRACE_MAX_DESTN)
        {
            dbg_send_response(dbg_parser_ptr->message_type,CSST_DBGTRACE_ERROR_DESTN_NUMBER);
            break;
        }
        if(dbg_parser_ptr->module_id == DBGTRACE_ALLMODULES)
        {
            U8 modno;

            for(modno=0; modno<DBGTRACE_MAX_MODULES; modno++)
            {
                DMPT[modno].debug_level = dbg_parser_ptr->dbg_info1;
                DMPT[modno].debug_msg_destination = dbg_parser_ptr->destn;
            }
        }
        else
        {
            /* vishnu changed dbg_parser_ptr->message_type to dbg_parser_ptr->module_id
            in the array index of DMPT */
            DMPT[dbg_parser_ptr->module_id].debug_level
            = dbg_parser_ptr->dbg_info1;
            DMPT[dbg_parser_ptr->module_id].debug_msg_destination
            = dbg_parser_ptr->destn;
        }
        dbg_send_response(dbg_parser_ptr->message_type,CSST_DBGTRACE_SUCCESS);
        break;
    case DBGTRACE_SET_FLOW_REQ:
        if(dbg_parser_ptr->module_id == DBGTRACE_ALLMODULES)
        {
            U8 modno;

            for(modno=0; modno<DBGTRACE_MAX_MODULES; modno++)
            {
                DMPT[modno].debug_flow_enable = dbg_parser_ptr->dbg_info1;
            }
        }
        else
        {
            DMPT[dbg_parser_ptr->module_id].debug_flow_enable
            = dbg_parser_ptr->dbg_info1;
        }
        dbg_send_response(dbg_parser_ptr->message_type,CSST_DBGTRACE_SUCCESS);
        break;
    case DBGTRACE_SET_TRACE_REQ:
        if(dbg_parser_ptr->module_id == DBGTRACE_ALLMODULES)
        {
            U8 modno;
            for(modno=0; modno<DBGTRACE_MAX_MODULES; modno++)
            {
                DMPT[modno].debug_pkt_trace_enable = dbg_parser_ptr->dbg_info1;
            }
        }
        else
        {
            DMPT[dbg_parser_ptr->module_id].debug_pkt_trace_enable
            = dbg_parser_ptr->dbg_info1;
        }
        dbg_send_response(dbg_parser_ptr->message_type,CSST_DBGTRACE_SUCCESS);
        break;
    }
    dbg_print(DBG_LEVEL_INFO,CSST_DBG_ID,"Exiting the dbg_recv function\n\r");
    sdu_dealloc(sdu_handle);
    return;
}

S32 dbg_pkt_trace(U8 module_id,U32 sdu_handle)
{
    U8 * buffer;
    U8 * tempbuffer;
    U8 * userdata;
    U32 strln;
    U32 i;
    U16 length = 0;

    if(dbg_initialised == FALSE)
        return CSST_DBGTRACE_SEND_FAIL;

    /*Check if the trace is enabled*/
    if(DMPT[module_id].debug_pkt_trace_enable == DBGTRACE_DISABLE)
    {
        return(CSST_DBGTRACE_SUCCESS);
    }

    userdata = sdu_get_user_data(sdu_handle, &length);
    buffer = (U8 *)malloc(length * 3);
    if (buffer == NULL)
    {
        return -1;
    }
    tempbuffer = buffer;
    for(i=0; i<length; i++)
    {
        strln = sprintf(tempbuffer,"%x ",userdata[i]);
		tempbuffer = tempbuffer + strln;
    }

    if(DMPT[module_id].debug_msg_destination == TRACER_DEST_CSST)
    {

        dbg_send_message_to_host(buffer,DBGTRACE_DISABLE);
    }
    else
    {
        console_printstr(buffer);
        console_printstr("\n\r");
    }
    free(buffer);
    return CSST_DBGTRACE_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    :S32 dbg_print_msg(U8 level,U8 Module ID,char * fmt,...)
+------------------------------------------------------------------------------
| Description :	This function can be called by any module.
|
|
| Parameters  :
|
| Returns     : OMAPFLASH_SUCCESS
|
+-----------------------------------------------------------------------------*/



//S32 dbg_print(U8 level,U8 module_id,const char * format,...)
S32 dbg_print_msg(U8 level,U8 module_id,const char * format,...)
{

    U8 *tempBuf;
    U8 *tempptr;
    va_list  args ;
    U32 debug_sdu_handle, flow_sdu_handle;
    U16 delta,delta_flow, sdu_length,flow_sdu_length;
    S16 len;

    if(dbg_initialised == FALSE)
        return CSST_DBGTRACE_SEND_FAIL;

    if(module_id > MAX_MODULES)
    {
        return CSST_DBGTRACE_ERROR_MODULE_ID;
    }

    if(DMPT[module_id].debug_flow_enable == DBGTRACE_ENABLE)
    {
        if(DMPT[module_id].debug_msg_destination == TRACER_DEST_CSST)
        {

            tempBuf = sdu_alloc(1,
                                DBGTRACE_MAX_MESSAGESIZE,&flow_sdu_handle);
            if(tempBuf==NULL)
            {
                return CSST_DBGTRACE_SEND_FAIL;
            }
            *tempBuf = DBGTRACE_MESSAGES;
            tempBuf++;


            len = sprintf(tempBuf,"%s, %s, %d\n\r",filename,functionname,linenum);

            delta_flow = DBGTRACE_MAX_MESSAGESIZE - (len+1+1);
            tempBuf = sdu_increase_space_after(flow_sdu_handle,delta_flow,&flow_sdu_length);
            disp_send(CSST_DBG_ID,flow_sdu_handle);
        }
        else
        {
            tempBuf = (U8 *)malloc (DBGTRACE_MAX_MESSAGESIZE);
            if (tempBuf == NULL)
            {
                return CSST_DBGTRACE_SEND_FAIL;
            }
            len = sprintf(tempBuf,"%s, %s, %d\n\r",filename,functionname,linenum);
            console_printstr(tempBuf);
            free (tempBuf);
        }
    }

    if(DMPT[module_id].debug_msg_destination == TRACER_DEST_CSST)
    {
        if(level <= DMPT[module_id].debug_level)
        {
            tempptr = sdu_alloc(1,
                                DBGTRACE_MAX_MESSAGESIZE,&debug_sdu_handle);
            if(tempptr == NULL)
            {
                return CSST_DBGTRACE_SEND_FAIL;
            }
            * tempptr = DBGTRACE_MESSAGES;
            tempptr++;

            va_start (args, format) ;

            len = vsnprintf (tempptr,(DBGTRACE_MAX_MESSAGESIZE-1), format, args) ;

            if (len < 0)
                return CSST_DBGTRACE_SEND_FAIL;

            tempptr[DBGTRACE_MAX_MESSAGESIZE-1-1] = 0; // -1 to compensate for tempptr++
            va_end(args);

            delta = DBGTRACE_MAX_MESSAGESIZE - (len+1+1);
            tempptr = sdu_increase_space_after(debug_sdu_handle,delta,&sdu_length);

            disp_send(CSST_DBG_ID,debug_sdu_handle);

        }
    }
    else
    {
        if(level <= DMPT[module_id].debug_level)
        {
            tempptr = (U8 *)malloc (DBGTRACE_MAX_MESSAGESIZE);
            if(tempptr == NULL)
            {
                return CSST_DBGTRACE_SEND_FAIL;
            }
            va_start (args, format) ;

            len = vsnprintf (tempptr,(DBGTRACE_MAX_MESSAGESIZE-1), format, args) ;

            if (len < 0)
                return CSST_DBGTRACE_SEND_FAIL;

            va_end(args);
            console_printstr(tempptr);
            free (tempptr);
        }
    }





#if 0
    if(level <= DMPT[module_id].debug_level)
    {
        tempBuf = (U8 *)malloc(DBGTRACE_MAX_MESSAGESIZE);
        if (tempBuf == NULL)
        {
            return -1;
        }
        tempptr = tempBuf;
        /* Init the variable argument list */
        va_start (args, format) ;
        /* Now just call vsnprintf to format our buffer */

        /*Use vsnprintf, since we dont want to
        spoil the memory/Stack*/
        vsnprintf (tempptr,(DBGTRACE_MAX_MESSAGESIZE-1), format, args) ;
        tempptr[DBGTRACE_MAX_MESSAGESIZE-1] = '\0';

        if(DMPT[module_id].debug_msg_destination == TRACER_DEST_CSST)
        {

            dbg_send_message_to_host(tempBuf,DMPT[module_id].debug_flow_enable);
        }
        else
        {
            if(DMPT[module_id].debug_flow_enable == DBGTRACE_ENABLE)
            {

                U8 line[6];
                console_printstr(filename);
                console_printstr(":");
                console_printstr(filename);
                console_printstr(":");
                ltoa(linenum,line);
                console_printstr(line);
                console_printstr("-");
                //printf("%s",dbgtrace_flow_string);
            }
            //printf("%s",tempBuf);
            console_printstr(tempBuf);
        }
        free(tempBuf);
        return CSST_DBGTRACE_SUCCESS;
    }
#endif
    return(OMAPFLASH_SUCCESS);

}





void dbg_update_flow_params(const C8 * file_name, const C8 * function_name,U32 line_num)
{

    strncpy(filename,file_name,FILE_NAME_SIZE);
    filename[FILE_NAME_SIZE-1] = '\0';
    strncpy(functionname,function_name,FUNC_NAME_SIZE);
    functionname[FUNC_NAME_SIZE-1] = '\0';
    linenum = line_num;

}

U32 dbg_send_message_to_host(U8 * message_string, U8 flow_flag)
{
    U16 length;
    U8 *buff_ptr;
    U32 cnf_sdu_handle = 0;
    U8 message_type = DBGTRACE_MESSAGES;
    S8 line[6];

    dbg_initialised = FALSE;

    length = strlen(message_string);
    length = length + sizeof(U8); /*for Message type*/
                                  /*If Flow is enabled, then Append the Flow Information*/

    if(flow_flag == DBGTRACE_ENABLE)
    {
        ltoa(linenum,line);
        length = length + strlen(filename) + strlen(functionname) + 3 + strlen(line);
    }

    if((buff_ptr = sdu_alloc(1,(length+1)
                             ,&cnf_sdu_handle)) == NULL)
    {
        //dbg_print(DBG_LEVEL_MAJOR,CSST_DBG_ID,"Could not allocate memory for the SDU\n\r");
        dbg_initialised = TRUE;
        return CSST_DBGTRACE_SEND_FAIL;
    }

    memcpy(buff_ptr,&message_type,sizeof(message_type));
    buff_ptr = buff_ptr + sizeof(message_type);

    if(flow_flag == DBGTRACE_ENABLE)
    {

        strcpy(buff_ptr,filename);
        buff_ptr = buff_ptr + strlen(filename);

        strcpy(buff_ptr,":");
        buff_ptr = buff_ptr + strlen(":");

        strcpy(buff_ptr,functionname);
        buff_ptr = buff_ptr + strlen(functionname);

        strcpy(buff_ptr,":");
        buff_ptr = buff_ptr + strlen(":");

        strcpy(buff_ptr,line);
        buff_ptr = buff_ptr + strlen(line);

        strcpy(buff_ptr,"-");
        buff_ptr = buff_ptr + strlen("-");

        /*we dont want the \0, hence -1 is put*/
    }

    strcpy(buff_ptr,message_string);
    disp_send(CSST_DBG_ID,cnf_sdu_handle);
    dbg_initialised = TRUE;
    return(CSST_DBGTRACE_SUCCESS);

}
/*-----------------------------------------------------------------------------
| Function    : U8 dbg_send_response( )
+------------------------------------------------------------------------------
| Description : Allocates memory for the response message depending on the size
|				and the type of the data.Send the final response to the host
|               via dispatcher from the sub modules of the diagnostics.
|
| Parameters  : sdu handle,status and Variable parameters
|
| Returns     :
+----------------------------------------------------------------------------*/

S32 dbg_send_response(U8 req_message_type,U32 status)
{
    U8 *buff_ptr;

    /*T_DBGTRACE_REQ_HEADER *dbg_data_ptr;*/
    U32  cnf_sdu_handle = 0;
    /*U16 length = 0;*/

    T_DBGTRACE_CNF_HEADER cnf_pkt;

    /* Allocate memory for the dbg response data */
    if((buff_ptr = sdu_alloc(DBG_EGRESS,sizeof(T_DBGTRACE_CNF_HEADER)
                             ,&cnf_sdu_handle)) == NULL)
    {
        dbg_print(DBG_LEVEL_MAJOR,CSST_DBG_ID,"Could not allocate memory for the SDU\n\r");
        return CSST_DBGTRACE_SEND_FAIL;
    }

    /* Switch to the message type of the received packet */
    switch(req_message_type)
    {

    case DBGTRACE_SET_DEBUG_PARAMS_REQ:
        cnf_pkt.message_type = DBGTRACE_SET_DEBUG_PARAMS_CNF;
        break;

    case DBGTRACE_SET_FLOW_REQ:
        cnf_pkt.message_type =  DBGTRACE_SET_FLOW_CNF;
        break;

    case DBGTRACE_SET_TRACE_REQ:
        cnf_pkt.message_type = DBGTRACE_SET_TRACE_CNF;
        break;

    default:
        cnf_pkt.message_type = DBGTRACE_TARGET_ERROR_IND;

    }

    cnf_pkt.status_code = status;

    memcpy(buff_ptr,&cnf_pkt,sizeof(cnf_pkt));

    disp_send(CSST_DBG_ID,cnf_sdu_handle);

    /* Exported from the dp_ex.c */
    /* The sdu handle is the one got during the sdu_malloc operation */

    dbg_print(DBG_LEVEL_INFO,CSST_DBG_ID,"Response sent successfully\n\r");
    return CSST_DBGTRACE_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    :S32 tgt_con_printf(char * fmt,...)
+------------------------------------------------------------------------------
| Description :	This function can be called by any module.
|
|
| Parameters  :
|
| Returns     : OMAPFLASH_SUCCESS
|
+-----------------------------------------------------------------------------*/
S32 tgt_con_printf(char * fmt,...) /*vishnu*/
{
    va_list ap;
    U8 tempBuf[DBGTRACE_MAX_MESSAGESIZE] ;

    va_start(ap, fmt);

    vsnprintf (tempBuf,(DBGTRACE_MAX_MESSAGESIZE-1), fmt, ap) ;

    va_end(ap);
    tempBuf[DBGTRACE_MAX_MESSAGESIZE-1] = '\0';

    console_printstr(tempBuf);
    return OMAPFLASH_SUCCESS;

}
/*-----------------------------------------------------------------------------
| Function    :S32 host_con_print(char * fmt,...)
+------------------------------------------------------------------------------
| Description :	This function can be called by any module.
|
|
| Parameters  :
|
| Returns     : OMAPFLASH_SUCCESS
|
+-----------------------------------------------------------------------------*/
S32 host_con_printf(char * fmt,...) /*vishnu*/
{
    va_list ap;
    U8 tempBuf[DBGTRACE_MAX_MESSAGESIZE] ;

    va_start(ap, fmt);

    vsnprintf (tempBuf,(DBGTRACE_MAX_MESSAGESIZE-1), fmt, ap) ;
    va_end(ap);

    tempBuf[DBGTRACE_MAX_MESSAGESIZE-1] = '\0';

    dbg_send_message_to_host(tempBuf, DBGTRACE_DISABLE);

    return OMAPFLASH_SUCCESS;
}

#endif

#endif //SECOND_DNLD_BUILD
