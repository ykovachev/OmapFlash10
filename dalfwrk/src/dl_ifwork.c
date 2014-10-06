/**
 * @file dl_ifwork.c
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

/*==== INCLUDES ==============================================================*/
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#define DL_IFWORK_PRIVATE
#include "types.h"
#include "mem.h"
#include "dl_ifwork.h"
#include "interrupt.h"

#ifdef INTERRUPT
/*==== PUBLIC FUNCTIONS ======================================================*/
extern U32 get_platform_interrupt_vector_base_address(void);
//extern void IRQ_Handler (void);

/**************************************************************************
*
* dal_interruptInit - initialize the interrupts.
*
* This function sets up all interrupts properly and disables them.
*
* RETURNS: N/A
*/

S32 dal_interrupt_init (void)
{
    int i;
    int sens;


    interrupt_init();  //must be performed dal_interrupt_set_sensitivity

    /* configure the 0-96 interrupts ... */
    for(i=0; i< g_max_interrupts; i++)
    {
        sens = DAL_INT_LEVEL;
        dal_interrupt_set_sensitivity (i, sens);
    }

    /*  ... initialize the user handler 'jump table'
    *  ... initialize the user exception handler 'jump table'
    *  ... initialize the user SWI handler
    */


    if ((dal_int_user_handlers = (T_DAL_INT_USER_HANDLER *)
         mem_alloc(sizeof(T_DAL_INT_USER_HANDLER) * g_max_interrupts)) == NULL)
    {

        return -1;
    }

    if ((dal_int_user_exc_handlers = (T_DAL_INT_USER_HANDLER *)
         mem_alloc(sizeof(T_DAL_INT_USER_HANDLER) * g_max_exceptions)) == NULL)
    {
        return -1;
    }

    memset(dal_int_user_handlers,0,(sizeof(T_DAL_INT_USER_HANDLER) * g_max_interrupts));
    memset(dal_int_user_exc_handlers,0,(sizeof(T_DAL_INT_USER_HANDLER) * g_max_exceptions));
    dal_int_swi_handler.handler = NULL;
    dal_int_swi_handler.data = NULL;

    /*  ... mask off all of 'em!  */
    dal_interrupt_mask_all();

    /*  ... enable interrupts globally ... all individual one's are still masked */
    dal_enable_interrupts();

    return(OMAPFLASH_SUCCESS);
}


/**************************************************************************
*
* dal_interruptDispatch - dispatches the user isr
*
* This function is called by the interrupt handler (actual assembly handler)
* after finding out the interrupt source. It calls the appropriate user
* interrupt service routiens, if registered
*
* RETURNS: return value from the isr / INT_NOT_SERVICED
*/
S32 dal_interrupt_dispatch ( U32 int_number)
{
    if(int_number > g_max_interrupts)
    {
        return(CSST_DAL_INT_NOT_SERVICED);
    }

    /*call the respective interrupt callback function*/
    /*depending on the int_number*/
    if (dal_int_user_handlers[int_number].handler != NULL)
    {
        return ((*dal_int_user_handlers[int_number].handler)
                (dal_int_user_handlers[int_number].data));
    }

    return CSST_DAL_INT_NOT_SERVICED;
}

/**************************************************************************
*
* dal_swiDispatch - dispatches the user swi
*
* This function is called by the swi handler (actual assembly handler)
* It calls the appropriate user swi handler, if registered
*
* RETURNS: N/A
*/
S32 dal_swi_dispatch (void)
{
    if((dal_int_swi_handler.handler) == NULL)
    {
        return CSST_DAL_INT_NOT_SERVICED;
    }
    /*call the swi handler*/
    return((*dal_int_swi_handler.handler) (dal_int_swi_handler.data));
}

/**************************************************************************
*
* dal_exceptionDispatch - dispatches the exception handler
*
* This function is called by the exception handler (actual assembly handler)
* It calls the appropriate user exception handler, if registered
*
* RETURNS: N/A
*/
S32 dal_exception_dispatch (U32 exc_number)
{
    if(exc_number>g_max_exceptions)
    {
        return CSST_DAL_INT_NOT_SERVICED;
    }
    return(*dal_int_user_exc_handlers[exc_number].handler)
    (dal_int_user_exc_handlers[exc_number].data);

}



/**************************************************************************
*
* dal_interrupt_add_handler - add the user interrupt handler.
*
* This function registers a handler for any of the interrupts. 'userData'
* will be passed to the handler when the interrupt occurs
*
* RETURNS: DAL_OK / DAL_ERROR
*/
S32 dal_interrupt_add_handler(
                              U32      int_num,
                              S32      (*handler) (void *),
                              void        *data)
{




    if(int_num > g_max_interrupts)
    {
        return(CSST_DAL_INT_ERROR_NUMBER);
    }


    /*  ... disable interrupts
    */
    dal_lock();//HEU 14/2 '08: Use dal_lock/unlock instead, since these are safe to use also within ISR context

    dal_int_user_handlers[int_num].handler = handler;
    dal_int_user_handlers[int_num].data = data;

    /*  ... enable the interrupt in the H/W INT handler
    */

    dal_interrupt_mask(int_num, MASK_CLEAR);


    /*  ... re-enable interrupts
    */
    dal_unlock();//HEU 14/2 '08: Use dal_lock/unlock instead, since these are safe to use also within ISR context

    return OMAPFLASH_SUCCESS;
}


/**************************************************************************
*
* dal_interruptRemoveHandler - remove a handler.
*
* This function removes a handler for any of the interrupts.
*
* RETURNS: DAL_OK / DAL_ERROR
*/
int dal_interrupt_remove_handler(U32 int_num)
{

    if(int_num > g_max_interrupts)
    {
        return(CSST_DAL_INT_ERROR_NUMBER);
    }

    /*  ... mask off the interrupt in the H/W INT handler
    */
    dal_interrupt_mask(int_num, MASK_SET);

    /*  ... disable interrupts
    */

    dal_lock();//HEU 14/2 '08: Use dal_lock/unlock instead, since these are safe to use also within ISR context

    dal_int_user_handlers[int_num].handler = NULL;
    dal_int_user_handlers[int_num].data = NULL;

    /*  ... re-enable interrupts
    */
    dal_unlock();//HEU 14/2 '08: Use dal_lock/unlock instead, since these are safe to use also within ISR context

    return OMAPFLASH_SUCCESS;
}

/**************************************************************************
*
* dal_get_exception_vectors - Get a list of the ARM exception vectors.
*
* RETURNS: DAL_OK / DAL_ERROR
*/

U32 dal_get_exception_vectors(T_EXCEPTION_VECTORS *vectors)
{
    U32 *interrupt_vector;
    U32 *interrupt_address;
    /*
    U32 *user_interrupt_vector;
    U32 *user_interrupt_address;
    */
    U32 i=0;

    //interrupt_vector = (U32*)interrupt_vector_base;
    interrupt_vector = (U32*)get_platform_interrupt_vector_base_address();
    memset(vectors,0,sizeof(T_EXCEPTION_VECTORS));

    for (i = 0;i < sizeof(T_EXCEPTION_VECTORS)/(2*sizeof(U32));i++)
    {
        if ((*interrupt_vector & 0xFFFFF000) == 0xE59FF000)  // 0xE59FFxxx : LDR PC xxx relative to current PC
        {

            // Get user specified interrupt vector
            interrupt_address = (U32 *)((*interrupt_vector & 0xFFF) + 8 + (U8 *)interrupt_vector);

            // Save user specified interrupt vector
            ((U32 *)vectors)[i] = *interrupt_vector;

            // Save user specified interrupt address
            ((U32 *)((U8 *)vectors+offsetof(struct exception_vectors, undef_instr_addr)))[i] = *interrupt_address;

        }

        else
            return VECTOR_TABLE_FORMAT_NOT_SUPPORTED;



        interrupt_vector++;
    }

    return OMAPFLASH_SUCCESS;
}


/**************************************************************************
*
* dal_set_exception_vectors - Set a list of the ARM exception vectors.
*
* RETURNS: DAL_OK / DAL_ERROR
*/
U32 dal_set_exception_vectors(T_EXCEPTION_VECTORS *vectors)
{

    U32 *interrupt_vector;
    U32 *interrupt_address;
    /*
    U32 *user_interrupt_vector;
    U32 *user_interrupt_address;
    */
    U32 i=1;

    interrupt_vector = (U32*)get_platform_interrupt_vector_base_address();

    for (i = 0;i < sizeof(T_EXCEPTION_VECTORS)/(2*sizeof(U32));i++)
    {
        if ((*interrupt_vector & 0xFFFFF000) == 0xE59FF000)  // 0xE59FFxxx : LDR PC xxx relative to current PC
        {


            // Restore user specified interrupt vector
            *interrupt_vector = ((U32 *)vectors)[i];

            // Get user specified interrupt vector
            interrupt_address = (U32 *)((*interrupt_vector & 0xFFF) + 8 + (U8 *)interrupt_vector);

            // Restore user specified interrupt address
            *interrupt_address = ((U32 *)((U8 *)vectors+offsetof(struct exception_vectors, undef_instr_addr)))[i];
        }

        else
            return VECTOR_TABLE_FORMAT_NOT_SUPPORTED;

        interrupt_vector++;
    }
    return OMAPFLASH_SUCCESS;

}



#ifndef SECOND_DNLD


S32 dal_interrupt_get_handler_address(
                                      U32 int_num,
                                      U32* handler_address
                                     )
{

    if(int_num > g_max_interrupts)
    {
        return(CSST_DAL_INT_ERROR_NUMBER);
    }

    *handler_address = (U32)(dal_int_user_handlers[int_num].handler);
    return OMAPFLASH_SUCCESS;
}

/**************************************************************************
*
* dal_interruptAddEXCHandler - add the user exception handler.
*
* This function registers a exception handler for the given exception
* condition
*
* RETURNS: DAL_OK / DAL_ERROR
*/
S32 dal_interrupt_add_exc_handler
(
 U8         exception,
 S32         (*handler) (void *),
 void        *data
)
{
    if (exception >= g_max_exceptions)
    {
        return CSST_DAL_ERROR_EXCEPTION_NUMBER;
    }

    dal_int_user_exc_handlers[exception].handler = handler;
    dal_int_user_exc_handlers[exception].data = data;

    return (OMAPFLASH_SUCCESS);
}

/**************************************************************************
*
* dal_interruptAddSWIHandler - add the swi handler.
*
* This function registers a handler for software interrupts (SWI, traps)
*
* RETURNS: DAL_OK / DAL_ERROR
*/
int dal_interruptAddSWIHandler
(
 S32         (*handler) (void *),
 void        *userData
)
{
    dal_int_swi_handler.handler = handler;
    dal_int_swi_handler.data = userData;

    return OMAPFLASH_SUCCESS;
}

#endif    /*#ifndef SECOND_DNLD*/


#endif //INTERRUPT
