/**
 * @file interrupt.c
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
#include <string.h>
#include <stdio.h>
#include "csst_tgt.h"
#include "interrupt.h"
#include "dl_ifwork.h"
#include "dl_ifwork_ex.h"
#include "silicon.h"

#ifdef INTERRUPT

U32 get_platform_interrupt_vector_base_address();
const U8 g_max_exceptions = MAX_NUMBER_OF_EXCEPTIONS;
const U8 g_max_interrupts = MAX_NUMBER_OF_INTERRUPTS;
#define DAL_INT_MASK_ALL 0xFFFFFFFF

#ifdef SDRAM_IVT
/* Check if we are in second download build mode.. - needs to be included in -d
 * WARNING: TO CHANGE THIS -ALIGN WITH cmeb.s and interrupt.c
 */
extern const unsigned int *INTREL_MEM_REL; /* Defined in Linker File */
#define INTERRUPT_VECTOR_BASE_ADDR ((U32*)&INTREL_MEM_REL)

#else

/* use SRAM base as vector location */
#define INTERRUPT_VECTOR_BASE_ADDR 0x4020FFC8

#endif /* #ifndef SECOND_DNLD_BUILD */

/*==== PUBLIC FUNCTIONS ======================================================*/

extern void IRQ_Handler (void);
extern U32 INTVECTS_PHY_LOC;
S32 dal_interrupt_mask_set (U32 bank, U32 new_mask);
U32 read_cpsr(void);
U32 dal_interrupt_controller_base_address;
U32 dal_interrupt_bank_base_adddress[3];

/**************************************************************************
*
* dal_interruptInit - initialize the interrupts.
*
* This function sets up all interrupts properly and disables them.
*
* RETURNS: N/A
*/

S32 interrupt_init (void)
{
  /*get the virtual address of the interrupt_controller*/
  dal_interrupt_controller_base_address = INTC_BASE;
  dal_interrupt_bank_base_adddress[0] = 
                dal_interrupt_controller_base_address + INTC_BANK0_BASE_OFFSET;

  dal_interrupt_bank_base_adddress[1] = 
                dal_interrupt_controller_base_address + INTC_BANK1_BASE_OFFSET;

  dal_interrupt_bank_base_adddress[2] = 
                dal_interrupt_controller_base_address + INTC_BANK2_BASE_OFFSET;

  return(OMAPFLASH_SUCCESS);
}

/**************************************************************************
*
* dal_interruptReadInputSource - reads the interrupt input register
*
* This function reads the interrupt input register
*
* RETURNS: ITRn value
*/
S32 dal_interrupt_read_input_source (U8 bank, U32 * int_source)
{
    if(bank > (MAX_NUMBER_OF_BANKS - 1))
  {
    return(CSST_DAL_INT_ERROR_BANK_NUMBER);
  }

    *int_source = in_regl((dal_interrupt_bank_base_adddress[bank] + INTC_ITR_OFFSET));

    return (OMAPFLASH_SUCCESS);
}


/**************************************************************************
*
* dal_interruptReadMask - reads the interrupt mask register
*
* This function reads the interrupt mask register
*
* RETURNS: content of MIRn register
*/
S32 dal_interrupt_read_mask (U8 bank, U32 * mask)
{

  if(bank > (MAX_NUMBER_OF_BANKS - 1))
  {
    return(CSST_DAL_INT_ERROR_BANK_NUMBER);
  }

  *mask = in_regl((dal_interrupt_bank_base_adddress[bank] + INTC_MIR_OFFSET));

  return (OMAPFLASH_SUCCESS);
}


/**************************************************************************
*
* dal_interruptMaskAll - masks all the interrupts.
*
* This function masks all the interrupts, by writing into all the banks.
*
* RETURNS: N/A
*/
void dal_interrupt_mask_all(void)
{
    dal_interrupt_mask_set(0,DAL_INT_MASK_ALL);
    dal_interrupt_mask_set(1,DAL_INT_MASK_ALL);
    dal_interrupt_mask_set(2,DAL_INT_MASK_ALL);
}


/**************************************************************************
*
* dal_interruptMaskSet - writes into the Mask Set Register.
*
* This function masks the interrupt, by writing the mask value into the
* Mask Set Register of the bank number passed.
*
* RETURNS: N/A
*/
S32 dal_interrupt_mask(U32 int_num, U8 set_flag)
{
  
  U8 bank;
  U32 new_mask = 0x00000001;
  if(int_num > MAX_NUMBER_OF_INTERRUPTS)
  {
    return(CSST_DAL_INT_ERROR_NUMBER);
  }

  /*The first 32 interrupts are of bank0 the 
  next 32 interrupts are for
  bank1 and the rest bank2, so derieve the bank number*/
  bank = (U8)int_num / 32;
  int_num -= bank*32;
  
  new_mask = new_mask << int_num;
  if(set_flag)
  {
    out_regl((dal_interrupt_bank_base_adddress[bank] + INTC_MIR_SET_OFFSET), new_mask);
  }
  else
  {

    out_regl((dal_interrupt_bank_base_adddress[bank] + INTC_MIR_CLEAR_OFFSET), new_mask);
  }
      
  return(OMAPFLASH_SUCCESS);
}

/**************************************************************************
*
* dal_interruptMaskSet - writes into the Mask Set Register.
*
* This function masks the interrupt, by writing the mask value into the
* Mask Set Register of the bank number passed.
*
* RETURNS: N/A
*/
S32 dal_interrupt_mask_set (U32 bank, U32 new_mask)
{
    if(bank > (MAX_NUMBER_OF_BANKS - 1))
  {
    return(CSST_DAL_INT_ERROR_BANK_NUMBER);
  }

    out_regl((dal_interrupt_bank_base_adddress[bank] + INTC_MIR_SET_OFFSET), new_mask);
  
  return(OMAPFLASH_SUCCESS);
}



/**************************************************************************
*
* dal_interruptReadIRQSource - reads the binary-coded source IRQ register
*
* This function reads the binary-coded source IRQ register
*
* RETURNS: content of SIR_IRQ register
*/
S32 dal_interrupt_read_irq_source (U32 * irq_source)
{
  *irq_source = in_regl((dal_interrupt_controller_base_address
                             + INTC_SIR_IRQ_OFFSET));

    return(OMAPFLASH_SUCCESS);
}

/**************************************************************************
*
* dal_interrupt_read_fiq_Source - reads the binary-coded source FIQ register
*
* This function reads the binary-coded source FIQ register
*
* RETURNS: content of SIR_FIQ register
*/
U32 dal_interrupt_read_fiq_source (U32 * fiq_source)
{
  *fiq_source = in_regl((dal_interrupt_controller_base_address
                            + INTC_SIR_FIQ_OFFSET));

    return(OMAPFLASH_SUCCESS);
}

/**************************************************************************
*
* dal_interrupt_get_priority - reads the priority of the interrupt
*
* This function reads reads the priority of the interrupt
*
* RETURNS: content of appropriate ILR register register
*/
S32 dal_interrupt_get_priority (U32 int_num, U32 * priority)
{
    U32 ilr;
    int bank;
    

    if(int_num > MAX_NUMBER_OF_INTERRUPTS)
    {
      return(CSST_DAL_INT_ERROR_NUMBER);
    }
    
    /*The first 32 interrupts are of bank0 tne next 32 interrupts are for
    bank1 and the rest bank2, so derieve the bank number*/
    bank = int_num / 32;
    int_num -= bank*32;
  
    switch (bank)
    {
        case 0:
            ilr = INTC_BANK0_ILR;
            break;
        case 1:
            ilr = INTC_BANK1_ILR;
              break;
        case 2:
            ilr = INTC_BANK2_ILR;
          break;
      default:
        return(CSST_DAL_INT_ERROR_BANK_NUMBER);
    }

    /* 
     * In the required bank, go to the ILR that corresponds to the interrupt number 
     * Every ILR is 4 bytes apart since ilr is an integer pointer moving it by the
     * interrupt number will move to the correct place.
     */
    ilr += int_num; 
        
  *priority = in_regl((dal_interrupt_controller_base_address+ilr));		
    /* Move the priority bits to the LSB position */
    *priority >>= DAL_INT_ILR_PRIORITY_BIT_POSITION; 
    return(OMAPFLASH_SUCCESS);
}

/**************************************************************************
*
* dal_interruptGetSensitivity - empty function
*
* In 24xx there is only one interrupt sensitivity, this functin is a 
* dummy function implemented for compatibility with legacy software.
*
* RETURNS: 1 always
*/
U32 dal_interrupt_get_sensitivity (U32 int_num, U8 * sensitivity)
{
  * sensitivity = 1; /*Level sensitive by default*/
    return(OMAPFLASH_SUCCESS);    
}


/**************************************************************************
*
* dal_interruptGetRouting - retruns the interrupt routing
*
* This function reads the interrupt level register and returns the binary
* coded value indicating whether the interrupt is routed to IRQ or FIQ.
*
* RETURNS: interrupt routing value
*/
S32 dal_interrupt_get_routing (U32 int_num, U32 * routing)
{
    U32 ilr;
    U8 bank;

    if(int_num >MAX_NUMBER_OF_INTERRUPTS)
    {
      return(CSST_DAL_INT_ERROR_NUMBER);
    }

    
    bank = (U8)int_num / 32;
    int_num -= bank*32;
    
    switch (bank)
    {
      case 0:
            ilr = INTC_BANK0_ILR;
            break;
          case 1:
              ilr = INTC_BANK1_ILR;
              break;			    		
        case 2:
            ilr = INTC_BANK2_ILR;
            break;
    }

    /* 
     * In the required bank, go to the ILR that corresponds to the interrupt number 
     * Every ILR is 4 bytes apart since ilr is an integer pointer moving it by the
     * interrupt number will move to the correct place.
     */    	
    ilr += int_num; 
    

    *routing = in_regl((dal_interrupt_controller_base_address+ilr));		
    /* make all the other bits 0 */
    *routing &= DAL_INT_ROUTING_BIT_POSITION; 
    return(OMAPFLASH_SUCCESS);
}

/**************************************************************************
*
* dal_interruptSetPriority - sets the priority for the interrupt
*
* This function confiugres the priority for the interrupt number int_num
*
* RETURNS: N/A
*/
S32 dal_interrupt_set_priority(U32 int_num, U32 priority)
{
    U32 ilr;
    U8 bank;
    U32 reg_val;

    if(int_num > MAX_NUMBER_OF_INTERRUPTS)
    {
      return(CSST_DAL_INT_ERROR_NUMBER);
    }


    bank = (U8)int_num / 32;
    int_num -= bank*32;
    
    switch (bank)
    {
        case 0:
            ilr = INTC_BANK0_ILR;
            break;
          case 1:
              ilr = INTC_BANK1_ILR;
              break;			    		
        case 2:
            ilr = INTC_BANK2_ILR;
            break;
      default:
        return(CSST_DAL_INT_ERROR_BANK_NUMBER);
        }

    /* 
     * In the required bank, go to the ILR that corresponds to the interrupt number 
     * Every ILR is 4 bytes apart since ilr is an integer pointer moving it by the
     * interrupt number will move to the correct place.
     */    	
    ilr += int_num; 
  ilr += dal_interrupt_controller_base_address;
    
    reg_val = in_regl(ilr);

    /* make the priority bits as 0 */
    reg_val &= DAL_INT_PRIORITY_BITS_MASK; 
    /* place the priority in the proper place */
    reg_val |= (priority << DAL_INT_ILR_PRIORITY_BIT_POSITION); 

  out_regl(ilr,reg_val);
    return(OMAPFLASH_SUCCESS);    
}

/**************************************************************************
*
* dal_interruptSetSensitivity - empty function
*
* In 24xx architecture all the interrutps are level sensitive. This function
* is implemented for compatibility with legacy software.
*
* RETURNS: N/A
*/ 
S32 dal_interrupt_set_sensitivity(U32      int_num,
                                    U32      sensitivity)
{
    return(OMAPFLASH_SUCCESS);
}


/**************************************************************************
*
* dal_interruptSetRouting - routes the interrupt to IRQ or FIQ.
*
* This function routes the interrupt represented by int_num to IRQ or FIQ
* based on 'routing'. If 'routing' is 0 route for IRQ, if it is 1
* route to FIQ.
*
* RETURNS: N/A
*/ 
S32 dal_interrupt_set_routing(U32 int_num, U32 routing)
{
  U32 ilr;
  U8 bank;
  U32 reg_val;

  if(int_num > MAX_NUMBER_OF_INTERRUPTS)
  {
    return(CSST_DAL_INT_ERROR_NUMBER);
  }


  bank = (U8)int_num / 32;
  int_num -= bank*32;
    
  switch (bank)
  {
    case 0:
        ilr = INTC_BANK0_ILR;
        break;
        case 1:
          ilr = INTC_BANK1_ILR;
            break;			    		
    case 2:
        ilr = INTC_BANK2_ILR;
        break;
    default:
      return(CSST_DAL_INT_ERROR_BANK_NUMBER);
  }

    /* 
     * In the required bank, go to the ILR that corresponds to the interrupt number 
     * Every ILR is 4 bytes apart since ilr is an integer pointer moving it by the
     * interrupt number will move to the correct place.
     */    	
    ilr += int_num; 
  ilr += dal_interrupt_controller_base_address;	

    reg_val = in_regl(ilr);
    /* make all the other bits 0 */
    reg_val &= DAL_INT_ROUTING_MASK; 
    reg_val |= (routing & DAL_INT_ROUTING_BIT_POSITION);
    out_regl(ilr,reg_val);
  return(OMAPFLASH_SUCCESS);
}
/* end of file interrupt.c */    

#endif //INTERRUPT 

void dal_enable_interrupts(void)
{
#ifndef SECOND_DNLD_BUILD  
  dal_fiq_enable ();
#endif  
  dal_irq_enable ();
}

void dal_disable_interrupts(void)
{
#ifndef SECOND_DNLD_BUILD  
  dal_fiq_disable ();
#endif
  dal_irq_disable ();
}

#ifdef INTERRUPT
#ifndef SECOND_DNLD_BUILD
/*-----------------------------------------------------------------------------
| Function    : dal_EX
+------------------------------------------------------------------------------
| Description : function to catch Exceptions and displays on 2 line LCD 
|                NOTE: this works only if the gpmc_init is complete.
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
static const char *expection_array[EXCEPTION_COUNT] =
{
    "Undef",
    "SWI",
    "DAbort",
    "PreF"
};
void dal_EX(U32 abort_id)
{
    volatile unsigned int i=1;
    volatile unsigned int k=0;
    U8 display_string[50];
    strcpy ((char *)display_string,"*EXP*: ");
    strcat ((char *)display_string,expection_array[abort_id - 1]);
    while(i)
    {
        chardisp_string_show(display_string);
        /* a short delay.. cant use timer..*/
        k=0x5000;
        while (k) 
        {
            k--;
        }
        chardisp_string_show((U8 *)"-+-+-+-+-+-+-+");
        k=0x100;
        while (k) 
        {
            k--;
        }
        i++;
    }
}
#endif

/*-----------------------------------------------------------------------------
| Function    : dal_IRQ_Hndl
+------------------------------------------------------------------------------
| Description : Interrupt handler routine (1st level)
|               This function identifies the interrupt source and dispatches
|               the correct 2nd level handler if it has been registered.
|
| Parameters  : void
|
| Returns     : void
+----------------------------------------------------------------------------*/
void dal_IRQ_Hndl(void)
{
    U32 temp;
    U32 irq_num;
    irq_num = in_regl(INTC_BASE + INTC_SIR_IRQ_OFFSET);
    dal_interrupt_dispatch(irq_num);
    temp = in_regl(INTC_BASE + INTC_CONTROL_OFFSET); 
    temp |= 1;
    out_regl((INTC_BASE + INTC_CONTROL_OFFSET), temp);
}

U32 get_platform_interrupt_vector_base_address()
{

return((U32)INTERRUPT_VECTOR_BASE_ADDR);


}



#endif //INTERRUPT
