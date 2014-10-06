/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  Download Module (ROM assisted)
+------------------------------------------------------------------------------

|             Copyright 2003 Texas Instruments.
|             All rights reserved.
|
|             This file is confidential and a trade secret of Texas
|             Instruments.
|             The receipt of or possession of this file does not convey
|             any rights to reproduce or disclose its contents or to
|             manufacture, use, or sell anything it may describe, in
|             whole, or in part, without the specific written consent of
|             Texas Instruments.
+------------------------------------------------------------------------------
| Filename:   dnld_32bit.c
| Author:     Søren Steen Christensen (ssc@ti.com)
| Purpose:    Target download module
+----------------------------------------------------------------------------*/
#include "types.h"
//#include "p_dnld_protocol.h"
#include "dnld.h"

extern struct T_dnld_trg_func_struct     * dnld_trg_func_struct_ptr;

static int runResult=CSST_SUCCESS;
static int runAddr;

asm("stackPointerSafeSpace          .long 0");
asm("runResult_address              .long   _runResult");
asm("runAddr_address                .long   _runAddr");
asm("stackPointerSafeSpace_address  .long   stackPointerSafeSpace");
asm("returnAfterExecute_address     .long   returnAfterExecute");

void restore_mmu_and_cache_post_branch()
{
    // Restore the MMU configuration
    if (dnld_trg_func_struct_ptr->mmu_restore_config)
        dnld_trg_func_struct_ptr->mmu_restore_config();

    // Enable the caches (and MMU) again
    if (dnld_trg_func_struct_ptr->cache_enable)
        dnld_trg_func_struct_ptr->cache_enable();
}


/* WARNING: When using this function it's IMPORTANT; that
physical and virtual address mapping is one to one.

This is needed because of the following:
1) Return address value is calculated with MMU enabled, but
used with MMU disabled
2) The global variable for saving the stack pointer is used
both with MMU enabled and disabled.

The way to fix this current requirement would be to implement a virtual to physical and
physical to virtual addressing translator, which could convert the nessecary values -
This is however not done - Yet...

Last but not least, this code can currently only jump to ARM code =>
Jump to images starting in Thumb mode isn't supported...
*/

U32 branchandreturn(U32 runcode)
{
    runAddr = runcode;

    // Save registers r0-r12, lr (r14) and cpsr on the stack
    asm("  stmfd   sp!,  {r0-r12,lr}");
    asm("  mrs     r1,   cpsr");                // Get current processor status register into r1.
    asm("  stmfd   sp!,  {r1}");                // Save cpsr register to the stack

    // Save sp_irq on the stack as well
    asm("  bic     r1,   r1, #0x1f");           // Clear M[3..0] bits.
    asm("  orr     r1,   r1, #0x12");           // Set mode bits.
    asm("  msr     cpsr, r1");                  // Set mode.
    asm("  mov     r2,   sp");                  // Get stack pointer.
    asm("  mrs     r1,   cpsr");                // Switch back to Supervisor mode
    asm("  bic     r1,   r1, #0x1f");
    asm("  orr     r1,   r1, #0x13");
    asm("  msr     cpsr, r1");
    asm("  stmfd   sp!,  {r2}");

    // Save the MMU configuration
    if (dnld_trg_func_struct_ptr->mmu_save_config)
        dnld_trg_func_struct_ptr->mmu_save_config();

    // Save stackpointer in global variable
    asm("  ldr     r2,   stackPointerSafeSpace_address");
    asm("  str     sp,   [r2]");

    // Flush caches and drain write buffer
    if (dnld_trg_func_struct_ptr->cache_flush)
        dnld_trg_func_struct_ptr->cache_flush();

    // Disable the caches (and MMU)
    if (dnld_trg_func_struct_ptr->cache_disable)
        dnld_trg_func_struct_ptr->cache_disable();

    // Get branch address and save return address in r0
    asm("  ldr     r1,   runAddr_address");
    asm("  ldr     r1,   [r1]");
    asm("  ldr     r0,   returnAfterExecute_address");

    // Do the actual branch
    asm("  mov     pc,   r1");
    asm("returnAfterExecute:");

    // Restore stackpointer
    asm("  ldr     r1,   stackPointerSafeSpace_address");
    asm("  ldr     sp,   [r1]");

    // Move return value from r0 to r10 in order to preserve it through the following function calls
    asm("  mov     r10,  r0");

    // Restore the MMU and cache setup - This needs to be put into a seperate function since the
    // optimizer else will think that context is preserved through the branch and return above...
    // Since we can't garantee this the return back to CSST would normally fail because of this
    // faulty compiler optimization
    restore_mmu_and_cache_post_branch();

    // Save return value into runResult
    asm("  ldr     r1,   runResult_address");
    asm("  str     r10,   [r1]");

    // Restore IRQ mode stack pointer as well
    asm("  ldmfd   sp!,  {r0}");
    asm("  mrs     r1,   cpsr");                // Get current processor status register.
    asm("  bic     r1,   r1,   #0x1f");         // Clear M[3..0] bits.
    asm("  orr     r1,   r1,   #0x12");         // Set mode bits.
    asm("  msr     cpsr, r1");                  // Set mode.
    asm("  mov     sp,   r0");                  // Set stack pointer.
    asm("  mrs     r1,   cpsr");                // Switch back to Supervisor mode
    asm("  bic     r1,   r1,   #0x1f");
    asm("  orr     r1,   r1,   #0x13");
    asm("  msr     cpsr, r1");

    // Restore cpsr, r0-r12 and r14 from the stack
    asm("  ldmfd   sp!,  {r1}");
    asm("  msr     cpsr, r1");
    asm("  ldmfd   sp!,  {r0-r12,lr}");

    // Dummy line in order to avoid compiler warning, since RunAddr is only referenced from ASM
    runAddr++; runAddr--;
    runResult++;runResult--;
    return runResult;
}
