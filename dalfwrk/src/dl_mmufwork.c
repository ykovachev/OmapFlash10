
/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  <Device Abstraction Layer>
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
| Purpose:    This File contains the DAL MMU framework API functions.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef CSST_DAL_MMUFWORK_C
#define CSST_DAL_MMUFWORK_C

/*==== INCLUDES ==============================================================*/
#define PRIVATE_DAL_MMUFWORK_H
#include "csst_tgt.h"
#include "dl_mmufwork.h"
#include "dl_mmufwork_ex.h"
//#include "silicon.h"

/*==== DEFINES ===============================================================*/

/******* Definitions of CP15 registers ********************/
#define C1_MMU          (1<<0)          /* mmu off/on */
#define C1_ALIGN        (1<<1)          /* alignment faults off/on */
#define C1_DC           (1<<2)          /* dcache off/on */
#define C1_WB           (1<<3)          /* merging write buffer on/off */
#define C1_BIG_ENDIAN   (1<<7)  /* big endian off/on */
#define C1_SYS_PROT     (1<<8)          /* system protection */
#define C1_ROM_PROT     (1<<9)          /* ROM protection */
#define C1_IC           (1<<12)         /* icache off/on */
#define C1_HIGH_VECTORS (1<<13) /* location of vectors: low/high addresses */
#define RESERVED_1      (0xf << 3)      /* must be 111b for R/W */

/*==== FUNCTION PROTOTYPE =====================================================*/
//extern const U32 MMU_TABLE_LOCATION;
//U32 *dal_mmu_translation_table = (U32 *) (&MMU_TABLE_LOCATION);

//static T_DAL_MMU_ENTRY * dal_mmu_find_entry (U32 phy_address);
static void cp_delay (void);
//static S32 dal_remove_entry(U32 virtualbase);
//extern U32 *dal_mmu_translation_table;
extern unsigned int read_cp15_c1(void);
extern void write_cp15_c1(unsigned int c1);
extern void arm_icache_flush(void);

/*==== PUBLIC FUNCTIONS ======================================================*/

/* Enable or Disable the Data Cache */
S32 dal_mmu_dcache(U8 enable)
{
    unsigned int reg = read_cp15_c1();
    if (enable)
    {
        reg |= C1_DC;
    } else {
        reg &= ~C1_DC;
    }
    cp_delay();
    write_cp15_c1(reg);
    /* TBD: Should flush and invalidate dcache..*/
    return CSST_DAL_SUCCESS;
}

/* Enable or Disable the Instruction Cache */
S32 dal_mmu_icache(U8 enable)
{
    unsigned int reg = read_cp15_c1();
    if (enable)
    {
        reg |= C1_IC;
    } else {
        reg &= ~C1_IC;
    }
    cp_delay();
    write_cp15_c1(reg);
    arm_icache_flush();
    return CSST_DAL_SUCCESS;
}

S32 dal_kind_of_mmu_control(U8 enable)
{
    unsigned int reg = read_cp15_c1();
    if (enable)
    {
        reg |= C1_MMU;
    } else {
        reg &= ~C1_MMU;
    }
    cp_delay();
    write_cp15_c1(reg);
    return CSST_DAL_SUCCESS;
}

#if 0
static T_DAL_MMU_ENTRY * dal_mmu_table_list_header;

S32 dal_mmu_init()
{

    S32 i;

    //  dal_mmu_translation_table = (U32 *)MMU_TABLE_LOCATION;
    if(dal_mmu_translation_table == NULL)
    {
        return CSST_DAL_ERROR;
    }

    /*  ... initialize the table (there are 4K = 0x1000 entries)
    */
    for (i = 0; i < 0x1000; i ++)
    {
        dal_mmu_translation_table[i] = DAL_MMU_SECTION | DAL_MMU_DFLT_BITS |
        DAL_MMU_CACHEABLE | DAL_MMU_DOMAIN(0) |
        DAL_MMU_ACCESS_MGR | DAL_MMU_SECTION_BASE(i*0x100000)
        + (i * 0x00100000);    /* EZ */
    }

    dal_mmu_arm_set_trans_table ((U32)dal_mmu_translation_table);

    /*Initialise the MMU header to NULL*/
    dal_mmu_table_list_header = NULL;

    dal_mmu_arm_set_domain_access (0, DAL_MPU_MMU_DOMAIN_ACCESS_MGR);

    return CSST_DAL_SUCCESS;

}

/**************************************************************************
* dal_mmuArmMapSection - sets up a section entry in the MMU Translation
*                        table and enables it
*
* This function sets up a section entry in the MMU Translation
* table and enables it. Sections are fixed at 1 Mbyte in size
* 'bufferable' if TRUE allows buffered writes
* 'cacheable' if TRUE allows cacheing of data or instructions
*
* RETURNS:  DAL_SUCCESS always
*/
S32 dal_mmu_map_section
(
 U32      virtual_base,
 U32      physical_base,
 S32      bufferable,
 S32      cacheable
)

{
    U32      section_base = physical_base & DAL_MMU_SECTION_MASK;
    U32      table_index = virtual_base >> DAL_TABLE_INDEX_BITS;
    U32      buffer, cache;
    T_DAL_MMU_ENTRY * mmu_node;

    buffer = ((bufferable) ? DAL_MMU_BUFFERABLE : 0);
    cache = ((cacheable) ? DAL_MMU_CACHEABLE : 0);

    /*Step1: add to the TLB*/
    dal_mmu_translation_table[table_index] = DAL_MMU_SECTION | DAL_MMU_DFLT_BITS |
    buffer | cache | DAL_MMU_DOMAIN(0) |
    DAL_MMU_ACCESS_MGR | section_base;

    /*Step2: add to software maintained Link List*/
    mmu_node = dal_mmu_find_entry(section_base);
    if(mmu_node == NULL)
    {
        if ((mmu_node = (T_DAL_MMU_ENTRY *)malloc(sizeof(T_DAL_MMU_ENTRY))) == NULL)
        {
            return -1;
        }
        mmu_node->nextptr = dal_mmu_table_list_header;
        dal_mmu_table_list_header = mmu_node;
    }
    mmu_node->virt_addr = virtual_base & DAL_MMU_SECTION_MASK;
    mmu_node->phy_addr = physical_base & DAL_MMU_SECTION_MASK;

    dal_mmu_arm_flush_tlbs (virtual_base);
    return CSST_DAL_SUCCESS;
}

/**************************************************************************
* dal_mmuArmUnmapSection - removes a section entry from the MMU Translation
*                          table
*
* This function removes a section entry from the MMU Translation table.
* Leaves a "map-through" entry in the table
*
* RETURNS:  DAL_SUCCESS always
*/
S32 dal_mmu_arm_unmap_section (U32 virtualbase)
{
    S32         tableindex = virtualbase >> 20;


    dal_fiq_disable ();
    dal_irq_disable ();


    dal_mmu_translation_table[tableindex] = DAL_MMU_SECTION | DAL_MMU_DFLT_BITS |
    DAL_MMU_CACHEABLE | DAL_MMU_DOMAIN(0) |
    DAL_MMU_ACCESS_MGR |
    DAL_MMU_SECTION_BASE(tableindex*0x100000)
    + (tableindex * 0x00100000);

    dal_mmu_arm_flush_tlbs(virtualbase);

    /*Remove the Node from the software list*/
    dal_remove_entry(virtualbase);

    /*re-enable interrupts  */
    dal_fiq_enable ();
    dal_irq_enable ();

    return CSST_DAL_SUCCESS;
}

S32 dal_mmu_get_va(U32 * virt_addr, U32 phy_addr)
{
    T_DAL_MMU_ENTRY * cur_mmu_cfg;
    cur_mmu_cfg = dal_mmu_find_entry(phy_addr);
    /*The return value NULL means that there is no
    entry, hence the virtual address = Physical address*/
    if(cur_mmu_cfg == NULL)
    {
        *virt_addr = phy_addr;
    }
    else
    {
        /*if the return value is not null, then
        the first 12 bits of the virtual address is
        equal to the first 12 bits of the virt_entry
        of the node found
        the last 24 bits will be same as the phy_address*/
        *virt_addr = (((cur_mmu_cfg->virt_addr)&DAL_MMU_SECTION_MASK) |
                      (phy_addr | (~DAL_MMU_SECTION_MASK)));
    }
    return(CSST_DAL_SUCCESS);

}

S32 dal_mmu_get_pa(U32 virt_addr, U32 * phy_addr)
{
    U32      table_index = virt_addr >> DAL_TABLE_INDEX_BITS;
    *phy_addr = ((dal_mmu_translation_table[table_index] & DAL_MMU_SECTION_MASK)
                 | (virt_addr | (~DAL_MMU_SECTION_MASK)));
    return(CSST_DAL_SUCCESS);
}

/*==== PRIVATE FUNCTIONS =====================================================*/

static T_DAL_MMU_ENTRY * dal_mmu_find_entry (U32 phy_address)
{
    T_DAL_MMU_ENTRY * cur_mmu_cfg;

    phy_address = phy_address & DAL_MMU_SECTION_MASK;
    /* Search for the requested target */
    for (cur_mmu_cfg = dal_mmu_table_list_header;
         cur_mmu_cfg != NULL;
         cur_mmu_cfg = cur_mmu_cfg-> nextptr
        )
    {
        /* Check to see if this is the one */
        if (cur_mmu_cfg->phy_addr == phy_address)
        {
            /* Found a match, bail out */
            break;
        }
    }

    /* Done */
    return(cur_mmu_cfg);
}

static S32 dal_remove_entry(U32 virtualbase)
{
    T_DAL_MMU_ENTRY * cur_mmu_cfg, * temp_mmu_cfg;

    virtualbase = virtualbase & DAL_MMU_SECTION_MASK;

    if(dal_mmu_table_list_header == NULL)
    {
        return(CSST_DAL_MMU_ENTRY_NOT_FOUND);
    }
    temp_mmu_cfg = dal_mmu_table_list_header;
    if(dal_mmu_table_list_header->virt_addr == virtualbase)
    {
        dal_mmu_table_list_header = dal_mmu_table_list_header->nextptr;
        free(temp_mmu_cfg);
        return(CSST_DAL_SUCCESS);
    }

    /* Search for the requested target */
    for (cur_mmu_cfg = dal_mmu_table_list_header->nextptr;
         cur_mmu_cfg != NULL;
         cur_mmu_cfg = cur_mmu_cfg-> nextptr
        )
    {
        /* Check to see if this is the one */
        if (cur_mmu_cfg->virt_addr == virtualbase)
        {
            temp_mmu_cfg->nextptr = cur_mmu_cfg->nextptr;
            free(cur_mmu_cfg);
            /* Found a match, bail out */
            return(CSST_DAL_SUCCESS);

        }
        temp_mmu_cfg = cur_mmu_cfg;
    }

    /* Done */
    return(CSST_DAL_MMU_ENTRY_NOT_FOUND);
}
#endif
static void cp_delay (void)
{
    volatile int i;
    /* Many OMAP regs need at least 2 nops  */
    for (i = 0; i < 100; i++);
}

#endif

