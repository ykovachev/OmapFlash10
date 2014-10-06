/*-----------------------------------------------------------------------------
|  Project :  CSST
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
| Purpose:    <description of purpose of this file>
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL =================================================*/

#ifndef CSST_DAL_MMUFWORK_H
#define CSST_DAL_MMUFWORK_H

/*==== INCLUDES ============================================================*/


/*==== CONSTS ==============================================================*/
#define DAL_MMU_SECTION	    0x2
#define DAL_MMU_DFLT_BITS 	0x10
#define DAL_MMU_BUFFERABLE 	0x4
#define DAL_MMU_CACHEABLE	  0x8
#define DAL_MMU_DOMAIN(x)   ((x & 0xF) << 5)
#define DAL_MMU_ACCESS_MGR	0xC00
#define DAL_MMU_SECTION_BASE(x)                 ((x & 0xFFF) << 20)

#define DAL_MPU_MMU_DOMAIN_ACCESS_CLIENT        0x1
#define DAL_MPU_MMU_DOMAIN_ACCESS_MGR           0x3
#define DAL_MMU_SECTION_MASK                    0xFFF00000
#define DAL_TABLE_INDEX_BITS                    20
/*TBD*/

/*==== TYPEDEFINES ===============================================================*/
/*TBD*/

typedef struct dal_mmu_entry
{
    struct dal_mmu_entry *               nextptr;
    U32                             phy_addr;
    U32                             virt_addr;
    U32                             entry_flags;
} T_DAL_MMU_ENTRY;

/*====== FUNCTION PROTOTYPES==================================================*/

extern S32 dal_mmu_arm_unmap_section (U32 virtualBase);

extern void dal_mmu_arm_disable (void);

extern void dal_mmu_arm_enable (void);

extern void dal_mmu_arm_flush_tlbs (U32 virtualAdrs);

extern void dal_mmu_arm_set_trans_table (U32 adrs);

extern void dal_mmu_arm_set_domain_access (U32 domain, U32 access);

extern S32 dal_mmu_dcache(U8 enable);
extern S32 dal_mmu_icache(U8 enable);

#endif /* CSST_DAL_MMUFWORK_H */
