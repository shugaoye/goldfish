/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2000, 07 MIPS Technologies, Inc.
 *
 * Multiprocessor Subsystem Register Definitions
 *
 */
#ifndef _ASM_GCMPREGS_H
#define _ASM_GCMPREGS_H


/* Offsets to major blocks within GCMP from GCMP base */
#define GCMP_GCB_OFS		0x0000 /* Global Control Block */
#define GCMP_CLCB_OFS		0x2000 /* Core Local Control Block */
#define GCMP_COCB_OFS		0x4000 /* Core Other Control Block */
#define GCMP_GDB_OFS		0x8000 /* Global Debug Block */

/* Offsets to individual GCMP registers from GCMP base */
#define GCMPOFS(block, tag, reg)	\
	(GCMP_##block##_OFS + GCMP_##tag##_##reg##_OFS)
#define GCMPOFSn(block, tag, reg, n) \
	(GCMP_##block##_OFS + GCMP_##tag##_##reg##_OFS(n))

#define GCMPGCBOFS(reg)		GCMPOFS(GCB, GCB, reg)
#define GCMPGCBOFSn(reg, n)	GCMPOFSn(GCB, GCB, reg, n)
#define GCMPCLCBOFS(reg)	GCMPOFS(CLCB, CCB, reg)
#define GCMPCLCBOFSn(reg, n)     GCMPOFSn(CLCB, CCB, reg, n)
#define GCMPCOCBOFS(reg)	GCMPOFS(COCB, CCB, reg)
#define GCMPCOCBOFSn(reg, n)     GCMPOFSn(COCB, CCB, reg, n)
#define GCMPGDBOFS(reg)		GCMPOFS(GDB, GDB, reg)

/* GCMP register access */
#define GCMPGCB(reg)			REGP(_gcmp_base, GCMPGCBOFS(reg))
#define GCMPGCBhi(reg)                  REGP(_gcmp_base, (GCMPGCBOFS(reg) + 4))
#define GCMPGCBlo(reg)                  GCMPGCB(reg)
//#define GCMPGCBaddr(reg)                REGA(_gcmp_base, GCMPGCBOFS(reg))
#if defined(CONFIG_64BIT) || defined(CONFIG_64BIT_PHYS_ADDR)
#define GCMPGCBaddr(reg)                ((((phys_addr_t)(GCMPGCBhi(reg))) << 32) | \
							 GCMPGCBlo(reg))
#define GCMPGCBaddrWrite(reg,val)       (GCMPGCBhi(reg) = (u32)((phys_addr_t)(val) >> 32), \
					 GCMPGCBlo(reg) = (u32)(val))
#else
#define GCMPGCBaddr(reg)                GCMPGCB(reg)
#define GCMPGCBaddrWrite(reg,val)       (GCMPGCB(reg) = (u32)(val))
#endif
#define GCMPGCBn(reg, n)	       REGP(_gcmp_base, GCMPGCBOFSn(reg, n))
#define GCMPCLCB(reg)			REGP(_gcmp_base, GCMPCLCBOFS(reg))
#define GCMPCLCBn(reg, n)               REGP(_gcmp_base, GCMPCLCBOFSn(reg, n))
#define GCMPCOCB(reg)			REGP(_gcmp_base, GCMPCOCBOFS(reg))
#define GCMPCOCBhi(reg)                   REGP(_gcmp_base, (GCMPCOCBOFS(reg) + 4))
#define GCMPCOCBlo(reg)                 GCMPCOCB(reg)
#define GCMPCOCBn(reg, n)               REGP(_gcmp_base, GCMPCOCBOFSn(reg, n))
#define GCMPGDB(reg)			REGP(_gcmp_base, GCMPGDBOFS(reg))

/* Mask generation */
#define GCMPMSK(block, reg, bits)	(MSK(bits)<<GCMP_##block##_##reg##_SHF)
#define GCMPGCBMSK(reg, bits)		GCMPMSK(GCB, reg, bits)
#define GCMPCCBMSK(reg, bits)		GCMPMSK(CCB, reg, bits)
#define GCMPGDBMSK(reg, bits)		GCMPMSK(GDB, reg, bits)

/* GCB registers */
#define GCMP_GCB_GC_OFS			0x0000	/* Global Config Register */
#define	 GCMP_GCB_GC_NUMIOCU_SHF	8
#define	 GCMP_GCB_GC_NUMIOCU_MSK	GCMPGCBMSK(GC_NUMIOCU, 4)
#define	 GCMP_GCB_GC_NUMCORES_SHF	0
#define	 GCMP_GCB_GC_NUMCORES_MSK	GCMPGCBMSK(GC_NUMCORES, 8)
#define GCMP_GCB_GCMPB_OFS		0x0008		/* Global GCMP Base */
#define	 GCMP_GCB_GCMPB_GCMPBASE_SHF	15
#ifndef CONFIG_64BIT
#define  GCMP_GCB_GCMPB_GCMPBASE_MSK    GCMPGCBMSK(GCMPB_GCMPBASE, 17)
#else
#define  GCMP_GCB_GCMPB_GCMPBASE_MSK    GCMPGCBMSK(GCMPB_GCMPBASE, 32)
#endif
#define	 GCMP_GCB_GCMPB_CMDEFTGT_SHF	0
#define	 GCMP_GCB_GCMPB_CMDEFTGT_MSK	GCMPGCBMSK(GCMPB_CMDEFTGT, 2)
#define	 GCMP_GCB_GCMPB_CMDEFTGT_DISABLED	0
#define	 GCMP_GCB_GCMPB_CMDEFTGT_MEM		1
#define	 GCMP_GCB_GCMPB_CMDEFTGT_IOCU1		2
#define	 GCMP_GCB_GCMPB_CMDEFTGT_IOCU2		3
#define GCMP_GCB_GCMC_OFS               0x0010  /* Global CM Control */
#define GCMP_GCB_GCMC2_OFS              0x0018  /* Global CM Control2/CM3 Alt Control */
#define GCMP_GCB_GCSRAP_OFS		0x0020	/* Global CSR Access Privilege */
#define	 GCMP_GCB_GCSRAP_CMACCESS_SHF	0
#define	 GCMP_GCB_GCSRAP_CMACCESS_MSK	GCMPGCBMSK(GCSRAP_CMACCESS, 8)
#define GCMP_GCB_GCMPREV_OFS		0x0030	/* GCMP Revision Register */
#define	 GCMP_GCB_GCMPREV_MAJOR_SHF	8
#define	 GCMP_GCB_GCMPREV_MAJOR_MSK	GCMPGCBMSK(GCMPREV_MAJOR, 8)
#define	 GCMP_GCB_GCMPREV_MINOR_SHF	0
#define	 GCMP_GCB_GCMPREV_MINOR_MSK	GCMPGCBMSK(GCMPREV_MINOR, 8)
#define GCMP_GCB_GCMECTL_OFS            0x0038  /* Global CM3 Error Control */
#define GCMP_GCB_GCMEM_OFS		0x0040	/* Global CM Error Mask */
#define GCMP_GCB_GCMEC_OFS		0x0048	/* Global CM Error Cause */
#define	 GCMP_GCB_GMEC_ERROR_TYPE_SHF	27
#define	 GCMP_GCB_GMEC_ERROR_TYPE_MSK	GCMPGCBMSK(GMEC_ERROR_TYPE, 5)
#define	 GCMP_GCB_GMEC_ERROR_INFO_SHF	0
#define  GCMP_GCB_GMEC_ERROR_INFO_MSK   GCMPGCBMSK(GMEC_ERROR_INFO, 27)
#define GCMP_GCB_GCMEA_OFS		0x0050	/* Global CM Error Address */
#define GCMP_GCB_GCMEO_OFS		0x0058	/* Global CM Error Multiple */
#define	 GCMP_GCB_GMEO_ERROR_2ND_SHF	0
#define	 GCMP_GCB_GMEO_ERROR_2ND_MSK	GCMPGCBMSK(GMEO_ERROR_2ND, 5)
#define GCMP_GCB_GCMCUS_OFS             0x0060  /* GCR Custom Base */
#define GCMP_GCB_GCMCST_OFS             0x0068  /* GCR Custom Status */
#define GCMP_GCB_GCML2S_OFS             0x0070  /* Global L2 only Sync Register */
#define  GCMP_GCB_GCML2S_EN_SHF         0
#define  GCMP_GCB_GCML2S_EN_MSK         GCMPGCBMSK(GCML2S_EN, 1)
#define GCMP_GCB_GICBA_OFS              0x0080  /* Global Interrupt Controller Base Address */
#define	 GCMP_GCB_GICBA_BASE_SHF	17
#define	 GCMP_GCB_GICBA_BASE_MSK	GCMPGCBMSK(GICBA_BASE, 15)
#define	 GCMP_GCB_GICBA_EN_SHF		0
#define	 GCMP_GCB_GICBA_EN_MSK		GCMPGCBMSK(GICBA_EN, 1)
#define GCMP_GCB_CPCBA_OFS              0x0088  /* CPC Base Address */
#define  GCMP_GCB_CPCBA_SHF             15
#define  GCMP_GCB_CPCBA_MSK             GCMPGCBMSK(CPCBA, 17)
#define  GCMP_GCB_CPCBA_EN_SHF          0
#define  GCMP_GCB_CPCBA_EN_MSK          GCMPGCBMSK(CPCBA_EN, 1)

#define GCMP_GCB_GICST_OFS              0x00D0  /* Global Interrupt Controller Status */
#define  GCMP_GCB_GICST_EN_SHF          0
#define  GCMP_GCB_GICST_EN_MSK          GCMPGCBMSK(GICST_EN, 1)
#define GCMP_GCB_GCSHREV_OFS            0x00E0  /* Cache Revision */
#define GCMP_GCB_CPCST_OFS              0x00F0  /* CPC Status */
#define  GCMP_GCB_CPCST_EN_SHF          0
#define  GCMP_GCB_CPCST_EN_MSK          GCMPGCBMSK(CPCST_EN, 1)

#define GCMP_GCB_IOCBASE_OFS            0x0100  /* IOCU Base Address */
#define GCMP_GCB_IOST_OFS               0x0108  /* IOMMU Status */
#define GCMP_GCB_G3CSRAP_OFS            0x0120  /* CM3 CSR Access Privilege Register */
#define GCMP_GCB_L2CONFIG_OFS           0x0130  /* CM3 L3 Config */
#define  GCMP_GCB_L2CONFIG_ASSOC_SHF    0
#define  GCMP_GCB_L2CONFIG_ASSOC_MASK   GCMPGCBMSK(L2CONFIG_ASSOC, 8)
#define  GCMP_GCB_L2CONFIG_LSIZE_SHF    8
#define  GCMP_GCB_L2CONFIG_LSIZE_MASK   GCMPGCBMSK(L2CONFIG_LSIZE, 4)
#define  GCMP_GCB_L2CONFIG_SSIZE_SHF    12
#define  GCMP_GCB_L2CONFIG_SSIZE_MASK   GCMPGCBMSK(L2CONFIG_SSIZE, 4)
#define  GCMP_GCB_L2CONFIG_BYPASS_SHF   20
#define  GCMP_GCB_L2CONFIG_BYPASS_MASK  GCMPGCBMSK(L2CONFIG_BYPASS, 1)

#define GCMP_GCB_SYSCONF_OFS            0x0140  /* CM3 SYS Config */
#define GCMP_GCB_SYSCONF2_OFS           0x0150  /* CM3 SYS Config2 */
#define  GCMP_GCB_SYSCONF2_VPWIDTH_SHF  0
#define  GCMP_GCB_SYSCONF2_VPWIDTH_MASK GCMPGCBMSK(SYSCONF2_VPWIDTH, 4)

/* GCB Regions */
#define GCMP_GCB_CMxBASE_OFS(n)		(0x0090+16*(n))		/* Global Region[0-3] Base Address */
#define	 GCMP_GCB_CMxBASE_BASE_SHF	16
#define	 GCMP_GCB_CMxBASE_BASE_MSK	GCMPGCBMSK(CMxBASE_BASE, 16)
#define GCMP_GCB_CMxMASK_OFS(n)		(0x0098+16*(n))		/* Global Region[0-3] Address Mask */
#define	 GCMP_GCB_CMxMASK_MASK_SHF	16
#define	 GCMP_GCB_CMxMASK_MASK_MSK	GCMPGCBMSK(CMxMASK_MASK, 16)
#define	 GCMP_GCB_CMxMASK_CMREGTGT_SHF	0
#define	 GCMP_GCB_CMxMASK_CMREGTGT_MSK	GCMPGCBMSK(CMxMASK_CMREGTGT, 2)
#define	 GCMP_GCB_CMxMASK_CMREGTGT_MEM	 0
#define	 GCMP_GCB_CMxMASK_CMREGTGT_MEM1	 1
#define	 GCMP_GCB_CMxMASK_CMREGTGT_IOCU1 2
#define	 GCMP_GCB_CMxMASK_CMREGTGT_IOCU2 3

#define GCMP_GCB_GAOR0BA_OFS              0x0190  /* Attribute-Only Region0 Base Address */
#define GCMP_GCB_GAOR0MASK_OFS            0x0198  /* Attribute-Only Region0 Mask  */
#define GCMP_GCB_GAOR1BA_OFS              0x01A0  /* Attribute-Only Region1 Base Address */
#define GCMP_GCB_GAOR1MASK_OFS            0x01A8  /* Attribute-Only Region1 Mask */

#define GCMP_GCB_IOCUREV_OFS              0x0200  /* IOCU Revision */

#define GCMP_GCB_GAOR2BA_OFS              0x0210  /* Attribute-Only Region2 Base Address */
#define GCMP_GCB_GAOR2MASK_OFS            0x0218  /* Attribute-Only Region2 Mask */
#define GCMP_GCB_GAOR3BA_OFS              0x0220  /* Attribute-Only Region3 Base Address */
#define GCMP_GCB_GAOR3MASK_OFS            0x0228  /* Attribute-Only Region3 Mask */
#define GCMP_GCB_L2RAMCONF_OFS            0x0240  /* CM3 L2 RAM Configuration */
#define GCMP_GCB_SCRATCH0_OFS             0x0280  /* CM3 Scratch 0 */
#define GCMP_GCB_SCRATCH1_OFS             0x0288  /* CM3 Scratch 1 */
#define GCMP_GCB_GCML2P_OFS               0x0300  /* L2 Prefetch Control */
#define  GCMP_GCB_GCML2P_PAGE_MASK          0xfffff000  /* ... page mask */
#define  GCMP_GCB_GCML2P_PFTEN              0x00000100  /* L2 Prefetch Enable */
#define  GCMP_GCB_GCML2P_NPFT               0x000000ff  /* N.of L2 Prefetch  */
#define GCMP_GCB_GCML2PB_OFS              0x0308  /* L2 Prefetch Control B */
#define  GCMP_GCB_GCML2PB_CODE_PFTEN        0x00000100  /* L2 Code Prefetch Enable */
#define GCMP_GCB_L2PREF_OFS               0x0320  /* CM3 L2 Prefetch Tuning */
#define GCMP_GCB_L2PREFAT0_OFS            0x0340  /* CM3 L2 Prefetch Tuning A Tier 0 */
#define GCMP_GCB_L2PREFBT0_OFS            0x0348  /* CM3 L2 Prefetch Tuning B Tier 0 */
#define GCMP_GCB_L2PREFAT1_OFS            0x0360  /* CM3 L2 Prefetch Tuning A Tier 1 */
#define GCMP_GCB_L2PREFBT1_OFS            0x0368  /* CM3 L2 Prefetch Tuning B Tier 1 */
#define GCMP_GCB_L2PREFAT2_OFS            0x0380  /* CM3 L2 Prefetch Tuning A Tier 2 */
#define GCMP_GCB_L2PREFBT2_OFS            0x0388  /* CM3 L2 Prefetch Tuning B Tier 2 */
#define GCMP_GCB_L2PREFAT3_OFS            0x03A0  /* CM3 L2 Prefetch Tuning A Tier 3 */
#define GCMP_GCB_L2PREFBT3_OFS            0x03A8  /* CM3 L2 Prefetch Tuning B Tier 3 */
#define GCMP_GCB_L2PREFAT4_OFS            0x03C0  /* CM3 L2 Prefetch Tuning A Tier 4 */
#define GCMP_GCB_L2PREFBT4_OFS            0x03C8  /* CM3 L2 Prefetch Tuning B Tier 4 */
#define GCMP_GCB_L2TRCADDR_OFS            0x0600  /* CM3 L2 Tag RAM Cache OP Addr */
#define GCMP_GCB_L2TRCST_OFS              0x0608  /* CM3 L2 Tag RAM Cache OP State */
#define GCMP_GCB_L2DRCOP_OFS              0x0610  /* CM3 L2 DATA RAM Cache OP */
#define GCMP_GCB_L2TDECCOP_OFS            0x0618  /* CM3 L2 Tag and DATA ECC Cache OP */
#define GCMP_GCB_BEVBASE_OFS              0x0680  /* CM3 BEV Base */

/* Core local/Core other control block registers */
#define GCMP_CCB_RESETR_OFS		0x0000			/* Reset Release */
#define	 GCMP_CCB_RESETR_INRESET_SHF	0
#define	 GCMP_CCB_RESETR_INRESET_MSK	GCMPCCBMSK(RESETR_INRESET, 16)
#define GCMP_CCB_COHENB_OFS             0x0008                  /* CM3 Coherence Enable */
#define GCMP_CCB_COHCTL_OFS		0x0008			/* Coherence Control */
#define	 GCMP_CCB_COHCTL_DOMAIN_SHF	0
#define	 GCMP_CCB_COHCTL_DOMAIN_MSK	GCMPCCBMSK(COHCTL_DOMAIN, 8)
#define  GCMP_CCB_COHCTL_DOMAIN_ENABLE  (GCMP_CCB_COHCTL_DOMAIN_MSK)
#define GCMP_CCB_CFG_OFS                0x0010                  /* Config */
#define	 GCMP_CCB_CFG_IOCUTYPE_SHF	10
#define	 GCMP_CCB_CFG_IOCUTYPE_MSK	GCMPCCBMSK(CFG_IOCUTYPE, 2)
#define	  GCMP_CCB_CFG_IOCUTYPE_CPU	0
#define	  GCMP_CCB_CFG_IOCUTYPE_NCIOCU	1
#define	  GCMP_CCB_CFG_IOCUTYPE_CIOCU	2
#define	 GCMP_CCB_CFG_NUMVPE_SHF	0
#define	 GCMP_CCB_CFG_NUMVPE_MSK	GCMPCCBMSK(CFG_NUMVPE, 10)
#define GCMP_CCB_OTHER_OFS		0x0018		/* Other Address */
#define	 GCMP_CCB_OTHER_CORENUM_SHF	16
#define	 GCMP_CCB_OTHER_CORENUM_MSK	GCMPCCBMSK(OTHER_CORENUM, 16)
#define GCMP_CCB_RESETBASE_OFS		0x0020		/* Reset Exception Base */
#define	 GCMP_CCB_RESETBASE_BEV_SHF	12
#define	 GCMP_CCB_RESETBASE_BEV_MSK	GCMPCCBMSK(RESETBASE_BEV, 20)
#define GCMP_CCB_RESETBASEEXT_OFS       0x0030          /* Reset Exception Base Extention */
#define  GCMP_CCB_RESETEXTBASE_BEV_SHF      20
#define  GCMP_CCB_RESETEXTBASE_BEV_MASK_MSK GCMPCCBMSK(RESETEXTBASE_BEV, 8)
#define  GCMP_CCB_RESETEXTBASE_LOWBITS_SHF     0
#define  GCMP_CCB_RESETEXTBASE_BEV_MASK_LOWBITS GCMPCCBMSK(RESETEXTBASE_LOWBITS, 20)
#define GCMP_CCB_ID_OFS			0x0028		/* Identification */
#define GCMP_CCB_DINTGROUP_OFS		0x0030		/* DINT Group Participate */
#define GCMP_CCB_DBGGROUP_OFS		0x0100		/* DebugBreak Group */

#define GCMP_CCB_TCIDxPRI_OFS(n)        (0x0040+8*(n))  /* TCID x PRIORITY */

extern int __init gcmp_probe(unsigned long, unsigned long);
extern void __init gcmp_setregion(int, unsigned long, unsigned long, int);
#ifdef CONFIG_MIPS_CMP
extern int __init gcmp_niocu(void);
extern int gcmp_present;
extern int gcmp3_present;
#define GCR3_OTHER(core,vpe)            ((core << 8) | vpe)
#define GCR3_OTHER_CPU_DATA(cpu)        ((cpu_data[cpu].core << 8) | cpu_data[cpu].vpe_id)
#else
#define gcmp_niocu(x)   (0)
#define gcmp_present    (0)
#define gcmp3_present    (0)
#endif
extern unsigned long _gcmp_base;
#define GCMP_L2SYNC_OFFSET              0x8100

#endif /* _ASM_GCMPREGS_H */
