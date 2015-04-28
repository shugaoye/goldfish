/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Cluster Power Controller Subsystem Register Definitions
 *
 * Copyright (C) 2013 Imagination Technologies Ltd
 *    Leonid Yegoshin (Leonid.Yegoshin@imgtec.com)
 *
 */
#ifndef _ASM_CPCREGS_H
#define _ASM_CPCREGS_H


/* Offsets to major blocks within CPC from CPC base */
#define CPC_GCB_OFS             0x0000 /* Global Control Block */
#define CPC_CLCB_OFS            0x2000 /* Core Local Control Block */
#define CPC_COCB_OFS            0x4000 /* Core Other Control Block */

#define CPCGCBOFS(x)            (CPC_##x##_OFS + CPC_GCB_OFS)
#define CPCLCBOFS(x)            (CPC_##x##_OFS + CPC_CLCB_OFS)
#define CPCOCBOFS(x)            (CPC_##x##_OFS + CPC_COCB_OFS)

#define CPCGCB(x)               REGP(_cpc_base, CPCGCBOFS(x))
#define CPCLCB(x)               REGP(_cpc_base, CPCLCBOFS(x))
#define CPCOCB(x)               REGP(_cpc_base, CPCOCBOFS(x))

/* Global section registers offsets */
#define CPC_CSRAPR_OFS          0x000
#define CPC_SEQDELAY_OFS        0x008
#define CPC_RAILDELAY_OFS       0x010
#define CPC_RESETWIDTH_OFS      0x018
#define CPC_REVID_OFS           0x020
#define CPC_CLCTL_OFS           0x028
#define CPC_PWRUP_OFS           0x030
#define CPC_RESETST_OFS         0x040

/* Local and Other Local sections registers offsets */
#define CPC_CMD_OFS             0x000
#define CPC_STATUS_OFS          0x008
#define CPC_OTHER_OFS           0x010
#define CPC_CCCTL_OFS           0x018
#define CPC_LPACK_OFS           0x020
#define CPC_VCRUN_OFS           0x028
#define CPC_VCSPND_OFS          0x030
#define CPC_RAMSLEEP_OFS        0x050

/* Command and Status registers fields masks and offsets */

#define CPCL_PWRUP_EVENT_MASK   0x00800000
#define CPCL_PWRUP_EVENT_SH     23

#define CPCL_STATUS_MASK        0x00780000
#define CPCL_STATUS_SH          19
#define CPCL_STATUS_U5          0x6
#define CPCL_STATUS_U6          0x7


#define CPCL_CLKGAT_IMPL_MASK   0x00020000
#define CPCL_CLKGAT_IMPL_SH     17

#define CPCL_PWRDN_IMPL_MASK    0x00010000
#define CPCL_PWRDN_IMPL_SH      16

#define CPCL_EJTAG_MASK         0x00008000
#define CPCL_EJTAG_SH           15

#define CPCL_IO_PWUP_POLICY_MASK    0x00000300
#define CPCL_IO_PWUP_POLICY_SH      8

#define CPCL_IO_TRFFC_EN_MASK   0x00000010
#define CPCL_IO_TRFFC_EN_SH     4

#define CPCL_CMD_MASK           0xf
#define CPCL_CMD_SH             0

extern int __init cpc_probe(unsigned long defaddr, unsigned long defsize);

#endif /* _ASM_CPCREGS_H */
