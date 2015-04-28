/*
 * Definitions for the MIPS sysmips(2) call
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995 by Ralf Baechle
 */
#ifndef _ASM_SYSMIPS_H
#define _ASM_SYSMIPS_H

/*
 * Commands for the sysmips(2) call
 *
 * sysmips(2) is deprecated - though some existing software uses it.
 * We only support the following commands.
 */
#define SETNAME			   1	/* set hostname			 */
#define FLUSH_CACHE		   3	/* writeback and invalidate caches */
#define MIPS_FIXADE		   7	/* control address error fixing	 */
#define MIPS_RDNVRAM		  10	/* read NVRAM */
#define MIPS_ATOMIC_SET		2001	/* atomically set variable	 */
#define MIPS_FPU_PRCTL          2002    /* FPU mode/emulation fine control */

#ifndef PR_SET_FP_MODE
#define PR_SET_FP_MODE 43
#endif
#ifndef PR_GET_FP_MODE
#define PR_GET_FP_MODE 44
#endif
#ifndef PR_FP_MODE_FR
#define PR_FP_MODE_FR  (1 << 0)
#endif
#ifndef PR_FP_MODE_FRE
#define PR_FP_MODE_FRE (1 << 1)
#endif

#endif /* _ASM_SYSMIPS_H */
