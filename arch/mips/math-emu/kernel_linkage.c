/*
 *  Kevin D. Kissell, kevink@mips and Carsten Langgaard, carstenl@mips.com
 *  Copyright (C) 2000 MIPS Technologies, Inc.	All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Routines corresponding to Linux kernel FP context
 * manipulation primitives for the Algorithmics MIPS
 * FPU Emulator
 */
#include <linux/sched.h>
#include <asm/processor.h>
#include <asm/signal.h>
#include <asm/uaccess.h>

#include <asm/fpu.h>
#include <asm/fpu_emulator.h>

#define SIGNALLING_NAN      0x7ff800007ff80000LL
#define SIGNALLING_NAN2008  0x7ff000007fa00000LL

extern unsigned int fpu_fcr31 __read_mostly;
extern unsigned int system_has_fpu __read_mostly;
static int nan2008 __read_mostly = -1;

static int __init setup_nan2008(char *str)
{
	get_option (&str, &nan2008);

	return 1;
}

__setup("nan2008=", setup_nan2008);

void fpu_emulator_init_fpu(struct task_struct *target)
{
	static int first = 1;
	int i;
	int j;

	if (first) {
		first = 0;
		printk("Algorithmics/MIPS FPU Emulator v1.5\n");
	}

	if (system_has_fpu)
		target->thread.fpu.fcr31 = fpu_fcr31;
	else if (nan2008 < 0) {
		if (test_thread_local_flags(LTIF_FPU_FR))
			target->thread.fpu.fcr31 = FPU_CSR_DEFAULT|FPU_CSR_MAC2008|FPU_CSR_ABS2008|FPU_CSR_NAN2008;
		else
			target->thread.fpu.fcr31 = FPU_CSR_DEFAULT;
	} else {
		if (nan2008)
			target->thread.fpu.fcr31 = FPU_CSR_DEFAULT|FPU_CSR_MAC2008|FPU_CSR_ABS2008|FPU_CSR_NAN2008;
		else
			target->thread.fpu.fcr31 = FPU_CSR_DEFAULT;
	}

	if (target->thread.fpu.fcr31 & FPU_CSR_NAN2008) {
		for (j = 0; j < (FPU_REG_WIDTH/64); j++)
			for (i = 0; i < NUM_FPU_REGS; i++)
				set_fpr64(&target->thread.fpu.fpr[i], j, SIGNALLING_NAN2008);
	} else
		for (i = 0; i < NUM_FPU_REGS; i++)
			set_fpr64(&target->thread.fpu.fpr[i], 0, SIGNALLING_NAN);
}
