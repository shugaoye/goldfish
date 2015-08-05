/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 95, 96, 97, 98, 99, 2003, 06 by Ralf Baechle
 * Copyright (C) 1996 by Paul M. Antoine
 * Copyright (C) 1999 Silicon Graphics
 * Kevin D. Kissell, kevink@mips.org and Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 2000 MIPS Technologies, Inc.
 */
#ifndef _ASM_SWITCH_TO_H
#define _ASM_SWITCH_TO_H

#include <asm/cpu-features.h>
#include <asm/watch.h>
#include <asm/dsp.h>
#include <asm/mmu_context.h>
#include <asm/msa.h>

struct task_struct;

enum {
	FP_SAVE_NONE	= 0,
	FP_SAVE_VECTOR	= -1,
	FP_SAVE_SCALAR	= 1,
};

/**
 * resume - resume execution of a task
 * @prev:	The task previously executed.
 * @next:	The task to begin executing.
 * @next_ti:	task_thread_info(next).
 * @fp_save:	Which, if any, FP context to save for prev.
 *
 * This function is used whilst scheduling to save the context of prev & load
 * the context of next. Returns prev.
 */
extern asmlinkage struct task_struct *resume(struct task_struct *prev,
		struct task_struct *next, struct thread_info *next_ti,
		s32 fp_save);

extern unsigned int ll_bit;
extern struct task_struct *ll_task;

#ifdef CONFIG_MIPS_MT_FPAFF

/*
 * Handle the scheduler resume end of FPU affinity management.	We do this
 * inline to try to keep the overhead down. If we have been forced to run on
 * a "CPU" with an FPU because of a previous high level of FP computation,
 * but did not actually use the FPU during the most recent time-slice (CU1
 * isn't set), we undo the restriction on cpus_allowed.
 *
 * We're not calling set_cpus_allowed() here, because we have no need to
 * force prompt migration - we're already switching the current CPU to a
 * different thread.
 */

#define __mips_mt_fpaff_switch_to(prev)					\
do {									\
	struct thread_info *__prev_ti = task_thread_info(prev);		\
									\
	if (cpu_has_fpu &&						\
	    test_ti_thread_flag(__prev_ti, TIF_FPUBOUND) &&		\
	    (!(KSTK_STATUS(prev) & ST0_CU1))) {				\
		clear_ti_thread_flag(__prev_ti, TIF_FPUBOUND);		\
		prev->cpus_allowed = prev->thread.user_cpus_allowed;	\
	}								\
	next->thread.emulated_fp = 0;					\
} while(0)

#else
#define __mips_mt_fpaff_switch_to(prev) do { (void) (prev); } while (0)
#endif

#ifdef CONFIG_CPU_MIPSR6
#define __clear_ll_bit()                                                \
do {									\
	write_c0_lladdr(0);                                             \
} while (0)
#else
#define __clear_ll_bit()                                                \
do {									\
	if (!__builtin_constant_p(cpu_has_llsc) || !cpu_has_llsc)	\
		ll_bit = 0;						\
} while (0)
#endif

#define switch_to(prev, next, last)					\
do {									\
	s32 __fpsave = FP_SAVE_NONE;					\
	__mips_mt_fpaff_switch_to(prev);				\
	if (cpu_has_dsp)						\
		__save_dsp(prev);					\
	__clear_ll_bit();                                               \
	if (test_and_clear_tsk_thread_flag(prev, TIF_USEDFPU))		\
		__fpsave = FP_SAVE_SCALAR;				\
	if (test_and_clear_tsk_thread_flag(prev, TIF_USEDMSA))		\
		__fpsave = FP_SAVE_VECTOR;				\
	(last) = resume(prev, next, task_thread_info(next), __fpsave);	\
} while (0)

static inline void flush_vdso_page(void)
{
	int cpu = raw_smp_processor_id();

	if (current->mm && cpu_context(cpu, current->mm) &&
	    (current->mm->context.vdso_page[cpu] != current_thread_info()->vdso_page) &&
	    (current->mm->context.vdso_asid[cpu] == cpu_asid(cpu, current->mm))) {
		local_flush_tlb_page(current->mm->mmap, (unsigned long)current->mm->context.vdso);
		current->mm->context.vdso_asid[cpu] = 0;
	}
}

#define finish_arch_switch(prev)					\
do {									\
	if (cpu_has_dsp)						\
		__restore_dsp(current);					\
	if (cpu_has_userlocal)						\
		write_c0_userlocal(current_thread_info()->tp_value);	\
	flush_vdso_page();                                              \
	__restore_watch();						\
} while (0)

#endif /* _ASM_SWITCH_TO_H */
