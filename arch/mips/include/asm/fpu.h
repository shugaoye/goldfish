/*
 * Copyright (C) 2002 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef _ASM_FPU_H
#define _ASM_FPU_H

#include <linux/sched.h>
#include <linux/thread_info.h>
#include <linux/bitops.h>

#include <asm/mipsregs.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/hazards.h>
#include <asm/processor.h>
#include <asm/current.h>
#include <asm/msa.h>

#ifdef CONFIG_MIPS_MT_FPAFF
#include <asm/mips_mt.h>
#endif

struct sigcontext;
struct sigcontext32;

extern void fpu_emulator_init_fpu(struct task_struct *target);
extern int _init_fpu(void);
extern void _save_fp(struct task_struct *);
extern void _restore_fp(struct task_struct *);

/*
 * This macro is used only to obtain FIR from FPU and it seems
 * like a BUG in 34K with single FPU affinity to VPE0.
 */
#define __enable_fpu()                                                  \
do {									\
	set_c0_status(ST0_CU1);						\
	enable_fpu_hazard();						\
} while (0)

#define clear_fpu_owner()	clear_thread_flag(TIF_USEDFPU)

static inline int __is_fpu_owner(void)
{
	return test_thread_flag(TIF_USEDFPU);
}

static inline int is_fpu_owner(void)
{
	return cpu_has_fpu && __is_fpu_owner();
}

static inline void thread_fpu_flags_update(void)
{
	if (current->mm)
		change_thread_local_flags(LTIF_FPU_FR|LTIF_FPU_FRE,
			current->mm->context.thread_flags & (LTIF_FPU_FR|LTIF_FPU_FRE));
}

static inline int __own_fpu(void)
{
	int ret = 0;

#if defined(CONFIG_CPU_MIPS32_R2) || defined(CONFIG_CPU_MIPS32_R6) || defined(CONFIG_CPU_MIPS64)
	u32 status;

	thread_fpu_flags_update();

	if (!test_thread_local_flags(LTIF_FPU_FR)) {
		status = change_c0_status(ST0_CU1|ST0_FR,ST0_CU1);
		enable_fpu_hazard();
		if (read_c0_status() & ST0_FR) {
			if (cpu_has_fre) {
				set_c0_config5(MIPS_CONF5_FRE);
				back_to_back_c0_hazard();
				KSTK_STATUS(current) |= ST0_CU1|ST0_FR;
			} else {
				write_c0_status(status & ~ST0_CU1);
				disable_fpu_hazard();
				return(SIGFPE);
			}
		} else {
			if (test_thread_local_flags(LTIF_FPU_FRE)) {
				if (cpu_has_fre) {
					set_c0_config5(MIPS_CONF5_FRE);
					back_to_back_c0_hazard();
				} else {
					write_c0_status(status & ~ST0_CU1);
					disable_fpu_hazard();
					return(SIGFPE);
				}
			}
			KSTK_STATUS(current) = (KSTK_STATUS(current) & ~ST0_FR) | ST0_CU1;
		}
	} else {
		status = set_c0_status(ST0_CU1|ST0_FR);
		enable_fpu_hazard();
		if (!(read_c0_status() & ST0_FR)) {
			write_c0_status(status & ~ST0_CU1);
			disable_fpu_hazard();
			return(SIGFPE);
		}
		if (cpu_has_fre) {
			if (test_thread_local_flags(LTIF_FPU_FRE)) {
				set_c0_config5(MIPS_CONF5_FRE);
				back_to_back_c0_hazard();
			} else {
				clear_c0_config5(MIPS_CONF5_FRE);
				back_to_back_c0_hazard();
			}
		} else if (test_thread_local_flags(LTIF_FPU_FRE)) {
			write_c0_status(status & ~ST0_CU1);
			disable_fpu_hazard();
			return(SIGFPE);
		}
		KSTK_STATUS(current) |= ST0_CU1|ST0_FR;
	}
#else
	thread_fpu_flags_update();

	if (test_thread_local_flags(LTIF_FPU_FR))
		return SIGFPE;  /* core has no 64bit FPU, so ... */

	set_c0_status(ST0_CU1);
	KSTK_STATUS(current) |= ST0_CU1;
	enable_fpu_hazard();
#endif
	set_thread_flag(TIF_USEDFPU);
	return ret;
}

static inline int own_fpu_inatomic(int restore)
{
	int ret = 0;

	if (cpu_has_fpu && !__is_fpu_owner()) {
		ret =__own_fpu();
		if (restore && !ret)
			_restore_fp(current);
	}
	return ret;
}

static inline int own_fpu(int restore)
{
	int ret;

	preempt_disable();
	ret = own_fpu_inatomic(restore);
	preempt_enable();

	return ret;
}

static inline unsigned int fpu_get_fcr31(void)
{
	unsigned int cp1status = read_32bit_cp1_register(CP1_STATUS);

#ifdef CONFIG_CPU_MIPSR6
	cp1status |= (current->thread.fpu.fcr31 &
		(FPU_CSR_COND0|FPU_CSR_COND1|FPU_CSR_COND2|FPU_CSR_COND3|
		 FPU_CSR_COND4|FPU_CSR_COND5|FPU_CSR_COND6|FPU_CSR_COND7));
#endif
	return cp1status;
}

static inline void lose_fpu_inatomic(int save)
{
	if (is_msa_enabled()) {
		if (save) {
			save_msa(current);
			current->thread.fpu.fcr31 = fpu_get_fcr31();
		}
		disable_msa();
		clear_thread_flag(TIF_USEDMSA);
	} else if (is_fpu_owner()) {
		if (save)
			_save_fp(current);
	}
	clear_c0_status(ST0_CU1);
	disable_fpu_hazard();
	KSTK_STATUS(current) &= ~ST0_CU1;
	clear_thread_flag(TIF_USEDFPU);
}

static inline void lose_fpu(int save)
{
	preempt_disable();
	lose_fpu_inatomic(save);
	preempt_enable();
}

static inline int init_fpu(void)
{
	int ret = 0;

	preempt_disable();
	if (cpu_has_fpu && !(ret = __own_fpu())) {
		if (cpu_has_fre) {
			unsigned int config5 = clear_c0_config5(MIPS_CONF5_FRE);
			back_to_back_c0_hazard();
			_init_fpu();
			write_c0_config5(config5);
			back_to_back_c0_hazard();
		} else
			_init_fpu();
	} else
		fpu_emulator_init_fpu(current);

	preempt_enable();

	set_used_math();

	return ret;
}

static inline void init_fp_ctx(struct task_struct *target)
{
	/* If FP has been used then the target already has context */
	if (used_math())
		return;

	fpu_emulator_init_fpu(target);

	/*
	 * Record that the target has "used" math, such that the context
	 * just initialised, and any modifications made by the caller,
	 * aren't discarded.
	 */
	set_used_math();
}

static inline void save_fp(struct task_struct *tsk)
{
	if (cpu_has_fpu)
		_save_fp(tsk);
}

static inline void restore_fp(struct task_struct *tsk)
{
	if (cpu_has_fpu)
		_restore_fp(tsk);
}

static inline union fpureg *get_fpu_regs(struct task_struct *tsk)
{
	if (tsk == current) {
		preempt_disable();
		if (is_fpu_owner())
			_save_fp(current);
		preempt_enable();
	}

	return tsk->thread.fpu.fpr;
}

#endif /* _ASM_FPU_H */
