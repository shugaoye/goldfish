/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 1996, 1997, 2000, 2001, 05 by Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2001 MIPS Technologies, Inc.
 */
#include <linux/capability.h>
#include <linux/errno.h>
#include <linux/linkage.h>
#include <linux/fs.h>
#include <linux/smp.h>
#include <linux/ptrace.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/utsname.h>
#include <linux/unistd.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/compiler.h>
#include <linux/ipc.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/elf.h>
#include <linux/prctl.h>

#include <asm/asm.h>
#include <asm/branch.h>
#include <asm/cachectl.h>
#include <asm/cacheflush.h>
#include <asm/asm-offsets.h>
#include <asm/signal.h>
#include <asm/sim.h>
#include <asm/shmparam.h>
#include <asm/sysmips.h>
#include <asm/uaccess.h>
#include <asm/switch_to.h>
#include <asm/fpu.h>

extern unsigned int system_has_fpu;
extern unsigned int global_fpu_id;
/*
 * For historic reasons the pipe(2) syscall on MIPS has an unusual calling
 * convention.	It returns results in registers $v0 / $v1 which means there
 * is no need for it to do verify the validity of a userspace pointer
 * argument.  Historically that used to be expensive in Linux.	These days
 * the performance advantage is negligible.
 */
asmlinkage int sysm_pipe(void)
{
	int fd[2];
	int error = do_pipe_flags(fd, 0);
	if (error)
		return error;
	current_pt_regs()->regs[3] = fd[1];
	return fd[0];
}

SYSCALL_DEFINE6(mips_mmap, unsigned long, addr, unsigned long, len,
	unsigned long, prot, unsigned long, flags, unsigned long,
	fd, off_t, offset)
{
	unsigned long result;

	result = -EINVAL;
	if (offset & ~PAGE_MASK)
		goto out;

	result = sys_mmap_pgoff(addr, len, prot, flags, fd, offset >> PAGE_SHIFT);

out:
	return result;
}

SYSCALL_DEFINE6(mips_mmap2, unsigned long, addr, unsigned long, len,
	unsigned long, prot, unsigned long, flags, unsigned long, fd,
	unsigned long, pgoff)
{
	if (pgoff & (~PAGE_MASK >> 12))
		return -EINVAL;

	return sys_mmap_pgoff(addr, len, prot, flags, fd, pgoff >> (PAGE_SHIFT-12));
}

save_static_function(sys_fork);
save_static_function(sys_clone);

SYSCALL_DEFINE1(set_thread_area, unsigned long, addr)
{
	struct thread_info *ti = task_thread_info(current);

	ti->tp_value = addr;
	if (cpu_has_userlocal)
		write_c0_userlocal(addr);

	return 0;
}

static inline int mips_atomic_set(unsigned long addr, unsigned long new)
{
	unsigned long old, tmp;
	struct pt_regs *regs;
	unsigned int err;

	if (unlikely(addr & 3))
		return -EINVAL;

	if (unlikely(!access_ok(VERIFY_WRITE, addr, 4)))
		return -EINVAL;

	if (cpu_has_llsc && R10000_LLSC_WAR) {
		__asm__ __volatile__ (
		"	.set	mips3					\n"
		"	li	%[err], 0				\n"
		"1:	ll	%[old], (%[addr])			\n"
		"	move	%[tmp], %[new]				\n"
		"2:	sc	%[tmp], (%[addr])			\n"
		"	beqzl	%[tmp], 1b				\n"
		"3:							\n"
		"	.section .fixup,\"ax\"				\n"
		"4:	li	%[err], %[efault]			\n"
		"	j	3b					\n"
		"	.previous					\n"
		"	.section __ex_table,\"a\"			\n"
		"	"STR(PTR)"	1b, 4b				\n"
		"	"STR(PTR)"	2b, 4b				\n"
		"	.previous					\n"
		"	.set	mips0					\n"
		: [old] "=&r" (old),
		  [err] "=&r" (err),
		  [tmp] "=&r" (tmp)
		: [addr] "r" (addr),
		  [new] "r" (new),
		  [efault] "i" (-EFAULT)
		: "memory");
	} else if (cpu_has_llsc) {
		__asm__ __volatile__ (
#ifdef CONFIG_CPU_MIPSR6
		"       .set    mips64r6                                \n"
#else
		"       .set    mips3                                   \n"
#endif
		"       li      %[err], 0                               \n"
		"1:	ll	%[old], (%[addr])			\n"
		"	move	%[tmp], %[new]				\n"
		"2:	sc	%[tmp], (%[addr])			\n"
		"	bnez	%[tmp], 4f				\n"
		"3:							\n"
		"	.subsection 2					\n"
		"4:	b	1b					\n"
		"	.previous					\n"
		"							\n"
		"	.section .fixup,\"ax\"				\n"
		"5:	li	%[err], %[efault]			\n"
		"	j	3b					\n"
		"	.previous					\n"
		"	.section __ex_table,\"a\"			\n"
		"	"STR(PTR)"	1b, 5b				\n"
		"	"STR(PTR)"	2b, 5b				\n"
		"	.previous					\n"
		"	.set	mips0					\n"
		: [old] "=&r" (old),
		  [err] "=&r" (err),
		  [tmp] "=&r" (tmp)
		: [addr] "r" (addr),
		  [new] "r" (new),
		  [efault] "i" (-EFAULT)
		: "memory");
	} else {
		do {
			preempt_disable();
			ll_bit = 1;
			ll_task = current;
			preempt_enable();

			err = __get_user(old, (unsigned int *) addr);
			err |= __put_user(new, (unsigned int *) addr);
			if (err)
				break;
			rmb();
		} while (!ll_bit);
	}

	if (unlikely(err))
		return err;

	regs = current_pt_regs();
	regs->regs[2] = old;
	regs->regs[7] = 0;	/* No error */

	/*
	 * Don't let your children do this ...
	 */
	__asm__ __volatile__(
	"	move	$29, %0						\n"
	"	j	syscall_exit					\n"
	: /* no outputs */
	: "r" (regs));

	/* unreached.  Honestly.  */
	unreachable();
}

asmlinkage void mips_lose_fpu(void)
{
	preempt_disable();
	clear_thread_flag(TIF_FPU_LOSE_REQUEST);
	lose_fpu_inatomic(1);
	preempt_enable_no_resched();
}

void mips_switch_fpu_mode(void *info)
{
	struct mm_struct *mm = info;

	if ((current->mm == mm) && (is_fpu_owner() || is_msa_enabled()))
		set_thread_flag(TIF_FPU_LOSE_REQUEST);
}

unsigned int mips_fpu_prctl(unsigned long type, unsigned long param)
{
	register unsigned long val;
	register unsigned long mask;
	register unsigned long *addr;

	switch (type) {
	case PR_SET_FP_MODE:
		if (param & ~(PR_FP_MODE_FR|PR_FP_MODE_FRE))
			return -EINVAL;

#ifndef CONFIG_MIPS_INCOMPATIBLE_ARCH_EMULATION
		if (system_has_fpu) {
			if ((param & PR_FP_MODE_FRE) && !cpu_has_fre)
				return -EOPNOTSUPP;
			if ((param & PR_FP_MODE_FR) && !(global_fpu_id & MIPS_FPIR_F64))
				return -EOPNOTSUPP;
			if (!(param & PR_FP_MODE_FR)) {
				unsigned int res;
				unsigned int status;

				/* test FPU with FR0 capability */
				local_irq_disable();
				status = change_c0_status(ST0_FR|ST0_CU1, ST0_CU1);
				enable_fpu_hazard();
				res = read_c0_status();
				write_c0_status(status);
				disable_fpu_hazard();
				local_irq_enable();
				if (res & ST0_FR)
					return -EOPNOTSUPP;
			}
		}
#endif
		val = (param & PR_FP_MODE_FR)? LTIF_FPU_FR : 0;
		val |= (param & PR_FP_MODE_FRE)? LTIF_FPU_FRE : 0;
		mask = ~(LTIF_FPU_FR|LTIF_FPU_FRE);
		preempt_disable();
		addr = &(current->mm->context.thread_flags);
		__asm__ __volatile__(
			".set push                      \n"
			".set noreorder                 \n"
			".set noat                      \n"
#ifdef CONFIG_64BIT
			"1: lld     $1, 0(%0)           \n"
			"   and     $1, $1, %1          \n"
			"   or      $1, $1, %2          \n"
			"   scd     $1, 0(%0)           \n"
#else
			"1: ll      $1, 0(%0)           \n"
			"   and     $1, $1, %1          \n"
			"   or      $1, $1, %2          \n"
			"   sc      $1, 0(%0)           \n"
#endif
			"   beqz    $1, 1b              \n"
			"     nop                       \n"
			".set pop                       \n"
			:
			: "r"(addr), "r"(mask), "r"(val)
			: "memory");
		smp_llsc_mb();
		/* send a "barrier" for FPU mode - force other CPUs to lose FPU */
		if (atomic_read(&current->mm->mm_users) != 1)
			smp_call_function(mips_switch_fpu_mode, (void *)(current->mm), 1);
		lose_fpu(1);
		preempt_enable();
		break;

	case PR_GET_FP_MODE:
		if (unlikely(param & 3))
			return -EINVAL;

		if (unlikely(!access_ok(VERIFY_WRITE, param, 4)))
			return -EINVAL;

		val = (current_thread_info()->local_flags & LTIF_FPU_FR)? PR_FP_MODE_FR : 0;
		val |= (current_thread_info()->local_flags & LTIF_FPU_FRE)? PR_FP_MODE_FRE : 0;
		if (put_user(val, (unsigned long *)param))
			return -EINVAL;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

long mips_get_process_fp_mode(struct task_struct *task)
{
	unsigned long val;

	val = (((struct thread_info *)task_stack_page(task))->local_flags & LTIF_FPU_FR)? PR_FP_MODE_FR : 0;
	val |= (((struct thread_info *)task_stack_page(task))->local_flags & LTIF_FPU_FRE)? PR_FP_MODE_FRE : 0;

	return val;
}

long mips_set_process_fp_mode(struct task_struct *task,
				    unsigned long value)
{
	return mips_fpu_prctl(PR_SET_FP_MODE, value);
}

SYSCALL_DEFINE3(sysmips, long, cmd, long, arg1, long, arg2)
{
	switch (cmd) {
	case MIPS_ATOMIC_SET:
		return mips_atomic_set(arg1, arg2);

	case MIPS_FIXADE:
		if (arg1 & ~3)
			return -EINVAL;

		if (arg1 & 1)
			set_thread_flag(TIF_FIXADE);
		else
			clear_thread_flag(TIF_FIXADE);
		if (arg1 & 2)
			set_thread_flag(TIF_LOGADE);
		else
			clear_thread_flag(TIF_LOGADE);

		return 0;

	case FLUSH_CACHE:
		__flush_cache_all();
		return 0;

	case MIPS_FPU_PRCTL:
		return mips_fpu_prctl(arg1, arg2);
	}

	return -EINVAL;
}

/*
 * No implemented yet ...
 */
SYSCALL_DEFINE3(cachectl, char *, addr, int, nbytes, int, op)
{
	return -ENOSYS;
}

/*
 * If we ever come here the user sp is bad.  Zap the process right away.
 * Due to the bad stack signaling wouldn't work.
 */
asmlinkage void bad_stack(void)
{
	do_exit(SIGSEGV);
}
