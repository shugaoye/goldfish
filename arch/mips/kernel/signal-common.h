/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1991, 1992  Linus Torvalds
 * Copyright (C) 1994 - 2000  Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 */

#ifndef __SIGNAL_COMMON_H
#define __SIGNAL_COMMON_H

/* #define DEBUG_SIG */

#ifdef DEBUG_SIG
#  define DEBUGP(fmt, args...) printk("%s: " fmt, __func__, ##args)
#else
#  define DEBUGP(fmt, args...)
#endif

/*
 * Determine which stack to use..
 */
extern void __user *get_sigframe(struct k_sigaction *ka, struct pt_regs *regs,
				 size_t frame_size);
/* Check and clear pending FPU exceptions in saved CSR */
extern int fpcsr_pending(unsigned int __user *fpcsr);

static inline void __user *sc_to_extcontext(void __user *sc)
{
	struct ucontext __user *uc;

	/*
	 * We can just pretend the sigcontext is always embedded in a struct
	 * ucontext here, because the offset from sigcontext to extended
	 * context is the same in the struct sigframe case.
	 */
	uc = container_of(sc, struct ucontext, uc_mcontext);
	return &uc->uc_extcontext;
}

extern int save_extcontext(void __user *buf);
extern int restore_extcontext(void __user *buf);

/* Make sure we will not lose FPU ownership */
#ifdef CONFIG_PREEMPT
#define lock_fpu_owner()	preempt_disable()
#define unlock_fpu_owner()	preempt_enable()
#else
#define lock_fpu_owner()	pagefault_disable()
#define unlock_fpu_owner()	pagefault_enable()
#endif

#endif	/* __SIGNAL_COMMON_H */
