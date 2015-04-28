/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1991, 1992  Linus Torvalds
 * Copyright (C) 1994 - 2000  Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2014, Imagination Technologies Ltd.
 */

/*
 * Thread saved context copy to/from a signal context presumed to be on the
 * user stack, and therefore accessed with appropriate macros from uaccess.h.
 */
static int SUFFIX(copy_fp_to_sigcontext)(user_sigcontext_t __user *sc)
{
	int i;
	int err = 0;
	int inc = (!test_thread_local_flags(LTIF_FPU_FR)) ? 2 : 1;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |=
		    __put_user(get_fpr64(&current->thread.fpu.fpr[i], 0),
			       &sc->sc_fpregs[i]);
	}
	err |= __put_user(current->thread.fpu.fcr31, &sc->sc_fpc_csr);

	return err;
}

static int SUFFIX(copy_fp_from_sigcontext)(user_sigcontext_t __user *sc)
{
	int i;
	int err = 0;
	int inc = (!test_thread_local_flags(LTIF_FPU_FR)) ? 2 : 1;
	u64 fpr_val;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |= __get_user(fpr_val, &sc->sc_fpregs[i]);
		set_fpr64(&current->thread.fpu.fpr[i], 0, fpr_val);
	}
	err |= __get_user(current->thread.fpu.fcr31, &sc->sc_fpc_csr);

	return err;
}

/*
 * Helper routines
 */
static int SUFFIX(protected_save_fp_context)(user_sigcontext_t __user *sc)
{
	int err;
#ifndef CONFIG_EVA
	while (1) {
		lock_fpu_owner();
		if (is_fpu_owner()) {
			err = SUFFIX(save_fp_context)(sc);
			unlock_fpu_owner();
		} else {
			unlock_fpu_owner();
			err = SUFFIX(copy_fp_to_sigcontext)(sc);
		}
		if (likely(!err))
			break;
		/* touch the sigcontext and try again */
		err = __put_user(0, &sc->sc_fpregs[0]) |
			__put_user(0, &sc->sc_fpregs[31]) |
			__put_user(0, &sc->sc_fpc_csr);
		if (err)
			break;	/* really bad sigcontext */
	}
#else
	/*
	 * EVA does not have FPU EVA instructions so saving fpu context directly
	 * does not work.
	 */
	lose_fpu(1);
	err = SUFFIX(save_fp_context)(sc); /* this might fail */
#endif
	return err;
}

static int SUFFIX(protected_restore_fp_context)(user_sigcontext_t __user *sc)
{
	int err, tmp __maybe_unused;
#ifndef CONFIG_EVA
	while (1) {
		lock_fpu_owner();
		if (is_fpu_owner()) {
			err = SUFFIX(restore_fp_context)(sc);
			unlock_fpu_owner();
		} else {
			unlock_fpu_owner();
			err = SUFFIX(copy_fp_from_sigcontext)(sc);
		}
		if (likely(!err))
			break;
		/* touch the sigcontext and try again */
		err = __get_user(tmp, &sc->sc_fpregs[0]) |
			__get_user(tmp, &sc->sc_fpregs[31]) |
			__get_user(tmp, &sc->sc_fpc_csr);
		if (err)
			break;	/* really bad sigcontext */
	}
#else
	/*
	 * EVA does not have FPU EVA instructions so restoring fpu context
	 * directly does not work.
	 */
	lose_fpu(0);
	err = SUFFIX(restore_fp_context)(sc); /* this might fail */
#endif
	return err;
}

int SUFFIX(setup_sigcontext)(struct pt_regs *regs, user_sigcontext_t __user *sc)
{
	int err = 0;
	int i;
	unsigned int used_math;

	err |= __put_user(regs->cp0_epc, &sc->sc_pc);

	err |= __put_user(0, &sc->sc_regs[0]);
	for (i = 1; i < 32; i++)
		err |= __put_user(regs->regs[i], &sc->sc_regs[i]);

#ifdef CONFIG_CPU_HAS_SMARTMIPS
	err |= __put_user(regs->acx, &sc->sc_acx);
#endif
	err |= __put_user(regs->hi, &sc->sc_mdhi);
	err |= __put_user(regs->lo, &sc->sc_mdlo);
#ifndef CONFIG_CPU_MIPSR6
	if (cpu_has_dsp) {
		err |= __put_user(mfhi1(), &sc->sc_hi1);
		err |= __put_user(mflo1(), &sc->sc_lo1);
		err |= __put_user(mfhi2(), &sc->sc_hi2);
		err |= __put_user(mflo2(), &sc->sc_lo2);
		err |= __put_user(mfhi3(), &sc->sc_hi3);
		err |= __put_user(mflo3(), &sc->sc_lo3);
		err |= __put_user(rddsp(DSP_MASK), &sc->sc_dsp);
	}
#endif
	used_math = used_math() ? USED_FP : 0;

	if (used_math) {
		if (test_thread_local_flags(LTIF_FPU_FR)) {
			used_math |= USED_FR1;

			if (test_thread_local_flags(LTIF_FPU_FRE))
				used_math |= USED_HYBRID_FPRS;
		}
	}

	if (used_math) {
		/*
		 * Save FPU state to signal context. Signal handler
		 * will "inherit" current FPU state.
		 */
		err |= SUFFIX(protected_save_fp_context)(sc);
	}

	if (!err) {
		err = save_extcontext(sc_to_extcontext(sc));
		if (err > 0) {
			used_math |= USED_EXTCONTEXT;
			err = 0;
		}
	}

	err |= __put_user(used_math, &sc->sc_used_math);

	return err;
}

static int SUFFIX(check_and_restore_fp_context)(user_sigcontext_t __user *sc)
{
	int err, sig;

	err = sig = fpcsr_pending(&sc->sc_fpc_csr);
	if (err > 0)
		err = 0;
	err |= SUFFIX(protected_restore_fp_context)(sc);
	return err ?: sig;
}

int SUFFIX(restore_sigcontext)(struct pt_regs *regs, user_sigcontext_t __user *sc)
{
	unsigned int used_math;
#ifndef CONFIG_CPU_MIPSR6
	unsigned int treg;
#endif
	int err = 0;
	int i;

	/* Always make any pending restarted system calls return -EINTR */
	current_thread_info()->restart_block.fn = do_no_restart_syscall;

	err |= __get_user(regs->cp0_epc, &sc->sc_pc);

#ifdef CONFIG_CPU_HAS_SMARTMIPS
	err |= __get_user(regs->acx, &sc->sc_acx);
#endif
	err |= __get_user(regs->hi, &sc->sc_mdhi);
	err |= __get_user(regs->lo, &sc->sc_mdlo);
#ifndef CONFIG_CPU_MIPSR6
	if (cpu_has_dsp) {
		err |= __get_user(treg, &sc->sc_hi1); mthi1(treg);
		err |= __get_user(treg, &sc->sc_lo1); mtlo1(treg);
		err |= __get_user(treg, &sc->sc_hi2); mthi2(treg);
		err |= __get_user(treg, &sc->sc_lo2); mtlo2(treg);
		err |= __get_user(treg, &sc->sc_hi3); mthi3(treg);
		err |= __get_user(treg, &sc->sc_lo3); mtlo3(treg);
		err |= __get_user(treg, &sc->sc_dsp); wrdsp(treg, DSP_MASK);
	}
#endif
	for (i = 1; i < 32; i++)
		err |= __get_user(regs->regs[i], &sc->sc_regs[i]);

	err |= __get_user(used_math, &sc->sc_used_math);
	conditional_used_math(used_math & USED_FP);

	if (used_math & USED_FP) {
		/* restore fpu context if we have used it before */
		if (!err)
			err = SUFFIX(check_and_restore_fp_context)(sc);
	} else {
		/* signal handler may have used FPU.  Give it up. */
		lose_fpu(0);
	}

	if (used_math & USED_EXTCONTEXT)
		err |= restore_extcontext(sc_to_extcontext(sc));

	return err;
}
