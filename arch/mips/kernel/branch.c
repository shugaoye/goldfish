/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996, 97, 2000, 2001 by Ralf Baechle
 * Copyright (C) 2001 MIPS Technologies, Inc.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <asm/branch.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/fpu.h>
#include <asm/fpu_emulator.h>
#include <asm/inst.h>
#include <asm/ptrace.h>
#include <asm/uaccess.h>

/*
 * Calculate and return exception PC in case of branch delay slot
 * for microMIPS and MIPS16e. It does not clear the ISA mode bit.
 */
int __isa_exception_epc(struct pt_regs *regs)
{
	unsigned short inst;
	long epc = regs->cp0_epc;

	/* Calculate exception PC in branch delay slot. */
	if (__get_user(inst, (u16 __user *) msk_isa16_mode(epc))) {
		/* This should never happen because delay slot was checked. */
		force_sig(SIGSEGV, current);
		return epc;
	}
	if (cpu_has_mips16) {
		if (((union mips16e_instruction)inst).ri.opcode
				== MIPS16e_jal_op)
			epc += 4;
		else
			epc += 2;
	} else if (mm_insn_16bit(inst))
		epc += 2;
	else
		epc += 4;

	return epc;
}

/*
 * Compute return address and emulate branch in microMIPS mode after an
 * exception only. It does not handle compact branches/jumps and cannot
 * be used in interrupt context. (Compact branches/jumps do not cause
 * exceptions.)
 */
int __microMIPS_compute_return_epc(struct pt_regs *regs)
{
	u16 __user *pc16;
	u16 halfword;
	unsigned int word;
	unsigned long contpc;
	struct mm_decoded_insn mminsn = { 0 };

	mminsn.micro_mips_mode = 1;

	/* This load never faults. */
	pc16 = (unsigned short __user *)msk_isa16_mode(regs->cp0_epc);
	__get_user(halfword, pc16);
	pc16++;
	contpc = regs->cp0_epc + 2;
	word = ((unsigned int)halfword << 16);
	mminsn.pc_inc = 2;

	if (!mm_insn_16bit(halfword)) {
		__get_user(halfword, pc16);
		pc16++;
		contpc = regs->cp0_epc + 4;
		mminsn.pc_inc = 4;
		word |= halfword;
	}
	mminsn.insn = word;

	if (get_user(halfword, pc16))
		goto sigsegv;
	mminsn.next_pc_inc = 2;
	word = ((unsigned int)halfword << 16);

	if (!mm_insn_16bit(halfword)) {
		pc16++;
		if (get_user(halfword, pc16))
			goto sigsegv;
		mminsn.next_pc_inc = 4;
		word |= halfword;
	}
	mminsn.next_insn = word;

	mm_isBranchInstr(regs, mminsn, &contpc);

	regs->cp0_epc = contpc;

	return 0;

sigsegv:
	force_sig(SIGSEGV, current);
	return -EFAULT;
}

/*
 * Compute return address and emulate branch in MIPS16e mode after an
 * exception only. It does not handle compact branches/jumps and cannot
 * be used in interrupt context. (Compact branches/jumps do not cause
 * exceptions.)
 */
int __MIPS16e_compute_return_epc(struct pt_regs *regs)
{
	u16 __user *addr;
	union mips16e_instruction inst;
	u16 inst2;
	u32 fullinst;
	long epc;

	epc = regs->cp0_epc;

	/* Read the instruction. */
	addr = (u16 __user *)msk_isa16_mode(epc);
	if (__get_user(inst.full, addr)) {
		force_sig(SIGSEGV, current);
		return -EFAULT;
	}

	switch (inst.ri.opcode) {
	case MIPS16e_extend_op:
		regs->cp0_epc += 4;
		return 0;

		/*
		 *  JAL and JALX in MIPS16e mode
		 */
	case MIPS16e_jal_op:
		addr += 1;
		if (__get_user(inst2, addr)) {
			force_sig(SIGSEGV, current);
			return -EFAULT;
		}
		fullinst = ((unsigned)inst.full << 16) | inst2;
		regs->regs[31] = epc + 6;
		epc += 4;
		epc >>= 28;
		epc <<= 28;
		/*
		 * JAL:5 X:1 TARGET[20-16]:5 TARGET[25:21]:5 TARGET[15:0]:16
		 *
		 * ......TARGET[15:0].................TARGET[20:16]...........
		 * ......TARGET[25:21]
		 */
		epc |=
		    ((fullinst & 0xffff) << 2) | ((fullinst & 0x3e00000) >> 3) |
		    ((fullinst & 0x1f0000) << 7);
		if (!inst.jal.x)
			set_isa16_mode(epc);	/* Set ISA mode bit. */
		regs->cp0_epc = epc;
		return 0;

		/*
		 *  J(AL)R(C)
		 */
	case MIPS16e_rr_op:
		if (inst.rr.func == MIPS16e_jr_func) {

			if (inst.rr.ra)
				regs->cp0_epc = regs->regs[31];
			else
				regs->cp0_epc =
				    regs->regs[reg16to32[inst.rr.rx]];

			if (inst.rr.l) {
				if (inst.rr.nd)
					regs->regs[31] = epc + 2;
				else
					regs->regs[31] = epc + 4;
			}
			return 0;
		}
		break;
	}

	/*
	 * All other cases have no branch delay slot and are 16-bits.
	 * Branches do not cause an exception.
	 */
	regs->cp0_epc += 2;

	return 0;
}

/**
 * __compute_return_epc_for_insn - Computes the return address and do emulate
 *				    branch simulation, if required.
 *
 * @regs:	Pointer to pt_regs
 * @insn:	branch instruction to decode
 * @returns:	-EFAULT on error and forces SIGBUS, and on success
 *		returns 0 or BRANCH_LIKELY_TAKEN as appropriate after
 *		evaluating the branch.
 */
/*  Note on R6 compact branches:
 *      Compact branches doesn't do exception (besides BC1EQZ/BC1NEZ)
 *      and doesn't execute instruction in Forbidden Slot if branch is
 *      to be taken. It means that return EPC for them can be safely set
 *      to EPC + 8 because it is the only case to get a BD precise exception
 *      doing instruction in Forbidden Slot while no branch.
 *
 *      Unconditional compact jump/branches added for full picture
 *      (not doing BD precise exception, actually).
 */
int __compute_return_epc_for_insn(struct pt_regs *regs,
				   union mips_instruction insn)
{
	unsigned int bit;
	long epc = regs->cp0_epc;
	int ret = 0;
	unsigned int fcr31;
#ifdef CONFIG_CPU_MIPSR6
	int reg;
#else
	unsigned dspcontrol;
#endif

	switch (insn.i_format.opcode) {
	/*
	 * jr and jalr are in r_format format.
	 */
	case spec_op:
		switch (insn.r_format.func) {
		case jalr_op:
			regs->regs[insn.r_format.rd] = epc + 8;
			regs->cp0_epc = regs->regs[insn.r_format.rs];
			break;
		case jr_op:
#ifdef CONFIG_CPU_MIPSR6
			if (!mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
			regs->cp0_epc = regs->regs[insn.r_format.rs];
			break;
		}
		break;

	/*
	 * This group contains:
	 * bltz_op, bgez_op, bltzl_op, bgezl_op,
	 * bltzal_op, bgezal_op, bltzall_op, bgezall_op.
	 */
	case bcond_op:
		switch (insn.i_format.rt) {
		case bltzl_op:
#ifdef CONFIG_CPU_MIPSR6
			if (!mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
		case bltz_op:
			if ((long)regs->regs[insn.i_format.rs] < 0) {
				epc = epc + 4 + (insn.i_format.simmediate << 2);
				if (insn.i_format.rt == bltzl_op)
					ret = BRANCH_LIKELY_TAKEN;
			} else
				epc += 8;
			regs->cp0_epc = epc;
			break;

		case bgezl_op:
#ifdef CONFIG_CPU_MIPSR6
			if (!mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
		case bgez_op:
			if ((long)regs->regs[insn.i_format.rs] >= 0) {
				epc = epc + 4 + (insn.i_format.simmediate << 2);
				if (insn.i_format.rt == bgezl_op)
					ret = BRANCH_LIKELY_TAKEN;
			} else
				epc += 8;
			regs->cp0_epc = epc;
			break;

		case bltzall_op:
#ifdef CONFIG_CPU_MIPSR6
			if (!mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
		case bltzal_op:
#ifdef CONFIG_CPU_MIPSR6
			/* MIPSR6: nal == bltzal $0 */
			if (insn.i_format.rs && !mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
			regs->regs[31] = epc + 8;
			if ((long)regs->regs[insn.i_format.rs] < 0) {
				epc = epc + 4 + (insn.i_format.simmediate << 2);
				if (insn.i_format.rt == bltzall_op)
					ret = BRANCH_LIKELY_TAKEN;
			} else
				epc += 8;
			regs->cp0_epc = epc;
			break;

		case bgezall_op:
#ifdef CONFIG_CPU_MIPSR6
			if (!mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
		case bgezal_op:
#ifdef CONFIG_CPU_MIPSR6
			/* MIPSR6: bal == bgezal $0 */
			if (insn.i_format.rs && !mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
			regs->regs[31] = epc + 8;
			if ((long)regs->regs[insn.i_format.rs] >= 0) {
				epc = epc + 4 + (insn.i_format.simmediate << 2);
				if (insn.i_format.rt == bgezall_op)
					ret = BRANCH_LIKELY_TAKEN;
			} else
				epc += 8;
			regs->cp0_epc = epc;
			break;

#ifndef CONFIG_CPU_MIPSR6
		case bposge32_op:
			if (!cpu_has_dsp)
				goto sigill;

			dspcontrol = rddsp(0x01);

			if (dspcontrol >= 32) {
				epc = epc + 4 + (insn.i_format.simmediate << 2);
			} else
				epc += 8;
			regs->cp0_epc = epc;
			break;
#endif
		}
		break;

	/*
	 * These are unconditional and in j_format.
	 */
	case jal_op:
		regs->regs[31] = regs->cp0_epc + 8;
	case j_op:
		epc += 4;
		epc >>= 28;
		epc <<= 28;
		epc |= (insn.j_format.target << 2);
		regs->cp0_epc = epc;
		if (insn.i_format.opcode == jalx_op)
			set_isa16_mode(regs->cp0_epc);
		break;

	/*
	 * These are conditional and in i_format.
	 */
	case beql_op:
#ifdef CONFIG_CPU_MIPSR6
		if (!mipsr2_emulation) {
			ret = -SIGILL;
			break;
		}
#endif
	case beq_op:
		if (regs->regs[insn.i_format.rs] ==
		    regs->regs[insn.i_format.rt]) {
			epc = epc + 4 + (insn.i_format.simmediate << 2);
			if (insn.i_format.opcode == beql_op)
				ret = BRANCH_LIKELY_TAKEN;
		} else
			epc += 8;
		regs->cp0_epc = epc;
		break;

	case bnel_op:
#ifdef CONFIG_CPU_MIPSR6
		if (!mipsr2_emulation) {
			ret = -SIGILL;
			break;
		}
#endif
	case bne_op:
		if (regs->regs[insn.i_format.rs] !=
		    regs->regs[insn.i_format.rt]) {
			epc = epc + 4 + (insn.i_format.simmediate << 2);
			if (insn.i_format.opcode == bnel_op)
				ret = BRANCH_LIKELY_TAKEN;
		} else
			epc += 8;
		regs->cp0_epc = epc;
		break;

	case blez_op: /* not really i_format */
	case blezl_op:
#ifdef CONFIG_CPU_MIPSR6
		/*
		 *  Compact branches: (blez:)  blezalc, bgezalc, bgeuc
		 *  Compact branches: (blezl:) blezc, bgezc, bgec
		 */
		if (insn.i_format.rt) {
			if (insn.i_format.opcode == blez_op)
				if ((insn.i_format.rs == insn.i_format.rt) ||
				    !insn.i_format.rs)  /* blezalc, bgezalc */
					regs->regs[31] = epc + 4;
			epc += 8;
			regs->cp0_epc = epc;
			break;
		}

		if ((insn.i_format.opcode != blez_op) && !mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
		/* rt field assumed to be zero */
		if ((long)regs->regs[insn.i_format.rs] <= 0) {
			epc = epc + 4 + (insn.i_format.simmediate << 2);
			if (insn.i_format.opcode == blezl_op)
				ret = BRANCH_LIKELY_TAKEN;
		} else
			epc += 8;
		regs->cp0_epc = epc;
		break;

	case bgtz_op:
	case bgtzl_op:
#ifdef CONFIG_CPU_MIPSR6
		/*
		 *  Compact branches: (bgtz:)  bltzalc, bgtzalc, bltuc
		 *  Compact branches: (bgtzl:) bltc, bltzc, bgtzc
		 */
		if (insn.i_format.rt) {
			if (insn.i_format.opcode == bgtz_op)
				if ((insn.i_format.rs == insn.i_format.rt) ||
				    !insn.i_format.rs)   /* bltzalc, bgtzalc */
					regs->regs[31] = epc + 4;
			epc += 8;
			regs->cp0_epc = epc;
			break;
		}

		if ((insn.i_format.opcode != bgtz_op) && !mipsr2_emulation) {
				ret = -SIGILL;
				break;
			}
#endif
		/* rt field assumed to be zero */
		if ((long)regs->regs[insn.i_format.rs] > 0) {
			epc = epc + 4 + (insn.i_format.simmediate << 2);
			if (insn.i_format.opcode == bgtzl_op)
				ret = BRANCH_LIKELY_TAKEN;
		} else
			epc += 8;
		regs->cp0_epc = epc;
		break;

#ifdef CONFIG_CPU_MIPSR6
	case cbcond0_op:
		/*
		 *  Compact branches: bovc, beqc, beqzalc
		 */

		/* fall through */
	case cbcond1_op:
		/*
		 *  Compact branches: bnvc, bnec, bnezalc
		 */
		if (insn.i_format.rt && !insn.i_format.rs)  /* beqzalc/bnezalc */
			regs->regs[31] = epc + 4;
		epc += 8;
		regs->cp0_epc = epc;

		break;
#endif

	/*
	 * And now the FPA/cp1 branch instructions.
	 */
	case cop1_op:
#ifdef CONFIG_CPU_MIPSR6
		if ((insn.i_format.rs == bc1eqz_op) ||
		    (insn.i_format.rs == bc1nez_op)) {

			if (!used_math()) {     /* First time FPU user.  */
				ret = init_fpu();
				if (ret && raw_cpu_has_fpu && !mipsr2_emulation) {
					ret = -ret;
					break;
				}
				ret = 0;
			}
			lose_fpu(1);    /* Save FPU state for the emulator. */
			reg = insn.i_format.rt;
			bit = 0;
			switch (insn.i_format.rs) {
			case bc1eqz_op:
				if (!(get_fpr64(&current->thread.fpu.fpr[reg], 0) & (__u64)0x1))
					bit = 1;
				break;
			case bc1nez_op:
				if (get_fpr64(&current->thread.fpu.fpr[reg], 0) & (__u64)0x1)
					bit = 1;
				break;
			}
			own_fpu(1);     /* Restore FPU state. */
			if (bit)
				epc = epc + 4 + (insn.i_format.simmediate << 2);
			else
				epc += 8;
			regs->cp0_epc = epc;

			break;
		}

		if (!mipsr2_emulation) {
			ret = -SIGILL;
			break;
		}
#endif
		preempt_disable();
		if (is_fpu_owner())
			fcr31 = fpu_get_fcr31();
		else
			fcr31 = current->thread.fpu.fcr31;
		preempt_enable();

		bit = (insn.i_format.rt >> 2);
		bit += (bit != 0);
		bit += 23;
		switch (insn.i_format.rt & 3) {
		case 0: /* bc1f */
		case 2: /* bc1fl */
			if (~fcr31 & (1 << bit)) {
				epc = epc + 4 + (insn.i_format.simmediate << 2);
				if (insn.i_format.rt == 2)
					ret = BRANCH_LIKELY_TAKEN;
			} else
				epc += 8;
			regs->cp0_epc = epc;
			break;

		case 1: /* bc1t */
		case 3: /* bc1tl */
			if (fcr31 & (1 << bit)) {
				epc = epc + 4 + (insn.i_format.simmediate << 2);
				if (insn.i_format.rt == 3)
					ret = BRANCH_LIKELY_TAKEN;
			} else
				epc += 8;
			regs->cp0_epc = epc;
			break;
		}
		break;

#ifdef CONFIG_CPU_MIPSR6
	case bc_op:
		epc += 8;
		regs->cp0_epc = epc;
		break;

	case jump_op:
		if (insn.i_format.rs)   /* beqzc */
			epc = epc + 8;
		else                    /* jic, no offset shift */
			epc = regs->regs[insn.i_format.rt] + insn.i_format.simmediate;
		regs->cp0_epc = epc;
		break;

	case balc_op:
		regs->regs[31] = epc + 4;
		epc = epc + 4 + (insn.i_format.simmediate << 2);
		regs->cp0_epc = epc;
		break;

	case jump2_op:
		if (insn.i_format.rs)   /* bnezc */
			epc = epc + 8;
		else {                  /* jialc, no offset shift */
			regs->regs[31] = epc + 4;
			epc = regs->regs[insn.i_format.rt] + insn.i_format.simmediate;
		}
		regs->cp0_epc = epc;
		break;
#endif

#ifdef CONFIG_CPU_CAVIUM_OCTEON
	case lwc2_op: /* This is bbit0 on Octeon */
		if ((regs->regs[insn.i_format.rs] & (1ull<<insn.i_format.rt))
		     == 0)
			epc = epc + 4 + (insn.i_format.simmediate << 2);
		else
			epc += 8;
		regs->cp0_epc = epc;
		break;
	case ldc2_op: /* This is bbit032 on Octeon */
		if ((regs->regs[insn.i_format.rs] &
		    (1ull<<(insn.i_format.rt+32))) == 0)
			epc = epc + 4 + (insn.i_format.simmediate << 2);
		else
			epc += 8;
		regs->cp0_epc = epc;
		break;
	case swc2_op: /* This is bbit1 on Octeon */
		if (regs->regs[insn.i_format.rs] & (1ull<<insn.i_format.rt))
			epc = epc + 4 + (insn.i_format.simmediate << 2);
		else
			epc += 8;
		regs->cp0_epc = epc;
		break;
	case sdc2_op: /* This is bbit132 on Octeon */
		if (regs->regs[insn.i_format.rs] &
		    (1ull<<(insn.i_format.rt+32)))
			epc = epc + 4 + (insn.i_format.simmediate << 2);
		else
			epc += 8;
		regs->cp0_epc = epc;
		break;
#endif
	}

	if (ret < 0) {
		force_sig(-ret,current);
		return -EFAULT;
	}
	return ret;

#ifndef CONFIG_CPU_MIPSR6
sigill:
	printk("%s: DSP branch but not DSP ASE - sending SIGBUS.\n", current->comm);
	force_sig(SIGBUS, current);
	return -EFAULT;
#endif
}
EXPORT_SYMBOL_GPL(__compute_return_epc_for_insn);

int __compute_return_epc(struct pt_regs *regs)
{
	unsigned int __user *addr;
	long epc;
	union mips_instruction insn;

	epc = regs->cp0_epc;
	if (epc & 3)
		goto unaligned;

	/*
	 * Read the instruction
	 */
	addr = (unsigned int __user *) epc;
	if (__get_user(insn.word, addr)) {
		force_sig(SIGSEGV, current);
		return -EFAULT;
	}

	return __compute_return_epc_for_insn(regs, insn);

unaligned:
	printk("%s: unaligned epc - sending SIGBUS.\n", current->comm);
	force_sig(SIGBUS, current);
	return -EFAULT;

}
