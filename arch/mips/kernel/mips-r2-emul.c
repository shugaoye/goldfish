/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2014 Imagination Technologies, LLC.  All rights reserved.
 *
 *      MIPS R2 user space instruction emulator
 *
 *      Author: Leonid Yegoshin
 */
#include <linux/bug.h>
#include <linux/compiler.h>
#include <linux/ptrace.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <asm/branch.h>
#include <asm/fpu.h>
#include <asm/fpu_emulator.h>
#include <asm/ptrace.h>
#include <asm/uaccess.h>
#include <asm/asm.h>

#include <asm/break.h>
#include <asm/inst.h>
#include <asm/local.h>

/* number of loop cycles before we gave up */
#define TOTAL_PASS      10

#ifdef CONFIG_CPU_MIPS64
#define ADDIU           "daddiu "
#define INS             "dins "
#define EXT             "dext "
#define SB              "sb "
#define LB              "lb "
#define LL              "ll "
#define SC              "sc "
#else /* !CONFIG_CPU_MIPS64 */
#define ADDIU           "addiu "
#define INS             "ins "
#define EXT             "ext "
#ifdef CONFIG_EVA
#define SB              "sbe "
#define LB              "lbe "
#define LL              "lle "
#define SC              "sce "
#else
#define SB              "sb "
#define LB              "lb "
#define LL              "ll "
#define SC              "sc "
#endif
#endif

#ifdef CONFIG_DEBUG_FS

struct mips_r2_emulator_stats {
	local_t movs;
	local_t hilo;
	local_t muls;
	local_t divs;
	local_t dsps;
	local_t bops;
	local_t traps;
	local_t fpus;
	local_t loads;
	local_t stores;
	local_t llsc;
	local_t dsemul;
};

DEFINE_PER_CPU(struct mips_r2_emulator_stats, mipsr2emustats);
DEFINE_PER_CPU(struct mips_r2_emulator_stats, mipsr2bdemustats);

#define MIPS_R2_STATS(M)                                       \
do {									\
	u32 nir;                                                        \
	int err;                                                        \
									\
	preempt_disable();						\
	__local_inc(&__get_cpu_var(mipsr2emustats).M);                     \
	err = __get_user(nir, (u32 __user *)regs->cp0_epc);                \
	if (!err) {                                                        \
		if (nir == BREAK_MATH)                                     \
			__local_inc(&__get_cpu_var(mipsr2bdemustats).M);   \
	}                                                                  \
	preempt_enable();						\
} while (0)

struct mips_r2br_emulator_stats {
	local_t jrs;
	local_t bltzl;
	local_t bgezl;
	local_t bltzll;
	local_t bgezll;
	local_t bltzall;
	local_t bgezall;
	local_t bltzal;
	local_t bgezal;
	local_t beql;
	local_t bnel;
	local_t blezl;
	local_t bgtzl;
};

DEFINE_PER_CPU(struct mips_r2br_emulator_stats, mipsr2bremustats);

#define MIPS_R2BR_STATS(M)                                       \
do {									\
	preempt_disable();						\
	__local_inc(&__get_cpu_var(mipsr2bremustats).M);                     \
	preempt_enable();						\
} while (0)

#else

#define MIPS_R2_STATS(M)          do { } while (0)
#define MIPS_R2BR_STATS(M)        do { } while (0)

#endif /* CONFIG_DEBUG_FS */

struct r2_decoder_table {
	u32     mask;
	u32     code;
	int     (*func)(struct pt_regs *regs, u32 inst);
};


int mipsr2_emulation = 1;

void do_trap_or_bp(struct pt_regs *regs, unsigned int code,
	const char *str);

static int __init nomipsr2_func(char *s)
{
	mipsr2_emulation = 0;
	return 1;
}
__setup("nomipsr2", nomipsr2_func);

/* Emulate some frequent R2/R5/R6 instructions in JR BD slot for performance.
 * Otherwise it will be emulated via stack trampoline... very slow.
 */
static inline int mipsr6_emul(struct pt_regs *regs, u32 ir)
{
	switch (MIPSInst_OPCODE(ir)) {
	case addiu_op:
		if (MIPSInst_RT(ir))
			regs->regs[MIPSInst_RT(ir)] =
				(s32)regs->regs[MIPSInst_RS(ir)] +
				(s32)MIPSInst_SIMM(ir);
		return(0);
#ifdef CONFIG_64BIT
	case daddiu_op:
		if (MIPSInst_RT(ir))
			regs->regs[MIPSInst_RT(ir)] =
				(s64)regs->regs[MIPSInst_RS(ir)] +
				(s64)MIPSInst_SIMM(ir);
		return(0);
#endif
	case lwc1_op:
	case swc1_op:
	case cop1_op:
	case cop1x_op:
		/* indicate FPU instructions */
		return(-SIGFPE);
	case spec_op:
		switch (MIPSInst_FUNC(ir)) {
		case or_op:
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					regs->regs[MIPSInst_RS(ir)] |
					regs->regs[MIPSInst_RT(ir)];
			return(0);
		case sll_op:
			if (MIPSInst_RS(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(s32)(((u32)regs->regs[MIPSInst_RT(ir)]) <<
						MIPSInst_FD(ir));
			return(0);
		case srl_op:
			if (MIPSInst_RS(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(s32)(((u32)regs->regs[MIPSInst_RT(ir)]) >>
						MIPSInst_FD(ir));
			return(0);
		case addu_op:
			if (MIPSInst_FD(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(s32)((u32)regs->regs[MIPSInst_RS(ir)] +
					      (u32)regs->regs[MIPSInst_RT(ir)]);
			return(0);
		case subu_op:
			if (MIPSInst_FD(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(s32)((u32)regs->regs[MIPSInst_RS(ir)] -
					      (u32)regs->regs[MIPSInst_RT(ir)]);
			return(0);
#ifdef CONFIG_64BIT
		case dsll_op:
			if (MIPSInst_RS(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(s64)(((u64)regs->regs[MIPSInst_RT(ir)]) <<
						MIPSInst_FD(ir));
			return(0);
		case dsrl_op:
			if (MIPSInst_RS(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(s64)(((u64)regs->regs[MIPSInst_RT(ir)]) >>
						MIPSInst_FD(ir));
			return(0);
		case daddu_op:
			if (MIPSInst_FD(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(u64)regs->regs[MIPSInst_RS(ir)] +
					(u64)regs->regs[MIPSInst_RT(ir)];
			return(0);
		case dsubu_op:
			if (MIPSInst_FD(ir))
				break;
			if (MIPSInst_RD(ir))
				regs->regs[MIPSInst_RD(ir)] =
					(s64)((u64)regs->regs[MIPSInst_RS(ir)] -
					      (u64)regs->regs[MIPSInst_RT(ir)]);
			return(0);
#endif
		}
		break;
	}
	return SIGILL;
}

static int movf_func(struct pt_regs *regs, u32 ir)
{
	u32 csr;
	u32 cond;

	csr = current->thread.fpu.fcr31;
	cond = fpucondbit[MIPSInst_RT(ir) >> 2];
	if (((csr & cond) == 0) && MIPSInst_RD(ir))
		regs->regs[MIPSInst_RD(ir)] = regs->regs[MIPSInst_RS(ir)];
	MIPS_R2_STATS(movs);
	return(0);
}

static int movt_func(struct pt_regs *regs, u32 ir)
{
	u32 csr;
	u32 cond;

	csr = current->thread.fpu.fcr31;
	cond = fpucondbit[MIPSInst_RT(ir) >> 2];
	if (((csr & cond) != 0) && MIPSInst_RD(ir))
		regs->regs[MIPSInst_RD(ir)] = regs->regs[MIPSInst_RS(ir)];
	MIPS_R2_STATS(movs);
	return(0);
}

static int jr_func(struct pt_regs *regs, u32 ir)
{
	int err;
	unsigned long epc;
	unsigned long cpc;
	unsigned long nepc;
	unsigned long r31;
	u32 nir;

	if (delay_slot(regs))
		return(SIGILL);
	nepc = regs->cp0_epc;
	regs->cp0_epc -= 4;
	epc = regs->cp0_epc;
	r31 = regs->regs[31];
	err = __compute_return_epc(regs);
	if (err < 0)
		return(SIGEMT);
	cpc = regs->cp0_epc;
	err = __get_user(nir, (u32 __user *)nepc);
	if (err)
		return(SIGSEGV);
	MIPS_R2BR_STATS(jrs);
	if (nir) {  /* NOP is easy */
		/* Negative err means FPU instruction in BD-slot,
		   Zero err means 'BD-slot emulation done' */
		if ((err = mipsr6_emul(regs,nir)) > 0) {
			regs->cp0_epc = nepc;
			err = mips_dsemul(regs, nir, cpc, epc, r31);
			if (err == SIGILL)
				err = SIGEMT;
			MIPS_R2_STATS(dsemul);
		}
	}
	return (err);
}

static int movz_func(struct pt_regs *regs, u32 ir)
{
	if (((regs->regs[MIPSInst_RT(ir)]) == 0) && MIPSInst_RD(ir))
		regs->regs[MIPSInst_RD(ir)] = regs->regs[MIPSInst_RS(ir)];
	MIPS_R2_STATS(movs);
	return(0);
}

static int movn_func(struct pt_regs *regs, u32 ir)
{
	if (((regs->regs[MIPSInst_RT(ir)]) != 0) && MIPSInst_RD(ir))
		regs->regs[MIPSInst_RD(ir)] = regs->regs[MIPSInst_RS(ir)];
	MIPS_R2_STATS(movs);
	return(0);
}

static int mfhi_func(struct pt_regs *regs, u32 ir)
{
	if (MIPSInst_RD(ir))
		regs->regs[MIPSInst_RD(ir)] = regs->hi;
	MIPS_R2_STATS(hilo);
	return(0);
}

static int mthi_func(struct pt_regs *regs, u32 ir)
{
	regs->hi = regs->regs[MIPSInst_RS(ir)];
	MIPS_R2_STATS(hilo);
	return(0);
}

static int mflo_func(struct pt_regs *regs, u32 ir)
{
	if (MIPSInst_RD(ir))
		regs->regs[MIPSInst_RD(ir)] = regs->lo;
	MIPS_R2_STATS(hilo);
	return(0);
}

static int mtlo_func(struct pt_regs *regs, u32 ir)
{
	regs->lo = regs->regs[MIPSInst_RS(ir)];
	MIPS_R2_STATS(hilo);
	return(0);
}

static int mult_func(struct pt_regs *regs, u32 ir)
{
	s64 res;
	s32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = (s64)rt * (s64)rs;

	rs = res;
	regs->lo = (s64)rs;
	rt = res >> 32;
	res = (s64)rt;
	regs->hi = res;
	MIPS_R2_STATS(muls);
	return(0);
}

static int multu_func(struct pt_regs *regs, u32 ir)
{
	u64 res;
	u32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = (u64)rt * (u64)rs;
	rt = res;
	regs->lo = (s64)(s32)rt;
	regs->hi = (s64)(s32)(res >> 32);
	MIPS_R2_STATS(muls);
	return(0);
}

static int div_func(struct pt_regs *regs, u32 ir)
{
	s32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];

	regs->lo = (s64)(rs / rt);
	regs->hi = (s64)(rs % rt);
	MIPS_R2_STATS(divs);
	return(0);
}

static int divu_func(struct pt_regs *regs, u32 ir)
{
	u32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];

	regs->lo = (s64)(rs / rt);
	regs->hi = (s64)(rs % rt);
	MIPS_R2_STATS(divs);
	return(0);
}

#ifdef CONFIG_64BIT
static int dmult_func(struct pt_regs *regs, u32 ir)
{
	s64 res;
	s64 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = rt * rs;

	regs->lo = res;
	__asm__ __volatile__("dmuh %0, %1, %2" : "=r"(res) : "r"(rt), "r"(rs));
	regs->hi = res;
	MIPS_R2_STATS(muls);
	return(0);
}

static int dmultu_func(struct pt_regs *regs, u32 ir)
{
	u64 res;
	u64 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = rt * rs;

	regs->lo = res;
	__asm__ __volatile__("dmuhu %0, %1, %2" : "=r"(res) : "r"(rt), "r"(rs));
	regs->hi = res;
	MIPS_R2_STATS(muls);
	return(0);
}

static int ddiv_func(struct pt_regs *regs, u32 ir)
{
	s64 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];

	regs->lo = rs / rt;
	regs->hi = rs % rt;
	MIPS_R2_STATS(divs);
	return(0);
}

static int ddivu_func(struct pt_regs *regs, u32 ir)
{
	u64 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];

	regs->lo = rs / rt;
	regs->hi = rs % rt;
	MIPS_R2_STATS(divs);
	return(0);
}
#endif

static struct r2_decoder_table spec_op_table[] = {
	{ 0xfc1ff83f, 0x00000008, jr_func },
	{ 0xfc00ffff, 0x00000018, mult_func },  /* case AC=0 only */
	{ 0xfc00ffff, 0x00000019, multu_func }, /* case AC=0 only */
#ifdef CONFIG_64BIT
	{ 0xfc00ffff, 0x0000001c, dmult_func },  /* case AC=0 only */
	{ 0xfc00ffff, 0x0000001d, dmultu_func }, /* case AC=0 only */
#endif
	{ 0xffff07ff, 0x00000010, mfhi_func },
	{ 0xfc1fffff, 0x00000011, mthi_func },
	{ 0xffff07ff, 0x00000012, mflo_func },
	{ 0xfc1fffff, 0x00000013, mtlo_func },
	{ 0xfc0307ff, 0x00000001, movf_func },
	{ 0xfc0307ff, 0x00010001, movt_func },
	{ 0xfc0007ff, 0x0000000a, movz_func },
	{ 0xfc0007ff, 0x0000000b, movn_func },
	{ 0xfc00ffff, 0x0000001a, div_func },
	{ 0xfc00ffff, 0x0000001b, divu_func },
#ifdef CONFIG_64BIT
	{ 0xfc00ffff, 0x0000001e, ddiv_func },
	{ 0xfc00ffff, 0x0000001f, ddivu_func },
#endif
	{ }
};

static int madd_func(struct pt_regs *regs, u32 ir)
{
	s64 res;
	s32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = (s64)rt * (s64)rs;
	rt = regs->hi;
	rs = regs->lo;
	res += ((((s64)rt) << 32) | (u32)rs);

	rt = res;
	regs->lo = (s64)rt;
	rs = res >> 32;
	regs->hi = (s64)rs;
	MIPS_R2_STATS(dsps);
	return(0);
}

static int maddu_func(struct pt_regs *regs, u32 ir)
{
	u64 res;
	u32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = (u64)rt * (u64)rs;
	rt = regs->hi;
	rs = regs->lo;
	res += ((((s64)rt) << 32) | (u32)rs);

	rt = res;
	regs->lo = (s64)(s32)rt;
	rs = res >> 32;
	regs->hi = (s64)(s32)rs;
	MIPS_R2_STATS(dsps);
	return(0);
}

static int msub_func(struct pt_regs *regs, u32 ir)
{
	s64 res;
	s32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = (s64)rt * (s64)rs;
	rt = regs->hi;
	rs = regs->lo;
	res = ((((s64)rt) << 32) | (u32)rs) - res;

	rt = res;
	regs->lo = (s64)rt;
	rs = res >> 32;
	regs->hi = (s64)rs;
	MIPS_R2_STATS(dsps);
	return(0);
}

static int msubu_func(struct pt_regs *regs, u32 ir)
{
	u64 res;
	u32 rt, rs;

	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = (u64)rt * (u64)rs;
	rt = regs->hi;
	rs = regs->lo;
	res = ((((s64)rt) << 32) | (u32)rs) - res;

	rt = res;
	regs->lo = (s64)(s32)rt;
	rs = res >> 32;
	regs->hi = (s64)(s32)rs;
	MIPS_R2_STATS(dsps);
	return(0);
}

static int mul_func(struct pt_regs *regs, u32 ir)
{
	s64 res;
	s32 rt, rs;

	if (!MIPSInst_RD(ir))
		return(0);
	rt = regs->regs[MIPSInst_RT(ir)];
	rs = regs->regs[MIPSInst_RS(ir)];
	res = (s64)rt * (s64)rs;

	rs = res;
	regs->regs[MIPSInst_RD(ir)] = (s64)rs;
	MIPS_R2_STATS(muls);
	return(0);
}

static int clz_func(struct pt_regs *regs, u32 ir)
{
	u32 res;
	u32 rs;

	if (!MIPSInst_RD(ir))
		return(0);
	rs = regs->regs[MIPSInst_RS(ir)];
	__asm__ __volatile__("clz %0, %1" : "=r"(res) : "r"(rs));
	regs->regs[MIPSInst_RD(ir)] = res;
	MIPS_R2_STATS(bops);
	return(0);
}

static int clo_func(struct pt_regs *regs, u32 ir)
{
	u32 res;
	u32 rs;

	if (!MIPSInst_RD(ir))
		return(0);
	rs = regs->regs[MIPSInst_RS(ir)];
	__asm__ __volatile__("clo %0, %1" : "=r"(res) : "r"(rs));
	regs->regs[MIPSInst_RD(ir)] = res;
	MIPS_R2_STATS(bops);
	return(0);
}

#ifdef CONFIG_64BIT
static int dclz_func(struct pt_regs *regs, u32 ir)
{
	u64 res;
	u64 rs;

	if (!MIPSInst_RD(ir))
		return(0);
	rs = regs->regs[MIPSInst_RS(ir)];
	__asm__ __volatile__("dclz %0, %1" : "=r"(res) : "r"(rs));
	regs->regs[MIPSInst_RD(ir)] = res;
	MIPS_R2_STATS(bops);
	return(0);
}

static int dclo_func(struct pt_regs *regs, u32 ir)
{
	u64 res;
	u64 rs;

	if (!MIPSInst_RD(ir))
		return(0);
	rs = regs->regs[MIPSInst_RS(ir)];
	__asm__ __volatile__("dclo %0, %1" : "=r"(res) : "r"(rs));
	regs->regs[MIPSInst_RD(ir)] = res;
	MIPS_R2_STATS(bops);
	return(0);
}
#endif

static struct r2_decoder_table spec2_op_table[] = {
	{ 0xfc00ffff, 0x70000000, madd_func },
	{ 0xfc00ffff, 0x70000001, maddu_func },
	{ 0xfc0007ff, 0x70000002, mul_func },
	{ 0xfc00ffff, 0x70000004, msub_func },  /* case AC=0 only */
	{ 0xfc00ffff, 0x70000005, msubu_func }, /* case AC=0 only */
	{ 0xfc0007ff, 0x70000020, clz_func },
	{ 0xfc0007ff, 0x70000021, clo_func },
#ifdef CONFIG_64BIT
	{ 0xfc0007ff, 0x70000024, dclz_func },
	{ 0xfc0007ff, 0x70000025, dclo_func },
#endif
/*         { 0xfc00003f, 0x7000003f, sdbbp_func }, - no SDBBP support in R2 emulation */
	{ }
};

static inline int mipsr2_find_ex(struct pt_regs *regs, u32 inst, struct r2_decoder_table *table)
{
	struct r2_decoder_table *p;
	int err;

	for (p=table; p->func; p++) {
		if ((inst & p->mask) == p->code) {
			err = (p->func)(regs, inst);
			return(err);
		}
	}
	return(SIGILL);
}

int mipsr2_decoder(struct pt_regs *regs, u32 instruction)
{
	int err = 0;
	unsigned long vaddr;
	u32 inst = instruction;
	u32 nir;
	unsigned long r31;
	unsigned long epc;
	unsigned long nepc;
	unsigned long cpc;
	unsigned long rt;
	unsigned long rs;
	unsigned long res;
	void __user *fault_addr = NULL;
	int pass = 0;

repeat:
	r31 = regs->regs[31];
	epc = regs->cp0_epc;
	err = compute_return_epc(regs);
	if (err < 0)
		return(SIGEMT);

	switch (MIPSInst_OPCODE(inst)) {

	case spec_op:
		err = mipsr2_find_ex(regs, inst, spec_op_table);
		if (err < 0) {
			/* FPU instruction under JR */
			regs->cp0_cause |= CAUSEF_BD;
			goto fpu_emul;
		}
		break;

	case spec2_op:
		err = mipsr2_find_ex(regs, inst, spec2_op_table);
		break;

	case bcond_op:
		rt = MIPSInst_RT(inst);
		rs = MIPSInst_RS(inst);
		switch (rt) {
		case tgei_op:
			if ((long)regs->regs[rs] >= MIPSInst_SIMM(inst))
				do_trap_or_bp(regs, 0, "TGEI");
			MIPS_R2_STATS(traps);
			break;
		case tgeiu_op:
			if (regs->regs[rs] >= MIPSInst_UIMM(inst))
				do_trap_or_bp(regs, 0, "TGEIU");
			MIPS_R2_STATS(traps);
			break;
		case tlti_op:
			if ((long)regs->regs[rs] < MIPSInst_SIMM(inst))
				do_trap_or_bp(regs, 0, "TLTI");
			MIPS_R2_STATS(traps);
			break;
		case tltiu_op:
			if (regs->regs[rs] < MIPSInst_UIMM(inst))
				do_trap_or_bp(regs, 0, "TLTIU");
			MIPS_R2_STATS(traps);
			break;
		case teqi_op:
			if (regs->regs[rs] == MIPSInst_SIMM(inst))
				do_trap_or_bp(regs, 0, "TEQI");
			MIPS_R2_STATS(traps);
			break;
		case tnei_op:
			if (regs->regs[rs] != MIPSInst_SIMM(inst))
				do_trap_or_bp(regs, 0, "TNEI");
			MIPS_R2_STATS(traps);
			break;
		case bltzl_op:
		case bgezl_op:
		case bltzall_op:
		case bgezall_op:
			if (delay_slot(regs)) {
				err = SIGILL;
				break;
			}
			regs->regs[31] = r31;
			regs->cp0_epc = epc;
			err = __compute_return_epc(regs);
			if (err < 0)
				return(SIGEMT);
			if (err != BRANCH_LIKELY_TAKEN)
				break;
			cpc = regs->cp0_epc;
			nepc = epc + 4;
			err = __get_user(nir, (u32 __user *)nepc);
			if (err) {
				err = SIGSEGV;
				break;
			}
#ifdef CONFIG_DEBUG_FS
			switch (rt) {
			case bltzl_op:
				MIPS_R2BR_STATS(bltzl);
				break;
			case bgezl_op:
				MIPS_R2BR_STATS(bgezl);
				break;
			case bltzall_op:
				MIPS_R2BR_STATS(bltzall);
				break;
			case bgezall_op:
				MIPS_R2BR_STATS(bgezall);
				break;
			}
#endif
			switch (MIPSInst_OPCODE(nir)) {
			case cop1_op:
			case cop1x_op:
			case lwc1_op:
			case swc1_op:
				regs->cp0_cause |= CAUSEF_BD;
				goto fpu_emul;
			}
			if (nir) {  /* NOP is easy */
				if ((err = mipsr6_emul(regs,nir)) > 0) {
					regs->cp0_epc = nepc;
					err = mips_dsemul(regs, nir, cpc, epc, r31);
					if (err == SIGILL)
						err = SIGEMT;
					MIPS_R2_STATS(dsemul);
				}
			}
			break;
		case bltzal_op:
		case bgezal_op:
			if (delay_slot(regs)) {
				err = SIGILL;
				break;
			}
			regs->regs[31] = r31;
			regs->cp0_epc = epc;
			err = __compute_return_epc(regs);
			if (err < 0)
				return(SIGEMT);
			cpc = regs->cp0_epc;
			nepc = epc + 4;
			err = __get_user(nir, (u32 __user *)nepc);
			if (err) {
				err = SIGSEGV;
				break;
			}
#ifdef CONFIG_DEBUG_FS
			switch (rt) {
			case bltzal_op:
				MIPS_R2BR_STATS(bltzal);
				break;
			case bgezal_op:
				MIPS_R2BR_STATS(bgezal);
				break;
			}
#endif
			switch (MIPSInst_OPCODE(nir)) {
			case cop1_op:
			case cop1x_op:
			case lwc1_op:
			case swc1_op:
				regs->cp0_cause |= CAUSEF_BD;
				goto fpu_emul;
			}
			if (nir) {  /* NOP is easy */
				if ((err = mipsr6_emul(regs,nir)) > 0) {
					regs->cp0_epc = nepc;
					err = mips_dsemul(regs, nir, cpc, epc, r31);
					if (err == SIGILL)
						err = SIGEMT;
					MIPS_R2_STATS(dsemul);
				}
			}
			break;
		default:
			regs->regs[31] = r31;
			regs->cp0_epc = epc;
			err = SIGILL;
			break;
		}
		break;

	case beql_op:
	case bnel_op:
	case blezl_op:
	case bgtzl_op:
		if (delay_slot(regs)) {
			err = SIGILL;
			break;
		}
		regs->regs[31] = r31;
		regs->cp0_epc = epc;
		err = __compute_return_epc(regs);
		if (err < 0)
			return(SIGEMT);
		if (err != BRANCH_LIKELY_TAKEN)
			break;
		cpc = regs->cp0_epc;
		nepc = epc + 4;
		err = __get_user(nir, (u32 __user *)nepc);
		if (err) {
			err = SIGSEGV;
			break;
		}
#ifdef CONFIG_DEBUG_FS
		switch (MIPSInst_OPCODE(inst)) {
		case beql_op:
			MIPS_R2BR_STATS(beql);
			break;
		case bnel_op:
			MIPS_R2BR_STATS(bnel);
			break;
		case blezl_op:
			MIPS_R2BR_STATS(blezl);
			break;
		case bgtzl_op:
			MIPS_R2BR_STATS(bgtzl);
			break;
		}
#endif
		switch (MIPSInst_OPCODE(nir)) {
		case cop1_op:
		case cop1x_op:
		case lwc1_op:
		case swc1_op:
			regs->cp0_cause |= CAUSEF_BD;
			goto fpu_emul;
		}
		if (nir) {  /* NOP is easy */
			if ((err = mipsr6_emul(regs,nir)) > 0) {
				regs->cp0_epc = nepc;
				err = mips_dsemul(regs, nir, cpc, epc, r31);
				if (err == SIGILL)
					err = SIGEMT;
				MIPS_R2_STATS(dsemul);
			}
		}
		break;
	case lwc1_op:
	case swc1_op:
	case cop1_op:
	case cop1x_op:
fpu_emul:
		regs->regs[31] = r31;
		regs->cp0_epc = epc;
		/* all cop1/cop1x RI in R6 are a removed R5 FPU instructions */
		if (!used_math())       /* First time FPU user.  */
			init_fpu();
		lose_fpu(1);    /* Save FPU state for the emulator. */

		err = fpu_emulator_cop1Handler(regs, &current->thread.fpu, 0,
					       &fault_addr);

		/* this is a tricky issue - lose_fpu() uses LL/SC atomics
		   if FPU is owned and effectively cancels user level LL/SC.
		   So, it could be logical to don't restore FPU ownership here.
		   But the sequence of multiple FPU instructions is much much
		   more often than LL-FPU-SC and I prefer loop here until
		   next scheduler cycle cancels FPU ownership. */
		own_fpu(1);	/* Restore FPU state. */

		if (err)
			current->thread.cp0_baduaddr = (unsigned long)fault_addr;

		MIPS_R2_STATS(fpus);
		break;;

	case lwl_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_READ, vaddr, 4)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
			"1:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 24, 8          \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
			"2:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 16, 8          \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
			"3:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 8, 8           \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
			"4:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 0, 8           \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
			"1:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 24, 8          \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
			"2:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 16, 8          \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
			"3:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 8, 8           \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
			"4:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 0, 8           \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:     sll     %0, %0, 0               \n"
			"10:                                    \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      10b                      \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV));
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = rt;

		MIPS_R2_STATS(loads);
		break;

	case lwr_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_READ, vaddr, 4)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
			"1:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 0, 8           \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
			"2:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 8, 8           \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
			"3:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 16, 8          \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
			"4:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 24, 8          \n"
			"       sll     %0, %0, 0               \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
			"1:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 0, 8           \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
			"2:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 8, 8           \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
			"3:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 16, 8          \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
			"4:"    LB      "%1, 0(%2)              \n"
				INS     "%0, %1, 24, 8          \n"
			"       sll     %0, %0, 0               \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:                                     \n"
			"10:                                    \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      10b                      \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV));
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = rt;

		MIPS_R2_STATS(loads);
		break;

	case swl_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_WRITE, vaddr, 4)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
				EXT     "%1, %0, 24, 8          \n"
			"1:"    SB      "%1, 0(%2)              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
				EXT     "%1, %0, 16, 8          \n"
			"2:"    SB      "%1, 0(%2)              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
				EXT     "%1, %0, 8, 8           \n"
			"3:"    SB      "%1, 0(%2)              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
				EXT     "%1, %0, 0, 8           \n"
			"4:"    SB      "%1, 0(%2)              \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
				EXT     "%1, %0, 24, 8          \n"
			"1:"    SB      "%1, 0(%2)              \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				EXT     "%1, %0, 16, 8          \n"
			"2:"    SB      "%1, 0(%2)              \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				EXT     "%1, %0, 8, 8           \n"
			"3:"    SB      "%1, 0(%2)              \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				EXT     "%1, %0, 0, 8           \n"
			"4:"    SB      "%1, 0(%2)              \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:                                     \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      9b                       \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV)
			: "memory");

		MIPS_R2_STATS(stores);
		break;

	case swr_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_WRITE, vaddr, 4)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
				EXT     "%1, %0, 0, 8           \n"
			"1:"    SB      "%1, 0(%2)              \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				EXT     "%1, %0, 8, 8           \n"
			"2:"    SB      "%1, 0(%2)              \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				EXT     "%1, %0, 16, 8          \n"
			"3:"    SB      "%1, 0(%2)              \n"
				ADDIU   "%2, %2, 1              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				EXT     "%1, %0, 24, 8          \n"
			"4:"    SB      "%1, 0(%2)              \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
				EXT     "%1, %0, 0, 8           \n"
			"1:"    SB      "%1, 0(%2)              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
				EXT     "%1, %0, 8, 8           \n"
			"2:"    SB      "%1, 0(%2)              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
				EXT     "%1, %0, 16, 8          \n"
			"3:"    SB      "%1, 0(%2)              \n"
			"       andi    %1, %2, 0x3             \n"
			"       beq     $0, %1, 9f              \n"
				ADDIU   "%2, %2, -1             \n"
				EXT     "%1, %0, 24, 8          \n"
			"4:"    SB      "%1, 0(%2)              \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:                                     \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      9b                       \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV)
			: "memory");

		MIPS_R2_STATS(stores);
		break;

#ifdef CONFIG_64BIT
	case ldl_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_READ, vaddr, 8)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
			"1:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 56, 8           \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"2:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 48, 8           \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"3:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 40, 8           \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"4:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 32, 8           \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"5:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 24, 8           \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"6:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 16, 8           \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"7:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 8, 8            \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"0:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 0, 8            \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
			"1:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 56, 8           \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"2:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 48, 8           \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"3:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 40, 8           \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"4:     lb      %1, 0(%2)               \n"
			"       dinsu   %0, %1, 32, 8           \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"5:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 24, 8           \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"6:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 16, 8           \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"7:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 8, 8            \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"0:     lb      %1, 0(%2)               \n"
			"       dins    %0, %1, 0, 8            \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:                                     \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      9b                       \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			__stringify(PTR) " 5b,8b                \n"
			__stringify(PTR) " 6b,8b                \n"
			__stringify(PTR) " 7b,8b                \n"
			__stringify(PTR) " 0b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV));
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = rt;

		MIPS_R2_STATS(loads);
		break;

	case ldr_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_READ, vaddr, 8)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
			"1:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 0, 8             \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"2:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 8, 8             \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"3:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 16, 8            \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"4:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 24, 8            \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"5:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 32, 8          \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"6:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 40, 8          \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"7:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 48, 8          \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"0:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 56, 8          \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
			"1:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 0, 8             \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"2:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 8, 8             \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"3:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 16, 8            \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"4:     lb      %1, 0(%2)               \n"
			"       dins   %0, %1, 24, 8            \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"5:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 32, 8          \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"6:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 40, 8          \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"7:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 48, 8          \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"0:     lb      %1, 0(%2)               \n"
			"       dinsu    %0, %1, 56, 8          \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:                                     \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      9b                       \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			__stringify(PTR) " 5b,8b                \n"
			__stringify(PTR) " 6b,8b                \n"
			__stringify(PTR) " 7b,8b                \n"
			__stringify(PTR) " 0b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV));
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = rt;

		MIPS_R2_STATS(loads);
		break;

	case sdl_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_WRITE, vaddr, 8)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
			"       dextu   %1, %0, 56, 8           \n"
			"1:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dextu   %1, %0, 48, 8           \n"
			"2:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dextu   %1, %0, 40, 8           \n"
			"3:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dextu   %1, %0, 32, 8           \n"
			"4:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dext    %1, %0, 24, 8           \n"
			"5:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dext    %1, %0, 16, 8           \n"
			"6:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dext    %1, %0, 8, 8            \n"
			"7:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dext    %1, %0, 0, 8            \n"
			"0:     sb      %1, 0(%2)               \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
			"       dextu   %1, %0, 56, 8           \n"
			"1:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dextu   %1, %0, 48, 8           \n"
			"2:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dextu   %1, %0, 40, 8           \n"
			"3:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dextu   %1, %0, 32, 8           \n"
			"4:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dext    %1, %0, 24, 8           \n"
			"5:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dext    %1, %0, 16, 8           \n"
			"6:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dext    %1, %0, 8, 8            \n"
			"7:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dext    %1, %0, 0, 8            \n"
			"0:     sb      %1, 0(%2)               \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:                                     \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      9b                       \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			__stringify(PTR) " 5b,8b                \n"
			__stringify(PTR) " 6b,8b                \n"
			__stringify(PTR) " 7b,8b                \n"
			__stringify(PTR) " 0b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV)
			: "memory");

		MIPS_R2_STATS(stores);
		break;

	case sdr_op:
		rt = regs->regs[MIPSInst_RT(inst)];
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (!access_ok(VERIFY_WRITE, vaddr, 8)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGSEGV;
			break;
		}
		__asm__ __volatile__(
			"       .set    push                    \n"
			"       .set    reorder                 \n"
#ifdef CONFIG_CPU_LITTLE_ENDIAN
			"       dext    %1, %0, 0, 8            \n"
			"1:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dext    %1, %0, 8, 8            \n"
			"2:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dext    %1, %0, 16, 8           \n"
			"3:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dext    %1, %0, 24, 8           \n"
			"4:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dextu   %1, %0, 32, 8           \n"
			"5:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dextu   %1, %0, 40, 8           \n"
			"6:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dextu   %1, %0, 48, 8           \n"
			"7:     sb      %1, 0(%2)               \n"
			"       daddiu  %2, %2, 1               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       dextu   %1, %0, 56, 8           \n"
			"0:     sb      %1, 0(%2)               \n"
#else /* !CONFIG_CPU_LITTLE_ENDIAN */
			"       dext    %1, %0, 0, 8            \n"
			"1:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dext    %1, %0, 8, 8            \n"
			"2:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dext    %1, %0, 16, 8           \n"
			"3:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dext    %1, %0, 24, 8           \n"
			"4:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dextu   %1, %0, 32, 8           \n"
			"5:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dextu   %1, %0, 40, 8           \n"
			"6:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dextu   %1, %0, 48, 8           \n"
			"7:     sb      %1, 0(%2)               \n"
			"       andi    %1, %2, 0x7             \n"
			"       beq     $0, %1, 9f              \n"
			"       daddiu  %2, %2, -1              \n"
			"       dextu   %1, %0, 56, 8           \n"
			"0:     sb      %1, 0(%2)               \n"
#endif /* CONFIG_CPU_LITTLE_ENDIAN */
			"9:                                     \n"
			"       .insn                           \n"
			"       .section        .fixup,\"ax\"   \n"
			"8:     li     %3,%4                    \n"
			"       j      9b                       \n"
			"       .previous                       \n"
			"       .section        __ex_table,\"a\"\n"
			__stringify(PTR) " 1b,8b                \n"
			__stringify(PTR) " 2b,8b                \n"
			__stringify(PTR) " 3b,8b                \n"
			__stringify(PTR) " 4b,8b                \n"
			__stringify(PTR) " 5b,8b                \n"
			__stringify(PTR) " 6b,8b                \n"
			__stringify(PTR) " 7b,8b                \n"
			__stringify(PTR) " 0b,8b                \n"
			"       .previous                       \n"
			"       .set    pop                     \n"
			: "+&r"(rt), "=&r"(rs),
			  "+&r"(vaddr), "+&r"(err)
			: "i"(SIGSEGV)
			: "memory");

		MIPS_R2_STATS(stores);
		break;
#endif
	case ll_op:
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (vaddr & 0x3) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		if (!access_ok(VERIFY_READ, vaddr, 4)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		__asm__ __volatile__(
			"1:                                 \n"
			LL  "   %0, 0(%2)                   \n"
			"2:                                 \n"
			".insn                              \n"
			".section        .fixup,\"ax\"      \n"
			"3:                                 \n"
			"li     %1, %3                      \n"
			"j      2b                          \n"
			".previous                          \n"
			".section        __ex_table,\"a\"   \n"
			__stringify(PTR) " 1b,3b            \n"
			".previous                          \n"
			:"=&r"(res), "+&r"(err)
			:"r"(vaddr), "i"(SIGSEGV)
			:"memory");
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = res;
		MIPS_R2_STATS(llsc);
		break;

	case sc_op:
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (vaddr & 0x3) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		if (!access_ok(VERIFY_WRITE, vaddr, 4)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		res = regs->regs[MIPSInst_RT(inst)];
		__asm__ __volatile__(
			"1:                                 \n"
			SC  "   %0, 0(%2)                   \n"
			"2:                                 \n"
			".insn                              \n"
			".section        .fixup,\"ax\"      \n"
			"3:                                 \n"
			"li     %1, %3                      \n"
			"j      2b                          \n"
			".previous                          \n"
			".section        __ex_table,\"a\"   \n"
			__stringify(PTR) " 1b,3b            \n"
			".previous                          \n"
			:"+&r"(res), "+&r"(err)
			:"r"(vaddr), "i"(SIGSEGV)
			:"memory");
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = res;
		MIPS_R2_STATS(llsc);
		break;

#ifdef CONFIG_64BIT
	case lld_op:
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (vaddr & 0x7) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		if (!access_ok(VERIFY_READ, vaddr, 8)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		__asm__ __volatile__(
			"1:                                 \n"
			"lld    %0, 0(%2)                   \n"
			"2:                                 \n"
			".insn                              \n"
			".section        .fixup,\"ax\"      \n"
			"3:                                 \n"
			"li     %1, %3                      \n"
			"j      2b                          \n"
			".previous                          \n"
			".section        __ex_table,\"a\"   \n"
			__stringify(PTR) " 1b,3b            \n"
			".previous                          \n"
			:"=&r"(res), "+&r"(err)
			:"r"(vaddr), "i"(SIGSEGV)
			:"memory");
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = res;
		MIPS_R2_STATS(llsc);
		break;

	case scd_op:
		vaddr = regs->regs[MIPSInst_RS(inst)] + MIPSInst_SIMM(inst);
		if (vaddr & 0x7) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		if (!access_ok(VERIFY_WRITE, vaddr, 8)) {
			current->thread.cp0_baduaddr = vaddr;
			err = SIGBUS;
			break;
		}
		res = regs->regs[MIPSInst_RT(inst)];
		__asm__ __volatile__(
			"1:                                 \n"
			"scd    %0, 0(%2)                   \n"
			"2:                                 \n"
			".insn                              \n"
			".section        .fixup,\"ax\"      \n"
			"3:                                 \n"
			"li     %1, %3                      \n"
			"j      2b                          \n"
			".previous                          \n"
			".section        __ex_table,\"a\"   \n"
			__stringify(PTR) " 1b,3b            \n"
			".previous                          \n"
			:"+&r"(res), "+&r"(err)
			:"r"(vaddr), "i"(SIGSEGV)
			:"memory");
		if (MIPSInst_RT(inst) && !err)
			regs->regs[MIPSInst_RT(inst)] = res;
		MIPS_R2_STATS(llsc);
		break;
#endif
	case pref_op:
		/* skip it */
		break;

	default:
		err = SIGILL;
		break;
	}

	if ((!err) && (pass++ < TOTAL_PASS)) {
		regs->cp0_cause &= ~CAUSEF_BD;
		err = get_user(inst, (u32 __user *)regs->cp0_epc);
		if (!err)
			goto repeat;
		if (err < 0)
			err = SIGSEGV;
	}
	if (err && (err != SIGEMT)) {
		regs->regs[31] = r31;
		regs->cp0_epc = epc;
	}
	/* it can be MIPS R6 instruction */
	if (pass && (err == SIGILL))
		err = 0;
	return(err);
}

#ifdef CONFIG_DEBUG_FS

static int mipsr2_stats_show(struct seq_file *s, void *unused)
{

	seq_printf(s, "Instruction\tTotal\tBDslot\n------------------------------\n");
	seq_printf(s, "movs\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).movs.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).movs.a.counter);
	seq_printf(s, "hilo\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).hilo.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).hilo.a.counter);
	seq_printf(s, "muls\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).muls.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).muls.a.counter);
	seq_printf(s, "divs\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).divs.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).divs.a.counter);
	seq_printf(s, "dsps\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).dsps.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).dsps.a.counter);
	seq_printf(s, "bops\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).bops.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).bops.a.counter);
	seq_printf(s, "traps\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).traps.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).traps.a.counter);
	seq_printf(s, "fpus\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).fpus.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).fpus.a.counter);
	seq_printf(s, "loads\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).loads.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).loads.a.counter);
	seq_printf(s, "stores\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).stores.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).stores.a.counter);
	seq_printf(s, "llsc\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).llsc.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).llsc.a.counter);
	seq_printf(s, "dsemul\t\t%ld\t%ld\n",(unsigned long)__get_cpu_var(mipsr2emustats).dsemul.a.counter,
					 (unsigned long)__get_cpu_var(mipsr2bdemustats).dsemul.a.counter);

	seq_printf(s, "jr\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).jrs.a.counter);
	seq_printf(s, "bltzl\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bltzl.a.counter);
	seq_printf(s, "bgezl\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bgezl.a.counter);
	seq_printf(s, "bltzll\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bltzll.a.counter);
	seq_printf(s, "bgezll\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bgezll.a.counter);
	seq_printf(s, "bltzal\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bltzal.a.counter);
	seq_printf(s, "bgezal\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bgezal.a.counter);
	seq_printf(s, "beql\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).beql.a.counter);
	seq_printf(s, "bnel\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bnel.a.counter);
	seq_printf(s, "blezl\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).blezl.a.counter);
	seq_printf(s, "bgtzl\t\t%ld\n",(unsigned long)__get_cpu_var(mipsr2bremustats).bgtzl.a.counter);

	return 0;
}

static int mipsr2_stats_clear_show(struct seq_file *s, void *unused)
{
	mipsr2_stats_show(s, unused);

	__get_cpu_var(mipsr2emustats).movs.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).movs.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).hilo.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).hilo.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).muls.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).muls.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).divs.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).divs.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).dsps.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).dsps.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).bops.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).bops.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).traps.a.counter   = 0;
	__get_cpu_var(mipsr2bdemustats).traps.a.counter = 0;
	__get_cpu_var(mipsr2emustats).fpus.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).fpus.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).loads.a.counter   = 0;
	__get_cpu_var(mipsr2bdemustats).loads.a.counter = 0;
	__get_cpu_var(mipsr2emustats).stores.a.counter  = 0;
	__get_cpu_var(mipsr2bdemustats).stores.a.counter= 0;
	__get_cpu_var(mipsr2emustats).llsc.a.counter    = 0;
	__get_cpu_var(mipsr2bdemustats).llsc.a.counter  = 0;
	__get_cpu_var(mipsr2emustats).dsemul.a.counter  = 0;
	__get_cpu_var(mipsr2bdemustats).dsemul.a.counter= 0;

	__get_cpu_var(mipsr2bremustats).jrs.a.counter   = 0;
	__get_cpu_var(mipsr2bremustats).bltzl.a.counter = 0;
	__get_cpu_var(mipsr2bremustats).bgezl.a.counter = 0;
	__get_cpu_var(mipsr2bremustats).bltzll.a.counter= 0;
	__get_cpu_var(mipsr2bremustats).bgezll.a.counter= 0;
	__get_cpu_var(mipsr2bremustats).bltzal.a.counter= 0;
	__get_cpu_var(mipsr2bremustats).bgezal.a.counter= 0;
	__get_cpu_var(mipsr2bremustats).beql.a.counter  = 0;
	__get_cpu_var(mipsr2bremustats).bnel.a.counter  = 0;
	__get_cpu_var(mipsr2bremustats).blezl.a.counter = 0;
	__get_cpu_var(mipsr2bremustats).bgtzl.a.counter = 0;

	return 0;
}

static int mipsr2_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mipsr2_stats_show, inode->i_private);
}

static int mipsr2_stats_clear_open(struct inode *inode, struct file *file)
{
	return single_open(file, mipsr2_stats_clear_show, inode->i_private);
}

static const struct file_operations mipsr2_fops = {
	.open                   = mipsr2_stats_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static const struct file_operations mipsr2_clear_fops = {
	.open                   = mipsr2_stats_clear_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};


int mipsr2_init_debugfs(void)
{
	struct dentry		*root;
	struct dentry		*file;
	int			ret;

	root = debugfs_create_dir("mipsr2-emulation", NULL);
	if (!root) {
		ret = -ENOMEM;
		goto err0;
	}

	file = debugfs_create_file("stats", S_IRUGO, root, NULL,
			&mipsr2_fops);
	if (!file) {
		ret = -ENOMEM;
		goto err1;
	}

	file = debugfs_create_file("stats-clear", S_IRUGO, root, NULL,
			&mipsr2_clear_fops);
	if (!file) {
		ret = -ENOMEM;
		goto err1;
	}

	return 0;

err1:
	debugfs_remove_recursive(root);

err0:
	return ret;
}
__initcall(mipsr2_init_debugfs);

#endif /* CONFIG_DEBUG_FS */
