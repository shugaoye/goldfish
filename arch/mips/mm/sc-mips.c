/*
 * Copyright (C) 2006 Chris Dearman (chris@mips.com),
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>

#include <asm/mipsregs.h>
#include <asm/gcmpregs.h>
#include <asm/gic.h>
#include <asm/bcache.h>
#include <asm/cacheops.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/mmu_context.h>
#include <asm/r4kcache.h>

/*
 * MIPS32/MIPS64 L2 cache handling
 */

extern int cm3_l2_init(unsigned long lsize, unsigned long indexbase,
		       unsigned long dcache_size, unsigned long gcmpbase);

/*
 * Writeback and invalidate the secondary cache before DMA.
 */
static void mips_sc_wback_inv(unsigned long addr, unsigned long size)
{
	if (!cpu_has_cm2)
		__sync();
	blast_scache_range(addr, addr + size);
	if (cpu_has_cm2_l2sync)
		*(unsigned long *)(_gcmp_base + GCMP_L2SYNC_OFFSET) = 0;
}

/*
 * Invalidate the secondary cache before DMA.
 */
static void mips_sc_inv(unsigned long addr, unsigned long size)
{
	unsigned long lsize = cpu_scache_line_size();
	unsigned long almask = ~(lsize - 1);

	cache_op(Hit_Writeback_Inv_SD, addr & almask);
	cache_op(Hit_Writeback_Inv_SD, (addr + size - 1) & almask);
	blast_inv_scache_range(addr, addr + size);
}

static void mips_sc_enable(void)
{
	/* L2 cache is permanently enabled */
}

static void mips_sc_disable(void)
{
	/* L2 cache is permanently enabled */
}

static struct bcache_ops mips_sc_ops = {
	.bc_enable = mips_sc_enable,
	.bc_disable = mips_sc_disable,
	.bc_wback_inv = mips_sc_wback_inv,
	.bc_inv = mips_sc_inv
};

/*
 * Check if the L2 cache controller is activated on a particular platform.
 * MTI's L2 controller and the L2 cache controller of Broadcom's BMIPS
 * cores both use c0_config2's bit 12 as "L2 Bypass" bit, that is the
 * cache being disabled.  However there is no guarantee for this to be
 * true on all platforms.  In an act of stupidity the spec defined bits
 * 12..15 as implementation defined so below function will eventually have
 * to be replaced by a platform specific probe.
 */
static inline int mips_sc_is_activated(struct cpuinfo_mips *c)
{
	unsigned int config2 = read_c0_config2();
	unsigned int tmp;

	/* Check the bypass bit (L2B) */
	switch (c->cputype) {
	case CPU_34K:
	case CPU_1004K:
	case CPU_74K:
	case CPU_PROAPTIV:	/* proAptiv havn't L2B capability but ... */
	case CPU_INTERAPTIV:
	case CPU_P5600:
	case CPU_BMIPS5000:
		if (config2 & (1 << 12))
			return 0;
	}

	tmp = (config2 >> 4) & 0x0f;
	if (0 < tmp && tmp <= 7)
		c->scache.linesz = 2 << tmp;
	else
		return 0;
	return 1;
}

#ifdef CONFIG_MIPS_CMP
static inline int cm3_l2_setup(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	unsigned int tmp;
	unsigned int l2config = 0;
	unsigned int l2p;

	if (gcmp3_present)
		l2config = GCMPGCB(L2CONFIG);
	if (!(l2config & MIPS_CONF_M))
		return 0;

	tmp = (l2config & GCMP_GCB_L2CONFIG_LSIZE_MASK) >>
	       GCMP_GCB_L2CONFIG_LSIZE_SHF;
	if (!tmp)
		return 0;

	if (l2config & GCMP_GCB_L2CONFIG_BYPASS_MASK) {
		if (!cm3_l2_init(c->dcache.linesz, INDEX_BASE,
				 c->dcache.sets * c->dcache.ways * c->dcache.linesz,
				 _gcmp_base))
			return 0;
		printk("GCR_L2_CONFIG now 0x%08x\n",GCMPGCB(L2CONFIG));
		printk("CM3 L2 initialized\n");
	}

	c->scache.linesz = 2 << tmp;
	tmp = (l2config & GCMP_GCB_L2CONFIG_ASSOC_MASK) >>
		GCMP_GCB_L2CONFIG_ASSOC_SHF;
	c->scache.ways = tmp + 1;
	tmp = (l2config & GCMP_GCB_L2CONFIG_SSIZE_MASK) >>
		GCMP_GCB_L2CONFIG_SSIZE_SHF;
	c->scache.sets = 64 << tmp;

	/* setup L2 prefetch */
	l2p = GCMPGCB(GCML2P);
	if (l2p & GCMP_GCB_GCML2P_NPFT) {
		GCMPGCB(GCML2P) = (l2p & ~GCMP_GCB_GCML2P_PAGE_MASK) |
			PAGE_MASK | GCMP_GCB_GCML2P_PFTEN;
		GCMPGCB(GCML2PB) |= GCMP_GCB_GCML2PB_CODE_PFTEN;
	}

	return 1;
}
#endif

static inline int __init mips_sc_probe(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	unsigned int config1, config2;
	unsigned int tmp;

	/* Mark as not present until probe completed */
	c->scache.flags |= MIPS_CACHE_NOT_PRESENT;

	/* Ignore anything but MIPSxx processors */
	if (!(c->isa_level & (MIPS_CPU_ISA_M32R1 | MIPS_CPU_ISA_M32R2 |
			      MIPS_CPU_ISA_M64R1 | MIPS_CPU_ISA_M64R2 |
			      MIPS_CPU_ISA_M32R6 | MIPS_CPU_ISA_M64R6)))
		return 0;

	/* Does this MIPS32/MIPS64 CPU have a config2 register? */
	config1 = read_c0_config1();
	if (!(config1 & MIPS_CONF_M))
		return 0;

	config2 = read_c0_config2();

	if (cpu_has_l2c || !(config2 & ~(MIPS_CONF_M|MIPS_CONF2_SU))) {
#ifdef CONFIG_MIPS_CMP
		if (!cm3_l2_setup())
#endif
			return 0;
	} else {
		if (!mips_sc_is_activated(c))
			return 0;

		tmp = (config2 >> 8) & 0x0f;
		if (0 <= tmp && tmp <= 7)
			c->scache.sets = 64 << tmp;
		else
			return 0;

		tmp = (config2 >> 0) & 0x0f;
		if (0 <= tmp && tmp <= 7)
			c->scache.ways = tmp + 1;
		else
			return 0;
	}

	c->scache.waysize = c->scache.sets * c->scache.linesz;
	c->scache.waybit = __ffs(c->scache.waysize);

	c->scache.flags &= ~MIPS_CACHE_NOT_PRESENT;

	return 1;
}

int __cpuinit mips_sc_init(void)
{
	int found = mips_sc_probe();
	if (found) {
		mips_sc_enable();
		bcops = &mips_sc_ops;
	} else
		cpu_data[0].options &= ~MIPS_CPU_CM2_L2SYNC;
	return found;
}
