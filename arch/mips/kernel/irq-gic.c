/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Ralf Baechle (ralf@linux-mips.org)
 * Copyright (C) 2012 MIPS Technologies, Inc.  All rights reserved.
 */
#include <linux/bitmap.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/irq.h>
#include <linux/clocksource.h>

#include <linux/cpu.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/gic.h>
#include <asm/setup.h>
#include <asm/traps.h>
#include <asm/gcmpregs.h>
#include <linux/hardirq.h>
#include <asm-generic/bitops/find.h>
#include <asm/irq_cpu.h>

unsigned int gic_frequency;
unsigned int gic_present;
unsigned long _gic_base;
unsigned int gic_irq_base;
unsigned int gic_irq_flags[GIC_NUM_INTRS];

/* The index into this array is the vector # of the interrupt. */
struct gic_shared_intr_map gic_shared_intr_map[GIC_NUM_INTRS];

static struct gic_pcpu_mask pcpu_masks[NR_CPUS];
static struct gic_pending_regs pending_regs[NR_CPUS];
static struct gic_intrmask_regs intrmask_regs[NR_CPUS];

#if defined(CONFIG_CSRC_GIC) || defined(CONFIG_CEVT_GIC)
cycle_t gic_read_count(void)
{
	unsigned int hi, hi2, lo;

	do {
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_63_32), hi);
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_31_00), lo);
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_63_32), hi2);
	} while (hi2 != hi);

	return (((cycle_t) hi) << 32) + lo;
}

void gic_write_compare(cycle_t cnt)
{
	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_HI),
				(int)(cnt >> 32));
	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_LO),
				(int)(cnt & 0xffffffff));
}

cycle_t gic_read_compare(void)
{
	unsigned int hi, lo;

	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_HI), hi);
	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_LO), lo);

	return (((cycle_t) hi) << 32) + lo;
}
#endif

unsigned int gic_get_timer_pending(void)
{
	unsigned int vpe_pending;

	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR), 0);
	GICREAD(GIC_REG(VPE_OTHER, GIC_VPE_PEND), vpe_pending);
	return (vpe_pending & GIC_VPE_PEND_TIMER_MSK);
}

void gic_bind_eic_interrupt(int irq, int set)
{
	/* Convert irq vector # to hw int # */
	irq -= GIC_PIN_TO_VEC_OFFSET;

	/* Set irq to use shadow set */
	GICWRITE(GIC_REG_ADDR(VPE_LOCAL, GIC_VPE_EIC_SS(irq)), set);
}

void gic_send_ipi(unsigned int intr)
{
	GICWRITE(GIC_REG(SHARED, GIC_SH_WEDGE), 0x80000000 | intr);
}

static void gic_eic_irq_dispatch(void)
{
	unsigned int cause = read_c0_cause();
	int irq;

	irq = (cause & ST0_IM) >> STATUSB_IP2;
	if (irq == 0)
		irq = -1;

	if (irq >= 0)
		do_IRQ(gic_irq_base + irq);
	else
		spurious_interrupt();
}

static void __init vpe_local_setup(void)
{
	unsigned long timer_intr = GIC_INT_TMR;
	unsigned long perf_intr = GIC_INT_PERFCTR;
	unsigned int vpe_ctl;

	if (cpu_has_veic) {
		/*
		 * GIC timer interrupt -> CPU HW Int X (vector X+2) ->
		 * map to pin X+2-1 (since GIC adds 1)
		 */
		timer_intr += (GIC_CPU_TO_VEC_OFFSET - GIC_PIN_TO_VEC_OFFSET);
		/*
		 * GIC perfcnt interrupt -> CPU HW Int X (vector X+2) ->
		 * map to pin X+2-1 (since GIC adds 1)
		 */
		perf_intr += (GIC_CPU_TO_VEC_OFFSET - GIC_PIN_TO_VEC_OFFSET);
	}

	/*
	 * Setup the default performance counter timer interrupts
	 */

	/* Are Interrupts locally routable? */
	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_CTL), vpe_ctl);
	if (vpe_ctl & GIC_VPE_CTL_TIMER_RTBL_MSK) {
		if (cp0_compare_irq >= 2)
			timer_intr = cp0_compare_irq - 2;
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_TIMER_MAP),
			 GIC_MAP_TO_PIN_MSK | timer_intr);
		mips_smp_c0_status_mask |= (0x400 << timer_intr);
	}
	if (cpu_has_veic) {
		set_vi_handler(timer_intr + GIC_PIN_TO_VEC_OFFSET,
			gic_eic_irq_dispatch);
		gic_shared_intr_map[timer_intr + GIC_PIN_TO_VEC_OFFSET].local_intr_mask |= GIC_VPE_RMASK_TIMER_MSK;
	}

	if (vpe_ctl & GIC_VPE_CTL_PERFCNT_RTBL_MSK) {
		if (cp0_perfcount_irq >= 2)
			perf_intr = cp0_perfcount_irq - 2;
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_PERFCTR_MAP),
			 GIC_MAP_TO_PIN_MSK | perf_intr);
		mips_smp_c0_status_mask |= (0x400 << perf_intr);
	}
	if (cpu_has_veic) {
		set_vi_handler(perf_intr + GIC_PIN_TO_VEC_OFFSET, gic_eic_irq_dispatch);
		gic_shared_intr_map[perf_intr + GIC_PIN_TO_VEC_OFFSET].local_intr_mask |= GIC_VPE_RMASK_PERFCNT_MSK;
	}
}

unsigned int gic_compare_int(void)
{
	unsigned int pending;

	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_PEND), pending);
	if (pending & GIC_VPE_PEND_CMP_MSK)
		return 1;
	else
		return 0;
}

unsigned int gic_get_int(void)
{
	unsigned int i;
	unsigned long *pending, *intrmask, *pcpu_mask;
	unsigned long *pending_abs, *intrmask_abs;

	/* Get per-cpu bitmaps */
	pending = pending_regs[smp_processor_id()].pending;
	intrmask = intrmask_regs[smp_processor_id()].intrmask;
	pcpu_mask = pcpu_masks[smp_processor_id()].pcpu_mask;

	pending_abs = (unsigned long *) GIC_REG_ABS_ADDR(SHARED,
							 GIC_SH_PEND_31_0_OFS);
	intrmask_abs = (unsigned long *) GIC_REG_ABS_ADDR(SHARED,
							  GIC_SH_MASK_31_0_OFS);

	for (i = 0; i < BITS_TO_LONGS(GIC_NUM_INTRS); i++) {
		GICREAD(*pending_abs, pending[i]);
		GICREAD(*intrmask_abs, intrmask[i]);
		pending_abs++;
		intrmask_abs++;
	}

	bitmap_and(pending, pending, intrmask, GIC_NUM_INTRS);
	bitmap_and(pending, pending, pcpu_mask, GIC_NUM_INTRS);

	return find_first_bit(pending, GIC_NUM_INTRS);
}

static void gic_mask_irq(struct irq_data *d)
{
	GIC_CLR_INTR_MASK(d->irq - gic_irq_base);
}

static void gic_unmask_irq(struct irq_data *d)
{
	GIC_SET_INTR_MASK(d->irq - gic_irq_base);
}

static DEFINE_SPINLOCK(gic_lock);

#ifdef CONFIG_SMP

static int gic_set_affinity(struct irq_data *d, const struct cpumask *cpumask,
			    bool force)
{
	unsigned int irq = (d->irq - gic_irq_base);
	cpumask_t	tmp = CPU_MASK_NONE;
	unsigned long	flags;
	int		i;

	cpumask_and(&tmp, cpumask, cpu_online_mask);
	if (cpus_empty(tmp))
		return -1;

	/* Assumption : cpumask refers to a single CPU */
	spin_lock_irqsave(&gic_lock, flags);

	/* Re-route this IRQ */
	GIC_SH_MAP_TO_VPE_SMASK(irq, first_cpu(tmp));

	/* Update the pcpu_masks */
	for (i = 0; i < NR_CPUS; i++)
		clear_bit(irq, pcpu_masks[i].pcpu_mask);
	set_bit(irq, pcpu_masks[first_cpu(tmp)].pcpu_mask);

	cpumask_copy(d->affinity, cpumask);
	spin_unlock_irqrestore(&gic_lock, flags);

	return IRQ_SET_MASK_OK_NOCOPY;
}
#endif

static struct irq_chip gic_irq_controller = {
	.name			=	"MIPS GIC",
	.irq_ack		=	gic_irq_ack,
	.irq_mask		=	gic_mask_irq,
	.irq_mask_ack		=	gic_mask_irq,
	.irq_unmask		=	gic_unmask_irq,
	.irq_eoi		=	gic_finish_irq,
#ifdef CONFIG_SMP
	.irq_set_affinity	=	gic_set_affinity,
#endif
};

static void __init gic_setup_intr(unsigned int intr,
	unsigned int pin, unsigned int polarity, unsigned int trigtype,
	unsigned int flags)
{
	struct gic_shared_intr_map *map_ptr;
	unsigned int gic_vpe;
	unsigned int cpu;

	/* Setup Intr to Pin mapping */
	if (pin & GIC_MAP_TO_NMI_MSK) {
		GICWRITE(GIC_REG_ADDR(SHARED, GIC_SH_MAP_TO_PIN(intr)), pin);
		/* FIXME: hack to route NMI to all cpu's */
		for (cpu = 0; cpu < NR_CPUS; cpu += 32) {
			GICWRITE(GIC_REG_ADDR(SHARED,
					  GIC_SH_MAP_TO_VPE_REG_OFF(intr, cpu)),
				 0xffffffff);
		}
	} else {
		GICWRITE(GIC_REG_ADDR(SHARED, GIC_SH_MAP_TO_PIN(intr)),
			 GIC_MAP_TO_PIN_MSK | pin);
		/* Setup Intr to CPU mapping */
		gic_vpe = GIC_ID(smp_processor_id());
		GIC_SH_MAP_TO_VPE_SMASK(intr, gic_vpe);
		if (cpu_has_veic) {
			set_vi_handler(pin + GIC_PIN_TO_VEC_OFFSET,
				gic_eic_irq_dispatch);
			map_ptr = &gic_shared_intr_map[pin + GIC_PIN_TO_VEC_OFFSET];
			if (map_ptr->num_shared_intr >= GIC_MAX_SHARED_INTR)
				BUG();
			map_ptr->intr_list[map_ptr->num_shared_intr++] = intr;
		}
	}

	/* Setup Intr Polarity */
	GIC_SET_POLARITY(intr, polarity);

	/* Setup Intr Trigger Type */
	GIC_SET_TRIGGER(intr, trigtype);

	/* Init Intr Masks */
	GIC_CLR_INTR_MASK(intr);
	/* Initialise per-cpu Interrupt software masks */
	if (flags & GIC_FLAG_IPI)
		set_bit(intr, pcpu_masks[smp_processor_id()].pcpu_mask);
	if (((flags & GIC_FLAG_TRANSPARENT) && (cpu_has_veic == 0)) ||
	    (flags & GIC_FLAG_IPI))
		GIC_SET_INTR_MASK(intr);
	if (trigtype == GIC_TRIG_EDGE)
		gic_irq_flags[intr] |= GIC_TRIG_EDGE;
}

static int _gic_intr_map_size;
static struct gic_intr_map *_gic_intr_map;

void vpe_gic_setup(void)
{
	unsigned int i, cpu;
	unsigned int pin_offset = 0;

	/*
	 * In EIC mode, the HW_INT# is offset by (2-1). Need to subtract
	 * one because the GIC will add one (since 0=no intr).
	 */
	if (cpu_has_veic)
		pin_offset = (GIC_CPU_TO_VEC_OFFSET - GIC_PIN_TO_VEC_OFFSET);

	/* Setup specifics */
	spin_lock(&gic_lock);
	for (i = 0; i < _gic_intr_map_size; i++) {
		cpu = _gic_intr_map[i].cpunum;
		if (cpu == GIC_UNUSED)
			continue;
		if (cpu == 0 && i != 0 && _gic_intr_map[i].flags == 0)
			continue;
		if (cpu != smp_processor_id())
			continue;
		gic_setup_intr(i,
			_gic_intr_map[i].pin + pin_offset,
			_gic_intr_map[i].polarity,
			_gic_intr_map[i].trigtype,
			_gic_intr_map[i].flags);
	}
	spin_unlock(&gic_lock);

	vpe_local_setup();
}

static void __init gic_basic_init(int numintrs,
			struct gic_intr_map *intrmap, int mapsize)
{
	unsigned int i;

	board_bind_eic_interrupt = &gic_bind_eic_interrupt;

	/* Setup defaults, no need to spin_lock because it is a boot */
	for (i = 0; i < numintrs; i++) {
		GIC_SET_POLARITY(i, GIC_POL_POS);
		GIC_SET_TRIGGER(i, GIC_TRIG_LEVEL);
		GIC_CLR_INTR_MASK(i);
		if (i < GIC_NUM_INTRS) {
			gic_irq_flags[i] = 0;
			gic_shared_intr_map[i].num_shared_intr = 0;
			gic_shared_intr_map[i].local_intr_mask = 0;
		}
	}

	_gic_intr_map_size = mapsize;
	_gic_intr_map = intrmap;
	vpe_gic_setup();
}

void __init gic_init(unsigned long gic_base_addr,
		     unsigned long gic_addrspace_size,
		     struct gic_intr_map *intr_map, unsigned int intr_map_size,
		     unsigned int irqbase)
{
	unsigned int gicconfig;
	int numintrs;

	_gic_base = (unsigned long) ioremap_nocache(gic_base_addr,
						    gic_addrspace_size);
	gic_irq_base = irqbase;

	GICREAD(GIC_REG(SHARED, GIC_SH_CONFIG), gicconfig);
	gicconfig &= ~GIC_SH_CONFIG_COUNTSTOP_MSK;
	GICWRITE(GIC_REG(SHARED, GIC_SH_CONFIG), gicconfig);
	numintrs = (gicconfig & GIC_SH_CONFIG_NUMINTRS_MSK) >>
		   GIC_SH_CONFIG_NUMINTRS_SHF;
	numintrs = ((numintrs + 1) * 8);

	gic_basic_init(numintrs, intr_map, intr_map_size);

	gic_platform_init(numintrs, &gic_irq_controller);
}


#ifdef CONFIG_SYSFS
static ssize_t show_gic_global(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int n = 0;
	int i,j;
	int numints;

	n = snprintf(buf, PAGE_SIZE,
		"GIC Config Register\t\t%08x\n"
		"GIC CounterLo\t\t\t%08x\n"
		"GIC CounterHi\t\t\t%08x\n"
		"GIC Revision\t\t\t%08x\n"

		"Global Interrupt Polarity Registers:\t\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t\t%08x %08x %08x %08x\n"
		"Global Interrupt Trigger Type Registers:\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t\t%08x %08x %08x %08x\n"
		"Global Interrupt Dual Edge Registers:\t\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t\t%08x %08x %08x %08x\n"
		"Global Interrupt Write Edge Register:\t\t%08x\n"
		"Global Interrupt Reset Mask Registers:\t\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t\t%08x %08x %08x %08x\n"
		"Global Interrupt Set Mask Registers:\t\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t\t%08x %08x %08x %08x\n"
		"Global Interrupt Mask Registers:\t\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t\t%08x %08x %08x %08x\n"
		"Global Interrupt Pending Registers:\t\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t\t%08x %08x %08x %08x\n"
		,
		GIC_REG(SHARED,GIC_SH_CONFIG),
		GIC_REG(SHARED,GIC_SH_COUNTER_31_00),
		GIC_REG(SHARED,GIC_SH_COUNTER_63_32),
		GIC_REG(SHARED,GIC_SH_REVISIONID),
		GIC_REG(SHARED,GIC_SH_POL_31_0),        GIC_REG(SHARED,GIC_SH_POL_63_32),
		GIC_REG(SHARED,GIC_SH_POL_95_64),       GIC_REG(SHARED,GIC_SH_POL_127_96),
		GIC_REG(SHARED,GIC_SH_POL_159_128),     GIC_REG(SHARED,GIC_SH_POL_191_160),
		GIC_REG(SHARED,GIC_SH_POL_223_192),     GIC_REG(SHARED,GIC_SH_POL_255_224),
		GIC_REG(SHARED,GIC_SH_TRIG_31_0),       GIC_REG(SHARED,GIC_SH_TRIG_63_32),
		GIC_REG(SHARED,GIC_SH_TRIG_95_64),      GIC_REG(SHARED,GIC_SH_TRIG_127_96),
		GIC_REG(SHARED,GIC_SH_TRIG_159_128),    GIC_REG(SHARED,GIC_SH_TRIG_191_160),
		GIC_REG(SHARED,GIC_SH_TRIG_223_192),    GIC_REG(SHARED,GIC_SH_TRIG_255_224),
		GIC_REG(SHARED,GIC_SH_DUAL_31_0),       GIC_REG(SHARED,GIC_SH_DUAL_63_32),
		GIC_REG(SHARED,GIC_SH_DUAL_95_64),      GIC_REG(SHARED,GIC_SH_DUAL_127_96),
		GIC_REG(SHARED,GIC_SH_DUAL_159_128),    GIC_REG(SHARED,GIC_SH_DUAL_191_160),
		GIC_REG(SHARED,GIC_SH_DUAL_223_192),    GIC_REG(SHARED,GIC_SH_DUAL_255_224),
		GIC_REG(SHARED,GIC_SH_WEDGE),
		GIC_REG(SHARED,GIC_SH_RMASK_31_0),      GIC_REG(SHARED,GIC_SH_RMASK_63_32),
		GIC_REG(SHARED,GIC_SH_RMASK_95_64),     GIC_REG(SHARED,GIC_SH_RMASK_127_96),
		GIC_REG(SHARED,GIC_SH_RMASK_159_128),   GIC_REG(SHARED,GIC_SH_RMASK_191_160),
		GIC_REG(SHARED,GIC_SH_RMASK_223_192),   GIC_REG(SHARED,GIC_SH_RMASK_255_224),
		GIC_REG(SHARED,GIC_SH_SMASK_31_0),      GIC_REG(SHARED,GIC_SH_SMASK_63_32),
		GIC_REG(SHARED,GIC_SH_SMASK_95_64),     GIC_REG(SHARED,GIC_SH_SMASK_127_96),
		GIC_REG(SHARED,GIC_SH_SMASK_159_128),   GIC_REG(SHARED,GIC_SH_SMASK_191_160),
		GIC_REG(SHARED,GIC_SH_SMASK_223_192),   GIC_REG(SHARED,GIC_SH_SMASK_255_224),
		GIC_REG(SHARED,GIC_SH_MASK_31_0),       GIC_REG(SHARED,GIC_SH_MASK_63_32),
		GIC_REG(SHARED,GIC_SH_MASK_95_64),      GIC_REG(SHARED,GIC_SH_MASK_127_96),
		GIC_REG(SHARED,GIC_SH_MASK_159_128),    GIC_REG(SHARED,GIC_SH_MASK_191_160),
		GIC_REG(SHARED,GIC_SH_MASK_223_192),    GIC_REG(SHARED,GIC_SH_MASK_255_224),
		GIC_REG(SHARED,GIC_SH_PEND_31_0),       GIC_REG(SHARED,GIC_SH_PEND_63_32),
		GIC_REG(SHARED,GIC_SH_PEND_95_64),      GIC_REG(SHARED,GIC_SH_PEND_127_96),
		GIC_REG(SHARED,GIC_SH_PEND_159_128),    GIC_REG(SHARED,GIC_SH_PEND_191_160),
		GIC_REG(SHARED,GIC_SH_PEND_223_192),    GIC_REG(SHARED,GIC_SH_PEND_255_224)
	);

	numints = (GIC_REG(SHARED,GIC_SH_CONFIG) & GIC_SH_CONFIG_NUMINTRS_MSK) >> GIC_SH_CONFIG_NUMINTRS_SHF;
	numints = (numints + 1) * 8;

	n += snprintf(buf+n, PAGE_SIZE-n,
		"\nGlobal Interrupt Map SrcX to Pin:\n");
	for (i=0; i<numints; i++) {

		if ((i % 8) == 0)
			n += snprintf(buf+n, PAGE_SIZE-n, "%02x:\t",i);
		n += snprintf(buf+n, PAGE_SIZE-n,
			"%08x ",GIC_REG_ADDR(SHARED,GIC_SH_MAP_TO_PIN(i)));
		if ((i % 8) == 7)
			n += snprintf(buf+n, PAGE_SIZE-n, "\n");
	};

	n += snprintf(buf+n, PAGE_SIZE-n,
		"\nGlobal Interrupt Map SrcX to VPE:\n");
	for (i=0; i<numints; i++) {
		if ((i % 4) == 0)
			n += snprintf(buf+n, PAGE_SIZE-n, "%02x:\t",i);
		for (j=0; j<2; j++) {
			n += snprintf(buf+n, PAGE_SIZE-n,
			    "%08x ",GIC_REG_ADDR(SHARED,GIC_SH_INTR_MAP_TO_VPE_BASE_OFS + ((j * 4) + (i * 32))));
		};
		n += snprintf(buf+n, PAGE_SIZE-n, "\t");
		if ((i % 4) == 3)
			n += snprintf(buf+n, PAGE_SIZE-n, "\n");
	};

	if (gcmp3_present) {
		n += snprintf(buf+n, PAGE_SIZE-n,
			"EJTAG Break register\t\t\t%08x%08x\n"
			"Debug Team ID low\t\t\t%08x%08x\n"
			"Debug Team ID high\t\t\t%08x%08x\n"
			"Debug Team ID for external VC\t\t%08x\n"
			"GIC Debug Mode Config Register\t\t%08x\n"
			"GIC Shared DINT Group Participate\t%08x%08x\n"
			"GIC Shared Debug Mode Status\t\t%08x%08x\n"
			,
			GIC_REGhi(SHARED,GIC_EJTAG), GIC_REG(SHARED,GIC_EJTAG),
			GIC_REGhi(SHARED,GIC_DTLOW), GIC_REG(SHARED,GIC_DTLOW),
			GIC_REGhi(SHARED,GIC_DTHI), GIC_REG(SHARED,GIC_DTHI),
			GIC_REG(SHARED,GIC_DTEXT),
			GIC_REG(SHARED,GIC_DMODECONF),
			GIC_REGhi(SHARED,GIC_DINTGRP), GIC_REG(SHARED,GIC_DINTGRP),
			GIC_REGhi(SHARED,GIC_DMSTATUS), GIC_REG(SHARED,GIC_DMSTATUS)
		);
	} else {
		n += snprintf(buf+n, PAGE_SIZE-n,
			"\nDINT Send to Group Register\t\t%08x\n",
			GIC_REG(SHARED,GIC_DINT));
	}

	return n;
}

static ssize_t show_gic_local(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	unsigned long irq_flags;
	int n = 0;
	int i;

	local_irq_save(irq_flags);
	GIC_REG(VPE_LOCAL,GIC_VPE_OTHER_ADDR) = (dev->id);

	n += snprintf(buf+n, PAGE_SIZE-n,
		"Local Interrupt Control Register:\t\t%08x\n"
		"Local Interrupt Pending Register:\t\t%08x\n"
		"Local Mask Register:\t\t\t\t%08x\n"
		"Local Reset Mask Register:\t\t\t%08x\n"
		"Local Set Mask Register:\t\t\t%08x\n"
		"Local WatchDog Map-to-Pin Register:\t\t%08x\n"
		"Local GIC Counter/Compare Map-to-Pin Register:\t%08x\n"
		"Local CPU Timer Map-to-Pin Register:\t\t%08x\n"
		"Local CPU Fast Debug Channel Map-to-Pin:\t%08x\n"
		"Local Perf Counter Map-to-Pin Register:\t\t%08x\n"
		"Local SWInt0 Map-to-Pin Register:\t\t%08x\n"
		"Local SWInt1 Map-to-Pin Register:\t\t%08x\n"
		"VPE-Other Addressing Register:\t\t\t%08x\n"
		"VPE-Local Identification Register:\t\t%08x\n"
		"Programmable/Watchdog Timer0 Config Register:\t\t%08x\n"
		"Programmable/Watchdog Timer0 Count Register:\t\t%08x\n"
		"Programmable/Watchdog Timer0 Initial Count Register:\t%08x\n"
		"CompareLo Register:\t\t\t\t%08x\n"
		"CompareHi Register:\t\t\t\t%08x\n"
		,
		GIC_REG(VPE_OTHER,GIC_VPE_CTL),
		GIC_REG(VPE_OTHER,GIC_VPE_PEND),
		GIC_REG(VPE_OTHER,GIC_VPE_MASK),
		GIC_REG(VPE_OTHER,GIC_VPE_RMASK),
		GIC_REG(VPE_OTHER,GIC_VPE_SMASK),
		GIC_REG(VPE_OTHER,GIC_VPE_WD_MAP),
		GIC_REG(VPE_OTHER,GIC_VPE_COMPARE_MAP),
		GIC_REG(VPE_OTHER,GIC_VPE_TIMER_MAP),
		GIC_REG(VPE_OTHER,GIC_VPE_FDEBUG_MAP),
		GIC_REG(VPE_OTHER,GIC_VPE_PERFCTR_MAP),
		GIC_REG(VPE_OTHER,GIC_VPE_SWINT0_MAP),
		GIC_REG(VPE_OTHER,GIC_VPE_SWINT1_MAP),
		GIC_REG(VPE_OTHER,GIC_VPE_OTHER_ADDR),
		GIC_REG(VPE_OTHER,GIC_VPE_ID),
		GIC_REG(VPE_OTHER,GIC_VPE_WD_CONFIG0),
		GIC_REG(VPE_OTHER,GIC_VPE_WD_COUNT0),
		GIC_REG(VPE_OTHER,GIC_VPE_WD_INITIAL0),
		GIC_REG(VPE_OTHER,GIC_VPE_COMPARE_LO),
		GIC_REG(VPE_OTHER,GIC_VPE_COMPARE_HI)
	);

	n += snprintf(buf+n, PAGE_SIZE-n,
		"\nEIC Shadow Set for Interrupt SrcX:\n");
	for (i=0; i<64; i++) {
		if ((i % 8) == 0)
			n += snprintf(buf+n, PAGE_SIZE-n, "%02x:\t",i);
		n += snprintf(buf+n, PAGE_SIZE-n,
			"%08x ",GIC_REG_ADDR(VPE_OTHER,GIC_VPE_EIC_SS(i)));
		if ((i % 8) == 7)
			n += snprintf(buf+n, PAGE_SIZE-n, "\n");
	};

	n += snprintf(buf+n, PAGE_SIZE-n,
		"\nGuest Counter Offset\t\t\t%08x\n",
		GIC_REG(VPE_OTHER,GIC_VPE_GCTR_OFFSET));

	n += snprintf(buf+n, PAGE_SIZE-n,
		"\nVPE Local DINT Group Participate Register:\t%08x\n"
		"VPE Local DebugBreak Group Register:\t\t%08x\n"
		,
		GIC_REG(VPE_OTHER,GIC_VPE_DINT),
		GIC_REG(VPE_OTHER,GIC_VPE_DEBUG_BREAK)
	);

	local_irq_restore(irq_flags);

	return n;
}

static DEVICE_ATTR(gic_global, 0444, show_gic_global, NULL);
static DEVICE_ATTR(gic_local, 0400, show_gic_local, NULL);

static struct bus_type gic_subsys = {
	.name = "gic",
	.dev_name = "gic",
};



static __cpuinit int gic_add_vpe(int cpu)
{
	struct device *dev;
	int err;
	char name[16];

	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->id = cpu;
	dev->bus = &gic_subsys;
	snprintf(name, sizeof name, "vpe%d",cpu);
	dev->init_name = name;

	err = device_register(dev);
	if (err)
		return err;

	err = device_create_file(dev, &dev_attr_gic_local);
	if (err)
		return err;

	return 0;
}

static int __init init_gic_sysfs(void)
{
	int rc;
	int vpeN;
	int vpe;
	int gvpe;

	if (!gic_present)
		return 0;

	rc = subsys_system_register(&gic_subsys, NULL);
	if (rc)
		return rc;

	rc = device_create_file(gic_subsys.dev_root, &dev_attr_gic_global);
	if (rc)
		return rc;

	vpeN = ((GIC_REG(SHARED,GIC_SH_CONFIG) & GIC_SH_CONFIG_NUMVPES_MSK) >> GIC_SH_CONFIG_NUMVPES_SHF) + 1;
	for (vpe=0; vpe<vpeN; vpe++) {
		gvpe = cpu_data[vpe].g_vpe;
		/* if gvpe == 0 it mean VPE is skipped during boot */
		if (vpe && !gvpe)
			continue;
		rc = gic_add_vpe(gvpe);
		if (rc)
			return rc;
	}

	return 0;
}

device_initcall_sync(init_gic_sysfs);

#endif /* CONFIG_SYSFS */
