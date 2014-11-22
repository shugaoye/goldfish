/*
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
 * Copyright (C) 2007 MIPS Technologies, Inc.
 *    Chris Dearman (chris@mips.com)
 */

#undef DEBUG

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/cpumask.h>
#include <linux/interrupt.h>
#include <linux/compiler.h>
#include <linux/cpu.h>

#include <linux/atomic.h>
#include <asm/cacheflush.h>
#include <asm/cpu.h>
#include <asm/processor.h>
#include <asm/hardirq.h>
#include <asm/mmu_context.h>
#include <asm/smp.h>
#include <asm/time.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/mips_mt.h>
#include <asm/amon.h>
#include <asm/gic.h>
#include <asm/gcmpregs.h>
#include <asm/bootinfo.h>
#include <asm/irq_cpu.h>

/* we need to keep _gcmp_base in a separate cacheline for uncoherent read */
unsigned long _gcmp_base __cacheline_aligned;
int gcmp_present __cacheline_aligned = -1;

static void ipi_call_function(unsigned int cpu)
{
	pr_debug("CPU%d: %s cpu %d status %08x\n",
		 smp_processor_id(), __func__, cpu, read_c0_status());

	gic_send_ipi(plat_ipi_call_int_xlate(cpu));
}


static void ipi_resched(unsigned int cpu)
{
	pr_debug("CPU%d: %s cpu %d status %08x\n",
		 smp_processor_id(), __func__, cpu, read_c0_status());

	gic_send_ipi(plat_ipi_resched_int_xlate(cpu));
}

/*
 * FIXME: This isn't restricted to CMP
 * The SMVP kernel could use GIC interrupts if available
 */
void cmp_send_ipi_single(int cpu, unsigned int action)
{
	unsigned long flags;

	local_irq_save(flags);

	switch (action) {
	case SMP_CALL_FUNCTION:
		ipi_call_function(cpu);
		break;

	case SMP_RESCHEDULE_YOURSELF:
		ipi_resched(cpu);
		break;
	}

	local_irq_restore(flags);
}

static void cmp_send_ipi_mask(const struct cpumask *mask, unsigned int action)
{
	unsigned int i;

	for_each_cpu(i, mask)
		cmp_send_ipi_single(i, action);
}

#ifdef CONFIG_EVA
static unsigned long bev_location = -1;

static int rd_bev_location(char *p)
{
	if (p && strlen(p)) {
		bev_location = memparse(p, &p);
	} else
		bev_location = 0xbfc00000;
	return 0;
}
early_param("force-bev-location", rd_bev_location);

static void BEV_overlay_segment_map_check(unsigned long excBase,
	unsigned long excMask, unsigned long excSize)
{
	unsigned long addr;

	if ((excBase == (IO_BASE + IO_SHIFT)) && (excSize == IO_SIZE))
		return;

	printk("WARNING: BEV overlay segment doesn't fit whole I/O reg space, NMI/EJTAG/sRESET may not work\n");

	if ((MAP_BASE < (excBase + excSize)) && (excBase < VMALLOC_END))
		panic("BEV Overlay segment overlaps VMALLOC area\n");
#ifdef CONFIG_HIGHMEM
	if ((PKMAP_BASE < (excBase + excSize)) &&
	    (excBase < (PKMAP_BASE + (PAGE_SIZE*(LAST_PKMAP-1)))))
		panic("BEV Overlay segment overlaps HIGHMEM/PKMAP area\n");
#endif
	for (addr = excBase; addr < (excBase + excSize); addr += PAGE_SIZE) {
		if (page_is_ram(__pa(addr>>PAGE_SHIFT)))
			panic("BEV Overlay segment overlaps memory at %lx\n",addr);
	}
}

void BEV_overlay_segment(void)
{
	unsigned long RExcBase;
	unsigned long RExcExtBase;
	unsigned long excBase;
	unsigned long excMask;
	unsigned long excSize;
	unsigned long addr;
	char *p;

	printk("IO: BASE = 0x%lx, SHIFT = 0x%lx, SIZE = 0x%lx\n",IO_BASE, IO_SHIFT, IO_SIZE);
	RExcBase = GCMPCLCB(RESETBASE);
	RExcExtBase = GCMPCLCB(RESETBASEEXT);
	printk("GCMP base addr = 0x%lx, CLB: ResetExcBase = 0x%lx, ResetExcExtBase = 0x%lx\n",
		_gcmp_base,RExcBase,RExcExtBase);
	if ( !(RExcExtBase & 0x1) )
		return;

	if (bev_location == -1) {
		if ((p = strstr(arcs_cmdline, "force-bev-location")))
			rd_bev_location(p);
	}
	if (bev_location != -1) {
		addr = fls((IO_BASE + IO_SHIFT) ^ bev_location);
nextSize:
		if (addr > 28)
			panic("enforced BEV location is too far from I/O reg space\n");

		excMask = (0xffffffffUL >> (32 - addr));
		excBase = bev_location & ~excMask;
		if (((IO_BASE + IO_SHIFT + IO_SIZE - 1) & ~excMask) != excBase) {
			addr++;
			goto nextSize;
		}
		excSize = ((excBase | excMask) + 1) - excBase;
		printk("Setting BEV = 0x%lx, Overlay segment = 0x%lx, size = 0x%lx\n",
			bev_location, excBase, excSize);

		BEV_overlay_segment_map_check(excBase, excMask, excSize);

		GCMPCLCB(RESETBASEEXT) = (GCMPCLCB(RESETBASEEXT) &
			~GCMP_CCB_RESETEXTBASE_BEV_MASK_MSK) |
			(excMask & GCMP_CCB_RESETEXTBASE_BEV_MASK_MSK);
		GCMPCLCB(RESETBASE) = (GCMPCLCB(RESETBASE) & ~GCMP_CCB_RESETBASE_BEV_MSK) |
			bev_location;
		RExcBase = GCMPCLCB(RESETBASE);
		RExcExtBase = GCMPCLCB(RESETBASEEXT);

		return;
	}

	excBase = RExcBase & GCMP_CCB_RESETBASE_BEV_MSK;
	excMask = (RExcExtBase & GCMP_CCB_RESETEXTBASE_BEV_MASK_MSK) |
		    GCMP_CCB_RESETEXTBASE_BEV_MASK_LOWBITS;
	excBase &= ~excMask;
	excSize = ((excBase | excMask) + 1) - excBase;
	printk("BEV Overlay segment = 0x%lx, size = 0x%lx\n",excBase, excSize);

	BEV_overlay_segment_map_check(excBase, excMask, excSize);
}
#endif

static void cmp_init_secondary(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;

	if (!cpu_has_veic) {
		set_c0_status(mips_smp_c0_status_mask);
		back_to_back_c0_hazard();
		printk("CPU%d: status register %08x\n", smp_processor_id(), read_c0_status());
	}
	c->core = (read_c0_ebase() & 0x3ff) >> (fls(smp_num_siblings)-1);
#if defined(CONFIG_MIPS_MT_SMP) || defined(CONFIG_MIPS_MT_SMTC)
	if (cpu_has_mipsmt)
		c->vpe_id = (read_c0_tcbind() >> TCBIND_CURVPE_SHIFT) &
			TCBIND_CURVPE;
#endif
#ifdef CONFIG_MIPS_MT_SMTC
	c->tc_id  = (read_c0_tcbind() & TCBIND_CURTC) >> TCBIND_CURTC_SHIFT;
#endif

#ifdef CONFIG_EVA
	if (gcmp_present)
		BEV_overlay_segment();
#endif
}

static void cmp_smp_finish(void)
{
	pr_debug("SMPCMP: CPU%d: %s\n", smp_processor_id(), __func__);

	/* CDFIXME: remove this? */
	write_c0_compare(read_c0_count() + (8 * mips_hpt_frequency / HZ));

#ifdef CONFIG_MIPS_MT_FPAFF
	/* If we have an FPU, enroll ourselves in the FPU-full mask */
	if (cpu_has_fpu)
		cpu_set(smp_processor_id(), mt_fpu_cpumask);
#endif /* CONFIG_MIPS_MT_FPAFF */

	local_irq_enable();
}

static void cmp_cpus_done(void)
{
	pr_debug("SMPCMP: CPU%d: %s\n", smp_processor_id(), __func__);
}

/*
 * Setup the PC, SP, and GP of a secondary processor and start it running
 * smp_bootstrap is the place to resume from
 * __KSTK_TOS(idle) is apparently the stack pointer
 * (unsigned long)idle->thread_info the gp
 */
static void cmp_boot_secondary(int cpu, struct task_struct *idle)
{
	struct thread_info *gp = task_thread_info(idle);
	unsigned long sp = __KSTK_TOS(idle);
	unsigned long pc = (unsigned long)&smp_bootstrap;
	unsigned long a0 = 0;

	pr_debug("SMPCMP: CPU%d: %s cpu %d\n", smp_processor_id(),
		__func__, cpu);

#if 0
	/* Needed? */
	local_flush_icache_range((unsigned long)gp,
			   (unsigned long)(gp + sizeof(struct thread_info)));
#endif

	amon_cpu_start(cpu, pc, sp, (unsigned long)gp, a0);
}

/*
 * Common setup before any secondaries are started
 */
void __init cmp_smp_setup(void)
{
	int i;
	int ncpu = 0;

	pr_debug("SMPCMP: CPU%d: %s\n", smp_processor_id(), __func__);

#ifdef CONFIG_MIPS_MT_FPAFF
	/* If we have an FPU, enroll ourselves in the FPU-full mask */
	if (cpu_has_fpu)
		cpu_set(0, mt_fpu_cpumask);
#endif /* CONFIG_MIPS_MT_FPAFF */

	for (i = 1; i < NR_CPUS; i++) {
		if (amon_cpu_avail(i)) {
			set_cpu_possible(i, true);
			__cpu_number_map[i]	= ++ncpu;
			__cpu_logical_map[ncpu] = i;
		}
	}

	if (cpu_has_mipsmt) {
		unsigned int nvpe = 1;
#ifdef CONFIG_MIPS_MT_SMP
		unsigned int mvpconf0 = read_c0_mvpconf0();

		nvpe = ((mvpconf0 & MVPCONF0_PVPE) >> MVPCONF0_PVPE_SHIFT) + 1;
#elif defined(CONFIG_MIPS_MT_SMTC)
		unsigned int mvpconf0 = read_c0_mvpconf0();

		nvpe = ((mvpconf0 & MVPCONF0_PTC) >> MVPCONF0_PTC_SHIFT) + 1;
#endif
		smp_num_siblings = nvpe;
	}
	pr_info("Detected %i available secondary CPU(s)\n", ncpu);
}

void __init cmp_prepare_cpus(unsigned int max_cpus)
{
	pr_debug("SMPCMP: CPU%d: %s max_cpus=%d\n",
		 smp_processor_id(), __func__, max_cpus);

	/*
	 * FIXME: some of these options are per-system, some per-core and
	 * some per-cpu
	 */
	mips_mt_set_cpuoptions();
}

struct plat_smp_ops cmp_smp_ops = {
	.send_ipi_single	= cmp_send_ipi_single,
	.send_ipi_mask		= cmp_send_ipi_mask,
	.init_secondary		= cmp_init_secondary,
	.smp_finish		= cmp_smp_finish,
	.cpus_done		= cmp_cpus_done,
	.boot_secondary		= cmp_boot_secondary,
	.smp_setup		= cmp_smp_setup,
	.prepare_cpus		= cmp_prepare_cpus,
};

/*
 * GCMP needs to be detected before any SMP initialisation
 */
int __init gcmp_probe(unsigned long addr, unsigned long size)
{
	unsigned long confaddr = 0;

	if (gcmp_present >= 0)
		return gcmp_present;

	if (cpu_has_mips_r2 && (read_c0_config3() & MIPS_CONF3_CMGCR)) {
		/* try CMGCRBase */
		confaddr = read_c0_cmgcrbase() << 4;
		_gcmp_base = (unsigned long) ioremap_nocache(confaddr, size);
		gcmp_present = ((GCMPGCB(GCMPB) & GCMP_GCB_GCMPB_GCMPBASE_MSK) == confaddr)? 1 : 0;
		if (gcmp_present) {
			/* reassign it to 'addr' */
			if (addr != confaddr)
				GCMPGCB(GCMPB) = (GCMPGCB(GCMPB) & ~GCMP_GCB_GCMPB_GCMPBASE_MSK) | addr;
			_gcmp_base = (unsigned long) ioremap_nocache(addr , size);
			gcmp_present = ((GCMPGCB(GCMPB) & GCMP_GCB_GCMPB_GCMPBASE_MSK) == confaddr)? 1 : 0;
			confaddr = addr;
			if (!gcmp_present) {
				/* reassignment failed, try CMGCRBase again */
				confaddr = read_c0_cmgcrbase() << 4;
				_gcmp_base = (unsigned long) ioremap_nocache(confaddr, size);
				gcmp_present =
					((GCMPGCB(GCMPB) & GCMP_GCB_GCMPB_GCMPBASE_MSK) == confaddr)? 1 : 0;
			}
		}
	}
	if (addr && (gcmp_present <= 0)) {
		/* try addr */
		_gcmp_base = (unsigned long) ioremap_nocache(addr, size);
		gcmp_present = ((GCMPGCB(GCMPB) & GCMP_GCB_GCMPB_GCMPBASE_MSK) == addr)? 1 : 0;
		confaddr = addr;
	}

	if (gcmp_present > 0) {
		printk("GCMP available\n");
		if (((GCMPGCB(GCMPREV) & GCMP_GCB_GCMPREV_MAJOR_MSK) >>
		      GCMP_GCB_GCMPREV_MAJOR_SHF) >= 6)
			cpu_data[0].options |= MIPS_CPU_CM2;
		if (cpu_has_cm2 && (size > 0x8000)) {
			GCMPGCB(GCML2S) = (confaddr + 0x8000) | GCMP_GCB_GCML2S_EN_MSK;
			cpu_data[0].options |= MIPS_CPU_CM2_L2SYNC;
			printk("L2-only SYNC available\n");
		}
		return gcmp_present;
	}

	gcmp_present = 0;
	return gcmp_present;
}

/* Return the number of IOCU's present */
int __init gcmp_niocu(void)
{
    return (gcmp_present > 0) ?
      (GCMPGCB(GC) & GCMP_GCB_GC_NUMIOCU_MSK) >> GCMP_GCB_GC_NUMIOCU_SHF :
      0;
}

/* Set GCMP region attributes */
void __init gcmp_setregion(int region, unsigned long base,
			  unsigned long mask, int type)
{
	GCMPGCBn(CMxBASE, region) = base;
	GCMPGCBn(CMxMASK, region) = mask | type;
}

#ifdef CONFIG_SYSFS
static ssize_t show_gcr_global(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int n = 0;

	n = snprintf(buf, PAGE_SIZE,
		"Global Config Register\t\t\t%08x\n"
		"GCR Base Register\t\t\t%08x\n"
		"Global CM Control Register\t\t%08x\n"
		"Global CM Control2 Register\t\t%08x\n"
		"Global CSR Access Privilege Register\t%08x\n"
		"GCR Revision Register\t\t\t%08x\n"
		"Global CM Error Mask Register\t\t%08x\n"
		"Global CM Error Cause Register\t\t%08x\n"
		"Global CM Error Address Register\t%08x\n"
		"Global CM Error Multiple Register\t%08x\n"
		"GCR Custom Base Register\t\t%08x\n"
		"GCR Custom Status Register\t\t%x\n"
		"Global L2 only Sync Register\t\t%08x\n"
		"GIC Base Address Register\t\t%08x\n"
		"CPC Base Address Register\t\t%08x\n"
		"Region0 Base Address Register\t\t%08x,\tMask\t%08x\n"
		"Region1 Base Address Register\t\t%08x,\tMask\t%08x\n"
		"Region2 Base Address Register\t\t%08x,\tMask\t%08x\n"
		"Region3 Base Address Register\t\t%08x,\tMask\t%08x\n"
		"GIC Status Register\t\t\t%x\n"
		"Cache Revision Register\t\t\t%08x\n"
		"CPC Status Register\t\t\t%x\n"
		"Attribute Region0 Base Address Register\t%08x,\tMask\t%08x\n"
		"Attribute Region0 Base Address Register\t%08x,\tMask\t%08x\n"
		"IOCU Revision Register\t\t\t%08x\n"
		"Attribute Region0 Base Address Register\t%08x,\tMask\t%08x\n"
		"Attribute Region0 Base Address Register\t%08x,\tMask\t%08x\n"
		,
		GCMPGCB(GC),
		GCMPGCB(GCMPB),
		GCMPGCB(GCMC),
		GCMPGCB(GCMC2),
		GCMPGCB(GCSRAP),
		GCMPGCB(GCMPREV),
		GCMPGCB(GCMEM),
		GCMPGCB(GCMEC),
		GCMPGCB(GCMEA),
		GCMPGCB(GCMEO),
		GCMPGCB(GCMCUS),
		GCMPGCB(GCMCST),
		GCMPGCB(GCML2S),
		GCMPGCB(GICBA),
		GCMPGCB(CPCBA),
		GCMPGCBn(CMxBASE, 0),   GCMPGCBn(CMxMASK, 0),
		GCMPGCBn(CMxBASE, 1),   GCMPGCBn(CMxMASK, 1),
		GCMPGCBn(CMxBASE, 2),   GCMPGCBn(CMxMASK, 2),
		GCMPGCBn(CMxBASE, 3),   GCMPGCBn(CMxMASK, 3),
		GCMPGCB(GICST),
		GCMPGCB(GCSHREV),
		GCMPGCB(CPCST),
		GCMPGCB(GAOR0BA),       GCMPGCB(GAOR0MASK),
		GCMPGCB(GAOR1BA),       GCMPGCB(GAOR1MASK),
		GCMPGCB(IOCUREV),
		GCMPGCB(GAOR2BA),       GCMPGCB(GAOR2MASK),
		GCMPGCB(GAOR3BA),       GCMPGCB(GAOR3MASK)
	);

	return n;
}

static ssize_t show_gcr_local(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int n = 0;

	GCMPCLCB(OTHER) = (dev->id)<<16;

	n += snprintf(buf+n, PAGE_SIZE-n,
		"Local Reset Release Register\t\t%08x\n"
		"Local Coherence Control Register\t%08x\n"
		"Local Config Register\t\t\t%08x\n"
		"Other Addressing Register\t\t%08x\n"
		"Local Reset Exception Base Register\t%08x\n"
		"Local Identification Register\t\t%08x\n"
		"Local Reset Exception Extended Base\t%08x\n"
		"Local TCID PRIORITYs\t\t\t%08x %08x %08x %08x\n"
		"\t\t\t\t\t%08x %08x %08x %08x %08x\n"
		,
		GCMPCOCB(RESETR),
		GCMPCOCB(COHCTL),
		GCMPCOCB(CFG),
		GCMPCOCB(OTHER),
		GCMPCOCB(RESETBASE),
		GCMPCOCB(ID),
		GCMPCOCB(RESETBASEEXT),
		GCMPCOCBn(TCIDxPRI, 0), GCMPCOCBn(TCIDxPRI, 1),
		GCMPCOCBn(TCIDxPRI, 2), GCMPCOCBn(TCIDxPRI, 3),
		GCMPCOCBn(TCIDxPRI, 4), GCMPCOCBn(TCIDxPRI, 5),
		GCMPCOCBn(TCIDxPRI, 6), GCMPCOCBn(TCIDxPRI, 7),
		GCMPCOCBn(TCIDxPRI, 8)
	);

	return n;
}

static DEVICE_ATTR(gcr_global, 0444, show_gcr_global, NULL);
static DEVICE_ATTR(gcr_local, 0444, show_gcr_local, NULL);

static struct bus_type gcmp_subsys = {
	.name = "gcmp",
	.dev_name = "gcmp",
};

static __cpuinit int gcmp_add_core(int cpu)
{
	struct device *dev;
	int err;
	char name[16];

	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->id = cpu;
	dev->bus = &gcmp_subsys;
	snprintf(name, sizeof name, "core%d",cpu);
	dev->init_name = name;

	err = device_register(dev);
	if (err)
		return err;

	err = device_create_file(dev, &dev_attr_gcr_local);
	if (err)
		return err;

	return 0;
}

static __cpuinit int gcmp_add_iocu(int cpu,int totcpu)
{
	struct device *dev;
	int err;
	char name[16];

	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* Ask Tom Berg @ IMGTec about more generic formula. LY22 */
	if (totcpu <= 4 )
		totcpu = 4;
	else
		totcpu = 6;

	dev->id = cpu + totcpu;
	dev->bus = &gcmp_subsys;
	snprintf(name, sizeof name, "iocu%d",cpu);
	dev->init_name = name;

	err = device_register(dev);
	if (err)
		return err;

	err = device_create_file(dev, &dev_attr_gcr_local);
	if (err)
		return err;

	return 0;
}

static int __init init_gcmp_sysfs(void)
{
	int rc;
	int cpuN, iocuN;
	int cpu;

	if (gcmp_present <= 0)
		return 0;

	rc = subsys_system_register(&gcmp_subsys, NULL);
	if (rc)
		return rc;

	rc = device_create_file(gcmp_subsys.dev_root, &dev_attr_gcr_global);
	if (rc)
		return rc;

	cpuN = ((GCMPGCB(GC) & GCMP_GCB_GC_NUMCORES_MSK) >> GCMP_GCB_GC_NUMCORES_SHF) + 1;
	for (cpu=0; cpu<cpuN; cpu++) {
		rc = gcmp_add_core(cpu);
		if (rc)
			return rc;
	}

	iocuN = ((GCMPGCB(GC) & GCMP_GCB_GC_NUMIOCU_MSK) >> GCMP_GCB_GC_NUMIOCU_SHF);
	for (cpu=0; cpu<iocuN; cpu++) {
		rc = gcmp_add_iocu(cpu,cpuN);
		if (rc)
			return rc;
	}

	return 0;
}

device_initcall_sync(init_gcmp_sysfs);

#endif /* CONFIG_SYSFS */
