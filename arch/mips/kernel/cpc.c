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
 * Copyright (C) 2013 Imagination Technologies Ltd
 *    Leonid Yegoshin (Leonid.Yegoshin@imgtec.com)
 */

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
#include <asm/cpcregs.h>
#include <asm/bootinfo.h>
#include <asm/irq_cpu.h>

unsigned long _cpc_base;
int cpc_present = -1;

int __init cpc_probe(unsigned long defaddr, unsigned long defsize)
{
	if (cpc_present >= 0)
		return cpc_present;

	if (gcmp_present <= 0) {
		cpc_present = 0;
		return 0;
	}

	if ((GCMPGCB(CPCST) & GCMP_GCB_CPCST_EN_MASK) == 0) {
		cpc_present = 0;
		return 0;
	}

	_cpc_base = GCMPGCB(CPCBA);
	if (_cpc_base & GCMP_GCB_CPCBA_EN_MASK)
		goto success;

	if (!defaddr) {
		cpc_present = 0;
		return 0;
	}

	/* Try to setup a platform value */
	GCMPGCB(CPCBA) = defaddr | GCMP_GCB_CPCBA_EN_MASK;
	_cpc_base = GCMPGCB(CPCBA);
	if ((_cpc_base & GCMP_GCB_CPCBA_EN_MASK) == 0) {
		cpc_present = 0;
		return 0;
	}
success:
	pr_info("CPC available\n");
	_cpc_base = (unsigned long) ioremap_nocache(_cpc_base & ~GCMP_GCB_CPCBA_EN_MASK, defsize);
	cpc_present = 1;
	return 1;
}

#ifdef CONFIG_SYSFS

static ssize_t show_cpc_global(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int n = 0;

	n = snprintf(buf, PAGE_SIZE,
		"CPC Global CSR Access Privilege Register\t%08x\n"
		"CPC Global Sequence Delay Counter\t\t%08x\n"
		"CPC Global Rail Delay Counter Register\t\t%08x\n"
		"CPC Global Reset Width Counter Register\t\t%08x\n"
		"CPC Global Revision Register\t\t\t%08x\n"
		,
		CPCGCB(CSRAPR),
		CPCGCB(SEQDELAY),
		CPCGCB(RAILDELAY),
		CPCGCB(RESETWIDTH),
		CPCGCB(REVID)
	);

	return n;
}

static char *cpc_cmd[] = { "0", "ClockOff", "PwrDown", "PwrUp", "Reset",
	    "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15" };

static char *cpc_status[] = { "PwrDwn", "VddOK", "UpDelay", "UClkOff", "Reset",
	    "ResetDly", "nonCoherent", "Coherent", "Isolate", "ClrBus",
	    "DClkOff", "11", "12", "13", "14", "15" };

static ssize_t show_cpc_local(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int n = 0;

	CPCLCB(OTHER) = (dev->id)<<16;

	n += snprintf(buf+n, PAGE_SIZE-n,
		"CPC Local Command Register\t\t\t%08x:  CMD=%s\n"
		"CPC Local Status and Configuration register\t%08x:   Status=%s, LastCMD=%s\n"
		"CPC Core Other Addressing Register\t\t%08x\n"
		,
		CPCOCB(CMD), cpc_cmd[(CPCOCB(CMD) & CPCL_CMD_MASK) >> CPCL_CMD_SH],
		CPCOCB(STATUS), cpc_status[(CPCOCB(STATUS) & CPCL_STATUS_MASK) >> CPCL_STATUS_SH],
			cpc_cmd[(CPCOCB(STATUS) & CPCL_CMD_MASK) >> CPCL_CMD_SH],
		CPCOCB(OTHER)
	);

	return n;
}

static DEVICE_ATTR(cpc_global, 0444, show_cpc_global, NULL);
static DEVICE_ATTR(cpc_local, 0444, show_cpc_local, NULL);

static struct bus_type cpc_subsys = {
	.name = "cpc",
	.dev_name = "cpc",
};



static __cpuinit int cpc_add_core(int cpu)
{
	struct device *dev;
	int err;
	char name[16];

	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->id = cpu;
	dev->bus = &cpc_subsys;
	snprintf(name, sizeof name, "core%d",cpu);
	dev->init_name = name;

	err = device_register(dev);
	if (err)
		return err;

	err = device_create_file(dev, &dev_attr_cpc_local);
	if (err)
		return err;

	return 0;
}

static int __init init_cpc_sysfs(void)
{
	int rc;
	int cpuN;
	int cpu;

	if (cpc_present <= 0)
		return 0;

	rc = subsys_system_register(&cpc_subsys, NULL);
	if (rc)
		return rc;

	rc = device_create_file(cpc_subsys.dev_root, &dev_attr_cpc_global);
	if (rc)
		return rc;

	cpuN = ((GCMPGCB(GC) & GCMP_GCB_GC_NUMCORES_MSK) >> GCMP_GCB_GC_NUMCORES_SHF) + 1;
	for (cpu=0; cpu<cpuN; cpu++) {
		rc = cpc_add_core(cpu);
		if (rc)
			return rc;
	}

	return 0;
}

device_initcall_sync(init_cpc_sysfs);

#endif /* CONFIG_SYSFS */
