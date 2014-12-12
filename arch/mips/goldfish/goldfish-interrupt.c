/* arch/mips/mach-goldfish/goldfish-platform.c
**
** Copyright (C) 2007 Google, Inc.
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/irqchip.h>
#include <linux/of.h>

#include <mach/hardware.h>
#include <mach/irq.h>
#include <asm/io.h>
#include <asm/irq_cpu.h>
#include <asm/setup.h>

#define GOLDFISH_INTERRUPT_STATUS       0x00 // number of pending interrupts
#define GOLDFISH_INTERRUPT_NUMBER       0x04
#define GOLDFISH_INTERRUPT_DISABLE_ALL  0x08
#define GOLDFISH_INTERRUPT_DISABLE      0x0c
#define GOLDFISH_INTERRUPT_ENABLE       0x10

static void __iomem *goldfish_interrupt;

void goldfish_mask_irq(struct irq_data *d)
{
	writel(d->irq-GOLDFISH_IRQ_BASE,
	       goldfish_interrupt + GOLDFISH_INTERRUPT_DISABLE);
}

void goldfish_unmask_irq(struct irq_data *d)
{
	writel(d->irq-GOLDFISH_IRQ_BASE,
	       goldfish_interrupt + GOLDFISH_INTERRUPT_ENABLE);
}

static struct irq_chip goldfish_irq_chip = {
	.name	= "goldfish",
	.irq_mask	= goldfish_mask_irq,
	.irq_mask_ack = goldfish_mask_irq,
	.irq_unmask = goldfish_unmask_irq,
};

void goldfish_init_irq(void)
{
	unsigned int i;
	uint32_t base;
	struct device_node *dn;

	if ((dn = of_find_node_by_name(NULL, "goldfish_pic")) == NULL) {
		panic("goldfish_init_irq() failed to "
			  "fetch device node \'goldfish-pic\'!\n");
	}

	if (of_property_read_u32(dn, "reg", &base) < 0) {
		panic("goldfish_init_irq() failed to "
			  "fetch device base address property \'reg\'!\n");
	}

	goldfish_interrupt = IO_ADDRESS(base);

	/*
	 * Disable all interrupt sources
	 */
	writel(1, goldfish_interrupt + GOLDFISH_INTERRUPT_DISABLE_ALL);

	for (i = GOLDFISH_IRQ_BASE; i < GOLDFISH_IRQ_BASE+32; i++) {
		irq_set_chip(i, &goldfish_irq_chip);
		irq_set_handler(i, handle_level_irq);
#if 0
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
#endif
	}
}

void goldfish_irq_dispatch(void)
{
	uint32_t irq;
	/*
	 * Disable all interrupt sources
	 */
	irq = readl(goldfish_interrupt + GOLDFISH_INTERRUPT_NUMBER);
	do_IRQ(GOLDFISH_IRQ_BASE+irq);
}

void goldfish_fiq_dispatch(void)
{
	panic("goldfish_fiq_dispatch");
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;

	if (pending & CAUSEF_IP2)
		goldfish_irq_dispatch();
	else if (pending & CAUSEF_IP3)
		goldfish_fiq_dispatch();
	else if (pending & CAUSEF_IP7)
		do_IRQ(MIPS_CPU_IRQ_BASE + 7);
	else
		spurious_interrupt();
}

static struct irqaction cascade = {
	.handler	= no_action,
	.flags      = IRQF_NO_THREAD,
	.name		= "cascade",
};

static void mips_timer_dispatch(void)
{
	do_IRQ(MIPS_CPU_IRQ_BASE + MIPS_CPU_IRQ_COMPARE);
}

void __init goldfish_pic_init(struct device_node *node, struct device_node *parent)
{
	mips_cpu_irq_init();
	goldfish_init_irq();

	if (cpu_has_vint) {
		set_vi_handler(MIPS_CPU_IRQ_PIC, goldfish_irq_dispatch);
		set_vi_handler(MIPS_CPU_IRQ_PIC, goldfish_fiq_dispatch);
	}
	setup_irq(MIPS_CPU_IRQ_BASE+MIPS_CPU_IRQ_PIC, &cascade);
	setup_irq(MIPS_CPU_IRQ_BASE+MIPS_CPU_IRQ_FIQ, &cascade);

	if (cpu_has_vint)
		set_vi_handler(MIPS_CPU_IRQ_COMPARE, mips_timer_dispatch);
}

void __init arch_init_irq(void)
{
	irqchip_init();
}

static const struct of_device_id irqchip_of_match_goldfish_pic
__used __section(__irqchip_of_table)
= { .compatible = "generic,goldfish-pic", .data = goldfish_pic_init };
