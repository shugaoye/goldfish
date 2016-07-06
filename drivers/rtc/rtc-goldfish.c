/* drivers/rtc/rtc-goldfish.c
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

#include <asm/io.h>
#include <linux/acpi.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>

enum {
	TIMER_TIME_LOW          = 0x00, // get low bits of current time and update TIMER_TIME_HIGH
	TIMER_TIME_HIGH         = 0x04, // get high bits of time at last TIMER_TIME_LOW read
	TIMER_ALARM_LOW         = 0x08, // set low bits of alarm and activate it
	TIMER_ALARM_HIGH        = 0x0c, // set high bits of next alarm
	TIMER_CLEAR_INTERRUPT   = 0x10,
	TIMER_CLEAR_ALARM       = 0x14
};

struct goldfish_rtc {
	void __iomem *base;
	uint32_t irq;
	struct rtc_device *rtc;
};

static irqreturn_t
goldfish_rtc_interrupt(int irq, void *dev_id)
{
	struct goldfish_rtc	*qrtc = dev_id;
	unsigned long		events = 0;
	void __iomem *base = qrtc->base;

	writel(1, base + TIMER_CLEAR_INTERRUPT);
	events = RTC_IRQF | RTC_AF;

	rtc_update_irq(qrtc->rtc, 1, events);

	return IRQ_HANDLED;
}

static int goldfish_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int64_t time;
	struct goldfish_rtc	*qrtc = platform_get_drvdata(to_platform_device(dev));
	void __iomem *base = qrtc->base;

	time = readl(base + TIMER_TIME_LOW);
	time |= (int64_t)readl(base + TIMER_TIME_HIGH) << 32;
	do_div(time, NSEC_PER_SEC);

	rtc_time_to_tm(time, tm);
	return 0;
}

static struct rtc_class_ops goldfish_rtc_ops = {
//	.ioctl		= goldfish_rtc_ioctl,
	.read_time	= goldfish_rtc_read_time,
//	.set_time	= goldfish_rtc_set_time,
//	.read_alarm	= goldfish_rtc_read_alarm,
//	.set_alarm	= goldfish_rtc_set_alarm,
};


static int goldfish_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct goldfish_rtc *qrtc;
	unsigned long rtc_dev_len;
	unsigned long rtc_dev_addr;

	qrtc = kzalloc(sizeof(*qrtc), GFP_KERNEL);
	if(qrtc == NULL) {
		ret = -ENOMEM;
		goto err_qrtc_alloc_failed;
	}
	platform_set_drvdata(pdev, qrtc);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(r == NULL) {
		ret = -ENODEV;
		goto err_no_io_base;
	}

	rtc_dev_addr = r->start;
	rtc_dev_len = resource_size(r);
	qrtc->base = ioremap(rtc_dev_addr, rtc_dev_len);
	qrtc->irq = platform_get_irq(pdev, 0);
	if(qrtc->irq < 0) {
		ret = -ENODEV;
		goto err_no_irq;
	}
	qrtc->rtc = rtc_device_register(pdev->name, &pdev->dev,
	                                &goldfish_rtc_ops, THIS_MODULE);
	if (IS_ERR(qrtc->rtc)) {
		ret = PTR_ERR(qrtc->rtc);
		goto err_rtc_device_register_failed;
	}

	ret = request_irq(qrtc->irq, goldfish_rtc_interrupt, 0, pdev->name, qrtc);
	if(ret)
		goto request_irq;

	return 0;

	free_irq(qrtc->irq, qrtc);
request_irq:
	rtc_device_unregister(qrtc->rtc);
err_rtc_device_register_failed:
err_no_irq:
err_no_io_base:
	kfree(qrtc);
err_qrtc_alloc_failed:
	return ret;
}

static int goldfish_rtc_remove(struct platform_device *pdev)
{
	struct goldfish_rtc	*qrtc = platform_get_drvdata(pdev);
	free_irq(qrtc->irq, qrtc);
	rtc_device_unregister(qrtc->rtc);
	kfree(qrtc);
	return 0;
}

static const struct of_device_id goldfish_rtc_of_match[] = {
	{ .compatible = "generic,goldfish-rtc", },
	{},
};
MODULE_DEVICE_TABLE(of, goldfish_rtc_of_match);

static const struct acpi_device_id goldfish_rtc_acpi_match[] = {
	{ "GFSH0007", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, golfish_rtc_acpi_match);

static struct platform_driver goldfish_rtc = {
	.probe = goldfish_rtc_probe,
	.remove = goldfish_rtc_remove,
	.driver = {
		.name = "goldfish_rtc",
		.of_match_table = goldfish_rtc_of_match,
		.acpi_match_table = ACPI_PTR(goldfish_rtc_acpi_match),
	}
};

module_platform_driver(goldfish_rtc);

