/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (C) 2012 Intel, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

enum {
	GOLDFISH_TTY_PUT_CHAR       = 0x00,
	GOLDFISH_TTY_BYTES_READY    = 0x04,
	GOLDFISH_TTY_CMD            = 0x08,

	GOLDFISH_TTY_DATA_PTR       = 0x10,
	GOLDFISH_TTY_DATA_LEN       = 0x14,
#ifdef CONFIG_64BIT
	GOLDFISH_TTY_DATA_PTR_HIGH  = 0x18,
#endif

	GOLDFISH_TTY_VERSION		= 0x20,

	GOLDFISH_TTY_CMD_INT_DISABLE    = 0,
	GOLDFISH_TTY_CMD_INT_ENABLE     = 1,
	GOLDFISH_TTY_CMD_WRITE_BUFFER   = 2,
	GOLDFISH_TTY_CMD_READ_BUFFER    = 3,
};

struct goldfish_tty {
	struct tty_port port;
	spinlock_t lock;
	void __iomem *base;
	u32 irq;
	int opencount;
	struct console console;
	u32 version;
	struct device *dev;
};

static DEFINE_MUTEX(goldfish_tty_lock);
static struct tty_driver *goldfish_tty_driver;
static u32 goldfish_tty_line_count = 8;
static u32 goldfish_tty_current_line_count;
static struct goldfish_tty *goldfish_ttys;

static inline void do_rw_io(struct goldfish_tty *qtty,
							unsigned long address,
							unsigned count, int is_write)
{
	unsigned long irq_flags;
	void __iomem *base = qtty->base;

	spin_lock_irqsave(&qtty->lock, irq_flags);
	writel((u32)address, base + GOLDFISH_TTY_DATA_PTR);
#ifdef CONFIG_64BIT
	writel((u32)((u64)address >> 32), base + GOLDFISH_TTY_DATA_PTR_HIGH);
#endif
	writel(count, base + GOLDFISH_TTY_DATA_LEN);

	if (is_write)
		writel(GOLDFISH_TTY_CMD_WRITE_BUFFER, base + GOLDFISH_TTY_CMD);
	else
		writel(GOLDFISH_TTY_CMD_READ_BUFFER, base + GOLDFISH_TTY_CMD);

	spin_unlock_irqrestore(&qtty->lock, irq_flags);
}

static inline void goldfish_tty_rw(struct goldfish_tty *qtty,
								   unsigned long address,
								   unsigned count, int is_write)
{
	dma_addr_t dma_handle;
	enum dma_data_direction dma_dir;
	dma_dir = (is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	if (qtty->version) {
		/* Goldfish TTY for Ranchu platform uses
		 * physical addresses and DMA for read/write operations
		 */
		unsigned long address_end = address + count;
		while (address < address_end) {
			unsigned long page_end = (address & PAGE_MASK) + PAGE_SIZE;
			unsigned long next     = page_end < address_end ? page_end
									: address_end;
			unsigned long avail    = next - address;

			/* Map the buffer's virtual address to the DMA address
			 * so the buffer can be accessed by the device.
			 */
			dma_handle = dma_map_single(qtty->dev, (void *)address, avail, dma_dir);

			if (dma_mapping_error(qtty->dev, dma_handle)) {
				dev_err(qtty->dev, "tty: DMA mapping error.\n");
				return;
			}
			do_rw_io(qtty, dma_handle, avail, is_write);

			/* Unmap the previously mapped region after the completition
			 * of the read/write operation.
			 */
			dma_unmap_single(qtty->dev, dma_handle, avail, dma_dir);

			address += avail;
		}
	} else {
		/* Old style Goldfish TTY used
		 * on the Goldfish platform uses
		 * virtual addresses
		 */
		do_rw_io(qtty, address, count, is_write);
	}

}

static void goldfish_tty_do_write(int line, const char *buf, unsigned count)
{
	struct goldfish_tty *qtty = &goldfish_ttys[line];
	unsigned long address = (unsigned long)(void *)buf;

	goldfish_tty_rw(qtty, address, count, 1);
}

static irqreturn_t goldfish_tty_interrupt(int irq, void *dev_id)
{
	struct goldfish_tty *qtty = dev_id;
	void __iomem *base = qtty->base;
	unsigned long address;
	unsigned char *buf;
	u32 count;

	count = readl(base + GOLDFISH_TTY_BYTES_READY);
	if(count == 0)
		return IRQ_NONE;

	count = tty_prepare_flip_string(&qtty->port, &buf, count);

	address = (unsigned long)(void *)buf;
	goldfish_tty_rw(qtty, address, count, 0);

	tty_schedule_flip(&qtty->port);
	return IRQ_HANDLED;
}

static int goldfish_tty_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct goldfish_tty *qtty = container_of(port, struct goldfish_tty, port);
	writel(GOLDFISH_TTY_CMD_INT_ENABLE, qtty->base + GOLDFISH_TTY_CMD);
	return 0;
}

static void goldfish_tty_shutdown(struct tty_port *port)
{
	struct goldfish_tty *qtty = container_of(port, struct goldfish_tty, port);
	writel(GOLDFISH_TTY_CMD_INT_DISABLE, qtty->base + GOLDFISH_TTY_CMD);
}

static int goldfish_tty_open(struct tty_struct * tty, struct file * filp)
{
	struct goldfish_tty *qtty = &goldfish_ttys[tty->index];
	return tty_port_open(&qtty->port, tty, filp);
}

static void goldfish_tty_close(struct tty_struct * tty, struct file * filp)
{
	tty_port_close(tty->port, tty, filp);
}

static void goldfish_tty_hangup(struct tty_struct *tty)
{
	tty_port_hangup(tty->port);
}

static int goldfish_tty_write(struct tty_struct * tty, const unsigned char *buf, int count)
{
	goldfish_tty_do_write(tty->index, buf, count);
	return count;
}

static int goldfish_tty_write_room(struct tty_struct *tty)
{
	return 0x10000;
}

static int goldfish_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct goldfish_tty *qtty = &goldfish_ttys[tty->index];
	void __iomem *base = qtty->base;
	return readl(base + GOLDFISH_TTY_BYTES_READY);
}

static void goldfish_tty_console_write(struct console *co, const char *b, unsigned count)
{
	goldfish_tty_do_write(co->index, b, count);
}

static struct tty_driver *goldfish_tty_console_device(struct console *c, int *index)
{
	*index = c->index;
	return goldfish_tty_driver;
}

static int goldfish_tty_console_setup(struct console *co, char *options)
{
	if((unsigned)co->index > goldfish_tty_line_count)
		return -ENODEV;
	if(goldfish_ttys[co->index].base == 0)
		return -ENODEV;
	return 0;
}

static struct tty_port_operations goldfish_port_ops = {
	.activate = goldfish_tty_activate,
	.shutdown = goldfish_tty_shutdown
};

static struct tty_operations goldfish_tty_ops = {
	.open = goldfish_tty_open,
	.close = goldfish_tty_close,
	.hangup = goldfish_tty_hangup,
	.write = goldfish_tty_write,
	.write_room = goldfish_tty_write_room,
	.chars_in_buffer = goldfish_tty_chars_in_buffer,
};

static int goldfish_tty_create_driver(void)
{
	int ret;
	struct tty_driver *tty;

	goldfish_ttys = kzalloc(sizeof(*goldfish_ttys) * goldfish_tty_line_count, GFP_KERNEL);
	if(goldfish_ttys == NULL) {
		ret = -ENOMEM;
		goto err_alloc_goldfish_ttys_failed;
	}
	tty = alloc_tty_driver(goldfish_tty_line_count);
	if(tty == NULL) {
		ret = -ENOMEM;
		goto err_alloc_tty_driver_failed;
	}
	tty->driver_name = "goldfish";
	tty->name = "ttyGF";
	tty->type = TTY_DRIVER_TYPE_SERIAL;
	tty->subtype = SERIAL_TYPE_NORMAL;
	tty->init_termios = tty_std_termios;
	tty->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(tty, &goldfish_tty_ops);
	ret = tty_register_driver(tty);
	if(ret)
		goto err_tty_register_driver_failed;

	goldfish_tty_driver = tty;
	return 0;

err_tty_register_driver_failed:
	put_tty_driver(tty);
err_alloc_tty_driver_failed:
	kfree(goldfish_ttys);
	goldfish_ttys = NULL;
err_alloc_goldfish_ttys_failed:
	return ret;
}

static void goldfish_tty_delete_driver(void)
{
	tty_unregister_driver(goldfish_tty_driver);
	put_tty_driver(goldfish_tty_driver);
	goldfish_tty_driver = NULL;
	kfree(goldfish_ttys);
	goldfish_ttys = NULL;
}

static int goldfish_tty_probe(struct platform_device *pdev)
{
	struct goldfish_tty *qtty;
	int ret = -EINVAL;
	int i;
	struct resource *r;
	struct device *ttydev;
	void __iomem *base;
	u32 irq;
	unsigned int line;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(r == NULL)
		return -EINVAL;

	base = ioremap(r->start, 0x1000);
	if (base == NULL)
		pr_err("goldfish_tty: unable to remap base\n");

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(r == NULL)
		goto err_unmap;

	irq = r->start;

	mutex_lock(&goldfish_tty_lock);

	if (pdev->id == PLATFORM_DEVID_NONE)
		line = goldfish_tty_current_line_count;
	else
		line = pdev->id;

	if(line >= goldfish_tty_line_count)
		goto err_create_driver_failed;

	if(goldfish_tty_current_line_count == 0) {
		ret = goldfish_tty_create_driver();
		if(ret)
			goto err_create_driver_failed;
	}
	goldfish_tty_current_line_count++;

	qtty = &goldfish_ttys[line];
	spin_lock_init(&qtty->lock);
	tty_port_init(&qtty->port);
	qtty->port.ops = &goldfish_port_ops;
	qtty->base = base;
	qtty->irq = irq;
	qtty->dev = &pdev->dev;

	/* Goldfish TTY device used by the Goldfish emulator
	 * should identify itself with 0, forcing the driver
	 * to use virtual addresses. Goldfish TTY device
	 * on Ranchu emulator (qemu2) returns 1 here and
	 * driver will use physical addresses.
	 */
	qtty->version = readl(base + GOLDFISH_TTY_VERSION);

	/* Goldfish TTY device on Ranchu emulator (qemu2)
	 * will use DMA for read/write IO operations.
	 */
	if (qtty->version > 0) {
		/* Initialize dma_mask to 32-bits.
		 */
		if (!pdev->dev.dma_mask)
			pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
		if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
			dev_err(&pdev->dev, "No suitable DMA available.\n");
			goto err_create_driver_failed;
		}
	}

	writel(GOLDFISH_TTY_CMD_INT_DISABLE, base + GOLDFISH_TTY_CMD);

	ret = request_irq(irq, goldfish_tty_interrupt, IRQF_SHARED, "goldfish_tty", qtty);
	if(ret)
		goto err_request_irq_failed;


	ttydev = tty_port_register_device(&qtty->port, goldfish_tty_driver,
							line, &pdev->dev);
	if(IS_ERR(ttydev)) {
		ret = PTR_ERR(ttydev);
		goto err_tty_register_device_failed;
	}

	strcpy(qtty->console.name, "ttyGF");
	qtty->console.write = goldfish_tty_console_write;
	qtty->console.device = goldfish_tty_console_device;
	qtty->console.setup = goldfish_tty_console_setup;
	qtty->console.flags = CON_PRINTBUFFER;
	qtty->console.index = line;
	register_console(&qtty->console);
	platform_set_drvdata(pdev, qtty);

	mutex_unlock(&goldfish_tty_lock);
	return 0;

	tty_unregister_device(goldfish_tty_driver, i);
err_tty_register_device_failed:
	free_irq(irq, pdev);
err_request_irq_failed:
	goldfish_tty_current_line_count--;
	if(goldfish_tty_current_line_count == 0)
		goldfish_tty_delete_driver();
err_create_driver_failed:
	mutex_unlock(&goldfish_tty_lock);
err_unmap:
	iounmap(base);
	return ret;
}

static int goldfish_tty_remove(struct platform_device *pdev)
{
	struct goldfish_tty *qtty = platform_get_drvdata(pdev);

	mutex_lock(&goldfish_tty_lock);

	unregister_console(&qtty->console);
	tty_unregister_device(goldfish_tty_driver, qtty->console.index);
	iounmap(qtty->base);
	qtty->base = 0;
	free_irq(qtty->irq, pdev);
	goldfish_tty_current_line_count--;
	if(goldfish_tty_current_line_count == 0)
		goldfish_tty_delete_driver();
	mutex_unlock(&goldfish_tty_lock);
	return 0;
}

static const struct of_device_id goldfish_tty_of_match[] = {
	{ .compatible = "generic,goldfish-tty", },
	{},
};

MODULE_DEVICE_TABLE(of, goldfish_tty_of_match);

static struct platform_driver goldfish_tty_platform_driver = {
	.probe = goldfish_tty_probe,
	.remove = goldfish_tty_remove,
	.driver = {
		.name = "goldfish_tty",
		.of_match_table = goldfish_tty_of_match,
	}
};

module_platform_driver(goldfish_tty_platform_driver);

MODULE_LICENSE("GPL v2");
