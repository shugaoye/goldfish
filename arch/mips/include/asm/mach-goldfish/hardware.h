/* include/asm-mips/mach-goldfish/hardware.h
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

#ifndef __ASM_MACH_GOLDFISH_HARDWARE_H
#define __ASM_ARCH_GOLDFISH_HARDWARE_H

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 */
#define IO_SIZE			0x00800000                 // How much?
#define IO_START		0x00000000                 // PA of IO

#define IO_ADDRESS(x) (ioremap(IO_START, IO_SIZE) + (x))

/* For now... */
#define dma_alloc_writecombine(dev, size, handle, gfp) \
	dma_alloc_coherent(dev, size, handle, gfp)


#define dma_free_writecombine(dev, size, cpu_addr, handle) \
	dma_free_coherent(dev, size, cpu_addr, handle)

#endif
