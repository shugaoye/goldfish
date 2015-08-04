/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2009, 2010 Cavium Networks, Inc.
 */


#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/binfmts.h>
#include <linux/elf.h>
#include <linux/vmalloc.h>
#include <linux/unistd.h>

#include <asm/vdso.h>
#include <asm/uasm.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

/*
 * Including <asm/unistd.h> would give use the 64-bit syscall numbers ...
 */
#define __NR_O32_sigreturn		4119
#define __NR_O32_rt_sigreturn		4193
#define __NR_N32_rt_sigreturn		6211

static struct page *vdso_page;

static void __init install_trampoline(u32 *tramp, unsigned int sigreturn)
{
	uasm_i_addiu(&tramp, 2, 0, sigreturn);	/* li v0, sigreturn */
	uasm_i_syscall(&tramp, 0);
}

static int __init init_vdso(void)
{
	struct mips_vdso *vdso;

	vdso_page = alloc_page(GFP_KERNEL);
	if (!vdso_page)
		panic("Cannot allocate vdso");

	vdso = vmap(&vdso_page, 1, 0, PAGE_KERNEL);
	if (!vdso)
		panic("Cannot map vdso");
	clear_page(vdso);

	install_trampoline(vdso->rt_signal_trampoline, __NR_rt_sigreturn);
#ifdef CONFIG_32BIT
	install_trampoline(vdso->signal_trampoline, __NR_sigreturn);
#else
	install_trampoline(vdso->n32_rt_signal_trampoline,
			   __NR_N32_rt_sigreturn);
	install_trampoline(vdso->o32_signal_trampoline, __NR_O32_sigreturn);
	install_trampoline(vdso->o32_rt_signal_trampoline,
			   __NR_O32_rt_sigreturn);
#endif

	vunmap(vdso);

	return 0;
}
subsys_initcall(init_vdso);

static unsigned long vdso_addr(unsigned long start)
{
	return STACK_TOP;
}

int arch_setup_additional_pages(struct linux_binprm *bprm, int uses_interp)
{
	int ret;
	unsigned long addr;
	struct mm_struct *mm = current->mm;

	down_write(&mm->mmap_sem);

	addr = vdso_addr(mm->start_stack);

	addr = get_unmapped_area(NULL, addr, PAGE_SIZE, 0, 0);
	if (IS_ERR_VALUE(addr)) {
		ret = addr;
		goto up_fail;
	}

	ret = install_special_mapping(mm, addr, PAGE_SIZE,
				      VM_READ|VM_EXEC|
				      VM_MAYREAD|VM_MAYEXEC,
				      &vdso_page);

	if (ret)
		goto up_fail;

	mm->context.vdso = (void *)addr;
	mm->context.thread_flags = current_thread_info()->local_flags;
	/* if cache aliasing - use a different cache flush later */
	if (cpu_has_rixi && (cpu_has_dc_aliases || cpu_has_ic_aliases))
		mm->context.vdso_vma = find_vma(mm,addr);

	mips_thread_vdso(current_thread_info());
	smp_wmb();
up_fail:
	up_write(&mm->mmap_sem);
	return ret;
}

const char *arch_vma_name(struct vm_area_struct *vma)
{
	if (vma->vm_mm && vma->vm_start == (long)vma->vm_mm->context.vdso)
		return "[vdso]";
	return NULL;
}

void mips_thread_vdso(struct thread_info *ti)
{
	struct page *vdso;
	unsigned long addr;

	if (cpu_has_rixi && ti->task->mm && !ti->vdso_page) {
		vdso = alloc_page(GFP_USER);
		if (!vdso)
			return;
		ti->vdso_page = vdso;
		ti->vdso_offset = PAGE_SIZE;
		addr = (unsigned long)page_address(vdso);
		copy_page((void *)addr, (void *)page_address(vdso_page));
		if (!cpu_has_ic_fills_f_dc)
			flush_data_cache_page(addr);
		/* any vma in mmap is used, just to get ASIDs back from mm */
		local_flush_tlb_page(ti->task->mm->mmap,(unsigned long)ti->task->mm->context.vdso);
	}
}

void arch_release_thread_info(struct thread_info *info)
{
	if (info->vdso_page) {
		if (info->task->mm) {
			preempt_disable();
			/* any vma in mmap is used, just to get ASIDs */
			local_flush_tlb_page(info->task->mm->mmap,(unsigned long)info->task->mm->context.vdso);
			info->task->mm->context.vdso_asid[smp_processor_id()] = 0;
			preempt_enable();
		}
		__free_page(info->vdso_page);
		info->vdso_page = NULL;
	}
}
