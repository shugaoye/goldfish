#ifndef __ASM_MMU_H
#define __ASM_MMU_H

typedef struct {
	unsigned long asid[NR_CPUS];
	unsigned long vdso_asid[NR_CPUS];
	struct page   *vdso_page[NR_CPUS];
	void *vdso;
	struct vm_area_struct   *vdso_vma;
} mm_context_t;

#endif /* __ASM_MMU_H */
