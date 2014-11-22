/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 95, 96, 97, 98, 99, 2000, 2003 Ralf Baechle
 * Copyright (C) 1999, 2000, 2001 Silicon Graphics, Inc.
 */
#ifndef _ASM_PGTABLE_32_H
#define _ASM_PGTABLE_32_H

#include <asm/addrspace.h>
#include <asm/page.h>

#include <linux/linkage.h>
#include <asm/cachectl.h>
#include <asm/fixmap.h>

#include <asm-generic/pgtable-nopmd.h>

/*
 * Basically we have the same two-level (which is the logical three level
 * Linux page table layout folded) page tables as the i386.  Some day
 * when we have proper page coloring support we can have a 1% quicker
 * tlb refill handling mechanism, but for now it is a bit slower but
 * works even with the cache aliasing problem the R4k and above have.
 */

/* PGDIR_SHIFT determines what a third-level page table entry can map */
#define PGDIR_SHIFT	(2 * PAGE_SHIFT + PTE_ORDER - PTE_T_LOG2)
#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

/*
 * Entries per page directory level: we use two-level, so
 * we don't really have any PUD/PMD directory physically.
 */
#define __PGD_ORDER	(32 - 3 * PAGE_SHIFT + PGD_T_LOG2 + PTE_T_LOG2)
#define PGD_ORDER	(__PGD_ORDER >= 0 ? __PGD_ORDER : 0)
#define PUD_ORDER	aieeee_attempt_to_allocate_pud
#define PMD_ORDER	1
#define PTE_ORDER	0

#define PTRS_PER_PGD	(USER_PTRS_PER_PGD * 2)
#define PTRS_PER_PTE	((PAGE_SIZE << PTE_ORDER) / sizeof(pte_t))

#define USER_PTRS_PER_PGD	(0x80000000UL/PGDIR_SIZE)
#define FIRST_USER_ADDRESS	0

#define VMALLOC_START	  MAP_BASE

#define PKMAP_BASE		(0xfe000000UL)

#ifndef VMALLOC_END
#ifdef CONFIG_HIGHMEM
# define VMALLOC_END	(PKMAP_BASE-2*PAGE_SIZE)
#else
# define VMALLOC_END	(FIXADDR_START-2*PAGE_SIZE)
#endif
#endif

#ifdef CONFIG_64BIT_PHYS_ADDR
#define pte_ERROR(e) \
	printk("%s:%d: bad pte %016Lx.\n", __FILE__, __LINE__, pte_val(e))
#else
#define pte_ERROR(e) \
	printk("%s:%d: bad pte %08lx.\n", __FILE__, __LINE__, pte_val(e))
#endif
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd %08lx.\n", __FILE__, __LINE__, pgd_val(e))

extern void load_pgd(unsigned long pg_dir);

extern pte_t invalid_pte_table[PAGE_SIZE/sizeof(pte_t)];

/*
 * Empty pgd/pmd entries point to the invalid_pte_table.
 */
static inline int pmd_none(pmd_t pmd)
{
	return pmd_val(pmd) == (unsigned long) invalid_pte_table;
}

#define pmd_bad(pmd)		(pmd_val(pmd) & ~PAGE_MASK)

static inline int pmd_present(pmd_t pmd)
{
	return pmd_val(pmd) != (unsigned long) invalid_pte_table;
}

static inline void pmd_clear(pmd_t *pmdp)
{
	pmd_val(*pmdp) = ((unsigned long) invalid_pte_table);
}

#if defined(CONFIG_64BIT_PHYS_ADDR) && defined(CONFIG_CPU_MIPS32)
#define pte_page(x)		pfn_to_page(pte_pfn(x))
#define pte_pfn(x)		((unsigned long)((x).pte_high >> 6))
static inline pte_t
pfn_pte(unsigned long pfn, pgprot_t prot)
{
	pte_t pte;
	pte.pte_high = (pfn << 6) | (pgprot_val(prot) & 0x3f);
	pte.pte_low = pgprot_val(prot);
	return pte;
}

#else

#define pte_page(x)		pfn_to_page(pte_pfn(x))

#ifdef CONFIG_CPU_VR41XX
#define pte_pfn(x)		((unsigned long)((x).pte >> (PAGE_SHIFT + 2)))
#define pfn_pte(pfn, prot)	__pte(((pfn) << (PAGE_SHIFT + 2)) | pgprot_val(prot))
#else
#define pte_pfn(x)		((unsigned long)((x).pte >> _PFN_SHIFT))
#define pfn_pte(pfn, prot)	__pte(((unsigned long long)(pfn) << _PFN_SHIFT) | pgprot_val(prot))
#endif
#endif /* defined(CONFIG_64BIT_PHYS_ADDR) && defined(CONFIG_CPU_MIPS32) */

#define __pgd_offset(address)	pgd_index(address)
#define __pud_offset(address)	(((address) >> PUD_SHIFT) & (PTRS_PER_PUD-1))
#define __pmd_offset(address)	(((address) >> PMD_SHIFT) & (PTRS_PER_PMD-1))

/* to find an entry in a kernel page-table-directory */
#define pgd_offset_k(address) pgd_offset(&init_mm, address)

#define pgd_index(address)	(((address) >> PGDIR_SHIFT) & (PTRS_PER_PGD-1))

/* to find an entry in a page-table-directory */
#define pgd_offset(mm, addr)	((mm)->pgd + pgd_index(addr))

/* Find an entry in the third-level page table.. */
#define __pte_offset(address)						\
	(((address) >> PAGE_SHIFT) & (PTRS_PER_PTE - 1))
#define pte_offset(dir, address)					\
	((pte_t *) pmd_page_vaddr(*(dir)) + __pte_offset(address))
#define pte_offset_kernel(dir, address)					\
	((pte_t *) pmd_page_vaddr(*(dir)) + __pte_offset(address))

#define pte_offset_map(dir, address)					\
	((pte_t *)page_address(pmd_page(*(dir))) + __pte_offset(address))
#define pte_unmap(pte) ((void)(pte))

#if defined(CONFIG_64BIT_PHYS_ADDR) && defined(CONFIG_CPU_MIPS32)

/*
 * Two words PTE case:
 * Bits 0 and 1 (V+G) of pte_high are taken, use the rest for the swaps and
 * page offset...
 * Bits F and P are in pte_low.
 *
 * Note: swp_entry_t is one word today.
 */
#define __swp_type(x)       \
		(((x).val >> __SWP_PTE_SKIP_BITS_NUM) & __SWP_TYPE_MASK)
#define __swp_offset(x)     \
		((x).val >> (__SWP_PTE_SKIP_BITS_NUM + __SWP_TYPE_BITS_NUM))
#define __swp_entry(type, offset)        \
		((swp_entry_t)  { ((type) << __SWP_PTE_SKIP_BITS_NUM) | \
		((offset) << (__SWP_TYPE_BITS_NUM + __SWP_PTE_SKIP_BITS_NUM)) })
#define __pte_to_swp_entry(pte) ((swp_entry_t) { (pte).pte_high })
#define __swp_entry_to_pte(x)	((pte_t) { 0, (x).val })

#define PTE_FILE_MAX_BITS       (32 - __SWP_PTE_SKIP_BITS_NUM)

#define pte_to_pgoff(_pte)      ((_pte).pte_high >> __SWP_PTE_SKIP_BITS_NUM)
#define pgoff_to_pte(off)   \
		((pte_t) { _PAGE_FILE, (off) << __SWP_PTE_SKIP_BITS_NUM })

#else /* CONFIG_MIPS32 && !CONFIG_64BIT_PHYS_ADDR */

/* Swap  entries must have V,G,P and F bits cleared. */
#define __swp_type(x)       (((x).val >> _PAGE_DIRTY_SHIFT) & __SWP_TYPE_MASK)
#define __swp_offset(x)     \
		((x).val >> (_PAGE_DIRTY_SHIFT + __SWP_TYPE_BITS_NUM))
#define __swp_entry(type, offset)        \
		((swp_entry_t) { ((type) << _PAGE_DIRTY_SHIFT) | \
		((offset) << (_PAGE_DIRTY_SHIFT + __SWP_TYPE_BITS_NUM)) })
#define __pte_to_swp_entry(pte) ((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(x)	((pte_t) { (x).val })

/*
 * Bits V+G, and F+P are taken, split up 28 bits of offset into this range:
 */
#define PTE_FILE_MAX_BITS       (32 - __FILE_PTE_TOTAL_BITS_NUM)

#define pte_to_pgoff(_pte)  \
		((((_pte).pte >> __FILE_PTE_TOTAL_BITS_NUM) & \
		    ~(__FILE_PTE_LOW_MASK)) | \
		 (((_pte).pte >> __FILE_PTE_LOW_BITS_NUM) & \
		    (__FILE_PTE_LOW_MASK)))

#define pgoff_to_pte(off)   \
		((pte_t) { (((off) & __FILE_PTE_LOW_MASK) << \
		      (__FILE_PTE_LOW_BITS_NUM)) | \
		   (((off) & ~(__FILE_PTE_LOW_MASK)) << \
		      (__FILE_PTE_TOTAL_BITS_NUM)) | \
		   _PAGE_FILE })

#endif /* CONFIG_64BIT_PHYS_ADDR && CONFIG_MIPS32 */
#endif /* _ASM_PGTABLE_32_H */
