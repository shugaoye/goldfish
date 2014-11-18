#ifndef __ASM_TLBMISC_H
#define __ASM_TLBMISC_H

/*
 * - add_wired_entry() add a fixed TLB entry, and move wired register
 */
extern void add_wired_entry(unsigned long entrylo0, unsigned long entrylo1,
	unsigned long entryhi, unsigned long pagemask);
void remove_wired_entry(void);
int wired_push(unsigned long entryhi, unsigned long entrylo0,
	       unsigned long entrylo1, unsigned long pagemask);
int wired_pop(void);
int install_vdso_tlb(void);

#endif /* __ASM_TLBMISC_H */
