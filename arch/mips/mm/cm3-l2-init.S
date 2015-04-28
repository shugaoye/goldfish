/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2014 Imagination Technologies Limited.  All rights reserved.
 * Written by Leonid Yegoshin on base of Tom Berg code example
 */
/*
 *  CM3 L2 cache initialization (if it is missed in uboot/zboot/*boot/etc)
 *  Runs once, during boot.
 *
 *  args:
 *      r4 - DCache linesize
 *      r5 - INDEX BASE for DCache flush
 *      r6 - DCache size
 *      r7 - GCR (GCMP) base VA
 */

#ifdef CONFIG_MIPS_CMP
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/regdef.h>


LEAF(cm3_l2_init)
	.set    noreorder

init_l2:
	# L2 Cache initialization routine

	# Check L2 cache size
	lw          $10, 0x0130($7)            # load GCR_L2_CONFIG

	# Isolate L2 Line Size
	ext         $11, $10, 11, 4            # line size

	# Skip ahead if No L2
	beq         $11, $0, done_l2cache
	  move      $2, $0
	li          $2, 1

	li          $14, 2
	sllv        $11, $14, $11               # Now have true L2 line size in bytes

	# Isolate L2 Sets per Way
	ext         $12, $10, 12, 4            # sets per way
	li          $14, 64
	sllv        $12, $14, $12              # L2 Sets per way

	# Isolate L2 Associativity
	# L2 Assoc (-1)
	ext         $13, $10, 0, 8              # assoc
	addiu       $13, $13, 1

	mul         $12, $12, $13               # Get total number of sets
	lui         $14, 0x8000                 # Get a KSeg0 address for cacheops

	# Clear GCR Tag/Data registers
	sw          $0, 0x600($7)               # GCR_TAG_ADDR  = 0
	sw          $0, 0x604($7)               # GCR_TAG_ADDR  = 0
	sw          $0, 0x608($7)               # GCR_TAG_STATE = 0
	sw          $0, 0x60c($7)               # GCR_TAG_STATE = 0
	sw          $0, 0x610($7)               # GCR_TAG_DATA  = 0
	sw          $0, 0x614($7)               # GCR_TAG_DATA  = 0

	li          $8, 0xfcffffff
	and         $10, $8                     # clear bits 25:24 to make sure ECC is calculated by HW
	li          $8, 0x04000000
	or          $10, $8                     # set bit 26 of GCR_L2_CONFIG to make sure LRU is written
	sw          $10, 0x130($7)              # write the GCR_L2_CONFIG

	sync                                    # make sure these are complete

	move        $15, $12

	# L2 Index Store Tag Cache Op
	# Will invalidate the tag entry
1:      cache       0xB, 0($14)                 # SCIndexStTag
	addiu       $15, $15, -1                # Decrement set counter

	bne         $15, $0, 1b
	  add       $14, $11                    # Get next line address

done_l2cache:

	# *** if the dcache has been enabled before this point,
	#     then it should be flushed at this point.
	beqz        $4, 3f
	  nop
2:
	cache       0x1, 0($5)
	subu        $6, $4
	bgtz        $6, 2b
	  add       $5, $4
	sync

3:
	# Enable coherence on this core
	li          $9, 0x01
	sw          $9, 0x2008($7)              # GCR_CL_COHERENCE
	sync

/* The following is only needed in a multi-core system
 * if you want to enable coherence on other cores from this core:
 */

	lw         $8, 0x0($7)                 # GCR_CONFIG
	ext        $8, $8, 0, 8                # num of cores-1
	beq        $8, $0, cm3_init_done       # if only single core system
	  nop

next_coherent_core:
       sll         $11, $8, 8                  # move to bits 13:8
       sw          $11, 0x2018($7)             # GCR_CL_OTHER[Core] = core number
       sync                                    # make sure write is complete before moving on
       sw          $9, 0x4008($7)              # GCR_CO_COHERENCE = 1
       sync                                    # make sure write is complete before moving on
       addiu       $8, $8, -1
       bne         $8, $9, next_coherent_core
	 nop

cm3_init_done:
       # turn off l2 bypass (may not be required if L1 caches are not enabled yet)
       li           $8, 20
       sllv         $25, $9, $8                # bit 20 = 1
       or           $10, $10, $25              # clear bit 20 in old value of GCR_L2_CONFIG
       xor          $10, $10, $25
       sw           $10, 0x0130($7)            # set GCR_L2_CONFIG.L2_BYPASS = 0
       sync
       jr           $31
	 nop

END(cm3_l2_init)
#endif
