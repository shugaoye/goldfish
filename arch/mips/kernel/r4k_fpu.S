/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996, 98, 99, 2000, 01 Ralf Baechle
 *
 * Multi-arch abstraction and asm macros for easier reading:
 * Copyright (C) 1996 David S. Miller (davem@davemloft.net)
 *
 * Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 2000 MIPS Technologies, Inc.
 * Copyright (C) 1999, 2001 Silicon Graphics, Inc.
 */
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/errno.h>
#include <asm/fpregdef.h>
#include <asm/mipsregs.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>

/* preprocessor replaces the fp in ".set fp=64" with $30 otherwise */
#undef fp

	.macro	EX insn, reg, src
	.set	push
	SET_HARDFLOAT
	.set	nomacro
.ex\@:	\insn	\reg, \src
	.set	pop
	.section __ex_table,"a"
	PTR	.ex\@, fault
	.previous
	.endm

	.set	noreorder
#ifdef CONFIG_CPU_MIPSR6
	.set    mips64r6
#else
	.set	arch=r4000
#endif
	SET_HARDFLOAT

LEAF(_save_fp_context)
	PTR_L   t2, TI_TASK($28)
	fpu_get_fcr31   t2 t0 t1

#if defined(CONFIG_64BIT) || defined(CONFIG_CPU_MIPSR6)
	/* Store the 16 odd double precision registers */
	EX	sdc1 $f1, SC_FPREGS+8(a0)
	EX	sdc1 $f3, SC_FPREGS+24(a0)
	EX	sdc1 $f5, SC_FPREGS+40(a0)
	EX	sdc1 $f7, SC_FPREGS+56(a0)
	EX	sdc1 $f9, SC_FPREGS+72(a0)
	EX	sdc1 $f11, SC_FPREGS+88(a0)
	EX	sdc1 $f13, SC_FPREGS+104(a0)
	EX	sdc1 $f15, SC_FPREGS+120(a0)
	EX	sdc1 $f17, SC_FPREGS+136(a0)
	EX	sdc1 $f19, SC_FPREGS+152(a0)
	EX	sdc1 $f21, SC_FPREGS+168(a0)
	EX	sdc1 $f23, SC_FPREGS+184(a0)
	EX	sdc1 $f25, SC_FPREGS+200(a0)
	EX	sdc1 $f27, SC_FPREGS+216(a0)
	EX	sdc1 $f29, SC_FPREGS+232(a0)
	EX	sdc1 $f31, SC_FPREGS+248(a0)
#else
#ifdef CONFIG_CPU_MIPS32_R2
	.set    push
	.set    mips32r2
	.set    fp=64
	SET_HARDFLOAT
	.set    noreorder
	mfc0    t0, CP0_STATUS
	sll     t0, t0, 31 - _ST0_FR
	bgez    t0, 1f              # 16 / 32 register mode?
	 nop

	/* Store the 16 odd double precision registers */
	EX      sdc1 $f1, SC_FPREGS+8(a0)
	EX      sdc1 $f3, SC_FPREGS+24(a0)
	EX      sdc1 $f5, SC_FPREGS+40(a0)
	EX      sdc1 $f7, SC_FPREGS+56(a0)
	EX      sdc1 $f9, SC_FPREGS+72(a0)
	EX      sdc1 $f11, SC_FPREGS+88(a0)
	EX      sdc1 $f13, SC_FPREGS+104(a0)
	EX      sdc1 $f15, SC_FPREGS+120(a0)
	EX      sdc1 $f17, SC_FPREGS+136(a0)
	EX      sdc1 $f19, SC_FPREGS+152(a0)
	EX      sdc1 $f21, SC_FPREGS+168(a0)
	EX      sdc1 $f23, SC_FPREGS+184(a0)
	EX      sdc1 $f25, SC_FPREGS+200(a0)
	EX      sdc1 $f27, SC_FPREGS+216(a0)
	EX      sdc1 $f29, SC_FPREGS+232(a0)
	EX      sdc1 $f31, SC_FPREGS+248(a0)
1:
	.set    pop
#endif
#endif

	/* Store the 16 even double precision registers */
	EX	sdc1 $f0, SC_FPREGS+0(a0)
	EX	sdc1 $f2, SC_FPREGS+16(a0)
	EX	sdc1 $f4, SC_FPREGS+32(a0)
	EX	sdc1 $f6, SC_FPREGS+48(a0)
	EX	sdc1 $f8, SC_FPREGS+64(a0)
	EX	sdc1 $f10, SC_FPREGS+80(a0)
	EX	sdc1 $f12, SC_FPREGS+96(a0)
	EX	sdc1 $f14, SC_FPREGS+112(a0)
	EX	sdc1 $f16, SC_FPREGS+128(a0)
	EX	sdc1 $f18, SC_FPREGS+144(a0)
	EX	sdc1 $f20, SC_FPREGS+160(a0)
	EX	sdc1 $f22, SC_FPREGS+176(a0)
	EX	sdc1 $f24, SC_FPREGS+192(a0)
	EX	sdc1 $f26, SC_FPREGS+208(a0)
	EX	sdc1 $f28, SC_FPREGS+224(a0)
	EX	sdc1 $f30, SC_FPREGS+240(a0)
	EX	sw t1, SC_FPC_CSR(a0)
	jr	ra
	 li	v0, 0					# success
	END(_save_fp_context)

#ifdef CONFIG_MIPS32_COMPAT
	/* Save 32-bit process floating point context */
LEAF(_save_fp_context32)
	PTR_L   t2, TI_TASK($28)
	fpu_get_fcr31   t2 t0 t1

	mfc0    t0, CP0_STATUS
	sll     t0, t0, 31 - _ST0_FR
	bgez    t0, 1f              # 16 / 32 register mode?
	 nop

	/* Store the 16 odd double precision registers */
	EX      sdc1 $f1, SC32_FPREGS+8(a0)
	EX      sdc1 $f3, SC32_FPREGS+24(a0)
	EX      sdc1 $f5, SC32_FPREGS+40(a0)
	EX      sdc1 $f7, SC32_FPREGS+56(a0)
	EX      sdc1 $f9, SC32_FPREGS+72(a0)
	EX      sdc1 $f11, SC32_FPREGS+88(a0)
	EX      sdc1 $f13, SC32_FPREGS+104(a0)
	EX      sdc1 $f15, SC32_FPREGS+120(a0)
	EX      sdc1 $f17, SC32_FPREGS+136(a0)
	EX      sdc1 $f19, SC32_FPREGS+152(a0)
	EX      sdc1 $f21, SC32_FPREGS+168(a0)
	EX      sdc1 $f23, SC32_FPREGS+184(a0)
	EX      sdc1 $f25, SC32_FPREGS+200(a0)
	EX      sdc1 $f27, SC32_FPREGS+216(a0)
	EX      sdc1 $f29, SC32_FPREGS+232(a0)
	EX      sdc1 $f31, SC32_FPREGS+248(a0)
1:

	EX	sdc1 $f0, SC32_FPREGS+0(a0)
	EX	sdc1 $f2, SC32_FPREGS+16(a0)
	EX	sdc1 $f4, SC32_FPREGS+32(a0)
	EX	sdc1 $f6, SC32_FPREGS+48(a0)
	EX	sdc1 $f8, SC32_FPREGS+64(a0)
	EX	sdc1 $f10, SC32_FPREGS+80(a0)
	EX	sdc1 $f12, SC32_FPREGS+96(a0)
	EX	sdc1 $f14, SC32_FPREGS+112(a0)
	EX	sdc1 $f16, SC32_FPREGS+128(a0)
	EX	sdc1 $f18, SC32_FPREGS+144(a0)
	EX	sdc1 $f20, SC32_FPREGS+160(a0)
	EX	sdc1 $f22, SC32_FPREGS+176(a0)
	EX	sdc1 $f24, SC32_FPREGS+192(a0)
	EX	sdc1 $f26, SC32_FPREGS+208(a0)
	EX	sdc1 $f28, SC32_FPREGS+224(a0)
	EX	sdc1 $f30, SC32_FPREGS+240(a0)
	EX	sw t1, SC32_FPC_CSR(a0)
	cfc1	t0, $0				# implementation/version
	EX	sw t0, SC32_FPC_EIR(a0)

	jr	ra
	 li	v0, 0					# success
	END(_save_fp_context32)
#endif

/*
 * Restore FPU state:
 *  - fp gp registers
 *  - cp1 status/control register
 */
LEAF(_restore_fp_context)
	EX	lw t0, SC_FPC_CSR(a0)
#if defined(CONFIG_64BIT) || defined(CONFIG_CPU_MIPSR6)
	EX	ldc1 $f1, SC_FPREGS+8(a0)
	EX	ldc1 $f3, SC_FPREGS+24(a0)
	EX	ldc1 $f5, SC_FPREGS+40(a0)
	EX	ldc1 $f7, SC_FPREGS+56(a0)
	EX	ldc1 $f9, SC_FPREGS+72(a0)
	EX	ldc1 $f11, SC_FPREGS+88(a0)
	EX	ldc1 $f13, SC_FPREGS+104(a0)
	EX	ldc1 $f15, SC_FPREGS+120(a0)
	EX	ldc1 $f17, SC_FPREGS+136(a0)
	EX	ldc1 $f19, SC_FPREGS+152(a0)
	EX	ldc1 $f21, SC_FPREGS+168(a0)
	EX	ldc1 $f23, SC_FPREGS+184(a0)
	EX	ldc1 $f25, SC_FPREGS+200(a0)
	EX	ldc1 $f27, SC_FPREGS+216(a0)
	EX	ldc1 $f29, SC_FPREGS+232(a0)
	EX	ldc1 $f31, SC_FPREGS+248(a0)

#else
#ifdef CONFIG_CPU_MIPS32_R2
	.set    push
	.set    mips32r2
	.set    fp=64
	SET_HARDFLOAT
	.set    noreorder
	mfc0    t1, CP0_STATUS
	sll     t1, t1, 31 - _ST0_FR
	bgez    t1, 1f                          # 16 / 32 register mode?
	 nop

	EX      ldc1 $f1, SC_FPREGS+8(a0)
	EX      ldc1 $f3, SC_FPREGS+24(a0)
	EX      ldc1 $f5, SC_FPREGS+40(a0)
	EX      ldc1 $f7, SC_FPREGS+56(a0)
	EX      ldc1 $f9, SC_FPREGS+72(a0)
	EX      ldc1 $f11, SC_FPREGS+88(a0)
	EX      ldc1 $f13, SC_FPREGS+104(a0)
	EX      ldc1 $f15, SC_FPREGS+120(a0)
	EX      ldc1 $f17, SC_FPREGS+136(a0)
	EX      ldc1 $f19, SC_FPREGS+152(a0)
	EX      ldc1 $f21, SC_FPREGS+168(a0)
	EX      ldc1 $f23, SC_FPREGS+184(a0)
	EX      ldc1 $f25, SC_FPREGS+200(a0)
	EX      ldc1 $f27, SC_FPREGS+216(a0)
	EX      ldc1 $f29, SC_FPREGS+232(a0)
	EX      ldc1 $f31, SC_FPREGS+248(a0)
1:
	.set    pop
#endif
#endif
	EX	ldc1 $f0, SC_FPREGS+0(a0)
	EX	ldc1 $f2, SC_FPREGS+16(a0)
	EX	ldc1 $f4, SC_FPREGS+32(a0)
	EX	ldc1 $f6, SC_FPREGS+48(a0)
	EX	ldc1 $f8, SC_FPREGS+64(a0)
	EX	ldc1 $f10, SC_FPREGS+80(a0)
	EX	ldc1 $f12, SC_FPREGS+96(a0)
	EX	ldc1 $f14, SC_FPREGS+112(a0)
	EX	ldc1 $f16, SC_FPREGS+128(a0)
	EX	ldc1 $f18, SC_FPREGS+144(a0)
	EX	ldc1 $f20, SC_FPREGS+160(a0)
	EX	ldc1 $f22, SC_FPREGS+176(a0)
	EX	ldc1 $f24, SC_FPREGS+192(a0)
	EX	ldc1 $f26, SC_FPREGS+208(a0)
	EX	ldc1 $f28, SC_FPREGS+224(a0)
	EX	ldc1 $f30, SC_FPREGS+240(a0)
	ctc1	t0, fcr31
	jr	ra
	 li	v0, 0					# success
	END(_restore_fp_context)

#ifdef CONFIG_MIPS32_COMPAT
LEAF(_restore_fp_context32)
	/* Restore an o32 sigcontext.  */
	EX	lw t0, SC32_FPC_CSR(a0)

	mfc0    t1, CP0_STATUS
	sll     t1, t1, 31 - _ST0_FR
	bgez    t1, 1f                          # 16 / 32 register mode?
	 nop

	EX      ldc1 $f1, SC32_FPREGS+8(a0)
	EX      ldc1 $f3, SC32_FPREGS+24(a0)
	EX      ldc1 $f5, SC32_FPREGS+40(a0)
	EX      ldc1 $f7, SC32_FPREGS+56(a0)
	EX      ldc1 $f9, SC32_FPREGS+72(a0)
	EX      ldc1 $f11, SC32_FPREGS+88(a0)
	EX      ldc1 $f13, SC32_FPREGS+104(a0)
	EX      ldc1 $f15, SC32_FPREGS+120(a0)
	EX      ldc1 $f17, SC32_FPREGS+136(a0)
	EX      ldc1 $f19, SC32_FPREGS+152(a0)
	EX      ldc1 $f21, SC32_FPREGS+168(a0)
	EX      ldc1 $f23, SC32_FPREGS+184(a0)
	EX      ldc1 $f25, SC32_FPREGS+200(a0)
	EX      ldc1 $f27, SC32_FPREGS+216(a0)
	EX      ldc1 $f29, SC32_FPREGS+232(a0)
	EX      ldc1 $f31, SC32_FPREGS+248(a0)
1:

	EX      ldc1 $f0, SC32_FPREGS+0(a0)
	EX	ldc1 $f2, SC32_FPREGS+16(a0)
	EX	ldc1 $f4, SC32_FPREGS+32(a0)
	EX	ldc1 $f6, SC32_FPREGS+48(a0)
	EX	ldc1 $f8, SC32_FPREGS+64(a0)
	EX	ldc1 $f10, SC32_FPREGS+80(a0)
	EX	ldc1 $f12, SC32_FPREGS+96(a0)
	EX	ldc1 $f14, SC32_FPREGS+112(a0)
	EX	ldc1 $f16, SC32_FPREGS+128(a0)
	EX	ldc1 $f18, SC32_FPREGS+144(a0)
	EX	ldc1 $f20, SC32_FPREGS+160(a0)
	EX	ldc1 $f22, SC32_FPREGS+176(a0)
	EX	ldc1 $f24, SC32_FPREGS+192(a0)
	EX	ldc1 $f26, SC32_FPREGS+208(a0)
	EX	ldc1 $f28, SC32_FPREGS+224(a0)
	EX	ldc1 $f30, SC32_FPREGS+240(a0)
	ctc1	t0, fcr31
	jr	ra
	 li	v0, 0					# success
	END(_restore_fp_context32)
#endif

#ifdef CONFIG_CPU_HAS_MSA

	.macro	save_msa_upper	wr, off, base
	.set	push
	.set	noat
#ifdef CONFIG_64BIT
	copy_u_d \wr, 1
	EX sd	$1, \off(\base)
#elif defined(CONFIG_CPU_LITTLE_ENDIAN)
	copy_u_w \wr, 2
	EX sw	$1, \off(\base)
	copy_u_w \wr, 3
	EX sw	$1, (\off+4)(\base)
#else /* CONFIG_CPU_BIG_ENDIAN */
	copy_u_w \wr, 2
	EX sw	$1, (\off+4)(\base)
	copy_u_w \wr, 3
	EX sw	$1, \off(\base)
#endif
	.set	pop
	.endm

LEAF(_save_msa_all_upper)
	save_msa_upper	0, 0x00, a0
	save_msa_upper	1, 0x08, a0
	save_msa_upper	2, 0x10, a0
	save_msa_upper	3, 0x18, a0
	save_msa_upper	4, 0x20, a0
	save_msa_upper	5, 0x28, a0
	save_msa_upper	6, 0x30, a0
	save_msa_upper	7, 0x38, a0
	save_msa_upper	8, 0x40, a0
	save_msa_upper	9, 0x48, a0
	save_msa_upper	10, 0x50, a0
	save_msa_upper	11, 0x58, a0
	save_msa_upper	12, 0x60, a0
	save_msa_upper	13, 0x68, a0
	save_msa_upper	14, 0x70, a0
	save_msa_upper	15, 0x78, a0
	save_msa_upper	16, 0x80, a0
	save_msa_upper	17, 0x88, a0
	save_msa_upper	18, 0x90, a0
	save_msa_upper	19, 0x98, a0
	save_msa_upper	20, 0xa0, a0
	save_msa_upper	21, 0xa8, a0
	save_msa_upper	22, 0xb0, a0
	save_msa_upper	23, 0xb8, a0
	save_msa_upper	24, 0xc0, a0
	save_msa_upper	25, 0xc8, a0
	save_msa_upper	26, 0xd0, a0
	save_msa_upper	27, 0xd8, a0
	save_msa_upper	28, 0xe0, a0
	save_msa_upper	29, 0xe8, a0
	save_msa_upper	30, 0xf0, a0
	save_msa_upper	31, 0xf8, a0
	jr	ra
	 li	v0, 0
	END(_save_msa_all_upper)

	.macro	restore_msa_upper	wr, off, base
	.set	push
	.set	noat
#ifdef CONFIG_64BIT
	EX ld	$1, \off(\base)
	insert_d \wr, 1
#elif defined(CONFIG_CPU_LITTLE_ENDIAN)
	EX lw	$1, \off(\base)
	insert_w \wr, 2
	EX lw	$1, (\off+4)(\base)
	insert_w \wr, 3
#else /* CONFIG_CPU_BIG_ENDIAN */
	EX lw	$1, (\off+4)(\base)
	insert_w \wr, 2
	EX lw	$1, \off(\base)
	insert_w \wr, 3
#endif
	.set	pop
	.endm

LEAF(_restore_msa_all_upper)
	restore_msa_upper	0, 0x00, a0
	restore_msa_upper	1, 0x08, a0
	restore_msa_upper	2, 0x10, a0
	restore_msa_upper	3, 0x18, a0
	restore_msa_upper	4, 0x20, a0
	restore_msa_upper	5, 0x28, a0
	restore_msa_upper	6, 0x30, a0
	restore_msa_upper	7, 0x38, a0
	restore_msa_upper	8, 0x40, a0
	restore_msa_upper	9, 0x48, a0
	restore_msa_upper	10, 0x50, a0
	restore_msa_upper	11, 0x58, a0
	restore_msa_upper	12, 0x60, a0
	restore_msa_upper	13, 0x68, a0
	restore_msa_upper	14, 0x70, a0
	restore_msa_upper	15, 0x78, a0
	restore_msa_upper	16, 0x80, a0
	restore_msa_upper	17, 0x88, a0
	restore_msa_upper	18, 0x90, a0
	restore_msa_upper	19, 0x98, a0
	restore_msa_upper	20, 0xa0, a0
	restore_msa_upper	21, 0xa8, a0
	restore_msa_upper	22, 0xb0, a0
	restore_msa_upper	23, 0xb8, a0
	restore_msa_upper	24, 0xc0, a0
	restore_msa_upper	25, 0xc8, a0
	restore_msa_upper	26, 0xd0, a0
	restore_msa_upper	27, 0xd8, a0
	restore_msa_upper	28, 0xe0, a0
	restore_msa_upper	29, 0xe8, a0
	restore_msa_upper	30, 0xf0, a0
	restore_msa_upper	31, 0xf8, a0
	jr	ra
	 li	v0, 0
	END(_restore_msa_all_upper)

	.macro  restore_msa_uppers      wr, off, base
	.set    push
	.set    noat
#ifdef CONFIG_64BIT
	ld   $1, \off(\base)
	insert_d \wr, 1
#elif defined(CONFIG_CPU_LITTLE_ENDIAN)
	lw   $1, \off(\base)
	insert_w \wr, 2
	lw   $1, (\off+4)(\base)
	insert_w \wr, 3
#else /* CONFIG_CPU_BIG_ENDIAN */
	lw   $1, (\off+4)(\base)
	insert_w \wr, 2
	lw   $1, \off(\base)
	insert_w \wr, 3
#endif
	.set    pop
	.endm

LEAF(_restore_msa_uppers_from_thread)
	restore_msa_uppers      0,  0x08, a0
	restore_msa_uppers      1,  0x18, a0
	restore_msa_uppers      2,  0x28, a0
	restore_msa_uppers      3,  0x38, a0
	restore_msa_uppers      4,  0x48, a0
	restore_msa_uppers      5,  0x58, a0
	restore_msa_uppers      6,  0x68, a0
	restore_msa_uppers      7,  0x78, a0
	restore_msa_uppers      8,  0x88, a0
	restore_msa_uppers      9,  0x98, a0
	restore_msa_uppers      10, 0xa8, a0
	restore_msa_uppers      11, 0xb8, a0
	restore_msa_uppers      12, 0xc8, a0
	restore_msa_uppers      13, 0xd8, a0
	restore_msa_uppers      14, 0xe8, a0
	restore_msa_uppers      15, 0xf8, a0
	restore_msa_uppers      16, 0x108, a0
	restore_msa_uppers      17, 0x118, a0
	restore_msa_uppers      18, 0x128, a0
	restore_msa_uppers      19, 0x138, a0
	restore_msa_uppers      20, 0x148, a0
	restore_msa_uppers      21, 0x158, a0
	restore_msa_uppers      22, 0x168, a0
	restore_msa_uppers      23, 0x178, a0
	restore_msa_uppers      24, 0x188, a0
	restore_msa_uppers      25, 0x198, a0
	restore_msa_uppers      26, 0x1a8, a0
	restore_msa_uppers      27, 0x1b8, a0
	restore_msa_uppers      28, 0x1c8, a0
	restore_msa_uppers      29, 0x1d8, a0
	restore_msa_uppers      30, 0x1e8, a0
	restore_msa_uppers      31, 0x1f8, a0
	jr      ra
	 li     v0, 0
	END(_restore_msa_uppers_from_thread)

	.macro  msa_ld_d    wd, base
	ld_d    \wd, 0, \base
	jalr    $0, $31
	  nop
	.align  4
	.endm

	.macro  msa_st_d    wd, base
	st_d    \wd, 0, \base
	jalr    $0, $31
	  nop
	.align  4
	.endm

LEAF(msa_to_wd)
	.set    push
	.set    noreorder
	sll         t0, a0, 4
	PTR_LA      t1, Lmsa_to
	PTR_ADDU    t0, t0, t1
	jalr        $0, t0
	  nop
	.align  4
Lmsa_to:
	msa_ld_d    0, a1
	msa_ld_d    1, a1
	msa_ld_d    2, a1
	msa_ld_d    3, a1
	msa_ld_d    4, a1
	msa_ld_d    5, a1
	msa_ld_d    6, a1
	msa_ld_d    7, a1
	msa_ld_d    8, a1
	msa_ld_d    9, a1
	msa_ld_d    10, a1
	msa_ld_d    11, a1
	msa_ld_d    12, a1
	msa_ld_d    13, a1
	msa_ld_d    14, a1
	msa_ld_d    15, a1
	msa_ld_d    16, a1
	msa_ld_d    17, a1
	msa_ld_d    18, a1
	msa_ld_d    19, a1
	msa_ld_d    20, a1
	msa_ld_d    21, a1
	msa_ld_d    22, a1
	msa_ld_d    23, a1
	msa_ld_d    24, a1
	msa_ld_d    25, a1
	msa_ld_d    26, a1
	msa_ld_d    27, a1
	msa_ld_d    28, a1
	msa_ld_d    29, a1
	msa_ld_d    30, a1
	msa_ld_d    31, a1
	.set    pop
	END(msa_to_wd)

LEAF(msa_from_wd)
	.set    push
	.set    noreorder
	sll         t0, a0, 4
	PTR_LA      t1, Lmsa_from
	PTR_ADDU    t0, t0, t1
	jalr        $0, t0
	  nop
	.align  4
Lmsa_from:
	msa_st_d    0, a1
	msa_st_d    1, a1
	msa_st_d    2, a1
	msa_st_d    3, a1
	msa_st_d    4, a1
	msa_st_d    5, a1
	msa_st_d    6, a1
	msa_st_d    7, a1
	msa_st_d    8, a1
	msa_st_d    9, a1
	msa_st_d    10, a1
	msa_st_d    11, a1
	msa_st_d    12, a1
	msa_st_d    13, a1
	msa_st_d    14, a1
	msa_st_d    15, a1
	msa_st_d    16, a1
	msa_st_d    17, a1
	msa_st_d    18, a1
	msa_st_d    19, a1
	msa_st_d    20, a1
	msa_st_d    21, a1
	msa_st_d    22, a1
	msa_st_d    23, a1
	msa_st_d    24, a1
	msa_st_d    25, a1
	msa_st_d    26, a1
	msa_st_d    27, a1
	msa_st_d    28, a1
	msa_st_d    29, a1
	msa_st_d    30, a1
	msa_st_d    31, a1
	.set    pop
	END(msa_from_wd)

#endif /* CONFIG_CPU_HAS_MSA */

	.set	reorder

	.type	fault@function
	.ent	fault
fault:	li	v0, -EFAULT				# failure
	jr	ra
	.end	fault
