/*
 * asmmacro.h: Assembler macros to make things easier to read.
 *
 * Copyright (C) 1996 David S. Miller (davem@davemloft.net)
 * Copyright (C) 1998, 1999, 2003 Ralf Baechle
 */
#ifndef _ASM_ASMMACRO_32_H
#define _ASM_ASMMACRO_32_H

#include <asm/asm-offsets.h>
#include <asm/regdef.h>
#include <asm/fpregdef.h>
#include <asm/mipsregs.h>

	.macro	cpu_save_nonscratch thread
	LONG_S	s0, THREAD_REG16(\thread)
	LONG_S	s1, THREAD_REG17(\thread)
	LONG_S	s2, THREAD_REG18(\thread)
	LONG_S	s3, THREAD_REG19(\thread)
	LONG_S	s4, THREAD_REG20(\thread)
	LONG_S	s5, THREAD_REG21(\thread)
	LONG_S	s6, THREAD_REG22(\thread)
	LONG_S	s7, THREAD_REG23(\thread)
	LONG_S	sp, THREAD_REG29(\thread)
	LONG_S	fp, THREAD_REG30(\thread)
	.endm

	.macro	cpu_restore_nonscratch thread
	LONG_L	s0, THREAD_REG16(\thread)
	LONG_L	s1, THREAD_REG17(\thread)
	LONG_L	s2, THREAD_REG18(\thread)
	LONG_L	s3, THREAD_REG19(\thread)
	LONG_L	s4, THREAD_REG20(\thread)
	LONG_L	s5, THREAD_REG21(\thread)
	LONG_L	s6, THREAD_REG22(\thread)
	LONG_L	s7, THREAD_REG23(\thread)
	LONG_L	sp, THREAD_REG29(\thread)
	LONG_L	fp, THREAD_REG30(\thread)
	LONG_L	ra, THREAD_REG31(\thread)
	.endm

/* preprocessor replaces the fp in ".set fp=64" with $30 otherwise */
#undef fp

	.macro  fpu_get_fcr31   thread  status  tmp
#ifdef CONFIG_CPU_MIPSR6
	lw      \status, THREAD_FCR31(\thread)
	lui     \tmp, (FPU_CSR_COND0|FPU_CSR_COND1|FPU_CSR_COND2|FPU_CSR_COND3| \
		    FPU_CSR_COND4|FPU_CSR_COND5|FPU_CSR_COND6|FPU_CSR_COND7)>>16
	and     \tmp, \status, \tmp
	cfc1    \status, fcr31
	or      \tmp, \status, \tmp
#else
	cfc1    \tmp, fcr31
#endif
	.endm

#if defined(CONFIG_CPU_MIPS32_R2) || defined(CONFIG_CPU_MIPS32_R6)

	/* copy stuff from MIPS64 */

	.macro  fpu_save_16even thread tmp=t0
	.set    push
	SET_HARDFLOAT
	sdc1    $f0,  THREAD_FPR0_LS64(\thread)
	sdc1    $f2,  THREAD_FPR2_LS64(\thread)
	sdc1    $f4,  THREAD_FPR4_LS64(\thread)
	sdc1    $f6,  THREAD_FPR6_LS64(\thread)
	sdc1    $f8,  THREAD_FPR8_LS64(\thread)
	sdc1    $f10, THREAD_FPR10_LS64(\thread)
	sdc1    $f12, THREAD_FPR12_LS64(\thread)
	sdc1    $f14, THREAD_FPR14_LS64(\thread)
	sdc1    $f16, THREAD_FPR16_LS64(\thread)
	sdc1    $f18, THREAD_FPR18_LS64(\thread)
	sdc1    $f20, THREAD_FPR20_LS64(\thread)
	sdc1    $f22, THREAD_FPR22_LS64(\thread)
	sdc1    $f24, THREAD_FPR24_LS64(\thread)
	sdc1    $f26, THREAD_FPR26_LS64(\thread)
	sdc1    $f28, THREAD_FPR28_LS64(\thread)
	sdc1    $f30, THREAD_FPR30_LS64(\thread)
	.set pop
	.endm

	.macro  fpu_save_16odd thread
	.set    push
	SET_HARDFLOAT
	.set    fp=64
	sdc1    $f1,  THREAD_FPR1_LS64(\thread)
	sdc1    $f3,  THREAD_FPR3_LS64(\thread)
	sdc1    $f5,  THREAD_FPR5_LS64(\thread)
	sdc1    $f7,  THREAD_FPR7_LS64(\thread)
	sdc1    $f9,  THREAD_FPR9_LS64(\thread)
	sdc1    $f11, THREAD_FPR11_LS64(\thread)
	sdc1    $f13, THREAD_FPR13_LS64(\thread)
	sdc1    $f15, THREAD_FPR15_LS64(\thread)
	sdc1    $f17, THREAD_FPR17_LS64(\thread)
	sdc1    $f19, THREAD_FPR19_LS64(\thread)
	sdc1    $f21, THREAD_FPR21_LS64(\thread)
	sdc1    $f23, THREAD_FPR23_LS64(\thread)
	sdc1    $f25, THREAD_FPR25_LS64(\thread)
	sdc1    $f27, THREAD_FPR27_LS64(\thread)
	sdc1    $f29, THREAD_FPR29_LS64(\thread)
	sdc1    $f31, THREAD_FPR31_LS64(\thread)
	.set    pop
	.endm

	.macro  fpu_save_double thread status tmp
	.set    push
	.set    noreorder
	SET_HARDFLOAT
	sll     \tmp, \status, 31 - _ST0_FR
	bgez    \tmp, 2f
	 nop
	fpu_save_16odd \thread
2:
	fpu_save_16even \thread \tmp
	fpu_get_fcr31   \thread \status \tmp
	sw	\tmp, THREAD_FCR31(\thread)
	.set    pop
	.endm

	.macro  fpu_restore_16even thread tmp=t0
	.set push
	SET_HARDFLOAT
	lw  \tmp, THREAD_FCR31(\thread)
	ldc1    $f0,  THREAD_FPR0_LS64(\thread)
	ldc1    $f2,  THREAD_FPR2_LS64(\thread)
	ldc1    $f4,  THREAD_FPR4_LS64(\thread)
	ldc1    $f6,  THREAD_FPR6_LS64(\thread)
	ldc1    $f8,  THREAD_FPR8_LS64(\thread)
	ldc1    $f10, THREAD_FPR10_LS64(\thread)
	ldc1    $f12, THREAD_FPR12_LS64(\thread)
	ldc1    $f14, THREAD_FPR14_LS64(\thread)
	ldc1    $f16, THREAD_FPR16_LS64(\thread)
	ldc1    $f18, THREAD_FPR18_LS64(\thread)
	ldc1    $f20, THREAD_FPR20_LS64(\thread)
	ldc1    $f22, THREAD_FPR22_LS64(\thread)
	ldc1    $f24, THREAD_FPR24_LS64(\thread)
	ldc1    $f26, THREAD_FPR26_LS64(\thread)
	ldc1    $f28, THREAD_FPR28_LS64(\thread)
	ldc1    $f30, THREAD_FPR30_LS64(\thread)
	ctc1    \tmp, fcr31
	.set pop
	.endm

	.macro  fpu_restore_16odd thread
	.set push
	SET_HARDFLOAT
	.set    fp=64
	ldc1    $f1,  THREAD_FPR1_LS64(\thread)
	ldc1    $f3,  THREAD_FPR3_LS64(\thread)
	ldc1    $f5,  THREAD_FPR5_LS64(\thread)
	ldc1    $f7,  THREAD_FPR7_LS64(\thread)
	ldc1    $f9,  THREAD_FPR9_LS64(\thread)
	ldc1    $f11, THREAD_FPR11_LS64(\thread)
	ldc1    $f13, THREAD_FPR13_LS64(\thread)
	ldc1    $f15, THREAD_FPR15_LS64(\thread)
	ldc1    $f17, THREAD_FPR17_LS64(\thread)
	ldc1    $f19, THREAD_FPR19_LS64(\thread)
	ldc1    $f21, THREAD_FPR21_LS64(\thread)
	ldc1    $f23, THREAD_FPR23_LS64(\thread)
	ldc1    $f25, THREAD_FPR25_LS64(\thread)
	ldc1    $f27, THREAD_FPR27_LS64(\thread)
	ldc1    $f29, THREAD_FPR29_LS64(\thread)
	ldc1    $f31, THREAD_FPR31_LS64(\thread)
	.set    pop
	.endm

	.macro  fpu_restore_double thread status tmp
	.set    push
	.set    noreorder
	sll     \tmp, \status, 31 - _ST0_FR
	bgez    \tmp, 1f                # 16 register mode?
	 nop

	fpu_restore_16odd \thread
1:      fpu_restore_16even \thread \tmp
	.set    pop
	.endm

#else

	.macro  fpu_save_double thread status tmp1=t0
	.set push
	SET_HARDFLOAT
	fpu_get_fcr31   \thread \status \tmp1
	sdc1    $f0,  THREAD_FPR0_LS64(\thread)
	sdc1    $f2,  THREAD_FPR2_LS64(\thread)
	sdc1    $f4,  THREAD_FPR4_LS64(\thread)
	sdc1    $f6,  THREAD_FPR6_LS64(\thread)
	sdc1    $f8,  THREAD_FPR8_LS64(\thread)
	sdc1    $f10, THREAD_FPR10_LS64(\thread)
	sdc1    $f12, THREAD_FPR12_LS64(\thread)
	sdc1    $f14, THREAD_FPR14_LS64(\thread)
	sdc1    $f16, THREAD_FPR16_LS64(\thread)
	sdc1    $f18, THREAD_FPR18_LS64(\thread)
	sdc1    $f20, THREAD_FPR20_LS64(\thread)
	sdc1    $f22, THREAD_FPR22_LS64(\thread)
	sdc1    $f24, THREAD_FPR24_LS64(\thread)
	sdc1    $f26, THREAD_FPR26_LS64(\thread)
	sdc1    $f28, THREAD_FPR28_LS64(\thread)
	sdc1    $f30, THREAD_FPR30_LS64(\thread)
	sw	\tmp1, THREAD_FCR31(\thread)
	.set pop
	.endm

	.macro	fpu_save_single thread tmp=t0
	.set push
	SET_HARDFLOAT
	cfc1	\tmp,  fcr31
	swc1	$f0,  THREAD_FPR0_LS64(\thread)
	swc1	$f1,  THREAD_FPR1_LS64(\thread)
	swc1	$f2,  THREAD_FPR2_LS64(\thread)
	swc1	$f3,  THREAD_FPR3_LS64(\thread)
	swc1	$f4,  THREAD_FPR4_LS64(\thread)
	swc1	$f5,  THREAD_FPR5_LS64(\thread)
	swc1	$f6,  THREAD_FPR6_LS64(\thread)
	swc1	$f7,  THREAD_FPR7_LS64(\thread)
	swc1	$f8,  THREAD_FPR8_LS64(\thread)
	swc1	$f9,  THREAD_FPR9_LS64(\thread)
	swc1	$f10, THREAD_FPR10_LS64(\thread)
	swc1	$f11, THREAD_FPR11_LS64(\thread)
	swc1	$f12, THREAD_FPR12_LS64(\thread)
	swc1	$f13, THREAD_FPR13_LS64(\thread)
	swc1	$f14, THREAD_FPR14_LS64(\thread)
	swc1	$f15, THREAD_FPR15_LS64(\thread)
	swc1	$f16, THREAD_FPR16_LS64(\thread)
	swc1	$f17, THREAD_FPR17_LS64(\thread)
	swc1	$f18, THREAD_FPR18_LS64(\thread)
	swc1	$f19, THREAD_FPR19_LS64(\thread)
	swc1	$f20, THREAD_FPR20_LS64(\thread)
	swc1	$f21, THREAD_FPR21_LS64(\thread)
	swc1	$f22, THREAD_FPR22_LS64(\thread)
	swc1	$f23, THREAD_FPR23_LS64(\thread)
	swc1	$f24, THREAD_FPR24_LS64(\thread)
	swc1	$f25, THREAD_FPR25_LS64(\thread)
	swc1	$f26, THREAD_FPR26_LS64(\thread)
	swc1	$f27, THREAD_FPR27_LS64(\thread)
	swc1	$f28, THREAD_FPR28_LS64(\thread)
	swc1	$f29, THREAD_FPR29_LS64(\thread)
	swc1	$f30, THREAD_FPR30_LS64(\thread)
	swc1	$f31, THREAD_FPR31_LS64(\thread)
	sw	\tmp, THREAD_FCR31(\thread)
	.set pop
	.endm

	.macro	fpu_restore_double thread status tmp=t0
	.set push
	SET_HARDFLOAT
	lw	\tmp, THREAD_FCR31(\thread)
	ldc1    $f0,  THREAD_FPR0_LS64(\thread)
	ldc1    $f2,  THREAD_FPR2_LS64(\thread)
	ldc1    $f4,  THREAD_FPR4_LS64(\thread)
	ldc1    $f6,  THREAD_FPR6_LS64(\thread)
	ldc1    $f8,  THREAD_FPR8_LS64(\thread)
	ldc1    $f10, THREAD_FPR10_LS64(\thread)
	ldc1    $f12, THREAD_FPR12_LS64(\thread)
	ldc1    $f14, THREAD_FPR14_LS64(\thread)
	ldc1    $f16, THREAD_FPR16_LS64(\thread)
	ldc1    $f18, THREAD_FPR18_LS64(\thread)
	ldc1    $f20, THREAD_FPR20_LS64(\thread)
	ldc1    $f22, THREAD_FPR22_LS64(\thread)
	ldc1    $f24, THREAD_FPR24_LS64(\thread)
	ldc1    $f26, THREAD_FPR26_LS64(\thread)
	ldc1    $f28, THREAD_FPR28_LS64(\thread)
	ldc1    $f30, THREAD_FPR30_LS64(\thread)
	ctc1	\tmp, fcr31
	.set pop
	.endm

	.macro	fpu_restore_single thread tmp=t0
	.set push
	SET_HARDFLOAT
	lw	\tmp, THREAD_FCR31(\thread)
	lwc1	$f0,  THREAD_FPR0_LS64(\thread)
	lwc1	$f1,  THREAD_FPR1_LS64(\thread)
	lwc1	$f2,  THREAD_FPR2_LS64(\thread)
	lwc1	$f3,  THREAD_FPR3_LS64(\thread)
	lwc1	$f4,  THREAD_FPR4_LS64(\thread)
	lwc1	$f5,  THREAD_FPR5_LS64(\thread)
	lwc1	$f6,  THREAD_FPR6_LS64(\thread)
	lwc1	$f7,  THREAD_FPR7_LS64(\thread)
	lwc1	$f8,  THREAD_FPR8_LS64(\thread)
	lwc1	$f9,  THREAD_FPR9_LS64(\thread)
	lwc1	$f10, THREAD_FPR10_LS64(\thread)
	lwc1	$f11, THREAD_FPR11_LS64(\thread)
	lwc1	$f12, THREAD_FPR12_LS64(\thread)
	lwc1	$f13, THREAD_FPR13_LS64(\thread)
	lwc1	$f14, THREAD_FPR14_LS64(\thread)
	lwc1	$f15, THREAD_FPR15_LS64(\thread)
	lwc1	$f16, THREAD_FPR16_LS64(\thread)
	lwc1	$f17, THREAD_FPR17_LS64(\thread)
	lwc1	$f18, THREAD_FPR18_LS64(\thread)
	lwc1	$f19, THREAD_FPR19_LS64(\thread)
	lwc1	$f20, THREAD_FPR20_LS64(\thread)
	lwc1	$f21, THREAD_FPR21_LS64(\thread)
	lwc1	$f22, THREAD_FPR22_LS64(\thread)
	lwc1	$f23, THREAD_FPR23_LS64(\thread)
	lwc1	$f24, THREAD_FPR24_LS64(\thread)
	lwc1	$f25, THREAD_FPR25_LS64(\thread)
	lwc1	$f26, THREAD_FPR26_LS64(\thread)
	lwc1	$f27, THREAD_FPR27_LS64(\thread)
	lwc1	$f28, THREAD_FPR28_LS64(\thread)
	lwc1	$f29, THREAD_FPR29_LS64(\thread)
	lwc1	$f30, THREAD_FPR30_LS64(\thread)
	lwc1	$f31, THREAD_FPR31_LS64(\thread)
	ctc1	\tmp, fcr31
	.set pop
	.endm

#endif  // CONFIG_CPU_MIPS32_R2
#endif /* _ASM_ASMMACRO_32_H */
