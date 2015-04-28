/*
 * Copyright (C) 2014 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 * Edited by: Leonid Yegoshin <Leonid.Yegoshin@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/elf.h>
#include <linux/sched.h>

/* ELF Binary and interpreter FPU ABI matching matrix.
 * Line format: { ANY,  DOUBLE,  SINGLE, SOFT,
 *                  OLD_64,   XX,  64,    64A }
 *
 * Note: SOFT + SMTHNG is set to SMTHNG because today there is NO inter calls
 *      between ELF binary and interpreter with float point arguments.
 *      It should be FP_ERROR in other case.
 *      It is not applied to NXX because of total incompatibility of O32 and N32/N64
 */
static int mips_abi_fp_mix[MIPS_ABI_FP_MAX][MIPS_ABI_FP_MAX] = {
/* ANY */
{ MIPS_ABI_FP_ANY, MIPS_ABI_FP_DOUBLE, MIPS_ABI_FP_SINGLE, MIPS_ABI_FP_SOFT,
	MIPS_ABI_FP_OLD_64, MIPS_ABI_FP_XX, MIPS_ABI_FP_64, MIPS_ABI_FP_64A },
/* DOUBLE */
{ MIPS_ABI_FP_DOUBLE, MIPS_ABI_FP_DOUBLE, FP_ERROR, MIPS_ABI_FP_DOUBLE,
	FP_ERROR, MIPS_ABI_FP_DOUBLE, FP_ERROR, FP_DOUBLE_64A },
/* SINGLE */
{ MIPS_ABI_FP_SINGLE, FP_ERROR, MIPS_ABI_FP_SINGLE, MIPS_ABI_FP_SINGLE,
	FP_ERROR, FP_ERROR, FP_ERROR, FP_ERROR },
/* SOFT */
{ MIPS_ABI_FP_SOFT, MIPS_ABI_FP_DOUBLE, MIPS_ABI_FP_SINGLE, MIPS_ABI_FP_SOFT,
	MIPS_ABI_FP_OLD_64, MIPS_ABI_FP_XX, MIPS_ABI_FP_64, MIPS_ABI_FP_64A },

/* OLD_64 */
{ MIPS_ABI_FP_OLD_64, FP_ERROR, FP_ERROR, MIPS_ABI_FP_OLD_64,
	MIPS_ABI_FP_OLD_64, FP_ERROR, FP_ERROR, FP_ERROR },
/* XX */
{ MIPS_ABI_FP_XX, MIPS_ABI_FP_DOUBLE, FP_ERROR, MIPS_ABI_FP_XX,
	FP_ERROR, MIPS_ABI_FP_XX, MIPS_ABI_FP_64, MIPS_ABI_FP_64A },
/* 64 */
{ MIPS_ABI_FP_64, FP_ERROR, FP_ERROR, MIPS_ABI_FP_64,
	FP_ERROR, MIPS_ABI_FP_64, MIPS_ABI_FP_64, MIPS_ABI_FP_64 },
/* 64A */
{ MIPS_ABI_FP_64A, FP_DOUBLE_64A, FP_ERROR, MIPS_ABI_FP_64A,
	FP_ERROR, MIPS_ABI_FP_64A, MIPS_ABI_FP_64, MIPS_ABI_FP_64A }
};

/* ELF Binary and interpreter FPU ABI matching matrix with UNKNOWN MIPS32 FPU
 * It is assumed that it is O32 DOUBLE FPU.
 * Line format: { ANY,  DOUBLE,  SINGLE, SOFT,
 *                  OLD_64,   XX,  64,    64A }
 */
static int mips_abi_fp_unknown[MIPS_ABI_FP_MAX] =
/* UNKNOWN */
{ MIPS_ABI_FP_UNKNOWN, MIPS_ABI_FP_DOUBLE, FP_ERROR, MIPS_ABI_FP_UNKNOWN,
	FP_ERROR, MIPS_ABI_FP_XX, FP_ERROR, FP_DOUBLE_64A };

int arch_elf_pt_proc(void *_ehdr, void *_phdr, struct file *elf,
		     bool is_interp, struct arch_elf_state *state)
{
	struct elf32_hdr *ehdr = _ehdr;
	struct elf32_phdr *phdr = _phdr;
	struct mips_elf_abiflags_v0 abiflags;
	int ret;

	if ((config_enabled(CONFIG_64BIT) &&
	     (ehdr->e_ident[EI_CLASS] != ELFCLASS32)) ||
	    ((ehdr->e_ident[EI_CLASS] == ELFCLASS32) &&
	     ((ehdr->e_flags & EF_MIPS_ABI2) != 0) &&
	     ((ehdr->e_flags & EF_MIPS_ABI) == 0)))
		return 0;

	if (phdr->p_type != PT_MIPS_ABIFLAGS)
		return 0;
	if (phdr->p_filesz < sizeof(abiflags))
		return -EINVAL;

	ret = kernel_read(elf, phdr->p_offset, (char *)&abiflags,
			  sizeof(abiflags));
	if (ret < 0)
		return ret;
	if (ret != sizeof(abiflags))
		return -EIO;

	/* Record the required FP ABIs for use by mips_check_elf */
	if (is_interp)
		state->interp_fp_abi = abiflags.fp_abi;
	else
		state->fp_abi = abiflags.fp_abi;

	return 0;
}

static inline unsigned get_fp_abi(struct elf32_hdr *ehdr, int in_abi)
{
	if ((in_abi >= MIPS_ABI_FP_MAX) &&
	    (in_abi != MIPS_ABI_FP_NXX) &&
	    (in_abi != MIPS_ABI_FP_UNKNOWN))
		return FP_ERROR;

	/* If the ABI requirement is provided, simply return that */
	if (in_abi != MIPS_ABI_FP_UNKNOWN)
		return in_abi;

	if ((config_enabled(CONFIG_64BIT) &&
	     (ehdr->e_ident[EI_CLASS] != ELFCLASS32)) ||
	    ((ehdr->e_ident[EI_CLASS] == ELFCLASS32) &&
	     ((ehdr->e_flags & EF_MIPS_ABI2) != 0) &&
	     ((ehdr->e_flags & EF_MIPS_ABI) == 0))) {
		/* Set NXX FP ABI for N32/N64 */
		return MIPS_ABI_FP_NXX;
	}

	/* If the EF_MIPS_32BITMODE_FP64 flag was set, return MIPS_ABI_FP_OLD_64 */
	if ((ehdr->e_ident[EI_CLASS] == ELFCLASS32) &&
	    (ehdr->e_flags & EF_MIPS_32BITMODE_FP64))
		return MIPS_ABI_FP_OLD_64;

	return MIPS_ABI_FP_UNKNOWN;
}

int arch_check_elf(void *_ehdr, bool has_interpreter,
		   struct arch_elf_state *state)
{
	struct elf32_hdr *ehdr = _ehdr;
	unsigned fp_abi, interp_fp_abi;

	fp_abi = get_fp_abi(ehdr, state->fp_abi);
	if (fp_abi == FP_ERROR)
		return -ELIBBAD;

	if (!has_interpreter) {
		state->overall_abi = fp_abi;
		return 0;
	}

	interp_fp_abi = get_fp_abi(ehdr, state->interp_fp_abi);
	if (fp_abi == FP_ERROR)
		return -ELIBBAD;

	if (interp_fp_abi == MIPS_ABI_FP_NXX) {
		if (fp_abi == MIPS_ABI_FP_NXX) {
			state->overall_abi = MIPS_ABI_FP_NXX;
			return 0;
		}
		return -ELIBBAD;
	}

	if ((interp_fp_abi == MIPS_ABI_FP_UNKNOWN) &&
	    (fp_abi == MIPS_ABI_FP_UNKNOWN)) {
		state->overall_abi = MIPS_ABI_FP_UNKNOWN;
		return 0;
	}
	if (interp_fp_abi == MIPS_ABI_FP_UNKNOWN)
		state->overall_abi = mips_abi_fp_unknown[fp_abi];
	else if (fp_abi == MIPS_ABI_FP_UNKNOWN)
		state->overall_abi = mips_abi_fp_unknown[interp_fp_abi];
	else
		state->overall_abi = mips_abi_fp_mix[fp_abi][interp_fp_abi];

	if (state->overall_abi == FP_ERROR)
		return -ELIBBAD;

	return 0;
}

void mips_set_personality_fp(struct arch_elf_state *state)
{
	switch (state->overall_abi) {
	case MIPS_ABI_FP_UNKNOWN:
	case MIPS_ABI_FP_DOUBLE:
	case MIPS_ABI_FP_SINGLE:
		/* FR=0 */
		clear_thread_local_flags(LTIF_FPU_FR|LTIF_FPU_FRE);
		break;

	case FP_DOUBLE_64A:
		/* Android elf bins mix, FR=1 and FRE=1 are enforced */
		set_thread_local_flags(LTIF_FPU_FR|LTIF_FPU_FRE);
		break;

	case MIPS_ABI_FP_NXX:
	case MIPS_ABI_FP_64:
	case MIPS_ABI_FP_64A:
	case MIPS_ABI_FP_OLD_64:
		/* FR=1 */
		set_thread_local_flags(LTIF_FPU_FR);
		clear_thread_local_flags(LTIF_FPU_FRE);
		break;

	case MIPS_ABI_FP_XX:
	case MIPS_ABI_FP_ANY:
	case MIPS_ABI_FP_SOFT:
		if (config_enabled(CONFIG_CPU_MIPSR6))
			set_thread_local_flags(LTIF_FPU_FR);
		else
			clear_thread_local_flags(LTIF_FPU_FR);

		clear_thread_local_flags(LTIF_FPU_FRE);
		break;

	default:
	case FP_ERROR:
		BUG();
	}
}
