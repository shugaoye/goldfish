/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995 - 2000, 2001 by Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2001 MIPS Technologies, Inc.
 * Copyright (C) 2004 Thiemo Seufer
 *
 * Hairy, the userspace application uses a different argument passing
 * convention than the kernel, so we have to translate things from o32
 * to ABI64 calling convention.	 64-bit syscalls are also processed
 * here for now.
 */
#include <linux/errno.h>
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/irqflags.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/thread_info.h>
#include <asm/unistd.h>
#include <asm/sysmips.h>

	.align	5
NESTED(handle_sys, PT_SIZE, sp)
	.set	noat
	SAVE_SOME
	TRACE_IRQS_ON_RELOAD
	STI
	.set	at
	ld	t1, PT_EPC(sp)		# skip syscall on return

	dsubu	t0, v0, __NR_O32_Linux	# check syscall number
	sltiu	t0, t0, __NR_O32_Linux_syscalls + 1
	daddiu	t1, 4			# skip to next instruction
	sd	t1, PT_EPC(sp)
	beqz	t0, not_o32_scall
#if 0
 SAVE_ALL
 move a1, v0
 PRINT("Scall %ld\n")
 RESTORE_ALL
#endif

	/* We don't want to stumble over broken sign extensions from
	   userland. O32 does never use the upper half. */
	sll	a0, a0, 0
	sll	a1, a1, 0
	sll	a2, a2, 0
	sll	a3, a3, 0

	dsll	t0, v0, 4		# offset into table
	ld	t2, (sys_call_table - (__NR_O32_Linux * 16))(t0)
	ld	t3, (sys_call_table - (__NR_O32_Linux * 16) + 8)(t0) # >= 0 if we need stack arguments
	sd	a3, PT_R26(sp)		# save a3 for syscall restarting
	bgez	t3, stackargs

stack_done:
	li	t1, _TIF_SYSCALL_TRACE | _TIF_SYSCALL_AUDIT
	LONG_L	t0, TI_FLAGS($28)	# syscall tracing enabled?
	and	t0, t1, t0
	bnez	t0, trace_a_syscall

	jalr	t2			# Do The Real Thing (TM)

	li	t0, -EMAXERRNO - 1	# error?
	sltu	t0, t0, v0
	sd	t0, PT_R7(sp)		# set error flag
	beqz	t0, 1f

	ld	t1, PT_R2(sp)		# syscall number
	dnegu	v0			# error
	sd	t1, PT_R0(sp)		# save it for syscall restarting
1:	sd	v0, PT_R2(sp)		# result

o32_syscall_exit:
	j	syscall_exit_partial

/* ------------------------------------------------------------------------ */

	/*
	 * More than four arguments.  Try to deal with it by copying the
	 * stack arguments from the user stack to the kernel stack.
	 * This Sucks (TM).
	 *
	 */
stackargs:
	ld	t0, PT_R29(sp)		# get old user stack pointer

	/*
	 * We intentionally keep the kernel stack a little below the top of
	 * userspace so we don't have to do a slower byte accurate check here.
	 */
	daddu	t1, t0, 32
	bltz	t1, bad_stack

	/* Ok, copy the args from the luser stack to the kernel stack.
	 * t3 is the precomputed number of instruction bytes needed to
	 * load or store arguments 6-8.
	 */

	dla	t1, 5f			# load up to 4 additional arguments
	subu	t1, t3
	.set	push
	.set	noreorder
	.set	nomacro
	jr	t1
	nop
2:	lw	a7, 28(t0)		# argument #8 from usp
3:	lw	a6, 24(t0)		# argument #7 from usp
4:	lw	a5, 20(t0)		# argument #6 from usp
5:
	b stack_done
1:	lw	a4, 16(t0)		# argument #5 from usp
	.set pop

	.section __ex_table,"a"
	PTR	1b, bad_stack
	PTR	2b, bad_stack
	PTR	3b, bad_stack
	PTR	4b, bad_stack
	.previous
/* ------------------------------------------------------------------------ */

trace_a_syscall:
	SAVE_STATIC
	sd	a4, PT_R8(sp)		# Save argument registers
	sd	a5, PT_R9(sp)
	sd	a6, PT_R10(sp)
	sd	a7, PT_R11(sp)		# For indirect syscalls

	move	s0, t2			# Save syscall pointer
	move	a0, sp
	jal	syscall_trace_enter

	move	t0, s0
	RESTORE_STATIC
	ld	a0, PT_R4(sp)		# Restore argument registers
	ld	a1, PT_R5(sp)
	ld	a2, PT_R6(sp)
	ld	a3, PT_R7(sp)
	ld	a4, PT_R8(sp)
	ld	a5, PT_R9(sp)
	ld	a6, PT_R10(sp)
	ld	a7, PT_R11(sp)		# For indirect syscalls
	jalr	t0

	li	t0, -EMAXERRNO - 1	# error?
	sltu	t0, t0, v0
	sd	t0, PT_R7(sp)		# set error flag
	beqz	t0, 1f

	ld	t1, PT_R2(sp)		# syscall number
	dnegu	v0			# error
	sd	t1, PT_R0(sp)		# save it for syscall restarting
1:	sd	v0, PT_R2(sp)		# result

	j	syscall_exit

/* ------------------------------------------------------------------------ */

	/*
	 * The stackpointer for a call with more than 4 arguments is bad.
	 */
bad_stack:
	li	v0, EFAULT
	sd	v0, PT_R2(sp)
	li	t0, 1			# set error flag
	sd	t0, PT_R7(sp)
	j	o32_syscall_exit

not_o32_scall:
	/*
	 * This is not an o32 compatibility syscall, pass it on
	 * to the 64-bit syscall handlers.
	 */
#ifdef CONFIG_MIPS32_N32
	j	handle_sysn32
#else
	j	handle_sys64
#endif
	END(handle_sys)

LEAF(sys32_syscall)
	subu	t0, a0, __NR_O32_Linux	# check syscall number
	sltiu	v0, t0, __NR_O32_Linux_syscalls + 1
	beqz	t0, einval		# do not recurse
	dsll	t1, t0, 4
	beqz	v0, einval
	ld	t2, sys_call_table(t1)		# syscall routine

	move	a0, a1			# shift argument registers
	move	a1, a2
	move	a2, a3
	move	a3, a4
	move	a4, a5
	move	a5, a6
	move	a6, a7
	sd	a0, PT_R4(sp)		# ... and push back a0 - a3, some
	sd	a1, PT_R5(sp)		# syscalls expect them there
	sd	a2, PT_R6(sp)
	sd	a3, PT_R7(sp)
	sd	a3, PT_R26(sp)		# update a3 for syscall restarting
	jr	t2
	/* Unreached */

einval: li	v0, -ENOSYS
	jr	ra
	END(sys32_syscall)

	/* We pre-compute the number of _instruction_ bytes needed to
	   load or store the arguments 6-8. Negative values are ignored. */

	.macro  sys function, nargs
	PTR     \function
	LONG    (\nargs << 2) - (5 << 2)
	.endm

	.align	3
	.type	sys_call_table,@object
sys_call_table:
	sys	sys32_syscall		8	/* 4000 */
	sys	sys_exit		1
	sys	__sys_fork		0
	sys	sys_read		3
	sys	sys_write		3
	sys	compat_sys_open		3	/* 4005 */
	sys	sys_close		1
	sys	sys_waitpid		3
	sys	sys_creat		2
	sys	sys_link		2
	sys	sys_unlink		1	/* 4010 */
	sys	compat_sys_execve	0
	sys	sys_chdir		1
	sys	compat_sys_time		1
	sys	sys_mknod		3
	sys	sys_chmod		2	/* 4015 */
	sys	sys_lchown		3
	sys	sys_ni_syscall		0
	sys	sys_ni_syscall		0	/* was sys_stat */
	sys	sys_lseek		3
	sys	sys_getpid		0	/* 4020 */
	sys	compat_sys_mount	5
	sys	sys_oldumount		1
	sys	sys_setuid		1
	sys	sys_getuid		0
	sys	compat_sys_stime	1	/* 4025 */
	sys	compat_sys_ptrace	4
	sys	sys_alarm		1
	sys	sys_ni_syscall		0	/* was sys_fstat */
	sys	sys_pause		0
	sys	compat_sys_utime	2	/* 4030 */
	sys	sys_ni_syscall		0
	sys	sys_ni_syscall		0
	sys	sys_access		2
	sys	sys_nice		1
	sys	sys_ni_syscall		0	/* 4035 */
	sys	sys_sync		0
	sys	sys_kill		2
	sys	sys_rename		2
	sys	sys_mkdir		2
	sys	sys_rmdir		1	/* 4040 */
	sys	sys_dup			1
	sys	sysm_pipe		0
	sys	compat_sys_times	1
	sys	sys_ni_syscall		0
	sys	sys_brk			1	/* 4045 */
	sys	sys_setgid		1
	sys	sys_getgid		0
	sys	sys_ni_syscall		0	/* was signal(2) */
	sys	sys_geteuid		0
	sys	sys_getegid		0	/* 4050 */
	sys	sys_acct		1
	sys	sys_umount		2
	sys	sys_ni_syscall		0
	sys	compat_sys_ioctl	3
	sys	compat_sys_fcntl	3	/* 4055 */
	sys	sys_ni_syscall		2
	sys	sys_setpgid		2
	sys	sys_ni_syscall		0
	sys	sys_olduname		1
	sys	sys_umask		1	/* 4060 */
	sys	sys_chroot		1
	sys	compat_sys_ustat	2
	sys	sys_dup2		2
	sys	sys_getppid		0
	sys	sys_getpgrp		0	/* 4065 */
	sys	sys_setsid		0
	sys	sys_32_sigaction	3
	sys	sys_sgetmask		0
	sys	sys_ssetmask		1
	sys	sys_setreuid		2	/* 4070 */
	sys	sys_setregid		2
	sys	sys32_sigsuspend	0
	sys	compat_sys_sigpending	1
	sys	sys_sethostname		2
	sys	compat_sys_setrlimit	2	/* 4075 */
	sys	compat_sys_getrlimit	2
	sys	compat_sys_getrusage	2
	sys	compat_sys_gettimeofday	2
	sys	compat_sys_settimeofday	2
	sys	sys_getgroups		2	/* 4080 */
	sys	sys_setgroups		2
	sys	sys_ni_syscall		0	/* old_select */
	sys	sys_symlink		2
	sys	sys_ni_syscall		0	/* was sys_lstat */
	sys	sys_readlink		3	/* 4085 */
	sys	sys_uselib		1
	sys	sys_swapon		2
	sys	sys_reboot		3
	sys	compat_sys_old_readdir	3
	sys	sys_mips_mmap		6	/* 4090 */
	sys	sys_munmap		2
	sys	compat_sys_truncate	2
	sys	compat_sys_ftruncate	2
	sys	sys_fchmod		2
	sys	sys_fchown		3	/* 4095 */
	sys	sys_getpriority		2
	sys	sys_setpriority		3
	sys	sys_ni_syscall		0
	sys	compat_sys_statfs	2
	sys	compat_sys_fstatfs	2	/* 4100 */
	sys	sys_ni_syscall		0	/* was ioperm(2) */
	sys	compat_sys_socketcall	2
	sys	sys_syslog		3
	sys	compat_sys_setitimer	3
	sys	compat_sys_getitimer	2	/* 4105 */
	sys	compat_sys_newstat	2
	sys	compat_sys_newlstat	2
	sys	compat_sys_newfstat	2
	sys	sys_uname		1
	sys	sys_ni_syscall		0	/* 4110 was iopl(2) */
	sys	sys_vhangup		0
	sys	sys_ni_syscall		0	/* was sys_idle() */
	sys	sys_ni_syscall		0	/* was sys_vm86 */
	sys	compat_sys_wait4	4
	sys	sys_swapoff		1	/* 4115 */
	sys	compat_sys_sysinfo	1
	sys	compat_sys_ipc		6
	sys	sys_fsync		1
	sys	sys32_sigreturn		0
	sys	__sys_clone		6	/* 4120 */
	sys	sys_setdomainname	2
	sys	sys_newuname		1
	sys	sys_ni_syscall		0	/* sys_modify_ldt */
	sys	compat_sys_adjtimex	1
	sys	sys_mprotect		3	/* 4125 */
	sys	compat_sys_sigprocmask	3
	sys	sys_ni_syscall		0	/* was create_module */
	sys	sys_init_module		5
	sys	sys_delete_module	1
	sys	sys_ni_syscall		0	/* 4130 was get_kernel_syms */
	sys	sys_quotactl		4
	sys	sys_getpgid		1
	sys	sys_fchdir		1
	sys	sys_bdflush		2
	sys	sys_sysfs		3	/* 4135 */
	sys	sys_32_personality	1
	sys	sys_ni_syscall		0	/* for afs_syscall */
	sys	sys_setfsuid		1
	sys	sys_setfsgid		1
	sys	sys_32_llseek		5	/* 4140 */
	sys	compat_sys_getdents	3
	sys	compat_sys_select	5
	sys	sys_flock		2
	sys	sys_msync		3
	sys	compat_sys_readv	3	/* 4145 */
	sys	compat_sys_writev	3
	sys	sys_cacheflush		3
	sys	sys_cachectl		3
	sys	sys_sysmips		4
	sys	sys_ni_syscall		0	/* 4150 */
	sys	sys_getsid		1
	sys	sys_fdatasync		1
	sys	compat_sys_sysctl	1
	sys	sys_mlock		2
	sys	sys_munlock		2	/* 4155 */
	sys	sys_mlockall		1
	sys	sys_munlockall		0
	sys	sys_sched_setparam	2
	sys	sys_sched_getparam	2
	sys	sys_sched_setscheduler	3	/* 4160 */
	sys	sys_sched_getscheduler	1
	sys	sys_sched_yield		0
	sys	sys_sched_get_priority_max 1
	sys	sys_sched_get_priority_min 1
	sys	compat_sys_sched_rr_get_interval 2	/* 4165 */
	sys	compat_sys_nanosleep	2
	sys	sys_mremap		5
	sys	sys_accept		3
	sys	sys_bind		3
	sys	sys_connect		3	/* 4170 */
	sys	sys_getpeername		3
	sys	sys_getsockname		3
	sys	sys_getsockopt		5
	sys	sys_listen		2
	sys	compat_sys_recv		4	/* 4175 */
	sys	compat_sys_recvfrom	6
	sys	compat_sys_recvmsg	3
	sys	sys_send		4
	sys	compat_sys_sendmsg	3
	sys	sys_sendto		6	/* 4180 */
	sys	compat_sys_setsockopt	5
	sys	sys_shutdown		2
	sys	sys_socket		3
	sys	sys_socketpair		4
	sys	sys_setresuid		3	/* 4185 */
	sys	sys_getresuid		3
	sys	sys_ni_syscall		0	/* was sys_query_module */
	sys	sys_poll		3
	sys	sys_ni_syscall		0	/* was nfsservctl */
	sys	sys_setresgid		3	/* 4190 */
	sys	sys_getresgid		3
	sys	sys_prctl		5
	sys	sys32_rt_sigreturn	0
	sys	compat_sys_rt_sigaction	4
	sys	compat_sys_rt_sigprocmask	4	/* 4195 */
	sys	compat_sys_rt_sigpending	2
	sys	compat_sys_rt_sigtimedwait	4
	sys	compat_sys_rt_sigqueueinfo	3
	sys	compat_sys_rt_sigsuspend	0
	sys	sys_32_pread		6	/* 4200 */
	sys	sys_32_pwrite		6
	sys	sys_chown		3
	sys	sys_getcwd		2
	sys	sys_capget		2
	sys	sys_capset		2	/* 4205 */
	sys	compat_sys_sigaltstack	0
	sys	compat_sys_sendfile	4
	sys	sys_ni_syscall		0
	sys	sys_ni_syscall		0
	sys	sys_mips_mmap2		6	/* 4210 */
	sys	sys_32_truncate64	4
	sys	sys_32_ftruncate64	4
	sys	sys_newstat             2
	sys	sys_newlstat            2
	sys	sys_newfstat		2	/* 4215 */
	sys	sys_pivot_root		2
	sys	sys_mincore		3
	sys	sys_madvise		3
	sys	sys_getdents64		3
	sys	compat_sys_fcntl64	3	/* 4220 */
	sys	sys_ni_syscall		0
	sys	sys_gettid		0
	sys	sys32_readahead		5
	sys	sys_setxattr		5
	sys	sys_lsetxattr		5	/* 4225 */
	sys	sys_fsetxattr		5
	sys	sys_getxattr		4
	sys	sys_lgetxattr		4
	sys	sys_fgetxattr		4
	sys	sys_listxattr		3	/* 4230 */
	sys	sys_llistxattr		3
	sys	sys_flistxattr		3
	sys	sys_removexattr		2
	sys	sys_lremovexattr	2
	sys	sys_fremovexattr	2	/* 4235 */
	sys	sys_tkill		2
	sys	sys_sendfile64		5
	sys	compat_sys_futex	6
#ifdef CONFIG_MIPS_MT_FPAFF
	/*
	 * For FPU affinity scheduling on MIPS MT processors, we need to
	 * intercept sys_sched_xxxaffinity() calls until we get a proper hook
	 * in kernel/sched.c.  Considered only temporary we only support these
	 * hooks for the 32-bit kernel - there is no MIPS64 MT processor atm.
	 */
	// FIXME These entry points do not exist
	sys	compat_mipsmt_sys_sched_setaffinity	3
	sys	compat_mipsmt_sys_sched_getaffinity	3
#else
	sys	compat_sys_sched_setaffinity	3
	sys	compat_sys_sched_getaffinity	3	/* 4240 */
#endif /* CONFIG_MIPS_MT_FPAFF */
	sys	compat_sys_io_setup		2
	sys	sys_io_destroy		1
	sys	compat_sys_io_getevents	5
	sys	compat_sys_io_submit		3
	sys	sys_io_cancel		3	/* 4245 */
	sys	sys_exit_group		1
	sys	compat_sys_lookup_dcookie	4
	sys	sys_epoll_create	1
	sys	sys_epoll_ctl		4
	sys	sys_epoll_wait		4	/* 4250 */
	sys	sys_remap_file_pages	5
	sys	sys_set_tid_address	1
	sys	sys_restart_syscall	0
	sys	sys32_fadvise64_64	7
	sys	compat_sys_statfs64	3	/* 4255 */
	sys	compat_sys_fstatfs64		2
	sys	compat_sys_timer_create	3
	sys	compat_sys_timer_settime        4
	sys	compat_sys_timer_gettime        2
	sys	sys_timer_getoverrun	1	/* 4260 */
	sys	sys_timer_delete	1
	sys	compat_sys_clock_settime	2
	sys	compat_sys_clock_gettime	2
	sys	compat_sys_clock_getres	2
	sys	compat_sys_clock_nanosleep	4	/* 4265 */
	sys	sys_tgkill		3
	sys	compat_sys_utimes		2
	sys	sys_ni_syscall		0	/* FIXME: scall32-o32 implements
 this system call with the wron number of args?! */
	sys	sys_ni_syscall		0	/* sys_get_mempolicy */
	sys	sys_ni_syscall		0	/* 4270 sys_set_mempolicy */
	sys	compat_sys_mq_open		4
	sys	sys_mq_unlink		1
	sys	compat_sys_mq_timedsend	5
	sys	compat_sys_mq_timedreceive	5
	sys	compat_sys_mq_notify		2	/* 4275 */
	sys	compat_sys_mq_getsetattr	3
	sys	sys_ni_syscall		0	/* sys_vserver */
	sys	compat_sys_waitid		5
	sys	sys_ni_syscall		0	/* available, was setaltroot */
	sys	sys_add_key		5	/* 4280 */
	sys	sys_request_key		4
	sys	sys_keyctl		5
	sys	sys_set_thread_area	1
	sys	sys_inotify_init	0
	sys	sys_inotify_add_watch	3	/* 4285 */
	sys	sys_inotify_rm_watch	2
	sys	sys_migrate_pages	4
	sys	compat_sys_openat		4
	sys	sys_mkdirat		3
	sys	sys_mknodat		4	/* 4290 */
	sys	sys_fchownat		5
	sys	compat_sys_futimesat		3
	sys	sys_newfstatat		4
	sys	sys_unlinkat		3
	sys	sys_renameat		4	/* 4295 */
	sys	sys_linkat		5
	sys	sys_symlinkat		3
	sys	sys_readlinkat		4
	sys	sys_fchmodat		3
	sys	sys_faccessat		3	/* 4300 */
	sys	compat_sys_pselect6		6
	sys	compat_sys_ppoll		5
	sys	sys_unshare		1
	sys	sys_splice		6
	sys	sys32_sync_file_range	7	/* 4305 */
	sys	sys_tee			4
	sys	compat_sys_vmsplice		4
	sys	compat_sys_move_pages		6
	sys	compat_sys_set_robust_list	2
	sys	compat_sys_get_robust_list	3	/* 4310 */
	sys	compat_sys_kexec_load		4
	sys	sys_getcpu		3
	sys	compat_sys_epoll_pwait		6
	sys	sys_ioprio_set		3
	sys	sys_ioprio_get		2	/* 4315 */
	sys	compat_sys_utimensat		4
	sys	compat_sys_signalfd		3
	sys	sys_ni_syscall		0	/* was timerfd */
	sys	sys_eventfd		1
	sys	sys32_fallocate		6	/* 4320 */
	sys	sys_timerfd_create	2
	sys	compat_sys_timerfd_gettime	2
	sys	compat_sys_timerfd_settime	4
	sys	compat_sys_signalfd4		4
	sys	sys_eventfd2		2	/* 4325 */
	sys	sys_epoll_create1	1
	sys	sys_dup3		3
	sys	sys_pipe2		2
	sys	sys_inotify_init1	1
	sys	compat_sys_preadv		6	/* 4330 */
	sys	compat_sys_pwritev		6
	sys	compat_sys_rt_tgsigqueueinfo	4
	sys	sys_perf_event_open	5
	sys	sys_accept4		4
	sys	compat_sys_recvmmsg		5	/* 4335 */
	sys	sys_fanotify_init	2
	sys	compat_sys_fanotify_mark	6
	sys	sys_prlimit64		4
	sys	sys_name_to_handle_at	5
	sys	compat_sys_open_by_handle_at	3	/* 4340 */
	sys	compat_sys_clock_adjtime	2
	sys	sys_syncfs		1
	sys	compat_sys_sendmmsg		4
	sys	sys_setns		2
	sys	compat_sys_process_vm_readv	6	/* 4345 */
	sys	compat_sys_process_vm_writev	6
	sys	sys_kcmp		5
	sys	sys_finit_module	3
	/* Backporting seccomp, skip a few ... */
	sys	sys_ni_syscall		0	/* sys_sched_setattr */
	sys	sys_ni_syscall		0	/* sys_sched_getattr */		/* 4350 */
	sys	sys_ni_syscall		0	/* sys_renameat2 */
	sys	sys_seccomp 3
	.size	sys_call_table,.-sys_call_table
