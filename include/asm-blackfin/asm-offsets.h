#ifndef __ASM_OFFSETS_H__
#define __ASM_OFFSETS_H__
/*
 * DO NOT MODIFY.
 *
 * This file was generated by arch/bfinnommu/Makefile
 *
 */

#define TASK_STATE 0 /* offsetof(struct task_struct, state) */
#define TASK_FLAGS 12 /* offsetof(struct task_struct, flags) */
#define TASK_PTRACE 16 /* offsetof(struct task_struct, ptrace) */
#define TASK_BLOCKED 844 /* offsetof(struct task_struct, blocked) */
#define TASK_THREAD 804 /* offsetof(struct task_struct, thread) */
#define TASK_THREAD_INFO 4 /* offsetof(struct task_struct, thread_info) */
#define TASK_MM 104 /* offsetof(struct task_struct, mm) */
#define TASK_ACTIVE_MM 108 /* offsetof(struct task_struct, active_mm) */
#define TASK_SIGPENDING 860 /* offsetof(struct task_struct, pending) */
#define STAT_IRQ 28 /* offsetof(struct kernel_stat, irqs) */
#define CPUSTAT_SOFTIRQ_PENDING 0 /* offsetof(irq_cpustat_t, __softirq_pending) */
#define THREAD_KSP 0 /* offsetof(struct thread_struct, ksp) */
#define THREAD_USP 4 /* offsetof(struct thread_struct, usp) */
#define THREAD_SR 8 /* offsetof(struct thread_struct, seqstat) */
#define PT_SR 8 /* offsetof(struct thread_struct, seqstat) */
#define THREAD_ESP0 12 /* offsetof(struct thread_struct, esp0) */
#define THREAD_PC 16 /* offsetof(struct thread_struct, pc) */
#define PT_R0 204 /* offsetof(struct pt_regs, r0) */
#define PT_ORIG_R0 208 /* offsetof(struct pt_regs, orig_r0) */
#define PT_R1 200 /* offsetof(struct pt_regs, r1) */
#define PT_R2 196 /* offsetof(struct pt_regs, r2) */
#define PT_R3 192 /* offsetof(struct pt_regs, r3) */
#define PT_R4 188 /* offsetof(struct pt_regs, r4) */
#define PT_R5 184 /* offsetof(struct pt_regs, r5) */
#define PT_R6 180 /* offsetof(struct pt_regs, r6) */
#define PT_R7 176 /* offsetof(struct pt_regs, r7) */
#define PT_P0 172 /* offsetof(struct pt_regs, p0) */
#define PT_P1 168 /* offsetof(struct pt_regs, p1) */
#define PT_P2 164 /* offsetof(struct pt_regs, p2) */
#define PT_P3 160 /* offsetof(struct pt_regs, p3) */
#define PT_P4 156 /* offsetof(struct pt_regs, p4) */
#define PT_P5 152 /* offsetof(struct pt_regs, p5) */
#define PT_A0w 72 /* offsetof(struct pt_regs, a0w) */
#define PT_A1w 64 /* offsetof(struct pt_regs, a1w) */
#define PT_A0x 76 /* offsetof(struct pt_regs, a0x) */
#define PT_A1x 68 /* offsetof(struct pt_regs, a1x) */
#define PT_PC 24 /* offsetof(struct pt_regs, pc) */
#define PT_IPEND 0 /* offsetof(struct pt_regs, ipend) */
#define PT_SYSCFG 4 /* offsetof(struct pt_regs, syscfg) */
#define PT_SEQSTAT 8 /* offsetof(struct pt_regs, seqstat) */
#define PT_RETE 12 /* offsetof(struct pt_regs, rete) */
#define PT_RETN 16 /* offsetof(struct pt_regs, retn) */
#define PT_RETX 20 /* offsetof(struct pt_regs, retx) */
#define PT_RETS 28 /* offsetof(struct pt_regs, rets) */
#define PT_RESERVED 32 /* offsetof(struct pt_regs, reserved) */
#define PT_ASTAT 36 /* offsetof(struct pt_regs, astat) */
#define PT_LB0 44 /* offsetof(struct pt_regs, lb0) */
#define PT_LB1 40 /* offsetof(struct pt_regs, lb1) */
#define PT_LT0 52 /* offsetof(struct pt_regs, lt0) */
#define PT_LT1 48 /* offsetof(struct pt_regs, lt1) */
#define PT_LC0 60 /* offsetof(struct pt_regs, lc0) */
#define PT_LC1 56 /* offsetof(struct pt_regs, lc1) */
#define PT_B0 92 /* offsetof(struct pt_regs, b0) */
#define PT_B1 88 /* offsetof(struct pt_regs, b1) */
#define PT_B2 84 /* offsetof(struct pt_regs, b2) */
#define PT_B3 80 /* offsetof(struct pt_regs, b3) */
#define PT_M0 124 /* offsetof(struct pt_regs, m0) */
#define PT_M1 120 /* offsetof(struct pt_regs, m1) */
#define PT_M2 116 /* offsetof(struct pt_regs, m2) */
#define PT_M3 112 /* offsetof(struct pt_regs, m3) */
#define PT_I0 140 /* offsetof(struct pt_regs, i0) */
#define PT_I1 136 /* offsetof(struct pt_regs, i1) */
#define PT_I2 132 /* offsetof(struct pt_regs, i2) */
#define PT_I3 128 /* offsetof(struct pt_regs, i3) */
#define PT_USP 144 /* offsetof(struct pt_regs, usp) */
#define PT_FP 148 /* offsetof(struct pt_regs, fp) */
#define PT_VECTOR 28 /* offsetof(struct pt_regs, pc) + 4 */
#define IRQ_HANDLER 0 /* offsetof(struct irq_node, handler) */
#define IRQ_DEVID 8 /* offsetof(struct irq_node, dev_id) */
#define IRQ_NEXT 16 /* offsetof(struct irq_node, next) */
#define STAT_IRQ 28 /* offsetof(struct kernel_stat, irqs) */
#define SIGSEGV 11 /* SIGSEGV */
#define SIGTRAP 5 /* SIGTRAP */

#endif
