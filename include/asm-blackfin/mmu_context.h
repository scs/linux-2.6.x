#ifndef __BLACKFIN_MMU_CONTEXT_H
#define __BLACKFIN_MMU_CONTEXT_H

#include <linux/config.h>
#include <asm/setup.h>
#include <asm/page.h>
#include <asm/pgalloc.h>

extern void *current_l1_stack_save;
extern int nr_l1stack_tasks;
extern unsigned long l1_stack_base, l1_stack_len;

static inline void enter_lazy_tlb(struct mm_struct *mm, struct task_struct *tsk)
{
}

/* Called when creating a new context during fork() or execve().  */
static inline int
init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
	return 0;
}

static inline void free_l1stack(void)
{
	nr_l1stack_tasks--;
	if (nr_l1stack_tasks == 0)
		l1sram_free (l1_stack_base);
}
static inline void destroy_context(struct mm_struct *mm)
{
	if (current_l1_stack_save == mm->context.l1_stack_save)
		current_l1_stack_save = 0;
	if (mm->context.l1_stack_save)
		free_l1stack();
}

static inline unsigned long
alloc_l1stack(unsigned long length, unsigned long *stack_base)
{
	if (nr_l1stack_tasks == 0) {
		l1_stack_base = l1sram_alloc_max (&l1_stack_len);
		if (!l1_stack_base)
			return 0;
	}

	if (l1_stack_len < length) {
		if (nr_l1stack_tasks == 0)
			l1sram_free (l1_stack_base);
		return 0;
	}
	*stack_base = l1_stack_base;
	nr_l1stack_tasks++;
	return l1_stack_len;
}

static inline int
activate_l1stack(struct mm_struct *mm, unsigned long sp_base)
{
	if (current_l1_stack_save)
		memcpy (current_l1_stack_save, (void *)l1_stack_base, l1_stack_len);
	mm->context.l1_stack_save = current_l1_stack_save = (void *)sp_base;
	memcpy ((void *)l1_stack_base, current_l1_stack_save, l1_stack_len);
	return 1;
}

#define deactivate_mm(tsk,mm)	do { } while (0)

static inline void activate_mm(struct mm_struct *prev_mm,
			       struct mm_struct *next_mm)
{
	if (!next_mm->context.l1_stack_save)
		return;
	if (next_mm->context.l1_stack_save == current_l1_stack_save)
		return;
	if (current_l1_stack_save) {
		memcpy (current_l1_stack_save, (void *)l1_stack_base, l1_stack_len);
	}
	current_l1_stack_save = next_mm->context.l1_stack_save;
	memcpy ((void *)l1_stack_base, current_l1_stack_save, l1_stack_len);
}

static noinline void switch_mm(struct mm_struct *prev, struct mm_struct *next,
			     struct task_struct *tsk)
{
	activate_mm (prev, next);
}


#endif
