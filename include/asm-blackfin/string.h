#ifndef _BFINNOMMU_STRING_H_
#define _BFINNOMMU_STRING_H_

#ifdef __KERNEL__ /* only set these up for kernel code */

#include <asm/setup.h>
#include <asm/page.h>

#define __HAVE_ARCH_STRCPY
static inline char * strcpy(char * dest,const char *src)
{
  char *xdest = dest;
  char  temp = 0;

  __asm__ __volatile__
       ("1:\t%2 = B [%1++] (Z);\n\t"
	"B [%0++] = %2;\n\t"
	"CC = %2;\n\t"
        "if cc jump 1b (bp);\n"
	: "+&a" (dest), "+&a" (src), "=&d" (temp)
        : : "memory", "CC");
  return xdest;
}

#define __HAVE_ARCH_STRNCPY
static inline char * strncpy(char *dest, const char *src, size_t n)
{
  char *xdest = dest;
  char  temp = 0;

  if (n == 0)
    return xdest;

  __asm__ __volatile__
       ("1:\t%3 = B [%1++] (Z);\n\t"
	"B [%0++] = %3;\n\t"
	"CC = %3;\n\t"
	"if ! cc jump 2f;\n\t"
        "%2 += -1;\n\t"
	"CC = %2 == 0;\n\t"
        "if ! cc jump 1b (bp);\n"
        "2:\n"
	: "+&a" (dest), "+&a" (src), "+&da" (n), "=&d" (temp)
        : : "memory", "CC");
  return xdest;
}

#define __HAVE_ARCH_STRCMP
static inline int strcmp(const char * cs,const char * ct)
{
  char __res1, __res2;

  __asm__
       ("1:\t%2 = B[%0++] (Z);\n\t" /* get *cs */
	"%3 = B[%1++] (Z);\n\t"	/* get *ct */
        "CC = %2 == %3;\n\t"       /* compare a byte */
        "if ! cc jump 2f;\n\t"         /* not equal, break out */
        "CC = %2;\n\t"           /* at end of cs? */
        "if cc jump 1b (bp);\n\t"           /* no, keep going */
        "jump.s 3f;\n"		/* strings are equal */
        "2:\t%2 = %2 - %3;\n"  /* *cs - *ct */
        "3:\n"
	: "+&a" (cs), "+&a" (ct), "=&d" (__res1), "=&d" (__res2)
        : : "CC");

  return __res1;
}

#define __HAVE_ARCH_STRNCMP
static inline int strncmp(const char * cs,const char * ct,size_t count)
{
  char __res1, __res2;

  if (!count)
    return 0;
  __asm__
       ("1:\t%3 = B[%0++] (Z);\n\t"        /* get *cs */
	"%4 = B[%1++] (Z);\n\t"		/* get *ct */
        "CC = %3 == %4;\n\t"            /* compare a byte */
        "if ! cc jump 3f;\n\t"                  /* not equal, break out */
        "CC = %3;\n\t"                  /* at end of cs? */
        "if ! cc jump 4f;\n\t"                 /* yes, all done */
        "%2 += -1;\n\t"              /* no, adjust count */
	"CC = %2 == 0;\n\t"
        "if ! cc jump 1b;\n"                 /* more to do, keep going */
        "2:\t%3 = 0;\n\t"           /* strings are equal */
        "jump.s    4f;\n"
        "3:\t%3 = %3 - %4;\n"          /* *cs - *ct */
        "4:"
	: "+&a" (cs), "+&a" (ct), "+&da" (count), "=&d" (__res1), "=&d" (__res2)
        : : "CC");
  return __res1;
}

#define __HAVE_ARCH_MEMSET
extern void * memset(void * s, int c, size_t count);

#define __HAVE_ARCH_MEMCPY
extern void * memcpy(void *d, const void *s, size_t count);

#define __HAVE_ARCH_MEMCMP
extern int memcmp(const void *,const void *,__kernel_size_t);

#endif /*__KERNEL__*/
#endif /* _BFIN_STRING_H_ */
