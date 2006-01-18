/*
 * Memory MAP
 * Common header file for blackfin BF561 of processors.
 *
 *
 */


#ifndef _MEM_MAP_561_H_
#define _MEM_MAP_561_H_

#define COREMMR_BASE           0xFFE00000     // Core MMRs
#define SYSMMR_BASE            0xFFC00000     // System MMRs

/* Level 3 SDRAM Memory */
#define RAM_START		0x1000
#define RAM_LENGTH		(CONFIG_MEM_SIZE * 1024 * 1024)
#define RAM_END 		(CONFIG_MEM_SIZE * 1024 * 1024)

/* Async Memory Banks */
#define ASYNC_BANK3_BASE	0x2C000000	// Async Bank 3
#define ASYNC_BANK3_SIZE	0x04000000	/* 1M */
#define ASYNC_BANK2_BASE	0x28000000	// Async Bank 2
#define ASYNC_BANK2_SIZE	0x04000000  /* 1M */
#define ASYNC_BANK1_BASE	0x24000000	// Async Bank 1
#define ASYNC_BANK1_SIZE	0x04000000	/* 1M */
#define ASYNC_BANK0_BASE	0x20000000	// Async Bank 0
#define ASYNC_BANK0_SIZE	0x04000000	/* 1M */

/* Level 1 Memory */

/* Memory Map for ADSP-BF561 processors */

#ifdef CONFIG_BF561
#define L1_CODE_START     0xFFA00000
#define L1_DATA_A_START     0xFF800000
#define L1_DATA_B_START     0xFF900000

#define L1_CODE_LENGTH      0x4000

#ifdef CONFIG_BLKFIN_DCACHE
#define DMEM_CNTR (ACACHE_BCACHE | ENDCPLB | PORT_PREF0)
#define L1_DATA_A_LENGTH      (0x8000 - 0x4000)
#else
#define DMEM_CNTR (ASRAM_BSRAM | ENDCPLB | PORT_PREF0)
#define L1_DATA_A_LENGTH      0x8000
#endif

#ifdef CONFIG_BLKFIN_DCACHE
#define L1_DATA_B_LENGTH      (0x8000 - 0x4000)
#else
#define L1_DATA_B_LENGTH      0x8000
#endif
#endif

/* Level 2 Memory */
#define L2_START		0xFEB00000
#define L2_LENGTH		0x20000

/* Scratch Pad Memory */

#if defined(CONFIG_BF561)
#define L1_SCRATCH_START	0xFFB00000
#define L1_SCRATCH_LENGTH	0x1000
#endif

#endif /* _MEM_MAP_533_H_ */
