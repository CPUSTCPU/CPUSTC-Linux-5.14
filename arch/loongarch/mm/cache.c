// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 *
 * Derived from MIPS:
 * Copyright (C) 1994 - 2003, 06, 07 by Ralf Baechle (ralf@linux-mips.org)
 * Copyright (C) 2007 MIPS Technologies, Inc.
 */
#include <linux/export.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/kernel.h>
#include <linux/linkage.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/syscalls.h>

#include <asm/cacheflush.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/dma.h>
#include <asm/loongarchregs.h>
#include <asm/processor.h>
#include <asm/setup.h>

#ifdef BX_SOC
void local_flush_cache_all(void)
{  
   u32 num, tmp = 0; 
   for (num = 0; num < 256; num++) {
       cache_op(9, tmp); 
       cache_op(9, tmp + 0x1);

       tmp = tmp >> 4;
       tmp += 1; 
       tmp = tmp << 4;
   }
}
#endif

/* Cache operations. */
void local_flush_icache_range(unsigned long start, unsigned long end)
{
	/* CPUSTCore's IBAR invalidates L1I but does not clean dirty L1D lines. */
	if (start < end)
		blast_dcache_range(start, end);
#ifdef BX_SOC
	local_flush_cache_all();
#endif
	asm volatile ("\tdbar 0\n\tibar 0\n" : : : "memory");
}

void __update_cache(unsigned long address, pte_t pte)
{
	struct page *page;
	unsigned long pfn, addr;

	pfn = pte_pfn(pte);
	if (unlikely(!pfn_valid(pfn)))
		return;
	page = pfn_to_page(pfn);
	if (Page_dcache_dirty(page)) {
		if (PageHighMem(page))
			addr = (unsigned long)kmap_atomic(page);
		else
			addr = (unsigned long)page_address(page);

		if (PageHighMem(page))
			kunmap_atomic((void *)addr);

		ClearPageDcacheDirty(page);
	}
}

void cache_error_setup(void)
{
	extern char __weak except_vec_cex;
	set_merr_handler(0x0, &except_vec_cex, 0x80);
}

static unsigned long icache_size __read_mostly;
static unsigned long dcache_size __read_mostly;
static unsigned long scache_size __read_mostly;

static char *way_string[] = { NULL, "direct mapped", "2-way",
	"3-way", "4-way", "5-way", "6-way", "7-way", "8-way",
	"9-way", "10-way", "11-way", "12-way",
	"13-way", "14-way", "15-way", "16-way",
};

static const char *cache_way_string(unsigned int ways)
{
	if (ways < ARRAY_SIZE(way_string) && way_string[ways])
		return way_string[ways];

	return "unknown associativity";
}

static bool decode_cache_geometry(struct cache_desc *cache, unsigned int config,
				  const char *name, unsigned long *size)
{
	unsigned int ways, sets_order, line_order;

	/* CPUCFG17 through CPUCFG20 use the same geometry layout. */
	ways = ((config & CPUCFG17_L1I_WAYS_M) >> CPUCFG17_L1I_WAYS) + 1;
	sets_order = (config & CPUCFG17_L1I_SETS_M) >> CPUCFG17_L1I_SETS;
	line_order = (config & CPUCFG17_L1I_SIZE_M) >> CPUCFG17_L1I_SIZE;

	/* cache_desc stores these values in u8/u16 fields. */
	if (ways > 0xff || sets_order >= 16 || line_order >= 8) {
		pr_warn("Invalid %s cache geometry in CPUCFG: 0x%08x\n",
			name, config);
		return false;
	}

	cache->ways = ways;
	cache->sets = 1U << sets_order;
	cache->linesz = 1U << line_order;
	cache->waysize = cache->sets * cache->linesz;
	cache->flags = CACHE_PRESENT;
	*size = cache->waysize * cache->ways;

	return true;
}

#ifdef CONFIG_32BIT

/* DMA cache operations. */
void (*_dma_cache_wback_inv)(unsigned long start, unsigned long size);
void (*_dma_cache_wback)(unsigned long start, unsigned long size);
void (*_dma_cache_inv)(unsigned long start, unsigned long size);

static void la32_dma_cache_wback_inv(unsigned long addr, unsigned long size)
{
	/* Catch bad driver code */
	BUG_ON(size == 0);

	if (!cpu_dcache_line_size())
		return;

	/* CPUSTCore index operations use a non-standard way encoding. */
	blast_dcache_range(addr, addr + size);
}

static void la32_dma_cache_inv(unsigned long addr, unsigned long size)
{
	/* CPUSTCore has no separate L1 hit-invalidate operation. */
	la32_dma_cache_wback_inv(addr, size);
}

#endif

static void probe_pcache(void)
{
	struct cpuinfo_loongarch *c = &current_cpu_data;
	unsigned int config, presence;

	memset(&c->icache, 0, sizeof(c->icache));
	memset(&c->dcache, 0, sizeof(c->dcache));
	memset(&c->scache, 0, sizeof(c->scache));
	icache_size = 0;
	dcache_size = 0;
	scache_size = 0;

	presence = read_cpucfg(LOONGARCH_CPUCFG16);

	if (presence & CPUCFG16_L1_UNIFY) {
		if (presence & CPUCFG16_L1_IUPRE) {
			config = read_cpucfg(LOONGARCH_CPUCFG17);
			if (decode_cache_geometry(&c->dcache, config,
						  "primary unified", &dcache_size))
				c->dcache.flags |= CACHE_PRIVATE;
		}
	} else {
		if (presence & CPUCFG16_L1_IUPRE) {
			config = read_cpucfg(LOONGARCH_CPUCFG17);
			if (decode_cache_geometry(&c->icache, config,
						  "primary instruction", &icache_size))
				c->icache.flags |= CACHE_PRIVATE;
		}

		if (presence & CPUCFG16_L1_DPRE) {
			config = read_cpucfg(LOONGARCH_CPUCFG18);
			if (decode_cache_geometry(&c->dcache, config,
						  "primary data", &dcache_size))
				c->dcache.flags |= CACHE_PRIVATE;
		}
	}

	if ((presence & CPUCFG16_L2_IUPRE) &&
	    (presence & CPUCFG16_L2_IUUNIFY)) {
		config = read_cpucfg(LOONGARCH_CPUCFG19);
		if (decode_cache_geometry(&c->scache, config,
					  "secondary unified", &scache_size)) {
			if (presence & CPUCFG16_L2_IUPRIV)
				c->scache.flags |= CACHE_PRIVATE;
			if (presence & CPUCFG16_L2_IUINCL)
				c->scache.flags |= CACHE_INCLUSIVE;
		}
	} else if (presence & (CPUCFG16_L2_IUPRE | CPUCFG16_L2_DPRE)) {
		pr_warn("Unsupported split L2 cache described by CPUCFG16: 0x%08x\n",
			presence);
	}

	c->options |= LOONGARCH_CPU_PREFETCH;

	if (c->icache.waysize)
		pr_info("Primary instruction cache %ldkB, %s, %s, linesize %d bytes.\n",
			icache_size >> 10, cache_way_string(c->icache.ways),
			"VIPT", c->icache.linesz);

	if (c->dcache.waysize)
		pr_info("Primary %s cache %ldkB, %s, %s, %s, linesize %d bytes\n",
			(presence & CPUCFG16_L1_UNIFY) ? "unified" : "data",
			dcache_size >> 10, cache_way_string(c->dcache.ways),
			"VIPT", "no aliases", c->dcache.linesz);

	if (c->scache.waysize)
		pr_info("Secondary unified cache %ldkB, %s, linesize %d bytes\n",
			scache_size >> 10, cache_way_string(c->scache.ways),
			c->scache.linesz);
#ifdef CONFIG_32BIT
	_dma_cache_wback_inv = la32_dma_cache_wback_inv;
	_dma_cache_wback = la32_dma_cache_wback_inv;
	_dma_cache_inv = la32_dma_cache_inv;
#endif

}

void cpu_cache_init(void)
{
	probe_pcache();
	shm_align_mask = PAGE_SIZE - 1;
}
