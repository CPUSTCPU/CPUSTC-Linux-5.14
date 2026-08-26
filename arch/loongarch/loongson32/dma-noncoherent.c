// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2000  Ani Joshi <ajoshi@unixbox.com>
 * Copyright (C) 2000, 2001, 06  Ralf Baechle <ralf@linux-mips.org>
 * Copyright (C) 2022, Loongson Technology Corporation Limited
 */
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/highmem.h>

#include <asm/barrier.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

static inline void dma_cache_l2_wback_inv(unsigned long addr, size_t size)
{
	if (!cpu_scache_line_size())
		return;

	blast_l2cache_range(addr, addr + size);
}

void *arch_dma_set_uncached(void *addr, size_t size)
{
	return (void *)(__pa(addr) + UNCAC_BASE);
}

void arch_dma_prep_coherent(struct page *page, size_t size)
{
	unsigned long addr = (unsigned long)page_address(page);

	dma_cache_wback_inv(addr, size);
	dma_cache_l2_wback_inv(addr, size);
	__sync();
}

static inline void dma_sync_virt_for_device(void *addr, size_t size,
		enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_TO_DEVICE:
		dma_cache_wback((unsigned long)addr, size);
		break;
	case DMA_FROM_DEVICE:
		dma_cache_inv((unsigned long)addr, size);
		break;
	case DMA_BIDIRECTIONAL:
		dma_cache_wback_inv((unsigned long)addr, size);
		break;
	default:
		BUG();
	}

	dma_cache_l2_wback_inv((unsigned long)addr, size);
}

static inline void dma_sync_virt_for_cpu(void *addr, size_t size,
		enum dma_data_direction dir)
{
	/*
	 * CPUSTCore has no data prefetcher, and streaming DMA ownership excludes
	 * cached CPU accesses until the buffer is returned to the CPU.  The
	 * device-side sync has already invalidated L1 and L2 and ordered the
	 * ownership handoff, so a DMA_FROM_DEVICE buffer remains invalid while
	 * the device owns it.
	 *
	 * dma_cache_inv() is implemented as writeback plus invalidate on this
	 * core.  Avoid that redundant post-DMA operation for DMA_FROM_DEVICE;
	 * retain the conservative handling for DMA_BIDIRECTIONAL mappings.
	 */
	switch (dir) {
	case DMA_TO_DEVICE:
	case DMA_FROM_DEVICE:
		break;
	case DMA_BIDIRECTIONAL:
		dma_cache_inv((unsigned long)addr, size);
		break;
	default:
		BUG();
	}
}

/*
 * A single sg entry may refer to multiple physically contiguous pages.  But
 * we still need to process highmem pages individually.  If highmem is not
 * configured then the bulk of this loop gets optimized out.
 */
static inline void dma_sync_phys(phys_addr_t paddr, size_t size,
		enum dma_data_direction dir, bool for_device)
{
	struct page *page = pfn_to_page(paddr >> PAGE_SHIFT);
	unsigned long offset = paddr & ~PAGE_MASK;
	size_t left = size;

	do {
		size_t len = left;
		void *addr;

		if (PageHighMem(page)) {
			if (offset + len > PAGE_SIZE)
				len = PAGE_SIZE - offset;
		}

		addr = kmap_atomic(page);
		if (for_device)
			dma_sync_virt_for_device(addr + offset, len, dir);
		else
			dma_sync_virt_for_cpu(addr + offset, len, dir);
		kunmap_atomic(addr);

		offset = 0;
		page++;
		left -= len;
	} while (left);
}

void arch_sync_dma_for_device(phys_addr_t paddr, size_t size,
		enum dma_data_direction dir)
{
	dma_sync_phys(paddr, size, dir, true);
	__sync();
}

#ifdef CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU
void arch_sync_dma_for_cpu(phys_addr_t paddr, size_t size,
		enum dma_data_direction dir)
{
	dma_sync_phys(paddr, size, dir, false);
	__sync();
}
#endif

#ifdef CONFIG_ARCH_HAS_SETUP_DMA_OPS
void arch_setup_dma_ops(struct device *dev, u64 dma_base, u64 size,
		const struct iommu_ops *iommu, bool coherent)
{
	dev->dma_coherent = coherent;
}
#endif
