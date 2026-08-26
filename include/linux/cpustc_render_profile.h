/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_CPUSTC_RENDER_PROFILE_H
#define _LINUX_CPUSTC_RENDER_PROFILE_H

#include <linux/types.h>

struct device;

enum cpustc_render_bo_stage {
	CPUSTC_RENDER_BO_CMA_ALLOC = 1,
	CPUSTC_RENDER_BO_PAGE_ALLOC_FALLBACK,
	CPUSTC_RENDER_BO_ZERO,
	CPUSTC_RENDER_BO_CACHE_SYNC,
	CPUSTC_RENDER_BO_SET_UNCACHED,
	CPUSTC_RENDER_BO_DMA_ALLOC_TOTAL,
	CPUSTC_RENDER_BO_GEM_OBJECT,
	CPUSTC_RENDER_BO_GEM_DMA_ALLOC,
	CPUSTC_RENDER_BO_GEM_HANDLE,
	CPUSTC_RENDER_BO_DUMB_TOTAL,
};

void cpustc_render_profile_record(struct device *dev, unsigned int stage,
				  u64 bo_id, u64 size, u64 duration_ns,
				  u32 handle, int ret);

#endif /* _LINUX_CPUSTC_RENDER_PROFILE_H */
