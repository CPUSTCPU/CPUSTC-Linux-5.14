/* SPDX-License-Identifier: MIT */
#ifndef __UAPI_CPUSTC_DRM_H__
#define __UAPI_CPUSTC_DRM_H__

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

enum drm_cpustc_2d_operation {
	DRM_CPUSTC_2D_FILL_RECT = 1,
	DRM_CPUSTC_2D_COPY_AREA = 2,
	DRM_CPUSTC_2D_IMAGE_BLIT1 = 3,
};

#define DRM_CPUSTC_2D_CAP_FILL_RECT	(1U << 0)
#define DRM_CPUSTC_2D_CAP_COPY_AREA	(1U << 1)
#define DRM_CPUSTC_2D_CAP_IMAGE_BLIT1	(1U << 2)
#define DRM_CPUSTC_2D_CAP_RGB565		(1U << 8)
#define DRM_CPUSTC_2D_CAP_OVERLAP_COPY	(1U << 9)
#define DRM_CPUSTC_2D_CAP_BATCH		(1U << 16)
#define DRM_CPUSTC_2D_BATCH_MAX_RECTS	32

/**
 * struct drm_cpustc_2d_caps - discover the implemented 2D interface
 */
struct drm_cpustc_2d_caps {
	__u32 version;
	__u32 capabilities;
	__u32 reserved[6];
};

/**
 * struct drm_cpustc_2d_submit - synchronous RGB565 2D operation
 *
 * Coordinates and dimensions are pixels. For IMAGE_BLIT1, src_x is a bit
 * offset and src_stride is measured in bytes. All other strides are bytes.
 */
struct drm_cpustc_2d_submit {
	__u32 operation;
	__u32 flags;
	__u32 src_handle;
	__u32 dst_handle;
	__u32 src_stride;
	__u32 dst_stride;
	__u16 src_x;
	__u16 src_y;
	__u16 dst_x;
	__u16 dst_y;
	__u16 width;
	__u16 height;
	__u16 foreground;
	__u16 background;
	__u32 reserved[4];
};

/**
 * struct drm_cpustc_2d_rect - one rectangle in a synchronous batch
 */
struct drm_cpustc_2d_rect {
	__u16 src_x;
	__u16 src_y;
	__u16 dst_x;
	__u16 dst_y;
	__u16 width;
	__u16 height;
	__u16 reserved[2];
};

/**
 * struct drm_cpustc_2d_batch - synchronous rectangles sharing one setup
 *
 * rectangles points to count drm_cpustc_2d_rect entries. completed reports
 * how many entries finished before a hardware error. A batch is bounded so a
 * single ioctl cannot monopolize the X server for an unbounded interval.
 */
struct drm_cpustc_2d_batch {
	__u32 operation;
	__u32 flags;
	__u32 src_handle;
	__u32 dst_handle;
	__u32 src_stride;
	__u32 dst_stride;
	__u16 foreground;
	__u16 background;
	__u32 count;
	__u32 completed;
	__u32 pad;
	__u64 rectangles;
	__u32 reserved[4];
};

#define DRM_CPUSTC_2D_SUBMIT 0x00
#define DRM_CPUSTC_2D_GET_CAPS 0x01
#define DRM_CPUSTC_2D_SUBMIT_BATCH 0x02
#define DRM_IOCTL_CPUSTC_2D_SUBMIT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_CPUSTC_2D_SUBMIT, \
		struct drm_cpustc_2d_submit)
#define DRM_IOCTL_CPUSTC_2D_GET_CAPS \
	DRM_IOR(DRM_COMMAND_BASE + DRM_CPUSTC_2D_GET_CAPS, \
		struct drm_cpustc_2d_caps)
#define DRM_IOCTL_CPUSTC_2D_SUBMIT_BATCH \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_CPUSTC_2D_SUBMIT_BATCH, \
		 struct drm_cpustc_2d_batch)

#if defined(__cplusplus)
}
#endif

#endif
