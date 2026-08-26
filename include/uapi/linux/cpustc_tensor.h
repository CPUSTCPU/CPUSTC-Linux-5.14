/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_CPUSTC_TENSOR_H
#define _UAPI_LINUX_CPUSTC_TENSOR_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define CPUSTC_TENSOR_ABI_VERSION	3U
#define CPUSTC_TENSOR_WORKSPACE_BYTES	(256U * 1024U)
#define CPUSTC_TENSOR_WORKSPACE_ALIGNMENT	64U
#define CPUSTC_TENSOR_MAX_M		65535U
#define CPUSTC_TENSOR_MAX_N		65535U
#define CPUSTC_TENSOR_MAX_K		256U
#define CPUSTC_TENSOR_PACK_COLUMNS	4U

#define CPUSTC_TENSOR_CAP_NHWC_WINDOW	(1U << 0)
#define CPUSTC_TENSOR_CAP_PRELU		(1U << 1)
#define CPUSTC_TENSOR_CAP_POOL2X2_CEIL	(1U << 2)
#define CPUSTC_TENSOR_CAP_APPEND_ONE	(1U << 3)

#define CPUSTC_TENSOR_INPUT_MATRIX	0U
#define CPUSTC_TENSOR_INPUT_NHWC_WINDOW	1U
#define CPUSTC_TENSOR_INPUT_MASK		3U
#define CPUSTC_TENSOR_POST_NONE		(0U << 2)
#define CPUSTC_TENSOR_POST_PRELU		(1U << 2)
#define CPUSTC_TENSOR_POST_PRELU_POOL2X2_CEIL	(2U << 2)
#define CPUSTC_TENSOR_POST_MASK		(3U << 2)
#define CPUSTC_TENSOR_APPEND_ONE		(1U << 4)
#define CPUSTC_TENSOR_MODE_MASK		0x1fU

#define CPUSTC_TENSOR_TRANSFER_TO_DEVICE	0U
#define CPUSTC_TENSOR_TRANSFER_FROM_DEVICE	1U

struct cpustc_tensor_info {
	__u32 abi_version;
	__u32 workspace_bytes;
	__u32 max_m;
	__u32 max_n;
	__u32 max_k;
	__u32 pack_columns;
	__u32 core_columns;
	__u32 tile_columns;
	__u32 capabilities;
	__u32 reserved[7];
};

/*
 * Transfers are the only operations that move workspace data through the CPU.
 * Intermediate RUN results remain in the workspace and can be used as the A
 * input of a later RUN by passing the same byte offset.
 */
struct cpustc_tensor_transfer {
	__u32 direction;
	__u32 offset;
	__u32 length;
	__u32 flags;
	__aligned_u64 data;
	__u32 reserved[4];
};

/*
 * A and C are row-major FP32. B is packed in groups of four output columns:
 *
 *   packed[((column / 4 * k + inner) * 4) + column % 4]
 *
 * Lanes beyond n in the last group must be present and zero-filled. All
 * addresses below are 64-byte-aligned byte offsets inside the driver-owned
 * DMA workspace.
 */
struct cpustc_tensor_run {
	__u32 mode;
	__u32 m;
	__u32 n;
	__u32 k;
	__u32 round_mode;
	__u32 a_offset;
	__u32 b_offset;
	__u32 c_offset;
	__u32 a_stride;
	__u32 c_stride;
	__u32 input_height;
	__u32 input_width;
	__u32 input_channels;
	__u32 output_height;
	__u32 output_width;
	__u32 kernel_height;
	__u32 kernel_width;
	__u32 stride_y;
	__u32 stride_x;
	__u32 pad_top;
	__u32 pad_left;
	__u32 prelu_offset;
	__u32 total_cycles;
	__u32 b_read_cycles;
	__u32 a_read_cycles;
	__u32 compute_cycles;
	__u32 c_write_cycles;
	__u32 window_cycles;
	__u32 post_cycles;
	__u32 error_code;
	__u32 reserved[4];
};

#define CPUSTC_TENSOR_IOC_GET_INFO \
	_IOR('T', 0x00, struct cpustc_tensor_info)
#define CPUSTC_TENSOR_IOC_TRANSFER \
	_IOW('T', 0x01, struct cpustc_tensor_transfer)
#define CPUSTC_TENSOR_IOC_RUN \
	_IOWR('T', 0x02, struct cpustc_tensor_run)

#endif /* _UAPI_LINUX_CPUSTC_TENSOR_H */
