// SPDX-License-Identifier: GPL-2.0-only
/* CPUSTC TensorCore workspace and GEMM misc device driver. */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/cpustc_tensor.h>

#define CPUSTC_TENSOR_STATUS		0x00
#define CPUSTC_TENSOR_CONTROL		0x04
#define CPUSTC_TENSOR_A_BASE		0x08
#define CPUSTC_TENSOR_B_BASE		0x0c
#define CPUSTC_TENSOR_C_BASE		0x10
#define CPUSTC_TENSOR_M			0x14
#define CPUSTC_TENSOR_N			0x18
#define CPUSTC_TENSOR_K			0x1c
#define CPUSTC_TENSOR_A_STRIDE		0x20
#define CPUSTC_TENSOR_C_STRIDE		0x24
#define CPUSTC_TENSOR_ROUND_MODE	0x28
#define CPUSTC_TENSOR_IRQ_ENABLE	0x2c
#define CPUSTC_TENSOR_IRQ_STATUS	0x30
#define CPUSTC_TENSOR_ERROR_CODE	0x34
#define CPUSTC_TENSOR_TOTAL_CYCLES	0x38
#define CPUSTC_TENSOR_B_READ_CYCLES	0x3c
#define CPUSTC_TENSOR_A_READ_CYCLES	0x40
#define CPUSTC_TENSOR_COMPUTE_CYCLES	0x44
#define CPUSTC_TENSOR_C_WRITE_CYCLES	0x48
#define CPUSTC_TENSOR_IDENTIFICATION	0x50
#define CPUSTC_TENSOR_CAPABILITIES0	0x54
#define CPUSTC_TENSOR_CAPABILITIES1	0x58
#define CPUSTC_TENSOR_VERSION		0x5c
#define CPUSTC_TENSOR_MODE		0x60
#define CPUSTC_TENSOR_INPUT_HEIGHT	0x64
#define CPUSTC_TENSOR_INPUT_WIDTH	0x68
#define CPUSTC_TENSOR_INPUT_CHANNELS	0x6c
#define CPUSTC_TENSOR_OUTPUT_HEIGHT	0x70
#define CPUSTC_TENSOR_OUTPUT_WIDTH	0x74
#define CPUSTC_TENSOR_KERNEL_HEIGHT	0x78
#define CPUSTC_TENSOR_KERNEL_WIDTH	0x7c
#define CPUSTC_TENSOR_STRIDE_Y		0x80
#define CPUSTC_TENSOR_STRIDE_X		0x84
#define CPUSTC_TENSOR_PAD_TOP		0x88
#define CPUSTC_TENSOR_PAD_LEFT		0x8c
#define CPUSTC_TENSOR_PRELU_BASE		0x90
#define CPUSTC_TENSOR_WINDOW_CYCLES	0x94
#define CPUSTC_TENSOR_POST_CYCLES	0x98
#define CPUSTC_TENSOR_CAPABILITIES2	0x9c
#define CPUSTC_TENSOR_SOURCE_EXTENT_BYTES	0xa0
#define CPUSTC_TENSOR_SOURCE_ROW_BYTES	0xa4
#define CPUSTC_TENSOR_SOURCE_PIXEL_BYTES	0xa8
#define CPUSTC_TENSOR_SOURCE_STEP_Y_BYTES	0xac
#define CPUSTC_TENSOR_SOURCE_STEP_X_BYTES	0xb0
#define CPUSTC_TENSOR_SOURCE_PAD_TOP_BYTES	0xb4
#define CPUSTC_TENSOR_SOURCE_PAD_LEFT_BYTES	0xb8
#define CPUSTC_TENSOR_RESULT_ROWS		0xbc

#define CPUSTC_TENSOR_STATUS_BUSY	BIT(0)
#define CPUSTC_TENSOR_STATUS_DONE	BIT(1)
#define CPUSTC_TENSOR_STATUS_ERROR	BIT(2)
#define CPUSTC_TENSOR_CONTROL_START	BIT(0)
#define CPUSTC_TENSOR_IRQ_PENDING	BIT(0)

#define CPUSTC_TENSOR_IDENTIFICATION_VALUE	0x54434734
#define CPUSTC_TENSOR_REGISTER_VERSION		4U
#define CPUSTC_TENSOR_CORE_COLUMNS		4
#define CPUSTC_TENSOR_TILE_COLUMNS		32
#define CPUSTC_TENSOR_B_BRAM_BYTES		32768
#define CPUSTC_TENSOR_TIMEOUT_MS		5000U

struct cpustc_tensor {
	struct kref ref;
	struct mutex lock; /* Serializes ioctls and removal. */
	struct completion completion;
	struct device *dev;
	void __iomem *base;
	struct miscdevice miscdev;
	void *dma_buffer;
	dma_addr_t dma_handle;
	int irq;
	bool opened;
	bool removed;
};

struct cpustc_tensor_derived_run {
	u32 source_extent_bytes;
	u32 source_row_bytes;
	u32 source_pixel_bytes;
	u32 source_step_y_bytes;
	u32 source_step_x_bytes;
	u32 source_pad_top_bytes;
	u32 source_pad_left_bytes;
	u32 result_rows;
};

static void cpustc_tensor_release_ref(struct kref *ref)
{
	struct cpustc_tensor *tensor =
		container_of(ref, struct cpustc_tensor, ref);

	kfree(tensor);
}

static bool cpustc_tensor_range_valid(u32 offset, u64 length)
{
	return !(offset & (CPUSTC_TENSOR_WORKSPACE_ALIGNMENT - 1)) && length &&
		length <= CPUSTC_TENSOR_WORKSPACE_BYTES &&
		offset <= CPUSTC_TENSOR_WORKSPACE_BYTES - length;
}

static bool cpustc_tensor_ranges_overlap(u32 first_offset, u64 first_length,
					 u32 second_offset, u64 second_length)
{
	return first_offset < second_offset + second_length &&
		second_offset < first_offset + first_length;
}

static int cpustc_tensor_validate_transfer(const struct cpustc_tensor_transfer *transfer)
{
	if (transfer->direction > CPUSTC_TENSOR_TRANSFER_FROM_DEVICE ||
	    transfer->flags || !transfer->data ||
	    transfer->reserved[0] || transfer->reserved[1] ||
	    transfer->reserved[2] || transfer->reserved[3] ||
	    !cpustc_tensor_range_valid(transfer->offset, transfer->length))
		return -EINVAL;

	return 0;
}

static int cpustc_tensor_derive_run(const struct cpustc_tensor_run *run,
				    struct cpustc_tensor_derived_run *derived)
{
	u32 input_mode = run->mode & CPUSTC_TENSOR_INPUT_MASK;
	u32 post_mode = run->mode & CPUSTC_TENSOR_POST_MASK;
	u64 source_extent_bytes = 0;
	u64 source_row_bytes = 0;
	u64 source_pixel_bytes = 0;
	u64 source_step_y_bytes = 0;
	u64 source_step_x_bytes = 0;
	u64 source_pad_top_bytes = 0;
	u64 source_pad_left_bytes = 0;
	u64 result_rows = run->m;

	if (post_mode == CPUSTC_TENSOR_POST_PRELU_POOL2X2_CEIL)
		result_rows = (u64)DIV_ROUND_UP(run->output_height, 2) *
			DIV_ROUND_UP(run->output_width, 2);

	if (input_mode == CPUSTC_TENSOR_INPUT_NHWC_WINDOW) {
		source_pixel_bytes = (u64)run->input_channels * sizeof(u32);
		source_row_bytes = (u64)run->input_width * source_pixel_bytes;
		source_extent_bytes = (u64)run->input_height * source_row_bytes;
		source_step_y_bytes = (u64)run->stride_y * source_row_bytes;
		source_step_x_bytes = (u64)run->stride_x * source_pixel_bytes;
		source_pad_top_bytes = (u64)run->pad_top * source_row_bytes;
		source_pad_left_bytes = (u64)run->pad_left * source_pixel_bytes;
	} else if (post_mode == CPUSTC_TENSOR_POST_PRELU_POOL2X2_CEIL) {
		source_pixel_bytes = run->a_stride;
		source_row_bytes = (u64)run->output_width * run->a_stride;
		source_extent_bytes = (u64)(run->m - 1) * run->a_stride +
			run->k * sizeof(u32);
		source_step_y_bytes = source_row_bytes;
		source_step_x_bytes = run->a_stride;
	}

	if (source_extent_bytes > U32_MAX || source_row_bytes > U32_MAX ||
	    source_pixel_bytes > U32_MAX || source_step_y_bytes > U32_MAX ||
	    source_step_x_bytes > U32_MAX || source_pad_top_bytes > U32_MAX ||
	    source_pad_left_bytes > U32_MAX || result_rows > U32_MAX)
		return -EOVERFLOW;

	derived->source_extent_bytes = source_extent_bytes;
	derived->source_row_bytes = source_row_bytes;
	derived->source_pixel_bytes = source_pixel_bytes;
	derived->source_step_y_bytes = source_step_y_bytes;
	derived->source_step_x_bytes = source_step_x_bytes;
	derived->source_pad_top_bytes = source_pad_top_bytes;
	derived->source_pad_left_bytes = source_pad_left_bytes;
	derived->result_rows = result_rows;
	return 0;
}

static int cpustc_tensor_validate_run(struct cpustc_tensor_run *run,
				      struct cpustc_tensor_derived_run *derived)
{
	u32 input_mode = run->mode & CPUSTC_TENSOR_INPUT_MASK;
	u32 post_mode = run->mode & CPUSTC_TENSOR_POST_MASK;
	u64 input_elements;
	u64 output_rows;
	u64 a_bytes;
	u64 b_bytes;
	u64 c_bytes;
	u64 prelu_bytes = 0;

	if (run->mode & ~CPUSTC_TENSOR_MODE_MASK ||
	    input_mode > CPUSTC_TENSOR_INPUT_NHWC_WINDOW ||
	    post_mode > CPUSTC_TENSOR_POST_PRELU_POOL2X2_CEIL ||
	    !run->m || run->m > CPUSTC_TENSOR_MAX_M ||
	    !run->n || run->n > CPUSTC_TENSOR_MAX_N ||
	    !run->k || run->k > CPUSTC_TENSOR_MAX_K ||
	    run->round_mode > 4 ||
	    (run->a_stride & 3) || (run->c_stride & 3) ||
	    run->c_stride < run->n * sizeof(u32) ||
	    run->reserved[0] || run->reserved[1] ||
	    run->reserved[2] || run->reserved[3])
		return -EINVAL;

	b_bytes = (u64)DIV_ROUND_UP(run->n, CPUSTC_TENSOR_PACK_COLUMNS) *
		run->k * CPUSTC_TENSOR_PACK_COLUMNS * sizeof(u32);
	if (input_mode == CPUSTC_TENSOR_INPUT_MATRIX) {
		if ((run->mode & CPUSTC_TENSOR_APPEND_ONE) ||
		    run->a_stride < run->k * sizeof(u32))
			return -EINVAL;
		a_bytes = (u64)(run->m - 1) * run->a_stride +
			run->k * sizeof(u32);
	} else {
		if (!run->input_height || !run->input_width ||
		    !run->input_channels || !run->output_height ||
		    !run->output_width || !run->kernel_height ||
		    !run->kernel_width || !run->stride_y || !run->stride_x ||
		    run->input_height > CPUSTC_TENSOR_MAX_M ||
		    run->input_width > CPUSTC_TENSOR_MAX_M ||
		    run->input_channels > CPUSTC_TENSOR_MAX_K ||
		    run->output_height > CPUSTC_TENSOR_MAX_M ||
		    run->output_width > CPUSTC_TENSOR_MAX_M ||
		    run->kernel_height > CPUSTC_TENSOR_MAX_K ||
		    run->kernel_width > CPUSTC_TENSOR_MAX_K)
			return -EINVAL;
		input_elements = (u64)run->input_height * run->input_width *
			run->input_channels;
		if ((u64)run->output_height * run->output_width != run->m ||
		    (u64)run->kernel_height * run->kernel_width *
			run->input_channels +
			!!(run->mode & CPUSTC_TENSOR_APPEND_ONE) != run->k)
			return -EINVAL;
		a_bytes = input_elements * sizeof(u32);
	}

	if (post_mode == CPUSTC_TENSOR_POST_PRELU_POOL2X2_CEIL) {
		if (!run->output_height || !run->output_width ||
		    (u64)run->output_height * run->output_width != run->m)
			return -EINVAL;
		output_rows = (u64)DIV_ROUND_UP(run->output_height, 2) *
			DIV_ROUND_UP(run->output_width, 2);
	} else {
		output_rows = run->m;
	}
	c_bytes = (output_rows - 1) * run->c_stride +
		run->n * sizeof(u32);
	if (post_mode != CPUSTC_TENSOR_POST_NONE)
		prelu_bytes = (u64)run->n * sizeof(u32);

	if (!cpustc_tensor_range_valid(run->a_offset, a_bytes) ||
	    !cpustc_tensor_range_valid(run->b_offset, b_bytes) ||
	    !cpustc_tensor_range_valid(run->c_offset, c_bytes) ||
	    (prelu_bytes &&
	     !cpustc_tensor_range_valid(run->prelu_offset, prelu_bytes)))
		return -E2BIG;
	if (cpustc_tensor_ranges_overlap(run->c_offset, c_bytes,
					 run->a_offset, a_bytes) ||
	    cpustc_tensor_ranges_overlap(run->c_offset, c_bytes,
					 run->b_offset, b_bytes) ||
	    (prelu_bytes && cpustc_tensor_ranges_overlap(run->c_offset, c_bytes,
							 run->prelu_offset,
							 prelu_bytes)))
		return -EINVAL;

	return cpustc_tensor_derive_run(run, derived);
}

static irqreturn_t cpustc_tensor_irq(int irq, void *data)
{
	struct cpustc_tensor *tensor = data;
	u32 pending;

	pending = readl(tensor->base + CPUSTC_TENSOR_IRQ_STATUS);
	if (!(pending & CPUSTC_TENSOR_IRQ_PENDING))
		return IRQ_NONE;

	writel(CPUSTC_TENSOR_IRQ_PENDING,
	       tensor->base + CPUSTC_TENSOR_IRQ_STATUS);
	complete(&tensor->completion);
	return IRQ_HANDLED;
}

static int cpustc_tensor_run_hardware(struct cpustc_tensor *tensor,
				      struct cpustc_tensor_run *run,
				      const struct cpustc_tensor_derived_run *derived)
{
	dma_addr_t a_dma = tensor->dma_handle + run->a_offset;
	dma_addr_t b_dma = tensor->dma_handle + run->b_offset;
	dma_addr_t c_dma = tensor->dma_handle + run->c_offset;
	dma_addr_t prelu_dma = tensor->dma_handle + run->prelu_offset;
	unsigned long completed;
	unsigned long timeout = msecs_to_jiffies(CPUSTC_TENSOR_TIMEOUT_MS);
	u32 status;

	status = readl(tensor->base + CPUSTC_TENSOR_STATUS);
	if (status & CPUSTC_TENSOR_STATUS_BUSY)
		return -EBUSY;

	reinit_completion(&tensor->completion);
	writel(CPUSTC_TENSOR_STATUS_DONE | CPUSTC_TENSOR_STATUS_ERROR,
	       tensor->base + CPUSTC_TENSOR_STATUS);
	writel(CPUSTC_TENSOR_IRQ_PENDING,
	       tensor->base + CPUSTC_TENSOR_IRQ_STATUS);
	writel(lower_32_bits(a_dma), tensor->base + CPUSTC_TENSOR_A_BASE);
	writel(lower_32_bits(b_dma), tensor->base + CPUSTC_TENSOR_B_BASE);
	writel(lower_32_bits(c_dma), tensor->base + CPUSTC_TENSOR_C_BASE);
	writel(run->m, tensor->base + CPUSTC_TENSOR_M);
	writel(run->n, tensor->base + CPUSTC_TENSOR_N);
	writel(run->k, tensor->base + CPUSTC_TENSOR_K);
	writel(run->a_stride, tensor->base + CPUSTC_TENSOR_A_STRIDE);
	writel(run->c_stride, tensor->base + CPUSTC_TENSOR_C_STRIDE);
	writel(run->round_mode, tensor->base + CPUSTC_TENSOR_ROUND_MODE);
	writel(run->mode, tensor->base + CPUSTC_TENSOR_MODE);
	writel(run->input_height, tensor->base + CPUSTC_TENSOR_INPUT_HEIGHT);
	writel(run->input_width, tensor->base + CPUSTC_TENSOR_INPUT_WIDTH);
	writel(run->input_channels, tensor->base + CPUSTC_TENSOR_INPUT_CHANNELS);
	writel(run->output_height, tensor->base + CPUSTC_TENSOR_OUTPUT_HEIGHT);
	writel(run->output_width, tensor->base + CPUSTC_TENSOR_OUTPUT_WIDTH);
	writel(run->kernel_height, tensor->base + CPUSTC_TENSOR_KERNEL_HEIGHT);
	writel(run->kernel_width, tensor->base + CPUSTC_TENSOR_KERNEL_WIDTH);
	writel(run->stride_y, tensor->base + CPUSTC_TENSOR_STRIDE_Y);
	writel(run->stride_x, tensor->base + CPUSTC_TENSOR_STRIDE_X);
	writel(run->pad_top, tensor->base + CPUSTC_TENSOR_PAD_TOP);
	writel(run->pad_left, tensor->base + CPUSTC_TENSOR_PAD_LEFT);
	writel(lower_32_bits(prelu_dma),
	       tensor->base + CPUSTC_TENSOR_PRELU_BASE);
	writel(derived->source_extent_bytes,
	       tensor->base + CPUSTC_TENSOR_SOURCE_EXTENT_BYTES);
	writel(derived->source_row_bytes,
	       tensor->base + CPUSTC_TENSOR_SOURCE_ROW_BYTES);
	writel(derived->source_pixel_bytes,
	       tensor->base + CPUSTC_TENSOR_SOURCE_PIXEL_BYTES);
	writel(derived->source_step_y_bytes,
	       tensor->base + CPUSTC_TENSOR_SOURCE_STEP_Y_BYTES);
	writel(derived->source_step_x_bytes,
	       tensor->base + CPUSTC_TENSOR_SOURCE_STEP_X_BYTES);
	writel(derived->source_pad_top_bytes,
	       tensor->base + CPUSTC_TENSOR_SOURCE_PAD_TOP_BYTES);
	writel(derived->source_pad_left_bytes,
	       tensor->base + CPUSTC_TENSOR_SOURCE_PAD_LEFT_BYTES);
	writel(derived->result_rows, tensor->base + CPUSTC_TENSOR_RESULT_ROWS);
	writel(1, tensor->base + CPUSTC_TENSOR_IRQ_ENABLE);
	dma_wmb();
	writel(CPUSTC_TENSOR_CONTROL_START,
	       tensor->base + CPUSTC_TENSOR_CONTROL);

	completed = wait_for_completion_timeout(&tensor->completion, timeout);
	synchronize_irq(tensor->irq);
	status = readl(tensor->base + CPUSTC_TENSOR_STATUS);
	if (!(status & CPUSTC_TENSOR_STATUS_BUSY))
		writel(0, tensor->base + CPUSTC_TENSOR_IRQ_ENABLE);

	run->error_code = readl(tensor->base + CPUSTC_TENSOR_ERROR_CODE);
	run->total_cycles = readl(tensor->base + CPUSTC_TENSOR_TOTAL_CYCLES);
	run->b_read_cycles = readl(tensor->base + CPUSTC_TENSOR_B_READ_CYCLES);
	run->a_read_cycles = readl(tensor->base + CPUSTC_TENSOR_A_READ_CYCLES);
	run->compute_cycles = readl(tensor->base + CPUSTC_TENSOR_COMPUTE_CYCLES);
	run->c_write_cycles = readl(tensor->base + CPUSTC_TENSOR_C_WRITE_CYCLES);
	run->window_cycles = readl(tensor->base + CPUSTC_TENSOR_WINDOW_CYCLES);
	run->post_cycles = readl(tensor->base + CPUSTC_TENSOR_POST_CYCLES);

	if (status & (CPUSTC_TENSOR_STATUS_DONE | CPUSTC_TENSOR_STATUS_ERROR)) {
		writel(status & (CPUSTC_TENSOR_STATUS_DONE |
				  CPUSTC_TENSOR_STATUS_ERROR),
		       tensor->base + CPUSTC_TENSOR_STATUS);
		writel(CPUSTC_TENSOR_IRQ_PENDING,
		       tensor->base + CPUSTC_TENSOR_IRQ_STATUS);
	}
	if (status & CPUSTC_TENSOR_STATUS_ERROR)
		return -EIO;
	if (status & CPUSTC_TENSOR_STATUS_DONE) {
		dma_rmb();
		return 0;
	}
	if (!completed)
		return -ETIMEDOUT;

	return -EIO;
}

static int cpustc_tensor_transfer(struct cpustc_tensor *tensor,
				  const struct cpustc_tensor_transfer *transfer)
{
	u8 *buffer = tensor->dma_buffer;
	void __user *data = u64_to_user_ptr(transfer->data);
	int error = 0;

	if (readl(tensor->base + CPUSTC_TENSOR_STATUS) &
	    CPUSTC_TENSOR_STATUS_BUSY)
		return -EBUSY;

	dma_sync_single_range_for_cpu(tensor->dev, tensor->dma_handle,
				      transfer->offset, transfer->length,
				      DMA_BIDIRECTIONAL);
	if (transfer->direction == CPUSTC_TENSOR_TRANSFER_TO_DEVICE) {
		if (copy_from_user(buffer + transfer->offset, data,
				   transfer->length))
			error = -EFAULT;
	} else if (copy_to_user(data, buffer + transfer->offset,
				transfer->length)) {
		error = -EFAULT;
	}
	dma_sync_single_range_for_device(tensor->dev, tensor->dma_handle,
					 transfer->offset, transfer->length,
					 DMA_BIDIRECTIONAL);
	return error;
}

static int cpustc_tensor_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct cpustc_tensor *tensor =
		container_of(miscdev, struct cpustc_tensor, miscdev);
	int error;

	error = mutex_lock_interruptible(&tensor->lock);
	if (error)
		return error;
	if (tensor->removed) {
		error = -ENODEV;
	} else if (tensor->opened) {
		error = -EBUSY;
	} else {
		tensor->opened = true;
		kref_get(&tensor->ref);
		file->private_data = tensor;
	}
	mutex_unlock(&tensor->lock);

	return error;
}

static int cpustc_tensor_release(struct inode *inode, struct file *file)
{
	struct cpustc_tensor *tensor = file->private_data;

	mutex_lock(&tensor->lock);
	tensor->opened = false;
	mutex_unlock(&tensor->lock);
	kref_put(&tensor->ref, cpustc_tensor_release_ref);
	return 0;
}

static long cpustc_tensor_ioctl(struct file *file, unsigned int command,
				unsigned long argument)
{
	struct cpustc_tensor *tensor = file->private_data;
	void __user *user_data = (void __user *)argument;
	struct cpustc_tensor_info info = {
		.abi_version = CPUSTC_TENSOR_ABI_VERSION,
		.workspace_bytes = CPUSTC_TENSOR_WORKSPACE_BYTES,
		.max_m = CPUSTC_TENSOR_MAX_M,
		.max_n = CPUSTC_TENSOR_MAX_N,
		.max_k = CPUSTC_TENSOR_MAX_K,
		.pack_columns = CPUSTC_TENSOR_PACK_COLUMNS,
		.core_columns = CPUSTC_TENSOR_CORE_COLUMNS,
		.tile_columns = CPUSTC_TENSOR_TILE_COLUMNS,
		.capabilities = CPUSTC_TENSOR_CAP_NHWC_WINDOW |
			CPUSTC_TENSOR_CAP_PRELU |
			CPUSTC_TENSOR_CAP_POOL2X2_CEIL |
			CPUSTC_TENSOR_CAP_APPEND_ONE,
	};
	struct cpustc_tensor_transfer transfer;
	struct cpustc_tensor_run run;
	struct cpustc_tensor_derived_run derived;
	bool report_run = false;
	int error;

	if (command == CPUSTC_TENSOR_IOC_GET_INFO) {
		error = mutex_lock_interruptible(&tensor->lock);
		if (error)
			return error;
		error = tensor->removed ? -ENODEV : 0;
		mutex_unlock(&tensor->lock);
		if (error)
			return error;
		return copy_to_user(user_data, &info, sizeof(info)) ? -EFAULT : 0;
	}
	if (command == CPUSTC_TENSOR_IOC_TRANSFER) {
		if (copy_from_user(&transfer, user_data, sizeof(transfer)))
			return -EFAULT;
		error = cpustc_tensor_validate_transfer(&transfer);
	} else if (command == CPUSTC_TENSOR_IOC_RUN) {
		if (copy_from_user(&run, user_data, sizeof(run)))
			return -EFAULT;
		error = cpustc_tensor_validate_run(&run, &derived);
		memset(&run.total_cycles, 0,
		       offsetofend(struct cpustc_tensor_run, error_code) -
		       offsetof(struct cpustc_tensor_run, total_cycles));
		report_run = true;
	} else {
		return -ENOTTY;
	}
	if (error)
		goto out_report;

	error = mutex_lock_interruptible(&tensor->lock);
	if (error)
		goto out_report;
	if (tensor->removed) {
		error = -ENODEV;
	} else if (command == CPUSTC_TENSOR_IOC_TRANSFER) {
		error = cpustc_tensor_transfer(tensor, &transfer);
	} else {
		error = cpustc_tensor_run_hardware(tensor, &run, &derived);
	}
	mutex_unlock(&tensor->lock);

out_report:
	if (report_run && copy_to_user(user_data, &run, sizeof(run)))
		error = -EFAULT;
	return error;
}

static const struct file_operations cpustc_tensor_fops = {
	.owner = THIS_MODULE,
	.open = cpustc_tensor_open,
	.release = cpustc_tensor_release,
	.unlocked_ioctl = cpustc_tensor_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.llseek = no_llseek,
};

static int cpustc_tensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpustc_tensor *tensor;
	u32 capabilities0;
	u32 capabilities1;
	u32 capabilities2;
	u32 identification;
	u32 version;
	u16 max_k;
	u8 core_columns;
	u8 tile_columns;
	int error;

	tensor = kzalloc(sizeof(*tensor), GFP_KERNEL);
	if (!tensor)
		return -ENOMEM;
	kref_init(&tensor->ref);
	mutex_init(&tensor->lock);
	init_completion(&tensor->completion);
	tensor->dev = dev;

	tensor->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tensor->base)) {
		error = PTR_ERR(tensor->base);
		goto err_put;
	}

	identification = readl(tensor->base + CPUSTC_TENSOR_IDENTIFICATION);
	capabilities0 = readl(tensor->base + CPUSTC_TENSOR_CAPABILITIES0);
	capabilities1 = readl(tensor->base + CPUSTC_TENSOR_CAPABILITIES1);
	version = readl(tensor->base + CPUSTC_TENSOR_VERSION);
	capabilities2 = readl(tensor->base + CPUSTC_TENSOR_CAPABILITIES2);
	max_k = capabilities0 & 0xffff;
	core_columns = (capabilities0 >> 16) & 0xff;
	tile_columns = capabilities0 >> 24;
	if (identification != CPUSTC_TENSOR_IDENTIFICATION_VALUE ||
	    version != CPUSTC_TENSOR_REGISTER_VERSION ||
	    max_k != CPUSTC_TENSOR_MAX_K ||
	    core_columns != CPUSTC_TENSOR_CORE_COLUMNS ||
	    tile_columns != CPUSTC_TENSOR_TILE_COLUMNS ||
	    capabilities1 != CPUSTC_TENSOR_B_BRAM_BYTES ||
	    capabilities2 != (CPUSTC_TENSOR_CAP_NHWC_WINDOW |
		CPUSTC_TENSOR_CAP_PRELU |
		CPUSTC_TENSOR_CAP_POOL2X2_CEIL |
		CPUSTC_TENSOR_CAP_APPEND_ONE)) {
		error = dev_err_probe(dev, -ENODEV,
				      "unsupported TensorCore id=%#x caps=%#x/%#x/%#x version=%u\n",
				      identification, capabilities0,
				      capabilities1, capabilities2, version);
		goto err_put;
	}

	error = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (error) {
		error = dev_err_probe(dev, error, "failed to set DMA mask\n");
		goto err_put;
	}
	tensor->dma_buffer =
		dma_alloc_noncoherent(dev, CPUSTC_TENSOR_WORKSPACE_BYTES,
				      &tensor->dma_handle, DMA_BIDIRECTIONAL,
				      GFP_KERNEL);
	if (!tensor->dma_buffer) {
		error = -ENOMEM;
		goto err_put;
	}
	if (upper_32_bits(tensor->dma_handle +
			 CPUSTC_TENSOR_WORKSPACE_BYTES - 1)) {
		error = dev_err_probe(dev, -ERANGE,
				      "DMA buffer is outside the 32-bit address space\n");
		goto err_free_dma;
	}
	if (readl(tensor->base + CPUSTC_TENSOR_STATUS) &
	    CPUSTC_TENSOR_STATUS_BUSY) {
		error = dev_err_probe(dev, -EBUSY,
				      "TensorCore is busy during probe\n");
		goto err_free_dma;
	}

	tensor->irq = platform_get_irq(pdev, 0);
	if (tensor->irq < 0) {
		error = tensor->irq;
		goto err_free_dma;
	}
	writel(0, tensor->base + CPUSTC_TENSOR_IRQ_ENABLE);
	writel(CPUSTC_TENSOR_STATUS_DONE | CPUSTC_TENSOR_STATUS_ERROR,
	       tensor->base + CPUSTC_TENSOR_STATUS);
	writel(CPUSTC_TENSOR_IRQ_PENDING,
	       tensor->base + CPUSTC_TENSOR_IRQ_STATUS);
	error = devm_request_irq(dev, tensor->irq, cpustc_tensor_irq, 0,
				 dev_name(dev), tensor);
	if (error) {
		error = dev_err_probe(dev, error,
				      "failed to request completion interrupt\n");
		goto err_free_dma;
	}

	tensor->miscdev.minor = MISC_DYNAMIC_MINOR;
	tensor->miscdev.name = "cpustc-tensor";
	tensor->miscdev.fops = &cpustc_tensor_fops;
	tensor->miscdev.parent = dev;
	error = misc_register(&tensor->miscdev);
	if (error) {
		dev_err(dev, "failed to register misc device: %d\n", error);
		goto err_free_dma;
	}

	platform_set_drvdata(pdev, tensor);
	dev_info(dev,
		 "TensorCore FP32 workspace registered (K<=%u, B tile=%u, workspace=%u KiB)\n",
		 max_k, tile_columns, CPUSTC_TENSOR_WORKSPACE_BYTES / 1024);
	return 0;

err_free_dma:
	dma_free_noncoherent(dev, CPUSTC_TENSOR_WORKSPACE_BYTES,
			     tensor->dma_buffer, tensor->dma_handle,
			     DMA_BIDIRECTIONAL);
err_put:
	kref_put(&tensor->ref, cpustc_tensor_release_ref);
	return error;
}

static int cpustc_tensor_remove(struct platform_device *pdev)
{
	struct cpustc_tensor *tensor = platform_get_drvdata(pdev);
	u32 status;

	mutex_lock(&tensor->lock);
	tensor->removed = true;
	status = readl(tensor->base + CPUSTC_TENSOR_STATUS);
	if (status & CPUSTC_TENSOR_STATUS_BUSY) {
		dev_warn(&pdev->dev,
			 "waiting for timed-out TensorCore operation during removal\n");
		wait_for_completion(&tensor->completion);
		synchronize_irq(tensor->irq);
	}
	writel(0, tensor->base + CPUSTC_TENSOR_IRQ_ENABLE);
	writel(CPUSTC_TENSOR_STATUS_DONE | CPUSTC_TENSOR_STATUS_ERROR,
	       tensor->base + CPUSTC_TENSOR_STATUS);
	writel(CPUSTC_TENSOR_IRQ_PENDING,
	       tensor->base + CPUSTC_TENSOR_IRQ_STATUS);
	mutex_unlock(&tensor->lock);
	synchronize_irq(tensor->irq);
	misc_deregister(&tensor->miscdev);
	dma_sync_single_for_cpu(&pdev->dev, tensor->dma_handle,
				CPUSTC_TENSOR_WORKSPACE_BYTES, DMA_BIDIRECTIONAL);
	dma_free_noncoherent(&pdev->dev, CPUSTC_TENSOR_WORKSPACE_BYTES,
			     tensor->dma_buffer, tensor->dma_handle,
			     DMA_BIDIRECTIONAL);
	platform_set_drvdata(pdev, NULL);
	kref_put(&tensor->ref, cpustc_tensor_release_ref);

	return 0;
}

static const struct of_device_id cpustc_tensor_of_match[] = {
	{ .compatible = "cpustc,tensorcore" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpustc_tensor_of_match);

static struct platform_driver cpustc_tensor_driver = {
	.probe = cpustc_tensor_probe,
	.remove = cpustc_tensor_remove,
	.driver = {
		.name = "cpustc-tensor",
		.of_match_table = cpustc_tensor_of_match,
	},
};
module_platform_driver(cpustc_tensor_driver);

MODULE_DESCRIPTION("CPUSTC TensorCore workspace and GEMM accelerator driver");
MODULE_LICENSE("GPL");
