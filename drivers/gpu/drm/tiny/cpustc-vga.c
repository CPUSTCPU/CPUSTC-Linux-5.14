// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM/KMS driver for the CPUSTC VGA scanout controller.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_connector.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include <drm/cpustc_drm.h>

#define CPUSTC_VGA_H_VISIBLE		0x00
#define CPUSTC_VGA_H_FRONT		0x04
#define CPUSTC_VGA_H_SYNC		0x08
#define CPUSTC_VGA_H_BACK		0x0c
#define CPUSTC_VGA_V_VISIBLE		0x10
#define CPUSTC_VGA_V_FRONT		0x14
#define CPUSTC_VGA_V_SYNC		0x18
#define CPUSTC_VGA_V_BACK		0x1c
#define CPUSTC_VGA_FRAME_BASE		0x20
#define CPUSTC_VGA_BURST_COUNT_MAX	0x24
#define CPUSTC_VGA_IRQ_STATUS		0x28
#define CPUSTC_VGA_IRQ_ENABLE		0x2c
#define CPUSTC_VGA_CONTROL		0x30
#define CPUSTC_CURSOR_IDENTIFICATION	0x34
#define CPUSTC_CURSOR_CAPABILITIES	0x38
#define CPUSTC_CURSOR_POSITION		0x3c
#define CPUSTC_CURSOR_SOURCE		0x40
#define CPUSTC_CURSOR_SIZE		0x44
#define CPUSTC_CURSOR_CONTROL		0x48
#define CPUSTC_CURSOR_RAM_ADDRESS	0x4c
#define CPUSTC_CURSOR_RAM_DATA		0x50
#define CPUSTC_CURSOR_STATUS		0x54

#define CPUSTC_VGA_IRQ_VBLANK		BIT(0)
#define CPUSTC_VGA_CONTROL_ENABLE	BIT(0)
#define CPUSTC_VGA_BYTES_PER_PIXEL	2
#define CPUSTC_VGA_PIXELS_PER_BURST	32

#define CPUSTC_CURSOR_IDENTIFICATION_VALUE	0x43555253
#define CPUSTC_CURSOR_CAP_VERSION_MASK		GENMASK(7, 0)
#define CPUSTC_CURSOR_CAP_WIDTH_MASK		GENMASK(15, 8)
#define CPUSTC_CURSOR_CAP_HEIGHT_MASK		GENMASK(23, 16)
#define CPUSTC_CURSOR_CAP_ARGB8888		BIT(24)
#define CPUSTC_CURSOR_CAP_PREMULTIPLIED		BIT(25)
#define CPUSTC_CURSOR_CAP_DUAL_BANK		BIT(26)
#define CPUSTC_CURSOR_CAP_REQUIRED		(CPUSTC_CURSOR_CAP_ARGB8888 | \
						 CPUSTC_CURSOR_CAP_PREMULTIPLIED | \
						 CPUSTC_CURSOR_CAP_DUAL_BANK)
#define CPUSTC_CURSOR_VERSION		1
#define CPUSTC_CURSOR_WIDTH		64
#define CPUSTC_CURSOR_HEIGHT		64
#define CPUSTC_CURSOR_CONTROL_ENABLE	BIT(0)
#define CPUSTC_CURSOR_CONTROL_BANK	BIT(1)
#define CPUSTC_CURSOR_RAM_BANK		BIT(12)
#define CPUSTC_CURSOR_STATUS_ACTIVE	BIT(0)
#define CPUSTC_CURSOR_STATUS_BANK	BIT(1)
#define CPUSTC_CURSOR_STATUS_BUSY	BIT(2)
#define CPUSTC_CURSOR_STATUS_UPLOAD_ERROR BIT(3)
#define CPUSTC_CURSOR_TIMEOUT_US		50000

#define CPUSTC_2D_STATUS		0x100
#define CPUSTC_2D_COMMAND		0x104
#define CPUSTC_2D_SRC_ADDRESS		0x108
#define CPUSTC_2D_DST_ADDRESS		0x10c
#define CPUSTC_2D_SRC_STRIDE		0x110
#define CPUSTC_2D_DST_STRIDE		0x114
#define CPUSTC_2D_SRC_XY		0x118
#define CPUSTC_2D_DST_XY		0x11c
#define CPUSTC_2D_SIZE			0x120
#define CPUSTC_2D_FOREGROUND		0x124
#define CPUSTC_2D_BACKGROUND		0x128
#define CPUSTC_2D_IDENTIFICATION	0x12c
#define CPUSTC_2D_CAPABILITIES		0x130
#define CPUSTC_2D_VERSION		0x134

#define CPUSTC_2D_STATUS_BUSY		BIT(0)
#define CPUSTC_2D_STATUS_DONE		BIT(1)
#define CPUSTC_2D_STATUS_ERROR		BIT(2)
#define CPUSTC_2D_IDENTIFICATION_VALUE	0x32444750
#define CPUSTC_2D_TIMEOUT_US		1000000
#define CPUSTC_2D_BATCH_MAX_PIXELS	131072
#define CPUSTC_2D_INTERFACE_VERSION_MIN	1
#define CPUSTC_2D_INTERFACE_VERSION_MAX	1

struct cpustc_vga {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_plane cursor_plane;
	struct drm_connector connector;
	void __iomem *regs;
	struct mutex two_d_lock;
	bool has_two_d;
	u32 two_d_capabilities;
	u32 two_d_version;
	bool has_cursor;
	bool cursor_active;
	u8 cursor_bank;
	s32 cursor_x;
	s32 cursor_y;
	u64 cursor_image_uploads;
	u64 cursor_position_updates;
	u64 cursor_disables;
	u64 cursor_failures;
};

static inline struct cpustc_vga *drm_to_cpustc_vga(struct drm_device *drm)
{
	return container_of(drm, struct cpustc_vga, drm);
}

static int cpustc_2d_validate_rgb565(struct drm_gem_object *obj, u32 stride,
				     u16 x, u16 y, u16 width, u16 height)
{
	u64 row_end = ((u64)x + width) * CPUSTC_VGA_BYTES_PER_PIXEL;
	u64 end;

	if (!width || !height || !stride || (stride & 1) || row_end > stride)
		return -EINVAL;
	end = (u64)(y + height - 1) * stride + row_end;
	return end <= obj->size ? 0 : -EINVAL;
}

static int cpustc_2d_validate_bitmap(struct drm_gem_object *obj, u32 stride,
				     u16 x, u16 y, u16 width, u16 height)
{
	u64 row_end = DIV_ROUND_UP((u64)x + width, 8);
	u64 end;

	if (!width || !height || !stride || row_end > stride)
		return -EINVAL;
	end = (u64)(y + height - 1) * stride + row_end;
	return end <= obj->size ? 0 : -EINVAL;
}

static bool cpustc_2d_dma_address_valid(struct drm_gem_cma_object *obj)
{
	u64 end = (u64)obj->paddr + obj->base.size;

	return !upper_32_bits(obj->paddr) && end <= (1ULL << 32);
}

struct cpustc_2d_buffers {
	struct drm_gem_object *src_gem;
	struct drm_gem_object *dst_gem;
	struct drm_gem_cma_object *src;
	struct drm_gem_cma_object *dst;
};

static int cpustc_2d_validate_operation(struct cpustc_vga *vga, u32 operation)
{
	if (!vga->has_two_d)
		return -ENODEV;
	if (operation < DRM_CPUSTC_2D_FILL_RECT ||
	    operation > DRM_CPUSTC_2D_IMAGE_BLIT1)
		return -EINVAL;
	if (!(vga->two_d_capabilities & BIT(operation - 1)))
		return -EOPNOTSUPP;

	return 0;
}

static void cpustc_2d_put_buffers(struct cpustc_2d_buffers *buffers)
{
	if (buffers->src_gem)
		drm_gem_object_put(buffers->src_gem);
	if (buffers->dst_gem)
		drm_gem_object_put(buffers->dst_gem);
}

static int cpustc_2d_lookup_buffers(struct drm_file *file,
				    const struct drm_cpustc_2d_submit *args,
				    struct cpustc_2d_buffers *buffers)
{
	buffers->dst_gem = drm_gem_object_lookup(file, args->dst_handle);
	if (!buffers->dst_gem)
		return -ENOENT;
	buffers->dst = to_drm_gem_cma_obj(buffers->dst_gem);
	if (!cpustc_2d_dma_address_valid(buffers->dst))
		return -ERANGE;

	if (args->operation == DRM_CPUSTC_2D_FILL_RECT)
		return 0;

	buffers->src_gem = drm_gem_object_lookup(file, args->src_handle);
	if (!buffers->src_gem)
		return -ENOENT;
	buffers->src = to_drm_gem_cma_obj(buffers->src_gem);
	if (!cpustc_2d_dma_address_valid(buffers->src))
		return -ERANGE;

	return 0;
}

static int cpustc_2d_validate_geometry(
	const struct drm_cpustc_2d_submit *args,
	const struct cpustc_2d_buffers *buffers)
{
	u64 src_start, src_end, dst_start, dst_end;
	int ret;

	if ((u32)args->dst_x + args->width > 1U << 16 ||
	    (u32)args->dst_y + args->height > 1U << 16 ||
	    (u32)args->src_x + args->width > 1U << 16 ||
	    (u32)args->src_y + args->height > 1U << 16)
		return -EINVAL;

	ret = cpustc_2d_validate_rgb565(buffers->dst_gem, args->dst_stride,
					args->dst_x, args->dst_y,
					args->width, args->height);
	if (ret)
		return ret;

	if (args->operation != DRM_CPUSTC_2D_FILL_RECT) {
		if (args->operation == DRM_CPUSTC_2D_COPY_AREA)
			ret = cpustc_2d_validate_rgb565(buffers->src_gem,
				args->src_stride, args->src_x, args->src_y,
				args->width, args->height);
		else
			ret = cpustc_2d_validate_bitmap(buffers->src_gem,
				args->src_stride, args->src_x, args->src_y,
				args->width, args->height);
		if (ret)
			return ret;
	}

	if (args->operation == DRM_CPUSTC_2D_COPY_AREA &&
	    (buffers->src_gem != buffers->dst_gem ||
	     args->src_stride != args->dst_stride)) {
		src_start = (u64)buffers->src->paddr +
			    (u64)args->src_y * args->src_stride +
			    (u64)args->src_x * 2;
		src_end = (u64)buffers->src->paddr +
			  (u64)(args->src_y + args->height - 1) * args->src_stride +
			  (u64)(args->src_x + args->width) * 2;
		dst_start = (u64)buffers->dst->paddr +
			    (u64)args->dst_y * args->dst_stride +
			    (u64)args->dst_x * 2;
		dst_end = (u64)buffers->dst->paddr +
			  (u64)(args->dst_y + args->height - 1) * args->dst_stride +
			  (u64)(args->dst_x + args->width) * 2;
		if (src_start < dst_end && dst_start < src_end)
			return -EINVAL;
	}

	return 0;
}

static int cpustc_2d_execute_locked(
	struct cpustc_vga *vga, const struct drm_cpustc_2d_submit *args,
	const struct cpustc_2d_buffers *buffers, bool program_common)
{
	u32 status;
	int ret;

	writel(CPUSTC_2D_STATUS_DONE | CPUSTC_2D_STATUS_ERROR,
	       vga->regs + CPUSTC_2D_STATUS);
	if (program_common) {
		writel(buffers->src ? lower_32_bits(buffers->src->paddr) : 0,
		       vga->regs + CPUSTC_2D_SRC_ADDRESS);
		writel(lower_32_bits(buffers->dst->paddr),
		       vga->regs + CPUSTC_2D_DST_ADDRESS);
		writel(args->src_stride, vga->regs + CPUSTC_2D_SRC_STRIDE);
		writel(args->dst_stride, vga->regs + CPUSTC_2D_DST_STRIDE);
		writel(args->foreground, vga->regs + CPUSTC_2D_FOREGROUND);
		writel(args->background, vga->regs + CPUSTC_2D_BACKGROUND);
	}
	writel(((u32)args->src_y << 16) | args->src_x,
	       vga->regs + CPUSTC_2D_SRC_XY);
	writel(((u32)args->dst_y << 16) | args->dst_x,
	       vga->regs + CPUSTC_2D_DST_XY);
	writel(((u32)args->height << 16) | args->width,
	       vga->regs + CPUSTC_2D_SIZE);
	/* Publish buffer writes and the complete register set before COMMAND. */
	wmb();
	writel(args->operation, vga->regs + CPUSTC_2D_COMMAND);

	ret = readl_poll_timeout(vga->regs + CPUSTC_2D_STATUS, status,
				 !(status & CPUSTC_2D_STATUS_BUSY), 10,
				 CPUSTC_2D_TIMEOUT_US);
	if (!ret && !(status & CPUSTC_2D_STATUS_DONE))
		ret = -EIO;
	if (!ret && (status & CPUSTC_2D_STATUS_ERROR))
		ret = -EIO;
	/* Order completed DMA writes before userspace observes ioctl return. */
	rmb();
	writel(status & (CPUSTC_2D_STATUS_DONE | CPUSTC_2D_STATUS_ERROR),
	       vga->regs + CPUSTC_2D_STATUS);
	return ret;
}

static int cpustc_2d_submit_ioctl(struct drm_device *drm, void *data,
				  struct drm_file *file)
{
	struct drm_cpustc_2d_submit *args = data;
	struct cpustc_vga *vga = drm_to_cpustc_vga(drm);
	struct cpustc_2d_buffers buffers = {};
	int ret;

	if (args->flags || memchr_inv(args->reserved, 0, sizeof(args->reserved)))
		return -EINVAL;
	ret = cpustc_2d_validate_operation(vga, args->operation);
	if (ret)
		return ret;
	ret = cpustc_2d_lookup_buffers(file, args, &buffers);
	if (ret)
		goto out_put;
	ret = cpustc_2d_validate_geometry(args, &buffers);
	if (ret)
		goto out_put;

	ret = mutex_lock_interruptible(&vga->two_d_lock);
	if (ret)
		goto out_put;
	ret = cpustc_2d_execute_locked(vga, args, &buffers, true);
	mutex_unlock(&vga->two_d_lock);

out_put:
	cpustc_2d_put_buffers(&buffers);
	return ret;
}

static void cpustc_2d_batch_to_submit(
	const struct drm_cpustc_2d_batch *batch,
	const struct drm_cpustc_2d_rect *rect,
	struct drm_cpustc_2d_submit *submit)
{
	memset(submit, 0, sizeof(*submit));
	submit->operation = batch->operation;
	submit->src_handle = batch->src_handle;
	submit->dst_handle = batch->dst_handle;
	submit->src_stride = batch->src_stride;
	submit->dst_stride = batch->dst_stride;
	submit->foreground = batch->foreground;
	submit->background = batch->background;
	submit->src_x = rect->src_x;
	submit->src_y = rect->src_y;
	submit->dst_x = rect->dst_x;
	submit->dst_y = rect->dst_y;
	submit->width = rect->width;
	submit->height = rect->height;
}

static int cpustc_2d_submit_batch_ioctl(struct drm_device *drm, void *data,
					struct drm_file *file)
{
	struct drm_cpustc_2d_batch *args = data;
	struct cpustc_vga *vga = drm_to_cpustc_vga(drm);
	struct cpustc_2d_buffers buffers = {};
	struct drm_cpustc_2d_submit submit;
	struct drm_cpustc_2d_rect *rects;
	u64 total_pixels = 0;
	u32 index;
	int ret;

	if (args->flags || args->completed || args->pad ||
	    memchr_inv(args->reserved, 0, sizeof(args->reserved)) ||
	    !args->rectangles || !args->count ||
	    args->count > DRM_CPUSTC_2D_BATCH_MAX_RECTS)
		return -EINVAL;
	ret = cpustc_2d_validate_operation(vga, args->operation);
	if (ret)
		return ret;

	rects = memdup_user(u64_to_user_ptr(args->rectangles),
			    args->count * sizeof(*rects));
	if (IS_ERR(rects))
		return PTR_ERR(rects);

	cpustc_2d_batch_to_submit(args, &rects[0], &submit);
	ret = cpustc_2d_lookup_buffers(file, &submit, &buffers);
	if (ret)
		goto out_free;

	for (index = 0; index < args->count; index++) {
		if (memchr_inv(rects[index].reserved, 0,
			       sizeof(rects[index].reserved))) {
			ret = -EINVAL;
			goto out_put;
		}
		cpustc_2d_batch_to_submit(args, &rects[index], &submit);
		ret = cpustc_2d_validate_geometry(&submit, &buffers);
		if (ret)
			goto out_put;
		total_pixels += (u64)submit.width * submit.height;
		if (total_pixels > CPUSTC_2D_BATCH_MAX_PIXELS) {
			ret = -E2BIG;
			goto out_put;
		}
	}

	ret = mutex_lock_interruptible(&vga->two_d_lock);
	if (ret)
		goto out_put;
	for (index = 0; index < args->count; index++) {
		cpustc_2d_batch_to_submit(args, &rects[index], &submit);
		ret = cpustc_2d_execute_locked(vga, &submit, &buffers,
					       index == 0);
		if (ret)
			break;
		args->completed = index + 1;
	}
	mutex_unlock(&vga->two_d_lock);

out_put:
	cpustc_2d_put_buffers(&buffers);
out_free:
	kfree(rects);
	return ret;
}

static int cpustc_2d_get_caps_ioctl(struct drm_device *drm, void *data,
				    struct drm_file *file)
{
	struct drm_cpustc_2d_caps *args = data;
	struct cpustc_vga *vga = drm_to_cpustc_vga(drm);

	if (!vga->has_two_d)
		return -ENODEV;
	if (memchr_inv(args->reserved, 0, sizeof(args->reserved)))
		return -EINVAL;

	args->version = vga->two_d_version;
	args->capabilities = vga->two_d_capabilities | DRM_CPUSTC_2D_CAP_BATCH;
	return 0;
}

static const struct drm_ioctl_desc cpustc_vga_ioctls[] = {
	DRM_IOCTL_DEF_DRV(CPUSTC_2D_SUBMIT, cpustc_2d_submit_ioctl, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(CPUSTC_2D_GET_CAPS, cpustc_2d_get_caps_ioctl, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(CPUSTC_2D_SUBMIT_BATCH,
			cpustc_2d_submit_batch_ioctl, DRM_AUTH),
};

static const struct drm_display_mode cpustc_vga_mode = {
	DRM_MODE("640x480", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		 25000, 640, 656, 752, 800, 0,
		 480, 490, 492, 525, 0,
		 DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
};

static inline struct cpustc_vga *pipe_to_cpustc_vga(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct cpustc_vga, pipe);
}

static inline struct cpustc_vga *cursor_plane_to_cpustc_vga(struct drm_plane *plane)
{
	return container_of(plane, struct cpustc_vga, cursor_plane);
}

static int cpustc_vga_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &cpustc_vga_mode);
	if (!mode)
		return 0;

	drm_mode_probed_add(connector, mode);
	return 1;
}

static enum drm_connector_status
cpustc_vga_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static const struct drm_connector_helper_funcs cpustc_vga_connector_helper_funcs = {
	.get_modes = cpustc_vga_connector_get_modes,
};

static const struct drm_connector_funcs cpustc_vga_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.detect = cpustc_vga_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int cpustc_cursor_wait_idle(struct cpustc_vga *vga)
{
	u32 status;

	return readl_poll_timeout(vga->regs + CPUSTC_CURSOR_STATUS, status,
				  !(status & CPUSTC_CURSOR_STATUS_BUSY), 1,
				  CPUSTC_CURSOR_TIMEOUT_US);
}

static int cpustc_cursor_upload(struct cpustc_vga *vga,
				struct drm_framebuffer *fb, u8 bank)
{
	struct drm_gem_cma_object *obj = drm_fb_cma_get_gem_obj(fb, 0);
	const u8 *source = obj->vaddr;
	u32 pixel;
	u32 status;
	u32 x, y;
	int ret;

	ret = cpustc_cursor_wait_idle(vga);
	if (ret)
		return ret;

	writel(bank ? CPUSTC_CURSOR_RAM_BANK : 0,
	       vga->regs + CPUSTC_CURSOR_RAM_ADDRESS);
	for (y = 0; y < CPUSTC_CURSOR_HEIGHT; y++) {
		for (x = 0; x < CPUSTC_CURSOR_WIDTH; x++) {
			pixel = 0;
			if (x < fb->width && y < fb->height)
				memcpy(&pixel,
				       source + fb->offsets[0] + y * fb->pitches[0] + x * 4,
				       sizeof(pixel));
			writel(pixel, vga->regs + CPUSTC_CURSOR_RAM_DATA);
		}
	}

	status = readl(vga->regs + CPUSTC_CURSOR_STATUS);
	if (status & CPUSTC_CURSOR_STATUS_UPLOAD_ERROR)
		return -EIO;

	vga->cursor_image_uploads++;
	return 0;
}

static int cpustc_cursor_commit(struct cpustc_vga *vga, bool enable, u8 bank,
				s32 x, s32 y, u8 src_x, u8 src_y,
				u8 width, u8 height)
{
	u32 control = bank ? CPUSTC_CURSOR_CONTROL_BANK : 0;
	u32 position = (u16)x | ((u32)(u16)y << 16);
	u32 source = src_x | ((u32)src_y << 8);
	u32 size = width | ((u32)height << 8);
	int ret;

	ret = cpustc_cursor_wait_idle(vga);
	if (ret)
		return ret;

	writel(position, vga->regs + CPUSTC_CURSOR_POSITION);
	writel(source, vga->regs + CPUSTC_CURSOR_SOURCE);
	writel(size, vga->regs + CPUSTC_CURSOR_SIZE);
	if (enable)
		control |= CPUSTC_CURSOR_CONTROL_ENABLE;
	/* Publish the complete staging payload before triggering the commit. */
	wmb();
	writel(control, vga->regs + CPUSTC_CURSOR_CONTROL);

	return cpustc_cursor_wait_idle(vga);
}

static void cpustc_cursor_report_failure(struct cpustc_vga *vga,
					 const char *operation, int ret)
{
	vga->cursor_failures++;
	dev_err_ratelimited(vga->drm.dev, "cursor %s failed: %d\n",
			    operation, ret);
}

static void cpustc_cursor_force_disable(struct cpustc_vga *vga)
{
	int ret;

	if (!vga->has_cursor)
		return;

	ret = cpustc_cursor_commit(vga, false, vga->cursor_bank,
				   0, 0, 0, 0, 0, 0);
	if (ret)
		cpustc_cursor_report_failure(vga, "disable", ret);
	vga->cursor_active = false;
}

static int cpustc_cursor_atomic_check(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	struct drm_gem_cma_object *obj;
	struct drm_framebuffer *fb;
	u64 end;
	int ret;

	if (!new_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  DRM_PLANE_HELPER_NO_SCALING,
						  DRM_PLANE_HELPER_NO_SCALING,
						  true, true);
	if (ret || !new_state->fb)
		return ret;

	fb = new_state->fb;
	if (fb->format->format != DRM_FORMAT_ARGB8888 ||
	    fb->modifier != DRM_FORMAT_MOD_LINEAR ||
	    !fb->width || fb->width > CPUSTC_CURSOR_WIDTH ||
	    !fb->height || fb->height > CPUSTC_CURSOR_HEIGHT ||
	    fb->pitches[0] < fb->width * 4 ||
	    new_state->rotation != DRM_MODE_ROTATE_0)
		return -EINVAL;

	if ((new_state->src.x1 | new_state->src.y1 |
	     new_state->src.x2 | new_state->src.y2) & 0xffff)
		return -EINVAL;

	obj = drm_fb_cma_get_gem_obj(fb, 0);
	if (!obj || !obj->vaddr)
		return -EINVAL;
	end = (u64)fb->offsets[0] + (fb->height - 1) * (u64)fb->pitches[0] +
	      fb->width * 4ULL;
	if (end > obj->base.size)
		return -EINVAL;

	return 0;
}

static void cpustc_cursor_atomic_update(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct cpustc_vga *vga = cursor_plane_to_cpustc_vga(plane);
	struct drm_plane_state *old_state =
		drm_atomic_get_old_plane_state(state, plane);
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	bool image_changed = old_state->fb != new_state->fb;
	u8 bank = vga->cursor_bank;
	u8 src_x = new_state->src.x1 >> 16;
	u8 src_y = new_state->src.y1 >> 16;
	u8 width = drm_rect_width(&new_state->dst);
	u8 height = drm_rect_height(&new_state->dst);
	int ret;

	if (image_changed) {
		bank ^= 1;
		ret = cpustc_cursor_upload(vga, new_state->fb, bank);
		if (ret) {
			cpustc_cursor_report_failure(vga, "upload", ret);
			cpustc_cursor_force_disable(vga);
			return;
		}
	}

	ret = cpustc_cursor_commit(vga, new_state->visible, bank,
				   new_state->dst.x1, new_state->dst.y1,
				   src_x, src_y, width, height);
	if (ret) {
		cpustc_cursor_report_failure(vga, "commit", ret);
		cpustc_cursor_force_disable(vga);
		return;
	}

	if (image_changed)
		vga->cursor_bank = bank;
	else
		vga->cursor_position_updates++;
	if (!new_state->visible)
		vga->cursor_disables++;
	vga->cursor_active = new_state->visible;
	vga->cursor_x = new_state->dst.x1;
	vga->cursor_y = new_state->dst.y1;
}

static void cpustc_cursor_atomic_disable(struct drm_plane *plane,
					 struct drm_atomic_state *state)
{
	struct cpustc_vga *vga = cursor_plane_to_cpustc_vga(plane);
	int ret;

	ret = cpustc_cursor_commit(vga, false, vga->cursor_bank,
				   0, 0, 0, 0, 0, 0);
	if (ret)
		cpustc_cursor_report_failure(vga, "atomic disable", ret);
	else
		vga->cursor_disables++;
	vga->cursor_active = false;
}

static const struct drm_plane_helper_funcs cpustc_cursor_plane_helper_funcs = {
	.atomic_check = cpustc_cursor_atomic_check,
	.atomic_update = cpustc_cursor_atomic_update,
	.atomic_disable = cpustc_cursor_atomic_disable,
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
};

static const struct drm_plane_funcs cpustc_cursor_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static bool cpustc_vga_mode_matches(const struct drm_display_mode *mode)
{
	return mode->clock == cpustc_vga_mode.clock &&
	       mode->hdisplay == cpustc_vga_mode.hdisplay &&
	       mode->hsync_start == cpustc_vga_mode.hsync_start &&
	       mode->hsync_end == cpustc_vga_mode.hsync_end &&
	       mode->htotal == cpustc_vga_mode.htotal &&
	       mode->vdisplay == cpustc_vga_mode.vdisplay &&
	       mode->vsync_start == cpustc_vga_mode.vsync_start &&
	       mode->vsync_end == cpustc_vga_mode.vsync_end &&
	       mode->vtotal == cpustc_vga_mode.vtotal &&
	       (mode->flags & (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NHSYNC |
			       DRM_MODE_FLAG_PVSYNC | DRM_MODE_FLAG_NVSYNC)) ==
	       cpustc_vga_mode.flags;
}

static enum drm_mode_status
cpustc_vga_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			   const struct drm_display_mode *mode)
{
	return cpustc_vga_mode_matches(mode) ? MODE_OK : MODE_BAD;
}

static int cpustc_vga_pipe_check(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *plane_state,
				 struct drm_crtc_state *crtc_state)
{
	struct drm_framebuffer *fb = plane_state->fb;
	dma_addr_t address;

	if (!crtc_state->enable || !fb)
		return 0;

	if (!cpustc_vga_mode_matches(&crtc_state->adjusted_mode))
		return -EINVAL;

	if (fb->format->format != DRM_FORMAT_RGB565 ||
	    fb->pitches[0] != crtc_state->adjusted_mode.hdisplay *
			      CPUSTC_VGA_BYTES_PER_PIXEL)
		return -EINVAL;

	if (!drm_fb_cma_get_gem_obj(fb, 0))
		return -EINVAL;

	address = drm_fb_cma_get_gem_addr(fb, plane_state, 0);
	if (upper_32_bits(address) || !IS_ALIGNED(address, 4))
		return -EINVAL;

	return 0;
}

static void cpustc_vga_program_mode(struct cpustc_vga *vga,
				    const struct drm_display_mode *mode)
{
	writel(mode->hdisplay, vga->regs + CPUSTC_VGA_H_VISIBLE);
	writel(mode->hsync_start - mode->hdisplay,
	       vga->regs + CPUSTC_VGA_H_FRONT);
	writel(mode->hsync_end - mode->hsync_start,
	       vga->regs + CPUSTC_VGA_H_SYNC);
	writel(mode->htotal - mode->hsync_end,
	       vga->regs + CPUSTC_VGA_H_BACK);
	writel(mode->vdisplay, vga->regs + CPUSTC_VGA_V_VISIBLE);
	writel(mode->vsync_start - mode->vdisplay,
	       vga->regs + CPUSTC_VGA_V_FRONT);
	writel(mode->vsync_end - mode->vsync_start,
	       vga->regs + CPUSTC_VGA_V_SYNC);
	writel(mode->vtotal - mode->vsync_end,
	       vga->regs + CPUSTC_VGA_V_BACK);
	writel(mode->hdisplay / CPUSTC_VGA_PIXELS_PER_BURST,
	       vga->regs + CPUSTC_VGA_BURST_COUNT_MAX);
}

static void cpustc_vga_pipe_enable(struct drm_simple_display_pipe *pipe,
				   struct drm_crtc_state *crtc_state,
				   struct drm_plane_state *plane_state)
{
	struct cpustc_vga *vga = pipe_to_cpustc_vga(pipe);
	dma_addr_t address;

	writel(0, vga->regs + CPUSTC_VGA_CONTROL);
	cpustc_vga_program_mode(vga, &crtc_state->adjusted_mode);

	address = drm_fb_cma_get_gem_addr(plane_state->fb, plane_state, 0);
	writel(lower_32_bits(address), vga->regs + CPUSTC_VGA_FRAME_BASE);
	writel(CPUSTC_VGA_CONTROL_ENABLE, vga->regs + CPUSTC_VGA_CONTROL);

	drm_crtc_vblank_on(&pipe->crtc);
}

static void cpustc_vga_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct cpustc_vga *vga = pipe_to_cpustc_vga(pipe);

	drm_crtc_vblank_off(&pipe->crtc);
	cpustc_cursor_force_disable(vga);
	writel(0, vga->regs + CPUSTC_VGA_CONTROL);
}

static void cpustc_vga_pipe_update(struct drm_simple_display_pipe *pipe,
				   struct drm_plane_state *old_state)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_pending_vblank_event *event = crtc->state->event;
	struct drm_plane_state *plane_state = pipe->plane.state;
	struct cpustc_vga *vga = pipe_to_cpustc_vga(pipe);
	unsigned long flags;
	dma_addr_t address;

	if (plane_state->fb) {
		address = drm_fb_cma_get_gem_addr(plane_state->fb, plane_state, 0);
		writel(lower_32_bits(address),
		       vga->regs + CPUSTC_VGA_FRAME_BASE);
	}

	if (!event)
		return;

	crtc->state->event = NULL;
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int cpustc_vga_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct cpustc_vga *vga = pipe_to_cpustc_vga(pipe);

	writel(CPUSTC_VGA_IRQ_VBLANK, vga->regs + CPUSTC_VGA_IRQ_STATUS);
	writel(CPUSTC_VGA_IRQ_VBLANK, vga->regs + CPUSTC_VGA_IRQ_ENABLE);
	return 0;
}

static void cpustc_vga_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct cpustc_vga *vga = pipe_to_cpustc_vga(pipe);

	writel(0, vga->regs + CPUSTC_VGA_IRQ_ENABLE);
}

static const struct drm_simple_display_pipe_funcs cpustc_vga_pipe_funcs = {
	.check = cpustc_vga_pipe_check,
	.mode_valid = cpustc_vga_pipe_mode_valid,
	.enable = cpustc_vga_pipe_enable,
	.disable = cpustc_vga_pipe_disable,
	.update = cpustc_vga_pipe_update,
	.prepare_fb = drm_gem_simple_display_pipe_prepare_fb,
	.enable_vblank = cpustc_vga_enable_vblank,
	.disable_vblank = cpustc_vga_disable_vblank,
};

static irqreturn_t cpustc_vga_irq(int irq, void *data)
{
	struct cpustc_vga *vga = data;
	u32 status;

	status = readl(vga->regs + CPUSTC_VGA_IRQ_STATUS);
	if (!(status & CPUSTC_VGA_IRQ_VBLANK))
		return IRQ_NONE;

	writel(CPUSTC_VGA_IRQ_VBLANK, vga->regs + CPUSTC_VGA_IRQ_STATUS);
	drm_crtc_handle_vblank(&vga->pipe.crtc);

	return IRQ_HANDLED;
}

static const struct drm_mode_config_funcs cpustc_vga_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

#ifdef CONFIG_DEBUG_FS
static int cpustc_cursor_debugfs_show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = m->private;
	struct cpustc_vga *vga = drm_to_cpustc_vga(node->minor->dev);
	u32 status = vga->has_cursor ?
		readl(vga->regs + CPUSTC_CURSOR_STATUS) : 0;

	seq_printf(m, "hardware_present: %u\n", vga->has_cursor);
	seq_printf(m, "active: %u\n", !!(status & CPUSTC_CURSOR_STATUS_ACTIVE));
	seq_printf(m, "active_bank: %u\n", !!(status & CPUSTC_CURSOR_STATUS_BANK));
	seq_printf(m, "busy: %u\n", !!(status & CPUSTC_CURSOR_STATUS_BUSY));
	seq_printf(m, "upload_error: %u\n",
		   !!(status & CPUSTC_CURSOR_STATUS_UPLOAD_ERROR));
	seq_printf(m, "last_position: %d %d\n",
		   READ_ONCE(vga->cursor_x), READ_ONCE(vga->cursor_y));
	seq_printf(m, "image_uploads: %llu\n",
		   (unsigned long long)READ_ONCE(vga->cursor_image_uploads));
	seq_printf(m, "position_updates: %llu\n",
		   (unsigned long long)READ_ONCE(vga->cursor_position_updates));
	seq_printf(m, "disables: %llu\n",
		   (unsigned long long)READ_ONCE(vga->cursor_disables));
	seq_printf(m, "failures: %llu\n",
		   (unsigned long long)READ_ONCE(vga->cursor_failures));
	return 0;
}

static int cpustc_2d_debugfs_show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = m->private;
	struct cpustc_vga *vga = drm_to_cpustc_vga(node->minor->dev);

	seq_printf(m, "hardware_present: %u\n", vga->has_two_d);
	seq_printf(m, "version: %u\n", vga->two_d_version);
	seq_printf(m, "capabilities: %#x\n", vga->two_d_capabilities);
	return 0;
}

static const struct drm_info_list cpustc_vga_debugfs_list[] = {
	{ "cpustc_cursor", cpustc_cursor_debugfs_show, 0 },
	{ "cpustc_2d", cpustc_2d_debugfs_show, 0 },
};

static void cpustc_vga_debugfs_init(struct drm_minor *minor)
{
	drm_debugfs_create_files(cpustc_vga_debugfs_list,
				 ARRAY_SIZE(cpustc_vga_debugfs_list),
				 minor->debugfs_root, minor);
}
#endif

DEFINE_DRM_GEM_CMA_FOPS(cpustc_vga_fops);

static const struct drm_driver cpustc_vga_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "cpustc-vga",
	.desc = "CPUSTC VGA DRM/KMS",
	.date = "20260730",
	.major = 1,
	.minor = 0,
	.fops = &cpustc_vga_fops,
	.ioctls = cpustc_vga_ioctls,
	.num_ioctls = ARRAY_SIZE(cpustc_vga_ioctls),
	DRM_GEM_CMA_DRIVER_OPS,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init = cpustc_vga_debugfs_init,
#endif
};

static int cpustc_vga_modeset_init(struct cpustc_vga *vga)
{
	static const u32 formats[] = { DRM_FORMAT_RGB565 };
	static const u32 cursor_formats[] = { DRM_FORMAT_ARGB8888 };
	struct drm_device *drm = &vga->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	/* The global framebuffer bounds also apply to cursor framebuffers. */
	drm->mode_config.min_width = 1;
	drm->mode_config.max_width = cpustc_vga_mode.hdisplay;
	drm->mode_config.min_height = 1;
	drm->mode_config.max_height = cpustc_vga_mode.vdisplay;
	drm->mode_config.preferred_depth = 16;
	drm->mode_config.funcs = &cpustc_vga_mode_config_funcs;

	ret = drm_connector_init(drm, &vga->connector,
				 &cpustc_vga_connector_funcs,
				 DRM_MODE_CONNECTOR_VGA);
	if (ret)
		return ret;
	drm_connector_helper_add(&vga->connector,
				 &cpustc_vga_connector_helper_funcs);

	ret = drm_simple_display_pipe_init(drm, &vga->pipe,
					   &cpustc_vga_pipe_funcs,
					   formats, ARRAY_SIZE(formats),
					   NULL, &vga->connector);
	if (ret)
		return ret;
	if (vga->has_cursor) {
		ret = drm_universal_plane_init(drm, &vga->cursor_plane,
					       drm_crtc_mask(&vga->pipe.crtc),
					       &cpustc_cursor_plane_funcs,
					       cursor_formats,
					       ARRAY_SIZE(cursor_formats), NULL,
					       DRM_PLANE_TYPE_CURSOR, NULL);
		if (ret)
			return ret;
		drm_plane_helper_add(&vga->cursor_plane,
				     &cpustc_cursor_plane_helper_funcs);
		vga->pipe.crtc.cursor = &vga->cursor_plane;
		drm->mode_config.cursor_width = CPUSTC_CURSOR_WIDTH;
		drm->mode_config.cursor_height = CPUSTC_CURSOR_HEIGHT;
	}

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	return 0;
}

static int cpustc_vga_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpustc_vga *vga;
	u32 cursor_caps = 0;
	int irq;
	int ret;

	vga = devm_drm_dev_alloc(dev, &cpustc_vga_drm_driver,
				 struct cpustc_vga, drm);
	if (IS_ERR(vga))
		return PTR_ERR(vga);

	vga->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vga->regs))
		return PTR_ERR(vga->regs);
	vga->has_cursor = device_property_read_bool(dev,
						    "cpustc,hardware-cursor");
	if (vga->has_cursor) {
		cursor_caps = readl(vga->regs + CPUSTC_CURSOR_CAPABILITIES);
		if (readl(vga->regs + CPUSTC_CURSOR_IDENTIFICATION) !=
						 CPUSTC_CURSOR_IDENTIFICATION_VALUE ||
		    (cursor_caps & CPUSTC_CURSOR_CAP_VERSION_MASK) !=
						 CPUSTC_CURSOR_VERSION ||
		    ((cursor_caps & CPUSTC_CURSOR_CAP_WIDTH_MASK) >> 8) !=
						 CPUSTC_CURSOR_WIDTH ||
		    ((cursor_caps & CPUSTC_CURSOR_CAP_HEIGHT_MASK) >> 16) !=
						 CPUSTC_CURSOR_HEIGHT ||
		    (cursor_caps & CPUSTC_CURSOR_CAP_REQUIRED) !=
						 CPUSTC_CURSOR_CAP_REQUIRED)
			vga->has_cursor = false;
	}
	if (device_property_read_bool(dev, "cpustc,hardware-cursor") &&
	    !vga->has_cursor)
		dev_warn(dev, "hardware cursor capability mismatch; using software cursor\n");
	else if (vga->has_cursor)
		dev_info(dev, "64x64 ARGB8888 hardware cursor capabilities %#x\n",
			 cursor_caps);
	mutex_init(&vga->two_d_lock);
	vga->has_two_d = readl(vga->regs + CPUSTC_2D_IDENTIFICATION) ==
			     CPUSTC_2D_IDENTIFICATION_VALUE;
	if (vga->has_two_d) {
		vga->two_d_capabilities = readl(vga->regs + CPUSTC_2D_CAPABILITIES);
		vga->two_d_version = readl(vga->regs + CPUSTC_2D_VERSION);
		if (vga->two_d_version < CPUSTC_2D_INTERFACE_VERSION_MIN ||
		    vga->two_d_version > CPUSTC_2D_INTERFACE_VERSION_MAX ||
		    !(vga->two_d_capabilities & DRM_CPUSTC_2D_CAP_RGB565))
			vga->has_two_d = false;
	}
	if (!vga->has_two_d)
		dev_warn(dev, "2D accelerator not found; submit ioctl disabled\n");
	else
		dev_info(dev, "2D accelerator v%u capabilities %#x\n",
			 vga->two_d_version, vga->two_d_capabilities);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	writel(0, vga->regs + CPUSTC_VGA_IRQ_ENABLE);
	writel(CPUSTC_VGA_IRQ_VBLANK, vga->regs + CPUSTC_VGA_IRQ_STATUS);
	writel(0, vga->regs + CPUSTC_VGA_CONTROL);
	cpustc_cursor_force_disable(vga);

	ret = devm_request_irq(dev, irq, cpustc_vga_irq, 0,
			       dev_name(dev), vga);
	if (ret)
		return ret;

	ret = cpustc_vga_modeset_init(vga);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, &vga->drm);

	ret = drm_dev_register(&vga->drm, 0);
	if (ret)
		return ret;

	drm_fbdev_generic_setup(&vga->drm, 16);
	return 0;
}

static int cpustc_vga_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct cpustc_vga *vga = drm_to_cpustc_vga(drm);

	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);
	writel(0, vga->regs + CPUSTC_VGA_IRQ_ENABLE);
	cpustc_cursor_force_disable(vga);
	writel(0, vga->regs + CPUSTC_VGA_CONTROL);

	return 0;
}

static void cpustc_vga_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct cpustc_vga *vga = drm_to_cpustc_vga(drm);

	drm_atomic_helper_shutdown(drm);
	cpustc_cursor_force_disable(vga);
}

static const struct of_device_id cpustc_vga_of_match[] = {
	{ .compatible = "cpustc,vga" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpustc_vga_of_match);

static struct platform_driver cpustc_vga_platform_driver = {
	.probe = cpustc_vga_probe,
	.remove = cpustc_vga_remove,
	.shutdown = cpustc_vga_shutdown,
	.driver = {
		.name = "cpustc-vga-drm",
		.of_match_table = cpustc_vga_of_match,
	},
};
module_platform_driver(cpustc_vga_platform_driver);

MODULE_DESCRIPTION("CPUSTC VGA DRM/KMS driver");
MODULE_AUTHOR("CPUSTCPU project");
MODULE_LICENSE("GPL");
