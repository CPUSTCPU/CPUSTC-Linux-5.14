// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPUSTC parallel camera capture controller
 *
 * The hardware intentionally has a very small contract: one fixed RGB565
 * mode and descriptor/completion FIFOs.  This driver keeps at most one
 * descriptor in hardware because the RTL has no command to flush descriptors.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#define CPUSTC_CAM_ID			0x000
#define CPUSTC_CAM_VERSION		0x004
#define CPUSTC_CAM_CAPABILITIES		0x008
#define CPUSTC_CAM_CONTROL		0x00c
#define CPUSTC_CAM_STATUS		0x010
#define CPUSTC_CAM_IRQ_STATUS		0x014
#define CPUSTC_CAM_IRQ_ENABLE		0x018
#define CPUSTC_CAM_ABORT		0x01c
#define CPUSTC_CAM_FORMAT		0x020
#define CPUSTC_CAM_WIDTH		0x024
#define CPUSTC_CAM_HEIGHT		0x028
#define CPUSTC_CAM_BYTES_PER_LINE	0x02c
#define CPUSTC_CAM_FRAME_BYTES		0x030
#define CPUSTC_CAM_QUEUE_ADDR		0x040
#define CPUSTC_CAM_QUEUE_TAG		0x044
#define CPUSTC_CAM_QUEUE_PUSH		0x048
#define CPUSTC_CAM_QUEUE_COUNT		0x04c
#define CPUSTC_CAM_DONE_TAG		0x060
#define CPUSTC_CAM_DONE_STATUS		0x064
#define CPUSTC_CAM_DONE_BYTES		0x068
#define CPUSTC_CAM_DONE_POP		0x06c
#define CPUSTC_CAM_DONE_COUNT		0x070

#define CPUSTC_CAM_ID_VALUE		0x43414d31
#define CPUSTC_CAM_VERSION_MAJOR	0x00010000

#define CPUSTC_CAM_CONTROL_ENABLE	BIT(0)
#define CPUSTC_CAM_CONTROL_BYTE_SWAP	BIT(1)

#define CPUSTC_CAM_STATUS_QUEUED		BIT(0)
#define CPUSTC_CAM_STATUS_DONE		BIT(1)
#define CPUSTC_CAM_STATUS_BUSY		BIT(2)

#define CPUSTC_CAM_IRQ_DONE		BIT(0)
#define CPUSTC_CAM_IRQ_FIFO_OVERFLOW	BIT(1)
#define CPUSTC_CAM_IRQ_FRAME_ERROR	BIT(2)
#define CPUSTC_CAM_IRQ_AXI_ERROR	BIT(3)
#define CPUSTC_CAM_IRQ_QUEUE_ERROR	BIT(4)
#define CPUSTC_CAM_IRQ_ABORTED		BIT(5)
#define CPUSTC_CAM_IRQ_ALL		GENMASK(5, 0)
#define CPUSTC_CAM_IRQ_STICKY		GENMASK(5, 1)

#define CPUSTC_CAM_DONE_FIFO_OVERFLOW	BIT(0)
#define CPUSTC_CAM_DONE_FRAME_SIZE	BIT(1)
#define CPUSTC_CAM_DONE_PROTOCOL	BIT(2)
#define CPUSTC_CAM_DONE_AXI_ERROR	BIT(3)
#define CPUSTC_CAM_DONE_ABORTED		BIT(4)

#define CPUSTC_CAM_WIDTH_VALUE		640U
#define CPUSTC_CAM_HEIGHT_VALUE		480U
#define CPUSTC_CAM_BYTES_PER_LINE_VALUE	1280U
#define CPUSTC_CAM_FRAME_BYTES_VALUE	614400U
#define CPUSTC_CAM_FRAME_INTERVAL_NUM	1U
#define CPUSTC_CAM_FRAME_INTERVAL_DEN	5U
#define CPUSTC_CAM_FRAME_RATE_MIN	1U
#define CPUSTC_CAM_FRAME_RATE_MAX	25U
#define CPUSTC_CAM_ADDR_ALIGN		64U

/* Two 1 fps frame periods plus margin for a descriptor queued near SOF. */
#define CPUSTC_CAM_DRAIN_TIMEOUT_MS	2500U
#define CPUSTC_CAM_ABORT_TIMEOUT_US	250000U

struct cpustc_camera_dma_lease {
	struct list_head quarantine_entry;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	bool device_owned;
};

struct cpustc_camera_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head queue_entry;
	struct cpustc_camera_dma_lease *lease;
	u32 tag;
};

struct cpustc_camera {
	struct device *dev;
	void __iomem *regs;
	int irq;

	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct video_device vdev;
	struct media_pad video_pad;
	struct vb2_queue vb2_queue;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *sensor;
	int sensor_source_pad;

	struct mutex lock;
	spinlock_t queue_lock;
	struct list_head pending;
	struct list_head quarantined_leases;
	struct cpustc_camera_buffer *hw_buffer;
	wait_queue_head_t stop_wait;

	u32 next_tag;
	u32 sequence;
	bool streaming;
	bool stopping;
	bool poisoned;
	bool video_registered;
	bool video_entity_initialized;
	bool media_registered;
};

static inline struct cpustc_camera_buffer *to_cpustc_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb),
			    struct cpustc_camera_buffer, vb);
}

static inline u32 cpustc_camera_read(struct cpustc_camera *camera, u32 reg)
{
	return readl(camera->regs + reg);
}

static inline void cpustc_camera_write(struct cpustc_camera *camera,
				       u32 reg, u32 value)
{
	writel(value, camera->regs + reg);
}

static unsigned long cpustc_camera_contiguous_size(struct sg_table *sgt)
{
	struct scatterlist *sg;
	u64 expected = sg_dma_address(sgt->sgl);
	unsigned int i;
	unsigned long size = 0;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		if (sg_dma_address(sg) != expected)
			break;
		expected += sg_dma_len(sg);
		size += sg_dma_len(sg);
	}

	return size;
}

static void cpustc_camera_sync_for_device(struct cpustc_camera *camera,
					  struct cpustc_camera_dma_lease *lease)
{
	if (WARN_ON(lease->device_owned))
		return;

	dma_sync_sgtable_for_device(camera->dev, lease->sgt, DMA_FROM_DEVICE);
	lease->device_owned = true;
}

static void cpustc_camera_sync_for_cpu(struct cpustc_camera *camera,
				       struct cpustc_camera_dma_lease *lease)
{
	if (!lease || !lease->device_owned)
		return;

	dma_sync_sgtable_for_cpu(camera->dev, lease->sgt, DMA_FROM_DEVICE);
	lease->device_owned = false;
}

static void cpustc_camera_release_lease(struct cpustc_camera *camera,
					struct cpustc_camera_dma_lease *lease)
{
	if (!lease)
		return;
	if (WARN_ON(lease->device_owned)) {
		dev_crit(camera->dev,
			 "refusing to release a DMA mapping still owned by hardware\n");
		return;
	}

	if (lease->sgt)
		dma_buf_unmap_attachment(lease->attachment, lease->sgt,
					 DMA_FROM_DEVICE);
	if (lease->attachment)
		dma_buf_detach(lease->dmabuf, lease->attachment);
	if (lease->dmabuf)
		dma_buf_put(lease->dmabuf);
	kfree(lease);
}

static int cpustc_camera_acquire_lease(struct cpustc_camera *camera,
				       struct vb2_buffer *vb,
				       struct cpustc_camera_buffer *buffer)
{
	struct cpustc_camera_dma_lease *lease;
	struct dma_buf *dmabuf;
	unsigned long contiguous;
	u64 end;

	if (buffer->lease)
		return 0;

	lease = kzalloc(sizeof(*lease), GFP_KERNEL);
	if (!lease)
		return -ENOMEM;

	INIT_LIST_HEAD(&lease->quarantine_entry);

	if (vb->memory == VB2_MEMORY_DMABUF) {
		dmabuf = vb->planes[0].dbuf;
		if (!dmabuf) {
			kfree(lease);
			return -EINVAL;
		}
		get_dma_buf(dmabuf);
	} else {
		dmabuf = vb->vb2_queue->mem_ops->get_dmabuf(
			vb->planes[0].mem_priv, O_RDWR);
		if (IS_ERR_OR_NULL(dmabuf)) {
			kfree(lease);
			return dmabuf ? PTR_ERR(dmabuf) : -EINVAL;
		}
	}
	lease->dmabuf = dmabuf;

	lease->attachment = dma_buf_attach(dmabuf, camera->dev);
	if (IS_ERR(lease->attachment)) {
		int ret = PTR_ERR(lease->attachment);

		lease->attachment = NULL;
		cpustc_camera_release_lease(camera, lease);
		return ret;
	}

	lease->sgt = dma_buf_map_attachment(lease->attachment, DMA_FROM_DEVICE);
	if (IS_ERR(lease->sgt)) {
		int ret = PTR_ERR(lease->sgt);

		lease->sgt = NULL;
		cpustc_camera_release_lease(camera, lease);
		return ret;
	}

	contiguous = cpustc_camera_contiguous_size(lease->sgt);
	lease->dma_addr = sg_dma_address(lease->sgt->sgl);
	end = (u64)lease->dma_addr + CPUSTC_CAM_FRAME_BYTES_VALUE - 1;
	if (contiguous < CPUSTC_CAM_FRAME_BYTES_VALUE ||
	    !IS_ALIGNED(lease->dma_addr, CPUSTC_CAM_ADDR_ALIGN) ||
	    upper_32_bits(lease->dma_addr) || upper_32_bits(end)) {
		dev_err(camera->dev,
			"buffer %u is not a 64-byte-aligned contiguous 32-bit DMA buffer\n",
			vb->index);
		cpustc_camera_release_lease(camera, lease);
		return -EINVAL;
	}

	buffer->lease = lease;
	return 0;
}

static void cpustc_camera_quarantine_buffer(struct cpustc_camera *camera,
						    struct cpustc_camera_buffer *buffer)
{
	if (!buffer || !buffer->lease)
		return;

	list_add_tail(&buffer->lease->quarantine_entry,
		      &camera->quarantined_leases);
	buffer->lease = NULL;
	dev_crit(camera->dev,
			 "DMA descriptor did not drain; retaining its buffer mapping until reboot\n");
}

static void cpustc_camera_submit_locked(struct cpustc_camera *camera)
{
	struct cpustc_camera_buffer *buffer;

	if (!camera->streaming || camera->stopping || camera->poisoned ||
	    camera->hw_buffer || list_empty(&camera->pending))
		return;

	if (cpustc_camera_read(camera, CPUSTC_CAM_QUEUE_COUNT) != 0) {
		camera->poisoned = true;
		dev_err(camera->dev,
			"descriptor FIFO unexpectedly non-empty; capture disabled until reboot\n");
		return;
	}

	buffer = list_first_entry(&camera->pending,
				  struct cpustc_camera_buffer, queue_entry);
	list_del_init(&buffer->queue_entry);
	buffer->tag = ++camera->next_tag;
	if (!buffer->tag)
		buffer->tag = ++camera->next_tag;
	camera->hw_buffer = buffer;
	cpustc_camera_sync_for_device(camera, buffer->lease);

	cpustc_camera_write(camera, CPUSTC_CAM_QUEUE_ADDR,
			    lower_32_bits(buffer->lease->dma_addr));
	cpustc_camera_write(camera, CPUSTC_CAM_QUEUE_TAG, buffer->tag);
	cpustc_camera_write(camera, CPUSTC_CAM_QUEUE_PUSH, 1);
}

static void cpustc_camera_complete(struct cpustc_camera *camera, u32 tag,
				   u32 status, u32 bytes)
{
	struct cpustc_camera_buffer *buffer = NULL;
	unsigned long flags;

	spin_lock_irqsave(&camera->queue_lock, flags);
	if (camera->hw_buffer && camera->hw_buffer->tag == tag) {
		buffer = camera->hw_buffer;
		camera->hw_buffer = NULL;
		cpustc_camera_submit_locked(camera);
	}
	spin_unlock_irqrestore(&camera->queue_lock, flags);

	if (!buffer) {
		dev_err_ratelimited(camera->dev,
			"unexpected completion tag %#x status %#x bytes %u\n",
			tag, status, bytes);
		wake_up_all(&camera->stop_wait);
		return;
	}
	cpustc_camera_sync_for_cpu(camera, buffer->lease);

	buffer->vb.sequence = camera->sequence++;
	buffer->vb.field = V4L2_FIELD_NONE;
	buffer->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_set_plane_payload(&buffer->vb.vb2_buf, 0,
			      min_t(u32, bytes, CPUSTC_CAM_FRAME_BYTES_VALUE));

	if (status || bytes != CPUSTC_CAM_FRAME_BYTES_VALUE) {
		dev_warn_ratelimited(camera->dev,
			"frame %u failed: status=%#x bytes=%u\n",
			buffer->vb.sequence, status, bytes);
		vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	} else {
		vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}

	wake_up_all(&camera->stop_wait);
}

static void cpustc_camera_drain_completions(struct cpustc_camera *camera)
{
	while (cpustc_camera_read(camera, CPUSTC_CAM_DONE_COUNT)) {
		u32 tag = cpustc_camera_read(camera, CPUSTC_CAM_DONE_TAG);
		u32 status = cpustc_camera_read(camera, CPUSTC_CAM_DONE_STATUS);
		u32 bytes = cpustc_camera_read(camera, CPUSTC_CAM_DONE_BYTES);

		cpustc_camera_write(camera, CPUSTC_CAM_DONE_POP, 1);
		cpustc_camera_complete(camera, tag, status, bytes);
	}
}

static irqreturn_t cpustc_camera_irq_thread(int irq, void *data)
{
	struct cpustc_camera *camera = data;
	u32 irq_status = cpustc_camera_read(camera, CPUSTC_CAM_IRQ_STATUS);

	if (!(irq_status & CPUSTC_CAM_IRQ_ALL))
		return IRQ_NONE;

	cpustc_camera_drain_completions(camera);
	if (irq_status & CPUSTC_CAM_IRQ_STICKY)
		cpustc_camera_write(camera, CPUSTC_CAM_IRQ_STATUS,
				    irq_status & CPUSTC_CAM_IRQ_STICKY);

	return IRQ_HANDLED;
}

static int cpustc_camera_queue_setup(struct vb2_queue *vq,
				     unsigned int *num_buffers,
				     unsigned int *num_planes,
				     unsigned int sizes[],
				     struct device *alloc_devs[])
{
	if (*num_planes) {
		if (*num_planes != 1 || sizes[0] < CPUSTC_CAM_FRAME_BYTES_VALUE)
			return -EINVAL;
	} else {
		*num_planes = 1;
		sizes[0] = CPUSTC_CAM_FRAME_BYTES_VALUE;
	}

	if (*num_buffers < 2)
		*num_buffers = 2;

	return 0;
}

static int cpustc_camera_buffer_init(struct vb2_buffer *vb)
{
	struct cpustc_camera_buffer *buffer = to_cpustc_buffer(vb);

	INIT_LIST_HEAD(&buffer->queue_entry);
	return 0;
}

static int cpustc_camera_buffer_prepare(struct vb2_buffer *vb)
{
	struct cpustc_camera *camera = vb2_get_drv_priv(vb->vb2_queue);
	struct cpustc_camera_buffer *buffer = to_cpustc_buffer(vb);
	int ret;

	if (vb2_plane_size(vb, 0) < CPUSTC_CAM_FRAME_BYTES_VALUE)
		return -EINVAL;

	ret = cpustc_camera_acquire_lease(camera, vb, buffer);
	if (ret)
		return ret;

	vb2_set_plane_payload(vb, 0, CPUSTC_CAM_FRAME_BYTES_VALUE);
	return 0;
}

static void cpustc_camera_buffer_cleanup(struct vb2_buffer *vb)
{
	struct cpustc_camera *camera = vb2_get_drv_priv(vb->vb2_queue);
	struct cpustc_camera_buffer *buffer = to_cpustc_buffer(vb);
	unsigned long flags;

	WARN_ON(!list_empty(&buffer->queue_entry));
	WARN_ON(READ_ONCE(camera->hw_buffer) == buffer);
	if (buffer->lease && WARN_ON(buffer->lease->device_owned)) {
		spin_lock_irqsave(&camera->queue_lock, flags);
		cpustc_camera_quarantine_buffer(camera, buffer);
		spin_unlock_irqrestore(&camera->queue_lock, flags);
		return;
	}

	cpustc_camera_release_lease(camera, buffer->lease);
	buffer->lease = NULL;
}

static void cpustc_camera_buffer_queue(struct vb2_buffer *vb)
{
	struct cpustc_camera *camera = vb2_get_drv_priv(vb->vb2_queue);
	struct cpustc_camera_buffer *buffer = to_cpustc_buffer(vb);
	unsigned long flags;

	spin_lock_irqsave(&camera->queue_lock, flags);
	list_add_tail(&buffer->queue_entry, &camera->pending);
	cpustc_camera_submit_locked(camera);
	spin_unlock_irqrestore(&camera->queue_lock, flags);
}

static void cpustc_camera_return_pending(struct cpustc_camera *camera,
					 enum vb2_buffer_state state)
{
	LIST_HEAD(done);
	struct cpustc_camera_buffer *buffer;
	struct cpustc_camera_buffer *tmp;
	unsigned long flags;

	spin_lock_irqsave(&camera->queue_lock, flags);
	list_splice_init(&camera->pending, &done);
	if (camera->hw_buffer) {
		list_add_tail(&camera->hw_buffer->queue_entry, &done);
		camera->hw_buffer = NULL;
	}
	spin_unlock_irqrestore(&camera->queue_lock, flags);

	list_for_each_entry_safe(buffer, tmp, &done, queue_entry) {
		list_del_init(&buffer->queue_entry);
		cpustc_camera_sync_for_cpu(camera, buffer->lease);
		vb2_buffer_done(&buffer->vb.vb2_buf, state);
	}

	wake_up_all(&camera->stop_wait);
}

static int cpustc_camera_start_streaming(struct vb2_queue *vq,
					 unsigned int count)
{
	struct cpustc_camera *camera = vb2_get_drv_priv(vq);
	unsigned long flags;
	u32 status;
	int ret;

	if (camera->poisoned) {
		ret = -EIO;
		goto fail_buffers;
	}

	ret = media_pipeline_start(&camera->vdev.entity, &camera->vdev.pipe);
	if (ret)
		goto fail_buffers;

	status = cpustc_camera_read(camera, CPUSTC_CAM_STATUS);
	if ((status & (CPUSTC_CAM_STATUS_QUEUED | CPUSTC_CAM_STATUS_BUSY)) ||
	    cpustc_camera_read(camera, CPUSTC_CAM_QUEUE_COUNT) ||
	    cpustc_camera_read(camera, CPUSTC_CAM_DONE_COUNT)) {
		dev_err(camera->dev, "capture hardware is not empty at STREAMON\n");
		camera->poisoned = true;
		ret = -EIO;
		goto stop_pipeline;
	}

	ret = v4l2_subdev_call(camera->sensor, video, s_stream, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		goto stop_pipeline;

	camera->sequence = 0;
	camera->stopping = false;
	camera->streaming = true;
	cpustc_camera_write(camera, CPUSTC_CAM_IRQ_STATUS,
			    CPUSTC_CAM_IRQ_STICKY);
	cpustc_camera_write(camera, CPUSTC_CAM_IRQ_ENABLE, CPUSTC_CAM_IRQ_ALL);
	cpustc_camera_write(camera, CPUSTC_CAM_CONTROL,
			    CPUSTC_CAM_CONTROL_ENABLE);

	spin_lock_irqsave(&camera->queue_lock, flags);
	cpustc_camera_submit_locked(camera);
	spin_unlock_irqrestore(&camera->queue_lock, flags);

	return 0;

stop_pipeline:
	media_pipeline_stop(&camera->vdev.entity);
fail_buffers:
	cpustc_camera_return_pending(camera, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void cpustc_camera_stop_streaming(struct vb2_queue *vq)
{
	struct cpustc_camera *camera = vb2_get_drv_priv(vq);
	unsigned long flags;
	u32 hw_status;
	u32 queue_count;
	u32 value;
	bool descriptor_stuck;
	bool dma_stuck = false;
	long waited;

	spin_lock_irqsave(&camera->queue_lock, flags);
	camera->stopping = true;
	spin_unlock_irqrestore(&camera->queue_lock, flags);

	waited = wait_event_timeout(camera->stop_wait,
				    !READ_ONCE(camera->hw_buffer),
				    msecs_to_jiffies(CPUSTC_CAM_DRAIN_TIMEOUT_MS));

	disable_irq(camera->irq);
	cpustc_camera_drain_completions(camera);

	if (!waited && READ_ONCE(camera->hw_buffer)) {
		/* Stop accepting a queued descriptor, or abort the active frame. */
		cpustc_camera_write(camera, CPUSTC_CAM_ABORT, 1);
		cpustc_camera_write(camera, CPUSTC_CAM_CONTROL, 0);
		if (readl_poll_timeout(camera->regs + CPUSTC_CAM_STATUS, value,
				       !(value & CPUSTC_CAM_STATUS_BUSY), 10,
				       CPUSTC_CAM_ABORT_TIMEOUT_US))
			dma_stuck = true;
		cpustc_camera_drain_completions(camera);
	}

	cpustc_camera_write(camera, CPUSTC_CAM_CONTROL, 0);
	cpustc_camera_write(camera, CPUSTC_CAM_IRQ_ENABLE, 0);
	hw_status = cpustc_camera_read(camera, CPUSTC_CAM_STATUS);
	queue_count = cpustc_camera_read(camera, CPUSTC_CAM_QUEUE_COUNT);
	descriptor_stuck = READ_ONCE(camera->hw_buffer) || queue_count ||
		(hw_status & (CPUSTC_CAM_STATUS_QUEUED |
			      CPUSTC_CAM_STATUS_BUSY)) || dma_stuck;

	if (descriptor_stuck) {
		camera->poisoned = true;
		dev_err(camera->dev,
			"STREAMOFF left hardware state status=%#x queued=%u; capture cannot restart before reset\n",
			hw_status, queue_count);
	}

	spin_lock_irqsave(&camera->queue_lock, flags);
	if (camera->hw_buffer && descriptor_stuck)
		cpustc_camera_quarantine_buffer(camera, camera->hw_buffer);
	camera->streaming = false;
	spin_unlock_irqrestore(&camera->queue_lock, flags);

	cpustc_camera_return_pending(camera, VB2_BUF_STATE_ERROR);
	enable_irq(camera->irq);

	if (v4l2_subdev_call(camera->sensor, video, s_stream, 0) < 0)
		dev_dbg(camera->dev, "sensor has no stream-off operation\n");
	media_pipeline_stop(&camera->vdev.entity);
}

static const struct vb2_ops cpustc_camera_vb2_ops = {
	.queue_setup = cpustc_camera_queue_setup,
	.buf_init = cpustc_camera_buffer_init,
	.buf_prepare = cpustc_camera_buffer_prepare,
	.buf_cleanup = cpustc_camera_buffer_cleanup,
	.buf_queue = cpustc_camera_buffer_queue,
	.start_streaming = cpustc_camera_start_streaming,
	.stop_streaming = cpustc_camera_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static void cpustc_camera_fill_format(struct v4l2_pix_format *pix)
{
	pix->width = CPUSTC_CAM_WIDTH_VALUE;
	pix->height = CPUSTC_CAM_HEIGHT_VALUE;
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = CPUSTC_CAM_BYTES_PER_LINE_VALUE;
	pix->sizeimage = CPUSTC_CAM_FRAME_BYTES_VALUE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
}

static int cpustc_camera_querycap(struct file *file, void *priv,
				  struct v4l2_capability *cap)
{
	struct cpustc_camera *camera = video_drvdata(file);

	strscpy(cap->driver, "cpustc-camera", sizeof(cap->driver));
	strscpy(cap->card, "CPUSTC OV7670 Camera", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(camera->dev));
	return 0;
}

static int cpustc_camera_enum_format(struct file *file, void *priv,
				     struct v4l2_fmtdesc *fmt)
{
	if (fmt->index)
		return -EINVAL;

	fmt->pixelformat = V4L2_PIX_FMT_RGB565;
	return 0;
}

static int cpustc_camera_get_format(struct file *file, void *priv,
				    struct v4l2_format *format)
{
	cpustc_camera_fill_format(&format->fmt.pix);
	return 0;
}

static int cpustc_camera_try_format(struct file *file, void *priv,
				    struct v4l2_format *format)
{
	cpustc_camera_fill_format(&format->fmt.pix);
	return 0;
}

static int cpustc_camera_set_format(struct file *file, void *priv,
				    struct v4l2_format *format)
{
	struct cpustc_camera *camera = video_drvdata(file);

	if (vb2_is_busy(&camera->vb2_queue))
		return -EBUSY;

	cpustc_camera_fill_format(&format->fmt.pix);
	return 0;
}

static int cpustc_camera_enum_framesizes(struct file *file, void *priv,
					 struct v4l2_frmsizeenum *size)
{
	if (size->index || size->pixel_format != V4L2_PIX_FMT_RGB565)
		return -EINVAL;

	size->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	size->discrete.width = CPUSTC_CAM_WIDTH_VALUE;
	size->discrete.height = CPUSTC_CAM_HEIGHT_VALUE;
	return 0;
}

static int cpustc_camera_enum_frameintervals(struct file *file, void *priv,
					     struct v4l2_frmivalenum *ival)
{
	if (ival->index >= CPUSTC_CAM_FRAME_RATE_MAX ||
	    ival->pixel_format != V4L2_PIX_FMT_RGB565 ||
	    ival->width != CPUSTC_CAM_WIDTH_VALUE ||
	    ival->height != CPUSTC_CAM_HEIGHT_VALUE)
		return -EINVAL;

	ival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	ival->discrete.numerator = CPUSTC_CAM_FRAME_INTERVAL_NUM;
	ival->discrete.denominator = CPUSTC_CAM_FRAME_RATE_MIN + ival->index;
	return 0;
}

static int cpustc_camera_get_parm(struct file *file, void *priv,
				  struct v4l2_streamparm *parm)
{
	struct cpustc_camera *camera = video_drvdata(file);

	return v4l2_g_parm_cap(video_devdata(file), camera->sensor, parm);
}

static int cpustc_camera_set_parm(struct file *file, void *priv,
				  struct v4l2_streamparm *parm)
{
	struct cpustc_camera *camera = video_drvdata(file);
	struct v4l2_fract *interval =
		&parm->parm.capture.timeperframe;
	unsigned int frame_rate;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    !interval->numerator ||
	    interval->denominator % interval->numerator)
		return -EINVAL;

	frame_rate = interval->denominator / interval->numerator;
	if (frame_rate < CPUSTC_CAM_FRAME_RATE_MIN ||
	    frame_rate > CPUSTC_CAM_FRAME_RATE_MAX)
		return -EINVAL;

	interval->numerator = CPUSTC_CAM_FRAME_INTERVAL_NUM;
	interval->denominator = frame_rate;

	return v4l2_s_parm_cap(video_devdata(file), camera->sensor, parm);
}

static int cpustc_camera_enum_input(struct file *file, void *priv,
				    struct v4l2_input *input)
{
	if (input->index)
		return -EINVAL;

	strscpy(input->name, "OV7670", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
	return 0;
}

static int cpustc_camera_get_input(struct file *file, void *priv,
				   unsigned int *input)
{
	*input = 0;
	return 0;
}

static int cpustc_camera_set_input(struct file *file, void *priv,
				   unsigned int input)
{
	return input ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops cpustc_camera_ioctl_ops = {
	.vidioc_querycap = cpustc_camera_querycap,
	.vidioc_enum_fmt_vid_cap = cpustc_camera_enum_format,
	.vidioc_g_fmt_vid_cap = cpustc_camera_get_format,
	.vidioc_try_fmt_vid_cap = cpustc_camera_try_format,
	.vidioc_s_fmt_vid_cap = cpustc_camera_set_format,
	.vidioc_enum_framesizes = cpustc_camera_enum_framesizes,
	.vidioc_enum_frameintervals = cpustc_camera_enum_frameintervals,
	.vidioc_g_parm = cpustc_camera_get_parm,
	.vidioc_s_parm = cpustc_camera_set_parm,
	.vidioc_enum_input = cpustc_camera_enum_input,
	.vidioc_g_input = cpustc_camera_get_input,
	.vidioc_s_input = cpustc_camera_set_input,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static int cpustc_camera_apply_sensor_format(struct cpustc_camera *camera)
{
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = 0,
		.format = {
			.width = CPUSTC_CAM_WIDTH_VALUE,
			.height = CPUSTC_CAM_HEIGHT_VALUE,
			.code = MEDIA_BUS_FMT_RGB565_2X8_LE,
			.field = V4L2_FIELD_NONE,
			.colorspace = V4L2_COLORSPACE_SRGB,
		},
	};
	struct v4l2_subdev_frame_interval interval = {
		.pad = 0,
		.interval = {
			.numerator = CPUSTC_CAM_FRAME_INTERVAL_NUM,
			.denominator = CPUSTC_CAM_FRAME_INTERVAL_DEN,
		},
	};
	int ret;

	ret = v4l2_subdev_call(camera->sensor, pad, set_fmt, NULL, &format);
	if (ret)
		return ret;
	if (format.format.width != CPUSTC_CAM_WIDTH_VALUE ||
	    format.format.height != CPUSTC_CAM_HEIGHT_VALUE ||
	    format.format.code != MEDIA_BUS_FMT_RGB565_2X8_LE)
		return -EINVAL;

	ret = v4l2_subdev_call(camera->sensor, video, s_frame_interval,
			       &interval);
	if (ret)
		return ret;
	if (interval.interval.numerator != CPUSTC_CAM_FRAME_INTERVAL_NUM ||
	    interval.interval.denominator != CPUSTC_CAM_FRAME_INTERVAL_DEN) {
		dev_err(camera->dev, "sensor cannot provide 5 fps exactly\n");
		return -EINVAL;
	}

	return 0;
}

static int cpustc_camera_open(struct file *file)
{
	struct cpustc_camera *camera = video_drvdata(file);
	int ret;

	if (mutex_lock_interruptible(&camera->lock))
		return -ERESTARTSYS;
	if (camera->poisoned) {
		ret = -EIO;
		goto unlock;
	}

	ret = v4l2_fh_open(file);
	if (ret)
		goto unlock;
	if (!v4l2_fh_is_singular_file(file))
		goto unlock;

	ret = v4l2_subdev_call(camera->sensor, core, s_power, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		goto release_fh;

	ret = cpustc_camera_apply_sensor_format(camera);
	if (ret) {
		v4l2_subdev_call(camera->sensor, core, s_power, 0);
		goto release_fh;
	}

	mutex_unlock(&camera->lock);
	return 0;

release_fh:
	v4l2_fh_release(file);
unlock:
	mutex_unlock(&camera->lock);
	return ret;
}

static int cpustc_camera_release(struct file *file)
{
	struct cpustc_camera *camera = video_drvdata(file);
	bool singular;
	int ret;

	mutex_lock(&camera->lock);
	singular = v4l2_fh_is_singular_file(file);
	ret = _vb2_fop_release(file, NULL);
	if (singular)
		v4l2_subdev_call(camera->sensor, core, s_power, 0);
	mutex_unlock(&camera->lock);

	return ret;
}

static const struct v4l2_file_operations cpustc_camera_fops = {
	.owner = THIS_MODULE,
	.open = cpustc_camera_open,
	.release = cpustc_camera_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int cpustc_camera_register_video(struct cpustc_camera *camera)
{
	struct vb2_queue *queue = &camera->vb2_queue;
	struct video_device *vdev = &camera->vdev;
	int ret;

	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_DMABUF;
	queue->drv_priv = camera;
	queue->ops = &cpustc_camera_vb2_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->buf_struct_size = sizeof(struct cpustc_camera_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->min_buffers_needed = 2;
	queue->lock = &camera->lock;
	queue->dev = camera->dev;
	queue->dma_dir = DMA_FROM_DEVICE;

	ret = vb2_queue_init(queue);
	if (ret)
		return ret;

	strscpy(vdev->name, "cpustc-camera", sizeof(vdev->name));
	vdev->fops = &cpustc_camera_fops;
	vdev->ioctl_ops = &cpustc_camera_ioctl_ops;
	vdev->v4l2_dev = &camera->v4l2_dev;
	vdev->queue = queue;
	vdev->lock = &camera->lock;
	vdev->ctrl_handler = camera->sensor->ctrl_handler;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vdev->vfl_dir = VFL_DIR_RX;
	video_set_drvdata(vdev, camera);
	camera->video_pad.flags = MEDIA_PAD_FL_SINK |
				  MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &camera->video_pad);
	if (ret)
		return ret;
	camera->video_entity_initialized = true;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		media_entity_cleanup(&vdev->entity);
		camera->video_entity_initialized = false;
		return ret;
	}

	camera->video_registered = true;
	dev_info(camera->dev, "registered %s: RGB565 640x480 at 5 fps\n",
		 video_device_node_name(vdev));
	return 0;
}

static void cpustc_camera_unregister_video(struct cpustc_camera *camera)
{
	if (camera->video_registered) {
		vb2_video_unregister_device(&camera->vdev);
		camera->video_registered = false;
	}
	if (camera->media_registered) {
		media_device_unregister(&camera->mdev);
		camera->media_registered = false;
	}
	if (camera->video_entity_initialized) {
		media_entity_cleanup(&camera->vdev.entity);
		camera->video_entity_initialized = false;
	}
}

static int cpustc_camera_notify_bound(struct v4l2_async_notifier *notifier,
				      struct v4l2_subdev *subdev,
				      struct v4l2_async_subdev *asd)
{
	struct cpustc_camera *camera =
		container_of(notifier, struct cpustc_camera, notifier);
	int source_pad;

	source_pad = media_entity_get_fwnode_pad(&subdev->entity,
					     subdev->fwnode,
					     MEDIA_PAD_FL_SOURCE);
	if (source_pad < 0)
		return source_pad;

	camera->sensor = subdev;
	camera->sensor_source_pad = source_pad;
	return 0;
}

static int cpustc_camera_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct cpustc_camera *camera =
		container_of(notifier, struct cpustc_camera, notifier);
	int ret;

	ret = cpustc_camera_register_video(camera);
	if (ret)
		return ret;

	ret = media_create_pad_link(&camera->sensor->entity,
				    camera->sensor_source_pad,
				    &camera->vdev.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto unregister_video;

	ret = media_device_register(&camera->mdev);
	if (ret)
		goto unregister_video;
	camera->media_registered = true;

	return 0;

unregister_video:
	cpustc_camera_unregister_video(camera);
	return ret;
}

static void cpustc_camera_notify_unbind(struct v4l2_async_notifier *notifier,
					struct v4l2_subdev *subdev,
					struct v4l2_async_subdev *asd)
{
	struct cpustc_camera *camera =
		container_of(notifier, struct cpustc_camera, notifier);

	cpustc_camera_unregister_video(camera);
	camera->sensor = NULL;
}

static const struct v4l2_async_notifier_operations cpustc_camera_notify_ops = {
	.bound = cpustc_camera_notify_bound,
	.complete = cpustc_camera_notify_complete,
	.unbind = cpustc_camera_notify_unbind,
};

static int cpustc_camera_check_endpoint(struct device *dev)
{
	struct v4l2_fwnode_endpoint endpoint = {
		.bus_type = V4L2_MBUS_PARALLEL,
	};
	struct device_node *node;
	u32 required;
	int ret;

	node = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!node)
		return -ENODEV;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &endpoint);
	of_node_put(node);
	if (ret)
		return ret;

	required = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		   V4L2_MBUS_VSYNC_ACTIVE_HIGH |
		   V4L2_MBUS_PCLK_SAMPLE_RISING;
	if (endpoint.bus_type != V4L2_MBUS_PARALLEL ||
	    endpoint.bus.parallel.bus_width != 8 ||
	    (endpoint.bus.parallel.flags & required) != required) {
		dev_err(dev, "requires 8-bit active-high HREF/VSYNC and rising-edge PCLK\n");
		return -EINVAL;
	}

	return 0;
}

static int cpustc_camera_check_hardware(struct cpustc_camera *camera)
{
	u32 status;

	cpustc_camera_write(camera, CPUSTC_CAM_IRQ_ENABLE, 0);
	cpustc_camera_write(camera, CPUSTC_CAM_CONTROL, 0);

	if (cpustc_camera_read(camera, CPUSTC_CAM_ID) != CPUSTC_CAM_ID_VALUE)
		return -ENODEV;
	if ((cpustc_camera_read(camera, CPUSTC_CAM_VERSION) & 0xffff0000) !=
	    CPUSTC_CAM_VERSION_MAJOR)
		return -EINVAL;
	if (!(cpustc_camera_read(camera, CPUSTC_CAM_CAPABILITIES) & BIT(0)) ||
	    cpustc_camera_read(camera, CPUSTC_CAM_FORMAT) != V4L2_PIX_FMT_RGB565 ||
	    cpustc_camera_read(camera, CPUSTC_CAM_WIDTH) != CPUSTC_CAM_WIDTH_VALUE ||
	    cpustc_camera_read(camera, CPUSTC_CAM_HEIGHT) != CPUSTC_CAM_HEIGHT_VALUE ||
	    cpustc_camera_read(camera, CPUSTC_CAM_BYTES_PER_LINE) !=
		CPUSTC_CAM_BYTES_PER_LINE_VALUE ||
	    cpustc_camera_read(camera, CPUSTC_CAM_FRAME_BYTES) !=
		CPUSTC_CAM_FRAME_BYTES_VALUE)
		return -EINVAL;

	status = cpustc_camera_read(camera, CPUSTC_CAM_STATUS);
	if ((status & (CPUSTC_CAM_STATUS_QUEUED | CPUSTC_CAM_STATUS_BUSY)) ||
	    cpustc_camera_read(camera, CPUSTC_CAM_QUEUE_COUNT)) {
		dev_err(camera->dev,
			"residual DMA descriptor detected; refusing to enable capture until hardware reset\n");
		return -EBUSY;
	}

	while (cpustc_camera_read(camera, CPUSTC_CAM_DONE_COUNT))
		cpustc_camera_write(camera, CPUSTC_CAM_DONE_POP, 1);
	cpustc_camera_write(camera, CPUSTC_CAM_IRQ_STATUS,
			    CPUSTC_CAM_IRQ_STICKY);

	return 0;
}

static int cpustc_camera_probe(struct platform_device *pdev)
{
	struct cpustc_camera *camera;
	struct v4l2_async_subdev *asd;
	struct device_node *endpoint;
	int ret;

	camera = devm_kzalloc(&pdev->dev, sizeof(*camera), GFP_KERNEL);
	if (!camera)
		return -ENOMEM;

	camera->dev = &pdev->dev;
	platform_set_drvdata(pdev, camera);
	mutex_init(&camera->lock);
	spin_lock_init(&camera->queue_lock);
	INIT_LIST_HEAD(&camera->pending);
	INIT_LIST_HEAD(&camera->quarantined_leases);
	init_waitqueue_head(&camera->stop_wait);
	camera->sensor_source_pad = -1;

	camera->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(camera->regs))
		return PTR_ERR(camera->regs);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = cpustc_camera_check_endpoint(&pdev->dev);
	if (ret)
		return ret;

	ret = cpustc_camera_check_hardware(camera);
	if (ret)
		return ret;

	camera->irq = platform_get_irq(pdev, 0);
	if (camera->irq < 0)
		return camera->irq;

	ret = devm_request_threaded_irq(&pdev->dev, camera->irq, NULL,
					 cpustc_camera_irq_thread, IRQF_ONESHOT,
					 dev_name(&pdev->dev), camera);
	if (ret)
		return ret;

	camera->mdev.dev = &pdev->dev;
	strscpy(camera->mdev.model, "CPUSTC OV7670 Camera",
		sizeof(camera->mdev.model));
	snprintf(camera->mdev.bus_info, sizeof(camera->mdev.bus_info),
		 "platform:%s", dev_name(&pdev->dev));
	media_device_init(&camera->mdev);
	camera->v4l2_dev.mdev = &camera->mdev;

	ret = v4l2_device_register(&pdev->dev, &camera->v4l2_dev);
	if (ret)
		goto cleanup_media;

	v4l2_async_notifier_init(&camera->notifier);
	endpoint = of_graph_get_next_endpoint(pdev->dev.of_node, NULL);
	if (!endpoint) {
		ret = -ENODEV;
		goto unregister_v4l2;
	}

	asd = v4l2_async_notifier_add_fwnode_remote_subdev(
		&camera->notifier, of_fwnode_handle(endpoint),
		struct v4l2_async_subdev);
	of_node_put(endpoint);
	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto cleanup_notifier;
	}

	camera->notifier.ops = &cpustc_camera_notify_ops;
	ret = v4l2_async_notifier_register(&camera->v4l2_dev,
					   &camera->notifier);
	if (ret)
		goto cleanup_notifier;

	dev_info(&pdev->dev, "CPUSTC camera capture controller ready\n");
	return 0;

cleanup_notifier:
	v4l2_async_notifier_cleanup(&camera->notifier);
unregister_v4l2:
	v4l2_device_unregister(&camera->v4l2_dev);

cleanup_media:
	media_device_cleanup(&camera->mdev);
	return ret;
}

static int cpustc_camera_remove(struct platform_device *pdev)
{
	struct cpustc_camera *camera = platform_get_drvdata(pdev);
	struct cpustc_camera_dma_lease *lease;
	struct cpustc_camera_dma_lease *tmp;
	u32 queue_count;
	u32 status;

	v4l2_async_notifier_unregister(&camera->notifier);
	v4l2_async_notifier_cleanup(&camera->notifier);
	cpustc_camera_unregister_video(camera);
	cpustc_camera_write(camera, CPUSTC_CAM_IRQ_ENABLE, 0);
	cpustc_camera_write(camera, CPUSTC_CAM_CONTROL, 0);
	v4l2_device_unregister(&camera->v4l2_dev);

	status = cpustc_camera_read(camera, CPUSTC_CAM_STATUS);
	queue_count = cpustc_camera_read(camera, CPUSTC_CAM_QUEUE_COUNT);
	if (queue_count ||
	    (status & (CPUSTC_CAM_STATUS_QUEUED | CPUSTC_CAM_STATUS_BUSY))) {
		dev_crit(camera->dev,
			 "driver removed with undrained DMA state status=%#x queued=%u; quarantined mappings intentionally retained\n",
			 status, queue_count);
		media_device_cleanup(&camera->mdev);
		return 0;
	}

	list_for_each_entry_safe(lease, tmp, &camera->quarantined_leases,
				 quarantine_entry) {
		list_del(&lease->quarantine_entry);
		cpustc_camera_sync_for_cpu(camera, lease);
		cpustc_camera_release_lease(camera, lease);
	}
	media_device_cleanup(&camera->mdev);

	return 0;
}

static const struct of_device_id cpustc_camera_of_match[] = {
	{ .compatible = "cpustc,camera-capture" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpustc_camera_of_match);

static struct platform_driver cpustc_camera_driver = {
	.probe = cpustc_camera_probe,
	.remove = cpustc_camera_remove,
	.driver = {
		.name = "cpustc-camera",
		.of_match_table = cpustc_camera_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(cpustc_camera_driver);

MODULE_DESCRIPTION("CPUSTC OV7670 V4L2 capture driver");
MODULE_AUTHOR("CPUSTC project");
MODULE_LICENSE("GPL");
