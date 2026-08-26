// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM/KMS driver for the CPUSTC NT35510 LCD controller.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_flip_work.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#define CPUSTC_LCD_CMD			0x00
#define CPUSTC_LCD_DATA			0x04
#define CPUSTC_LCD_CONTROL		0x08
#define CPUSTC_LCD_STATUS		0x0c
#define CPUSTC_LCD_DMA_BASE		0x10
#define CPUSTC_LCD_WRITE_TIMING		0x18
#define CPUSTC_LCD_IRQ_ENABLE		0x1c
#define CPUSTC_LCD_DMA_WIDTH		0x20
#define CPUSTC_LCD_DMA_HEIGHT		0x24
#define CPUSTC_LCD_DMA_SRC_STRIDE	0x28
#define CPUSTC_LCD_POWER_CONTROL	0x2c

#define CPUSTC_LCD_CONTROL_RESET_N	BIT(0)
#define CPUSTC_LCD_CONTROL_DMA_START	BIT(1)
#define CPUSTC_LCD_CONTROL_DMA_2D	BIT(2)

#define CPUSTC_LCD_STATUS_DMA_BUSY	BIT(1)
#define CPUSTC_LCD_STATUS_DMA_DONE	BIT(2)
#define CPUSTC_LCD_STATUS_DMA_ERROR	BIT(3)
#define CPUSTC_LCD_STATUS_CONTROL_CHANGED BIT(4)
#define CPUSTC_LCD_STATUS_RECOVERY_PENDING BIT(5)
#define CPUSTC_LCD_STATUS_RECOVERY_ACTIVE	BIT(6)
#define CPUSTC_LCD_STATUS_DISPLAY_TRANSITION BIT(7)
#define CPUSTC_LCD_STATUS_DMA_W1C	(CPUSTC_LCD_STATUS_DMA_DONE | \
					 CPUSTC_LCD_STATUS_DMA_ERROR)
#define CPUSTC_LCD_STATUS_EVENT_W1C	(CPUSTC_LCD_STATUS_DMA_W1C | \
					 CPUSTC_LCD_STATUS_CONTROL_CHANGED)

#define CPUSTC_LCD_IRQ_DMA_DONE		BIT(0)
#define CPUSTC_LCD_IRQ_CONTROL_CHANGED	BIT(1)
#define CPUSTC_LCD_IRQ_RECOVERY		BIT(2)
#define CPUSTC_LCD_IRQ_ALL		(CPUSTC_LCD_IRQ_DMA_DONE | \
					 CPUSTC_LCD_IRQ_CONTROL_CHANGED | \
					 CPUSTC_LCD_IRQ_RECOVERY)

#define CPUSTC_LCD_POWER_BACKLIGHT	BIT(0)
#define CPUSTC_LCD_POWER_DISPLAY		BIT(1)
#define CPUSTC_LCD_POWER_TOUCH		BIT(2)
#define CPUSTC_LCD_POWER_RECOVER		BIT(3)
#define CPUSTC_LCD_POWER_REQUEST_MASK	(CPUSTC_LCD_POWER_BACKLIGHT | \
					 CPUSTC_LCD_POWER_DISPLAY | \
					 CPUSTC_LCD_POWER_TOUCH)
#define CPUSTC_LCD_WRITE_TIMING_DEFAULT	0x00050503

#define CPUSTC_LCD_WIDTH		480
#define CPUSTC_LCD_HEIGHT		800
#define CPUSTC_LCD_BYTES_PER_PIXEL	2
#define CPUSTC_LCD_FRAME_BYTES		(CPUSTC_LCD_WIDTH * \
					 CPUSTC_LCD_HEIGHT * \
					 CPUSTC_LCD_BYTES_PER_PIXEL)
#define CPUSTC_LCD_LINE_BYTES		(CPUSTC_LCD_WIDTH * \
					 CPUSTC_LCD_BYTES_PER_PIXEL)
#define CPUSTC_LCD_CHECK_PERIOD_MS	20
#define CPUSTC_LCD_DMA_TIMEOUT_MS	500

struct nt35510_reg_value {
	u16 reg;
	u8 value;
};

struct cpustc_lcd_rect {
	u32 x1;
	u32 y1;
	u32 x2;
	u32 y2;
	bool valid;
};

struct cpustc_lcd {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;
	void __iomem *regs;
	struct device *dev;
	int irq;
	struct backlight_device *backlight;

	struct mutex update_lock;
	spinlock_t state_lock;
	struct delayed_work refresh_work;
	struct work_struct control_work;
	struct work_struct recovery_work;
	struct drm_flip_work fb_cleanup_work;
	bool dma_busy;
	bool hw_busy_observed;
	bool refresh_timer_started;
	bool refresh_pending;
	bool recovering;
	bool suspended;
	bool stopping;
	unsigned long dma_deadline;
	u32 power_requests;
	u32 resume_power_requests;
	struct cpustc_lcd_rect damage;
	u64 debug_damage_queued;
	u64 debug_periodic_checks;
	u64 debug_dma_started;
	u64 debug_dma_completed;
	u64 debug_missed_completions;
	u64 debug_dma_errors;
	u64 debug_busy_deferrals;
	u64 debug_dma_timeouts;
	u64 debug_recoveries;
	u64 debug_irq_count;
	u64 debug_irq_none;

	struct drm_framebuffer *pending_fb;
	dma_addr_t pending_address;
	struct drm_framebuffer *active_fb;
	struct drm_flip_task *active_fb_cleanup_task;
};

static const struct nt35510_reg_value nt35510_page1_power[] = {
	{ 0xF000, 0x55 }, { 0xF001, 0xAA }, { 0xF002, 0x52 },
	{ 0xF003, 0x08 }, { 0xF004, 0x01 },
	{ 0xB000, 0x0D }, { 0xB001, 0x0D }, { 0xB002, 0x0D },
	{ 0xB600, 0x34 }, { 0xB601, 0x34 }, { 0xB602, 0x34 },
	{ 0xB100, 0x0D }, { 0xB101, 0x0D }, { 0xB102, 0x0D },
	{ 0xB700, 0x34 }, { 0xB701, 0x34 }, { 0xB702, 0x34 },
	{ 0xB200, 0x00 }, { 0xB201, 0x00 }, { 0xB202, 0x00 },
	{ 0xB800, 0x24 }, { 0xB801, 0x24 }, { 0xB802, 0x24 },
	{ 0xBF00, 0x01 },
	{ 0xB300, 0x0F }, { 0xB301, 0x0F }, { 0xB302, 0x0F },
	{ 0xB900, 0x34 }, { 0xB901, 0x34 }, { 0xB902, 0x34 },
	{ 0xB500, 0x08 }, { 0xB501, 0x08 }, { 0xB502, 0x08 },
	{ 0xC200, 0x03 },
	{ 0xBA00, 0x24 }, { 0xBA01, 0x24 }, { 0xBA02, 0x24 },
	{ 0xBC00, 0x00 }, { 0xBC01, 0x78 }, { 0xBC02, 0x00 },
	{ 0xBD00, 0x00 }, { 0xBD01, 0x78 }, { 0xBD02, 0x00 },
	{ 0xBE00, 0x00 }, { 0xBE01, 0x64 },
};

static const u16 nt35510_gamma_base[] = {
	0xD100, 0xD200, 0xD300, 0xD400, 0xD500, 0xD600,
};

static const u8 nt35510_gamma[] = {
	0x00, 0x33, 0x00, 0x34, 0x00, 0x3A, 0x00, 0x4A,
	0x00, 0x5C, 0x00, 0x81, 0x00, 0xA6, 0x00, 0xE5,
	0x01, 0x13, 0x01, 0x54, 0x01, 0x82, 0x01, 0xCA,
	0x02, 0x00, 0x02, 0x01, 0x02, 0x34, 0x02, 0x67,
	0x02, 0x84, 0x02, 0xA4, 0x02, 0xB7, 0x02, 0xCF,
	0x02, 0xDE, 0x02, 0xF2, 0x02, 0xFE, 0x03, 0x10,
	0x03, 0x33, 0x03, 0x6D,
};

static const struct nt35510_reg_value nt35510_page0_display[] = {
	{ 0xF000, 0x55 }, { 0xF001, 0xAA }, { 0xF002, 0x52 },
	{ 0xF003, 0x08 }, { 0xF004, 0x00 },
	{ 0xB100, 0xCC }, { 0xB101, 0x00 },
	{ 0xB600, 0x05 },
	{ 0xB700, 0x70 }, { 0xB701, 0x70 },
	{ 0xB800, 0x01 }, { 0xB801, 0x03 },
	{ 0xB802, 0x03 }, { 0xB803, 0x03 },
	{ 0xBC00, 0x02 }, { 0xBC01, 0x00 }, { 0xBC02, 0x00 },
	{ 0xC900, 0xD0 }, { 0xC901, 0x02 }, { 0xC902, 0x50 },
	{ 0xC903, 0x50 }, { 0xC904, 0x50 },
	{ 0x3500, 0x00 },
	{ 0x3A00, 0x55 },
};

static const struct drm_display_mode cpustc_lcd_mode = {
	DRM_SIMPLE_MODE(CPUSTC_LCD_WIDTH, CPUSTC_LCD_HEIGHT, 108, 180),
};

static inline struct cpustc_lcd *drm_to_cpustc_lcd(struct drm_device *drm)
{
	return container_of(drm, struct cpustc_lcd, drm);
}

static inline struct cpustc_lcd *pipe_to_cpustc_lcd(
	struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct cpustc_lcd, pipe);
}

static void cpustc_lcd_write_cmd(struct cpustc_lcd *lcd, u16 command)
{
	writel(command, lcd->regs + CPUSTC_LCD_CMD);
}

static void cpustc_lcd_write_data(struct cpustc_lcd *lcd, u16 data)
{
	writel(data, lcd->regs + CPUSTC_LCD_DATA);
}

static void cpustc_lcd_write_reg(struct cpustc_lcd *lcd, u16 reg, u8 value)
{
	cpustc_lcd_write_cmd(lcd, reg);
	cpustc_lcd_write_data(lcd, value);
}

static void cpustc_lcd_write_sequence(struct cpustc_lcd *lcd,
				      const struct nt35510_reg_value *sequence,
				      size_t count)
{
	size_t i;

	for (i = 0; i < count; i++)
		cpustc_lcd_write_reg(lcd, sequence[i].reg, sequence[i].value);
}

static void nt35510_set_window(struct cpustc_lcd *lcd,
			       const struct cpustc_lcd_rect *rect)
{
	u16 x_start = rect->x1;
	u16 y_start = rect->y1;
	u16 x_end = rect->x2 - 1;
	u16 y_end = rect->y2 - 1;

	cpustc_lcd_write_reg(lcd, 0x2A00, x_start >> 8);
	cpustc_lcd_write_reg(lcd, 0x2A01, x_start);
	cpustc_lcd_write_reg(lcd, 0x2A02, x_end >> 8);
	cpustc_lcd_write_reg(lcd, 0x2A03, x_end);
	cpustc_lcd_write_reg(lcd, 0x2B00, y_start >> 8);
	cpustc_lcd_write_reg(lcd, 0x2B01, y_start);
	cpustc_lcd_write_reg(lcd, 0x2B02, y_end >> 8);
	cpustc_lcd_write_reg(lcd, 0x2B03, y_end);
}

static void nt35510_init(struct cpustc_lcd *lcd)
{
	const struct cpustc_lcd_rect full = {
		.x1 = 0,
		.y1 = 0,
		.x2 = CPUSTC_LCD_WIDTH,
		.y2 = CPUSTC_LCD_HEIGHT,
		.valid = true,
	};
	size_t bank;
	size_t i;

	writel(0, lcd->regs + CPUSTC_LCD_CONTROL);
	msleep(20);
	writel(CPUSTC_LCD_WRITE_TIMING_DEFAULT,
	       lcd->regs + CPUSTC_LCD_WRITE_TIMING);
	writel(CPUSTC_LCD_CONTROL_RESET_N,
	       lcd->regs + CPUSTC_LCD_CONTROL);
	msleep(120);
	cpustc_lcd_write_sequence(lcd, nt35510_page1_power,
				  ARRAY_SIZE(nt35510_page1_power));
	for (bank = 0; bank < ARRAY_SIZE(nt35510_gamma_base); bank++)
		for (i = 0; i < ARRAY_SIZE(nt35510_gamma); i++)
			cpustc_lcd_write_reg(lcd, nt35510_gamma_base[bank] + i,
					     nt35510_gamma[i]);
	cpustc_lcd_write_sequence(lcd, nt35510_page0_display,
				  ARRAY_SIZE(nt35510_page0_display));
	cpustc_lcd_write_cmd(lcd, 0x1100);
	msleep(120);
	cpustc_lcd_write_cmd(lcd, 0x2900);
	usleep_range(10000, 12000);
	cpustc_lcd_write_reg(lcd, 0x3600, 0x00);
	nt35510_set_window(lcd, &full);
}

static u32 cpustc_lcd_read_power_requests(struct cpustc_lcd *lcd)
{
	return readl(lcd->regs + CPUSTC_LCD_POWER_CONTROL) &
	       CPUSTC_LCD_POWER_REQUEST_MASK;
}

static void cpustc_lcd_write_power_requests_locked(struct cpustc_lcd *lcd,
						   u32 requests)
{
	requests &= CPUSTC_LCD_POWER_REQUEST_MASK;
	writel(requests, lcd->regs + CPUSTC_LCD_POWER_CONTROL);
	lcd->power_requests = requests;
}

static void cpustc_lcd_update_power_locked(struct cpustc_lcd *lcd, u32 mask,
						   bool enabled)
{
	u32 requests = cpustc_lcd_read_power_requests(lcd);

	if (enabled)
		requests |= mask;
	else
		requests &= ~mask;
	cpustc_lcd_write_power_requests_locked(lcd, requests);
}

static int cpustc_lcd_request_recovery_locked(struct cpustc_lcd *lcd)
{
	u32 status = readl(lcd->regs + CPUSTC_LCD_STATUS);
	u32 requests;

	if (status & (CPUSTC_LCD_STATUS_RECOVERY_ACTIVE |
		      CPUSTC_LCD_STATUS_RECOVERY_PENDING))
		return -EBUSY;
	requests = cpustc_lcd_read_power_requests(lcd);
	writel(requests | CPUSTC_LCD_POWER_RECOVER,
	       lcd->regs + CPUSTC_LCD_POWER_CONTROL);
	return 0;
}

static int cpustc_lcd_wait_control_idle(struct cpustc_lcd *lcd)
{
	u32 status;

	return readl_poll_timeout(lcd->regs + CPUSTC_LCD_STATUS, status,
				  !(status & (CPUSTC_LCD_STATUS_RECOVERY_ACTIVE |
					       CPUSTC_LCD_STATUS_DISPLAY_TRANSITION)),
				  100, 100000);
}

static int cpustc_lcd_wait_recovery_ready(struct cpustc_lcd *lcd)
{
	u32 status;

	return readl_poll_timeout(lcd->regs + CPUSTC_LCD_STATUS, status,
				  !(status & CPUSTC_LCD_STATUS_RECOVERY_ACTIVE),
				  100, 100000);
}

static int cpustc_lcd_initialize_panel_locked(struct cpustc_lcd *lcd,
						      u32 requests, bool recovering)
{
	int ret;

	requests &= CPUSTC_LCD_POWER_REQUEST_MASK;
	if (!recovering) {
		cpustc_lcd_write_power_requests_locked(
			lcd, requests | CPUSTC_LCD_POWER_DISPLAY);
		ret = cpustc_lcd_wait_control_idle(lcd);
		if (ret)
			return ret;
	}
	nt35510_init(lcd);
	if (recovering)
		writel(CPUSTC_LCD_STATUS_RECOVERY_PENDING,
		       lcd->regs + CPUSTC_LCD_STATUS);
	cpustc_lcd_write_power_requests_locked(lcd, requests);
	return cpustc_lcd_wait_control_idle(lcd);
}

static void cpustc_lcd_mark_damage_locked(struct cpustc_lcd *lcd,
					  const struct drm_rect *rect)
{
	u32 x1 = clamp_t(int, rect->x1, 0, CPUSTC_LCD_WIDTH);
	u32 y1 = clamp_t(int, rect->y1, 0, CPUSTC_LCD_HEIGHT);
	u32 x2 = clamp_t(int, rect->x2, 0, CPUSTC_LCD_WIDTH);
	u32 y2 = clamp_t(int, rect->y2, 0, CPUSTC_LCD_HEIGHT);

	if (x1 >= x2 || y1 >= y2)
		return;
	if (lcd->damage.valid) {
		lcd->damage.x1 = min(lcd->damage.x1, x1);
		lcd->damage.y1 = min(lcd->damage.y1, y1);
		lcd->damage.x2 = max(lcd->damage.x2, x2);
		lcd->damage.y2 = max(lcd->damage.y2, y2);
	} else {
		lcd->damage.x1 = x1;
		lcd->damage.y1 = y1;
		lcd->damage.x2 = x2;
		lcd->damage.y2 = y2;
		lcd->damage.valid = true;
	}
}

static void cpustc_lcd_mark_full_damage_locked(struct cpustc_lcd *lcd)
{
	struct drm_rect full = { 0, 0, CPUSTC_LCD_WIDTH, CPUSTC_LCD_HEIGHT };

	cpustc_lcd_mark_damage_locked(lcd, &full);
}

static void cpustc_lcd_queue_refresh(struct cpustc_lcd *lcd,
					   unsigned long delay)
{
	unsigned long flags;
	bool queue;

	spin_lock_irqsave(&lcd->state_lock, flags);
	queue = !lcd->stopping && !lcd->suspended &&
		!lcd->refresh_timer_started;
	if (queue)
		lcd->refresh_timer_started = true;
	spin_unlock_irqrestore(&lcd->state_lock, flags);
	if (queue)
		queue_delayed_work(system_highpri_wq, &lcd->refresh_work, delay);
}

static void cpustc_lcd_queue_state_locked(struct cpustc_lcd *lcd,
						  struct drm_plane_state *state,
						  const struct drm_rect *damage)
{
	struct drm_framebuffer *old_fb;
	struct drm_framebuffer *fb;
	dma_addr_t address;
	unsigned long flags;

	if (!state || !state->fb || !state->visible)
		return;
	fb = state->fb;
	address = drm_fb_cma_get_gem_addr(fb, state, 0);
	drm_framebuffer_get(fb);

	spin_lock_irqsave(&lcd->state_lock, flags);
	old_fb = lcd->pending_fb;
	lcd->pending_fb = fb;
	lcd->pending_address = address;
	cpustc_lcd_mark_damage_locked(lcd, damage);
	lcd->debug_damage_queued++;
	lcd->refresh_pending = lcd->damage.valid;
	spin_unlock_irqrestore(&lcd->state_lock, flags);
	if (old_fb)
		drm_framebuffer_put(old_fb);
}

static void cpustc_lcd_queue_current_frame(struct cpustc_lcd *lcd, bool full)
{
	struct drm_plane_state *state;
	struct drm_rect damage;

	damage = (struct drm_rect){ 0, 0, CPUSTC_LCD_WIDTH, CPUSTC_LCD_HEIGHT };
	drm_modeset_lock(&lcd->pipe.plane.mutex, NULL);
	state = lcd->pipe.plane.state;
	if (state && state->fb && state->visible) {
		if (full)
			cpustc_lcd_queue_state_locked(lcd, state, &damage);
	}
	drm_modeset_unlock(&lcd->pipe.plane.mutex);
	if (full)
		cpustc_lcd_queue_refresh(lcd, 0);
}

static void cpustc_lcd_start_refresh(struct cpustc_lcd *lcd)
{
	struct drm_flip_task *cleanup_task;
	struct drm_flip_task *completed_task = NULL;
	struct cpustc_lcd_rect damage;
	struct drm_framebuffer *fb;
	dma_addr_t address;
	dma_addr_t dma_address;
	unsigned long flags;
	u32 status;
	u32 power;
	u32 width;
	u32 height;
	bool pending;
	bool damage_valid;
	bool software_busy;
	bool start_recovery = false;
	bool recovery_requested = false;
	bool dma_timed_out = false;
	bool missed_completion = false;

	cleanup_task = drm_flip_work_allocate_task(NULL, GFP_KERNEL);
	if (!cleanup_task) {
		dev_err_ratelimited(lcd->dev,
				    "failed to allocate framebuffer cleanup task\n");
		return;
	}
	mutex_lock(&lcd->update_lock);
	status = readl(lcd->regs + CPUSTC_LCD_STATUS);
	power = cpustc_lcd_read_power_requests(lcd);
	spin_lock_irqsave(&lcd->state_lock, flags);
	if (lcd->stopping || lcd->suspended ||
	    !(power & CPUSTC_LCD_POWER_DISPLAY)) {
		lcd->refresh_pending = lcd->damage.valid;
		spin_unlock_irqrestore(&lcd->state_lock, flags);
		goto out_unlock;
	}
	if (status & CPUSTC_LCD_STATUS_RECOVERY_PENDING) {
		if (!lcd->recovering) {
			lcd->recovering = true;
			lcd->dma_busy = false;
			lcd->hw_busy_observed = false;
			completed_task = lcd->active_fb_cleanup_task;
			lcd->active_fb_cleanup_task = NULL;
			lcd->active_fb = NULL;
			start_recovery = true;
		}
		lcd->refresh_pending = lcd->damage.valid;
		spin_unlock_irqrestore(&lcd->state_lock, flags);
		goto out_unlock;
	}
	if (lcd->recovering ||
	    (status & (CPUSTC_LCD_STATUS_DISPLAY_TRANSITION |
		       CPUSTC_LCD_STATUS_RECOVERY_ACTIVE))) {
		lcd->refresh_pending = lcd->damage.valid;
		spin_unlock_irqrestore(&lcd->state_lock, flags);
		goto out_unlock;
	}
	software_busy = lcd->dma_busy;
	if (software_busy && !(status & CPUSTC_LCD_STATUS_DMA_BUSY)) {
		missed_completion = true;
		lcd->debug_missed_completions++;
		if (status & CPUSTC_LCD_STATUS_DMA_ERROR)
			lcd->debug_dma_errors++;
		lcd->dma_busy = false;
		lcd->hw_busy_observed = false;
		completed_task = lcd->active_fb_cleanup_task;
		lcd->active_fb_cleanup_task = NULL;
		lcd->active_fb = NULL;
		writel(status & CPUSTC_LCD_STATUS_DMA_W1C,
		       lcd->regs + CPUSTC_LCD_STATUS);
	}
	if (status & CPUSTC_LCD_STATUS_DMA_BUSY) {
		software_busy = lcd->dma_busy;
		lcd->debug_busy_deferrals++;
		if (!lcd->hw_busy_observed)
			lcd->dma_deadline = jiffies +
				msecs_to_jiffies(CPUSTC_LCD_DMA_TIMEOUT_MS);
		lcd->hw_busy_observed = true;
		lcd->refresh_pending = lcd->damage.valid;
		pending = lcd->refresh_pending;
		damage_valid = lcd->damage.valid;
		if (time_after_eq(jiffies, lcd->dma_deadline)) {
			dma_timed_out = true;
			lcd->debug_dma_timeouts++;
			recovery_requested =
				!cpustc_lcd_request_recovery_locked(lcd);
		}
		spin_unlock_irqrestore(&lcd->state_lock, flags);
		dev_warn_ratelimited(lcd->dev,
			"LCD refresh deferred: sw_busy=%u status=%#x pending=%u damage=%u\n",
			software_busy, status, pending, damage_valid);
		goto out_unlock;
	}
	lcd->hw_busy_observed = false;
	if (!lcd->damage.valid || !lcd->pending_fb) {
		lcd->refresh_pending = false;
		spin_unlock_irqrestore(&lcd->state_lock, flags);
		goto out_unlock;
	}
	damage = lcd->damage;
	lcd->damage.valid = false;
	lcd->refresh_pending = false;
	lcd->dma_busy = true;
	lcd->hw_busy_observed = true;
	fb = lcd->pending_fb;
	lcd->pending_fb = NULL;
	address = lcd->pending_address;
	lcd->active_fb = fb;
	cleanup_task->data = fb;
	lcd->active_fb_cleanup_task = cleanup_task;
	lcd->debug_dma_started++;
	lcd->dma_deadline = jiffies +
		msecs_to_jiffies(CPUSTC_LCD_DMA_TIMEOUT_MS);
	cleanup_task = NULL;
	spin_unlock_irqrestore(&lcd->state_lock, flags);

	dma_address = address + damage.y1 * CPUSTC_LCD_LINE_BYTES +
		damage.x1 * CPUSTC_LCD_BYTES_PER_PIXEL;
	dma_sync_single_for_device(lcd->dev, address, CPUSTC_LCD_FRAME_BYTES,
				   DMA_TO_DEVICE);
	nt35510_set_window(lcd, &damage);
	cpustc_lcd_write_cmd(lcd, 0x2C00);
	width = damage.x2 - damage.x1;
	height = damage.y2 - damage.y1;
	writel(CPUSTC_LCD_STATUS_DMA_W1C, lcd->regs + CPUSTC_LCD_STATUS);
	writel(lower_32_bits(dma_address), lcd->regs + CPUSTC_LCD_DMA_BASE);
	writel(width, lcd->regs + CPUSTC_LCD_DMA_WIDTH);
	writel(height, lcd->regs + CPUSTC_LCD_DMA_HEIGHT);
	writel(CPUSTC_LCD_LINE_BYTES, lcd->regs + CPUSTC_LCD_DMA_SRC_STRIDE);
	dma_wmb();
	writel(CPUSTC_LCD_CONTROL_RESET_N | CPUSTC_LCD_CONTROL_DMA_START |
	       CPUSTC_LCD_CONTROL_DMA_2D, lcd->regs + CPUSTC_LCD_CONTROL);

out_unlock:
	mutex_unlock(&lcd->update_lock);
	if (completed_task) {
		drm_flip_work_queue_task(&lcd->fb_cleanup_work, completed_task);
		drm_flip_work_commit(&lcd->fb_cleanup_work, system_unbound_wq);
	}
	if (missed_completion)
		dev_warn_ratelimited(lcd->dev,
			"recovered a missed LCD DMA completion: status=%#x\n",
			status);
	if (dma_timed_out && recovery_requested)
		dev_err_ratelimited(lcd->dev,
			"LCD DMA timed out; requesting recovery: status=%#x\n",
			status);
	kfree(cleanup_task);
	if (start_recovery)
		schedule_work(&lcd->recovery_work);
}

static void cpustc_lcd_refresh_work(struct work_struct *work)
{
	struct cpustc_lcd *lcd = container_of(to_delayed_work(work),
						 struct cpustc_lcd, refresh_work);
	unsigned long flags;
	bool requeue;

	spin_lock_irqsave(&lcd->state_lock, flags);
	lcd->debug_periodic_checks++;
	spin_unlock_irqrestore(&lcd->state_lock, flags);
	cpustc_lcd_start_refresh(lcd);
	spin_lock_irqsave(&lcd->state_lock, flags);
	requeue = !lcd->stopping && !lcd->suspended;
	if (!requeue)
		lcd->refresh_timer_started = false;
	spin_unlock_irqrestore(&lcd->state_lock, flags);
	if (requeue)
		queue_delayed_work(
			system_highpri_wq, &lcd->refresh_work,
			msecs_to_jiffies(CPUSTC_LCD_CHECK_PERIOD_MS));
}

static int cpustc_lcd_backlight_update_status(
	struct backlight_device *backlight)
{
	struct cpustc_lcd *lcd = bl_get_data(backlight);
	bool enabled = backlight_get_brightness(backlight) != 0;

	mutex_lock(&lcd->update_lock);
	cpustc_lcd_update_power_locked(lcd, CPUSTC_LCD_POWER_BACKLIGHT,
				       enabled);
	mutex_unlock(&lcd->update_lock);
	return 0;
}

static int cpustc_lcd_backlight_get_brightness(
	struct backlight_device *backlight)
{
	struct cpustc_lcd *lcd = bl_get_data(backlight);

	return !!(cpustc_lcd_read_power_requests(lcd) &
		  CPUSTC_LCD_POWER_BACKLIGHT);
}

static const struct backlight_ops cpustc_lcd_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = cpustc_lcd_backlight_update_status,
	.get_brightness = cpustc_lcd_backlight_get_brightness,
};

static ssize_t display_enabled_show(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%u\n",
			  !!(cpustc_lcd_read_power_requests(lcd) &
			     CPUSTC_LCD_POWER_DISPLAY));
}

static ssize_t display_enabled_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(dev_get_drvdata(dev));
	bool enabled;

	if (kstrtobool(buf, &enabled))
		return -EINVAL;
	mutex_lock(&lcd->update_lock);
	cpustc_lcd_update_power_locked(lcd, CPUSTC_LCD_POWER_DISPLAY, enabled);
	mutex_unlock(&lcd->update_lock);
	if (enabled) {
		cpustc_lcd_queue_current_frame(lcd, true);
		cpustc_lcd_queue_refresh(lcd, 0);
	}
	return count;
}
static DEVICE_ATTR_RW(display_enabled);

static ssize_t touch_enabled_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%u\n",
			  !!(cpustc_lcd_read_power_requests(lcd) &
			     CPUSTC_LCD_POWER_TOUCH));
}

static ssize_t touch_enabled_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(dev_get_drvdata(dev));
	bool enabled;

	if (kstrtobool(buf, &enabled))
		return -EINVAL;
	mutex_lock(&lcd->update_lock);
	cpustc_lcd_update_power_locked(lcd, CPUSTC_LCD_POWER_TOUCH, enabled);
	mutex_unlock(&lcd->update_lock);
	return count;
}
static DEVICE_ATTR_RW(touch_enabled);

static ssize_t recover_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(dev_get_drvdata(dev));
	bool recover;
	int ret;

	if (kstrtobool(buf, &recover) || !recover)
		return -EINVAL;
	mutex_lock(&lcd->update_lock);
	ret = cpustc_lcd_request_recovery_locked(lcd);
	mutex_unlock(&lcd->update_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(recover);

static ssize_t debug_stats_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(dev_get_drvdata(dev));
	struct cpustc_lcd_rect damage;
	unsigned long flags;
	u64 damage_queued;
	u64 periodic_checks;
	u64 dma_started;
	u64 dma_completed;
	u64 missed_completions;
	u64 dma_errors;
	u64 busy_deferrals;
	u64 dma_timeouts;
	u64 recoveries;
	u64 irq_count;
	u64 irq_none;
	u32 status;
	bool dma_busy;
	bool refresh_pending;
	bool recovering;

	status = readl(lcd->regs + CPUSTC_LCD_STATUS);
	spin_lock_irqsave(&lcd->state_lock, flags);
	damage_queued = lcd->debug_damage_queued;
	periodic_checks = lcd->debug_periodic_checks;
	dma_started = lcd->debug_dma_started;
	dma_completed = lcd->debug_dma_completed;
	missed_completions = lcd->debug_missed_completions;
	dma_errors = lcd->debug_dma_errors;
	busy_deferrals = lcd->debug_busy_deferrals;
	dma_timeouts = lcd->debug_dma_timeouts;
	recoveries = lcd->debug_recoveries;
	irq_count = lcd->debug_irq_count;
	irq_none = lcd->debug_irq_none;
	dma_busy = lcd->dma_busy;
	refresh_pending = lcd->refresh_pending;
	recovering = lcd->recovering;
	damage = lcd->damage;
	spin_unlock_irqrestore(&lcd->state_lock, flags);

	return sysfs_emit(buf,
		"status=%#x sw_dma_busy=%u hw_dma_busy=%u refresh_pending=%u "
		"recovering=%u damage=%u:%u,%u-%u,%u "
		"damage_queued=%llu periodic_checks=%llu "
		"dma_started=%llu dma_completed=%llu "
		"missed_completions=%llu dma_errors=%llu "
		"busy_deferrals=%llu dma_timeouts=%llu recoveries=%llu "
		"irq_count=%llu irq_none=%llu\n",
		status, dma_busy, !!(status & CPUSTC_LCD_STATUS_DMA_BUSY),
		refresh_pending, recovering, damage.valid, damage.x1, damage.y1,
		damage.x2, damage.y2, (unsigned long long)damage_queued,
		(unsigned long long)periodic_checks,
		(unsigned long long)dma_started,
		(unsigned long long)dma_completed,
		(unsigned long long)missed_completions,
		(unsigned long long)dma_errors,
		(unsigned long long)busy_deferrals,
		(unsigned long long)dma_timeouts,
		(unsigned long long)recoveries,
		(unsigned long long)irq_count,
		(unsigned long long)irq_none);
}
static DEVICE_ATTR_RO(debug_stats);

static struct attribute *cpustc_lcd_attrs[] = {
	&dev_attr_display_enabled.attr,
	&dev_attr_touch_enabled.attr,
	&dev_attr_recover.attr,
	&dev_attr_debug_stats.attr,
	NULL,
};

static const struct attribute_group cpustc_lcd_attr_group = {
	.attrs = cpustc_lcd_attrs,
};

static int cpustc_lcd_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &cpustc_lcd_mode);
	if (!mode)
		return 0;
	drm_mode_probed_add(connector, mode);
	return 1;
}

static enum drm_connector_status
cpustc_lcd_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static const struct drm_connector_helper_funcs
cpustc_lcd_connector_helper_funcs = {
	.get_modes = cpustc_lcd_connector_get_modes,
};

static const struct drm_connector_funcs cpustc_lcd_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.detect = cpustc_lcd_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static bool cpustc_lcd_mode_matches(const struct drm_display_mode *mode)
{
	return mode->hdisplay == cpustc_lcd_mode.hdisplay &&
	       mode->vdisplay == cpustc_lcd_mode.vdisplay;
}

static enum drm_mode_status
cpustc_lcd_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
				   const struct drm_display_mode *mode)
{
	return cpustc_lcd_mode_matches(mode) ? MODE_OK : MODE_BAD;
}

static int cpustc_lcd_pipe_check(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *plane_state,
				 struct drm_crtc_state *crtc_state)
{
	struct drm_framebuffer *fb = plane_state->fb;
	dma_addr_t address;

	if (!crtc_state->enable || !fb)
		return 0;
	if (!cpustc_lcd_mode_matches(&crtc_state->adjusted_mode))
		return -EINVAL;
	if (fb->format->format != DRM_FORMAT_RGB565 ||
	    fb->pitches[0] != CPUSTC_LCD_LINE_BYTES)
		return -EINVAL;
	if (!drm_fb_cma_get_gem_obj(fb, 0))
		return -EINVAL;
	address = drm_fb_cma_get_gem_addr(fb, plane_state, 0);
	if (upper_32_bits(address) || !IS_ALIGNED(address, 4))
		return -EINVAL;
	return 0;
}

static void cpustc_lcd_pipe_enable(struct drm_simple_display_pipe *pipe,
					   struct drm_crtc_state *crtc_state,
					   struct drm_plane_state *plane_state)
{
	struct cpustc_lcd *lcd = pipe_to_cpustc_lcd(pipe);

	mutex_lock(&lcd->update_lock);
	cpustc_lcd_update_power_locked(lcd, CPUSTC_LCD_POWER_DISPLAY, true);
	if (cpustc_lcd_wait_control_idle(lcd))
		dev_err(lcd->dev, "LCD display power did not become idle\n");
	mutex_unlock(&lcd->update_lock);
	cpustc_lcd_queue_refresh(
		lcd, msecs_to_jiffies(CPUSTC_LCD_CHECK_PERIOD_MS));
}

static void cpustc_lcd_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct cpustc_lcd *lcd = pipe_to_cpustc_lcd(pipe);
	struct drm_framebuffer *pending_fb;
	unsigned long flags;
	u32 status;

	cancel_delayed_work_sync(&lcd->refresh_work);
	readl_poll_timeout(lcd->regs + CPUSTC_LCD_STATUS, status,
				  !(status & CPUSTC_LCD_STATUS_DMA_BUSY),
				  1000, 2000000);
	spin_lock_irqsave(&lcd->state_lock, flags);
	lcd->refresh_timer_started = false;
	pending_fb = lcd->pending_fb;
	lcd->pending_fb = NULL;
	lcd->damage.valid = false;
	lcd->refresh_pending = false;
	spin_unlock_irqrestore(&lcd->state_lock, flags);
	if (pending_fb)
		drm_framebuffer_put(pending_fb);
	mutex_lock(&lcd->update_lock);
	cpustc_lcd_update_power_locked(lcd, CPUSTC_LCD_POWER_DISPLAY, false);
	mutex_unlock(&lcd->update_lock);
}

static void cpustc_lcd_pipe_update(struct drm_simple_display_pipe *pipe,
					   struct drm_plane_state *old_state)
{
	struct cpustc_lcd *lcd = pipe_to_cpustc_lcd(pipe);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_pending_vblank_event *event = crtc->state->event;
	struct drm_rect damage;
	unsigned long flags;

	if (state->fb && state->visible &&
	    drm_atomic_helper_damage_merged(old_state, state, &damage)) {
		cpustc_lcd_queue_state_locked(lcd, state, &damage);
		cpustc_lcd_queue_refresh(lcd, 0);
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

static const struct drm_simple_display_pipe_funcs cpustc_lcd_pipe_funcs = {
	.check = cpustc_lcd_pipe_check,
	.mode_valid = cpustc_lcd_pipe_mode_valid,
	.enable = cpustc_lcd_pipe_enable,
	.disable = cpustc_lcd_pipe_disable,
	.update = cpustc_lcd_pipe_update,
	.prepare_fb = drm_gem_simple_display_pipe_prepare_fb,
};

static void cpustc_lcd_control_work(struct work_struct *work)
{
	struct cpustc_lcd *lcd = container_of(work, struct cpustc_lcd,
						      control_work);
	u32 previous;
	u32 requests;
	bool backlight_changed;
	bool display_changed;
	bool touch_changed;

	mutex_lock(&lcd->update_lock);
	previous = lcd->power_requests;
	requests = cpustc_lcd_read_power_requests(lcd);
	lcd->power_requests = requests;
	mutex_unlock(&lcd->update_lock);
	backlight_changed = (previous ^ requests) & CPUSTC_LCD_POWER_BACKLIGHT;
	display_changed = (previous ^ requests) & CPUSTC_LCD_POWER_DISPLAY;
	touch_changed = (previous ^ requests) & CPUSTC_LCD_POWER_TOUCH;
	if (display_changed && (requests & CPUSTC_LCD_POWER_DISPLAY)) {
		cpustc_lcd_queue_current_frame(lcd, true);
		cpustc_lcd_queue_refresh(lcd, 0);
	}
	if (backlight_changed && lcd->backlight)
		backlight_force_update(lcd->backlight, BACKLIGHT_UPDATE_HOTKEY);
	if (display_changed)
		sysfs_notify(&lcd->dev->kobj, NULL, "display_enabled");
	if (touch_changed)
		sysfs_notify(&lcd->dev->kobj, NULL, "touch_enabled");
}

static void cpustc_lcd_recovery_work(struct work_struct *work)
{
	struct cpustc_lcd *lcd = container_of(work, struct cpustc_lcd,
						      recovery_work);
	u32 requests;
	int ret;

	mutex_lock(&lcd->update_lock);
	requests = cpustc_lcd_read_power_requests(lcd);
	ret = cpustc_lcd_wait_recovery_ready(lcd);
	if (!ret)
		ret = cpustc_lcd_initialize_panel_locked(lcd, requests, true);
	if (ret) {
		requests &= ~(CPUSTC_LCD_POWER_BACKLIGHT |
			      CPUSTC_LCD_POWER_DISPLAY);
		cpustc_lcd_write_power_requests_locked(lcd, requests);
		writel(CPUSTC_LCD_STATUS_RECOVERY_PENDING,
		       lcd->regs + CPUSTC_LCD_STATUS);
	}
	mutex_unlock(&lcd->update_lock);
	spin_lock_irq(&lcd->state_lock);
	lcd->recovering = false;
	lcd->dma_busy = false;
	lcd->hw_busy_observed = false;
	if (!ret) {
		cpustc_lcd_mark_full_damage_locked(lcd);
		lcd->refresh_pending = true;
	}
	spin_unlock_irq(&lcd->state_lock);
	writel(CPUSTC_LCD_IRQ_ALL, lcd->regs + CPUSTC_LCD_IRQ_ENABLE);
	if (lcd->backlight)
		backlight_force_update(lcd->backlight, BACKLIGHT_UPDATE_HOTKEY);
	sysfs_notify(&lcd->dev->kobj, NULL, "display_enabled");
	sysfs_notify(&lcd->dev->kobj, NULL, "touch_enabled");
	if (ret) {
		dev_err(lcd->dev,
			"LCD recovery failed; display disabled: %d\n", ret);
		return;
	}
	cpustc_lcd_queue_current_frame(lcd, true);
	cpustc_lcd_queue_refresh(lcd, 0);
}

static void cpustc_lcd_fb_cleanup(struct drm_flip_work *work, void *val)
{
	struct drm_framebuffer *fb = val;

	(void)work;
	drm_framebuffer_put(fb);
}

static irqreturn_t cpustc_lcd_irq(int irq, void *data)
{
	struct cpustc_lcd *lcd = data;
	struct drm_flip_task *cleanup_task = NULL;
	unsigned long flags;
	u32 status;
	bool recovery;

	status = readl(lcd->regs + CPUSTC_LCD_STATUS);
	if (!(status & (CPUSTC_LCD_STATUS_DMA_DONE |
			CPUSTC_LCD_STATUS_CONTROL_CHANGED |
			CPUSTC_LCD_STATUS_RECOVERY_PENDING))) {
		spin_lock_irqsave(&lcd->state_lock, flags);
		lcd->debug_irq_none++;
		spin_unlock_irqrestore(&lcd->state_lock, flags);
		return IRQ_NONE;
	}
	writel(status & CPUSTC_LCD_STATUS_EVENT_W1C,
	       lcd->regs + CPUSTC_LCD_STATUS);
	recovery = status & CPUSTC_LCD_STATUS_RECOVERY_PENDING;
	if (recovery)
		writel(CPUSTC_LCD_IRQ_DMA_DONE |
		       CPUSTC_LCD_IRQ_CONTROL_CHANGED,
		       lcd->regs + CPUSTC_LCD_IRQ_ENABLE);
	spin_lock_irqsave(&lcd->state_lock, flags);
	lcd->debug_irq_count++;
	if (status & CPUSTC_LCD_STATUS_DMA_DONE) {
		lcd->debug_dma_completed++;
		if (status & CPUSTC_LCD_STATUS_DMA_ERROR)
			lcd->debug_dma_errors++;
		lcd->dma_busy = false;
		lcd->hw_busy_observed = false;
		cleanup_task = lcd->active_fb_cleanup_task;
		lcd->active_fb_cleanup_task = NULL;
		lcd->active_fb = NULL;
	}
	if (recovery) {
		lcd->debug_recoveries++;
		if (lcd->active_fb) {
			cleanup_task = lcd->active_fb_cleanup_task;
			lcd->active_fb_cleanup_task = NULL;
			lcd->active_fb = NULL;
		}
		lcd->recovering = true;
		lcd->dma_busy = false;
		lcd->hw_busy_observed = false;
		lcd->refresh_pending = lcd->damage.valid;
	}
	spin_unlock_irqrestore(&lcd->state_lock, flags);
	if (cleanup_task) {
		drm_flip_work_queue_task(&lcd->fb_cleanup_work, cleanup_task);
		drm_flip_work_commit(&lcd->fb_cleanup_work, system_unbound_wq);
	}
	if (status & CPUSTC_LCD_STATUS_DMA_ERROR)
		dev_err_ratelimited(lcd->dev,
				    "LCD DMA completed with an AXI error: status=%#x\n",
				    status);
	if (recovery)
		dev_warn_ratelimited(lcd->dev,
				     "LCD recovery requested: status=%#x\n",
				     status);
	if (status & CPUSTC_LCD_STATUS_CONTROL_CHANGED)
		schedule_work(&lcd->control_work);
	if (recovery)
		schedule_work(&lcd->recovery_work);
	return IRQ_HANDLED;
}

static const struct drm_mode_config_funcs cpustc_lcd_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_CMA_FOPS(cpustc_lcd_fops);

static const struct drm_driver cpustc_lcd_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "cpustc-lcd",
	.desc = "CPUSTC NT35510 LCD DRM/KMS",
	.date = "20260731",
	.major = 1,
	.minor = 0,
	.fops = &cpustc_lcd_fops,
	DRM_GEM_CMA_DRIVER_OPS,
};

static int cpustc_lcd_modeset_init(struct cpustc_lcd *lcd)
{
	static const u32 formats[] = { DRM_FORMAT_RGB565 };
	struct drm_device *drm = &lcd->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = CPUSTC_LCD_WIDTH;
	drm->mode_config.max_width = CPUSTC_LCD_WIDTH;
	drm->mode_config.min_height = CPUSTC_LCD_HEIGHT;
	drm->mode_config.max_height = CPUSTC_LCD_HEIGHT;
	drm->mode_config.preferred_depth = 16;
	drm->mode_config.funcs = &cpustc_lcd_mode_config_funcs;
	ret = drm_connector_init(drm, &lcd->connector,
				 &cpustc_lcd_connector_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;
	drm_connector_helper_add(&lcd->connector,
				 &cpustc_lcd_connector_helper_funcs);
	ret = drm_simple_display_pipe_init(drm, &lcd->pipe,
					   &cpustc_lcd_pipe_funcs,
					   formats, ARRAY_SIZE(formats),
					   NULL, &lcd->connector);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	return 0;
}

static int cpustc_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties backlight_props = { 0 };
	struct cpustc_lcd *lcd;
	struct resource *res;
	u32 requests;
	u32 status;
	int ret;

	lcd = devm_drm_dev_alloc(dev, &cpustc_lcd_drm_driver,
				 struct cpustc_lcd, drm);
	if (IS_ERR(lcd))
		return PTR_ERR(lcd);
	lcd->dev = dev;
	mutex_init(&lcd->update_lock);
	spin_lock_init(&lcd->state_lock);
	INIT_DELAYED_WORK(&lcd->refresh_work, cpustc_lcd_refresh_work);
	INIT_WORK(&lcd->control_work, cpustc_lcd_control_work);
	INIT_WORK(&lcd->recovery_work, cpustc_lcd_recovery_work);
	drm_flip_work_init(&lcd->fb_cleanup_work, "cpustc_lcd_fb_cleanup",
			   cpustc_lcd_fb_cleanup);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lcd->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(lcd->regs))
		return PTR_ERR(lcd->regs);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	lcd->irq = platform_get_irq(pdev, 0);
	if (lcd->irq < 0)
		return lcd->irq;
	writel(0, lcd->regs + CPUSTC_LCD_IRQ_ENABLE);
	writel(CPUSTC_LCD_STATUS_EVENT_W1C,
	       lcd->regs + CPUSTC_LCD_STATUS);
	ret = devm_request_irq(dev, lcd->irq, cpustc_lcd_irq, 0,
				       dev_name(dev), lcd);
	if (ret)
		return ret;
	requests = cpustc_lcd_read_power_requests(lcd);
	lcd->power_requests = requests;
	ret = cpustc_lcd_wait_control_idle(lcd);
	if (ret)
		return ret;
	status = readl(lcd->regs + CPUSTC_LCD_STATUS);
	ret = cpustc_lcd_initialize_panel_locked(
		lcd, requests, status & CPUSTC_LCD_STATUS_RECOVERY_PENDING);
	if (ret)
		return ret;
	writel(CPUSTC_LCD_STATUS_EVENT_W1C,
	       lcd->regs + CPUSTC_LCD_STATUS);
	writel(CPUSTC_LCD_IRQ_ALL, lcd->regs + CPUSTC_LCD_IRQ_ENABLE);
	platform_set_drvdata(pdev, &lcd->drm);
	backlight_props.type = BACKLIGHT_RAW;
	backlight_props.max_brightness = 1;
	backlight_props.brightness = !!(requests & CPUSTC_LCD_POWER_BACKLIGHT);
	lcd->backlight = devm_backlight_device_register(
		dev, "cpustc-lcd", dev, lcd, &cpustc_lcd_backlight_ops,
		&backlight_props);
	if (IS_ERR(lcd->backlight))
		return PTR_ERR(lcd->backlight);
	ret = cpustc_lcd_modeset_init(lcd);
	if (ret)
		return ret;
	ret = devm_device_add_group(dev, &cpustc_lcd_attr_group);
	if (ret)
		return ret;
	ret = drm_dev_register(&lcd->drm, 0);
	if (ret)
		return ret;
	drm_fbdev_generic_setup(&lcd->drm, 16);
	cpustc_lcd_queue_refresh(
		lcd, msecs_to_jiffies(CPUSTC_LCD_CHECK_PERIOD_MS));
	return 0;
}

static int cpustc_lcd_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(drm);
	struct drm_framebuffer *pending_fb;
	struct drm_framebuffer *active_fb;
	struct drm_flip_task *cleanup_task;
	unsigned long flags;
	u32 status;
	int ret;

	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);
	lcd->stopping = true;
	cancel_delayed_work_sync(&lcd->refresh_work);
	cancel_work_sync(&lcd->control_work);
	cancel_work_sync(&lcd->recovery_work);
	ret = readl_poll_timeout(lcd->regs + CPUSTC_LCD_STATUS, status,
				  !(status & CPUSTC_LCD_STATUS_DMA_BUSY),
				  1000, 2000000);
	if (ret)
		dev_warn(lcd->dev, "LCD DMA did not stop during remove: %d\n", ret);
	writel(0, lcd->regs + CPUSTC_LCD_IRQ_ENABLE);
	disable_irq(lcd->irq);
	flush_work(&lcd->fb_cleanup_work.worker);
	spin_lock_irqsave(&lcd->state_lock, flags);
	pending_fb = lcd->pending_fb;
	lcd->pending_fb = NULL;
	active_fb = lcd->active_fb;
	lcd->active_fb = NULL;
	cleanup_task = lcd->active_fb_cleanup_task;
	lcd->active_fb_cleanup_task = NULL;
	spin_unlock_irqrestore(&lcd->state_lock, flags);
	if (pending_fb)
		drm_framebuffer_put(pending_fb);
	if (active_fb)
		drm_framebuffer_put(active_fb);
	kfree(cleanup_task);
	drm_flip_work_cleanup(&lcd->fb_cleanup_work);
	writel(0, lcd->regs + CPUSTC_LCD_CONTROL);
	return 0;
}

static int __maybe_unused cpustc_lcd_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(drm);
	int ret;

	lcd->resume_power_requests = cpustc_lcd_read_power_requests(lcd);
	spin_lock_irq(&lcd->state_lock);
	lcd->suspended = true;
	spin_unlock_irq(&lcd->state_lock);
	ret = drm_mode_config_helper_suspend(drm);
	if (ret) {
		spin_lock_irq(&lcd->state_lock);
		lcd->suspended = false;
		spin_unlock_irq(&lcd->state_lock);
	}
	return ret;
}

static int __maybe_unused cpustc_lcd_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct cpustc_lcd *lcd = drm_to_cpustc_lcd(drm);
	int ret;

	mutex_lock(&lcd->update_lock);
	ret = cpustc_lcd_initialize_panel_locked(lcd,
						 lcd->resume_power_requests,
						 false);
	if (!ret) {
		writel(CPUSTC_LCD_STATUS_EVENT_W1C,
		       lcd->regs + CPUSTC_LCD_STATUS);
		writel(CPUSTC_LCD_IRQ_ALL, lcd->regs + CPUSTC_LCD_IRQ_ENABLE);
	}
	mutex_unlock(&lcd->update_lock);
	if (ret)
		return ret;
	spin_lock_irq(&lcd->state_lock);
	lcd->suspended = false;
	spin_unlock_irq(&lcd->state_lock);
	ret = drm_mode_config_helper_resume(drm);
	if (!ret)
		cpustc_lcd_queue_refresh(
			lcd, msecs_to_jiffies(CPUSTC_LCD_CHECK_PERIOD_MS));
	return ret;
}

static void cpustc_lcd_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct cpustc_lcd *lcd;

	if (!drm)
		return;
	lcd = drm_to_cpustc_lcd(drm);
	drm_atomic_helper_shutdown(drm);
	lcd->stopping = true;
	writel(0, lcd->regs + CPUSTC_LCD_IRQ_ENABLE);
	cancel_delayed_work_sync(&lcd->refresh_work);
	cancel_work_sync(&lcd->control_work);
	cancel_work_sync(&lcd->recovery_work);
	writel(0, lcd->regs + CPUSTC_LCD_CONTROL);
}

static SIMPLE_DEV_PM_OPS(cpustc_lcd_pm_ops, cpustc_lcd_suspend,
				 cpustc_lcd_resume);

static const struct of_device_id cpustc_lcd_of_match[] = {
	{ .compatible = "cpustc,nt35510-lcd" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpustc_lcd_of_match);

static struct platform_driver cpustc_lcd_platform_driver = {
	.probe = cpustc_lcd_probe,
	.remove = cpustc_lcd_remove,
	.shutdown = cpustc_lcd_shutdown,
	.driver = {
		.name = "cpustc-lcd-drm",
		.of_match_table = cpustc_lcd_of_match,
		.pm = &cpustc_lcd_pm_ops,
	},
};
module_platform_driver(cpustc_lcd_platform_driver);

MODULE_DESCRIPTION("CPUSTC NT35510 LCD DRM/KMS driver");
MODULE_LICENSE("GPL");
