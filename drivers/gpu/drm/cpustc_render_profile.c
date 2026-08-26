// SPDX-License-Identifier: GPL-2.0
#include <linux/atomic.h>
#include <linux/cpustc_render_profile.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#define CPUSTC_RENDER_PROFILE_EVENTS 1024

struct cpustc_render_profile_event {
	u64 sequence;
	u64 timestamp_ns;
	u64 bo_id;
	u64 size;
	u64 duration_ns;
	u32 handle;
	u32 stage;
	s32 ret;
	pid_t pid;
	char device[24];
	char comm[TASK_COMM_LEN];
};

static struct cpustc_render_profile_event
	cpustc_render_profile_events[CPUSTC_RENDER_PROFILE_EVENTS];
static atomic64_t cpustc_render_profile_sequence = ATOMIC64_INIT(0);
static DEFINE_SPINLOCK(cpustc_render_profile_lock);

static const char *cpustc_render_profile_stage_name(unsigned int stage)
{
	switch (stage) {
	case CPUSTC_RENDER_BO_CMA_ALLOC:
		return "cma_alloc";
	case CPUSTC_RENDER_BO_PAGE_ALLOC_FALLBACK:
		return "page_alloc_fallback";
	case CPUSTC_RENDER_BO_ZERO:
		return "zero";
	case CPUSTC_RENDER_BO_CACHE_SYNC:
		return "cache_sync";
	case CPUSTC_RENDER_BO_SET_UNCACHED:
		return "set_uncached";
	case CPUSTC_RENDER_BO_DMA_ALLOC_TOTAL:
		return "dma_alloc_total";
	case CPUSTC_RENDER_BO_GEM_OBJECT:
		return "gem_object";
	case CPUSTC_RENDER_BO_GEM_DMA_ALLOC:
		return "gem_dma_alloc";
	case CPUSTC_RENDER_BO_GEM_HANDLE:
		return "gem_handle";
	case CPUSTC_RENDER_BO_DUMB_TOTAL:
		return "dumb_total";
	default:
		return "unknown";
	}
}

void cpustc_render_profile_record(struct device *dev, unsigned int stage,
				  u64 bo_id, u64 size, u64 duration_ns,
				  u32 handle, int ret)
{
	struct cpustc_render_profile_event *event;
	struct cpustc_render_profile_event new_event = { 0 };
	const char *device;
	unsigned long flags;
	u64 sequence;

	device = dev_driver_string(dev);
	if (!device || strncmp(device, "cpustc-", 7))
		return;

	sequence = atomic64_inc_return(&cpustc_render_profile_sequence);
	event = &cpustc_render_profile_events[(sequence - 1) %
						CPUSTC_RENDER_PROFILE_EVENTS];
	new_event.sequence = sequence;
	new_event.timestamp_ns = ktime_get_ns();
	new_event.bo_id = bo_id;
	new_event.size = size;
	new_event.duration_ns = duration_ns;
	new_event.handle = handle;
	new_event.stage = stage;
	new_event.ret = ret;
	new_event.pid = current->pid;
	strscpy(new_event.device, device, sizeof(new_event.device));
	get_task_comm(new_event.comm, current);

	spin_lock_irqsave(&cpustc_render_profile_lock, flags);
	*event = new_event;
	spin_unlock_irqrestore(&cpustc_render_profile_lock, flags);
}
EXPORT_SYMBOL_GPL(cpustc_render_profile_record);

static int cpustc_render_profile_show(struct seq_file *m, void *unused)
{
	struct cpustc_render_profile_event event;
	unsigned long flags;
	u64 end;
	u64 sequence;
	u64 start;

	end = atomic64_read(&cpustc_render_profile_sequence);
	start = end >= CPUSTC_RENDER_PROFILE_EVENTS ?
		end - CPUSTC_RENDER_PROFILE_EVENTS + 1 : 1;

	seq_puts(m, "sequence,timestamp_ns,pid,comm,device,stage,bo_id,size,duration_ns,handle,ret\n");
	for (sequence = start; sequence <= end; sequence++) {
		spin_lock_irqsave(&cpustc_render_profile_lock, flags);
		event = cpustc_render_profile_events[(sequence - 1) %
						CPUSTC_RENDER_PROFILE_EVENTS];
		spin_unlock_irqrestore(&cpustc_render_profile_lock, flags);
		if (event.sequence != sequence)
			continue;
		seq_printf(m, "%llu,%llu,%d,%s,%s,%s,%#llx,%llu,%llu,%u,%d\n",
			   event.sequence, event.timestamp_ns, event.pid,
			   event.comm, event.device,
			   cpustc_render_profile_stage_name(event.stage),
			   event.bo_id, event.size, event.duration_ns,
			   event.handle, event.ret);
	}

	return 0;
}

static int cpustc_render_profile_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpustc_render_profile_show, inode->i_private);
}

static ssize_t cpustc_render_profile_write(struct file *file,
					   const char __user *buf,
					   size_t count, loff_t *ppos)
{
	unsigned long flags;

	spin_lock_irqsave(&cpustc_render_profile_lock, flags);
	atomic64_set(&cpustc_render_profile_sequence, 0);
	spin_unlock_irqrestore(&cpustc_render_profile_lock, flags);
	return count;
}

static const struct file_operations cpustc_render_profile_fops = {
	.owner = THIS_MODULE,
	.open = cpustc_render_profile_open,
	.read = seq_read,
	.write = cpustc_render_profile_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init cpustc_render_profile_init(void)
{
	struct dentry *dir;

	dir = debugfs_create_dir("cpustc_render_profile", NULL);
	if (IS_ERR_OR_NULL(dir))
		return 0;
	debugfs_create_file("bo_events", 0600, dir, NULL,
			    &cpustc_render_profile_fops);
	return 0;
}
late_initcall(cpustc_render_profile_init);
