// SPDX-License-Identifier: GPL-2.0-only
/* CPUSTC 8x8 LED dot matrix misc device driver. */

#include <linux/fs.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define CPUSTC_DOTMATRIX_PATTERN_LOW	0x00
#define CPUSTC_DOTMATRIX_PATTERN_HIGH	0x04
#define CPUSTC_DOTMATRIX_CONTROL		0x08
#define CPUSTC_DOTMATRIX_SCAN_DIVIDER	0x0c

#define CPUSTC_DOTMATRIX_ENABLE		BIT(0)
#define CPUSTC_DOTMATRIX_COLUMN_ACTIVE_LOW BIT(2)
#define CPUSTC_DOTMATRIX_BRIGHTNESS_SHIFT 8

#define CPUSTC_DOTMATRIX_DEFAULT_DIVIDER	33000
#define CPUSTC_DOTMATRIX_DEFAULT_BRIGHTNESS 255
#define CPUSTC_DOTMATRIX_FRAME_SIZE	8

struct cpustc_dotmatrix {
	struct kref ref;
	struct mutex lock; /* Serializes state changes and MMIO writes. */
	void __iomem *base;
	struct miscdevice miscdev;
	u32 control;
	u32 scan_divider;
	bool opened;
	bool removed;
};

static void cpustc_dotmatrix_release_ref(struct kref *ref)
{
	struct cpustc_dotmatrix *dotmatrix =
		container_of(ref, struct cpustc_dotmatrix, ref);

	kfree(dotmatrix);
}

static void cpustc_dotmatrix_disable_locked(struct cpustc_dotmatrix *dotmatrix)
{
	writel(dotmatrix->control, dotmatrix->base + CPUSTC_DOTMATRIX_CONTROL);
	writel(0, dotmatrix->base + CPUSTC_DOTMATRIX_PATTERN_LOW);
	writel(0, dotmatrix->base + CPUSTC_DOTMATRIX_PATTERN_HIGH);
}

static int cpustc_dotmatrix_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct cpustc_dotmatrix *dotmatrix =
		container_of(miscdev, struct cpustc_dotmatrix, miscdev);
	int error = 0;

	error = mutex_lock_interruptible(&dotmatrix->lock);
	if (error)
		return error;

	if (dotmatrix->removed) {
		error = -ENODEV;
	} else if (dotmatrix->opened) {
		error = -EBUSY;
	} else {
		dotmatrix->opened = true;
		kref_get(&dotmatrix->ref);
		file->private_data = dotmatrix;
		writel(dotmatrix->scan_divider,
		       dotmatrix->base + CPUSTC_DOTMATRIX_SCAN_DIVIDER);
		writel(0, dotmatrix->base + CPUSTC_DOTMATRIX_PATTERN_LOW);
		writel(0, dotmatrix->base + CPUSTC_DOTMATRIX_PATTERN_HIGH);
		writel(dotmatrix->control | CPUSTC_DOTMATRIX_ENABLE,
		       dotmatrix->base + CPUSTC_DOTMATRIX_CONTROL);
	}

	mutex_unlock(&dotmatrix->lock);
	return error;
}

static ssize_t cpustc_dotmatrix_write(struct file *file,
				      const char __user *buffer, size_t count,
				      loff_t *position)
{
	struct cpustc_dotmatrix *dotmatrix = file->private_data;
	u8 rows[CPUSTC_DOTMATRIX_FRAME_SIZE];
	u32 pattern_low = 0;
	u32 pattern_high = 0;
	int row;
	int error;

	if (count != CPUSTC_DOTMATRIX_FRAME_SIZE)
		return -EINVAL;
	if (copy_from_user(rows, buffer, sizeof(rows)))
		return -EFAULT;

	for (row = 0; row < 4; row++)
		pattern_low |= (u32)rows[row] << (row * 8);
	for (row = 4; row < CPUSTC_DOTMATRIX_FRAME_SIZE; row++)
		pattern_high |= (u32)rows[row] << ((row - 4) * 8);

	error = mutex_lock_interruptible(&dotmatrix->lock);
	if (error)
		return error;
	if (dotmatrix->removed) {
		error = -ENODEV;
	} else {
		writel(dotmatrix->control,
		       dotmatrix->base + CPUSTC_DOTMATRIX_CONTROL);
		writel(pattern_low,
		       dotmatrix->base + CPUSTC_DOTMATRIX_PATTERN_LOW);
		writel(pattern_high,
		       dotmatrix->base + CPUSTC_DOTMATRIX_PATTERN_HIGH);
		writel(dotmatrix->control | CPUSTC_DOTMATRIX_ENABLE,
		       dotmatrix->base + CPUSTC_DOTMATRIX_CONTROL);
		error = count;
	}
	mutex_unlock(&dotmatrix->lock);

	return error;
}

static int cpustc_dotmatrix_release(struct inode *inode, struct file *file)
{
	struct cpustc_dotmatrix *dotmatrix = file->private_data;

	mutex_lock(&dotmatrix->lock);
	if (!dotmatrix->removed)
		cpustc_dotmatrix_disable_locked(dotmatrix);
	dotmatrix->opened = false;
	mutex_unlock(&dotmatrix->lock);

	kref_put(&dotmatrix->ref, cpustc_dotmatrix_release_ref);
	return 0;
}

static const struct file_operations cpustc_dotmatrix_fops = {
	.owner = THIS_MODULE,
	.open = cpustc_dotmatrix_open,
	.write = cpustc_dotmatrix_write,
	.release = cpustc_dotmatrix_release,
	.llseek = no_llseek,
};

static int cpustc_dotmatrix_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpustc_dotmatrix *dotmatrix;
	u32 brightness = CPUSTC_DOTMATRIX_DEFAULT_BRIGHTNESS;
	int error;

	dotmatrix = kzalloc(sizeof(*dotmatrix), GFP_KERNEL);
	if (!dotmatrix)
		return -ENOMEM;

	kref_init(&dotmatrix->ref);
	mutex_init(&dotmatrix->lock);
	dotmatrix->scan_divider = CPUSTC_DOTMATRIX_DEFAULT_DIVIDER;

	dotmatrix->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dotmatrix->base)) {
		error = PTR_ERR(dotmatrix->base);
		goto err_put;
	}

	device_property_read_u32(dev, "cpustc,scan-divider",
				 &dotmatrix->scan_divider);
	device_property_read_u32(dev, "cpustc,default-brightness",
				 &brightness);
	if (!dotmatrix->scan_divider || brightness > 255) {
		error = dev_err_probe(dev, -EINVAL,
				      "invalid scan divider or brightness\n");
		goto err_put;
	}

	/* Board wiring uses active-high rows and active-low columns. */
	dotmatrix->control = CPUSTC_DOTMATRIX_COLUMN_ACTIVE_LOW |
		(brightness << CPUSTC_DOTMATRIX_BRIGHTNESS_SHIFT);
	cpustc_dotmatrix_disable_locked(dotmatrix);
	writel(dotmatrix->scan_divider,
	       dotmatrix->base + CPUSTC_DOTMATRIX_SCAN_DIVIDER);

	dotmatrix->miscdev.minor = MISC_DYNAMIC_MINOR;
	dotmatrix->miscdev.name = "cpustc-dotmatrix";
	dotmatrix->miscdev.fops = &cpustc_dotmatrix_fops;
	dotmatrix->miscdev.parent = dev;

	error = misc_register(&dotmatrix->miscdev);
	if (error) {
		dev_err(dev, "failed to register misc device: %d\n", error);
		goto err_put;
	}

	platform_set_drvdata(pdev, dotmatrix);
	return 0;

err_put:
	kref_put(&dotmatrix->ref, cpustc_dotmatrix_release_ref);
	return error;
}

static int cpustc_dotmatrix_remove(struct platform_device *pdev)
{
	struct cpustc_dotmatrix *dotmatrix = platform_get_drvdata(pdev);

	/* misc_deregister serializes against misc open callbacks. */
	misc_deregister(&dotmatrix->miscdev);
	mutex_lock(&dotmatrix->lock);
	dotmatrix->removed = true;
	cpustc_dotmatrix_disable_locked(dotmatrix);
	mutex_unlock(&dotmatrix->lock);
	platform_set_drvdata(pdev, NULL);
	kref_put(&dotmatrix->ref, cpustc_dotmatrix_release_ref);

	return 0;
}

static const struct of_device_id cpustc_dotmatrix_of_match[] = {
	{ .compatible = "cpustc,dotmatrix" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpustc_dotmatrix_of_match);

static struct platform_driver cpustc_dotmatrix_driver = {
	.probe = cpustc_dotmatrix_probe,
	.remove = cpustc_dotmatrix_remove,
	.driver = {
		.name = "cpustc-dotmatrix",
		.of_match_table = cpustc_dotmatrix_of_match,
	},
};
module_platform_driver(cpustc_dotmatrix_driver);

MODULE_DESCRIPTION("CPUSTC 8x8 LED dot matrix driver");
MODULE_LICENSE("GPL");
