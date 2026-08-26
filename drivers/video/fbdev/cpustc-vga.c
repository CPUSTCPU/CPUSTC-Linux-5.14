// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPUSTC VGA frame-buffer driver, as a platform device
 *
 * Copyright (c) 2013, Stephen Warren
 *
 * Based on q40fb.c, which was:
 * Copyright (C) 2001 Richard Zidlicky <rz@linux-m68k.org>
 *
 * Also based on offb.c, which was:
 * Copyright (C) 1997 Geert Uytterhoeven
 * Copyright (C) 1996 Paul Mackerras
 */

#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/of_platform.h>
#include <linux/parser.h>

#define CPUSTC_VGA_H_VISIBLE		0x00
#define CPUSTC_VGA_H_FRONT		0x04
#define CPUSTC_VGA_H_SYNC		0x08
#define CPUSTC_VGA_H_BACK		0x0c
#define CPUSTC_VGA_V_VISIBLE		0x10
#define CPUSTC_VGA_V_FRONT		0x14
#define CPUSTC_VGA_V_SYNC		0x18
#define CPUSTC_VGA_V_BACK		0x1c
#define CPUSTC_VGA_BURST_COUNT_MAX	0x24

#define CPUSTC_VGA_BYTES_PER_PIXEL	2
#define CPUSTC_VGA_PIXELS_PER_BURST	32
#define CPUSTC_VGA_MAX_BURSTS		32

static const struct fb_fix_screeninfo cpustc_vga_fix = {
	.id		= "cpustc-vga",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.accel		= FB_ACCEL_NONE,
};

static const struct fb_var_screeninfo cpustc_vga_var = {
	.height		= -1,
	.width		= -1,
	.activate	= FB_ACTIVATE_NOW,
	.vmode		= FB_VMODE_NONINTERLACED,
};

#define PSEUDO_PALETTE_SIZE 16

struct cpustc_vga_par {
	u32 palette[PSEUDO_PALETTE_SIZE];
	void __iomem *regs;
#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
	bool clks_enabled;
	unsigned int clk_count;
	struct clk **clks;
#endif
};

static int cpustc_vga_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	u64 framebuffer_size;

	if (!var->xres || var->xres >
	    CPUSTC_VGA_PIXELS_PER_BURST * CPUSTC_VGA_MAX_BURSTS ||
	    var->xres % CPUSTC_VGA_PIXELS_PER_BURST || !var->yres)
		return -EINVAL;

	if (!var->hsync_len || !var->vsync_len ||
	    (var->vmode & FB_VMODE_MASK) != FB_VMODE_NONINTERLACED)
		return -EINVAL;

	framebuffer_size = (u64)var->xres * var->yres *
			   CPUSTC_VGA_BYTES_PER_PIXEL;
	if (framebuffer_size > info->fix.smem_len)
		return -EINVAL;

	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	var->xoffset = 0;
	var->yoffset = 0;
	var->bits_per_pixel = 16;
	var->red.offset = 11;
	var->red.length = 5;
	var->green.offset = 5;
	var->green.length = 6;
	var->blue.offset = 0;
	var->blue.length = 5;
	var->transp.offset = 0;
	var->transp.length = 0;

	return 0;
}

static int cpustc_vga_set_par(struct fb_info *info)
{
	struct cpustc_vga_par *par = info->par;
	struct fb_var_screeninfo *var = &info->var;

	info->fix.line_length = var->xres * CPUSTC_VGA_BYTES_PER_PIXEL;

	writel(var->xres, par->regs + CPUSTC_VGA_H_VISIBLE);
	writel(var->right_margin, par->regs + CPUSTC_VGA_H_FRONT);
	writel(var->hsync_len, par->regs + CPUSTC_VGA_H_SYNC);
	writel(var->left_margin, par->regs + CPUSTC_VGA_H_BACK);
	writel(var->yres, par->regs + CPUSTC_VGA_V_VISIBLE);
	writel(var->lower_margin, par->regs + CPUSTC_VGA_V_FRONT);
	writel(var->vsync_len, par->regs + CPUSTC_VGA_V_SYNC);
	writel(var->upper_margin, par->regs + CPUSTC_VGA_V_BACK);
	writel(var->xres / CPUSTC_VGA_PIXELS_PER_BURST,
	       par->regs + CPUSTC_VGA_BURST_COUNT_MAX);

	/* Complete all APB writes before framebuffer users resume drawing. */
	readl(par->regs + CPUSTC_VGA_BURST_COUNT_MAX);

	return 0;
}

static int cpustc_vga_setcolreg(u_int regno, u_int red, u_int green,
			       u_int blue, u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= PSEUDO_PALETTE_SIZE)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;

	return 0;
}

struct cpustc_vga_par;
static void cpustc_vga_clocks_destroy(struct cpustc_vga_par *par);

static void cpustc_vga_destroy(struct fb_info *info)
{
	cpustc_vga_clocks_destroy(info->par);
	if (info->screen_base)
		iounmap(info->screen_base);
}

static const struct fb_ops cpustc_vga_ops = {
	.owner		= THIS_MODULE,
	.fb_destroy	= cpustc_vga_destroy,
	.fb_check_var	= cpustc_vga_check_var,
	.fb_set_par	= cpustc_vga_set_par,
	.fb_setcolreg	= cpustc_vga_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static struct simplefb_format cpustc_vga_formats[] = SIMPLEFB_FORMATS;

struct cpustc_vga_params {
	u32 width;
	u32 height;
	u32 stride;
	struct simplefb_format *format;
};

static int cpustc_vga_parse_dt(struct platform_device *pdev,
			       struct cpustc_vga_params *params)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;
	const char *format;
	int i;

	ret = of_property_read_u32(np, "width", &params->width);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse width property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "height", &params->height);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse height property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "stride", &params->stride);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse stride property\n");
		return ret;
	}

	ret = of_property_read_string(np, "format", &format);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse format property\n");
		return ret;
	}
	params->format = NULL;
	for (i = 0; i < ARRAY_SIZE(cpustc_vga_formats); i++) {
		if (strcmp(format, cpustc_vga_formats[i].name))
			continue;
		params->format = &cpustc_vga_formats[i];
		break;
	}
	if (!params->format) {
		dev_err(&pdev->dev, "Invalid format value\n");
		return -EINVAL;
	}

	return 0;
}

static int cpustc_vga_parse_pd(struct platform_device *pdev,
			      struct cpustc_vga_params *params)
{
	struct simplefb_platform_data *pd = dev_get_platdata(&pdev->dev);
	int i;

	params->width = pd->width;
	params->height = pd->height;
	params->stride = pd->stride;

	params->format = NULL;
	for (i = 0; i < ARRAY_SIZE(cpustc_vga_formats); i++) {
		if (strcmp(pd->format, cpustc_vga_formats[i].name))
			continue;

		params->format = &cpustc_vga_formats[i];
		break;
	}

	if (!params->format) {
		dev_err(&pdev->dev, "Invalid format value\n");
		return -EINVAL;
	}

	return 0;
}

#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
/*
 * Clock handling code.
 *
 * Here we handle the clocks property of our "cpustc,vga" DT node.
 * This is necessary so that we can make sure that any clocks needed by
 * the display engine that the bootloader set up for us (and for which it
 * provided a cpustc-vga DT node), stay up, for the life of the cpustc-vga
 * driver.
 *
 * When the driver unloads, we cleanly disable, and then release the clocks.
 *
 * We only complain about errors here, no action is taken as the most likely
 * error can only happen due to a mismatch between the bootloader which set
 * up cpustc-vga, and the clock definitions in the device tree. Chances are
 * that there are no adverse effects, and if there are, a clean teardown of
 * the fb probe will not help us much either. So just complain and carry on,
 * and hope that the user actually gets a working fb at the end of things.
 */
static int cpustc_vga_clocks_get(struct cpustc_vga_par *par,
				 struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk *clock;
	int i;

	if (dev_get_platdata(&pdev->dev) || !np)
		return 0;

	par->clk_count = of_clk_get_parent_count(np);
	if (!par->clk_count)
		return 0;

	par->clks = kcalloc(par->clk_count, sizeof(struct clk *), GFP_KERNEL);
	if (!par->clks)
		return -ENOMEM;

	for (i = 0; i < par->clk_count; i++) {
		clock = of_clk_get(np, i);
		if (IS_ERR(clock)) {
			if (PTR_ERR(clock) == -EPROBE_DEFER) {
				while (--i >= 0) {
					if (par->clks[i])
						clk_put(par->clks[i]);
				}
				kfree(par->clks);
				return -EPROBE_DEFER;
			}
			dev_err(&pdev->dev, "%s: clock %d not found: %ld\n",
				__func__, i, PTR_ERR(clock));
			continue;
		}
		par->clks[i] = clock;
	}

	return 0;
}

static void cpustc_vga_clocks_enable(struct cpustc_vga_par *par,
				     struct platform_device *pdev)
{
	int i, ret;

	for (i = 0; i < par->clk_count; i++) {
		if (par->clks[i]) {
			ret = clk_prepare_enable(par->clks[i]);
			if (ret) {
				dev_err(&pdev->dev,
					"%s: failed to enable clock %d: %d\n",
					__func__, i, ret);
				clk_put(par->clks[i]);
				par->clks[i] = NULL;
			}
		}
	}
	par->clks_enabled = true;
}

static void cpustc_vga_clocks_destroy(struct cpustc_vga_par *par)
{
	int i;

	if (!par->clks)
		return;

	for (i = 0; i < par->clk_count; i++) {
		if (par->clks[i]) {
			if (par->clks_enabled)
				clk_disable_unprepare(par->clks[i]);
			clk_put(par->clks[i]);
		}
	}

	kfree(par->clks);
}
#else
static int cpustc_vga_clocks_get(struct cpustc_vga_par *par,
	struct platform_device *pdev) { return 0; }
static void cpustc_vga_clocks_enable(struct cpustc_vga_par *par,
	struct platform_device *pdev) { }
static void cpustc_vga_clocks_destroy(struct cpustc_vga_par *par) { }
#endif

static int cpustc_vga_probe(struct platform_device *pdev)
{
	int ret;
	struct cpustc_vga_params params;
	struct fb_info *info;
	struct cpustc_vga_par *par;
	struct resource *mem;

	if (fb_get_options("cpustc-vga", NULL))
		return -ENODEV;

	ret = -ENODEV;
	if (dev_get_platdata(&pdev->dev))
		ret = cpustc_vga_parse_pd(pdev, &params);
	else if (pdev->dev.of_node)
		ret = cpustc_vga_parse_dt(pdev, &params);

	if (ret)
		return ret;
	if (strcmp(params.format->name, "r5g6b5")) {
		dev_err(&pdev->dev, "Only the r5g6b5 format is supported\n");
		return -EINVAL;
	}
	if (params.stride != params.width * CPUSTC_VGA_BYTES_PER_PIXEL) {
		dev_err(&pdev->dev, "Stride must be width * 2 for RGB565\n");
		return -EINVAL;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct cpustc_vga_par), &pdev->dev);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);

	par = info->par;
	par->regs = devm_platform_ioremap_resource_byname(pdev, "control");
	if (IS_ERR(par->regs)) {
		ret = PTR_ERR(par->regs);
		goto error_fb_release;
	}

	info->fix = cpustc_vga_fix;
	info->fix.smem_start = mem->start;
	info->fix.smem_len = resource_size(mem);
	info->fix.line_length = params.stride;

	info->var = cpustc_vga_var;
	info->var.xres = params.width;
	info->var.yres = params.height;
	info->var.xres_virtual = params.width;
	info->var.yres_virtual = params.height;
	info->var.bits_per_pixel = params.format->bits_per_pixel;
	info->var.red = params.format->red;
	info->var.green = params.format->green;
	info->var.blue = params.format->blue;
	info->var.transp = params.format->transp;
	info->var.right_margin = readl(par->regs + CPUSTC_VGA_H_FRONT);
	info->var.hsync_len = readl(par->regs + CPUSTC_VGA_H_SYNC);
	info->var.left_margin = readl(par->regs + CPUSTC_VGA_H_BACK);
	info->var.lower_margin = readl(par->regs + CPUSTC_VGA_V_FRONT);
	info->var.vsync_len = readl(par->regs + CPUSTC_VGA_V_SYNC);
	info->var.upper_margin = readl(par->regs + CPUSTC_VGA_V_BACK);

	ret = cpustc_vga_check_var(&info->var, info);
	if (ret) {
		dev_err(&pdev->dev, "Unsupported initial framebuffer mode\n");
		goto error_fb_release;
	}

	info->apertures = alloc_apertures(1);
	if (!info->apertures) {
		ret = -ENOMEM;
		goto error_fb_release;
	}
	info->apertures->ranges[0].base = info->fix.smem_start;
	info->apertures->ranges[0].size = info->fix.smem_len;

	info->fbops = &cpustc_vga_ops;
	info->flags = FBINFO_DEFAULT | FBINFO_MISC_FIRMWARE;
	info->screen_base = ioremap(info->fix.smem_start,
				    info->fix.smem_len);
	if (!info->screen_base) {
		ret = -ENOMEM;
		goto error_fb_release;
	}
	info->pseudo_palette = par->palette;

	ret = cpustc_vga_clocks_get(par, pdev);
	if (ret < 0)
		goto error_unmap;

	cpustc_vga_clocks_enable(par, pdev);
	cpustc_vga_set_par(info);

	dev_info(&pdev->dev, "framebuffer at 0x%lx, 0x%x bytes\n",
			     info->fix.smem_start, info->fix.smem_len);
	dev_info(&pdev->dev, "format=%s, mode=%dx%dx%d, linelength=%d\n",
			     params.format->name,
			     info->var.xres, info->var.yres,
			     info->var.bits_per_pixel, info->fix.line_length);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register cpustc-vga: %d\n", ret);
		goto error_clocks;
	}

	dev_info(&pdev->dev, "fb%d: cpustc-vga registered!\n", info->node);

	return 0;
error_clocks:
	cpustc_vga_clocks_destroy(par);
error_unmap:
	iounmap(info->screen_base);
error_fb_release:
	framebuffer_release(info);
	return ret;
}

static int cpustc_vga_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	unregister_framebuffer(info);
	framebuffer_release(info);

	return 0;
}

static const struct of_device_id cpustc_vga_of_match[] = {
	{ .compatible = "cpustc,vga", },
	{ },
};
MODULE_DEVICE_TABLE(of, cpustc_vga_of_match);

static struct platform_driver cpustc_vga_driver = {
	.driver = {
		.name = "cpustc-vga",
		.of_match_table = cpustc_vga_of_match,
	},
	.probe = cpustc_vga_probe,
	.remove = cpustc_vga_remove,
};

static int __init cpustc_vga_init(void)
{
	int ret;
	struct device_node *np;

	ret = platform_driver_register(&cpustc_vga_driver);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_OF_ADDRESS) && of_chosen) {
		for_each_child_of_node(of_chosen, np) {
			if (of_device_is_compatible(np, "cpustc,vga"))
				of_platform_device_create(np, NULL, NULL);
		}
	}

	return 0;
}

fs_initcall(cpustc_vga_init);

MODULE_AUTHOR("Stephen Warren <swarren@wwwdotorg.org>");
MODULE_DESCRIPTION("CPUSTC VGA framebuffer driver");
MODULE_LICENSE("GPL v2");
