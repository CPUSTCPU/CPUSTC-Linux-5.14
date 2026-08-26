// SPDX-License-Identifier: GPL-2.0
/*
 * CPUSTC NAND controller support.
 *
 * The register programming and DMA descriptor format match the controller
 * implementation used by the board's verified U-Boot NAND driver.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define CPUSTC_NAND_CMD			0x00
#define CPUSTC_NAND_ADDR_LOW		0x04
#define CPUSTC_NAND_ADDR_HIGH		0x08
#define CPUSTC_NAND_TIMING		0x0c
#define CPUSTC_NAND_ID_LOW		0x10
#define CPUSTC_NAND_STATUS_IDHIGH		0x14
#define CPUSTC_NAND_PARAM			0x18
#define CPUSTC_NAND_OP_NUM		0x1c
#define CPUSTC_NAND_CE_MAP0		0x20

#define CPUSTC_CMD_VALID			BIT(0)
#define CPUSTC_CMD_READ			BIT(1)
#define CPUSTC_CMD_WRITE			BIT(2)
#define CPUSTC_CMD_ERASE_ONE		BIT(3)
#define CPUSTC_CMD_READ_ID		BIT(5)
#define CPUSTC_CMD_RESET			BIT(6)
#define CPUSTC_CMD_OP_MAIN		BIT(8)
#define CPUSTC_CMD_OP_SPARE		BIT(9)
#define CPUSTC_CMD_DONE			BIT(10)

#define CPUSTC_DMA_INT_MASK		BIT(0)
#define CPUSTC_DMA_START			BIT(3)
#define CPUSTC_DMA_WRITE			BIT(12)
#define CPUSTC_DMA_ACCESS_ADDR		0x1fe0c040

#define CPUSTC_NAND_PARAM_1G		0x08005300
#define CPUSTC_NAND_CS_RDY_MAP		0x88442200
#define CPUSTC_NAND_TIMING_VALUE		0x00000412
#define CPUSTC_NAND_TIMEOUT_US		2000000
#define CPUSTC_NAND_BUFFER_SIZE		4096
#define CPUSTC_DMA_DESC_SIZE		32
#define CPUSTC_DMA_DESC_ALIGN		32

struct cpustc_dma_desc {
	u32 order_addr;
	u32 source_addr;
	u32 dest_addr;
	u32 length;
	u32 step_length;
	u32 step_times;
	u32 command;
};

struct cpustc_nand {
	struct nand_chip chip;
	struct device *dev;
	void __iomem *nand_base;
	void __iomem *dma_order;

	u8 id_data[8];
	u8 *buffer;
	dma_addr_t buffer_dma;
	struct cpustc_dma_desc *desc;
	dma_addr_t desc_dma;

	unsigned int pos;
	unsigned int count;
	unsigned int buffer_origin;
	unsigned int seqin_column;
	unsigned int seqin_page;
	unsigned int write_len;
	int last_error;
	bool buffer_active;
};

static void cpustc_nand_write(struct cpustc_nand *priv, u32 value,
			    unsigned int reg)
{
	writel(value, priv->nand_base + reg);
}

static u32 cpustc_nand_read(struct cpustc_nand *priv, unsigned int reg)
{
	return readl(priv->nand_base + reg);
}

static int cpustc_nand_wait_done(struct cpustc_nand *priv)
{
	u32 value;

	return readl_poll_timeout(priv->nand_base + CPUSTC_NAND_CMD, value,
				  value & CPUSTC_CMD_DONE, 1,
				  CPUSTC_NAND_TIMEOUT_US);
}

static void cpustc_nand_start_command(struct cpustc_nand *priv, u32 command)
{
	cpustc_nand_write(priv, 0, CPUSTC_NAND_CMD);
	cpustc_nand_write(priv, command | CPUSTC_CMD_VALID, CPUSTC_NAND_CMD);
}

static void cpustc_nand_set_operation(struct cpustc_nand *priv,
				    unsigned int page,
				    unsigned int column,
				    unsigned int length,
				    u32 command)
{
	u32 param = cpustc_nand_read(priv, CPUSTC_NAND_PARAM);

	cpustc_nand_write(priv, 0, CPUSTC_NAND_CMD);
	cpustc_nand_write(priv, column, CPUSTC_NAND_ADDR_LOW);
	cpustc_nand_write(priv, page, CPUSTC_NAND_ADDR_HIGH);
	cpustc_nand_write(priv, length, CPUSTC_NAND_OP_NUM);
	param = (param & 0xc000ffff) | (length << 16);
	cpustc_nand_write(priv, param, CPUSTC_NAND_PARAM);
	cpustc_nand_write(priv, command | CPUSTC_CMD_VALID, CPUSTC_NAND_CMD);
}

static int cpustc_nand_dma_transfer(struct cpustc_nand *priv,
				  unsigned int length, bool write)
{
	struct cpustc_dma_desc *desc = priv->desc;
	u32 value;
	int ret;

	if (!length || length > CPUSTC_NAND_BUFFER_SIZE)
		return -EINVAL;

	memset(desc, 0, CPUSTC_DMA_DESC_SIZE);
	desc->source_addr = lower_32_bits(priv->buffer_dma);
	desc->dest_addr = CPUSTC_DMA_ACCESS_ADDR;
	desc->length = DIV_ROUND_UP(length, sizeof(u32));
	desc->step_times = 1;
	desc->command = CPUSTC_DMA_INT_MASK | (write ? CPUSTC_DMA_WRITE : 0);

	/* Publish the descriptor before starting the DMA engine. */
	wmb();
	writel(lower_32_bits(priv->desc_dma) | CPUSTC_DMA_START,
	       priv->dma_order);

	ret = readl_poll_timeout(priv->dma_order, value,
				 !(value & CPUSTC_DMA_START), 1,
				 CPUSTC_NAND_TIMEOUT_US);
	if (ret)
		return ret;

	return cpustc_nand_wait_done(priv);
}

static void cpustc_nand_read_id(struct cpustc_nand *priv)
{
	u32 id_low;
	u32 id_high;

	cpustc_nand_start_command(priv, CPUSTC_CMD_READ_ID);
	udelay(1);
	id_low = cpustc_nand_read(priv, CPUSTC_NAND_ID_LOW);
	id_high = cpustc_nand_read(priv, CPUSTC_NAND_STATUS_IDHIGH);

	priv->id_data[0] = id_high & 0xff;
	priv->id_data[1] = (id_low >> 24) & 0xff;
	priv->id_data[2] = (id_low >> 16) & 0xff;
	priv->id_data[3] = (id_low >> 8) & 0xff;
	priv->id_data[4] = id_low & 0xff;
	priv->pos = 0;
	priv->count = 5;
	priv->buffer_active = false;
}

static void cpustc_nand_read_page(struct cpustc_nand *priv, int page,
				int column, bool oob_only)
{
	struct mtd_info *mtd = nand_to_mtd(&priv->chip);
	unsigned int origin = oob_only ? mtd->writesize : 0;
	unsigned int length = oob_only ? mtd->oobsize :
					       mtd->writesize + mtd->oobsize;
	u32 command = CPUSTC_CMD_READ | CPUSTC_CMD_OP_SPARE;

	priv->last_error = 0;
	priv->buffer_origin = origin;
	priv->pos = column > 0 ? min_t(unsigned int, column, length) : 0;
	priv->count = length;
	priv->buffer_active = true;
	memset(priv->buffer, 0xff, CPUSTC_NAND_BUFFER_SIZE);

	if (length > CPUSTC_NAND_BUFFER_SIZE) {
		priv->last_error = -EOVERFLOW;
		return;
	}

	if (!oob_only)
		command |= CPUSTC_CMD_OP_MAIN;

	cpustc_nand_set_operation(priv, page, origin, length, command);
	priv->last_error = cpustc_nand_dma_transfer(priv, length, false);
	if (priv->last_error)
		dev_err_ratelimited(priv->dev,
				    "page %d read failed: %d\n",
				    page, priv->last_error);
}

static void cpustc_nand_program_page(struct cpustc_nand *priv)
{
	struct mtd_info *mtd = nand_to_mtd(&priv->chip);
	u32 command = CPUSTC_CMD_WRITE | CPUSTC_CMD_OP_SPARE;

	if (priv->last_error)
		return;

	if (!priv->buffer_active || !priv->write_len) {
		priv->last_error = -EINVAL;
		return;
	}

	if (priv->seqin_column < mtd->writesize)
		command |= CPUSTC_CMD_OP_MAIN;

	cpustc_nand_set_operation(priv, priv->seqin_page,
				priv->seqin_column, priv->write_len, command);
	priv->last_error = cpustc_nand_dma_transfer(priv, priv->write_len,
						  true);
	priv->buffer_active = false;
	if (priv->last_error)
		dev_err(priv->dev, "page %u program failed: %d\n",
			priv->seqin_page, priv->last_error);
}

static void cpustc_nand_erase_block(struct cpustc_nand *priv, int page)
{
	priv->last_error = 0;
	priv->buffer_active = false;
	cpustc_nand_set_operation(priv, page, 0, 0, CPUSTC_CMD_ERASE_ONE);
	priv->last_error = cpustc_nand_wait_done(priv);
	if (priv->last_error)
		dev_err(priv->dev, "block at page %d erase failed: %d\n",
			page, priv->last_error);
}

static void cpustc_nand_cmdfunc(struct nand_chip *chip, unsigned int command,
			      int column, int page)
{
	struct cpustc_nand *priv = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	unsigned int total = mtd->writesize + mtd->oobsize;

	switch (command) {
	case NAND_CMD_RESET:
		priv->buffer_active = false;
		priv->last_error = 0;
		cpustc_nand_start_command(priv, CPUSTC_CMD_RESET);
		priv->last_error = cpustc_nand_wait_done(priv);
		if (priv->last_error)
			dev_err(priv->dev, "reset timed out\n");
		break;
	case NAND_CMD_READID:
		priv->last_error = 0;
		cpustc_nand_read_id(priv);
		break;
	case NAND_CMD_STATUS:
		priv->id_data[0] =
			(cpustc_nand_read(priv, CPUSTC_NAND_STATUS_IDHIGH) >> 16) |
			NAND_STATUS_WP;
		priv->pos = 0;
		priv->count = 1;
		priv->buffer_active = false;
		break;
	case NAND_CMD_READ0:
		cpustc_nand_read_page(priv, page, column, false);
		break;
	case NAND_CMD_READOOB:
		cpustc_nand_read_page(priv, page, column, true);
		break;
	case NAND_CMD_RNDOUT:
		if (column < priv->buffer_origin ||
		    column - priv->buffer_origin > priv->count)
			priv->pos = priv->count;
		else
			priv->pos = column - priv->buffer_origin;
		break;
	case NAND_CMD_RNDOUTSTART:
		break;
	case NAND_CMD_SEQIN:
		priv->last_error = 0;
		priv->buffer_active = true;
		priv->seqin_column = column;
		priv->seqin_page = page;
		priv->buffer_origin = column;
		priv->pos = 0;
		priv->write_len = 0;
		memset(priv->buffer, 0xff, CPUSTC_NAND_BUFFER_SIZE);
		if (column < 0 || column > total) {
			priv->count = 0;
			priv->last_error = -EINVAL;
		} else {
			priv->count = total - column;
		}
		break;
	case NAND_CMD_RNDIN:
		if (column < priv->buffer_origin ||
		    column - priv->buffer_origin > priv->count) {
			priv->last_error = -EINVAL;
		} else {
			priv->pos = column - priv->buffer_origin;
		}
		break;
	case NAND_CMD_PAGEPROG:
		cpustc_nand_program_page(priv);
		break;
	case NAND_CMD_ERASE1:
		cpustc_nand_erase_block(priv, page);
		break;
	case NAND_CMD_ERASE2:
	case NAND_CMD_READ1:
		break;
	default:
		memset(priv->id_data, 0xff, sizeof(priv->id_data));
		priv->pos = 0;
		priv->count = sizeof(priv->id_data);
		priv->buffer_active = false;
		dev_warn_ratelimited(priv->dev,
				     "unsupported NAND command 0x%x\n",
				     command);
		break;
	}
}

static u8 cpustc_nand_read_byte(struct nand_chip *chip)
{
	struct cpustc_nand *priv = nand_get_controller_data(chip);
	u8 *buffer = priv->buffer_active ? priv->buffer : priv->id_data;

	if (priv->pos >= priv->count)
		return 0xff;

	return buffer[priv->pos++];
}

static void cpustc_nand_read_buf(struct nand_chip *chip, u8 *buf, int len)
{
	struct cpustc_nand *priv = nand_get_controller_data(chip);
	u8 *source = priv->buffer_active ? priv->buffer : priv->id_data;
	unsigned int available = priv->pos < priv->count ?
				 priv->count - priv->pos : 0;
	unsigned int copy = min_t(unsigned int, len, available);

	memcpy(buf, source + priv->pos, copy);
	if (copy < len)
		memset(buf + copy, 0xff, len - copy);
	priv->pos += copy;
}

static void cpustc_nand_write_buf(struct nand_chip *chip, const u8 *buf,
				int len)
{
	struct cpustc_nand *priv = nand_get_controller_data(chip);
	unsigned int available = priv->pos < priv->count ?
				 priv->count - priv->pos : 0;
	unsigned int copy = min_t(unsigned int, len, available);

	memcpy(priv->buffer + priv->pos, buf, copy);
	priv->pos += copy;
	priv->write_len = max(priv->write_len, priv->pos);
	if (copy < len)
		priv->last_error = -ENOSPC;
}

static int cpustc_nand_dev_ready(struct nand_chip *chip)
{
	struct cpustc_nand *priv = nand_get_controller_data(chip);

	return !!(cpustc_nand_read(priv, CPUSTC_NAND_CMD) & CPUSTC_CMD_DONE);
}

static int cpustc_nand_waitfunc(struct nand_chip *chip)
{
	struct cpustc_nand *priv = nand_get_controller_data(chip);

	if (priv->last_error)
		return NAND_STATUS_FAIL | NAND_STATUS_READY | NAND_STATUS_WP;

	return (cpustc_nand_read(priv, CPUSTC_NAND_STATUS_IDHIGH) >> 16) |
		NAND_STATUS_WP;
}

static void cpustc_nand_select_chip(struct nand_chip *chip, int cs)
{
}

static int cpustc_nand_attach_chip(struct nand_chip *chip)
{
	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_SOFT;
	chip->ecc.algo = NAND_ECC_ALGO_HAMMING;

	return 0;
}

static const struct nand_controller_ops cpustc_nand_controller_ops = {
	.attach_chip = cpustc_nand_attach_chip,
};

static void cpustc_nand_hw_init(struct cpustc_nand *priv)
{
	cpustc_nand_write(priv, CPUSTC_NAND_TIMING_VALUE, CPUSTC_NAND_TIMING);
	cpustc_nand_write(priv, CPUSTC_NAND_PARAM_1G, CPUSTC_NAND_PARAM);
	cpustc_nand_write(priv, CPUSTC_NAND_CS_RDY_MAP, CPUSTC_NAND_CE_MAP0);
	cpustc_nand_write(priv, 0, CPUSTC_NAND_OP_NUM);
}

static int cpustc_nand_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpustc_nand *priv;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->nand_base =
		devm_platform_ioremap_resource_byname(pdev, "nand");
	if (IS_ERR(priv->nand_base))
		return PTR_ERR(priv->nand_base);

	priv->dma_order =
		devm_platform_ioremap_resource_byname(pdev, "dma-order");
	if (IS_ERR(priv->dma_order))
		return PTR_ERR(priv->dma_order);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	priv->buffer = dmam_alloc_coherent(dev, CPUSTC_NAND_BUFFER_SIZE,
					   &priv->buffer_dma, GFP_KERNEL);
	if (!priv->buffer)
		return -ENOMEM;

	priv->desc = dmam_alloc_coherent(dev, CPUSTC_DMA_DESC_SIZE,
					 &priv->desc_dma, GFP_KERNEL);
	if (!priv->desc)
		return -ENOMEM;

	if (!IS_ALIGNED((unsigned long)priv->desc, CPUSTC_DMA_DESC_ALIGN) ||
	    !IS_ALIGNED(priv->desc_dma, CPUSTC_DMA_DESC_ALIGN))
		return dev_err_probe(dev, -EINVAL,
				     "DMA descriptor is not 32-byte aligned\n");

	chip = &priv->chip;
	mtd = nand_to_mtd(chip);
	mtd->dev.parent = dev;
	mtd->owner = THIS_MODULE;
	nand_set_flash_node(chip, dev->of_node);
	nand_set_controller_data(chip, priv);

	chip->legacy.cmdfunc = cpustc_nand_cmdfunc;
	chip->legacy.read_byte = cpustc_nand_read_byte;
	chip->legacy.read_buf = cpustc_nand_read_buf;
	chip->legacy.write_buf = cpustc_nand_write_buf;
	chip->legacy.select_chip = cpustc_nand_select_chip;
	chip->legacy.dev_ready = cpustc_nand_dev_ready;
	chip->legacy.waitfunc = cpustc_nand_waitfunc;
	chip->legacy.chip_delay = 50;
	chip->legacy.dummy_controller.ops = &cpustc_nand_controller_ops;
	chip->options |= NAND_NO_SUBPAGE_WRITE;

	cpustc_nand_hw_init(priv);

	ret = nand_scan(chip, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to scan NAND\n");

	if (mtd->writesize + mtd->oobsize > CPUSTC_NAND_BUFFER_SIZE) {
		dev_err(dev, "unsupported page and OOB size: %u + %u\n",
			mtd->writesize, mtd->oobsize);
		ret = -EOVERFLOW;
		goto cleanup_nand;
	}

	ret = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
	if (ret)
		goto cleanup_nand;

	platform_set_drvdata(pdev, priv);
	dev_info(dev, "registered %llu MiB NAND with software Hamming ECC\n",
		 (unsigned long long)(mtd->size >> 20));

	return 0;

cleanup_nand:
	nand_cleanup(chip);
	return ret;
}

static int cpustc_nand_remove(struct platform_device *pdev)
{
	struct cpustc_nand *priv = platform_get_drvdata(pdev);
	int ret;

	ret = mtd_device_unregister(nand_to_mtd(&priv->chip));
	WARN_ON(ret);
	nand_cleanup(&priv->chip);

	return ret;
}

static const struct of_device_id cpustc_nand_of_match[] = {
	{ .compatible = "cpustc,nand" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpustc_nand_of_match);

static struct platform_driver cpustc_nand_driver = {
	.probe = cpustc_nand_probe,
	.remove = cpustc_nand_remove,
	.driver = {
		.name = "cpustc-nand",
		.of_match_table = cpustc_nand_of_match,
	},
};
module_platform_driver(cpustc_nand_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CPUSTC NAND controller driver");
