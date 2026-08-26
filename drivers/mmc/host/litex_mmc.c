// SPDX-License-Identifier: GPL-2.0
/*
 * LiteX LiteSDCard driver
 *
 * Copyright (C) 2019-2020 Antmicro <contact@antmicro.com>
 * Copyright (C) 2019-2020 Kamil Rakoczy <krakoczy@antmicro.com>
 * Copyright (C) 2019-2020 Maciej Dudek <mdudek@internships.antmicro.com>
 * Copyright (C) 2020 Paul Mackerras <paulus@ozlabs.org>
 * Copyright (C) 2020-2022 Gabriel Somlo <gsomlo@gmail.com>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/litex.h>
#include <linux/math.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#define LITEX_PHY_CARDDETECT  0x00
#define LITEX_PHY_CLOCKERDIV  0x04
#define LITEX_PHY_INITIALIZE  0x08
#define LITEX_PHY_WRITESTATUS 0x0C
#define LITEX_CORE_CMDARG     0x00
#define LITEX_CORE_CMDCMD     0x04
#define LITEX_CORE_CMDSND     0x08
#define LITEX_CORE_CMDRSP     0x0C
#define LITEX_CORE_CMDEVT     0x1C
#define LITEX_CORE_DATEVT     0x20
#define LITEX_CORE_BLKLEN     0x24
#define LITEX_CORE_BLKCNT     0x28
#define LITEX_BLK2MEM_BASE    0x00
#define LITEX_BLK2MEM_LEN     0x08
#define LITEX_BLK2MEM_ENA     0x0C
#define LITEX_BLK2MEM_DONE    0x10
#define LITEX_BLK2MEM_LOOP    0x14
#define LITEX_BLK2MEM_OFFSET  0x18
#define LITEX_MEM2BLK_BASE    0x00
#define LITEX_MEM2BLK_LEN     0x08
#define LITEX_MEM2BLK_ENA     0x0C
#define LITEX_MEM2BLK_DONE    0x10
#define LITEX_MEM2BLK_LOOP    0x14
#define LITEX_MEM2BLK_OFFSET  0x18
#define LITEX_IRQ_STATUS      0x00
#define LITEX_IRQ_PENDING     0x04
#define LITEX_IRQ_ENABLE      0x08

#define LITEX_SG_CAP             0x00
#define LITEX_SG_TABLE_ADDR      0x04
#define LITEX_SG_ENTRY_COUNT     0x08
#define LITEX_SG_TOTAL_BYTES     0x0c
#define LITEX_SG_CONTROL         0x10
#define LITEX_SG_STATUS          0x14
#define LITEX_SG_CURRENT_INDEX   0x18
#define LITEX_SG_ERROR_INDEX     0x1c
#define LITEX_SG_COMPLETED       0x20
#define LITEX_SG_FETCH_CYCLES    0x24
#define LITEX_SG_MAX_GAP_CYCLES  0x28
#define LITEX_SG_ERROR_DETAIL    0x2c

#define LITEX_SG_CAP_V1             0x01080100
#define LITEX_SG_MAX_ENTRIES        256
#define LITEX_SG_ENTRY_SIZE         8
#define LITEX_SG_TABLE_SIZE         (LITEX_SG_MAX_ENTRIES * \
					     LITEX_SG_ENTRY_SIZE)

#define LITEX_SG_CONTROL_ENABLE     BIT(0)
#define LITEX_SG_CONTROL_ARM        BIT(1)
#define LITEX_SG_CONTROL_ABORT      BIT(2)
#define LITEX_SG_CONTROL_CLEAR      BIT(3)

#define LITEX_SG_STATUS_ENABLED     BIT(0)
#define LITEX_SG_STATUS_FETCH_BUSY  BIT(1)
#define LITEX_SG_STATUS_TABLE_READY BIT(2)
#define LITEX_SG_STATUS_ACTIVE      BIT(3)
#define LITEX_SG_STATUS_DONE        BIT(4)
#define LITEX_SG_STATUS_ERROR       BIT(5)
#define LITEX_SG_STATUS_ABORTED     BIT(6)
#define LITEX_SG_STATUS_TABLE_OWNER BIT(7)
#define LITEX_SG_STATUS_ERROR_SHIFT 8
#define LITEX_SG_STATUS_ERROR_MASK  GENMASK(15, 8)
#define LITEX_SG_STATUS_QUIESCENT_MASK \
	(LITEX_SG_STATUS_FETCH_BUSY | LITEX_SG_STATUS_TABLE_READY | \
	 LITEX_SG_STATUS_ACTIVE | LITEX_SG_STATUS_TABLE_OWNER)

#define SD_CTL_DATA_XFER_NONE  0
#define SD_CTL_DATA_XFER_READ  1
#define SD_CTL_DATA_XFER_WRITE 2

#define SD_CTL_RESP_NONE       0
#define SD_CTL_RESP_SHORT      1
#define SD_CTL_RESP_LONG       2
#define SD_CTL_RESP_SHORT_BUSY 3

#define SD_BIT_DONE    BIT(0)
#define SD_BIT_WR_ERR  BIT(1)
#define SD_BIT_TIMEOUT BIT(2)
#define SD_BIT_CRC_ERR BIT(3)

#define LITEX_DMA_DONE_SUCCESS  BIT(0)
#define LITEX_DMA_DONE_SLVERR   BIT(1)
#define LITEX_DMA_DONE_DECERR   BIT(2)
#define LITEX_DMA_DONE_ABORTED  BIT(3)
#define LITEX_DMA_DONE_MISMATCH BIT(4)
#define LITEX_DMA_DONE_BUSY     BIT(5)
#define LITEX_DMA_DONE_INVALID  BIT(6)
#define LITEX_DMA_DONE_TERMINAL (LITEX_DMA_DONE_SUCCESS | \
				 LITEX_DMA_DONE_SLVERR | \
				 LITEX_DMA_DONE_DECERR | \
				 LITEX_DMA_DONE_ABORTED | \
				 LITEX_DMA_DONE_MISMATCH | \
				 LITEX_DMA_DONE_INVALID)

#define SD_SLEEP_US       5
#define SD_TIMEOUT_US          20000
#define SD_IRQ_TIMEOUT_US    1000000
#define SD_DATA_TIMEOUT_MAX_US 30000000

#define SD_INIT_DELAY_US  1000
#define SD_INIT_CLK_HZ    400000

#define LITEX_MMC_MAX_REQ_SIZE SZ_1M
#define LITEX_MMC_DIRECT_MAX_SEGS 1
#define LITEX_MMC_ASYNC_SLOTS 2
#define LITEX_BAD_READ_SNAPSHOT_SIZE SZ_4K
#define LITEX_BAD_READ_NON_FF_SAMPLES 16

#define SDIRQ_CARD_DETECT    BIT(0)
#define SDIRQ_SD_TO_MEM_DONE BIT(1)
#define SDIRQ_MEM_TO_SD_DONE BIT(2)
#define SDIRQ_DATA_DONE      BIT(3)
#define SDIRQ_CMD_DONE       BIT(4)

enum litex_mmc_dma_path {
	LITEX_MMC_DMA_BOUNCE,
	LITEX_MMC_DMA_DIRECT,
	LITEX_MMC_DMA_SG,
};

enum litex_mmc_dma_cookie {
	LITEX_MMC_COOKIE_UNMAPPED,
	LITEX_MMC_COOKIE_PREMAPPED,
	LITEX_MMC_COOKIE_MAPPED,
};

struct litex_mmc_sg_entry {
	__le32 dma_addr;
	__le32 length;
};

static_assert(sizeof(struct litex_mmc_sg_entry) == LITEX_SG_ENTRY_SIZE);

struct litex_mmc_dma_context {
	enum litex_mmc_dma_path path;
	enum dma_data_direction direction;
	dma_addr_t frontend_dma;
	u32 len;
	u32 min_segment_bytes;
	u32 max_segment_bytes;
	u32 sg_status;
	u32 sg_fetch_cycles;
	u32 sg_max_gap_cycles;
	int mapped_nents;
	u8 transfer;
	bool mapped;
	bool sg_armed;
	bool frontend_programmed;
	bool write_dma_prestarted;
};

struct litex_mmc_bad_read_snapshot {
	bool valid;
	u32 generation;
	u64 request_sequence;
	u8 opcode;
	u32 cmd_arg;
	u32 requested_len;
	u32 copied_len;
	u8 dma_done;
	u32 dma_offset;
	u32 crc32;
	u32 non_ff_count;
	u8 non_ff_sample_count;
	u16 non_ff_offsets[LITEX_BAD_READ_NON_FF_SAMPLES];
	u8 non_ff_values[LITEX_BAD_READ_NON_FF_SAMPLES];
	u8 data[LITEX_BAD_READ_SNAPSHOT_SIZE];
};

struct litex_mmc_dma_stats {
	u64 requests;
	u64 bytes;
	u64 read_requests;
	u64 read_bytes;
	u64 write_requests;
	u64 write_bytes;
	u64 direct_requests;
	u64 direct_bytes;
	u64 sg_requests;
	u64 sg_bytes;
	u64 sg_entries;
	u64 bounce_requests;
	u64 bounce_bytes;
	u64 size_le_4k;
	u64 size_le_16k;
	u64 size_le_64k;
	u64 size_le_256k;
	u64 size_le_512k;
	u64 size_le_1m;
	u32 min_request_bytes;
	u32 max_request_bytes;
	u32 max_sg_len;
	u32 max_mapped_nents;
	u64 pre_req_count;
	u64 pre_req_mapped;
	u64 pre_req_failed;
	u64 pre_req_map_ns;
	u64 max_pre_req_map_ns;
	u64 post_req_count;
	u64 post_req_unmap_ns;
	u64 max_post_req_unmap_ns;
	u64 async_requests;
};

struct litex_mmc_cmd_timing {
	u64 setup_ns;
	u64 irq_wait_ns;
	u64 cmd_poll_ns;
	u64 response_ns;
	u64 data_poll_ns;
	u64 dma_poll_ns;
};

struct litex_mmc_dma_prep_timing {
	u64 map_ns;
	u64 bounce_copy_ns;
	u64 sg_table_build_ns;
	u64 sg_table_fetch_ns;
	u64 program_ns;
	u64 sg_activate_ns;
};

struct litex_mmc_request_timing {
	u64 request_start_ns;
	u64 sbc_ns;
	u64 bus_width_ns;
	struct litex_mmc_dma_prep_timing dma_prep;
	struct litex_mmc_cmd_timing command;
	u64 sg_complete_ns;
	u64 stop_ns;
	u64 frontend_abort_ns;
	u64 sg_abort_ns;
	u64 sg_cleanup_ns;
	u64 dma_unmap_ns;
	u64 post_ns;
};

struct litex_mmc_started_cmd {
	bool wait_irq;
	bool trace_large_read;
	u64 trace_start_ns;
	u64 wait_start_ns;
};

struct litex_mmc_host;

struct litex_mmc_async_request {
	struct work_struct work;
	struct litex_mmc_host *host;
	struct mmc_request *mrq;
	struct litex_mmc_dma_context dma_ctx;
	struct litex_mmc_request_timing timing;
	struct litex_mmc_started_cmd started_cmd;
	unsigned long data_timeout_us;
	u64 trace_start_ns;
	bool trace_large_read;
	bool profile_write;
	bool in_use;
};

struct litex_mmc_write_profile {
	u64 requests;
	u64 bytes;
	u64 direct_requests;
	u64 sg_requests;
	u64 bounce_requests;
	u64 cmd24_requests;
	u64 cmd25_requests;
	u64 failed_requests;
	u64 total_ns;
	u64 sbc_ns;
	u64 bus_width_ns;
	u64 dma_map_ns;
	u64 bounce_copy_ns;
	u64 sg_table_build_ns;
	u64 sg_table_fetch_ns;
	u64 dma_program_ns;
	u64 sg_activate_ns;
	u64 cmd_setup_ns;
	u64 irq_wait_ns;
	u64 cmd_poll_ns;
	u64 response_ns;
	u64 data_poll_ns;
	u64 dma_poll_ns;
	u64 sg_complete_ns;
	u64 stop_ns;
	u64 frontend_abort_ns;
	u64 sg_abort_ns;
	u64 sg_cleanup_ns;
	u64 dma_unmap_ns;
	u64 post_ns;
	u64 max_total_ns;
	u64 max_dma_map_ns;
	u64 max_irq_wait_ns;
	u64 max_data_poll_ns;
	u32 min_request_bytes;
	u32 max_request_bytes;
};

struct litex_mmc_sg_profile {
	u64 requests;
	u64 read_requests;
	u64 write_requests;
	u64 failed_requests;
	u64 aborted_requests;
	u64 bytes;
	u64 entries;
	u64 total_ns;
	u64 sbc_ns;
	u64 bus_width_ns;
	u64 dma_map_ns;
	u64 table_build_ns;
	u64 table_fetch_ns;
	u64 frontend_program_ns;
	u64 activate_ns;
	u64 command_setup_ns;
	u64 irq_wait_ns;
	u64 command_poll_ns;
	u64 response_ns;
	u64 data_poll_ns;
	u64 frontend_dma_poll_ns;
	u64 sg_complete_ns;
	u64 stop_ns;
	u64 frontend_abort_ns;
	u64 abort_ns;
	u64 cleanup_ns;
	u64 dma_unmap_ns;
	u64 post_ns;
	u64 fetch_cycles;
	u64 max_gap_cycles;
	u64 last_request_sequence;
	u64 last_total_ns;
	u64 last_sbc_ns;
	u64 last_bus_width_ns;
	u64 last_dma_map_ns;
	u64 last_table_build_ns;
	u64 last_table_fetch_ns;
	u64 last_frontend_program_ns;
	u64 last_activate_ns;
	u64 last_command_setup_ns;
	u64 last_irq_wait_ns;
	u64 last_command_poll_ns;
	u64 last_response_ns;
	u64 last_data_poll_ns;
	u64 last_frontend_dma_poll_ns;
	u64 last_sg_complete_ns;
	u64 last_stop_ns;
	u64 last_frontend_abort_ns;
	u64 last_abort_ns;
	u64 last_cleanup_ns;
	u64 last_dma_unmap_ns;
	u64 last_post_ns;
	u32 last_bytes;
	u32 last_entries;
	u32 last_min_segment_bytes;
	u32 last_max_segment_bytes;
	u32 last_status;
	u32 last_fetch_cycles;
	u32 last_max_gap_cycles;
	u8 last_transfer;
	bool last_failed;
};

struct litex_mmc_host {
	struct mmc_host *mmc;

	void __iomem *sdphy;
	void __iomem *sdcore;
	void __iomem *sdreader;
	void __iomem *sdwriter;
	void __iomem *sdirq;
	void __iomem *sdsg;

	void *buffer;
	size_t buf_size;
	dma_addr_t dma;
	struct litex_mmc_sg_entry *sg_table;
	dma_addr_t sg_table_dma;
	bool sg_supported;

	struct completion data_done;
	int irq;
	spinlock_t request_lock; /* Protects async slot ownership. */
	struct litex_mmc_async_request async[LITEX_MMC_ASYNC_SLOTS];

	unsigned int ref_clk;
	unsigned int sd_clk;

	u32 resp[4];
	u16 rca;

	bool is_bus_width_set;
	bool app_cmd;
	bool card_present;
	bool first_block_request_logged;
	bool large_read_transfer_timing_logged;
	bool large_read_request_timing_logged;
	bool error_snapshot_logged;
	u32 card_generation;
	u64 request_sequence;
	u64 active_request_sequence;
	struct litex_mmc_bad_read_snapshot bad_read;
	struct debugfs_blob_wrapper bad_read_blob;
	spinlock_t dma_stats_lock; /* Protects dma_stats on 32-bit systems. */
	struct litex_mmc_dma_stats dma_stats;
	struct litex_mmc_write_profile write_profile;
	struct litex_mmc_sg_profile sg_profile;
};

static unsigned long litex_mmc_data_timeout_us(struct litex_mmc_host *host,
					       struct mmc_data *data);

static const char *litex_mmc_dma_path_name(enum litex_mmc_dma_path path)
{
	switch (path) {
	case LITEX_MMC_DMA_DIRECT:
		return "direct";
	case LITEX_MMC_DMA_SG:
		return "sg";
	case LITEX_MMC_DMA_BOUNCE:
	default:
		return "bounce";
	}
}

static int litex_mmc_sg_status_to_errno(u32 status)
{
	u32 error_code = (status & LITEX_SG_STATUS_ERROR_MASK) >>
			 LITEX_SG_STATUS_ERROR_SHIFT;

	if (status & LITEX_SG_STATUS_ABORTED)
		return -ECANCELED;
	if (!(status & LITEX_SG_STATUS_ERROR))
		return 0;

	switch (error_code) {
	case 1: /* SG disabled */
	case 2: /* AXI read path busy */
	case 3: /* table address alignment */
	case 4: /* entry count */
	case 5: /* total length */
	case 8: /* payload address alignment */
	case 9: /* entry length */
	case 10: /* payload address overflow */
	case 11: /* accumulated length overflow */
	case 12: /* accumulated length mismatch */
	case 13: /* logical LiteSD length mismatch */
		return -EINVAL;
	case 15: /* aborted */
		return -ECANCELED;
	case 6: /* table AXI DMA response */
	case 7: /* table stream framing */
	case 14: /* payload stream framing */
	case 16: /* payload AXI DMA response */
	default:
		return -EIO;
	}
}

static void litex_mmc_log_sg_error(struct litex_mmc_host *host,
				   const char *stage, u32 status)
{
	struct device *dev = mmc_dev(host->mmc);
	u32 error_code = (status & LITEX_SG_STATUS_ERROR_MASK) >>
			 LITEX_SG_STATUS_ERROR_SHIFT;

	dev_err(dev,
		"SG %s failed: status=%#010x code=%u current=%u error_index=%u detail=%#010x fetch_cycles=%u max_gap_cycles=%u\n",
		stage, status, error_code,
		litex_read32(host->sdsg + LITEX_SG_CURRENT_INDEX),
		litex_read32(host->sdsg + LITEX_SG_ERROR_INDEX),
		litex_read32(host->sdsg + LITEX_SG_ERROR_DETAIL),
		litex_read32(host->sdsg + LITEX_SG_FETCH_CYCLES),
		litex_read32(host->sdsg + LITEX_SG_MAX_GAP_CYCLES));
}

static int litex_mmc_sg_clear(struct litex_mmc_host *host,
			      unsigned long timeout_us, u32 *last_status)
{
	void __iomem *status_reg = host->sdsg + LITEX_SG_STATUS;
	u32 status;
	int ret;

	if (!host->sg_supported)
		return 0;

	litex_write32(host->sdsg + LITEX_SG_CONTROL,
		      LITEX_SG_CONTROL_CLEAR);
	ret = readx_poll_timeout(litex_read32, status_reg, status,
				 !(status & (LITEX_SG_STATUS_ENABLED |
					    LITEX_SG_STATUS_QUIESCENT_MASK)),
				 SD_SLEEP_US, timeout_us);
	if (last_status)
		*last_status = status;

	return ret;
}

static int litex_mmc_build_sg_table(struct litex_mmc_host *host,
				    struct mmc_data *data,
				    struct litex_mmc_dma_context *ctx)
{
	struct scatterlist *sg;
	u64 total = 0;
	int i;

	if (ctx->mapped_nents < 2 ||
	    ctx->mapped_nents > LITEX_SG_MAX_ENTRIES)
		return -EINVAL;

	ctx->min_segment_bytes = U32_MAX;
	ctx->max_segment_bytes = 0;
	for_each_sg(data->sg, sg, ctx->mapped_nents, i) {
		dma_addr_t dma_addr = sg_dma_address(sg);
		u32 length = sg_dma_len(sg);
		u64 end = (u64)dma_addr + length;

		if ((u64)dma_addr > U32_MAX || !length ||
		    (dma_addr & 0x3) || (length & 0x3) ||
		    length > LITEX_MMC_MAX_REQ_SIZE || end > U32_MAX)
			return -EINVAL;
		if (total + length > LITEX_MMC_MAX_REQ_SIZE)
			return -EINVAL;

		host->sg_table[i].dma_addr = cpu_to_le32(lower_32_bits(dma_addr));
		host->sg_table[i].length = cpu_to_le32(length);
		total += length;
		ctx->min_segment_bytes = min(ctx->min_segment_bytes, length);
		ctx->max_segment_bytes = max(ctx->max_segment_bytes, length);
	}

	if (total != ctx->len)
		return -EINVAL;

	dma_wmb();
	return 0;
}

static int litex_mmc_sg_prepare(struct litex_mmc_host *host,
				struct litex_mmc_dma_context *ctx,
				unsigned long timeout_us, u32 *last_status)
{
	void __iomem *status_reg = host->sdsg + LITEX_SG_STATUS;
	u32 status = 0;
	int ret;

	ret = litex_mmc_sg_clear(host, timeout_us, &status);
	if (ret)
		goto out;

	litex_write32(host->sdsg + LITEX_SG_TABLE_ADDR,
		      lower_32_bits(host->sg_table_dma));
	litex_write32(host->sdsg + LITEX_SG_ENTRY_COUNT, ctx->mapped_nents);
	litex_write32(host->sdsg + LITEX_SG_TOTAL_BYTES, ctx->len);
	litex_write32(host->sdsg + LITEX_SG_CONTROL,
		      LITEX_SG_CONTROL_ENABLE | LITEX_SG_CONTROL_ARM);
	ctx->sg_armed = true;

	ret = readx_poll_timeout(litex_read32, status_reg, status,
				 status & (LITEX_SG_STATUS_TABLE_READY |
					  LITEX_SG_STATUS_ERROR),
				 SD_SLEEP_US, timeout_us);
	if (!ret && !(status & LITEX_SG_STATUS_TABLE_READY))
		ret = litex_mmc_sg_status_to_errno(status);
out:
	if (last_status)
		*last_status = status;
	if (ret)
		litex_mmc_log_sg_error(host, "prepare", status);

	return ret;
}

static int litex_mmc_sg_activate_write(struct litex_mmc_host *host,
				       unsigned long timeout_us, u32 *last_status)
{
	void __iomem *status_reg = host->sdsg + LITEX_SG_STATUS;
	u32 status = 0;
	int ret;

	/*
	 * SG must consume the logical descriptor before CMD25.  Its waitPayload
	 * state then keeps physical DDR descriptors gated until LiteSD requests
	 * write payload, preserving the upstream enable-before-command ordering
	 * without pre-filling the frontend FIFO.
	 */
	litex_write8(host->sdwriter + LITEX_MEM2BLK_ENA, 1);
	ret = readx_poll_timeout(litex_read32, status_reg, status,
				 status & (LITEX_SG_STATUS_ACTIVE |
					   LITEX_SG_STATUS_ERROR),
				 SD_SLEEP_US, timeout_us);
	if (!ret)
		ret = litex_mmc_sg_status_to_errno(status);
	if (!ret && !(status & LITEX_SG_STATUS_ACTIVE))
		ret = -EIO;
	if (last_status)
		*last_status = status;
	if (ret) {
		/* Let the SG abort path drain without waiting for card payload. */
		litex_write8(host->sdwriter + LITEX_MEM2BLK_ENA, 0);
		litex_mmc_log_sg_error(host, "activate", status);
	} else {
		dev_info_once(mmc_dev(host->mmc),
			      "SG write active before command: status=%#010x\n",
			      status);
	}

	return ret;
}

static int litex_mmc_sg_wait_done(struct litex_mmc_host *host,
				  struct litex_mmc_dma_context *ctx,
				  unsigned long timeout_us, u32 *last_status)
{
	void __iomem *status_reg = host->sdsg + LITEX_SG_STATUS;
	u32 status;
	int ret;

	ret = readx_poll_timeout(litex_read32, status_reg, status,
				 (status & (LITEX_SG_STATUS_DONE |
					   LITEX_SG_STATUS_ERROR |
					   LITEX_SG_STATUS_ABORTED)) &&
				 !(status & LITEX_SG_STATUS_QUIESCENT_MASK),
				 SD_SLEEP_US, timeout_us);
	if (!ret)
		ret = litex_mmc_sg_status_to_errno(status);
	if (last_status)
		*last_status = status;
	ctx->sg_fetch_cycles = litex_read32(host->sdsg +
						 LITEX_SG_FETCH_CYCLES);
	ctx->sg_max_gap_cycles = litex_read32(host->sdsg +
						   LITEX_SG_MAX_GAP_CYCLES);
	if (ret)
		litex_mmc_log_sg_error(host, "completion", status);

	return ret;
}

static int litex_mmc_sg_abort(struct litex_mmc_host *host,
			      struct litex_mmc_dma_context *ctx,
			      unsigned long timeout_us, u32 *last_status)
{
	void __iomem *status_reg = host->sdsg + LITEX_SG_STATUS;
	u32 status = 0;
	u32 terminal_status = 0;
	int ret;

	if (!ctx->sg_armed)
		return 0;

	litex_write32(host->sdsg + LITEX_SG_CONTROL,
		      LITEX_SG_CONTROL_ENABLE | LITEX_SG_CONTROL_ABORT);
	ret = readx_poll_timeout(litex_read32, status_reg, status,
				 !(status & (LITEX_SG_STATUS_FETCH_BUSY |
					    LITEX_SG_STATUS_ACTIVE |
					    LITEX_SG_STATUS_TABLE_OWNER)),
				 SD_SLEEP_US, timeout_us);
	if (ret)
		goto out;

	terminal_status = status;
	ctx->sg_fetch_cycles = litex_read32(host->sdsg +
						 LITEX_SG_FETCH_CYCLES);
	if (ctx->frontend_programmed)
		ctx->sg_max_gap_cycles = litex_read32(host->sdsg +
							   LITEX_SG_MAX_GAP_CYCLES);
	ret = litex_mmc_sg_clear(host, timeout_us, &status);
	if (!ret)
		ctx->sg_armed = false;
out:
	if (last_status)
		*last_status = terminal_status ? terminal_status : status;
	if (ret)
		litex_mmc_log_sg_error(host, "abort", status);

	return ret;
}

static void litex_mmc_log_failure(struct litex_mmc_host *host,
				  const char *stage, u8 cmd, u32 arg,
				  u8 transfer)
{
	struct device *dev = mmc_dev(host->mmc);
	u64 reader_base, writer_base;
	u32 irq_status = 0, irq_pending = 0, irq_enable = 0;

	if (host->error_snapshot_logged)
		return;
	host->error_snapshot_logged = true;

	reader_base = litex_read64(host->sdreader + LITEX_BLK2MEM_BASE);
	writer_base = litex_read64(host->sdwriter + LITEX_MEM2BLK_BASE);
	if (host->sdirq) {
		irq_status = litex_read32(host->sdirq + LITEX_IRQ_STATUS);
		irq_pending = litex_read32(host->sdirq + LITEX_IRQ_PENDING);
		irq_enable = litex_read32(host->sdirq + LITEX_IRQ_ENABLE);
	}

	dev_err(dev,
		"first failure: generation=%u request=%llu stage=%s cmd=%u arg=%#010x xfer=%u cmd_evt=%#x data_evt=%#x blk_len=%u blk_count=%u\n",
		host->card_generation,
		(unsigned long long)host->active_request_sequence,
		stage, cmd, arg, transfer,
		litex_read32(host->sdcore + LITEX_CORE_CMDEVT),
		litex_read32(host->sdcore + LITEX_CORE_DATEVT),
		litex_read32(host->sdcore + LITEX_CORE_BLKLEN),
		litex_read32(host->sdcore + LITEX_CORE_BLKCNT));
	dev_err(dev,
		"first failure DMA: reader base=%#018llx len=%u enable=%u done=%u offset=%u; writer base=%#018llx len=%u enable=%u done=%u offset=%u\n",
		(unsigned long long)reader_base,
		litex_read32(host->sdreader + LITEX_BLK2MEM_LEN),
		litex_read32(host->sdreader + LITEX_BLK2MEM_ENA),
		litex_read32(host->sdreader + LITEX_BLK2MEM_DONE),
		litex_read32(host->sdreader + LITEX_BLK2MEM_OFFSET),
		(unsigned long long)writer_base,
		litex_read32(host->sdwriter + LITEX_MEM2BLK_LEN),
		litex_read32(host->sdwriter + LITEX_MEM2BLK_ENA),
		litex_read32(host->sdwriter + LITEX_MEM2BLK_DONE),
		litex_read32(host->sdwriter + LITEX_MEM2BLK_OFFSET));
	dev_err(dev,
		"first failure IRQ: status=%#x pending=%#x enable=%#x\n",
		irq_status, irq_pending, irq_enable);
	if (host->sg_supported)
		dev_err(dev,
			"first failure SG: status=%#010x current=%u error_index=%u completed=%u fetch_cycles=%u max_gap_cycles=%u detail=%#010x\n",
			litex_read32(host->sdsg + LITEX_SG_STATUS),
			litex_read32(host->sdsg + LITEX_SG_CURRENT_INDEX),
			litex_read32(host->sdsg + LITEX_SG_ERROR_INDEX),
			litex_read32(host->sdsg + LITEX_SG_COMPLETED),
			litex_read32(host->sdsg + LITEX_SG_FETCH_CYCLES),
			litex_read32(host->sdsg + LITEX_SG_MAX_GAP_CYCLES),
			litex_read32(host->sdsg + LITEX_SG_ERROR_DETAIL));
	dev_err(dev,
		"first failure PHY: card_detect=%u clock_div=%u sd_clk=%u\n",
		litex_read8(host->sdphy + LITEX_PHY_CARDDETECT),
		litex_read16(host->sdphy + LITEX_PHY_CLOCKERDIV),
		host->sd_clk);
}

static void litex_mmc_log_data_start(struct litex_mmc_host *host,
				     struct mmc_command *cmd,
				     struct mmc_data *data,
				     unsigned int len,
				     enum litex_mmc_dma_path path,
				     int mapped_nents, u8 transfer)
{
	struct device *dev = mmc_dev(host->mmc);
	void __iomem *dma_regs;
	u32 irq_status = 0, irq_pending = 0, irq_enable = 0;

	if (cmd->opcode != MMC_READ_SINGLE_BLOCK &&
	    cmd->opcode != MMC_READ_MULTIPLE_BLOCK &&
	    cmd->opcode != MMC_WRITE_BLOCK &&
	    cmd->opcode != MMC_WRITE_MULTIPLE_BLOCK)
		return;
	if (host->first_block_request_logged)
		return;
	host->first_block_request_logged = true;

	dma_regs = transfer == SD_CTL_DATA_XFER_READ ?
		host->sdreader : host->sdwriter;
	if (host->sdirq) {
		irq_status = litex_read32(host->sdirq + LITEX_IRQ_STATUS);
		irq_pending = litex_read32(host->sdirq + LITEX_IRQ_PENDING);
		irq_enable = litex_read32(host->sdirq + LITEX_IRQ_ENABLE);
	}

	dev_info(dev,
		 "first block request: generation=%u request=%llu cmd=%u arg=%#010x blocks=%u blksz=%u flags=%#x sg_len=%u mapped_nents=%d path=%s xfer=%u dma_base=%#018llx dma_len=%u sd_clk=%u timeout_us=%lu\n",
		 host->card_generation,
		 (unsigned long long)host->active_request_sequence,
		 cmd->opcode, cmd->arg, data->blocks, data->blksz, data->flags,
		 data->sg_len, mapped_nents, litex_mmc_dma_path_name(path),
		 transfer, (unsigned long long)litex_read64(dma_regs), len,
		 host->sd_clk, litex_mmc_data_timeout_us(host, data));
	dev_info(dev,
		 "first block prestart: cmd_evt=%#x data_evt=%#x dma_done=%#x dma_offset=%u irq_status=%#x irq_pending=%#x irq_enable=%#x\n",
		 litex_read32(host->sdcore + LITEX_CORE_CMDEVT),
		 litex_read32(host->sdcore + LITEX_CORE_DATEVT),
		 litex_read32(dma_regs + LITEX_BLK2MEM_DONE),
		 litex_read32(dma_regs + LITEX_BLK2MEM_OFFSET),
		 irq_status, irq_pending, irq_enable);
}

static void litex_mmc_capture_bad_read(struct litex_mmc_host *host,
				       struct mmc_command *cmd,
				       struct mmc_data *data,
				       unsigned int dma_len,
				       u8 dma_done, u32 dma_offset)
{
	struct litex_mmc_bad_read_snapshot *snapshot = &host->bad_read;
	struct device *dev = mmc_dev(host->mmc);
	size_t wanted, copied;
	unsigned int i;

	if (snapshot->valid)
		return;

	wanted = min_t(size_t, dma_len, LITEX_BAD_READ_SNAPSHOT_SIZE);
	memset(snapshot, 0, sizeof(*snapshot));
	copied = sg_copy_to_buffer(data->sg, data->sg_len,
				   snapshot->data, wanted);

	snapshot->generation = host->card_generation;
	snapshot->request_sequence = host->active_request_sequence;
	snapshot->opcode = cmd->opcode;
	snapshot->cmd_arg = cmd->arg;
	snapshot->requested_len = data->blksz * data->blocks;
	snapshot->copied_len = copied;
	snapshot->dma_done = dma_done;
	snapshot->dma_offset = dma_offset;
	snapshot->crc32 = crc32_le(~0, snapshot->data, copied) ^ ~0;

	for (i = 0; i < copied; i++) {
		if (snapshot->data[i] == 0xff)
			continue;
		if (snapshot->non_ff_sample_count <
		    LITEX_BAD_READ_NON_FF_SAMPLES) {
			u8 sample = snapshot->non_ff_sample_count++;

			snapshot->non_ff_offsets[sample] = i;
			snapshot->non_ff_values[sample] = snapshot->data[i];
		}
		snapshot->non_ff_count++;
	}

	snapshot->valid = true;
	/* Publish the blob size only after the complete snapshot is visible. */
	smp_wmb();
	WRITE_ONCE(host->bad_read_blob.size, copied);

	dev_err(dev,
		"captured first CRC-failed read: generation=%u request=%llu cmd=%u arg=%#010x requested=%u copied=%u dma_done=%#x dma_offset=%u crc32=%#010x non_ff=%u\n",
		snapshot->generation,
		(unsigned long long)snapshot->request_sequence,
		snapshot->opcode, snapshot->cmd_arg, snapshot->requested_len,
		snapshot->copied_len, snapshot->dma_done, snapshot->dma_offset,
		snapshot->crc32, snapshot->non_ff_count);
	for (i = 0; i < snapshot->non_ff_sample_count; i++)
		dev_err(dev,
			"first CRC-failed read non-ff[%u]: offset=%u value=%#04x\n",
			i, snapshot->non_ff_offsets[i],
			snapshot->non_ff_values[i]);
}

static int litex_mmc_bad_read_meta_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct litex_mmc_host *host = dev_get_drvdata(dev);
	struct litex_mmc_bad_read_snapshot *snapshot;
	unsigned int i;

	if (!host)
		return -ENODEV;

	snapshot = &host->bad_read;
	seq_printf(s, "valid=%u\n", snapshot->valid);
	if (!snapshot->valid)
		return 0;

	seq_printf(s, "generation=%u\n", snapshot->generation);
	seq_printf(s, "request_sequence=%llu\n",
		   (unsigned long long)snapshot->request_sequence);
	seq_printf(s, "opcode=%u\n", snapshot->opcode);
	seq_printf(s, "cmd_arg=%#010x\n", snapshot->cmd_arg);
	seq_printf(s, "requested_len=%u\n", snapshot->requested_len);
	seq_printf(s, "copied_len=%u\n", snapshot->copied_len);
	seq_printf(s, "dma_done=%#x\n", snapshot->dma_done);
	seq_printf(s, "dma_offset=%u\n", snapshot->dma_offset);
	seq_printf(s, "crc32=%#010x\n", snapshot->crc32);
	seq_printf(s, "non_ff_count=%u\n", snapshot->non_ff_count);
	for (i = 0; i < snapshot->non_ff_sample_count; i++)
		seq_printf(s, "non_ff[%u]=%u:%#04x\n", i,
			   snapshot->non_ff_offsets[i],
			   snapshot->non_ff_values[i]);

	return 0;
}

static int litex_mmc_dma_stats_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct litex_mmc_host *host = dev_get_drvdata(dev);
	struct litex_mmc_dma_stats stats;
	unsigned long flags;

	if (!host)
		return -ENODEV;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	stats = host->dma_stats;
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);

	seq_printf(s, "max_segs=%u\n", host->mmc->max_segs);
	seq_printf(s, "max_seg_size=%u\n", host->mmc->max_seg_size);
	seq_printf(s, "max_req_size=%u\n", host->mmc->max_req_size);
	seq_printf(s, "max_blk_count=%u\n", host->mmc->max_blk_count);
	seq_printf(s, "requests=%llu\n", stats.requests);
	seq_printf(s, "bytes=%llu\n", stats.bytes);
	seq_printf(s, "read_requests=%llu\n", stats.read_requests);
	seq_printf(s, "read_bytes=%llu\n", stats.read_bytes);
	seq_printf(s, "write_requests=%llu\n", stats.write_requests);
	seq_printf(s, "write_bytes=%llu\n", stats.write_bytes);
	seq_printf(s, "direct_requests=%llu\n", stats.direct_requests);
	seq_printf(s, "direct_bytes=%llu\n", stats.direct_bytes);
	seq_printf(s, "sg_requests=%llu\n", stats.sg_requests);
	seq_printf(s, "sg_bytes=%llu\n", stats.sg_bytes);
	seq_printf(s, "sg_entries=%llu\n", stats.sg_entries);
	seq_printf(s, "bounce_requests=%llu\n", stats.bounce_requests);
	seq_printf(s, "bounce_bytes=%llu\n", stats.bounce_bytes);
	seq_printf(s, "size_le_4k=%llu\n", stats.size_le_4k);
	seq_printf(s, "size_le_16k=%llu\n", stats.size_le_16k);
	seq_printf(s, "size_le_64k=%llu\n", stats.size_le_64k);
	seq_printf(s, "size_le_256k=%llu\n", stats.size_le_256k);
	seq_printf(s, "size_le_512k=%llu\n", stats.size_le_512k);
	seq_printf(s, "size_le_1m=%llu\n", stats.size_le_1m);
	seq_printf(s, "min_request_bytes=%u\n", stats.min_request_bytes);
	seq_printf(s, "max_request_bytes=%u\n", stats.max_request_bytes);
	seq_printf(s, "max_sg_len=%u\n", stats.max_sg_len);
	seq_printf(s, "max_mapped_nents=%u\n", stats.max_mapped_nents);
	seq_printf(s, "pre_req_count=%llu\n", stats.pre_req_count);
	seq_printf(s, "pre_req_mapped=%llu\n", stats.pre_req_mapped);
	seq_printf(s, "pre_req_failed=%llu\n", stats.pre_req_failed);
	seq_printf(s, "pre_req_map_ns=%llu\n", stats.pre_req_map_ns);
	seq_printf(s, "max_pre_req_map_ns=%llu\n", stats.max_pre_req_map_ns);
	seq_printf(s, "post_req_count=%llu\n", stats.post_req_count);
	seq_printf(s, "post_req_unmap_ns=%llu\n", stats.post_req_unmap_ns);
	seq_printf(s, "max_post_req_unmap_ns=%llu\n",
		   stats.max_post_req_unmap_ns);
	seq_printf(s, "async_requests=%llu\n", stats.async_requests);
	seq_puts(s, "profile_total_excludes_pre_req=1\n");
	seq_puts(s, "profile_total_excludes_post_req=1\n");

	return 0;
}

static int litex_mmc_sg_profile_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct litex_mmc_host *host = dev_get_drvdata(dev);
	struct litex_mmc_sg_profile profile;
	u64 attributed_ns, unattributed_ns;
	unsigned long flags;

	if (!host)
		return -ENODEV;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	profile = host->sg_profile;
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);

	attributed_ns = profile.sbc_ns + profile.bus_width_ns +
		profile.dma_map_ns + profile.table_build_ns +
		profile.table_fetch_ns + profile.frontend_program_ns +
		profile.activate_ns + profile.command_setup_ns +
		profile.irq_wait_ns +
		profile.command_poll_ns + profile.response_ns +
		profile.data_poll_ns + profile.frontend_dma_poll_ns +
		profile.sg_complete_ns + profile.stop_ns +
		profile.frontend_abort_ns + profile.abort_ns +
		profile.cleanup_ns + profile.dma_unmap_ns + profile.post_ns;
	unattributed_ns = profile.total_ns > attributed_ns ?
		profile.total_ns - attributed_ns : 0;

	seq_printf(s, "supported=%u\n", host->sg_supported);
	seq_printf(s, "cap=%#010x\n", host->sdsg ?
		   litex_read32(host->sdsg + LITEX_SG_CAP) : 0);
	seq_printf(s, "requests=%llu\n", profile.requests);
	seq_printf(s, "read_requests=%llu\n", profile.read_requests);
	seq_printf(s, "write_requests=%llu\n", profile.write_requests);
	seq_printf(s, "failed_requests=%llu\n", profile.failed_requests);
	seq_printf(s, "aborted_requests=%llu\n", profile.aborted_requests);
	seq_printf(s, "bytes=%llu\n", profile.bytes);
	seq_printf(s, "entries=%llu\n", profile.entries);
	seq_printf(s, "total_ns=%llu\n", profile.total_ns);
	seq_printf(s, "sbc_ns=%llu\n", profile.sbc_ns);
	seq_printf(s, "bus_width_ns=%llu\n", profile.bus_width_ns);
	seq_printf(s, "dma_map_ns=%llu\n", profile.dma_map_ns);
	seq_printf(s, "table_build_ns=%llu\n", profile.table_build_ns);
	seq_printf(s, "table_fetch_ns=%llu\n", profile.table_fetch_ns);
	seq_printf(s, "frontend_program_ns=%llu\n",
		   profile.frontend_program_ns);
	seq_printf(s, "activate_ns=%llu\n", profile.activate_ns);
	seq_printf(s, "command_setup_ns=%llu\n", profile.command_setup_ns);
	seq_printf(s, "irq_wait_ns=%llu\n", profile.irq_wait_ns);
	seq_printf(s, "command_poll_ns=%llu\n", profile.command_poll_ns);
	seq_printf(s, "response_ns=%llu\n", profile.response_ns);
	seq_printf(s, "data_poll_ns=%llu\n", profile.data_poll_ns);
	seq_printf(s, "frontend_dma_poll_ns=%llu\n",
		   profile.frontend_dma_poll_ns);
	seq_printf(s, "sg_complete_ns=%llu\n", profile.sg_complete_ns);
	seq_printf(s, "stop_ns=%llu\n", profile.stop_ns);
	seq_printf(s, "frontend_abort_ns=%llu\n", profile.frontend_abort_ns);
	seq_printf(s, "abort_ns=%llu\n", profile.abort_ns);
	seq_printf(s, "cleanup_ns=%llu\n", profile.cleanup_ns);
	seq_printf(s, "dma_unmap_ns=%llu\n", profile.dma_unmap_ns);
	seq_printf(s, "post_ns=%llu\n", profile.post_ns);
	seq_printf(s, "unattributed_ns=%llu\n", unattributed_ns);
	seq_printf(s, "fetch_cycles=%llu\n", profile.fetch_cycles);
	seq_printf(s, "max_gap_cycles=%llu\n", profile.max_gap_cycles);
	seq_printf(s, "last_request_sequence=%llu\n",
		   profile.last_request_sequence);
	seq_printf(s, "last_bytes=%u\n", profile.last_bytes);
	seq_printf(s, "last_entries=%u\n", profile.last_entries);
	seq_printf(s, "last_min_segment_bytes=%u\n",
		   profile.last_min_segment_bytes);
	seq_printf(s, "last_max_segment_bytes=%u\n",
		   profile.last_max_segment_bytes);
	seq_printf(s, "last_transfer=%u\n", profile.last_transfer);
	seq_printf(s, "last_failed=%u\n", profile.last_failed);
	seq_printf(s, "last_status=%#010x\n", profile.last_status);
	seq_printf(s, "last_fetch_cycles=%u\n", profile.last_fetch_cycles);
	seq_printf(s, "last_max_gap_cycles=%u\n",
		   profile.last_max_gap_cycles);
	seq_printf(s, "last_total_ns=%llu\n", profile.last_total_ns);
	seq_printf(s, "last_sbc_ns=%llu\n", profile.last_sbc_ns);
	seq_printf(s, "last_bus_width_ns=%llu\n", profile.last_bus_width_ns);
	seq_printf(s, "last_dma_map_ns=%llu\n", profile.last_dma_map_ns);
	seq_printf(s, "last_table_build_ns=%llu\n",
		   profile.last_table_build_ns);
	seq_printf(s, "last_table_fetch_ns=%llu\n",
		   profile.last_table_fetch_ns);
	seq_printf(s, "last_frontend_program_ns=%llu\n",
		   profile.last_frontend_program_ns);
	seq_printf(s, "last_activate_ns=%llu\n", profile.last_activate_ns);
	seq_printf(s, "last_command_setup_ns=%llu\n",
		   profile.last_command_setup_ns);
	seq_printf(s, "last_irq_wait_ns=%llu\n", profile.last_irq_wait_ns);
	seq_printf(s, "last_command_poll_ns=%llu\n",
		   profile.last_command_poll_ns);
	seq_printf(s, "last_response_ns=%llu\n", profile.last_response_ns);
	seq_printf(s, "last_data_poll_ns=%llu\n", profile.last_data_poll_ns);
	seq_printf(s, "last_frontend_dma_poll_ns=%llu\n",
		   profile.last_frontend_dma_poll_ns);
	seq_printf(s, "last_sg_complete_ns=%llu\n",
		   profile.last_sg_complete_ns);
	seq_printf(s, "last_stop_ns=%llu\n", profile.last_stop_ns);
	seq_printf(s, "last_frontend_abort_ns=%llu\n",
		   profile.last_frontend_abort_ns);
	seq_printf(s, "last_abort_ns=%llu\n", profile.last_abort_ns);
	seq_printf(s, "last_cleanup_ns=%llu\n", profile.last_cleanup_ns);
	seq_printf(s, "last_dma_unmap_ns=%llu\n",
		   profile.last_dma_unmap_ns);
	seq_printf(s, "last_post_ns=%llu\n", profile.last_post_ns);

	return 0;
}

static int litex_mmc_write_profile_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct litex_mmc_host *host = dev_get_drvdata(dev);
	struct litex_mmc_write_profile profile;
	u64 attributed_ns, unattributed_ns;
	unsigned long flags;

	if (!host)
		return -ENODEV;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	profile = host->write_profile;
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);

	attributed_ns = profile.sbc_ns + profile.bus_width_ns +
		profile.dma_map_ns +
		profile.bounce_copy_ns + profile.sg_table_build_ns +
		profile.sg_table_fetch_ns + profile.dma_program_ns +
		profile.sg_activate_ns + profile.cmd_setup_ns +
		profile.irq_wait_ns +
		profile.cmd_poll_ns + profile.response_ns +
		profile.data_poll_ns + profile.dma_poll_ns +
		profile.sg_complete_ns + profile.stop_ns +
		profile.frontend_abort_ns +
		profile.sg_abort_ns + profile.sg_cleanup_ns +
		profile.dma_unmap_ns + profile.post_ns;
	unattributed_ns = profile.total_ns > attributed_ns ?
		profile.total_ns - attributed_ns : 0;

	seq_printf(s, "requests=%llu\n", profile.requests);
	seq_printf(s, "bytes=%llu\n", profile.bytes);
	seq_printf(s, "direct_requests=%llu\n", profile.direct_requests);
	seq_printf(s, "sg_requests=%llu\n", profile.sg_requests);
	seq_printf(s, "bounce_requests=%llu\n", profile.bounce_requests);
	seq_printf(s, "cmd24_requests=%llu\n", profile.cmd24_requests);
	seq_printf(s, "cmd25_requests=%llu\n", profile.cmd25_requests);
	seq_printf(s, "failed_requests=%llu\n", profile.failed_requests);
	seq_printf(s, "total_ns=%llu\n", profile.total_ns);
	seq_printf(s, "sbc_ns=%llu\n", profile.sbc_ns);
	seq_printf(s, "bus_width_ns=%llu\n", profile.bus_width_ns);
	seq_printf(s, "dma_map_ns=%llu\n", profile.dma_map_ns);
	seq_printf(s, "bounce_copy_ns=%llu\n", profile.bounce_copy_ns);
	seq_printf(s, "sg_table_build_ns=%llu\n", profile.sg_table_build_ns);
	seq_printf(s, "sg_table_fetch_ns=%llu\n", profile.sg_table_fetch_ns);
	seq_printf(s, "dma_program_ns=%llu\n", profile.dma_program_ns);
	seq_printf(s, "sg_activate_ns=%llu\n", profile.sg_activate_ns);
	seq_printf(s, "cmd_setup_ns=%llu\n", profile.cmd_setup_ns);
	seq_printf(s, "irq_wait_ns=%llu\n", profile.irq_wait_ns);
	seq_printf(s, "cmd_poll_ns=%llu\n", profile.cmd_poll_ns);
	seq_printf(s, "response_ns=%llu\n", profile.response_ns);
	seq_printf(s, "data_poll_ns=%llu\n", profile.data_poll_ns);
	seq_printf(s, "dma_poll_ns=%llu\n", profile.dma_poll_ns);
	seq_printf(s, "sg_complete_ns=%llu\n", profile.sg_complete_ns);
	seq_printf(s, "stop_ns=%llu\n", profile.stop_ns);
	seq_printf(s, "frontend_abort_ns=%llu\n", profile.frontend_abort_ns);
	seq_printf(s, "sg_abort_ns=%llu\n", profile.sg_abort_ns);
	seq_printf(s, "sg_cleanup_ns=%llu\n", profile.sg_cleanup_ns);
	seq_printf(s, "dma_unmap_ns=%llu\n", profile.dma_unmap_ns);
	seq_printf(s, "post_ns=%llu\n", profile.post_ns);
	seq_printf(s, "unattributed_ns=%llu\n", unattributed_ns);
	seq_printf(s, "max_total_ns=%llu\n", profile.max_total_ns);
	seq_printf(s, "max_dma_map_ns=%llu\n", profile.max_dma_map_ns);
	seq_printf(s, "max_irq_wait_ns=%llu\n", profile.max_irq_wait_ns);
	seq_printf(s, "max_data_poll_ns=%llu\n", profile.max_data_poll_ns);
	seq_printf(s, "min_request_bytes=%u\n", profile.min_request_bytes);
	seq_printf(s, "max_request_bytes=%u\n", profile.max_request_bytes);

	return 0;
}

static ssize_t litex_mmc_write_profile_reset(struct file *file,
					     const char __user *buf,
					     size_t count, loff_t *ppos)
{
	struct litex_mmc_host *host = file->private_data;
	unsigned long flags;

	if (!host)
		return -ENODEV;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	memset(&host->write_profile, 0, sizeof(host->write_profile));
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);

	return count;
}

static const struct file_operations litex_mmc_write_profile_reset_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = litex_mmc_write_profile_reset,
	.llseek = no_llseek,
};

static ssize_t litex_mmc_sg_profile_reset(struct file *file,
					  const char __user *buf,
					  size_t count, loff_t *ppos)
{
	struct litex_mmc_host *host = file->private_data;
	unsigned long flags;

	if (!host)
		return -ENODEV;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	memset(&host->sg_profile, 0, sizeof(host->sg_profile));
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);

	return count;
}

static const struct file_operations litex_mmc_sg_profile_reset_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = litex_mmc_sg_profile_reset,
	.llseek = no_llseek,
};

static void litex_mmc_account_write_profile(struct litex_mmc_host *host,
					    struct mmc_command *cmd,
					    unsigned int len,
					    enum litex_mmc_dma_path path,
					    bool failed,
					    const struct litex_mmc_request_timing *timing,
					    u64 total_ns)
{
	struct litex_mmc_write_profile *profile = &host->write_profile;
	unsigned long flags;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	profile->requests++;
	profile->bytes += len;
	if (path == LITEX_MMC_DMA_DIRECT)
		profile->direct_requests++;
	else if (path == LITEX_MMC_DMA_SG)
		profile->sg_requests++;
	else
		profile->bounce_requests++;
	if (cmd->opcode == MMC_WRITE_BLOCK)
		profile->cmd24_requests++;
	else if (cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)
		profile->cmd25_requests++;
	if (failed)
		profile->failed_requests++;
	profile->total_ns += total_ns;
	profile->sbc_ns += timing->sbc_ns;
	profile->bus_width_ns += timing->bus_width_ns;
	profile->dma_map_ns += timing->dma_prep.map_ns;
	profile->bounce_copy_ns += timing->dma_prep.bounce_copy_ns;
	profile->sg_table_build_ns += timing->dma_prep.sg_table_build_ns;
	profile->sg_table_fetch_ns += timing->dma_prep.sg_table_fetch_ns;
	profile->dma_program_ns += timing->dma_prep.program_ns;
	profile->sg_activate_ns += timing->dma_prep.sg_activate_ns;
	profile->cmd_setup_ns += timing->command.setup_ns;
	profile->irq_wait_ns += timing->command.irq_wait_ns;
	profile->cmd_poll_ns += timing->command.cmd_poll_ns;
	profile->response_ns += timing->command.response_ns;
	profile->data_poll_ns += timing->command.data_poll_ns;
	profile->dma_poll_ns += timing->command.dma_poll_ns;
	profile->sg_complete_ns += timing->sg_complete_ns;
	profile->stop_ns += timing->stop_ns;
	profile->frontend_abort_ns += timing->frontend_abort_ns;
	profile->sg_abort_ns += timing->sg_abort_ns;
	profile->sg_cleanup_ns += timing->sg_cleanup_ns;
	profile->dma_unmap_ns += timing->dma_unmap_ns;
	profile->post_ns += timing->post_ns;
	profile->max_total_ns = max(profile->max_total_ns, total_ns);
	profile->max_dma_map_ns = max(profile->max_dma_map_ns,
				      timing->dma_prep.map_ns);
	profile->max_irq_wait_ns = max(profile->max_irq_wait_ns,
				       timing->command.irq_wait_ns);
	profile->max_data_poll_ns = max(profile->max_data_poll_ns,
					timing->command.data_poll_ns);
	if (!profile->min_request_bytes || len < profile->min_request_bytes)
		profile->min_request_bytes = len;
	profile->max_request_bytes = max(profile->max_request_bytes, len);
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);
}

static void litex_mmc_account_sg_profile(struct litex_mmc_host *host,
					 struct mmc_data *data,
					 const struct litex_mmc_dma_context *ctx,
					 bool failed,
					 const struct litex_mmc_request_timing *timing,
					 u64 total_ns, u32 status)
{
	struct litex_mmc_sg_profile *profile = &host->sg_profile;
	u32 fetch_cycles = ctx->sg_fetch_cycles;
	u32 max_gap_cycles = ctx->sg_max_gap_cycles;
	unsigned long flags;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	profile->requests++;
	if (data->flags & MMC_DATA_READ)
		profile->read_requests++;
	else if (data->flags & MMC_DATA_WRITE)
		profile->write_requests++;
	if (failed)
		profile->failed_requests++;
	if (timing->sg_abort_ns)
		profile->aborted_requests++;
	profile->bytes += ctx->len;
	profile->entries += ctx->mapped_nents;
	profile->total_ns += total_ns;
	profile->sbc_ns += timing->sbc_ns;
	profile->bus_width_ns += timing->bus_width_ns;
	profile->dma_map_ns += timing->dma_prep.map_ns;
	profile->table_build_ns += timing->dma_prep.sg_table_build_ns;
	profile->table_fetch_ns += timing->dma_prep.sg_table_fetch_ns;
	profile->frontend_program_ns += timing->dma_prep.program_ns;
	profile->activate_ns += timing->dma_prep.sg_activate_ns;
	profile->command_setup_ns += timing->command.setup_ns;
	profile->irq_wait_ns += timing->command.irq_wait_ns;
	profile->command_poll_ns += timing->command.cmd_poll_ns;
	profile->response_ns += timing->command.response_ns;
	profile->data_poll_ns += timing->command.data_poll_ns;
	profile->frontend_dma_poll_ns += timing->command.dma_poll_ns;
	profile->sg_complete_ns += timing->sg_complete_ns;
	profile->stop_ns += timing->stop_ns;
	profile->frontend_abort_ns += timing->frontend_abort_ns;
	profile->abort_ns += timing->sg_abort_ns;
	profile->cleanup_ns += timing->sg_cleanup_ns;
	profile->dma_unmap_ns += timing->dma_unmap_ns;
	profile->post_ns += timing->post_ns;
	profile->fetch_cycles += fetch_cycles;
	profile->max_gap_cycles = max_t(u64, profile->max_gap_cycles,
					max_gap_cycles);
	profile->last_request_sequence = host->active_request_sequence;
	profile->last_total_ns = total_ns;
	profile->last_sbc_ns = timing->sbc_ns;
	profile->last_bus_width_ns = timing->bus_width_ns;
	profile->last_dma_map_ns = timing->dma_prep.map_ns;
	profile->last_table_build_ns = timing->dma_prep.sg_table_build_ns;
	profile->last_table_fetch_ns = timing->dma_prep.sg_table_fetch_ns;
	profile->last_frontend_program_ns = timing->dma_prep.program_ns;
	profile->last_activate_ns = timing->dma_prep.sg_activate_ns;
	profile->last_command_setup_ns = timing->command.setup_ns;
	profile->last_irq_wait_ns = timing->command.irq_wait_ns;
	profile->last_command_poll_ns = timing->command.cmd_poll_ns;
	profile->last_response_ns = timing->command.response_ns;
	profile->last_data_poll_ns = timing->command.data_poll_ns;
	profile->last_frontend_dma_poll_ns = timing->command.dma_poll_ns;
	profile->last_sg_complete_ns = timing->sg_complete_ns;
	profile->last_stop_ns = timing->stop_ns;
	profile->last_frontend_abort_ns = timing->frontend_abort_ns;
	profile->last_abort_ns = timing->sg_abort_ns;
	profile->last_cleanup_ns = timing->sg_cleanup_ns;
	profile->last_dma_unmap_ns = timing->dma_unmap_ns;
	profile->last_post_ns = timing->post_ns;
	profile->last_bytes = ctx->len;
	profile->last_entries = ctx->mapped_nents;
	profile->last_min_segment_bytes = ctx->min_segment_bytes;
	profile->last_max_segment_bytes = ctx->max_segment_bytes;
	profile->last_status = status;
	profile->last_fetch_cycles = fetch_cycles;
	profile->last_max_gap_cycles = max_gap_cycles;
	profile->last_transfer = ctx->transfer;
	profile->last_failed = failed;
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);
}

static void litex_mmc_account_dma(struct litex_mmc_host *host,
				  struct mmc_data *data, unsigned int len,
				  enum litex_mmc_dma_path path,
				  int mapped_nents)
{
	struct litex_mmc_dma_stats *stats = &host->dma_stats;
	u32 mapped_nents_u32 = max(mapped_nents, 0);
	unsigned long flags;

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	stats->requests++;
	stats->bytes += len;
	if (data->flags & MMC_DATA_READ) {
		stats->read_requests++;
		stats->read_bytes += len;
	} else if (data->flags & MMC_DATA_WRITE) {
		stats->write_requests++;
		stats->write_bytes += len;
	}
	if (path == LITEX_MMC_DMA_DIRECT) {
		stats->direct_requests++;
		stats->direct_bytes += len;
	} else if (path == LITEX_MMC_DMA_SG) {
		stats->sg_requests++;
		stats->sg_bytes += len;
		stats->sg_entries += mapped_nents_u32;
	} else {
		stats->bounce_requests++;
		stats->bounce_bytes += len;
	}
	if (len <= SZ_4K)
		stats->size_le_4k++;
	else if (len <= SZ_16K)
		stats->size_le_16k++;
	else if (len <= SZ_64K)
		stats->size_le_64k++;
	else if (len <= SZ_256K)
		stats->size_le_256k++;
	else if (len <= SZ_512K)
		stats->size_le_512k++;
	else
		stats->size_le_1m++;
	if (!stats->min_request_bytes || len < stats->min_request_bytes)
		stats->min_request_bytes = len;
	stats->max_request_bytes = max(stats->max_request_bytes, len);
	stats->max_sg_len = max(stats->max_sg_len, data->sg_len);
	stats->max_mapped_nents = max(stats->max_mapped_nents, mapped_nents_u32);
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);
}

static void litex_mmc_disable_dma(struct litex_mmc_host *host, u8 transfer)
{
	void __iomem *enable;

	if (transfer == SD_CTL_DATA_XFER_READ)
		enable = host->sdreader + LITEX_BLK2MEM_ENA;
	else if (transfer == SD_CTL_DATA_XFER_WRITE)
		enable = host->sdwriter + LITEX_MEM2BLK_ENA;
	else
		return;

	litex_write8(enable, 0);
	/* Ensure the disable reaches the frontend before issuing CMD12. */
	litex_read8(enable);
}

static unsigned long litex_mmc_data_timeout_us(struct litex_mmc_host *host,
					       struct mmc_data *data)
{
	u64 timeout_us, transfer_us = 0;

	timeout_us = DIV_ROUND_UP_ULL(data->timeout_ns, NSEC_PER_USEC);
	if (host->sd_clk && data->timeout_clks)
		timeout_us += DIV_ROUND_UP_ULL((u64)data->timeout_clks *
						USEC_PER_SEC, host->sd_clk);

	/* Follow the SDHCI software-timeout model for the full request. */
	timeout_us *= data->blocks;
	if (host->sd_clk) {
		/* LiteSDCard is fixed to a four-bit data bus. */
		transfer_us = DIV_ROUND_UP_ULL((u64)data->blksz * data->blocks *
						8 * USEC_PER_SEC,
						4 * host->sd_clk);
		timeout_us += 2 * transfer_us;
	}

	return clamp_val(timeout_us, SD_TIMEOUT_US,
			 SD_DATA_TIMEOUT_MAX_US);
}

static int litex_mmc_sdcard_wait_done(void __iomem *reg, struct device *dev,
				      unsigned long timeout_us)
{
	u8 evt;
	int ret;

	ret = readx_poll_timeout(litex_read8, reg, evt, evt & SD_BIT_DONE,
				 SD_SLEEP_US, timeout_us);
	if (ret)
		return ret;
	if (evt == SD_BIT_DONE)
		return 0;
	if (evt & SD_BIT_WR_ERR)
		return -EIO;
	if (evt & SD_BIT_TIMEOUT)
		return -ETIMEDOUT;
	if (evt & SD_BIT_CRC_ERR)
		return -EILSEQ;
	dev_err(dev, "%s: unknown error (evt=%x)\n", __func__, evt);
	return -EINVAL;
}

static int litex_mmc_dma_status_to_errno(u8 status)
{
	if (status & (LITEX_DMA_DONE_SLVERR | LITEX_DMA_DONE_DECERR))
		return -EIO;
	if (status & LITEX_DMA_DONE_INVALID)
		return -EINVAL;
	if (status & LITEX_DMA_DONE_ABORTED)
		return -ECANCELED;
	if (status & LITEX_DMA_DONE_MISMATCH)
		return -EIO;
	if (status & LITEX_DMA_DONE_SUCCESS)
		return 0;

	return -EIO;
}

static int litex_mmc_dma_wait_done(void __iomem *reg,
				   unsigned long timeout_us, u8 *status)
{
	u8 done;
	int ret;

	ret = readx_poll_timeout(litex_read8, reg, done,
				 (done & LITEX_DMA_DONE_TERMINAL) &&
				 !(done & LITEX_DMA_DONE_BUSY),
				 SD_SLEEP_US, timeout_us);
	*status = done;
	if (ret)
		return ret;

	return litex_mmc_dma_status_to_errno(done);
}

static int litex_mmc_dma_wait_idle(struct litex_mmc_host *host, u8 transfer,
				   unsigned long timeout_us, u8 *status)
{
	void __iomem *reg;
	u8 done;
	int ret;

	if (transfer == SD_CTL_DATA_XFER_READ) {
		reg = host->sdreader + LITEX_BLK2MEM_DONE;
	} else if (transfer == SD_CTL_DATA_XFER_WRITE) {
		reg = host->sdwriter + LITEX_MEM2BLK_DONE;
	} else {
		*status = 0;
		return 0;
	}

	ret = readx_poll_timeout(litex_read8, reg, done,
				 !(done & LITEX_DMA_DONE_BUSY),
				 SD_SLEEP_US, timeout_us);
	*status = done;

	return ret;
}

static void litex_mmc_start_cmd(struct litex_mmc_host *host,
				u8 cmd, u32 arg, u8 response_len, u8 transfer,
				struct mmc_data *data,
				bool write_dma_prestarted,
				struct litex_mmc_started_cmd *started,
				struct litex_mmc_cmd_timing *timing)
{
	u64 phase_start_ns = timing ? ktime_get_ns() : 0;
	u64 now_ns;

	memset(started, 0, sizeof(*started));
	if (timing)
		memset(timing, 0, sizeof(*timing));

	started->trace_large_read = data && (data->flags & MMC_DATA_READ) &&
		data->blocks >= LITEX_MMC_MAX_REQ_SIZE / data->blksz &&
		!host->large_read_transfer_timing_logged;
	if (started->trace_large_read)
		started->trace_start_ns = ktime_get_ns();

	/*
	 * Wait for an interrupt if we have an interrupt and either there is
	 * data to be transferred, or if the card can report busy via DAT0.
	 */
	started->wait_irq = host->irq > 0 &&
		(transfer != SD_CTL_DATA_XFER_NONE ||
		 response_len == SD_CTL_RESP_SHORT_BUSY);
	if (started->wait_irq)
		reinit_completion(&host->data_done);

	litex_write32(host->sdcore + LITEX_CORE_CMDARG, arg);
	litex_write32(host->sdcore + LITEX_CORE_CMDCMD,
		      cmd << 8 | transfer << 5 | response_len);
	litex_write8(host->sdcore + LITEX_CORE_CMDSND, 1);

	if (started->wait_irq) {
		/*
		 * DATA_DONE marks the whole command/data transaction, but is high
		 * while idle. Enable it only after CMDSND clears the idle level.
		 */
		litex_write32(host->sdirq + LITEX_IRQ_PENDING,
			      SDIRQ_DATA_DONE);
		litex_write32(host->sdirq + LITEX_IRQ_ENABLE,
			      SDIRQ_DATA_DONE | SDIRQ_CARD_DETECT);
	}

	/*
	 * Direct and bounce writes start only after the command and completion IRQ
	 * are armed. SG writes are already waiting for payload and must not be
	 * restarted here.
	 */
	if (transfer == SD_CTL_DATA_XFER_WRITE && !write_dma_prestarted)
		litex_write8(host->sdwriter + LITEX_MEM2BLK_ENA, 1);

	now_ns = ktime_get_ns();
	if (timing)
		timing->setup_ns = now_ns - phase_start_ns;
	started->wait_start_ns = now_ns;
}

/* Return command errors; report data and DMA errors through data->error. */
static int litex_mmc_finish_cmd(struct litex_mmc_host *host,
				u8 cmd, u32 arg, u8 response_len, u8 transfer,
				struct mmc_data *data,
				struct litex_mmc_started_cmd *started,
				struct litex_mmc_cmd_timing *timing)
{
	struct device *dev = mmc_dev(host->mmc);
	unsigned long timeout_us = data ?
		litex_mmc_data_timeout_us(host, data) : SD_TIMEOUT_US;
	unsigned long irq_timeout_us = data ? timeout_us : SD_IRQ_TIMEOUT_US;
	void __iomem *reg;
	u64 trace_irq_ns = 0;
	u64 trace_cmd_ns = 0;
	u64 trace_data_ns = 0;
	u64 trace_dma_ns = 0;
	u64 phase_start_ns = started->wait_start_ns;
	u64 now_ns;
	int ret;
	u8 evt;

	if (started->wait_irq &&
	    !wait_for_completion_timeout(&host->data_done,
					 usecs_to_jiffies(irq_timeout_us))) {
		litex_write32(host->sdirq + LITEX_IRQ_ENABLE,
			      SDIRQ_CARD_DETECT);
		litex_mmc_log_failure(host, "irq-timeout", cmd, arg, transfer);
		dev_err(dev, "Command (cmd %d) interrupt timeout\n", cmd);
		if (timing)
			timing->irq_wait_ns = ktime_get_ns() - phase_start_ns;
		return -ETIMEDOUT;
	}
	if (timing) {
		now_ns = ktime_get_ns();
		timing->irq_wait_ns = started->wait_irq ?
			now_ns - phase_start_ns : 0;
		phase_start_ns = now_ns;
	}
	if (started->trace_large_read)
		trace_irq_ns = ktime_get_ns();

	ret = litex_mmc_sdcard_wait_done(host->sdcore + LITEX_CORE_CMDEVT, dev,
					 SD_TIMEOUT_US);
	if (timing) {
		now_ns = ktime_get_ns();
		timing->cmd_poll_ns = now_ns - phase_start_ns;
		phase_start_ns = now_ns;
	}
	if (ret) {
		litex_mmc_log_failure(host, "command-event", cmd, arg, transfer);
		dev_err(dev, "Command (cmd %d) error, status %d\n", cmd, ret);
		return ret;
	}
	if (started->trace_large_read)
		trace_cmd_ns = ktime_get_ns();

	if (response_len != SD_CTL_RESP_NONE) {
		/*
		 * Each LiteX CSR subregister occupies one aligned 32-bit word. Use
		 * LiteX accessors for bridges that only implement 32-bit accesses.
		 */
		host->resp[0] = litex_read32(host->sdcore + LITEX_CORE_CMDRSP);
		host->resp[1] = litex_read32(host->sdcore + LITEX_CORE_CMDRSP + 0x04);
		host->resp[2] = litex_read32(host->sdcore + LITEX_CORE_CMDRSP + 0x08);
		host->resp[3] = litex_read32(host->sdcore + LITEX_CORE_CMDRSP + 0x0c);
	}
	if (timing) {
		now_ns = ktime_get_ns();
		timing->response_ns = now_ns - phase_start_ns;
		phase_start_ns = now_ns;
	}

	if (!host->app_cmd && cmd == SD_SEND_RELATIVE_ADDR)
		host->rca = host->resp[3] >> 16;
	host->app_cmd = cmd == MMC_APP_CMD;

	if (transfer == SD_CTL_DATA_XFER_NONE)
		return 0;

	ret = litex_mmc_sdcard_wait_done(host->sdcore + LITEX_CORE_DATEVT, dev,
					 timeout_us);
	if (timing) {
		now_ns = ktime_get_ns();
		timing->data_poll_ns = now_ns - phase_start_ns;
		phase_start_ns = now_ns;
	}
	if (ret) {
		litex_mmc_log_failure(host, "data-event", cmd, arg, transfer);
		dev_err(dev, "Data xfer (cmd %d) error, status %d\n", cmd, ret);
		data->error = ret;
		return 0;
	}
	if (started->trace_large_read)
		trace_data_ns = ktime_get_ns();

	reg = transfer == SD_CTL_DATA_XFER_READ ?
		host->sdreader + LITEX_BLK2MEM_DONE :
		host->sdwriter + LITEX_MEM2BLK_DONE;
	ret = litex_mmc_dma_wait_done(reg, timeout_us, &evt);
	if (timing)
		timing->dma_poll_ns = ktime_get_ns() - phase_start_ns;
	if (ret) {
		litex_mmc_log_failure(host,
				      ret == -ETIMEDOUT ? "dma-timeout" :
				      "dma-status", cmd, arg, transfer);
		dev_err(dev, "DMA error (cmd %d, status %d, done=%#x)\n",
			cmd, ret, evt);
	}
	data->error = ret;
	if (started->trace_large_read) {
		trace_dma_ns = ktime_get_ns();
		dev_info(dev,
			 "first 1MiB read transfer timing: request=%llu blocks=%u total_us=%llu irq_wait_us=%llu cmd_poll_us=%llu data_poll_us=%llu dma_poll_us=%llu wait_irq=%u\n",
			 (unsigned long long)host->active_request_sequence,
			 data->blocks,
			 (unsigned long long)div_u64(trace_dma_ns -
				started->trace_start_ns, NSEC_PER_USEC),
			 (unsigned long long)div_u64(trace_irq_ns -
				started->trace_start_ns, NSEC_PER_USEC),
			 (unsigned long long)div_u64(trace_cmd_ns - trace_irq_ns,
							 NSEC_PER_USEC),
			 (unsigned long long)div_u64(trace_data_ns - trace_cmd_ns,
							 NSEC_PER_USEC),
			 (unsigned long long)div_u64(trace_dma_ns - trace_data_ns,
							 NSEC_PER_USEC),
			 started->wait_irq);
		host->large_read_transfer_timing_logged = true;
	}

	return 0;
}

static int __litex_mmc_send_cmd(struct litex_mmc_host *host,
				u8 cmd, u32 arg, u8 response_len, u8 transfer,
				struct mmc_data *data,
				bool write_dma_prestarted,
				struct litex_mmc_cmd_timing *timing)
{
	struct litex_mmc_started_cmd started;

	litex_mmc_start_cmd(host, cmd, arg, response_len, transfer, data,
			    write_dma_prestarted, &started, timing);
	return litex_mmc_finish_cmd(host, cmd, arg, response_len, transfer, data,
				    &started, timing);
}

static int litex_mmc_send_cmd(struct litex_mmc_host *host,
			      u8 cmd, u32 arg, u8 response_len, u8 transfer,
			      struct mmc_data *data)
{
	return __litex_mmc_send_cmd(host, cmd, arg, response_len, transfer,
				    data, false, NULL);
}

static int litex_mmc_send_app_cmd(struct litex_mmc_host *host)
{
	return litex_mmc_send_cmd(host, MMC_APP_CMD, host->rca << 16,
				  SD_CTL_RESP_SHORT, SD_CTL_DATA_XFER_NONE, NULL);
}

static int litex_mmc_send_set_bus_w_cmd(struct litex_mmc_host *host, u32 width)
{
	return litex_mmc_send_cmd(host, SD_APP_SET_BUS_WIDTH, width,
				  SD_CTL_RESP_SHORT, SD_CTL_DATA_XFER_NONE, NULL);
}

static int litex_mmc_set_bus_width(struct litex_mmc_host *host)
{
	bool app_cmd_sent;
	int ret;

	if (host->is_bus_width_set)
		return 0;

	/* Ensure 'app_cmd' precedes 'app_set_bus_width_cmd' */
	app_cmd_sent = host->app_cmd; /* was preceding command app_cmd? */
	if (!app_cmd_sent) {
		ret = litex_mmc_send_app_cmd(host);
		if (ret)
			return ret;
	}

	/* LiteSDCard only supports 4-bit bus width */
	ret = litex_mmc_send_set_bus_w_cmd(host, MMC_BUS_WIDTH_4);
	if (ret)
		return ret;

	/* Re-send 'app_cmd' if necessary */
	if (app_cmd_sent) {
		ret = litex_mmc_send_app_cmd(host);
		if (ret)
			return ret;
	}

	host->is_bus_width_set = true;

	return 0;
}

static int litex_mmc_get_cd(struct mmc_host *mmc)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	bool present;
	int ret;

	if (!mmc_card_is_removable(mmc))
		return 1;

	present = !litex_read8(host->sdphy + LITEX_PHY_CARDDETECT);
	if (present != host->card_present) {
		host->card_present = present;
		host->first_block_request_logged = false;
		host->error_snapshot_logged = false;
		host->request_sequence = 0;
		if (present) {
			host->card_generation++;
			host->bad_read.valid = false;
			host->bad_read_blob.size = 0;
			host->large_read_transfer_timing_logged = false;
			host->large_read_request_timing_logged = false;
		}
		dev_info(mmc_dev(mmc), "card detect: %s generation=%u\n",
			 present ? "inserted" : "removed",
			 host->card_generation);
	}

	ret = present;
	if (ret)
		return ret;

	/* Ensure bus width will be set (again) upon card (re)insertion */
	host->is_bus_width_set = false;

	return 0;
}

static irqreturn_t litex_mmc_interrupt(int irq, void *arg)
{
	struct mmc_host *mmc = arg;
	struct litex_mmc_host *host = mmc_priv(mmc);
	u32 pending = litex_read32(host->sdirq + LITEX_IRQ_PENDING);
	irqreturn_t ret = IRQ_NONE;

	/* Check for card change interrupt */
	if (pending & SDIRQ_CARD_DETECT) {
		litex_write32(host->sdirq + LITEX_IRQ_PENDING,
			      SDIRQ_CARD_DETECT);
		mmc_detect_change(mmc, msecs_to_jiffies(10));
		ret = IRQ_HANDLED;
	}

	/* Check for completion of the whole command/data transaction. */
	if (pending & SDIRQ_DATA_DONE) {
		/* Acknowledge and disable it so it doesn't keep interrupting. */
		litex_write32(host->sdirq + LITEX_IRQ_PENDING,
			      SDIRQ_DATA_DONE);
		litex_write32(host->sdirq + LITEX_IRQ_ENABLE,
			      SDIRQ_CARD_DETECT);
		complete(&host->data_done);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static u32 litex_mmc_response_len(struct mmc_command *cmd)
{
	if (cmd->flags & MMC_RSP_136)
		return SD_CTL_RESP_LONG;
	if (!(cmd->flags & MMC_RSP_PRESENT))
		return SD_CTL_RESP_NONE;
	if (cmd->flags & MMC_RSP_BUSY)
		return SD_CTL_RESP_SHORT_BUSY;
	return SD_CTL_RESP_SHORT;
}

static int litex_mmc_map_data(struct litex_mmc_host *host,
			      struct mmc_data *data,
			      enum litex_mmc_dma_cookie cookie,
			      u64 *map_ns)
{
	u64 start_ns = map_ns ? ktime_get_ns() : 0;
	int sg_count;

	if (data->host_cookie == LITEX_MMC_COOKIE_PREMAPPED)
		return data->sg_count;

	sg_count = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			      mmc_get_dma_dir(data));
	if (map_ns)
		*map_ns += ktime_get_ns() - start_ns;
	if (sg_count <= 0)
		return sg_count;

	data->sg_count = sg_count;
	data->host_cookie = cookie;
	return sg_count;
}

static void litex_mmc_unmap_data(struct litex_mmc_host *host,
				 struct mmc_data *data)
{
	if (data->host_cookie == LITEX_MMC_COOKIE_UNMAPPED)
		return;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		     mmc_get_dma_dir(data));
	data->host_cookie = LITEX_MMC_COOKIE_UNMAPPED;
}

static int litex_mmc_do_dma(struct litex_mmc_host *host, struct mmc_data *data,
			    struct litex_mmc_dma_context *ctx,
			    struct litex_mmc_dma_prep_timing *timing)
{
	u32 sg_status = 0;
	u64 phase_start_ns = 0;
	int ret;
	int sg_count;

	memset(ctx, 0, sizeof(*ctx));
	ctx->path = LITEX_MMC_DMA_BOUNCE;
	ctx->direction = mmc_get_dma_dir(data);
	ctx->frontend_dma = host->dma;
	ctx->len = data->blksz * data->blocks;
	if (ctx->len > host->buf_size)
		ctx->len = host->buf_size;

	if (data->flags & MMC_DATA_READ)
		ctx->transfer = SD_CTL_DATA_XFER_READ;
	else if (data->flags & MMC_DATA_WRITE)
		ctx->transfer = SD_CTL_DATA_XFER_WRITE;
	else
		return -EINVAL;

	if (timing)
		memset(timing, 0, sizeof(*timing));

	sg_count = litex_mmc_map_data(host, data, LITEX_MMC_COOKIE_MAPPED,
				      timing ? &timing->map_ns : NULL);
	ctx->mapped_nents = sg_count;
	if (sg_count == 1) {
		u64 end = (u64)sg_dma_address(data->sg) +
			sg_dma_len(data->sg);

		if (sg_dma_len(data->sg) == ctx->len &&
		    (u64)sg_dma_address(data->sg) <= U32_MAX &&
		    end <= U32_MAX) {
			ctx->path = LITEX_MMC_DMA_DIRECT;
			ctx->mapped = true;
			ctx->frontend_dma = sg_dma_address(data->sg);
			ctx->min_segment_bytes = ctx->len;
			ctx->max_segment_bytes = ctx->len;
		} else {
			if (timing)
				phase_start_ns = ktime_get_ns();
			litex_mmc_unmap_data(host, data);
			if (timing)
				timing->map_ns += ktime_get_ns() - phase_start_ns;
		}
	} else if (sg_count > 1 && host->sg_supported) {
		ctx->path = LITEX_MMC_DMA_SG;
		ctx->mapped = true;
		ctx->frontend_dma = sg_dma_address(data->sg);
		if (timing)
			phase_start_ns = ktime_get_ns();
		ret = litex_mmc_build_sg_table(host, data, ctx);
		if (timing)
			timing->sg_table_build_ns =
				ktime_get_ns() - phase_start_ns;
		if (ret) {
			if (timing)
				phase_start_ns = ktime_get_ns();
			litex_mmc_unmap_data(host, data);
			ctx->mapped = false;
			ctx->path = LITEX_MMC_DMA_BOUNCE;
			ctx->frontend_dma = host->dma;
			if (timing)
				timing->map_ns += ktime_get_ns() - phase_start_ns;
		} else {
			if (timing)
				phase_start_ns = ktime_get_ns();
			ret = litex_mmc_sg_prepare(host, ctx, SD_TIMEOUT_US,
						   &sg_status);
			ctx->sg_status = sg_status;
			if (timing)
				timing->sg_table_fetch_ns =
					ktime_get_ns() - phase_start_ns;
			if (ret)
				return ret;
		}
	} else if (sg_count > 1) {
		if (timing)
			phase_start_ns = ktime_get_ns();
		litex_mmc_unmap_data(host, data);
		if (timing)
			timing->map_ns += ktime_get_ns() - phase_start_ns;
	}

	if (ctx->path == LITEX_MMC_DMA_BOUNCE &&
	    ctx->transfer == SD_CTL_DATA_XFER_WRITE) {
		if (timing)
			phase_start_ns = ktime_get_ns();
		sg_copy_to_buffer(data->sg, data->sg_len,
				  host->buffer, ctx->len);
		if (timing)
			timing->bounce_copy_ns = ktime_get_ns() - phase_start_ns;
	}

	if (timing)
		phase_start_ns = ktime_get_ns();
	if (ctx->transfer == SD_CTL_DATA_XFER_READ) {
		litex_write8(host->sdreader + LITEX_BLK2MEM_ENA, 0);
		litex_write64(host->sdreader + LITEX_BLK2MEM_BASE,
			      ctx->frontend_dma);
		litex_write32(host->sdreader + LITEX_BLK2MEM_LEN, ctx->len);
		litex_write8(host->sdreader + LITEX_BLK2MEM_ENA, 1);
	} else {
		litex_write8(host->sdwriter + LITEX_MEM2BLK_ENA, 0);
		litex_write64(host->sdwriter + LITEX_MEM2BLK_BASE,
			      ctx->frontend_dma);
		litex_write32(host->sdwriter + LITEX_MEM2BLK_LEN, ctx->len);
	}

	litex_write16(host->sdcore + LITEX_CORE_BLKLEN, data->blksz);
	litex_write32(host->sdcore + LITEX_CORE_BLKCNT, data->blocks);
	ctx->frontend_programmed = true;
	if (timing)
		timing->program_ns = ktime_get_ns() - phase_start_ns;

	if (ctx->path == LITEX_MMC_DMA_SG &&
	    ctx->transfer == SD_CTL_DATA_XFER_WRITE) {
		if (timing)
			phase_start_ns = ktime_get_ns();
		ret = litex_mmc_sg_activate_write(host, SD_TIMEOUT_US,
						  &ctx->sg_status);
		if (timing)
			timing->sg_activate_ns = ktime_get_ns() - phase_start_ns;
		if (ret)
			return ret;
		ctx->write_dma_prestarted = true;
	}

	return 0;
}

static void litex_mmc_unmap_dma(struct litex_mmc_host *host,
				struct mmc_data *data,
				struct litex_mmc_dma_context *ctx,
				struct litex_mmc_request_timing *timing)
{
	u64 phase_start_ns;

	if (!ctx->mapped ||
	    data->host_cookie != LITEX_MMC_COOKIE_MAPPED)
		return;

	phase_start_ns = ktime_get_ns();
	litex_mmc_unmap_data(host, data);
	ctx->mapped = false;
	if (timing)
		timing->dma_unmap_ns = ktime_get_ns() - phase_start_ns;
}

static void litex_mmc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct litex_mmc_dma_stats *stats = &host->dma_stats;
	unsigned long flags;
	u64 map_ns = 0;
	int sg_count;

	if (!data)
		return;

	WARN_ON_ONCE(data->host_cookie != LITEX_MMC_COOKIE_UNMAPPED);
	data->host_cookie = LITEX_MMC_COOKIE_UNMAPPED;
	data->sg_count = 0;
	sg_count = litex_mmc_map_data(host, data,
				      LITEX_MMC_COOKIE_PREMAPPED, &map_ns);

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	stats->pre_req_count++;
	stats->pre_req_map_ns += map_ns;
	stats->max_pre_req_map_ns = max(stats->max_pre_req_map_ns, map_ns);
	if (sg_count > 0)
		stats->pre_req_mapped++;
	else
		stats->pre_req_failed++;
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);
}

static void litex_mmc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
			       int err)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct litex_mmc_dma_stats *stats = &host->dma_stats;
	unsigned long flags;
	u64 unmap_ns = 0;

	if (!data)
		return;

	if (data->host_cookie != LITEX_MMC_COOKIE_UNMAPPED) {
		unmap_ns = ktime_get_ns();
		litex_mmc_unmap_data(host, data);
		unmap_ns = ktime_get_ns() - unmap_ns;
	}

	spin_lock_irqsave(&host->dma_stats_lock, flags);
	stats->post_req_count++;
	stats->post_req_unmap_ns += unmap_ns;
	stats->max_post_req_unmap_ns =
		max(stats->max_post_req_unmap_ns, unmap_ns);
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);
}

static struct litex_mmc_async_request *
litex_mmc_acquire_async(struct litex_mmc_host *host)
{
	struct litex_mmc_async_request *async = NULL;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&host->request_lock, flags);
	for (i = 0; i < LITEX_MMC_ASYNC_SLOTS; i++) {
		if (!host->async[i].in_use) {
			host->async[i].in_use = true;
			async = &host->async[i];
			break;
		}
	}
	spin_unlock_irqrestore(&host->request_lock, flags);
	return async;
}

static void litex_mmc_release_async(struct litex_mmc_async_request *async)
{
	struct litex_mmc_host *host = async->host;
	unsigned long flags;

	spin_lock_irqsave(&host->request_lock, flags);
	async->in_use = false;
	spin_unlock_irqrestore(&host->request_lock, flags);
}

static void litex_mmc_async_done(struct litex_mmc_async_request *async)
{
	struct litex_mmc_host *host = async->host;
	struct mmc_request *mrq = async->mrq;

	/* The completion callback may submit the next request immediately. */
	async->mrq = NULL;
	litex_mmc_release_async(async);
	mmc_request_done(host->mmc, mrq);
}

static void litex_mmc_request_no_data(struct mmc_host *mmc,
				      struct mmc_request *mrq)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_command *sbc = mrq->sbc;
	struct mmc_command *stop = mrq->stop;
	unsigned int retries = cmd->retries;
	u32 response_len = litex_mmc_response_len(cmd);
	bool request_failed;

	if (!litex_mmc_get_cd(mmc)) {
		cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}
	host->active_request_sequence = ++host->request_sequence;

	if (sbc) {
		sbc->error = litex_mmc_send_cmd(host, sbc->opcode, sbc->arg,
						litex_mmc_response_len(sbc),
						SD_CTL_DATA_XFER_NONE, NULL);
		if (sbc->error) {
			host->is_bus_width_set = false;
			mmc_request_done(mmc, mrq);
			return;
		}
	}

	do {
		cmd->error = litex_mmc_send_cmd(host, cmd->opcode, cmd->arg,
						response_len,
						SD_CTL_DATA_XFER_NONE, NULL);
	} while (cmd->error && retries-- > 0);
	request_failed = cmd->error;

	if (response_len == SD_CTL_RESP_SHORT) {
		cmd->resp[0] = host->resp[3];
		cmd->resp[1] = host->resp[2] & 0xff;
	} else if (response_len == SD_CTL_RESP_LONG) {
		memcpy(cmd->resp, host->resp, sizeof(cmd->resp));
	}

	if (stop && (request_failed || !sbc)) {
		stop->error = litex_mmc_send_cmd(host, stop->opcode, stop->arg,
						 litex_mmc_response_len(stop),
						 SD_CTL_DATA_XFER_NONE, NULL);
		if (stop->error)
			host->is_bus_width_set = false;
	}

	mmc_request_done(mmc, mrq);
}

static void litex_mmc_finish_data_request(struct work_struct *work)
{
	struct litex_mmc_async_request *async =
		container_of(work, struct litex_mmc_async_request, work);
	struct litex_mmc_host *host = async->host;
	struct mmc_host *mmc = host->mmc;
	struct device *dev = mmc_dev(mmc);
	struct mmc_request *mrq = async->mrq;
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_command *sbc = mrq->sbc;
	struct mmc_data *data = mrq->data;
	struct mmc_command *stop = mrq->stop;
	struct litex_mmc_dma_context *dma_ctx = &async->dma_ctx;
	struct litex_mmc_request_timing *timing = &async->timing;
	u32 response_len = litex_mmc_response_len(cmd);
	u8 dma_status = 0;
	u8 failed_dma_done = 0;
	u32 failed_dma_offset = 0;
	u32 clear_status = 0;
	u32 abort_status = 0;
	bool request_failed;
	bool capture_bad_read;
	u64 trace_transfer_ns = 0;
	u64 trace_stop_ns = 0;
	u64 trace_cleanup_ns;
	u64 phase_start_ns;
	u64 post_start_ns;
	u64 now_ns;
	int ret;

	cmd->error = litex_mmc_finish_cmd(host, cmd->opcode, cmd->arg,
					  response_len, dma_ctx->transfer, data,
					  &async->started_cmd,
					  &timing->command);

	if (dma_ctx->path == LITEX_MMC_DMA_SG &&
	    !cmd->error && !data->error) {
		phase_start_ns = ktime_get_ns();
		ret = litex_mmc_sg_wait_done(host, dma_ctx,
					     async->data_timeout_us,
					     &dma_ctx->sg_status);
		timing->sg_complete_ns = ktime_get_ns() - phase_start_ns;
		if (ret)
			data->error = ret;
	}

	request_failed = cmd->error || data->error;
	if (async->trace_large_read)
		trace_transfer_ns = ktime_get_ns();
	capture_bad_read = dma_ctx->path != LITEX_MMC_DMA_BOUNCE &&
		dma_ctx->transfer == SD_CTL_DATA_XFER_READ &&
		data->error == -EILSEQ && !host->bad_read.valid;
	if (capture_bad_read) {
		failed_dma_done = litex_read8(host->sdreader +
					     LITEX_BLK2MEM_DONE);
		failed_dma_offset = litex_read32(host->sdreader +
						 LITEX_BLK2MEM_OFFSET);
	}

	if (request_failed) {
		litex_mmc_disable_dma(host, dma_ctx->transfer);
		if (dma_ctx->sg_armed)
			litex_write32(host->sdsg + LITEX_SG_CONTROL,
				      LITEX_SG_CONTROL_ENABLE |
				      LITEX_SG_CONTROL_ABORT);
		host->is_bus_width_set = false;
	}

	if (response_len == SD_CTL_RESP_SHORT) {
		cmd->resp[0] = host->resp[3];
		cmd->resp[1] = host->resp[2] & 0xff;
	} else if (response_len == SD_CTL_RESP_LONG) {
		memcpy(cmd->resp, host->resp, sizeof(cmd->resp));
	}

	if (stop && (request_failed || !sbc)) {
		phase_start_ns = ktime_get_ns();
		stop->error = litex_mmc_send_cmd(host, stop->opcode, stop->arg,
						 litex_mmc_response_len(stop),
						 SD_CTL_DATA_XFER_NONE, NULL);
		timing->stop_ns = ktime_get_ns() - phase_start_ns;
		if (stop->error)
			host->is_bus_width_set = false;
	}
	if (async->trace_large_read)
		trace_stop_ns = ktime_get_ns();

	if (request_failed) {
		phase_start_ns = ktime_get_ns();
		ret = litex_mmc_dma_wait_idle(host, dma_ctx->transfer,
					      async->data_timeout_us, &dma_status);
		timing->frontend_abort_ns = ktime_get_ns() - phase_start_ns;
		if (ret) {
			dev_crit(dev,
				 "DMA abort did not quiesce (cmd %u, done=%#x); keeping request mapped and incomplete\n",
				 cmd->opcode, dma_status);
			return;
		}
	}

	if (dma_ctx->sg_armed) {
		if (request_failed) {
			phase_start_ns = ktime_get_ns();
			ret = litex_mmc_sg_abort(host, dma_ctx,
						 async->data_timeout_us, &abort_status);
			timing->sg_abort_ns = ktime_get_ns() - phase_start_ns;
			if (!(dma_ctx->sg_status & LITEX_SG_STATUS_ERROR))
				dma_ctx->sg_status = abort_status;
			if (ret) {
				dev_crit(dev,
					 "SG abort did not quiesce (cmd %u, status=%#010x); keeping request mapped and incomplete\n",
					 cmd->opcode, dma_ctx->sg_status);
				return;
			}
		} else {
			phase_start_ns = ktime_get_ns();
			ret = litex_mmc_sg_clear(host, SD_TIMEOUT_US,
						 &clear_status);
			timing->sg_cleanup_ns = ktime_get_ns() - phase_start_ns;
			if (ret) {
				litex_mmc_log_sg_error(host, "cleanup", clear_status);
				data->error = ret;
				request_failed = true;
			} else {
				dma_ctx->sg_armed = false;
			}
		}
	}

	litex_mmc_unmap_dma(host, data, dma_ctx, timing);
	if (capture_bad_read &&
	    data->host_cookie != LITEX_MMC_COOKIE_UNMAPPED) {
		phase_start_ns = ktime_get_ns();
		litex_mmc_unmap_data(host, data);
		timing->dma_unmap_ns += ktime_get_ns() - phase_start_ns;
		dma_ctx->mapped = false;
	}
	post_start_ns = ktime_get_ns();
	if (capture_bad_read)
		litex_mmc_capture_bad_read(host, cmd, data, dma_ctx->len,
					   failed_dma_done, failed_dma_offset);

	if (!request_failed) {
		data->bytes_xfered = min(dma_ctx->len, mmc->max_req_size);
		if (dma_ctx->transfer == SD_CTL_DATA_XFER_READ &&
		    dma_ctx->path == LITEX_MMC_DMA_BOUNCE)
			sg_copy_from_buffer(data->sg, sg_nents(data->sg),
					    host->buffer, data->bytes_xfered);
	}

	if (async->trace_large_read) {
		trace_cleanup_ns = ktime_get_ns();
		dev_info(dev,
			 "first 1MiB read request timing: request=%llu blocks=%u total_us=%llu transfer_us=%llu stop_us=%llu cleanup_us=%llu path=%s sg_len=%u mapped_nents=%d\n",
			 (unsigned long long)host->active_request_sequence,
			 data->blocks,
			 (unsigned long long)div_u64(trace_cleanup_ns -
				async->trace_start_ns, NSEC_PER_USEC),
			 (unsigned long long)div_u64(trace_transfer_ns -
				async->trace_start_ns, NSEC_PER_USEC),
			 (unsigned long long)div_u64(trace_stop_ns - trace_transfer_ns,
							 NSEC_PER_USEC),
			 (unsigned long long)div_u64(trace_cleanup_ns - trace_stop_ns,
							 NSEC_PER_USEC),
			 litex_mmc_dma_path_name(dma_ctx->path), data->sg_len,
			 dma_ctx->mapped_nents);
		host->large_read_request_timing_logged = true;
	}

	now_ns = ktime_get_ns();
	timing->post_ns = now_ns - post_start_ns;
	if (async->profile_write)
		litex_mmc_account_write_profile(host, cmd, dma_ctx->len,
						dma_ctx->path, request_failed,
						timing,
						now_ns - timing->request_start_ns);
	if (dma_ctx->path == LITEX_MMC_DMA_SG)
		litex_mmc_account_sg_profile(host, data, dma_ctx,
					     request_failed, timing,
					     now_ns - timing->request_start_ns,
					     dma_ctx->sg_status);

	litex_mmc_async_done(async);
}

static void litex_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	struct device *dev = mmc_dev(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_command *sbc = mrq->sbc;
	struct mmc_data *data = mrq->data;
	struct litex_mmc_async_request *async;
	struct litex_mmc_dma_stats *stats = &host->dma_stats;
	unsigned long flags;
	u64 phase_start_ns;
	u64 post_start_ns;
	u64 now_ns;
	u32 abort_status = 0;
	int ret;

	if (!data) {
		litex_mmc_request_no_data(mmc, mrq);
		return;
	}

	if (!litex_mmc_get_cd(mmc)) {
		cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	async = litex_mmc_acquire_async(host);
	if (WARN_ON_ONCE(!async)) {
		cmd->error = -EBUSY;
		data->error = -EBUSY;
		mmc_request_done(mmc, mrq);
		return;
	}

	async->mrq = mrq;
	memset(&async->dma_ctx, 0, sizeof(async->dma_ctx));
	memset(&async->timing, 0, sizeof(async->timing));
	memset(&async->started_cmd, 0, sizeof(async->started_cmd));
	async->timing.request_start_ns = ktime_get_ns();
	async->data_timeout_us = litex_mmc_data_timeout_us(host, data);
	async->profile_write = data->flags & MMC_DATA_WRITE;
	async->trace_large_read = (data->flags & MMC_DATA_READ) &&
		data->blocks >= LITEX_MMC_MAX_REQ_SIZE / data->blksz &&
		!host->large_read_request_timing_logged;
	async->trace_start_ns = async->trace_large_read ? ktime_get_ns() : 0;
	data->bytes_xfered = 0;
	host->active_request_sequence = ++host->request_sequence;

	if (sbc) {
		phase_start_ns = ktime_get_ns();
		sbc->error = litex_mmc_send_cmd(host, sbc->opcode, sbc->arg,
						litex_mmc_response_len(sbc),
						SD_CTL_DATA_XFER_NONE, NULL);
		async->timing.sbc_ns = ktime_get_ns() - phase_start_ns;
		if (sbc->error) {
			host->is_bus_width_set = false;
			litex_mmc_async_done(async);
			return;
		}
	}

	phase_start_ns = ktime_get_ns();
	cmd->error = litex_mmc_set_bus_width(host);
	async->timing.bus_width_ns = ktime_get_ns() - phase_start_ns;
	if (cmd->error) {
		dev_err(dev, "Can't set bus width!\n");
		litex_mmc_async_done(async);
		return;
	}

	ret = litex_mmc_do_dma(host, data, &async->dma_ctx,
			       &async->timing.dma_prep);
	litex_mmc_account_dma(host, data, async->dma_ctx.len,
			      async->dma_ctx.path,
			      async->dma_ctx.mapped_nents);
	if (ret) {
		cmd->error = ret;
		data->error = ret;
		goto dma_prepare_failed;
	}
	litex_mmc_log_data_start(host, cmd, data, async->dma_ctx.len,
				 async->dma_ctx.path,
				 async->dma_ctx.mapped_nents,
				 async->dma_ctx.transfer);

	litex_mmc_start_cmd(host, cmd->opcode, cmd->arg,
			    litex_mmc_response_len(cmd),
			    async->dma_ctx.transfer, data,
			    async->dma_ctx.write_dma_prestarted,
			    &async->started_cmd, &async->timing.command);
	if (WARN_ON_ONCE(!schedule_work(&async->work))) {
		dev_crit(dev,
			 "Could not queue completion work (cmd %u); keeping request mapped and incomplete\n",
			 cmd->opcode);
		return;
	}
	spin_lock_irqsave(&host->dma_stats_lock, flags);
	stats->async_requests++;
	spin_unlock_irqrestore(&host->dma_stats_lock, flags);
	return;

dma_prepare_failed:
	if (async->dma_ctx.sg_armed) {
		phase_start_ns = ktime_get_ns();
		ret = litex_mmc_sg_abort(host, &async->dma_ctx, SD_TIMEOUT_US,
					 &abort_status);
		async->timing.sg_abort_ns = ktime_get_ns() - phase_start_ns;
		if (!(async->dma_ctx.sg_status & LITEX_SG_STATUS_ERROR))
			async->dma_ctx.sg_status = abort_status;
		if (ret) {
			dev_crit(dev,
				 "SG prepare abort did not quiesce (cmd %u, status=%#010x); keeping request mapped and incomplete\n",
				 cmd->opcode, async->dma_ctx.sg_status);
			return;
		}
	}
	litex_mmc_unmap_dma(host, data, &async->dma_ctx, &async->timing);
	post_start_ns = ktime_get_ns();
	now_ns = ktime_get_ns();
	async->timing.post_ns = now_ns - post_start_ns;
	if (async->profile_write)
		litex_mmc_account_write_profile(host, cmd, async->dma_ctx.len,
						async->dma_ctx.path, true,
						&async->timing,
						now_ns -
						async->timing.request_start_ns);
	if (async->dma_ctx.path == LITEX_MMC_DMA_SG)
		litex_mmc_account_sg_profile(host, data, &async->dma_ctx, true,
					     &async->timing,
					     now_ns -
					     async->timing.request_start_ns,
					     async->dma_ctx.sg_status);
	litex_mmc_async_done(async);
}

static void litex_mmc_setclk(struct litex_mmc_host *host, unsigned int freq)
{
	struct device *dev = mmc_dev(host->mmc);
	u32 div;

	div = freq ? DIV_ROUND_UP(host->ref_clk, freq) : 256U;
	div = clamp(div, 2U, 256U);
	dev_dbg(dev, "sd_clk_freq=%d: set to %d via div=%d\n",
		freq, host->ref_clk / ((div + 1) & ~1U), div);
	litex_write16(host->sdphy + LITEX_PHY_CLOCKERDIV, div);
	host->sd_clk = freq;
}

static void litex_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct litex_mmc_host *host = mmc_priv(mmc);

	/* The SD specification requires at least 74 idle clocks before CMD0. */
	if (ios->chip_select == MMC_CS_HIGH) {
		litex_mmc_setclk(host, SD_INIT_CLK_HZ);
		litex_write8(host->sdphy + LITEX_PHY_INITIALIZE, 1);
		fsleep(SD_INIT_DELAY_US);
		return;
	}

	/*
	 * NOTE: Ignore any ios->bus_width updates; they occur right after
	 * the mmc core sends its own acmd6 bus-width change notification,
	 * which is redundant since we snoop on the command flow and inject
	 * an early acmd6 before the first data transfer command is sent!
	 */

	/* Update sd_clk */
	if (ios->clock != host->sd_clk)
		litex_mmc_setclk(host, ios->clock);
}

static const struct mmc_host_ops litex_mmc_ops = {
	.get_cd = litex_mmc_get_cd,
	.pre_req = litex_mmc_pre_req,
	.post_req = litex_mmc_post_req,
	.request = litex_mmc_request,
	.set_ios = litex_mmc_set_ios,
};

static int litex_mmc_irq_init(struct platform_device *pdev,
			      struct litex_mmc_host *host)
{
	struct device *dev = mmc_dev(host->mmc);
	int ret;

	ret = platform_get_irq_optional(pdev, 0);
	if (ret < 0 && ret != -ENXIO)
		return ret;
	if (ret > 0) {
		host->irq = ret;
	} else {
		dev_warn(dev, "Failed to get IRQ, using polling\n");
		goto use_polling;
	}

	host->sdirq = devm_platform_ioremap_resource_byname(pdev, "irq");
	if (IS_ERR(host->sdirq))
		return PTR_ERR(host->sdirq);

	ret = devm_request_irq(dev, host->irq, litex_mmc_interrupt, 0,
			       "litex-mmc", host->mmc);
	if (ret < 0) {
		dev_warn(dev, "IRQ request error %d, using polling\n", ret);
		goto use_polling;
	}

	/* Clear & enable card-change interrupts */
	litex_write32(host->sdirq + LITEX_IRQ_PENDING, SDIRQ_CARD_DETECT);
	litex_write32(host->sdirq + LITEX_IRQ_ENABLE, SDIRQ_CARD_DETECT);

	return 0;

use_polling:
	host->mmc->caps |= MMC_CAP_NEEDS_POLL;
	host->irq = 0;
	return 0;
}

static void litex_mmc_free_host_wrapper(void *mmc)
{
	mmc_free_host(mmc);
}

static int litex_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct litex_mmc_host *host;
	struct mmc_host *mmc;
	struct litex_mmc_sg_entry *sg_table;
	struct resource *res;
	struct clk *clk;
	u32 sg_cap = 0;
	u32 sg_status = 0;
	int i;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct litex_mmc_host), dev);
	if (!mmc)
		return -ENOMEM;

	mmc->max_segs = LITEX_MMC_DIRECT_MAX_SEGS;
	mmc->max_seg_size = LITEX_MMC_MAX_REQ_SIZE;
	mmc->max_req_size = LITEX_MMC_MAX_REQ_SIZE;
	mmc->max_blk_count = LITEX_MMC_MAX_REQ_SIZE / mmc->max_blk_size;

	ret = devm_add_action_or_reset(dev, litex_mmc_free_host_wrapper, mmc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Can't register mmc_free_host action\n");

	host = mmc_priv(mmc);
	host->mmc = mmc;
	spin_lock_init(&host->dma_stats_lock);
	spin_lock_init(&host->request_lock);
	for (i = 0; i < LITEX_MMC_ASYNC_SLOTS; i++) {
		host->async[i].host = host;
		INIT_WORK(&host->async[i].work,
			  litex_mmc_finish_data_request);
	}

	/* Initialize clock source */
	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "can't get clock\n");
	host->ref_clk = clk_get_rate(clk);
	host->sd_clk = 0;

	/*
	 * LiteSDCard only supports 4-bit bus width; therefore, we MUST inject
	 * a SET_BUS_WIDTH (acmd6) before the very first data transfer, earlier
	 * than when the mmc subsystem would normally get around to it!
	 */
	host->is_bus_width_set = false;
	host->app_cmd = false;

	/* CPUSTC's AXI Stream DMA engines expose a 32-bit address bus. */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	host->buf_size = mmc->max_req_size * 2;
	host->buffer = dmam_alloc_coherent(dev, host->buf_size,
					   &host->dma, GFP_KERNEL);
	if (!host->buffer)
		return -ENOMEM;
	host->bad_read_blob.data = host->bad_read.data;

	host->sdphy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(host->sdphy))
		return PTR_ERR(host->sdphy);

	host->sdcore = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(host->sdcore))
		return PTR_ERR(host->sdcore);

	host->sdreader = devm_platform_ioremap_resource_byname(pdev, "reader");
	if (IS_ERR(host->sdreader))
		return PTR_ERR(host->sdreader);

	host->sdwriter = devm_platform_ioremap_resource_byname(pdev, "writer");
	if (IS_ERR(host->sdwriter))
		return PTR_ERR(host->sdwriter);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sg");
	if (res) {
		host->sdsg = devm_ioremap_resource(dev, res);
		if (IS_ERR(host->sdsg))
			return PTR_ERR(host->sdsg);

		sg_cap = litex_read32(host->sdsg + LITEX_SG_CAP);
		if (sg_cap == LITEX_SG_CAP_V1) {
			sg_table = dmam_alloc_coherent(dev, LITEX_SG_TABLE_SIZE,
						       &host->sg_table_dma, GFP_KERNEL);
			if (!sg_table)
				return -ENOMEM;
			host->sg_table = sg_table;
			host->sg_supported = true;
			ret = litex_mmc_sg_clear(host, SD_TIMEOUT_US,
						 &sg_status);
			if (ret)
				return dev_err_probe(dev, ret,
					"SG ABI v1 did not enter idle state (status=%#010x)\n",
					sg_status);
			mmc->max_segs = LITEX_SG_MAX_ENTRIES;
		} else {
			dev_info(dev,
				 "SG ABI unavailable (cap=%#010x); using single-segment DMA\n",
				 sg_cap);
		}
	}

	/* Ensure DMA bus masters are disabled */
	litex_write8(host->sdreader + LITEX_BLK2MEM_ENA, 0);
	litex_write8(host->sdwriter + LITEX_MEM2BLK_ENA, 0);

	init_completion(&host->data_done);
	ret = litex_mmc_irq_init(pdev, host);
	if (ret)
		return ret;

	mmc->ops = &litex_mmc_ops;

	ret = mmc_regulator_get_supply(mmc);
	if (ret || mmc->ocr_avail == 0) {
		dev_warn(dev, "can't get voltage, defaulting to 3.3V\n");
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	}

	/* Keep card identification at the SD-defined initialization rate. */
	mmc->f_min = SD_INIT_CLK_HZ;
	mmc->f_max = host->ref_clk / 2;

	ret = mmc_of_parse(mmc);
	if (ret)
		return ret;

	/* Force 4-bit bus_width (only width supported by hardware) */
	mmc->caps &= ~MMC_CAP_8_BIT_DATA;
	mmc->caps |= MMC_CAP_4_BIT_DATA;

	/* Set default capabilities */
	mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY |
		     MMC_CAP_DRIVER_TYPE_D |
		     MMC_CAP_CMD23;
	mmc->caps2 |= MMC_CAP2_NO_WRITE_PROTECT |
		      MMC_CAP2_NO_SDIO |
		      MMC_CAP2_NO_MMC;

	platform_set_drvdata(pdev, host);

	ret = mmc_add_host(mmc);
	if (ret)
		return ret;
	if (mmc->debugfs_root) {
		debugfs_create_blob("litex_bad_read.bin", 0400,
				    mmc->debugfs_root, &host->bad_read_blob);
		debugfs_create_devm_seqfile(dev, "litex_bad_read_meta",
					    mmc->debugfs_root,
					    litex_mmc_bad_read_meta_show);
		debugfs_create_devm_seqfile(dev, "litex_dma_stats",
					    mmc->debugfs_root,
					    litex_mmc_dma_stats_show);
		debugfs_create_devm_seqfile(dev, "litex_write_profile",
					    mmc->debugfs_root,
					    litex_mmc_write_profile_show);
		debugfs_create_file("litex_write_profile_reset", 0200,
				    mmc->debugfs_root, host,
				    &litex_mmc_write_profile_reset_fops);
		debugfs_create_devm_seqfile(dev, "litex_sg_profile",
					    mmc->debugfs_root,
					    litex_mmc_sg_profile_show);
		debugfs_create_file("litex_sg_profile_reset", 0200,
				    mmc->debugfs_root, host,
				    &litex_mmc_sg_profile_reset_fops);
	}

	dev_info(dev,
		 "LiteX MMC controller initialized (ref=%u Hz, range=%u-%u Hz).\n",
		 host->ref_clk, mmc->f_min, mmc->f_max);
	dev_info(dev, "DMA limits: max_req=%u max_seg=%u max_segs=%u.\n",
		 mmc->max_req_size, mmc->max_seg_size, mmc->max_segs);
	if (host->sg_supported)
		dev_info(dev,
			 "SG ABI v1 enabled: table=%pad entries=%u bytes=%u.\n",
			 &host->sg_table_dma, LITEX_SG_MAX_ENTRIES,
			 LITEX_SG_TABLE_SIZE);
	return 0;
}

static int litex_mmc_remove(struct platform_device *pdev)
{
	struct litex_mmc_host *host = platform_get_drvdata(pdev);
	int i;

	mmc_remove_host(host->mmc);
	for (i = 0; i < LITEX_MMC_ASYNC_SLOTS; i++)
		cancel_work_sync(&host->async[i].work);
	return 0;
}

static const struct of_device_id litex_match[] = {
	{ .compatible = "litex,mmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, litex_match);

static struct platform_driver litex_mmc_driver = {
	.probe = litex_mmc_probe,
	.remove = litex_mmc_remove,
	.driver = {
		.name = "litex-mmc",
		.of_match_table = litex_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(litex_mmc_driver);

MODULE_DESCRIPTION("LiteX SDCard driver");
MODULE_AUTHOR("Antmicro <contact@antmicro.com>");
MODULE_AUTHOR("Kamil Rakoczy <krakoczy@antmicro.com>");
MODULE_AUTHOR("Maciej Dudek <mdudek@internships.antmicro.com>");
MODULE_AUTHOR("Paul Mackerras <paulus@ozlabs.org>");
MODULE_AUTHOR("Gabriel Somlo <gsomlo@gmail.com>");
MODULE_LICENSE("GPL v2");

#if IS_ENABLED(CONFIG_MMC_LITEX_KUNIT_TEST)
#include "litex_mmc_test.c"
#endif
