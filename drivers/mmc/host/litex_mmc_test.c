// SPDX-License-Identifier: GPL-2.0

#include <kunit/test.h>

static void litex_mmc_sg_table_two_entries(struct kunit *test)
{
	struct litex_mmc_sg_entry table[2] = { };
	struct litex_mmc_dma_context ctx = {
		.len = 1024,
		.mapped_nents = 2,
	};
	struct litex_mmc_host *host;
	struct scatterlist sg[2];
	struct mmc_data data = {
		.sg = sg,
	};
	int ret;

	host = kunit_kzalloc(test, sizeof(*host), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, host);
	host->sg_table = table;
	sg_init_table(sg, ARRAY_SIZE(sg));
	sg_dma_address(&sg[0]) = 0x81000000;
	sg_dma_len(&sg[0]) = 512;
	sg_dma_address(&sg[1]) = 0x82000000;
	sg_dma_len(&sg[1]) = 512;

	ret = litex_mmc_build_sg_table(host, &data, &ctx);

	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(table[0].dma_addr), 0x81000000U);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(table[0].length), 512U);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(table[1].dma_addr), 0x82000000U);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(table[1].length), 512U);
	KUNIT_EXPECT_EQ(test, ctx.min_segment_bytes, 512U);
	KUNIT_EXPECT_EQ(test, ctx.max_segment_bytes, 512U);
}

static void litex_mmc_sg_table_rejects_total_mismatch(struct kunit *test)
{
	struct litex_mmc_sg_entry table[2] = { };
	struct litex_mmc_dma_context ctx = {
		.len = 1024,
		.mapped_nents = 2,
	};
	struct litex_mmc_host *host;
	struct scatterlist sg[2];
	struct mmc_data data = {
		.sg = sg,
	};

	host = kunit_kzalloc(test, sizeof(*host), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, host);
	host->sg_table = table;
	sg_init_table(sg, ARRAY_SIZE(sg));
	sg_dma_address(&sg[0]) = 0x81000000;
	sg_dma_len(&sg[0]) = 512;
	sg_dma_address(&sg[1]) = 0x82000000;
	sg_dma_len(&sg[1]) = 256;

	KUNIT_EXPECT_EQ(test,
			litex_mmc_build_sg_table(host, &data, &ctx), -EINVAL);
}

static void litex_mmc_sg_table_rejects_address_overflow(struct kunit *test)
{
	struct litex_mmc_sg_entry table[2] = { };
	struct litex_mmc_dma_context ctx = {
		.len = 1024,
		.mapped_nents = 2,
	};
	struct litex_mmc_host *host;
	struct scatterlist sg[2];
	struct mmc_data data = {
		.sg = sg,
	};

	host = kunit_kzalloc(test, sizeof(*host), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, host);
	host->sg_table = table;
	sg_init_table(sg, ARRAY_SIZE(sg));
	sg_dma_address(&sg[0]) = 0xfffffe00;
	sg_dma_len(&sg[0]) = 512;
	sg_dma_address(&sg[1]) = 0x82000000;
	sg_dma_len(&sg[1]) = 512;

	KUNIT_EXPECT_EQ(test,
			litex_mmc_build_sg_table(host, &data, &ctx), -EINVAL);
}

static void litex_mmc_sg_status_errors(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			litex_mmc_sg_status_to_errno(LITEX_SG_STATUS_DONE), 0);
	KUNIT_EXPECT_EQ(test,
			litex_mmc_sg_status_to_errno(LITEX_SG_STATUS_ERROR |
				(3 << LITEX_SG_STATUS_ERROR_SHIFT)), -EINVAL);
	KUNIT_EXPECT_EQ(test,
			litex_mmc_sg_status_to_errno(LITEX_SG_STATUS_ERROR |
				(16 << LITEX_SG_STATUS_ERROR_SHIFT)), -EIO);
	KUNIT_EXPECT_EQ(test,
			litex_mmc_sg_status_to_errno(LITEX_SG_STATUS_ABORTED),
				-ECANCELED);
}

static void litex_mmc_async_slots_track_in_use_work(struct kunit *test)
{
	struct litex_mmc_async_request *first;
	struct litex_mmc_async_request *second;
	struct litex_mmc_async_request *reused;
	struct litex_mmc_host *host;
	int i;

	host = kunit_kzalloc(test, sizeof(*host), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, host);
	spin_lock_init(&host->request_lock);
	for (i = 0; i < LITEX_MMC_ASYNC_SLOTS; i++)
		host->async[i].host = host;

	first = litex_mmc_acquire_async(host);
	second = litex_mmc_acquire_async(host);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, first);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, second);
	KUNIT_EXPECT_TRUE(test, first != second);
	KUNIT_EXPECT_PTR_EQ(test, litex_mmc_acquire_async(host), NULL);

	litex_mmc_release_async(first);
	reused = litex_mmc_acquire_async(host);
	KUNIT_EXPECT_PTR_EQ(test, reused, first);

	litex_mmc_release_async(second);
	litex_mmc_release_async(reused);
}

static struct kunit_case litex_mmc_test_cases[] = {
	KUNIT_CASE(litex_mmc_sg_table_two_entries),
	KUNIT_CASE(litex_mmc_sg_table_rejects_total_mismatch),
	KUNIT_CASE(litex_mmc_sg_table_rejects_address_overflow),
	KUNIT_CASE(litex_mmc_sg_status_errors),
	KUNIT_CASE(litex_mmc_async_slots_track_in_use_work),
	{ }
};

static struct kunit_suite litex_mmc_test_suite = {
	.name = "litex-mmc",
	.test_cases = litex_mmc_test_cases,
};

static struct kunit_suite *litex_mmc_test_suite_array[] = {
	&litex_mmc_test_suite,
	NULL,
};

static struct kunit_suite **litex_mmc_test_suites
	__used __section(".kunit_test_suites") = litex_mmc_test_suite_array;
