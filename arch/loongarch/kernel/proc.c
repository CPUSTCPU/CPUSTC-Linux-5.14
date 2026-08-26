// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/bootinfo.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/idle.h>
#include <asm/processor.h>
#include <asm/time.h>

/*
 * No lock; only written during early bootup by CPU 0.
 */
static RAW_NOTIFIER_HEAD(proc_cpuinfo_chain);

int __ref register_proc_cpuinfo_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&proc_cpuinfo_chain, nb);
}

int proc_cpuinfo_notifier_call_chain(unsigned long val, void *v)
{
	return raw_notifier_call_chain(&proc_cpuinfo_chain, val, v);
}

static int show_cpuinfo(struct seq_file *m, void *v)
{
	unsigned long n = (unsigned long)v - 1;

	seq_printf(m, "processor\t\t: %ld\n", n);
	seq_printf(m, "CPU Family\t\t: %s\n", __cpu_family[n] ? : "Unknown");
	seq_printf(m, "Model Name\t\t: %s\n", __cpu_full_name[n] ? : "Unknown");
	if (cpu_clock_freq) {
		u32 mhz_x100 = (u32)div_u64(cpu_clock_freq, 10000);

		seq_printf(m, "CPU MHz\t\t\t: %u.%02u\n", mhz_x100 / 100, mhz_x100 % 100);
	}
	seq_puts(m, "\n");

	return 0;
}

static void *c_start(struct seq_file *m, loff_t *pos)
{
	unsigned long i = *pos;

	return i < NR_CPUS ? (void *)(i + 1) : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return c_start(m, pos);
}

static void c_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations cpuinfo_op = {
	.start	= c_start,
	.next	= c_next,
	.stop	= c_stop,
	.show	= show_cpuinfo,
};
