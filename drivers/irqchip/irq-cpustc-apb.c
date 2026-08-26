// SPDX-License-Identifier: GPL-2.0
/*
 * CPUSTC APB interrupt controller
 */

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define CPUSTC_APB_INTC_PENDING		0x00
#define CPUSTC_APB_INTC_MASK		0x04
#define CPUSTC_APB_INTC_NR_IRQS		8
#define CPUSTC_APB_INTC_VALID_MASK	GENMASK(7, 0)

static void cpustc_apb_intc_handle_irq(struct irq_desc *desc)
{
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct irq_chip_generic *gc;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 pending;

	chained_irq_enter(chip, desc);

	gc = irq_get_domain_generic_chip(domain, 0);
	pending = readl(gc->reg_base + CPUSTC_APB_INTC_PENDING) &
		  CPUSTC_APB_INTC_VALID_MASK;

	while (pending) {
		unsigned int hwirq = __ffs(pending);

		generic_handle_domain_irq(domain, hwirq);
		pending &= ~BIT(hwirq);
	}

	chained_irq_exit(chip, desc);
}

static int __init cpustc_apb_intc_init(struct device_node *node,
				       struct device_node *parent)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	struct irq_domain *domain;
	void __iomem *base;
	int parent_irq;
	int ret;

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%pOF: unable to map registers\n", node);
		return -ENOMEM;
	}

	parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq) {
		pr_err("%pOF: unable to map parent interrupt\n", node);
		ret = -EINVAL;
		goto err_unmap;
	}

	domain = irq_domain_add_linear(node, CPUSTC_APB_INTC_NR_IRQS,
				       &irq_generic_chip_ops, NULL);
	if (!domain) {
		pr_err("%pOF: unable to create IRQ domain\n", node);
		ret = -ENOMEM;
		goto err_dispose_parent;
	}

	ret = irq_alloc_domain_generic_chips(domain, CPUSTC_APB_INTC_NR_IRQS,
					     1, "CPUSTC-APB",
					     handle_fasteoi_irq,
					     IRQ_NOREQUEST | IRQ_NOPROBE |
					     IRQ_NOAUTOEN, 0,
					     IRQ_GC_INIT_MASK_CACHE);
	if (ret) {
		pr_err("%pOF: unable to allocate generic IRQ chip\n", node);
		goto err_remove_domain;
	}

	gc = irq_get_domain_generic_chip(domain, 0);
	gc->reg_base = base;

	ct = gc->chip_types;
	ct->regs.mask = CPUSTC_APB_INTC_MASK;
	ct->regs.eoi = CPUSTC_APB_INTC_PENDING;
	ct->chip.irq_mask = irq_gc_mask_clr_bit;
	ct->chip.irq_unmask = irq_gc_mask_set_bit;
	ct->chip.irq_eoi = irq_gc_eoi;
	ct->chip.flags = IRQCHIP_EOI_THREADED;

	/* Keep the parent quiet until a child IRQ is requested. */
	writel(0, base + CPUSTC_APB_INTC_MASK);
	writel(CPUSTC_APB_INTC_VALID_MASK,
	       base + CPUSTC_APB_INTC_PENDING);

	irq_set_chained_handler_and_data(parent_irq,
					 cpustc_apb_intc_handle_irq, domain);

	return 0;

err_remove_domain:
	irq_domain_remove(domain);
err_dispose_parent:
	irq_dispose_mapping(parent_irq);
err_unmap:
	iounmap(base);
	return ret;
}

IRQCHIP_DECLARE(cpustc_apb_intc, "cpustc,apb-interrupt-controller",
		cpustc_apb_intc_init);
