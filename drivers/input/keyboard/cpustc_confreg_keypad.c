// SPDX-License-Identifier: GPL-2.0-only
/* CPUSTC confreg 4x4 polled keypad driver. */

#include <linux/bitops.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define CPUSTC_KEYPAD_ROWS		4
#define CPUSTC_KEYPAD_COLUMNS		4
#define CPUSTC_KEYPAD_MASK		GENMASK(15, 0)
#define CPUSTC_KEYPAD_POLL_MS		10

struct cpustc_confreg_keypad {
	void __iomem *base;
	int active_scan;
};

static void cpustc_confreg_keypad_poll(struct input_dev *input)
{
	struct cpustc_confreg_keypad *keypad = input_get_drvdata(input);
	const unsigned short *keycodes = input->keycode;
	u32 state = readl(keypad->base) & CPUSTC_KEYPAD_MASK;
	int scan = -1;

	if (is_power_of_2(state)) {
		int candidate = __ffs(state);

		if (keycodes[candidate] != KEY_RESERVED)
			scan = candidate;
	}

	if (scan == keypad->active_scan)
		return;

	if (keypad->active_scan >= 0) {
		input_event(input, EV_MSC, MSC_SCAN, keypad->active_scan);
		input_report_key(input, keycodes[keypad->active_scan], 0);
	}

	if (scan >= 0) {
		input_event(input, EV_MSC, MSC_SCAN, scan);
		input_report_key(input, keycodes[scan], 1);
	}

	input_sync(input);
	keypad->active_scan = scan;
}

static int cpustc_confreg_keypad_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpustc_confreg_keypad *keypad;
	struct input_dev *input;
	u32 poll_interval = CPUSTC_KEYPAD_POLL_MS;
	int error;

	keypad = devm_kzalloc(dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	keypad->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(keypad->base))
		return PTR_ERR(keypad->base);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	keypad->active_scan = -1;
	input->name = "CPUSTC confreg keypad";
	input->phys = "cpustc-confreg-keypad/input0";
	input->id.bustype = BUS_HOST;
	input->dev.parent = dev;
	input_set_drvdata(input, keypad);

	error = matrix_keypad_build_keymap(NULL, NULL,
					   CPUSTC_KEYPAD_ROWS,
					   CPUSTC_KEYPAD_COLUMNS,
					   NULL, input);
	if (error)
		return error;

	input_set_capability(input, EV_MSC, MSC_SCAN);

	error = input_setup_polling(input, cpustc_confreg_keypad_poll);
	if (error)
		return error;

	device_property_read_u32(dev, "poll-interval", &poll_interval);
	if (!poll_interval)
		return dev_err_probe(dev, -EINVAL,
				     "poll-interval must be nonzero\n");
	input_set_poll_interval(input, poll_interval);

	error = input_register_device(input);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to register input device\n");

	platform_set_drvdata(pdev, keypad);
	return 0;
}

static const struct of_device_id cpustc_confreg_keypad_of_match[] = {
	{ .compatible = "cpustc,confreg-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpustc_confreg_keypad_of_match);

static struct platform_driver cpustc_confreg_keypad_driver = {
	.probe = cpustc_confreg_keypad_probe,
	.driver = {
		.name = "cpustc-confreg-keypad",
		.of_match_table = cpustc_confreg_keypad_of_match,
	},
};
module_platform_driver(cpustc_confreg_keypad_driver);

MODULE_DESCRIPTION("CPUSTC confreg polled keypad driver");
MODULE_LICENSE("GPL");
