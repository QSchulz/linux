/* Touchscreen driver for Allwinner SoCs (A10, A13, A31) GPADC
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/iio/consumer.h>
#include <linux/mfd/sunxi-gpadc-mfd.h>

struct sunxi_gpadc_ts {
	struct iio_cb_buffer	*buffer;
	struct input_dev	*input;
	struct platform_device	*pdev;
	unsigned int		tp_up_irq;
	bool			ignore_fifo_data;
};

static int sunxi_gpadc_ts_open(struct input_dev *dev)
{
	struct sunxi_gpadc_ts *info = input_get_drvdata(dev);
	int ret;

	ret = iio_channel_start_all_cb(info->buffer);
	if (ret) {
		dev_err(dev->dev.parent,
			"failed to start iio channels with callback\n");
		return ret;
	}

	enable_irq(info->tp_up_irq);

	return 0;
}

static void sunxi_gpadc_ts_close(struct input_dev *dev)
{
	struct sunxi_gpadc_ts *info = input_get_drvdata(dev);

	iio_channel_stop_all_cb(info->buffer);

	disable_irq(info->tp_up_irq);
}

/*
 * This function will be called by iio_push_to_buffers from another driver
 * (namely sunxi-gpadc-iio). It will be passed the buffer filled with input
 * values (X value then Y value) and the sunxi_gpadc_ts structure representing
 * the device.
 */
static int sunxi_gpadc_ts_callback(const void *data, void *private)
{
	const struct sunxi_gpadc_buffer *buffer = data;
	struct sunxi_gpadc_ts *info = private;
	int i = 0;

	/* Locations in the first buffer after an up event are unreliable */
	if (info->ignore_fifo_data) {
		info->ignore_fifo_data = false;
		return 0;
	}

	while (i + 1 < buffer->buff_size) {
		input_event(info->input, EV_ABS, ABS_X, buffer->buffer[i++]);
		input_event(info->input, EV_ABS, ABS_Y, buffer->buffer[i++]);
		input_event(info->input, EV_KEY, BTN_TOUCH, 1);
		input_sync(info->input);
	}

	return 0;
}

static irqreturn_t sunxi_gpadc_tp_up_irq_handler(int irq, void *dev_id)
{
	struct sunxi_gpadc_ts *info = dev_id;

	info->ignore_fifo_data = true;

	input_event(info->input, EV_KEY, BTN_TOUCH, 0);
	input_sync(info->input);

	return IRQ_HANDLED;
}

static int sunxi_gpadc_ts_probe(struct platform_device *pdev)
{
	struct input_dev *input;
	struct sunxi_gpadc_ts *info;
	int ret, irq;
	struct sunxi_gpadc_mfd_dev *sunxi_gpadc_mfd_dev;

	input = devm_input_allocate_device(&pdev->dev);
	if (!input)
		return -ENOMEM;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->buffer = iio_channel_get_all_cb(&pdev->dev,
					      &sunxi_gpadc_ts_callback,
					      (void *)info);
	if (IS_ERR(info->buffer)) {
		if (PTR_ERR(info->buffer) == -ENODEV)
			return -EPROBE_DEFER;
		return PTR_ERR(info->buffer);
	}

	info->pdev = pdev;
	info->input = input;
	info->ignore_fifo_data = false;
	platform_set_drvdata(pdev, info);

	input->dev.parent = &pdev->dev;
	input->name = "sunxi-gpadc-ts";
	input->id.bustype = BUS_HOST;
	input->open = sunxi_gpadc_ts_open;
	input->close = sunxi_gpadc_ts_close;
	__set_bit(EV_SYN, input->evbit);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, 4095, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 4095, 0, 0);
	input_set_drvdata(input, info);

	irq = platform_get_irq_byname(pdev, "TP_UP_PENDING");
	if (irq < 0) {
		dev_err(&pdev->dev, "no TP_UP_PENDING interrupt registered\n");
		ret = irq;
		goto err;
	}

	sunxi_gpadc_mfd_dev = dev_get_drvdata(pdev->dev.parent);

	irq = regmap_irq_get_virq(sunxi_gpadc_mfd_dev->regmap_irqc, irq);
	ret = devm_request_any_context_irq(&pdev->dev, irq,
					   sunxi_gpadc_tp_up_irq_handler, 0,
					   "tp_up", info);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"could not request TP_UP_PENDING interrupt: %d\n", ret);
		goto err;
	}

	info->tp_up_irq = irq;
	disable_irq(irq);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err;
	}

	return 0;

err:
	iio_channel_release_all_cb(info->buffer);

	return ret;
}

static int sunxi_gpadc_ts_remove(struct platform_device *pdev)
{
	struct sunxi_gpadc_ts *info = platform_get_drvdata(pdev);

	iio_channel_stop_all_cb(info->buffer);
	iio_channel_release_all_cb(info->buffer);

	disable_irq(info->tp_up_irq);

	return 0;
}

static struct platform_driver sunxi_gpadc_ts_driver = {
	.driver = {
		.name = "sunxi-gpadc-ts",
	},
	.probe = sunxi_gpadc_ts_probe,
	.remove = sunxi_gpadc_ts_remove,
};

module_platform_driver(sunxi_gpadc_ts_driver);

MODULE_DESCRIPTION("Touchscreen driver for Allwinner SoCs (A10, A13, A31) GPADC");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL v2");
