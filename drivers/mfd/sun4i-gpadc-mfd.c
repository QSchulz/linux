/* ADC MFD core driver for sunxi platforms
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>

#include <linux/mfd/sun4i-gpadc-mfd.h>

static struct resource adc_resources[] = {
	{
		.name	= "FIFO_DATA_PENDING",
		.start	= SUN4I_GPADC_IRQ_FIFO_DATA,
		.end	= SUN4I_GPADC_IRQ_FIFO_DATA,
		.flags	= IORESOURCE_IRQ,
	}, {
		.name	= "TEMP_DATA_PENDING",
		.start	= SUN4I_GPADC_IRQ_TEMP_DATA,
		.end	= SUN4I_GPADC_IRQ_TEMP_DATA,
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct regmap_irq sun4i_gpadc_mfd_regmap_irq[] = {
	REGMAP_IRQ_REG(SUN4I_GPADC_IRQ_FIFO_DATA, 0,
		       SUN4I_GPADC_INT_FIFOC_TP_DATA_IRQ_EN),
	REGMAP_IRQ_REG(SUN4I_GPADC_IRQ_TEMP_DATA, 0,
		       SUN4I_GPADC_INT_FIFOC_TEMP_IRQ_EN),
};

static const struct regmap_irq_chip sun4i_gpadc_mfd_regmap_irq_chip = {
	.name = "sun4i_gpadc_mfd_irq_chip",
	.status_base = SUN4I_GPADC_INT_FIFOS,
	.ack_base = SUN4I_GPADC_INT_FIFOS,
	.mask_base = SUN4I_GPADC_INT_FIFOC,
	.init_ack_masked = true,
	.mask_invert = true,
	.irqs = sun4i_gpadc_mfd_regmap_irq,
	.num_irqs = ARRAY_SIZE(sun4i_gpadc_mfd_regmap_irq),
	.num_regs = 1,
};

static struct mfd_cell sun4i_gpadc_mfd_cells[] = {
	{
		.name	= "sun4i-a10-gpadc-iio",
		.resources = adc_resources,
		.num_resources = ARRAY_SIZE(adc_resources),
	}, {
		.name = "iio_hwmon",
	}
};

static struct mfd_cell sun5i_gpadc_mfd_cells[] = {
	{
		.name	= "sun5i-a13-gpadc-iio",
		.resources = adc_resources,
		.num_resources = ARRAY_SIZE(adc_resources),
	}, {
		.name = "iio_hwmon",
	},
};

static struct mfd_cell sun6i_gpadc_mfd_cells[] = {
	{
		.name	= "sun6i-a31-gpadc-iio",
		.resources = adc_resources,
		.num_resources = ARRAY_SIZE(adc_resources),
	}, {
		.name = "iio_hwmon",
	},
};

static const struct regmap_config sun4i_gpadc_mfd_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
};

static const struct of_device_id sun4i_gpadc_mfd_of_match[] = {
	{
		.compatible = "allwinner,sun4i-a10-ts",
		.data = &sun4i_gpadc_mfd_cells,
	}, {
		.compatible = "allwinner,sun5i-a13-ts",
		.data = &sun5i_gpadc_mfd_cells,
	}, {
		.compatible = "allwinner,sun6i-a31-ts",
		.data = &sun6i_gpadc_mfd_cells,
	}, { /* sentinel */ }
};

static int sun4i_gpadc_mfd_probe(struct platform_device *pdev)
{
	struct sun4i_gpadc_mfd_dev *mfd_dev;
	struct resource *mem;
	const struct of_device_id *of_id;
	const struct mfd_cell *mfd_cells;
	int ret;

	of_id = of_match_node(sun4i_gpadc_mfd_of_match, pdev->dev.of_node);
	if (!of_id)
		return -EINVAL;

	mfd_dev = devm_kzalloc(&pdev->dev, sizeof(*mfd_dev), GFP_KERNEL);
	if (!mfd_dev)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		printk("mem NULL\n");
	mfd_dev->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(mfd_dev->regs))
		return PTR_ERR(mfd_dev->regs);

	mfd_dev->dev = &pdev->dev;
	dev_set_drvdata(mfd_dev->dev, mfd_dev);

	mfd_dev->regmap = devm_regmap_init_mmio(mfd_dev->dev, mfd_dev->regs,
						&sun4i_gpadc_mfd_regmap_config);
	if (IS_ERR(mfd_dev->regmap)) {
		ret = PTR_ERR(mfd_dev->regmap);
		dev_err(&pdev->dev, "failed to init regmap: %d\n", ret);
		return ret;
	}

	/* Disable all interrupts */
	regmap_write(mfd_dev->regmap, SUN4I_GPADC_INT_FIFOC, 0);

	mfd_dev->irq = platform_get_irq(pdev, 0);
	ret = regmap_add_irq_chip(mfd_dev->regmap, mfd_dev->irq, IRQF_ONESHOT,
				  0, &sun4i_gpadc_mfd_regmap_irq_chip,
				  &mfd_dev->regmap_irqc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add irq chip: %d\n", ret);
		return ret;
	}

	mfd_cells = of_id->data;
	ret = mfd_add_devices(mfd_dev->dev, 0, mfd_cells, 2, NULL, 0, NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to add MFD devices: %d\n", ret);
		goto err;
	}

	printk("probe\n");

	return 0;
err:
	regmap_del_irq_chip(mfd_dev->irq, mfd_dev->regmap_irqc);

	return ret;
}

static int sun4i_gpadc_mfd_remove(struct platform_device *pdev)
{
	struct sun4i_gpadc_mfd_dev *mfd_dev;

	mfd_remove_devices(&pdev->dev);
	mfd_dev = dev_get_drvdata(&pdev->dev);
	regmap_del_irq_chip(mfd_dev->irq, mfd_dev->regmap_irqc);

	return 0;
}

MODULE_DEVICE_TABLE(of, sun4i_gpadc_mfd_of_match);

static struct platform_driver sun4i_gpadc_mfd_driver = {
	.driver = {
		.name = "sun4i-adc-mfd",
		.of_match_table = of_match_ptr(sun4i_gpadc_mfd_of_match),
	},
	.probe = sun4i_gpadc_mfd_probe,
	.remove = sun4i_gpadc_mfd_remove,
};

module_platform_driver(sun4i_gpadc_mfd_driver);

MODULE_DESCRIPTION("Allwinner sunxi platforms' GPADC MFD core driver");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL v2");
