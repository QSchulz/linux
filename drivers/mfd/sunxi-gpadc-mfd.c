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

#include <linux/mfd/sunxi-gpadc-mfd.h>

#define SUNXI_IRQ_FIFO_DATA	0
#define SUNXI_IRQ_TEMP_DATA	1
#define SUNXI_IRQ_TP_UP		2


static struct resource adc_resources[] = {
	{
		.name	= "FIFO_DATA_PENDING",
		.start	= SUNXI_IRQ_FIFO_DATA,
		.end	= SUNXI_IRQ_FIFO_DATA,
		.flags	= IORESOURCE_IRQ,
	}, {
		.name	= "TEMP_DATA_PENDING",
		.start	= SUNXI_IRQ_TEMP_DATA,
		.end	= SUNXI_IRQ_TEMP_DATA,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ts_resources[] = {
	{
		.name	= "TP_UP_PENDING",
		.start	= SUNXI_IRQ_TP_UP,
		.end	= SUNXI_IRQ_TP_UP,
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct regmap_irq sunxi_gpadc_mfd_regmap_irq[] = {
	REGMAP_IRQ_REG(SUNXI_IRQ_FIFO_DATA, 0, BIT(16)),
	REGMAP_IRQ_REG(SUNXI_IRQ_TEMP_DATA, 0, BIT(18)),
	REGMAP_IRQ_REG(SUNXI_IRQ_TP_UP, 0, BIT(1)),
};

static const struct regmap_irq_chip sunxi_gpadc_mfd_regmap_irq_chip = {
	.name = "sunxi_gpadc_mfd_irq_chip",
	.status_base = SUNXI_GPADC_TP_INT_FIFOS,
	.ack_base = SUNXI_GPADC_TP_INT_FIFOS,
	.mask_base = SUNXI_GPADC_TP_INT_FIFOC,
	.init_ack_masked = true,
	.mask_invert = true,
	.irqs = sunxi_gpadc_mfd_regmap_irq,
	.num_irqs = ARRAY_SIZE(sunxi_gpadc_mfd_regmap_irq),
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

static const struct regmap_config sunxi_gpadc_mfd_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
};

static int sunxi_gpadc_mfd_probe(struct platform_device *pdev)
{
	struct sunxi_gpadc_mfd_dev *sunxi_gpadc_mfd_dev = NULL;
	struct resource *mem = NULL;
	unsigned int irq;
	int ret;

	sunxi_gpadc_mfd_dev = devm_kzalloc(&pdev->dev,
					   sizeof(*sunxi_gpadc_mfd_dev),
					   GFP_KERNEL);
	if (!sunxi_gpadc_mfd_dev)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sunxi_gpadc_mfd_dev->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(sunxi_gpadc_mfd_dev->regs))
		return PTR_ERR(sunxi_gpadc_mfd_dev->regs);

	sunxi_gpadc_mfd_dev->dev = &pdev->dev;
	dev_set_drvdata(sunxi_gpadc_mfd_dev->dev, sunxi_gpadc_mfd_dev);

	sunxi_gpadc_mfd_dev->regmap =
		devm_regmap_init_mmio(sunxi_gpadc_mfd_dev->dev,
				      sunxi_gpadc_mfd_dev->regs,
				      &sunxi_gpadc_mfd_regmap_config);
	if (IS_ERR(sunxi_gpadc_mfd_dev->regmap)) {
		ret = PTR_ERR(sunxi_gpadc_mfd_dev->regmap);
		dev_err(&pdev->dev, "failed to init regmap: %d\n", ret);
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	ret = regmap_add_irq_chip(sunxi_gpadc_mfd_dev->regmap, irq,
				  IRQF_ONESHOT, 0,
				  &sunxi_gpadc_mfd_regmap_irq_chip,
				  &sunxi_gpadc_mfd_dev->regmap_irqc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add irq chip: %d\n", ret);
		return ret;
	}

	if (of_device_is_compatible(pdev->dev.of_node,
				    "allwinner,sun4i-a10-ts"))
		ret = mfd_add_devices(sunxi_gpadc_mfd_dev->dev, 0,
				      sun4i_gpadc_mfd_cells,
				      ARRAY_SIZE(sun4i_gpadc_mfd_cells), NULL,
				      0, NULL);
	else if (of_device_is_compatible(pdev->dev.of_node,
					 "allwinner,sun5i-a13-ts"))
		ret = mfd_add_devices(sunxi_gpadc_mfd_dev->dev, 0,
				      sun5i_gpadc_mfd_cells,
				      ARRAY_SIZE(sun5i_gpadc_mfd_cells), NULL,
				      0, NULL);
	else if (of_device_is_compatible(pdev->dev.of_node,
					 "allwinner,sun6i-a31-ts"))
		ret = mfd_add_devices(sunxi_gpadc_mfd_dev->dev, 0,
				      sun6i_gpadc_mfd_cells,
				      ARRAY_SIZE(sun6i_gpadc_mfd_cells), NULL,
				      0, NULL);

	if (ret) {
		dev_err(&pdev->dev, "failed to add MFD devices: %d\n", ret);
		regmap_del_irq_chip(irq, sunxi_gpadc_mfd_dev->regmap_irqc);
		return ret;
	}

	dev_info(&pdev->dev, "successfully loaded\n");

	return 0;
}

static int sunxi_gpadc_mfd_remove(struct platform_device *pdev)
{
	struct sunxi_gpadc_mfd_dev *sunxi_gpadc_mfd_dev;
	unsigned int irq;

	irq = platform_get_irq(pdev, 0);
	mfd_remove_devices(&pdev->dev);
	sunxi_gpadc_mfd_dev = dev_get_drvdata(&pdev->dev);
	regmap_del_irq_chip(irq, sunxi_gpadc_mfd_dev->regmap_irqc);

	return 0;
}

static const struct of_device_id sunxi_gpadc_mfd_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-ts" },
	{ .compatible = "allwinner,sun5i-a13-ts" },
	{ .compatible = "allwinner,sun6i-a31-ts" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, sunxi_gpadc_mfd_of_match);

static struct platform_driver sunxi_gpadc_mfd_driver = {
	.driver = {
		.name = "sunxi-adc-mfd",
		.of_match_table = of_match_ptr(sunxi_gpadc_mfd_of_match),
	},
	.probe = sunxi_gpadc_mfd_probe,
	.remove = sunxi_gpadc_mfd_remove,
};

module_platform_driver(sunxi_gpadc_mfd_driver);

MODULE_DESCRIPTION("ADC MFD core driver for sunxi platforms");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL v2");
