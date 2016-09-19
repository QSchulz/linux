/* ADC driver for AXP20X PMICs
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 *
 */

#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include <linux/mfd/axp20x.h>

#define AXP20X_ADC_CHANNEL(_channel, _name, _type, _reg) {	\
	.type = _type,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.address = _reg,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.datasheet_name = _name,				\
}

struct axp20x_adc_iio {
	struct iio_dev		*indio_dev;
	struct regmap		*regmap;
};

enum adc_channel {
	ACIN_V = 0,
	ACIN_I,
	VBUS_V,
	VBUS_I,
	TEMP_ADC,
	TS_IN,
	GPIO0_V,
	GPIO1_V,
	PWR_BATT,
	BATT_V,
	BATT_CHRG_I,
	BATT_DISCHRG_I,
	IPSOUT_V,
};

static const struct iio_chan_spec axp20x_adc_channels[] = {
	AXP20X_ADC_CHANNEL(ACIN_V, "acin_v", IIO_VOLTAGE, AXP20X_ACIN_V_ADC_H),
	AXP20X_ADC_CHANNEL(ACIN_I, "acin_i", IIO_CURRENT, AXP20X_ACIN_I_ADC_H),
	AXP20X_ADC_CHANNEL(VBUS_V, "vbus_v", IIO_VOLTAGE, AXP20X_VBUS_V_ADC_H),
	AXP20X_ADC_CHANNEL(VBUS_I, "vbus_i", IIO_CURRENT, AXP20X_VBUS_I_ADC_H),
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = TEMP_ADC,
		.address = AXP20X_TEMP_ADC_H,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.datasheet_name = "temp_adc",
	},
	AXP20X_ADC_CHANNEL(TS_IN, "ts_in", IIO_VOLTAGE, AXP20X_TS_IN_H),
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = GPIO0_V,
		.address = AXP20X_GPIO0_V_ADC_H,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.datasheet_name = "gpio0_v",
	}, {
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = GPIO1_V,
		.address = AXP20X_GPIO1_V_ADC_H,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.datasheet_name = "gpio1_v",
	},
	// What type for PWR_BATT?
	AXP20X_ADC_CHANNEL(PWR_BATT, "pwr_batt", IIO_VOLTAGE, AXP20X_PWR_BATT_H),
	AXP20X_ADC_CHANNEL(BATT_V, "batt_v", IIO_VOLTAGE, AXP20X_BATT_V_H),
	AXP20X_ADC_CHANNEL(BATT_CHRG_I, "batt_chrg_i", IIO_CURRENT, AXP20X_BATT_CHRG_I_H),
	AXP20X_ADC_CHANNEL(BATT_DISCHRG_I, "batt_dischrg_i", IIO_CURRENT, AXP20X_BATT_DISCHRG_I_H),
	AXP20X_ADC_CHANNEL(IPSOUT_V, "ipsout_v", IIO_VOLTAGE, AXP20X_IPSOUT_V_HIGH_H),
};

static int axp20x_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *channel, int *val,
			       int *val2)
{
	struct axp20x_adc_iio *info = iio_priv(indio_dev);
	int size = 12, ret;

	switch(channel->channel) {
	case BATT_DISCHRG_I:
		size = 13;
	case ACIN_V:
	case ACIN_I:
	case VBUS_V:
	case VBUS_I:
	case TEMP_ADC:
	case TS_IN:
	case BATT_V:
	case BATT_CHRG_I:
	case IPSOUT_V:
		ret = axp20x_read_variable_width(info->regmap, channel->address,
						 size);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
/*	case GPIO0_V:
	case GPIO1_V:
*/
	case PWR_BATT:
		ret = regmap_bulk_read(info->regmap, channel->address, val, 3);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp20x_adc_scale(int channel, int *val, int *val2)
{
	switch(channel) {
	case ACIN_V:
	case VBUS_V:
		*val = 1;
		*val2 = 700000;
		return IIO_VAL_INT_PLUS_MICRO;

	case ACIN_I:
		*val = 0;
		*val2 = 625000;
		return IIO_VAL_INT_PLUS_MICRO;

	case VBUS_I:
		*val = 0;
		*val2 = 375000;
		return IIO_VAL_INT_PLUS_MICRO;

	case TEMP_ADC:
		*val = 100;
		return IIO_VAL_INT;

	case TS_IN:
		*val = 0;
		*val2 = 800000;
		return IIO_VAL_INT_PLUS_MICRO;
/*
	case GPIO0_V:
	case GPIO1_V:
		*val = 0;
		*val2 = 500000;
		return IIO_VAL_INT_PLUS_MICRO;
*/
	case PWR_BATT:
		*val = 0;
		*val2 = 550;
		return IIO_VAL_INT_PLUS_MICRO;

	case BATT_V:
		*val = 1;
		*val2 = 100000;
		return IIO_VAL_INT_PLUS_MICRO;

	case BATT_DISCHRG_I:
	case BATT_CHRG_I:
		*val = 0;
		*val2 = 500000;
		return IIO_VAL_INT_PLUS_MICRO;

	case IPSOUT_V:
		*val = 1;
		*val2 = 400000;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp20x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		//FIXME: GPIO offset if register set
		*val = -1447;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		return axp20x_adc_scale(chan->channel, val, val2);

	case IIO_CHAN_INFO_RAW:
		return axp20x_adc_read_raw(indio_dev, chan, val, val2);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static const struct iio_info axp20x_adc_iio_info = {
	.read_raw = axp20x_read_raw,
	.driver_module = THIS_MODULE,
};

static int axp20x_probe(struct platform_device *pdev)
{
	struct axp20x_adc_iio *info;
	struct iio_dev *indio_dev;
	int ret;
	struct axp20x_dev *axp20x_dev;

	axp20x_dev = dev_get_drvdata(pdev->dev.parent);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);
	platform_set_drvdata(pdev, indio_dev);

	info->regmap = axp20x_dev->regmap;
	info->indio_dev = indio_dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &axp20x_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = ARRAY_SIZE(axp20x_adc_channels);
	indio_dev->channels = axp20x_adc_channels;

	/* Enable the ADCs on IP */
	regmap_write(info->regmap, AXP20X_ADC_EN1, AXP20X_ADC_EN1_MASK);

	/*Configure ADCs rate*/
	regmap_update_bits(info->regmap, AXP20X_ADC_RATE, AXP20X_ADC_RATE_MASK,
			   AXP20X_ADC_RATE_50HZ);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register the device\n");
		goto err;
	}

	return 0;

err:
	regmap_write(info->regmap, AXP20X_ADC_EN1, ~AXP20X_ADC_EN1_MASK);

	return ret;
}

static int axp20x_remove(struct platform_device *pdev)
{
	struct axp20x_adc_iio *info;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	regmap_write(info->regmap, AXP20X_ADC_EN1, ~AXP20X_ADC_EN1_MASK);
	info = iio_priv(indio_dev);
	iio_device_unregister(indio_dev);

	return 0;
}

static const struct platform_device_id axp20x_adc_id[] = {
	{ "x-powers,axp20x-adc", },
	{ /* sentinel */ },
};

static struct platform_driver axp20x_adc_driver = {
	.driver = {
		.name = "axp20x_adc",
	},
	.id_table = axp20x_adc_id,
	.probe = axp20x_probe,
	.remove = axp20x_remove,
};

module_platform_driver(axp20x_adc_driver);

MODULE_DESCRIPTION("ADC driver for AXP20X PMICs");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPLv2");
