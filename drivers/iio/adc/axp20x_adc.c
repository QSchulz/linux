/* ADC driver for AXP20X and AXP22X PMICs
 *
 * Copyright (c) 2016 Free Electrons NextThing Co.
 *	Quentin Schulz <quentin.schulz@free-electrons.com>
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
#include <linux/mfd/axp20x.h>

#define AXP20X_ADC_EN1_MASK			GENMASK(7, 0)

#define AXP20X_ADC_EN2_MASK			(GENMASK(3, 2) | BIT(7))
#define AXP22X_ADC_EN1_MASK			(GENMASK(7, 5) | BIT(0))
#define AXP20X_ADC_EN2_TEMP_ADC			BIT(7)
#define AXP20X_ADC_EN2_GPIO0_ADC		BIT(3)
#define AXP20X_ADC_EN2_GPIO1_ADC		BIT(2)

#define AXP20X_GPIO10_IN_RANGE_GPIO0		BIT(0)
#define AXP20X_GPIO10_IN_RANGE_GPIO1		BIT(1)
#define AXP20X_GPIO10_IN_RANGE_GPIO0_VAL(x)	((x) & BIT(0))
#define AXP20X_GPIO10_IN_RANGE_GPIO1_VAL(x)	(((x) & BIT(0)) << 1)

#define AXP20X_ADC_RATE_MASK			(3 << 6)
#define AXP20X_ADC_RATE_25HZ			(0 << 6)
#define AXP20X_ADC_RATE_50HZ			BIT(6)
#define AXP20X_ADC_RATE_100HZ			(2 << 6)
#define AXP20X_ADC_RATE_200HZ			(3 << 6)

#define AXP22X_ADC_RATE_100HZ			(0 << 6)
#define AXP22X_ADC_RATE_200HZ			BIT(6)
#define AXP22X_ADC_RATE_400HZ			(2 << 6)
#define AXP22X_ADC_RATE_800HZ			(3 << 6)

#define AXP20X_ADC_CHANNEL(_channel, _name, _type, _reg)	\
	{							\
		.type = _type,					\
		.indexed = 1,					\
		.channel = _channel,				\
		.address = _reg,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
				      BIT(IIO_CHAN_INFO_SCALE),	\
		.datasheet_name = _name,			\
	}

#define AXP20X_ADC_CHANNEL_OFFSET(_channel, _name, _type, _reg) \
	{							\
		.type = _type,					\
		.indexed = 1,					\
		.channel = _channel,				\
		.address = _reg,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
				      BIT(IIO_CHAN_INFO_SCALE) |\
				      BIT(IIO_CHAN_INFO_OFFSET),\
		.datasheet_name = _name,			\
	}

struct axp20x_adc_iio {
	struct iio_dev		*indio_dev;
	struct regmap		*regmap;
};

enum axp20x_adc_channel {
	AXP20X_ACIN_V = 0,
	AXP20X_ACIN_I,
	AXP20X_VBUS_V,
	AXP20X_VBUS_I,
	AXP20X_TEMP_ADC,
	AXP20X_GPIO0_V,
	AXP20X_GPIO1_V,
	AXP20X_BATT_V,
	AXP20X_BATT_CHRG_I,
	AXP20X_BATT_DISCHRG_I,
	AXP20X_IPSOUT_V,
};

enum axp22x_adc_channel {
	AXP22X_TEMP_ADC = 0,
	AXP22X_BATT_V,
	AXP22X_BATT_CHRG_I,
	AXP22X_BATT_DISCHRG_I,
};

static const struct iio_chan_spec axp20x_adc_channels[] = {
	AXP20X_ADC_CHANNEL(AXP20X_ACIN_V, "acin_v", IIO_VOLTAGE,
			   AXP20X_ACIN_V_ADC_H),
	AXP20X_ADC_CHANNEL(AXP20X_ACIN_I, "acin_i", IIO_CURRENT,
			   AXP20X_ACIN_I_ADC_H),
	AXP20X_ADC_CHANNEL(AXP20X_VBUS_V, "vbus_v", IIO_VOLTAGE,
			   AXP20X_VBUS_V_ADC_H),
	AXP20X_ADC_CHANNEL(AXP20X_VBUS_I, "vbus_i", IIO_CURRENT,
			   AXP20X_VBUS_I_ADC_H),
	AXP20X_ADC_CHANNEL_OFFSET(AXP20X_TEMP_ADC, "temp_adc", IIO_TEMP,
				  AXP20X_TEMP_ADC_H),
	AXP20X_ADC_CHANNEL_OFFSET(AXP20X_GPIO0_V, "gpio0_v", IIO_VOLTAGE,
				  AXP20X_GPIO0_V_ADC_H),
	AXP20X_ADC_CHANNEL_OFFSET(AXP20X_GPIO1_V, "gpio1_v", IIO_VOLTAGE,
				  AXP20X_GPIO1_V_ADC_H),
	AXP20X_ADC_CHANNEL(AXP20X_BATT_V, "batt_v", IIO_VOLTAGE,
			   AXP20X_BATT_V_H),
	AXP20X_ADC_CHANNEL(AXP20X_BATT_CHRG_I, "batt_chrg_i", IIO_CURRENT,
			   AXP20X_BATT_CHRG_I_H),
	AXP20X_ADC_CHANNEL(AXP20X_BATT_DISCHRG_I, "batt_dischrg_i", IIO_CURRENT,
			   AXP20X_BATT_DISCHRG_I_H),
	AXP20X_ADC_CHANNEL(AXP20X_IPSOUT_V, "ipsout_v", IIO_VOLTAGE,
			   AXP20X_IPSOUT_V_HIGH_H),
};

static const struct iio_chan_spec axp22x_adc_channels[] = {
	AXP20X_ADC_CHANNEL_OFFSET(AXP22X_TEMP_ADC, "temp_adc", IIO_TEMP,
				  AXP22X_TEMP_ADC_H),
	AXP20X_ADC_CHANNEL(AXP22X_BATT_V, "batt_v", IIO_VOLTAGE,
			   AXP20X_BATT_V_H),
	AXP20X_ADC_CHANNEL(AXP22X_BATT_CHRG_I, "batt_chrg_i", IIO_CURRENT,
			   AXP20X_BATT_CHRG_I_H),
	AXP20X_ADC_CHANNEL(AXP22X_BATT_DISCHRG_I, "batt_dischrg_i", IIO_CURRENT,
			   AXP20X_BATT_DISCHRG_I_H),
};

static int axp20x_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *channel, int *val,
			       int *val2)
{
	struct axp20x_adc_iio *info = iio_priv(indio_dev);
	int size = 12, ret;

	switch (channel->channel) {
	case AXP20X_BATT_DISCHRG_I:
		size = 13;
	case AXP20X_ACIN_V:
	case AXP20X_ACIN_I:
	case AXP20X_VBUS_V:
	case AXP20X_VBUS_I:
	case AXP20X_TEMP_ADC:
	case AXP20X_BATT_V:
	case AXP20X_BATT_CHRG_I:
	case AXP20X_IPSOUT_V:
	case AXP20X_GPIO0_V:
	case AXP20X_GPIO1_V:
		ret = axp20x_read_variable_width(info->regmap, channel->address,
						 size);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp22x_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *channel, int *val,
			       int *val2)
{
	struct axp20x_adc_iio *info = iio_priv(indio_dev);
	int size = 12, ret;

	switch (channel->channel) {
	case AXP22X_BATT_DISCHRG_I:
		size = 13;
	case AXP22X_TEMP_ADC:
	case AXP22X_BATT_V:
	case AXP22X_BATT_CHRG_I:
		ret = axp20x_read_variable_width(info->regmap, channel->address,
						 size);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp20x_adc_scale(int channel, int *val, int *val2)
{
	switch (channel) {
	case AXP20X_ACIN_V:
	case AXP20X_VBUS_V:
		*val = 1;
		*val2 = 700000;
		return IIO_VAL_INT_PLUS_MICRO;

	case AXP20X_ACIN_I:
		*val = 0;
		*val2 = 625000;
		return IIO_VAL_INT_PLUS_MICRO;

	case AXP20X_VBUS_I:
		*val = 0;
		*val2 = 375000;
		return IIO_VAL_INT_PLUS_MICRO;

	case AXP20X_TEMP_ADC:
		*val = 100;
		return IIO_VAL_INT;

	case AXP20X_GPIO0_V:
	case AXP20X_GPIO1_V:
		*val = 0;
		*val2 = 500000;
		return IIO_VAL_INT_PLUS_MICRO;

	case AXP20X_BATT_V:
		*val = 1;
		*val2 = 100000;
		return IIO_VAL_INT_PLUS_MICRO;

	case AXP20X_BATT_DISCHRG_I:
	case AXP20X_BATT_CHRG_I:
		*val = 0;
		*val2 = 500000;
		return IIO_VAL_INT_PLUS_MICRO;

	case AXP20X_IPSOUT_V:
		*val = 1;
		*val2 = 400000;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp22x_adc_scale(int channel, int *val, int *val2)
{
	switch (channel) {
	case AXP22X_TEMP_ADC:
		*val = 100;
		return IIO_VAL_INT;

	case AXP22X_BATT_V:
		*val = 1;
		*val2 = 100000;
		return IIO_VAL_INT_PLUS_MICRO;

	case AXP22X_BATT_DISCHRG_I:
	case AXP22X_BATT_CHRG_I:
		*val = 0;
		*val2 = 500000;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp20x_adc_offset(struct iio_dev *indio_dev, int channel, int *val)
{
	struct axp20x_adc_iio *info = iio_priv(indio_dev);
	int ret, reg;

	switch (channel) {
	case AXP20X_TEMP_ADC:
		*val = -1447;
		return IIO_VAL_INT;

	case AXP20X_GPIO0_V:
	case AXP20X_GPIO1_V:
		ret = regmap_read(info->regmap, AXP20X_GPIO10_IN_RANGE, &reg);
		if (ret < 0)
			return ret;

		if (channel == AXP20X_GPIO0_V)
			*val = reg & AXP20X_GPIO10_IN_RANGE_GPIO0;
		else
			*val = reg & AXP20X_GPIO10_IN_RANGE_GPIO1;

		*val = !!(*val) * 700000;

		return IIO_VAL_INT;

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
		return axp20x_adc_offset(indio_dev, chan->channel, val);

	case IIO_CHAN_INFO_SCALE:
		return axp20x_adc_scale(chan->channel, val, val2);

	case IIO_CHAN_INFO_RAW:
		return axp20x_adc_read_raw(indio_dev, chan, val, val2);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp22x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		*val = -2667;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		return axp22x_adc_scale(chan->channel, val, val2);

	case IIO_CHAN_INFO_RAW:
		return axp22x_adc_read_raw(indio_dev, chan, val, val2);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int axp20x_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	struct axp20x_adc_iio *info = iio_priv(indio_dev);

	/*
	 * The AXP20X PMIC allows the user to choose between 0V and 0.7V offsets
	 * for (independently) GPIO0 and GPIO1 when in ADC mode.
	 */
	if (mask != IIO_CHAN_INFO_OFFSET)
		return -EINVAL;

	if (chan->channel != AXP20X_GPIO0_V && chan->channel != AXP20X_GPIO1_V)
		return -EINVAL;

	if (val != 0 && val != 700000)
		return -EINVAL;

	if (chan->channel == AXP20X_GPIO0_V)
		return regmap_update_bits(info->regmap, AXP20X_GPIO10_IN_RANGE,
					  AXP20X_GPIO10_IN_RANGE_GPIO0,
					  AXP20X_GPIO10_IN_RANGE_GPIO0_VAL(!!val));

	return regmap_update_bits(info->regmap, AXP20X_GPIO10_IN_RANGE,
				  AXP20X_GPIO10_IN_RANGE_GPIO1,
				  AXP20X_GPIO10_IN_RANGE_GPIO1_VAL(!!val));
}

static const struct iio_info axp20x_adc_iio_info = {
	.read_raw = axp20x_read_raw,
	.write_raw = axp20x_write_raw,
	.driver_module = THIS_MODULE,
};

static const struct iio_info axp22x_adc_iio_info = {
	.read_raw = axp22x_read_raw,
	.driver_module = THIS_MODULE,
};

static const struct of_device_id axp20x_adc_of_match[] = {
	{ .compatible = "x-powers,axp209-adc", .data = (void *)AXP209_ID, },
	{ .compatible = "x-powers,axp221-adc", .data = (void *)AXP221_ID, },
	{ /* sentinel */ },
};

static int axp20x_probe(struct platform_device *pdev)
{
	struct axp20x_adc_iio *info;
	struct iio_dev *indio_dev;
	struct axp20x_dev *axp20x_dev;
	int ret, axp20x_id;

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
	indio_dev->modes = INDIO_DIRECT_MODE;

	axp20x_id = (int)of_device_get_match_data(&pdev->dev);

	switch (axp20x_id) {
	case AXP209_ID:
		indio_dev->info = &axp20x_adc_iio_info;
		indio_dev->num_channels = ARRAY_SIZE(axp20x_adc_channels);
		indio_dev->channels = axp20x_adc_channels;

		/* Enable the ADCs on IP */
		regmap_write(info->regmap, AXP20X_ADC_EN1, AXP20X_ADC_EN1_MASK);

		/* Enable GPIO0/1 and internal temperature ADCs */
		regmap_update_bits(info->regmap, AXP20X_ADC_EN2,
				   AXP20X_ADC_EN2_MASK, AXP20X_ADC_EN2_MASK);

		/* Configure ADCs rate */
		regmap_update_bits(info->regmap, AXP20X_ADC_RATE,
				   AXP20X_ADC_RATE_MASK, AXP20X_ADC_RATE_50HZ);
		break;

	case AXP221_ID:
		indio_dev->info = &axp22x_adc_iio_info;
		indio_dev->num_channels = ARRAY_SIZE(axp22x_adc_channels);
		indio_dev->channels = axp22x_adc_channels;

		/* Enable the ADCs on IP */
		regmap_write(info->regmap, AXP20X_ADC_EN1, AXP22X_ADC_EN1_MASK);

		/* Configure ADCs rate */
		regmap_update_bits(info->regmap, AXP20X_ADC_RATE,
				   AXP20X_ADC_RATE_MASK, AXP22X_ADC_RATE_200HZ);
		break;

	default:
		return -EINVAL;
	}

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register the device\n");
		regmap_write(info->regmap, AXP20X_ADC_EN1, 0);
		regmap_write(info->regmap, AXP20X_ADC_EN2, 0);
		return ret;
	}

	return 0;
}

static int axp20x_remove(struct platform_device *pdev)
{
	struct axp20x_adc_iio *info;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	info = iio_priv(indio_dev);
	regmap_write(info->regmap, AXP20X_ADC_EN1, 0);
	regmap_write(info->regmap, AXP20X_ADC_EN2, 0);

	return 0;
}

static struct platform_driver axp20x_adc_driver = {
	.driver = {
		.name = "axp20x-adc",
		.of_match_table = axp20x_adc_of_match,
	},
	.probe = axp20x_probe,
	.remove = axp20x_remove,
};

module_platform_driver(axp20x_adc_driver);

MODULE_DESCRIPTION("ADC driver for AXP20X and AXP22X PMICs");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL");
