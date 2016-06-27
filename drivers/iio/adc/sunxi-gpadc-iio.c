/* ADC driver for sunxi platforms' (A10, A13 and A31) GPADC
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 *
 * The Allwinner SoCs all have an ADC that can also act as a touchscreen
 * controller and a thermal sensor.
 * The thermal sensor works only when the ADC acts as a touchscreen controller
 * and is configured to throw an interrupt every fixed periods of time (let say
 * every X seconds).
 * One would be tempted to disable the IP on the hardware side rather than
 * disabling interrupts to save some power but that reset the internal clock of
 * the IP, resulting in having to wait X seconds every time we want to read the
 * value of the thermal sensor.
 * This is also the reason of using autosuspend in pm_runtime. If there were no
 * autosuspend, the thermal sensor would need X seconds after every
 * pm_runtime_get_sync to get a value from the ADC. The autosuspend allows the
 * thermal sensor to be requested again in a certain time span before it gets
 * shutdown for not being used.
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
#include <linux/mfd/sunxi-gpadc-mfd.h>

const unsigned int sun4i_gpadc_chan_select(unsigned int chan)
{
	return SUNXI_GPADC_TP_CTRL1_ADC_CHAN_SELECT(chan);
}

const unsigned int sun6i_gpadc_chan_select(unsigned int chan)
{
	return SUNXI_GPADC_TP_CTRL1_SUN6I_ADC_CHAN_SELECT(chan);
}

struct sunxi_gpadc_soc_specific {
	const int		temp_offset;
	const int		temp_scale;
	const unsigned int	tp_mode_en;
	const unsigned int	tp_adc_select;
	const unsigned int	(*adc_chan_select)(unsigned int chan);
};

static const struct sunxi_gpadc_soc_specific sun4i_gpadc_soc_specific = {
	.temp_offset = -1932,
	.temp_scale = 133,
	.tp_mode_en = SUNXI_GPADC_TP_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUNXI_GPADC_TP_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
};

static const struct sunxi_gpadc_soc_specific sun5i_gpadc_soc_specific = {
	.temp_offset = -1447,
	.temp_scale = 100,
	.tp_mode_en = SUNXI_GPADC_TP_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUNXI_GPADC_TP_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
};

static const struct sunxi_gpadc_soc_specific sun6i_gpadc_soc_specific = {
	.temp_offset = -1623,
	.temp_scale = 167,
	.tp_mode_en = SUNXI_GPADC_TP_CTRL1_SUN6I_TP_MODE_EN,
	.tp_adc_select = SUNXI_GPADC_TP_CTRL1_SUN6I_TP_ADC_SELECT,
	.adc_chan_select = &sun6i_gpadc_chan_select,
};

struct sunxi_gpadc_dev {
	struct iio_dev				*indio_dev;
	void __iomem				*regs;
	struct completion			completion;
	int					temp_data;
	u32					adc_data;
	struct regmap				*regmap;
	unsigned int				fifo_data_irq;
	atomic_t				ignore_fifo_data_irq;
	unsigned int				temp_data_irq;
	atomic_t				ignore_temp_data_irq;
	const struct sunxi_gpadc_soc_specific	*soc_specific;
	struct mutex				mutex;
};

#define SUNXI_GPADC_ADC_CHANNEL(_channel, _name) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.datasheet_name = _name,				\
}

static struct iio_map sunxi_gpadc_hwmon_maps[] = {
	{
		.adc_channel_label = "temp_adc",
		.consumer_dev_name = "iio_hwmon.0",
	},
	{ /* sentinel */ },
};

static const struct iio_chan_spec sunxi_gpadc_channels[] = {
	SUNXI_GPADC_ADC_CHANNEL(0, "adc_chan0"),
	SUNXI_GPADC_ADC_CHANNEL(1, "adc_chan1"),
	SUNXI_GPADC_ADC_CHANNEL(2, "adc_chan2"),
	SUNXI_GPADC_ADC_CHANNEL(3, "adc_chan3"),
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.datasheet_name = "temp_adc",
	},
};

static int sunxi_gpadc_adc_read(struct iio_dev *indio_dev, int channel,
				int *val)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);
	int ret = 0;

	pm_runtime_get_sync(indio_dev->dev.parent);
	mutex_lock(&info->mutex);

	reinit_completion(&info->completion);
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
		     info->soc_specific->tp_mode_en |
		     info->soc_specific->tp_adc_select |
		     info->soc_specific->adc_chan_select(channel));
	regmap_write(info->regmap, SUNXI_GPADC_TP_INT_FIFOC,
		     SUNXI_GPADC_TP_INT_FIFOC_TP_FIFO_TRIG_LEVEL(1) |
		     SUNXI_GPADC_TP_INT_FIFOC_TP_FIFO_FLUSH);
	enable_irq(info->fifo_data_irq);

	if (!wait_for_completion_timeout(&info->completion,
					 msecs_to_jiffies(100))) {
		ret = -ETIMEDOUT;
		goto out;
	}

	*val = info->adc_data;

out:
	disable_irq(info->fifo_data_irq);
	mutex_unlock(&info->mutex);
	pm_runtime_mark_last_busy(indio_dev->dev.parent);
	pm_runtime_put_autosuspend(indio_dev->dev.parent);

	return ret;
}

static int sunxi_gpadc_temp_read(struct iio_dev *indio_dev, int *val)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);
	int ret = 0;

	pm_runtime_get_sync(indio_dev->dev.parent);
	mutex_lock(&info->mutex);

	reinit_completion(&info->completion);

	regmap_write(info->regmap, SUNXI_GPADC_TP_INT_FIFOC,
		     SUNXI_GPADC_TP_INT_FIFOC_TP_FIFO_TRIG_LEVEL(1) |
		     SUNXI_GPADC_TP_INT_FIFOC_TP_FIFO_FLUSH);
	/*
	 * The temperature sensor returns valid data only when the ADC operates
	 * in touchscreen mode.
	 */
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
		     info->soc_specific->tp_mode_en);
	enable_irq(info->temp_data_irq);

	if (!wait_for_completion_timeout(&info->completion,
					 msecs_to_jiffies(100))) {
		ret = -ETIMEDOUT;
		goto out;
	}

	*val = info->temp_data;

out:
	disable_irq(info->temp_data_irq);
	mutex_unlock(&info->mutex);
	pm_runtime_mark_last_busy(indio_dev->dev.parent);
	pm_runtime_put_autosuspend(indio_dev->dev.parent);

	return ret;
}

static int sunxi_gpadc_temp_offset(struct iio_dev *indio_dev, int *val)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);

	*val = info->soc_specific->temp_offset;

	return 0;
}

static int sunxi_gpadc_temp_scale(struct iio_dev *indio_dev, int *val)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);

	*val = info->soc_specific->temp_scale;

	return 0;
}

static int sunxi_gpadc_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		ret = sunxi_gpadc_temp_offset(indio_dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_VOLTAGE) {
			ret = sunxi_gpadc_adc_read(indio_dev, chan->channel,
						   val);
			if (ret)
				return ret;
		} else {
			ret = sunxi_gpadc_temp_read(indio_dev, val);
			if (ret)
				return ret;
		}

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = sunxi_gpadc_temp_scale(indio_dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static const struct iio_info sunxi_gpadc_iio_info = {
	.read_raw = sunxi_gpadc_read_raw,
	.driver_module = THIS_MODULE,
};

static irqreturn_t sunxi_gpadc_temp_data_irq_handler(int irq, void *dev_id)
{
	struct sunxi_gpadc_dev *info = dev_id;

	if (atomic_read(&info->ignore_temp_data_irq))
		return IRQ_HANDLED;

	if (!regmap_read(info->regmap, SUNXI_GPADC_TEMP_DATA, &info->temp_data))
		complete(&info->completion);

	return IRQ_HANDLED;
}

static irqreturn_t sunxi_gpadc_fifo_data_irq_handler(int irq, void *dev_id)
{
	struct sunxi_gpadc_dev *info = dev_id;

	if (atomic_read(&info->ignore_fifo_data_irq))
		return IRQ_HANDLED;

	if (!regmap_read(info->regmap, SUNXI_GPADC_TP_DATA, &info->adc_data))
		complete(&info->completion);

	return IRQ_HANDLED;
}

static int sunxi_gpadc_runtime_suspend(struct device *dev)
{
	struct sunxi_gpadc_dev *info = iio_priv(dev_get_drvdata(dev));

	mutex_lock(&info->mutex);

	/* Disable the ADC on IP */
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1, 0);
	/* Disable temperature sensor on IP */
	regmap_write(info->regmap, SUNXI_GPADC_TP_TPR, 0);

	mutex_unlock(&info->mutex);

	return 0;
}

static int sunxi_gpadc_runtime_resume(struct device *dev)
{
	struct sunxi_gpadc_dev *info = iio_priv(dev_get_drvdata(dev));

	mutex_lock(&info->mutex);

	/* clkin = 6MHz */
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL0,
		     SUNXI_GPADC_TP_CTRL0_ADC_CLK_DIVIDER(2) |
		     SUNXI_GPADC_TP_CTRL0_FS_DIV(7) |
		     SUNXI_GPADC_TP_CTRL0_T_ACQ(63));
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
		     info->soc_specific->tp_mode_en);
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL3,
		     SUNXI_GPADC_TP_CTRL3_FILTER_EN |
		     SUNXI_GPADC_TP_CTRL3_FILTER_TYPE(1));
	/* period = SUNXI_GPADC_TP_TPR_TEMP_PERIOD * 256 * 16 / clkin; ~1.3s */
	regmap_write(info->regmap, SUNXI_GPADC_TP_TPR,
		     SUNXI_GPADC_TP_TPR_TEMP_ENABLE |
		     SUNXI_GPADC_TP_TPR_TEMP_PERIOD(1953));

	mutex_unlock(&info->mutex);

	return 0;
}

static int sunxi_gpadc_get_temp(void *data, int *temp)
{
	struct sunxi_gpadc_dev *info = (struct sunxi_gpadc_dev *)data;
	int val, scale, offset;

	/* If reading temperature times out, take stored previous value. */
	if (sunxi_gpadc_temp_read(info->indio_dev, &val))
		val = info->temp_data;
	sunxi_gpadc_temp_scale(info->indio_dev, &scale);
	sunxi_gpadc_temp_offset(info->indio_dev, &offset);

	*temp = (val + offset) * scale;

	return 0;
}

static const struct thermal_zone_of_device_ops sunxi_ts_tz_ops = {
	.get_temp = &sunxi_gpadc_get_temp,
};

static const struct dev_pm_ops sunxi_gpadc_pm_ops = {
	.runtime_suspend = &sunxi_gpadc_runtime_suspend,
	.runtime_resume = &sunxi_gpadc_runtime_resume,
};

static int sunxi_gpadc_probe(struct platform_device *pdev)
{
	struct sunxi_gpadc_dev *info;
	struct iio_dev *indio_dev;
	int ret, irq;
	struct sunxi_gpadc_mfd_dev *sunxi_gpadc_mfd_dev;
	struct thermal_zone_device *tzd;

	sunxi_gpadc_mfd_dev = dev_get_drvdata(pdev->dev.parent);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);

	mutex_init(&info->mutex);
	info->regmap = sunxi_gpadc_mfd_dev->regmap;
	info->indio_dev = indio_dev;
	atomic_set(&info->ignore_fifo_data_irq, 1);
	atomic_set(&info->ignore_temp_data_irq, 1);
	init_completion(&info->completion);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &sunxi_gpadc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = ARRAY_SIZE(sunxi_gpadc_channels);
	indio_dev->channels = sunxi_gpadc_channels;

	info->soc_specific = (struct sunxi_gpadc_soc_specific *)platform_get_device_id(pdev)->driver_data;

	tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, info,
						   &sunxi_ts_tz_ops);
	if (IS_ERR(tzd)) {
		dev_err(&pdev->dev, "could not register thermal sensor: %ld\n",
			PTR_ERR(tzd));
		return PTR_ERR(tzd);
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 SUNXI_GPADC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	irq = platform_get_irq_byname(pdev, "TEMP_DATA_PENDING");
	if (irq < 0) {
		dev_err(&pdev->dev,
			"no TEMP_DATA_PENDING interrupt registered\n");
		ret = irq;
		goto err;
	}

	irq = regmap_irq_get_virq(sunxi_gpadc_mfd_dev->regmap_irqc, irq);
	ret = devm_request_any_context_irq(&pdev->dev, irq,
					   sunxi_gpadc_temp_data_irq_handler, 0,
					   "temp_data", info);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"could not request TEMP_DATA_PENDING interrupt: %d\n",
			ret);
		goto err;
	}

	disable_irq(irq);
	info->temp_data_irq = irq;
	atomic_set(&info->ignore_temp_data_irq, 0);

	irq = platform_get_irq_byname(pdev, "FIFO_DATA_PENDING");
	if (irq < 0) {
		dev_err(&pdev->dev,
			"no FIFO_DATA_PENDING interrupt registered\n");
		ret = irq;
		goto err;
	}

	irq = regmap_irq_get_virq(sunxi_gpadc_mfd_dev->regmap_irqc, irq);
	ret = devm_request_any_context_irq(&pdev->dev, irq,
					   sunxi_gpadc_fifo_data_irq_handler, 0,
					   "fifo_data", info);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"could not request FIFO_DATA_PENDING interrupt: %d\n",
			ret);
		goto err;
	}

	disable_irq(irq);
	info->fifo_data_irq = irq;
	atomic_set(&info->ignore_fifo_data_irq, 0);

	ret = iio_map_array_register(indio_dev, sunxi_gpadc_hwmon_maps);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register iio map array\n");
		goto err;
	}

	platform_set_drvdata(pdev, indio_dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register the device\n");
		goto err_map;
	}

	return 0;

err_map:
	iio_map_array_unregister(indio_dev);

err:
	pm_runtime_put(&pdev->dev);
	/* Disable all hardware interrupts */
	regmap_write(info->regmap, SUNXI_GPADC_TP_INT_FIFOC, 0);

	return ret;
}

static int sunxi_gpadc_remove(struct platform_device *pdev)
{
	struct sunxi_gpadc_dev *info;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	info = iio_priv(indio_dev);
	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	/* Disable all hardware interrupts */
	regmap_write(info->regmap, SUNXI_GPADC_TP_INT_FIFOC, 0);

	return 0;
}

static const struct platform_device_id sunxi_gpadc_id[] = {
	{ "sun4i-a10-gpadc-iio", (kernel_ulong_t)&sun4i_gpadc_soc_specific },
	{ "sun5i-a13-gpadc-iio", (kernel_ulong_t)&sun5i_gpadc_soc_specific },
	{ "sun6i-a31-gpadc-iio", (kernel_ulong_t)&sun6i_gpadc_soc_specific },
	{ /* sentinel */ },
};

static struct platform_driver sunxi_gpadc_driver = {
	.driver = {
		.name = "sunxi-gpadc-iio",
		.pm = &sunxi_gpadc_pm_ops,
	},
	.id_table = sunxi_gpadc_id,
	.probe = sunxi_gpadc_probe,
	.remove = sunxi_gpadc_remove,
};

module_platform_driver(sunxi_gpadc_driver);

MODULE_DESCRIPTION("ADC driver for sunxi platforms");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL v2");
