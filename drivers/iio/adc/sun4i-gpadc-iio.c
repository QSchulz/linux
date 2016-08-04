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
#include <linux/mfd/sun4i-gpadc-mfd.h>

const unsigned int sun4i_gpadc_chan_select(unsigned int chan)
{
	return SUN4I_GPADC_CTRL1_ADC_CHAN_SELECT(chan);
}

const unsigned int sun6i_gpadc_chan_select(unsigned int chan)
{
	return SUN6I_GPADC_CTRL1_ADC_CHAN_SELECT(chan);
}

struct soc_specific {
	const int		temp_offset;
	const int		temp_scale;
	const unsigned int	tp_mode_en;
	const unsigned int	tp_adc_select;
	const unsigned int	(*adc_chan_select)(unsigned int chan);
};

static const struct soc_specific sun4i_gpadc_soc_specific = {
	.temp_offset = -1932,
	.temp_scale = 133,
	.tp_mode_en = SUN4I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN4I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
};

static const struct soc_specific sun5i_gpadc_soc_specific = {
	.temp_offset = -1447,
	.temp_scale = 100,
	.tp_mode_en = SUN4I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN4I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
};

static const struct soc_specific sun6i_gpadc_soc_specific = {
	.temp_offset = -1623,
	.temp_scale = 167,
	.tp_mode_en = SUN6I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN6I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun6i_gpadc_chan_select,
};

struct sun4i_gpadc_dev {
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
	const struct soc_specific	*soc_specific;
	/* prevents concurrent reads of temperature and ADC */
	struct mutex				mutex;
};

#define SUN4I_GPADC_ADC_CHANNEL(_channel, _name) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.datasheet_name = _name,				\
}

static struct iio_map sun4i_gpadc_hwmon_maps[] = {
	{
		.adc_channel_label = "temp_adc",
		.consumer_dev_name = "iio_hwmon.0",
	},
	{ /* sentinel */ },
};

static const struct iio_chan_spec sun4i_gpadc_channels[] = {
	SUN4I_GPADC_ADC_CHANNEL(0, "adc_chan0"),
	SUN4I_GPADC_ADC_CHANNEL(1, "adc_chan1"),
	SUN4I_GPADC_ADC_CHANNEL(2, "adc_chan2"),
	SUN4I_GPADC_ADC_CHANNEL(3, "adc_chan3"),
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.datasheet_name = "temp_adc",
	},
};

static int sun4i_gpadc_read(struct iio_dev *indio_dev, int channel, int *val,
			    unsigned int irq)
{
	struct sun4i_gpadc_dev *info = iio_priv(indio_dev);
	int ret = 0;

	pm_runtime_get_sync(indio_dev->dev.parent);
	mutex_lock(&info->mutex);

	reinit_completion(&info->completion);

	regmap_write(info->regmap, SUN4I_GPADC_INT_FIFOC,
		     SUN4I_GPADC_INT_FIFOC_TP_FIFO_TRIG_LEVEL(1) |
		     SUN4I_GPADC_INT_FIFOC_TP_FIFO_FLUSH);
	if (irq == info->fifo_data_irq) {
		regmap_write(info->regmap, SUN4I_GPADC_CTRL1,
			     info->soc_specific->tp_mode_en |
			     info->soc_specific->tp_adc_select |
			     info->soc_specific->adc_chan_select(channel));
	} else {
		/*
		 * The temperature sensor returns valid data only when the ADC
		 * operates in touchscreen mode.
		 */
		regmap_write(info->regmap, SUN4I_GPADC_CTRL1,
			     info->soc_specific->tp_mode_en);
	}

	enable_irq(irq);

	if (!wait_for_completion_timeout(&info->completion,
					 msecs_to_jiffies(100))) {
		ret = -ETIMEDOUT;
		goto out;
	}

	if (irq == info->fifo_data_irq)
		*val = info->adc_data;
	else
		*val = info->temp_data;

out:
	disable_irq(irq);
	mutex_unlock(&info->mutex);
	pm_runtime_mark_last_busy(indio_dev->dev.parent);
	pm_runtime_put_autosuspend(indio_dev->dev.parent);

	return ret;
}

static int sun4i_gpadc_adc_read(struct iio_dev *indio_dev, int channel,
				int *val)
{
	struct sun4i_gpadc_dev *info = iio_priv(indio_dev);

	return sun4i_gpadc_read(indio_dev, channel, val, info->fifo_data_irq);
}

static int sun4i_gpadc_temp_read(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_dev *info = iio_priv(indio_dev);

	return sun4i_gpadc_read(indio_dev, 0, val, info->temp_data_irq);
}

static int sun4i_gpadc_temp_offset(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_dev *info = iio_priv(indio_dev);

	*val = info->soc_specific->temp_offset;

	return 0;
}

static int sun4i_gpadc_temp_scale(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_dev *info = iio_priv(indio_dev);

	*val = info->soc_specific->temp_scale;

	return 0;
}

static int sun4i_gpadc_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		ret = sun4i_gpadc_temp_offset(indio_dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_VOLTAGE) {
			ret = sun4i_gpadc_adc_read(indio_dev, chan->channel,
						   val);
			if (ret)
				return ret;
		} else {
			ret = sun4i_gpadc_temp_read(indio_dev, val);
			if (ret)
				return ret;
		}

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = sun4i_gpadc_temp_scale(indio_dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static const struct iio_info sun4i_gpadc_iio_info = {
	.read_raw = sun4i_gpadc_read_raw,
	.driver_module = THIS_MODULE,
};

static irqreturn_t sun4i_gpadc_temp_data_irq_handler(int irq, void *dev_id)
{
	struct sun4i_gpadc_dev *info = dev_id;

	if (atomic_read(&info->ignore_temp_data_irq))
		return IRQ_HANDLED;

	if (!regmap_read(info->regmap, SUN4I_GPADC_TEMP_DATA, &info->temp_data))
		complete(&info->completion);

	return IRQ_HANDLED;
}

static irqreturn_t sun4i_gpadc_fifo_data_irq_handler(int irq, void *dev_id)
{
	struct sun4i_gpadc_dev *info = dev_id;

	if (atomic_read(&info->ignore_fifo_data_irq))
		return IRQ_HANDLED;

	if (!regmap_read(info->regmap, SUN4I_GPADC_DATA, &info->adc_data))
		complete(&info->completion);

	return IRQ_HANDLED;
}

static int sun4i_gpadc_runtime_suspend(struct device *dev)
{
	struct sun4i_gpadc_dev *info = iio_priv(dev_get_drvdata(dev));

	mutex_lock(&info->mutex);

	/* Disable the ADC on IP */
	regmap_write(info->regmap, SUN4I_GPADC_CTRL1, 0);
	/* Disable temperature sensor on IP */
	regmap_write(info->regmap, SUN4I_GPADC_TPR, 0);

	mutex_unlock(&info->mutex);

	return 0;
}

static int sun4i_gpadc_runtime_resume(struct device *dev)
{
	struct sun4i_gpadc_dev *info = iio_priv(dev_get_drvdata(dev));

	mutex_lock(&info->mutex);

	/* clkin = 6MHz */
	regmap_write(info->regmap, SUN4I_GPADC_CTRL0,
		     SUN4I_GPADC_CTRL0_ADC_CLK_DIVIDER(2) |
		     SUN4I_GPADC_CTRL0_FS_DIV(7) |
		     SUN4I_GPADC_CTRL0_T_ACQ(63));
	regmap_write(info->regmap, SUN4I_GPADC_CTRL1,
		     info->soc_specific->tp_mode_en);
	regmap_write(info->regmap, SUN4I_GPADC_CTRL3,
		     SUN4I_GPADC_CTRL3_FILTER_EN |
		     SUN4I_GPADC_CTRL3_FILTER_TYPE(1));
	/* period = SUN4I_GPADC_TPR_TEMP_PERIOD * 256 * 16 / clkin; ~1.3s */
	regmap_write(info->regmap, SUN4I_GPADC_TPR,
		     SUN4I_GPADC_TPR_TEMP_ENABLE |
		     SUN4I_GPADC_TPR_TEMP_PERIOD(1953));

	mutex_unlock(&info->mutex);

	return 0;
}

static int sun4i_gpadc_get_temp(void *data, int *temp)
{
	struct sun4i_gpadc_dev *info = (struct sun4i_gpadc_dev *)data;
	int val, scale, offset;

	/* If reading temperature times out, take stored previous value. */
	if (sun4i_gpadc_temp_read(info->indio_dev, &val))
		val = info->temp_data;
	sun4i_gpadc_temp_scale(info->indio_dev, &scale);
	sun4i_gpadc_temp_offset(info->indio_dev, &offset);

	*temp = (val + offset) * scale;

	return 0;
}

static const struct thermal_zone_of_device_ops sun4i_ts_tz_ops = {
	.get_temp = &sun4i_gpadc_get_temp,
};

static const struct dev_pm_ops sun4i_gpadc_pm_ops = {
	.runtime_suspend = &sun4i_gpadc_runtime_suspend,
	.runtime_resume = &sun4i_gpadc_runtime_resume,
};

static int sun4i_irq_init(struct platform_device *pdev, const char *name,
			  irq_handler_t handler, const char *devname,
			  unsigned int *irq, atomic_t *atomic)
{
	int ret;
	struct sun4i_gpadc_mfd_dev *mfd_dev = dev_get_drvdata(pdev->dev.parent);
	struct sun4i_gpadc_dev *info = iio_priv(dev_get_drvdata(&pdev->dev));

	/*
	 * Once the interrupt is activated, the IP continuously performs
	 * conversions thus throws interrupts. The interrupt is activated right
	 * after being requested but we want to control when these interrupts
	 * occurs thus we disable it right after being requested. However, an
	 * interrupt might occur between these two instructions and we have to
	 * make sure that does not happen, by using atomic flags. We set the
	 * flag before requesting the interrupt and unset it right after
	 * disabling the interrupt. When an interrupt occurs between these two
	 * instructions, reading the atomic flag will tell us to ignore the
	 * interrupt.
	 */
	atomic_set(atomic, 1);

	*irq = platform_get_irq_byname(pdev, name);
	if (*irq < 0) {
		dev_err(&pdev->dev, "no %s interrupt registered\n", name);
		return *irq;
	}

	*irq = regmap_irq_get_virq(mfd_dev->regmap_irqc, *irq);
	ret = devm_request_any_context_irq(&pdev->dev, *irq, handler, 0, devname,
					   info);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not request %s interrupt: %d\n",
			name, ret);
		return ret;
	}

	disable_irq(*irq);
	atomic_set(atomic, 0);

	return 0;
}

static int sun4i_gpadc_probe(struct platform_device *pdev)
{
	struct sun4i_gpadc_dev *info;
	struct iio_dev *indio_dev;
	int ret;
	struct sun4i_gpadc_mfd_dev *sun4i_gpadc_mfd_dev;
	struct thermal_zone_device *tzd;

	sun4i_gpadc_mfd_dev = dev_get_drvdata(pdev->dev.parent);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);
	platform_set_drvdata(pdev, indio_dev);

	mutex_init(&info->mutex);
	info->regmap = sun4i_gpadc_mfd_dev->regmap;
	info->indio_dev = indio_dev;
	init_completion(&info->completion);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &sun4i_gpadc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = ARRAY_SIZE(sun4i_gpadc_channels);
	indio_dev->channels = sun4i_gpadc_channels;

	info->soc_specific = (struct soc_specific *)platform_get_device_id(pdev)->driver_data;

	tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, info,
						   &sun4i_ts_tz_ops);
	if (IS_ERR(tzd)) {
		dev_err(&pdev->dev, "could not register thermal sensor: %ld\n",
			PTR_ERR(tzd));
		return PTR_ERR(tzd);
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 SUN4I_GPADC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = sun4i_irq_init(pdev, "TEMP_DATA_PENDING",
			     sun4i_gpadc_temp_data_irq_handler, "temp_data",
			     &info->temp_data_irq, &info->ignore_temp_data_irq);
	if (ret < 0)
		goto err;

	ret = sun4i_irq_init(pdev, "FIFO_DATA_PENDING",
			     sun4i_gpadc_fifo_data_irq_handler, "fifo_data",
			     &info->fifo_data_irq, &info->ignore_fifo_data_irq);
	if (ret < 0)
		goto err;

	ret = iio_map_array_register(indio_dev, sun4i_gpadc_hwmon_maps);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register iio map array\n");
		goto err;
	}

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
	pm_runtime_disable(&pdev->dev);
	/* Disable all interrupts */
	regmap_write(info->regmap, SUN4I_GPADC_INT_FIFOC, 0);
	printk("ret: %d\n", ret);

	return ret;
}

static int sun4i_gpadc_remove(struct platform_device *pdev)
{
	struct sun4i_gpadc_dev *info;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	info = iio_priv(indio_dev);
	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	/* Disable all interrupts */
	regmap_write(info->regmap, SUN4I_GPADC_INT_FIFOC, 0);

	return 0;
}

static const struct platform_device_id sun4i_gpadc_id[] = {
	{ "sun4i-a10-gpadc-iio", (kernel_ulong_t)&sun4i_gpadc_soc_specific },
	{ "sun5i-a13-gpadc-iio", (kernel_ulong_t)&sun5i_gpadc_soc_specific },
	{ "sun6i-a31-gpadc-iio", (kernel_ulong_t)&sun6i_gpadc_soc_specific },
	{ /* sentinel */ },
};

static struct platform_driver sun4i_gpadc_driver = {
	.driver = {
		.name = "sun4i-gpadc-iio",
		.pm = &sun4i_gpadc_pm_ops,
	},
	.id_table = sun4i_gpadc_id,
	.probe = sun4i_gpadc_probe,
	.remove = sun4i_gpadc_remove,
};

module_platform_driver(sun4i_gpadc_driver);

MODULE_DESCRIPTION("ADC driver for sunxi platforms");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL v2");
