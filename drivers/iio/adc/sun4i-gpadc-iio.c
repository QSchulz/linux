/* ADC driver for sunxi platforms' (A10, A13, A31 and A33) GPADC
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons.com>
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
 * disabling interrupts to save some power but that resets the internal clock of
 * the IP, resulting in having to wait X seconds every time we want to read the
 * value of the thermal sensor.
 * This is also the reason of using autosuspend in pm_runtime. If there was no
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
#include <linux/mfd/sun4i-gpadc.h>

static unsigned int sun4i_gpadc_chan_select(unsigned int chan)
{
	return SUN4I_GPADC_CTRL1_ADC_CHAN_SELECT(chan);
}

static unsigned int sun6i_gpadc_chan_select(unsigned int chan)
{
	return SUN6I_GPADC_CTRL1_ADC_CHAN_SELECT(chan);
}

struct gpadc_data {
	int		temp_offset;
	int		temp_scale;
	unsigned int	tp_mode_en;
	unsigned int	tp_adc_select;
	unsigned int	(*adc_chan_select)(unsigned int chan);
};

static const struct gpadc_data sun4i_gpadc_data = {
	.temp_offset = -1932,
	.temp_scale = 133,
	.tp_mode_en = SUN4I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN4I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
};

static const struct gpadc_data sun5i_gpadc_data = {
	.temp_offset = -1447,
	.temp_scale = 100,
	.tp_mode_en = SUN4I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN4I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
};

static const struct gpadc_data sun6i_gpadc_data = {
	.temp_offset = -1623,
	.temp_scale = 167,
	.tp_mode_en = SUN6I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN6I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun6i_gpadc_chan_select,
};

static const struct gpadc_data sun8i_gpadc_data = {
	.temp_offset = -1665,
	.temp_scale = 162,
	.tp_mode_en = 0x100,
};

struct sun4i_gpadc_iio {
	struct iio_dev			*indio_dev;
	struct completion		completion;
	int				temp_data;
	u32				adc_data;
	struct regmap			*regmap;
	unsigned int			fifo_data_irq;
	atomic_t			ignore_fifo_data_irq;
	unsigned int			temp_data_irq;
	atomic_t			ignore_temp_data_irq;
	const struct gpadc_data		*data;
	bool				use_dt;
	/* prevents concurrent reads of temperature and ADC */
	struct mutex			mutex;
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

static const struct iio_chan_spec sun8i_gpadc_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.datasheet_name = "temp_adc",
	},
};

static const struct regmap_config sun4i_gpadc_regmap_config = {
        .reg_bits = 32,
        .val_bits = 32,
        .reg_stride = 4,
        .fast_io = true,
};

static void sun4i_disable_irq(struct sun4i_gpadc_iio *info, unsigned int irq)
{
	if (!info->use_dt)
		disable_irq(irq);
	else
		regmap_update_bits(info->regmap, SUN4I_GPADC_INT_FIFOC,
				   SUN4I_GPADC_INT_FIFOC_TEMP_IRQ_EN, 0);
}

static void sun4i_enable_irq(struct sun4i_gpadc_iio *info, unsigned int irq)
{
	if (!info->use_dt)
		enable_irq(irq);
	else
		regmap_update_bits(info->regmap, SUN4I_GPADC_INT_FIFOC,
				   SUN4I_GPADC_INT_FIFOC_TEMP_IRQ_EN,
				   SUN4I_GPADC_INT_FIFOC_TEMP_IRQ_EN);
}

static int sun4i_gpadc_read(struct iio_dev *indio_dev, int channel, int *val,
			    unsigned int irq)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&info->mutex);
	pm_runtime_get_sync(indio_dev->dev.parent);

	reinit_completion(&info->completion);

	ret = regmap_write(info->regmap, SUN4I_GPADC_INT_FIFOC,
			   SUN4I_GPADC_INT_FIFOC_TP_FIFO_TRIG_LEVEL(1) |
			   SUN4I_GPADC_INT_FIFOC_TP_FIFO_FLUSH);
	if (ret)
		goto err;

	if (irq == info->fifo_data_irq) {
		ret = regmap_write(info->regmap, SUN4I_GPADC_CTRL1,
				   info->data->tp_mode_en |
				   info->data->tp_adc_select |
				   info->data->adc_chan_select(channel));
	} else {
		/*
		 * The temperature sensor returns valid data only when the ADC
		 * operates in touchscreen mode.
		 */
		ret = regmap_write(info->regmap, SUN4I_GPADC_CTRL1,
				   info->data->tp_mode_en);
	}

	if (ret)
		goto err;

	sun4i_enable_irq(info, irq);

	if (!wait_for_completion_timeout(&info->completion,
					 msecs_to_jiffies(100))) {
		if ((irq == info->fifo_data_irq && info->adc_data == -1) ||
		    (irq == info->temp_data_irq && info->temp_data == -1)) {
			ret = -ETIMEDOUT;
			goto out;
		}
	}

	if (irq == info->fifo_data_irq)
		*val = info->adc_data;
	else
		*val = info->temp_data;

	ret = 0;

out:
	sun4i_disable_irq(info, irq);
	pm_runtime_mark_last_busy(indio_dev->dev.parent);
	pm_runtime_put_autosuspend(indio_dev->dev.parent);
err:
	mutex_unlock(&info->mutex);

	return ret;
}

static int sun4i_gpadc_adc_read(struct iio_dev *indio_dev, int channel,
				int *val)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);

	return sun4i_gpadc_read(indio_dev, channel, val, info->fifo_data_irq);
}

static int sun4i_gpadc_temp_read(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);

	return sun4i_gpadc_read(indio_dev, 0, val, info->temp_data_irq);
}

static int sun4i_gpadc_temp_offset(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);

	*val = info->data->temp_offset;

	return 0;
}

static int sun4i_gpadc_temp_scale(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);

	*val = info->data->temp_scale;

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
		if (chan->type == IIO_VOLTAGE)
			ret = sun4i_gpadc_adc_read(indio_dev, chan->channel,
						   val);
		else
			ret = sun4i_gpadc_temp_read(indio_dev, val);

		if (ret)
			return ret;

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
	struct sun4i_gpadc_iio *info = dev_id;

	if (atomic_read(&info->ignore_temp_data_irq))
		goto out;

	if (!regmap_read(info->regmap, SUN4I_GPADC_TEMP_DATA, &info->temp_data))
		complete(&info->completion);

out:
	if (info->use_dt)
		regmap_update_bits(info->regmap, SUN4I_GPADC_INT_FIFOS,
				   SUN4I_GPADC_INT_FIFOS_TEMP_DATA_PENDING,
				   SUN4I_GPADC_INT_FIFOS_TEMP_DATA_PENDING);
	return IRQ_HANDLED;
}

static irqreturn_t sun4i_gpadc_fifo_data_irq_handler(int irq, void *dev_id)
{
	struct sun4i_gpadc_iio *info = dev_id;

	if (atomic_read(&info->ignore_fifo_data_irq))
		return IRQ_HANDLED;

	if (!regmap_read(info->regmap, SUN4I_GPADC_DATA, &info->adc_data))
		complete(&info->completion);

	return IRQ_HANDLED;
}

static int sun4i_gpadc_runtime_suspend(struct device *dev)
{
	struct sun4i_gpadc_iio *info = iio_priv(dev_get_drvdata(dev));

	/* Disable the ADC on IP */
	regmap_write(info->regmap, SUN4I_GPADC_CTRL1, 0);
	/* Disable temperature sensor on IP */
	regmap_write(info->regmap, SUN4I_GPADC_TPR, 0);

	return 0;
}

static int sun4i_gpadc_runtime_resume(struct device *dev)
{
	struct sun4i_gpadc_iio *info = iio_priv(dev_get_drvdata(dev));

	/* clkin = 6MHz */
	regmap_write(info->regmap, SUN4I_GPADC_CTRL0, 0x002000ff);/*
		     SUN4I_GPADC_CTRL0_ADC_CLK_DIVIDER(2) |
		     SUN4I_GPADC_CTRL0_FS_DIV(7) |
		     SUN4I_GPADC_CTRL0_T_ACQ(63));*/
	regmap_write(info->regmap, SUN4I_GPADC_CTRL1, info->data->tp_mode_en);
/*	regmap_write(info->regmap, SUN4I_GPADC_CTRL3,
		     SUN4I_GPADC_CTRL3_FILTER_EN |
		     SUN4I_GPADC_CTRL3_FILTER_TYPE(1));*/
	/* period = SUN4I_GPADC_TPR_TEMP_PERIOD * 256 * 16 / clkin; ~1.3s */
	regmap_write(info->regmap, SUN4I_GPADC_TPR, 0x1005f);/*
		     SUN4I_GPADC_TPR_TEMP_ENABLE |
		     SUN4I_GPADC_TPR_TEMP_PERIOD(1953));*/

	return 0;
}

static int sun4i_gpadc_get_temp(void *data, int *temp)
{
	struct sun4i_gpadc_iio *info = (struct sun4i_gpadc_iio *)data;
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
	struct sun4i_gpadc_dev *mfd_dev;
	struct sun4i_gpadc_iio *info = iio_priv(dev_get_drvdata(&pdev->dev));

	/*
	 * Once the interrupt is activated, the IP continuously performs
	 * conversions thus throws interrupts. The interrupt is activated right
	 * after being requested but we want to control when these interrupts
	 * occur thus we disable it right after being requested. However, an
	 * interrupt might occur between these two instructions and we have to
	 * make sure that does not happen, by using atomic flags. We set the
	 * flag before requesting the interrupt and unset it right after
	 * disabling the interrupt. When an interrupt occurs between these two
	 * instructions, reading the atomic flag will tell us to ignore the
	 * interrupt.
	 */
	atomic_set(atomic, 1);

	if (!info->use_dt) {
		mfd_dev = dev_get_drvdata(pdev->dev.parent);

		*irq = platform_get_irq_byname(pdev, name);
		if (*irq < 0) {
			dev_err(&pdev->dev, "no %s interrupt registered\n",
				name);
			return *irq;
		}

		*irq = regmap_irq_get_virq(mfd_dev->regmap_irqc, *irq);
	} else {
		*irq = platform_get_irq(pdev, 0);
		if (*irq < 0) {
			dev_err(&pdev->dev, "no interrupt registered in DT\n");
			return *irq;
		}
	}

	ret = devm_request_any_context_irq(&pdev->dev, *irq, handler, 0,
					   devname, info);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not request %s interrupt: %d\n",
			name, ret);
		return ret;
	}

	sun4i_disable_irq(info, *irq);
	atomic_set(atomic, 0);

	return 0;
}

static const struct of_device_id sun4i_gpadc_of_id[] = {
	{
		.compatible = "allwinner,sun8i-a33-gpadc-iio",
		.data = &sun8i_gpadc_data, },
	{ /* sentinel */ }
};

static int sun4i_gpadc_probe(struct platform_device *pdev)
{
	struct sun4i_gpadc_iio *info;
	struct iio_dev *indio_dev;
	int ret;
	struct sun4i_gpadc_dev *sun4i_gpadc_dev;
	struct thermal_zone_device *tzd;
	const struct of_device_id *of_dev;
	struct resource *mem;
	void __iomem *base;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);
	platform_set_drvdata(pdev, indio_dev);

	mutex_init(&info->mutex);
	info->indio_dev = indio_dev;
	info->temp_data = -1;
	info->adc_data = -1;
	init_completion(&info->completion);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &sun4i_gpadc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (pdev->dev.of_node) {
		of_dev = of_match_device(sun4i_gpadc_of_id, &pdev->dev);
		if (!of_dev)
			return -ENODEV;

		info->use_dt = true;
		info->data = (struct gpadc_data *)of_dev->data;
		indio_dev->num_channels = ARRAY_SIZE(sun8i_gpadc_channels);
		indio_dev->channels = sun8i_gpadc_channels;

		mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		base = devm_ioremap_resource(&pdev->dev, mem);
		if (IS_ERR(base))
			return PTR_ERR(base);

		info->regmap = devm_regmap_init_mmio(&pdev->dev, base,
						     &sun4i_gpadc_regmap_config);
		if (IS_ERR(info->regmap)) {
			ret = PTR_ERR(info->regmap);
			dev_err(&pdev->dev, "failed to init regmap: %d\n", ret);
			return ret;
		}

	/*	tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, info,
							   &sun4i_ts_tz_ops);
		if (IS_ERR(tzd)) {
			dev_err(&pdev->dev,
				"could not register thermal sensor: %ld\n",
				PTR_ERR(tzd));
			return PTR_ERR(tzd);
		}*/

		ret = sun4i_irq_init(pdev, NULL,
				     sun4i_gpadc_temp_data_irq_handler,
				     "temp_data", &info->temp_data_irq,
				     &info->ignore_temp_data_irq);
		if (ret < 0)
			return ret;
	} else {
		info->use_dt = false;
		sun4i_gpadc_dev = dev_get_drvdata(pdev->dev.parent);
		info->regmap = sun4i_gpadc_dev->regmap;

		indio_dev->num_channels = ARRAY_SIZE(sun4i_gpadc_channels);
		indio_dev->channels = sun4i_gpadc_channels;

		info->data = (struct gpadc_data *)platform_get_device_id(pdev)->driver_data;

		/*
		 * This driver is a child of an MFD which has a node in the DT
		 * but not its children. Therefore, the resulting devices of
		 * this driver do not have an of_node variable.
		 * However, its parent (the MFD driver) has an of_node variable
		 * and since devm_thermal_zone_of_sensor_register uses its first
		 * argument to match the phandle defined in the node of the
		 * thermal driver with the of_node of the device passed as first
		 * argument and the third argument to call ops from
		 * thermal_zone_of_device_ops, the solution is to use the parent
		 * device as first argument to match the phandle with its
		 * of_node, and the device from this driver as third argument to
		 * return the temperature.
		 */
		tzd = devm_thermal_zone_of_sensor_register(pdev->dev.parent, 0,
							   info,
							   &sun4i_ts_tz_ops);
		if (IS_ERR(tzd)) {
			dev_err(&pdev->dev,
				"could not register thermal sensor: %ld\n",
				PTR_ERR(tzd));
			return PTR_ERR(tzd);
		}

		ret = sun4i_irq_init(pdev, "TEMP_DATA_PENDING",
				     sun4i_gpadc_temp_data_irq_handler,
				     "temp_data", &info->temp_data_irq,
				     &info->ignore_temp_data_irq);
		if (ret < 0)
			return ret;

		ret = sun4i_irq_init(pdev, "FIFO_DATA_PENDING",
				     sun4i_gpadc_fifo_data_irq_handler,
				     "fifo_data", &info->fifo_data_irq,
				     &info->ignore_fifo_data_irq);
		if (ret < 0)
			return ret;

		ret = iio_map_array_register(indio_dev, sun4i_gpadc_hwmon_maps);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to register iio map array\n");
			return ret;
		}
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 SUN4I_GPADC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register the device\n");
		goto err_map;
	}

	return 0;

err_map:
	//FIXME: what if not registered?
	iio_map_array_unregister(indio_dev);
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int sun4i_gpadc_remove(struct platform_device *pdev)
{
	struct sun4i_gpadc_iio *info;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	info = iio_priv(indio_dev);
	if (info->use_dt)
		disable_irq(info->temp_data_irq);
	//FIXME: WHAT if not registered
	iio_map_array_unregister(indio_dev);
	iio_device_unregister(indio_dev);

	return 0;
}

static const struct platform_device_id sun4i_gpadc_id[] = {
	{ "sun4i-a10-gpadc-iio", (kernel_ulong_t)&sun4i_gpadc_data },
	{ "sun5i-a13-gpadc-iio", (kernel_ulong_t)&sun5i_gpadc_data },
	{ "sun6i-a31-gpadc-iio", (kernel_ulong_t)&sun6i_gpadc_data },
	{ /* sentinel */ },
};

static struct platform_driver sun4i_gpadc_driver = {
	.driver = {
		.name = "sun4i-gpadc-iio",
		.of_match_table = sun4i_gpadc_of_id,
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
