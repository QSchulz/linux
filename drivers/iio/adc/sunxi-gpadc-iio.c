/* ADC driver for sunxi platforms
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/mfd/sunxi-gpadc-mfd.h>

#define SUNXI_GPADC_TP_CTRL0			0x00
#define SUNXI_GPADC_TP_CTRL1			0x04
#define SUNXI_GPADC_TP_CTRL2			0x08
#define SUNXI_GPADC_TP_CTRL3			0x0c
#define SUNXI_GPADC_TP_TPR			0x18
#define SUNXI_GPADC_TP_CDAT			0x1c
#define SUNXI_GPADC_TEMP_DATA			0x20
#define SUNXI_GPADC_TP_DATA			0x24

/* TP_CTRL0 bits */
#define SUNXI_GPADC_ADC_FIRST_DLY(x)		((x) << 24) /* 8 bits */
#define SUNXI_GPADC_ADC_FIRST_DLY_MODE		BIT(23)
#define SUNXI_GPADC_ADC_CLK_SELECT		BIT(22)
#define SUNXI_GPADC_ADC_CLK_DIVIDER(x)		((x) << 20) /* 2 bits */
#define SUNXI_GPADC_FS_DIV(x)			((x) << 16) /* 4 bits */
#define SUNXI_GPADC_T_ACQ(x)			((x) << 0)  /* 16 bits */

/* TP_CTRL1 bits */
#define SUNXI_GPADC_STYLUS_UP_DEBOUNCE(x)	((x) << 12) /* 8 bits */
#define SUNXI_GPADC_STYLUS_UP_DEBOUNCE_EN	BIT(9)
#define SUNXI_GPADC_TOUCH_PAN_CALI_EN		BIT(6)
#define SUNXI_GPADC_TP_DUAL_EN			BIT(5)
#define SUNXI_GPADC_TP_MODE_EN			BIT(4)
#define SUNXI_GPADC_TP_ADC_SELECT		BIT(3)
#define SUNXI_GPADC_ADC_CHAN_SELECT(x)		((x) << 0)  /* 3 bits */

/* TP_CTRL1 bits for sun6i SOCs */
#define SUNXI_GPADC_SUN6I_TOUCH_PAN_CALI_EN	BIT(7)
#define SUNXI_GPADC_SUN6I_TP_DUAL_EN		BIT(6)
#define SUNXI_GPADC_SUN6I_TP_MODE_EN		BIT(5)
#define SUNXI_GPADC_SUN6I_TP_ADC_SELECT		BIT(4)
#define SUNXI_GPADC_SUN6I_ADC_CHAN_SELECT(x)	BIT(x)  /* 4 bits */

/* TP_CTRL2 bits */
#define SUNXI_GPADC_TP_SENSITIVE_ADJUST(x)	((x) << 28) /* 4 bits */
#define SUNXI_GPADC_TP_MODE_SELECT(x)		((x) << 26) /* 2 bits */
#define SUNXI_GPADC_PRE_MEA_EN			BIT(24)
#define SUNXI_GPADC_PRE_MEA_THRE_CNT(x)		((x) << 0)  /* 24 bits */

/* TP_CTRL3 bits */
#define SUNXI_GPADC_FILTER_EN			BIT(2)
#define SUNXI_GPADC_FILTER_TYPE(x)		((x) << 0)  /* 2 bits */

/* TP_INT_FIFOC irq and fifo mask / control bits */
#define SUNXI_GPADC_TEMP_IRQ_EN			BIT(18)
#define SUNXI_GPADC_TP_OVERRUN_IRQ_EN		BIT(17)
#define SUNXI_GPADC_TP_DATA_IRQ_EN		BIT(16)
#define SUNXI_GPADC_TP_DATA_XY_CHANGE		BIT(13)
#define SUNXI_GPADC_TP_FIFO_TRIG_LEVEL(x)	((x) << 8)  /* 5 bits */
#define SUNXI_GPADC_TP_DATA_DRQ_EN		BIT(7)
/* Be careful, flushing FIFO spawns SUNXI_GPADC_FIFO_DATA_PENDING interrupts */
#define SUNXI_GPADC_TP_FIFO_FLUSH		BIT(4)
#define SUNXI_GPADC_TP_UP_IRQ_EN		BIT(1)
#define SUNXI_GPADC_TP_DOWN_IRQ_EN		BIT(0)

/* TP_INT_FIFOS irq and fifo status bits */
#define SUNXI_GPADC_TEMP_DATA_PENDING		BIT(18)
#define SUNXI_GPADC_FIFO_OVERRUN_PENDING	BIT(17)
#define SUNXI_GPADC_FIFO_DATA_PENDING		BIT(16)
#define SUNXI_GPADC_RXA_CNT			GENMASK(12, 8)
#define SUNXI_GPADC_TP_IDLE_FLG			BIT(2)
#define SUNXI_GPADC_TP_UP_PENDING		BIT(1)
#define SUNXI_GPADC_TP_DOWN_PENDING		BIT(0)

/* TP_TPR bits */
#define SUNXI_GPADC_TEMP_ENABLE(x)		((x) << 16)
/* t = x * 256 * 16 / clkin */
#define SUNXI_GPADC_TEMP_PERIOD(x)		((x) << 0)

#define SUNXI_GPADC_ARCH_SUN4I			BIT(0)
#define SUNXI_GPADC_ARCH_SUN5I			BIT(1)
#define SUNXI_GPADC_ARCH_SUN6I			BIT(2)

struct sunxi_gpadc_dev {
	void __iomem			*regs;
	struct completion		completion;
	int				temp_data;
	u32				adc_data;
	struct regmap			*regmap;
	unsigned int			fifo_data_irq;
	unsigned int			temp_data_irq;
	unsigned int			flags;
	struct iio_dev			*indio_dev;
	struct sunxi_gpadc_buffer	buffer;
	bool				ts_attached;
	bool				buffered;
};

#define SUNXI_GPADC_ADC_CHANNEL(_channel, _name, _index) {	\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.datasheet_name = _name,				\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 12,					\
		.storagebits = 16,				\
		.shift = 0,					\
		.endianness = IIO_LE,				\
	},							\
}

static struct iio_map sunxi_gpadc_hwmon_maps[] = {
	{
		.adc_channel_label = "adc_chan0",
		.consumer_dev_name = "sunxi-gpadc-ts.0",
	}, {
		.adc_channel_label = "adc_chan1",
		.consumer_dev_name = "sunxi-gpadc-ts.0",
	}, {
		.adc_channel_label = "adc_chan2",
		.consumer_dev_name = "sunxi-gpadc-ts.0",
	}, {
		.adc_channel_label = "adc_chan3",
		.consumer_dev_name = "sunxi-gpadc-ts.0",
	}, {
		.adc_channel_label = "temp_adc",
		.consumer_dev_name = "iio_hwmon.0",
	},
	{ /* sentinel */ },
};

static const struct iio_chan_spec sunxi_gpadc_channels[] = {
	SUNXI_GPADC_ADC_CHANNEL(0, "adc_chan0", 1),
	SUNXI_GPADC_ADC_CHANNEL(1, "adc_chan1", 2),
	SUNXI_GPADC_ADC_CHANNEL(2, "adc_chan2", 3),
	SUNXI_GPADC_ADC_CHANNEL(3, "adc_chan3", 4),
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.datasheet_name = "temp_adc",
		.extend_name = "SoC temperature",
	},
};

static int sunxi_gpadc_adc_read(struct iio_dev *indio_dev, int channel,
				int *val)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);
	bool buffered = info->buffered;
	int ret = 0;
	unsigned int reg;

	mutex_lock(&indio_dev->mlock);

	reinit_completion(&info->completion);

	reg = SUNXI_GPADC_TP_FIFO_TRIG_LEVEL(1) | SUNXI_GPADC_TP_FIFO_FLUSH;
	regmap_update_bits(info->regmap, SUNXI_GPADC_TP_INT_FIFOC, reg, reg);

	if (info->flags & SUNXI_GPADC_ARCH_SUN6I)
		regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
			     SUNXI_GPADC_SUN6I_TP_MODE_EN |
			     SUNXI_GPADC_SUN6I_TP_ADC_SELECT |
			     SUNXI_GPADC_SUN6I_ADC_CHAN_SELECT(channel));
	else
		regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
			     SUNXI_GPADC_TP_MODE_EN |
			     SUNXI_GPADC_TP_ADC_SELECT |
			     SUNXI_GPADC_ADC_CHAN_SELECT(channel));

	info->buffered = false;

	enable_irq(info->fifo_data_irq);

	if (!wait_for_completion_timeout(&info->completion,
					 msecs_to_jiffies(100))) {
		ret = -ETIMEDOUT;
		goto out;
	}

	*val = info->adc_data;

out:
	disable_irq(info->fifo_data_irq);
	mutex_unlock(&indio_dev->mlock);
	info->buffered = buffered;

	return ret;
}

static int sunxi_gpadc_temp_read(struct iio_dev *indio_dev, int *val)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);
	int ret = 0;
	unsigned int reg;

	mutex_lock(&indio_dev->mlock);

	reinit_completion(&info->completion);

	reg = SUNXI_GPADC_TP_FIFO_TRIG_LEVEL(1) | SUNXI_GPADC_TP_FIFO_FLUSH;
	regmap_update_bits(info->regmap, SUNXI_GPADC_TP_INT_FIFOC, reg, reg);

	if (info->flags & SUNXI_GPADC_ARCH_SUN6I)
		regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
			     SUNXI_GPADC_SUN6I_TP_MODE_EN);
	else
		regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
			     SUNXI_GPADC_TP_MODE_EN);

	enable_irq(info->temp_data_irq);

	if (!wait_for_completion_timeout(&info->completion,
					 msecs_to_jiffies(100))) {
		ret = -ETIMEDOUT;
		goto out;
	}

	if (info->flags & SUNXI_GPADC_ARCH_SUN4I)
		*val = info->temp_data * 133 - 257000;
	else if (info->flags & SUNXI_GPADC_ARCH_SUN5I)
		*val = info->temp_data * 100 - 144700;
	else if (info->flags & SUNXI_GPADC_ARCH_SUN6I)
		*val = info->temp_data * 167 - 271000;

out:
	disable_irq(info->temp_data_irq);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int sunxi_gpadc_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	int ret;
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		if (info->buffered && !info->ts_attached)
			return -EBUSY;

		ret = sunxi_gpadc_temp_read(indio_dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (info->buffered)
			return -EBUSY;

		ret = sunxi_gpadc_adc_read(indio_dev, chan->channel, val);
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
	int ret;

	ret = regmap_read(info->regmap, SUNXI_GPADC_TEMP_DATA,
			  &info->temp_data);
	if (ret == 0)
		complete(&info->completion);

	return IRQ_HANDLED;
}

static irqreturn_t sunxi_gpadc_fifo_data_irq_handler(int irq, void *dev_id)
{
	struct sunxi_gpadc_dev *info = dev_id;
	int ret, reg, i, fifo_count;

	if (info->buffered) {
		if (regmap_read(info->regmap, SUNXI_GPADC_TP_INT_FIFOS, &reg))
			return IRQ_HANDLED;

		fifo_count = (reg & SUNXI_GPADC_RXA_CNT) >> 8;
		/* Sometimes, the interrupt occurs when the FIFO is empty. */
		if (!fifo_count)
			return IRQ_HANDLED;

		for (i = 0; i < fifo_count; i++) {
			if (regmap_read(info->regmap, SUNXI_GPADC_TP_DATA,
					&info->buffer.buffer[i]))
				return IRQ_HANDLED;
		}

		info->buffer.buff_size = i;

		iio_push_to_buffers(info->indio_dev, &info->buffer);

		return IRQ_HANDLED;
	}

	ret = regmap_read(info->regmap, SUNXI_GPADC_TP_DATA, &info->adc_data);
	if (ret == 0)
		complete(&info->completion);

	return IRQ_HANDLED;
}

static int sunxi_gpadc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	reg = SUNXI_GPADC_TP_FIFO_TRIG_LEVEL(1) | SUNXI_GPADC_TP_FIFO_FLUSH;
	regmap_update_bits(info->regmap, SUNXI_GPADC_TP_INT_FIFOC, reg, reg);

	if (info->ts_attached) {
		reg = SUNXI_GPADC_STYLUS_UP_DEBOUNCE(5) |
		      SUNXI_GPADC_STYLUS_UP_DEBOUNCE_EN;

		if (info->flags & SUNXI_GPADC_ARCH_SUN6I)
			reg |= SUNXI_GPADC_SUN6I_TP_MODE_EN;
		else
			reg |= SUNXI_GPADC_TP_MODE_EN;
	} else {
		if (info->flags & SUNXI_GPADC_ARCH_SUN6I)
			reg = SUNXI_GPADC_SUN6I_TP_MODE_EN |
			      SUNXI_GPADC_SUN6I_TP_ADC_SELECT;
		else
			reg = SUNXI_GPADC_TP_MODE_EN |
			      SUNXI_GPADC_TP_ADC_SELECT;
	}

	if (regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1, reg))
		return ret;

	info->buffered = true;

	enable_irq(info->fifo_data_irq);

	return 0;
}

static int sunxi_gpadc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct sunxi_gpadc_dev *info = iio_priv(indio_dev);

	disable_irq(info->fifo_data_irq);

	info->buffered = false;

	return 0;
}

static const struct iio_buffer_setup_ops sunxi_gpadc_buffer_setup_ops = {
	.postenable = sunxi_gpadc_buffer_postenable,
	.predisable = sunxi_gpadc_buffer_predisable,
};

static int sunxi_gpadc_probe(struct platform_device *pdev)
{
	struct sunxi_gpadc_dev *info;
	struct iio_dev *indio_dev;
	int ret, irq;
	struct sunxi_gpadc_mfd_dev *sunxi_gpadc_mfd_dev;

	sunxi_gpadc_mfd_dev = dev_get_drvdata(pdev->dev.parent);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);

	info->regmap = sunxi_gpadc_mfd_dev->regmap;
	info->indio_dev = indio_dev;
	init_completion(&info->completion);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &sunxi_gpadc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->num_channels = ARRAY_SIZE(sunxi_gpadc_channels);
	indio_dev->channels = sunxi_gpadc_channels;
	indio_dev->setup_ops = &sunxi_gpadc_buffer_setup_ops;

	info->flags = platform_get_device_id(pdev)->driver_data;
	info->ts_attached = of_property_read_bool(pdev->dev.parent->of_node,
						  "allwinner,ts-attached");

	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL0, SUNXI_GPADC_FS_DIV(7) |
		     SUNXI_GPADC_ADC_CLK_DIVIDER(2) | SUNXI_GPADC_T_ACQ(63));
	if (info->flags & SUNXI_GPADC_ARCH_SUN6I)
		regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
			     SUNXI_GPADC_SUN6I_TP_MODE_EN);
	else
		regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1,
			     SUNXI_GPADC_TP_MODE_EN);
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL2,
		     SUNXI_GPADC_TP_SENSITIVE_ADJUST(15));
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL3, SUNXI_GPADC_FILTER_EN |
		     SUNXI_GPADC_FILTER_TYPE(1));
	regmap_write(info->regmap, SUNXI_GPADC_TP_TPR,
		     SUNXI_GPADC_TEMP_ENABLE(1) |
		     SUNXI_GPADC_TEMP_PERIOD(1953));

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

	info->temp_data_irq = irq;
	disable_irq(irq);

	irq = platform_get_irq_byname(pdev, "FIFO_DATA_PENDING");
	if (irq < 0) {
		dev_err(&pdev->dev,
			"no FIFO_DATA_PENDING interrupt registered\n");
		ret = irq;
		goto err;
	}

	irq = regmap_irq_get_virq(sunxi_gpadc_mfd_dev->regmap_irqc, irq);
	ret = devm_request_any_context_irq(&pdev->dev, irq,
					   sunxi_gpadc_fifo_data_irq_handler,
					   0, "fifo_data", info);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"could not request FIFO_DATA_PENDING interrupt: %d\n",
			ret);
		goto err;
	}

	info->fifo_data_irq = irq;
	disable_irq(irq);

	ret = iio_map_array_register(indio_dev, sunxi_gpadc_hwmon_maps);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register iio map array\n");
		goto err;
	}

	platform_set_drvdata(pdev, indio_dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register the device\n");
		iio_map_array_unregister(indio_dev);
		goto err;
	}

	return 0;

err:
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1, 0);
	regmap_write(info->regmap, SUNXI_GPADC_TP_TPR, 0);

	return ret;
}

static int sunxi_gpadc_remove(struct platform_device *pdev)
{
	struct sunxi_gpadc_dev *info;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	info = iio_priv(indio_dev);
	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);
	regmap_write(info->regmap, SUNXI_GPADC_TP_INT_FIFOC, 0);
	regmap_write(info->regmap, SUNXI_GPADC_TP_CTRL1, 0);
	regmap_write(info->regmap, SUNXI_GPADC_TP_TPR, 0);

	return 0;
}

static const struct platform_device_id sunxi_gpadc_id[] = {
	{ "sun4i-a10-gpadc-iio", SUNXI_GPADC_ARCH_SUN4I },
	{ "sun5i-a13-gpadc-iio", SUNXI_GPADC_ARCH_SUN5I },
	{ "sun6i-a31-gpadc-iio", SUNXI_GPADC_ARCH_SUN6I },
	{ /* sentinel */ },
};

static struct platform_driver sunxi_gpadc_driver = {
	.driver = {
		.name = "sunxi-gpadc-iio",
	},
	.id_table = sunxi_gpadc_id,
	.probe = sunxi_gpadc_probe,
	.remove = sunxi_gpadc_remove,
};

module_platform_driver(sunxi_gpadc_driver);

MODULE_DESCRIPTION("ADC driver for sunxi platforms");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL v2");
