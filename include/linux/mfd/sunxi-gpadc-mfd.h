/* Header of ADC MFD core driver for sunxi platforms
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __SUNXI_GPADC_MFD__H__
#define __SUNXI_GPADC_MFD__H__

#define SUNXI_GPADC_TP_INT_FIFOC            0x10
#define SUNXI_GPADC_TP_INT_FIFOS            0x14

struct sunxi_gpadc_mfd_dev {
	struct device			*dev;
	struct regmap			*regmap;
	struct regmap_irq_chip_data	*regmap_irqc;
	void __iomem			*regs;
};

struct sunxi_gpadc_buffer {
	u32				buffer[32];
	unsigned int			buff_size;
};

#endif
