/*
 * AXP20x pinctrl and GPIO driver
 *
 * Copyright (C) 2016 Maxime Ripard <maxime.ripard@free-electrons.com>
 * Copyright (C) 2017 Quentin Schulz <quentin.schulz@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/axp20x.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>

#define AXP20X_GPIO_FUNCTIONS		0x7
#define AXP20X_GPIO_FUNCTION_OUT_LOW	0
#define AXP20X_GPIO_FUNCTION_OUT_HIGH	1
#define AXP20X_GPIO_FUNCTION_INPUT	2

struct axp20x_pinctrl_desc {
	const struct pinctrl_pin_desc	*pins;
	unsigned int			npins;
	/* Stores the pins supporting LDO function. Bit offset is pin number. */
	unsigned int			ldo_mask;
	/* Stores the pins supporting ADC function. Bit offset is pin number. */
	unsigned int			adc_mask;
	unsigned int			gpio_status_offset;
};

struct axp20x_pinctrl_function {
	const char	*name;
	unsigned int	muxval;
	const char	**groups;
	unsigned int	ngroups;
};

struct axp20x_pctl {
	struct gpio_chip	chip;
	struct regmap		*regmap;
	struct pinctrl_dev			*pctl_dev;
	struct device				*dev;
	const struct axp20x_pinctrl_desc	*desc;
	struct axp20x_pinctrl_function		funcs[4];
};

static const struct pinctrl_pin_desc axp209_pins[] = {
	PINCTRL_PIN(0, "GPIO0"),
	PINCTRL_PIN(1, "GPIO1"),
	PINCTRL_PIN(2, "GPIO2"),
};

static const struct axp20x_pinctrl_desc axp20x_data = {
	.pins	= axp209_pins,
	.npins	= ARRAY_SIZE(axp209_pins),
	.ldo_mask = BIT(0) | BIT(1),
	.adc_mask = BIT(0) | BIT(1),
	.gpio_status_offset = 4,
};

static int axp20x_gpio_get_reg(unsigned offset)
{
	switch (offset) {
	case 0:
		return AXP20X_GPIO0_CTRL;
	case 1:
		return AXP20X_GPIO1_CTRL;
	case 2:
		return AXP20X_GPIO2_CTRL;
	}

	return -EINVAL;
}

static int axp20x_gpio_input(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int axp20x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct axp20x_pctl *pctl = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	ret = regmap_read(pctl->regmap, AXP20X_GPIO20_SS, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset + pctl->desc->gpio_status_offset));
}

static int axp20x_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct axp20x_pctl *pctl = gpiochip_get_data(chip);
	unsigned int val;
	int reg, ret;

	reg = axp20x_gpio_get_reg(offset);
	if (reg < 0)
		return reg;

	ret = regmap_read(pctl->regmap, reg, &val);
	if (ret)
		return ret;

	/*
	 * This shouldn't really happen if the pin is in use already,
	 * or if it's not in use yet, it doesn't matter since we're
	 * going to change the value soon anyway. Default to output.
	 */
	if ((val & AXP20X_GPIO_FUNCTIONS) > 2)
		return 0;

	/*
	 * The GPIO directions are the three lowest values.
	 * 2 is input, 0 and 1 are output
	 */
	return val & 2;
}

static int axp20x_gpio_output(struct gpio_chip *chip, unsigned offset,
			      int value)
{
	chip->set(chip, offset, value);

	return 0;
}

static void axp20x_gpio_set(struct gpio_chip *chip, unsigned offset,
			    int value)
{
	struct axp20x_pctl *pctl = gpiochip_get_data(chip);
	int reg;

	reg = axp20x_gpio_get_reg(offset);
	if (reg < 0)
		return;

	regmap_update_bits(pctl->regmap, reg,
			   AXP20X_GPIO_FUNCTIONS,
			   value ? AXP20X_GPIO_FUNCTION_OUT_HIGH :
			   AXP20X_GPIO_FUNCTION_OUT_LOW);
}

static int axp20x_pmx_set(struct pinctrl_dev *pctldev, unsigned int offset,
			  u8 config)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);
	int reg;

	reg = axp20x_gpio_get_reg(offset);
	if (reg < 0)
		return reg;

	return regmap_update_bits(pctl->regmap, reg, AXP20X_GPIO_FUNCTIONS,
				  config);
}

static int axp20x_pmx_func_cnt(struct pinctrl_dev *pctldev)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return ARRAY_SIZE(pctl->funcs);
}

static const char *axp20x_pmx_func_name(struct pinctrl_dev *pctldev,
					unsigned int selector)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->funcs[selector].name;
}

static int axp20x_pmx_func_groups(struct pinctrl_dev *pctldev,
				  unsigned int selector,
				  const char * const **groups,
				  unsigned int *num_groups)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctl->funcs[selector].groups;
	*num_groups = pctl->funcs[selector].ngroups;

	return 0;
}

static int axp20x_pmx_set_mux(struct pinctrl_dev *pctldev,
			      unsigned int function, unsigned int group)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int mask;

	/* Every pin supports GPIO_OUT and GPIO_IN functions */
	if (function < 2)
		return axp20x_pmx_set(pctldev, group,
				      pctl->funcs[function].muxval);

	if (function == 2)
		mask = pctl->desc->ldo_mask;
	else
		mask = pctl->desc->adc_mask;

	if (!(BIT(group) & mask))
		return -EINVAL;

	return axp20x_pmx_set(pctldev, group, pctl->funcs[function].muxval);
}

static int axp20x_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
					 struct pinctrl_gpio_range *range,
					 unsigned int offset, bool input)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if (input)
		return axp20x_pmx_set(pctldev, offset, pctl->funcs[1].muxval);

	return axp20x_pmx_set(pctldev, offset, pctl->funcs[0].muxval);
}

static const struct pinmux_ops axp20x_pmx_ops = {
	.get_functions_count	= axp20x_pmx_func_cnt,
	.get_function_name	= axp20x_pmx_func_name,
	.get_function_groups	= axp20x_pmx_func_groups,
	.set_mux		= axp20x_pmx_set_mux,
	.gpio_set_direction	= axp20x_pmx_gpio_set_direction,
	.strict			= true,
};

static int axp20x_groups_cnt(struct pinctrl_dev *pctldev)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->desc->npins;
}

static int axp20x_group_pins(struct pinctrl_dev *pctldev, unsigned int selector,
			     const unsigned int **pins, unsigned int *num_pins)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = (unsigned int *)&pctl->desc->pins[selector];
	*num_pins = 1;

	return 0;
}

static const char *axp20x_group_name(struct pinctrl_dev *pctldev,
				     unsigned int selector)
{
	struct axp20x_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->desc->pins[selector].name;
}

static const struct pinctrl_ops axp20x_pctrl_ops = {
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinconf_generic_dt_free_map,
	.get_groups_count	= axp20x_groups_cnt,
	.get_group_name		= axp20x_group_name,
	.get_group_pins		= axp20x_group_pins,
};

static void axp20x_funcs_groups_from_mask(struct platform_device *pdev,
					  unsigned int mask,
					  unsigned int mask_len,
					  struct axp20x_pinctrl_function *func,
					  const struct pinctrl_pin_desc *pins)

{
	unsigned long mask_cpy = mask;
	const char **group;
	int i;

	func->ngroups = hweight8(mask);
	if (func->ngroups > 0) {
		func->groups = devm_kzalloc(&pdev->dev,
					    hweight8(mask) * sizeof(const char *),
					    GFP_KERNEL);
		group = func->groups;
		i = 0;
		while (find_next_bit(&mask_cpy, mask_len, i) < mask_len){
			i = find_next_bit(&mask_cpy, mask_len, i);
			*group = pins[i].name;
			i++;
			group++;
		}
	}
}

static void axp20x_build_funcs_groups(struct platform_device *pdev)
{
	struct axp20x_pctl *pctl = platform_get_drvdata(pdev);
	int i, pin;

	pctl->funcs[0].name = "gpio_out";
	pctl->funcs[0].muxval = 0x0;
	pctl->funcs[1].name = "gpio_in";
	pctl->funcs[1].muxval = 0x2;
	pctl->funcs[2].name = "ldo";
	pctl->funcs[2].muxval = 0x3;
	pctl->funcs[3].name = "adc";
	pctl->funcs[3].muxval = 0x4;

	/* Every pin supports GPIO_OUT and GPIO_IN functions */
	for (i = 0; i < 2; i++) {
		pctl->funcs[i].ngroups = pctl->desc->npins;
		pctl->funcs[i].groups = devm_kzalloc(&pdev->dev,
						     pctl->desc->npins * sizeof(const char *),
						     GFP_KERNEL);
		for (pin = 0; pin < pctl->desc->npins; pin++) {
			pctl->funcs[i].groups[pin] = pctl->desc->pins[pin].name;
		}
	}

	axp20x_funcs_groups_from_mask(pdev, pctl->desc->ldo_mask,
				      pctl->desc->npins, &pctl->funcs[2],
				      pctl->desc->pins);

	axp20x_funcs_groups_from_mask(pdev, pctl->desc->adc_mask,
				      pctl->desc->npins, &pctl->funcs[3],
				      pctl->desc->pins);
}

static int axp20x_pctl_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	struct axp20x_pctl *pctl;
	struct pinctrl_desc *pctrl_desc;
	int ret;

	if (!of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	if (!axp20x) {
		dev_err(&pdev->dev, "Parent drvdata not set\n");
		return -EINVAL;
	}

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->chip.base			= -1;
	pctl->chip.can_sleep		= true;
	pctl->chip.request		= gpiochip_generic_request;
	pctl->chip.free			= gpiochip_generic_free;
	pctl->chip.parent		= &pdev->dev;
	pctl->chip.label		= dev_name(&pdev->dev);
	pctl->chip.owner		= THIS_MODULE;
	pctl->chip.get			= axp20x_gpio_get;
	pctl->chip.get_direction	= axp20x_gpio_get_direction;
	pctl->chip.set			= axp20x_gpio_set;
	pctl->chip.direction_input	= axp20x_gpio_input;
	pctl->chip.direction_output	= axp20x_gpio_output;
	pctl->chip.ngpio		= 3;

	pctl->desc = &axp20x_data;

	pctl->regmap = axp20x->regmap;

	pctl->dev = &pdev->dev;

	platform_set_drvdata(pdev, pctl);

	axp20x_build_funcs_groups(pdev);

	pctrl_desc = devm_kzalloc(&pdev->dev, sizeof(*pctrl_desc), GFP_KERNEL);
	if (!pctrl_desc)
		return -ENOMEM;

	pctrl_desc->name = dev_name(&pdev->dev);
	pctrl_desc->owner = THIS_MODULE;
	pctrl_desc->pins = pctl->desc->pins;
	pctrl_desc->npins = pctl->desc->npins;
	pctrl_desc->pctlops = &axp20x_pctrl_ops;
	pctrl_desc->pmxops = &axp20x_pmx_ops;

	pctl->pctl_dev = devm_pinctrl_register(&pdev->dev, pctrl_desc, pctl);
	if (IS_ERR(pctl->pctl_dev)) {
		dev_err(&pdev->dev, "couldn't register pinctrl driver\n");
		return PTR_ERR(pctl->pctl_dev);
	}

	ret = devm_gpiochip_add_data(&pdev->dev, &pctl->chip, pctl);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register GPIO chip\n");
		return ret;
	}

	ret = gpiochip_add_pin_range(&pctl->chip, dev_name(&pdev->dev),
				     pctl->desc->pins->number,
				     pctl->desc->pins->number,
				     pctl->desc->npins);
	if (ret) {
		dev_err(&pdev->dev, "failed to add pin range\n");
		return ret;
	}

	dev_info(&pdev->dev, "AXP209 pinctrl and GPIO driver loaded\n");

	return 0;
}

static const struct of_device_id axp20x_pctl_match[] = {
	{ .compatible = "x-powers,axp209-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, axp20x_pctl_match);

static struct platform_driver axp20x_pctl_driver = {
	.probe		= axp20x_pctl_probe,
	.driver = {
		.name		= "axp20x-gpio",
		.of_match_table	= axp20x_pctl_match,
	},
};

module_platform_driver(axp20x_pctl_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_DESCRIPTION("AXP20x PMIC pinctrl and GPIO driver");
MODULE_LICENSE("GPL");
