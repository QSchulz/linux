/*
 * Battery power input driver for X-Powers AXP20x PMICs
 *
 * Copyright 2014 Bruno Prémont <bonbons at linux-vserver.org>
 * Copyright 2016 Quentin Schulz <quentin.schulz@free-electrons.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/mfd/axp20x.h>

struct axp20x_power {
	struct axp20x_dev *axp20x;
	/* RTC / Backup battery */
	struct power_supply *backup;
	char backup_name[24];
	/* ACIN power supply */
	struct power_supply *ac;
	char ac_name[24];
	/* VBUS/OTG power supply */
	struct power_supply *vbus;
	char vbus_name[24];
	/* Battery charger */
	struct power_supply *battery;
	char battery_name[24];
	char *battery_supplies[2];
	/* AXP state tracking */
	struct work_struct work;
	spinlock_t lock;
	struct timespec next_check;
	uint8_t status1;
	uint8_t status2;
	uint8_t vbusmgt;
	int vvbus;
	int ivbus;
	int vac;
	int iac;
	int vbatt;
	int ibatt;
	int pbatt;
	int tbatt;
	int tbatt_min;
	int tbatt_max;
	int batt_percent;
	int batt_capacity;
	int batt_health;
	int batt_user_imax;
};

/* Fields of AXP20X_VBUS_IPSOUT_MGMT */
#define AXP20X_VBUS_VHOLD_MASK   (7 << 3)
#define AXP20X_VBUS_VHOLD_mV(b)  (4000000 + (((b) >> 3) & 7) * 100000)
#define AXP20X_VBUS_CLIMIT_MASK  (3)
#define AXP20X_VBUC_CLIMIT_900mA (0)
#define AXP20X_VBUC_CLIMIT_500mA (1)
#define AXP20X_VBUC_CLIMIT_100mA (2)
#define AXP20X_VBUC_CLIMIT_NONE  (3)

/* Fields of AXP20X_OFF_CTRL */
#define AXP20X_OFF_CTRL_BATT_MON    (1 << 6)
#define AXP20X_OFF_CTRL_CHGLED_MASK (3 << 4)
#define AXP20X_OFF_CTRL_CHGLED_HR   (0 << 4)
#define AXP20X_OFF_CTRL_CHGLED_1Hz  (1 << 4)
#define AXP20X_OFF_CTRL_CHGLED_4Hz  (2 << 4)
#define AXP20X_OFF_CTRL_CHGLED_LOW  (3 << 4)
#define AXP20X_OFF_CTRL_CHGLED_FIX  (1 << 3)
/* Fields of AXP20X_CHRG_CTRL1 */
#define AXP20X_CHRG_CTRL1_ENABLE    (1 << 7)
#define AXP20X_CHRG_CTRL1_TGT_VOLT  (3 << 5)
#define AXP20X_CHRG_CTRL1_TGT_4_1V  (0 << 5)
#define AXP20X_CHRG_CTRL1_TGT_4_15V (1 << 5)
#define AXP20X_CHRG_CTRL1_TGT_4_2V  (2 << 5)
#define AXP20X_CHRG_CTRL1_TGT_4_36V (3 << 5)
#define AXP20X_CHRG_CTRL1_END_CURR  (1 << 4)
#define AXP20X_CHRG_CTRL1_TGT_CURR  0x0f
/* Fields of AXP20X_CHRG_CTRL2 */
#define AXP20X_CHRG_CTRL2_PRE_MASK  (3 << 6)
#define AXP20X_CHRG_CTRL2_PRE_40MIN (0 << 6)
#define AXP20X_CHRG_CTRL2_PRE_50MIN (1 << 6)
#define AXP20X_CHRG_CTRL2_PRE_60MIN (2 << 6)
#define AXP20X_CHRG_CTRL2_PRE_70MIN (3 << 6)
#define AXP20X_CHRG_CTRL2_CHGLED_FL (1 << 4)
#define AXP20X_CHRG_CTRL2_CHG_MASK  (0 << 6)
#define AXP20X_CHRG_CTRL2_CHG_6H    (0 << 0)
#define AXP20X_CHRG_CTRL2_CHG_8H    (1 << 0)
#define AXP20X_CHRG_CTRL2_CHG_10H   (2 << 6)
#define AXP20X_CHRG_CTRL2_CHG_12H   (3 << 0)

static void axp20x_battery_chg_reconfig(struct power_supply *psy);

static int axp20x_battery_config(struct platform_device *pdev,
				 struct axp20x_power *devdata,
				 struct axp20x_dev *axp20x)
{
	struct device_node *np;
	int i, ret = 0, reg, new_reg = 0;
	u32 ocv[16], temp[3], rdc, capa;

	ret = regmap_read(axp20x->regmap, AXP20X_PWR_OP_MODE, &reg);
	if (ret)
		return ret;

	np = of_node_get(axp20x->dev->of_node);
	if (!np)
		return -ENODEV;

	ret = of_property_read_u32_array(np, "battery.ocv", ocv, 16);
	for (i = 0; ret == 0 && i < ARRAY_SIZE(ocv); i++)
		if (ocv[i] > 100) {
			dev_warn(&pdev->dev, "OCV[%d] %u > 100\n", i, ocv[i]);
			ret = -EINVAL;
			goto err;
		}

	ret = of_property_read_u32_array(np, "battery.resistance", &rdc, 1);
	if (ret != 0)
		rdc = 100;

	ret = of_property_read_u32_array(np, "battery.capacity", &capa, 1);
	if (ret != 0)
		capa = 0;

	ret = of_property_read_u32_array(np, "battery.temp_sensor", temp, 3);
	if (ret != 0)
		memset(temp, 0, sizeof(temp));
	else if (temp[0] != 20 && temp[0] != 40 && temp[0] != 60 &&
		 temp[0] != 80) {
		dev_warn(&pdev->dev, "Invalid battery temperature sensor current setting\n");
		ret = -EINVAL;
		memset(temp, 0, sizeof(temp));
	}

	dev_info(&pdev->dev, "FDT settings: capacity=%d, resistance=%d, temp_sensor=<%d %d %d>\n", capa, rdc, temp[0], temp[1], temp[2]);
	/* apply settings */
	devdata->batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	regmap_update_bits(axp20x->regmap, AXP20X_FG_RES, AXP20X_FG_ENABLE, 0x00);
	regmap_update_bits(axp20x->regmap, AXP20X_RDC_H, 0x80, 0x00);
	regmap_update_bits(axp20x->regmap, AXP20X_RDC_L, 0xff, (rdc * 10000 + 5371) / 10742);
	regmap_update_bits(axp20x->regmap, AXP20X_RDC_H, 0x1f, ((rdc * 10000 + 5371) / 10742) >> 8);
	if (of_find_property(np, "battery.ocv", NULL))
		for (i = 0; i < ARRAY_SIZE(ocv); i++) {
			ret = regmap_update_bits(axp20x->regmap, AXP20X_OCV(i),
						 0xff, ocv[i]);
			if (ret)
				dev_warn(&pdev->dev,
					 "Failed to store OCV[%d] setting: %d\n",
					 i, ret);
		}
	regmap_update_bits(axp20x->regmap, AXP20X_FG_RES, AXP20X_FG_ENABLE, AXP20X_FG_ENABLE);



/* Not OCV */






	if (capa == 0 && !(reg & AXP20X_PWR_OP_BATT_PRESENT)) {
		/* No battery present or configured -> disable */
		regmap_update_bits(axp20x->regmap, AXP20X_CHRG_CTRL1, AXP20X_CHRG_CTRL1_ENABLE, 0x00);
		regmap_update_bits(axp20x->regmap, AXP20X_OFF_CTRL, AXP20X_OFF_CTRL_BATT_MON, 0x00);
		dev_info(&pdev->dev, "No battery, disabling charger\n");
		ret = -ENODEV;
		goto err;
	}

	if (temp[0] == 0) {
		regmap_update_bits(axp20x->regmap, AXP20X_ADC_RATE,
				   AXP20X_ADR_TS_WHEN_MASK |
				   AXP20X_ADR_TS_UNRELATED,
				   AXP20X_ADR_TS_UNRELATED |
				   AXP20X_ADR_TS_WHEN_OFF);
	} else {
		devdata->tbatt_min = temp[1];
		devdata->tbatt_max = temp[2];
		switch (temp[0]) {
		case 20:
			regmap_update_bits(axp20x->regmap, AXP20X_ADC_RATE,
					   AXP20X_ADR_TS_CURR_MASK |
					   AXP20X_ADR_TS_WHEN_MASK |
					   AXP20X_ADR_TS_UNRELATED,
					   AXP20X_ADR_TS_CURR_20uA |
					   AXP20X_ADR_TS_WHEN_ADC);
			break;
		case 40:
			regmap_update_bits(axp20x->regmap, AXP20X_ADC_RATE,
					   AXP20X_ADR_TS_CURR_MASK |
					   AXP20X_ADR_TS_WHEN_MASK |
					   AXP20X_ADR_TS_UNRELATED,
					   AXP20X_ADR_TS_CURR_40uA |
					   AXP20X_ADR_TS_WHEN_ADC);
			break;
		case 60:
			regmap_update_bits(axp20x->regmap, AXP20X_ADC_RATE,
					   AXP20X_ADR_TS_CURR_MASK |
					   AXP20X_ADR_TS_WHEN_MASK |
					   AXP20X_ADR_TS_UNRELATED,
					   AXP20X_ADR_TS_CURR_60uA |
					   AXP20X_ADR_TS_WHEN_ADC);
			break;
		case 80:
			regmap_update_bits(axp20x->regmap, AXP20X_ADC_RATE,
					   AXP20X_ADR_TS_CURR_MASK |
					   AXP20X_ADR_TS_WHEN_MASK |
					   AXP20X_ADR_TS_UNRELATED,
					   AXP20X_ADR_TS_CURR_80uA |
					   AXP20X_ADR_TS_WHEN_ADC);
			break;
		}
		new_reg = temp[1] / (0x10 * 800);
		regmap_update_bits(axp20x->regmap, AXP20X_V_HTF_CHRG, 0xff,
				   new_reg);
		regmap_update_bits(axp20x->regmap, AXP20X_V_HTF_DISCHRG, 0xff,
				   new_reg);
		new_reg = temp[2] / (0x10 * 800);
		regmap_update_bits(axp20x->regmap, AXP20X_V_LTF_CHRG, 0xff,
				   new_reg);
		regmap_update_bits(axp20x->regmap, AXP20X_V_LTF_DISCHRG, 0xff,
				   new_reg);
	}
	devdata->batt_capacity  = capa * 1000;
	devdata->batt_user_imax = (capa < 300 ? 300 : capa) * 1000;
	/* Prefer longer battery life over longer runtime. */
	regmap_update_bits(devdata->axp20x->regmap, AXP20X_CHRG_CTRL1,
			   AXP20X_CHRG_CTRL1_TGT_VOLT,
			   AXP20X_CHRG_CTRL1_TGT_4_15V);

	/* TODO: configure CHGLED? */

	/* Default to about 5% capacity, about 3.5V */
	regmap_update_bits(axp20x->regmap, AXP20X_APS_WARN_L1, 0xff,
			   (3500000 - 2867200) / 4 / 1400);
	regmap_update_bits(axp20x->regmap, AXP20X_APS_WARN_L2, 0xff,
			   (3304000 - 2867200) / 4 / 1400);
	/* RDC - disable capacity monitor, reconfigure, re-enable */
	regmap_update_bits(axp20x->regmap, AXP20X_FG_RES, 0x80, 0x80);
	regmap_update_bits(axp20x->regmap, AXP20X_RDC_H, 0x80, 0x00);
	regmap_update_bits(axp20x->regmap, AXP20X_RDC_H, 0x1f, ((rdc * 10000 + 5371) / 10742) >> 8);
	regmap_update_bits(axp20x->regmap, AXP20X_RDC_L, 0xff, (rdc * 10000 + 5371) / 10742);
	regmap_update_bits(axp20x->regmap, AXP20X_FG_RES, 0x80, 0x00);
	regmap_update_bits(axp20x->regmap, AXP20X_OFF_CTRL, AXP20X_OFF_CTRL_BATT_MON, AXP20X_OFF_CTRL_BATT_MON);
	axp20x_battery_chg_reconfig(devdata->battery);
	ret = 0;

err:
	of_node_put(np);
	return ret;
}

static int axp20x_battery_get_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct axp20x_power *devdata = dev_get_drvdata(psy->dev.parent);
	int ret, reg;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = regmap_read(devdata->axp20x->regmap, AXP20X_CHRG_CTRL1,
				  &reg);
		if (ret)
			return ret;
		val->intval = (reg & AXP20X_CHRG_CTRL1_TGT_CURR) * 100000 +
			      300000;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		ret = regmap_read(devdata->axp20x->regmap, AXP20X_CHRG_CTRL1,
				  &reg);
		if (ret)
			return ret;
		switch (reg & AXP20X_CHRG_CTRL1_TGT_VOLT) {
		case AXP20X_CHRG_CTRL1_TGT_4_1V:
			val->intval = 4100000;
			break;
		case AXP20X_CHRG_CTRL1_TGT_4_15V:
			val->intval = 4150000;
			break;
		case AXP20X_CHRG_CTRL1_TGT_4_2V:
			val->intval = 4200000;
			break;
		case AXP20X_CHRG_CTRL1_TGT_4_36V:
			val->intval = 4360000;
			break;
		default:
			ret = -EINVAL;
		}
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = regmap_read(devdata->axp20x->regmap, AXP20X_APS_WARN_L2,
				  &reg);
		if (ret)
			return ret;
		val->intval = 2867200 + 1400 * reg * 4;
		return 0;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;

	default:
		break;
	}

	ret = axp20x_power_poll(devdata, 0);
	if (ret)
		return ret;

	spin_lock(&devdata->lock);
	switch (psp)  {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(devdata->status2 & AXP20X_PWR_OP_BATT_PRESENT);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		if (devdata->status1 & AXP20X_PWR_STATUS_BAT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (devdata->ibatt == 0 && devdata->batt_percent == 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (devdata->ibatt == 0)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = devdata->ibatt;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		// POWER_SUPPLY_HEALTH_GOOD, POWER_SUPPLY_HEALTH_OVERHEAT, POWER_SUPPLY_HEALTH_DEAD, POWER_SUPPLY_HEALTH_OVERVOLTAGE, POWER_SUPPLY_HEALTH_UNSPEC_FAILURE, POWER_SUPPLY_HEALTH_COLD, POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = devdata->vbatt;
		break;

	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = devdata->pbatt;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = devdata->batt_capacity;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		/* TODO */
		val->intval = 12345;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = devdata->batt_percent;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = axp20x_battery_uv_to_temp(devdata,
							devdata->tbatt);
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		val->intval = axp20x_battery_uv_to_temp(devdata,
							devdata->tbatt_min);
		break;

	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		val->intval = axp20x_battery_uv_to_temp(devdata,
							devdata->tbatt_max);
		break;

	default:
		ret = -EINVAL;
	}
	spin_unlock(&devdata->lock);

	return ret;
}

static int axp20x_battery_max_chg_current(struct axp20x_power *devdata)
{
	if ((devdata->status1 & AXP20X_PWR_STATUS_AC_PRESENT) &&
	    (devdata->status1 & AXP20X_PWR_STATUS_AC_AVAILABLE)) {
		/* AC available - unrestricted power */
		return devdata->batt_capacity / 2;
	} else if ((devdata->status1 & AXP20X_PWR_STATUS_VBUS_PRESENT) &&
		   (devdata->status1 & AXP20X_PWR_STATUS_VBUS_AVAILABLE)) {
		/* VBUS available - limited power */
		switch (devdata->vbusmgt & AXP20X_VBUS_CLIMIT_MASK) {
		case AXP20X_VBUC_CLIMIT_100mA:
			return 0;
		case AXP20X_VBUC_CLIMIT_500mA:
			return 300000;
		case AXP20X_VBUC_CLIMIT_900mA:
			return 600000;
		case AXP20X_VBUC_CLIMIT_NONE:
			return devdata->batt_capacity / 2;
		default:
			return 0;
		}
	} else {
		/* on-battery */
		return 0;
	}
}

static int axp20x_battery_set_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct axp20x_power *devdata = dev_get_drvdata(psy->dev.parent);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (val->intval == POWER_SUPPLY_STATUS_CHARGING) {
			ret = axp20x_battery_max_chg_current(devdata);
			if (ret == 0) {
				ret = -EBUSY;
				break;
			}
			ret = regmap_update_bits(devdata->axp20x->regmap,
						 AXP20X_PWR_OP_MODE,
						 AXP20X_PWR_OP_CHARGING,
						 AXP20X_PWR_OP_CHARGING);
			if (ret == 0)
				axp20x_battery_chg_reconfig(devdata->battery);
		} else if (val->intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			ret = regmap_update_bits(devdata->axp20x->regmap,
						 AXP20X_PWR_OP_MODE,
						 AXP20X_PWR_OP_CHARGING, 0);
		} else
			ret = -EINVAL;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		/* TODO: adjust AXP20X_APS_WARN_L1 and AXP20X_APS_WARN_L2 accordingly */
		ret = -EINVAL;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		switch (val->intval) {
		case 4100000:
			ret = regmap_update_bits(devdata->axp20x->regmap,
						 AXP20X_CHRG_CTRL1,
						 AXP20X_CHRG_CTRL1_TGT_VOLT,
						 AXP20X_CHRG_CTRL1_TGT_4_1V);
			break;
		case 4150000:
			ret = regmap_update_bits(devdata->axp20x->regmap,
						 AXP20X_CHRG_CTRL1,
						 AXP20X_CHRG_CTRL1_TGT_VOLT,
						 AXP20X_CHRG_CTRL1_TGT_4_15V);
			break;
		case 4200000:
			ret = regmap_update_bits(devdata->axp20x->regmap,
						 AXP20X_CHRG_CTRL1,
						 AXP20X_CHRG_CTRL1_TGT_VOLT,
						 AXP20X_CHRG_CTRL1_TGT_4_2V);
			break;
		case 4360000:
			/* refuse this as it's too much for Li-ion! */
		default:
			ret = -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (((val->intval - 300000) / 100000) > 0x0f)
			ret = -EINVAL;
		else if (val->intval < 300000)
			ret = -EINVAL;
		else {
			devdata->batt_user_imax = val->intval;
			axp20x_battery_chg_reconfig(devdata->battery);
			ret = 0;
		}
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static enum power_supply_property axp20x_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	/* POWER_SUPPLY_PROP_CHARGE_NOW, */
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
};

static int axp20x_battery_prop_writeable(struct power_supply *psy,
				      enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN ||
	       psp == POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN ||
	       psp == POWER_SUPPLY_PROP_CURRENT_MAX ||
	       psp == POWER_SUPPLY_PROP_STATUS;
}

static void axp20x_battery_chg_reconfig(struct power_supply *psy)
{
	struct axp20x_power *devdata = container_of(&psy,
				       struct axp20x_power, battery);
	int charge_max, ret;

	ret = axp20x_power_poll(devdata, 0);
	if (ret)
		return;

	charge_max = axp20x_battery_max_chg_current(devdata);

	if (charge_max == 0) {
		ret = regmap_update_bits(devdata->axp20x->regmap,
					 AXP20X_PWR_OP_MODE,
					 AXP20X_PWR_OP_CHARGING, 0);
	} else {
		if (devdata->batt_user_imax < charge_max)
			charge_max = devdata->batt_user_imax;
		if (((charge_max - 300000) / 100000) > 0x0f)
			charge_max = 300000 + 0x0f * 100000;
		ret = regmap_update_bits(devdata->axp20x->regmap,
					 AXP20X_CHRG_CTRL1,
					 AXP20X_CHRG_CTRL1_TGT_CURR,
					(charge_max - 300000) / 100000);
		ret = regmap_update_bits(devdata->axp20x->regmap,
					 AXP20X_PWR_OP_MODE,
				 AXP20X_PWR_OP_CHARGING,
					 AXP20X_PWR_OP_CHARGING);
	}
}

/* ********************************************** *
 * ***  Platform driver code                  *** *
 * ********************************************** */

static int axp20x_init_irq(struct platform_device *pdev,
	struct axp20x_dev *axp20x, const char *irq_name,
	const char *dev_name, irq_handler_t handler)
{
	int irq = platform_get_irq_byname(pdev, irq_name);
	int ret;

	if (irq < 0) {
		dev_warn(&pdev->dev, "No IRQ for %s: %d\n", irq_name, irq);
		return irq;
	}
	irq = regmap_irq_get_virq(axp20x->regmap_irqc, irq);

	ret = devm_request_any_context_irq(&pdev->dev, irq, handler, 0,
					dev_name, pdev);
	if (ret < 0)
		dev_warn(&pdev->dev, "Failed to request %s IRQ#%d: %d\n", irq_name, irq, ret);
	return ret;
}


static const struct power_supply_desc axp20x_batt_ps_desc = {
	.name = "axp20x-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = axp20x_battery_props,
	.num_properties = ARRAY_SIZE(axp20x_battery_props),
	.property_is_writeable = axp20x_battery_prop_writeable,
	.get_property = axp20x_battery_get_prop,
	.set_property = axp20x_battery_set_prop,
//	.external_power_changed = axp20x_battery_chg_reconfig,
};

struct axp20x_batt_ps {
	struct device *dev;
	struct power_supply *batt;
};

static int axp20x_power_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	struct axp20x_batt_ps *devdata;
	struct power_supply_desc *batt_desc;
	struct power_supply_config psy_cfg = {};
	struct iio_channel *pwr_batt, *batt_v, *batt_chrg_i, *batt_dischrg_i;
	int ret;

	devdata = devm_kzalloc(&pdev->dev, sizeof(struct axp20x_batt_ps),
			       GFP_KERNEL);
	if (!devdata)
		return -ENOMEM;

	batt_desc = devm_kzalloc(&pdev->dev, sizeof(struct power_supply_desc),
				 GFP_KERNEL);
	if (!batt_desc)
		return -ENOMEM;

	pwr_batt = devm_iio_channel_get(&pdev->dev, "pwr_batt");
	if (IS_ERR(pwr_batt))
		return ERR_PTR(pwr_batt);

	batt_v = devm_iio_channel_get(&pdev->dev, "batt_v");
	if (IS_ERR(batt_v))
		return ERR_PTR(batt_v);

	batt_chrg_i = devm_iio_channel_get(&pdev->dev, "batt_chrg_i");
	if (IS_ERR(batt_chrg_i))
		return ERR_PTR(batt_chrg_i);

	batt_dischrg_i = devm_iio_channel_get(&pdev->dev, "batt_dischrg_i");
	if (IS_ERR(batt_dischrg_i))
		return ERR_PTR(batt_dischrg_i);

	//spin_lock_init(&devdata->lock);
	devdata->axp20x = axp20x;
	platform_set_drvdata(pdev, devdata);

/*	ret = axp20x_battery_config(pdev, devdata, axp20x);
	if (ret)
		devdata->battery_name[0] = '\0';
	else if (devdata->tbatt_min == 0 && devdata->tbatt_max == 0)
		battery_desc->num_properties -= 3;
*/
	psy_cfg.drv_data = devdata;
	psy_cfg.of_node = pdev->dev.of_node;

	devdata->batt = devm_power_supply_register(&pdev->dev,
						   &axp20x_batt_ps_desc,
						   &psy_cfg);
	if (IS_ERR(devdata->batt))
		return PTR_ERR(devdata->batt);

//	power_supply_changed(devdata->battery);

	axp20x_init_irq(pdev, axp20x, "BATT_PLUGIN", battery_desc->name, axp20x_irq_batt_plugin);
	axp20x_init_irq(pdev, axp20x, "BATT_REMOVAL", battery_desc->name, axp20x_irq_batt_removal);
	axp20x_init_irq(pdev, axp20x, "BATT_ACTIVATE", battery_desc->name, axp20x_irq_batt_activation);
	axp20x_init_irq(pdev, axp20x, "BATT_ACTIVATED", battery_desc->name, axp20x_irq_batt_activated);
	axp20x_init_irq(pdev, axp20x, "BATT_CHARGING", battery_desc->name, axp20x_irq_batt_charging);
	axp20x_init_irq(pdev, axp20x, "BATT_CHARGED", battery_desc->name, axp20x_irq_batt_charged);
	if (devdata->tbatt_min != 0 || devdata->tbatt_max != 0) {
		axp20x_init_irq(pdev, axp20x, "BATT_HOT", battery_desc->name, axp20x_irq_batt_high_temp);
		axp20x_init_irq(pdev, axp20x, "BATT_COLD", battery_desc->name, axp20x_irq_batt_low_temp);
	}
	axp20x_init_irq(pdev, axp20x, "BATT_CHG_CURR_LOW", battery_desc->name, axp20x_irq_batt_chg_curr_low);

	axp20x_init_irq(pdev, axp20x, "POWER_LOW_WARN", battery_desc->name, axp20x_irq_power_low);
	axp20x_init_irq(pdev, axp20x, "POWER_LOW_CRIT", battery_desc->name, axp20x_irq_power_low_crit);

	return 0;
}

static int axp20x_power_remove(struct platform_device *pdev)
{
	struct axp20x_power *devdata = platform_get_drvdata(pdev);

	power_supply_unregister(devdata->batt);

	return 0;
}

static const struct platform_device_id axp20x_battery_ps_id[] = {
	{ "x-powers,axp20x_battery_power_supply" },
	{ /* sentinel */ },
};

static struct platform_driver axp20x_power_driver = {
	.probe    = axp20x_power_probe,
	.remove   = axp20x_power_remove,
	.suspend  = axp20x_power_suspend,
	.resume   = axp20x_power_resume,
	.shutdown = axp20x_power_shutdown,
	.id_table = axp20x_battery_ps_id;
	.driver   = {
		.name  = "axp20x_battery_power_supply",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(axp20x_power_driver);

MODULE_DESCRIPTION("Battery power supply driver for AXP20x PMICs");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPLv2");
MODULE_ALIAS("platform:axp20x-power");
