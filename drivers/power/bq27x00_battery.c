/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>

#define DRIVER_VERSION			"1.1.0"

#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_TTECP		0x26

#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_FLAG_CHGS		BIT(7)

#define BQ27500_REG_SOC			0x2c
#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27500_FLAG_BAT_DET		BIT(3)
#define BQ27500_FLAG_FC			BIT(9)

#define BQ27510_CNTL			0x00
#define BQ27510_ATRATE			0x02
#define BQ27510_ENERGY_AVAIL		0x22
#define BQ27510_POWER_AVG		0x24
#define BQ27510_CYCLE_COUNT		0x2a
/* bq27510-g2 control register sub-commands*/
#define BQ27510_CNTL_DEVICE_TYPE	0x0001
#define BQ27510_CNTL_SET_SLEEP		0x0013
#define BQ27510_CNTL_CLEAR_SLEEP	0x0014

/* Define battery poll period to 30ms */
#define BATTERY_POLL_PERIOD		30000

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27x00_device_info *di);
};

enum bq27x00_chip { BQ27000, BQ27500, BQ27510 };

struct bq27x00_device_info {
	struct device		*dev;
	int			id;
	struct bq27x00_access_methods	*bus;
	struct power_supply	bat;
	struct power_supply	ac;
	struct timer_list	battery_poll_timer;
	enum bq27x00_chip	chip;
	int			irq;
	bool			battery_present;
	struct i2c_client	*client;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

/*
 * Common code for BQ27x00 devices
 */

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27x00_read(BQ27x00_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	if ((di->chip == BQ27500) || (di->chip == BQ27510))
		return temp - 2731;
	else
		return ((temp >> 2) - 273) * 10;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27x00_read(BQ27x00_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt * 1000;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di)
{
	int ret;
	int curr = 0;
	int flags = 0;

	ret = bq27x00_read(BQ27x00_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	if ((di->chip == BQ27500) || (di->chip == BQ27510)) {
		/* bq27500 returns signed value */
		curr = (int)(s16)curr;
	} else {
		ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
		if (ret < 0) {
			dev_err(di->dev, "error reading flags\n");
			return 0;
		}
		if (flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}
	}

	return curr * 1000;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{
	int ret;
	int rsoc = 0;

	if ((di->chip == BQ27500) || (di->chip == BQ27510))
		ret = bq27x00_read(BQ27500_REG_SOC, &rsoc, 0, di);
	else
		ret = bq27x00_read(BQ27000_REG_RSOC, &rsoc, 1, di);
	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	return rsoc;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
				  union power_supply_propval *val)
{
	int flags = 0;
	int status;
	int ret;

	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	if ((di->chip == BQ27500) || (di->chip == BQ27510)) {
		if (flags & BQ27500_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (flags & BQ27500_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	val->intval = status;
	return 0;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_time(struct bq27x00_device_info *di, int reg,
				union power_supply_propval *val)
{
	int tval = 0;
	int ret;

	ret = bq27x00_read(reg, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading register %02x\n", reg);
		return ret;
	}

	if (tval == 65535)
		return -ENODATA;

	val->intval = tval * 60;
	return 0;
}

static int bq27510_battery_present(struct bq27x00_device_info *di,
					union power_supply_propval *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(di->client, BQ27x00_REG_FLAGS);
	if (!(ret & BQ27500_FLAG_BAT_DET))
		val->intval = 0;
	else
		val->intval = 1;
	return val->intval;
}

static char bq27510_serial[5];
static int bq27510_get_battery_serial_number(struct bq27x00_device_info *di,
					union power_supply_propval *val)
{
	int ret;

	if (di->chip == BQ27510) {
		ret = i2c_smbus_write_word_data(di->client, BQ27510_CNTL,
					BQ27510_CNTL_DEVICE_TYPE);
		if (ret < 0) {
			dev_err(di->dev, "write failure\n");
			return ret;
		}
		ret = i2c_smbus_read_word_data(di->client, 0x00);
		if (ret  < 0) {
			dev_err(di->dev, "read failure\n");
			return ret;
		}

		ret = sprintf(bq27510_serial, "%04x", ret);
		val->strval = bq27510_serial;
		return 0;
	} else {
		return 1;
	}
}

/*
 * energy now is the predicted charge or energy remaining in the
 * battery. The value is reported in units of 10mWh.
 */

static int bq27510_battery_energy_now(struct bq27x00_device_info *di,
				int reg_offset)
{
	int ret;

	if (di->chip == BQ27510) {
		ret = i2c_smbus_read_word_data(di->client, reg_offset);
		if (ret  < 0) {
			dev_err(di->dev, "read failure\n");
			return ret;
		}
		return ret * (10 * 1000);
	} else {
		return -1;
	}
}

static int bq27510_battery_power_avg(struct bq27x00_device_info *di,
				int reg_offset)
{
	if (di->chip == BQ27510)
		return i2c_smbus_read_word_data(di->client, reg_offset);
	else
		return 0;
}

static int bq27510_battery_cycle_count(struct bq27x00_device_info *di,
				int reg_offset)
{
	int ret;

	if (di->chip == BQ27510) {
		ret = i2c_smbus_read_word_data(di->client, reg_offset);
		if (ret  < 0)
			dev_err(di->dev, "read failure\n");
		return ret;
	} else {
		return -1;
	}
}

#define bat_to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = bat_to_bq27x00_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq27x00_battery_voltage(di);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27510_battery_present(di, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27x00_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27x00_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27x00_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_battery_time(di, BQ27x00_REG_TTE, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_battery_time(di, BQ27x00_REG_TTECP, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_battery_time(di, BQ27x00_REG_TTF, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = bq27510_battery_energy_now(di,
					BQ27510_ENERGY_AVAIL);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		val->intval = bq27510_battery_power_avg(di,
					BQ27510_POWER_AVG);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = bq27510_battery_cycle_count(di,
					BQ27510_CYCLE_COUNT);
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		if (bq27510_get_battery_serial_number(di, val))
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

#define ac_to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, ac);

static enum power_supply_property bq27x00_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *ac_power_supplied_to[] = {
	"bq27x00-bat",
};

static int bq27x00_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	struct bq27x00_device_info *di = ac_to_bq27x00_device_info(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = gpio_get_value(di->irq);
		break;
	default:
		dev_err(&di->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	if (di->battery_present) {
		di->bat.name = "bq27x00-bat";
		di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
		di->bat.properties = bq27x00_battery_props;
		di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
		di->bat.get_property = bq27x00_battery_get_property;
		di->bat.external_power_changed = NULL;
	}

	di->ac.name = "bq27x00-ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.supplied_to = ac_power_supplied_to;
	di->ac.num_supplicants = ARRAY_SIZE(ac_power_supplied_to);
	di->ac.properties = bq27x00_ac_props;
	di->ac.num_properties = ARRAY_SIZE(bq27x00_ac_props);
	di->ac.get_property = bq27x00_ac_get_property;
}

/*
 * i2c specific code
 */

static int bq27x00_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

static irqreturn_t ac_present_irq(int irq, void *data)
{
	struct bq27x00_device_info *di = data;
	power_supply_changed(&di->ac);
	return IRQ_HANDLED;
}

static void battery_poll_timer_func(unsigned long pdi)
{
	struct bq27x00_device_info *di = (void *)pdi;
	power_supply_changed(&di->bat);
	mod_timer(&di->battery_poll_timer,
		jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));
}

static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int num;
	u16 read_data;
	int retval = 0;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;

	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
	di->id = num;
	di->chip = id->driver_data;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	di->irq = client->irq;
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27x00_read_i2c;
	di->bus = bus;
	di->client = client;

	/* Let's see whether this adapter can support what we need. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "insufficient functionality!\n");
		retval = -ENODEV;
		goto batt_failed_3;
	}

	read_data = i2c_smbus_read_word_data(di->client, BQ27x00_REG_FLAGS);

	if (!(read_data & BQ27500_FLAG_BAT_DET))
		dev_err(&client->dev, "no battery present\n");
	else
		di->battery_present = true;

	bq27x00_powersupply_init(di);

	if (di->battery_present) {
		retval = power_supply_register(&client->dev, &di->bat);
		if (retval) {
			dev_err(&client->dev, "failed to register battery\n");
			goto batt_failed_3;
		}

		setup_timer(&di->battery_poll_timer,
			battery_poll_timer_func, (unsigned long) di);
		mod_timer(&di->battery_poll_timer,
			jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));
	}

	retval = power_supply_register(&client->dev, &di->ac);
	if (retval) {
		dev_err(&client->dev, "failed to register ac power supply\n");
		goto batt_failed_4;
	}

	retval = request_threaded_irq(di->irq, NULL,
		ac_present_irq,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"ac_present", di);
	if (retval < 0) {
		dev_err(&di->client->dev,
			"%s: request_irq failed(%d)\n", __func__, retval);
		goto batt_failed_5;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);
	return 0;

batt_failed_5:
	power_supply_unregister(&di->ac);
batt_failed_4:
	if (di->battery_present) {
		power_supply_unregister(&di->bat);
		del_timer_sync(&di->battery_poll_timer);
	}
batt_failed_3:
	kfree(bus);
batt_failed_2:
	kfree(di);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	free_irq(di->irq, di);
	power_supply_unregister(&di->ac);

	if (di->battery_present) {
		power_supply_unregister(&di->bat);
		del_timer_sync(&di->battery_poll_timer);
	}

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di->bus);
	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int bq27x00_battery_suspend(struct i2c_client *client,
	pm_message_t state)
{
	u8 ret;
	struct bq27x00_device_info *bq27500_device;

	bq27500_device = i2c_get_clientdata(client);

	if (!bq27500_device->battery_present)
		return 0;

	del_timer_sync(&bq27500_device->battery_poll_timer);

	if (bq27500_device->chip == BQ27510) {
		ret = i2c_smbus_write_word_data(bq27500_device->client,
			BQ27510_CNTL, BQ27510_CNTL_SET_SLEEP);
		if (ret < 0) {
			dev_err(&bq27500_device->client->dev,
				"write failure\n");
			return ret;
		}
		ret = i2c_smbus_write_word_data(bq27500_device->client,
				BQ27510_CNTL, BQ27510_CNTL_DEVICE_TYPE);
		if (ret  < 0) {
			dev_err(&bq27500_device->client->dev,
				"write failure\n");
			return ret;
		}
	}
	return 0;
}

static int bq27x00_battery_resume(struct i2c_client *client)
{
	u8 ret;
	struct bq27x00_device_info *bq27500_device;

	bq27500_device = i2c_get_clientdata(client);

	if (!bq27500_device->battery_present)
		return 0;

	if (bq27500_device->chip == BQ27510) {
		ret = i2c_smbus_write_word_data(bq27500_device->client,
				BQ27510_CNTL, BQ27510_CNTL_CLEAR_SLEEP);
		if (ret < 0) {
			dev_err(&bq27500_device->client->dev,
				"write failure\n");
			return ret;
		}
		ret = i2c_smbus_write_word_data(bq27500_device->client,
				BQ27510_CNTL, BQ27510_CNTL_DEVICE_TYPE);
		if (ret  < 0) {
			dev_err(&bq27500_device->client->dev,
				"write failure\n");
			return ret;
		}
	}

	setup_timer(&bq27500_device->battery_poll_timer,
		battery_poll_timer_func, (unsigned long) bq27500_device);
	mod_timer(&bq27500_device->battery_poll_timer,
		jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));

	return 0;
}
#endif


/*
 * Module stuff
 */

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27000 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27500", BQ27500 },
	{ "bq27510", BQ27510 },
	{},
};

static struct i2c_driver bq27x00_battery_driver = {
	.probe		= bq27x00_battery_probe,
	.remove		= bq27x00_battery_remove,
#if defined(CONFIG_PM)
	.suspend	= bq27x00_battery_suspend,
	.resume		= bq27x00_battery_resume,
#endif
	.id_table = bq27x00_id,
	.driver = {
		.name = "bq27x00-battery",
	},
};

static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27x00_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 driver\n");

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
