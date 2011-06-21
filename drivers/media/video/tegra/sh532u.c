/*
 * SH532U focuser driver.
 *
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/sh532u.h>

#include <asm/traps.h>

#define POS_LOW (0xA000)
#define POS_HIGH (0x6000)
#define SETTLETIME_MS (7)
#define FOCAL_LENGTH 0x408d70a4 /* (4.42f) */
#define FNUMBER 0x40333333 /* (2.8f) */


struct sh532u_info {
	struct i2c_client *i2c_client;
	struct sh532u_config config;
};

static struct sh532u_info *info;

static int sh532u_read_u8(u8 dev, u8 addr, u8 *val)
{
	struct i2c_client *client = info->i2c_client;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (dev)
		msg[0].addr = dev;
	else
		msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	data[0] = (u8)addr;

	if (dev)
		msg[1].addr = dev;
	else
		msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -1;
	*val = data[2];
	return 0;
}

static int sh532u_read_u16(u8 addr, u16 *val)
{
	struct i2c_client *client = info->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[4];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];

	/* high byte goes out first */
	buf[0] = (u8) (addr);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = &buf[1];

	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -1;
	*val = (((u16)buf[1] << 8) | (u16)buf[2]);
	return 0;
}

static int eeprom_read_u32(u8 addr, u32 *val)
{
	struct i2c_client *client = info->i2c_client;
	struct i2c_msg msg[2];
	union {
		u8   dataU8[8];
		u32  dataU32[2];
	} buffer;

	msg[0].addr = 0x50;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &(buffer.dataU8[0]);

	/* high byte goes out first */
	buffer.dataU8[0] = (u8) (addr);
	buffer.dataU8[1] = (u8) (0);

	msg[1].addr = 0x50;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 4;
	msg[1].buf = (u8 *)&(buffer.dataU32[1]);

	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -1;
	*val = buffer.dataU32[1];
	return 0;
}

static int sh532u_write_u8(u16 addr, u8 val)
{
	struct i2c_client *client = info->i2c_client;
	struct i2c_msg msg;
	unsigned char data[2];
	u8 tmp;

	data[0] = (u8) (addr & 0xff);
	data[1] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	if (i2c_transfer(client->adapter, &msg, 1) != 1)
		return -1;

	return 0;
}

static int sh532u_write_u16(u16 addr, u16 val)
{
	struct i2c_client *client = info->i2c_client;
	struct i2c_msg msg;
	unsigned char data[3];

	data[0] = (u8) (addr & 0xff);
	data[1] = (u8) (val >> 8);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	if (i2c_transfer(client->adapter, &msg, 1) != 1)
		return -1;
	return 0;
}

static void move_driver(s16 tarPos)
{
	s16 curPos, moveStep;
	u16 moveDistance;
	int err;

	/* Read Current Position */
	err = sh532u_read_u16(RZ_211H, &curPos);
	if (err)
		goto move_driver_error;
	/* Check move distance to Target Position */
	moveDistance = abs((int)curPos - (int)tarPos);

	/* if move distance is shorter than MS1Z12(=Step width) */
	if (moveDistance <= STMV_SIZE) {
		err = sh532u_write_u8(MSSET_211, (INI_MSSET_211 | 0x01));
		err = err | sh532u_write_u16(MS1Z22_211H, tarPos);
		if (err)
			goto move_driver_error;
	} else {
		if (curPos < tarPos)
			moveStep = STMV_SIZE;
		else
			moveStep = -STMV_SIZE;

		/* Set StepMove Target Positon */
		err = sh532u_write_u16(MS1Z12_211H, moveStep);
		err = err | sh532u_write_u16(STMVENDH_211, tarPos);
		/* Start StepMove */
		err = err |
		      sh532u_write_u8(
			STMVEN_211,
			(STMCHTG_ON | STMSV_ON | STMLFF_OFF | STMVEN_ON));
		if (err)
			goto move_driver_error;
	}

	return;
move_driver_error:
	pr_err("Focuser: %s failed!\n", __func__);
}

static void wait_for_move(void)
{
	u16 usSmvFin;
	u8 moveTime, ucParMod, tmp;
	int err;

	moveTime = 0;
	do {
		mdelay(1);
		err = sh532u_read_u8(0, STMVEN_211, &ucParMod);
		err = err | sh532u_read_u16(RZ_211H, &usSmvFin);
		if (err)
			goto wait_for_move_error;
		/* StepMove Error Handling, Unexpected Position */
		if ((usSmvFin == 0x7FFF) || (usSmvFin == 0x8001)) {
			/* Stop StepMove Operation */
			err = sh532u_write_u8(STMVEN_211, ucParMod & 0xFE);
			if (err)
				goto wait_for_move_error;
		}
		moveTime++;
		/* Wait StepMove operation end */
	} while ((ucParMod & STMVEN_ON) && (moveTime < 50));

	moveTime = 0;
	if ((ucParMod & 0x08) == STMCHTG_ON) {
		mdelay(5);
		do {
			mdelay(1);
			moveTime++;
			err = sh532u_read_u8(0, MSSET_211, &tmp);
			if (err)
				goto wait_for_move_error;
		} while ((tmp & CHTGST_ON) && (moveTime < 15));
	}

	return;
wait_for_move_error:
	pr_err("Focuser: %s failed!\n", __func__);
}

static void lens_move_pulse(s16 position)
{
	move_driver(position);
	wait_for_move();
}

static void get_rom_info(void)
{
	u8 tmp;
	int err;

	/* Get Inf1, Mac1
	Inf1 and Mac1 are the mechanical limit position.
	Inf1     : Bottom limit.
	Mac1 : Top limit. */
	err = sh532u_read_u8(0x50, addrMac1, &tmp);
	if (err)
		goto get_rom_info_error;
	info->config.limit_low = (tmp<<8) & 0xff00;
	err = sh532u_read_u8(0x50, addrInf1, &tmp);
	if (err)
		goto get_rom_info_error;
	info->config.limit_high = (tmp<<8) & 0xff00;

	/* Get Inf2, Mac2
	Inf2 and Mac2 are the calibration data for SEMCO AF lens.
	Inf2: Best focus (lens position) when object distance is 1.2M.
	Mac2: Best focus (lens position) when object distance is 10cm. */
	err = sh532u_read_u8(0x50, addrMac2, &tmp);
	if (err)
		goto get_rom_info_error;
	info->config.pos_low = (tmp << 8) & 0xff00;
	err = sh532u_read_u8(0x50, addrInf2, &tmp);
	if (err)
		goto get_rom_info_error;
	info->config.pos_high = (tmp << 8) & 0xff00;

	return;
get_rom_info_error:
	pr_err("Focuser: %s failed!\n", __func__);
	info->config.limit_high = POS_HIGH;
	info->config.limit_low = POS_LOW;
	info->config.pos_high = POS_HIGH;
	info->config.pos_low = POS_LOW;
}

static void init_hvca_pos(void)
{
	short sBottomLimit, sTopLimit;

	get_rom_info();
	sBottomLimit = (((int)info->config.limit_low * 5) >> 3) & 0xFFC0;
	lens_move_pulse(sBottomLimit);
	sTopLimit = (((int)info->config.limit_high * 5) >> 3) & 0xFFC0;
	lens_move_pulse(sTopLimit);
	lens_move_pulse(info->config.pos_high);
}

static unsigned int a2buf[] = {
	0x0018019c,
	0x0018019d,
	0x0000019e,
	0x007f0192,
	0x00000194,
	0x00f00184,
	0x00850187,
	0x0000018a,
	0x00fd7187,
	0x007f7183,
	0x0008025a,
	0x05042218,
	0x80010216,
	0x000601a0,
	0x00808183,
	0xffffffff
};

/* Write 1 byte data to the HVCA Drive IC by data type */
static void sh532u_hvca_wr1(u8 ep_type, u8 ep_data1, u8 ep_addr)
{
	int err = 0;
	u8 us_data;

	switch (ep_type & 0xF0) {
	case DIRECT_MODE:
		us_data = ep_data1;
		break;

	case INDIRECT_EEPROM:
		err = sh532u_read_u8(0x50, ep_data1, &us_data);
		break;

	case INDIRECT_HVCA:
		err = sh532u_read_u8(0, (u16)ep_data1, &us_data);
		break;

	case MASK_AND:
		err = sh532u_read_u8(0, (u16)ep_addr, &us_data);
		us_data = us_data & ep_data1;
		break;

	case MASK_OR:
		err = sh532u_read_u8(0, (u16)ep_addr, &us_data);
		us_data = us_data | ep_data1;
		break;

	default:
		err = 1;
	}
	if (!err)
		err = sh532u_write_u8((u16)ep_addr, us_data);
	if (err)
		pr_err("Focuser: Failed to init!\n");
}

/* Write 2 byte data to the HVCA Drive IC by data type */
static void sh532u_hvca_wr2(u8 ep_type, u8 ep_data1, u8 ep_data2, u8 ep_addr)
{
	int err = 0;
	u8 uc_data1;
	u8 uc_data2;
	u16 us_data;

	switch (ep_type & 0xF0) {
	case DIRECT_MODE:
		us_data = (((u16)ep_data1 << 8) & 0xFF00) |
			((u16)ep_data2 & 0x00FF);
		break;

	case INDIRECT_EEPROM:
		err = sh532u_read_u8(0x50, (u16)ep_data1, &uc_data1);
		err = err | sh532u_read_u8(0x50, (u16)ep_data2, &uc_data2);
		us_data = (((u16)uc_data1 << 8) & 0xFF00) |
			((u16)uc_data2 & 0x00FF);
		break;

	case INDIRECT_HVCA:
		err = sh532u_read_u8(0, (u16)ep_data1, &uc_data1);
		err = err | sh532u_read_u8(0, (u16)ep_data2, &uc_data2);
		us_data = (((u16)uc_data1 << 8) & 0xFF00) |
			((u16)uc_data2 & 0x00FF);
		break;

	case MASK_AND:
		err = sh532u_read_u16((u16)ep_addr, &us_data);
		us_data = us_data & ((((u16)ep_data1 << 8) & 0xFF00) |
			((u16)ep_data2 & 0x00FF));
		break;

	case MASK_OR:
		err = sh532u_read_u16((u16)ep_addr, &us_data);
		us_data = us_data | ((((u16)ep_data1 << 8) & 0xFF00) |
			((u16)ep_data2 & 0x00FF));
		break;

	default:
		err = 1;
	}
	if (!err)
		err = sh532u_write_u16((u16)ep_addr, us_data);
	if (err)
		pr_err("Focuser: Failed to init!\n");
}

static void init_driver(void)
{
	int eeprom_addr;
	unsigned int eeprom_data;
	u8 ep_addr, ep_type, ep_data1, ep_data2, uc_data;

	for (eeprom_addr = 0x30; eeprom_addr <= 0x013C; eeprom_addr += 4) {
		if (eeprom_addr > 0xff) {
			/* use hardcoded data instead */
			eeprom_data = a2buf[(eeprom_addr & 0xFF) / 4];
		} else {
			if (eeprom_read_u32(eeprom_addr & 0xFF, &eeprom_data))
				pr_info("sh532u: cannot read eeprom\n");
		}

		/* HVCA Address to write eeprom Data1,Data2 by the Data type */
		ep_addr = (u8)(eeprom_data & 0x000000ff);
		ep_type = (u8)((eeprom_data & 0x0000ff00) >> 8);
		ep_data1 = (u8)((eeprom_data & 0x00ff0000) >> 16);
		ep_data2 = (u8)((eeprom_data & 0xff000000) >> 24);

		if (ep_addr == 0xFF)
			break;

		if (ep_addr == 0xDD) {
			mdelay((unsigned int)((ep_data1 << 8) | ep_data2));
		} else {
			if ((ep_type & 0x0F) == DATA_1BYTE) {
				sh532u_hvca_wr1(ep_type, ep_data1, ep_addr);
			} else {
				sh532u_hvca_wr2(ep_type,
						ep_data1,
						ep_data2,
						ep_addr);
			}
		}
	}
	msleep(300);

	init_hvca_pos();
}


static int sh532u_set_position(struct sh532u_info *info, s16 position)
{
	if (position > info->config.limit_high)
		return -1;
	lens_move_pulse(position);
	return 0;
}

static long sh532u_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	struct sh532u_info *info = file->private_data;

	switch (cmd) {
	case SH532U_IOCTL_GET_CONFIG:
		if (copy_to_user((void __user *) arg,
				 &info->config,
				 sizeof(info->config))) {
			pr_err("%s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;

	case SH532U_IOCTL_SET_POSITION:
		return sh532u_set_position(info, (s16)(arg & 0xffff));

	default:
		return -EINVAL;
	}
}


static int sh532u_open(struct inode *inode, struct file *file)
{
	pr_info("sh532 open\n");
	file->private_data = info;
	init_driver();
	return 0;
}

int sh532u_release(struct inode *inode, struct file *file)
{
	pr_info("sh532 release\n");
	file->private_data = NULL;
	return 0;
}


static const struct file_operations sh532u_fileops = {
	.owner = THIS_MODULE,
	.open = sh532u_open,
	.unlocked_ioctl = sh532u_ioctl,
	.release = sh532u_release,
};

static struct miscdevice sh532u_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sh532u",
	.fops = &sh532u_fileops,
};

static int sh532u_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err;

	pr_info("sh532u: probing sensor.\n");
	info = kzalloc(sizeof(struct sh532u_info), GFP_KERNEL);
	if (!info) {
		pr_err("sh532u: Unable to allocate memory!\n");
		return -ENOMEM;
	}
	err = misc_register(&sh532u_device);
	if (err) {
		pr_err("sh532u: Unable to register misc device!\n");
		kfree(info);
		return err;
	}
	info->i2c_client = client;
	info->config.settle_time = SETTLETIME_MS;
	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.pos_low = POS_LOW;
	info->config.pos_high = POS_HIGH;
	i2c_set_clientdata(client, info);
	return 0;
}

static int sh532u_remove(struct i2c_client *client)
{
	struct sh532u_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&sh532u_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sh532u_id[] = {
	{ "sh532u", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sh532u_id);

static struct i2c_driver sh532u_i2c_driver = {
	.driver = {
		.name = "sh532u",
		.owner = THIS_MODULE,
	},
	.probe = sh532u_probe,
	.remove = sh532u_remove,
	.id_table = sh532u_id,
};

static int __init sh532u_init(void)
{
	return i2c_add_driver(&sh532u_i2c_driver);
}

static void __exit sh532u_exit(void)
{
	i2c_del_driver(&sh532u_i2c_driver);
}

module_init(sh532u_init);
module_exit(sh532u_exit);

