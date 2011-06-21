/*
 * $License:
 *    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 * $
 */

/**
 *  @defgroup
 *  @brief
 *
 *  @{
 *      @file   mpu-i2c.c
 *      @brief
 *
 */

#include <linux/i2c.h>
#include "mpu.h"

int sensor_i2c_write(struct i2c_adapter *i2c_adap,
		     unsigned char address,
		     unsigned int len, unsigned char const *data)
{
	struct i2c_msg msgs[1];
	int res;

	if (NULL == data || NULL == i2c_adap)
		return -EINVAL;

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *) data;
	msgs[0].len = len;

	res = i2c_transfer(i2c_adap, msgs, 1);
	if (res < 1) {
		pr_err("%s: i2c_transfer() failed on slave addr 0x%x\n",
			__func__, address);
		WARN(1, "mpu sensor_i2c_write()");
		return res;
	}
	return 0;
}

int sensor_i2c_write_register(struct i2c_adapter *i2c_adap,
			      unsigned char address,
			      unsigned char reg, unsigned char value)
{
	unsigned char data[2];

	data[0] = reg;
	data[1] = value;
	return sensor_i2c_write(i2c_adap, address, 2, data);
}

int sensor_i2c_read(struct i2c_adapter *i2c_adap,
		    unsigned char address,
		    unsigned char reg,
		    unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (NULL == data || NULL == i2c_adap)
		return -EINVAL;

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;

	res = i2c_transfer(i2c_adap, msgs, 2);
	if (res < 2) {
		pr_err("%s: i2c_transfer() failed on slave addr 0x%x\n",
			__func__, address);
		WARN(1, "mpu sensor_i2c_read()");
		return res;
	}
	return 0;
}

int mpu_memory_read(struct i2c_adapter *i2c_adap,
		    unsigned char mpu_addr,
		    unsigned short mem_addr,
		    unsigned int len, unsigned char *data)
{
	unsigned char bank[2];
	unsigned char addr[2];
	unsigned char buf;

	struct i2c_msg msgs[4];
	int ret;

	if (NULL == data || NULL == i2c_adap)
		return -EINVAL;

	bank[0] = MPUREG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = MPUREG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf = MPUREG_MEM_R_W;

	/* Write Message */
	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = mpu_addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = mpu_addr;
	msgs[2].flags = 0;
	msgs[2].buf = &buf;
	msgs[2].len = 1;

	msgs[3].addr = mpu_addr;
	msgs[3].flags = I2C_M_RD;
	msgs[3].buf = data;
	msgs[3].len = len;

	ret = i2c_transfer(i2c_adap, msgs, 4);
	if (ret != 4) {
		WARN(1, "mpu_memory_read()");
		return ret;
	}
	return 0;
}

int mpu_memory_write(struct i2c_adapter *i2c_adap,
		     unsigned char mpu_addr,
		     unsigned short mem_addr,
		     unsigned int len, unsigned char const *data)
{
	unsigned char bank[2];
	unsigned char addr[2];
	unsigned char buf[513];

	struct i2c_msg msgs[3];
	int ret;

	if (NULL == data || NULL == i2c_adap)
		return -EINVAL;
	if (len >= (sizeof(buf) - 1))
		return -ENOMEM;

	bank[0] = MPUREG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = MPUREG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf[0] = MPUREG_MEM_R_W;
	memcpy(buf + 1, data, len);

	/* Write Message */
	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = mpu_addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = mpu_addr;
	msgs[2].flags = 0;
	msgs[2].buf = (unsigned char *) buf;
	msgs[2].len = len + 1;

	ret = i2c_transfer(i2c_adap, msgs, 3);
	if (ret != 3) {
		WARN(1, "mpu_memory_write()");
		return ret;
	}
	return 0;
}

/**
 *  @}
 */
