/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/melfas_ts.h>
#include <mach/gpio.h>
#include <mach/gpio-sec.h>

#define TS_READ_START_ADDR 0x10
#define TS_READ_REGS_LEN 5
#define TS_WRITE_REGS_LEN 16
#define DOWNLOAD_RETRY_CNT	5
#define P5_MAX_TOUCH	10

#ifndef I2C_M_WR
#define I2C_M_WR 0
#endif

#define DEBUG_MODE
#define SET_DOWNLOAD_BY_GPIO
//#define TS_TOUCH_KEY

#ifdef TS_TOUCH_KEY
#define PRESS_KEY	1
#define RELEASE_KEY	0
#endif

#define REPORT_MT(touch_number, x, y, amplitude) \
do {     \
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_number);\
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, amplitude);         \
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, amplitude); \
	input_mt_sync(ts->input_dev);                                      \
} while (0)

#ifdef SET_DOWNLOAD_BY_GPIO
#include "mcs8000_download.h"
#endif

struct melfas_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct melfas_tsi_platform_data *pdata;
	struct early_suspend early_suspend;
	uint32_t flags;
#ifdef SET_DOWNLOAD_BY_GPIO
	int gpio_scl;
	int gpio_sda;
#endif
	int (*power)(int on);
	void (*power_enable)(int en);
};

#ifdef CONFIG_SAMSUNG_INPUT
extern struct class *sec_class;
#endif

#ifdef FW_FROM_FILE
static void remove_i2c_driver(void);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static struct muti_touch_info g_Mtouch_info[P5_MAX_TOUCH];
static bool debug_print = false;

#ifdef DEBUG_MODE
static bool debug_on = false;
static tCommandInfo_t tCommandInfo[] = {
	{ '?', "Help" },
	{ 'T', "Go to LOGGING mode" },
	{ 'M', "Go to MTSI_1_2_0 mode" },
	{ 'R', "Toggle LOG ([R]awdata)" },
	{ 'F', "Toggle LOG (Re[f]erence)" },
	{ 'I', "Toggle LOG ([I]ntensity)" },
	{ 'G', "Toggle LOG ([G]roup Image)" },
	{ 'D', "Toggle LOG ([D]elay Image)" },
	{ 'P', "Toggle LOG ([P]osition)" },
	{ 'B', "Toggle LOG (De[b]ug)" },
	{ 'V', "Toggle LOG (Debug2)" },
	{ 'L', "Toggle LOG (Profi[l]ing)" },
	{ 'O', "[O]ptimize Delay" },
	{ 'N', "[N]ormalize Intensity" }
};

static bool vbLogType[LT_LIMIT] = {0, };
static const char mcLogTypeName[LT_LIMIT][20] = {
	"LT_DIAGNOSIS_IMG",
	"LT_RAW_IMG",
	"LT_REF_IMG",
	"LT_INTENSITY_IMG",
	"LT_GROUP_IMG",
	"LT_DELAY_IMG",
	"LT_POS",
	"LT_DEBUG",
	"LT_DEBUG2",
	"LT_PROFILING",
};

static void toggle_log(struct melfas_ts_data *ts, eLogType_t _eLogType);
static void print_command_list(void);
static int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 length, u8 *value);

static void debug_i2c_read(struct i2c_client *client, u16 addr, u8 *value, u16 length)
{
	melfas_i2c_read(client, addr, length, value);
}

static int debug_i2c_write(struct i2c_client *client, u8 *value, u16 length)
{
	return i2c_master_send(client, value, length);
}

static void key_handler(struct melfas_ts_data *ts, char key_val)
{
	u8 write_buf[2];
	u8 read_buf[2];
	int try_cnt = 0;
	pr_info("[TSP] %s - %c\n", __func__, key_val);
	switch (key_val) {
	case '?':
	case '/':
		print_command_list();
		break;
	case 'T':
	case 't':
		write_buf[0] = ADDR_ENTER_LOGGING;
		write_buf[1] = 1;
		debug_i2c_write(ts->client, write_buf, 2);
		debug_on = true;
		for ( try_cnt = 1; try_cnt - 1 < 10; try_cnt++) {
			msleep(100);
			/* verify the register was written */
			i2c_master_recv(ts->client, read_buf, 1);
			if (read_buf[0] == 72) {
				pr_info("[TSP] success - %c \n", key_val);
				break;
			} else {
				pr_info("[TSP] try again : val : %d , cnt : %d\n", read_buf[0], try_cnt);
				debug_i2c_write(ts->client, write_buf, 2);
			}
		}
		break;
	case 'M':
	case 'm':
		write_buf[0] = ADDR_CHANGE_PROTOCOL;
		write_buf[1] = PTC_STSI_1_0_0;
		debug_i2c_write(ts->client, write_buf, 2);
		debug_on = false;
		break;
	case 'R':
	case 'r':
		toggle_log(ts, LT_RAW_IMG);
		break;
	case 'F':
	case 'f':
		toggle_log(ts, LT_REF_IMG);
		break;
	case 'I':
	case 'i':
		toggle_log(ts, LT_INTENSITY_IMG);
		break;
	case 'G':
	case 'g':
		toggle_log(ts, LT_GROUP_IMG);
		break;
	case 'D':
	case 'd':
		toggle_log(ts, LT_DELAY_IMG);
		break;
	case 'P':
	case 'p':
		toggle_log(ts, LT_POS);
		break;
	case 'B':
	case 'b':
		toggle_log(ts, LT_DEBUG);
		break;
	case 'V':
	case 'v':
		toggle_log(ts, LT_DEBUG2);
		break;
	case 'L':
	case 'l':
		toggle_log(ts, LT_PROFILING);
		break;
	case 'O':
	case 'o':
		pr_info("Enter 'Optimize Delay' mode!!!\n");
		write_buf[0] = ADDR_CHANGE_OPMODE;
		write_buf[1] = OM_OPTIMIZE_DELAY;
		if (!debug_i2c_write(ts->client, write_buf, 2))
			goto ERROR_HANDLE;
		break;
	case 'N':
	case 'n':
		pr_info("Enter 'Normalize Intensity' mode!!!\n");
		write_buf[0] = ADDR_CHANGE_OPMODE;
		write_buf[1] = OM_NORMALIZE_INTENSITY;
		if (!debug_i2c_write(ts->client, write_buf, 2))
			goto ERROR_HANDLE;
		break;
	default:
		;
	}
	return;
ERROR_HANDLE:
	pr_info("ERROR!!! \n");
}

static void print_command_list(void)
{
	int i;
	pr_info("######################################################\n");
	for (i = 0; i < sizeof(tCommandInfo) / sizeof(tCommandInfo_t); i++) {
		pr_info("[%c]: %s\n", tCommandInfo[i].cCommand, tCommandInfo[i].sDescription);
	}
	pr_info("######################################################\n");
}

static void toggle_log(struct melfas_ts_data *ts, eLogType_t _eLogType)
{
	u8 write_buf[2];
	vbLogType[_eLogType] ^= 1;
	if (vbLogType[_eLogType]) {
		write_buf[0] = ADDR_LOGTYPE_ON;
		pr_info("%s ON\n", mcLogTypeName[_eLogType]);
	} else {
		write_buf[0] = ADDR_LOGTYPE_OFF;
		pr_info("%s OFF\n", mcLogTypeName[_eLogType]);
	}
	write_buf[1] = _eLogType;
	debug_i2c_write(ts->client, write_buf, 2);
}

static void logging_function(struct melfas_ts_data *ts)
{
	u8 read_buf[100];
	u8 read_mode, read_num;
	int FingerX, FingerY, FingerID;
	int i;
	static int past_read_mode = HEADER_NONE;
	static char *ps;
	static char s[500];

	debug_i2c_read(ts->client, LOG_READ_ADDR, read_buf, 2);

	read_mode = read_buf[0];
	read_num = read_buf[1];

	//pr_info("[TSP] read_mode : %d,  read_num : %d\n", read_mode, read_num);

	switch (read_mode) {
	case HEADER_U08://Unsigned character
	{
		unsigned char* p = (unsigned char*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num + 2);
		ps = s;
		s[0] = '\0';

		for (i = 0; i < read_num - 1; i++)
		{
			sprintf(ps, "%4d,", p[i]);
			ps = s + strlen(s);
		}
		sprintf(ps, "%4d\n", p[i]);
		ps = s + strlen(s);
		printk(KERN_DEBUG "%s", s);
		break;
	}
	case HEADER_S08://Unsigned character
	{
		signed char* p = (signed char*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num + 2);
		ps = s;
		s[0] = '\0';

		for (i = 0; i < read_num - 1; i++)
		{
			sprintf(ps, "%4d,", p[i]);
			ps = s + strlen(s);
		}
		sprintf(ps, "%4d\n", p[i]);
		ps = s + strlen(s);
		printk(KERN_DEBUG "%s", s);
		break;
	}
	case HEADER_U16://Unsigned short
	{
		unsigned short* p = (unsigned short*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 2 + 2);
		if (past_read_mode != HEADER_U16_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}

		for (i = 0; i < read_num - 1; i++)
		{
			sprintf(ps, "%5d,", p[i]);
			ps = s + strlen(s);
		}
		sprintf(ps, "%5d\n", p[i]);
		ps = s + strlen(s);
		printk(KERN_DEBUG "%s", s);
		break;
	}
	case HEADER_U16_NOCR:
	{
		unsigned short* p = (unsigned short*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 2 + 2);

		if (past_read_mode != HEADER_U16_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}
		for (i = 0; i < read_num; i++)
		{
			sprintf(ps, "%5d,", p[i]);
			ps = s + strlen(s);
		}
		break;
	}
	case HEADER_S16://Unsigned short
	{
		signed short* p = (signed short*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 2 + 2);

		if (past_read_mode != HEADER_S16_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}

		for (i = 0; i < read_num - 1; i++)
		{
			sprintf(ps, "%5d,", p[i]);
			ps = s + strlen(s);
		}
		sprintf(ps, "%5d\n", p[i]);
		ps = s + strlen(s);
		printk(KERN_DEBUG "%s", s);
		break;
	}
	case HEADER_S16_NOCR:
	{
		signed short* p = (signed short*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 2 + 2);

		if (past_read_mode != HEADER_S16_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}
		for (i = 0; i < read_num; i++)
		{
			sprintf(ps, "%5d,", p[i]);
			ps = s + strlen(s);
		}
		break;
	}
	case HEADER_U32://Unsigned short
	{
		unsigned long* p = (unsigned long*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 4 + 4);

		if (past_read_mode != HEADER_U32_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}

		for (i = 0; i < read_num - 1; i++)
		{
			sprintf(ps, "%10ld,", p[i]);
			ps = s + strlen(s);
		}
		sprintf(ps, "%10ld\n", p[i]);
		ps = s + strlen(s);
		printk(KERN_DEBUG "%s", s);
		break;
	}
	case HEADER_U32_NOCR://Unsigned short
	{
		unsigned long* p = (unsigned long*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 4 + 4);

		if (past_read_mode != HEADER_U32_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}
		for (i = 0; i < read_num; i++)
		{
			sprintf(ps, "%10ld,", p[i]);
			ps = s + strlen(s);
		}
		break;
	}
	case HEADER_S32://Unsigned short
	{
		signed long* p = (signed long*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 4 + 4);

		if (past_read_mode != HEADER_S32_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}

		for (i = 0; i < read_num - 1; i++)
		{
			sprintf(ps, "%10ld,", p[i]);
			ps = s + strlen(s);
		}
		sprintf(ps, "%10ld\n", p[i]);
		ps = s + strlen(s);
		printk(KERN_DEBUG "%s", s);
		break;
	}
	case HEADER_S32_NOCR://Unsigned short
	{
		signed long* p = (signed long*) &read_buf[2];
		i2c_master_recv(ts->client, read_buf, read_num * 4 + 4);

		if (past_read_mode != HEADER_S32_NOCR)
		{
			ps = s;
			s[0] = '\0';
		}
		for (i = 0; i < read_num; i++)
		{
			sprintf(ps, "%10ld,", p[i]);
			ps = s + strlen(s);
		}
		break;
	}
	case HEADER_TEXT://Text
		i2c_master_recv(ts->client, read_buf, read_num + 2);

		ps = s;
		s[0] = '\0';

		for (i = 2; i < read_num + 2; i++)
		{
			sprintf(ps, "%c", read_buf[i]);
			ps = s + strlen(s);
		}
		printk(KERN_DEBUG "%s\n", s);
		break;
	case HEADER_FINGER:
		i2c_master_recv(ts->client, read_buf, read_num * 4 + 2);

		ps = s;
		s[0] = '\0';
		for (i = 2; i < read_num * 4 + 2; i = i + 4)
		{
			//log_printf( device_idx, " %5ld", read_buf[i]  , 0,0);
			FingerX = (read_buf[i + 1] & 0x07) << 8 | read_buf[i];
			FingerY = (read_buf[i + 3] & 0x07) << 8 | read_buf[i + 2];

			FingerID = (read_buf[i + 1] & 0xF8) >> 3;
			sprintf(ps, "%2d (%4d,%4d) | ", FingerID, FingerX, FingerY);
			ps = s + strlen(s);
		}
		printk(KERN_DEBUG "%s\n", s);
		break;
	default:
		break;
	}

	past_read_mode = read_mode;
}
#endif /* DEBUG_MODE */

static int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];

	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;

	if  (i2c_transfer(adapter, msg, 2) == 2)
		return 0;
	else
		return -EIO;

}

static void release_all_fingers(struct melfas_ts_data *ts)
{
	int i;
	for(i=0; i<P5_MAX_TOUCH; i++) {
		if(-1 == g_Mtouch_info[i].strength) {
			g_Mtouch_info[i].posX = 0;
			g_Mtouch_info[i].posY = 0;
			continue;
		}

		g_Mtouch_info[i].strength = 0;

		REPORT_MT(i, g_Mtouch_info[i].posX,
				g_Mtouch_info[i].posY, g_Mtouch_info[i].strength);

		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;

		if(0 == g_Mtouch_info[i].strength)
			g_Mtouch_info[i].strength = -1;
	}
}

static int read_input_info(struct melfas_ts_data *ts, u8 *val)
{
	return melfas_i2c_read(ts->client, TS_READ_START_ADDR, TS_READ_REGS_LEN, val);
}

static int check_firmware(struct melfas_ts_data *ts, u8 *val)
{
	return melfas_i2c_read(ts->client, MCSTS_FIRMWARE_VER_REG_MASTER, 1, val);
}

static int check_firmware_slave(struct melfas_ts_data *ts, u8 *val)
{
	return melfas_i2c_read(ts->client, MCSTS_FIRMWARE_VER_REG_SLAVE, 1, val);
}

static int check_slave_boot(struct melfas_ts_data *ts, u8 *val)
{
	return melfas_i2c_read(ts->client, 0xb1, 1, val);
}

static int firmware_update(struct melfas_ts_data *ts)
{
	int ret =0;
	int i =0;

#ifdef SET_DOWNLOAD_BY_GPIO
	/* enable gpio */
#ifdef CONFIG_ARCH_TEGRA
	tegra_gpio_enable(ts->gpio_sda);
	tegra_gpio_enable(ts->gpio_scl);
#endif
	gpio_request(ts->gpio_sda, "TSP_SDA");
	gpio_request(ts->gpio_scl, "TSP_SCL");

	msleep(5000);

	for(i=0; i<DOWNLOAD_RETRY_CNT;i++) {
		ret = mcsdl_download_binary_data();

		// Check download result
		if (ret)
			pr_err("[TSP] SET Download Fail - error code [%d]\n", ret);
		else
			break;
	}

	gpio_free(ts->gpio_sda);
	gpio_free(ts->gpio_scl);

	/* disable gpio */
#ifdef CONFIG_ARCH_TEGRA
	tegra_gpio_disable(ts->gpio_sda);
	tegra_gpio_disable(ts->gpio_scl);
#endif
#endif /* SET_DOWNLOAD_BY_GPIO */
	msleep(100);

	/* reset chip */
	ts->power_enable(0);
	msleep(200);

	ts->power_enable(1);
	msleep(100);

	return 0;

}

static void melfas_ts_read_input(struct melfas_ts_data *ts)
{
	int ret = 0, i;
	u8 buf[TS_READ_REGS_LEN];
	int touchType=0, touchState =0, touchID=0, posX=0, posY=0, strength=0, keyID = 0, reportID = 0;

#ifdef DEBUG_MODE
	if (debug_on) {
		logging_function(ts);
		return ;
	}
#endif

	ret = read_input_info(ts, buf);
	if (ret < 0) {
		pr_err("[TSP] Failed to read the touch info\n");
		return ;
	}
	else{
		touchType  = (buf[0]>>6)&0x03;
		touchState = (buf[0]>>4)&0x01;
		reportID = (buf[0]&0x0f);
		posX = ((buf[1]& 0x0F) << 8) | buf[2];
		posY = ((buf[1]& 0xF0) << 4) | buf[3];
		keyID = strength = buf[4];

		if(reportID == 0x0f ) { // ESD Detection
			pr_info("[TSP] MELFAS_ESD Detection");

			release_all_fingers(ts);
			ts->power_enable(0);

			msleep(700);
			ts->power_enable(1);
			return ;
		}
/*		else if(reportID == 0x0E)// Plam
		{
			pr_info("[TSP] MELFAS_Plam");
			release_all_fingers(ts);
			return ;
		}
*/

		touchID = reportID-1;

		if (debug_print)
			pr_info("[TSP] reportID: %d\n", reportID);

		if(reportID > P5_MAX_TOUCH || reportID < 1) {
			pr_err("[TSP] Invalid touch id.\n");
			release_all_fingers(ts);
			return ;
		}

		if(touchType == TOUCH_SCREEN) {
			g_Mtouch_info[touchID].posX= posX;
			g_Mtouch_info[touchID].posY= posY;

			if(touchState) {
#ifdef CONFIG_KERNEL_DEBUG_SEC
				if (0 >= g_Mtouch_info[touchID].strength)
					pr_info("[TSP] Press    - ID : %d  [%d,%d] WIDTH : %d",
						touchID,
						g_Mtouch_info[touchID].posX,
						g_Mtouch_info[touchID].posY,
						strength);
#endif
				g_Mtouch_info[touchID].strength= strength;
			} else {
#ifdef CONFIG_KERNEL_DEBUG_SEC
				if (g_Mtouch_info[touchID].strength)
					pr_info("[TSP] Release - ID : %d [%d,%d]",
						touchID,
						g_Mtouch_info[touchID].posX,
						g_Mtouch_info[touchID].posY);
#endif
				g_Mtouch_info[touchID].strength = 0;
			}

			for(i=0; i<P5_MAX_TOUCH; i++) {
				if(g_Mtouch_info[i].strength== -1)
					continue;

				REPORT_MT(i, g_Mtouch_info[i].posX,
						g_Mtouch_info[i].posY, g_Mtouch_info[i].strength);

				if (debug_print)
					pr_info("[TSP] Touch ID: %d, State : %d, x: %d, y: %d, z: %d\n",
						i, touchState, g_Mtouch_info[i].posX,
						g_Mtouch_info[i].posY, g_Mtouch_info[i].strength);

				if(g_Mtouch_info[i].strength == 0)
					g_Mtouch_info[i].strength = -1;
			}
		}
#ifdef TS_TOUCH_KEY
		else if(touchType == TOUCH_KEY)
		{
			if (keyID == 0x1)
				input_report_key(ts->input_dev, KEY_MENU, touchState ? PRESS_KEY : RELEASE_KEY);
			if (keyID == 0x2)
				input_report_key(ts->input_dev, KEY_HOME, touchState ? PRESS_KEY : RELEASE_KEY);
			if (keyID == 0x4)
				input_report_key(ts->input_dev, KEY_BACK, touchState ? PRESS_KEY : RELEASE_KEY);

#ifdef CONFIG_KERNEL_DEBUG_SEC
			pr_info("[TSP] keyID : %d, touchState: %d\n", keyID, touchState);
#endif
		}
#endif
		input_sync(ts->input_dev);
	}
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;
	if (debug_print)
		pr_info("[TSP] %s\n", __func__);

	melfas_ts_read_input(ts);
	return IRQ_HANDLED;
}

static ssize_t show_firmware_dev(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	u8 ver = 0;

	check_firmware(ts, &ver);

	return snprintf(buf, PAGE_SIZE,	"MEL_%dx%d\n", ver, 0x0);
}

static ssize_t store_firmware(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int i ;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

#ifdef FW_FROM_FILE
	remove_i2c_driver();
#endif

	firmware_update(ts);

	return count;
}

static ssize_t show_firmware_bin(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	int ver =	BINARY_FIRMWARE_VERSION;

	return snprintf(buf, PAGE_SIZE,	"MEL_%dx%d\n", ver, 0x0);
}

#if 0
static ssize_t show_reg(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	u8 val[0x5];
	int ret = 0;

	ret = melfas_i2c_read(ts->client, 0, 5, &val);

	pr_info("[TSP] status : %u", val[0]);
	pr_info("[TSP] ctl : %u", val[1]);
	pr_info("[TSP] resolution x : %u", val[3] | (val[2] & 0x0f) << 8);
	pr_info("[TSP] resolution y : %u", val[4] | (val[2] & 0xf0) << 4);

	return snprintf(buf, PAGE_SIZE,	"%d\n", ret);
}
#endif

static ssize_t store_debug_mode(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char ch;

	if (sscanf(buf, "%c", &ch) != 1)
		return -EINVAL;

	key_handler(ts, ch);

	return count;
}

static ssize_t show_debug_mode(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return sprintf(buf, debug_on ? "ON\n" : "OFF\n");
}

static ssize_t store_debug_log(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t count)
{
	int i;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

	if (i)
		debug_print = true;
	else
		debug_print = false;

	return count;
}

static DEVICE_ATTR(fw_dev, S_IWUSR|S_IRUGO, show_firmware_dev, store_firmware);
static DEVICE_ATTR(fw_bin, S_IRUGO, show_firmware_bin, NULL);
/* static DEVICE_ATTR(debug_reg, S_IWUSR|S_IRUGO, show_reg, NULL); */
static DEVICE_ATTR(debug_mode, S_IWUSR|S_IRUGO, show_debug_mode, store_debug_mode);
static DEVICE_ATTR(debug_log, S_IWUSR|S_IRUGO, NULL, store_debug_log);

static struct attribute *maxTouch_attributes[] = {
	&dev_attr_fw_dev.attr,
	&dev_attr_fw_bin.attr,
/*	&dev_attr_debug_reg.attr, */
	&dev_attr_debug_mode.attr,
	&dev_attr_debug_log.attr,
	NULL,
};

static struct attribute_group maxtouch_attr_group = {
	.attrs = maxTouch_attributes,
};

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	struct device *tsp_dev;
	struct input_dev *input;
	bool empty_chip = false;
	u8 val;
	u8 val_slv;
	int ret 	= 0;
	int ret_slv = 0;
	int i 		= 0;
	int irq 	= 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[TSP] melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("[TSP] %s: failed to create a state of melfas-ts\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_failed;
	}

	ts->pdata = client->dev.platform_data;
	if (ts->pdata->power_enable)
		ts->power_enable = ts->pdata->power_enable;

#ifdef SET_DOWNLOAD_BY_GPIO
	ts->gpio_scl = ts->pdata->gpio_scl;
	ts->gpio_sda = ts->pdata->gpio_sda;
#endif

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = check_firmware(ts, &val);
	ret_slv = check_firmware_slave(ts, &val_slv);

	if (ret || ret_slv) {
		empty_chip = true;
		pr_err("[TSP] Failed to check firmware version : %d , %d", ret, ret_slv);
	} else {
		pr_info("[TSP] firmware version : 0x%x , 0x%x\n", val, val_slv);
		pr_info("[TSP] new version : 0x%x\n", BINARY_FIRMWARE_VERSION);
	}

	/* (current version  < binary verion) */
	if (val != val_slv || val < BINARY_FIRMWARE_VERSION || empty_chip)
		firmware_update(ts);

	ret = check_slave_boot(ts, &val);
	if (ret)
		pr_err("[TSP] Failed to check slave boot : %d", ret);
	else
		pr_info("[TSP] SLVAE Active Ret [%d]", val);

	input = input_allocate_device();
	if (!input) {
		pr_err("[TSP] %s: failed to allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_failed;
	}

	ts->input_dev = input;

#ifdef CONFIG_SAMSUNG_INPUT
	input->name = "sec_touchscreen";
#else
	input->name = client->name;
#endif

	set_bit(EV_ABS,  input->evbit);
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->pdata->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->pdata->max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, ts->pdata->max_pressure, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, ts->pdata->max_width, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, P5_MAX_TOUCH-1, 0, 0);

#ifdef TS_TOUCH_KEY
	input_set_capability(input, EV_KEY, KEY_MENU);
	input_set_capability(input, EV_KEY, KEY_HOME);
	input_set_capability(input, EV_KEY, KEY_BACK);
#endif

	ret = input_register_device(input);
	if (ret) {
		pr_err("[TSP] %s: failed to register input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_register_device_failed;
	}

	if (client->irq) {
		irq = client->irq;

		ret = request_threaded_irq(irq, NULL, melfas_ts_irq_handler,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					ts->client->name, ts);
		if (ret) {
			pr_err("[TSP] %s: Can't allocate irq %d, ret %d\n", __func__, irq, ret);
			ret = -EBUSY;
			goto err_request_irq;
		}
	}

	for (i = 0; i < P5_MAX_TOUCH ; i++)  /* _SUPPORT_MULTITOUCH_ */
		g_Mtouch_info[i].strength = -1;

#ifdef CONFIG_SAMSUNG_INPUT
	tsp_dev  = device_create(sec_class, NULL, 0, ts, "sec_touch");
	if (IS_ERR(tsp_dev))
		pr_err("[TSP] Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&tsp_dev->kobj, &maxtouch_attr_group);
	if (ret)
		pr_err("[TSP] Failed to create sysfs group\n");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	return 0;

err_request_irq:
	free_irq(client->irq, ts);
err_input_register_device_failed:
	input_free_device(input);
err_alloc_failed:
	kfree(ts);
err_check_functionality_failed:
	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	pr_warning("[TSP] %s\n", __func__);

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts = container_of(h,
		struct melfas_ts_data, early_suspend);
#else
static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
#endif

	pr_info("[TSP] %s\n", __func__);

	release_all_fingers(ts);

	disable_irq(ts->client->irq);

	if (ts->power_enable)
		ts->power_enable(0);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts = container_of(h,
		struct melfas_ts_data, early_suspend);
#else
static int melfas_ts_resume(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
#endif

	pr_info("[TSP] %s\n", __func__);

	if (ts->power_enable)
		ts->power_enable(1);

	enable_irq(ts->client->irq);
}

static void melfas_ts_shutdown(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	ts->power_enable(0);
#ifdef CONFIG_ARCH_TEGRA
	gpio_direction_output(GPIO_TOUCH_INT, 0);
#endif
}

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_TS_NAME, 0 },
	{ }
};


static struct i2c_driver melfas_ts_driver = {
	.driver		= {
		.name	= MELFAS_TS_NAME,
	},
	.id_table		= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= __devexit_p (melfas_ts_remove),
	.shutdown	= melfas_ts_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend		= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
}

#ifdef FW_FROM_FILE
static void remove_i2c_driver(void)
{
	i2c_del_driver(&melfas_ts_driver);
}
#endif

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
