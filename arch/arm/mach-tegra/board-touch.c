/*
 * arch/arm/mach-tegra/board-touch.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define VERBOSE_DEBUG	1	/* When set, emit verbose debug messages.	*/
#define MULTI_SKU	1	/* When set, multi-driver detection is enabled.	*/
				/* When clear, DEFAULT_SKU is used.		*/

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "board.h"
#ifdef CONFIG_MACH_VENTANA
#include "board-ventana.h"
#elif CONFIG_MACH_CARDHU
#include "board-cardhu.h"
#endif
#include "gpio-names.h"
#include "touch.h"

#define SKU_MASK	0xFF00	/* Mask BoardInfo before testing sku.	*/
#define UNKNOWN_SKU	0xFF00	/* When BoardInfo not programmed...	*/


/****************************************************************************
	This touch screen driver interface is a generic interface, designed
	to support any number of touch screen devices.  It is also designed to
	allow multiple touchscreen drivers and dynamic detection of the touch
	screen.  The touch screen is detected by reading the BoardInfo EEPROM
	structure.  The 'sku' field indicates which touch screen is implemented.

	In the event that the BoardInfo EEPROM is not programmed, this interface
	design allows for a default driver to be loaded anyways.

	In order to support multi-driver implementations, each touch screen must
	register itself in this file, and implement a few basic items.  The
	basic implementation is as follows:

#if defined(CONFIG_TOUCHSCREEN_YOUR_TOUCHSCREEN_DRIVER_NAME)
	#include <your_touch.h>
	extern struct tegra_touchscreen_init ts_init_data;
	#define YOUR_TOUCHSCREEN_SKU1	VALUE1	// This is the value in BoardInfo.sku
	#define YOUR_TOUCHSCREEN_SKU2	VALUE2	// Another value from BoardInfo.sku
	#define YOUR_TOUCHSCREEN_SKU-N	VALUE-N	// Yet another BoardInfo.sku value
	#if defined(DEFAULT_SKU) && (MULTI_SKU == 1) //Make  sure only one SKU is defined.
	#if defined(DEFAULT_SKU) && (MULTI_SKU == 0)
	#error ERROR!  When MULTI_SKU=0, only one touch screen driver is supported at a time.\
	Check your _defconfig file and remove all but one touch screen driver instances.
	#else
	#ifndef DEFAULT_SKU
	#define DEFAULT_SKU		FAV_SKU	// When MULTI_SKU=0, default to this SKU
	#endif
	#endif
#endif
****************************************************************************/


/*** PANJIT Support ********************************************************/
#if defined(CONFIG_TOUCHSCREEN_PANJIT_I2C)
#include <linux/i2c/panjit_ts.h>
extern struct tegra_touchscreen_init panjit_init_data;
#define PANJIT_TOUCHSCREEN_SKU	0x0000
#if defined(DEFAULT_SKU) && (MULTI_SKU == 0)
#error ERROR!  When MULTI_SKU=0, only one touch screen driver is supported at a time.\
Check your _defconfig file and remove all but one touch screen driver instances.
#else
#ifndef DEFAULT_SKU
#define DEFAULT_SKU		PANJIT_TOUCHSCREEN_SKU
#endif
#endif
#endif

/*** ATMEL Multitouch Support **********************************************/
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
#include <linux/i2c/atmel_maxtouch.h>
extern struct tegra_touchscreen_init atmel_mxt_init_data;
#define ATMEL_TOUCHSCREEN_SKU	0x0A00
#define ATMEL_TOUCHSCREEN_T25	0x0B00
#if defined(DEFAULT_SKU) && (MULTI_SKU == 0)
#error ERROR!  When MULTI_SKU=0, only one touch screen driver is supported at a time.\
Check your _defconfig file and remove all but one touch screen driver instances.
#else
#ifndef DEFAULT_SKU
#define DEFAULT_SKU		ATMEL_TOUCHSCREEN_SKU
#endif
#endif
#endif


int generic_touch_init(struct tegra_touchscreen_init *tsdata)
{
	if (VERBOSE_DEBUG >= 1)
		pr_info("### TOUCHSCREEN:  Inside generic_touch_init()\n");
	tegra_gpio_enable(tsdata->irq_gpio);
	tegra_gpio_enable(tsdata->rst_gpio);
	if (tsdata->sv_gpio1.valid)
		gpio_set_value(tsdata->sv_gpio1.gpio, tsdata->sv_gpio1.value);
	if (tsdata->sv_gpio1.delay)
		msleep(tsdata->sv_gpio1.delay);
	if (tsdata->sv_gpio2.valid)
		gpio_set_value(tsdata->sv_gpio2.gpio, tsdata->sv_gpio2.value);
	if (tsdata->sv_gpio2.delay)
		msleep(tsdata->sv_gpio2.delay);
	i2c_register_board_info(tsdata->ts_boardinfo.busnum,
		tsdata->ts_boardinfo.info,
		tsdata->ts_boardinfo.n);
	return 0;
}

int __init touch_init(void)
{
	int retval = 0;
	int sku = 0xFFFF;
	struct board_info BoardInfo;

	if (VERBOSE_DEBUG >= 1)
		pr_info("### Touch Init...\n");

	tegra_get_board_info(&BoardInfo);

	if (VERBOSE_DEBUG >= 1)
		pr_info("### BoardInfo.sku = %04X\n", BoardInfo.sku);

	if (MULTI_SKU)
		sku = BoardInfo.sku & SKU_MASK;
	else
		sku = DEFAULT_SKU;

	if (VERBOSE_DEBUG >= 1)
		pr_info("### sku = %04X\n", sku);

	switch (sku) {
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
	case ATMEL_TOUCHSCREEN_SKU:
	case ATMEL_TOUCHSCREEN_T25:
		retval = generic_touch_init(&atmel_mxt_init_data);
		break;
#endif
#if defined(CONFIG_TOUCHSCREEN_PANJIT_I2C)
	case PANJIT_TOUCHSCREEN_SKU:
		retval = generic_touch_init(&panjit_init_data);
		break;
#endif

	case UNKNOWN_SKU:
		pr_info("*** ERROR ***\n");
		pr_info("    Invalid BoardInfo EEPROM.  ");
		pr_info("    BoardInfo.sku is programmed with 0xFFFF.\n");
		pr_info("    No touch screen support.\n");
		break;

	default:
		pr_info("*** ERROR ***\n");
		pr_info("    Invalid BoardInfo EEPROM.  ");
		pr_info("    BoardInfo.sku contains unknown SKU: %04X\n", BoardInfo.sku);
		pr_info("    No touch screen support.\n");
		break;
	}

	return retval;
}
