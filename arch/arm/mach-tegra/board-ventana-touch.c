/*
 * arch/arm/mach-tegra/board-ventana-touch.c
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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
#include <linux/i2c/panjit_ts.h>
extern struct tegra_touchscreen_init panjit_init_data;
#define PANJIT_TOUCHSCREEN_SKU	0x0000
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
#include <linux/i2c/atmel_maxtouch.h>
extern struct tegra_touchscreen_init atmel_mxt_init_data;
#define ATMEL_TOUCHSCREEN_SKU	0x0A00
#define ATMEL_TOUCHSCREEN_T25	0x0B00
#endif

#include "board.h"
#include "board-ventana.h"
#include "gpio-names.h"
#include "touch.h"

#define SKU_MASK	0xFF00
#define UNKNOWN_SKU	0XFFFF


int generic_touch_init(struct tegra_touchscreen_init *tsdata)
{
/*	pr_info("### TOUCHSCREEN:  Inside generic_touch_init()\n");	*/
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

int __init ventana_touch_init(void)
{
	int retval = 0;
	struct board_info BoardInfo;

	tegra_get_board_info(&BoardInfo);

	switch(BoardInfo.sku & SKU_MASK) {
#if defined (CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
		case ATMEL_TOUCHSCREEN_SKU :
		case ATMEL_TOUCHSCREEN_T25 :
			retval = generic_touch_init(&atmel_mxt_init_data);
			break;
#endif
#if defined (CONFIG_TOUCHSCREEN_PANJIT_I2C)
		case PANJIT_TOUCHSCREEN_SKU :
			retval = generic_touch_init(&panjit_init_data);
			break;
#endif

		case UNKNOWN_SKU :
			pr_info("*** ERROR ***\n");
			pr_info("    Invalid BoardInfo EEPROM.  ");
			pr_info("    BoardInfo.sku is programmed with 0xFFFF.\n");
			pr_info("    No touch screen support.\n");
			break;

		default :
			pr_info("*** ERROR ***\n");
			pr_info("    Invalid BoardInfo EEPROM.  ");
			pr_info("    BoardInfo.sku contains unknown SKU: %04X\n", BoardInfo.sku);
			pr_info("    No touch screen support.\n");
			break;
	}

	return retval;
}
