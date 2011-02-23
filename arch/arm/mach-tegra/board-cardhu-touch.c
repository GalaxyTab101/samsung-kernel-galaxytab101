/*
 * arch/arm/mach-tegra/board-cardhu-touch.c
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
#include "board-cardhu.h"
#include "gpio-names.h"
#include "touch.h"

#define SKU_MASK	0xFF00
#define UNKNOWN_SKU	0XFFFF


int generic_touch_init(struct tegra_touchscreen_init *tsdata)
{
	int ret;
/*	pr_info("### TOUCHSCREEN:  Inside generic_touch_init()\n");	*/
	ret = gpio_request(tsdata->rst_gpio, "touch-reset");
	if (ret < 0) {
		pr_err("%s(): gpio_request() fails for gpio %d (touch-reset)\n",
						__func__, tsdata->rst_gpio);
		return ret;
	}

	ret = gpio_request(tsdata->irq_gpio, "touch-irq");
	if (ret < 0) {
		pr_err("%s(): gpio_request() fails for gpio %d (touch-irq)\n",
						__func__, tsdata->irq_gpio);
		gpio_free(tsdata->rst_gpio);
		return ret;
	}

	tegra_gpio_enable(tsdata->irq_gpio);
	tegra_gpio_enable(tsdata->rst_gpio);
	gpio_direction_output(tsdata->rst_gpio, 1);
	gpio_direction_input(tsdata->irq_gpio);
/*
	FIXME!! Avoiding manual reset of touch panel
	if (tsdata->sv_gpio1.valid)
		gpio_set_value(tsdata->sv_gpio1.gpio, tsdata->sv_gpio1.value);
	if (tsdata->sv_gpio1.delay)
		msleep(tsdata->sv_gpio1.delay);
	if (tsdata->sv_gpio2.valid)
		gpio_set_value(tsdata->sv_gpio2.gpio, tsdata->sv_gpio2.value);
	if (tsdata->sv_gpio2.delay)
		msleep(tsdata->sv_gpio2.delay);
*/
	i2c_register_board_info(tsdata->ts_boardinfo.busnum,
		tsdata->ts_boardinfo.info,
		tsdata->ts_boardinfo.n);
	return 0;
}

int __init cardhu_touch_init(void)
{
	int err = 0;
#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
	err = generic_touch_init(&panjit_init_data);
	if (err)
		return err;
#endif
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
	err = generic_touch_init(&atmel_mxt_init_data);
	if (err)
		return err;
#endif
	return err;
}
