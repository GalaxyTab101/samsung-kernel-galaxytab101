/*
 * arch/arm/mach-tegra/board-touch-pangit_i2c.c
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
#include <linux/i2c/panjit_ts.h>

#if defined (CONFIG_MACH_CARDHU)
#include "board-cardhu.h"
#endif

#if defined (CONFIG_MACH_VENTANA)
#include "board-ventana.h"
#endif

#include "gpio-names.h"
#include "touch.h"

static struct panjit_i2c_ts_platform_data panjit_data = {
	.gpio_reset = TOUCH_GPIO_RST_PANJIT,
};

static const struct i2c_board_info panjit_i2c_info[] = {
	{
	 I2C_BOARD_INFO("panjit_touch", 0x3),
	 .irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ_PANJIT),
	 .platform_data = &panjit_data,
	 },
};

struct tegra_touchscreen_init panjit_init_data = {
	.irq_gpio = TOUCH_GPIO_IRQ_PANJIT,	/* GPIO1 Value for IRQ */
	.rst_gpio = TOUCH_GPIO_RST_PANJIT,	/* GPIO2 Value for RST */
	.sv_gpio1 = {0},			/* Valid, GPIOx, Set value, Delay      */
	.sv_gpio2 = {0},			/* Valid, GPIOx, Set value, Delay      */
	.ts_boardinfo = {0, panjit_i2c_info, 1}	/* BusNum, BoardInfo, Value     */
};

