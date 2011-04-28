/*
 * arch/arm/mach-tegra/audio_jack.c
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

#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <mach/gpio.h>
#include <mach/audio.h>

#include "gpio-names.h"

extern struct wired_jack_conf audio_wr_jack_conf;

static struct platform_device audio_hs_jack_device = {
	.name = "tegra_wired_jack",
	.id = -1,
	.dev = {
		.platform_data = &audio_wr_jack_conf,
	},
};

int __init audio_wired_jack_init(void)
{
	int ret;

	tegra_gpio_enable(audio_wr_jack_conf.hp_det_n);

	ret = platform_device_register(&audio_hs_jack_device);
	return ret;
}
