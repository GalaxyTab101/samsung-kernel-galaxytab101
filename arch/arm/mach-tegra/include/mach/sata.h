/*
 * arch/arm/mach-tegra/include/mach/sata.h
 *
 * SATA driver platform data definitions
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#ifndef __MACH_TEGRA_SATA_H
#define __MACH_TEGRA_SATA_H

struct tegra_sata_platform_data {
	void *clk_sata;			/* to store the clk_sata */
	void *clk_sata_oob;		/* to store the clk_sata_oob */
	void __iomem *bars_table[6];	/* virtual bar table: this is to store
					 * AHCI_BAR5 address */
};

#endif
