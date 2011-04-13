/*
 * arch/arm/mach-tegra/tegra_reg_access.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <mach/iomap.h>

static struct miscdevice dev;

static int tegra_reg_access_open(struct inode *inode, struct file *filp)
{
	int ret;

	ret = nonseekable_open(inode, filp);
	if (unlikely(ret))
		return ret;
	return 0;
}

static int tegra_reg_access_release(struct inode *inode, struct file *filp)
{
	return 0;
}

#define TEGRA_REG_WRITE 0
#define TEGRA_REG_READ 1

static long tegra_reg_access_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/*
	 * For Reg Read,
	 * arg[0] = phys address, arg[1] = returns value read.
	 * For Reg Write,
	 * arg[0] = phys address, arg[1] = value to write.
	 */
	unsigned long *uarg = (unsigned long *)arg;
	void __iomem *reg_addr = IO_ADDRESS(uarg[0]);

	if (reg_addr == NULL)
		return -EINVAL;

	switch (cmd) {
	case TEGRA_REG_READ:
		uarg[1] = readl(reg_addr);
		break;
	case TEGRA_REG_WRITE:
		writel(uarg[1], reg_addr);
		break;
	default:
		pr_err("\n tegra_reg_access unknown ioctl cmd=%d\n", cmd);
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations tegra_reg_access_fops = {
	.owner		= THIS_MODULE,
	.open		= tegra_reg_access_open,
	.release	= tegra_reg_access_release,
	.unlocked_ioctl	= tegra_reg_access_ioctl,
};

static int __devinit tegra_reg_access_init(void)
{
	int e;

	dev.minor = MISC_DYNAMIC_MINOR;
	dev.name = "tegra_reg_access";
	dev.fops = &tegra_reg_access_fops;
	dev.parent = NULL;

	e = misc_register(&dev);
	if (e) {
		pr_err("\nunable to register tegra_reg_access dev\n");
		return -ENODEV;
	}
	pr_warn("## WARNING: Tegra registers access, from user space, is enabled.\n"
		"## This creates a hole in the system security.\n"
		"## Ensure that this is intentional.\n");
	return 0;
}

static void __exit tegra_reg_access_exit(void)
{
	misc_deregister(&dev);
}

module_init(tegra_reg_access_init);
module_exit(tegra_reg_access_exit);
