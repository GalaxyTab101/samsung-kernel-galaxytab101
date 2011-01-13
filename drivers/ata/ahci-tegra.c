/*
 *  ahci-tegra.c - AHCI SATA support for TEGRA AHCI device
 *
 *  Copyright (c) 2011, NVIDIA Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 * libata documentation is available via 'make {ps|pdf}docs',
 * as Documentation/DocBook/libata.*
 *
 * AHCI hardware documentation:
 * http://www.intel.com/technology/serialata/pdf/rev1_0.pdf
 * http://www.intel.com/technology/serialata/pdf/rev1_1.pdf
 *
 */

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/dmi.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_cmnd.h>
#include <linux/libata.h>
#include "ahci.h"

#include <mach/sata.h>
#include <linux/clk.h>
#include <mach/clk.h>

#include "mach/iomap.h"
#include "mach/io.h"


#define DRV_NAME	"tegra-sata"
#define DRV_VERSION	"1.0"

#define TEGRA_AHCI_IDLE_TIMEOUT 2000	/* idle timeout for PM in msec */

/* Bit 0 (EN_FPCI) to allow FPCI accesses to SATA */
#define SATA_CONFIGURATION_0_OFFSET			0x180
#define EN_FPCI						(1 << 0)
#define SATA_INTR_MASK_0_OFFSET				0x188
#define IP_INT_MASK					(1 << 16)

/* Need to write 0x00400200 to 0x70020094 */
#define SATA_FPCI_BAR5_0_OFFSET				0x094
#define PRI_ICTLR_CPU_IER_SET_0_OFFSET			0x024
#define CPU_IER_SATA_CTL				(1 << 23)

/* SATA related private defines */
#define TEGRA_PRIVATE_IDDQ_OFFSET			0x120
#define TEGRA_PRIVATE_IDDQ_BIT1				(1 << 5)
#define TEGRA_PRIVATE_IDDQ_BIT2				(1 << 6)
#define AHCI_BAR5_CONFIG_LOCATION			0x24

/* AHCI config space defines */
#define TEGRA_PRIVATE_AHCI_CC_BKDR			0x4A4
#define TEGRA_PRIVATE_AHCI_CC_BKDR_OVERRIDE		0x54C
#define TEGRA_PRIVATE_AHCI_CC_BKDR_OVERRIDE_EN		(1 << 12)
#define TEGRA_PRIVATE_AHCI_CC_BKDR_PGM			0x01060100

/* AHCI HBA_CAP */
#define TEGRA_PRIVATE_AHCI_CAP_BKDR			0xA0

/* other defines */
#define TEGRA_SATA_BAR5_INIT_PROGRAM			0xFFFFFFFF
#define TEGRA_SATA_BAR5_FINAL_PROGRAM			0x40020000

#define SSTAT_IPM_STATE_MASK				0xF00
#define SSTAT_IPM_SLUMBER_STATE				0x600

#define TEGRA_SATA_IO_SPACE_OFFSET			4
#define TEGRA_SATA_ENABLE_IO_SPACE			(1 << 0)
#define TEGRA_SATA_ENABLE_MEM_SPACE			(1 << 1)
#define TEGRA_SATA_ENABLE_BUS_MASTER			(1 << 2)
#define TEGRA_SATA_ENABLE_SERR				(1 << 8)

#define TEGRA_SATA_CORE_CLOCK_FREQ_HZ			(108*1000*1000)
#define TEGRA_SATA_OOB_CLOCK_FREQ_HZ			(216*1000*1000)

enum {
	AHCI_PCI_BAR = 5,
};

static int tegra_ahci_init_one(struct platform_device *pdev);
static int tegra_ahci_remove_one(struct platform_device *pdev);

#ifdef CONFIG_PM
static int tegra_ahci_suspend(struct platform_device *pdev, pm_message_t mesg);
static int tegra_ahci_resume(struct platform_device *pdev);
static void tegra_ahci_idle_timer(unsigned long arg);
static u8 tegra_ahci_is_port_idle(struct ata_port *ap);
static u8 tegra_ahci_is_port_slumber(struct ata_port *ap);
static unsigned int tegra_ahci_qc_issue(struct ata_queued_cmd *qc);
#endif

/*  tegra_ahci_host_priv is the extension of ahci_host_priv
 *  with 3 extra fields: idle_timer, pg_save, pg_state.
 */
struct tegra_ahci_host_priv {
	struct ahci_host_priv	ahci_host_priv;
	struct timer_list	idle_timer;
	void			*pg_save;
	u32			pg_state;
};

static struct scsi_host_template ahci_sht = {
	AHCI_SHT("tegra-data"),
};

static struct ata_port_operations tegra_ahci_ops = {
	.inherits	= &ahci_ops,
#ifdef CONFIG_PM
	.qc_issue	= tegra_ahci_qc_issue,
#endif
};

static const struct ata_port_info ahci_port_info = {
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= 0x1f, /* pio0-4 */
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &tegra_ahci_ops,
};

static struct platform_driver tegra_platform_ahci_driver = {
	.probe		= tegra_ahci_init_one,
	.remove		= tegra_ahci_remove_one,
#ifdef CONFIG_PM
	.suspend	= tegra_ahci_suspend,
	.resume		= tegra_ahci_resume,
#endif
	.driver = {
		.name = DRV_NAME,
	}
};

static int tegra_ahci_controller_init(struct platform_device *pdev)
{
	void __iomem *mmio, *sata_config;
	struct tegra_sata_platform_data *pdata;
	struct resource *res0, *res1;
	int err = 0;
	unsigned long clk_rate;
	struct clk *clk_sata = NULL;
	struct clk *clk_sata_oob = NULL;
	void __iomem *virt;
	u32 temp;

	/* grab platform specific data*/
	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		return -ENODEV;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (!res0 || !res1)
		return -ENOMEM;

	/* AHCI bar */
	mmio = devm_ioremap(&pdev->dev, res0->start, (res0->end-res0->start+1));
	/* AHCI config */
	sata_config = devm_ioremap(&pdev->dev, res1->start,
			res1->end - res1->start + 1);

	if (!mmio || !sata_config) {
		err = -ENOMEM;
		goto fail;
	}

	clk_sata = clk_get_sys("tegra_sata", NULL);
	if (IS_ERR_OR_NULL(clk_sata)) {
		pr_err("%s: unable to get SATA clock\n", __func__);
		err = -ENODEV;
		goto fail;
	}
	pdata->clk_sata = clk_sata;

	clk_sata_oob = clk_get_sys("tegra_sata_oob", NULL);
	if (IS_ERR_OR_NULL(clk_sata_oob)) {
		pr_err("%s: unable to get SATA OOB clock\n", __func__);
		err = -ENODEV;
		goto fail;
	}
	pdata->clk_sata_oob = clk_sata_oob;

	tegra_periph_reset_assert(clk_sata);
	tegra_periph_reset_assert(clk_sata_oob);
	udelay(10);

	/* Configure SATA clocks */
	/* Core clock runs at 108MHz */
	clk_rate = TEGRA_SATA_CORE_CLOCK_FREQ_HZ;
	if (clk_set_rate(clk_sata, clk_rate)) {
		err = -ENODEV;
		goto fail;
	}
	/* OOB clock runs at 216MHz */
	clk_rate = TEGRA_SATA_OOB_CLOCK_FREQ_HZ;
	if (clk_set_rate(clk_sata_oob, clk_rate)) {
		err = -ENODEV;
		goto fail;
	}
	/* Enable the clocks for sata module. */
	if (clk_enable(clk_sata)) {
		err = -ENODEV;
		goto fail;
	}
	if (clk_enable(clk_sata_oob)) {
		err = -ENODEV;
		goto fail;
	}

	/* Deassert reset now */
	tegra_periph_reset_deassert(clk_sata);
	tegra_periph_reset_deassert(clk_sata_oob);

	virt = IO_ADDRESS(TEGRA_SATA_BASE);
	temp = readl(virt + SATA_CONFIGURATION_0_OFFSET);
	temp |= EN_FPCI; /* bit 0 to enable */
	writel(temp, virt + SATA_CONFIGURATION_0_OFFSET);

	/* program class code and programming interface for AHCI */
	temp = readl(sata_config + TEGRA_PRIVATE_AHCI_CC_BKDR_OVERRIDE);
	writel((temp | TEGRA_PRIVATE_AHCI_CC_BKDR_OVERRIDE_EN),
		sata_config + TEGRA_PRIVATE_AHCI_CC_BKDR_OVERRIDE);
	writel(TEGRA_PRIVATE_AHCI_CC_BKDR_PGM,
		sata_config + TEGRA_PRIVATE_AHCI_CC_BKDR);
	writel(temp, sata_config + TEGRA_PRIVATE_AHCI_CC_BKDR_OVERRIDE);

	/* Program config space registers */
	/* Enable BUS_MASTER+MEM+IO space, and SERR */
	temp = readl(sata_config + TEGRA_SATA_IO_SPACE_OFFSET);
	temp |= TEGRA_SATA_ENABLE_IO_SPACE |
		TEGRA_SATA_ENABLE_MEM_SPACE |
		TEGRA_SATA_ENABLE_BUS_MASTER |
		TEGRA_SATA_ENABLE_SERR;
	writel(temp, sata_config + TEGRA_SATA_IO_SPACE_OFFSET);

	/* program bar5 space */
	/* write 1's to bar5 register */
	writel(TEGRA_SATA_BAR5_INIT_PROGRAM,
		sata_config + AHCI_BAR5_CONFIG_LOCATION);
	/* flush */
	temp = readl(sata_config + AHCI_BAR5_CONFIG_LOCATION);
	writel(TEGRA_SATA_BAR5_FINAL_PROGRAM,
		sata_config + AHCI_BAR5_CONFIG_LOCATION);

	/* virt still has the virt_addr of TEGRA_SATA_BASE */
	writel((TEGRA_SATA_BAR5_FINAL_PROGRAM >> 8),
				virt + SATA_FPCI_BAR5_0_OFFSET);
	/* flush */
	readl(sata_config + AHCI_BAR5_CONFIG_LOCATION);

	temp = readl(mmio + TEGRA_PRIVATE_AHCI_CAP_BKDR);
	temp |= (HOST_CAP_ALPM | HOST_CAP_SSC | HOST_CAP_PART);
	writel(temp, mmio + TEGRA_PRIVATE_AHCI_CAP_BKDR);

	/* enable Interrupt channel */
	virt = IO_ADDRESS(TEGRA_PRIMARY_ICTLR_BASE);
	temp = readl(virt + PRI_ICTLR_CPU_IER_SET_0_OFFSET);
	temp |= CPU_IER_SATA_CTL;
	writel(temp, virt + PRI_ICTLR_CPU_IER_SET_0_OFFSET);
	/* set IP_INT_MASK */
	virt = IO_ADDRESS(TEGRA_SATA_BASE);
	temp = readl(virt + SATA_INTR_MASK_0_OFFSET);
	temp |= IP_INT_MASK;
	writel(temp, virt + SATA_INTR_MASK_0_OFFSET);

	err = 0;
	goto exit;

fail:
	if (!IS_ERR_OR_NULL(clk_sata))
		clk_put(clk_sata);
	if (!IS_ERR_OR_NULL(clk_sata_oob))
		clk_put(clk_sata_oob);
exit:
	/* unmap the resources we mapped above */
	if (mmio)
		devm_iounmap(&pdev->dev, mmio);
	if (sata_config)
		devm_iounmap(&pdev->dev, sata_config);
	return err;
}

static void tegra_ahci_save_initial_config(struct platform_device *pdev,
					   struct ahci_host_priv *hpriv)
{
	ahci_save_initial_config(&pdev->dev, hpriv, 0, 0);
}

static void tegra_ahci_controller_remove(struct platform_device *pdev)
{
	struct tegra_sata_platform_data *pdata;

	/* grab platform specific data*/
	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		return;
	clk_disable(pdata->clk_sata_oob);
	clk_put(pdata->clk_sata_oob);
	clk_disable(pdata->clk_sata);
	clk_put(pdata->clk_sata);
}

#ifdef CONFIG_PM
static int tegra_ahci_controller_suspend(struct platform_device *pdev)
{
	struct tegra_sata_platform_data *pdata;

	/* grab platform specific data*/
	pdata = pdev->dev.platform_data;
	clk_disable(pdata->clk_sata_oob);
	clk_put(pdata->clk_sata_oob);
	clk_disable(pdata->clk_sata);
	clk_put(pdata->clk_sata);
	return 0;
}

static int tegra_ahci_controller_resume(struct platform_device *pdev)
{
	struct tegra_sata_platform_data *pdata;
	int ret;

	/* grab platform specific data*/
	pdata = pdev->dev.platform_data;
	ret = clk_enable(pdata->clk_sata);
	if (ret == 0)
		ret = clk_enable(pdata->clk_sata_oob);
	return ret;
}

static int tegra_ahci_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	void __iomem *mmio = host->iomap[AHCI_PCI_BAR];
	u32 ctl;
	int rc;

	if (mesg.event & PM_EVENT_SLEEP) {
		/* AHCI spec rev1.1 section 8.3.3:
		 * Software must disable interrupts prior to requesting a
		 * transition of the HBA to D3 state.
		 */
		ctl = readl(mmio + HOST_CTL);
		ctl &= ~HOST_IRQ_EN;
		writel(ctl, mmio + HOST_CTL);
		readl(mmio + HOST_CTL); /* flush */
	}

	rc = ata_host_suspend(host, mesg);
	if (rc)
		return rc;

	return tegra_ahci_controller_suspend(pdev);
}

static int tegra_ahci_resume(struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	int rc;

	rc = tegra_ahci_controller_resume(pdev);
	if (rc)
		return rc;

	if (pdev->dev.power.power_state.event == PM_EVENT_SUSPEND) {
		rc = ahci_reset_controller(host);
		if (rc)
			return rc;

		ahci_init_controller(host);
	}

	ata_host_resume(host);
	return 0;
}

static u32 pg_save_bar5_registers[] = {
	0x0A4,	/* T_AHCI_HBA_SPARE_0 */
	0x018,	/* T_AHCI_HBA_CCC_PORTS */
	0x004,	/* T_AHCI_HBA_GHC */
	0x014,	/* T_AHCI_HBA_CCC_CTL - OP (optional) */
	0x01C,	/* T_AHCI_HBA_EM_LOC */
	0x020,	/* T_AHCI_HBA_EM_CTL - OP */
	0x100,	/* T_AHCI_PORT_PXCLB */
	0x104,	/* T_AHCI_PORT_PXCLBU */
	0x108,	/* T_AHCI_PORT_PXFB */
	0x10C,	/* T_AHCI_PORT_PXFBU */
	0x114,	/* T_AHCI_PORT_PXIE */
	0x118,	/* T_AHCI_PORT_PXCMD */
	0x0AC,	/* T_AHCI_HBA_SHUTDOWN_TIMER */
	0x0A8,	/* T_AHCI_HBA_PLL_CTRL */
	0x12C,	/* T_AHCI_PORT_PXSCTL */
	0x140,	/* T_AHCI_PORT_PXFBS */
	0x17C	/* T_AHCI_PORT_MP */
/* FIXME: */
#if 0
	/* Save and restore via bkdr writes */
	0x000,	/* T_AHCI_HBA_CAP */
	0x00C,	/* T_AHCI_HBA_PI */
	0x024,	/* T_AHCI_HBA_CAP2 */
	0x120,	/* NV_PROJ__SATA0_CHX_AHCI_PORT_PXTFD  */
	0x124,	/* NV_PROJ__SATA0_CHX_AHCI_PORT_PXSIG */
	0x128,	/* NV_PROJ__SATA0_CHX_AHCI_PORT_PXSSTS */
	0x170,	/* T_AHCI_PORT_BKDR */
#endif
};

static u32 pg_save_config_registers[] = {
	0x000,	/* T_SATA0_CFG_0 */
	0x004,	/* T_SATA0_CFG_1 */
	0x008,	/* T_SATA0_CFG_2 */
	0x00C,	/* T_SATA0_CFG_3 */
	0x024,	/* T_SATA0_CFG_9 */
	0x028,	/* T_SATA0_CFG_10 */
	0x02C,	/* T_SATA0_CFG_11 */
	0x030,	/* T_SATA0_CFG_12 */
	0x034,	/* T_SATA0_CFG_13 */
	0x038,	/* T_SATA0_CFG_14 */
	0x03C,	/* T_SATA0_CFG_15 */
	0x040,	/* T_SATA0_CFG_16 */
	0x044,	/* T_SATA0_CFG_17 */
	0x048,	/* T_SATA0_CFG_18 */
	0x04C,	/* T_SATA0_FPCI_DBG_0 */
	0x050,	/* T_SATA0_FPCI_DBG_1 */
	0x054,	/* T_SATA0_FPCI_SW */
	0x08C,	/* T_SATA0_ATACAP0 */
	0x090,	/* T_SATA0_ATACAP1 */
	0x094,	/* T_SATA0_CFG_35 */
	0x098,	/* T_SATA0_AHCI_IDP1 */
	0x0B0,	/* T_SATA0_MSI_CTRL */
	0x0B4,	/* T_SATA0_MSI_ADDR1 */
	0x0B8,	/* T_SATA0_MSI_ADDR2 */
	0x0BC,	/* T_SATA0_MSI_DATA */
	0x0C0,	/* T_SATA0_MSI_QUEUE */
	0x0EC,	/* T_SATA0_MSI_MAP */
	0x0F0,	/* T_SATA0_INDIRECT_IDP0 */
	0x0F4,	/* T_SATA0_INDIRECT_IDP1 */
	0x0F8,	/* T_SATA0_FPCICFG */
	0x0FC,	/* T_SATA0_SCRATCH_1 */
	0x540,	/* T_SATA0_CTRL */
	0x114,	/* T_SATA0_NVOOB */
	0x118,	/* T_SATA0_CROSS_BAR */
	0x11C,	/* T_SATA0_PMUCTL */
	0x120,	/* T_SATA0_CFG_PHY_0 */
	0x124,	/* T_SATA0_CFG_PHY_POWER */
	0x128,	/* T_SATA0_CFG_PHY_POWER_1 */
	0x12C,	/* T_SATA0_CFG_PHY_1 */
	0x170,	/* T_SATA0_FIFO */
	0x174,	/* T_SATA0_CFG_LINK_0 */
	0x178,	/* T_SATA0_CFG_LINK_1 */
	0x1D0,	/* MCP_SATA0_CFG_TRANS_0 */
	0x238,	/* T_SATA0_ALPM_CTRL */
	0x23C,	/* T_SATA0_FBS_CONFIG_0 - OP */
	0x304,	/* T_SATA0_AHCI_HBA_HOLD_GEN */
	0x30C,	/* T_SATA0_AHCI_HBA_CYA_0 */
	0x318,	/* T_SATA0_AHCI_HBA_BIST_OVERRIDE_CTL */
	0x320,	/* T_SATA0_AHCI_HBA_SPARE_1 */
	0x324,	/* T_SATA0_AHCI_HBA_SPARE_2 */
	0x328,	/* T_SATA0_AHCI_HBA_DYN_CLK_CLAMP */
	0x32C,	/* T_SATA0_AHCI_CFG_ERR_CTRL */
	0x338,	/* T_SATA0_AHCI_HBA_CYA_1 */
	0x340,	/* T_SATA0_AHCI_HBA_PRE_STAGING_CONTROL */
	0x370,	/* T_SATA0_CFG */
	0x374,	/* T_SATA0_CFG_ATAPI */
	0x378,	/* T_SATA0_CYA_SHADOW */
	0x430,	/* T_SATA0_CFG_FPCI_0 */
	0x490,	/* T_SATA0_IDE1 (OLD_IDE_TIMING) */
	0x494,	/* T_SATA0_CFG_ESATA_CTRL */
	0x4A0,	/* T_SATA0_CYA1 */
	0x4A8,	/* T_SATA0_CFG_CTRL_1 */
	0x4B0,	/* T_SATA0_CFG_GLUE */
	0x4BC,	/* T_SATA0_CFG_CTL_FA */
	0x4F0,	/* T_SATA0_PERF0 */
	0x4F4,	/* T_SATA0_PERF1 */
	0x534,	/* T_SATA0_PHY_CTRL */
	0x540,	/* T_SATA0_CTRL */
	0x54C,	/* T_SATA0_CFG_SATA */
	0x554	/* T_SATA0_LOW_POWER_COUNT */
/* FIXME: */
#if 0
	/* Save and restore following protocol */
	0x4A4,	/* T_SATA0_BKDOOR_CC */
	0x530,	/* T_SATA0_CHXCFG1 */
	0x684,	/* T_SATA0_CHX_MISC */
	0x700,	/* T_SATA0_CHXCFG3 */
	0x704,	/* T_SATA0_CHXCFG4_CHX */
	0x690,	/* T_SATA0_CHX_PHY_CTRL1_GEN1 */
	0x694,	/* T_SATA0_CHX_PHY_CTRL1_GEN2 */
	0x698,	/* T_SATA0_CHX_PHY_CTRL1_GEN3 */
	0x69C,	/* T_SATA0_CHX_PHY_CTRL_2 */
	0x6B0,	/* T_SATA0_CHX_PHY_CTRL_3 */
	0x6B4,	/* T_SATA0_CHX_PHY_CTRL_4 */
	0x6B8,	/* T_SATA0_CHX_PHY_CTRL_5 */
	0x6BC,	/* T_SATA0_CHX_PHY_CTRL_6 */
	0x714,	/* T_SATA0_PRBS_CHX - OP */
	0x750,	/* T_SATA0_CHX_LINK0 */
	0x7F0,	/* T_SATA0_CHX_GLUE */
	0x6B0,	/* T_SATA0_CHX_PHY_COMMON */
	0x300,	/* T_SATA0_AHCI_HBA_CAP_BKDR */
	0x330,	/* T_SATA0_AHCI_HBA_CAP2_BKDR */
	0x33C,	/* T_SATA0_AHCI_HBA_PI_BKDR */
	0x790,	/* T_SATA0_CHX_AHCI_PORT_PXTFD  */
	0x794,	/* T_SATA0_CHX_AHCI_PORT_PXSIG */
	0x798,	/* T_SATA0_CHX_AHCI_PORT_PXSSTS */
#endif
};

static u32 pg_save_ipfs_registers[] = {
	0x000,	/* SATA_AXI_BAR0_SZ_0 */
	0x004,	/* SATA_AXI_BAR1_SZ_0 */
	0x008,	/* SATA_AXI_BAR2_SZ_0 */
	0x00C,	/* SATA_AXI_BAR3_SZ_0 */
	0x010,	/* SATA_AXI_BAR4_SZ_0 */
	0x014,	/* SATA_AXI_BAR5_SZ_0 */
	0x018,	/* SATA_AXI_BAR6_SZ_0 */
	0x01C,	/* SATA_AXI_BAR7_SZ_0 */
	0x040,	/* SATA_AXI_BAR0_START_0 */
	0x044,	/* SATA_AXI_BAR1_START_0 */
	0x048,	/* SATA_AXI_BAR2_START_0 */
	0x04C,	/* SATA_AXI_BAR3_START_0 */
	0x050,	/* SATA_AXI_BAR4_START_0 */
	0x054,	/* SATA_AXI_BAR5_START_0 */
	0x058,	/* SATA_AXI_BAR6_START_0 */
	0x05C,	/* SATA_AXI_BAR7_START_0 */
	0x080,	/* SATA_FPCI_BAR0_0 */
	0x084,	/* SATA_FPCI_BAR1_0 */
	0x088,	/* SATA_FPCI_BAR2_0 */
	0x08C,	/* SATA_FPCI_BAR3_0 */
	0x090,	/* SATA_FPCI_BAR4_0 */
	0x094,	/* SATA_FPCI_BAR5_0 */
	0x098,	/* SATA_FPCI_BAR6_0 */
	0x09C,	/* SATA_FPCI_BAR7_0 */
	0x0C0,	/* SATA_MSI_BAR_SZ_0 */
	0x0C4,	/* SATA_MSI_AXI_BAR_ST_0 */
	0x0C8,	/* SATA_MSI_FPCI_BAR_ST_0 */
	0x140,	/* SATA_MSI_EN_VEC0_0 */
	0x144,	/* SATA_MSI_EN_VEC1_0 */
	0x148,	/* SATA_MSI_EN_VEC2_0 */
	0x14C,	/* SATA_MSI_EN_VEC3_0 */
	0x150,	/* SATA_MSI_EN_VEC4_0 */
	0x154,	/* SATA_MSI_EN_VEC5_0 */
	0x158,	/* SATA_MSI_EN_VEC6_0 */
	0x15C,	/* SATA_MSI_EN_VEC7_0 */
	0x180,	/* SATA_CONFIGURATION_0 */
	0x184,	/* SATA_FPCI_ERROR_MASKS_0 */
	0x188,	/* SATA_INTR_MASK_0 */
	0x198,	/* SATA_IPFS_INTR_ENABLE_0 */
	0x19C,	/* SATA_UFPCI_CONFIG_0 */
	0x1A0,	/* SATA_CFG_REVID_0 */
	0x1A4,	/* SATA_FPCI_TIMEOUT_0 */
	0x1A8,	/* SATA_TOM_0 */
	0x1B8,	/* SATA_DFPCI_BEN_0 */
	0x1BC,	/* SATA_CLKGATE_HYSTERSIS_0 */
	0x1D8,	/* SATA_SPARE_REG0_0 */
	0x1DC,	/* SATA_SATA_MCCIF_FIFOCTRL_0 */
};

static void tegra_ahci_save_regs(void *save_addr,
				 void __iomem *virt,
				 u32 regs)
{
	u32 i;
	u32 *dest = (u32 *)save_addr;
	u32 *src = (u32 *)virt;

	for (i = 0; i < regs; ++i, ++src, ++dest)
		*dest = readl((void __iomem *)src);
}

static void tegra_ahci_restore_regs(void *save_addr,
				 void __iomem *virt,
				 u32 regs)
{
	u32 i;
	u32 *src = (u32 *)save_addr;
	u32 *dest = (u32 *)virt;

	for (i = 0; i < regs; ++i, ++src, ++dest)
		writel(*src, (void __iomem *)dest);
}

static void tegra_ahci_pg_save_registers(struct ata_host *host)
{
	void __iomem *virt;
	struct tegra_ahci_host_priv *tegra_hpriv;
	void *pg_save;
	u32 offset;
	u32 regs;

	tegra_hpriv = (struct tegra_ahci_host_priv *)host->private_data;
	pg_save = tegra_hpriv->pg_save;

	/* save BAR5 registers */
	virt = IO_ADDRESS(TEGRA_SATA_BAR5_BASE);
	offset = 0;
	regs = sizeof(pg_save_bar5_registers)/sizeof(u32);
	tegra_ahci_save_regs(pg_save, virt, regs);

	/* save CONFIG registers */
	virt = IO_ADDRESS(TEGRA_SATA_CONFIG_BASE);
	offset += sizeof(pg_save_bar5_registers);
	regs = sizeof(pg_save_config_registers)/sizeof(u32);
	tegra_ahci_save_regs((void *)((char *)pg_save+offset), virt, regs);

	/* save IPFS registers */
	virt = IO_ADDRESS(TEGRA_SATA_BASE);
	offset += sizeof(pg_save_config_registers);
	regs = sizeof(pg_save_ipfs_registers)/sizeof(u32);
	tegra_ahci_save_regs((void *)((char *)pg_save+offset), virt, regs);
}

static void tegra_ahci_pg_restore_registers(struct ata_host *host)
{
	void __iomem *virt;
	struct tegra_ahci_host_priv *tegra_hpriv;
	void *pg_save;
	u32 offset;
	u32 regs;

	pg_save = tegra_hpriv->pg_save;

	/* restore BAR5 registers */
	virt = IO_ADDRESS(TEGRA_SATA_BAR5_BASE);
	offset = 0;
	regs = sizeof(pg_save_bar5_registers)/sizeof(u32);
	tegra_ahci_restore_regs(pg_save, virt, regs);

	/* restore CONFIG registers */
	virt = IO_ADDRESS(TEGRA_SATA_CONFIG_BASE);
	offset += sizeof(pg_save_bar5_registers);
	regs = sizeof(pg_save_config_registers)/sizeof(u32);
	tegra_ahci_restore_regs((void *)((char *)pg_save+offset), virt, regs);

	/* restore IPFS registers */
	virt = IO_ADDRESS(TEGRA_SATA_BASE);
	offset += sizeof(pg_save_config_registers);
	regs = sizeof(pg_save_ipfs_registers)/sizeof(u32);
	tegra_ahci_restore_regs((void *)((char *)pg_save+offset), virt, regs);
}

static u8 tegra_ahci_is_port_idle(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);

	if (readl(port_mmio + PORT_CMD_ISSUE) ||
	    readl(port_mmio + PORT_SCR_ACT))
		return 0;
	return 1;
}

static u8 tegra_ahci_is_port_slumber(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	u32 sstat;

	if (!tegra_ahci_is_port_idle(ap))
		return 0;

	/* return 1 if PORT_SCR_STAT is in IPM_SLUMBER_STATE */
	sstat = readl(port_mmio + PORT_SCR_STAT);
	if ((sstat & SSTAT_IPM_STATE_MASK) == SSTAT_IPM_SLUMBER_STATE)
		return 1;
	return 0;
}

static void tegra_ahci_idle_timer(unsigned long arg)
{
	struct ata_host *host = (void *)arg;
	struct ahci_host_priv *hpriv = host->private_data;
	struct ata_port *ap;
	u8 power_gate = 1;
	u8 i;

	/* check if interfaces are in slumber and no cmds are active */
	for (i = 0; i < ahci_nr_ports(hpriv->cap); i++) {
		if (hpriv->port_map & (1 << i)) {
			ap = host->ports[i];
			if (tegra_ahci_is_port_slumber(ap) == 0)
				power_gate = 0;
		}
	}

	((struct tegra_ahci_host_priv *)hpriv)->pg_state = power_gate;
	if (power_gate) {
		dev_printk(KERN_DEBUG, host->dev, "** power-down sata **\n");
		/* FIXME: */
		/* tegra_ahci_pg_prep(hpriv->pdev); */
		tegra_ahci_pg_save_registers(host);
		/* tegra_ahci_pg(hpriv->pdev); */
	} else {
		dev_printk(KERN_DEBUG, host->dev, "** keep power **\n");
	}
}

static unsigned int tegra_ahci_qc_issue(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct ata_host *host = ap->host;
	struct tegra_ahci_host_priv *tegra_hpriv = host->private_data;

	/* stop the idle timer */
	if (timer_pending(&tegra_hpriv->idle_timer))
		del_timer(&tegra_hpriv->idle_timer);

	if (tegra_hpriv->pg_state) {
		dev_printk(KERN_DEBUG, host->dev, "** power-up sata **\n");
		/* FIXME: */
		/* tegra_ahci_un_pg(hpriv->pdev); */
		/* tegra_ahci_pg_restore_registers(host); */
		tegra_hpriv->pg_state = 0;
	}

	return ahci_ops.qc_issue(qc);
}
#endif

static irqreturn_t tegra_ahci_interrupt(int irq, void *dev_instance)
{
	irqreturn_t irq_retval;
#ifdef CONFIG_PM
	u8 i;
	struct ata_host *host;
	struct ahci_host_priv *hpriv;
	struct tegra_ahci_host_priv *tegra_hpriv;
#endif

	irq_retval = ahci_interrupt(irq, dev_instance);
	if (irq_retval == IRQ_NONE)
		return IRQ_NONE;

#ifdef CONFIG_PM
	host = dev_instance;
	hpriv = host->private_data;
	spin_lock(&host->lock);

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap;

		ap = host->ports[i];
		if (ap && tegra_ahci_is_port_idle(ap)) {
			tegra_hpriv = (struct tegra_ahci_host_priv *)hpriv;
			if (timer_pending(&tegra_hpriv->idle_timer))
				del_timer(&tegra_hpriv->idle_timer);
			tegra_hpriv->idle_timer.expires =
				ata_deadline(jiffies, TEGRA_AHCI_IDLE_TIMEOUT);
			add_timer(&tegra_hpriv->idle_timer);
		}
	}
	spin_unlock(&host->lock);
#endif

	return irq_retval;
}

static int __devexit tegra_ahci_remove_one(struct platform_device *pdev)
{
	/* FIXME add a iounmap here to unmap the bar resource */
	tegra_ahci_controller_remove(pdev);
	return 0;
}

static int __devinit tegra_ahci_init_one(struct platform_device *pdev)
{
	static int printed_version;
	struct ata_port_info pi = ahci_port_info;
	const struct ata_port_info *ppi[] = { &pi, NULL };
	struct device *dev = &pdev->dev;
	struct ahci_host_priv *hpriv = NULL;
	struct ata_host *host = NULL;
	struct tegra_sata_platform_data *pdata;
	int n_ports, i, rc;
	struct resource *res, *irq_res;
	void __iomem *mmio;

	VPRINTK("ENTER\n");

	WARN_ON(ATA_MAX_QUEUE > AHCI_MAX_CMDS);

	if (!printed_version++)
		dev_printk(KERN_DEBUG, &pdev->dev, "version " DRV_VERSION "\n");

	/* Simple resource validation */
	if (pdev->num_resources != 3) {
		dev_err(&pdev->dev, "invalid number of resources\n");
		dev_printk(KERN_ERR, &pdev->dev, "not enough SATA resources\n");
		return -EINVAL;
	}

	/* acquire bar resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -EINVAL;

	/* acquire IRQ resource */
	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (irq_res == NULL)
		return -EINVAL;
	if (irq_res->start <= 0)
		return -EINVAL;

	/* grab platform specific data*/
	pdata = pdev->dev.platform_data;
	/* allocate sizeof tegra_ahci_host_priv, which contains 3 extra fields */
	hpriv = devm_kzalloc(dev, sizeof(struct tegra_ahci_host_priv), GFP_KERNEL);
	if (!hpriv) {
		rc = -ENOMEM;
		goto fail;
	}
	hpriv->flags |= (unsigned long)pi.private_data;

	/* Call tegra init routine */
	rc = tegra_ahci_controller_init(pdev);
	if (rc != 0) {
		dev_printk(KERN_ERR, &pdev->dev, "TEGRA SATA init failed\n");
		goto fail;
	}

	/*
	 * We reserve a table of 6 BARs in platform_data to store BARs.
	 * Save the mapped AHCI_PCI_BAR address to the table.
	 */
	mmio = devm_ioremap(&pdev->dev, res->start, (res->end-res->start+1));
	pdata->bars_table[AHCI_PCI_BAR] = mmio;
	hpriv->mmio = mmio;

	/* save initial config */
	tegra_ahci_save_initial_config(pdev, hpriv);
	dev_printk(KERN_DEBUG, &pdev->dev, "past save init config\n");

	/* prepare host */
	if (hpriv->cap & HOST_CAP_NCQ) {
		pi.flags |= ATA_FLAG_NCQ;
		pi.flags |= ATA_FLAG_FPDMA_AA;
	}

	/* CAP.NP sometimes indicate the index of the last enabled
	 * port, at other times, that of the last possible port, so
	 * determining the maximum port number requires looking at
	 * both CAP.NP and port_map.
	 */
	n_ports = max(ahci_nr_ports(hpriv->cap), fls(hpriv->port_map));
	host = ata_host_alloc_pinfo(&pdev->dev, ppi, n_ports);
	if (!host) {
		rc = -ENOMEM;
		goto fail;
	}
	host->private_data = hpriv;
	host->iomap = pdata->bars_table;

	if (!(hpriv->cap & HOST_CAP_SSS))
		host->flags |= ATA_HOST_PARALLEL_SCAN;
	else
		printk(KERN_INFO "ahci: SSS flag set, parallel bus scan disabled\n");

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];

		/* set initial link pm policy */
		ap->pm_policy = NOT_AVAILABLE;

		/* disabled/not-implemented port */
		if (!(hpriv->port_map & (1 << i)))
			ap->ops = &ata_dummy_port_ops;
		else
			ap->pm_policy = MIN_POWER;
	}

	rc = ahci_reset_controller(host);
	if (rc) {
		dev_printk(KERN_ERR, &pdev->dev, "Reset controller failed!\n");
		goto fail;
	}

	ahci_init_controller(host);
	ahci_print_info(host, "TEGRA-SATA");
	dev_printk(KERN_DEBUG, &pdev->dev, "controller init okay\n");

#ifdef CONFIG_PM
	{
		struct tegra_ahci_host_priv *tegra_hpriv;

		/* setup sata idle timer */
		tegra_hpriv = (struct tegra_ahci_host_priv *)hpriv;
		init_timer_deferrable(&tegra_hpriv->idle_timer);
		tegra_hpriv->idle_timer.function = tegra_ahci_idle_timer;
		tegra_hpriv->idle_timer.data = (unsigned long)host;

		/* setup PG save/restore area */
		tegra_hpriv->pg_save = devm_kzalloc(dev,
					(sizeof(pg_save_bar5_registers) +
					 sizeof(pg_save_config_registers) +
					 sizeof(pg_save_ipfs_registers)),
					GFP_KERNEL);
		if (!tegra_hpriv->pg_save) {
			rc = -ENOMEM;
			goto fail;
		}
	}
#endif

	rc = ata_host_activate(host, irq_res->start, tegra_ahci_interrupt,
				IRQF_SHARED, &ahci_sht);
	return rc;
fail:
	if (host) {
		if (host->iomap[AHCI_PCI_BAR])
			devm_iounmap(&pdev->dev, host->iomap[AHCI_PCI_BAR]);
		devres_free(host);
	}
	if (hpriv)
		devm_kfree(&pdev->dev, hpriv);

	return rc;
}

static int __init ahci_init(void)
{
	return platform_driver_register(&tegra_platform_ahci_driver);
}

static void __exit ahci_exit(void)
{
	platform_driver_unregister(&tegra_platform_ahci_driver);
}


#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static void dbg_ahci_dump_regs(struct seq_file *s, u32 *ptr, u32 base, u32 regs)
{
#define REGS_PER_LINE	4

	u32 i, j;
	u32 lines = regs / REGS_PER_LINE;

	for (i = 0; i < lines; i++) {
		seq_printf(s, "0x%08x: ", base+(i*16));
		for (j = 0; j < REGS_PER_LINE; ++j) {
			seq_printf(s, "0x%08x ", readl(ptr));
			++ptr;
		}
		seq_printf(s, "\n");
	}
#undef REGS_PER_LINE
}

static int dbg_ahci_dump_show(struct seq_file *s, void *unused)
{
	u32 base;
	u32 *ptr;

	base = TEGRA_SATA_CONFIG_BASE;
	ptr = (u32 *)IO_TO_VIRT(base);
	seq_printf(s, "SATA CONFIG Registers:\n");
	seq_printf(s, "----------------------\n");
	dbg_ahci_dump_regs(s, ptr, base, 0x200);

	base = TEGRA_SATA_BAR5_BASE;
	ptr = (u32 *)IO_TO_VIRT(base);
	seq_printf(s, "\nAHCI HBA Registers:\n");
	seq_printf(s, "-------------------\n");
	dbg_ahci_dump_regs(s, ptr, base, 64);

	base = TEGRA_SATA_BAR5_BASE+0x100;
	ptr = (u32 *)IO_TO_VIRT(base);
	seq_printf(s, "\nPort Registers:\n");
	seq_printf(s, "---------------\n");
	dbg_ahci_dump_regs(s, ptr, base, 16);
	return 0;
}

static int dbg_ahci_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_ahci_dump_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_ahci_dump_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_ahci_dump_debuginit(void)
{
	(void) debugfs_create_file("tegra_ahci", S_IRUGO,
					NULL, NULL, &debug_fops);
	return 0;
}
late_initcall(tegra_ahci_dump_debuginit);
#endif

MODULE_AUTHOR("NVIDIA");
MODULE_DESCRIPTION("Tegra AHCI SATA low-level driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(ahci_init);
module_exit(ahci_exit);
