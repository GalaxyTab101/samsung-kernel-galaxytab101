/*
 * arch/arm/mach-tegra/spdif.c
 *
 * S/PDIF audio driver
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <mach/iomap.h>
#include <mach/spdif.h>
#include <mach/dma.h>
#include <mach/audio.h>
#include <mach/dma.h>
#include <mach/audio_switch.h>

#define ENABLE_SPDIF_DEBUG_PRINT	0
#if  ENABLE_SPDIF_DEBUG_PRINT
#define SPDIF_DEBUG_PRINT(fmt, arg...)  printk(fmt, ## arg)
#else
#define SPDIF_DEBUG_PRINT(fmt, arg...) do {} while (0)
#endif

static inline void spdif_writel(unsigned long base, u32 val, u32 reg)
{
	SPDIF_DEBUG_PRINT("Spdif Write 0x%lx : %08x\n",base + reg, val);
	writel(val, base + reg);
}

static inline u32 spdif_readl(unsigned long base, u32 reg)
{
	u32 val = readl(base + reg);
	SPDIF_DEBUG_PRINT("Spdif Read 0x%lx : %08x\n",base + reg, val);
	return val;
}

int spdif_set_bit_mode(unsigned long base, unsigned mode)
{
	u32 val = spdif_readl(base, SPDIF_CTRL_0);
	val &= ~SPDIF_CTRL_0_BIT_MODE_MASK;

	if (mode > SPDIF_BIT_MODE_MODERAW) {
		pr_err("%s: invalid bit_size selector %d\n", __func__,
			mode);
		return -EINVAL;
	}

	val |= mode << SPDIF_CTRL_0_BIT_MODE_SHIFT;

	spdif_writel(base, val, SPDIF_CTRL_0);
	return 0;
}

int spdif_set_sample_rate(unsigned long base, unsigned int sample_rate)
{
	unsigned int ch_sta[] = {
		0x0, /* 44.1, default values */
		0x0,
		0x0,
		0x0,
		0x0,
		0x0,
	};

	switch (sample_rate) {
	case 32000:
		ch_sta[0] = 0x3 << 24;
		ch_sta[1] = 0xC << 4;
		break;
	case 44100:
		ch_sta[0] = 0x0;
		ch_sta[1] = 0xF << 4;
		break;
	case 48000:
		ch_sta[0] = 0x2 << 24;
		ch_sta[1] = 0xD << 4;
		break;
	case 88200:
	case 96000:
	case 176400:
	case 192000:
		break;
	default:
		return -1;
	}

	spdif_writel(base, ch_sta[0], SPDIF_CH_STA_TX_A_0);
	spdif_writel(base, ch_sta[1], SPDIF_CH_STA_TX_B_0);
	spdif_writel(base, ch_sta[2], SPDIF_CH_STA_TX_C_0);
	spdif_writel(base, ch_sta[3], SPDIF_CH_STA_TX_D_0);
	spdif_writel(base, ch_sta[4], SPDIF_CH_STA_TX_E_0);
	spdif_writel(base, ch_sta[5], SPDIF_CH_STA_TX_F_0);

	return 0;
}

u32 spdif_get_control(unsigned long base)
{
	return spdif_readl(base, SPDIF_CTRL_0);
}

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)

void spdif_fifo_write(unsigned long base, int mode, u32 data)
{
	if (mode == AUDIO_TX_MODE) {
		spdif_writel(base, data, SPDIF_DATA_OUT_0);
	} else {
		spdif_writel(base, data, SPDIF_DATA_IN_0);
	}
}

int spdif_fifo_set_attention_level(unsigned long base, int mode, unsigned level)
{
	u32 val;

	if (level > SPDIF_FIFO_ATN_LVL_TWELVE_SLOTS) {
		pr_err("%s: invalid fifo level selector %d\n", __func__,
			level);
		return -EINVAL;
	}

	val = spdif_readl(base, SPDIF_DATA_FIFO_CSR_0);

	if (mode == AUDIO_TX_MODE) {
		val &= ~SPDIF_DATA_FIFO_CSR_0_TX_ATN_LVL_MASK;
		val |= level << SPDIF_DATA_FIFO_CSR_0_TX_ATN_LVL_SHIFT;
	}

	spdif_writel(base, val, SPDIF_DATA_FIFO_CSR_0);
	return 0;
}

void spdif_fifo_clear(unsigned long base, int mode)
{
	u32 val = spdif_readl(base, SPDIF_DATA_FIFO_CSR_0);

	if (mode == AUDIO_TX_MODE) {
		val &= ~(SPDIF_DATA_FIFO_CSR_0_TX_CLR |
			SPDIF_DATA_FIFO_CSR_0_TU_CLR);
		val |= SPDIF_DATA_FIFO_CSR_0_TX_CLR |
			SPDIF_DATA_FIFO_CSR_0_TU_CLR;
	}
	spdif_writel(base, val, SPDIF_DATA_FIFO_CSR_0);
}

int spdif_set_fifo_packed(unsigned long base, unsigned on)
{
	u32 val = spdif_readl(base, SPDIF_CTRL_0);

	val &= ~SPDIF_CTRL_0_PACK;
	val |= on ? (SPDIF_CTRL_0_PACK) : 0;
	spdif_writel(base, val, SPDIF_CTRL_0);
	return 0;
}

u32 spdif_get_status(unsigned long base, int mode)
{
	return spdif_readl(base, SPDIF_STATUS_0);
}

void spdif_ack_status(unsigned long base)
{
	return spdif_writel(base, spdif_readl(base, SPDIF_STATUS_0),
				SPDIF_STATUS_0);
}

u32 spdif_get_fifo_scr(unsigned long base)
{
	return spdif_readl(base, SPDIF_DATA_FIFO_CSR_0);
}

phys_addr_t spdif_get_fifo_phy_base(phys_addr_t phy_base, int mode)
{
	if (mode == AUDIO_TX_MODE)
		return phy_base + SPDIF_DATA_OUT_0;
	else
		return phy_base + SPDIF_DATA_IN_0;

	return 0;
}

u32 spdif_get_fifo_full_empty_count(unsigned long base, int mode)
{
	u32 val = spdif_readl(base, SPDIF_DATA_FIFO_CSR_0);

	if (mode == AUDIO_TX_MODE) {
		val = val >> SPDIF_DATA_FIFO_CSR_0_TD_EMPTY_COUNT_SHIFT;
		return val & SPDIF_DATA_FIFO_CSR_0_TD_EMPTY_COUNT_MASK;
	}

	return 0;
}

int spdif_get_dma_requestor(int ifc, int fifo_mode)
{
	return TEGRA_DMA_REQ_SEL_SPD_I;
}

int spdif_initialize(unsigned long base, int mode)
{
	/* disable interrupts from SPDIF */
	spdif_writel(base, 0x0, SPDIF_CTRL_0);
	spdif_fifo_clear(base, mode);
	spdif_fifo_enable(base, mode, 0);

	spdif_set_bit_mode(base, SPDIF_BIT_MODE_MODE16BIT);
	spdif_set_fifo_packed(base, 1);

	spdif_set_sample_rate(base, 44100);

	return 0;
}

void spdif_get_all_regs(unsigned long base, struct spdif_regs_cache* regs)
{
	regs->spdif_ctrl_0 = spdif_readl(base, SPDIF_CTRL_0);
	regs->spdif_status_0 = spdif_readl(base, SPDIF_STATUS_0);
	regs->spdif_strobe_ctrl_0 = spdif_readl(base, SPDIF_STROBE_CTRL_0);
	regs->spdif_data_fifo_scr_0 = spdif_readl(base, SPDIF_DATA_FIFO_CSR_0);
	regs->spdif_ch_sta_rx_a_0 = spdif_readl(base, SPDIF_CH_STA_RX_A_0);
	regs->spdif_ch_sta_rx_b_0 = spdif_readl(base, SPDIF_CH_STA_RX_B_0);
	regs->spdif_ch_sta_rx_c_0 = spdif_readl(base, SPDIF_CH_STA_RX_C_0);
	regs->spdif_ch_sta_rx_d_0 = spdif_readl(base, SPDIF_CH_STA_RX_D_0);
	regs->spdif_ch_sta_rx_e_0 = spdif_readl(base, SPDIF_CH_STA_RX_E_0);
	regs->spdif_ch_sta_rx_f_0 = spdif_readl(base, SPDIF_CH_STA_RX_F_0);
	regs->spdif_ch_sta_tx_a_0 = spdif_readl(base, SPDIF_CH_STA_TX_A_0);
	regs->spdif_ch_sta_tx_b_0 = spdif_readl(base, SPDIF_CH_STA_TX_B_0);
	regs->spdif_ch_sta_tx_c_0 = spdif_readl(base, SPDIF_CH_STA_TX_C_0);
	regs->spdif_ch_sta_tx_d_0 = spdif_readl(base, SPDIF_CH_STA_TX_D_0);
	regs->spdif_ch_sta_tx_e_0 = spdif_readl(base, SPDIF_CH_STA_TX_E_0);
	regs->spdif_ch_sta_tx_f_0 = spdif_readl(base, SPDIF_CH_STA_TX_F_0);
	regs->spdif_usr_sta_rx_a_0 = spdif_readl(base, SPDIF_USR_STA_RX_A_0);
	regs->spdif_usr_dat_tx_a_0 = spdif_readl(base, SPDIF_USR_DAT_TX_A_0);
}

void spdif_set_all_regs(unsigned long base, struct spdif_regs_cache* regs)
{
	spdif_writel(base, regs->spdif_ctrl_0, SPDIF_CTRL_0);
	spdif_writel(base, regs->spdif_status_0, SPDIF_STATUS_0);
	spdif_writel(base, regs->spdif_strobe_ctrl_0, SPDIF_STROBE_CTRL_0);
	spdif_writel(base, regs->spdif_data_fifo_scr_0, SPDIF_DATA_FIFO_CSR_0);
	spdif_writel(base, regs->spdif_ch_sta_rx_a_0, SPDIF_CH_STA_RX_A_0);
	spdif_writel(base, regs->spdif_ch_sta_rx_b_0, SPDIF_CH_STA_RX_B_0);
	spdif_writel(base, regs->spdif_ch_sta_rx_c_0, SPDIF_CH_STA_RX_C_0);
	spdif_writel(base, regs->spdif_ch_sta_rx_d_0, SPDIF_CH_STA_RX_D_0);
	spdif_writel(base, regs->spdif_ch_sta_rx_e_0, SPDIF_CH_STA_RX_E_0);
	spdif_writel(base, regs->spdif_ch_sta_rx_f_0, SPDIF_CH_STA_RX_F_0);
	spdif_writel(base, regs->spdif_ch_sta_tx_a_0, SPDIF_CH_STA_TX_A_0);
	spdif_writel(base, regs->spdif_ch_sta_tx_b_0, SPDIF_CH_STA_TX_B_0);
	spdif_writel(base, regs->spdif_ch_sta_tx_c_0, SPDIF_CH_STA_TX_C_0);
	spdif_writel(base, regs->spdif_ch_sta_tx_d_0, SPDIF_CH_STA_TX_D_0);
	spdif_writel(base, regs->spdif_ch_sta_tx_e_0, SPDIF_CH_STA_TX_E_0);
	spdif_writel(base, regs->spdif_ch_sta_tx_f_0, SPDIF_CH_STA_TX_F_0);
	spdif_writel(base, regs->spdif_usr_sta_rx_a_0, SPDIF_USR_STA_RX_A_0);
	spdif_writel(base, regs->spdif_usr_dat_tx_a_0, SPDIF_USR_DAT_TX_A_0);
}
#else

struct spdif_controller_info {
/*FIXME: add clock here or move the struct to common place */
	int	dma_ch[AUDIO_FIFO_CNT];
	int	stream_index[AUDIO_FIFO_CNT];
	unsigned int base;
};

static struct spdif_controller_info spdif_cont_info;

static int spdif_get_apbif_channel(int fifo_mode)
{
	return spdif_cont_info.dma_ch[fifo_mode];
}

int spdif_set_fifo_packed(unsigned long base, unsigned on)
{
	/* This register is obsolete after T20 */
	return 0;
}

void spdif_fifo_write(unsigned long base, int mode, u32 data)
{
	int apbif_ifc = spdif_get_apbif_channel(mode);

	if (apbif_ifc != -ENOENT)
		apbif_fifo_write(apbif_ifc, mode, data);
}

int spdif_fifo_set_attention_level(unsigned long base, int mode,
			unsigned level)
{
	int apbif_ifc = spdif_get_apbif_channel(mode);

	/* expected the level as four slots now */
	level = SPDIF_FIFO_ATN_LVL_FOUR_SLOTS;

	if (apbif_ifc != -ENOENT)
		return apbif_fifo_set_attention_level(apbif_ifc,
					 mode, (level - 1));

	return 0;
}

void spdif_fifo_clear(unsigned long base, int mode)
{
	int apbif_ifc = spdif_get_apbif_channel(mode);

	if (apbif_ifc != -ENOENT)
		apbif_soft_reset(apbif_ifc, mode, 1);
}

u32 spdif_get_status(unsigned long base, int mode)
{
	int apbif_ifc = spdif_get_apbif_channel(mode);

	if (apbif_ifc != -ENOENT)
		return apbif_get_fifo_mode(apbif_ifc, mode);

	return 0;
}

void spdif_ack_status(unsigned long base)
{
	/*FIXME: add apbif call here */
}

u32 spdif_get_fifo_scr(unsigned long base)
{
	/*FIXME: add apbif call here */
	return 0;
}

phys_addr_t spdif_get_fifo_phy_base(phys_addr_t phy_base, int mode)
{
	int apbif_ifc = spdif_get_apbif_channel(mode);

	if (apbif_ifc != -ENOENT)
		return apbif_get_fifo_phy_base(apbif_ifc, mode);

	return 0;
}

u32 spdif_get_fifo_full_empty_count(unsigned long base, int mode)
{
	int apbif_ifc = spdif_get_apbif_channel(mode);

	if (apbif_ifc != -ENOENT)
		return apbif_get_fifo_freecount(apbif_ifc, mode);

	return 0;
}

int spdif_free_dma_requestor(int ifc, int fifo_mode)
{
	int apbif_ifc = spdif_get_apbif_channel(fifo_mode);

	if (apbif_ifc != -ENOENT)
		audio_apbif_free_channel(apbif_ifc, fifo_mode);

	return 0;
}

static  struct audio_cif  spdif_audiocif;

int spdif_set_acif(int fifo_mode, struct audio_cif *cifInfo)
{
	struct audio_cif  *tx_audio_cif = &spdif_audiocif;

	/* set spdif audiocif */
	/* setting base value for acif */
	memset(tx_audio_cif, 0 , sizeof(struct audio_cif));
	tx_audio_cif->audio_channels	= AUDIO_CHANNEL_2;
	tx_audio_cif->client_channels	= AUDIO_CHANNEL_2;
	tx_audio_cif->audio_bits	= AUDIO_BIT_SIZE_16;
	tx_audio_cif->client_bits	= AUDIO_BIT_SIZE_16;

	if (fifo_mode == AUDIO_TX_MODE)
		audio_switch_set_acif(spdif_cont_info.base +
			 SPDIF_AUDIOCIF_TXDATA_CTRL_0, tx_audio_cif);
	else
		audio_switch_set_acif(spdif_cont_info.base +
			 SPDIF_AUDIOCIF_RXDATA_CTRL_0, tx_audio_cif);

	audio_apbif_set_acif(spdif_get_apbif_channel(fifo_mode),
		fifo_mode, tx_audio_cif);

	return 0;
}

int spdif_get_dma_requestor(int ifc, int fifo_mode)
{
	int dma_index =	0;
	int apbif_ifc = ahubtx0_spdif;

	if (fifo_mode == AUDIO_RX_MODE)
		apbif_ifc = ahubrx0_spdif;

	dma_index = apbif_get_channel(apbif_ifc, fifo_mode);

	if (dma_index != -ENOENT) {
		spdif_cont_info.dma_ch[fifo_mode] = dma_index - 1;
		/* FIXME : this need to be called on connection request
		*/
		spdif_set_acif(fifo_mode, 0);
	}

	return dma_index;
}

int spdif_initialize(unsigned long base, int mode)
{
	int err = 0;

	spdif_cont_info.dma_ch[0] = -ENOENT;
	spdif_cont_info.dma_ch[1] = -ENOENT;
	spdif_cont_info.base = base;

	err = audio_switch_open();
	if (err)
		return err;

	/* disable interrupts from SPDIF */
	spdif_writel(base, 0x0, SPDIF_CTRL_0);
	spdif_fifo_clear(base, mode);

	spdif_fifo_enable(base, mode, 0);

	spdif_set_bit_mode(base, SPDIF_BIT_MODE_MODE16BIT);
	spdif_set_fifo_packed(base, 1);

	spdif_set_sample_rate(base, 44100);

	return 0;
}
#endif

void spdif_fifo_enable(unsigned long base, int mode, int on)
{
	u32 val = 0;

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)

	int apbif_ifc = spdif_get_apbif_channel(mode);

	if (apbif_ifc == -ENOENT)
		return;

	apbif_channel_enable(apbif_ifc, mode, on);
#endif

	val = spdif_readl(base, SPDIF_CTRL_0);

	if (mode == AUDIO_TX_MODE) {
		val &= ~(SPDIF_CTRL_0_TU_EN);
		set_reg_mode(val, SPDIF_CTRL_0_TC_EN, on);
		set_reg_mode(val, SPDIF_CTRL_0_TX_EN, on);
	} else {
		set_reg_mode(val, SPDIF_CTRL_0_RX_EN, on);
	}

	spdif_writel(base, val, SPDIF_CTRL_0);
}
