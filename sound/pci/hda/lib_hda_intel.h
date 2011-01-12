/*
 *  Copyright(c) 2004 Intel Corporation. All rights reserved.
 *
 *  Copyright (c) 2004 Takashi Iwai <tiwai@suse.de>
 *                     PeiSen Hou <pshou@realtek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the Free
 *  Software Foundation; either version 2 of the License, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License along with
 *  this program; if not, write to the Free Software Foundation, Inc., 59
 *  Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 *  CONTACTS:
 *
 *  Matt Jared		matt.jared@intel.com
 *  Andy Kopp		andy.kopp@intel.com
 *  Dan Kogan		dan.d.kogan@intel.com
 *
 */

#ifndef __LINUX_LIB_HDA_INTEL_H__
#define __LINUX_LIB_HDA_INTEL_H__

#include <asm/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <sound/core.h>
#include <sound/initval.h>
#include "hda_codec.h"

MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{Intel, ICH6},"
			 "{Intel, ICH6M},"
			 "{Intel, ICH7},"
			 "{Intel, ESB2},"
			 "{Intel, ICH8},"
			 "{Intel, ICH9},"
			 "{Intel, ICH10},"
			 "{Intel, PCH},"
			 "{Intel, CPT},"
			 "{Intel, PBG},"
			 "{Intel, SCH},"
			 "{ATI, SB450},"
			 "{ATI, SB600},"
			 "{ATI, RS600},"
			 "{ATI, RS690},"
			 "{ATI, RS780},"
			 "{ATI, R600},"
			 "{ATI, RV630},"
			 "{ATI, RV610},"
			 "{ATI, RV670},"
			 "{ATI, RV635},"
			 "{ATI, RV620},"
			 "{ATI, RV770},"
			 "{VIA, VT8251},"
			 "{VIA, VT8237A},"
			 "{SiS, SIS966},"
			 "{ULI, M5461}}");
MODULE_DESCRIPTION("Intel HDA driver");

#ifdef CONFIG_SND_VERBOSE_PRINTK
#define SFX	/* nop */
#else
#define SFX	"hda-intel: "
#endif

/*
 * registers
 */
#define ICH6_REG_GCAP			0x00
#define   ICH6_GCAP_64OK	(1 << 0)   /* 64bit address support */
#define   ICH6_GCAP_NSDO	(3 << 1)   /* # of serial data out signals */
#define   ICH6_GCAP_BSS		(31 << 3)  /* # of bidirectional streams */
#define   ICH6_GCAP_ISS		(15 << 8)  /* # of input streams */
#define   ICH6_GCAP_OSS		(15 << 12) /* # of output streams */
#define ICH6_REG_VMIN			0x02
#define ICH6_REG_VMAJ			0x03
#define ICH6_REG_OUTPAY			0x04
#define ICH6_REG_INPAY			0x06
#define ICH6_REG_GCTL			0x08
#define   ICH6_GCTL_RESET	(1 << 0)   /* controller reset */
#define   ICH6_GCTL_FCNTRL	(1 << 1)   /* flush control */
#define   ICH6_GCTL_UNSOL	(1 << 8)   /* accept unsol. response enable */
#define ICH6_REG_WAKEEN			0x0c
#define ICH6_REG_STATESTS		0x0e
#define ICH6_REG_GSTS			0x10
#define   ICH6_GSTS_FSTS	(1 << 1)   /* flush status */
#define ICH6_REG_INTCTL			0x20
#define ICH6_REG_INTSTS			0x24
#define ICH6_REG_WALLCLK		0x30	/* 24Mhz source */
#define ICH6_REG_SYNC			0x34
#define ICH6_REG_CORBLBASE		0x40
#define ICH6_REG_CORBUBASE		0x44
#define ICH6_REG_CORBWP			0x48
#define ICH6_REG_CORBRP			0x4a
#define   ICH6_CORBRP_RST	(1 << 15)  /* read pointer reset */
#define ICH6_REG_CORBCTL		0x4c
#define   ICH6_CORBCTL_RUN	(1 << 1)   /* enable DMA */
#define   ICH6_CORBCTL_CMEIE	(1 << 0)   /* enable memory error irq */
#define ICH6_REG_CORBSTS		0x4d
#define   ICH6_CORBSTS_CMEI	(1 << 0)   /* memory error indication */
#define ICH6_REG_CORBSIZE		0x4e

#define ICH6_REG_RIRBLBASE		0x50
#define ICH6_REG_RIRBUBASE		0x54
#define ICH6_REG_RIRBWP			0x58
#define   ICH6_RIRBWP_RST	(1 << 15)  /* write pointer reset */
#define ICH6_REG_RINTCNT		0x5a
#define ICH6_REG_RIRBCTL		0x5c
#define   ICH6_RBCTL_IRQ_EN	(1 << 0)   /* enable IRQ */
#define   ICH6_RBCTL_DMA_EN	(1 << 1)   /* enable DMA */
#define   ICH6_RBCTL_OVERRUN_EN	(1 << 2)   /* enable overrun irq */
#define ICH6_REG_RIRBSTS		0x5d
#define   ICH6_RBSTS_IRQ	(1 << 0)   /* response irq */
#define   ICH6_RBSTS_OVERRUN	(1 << 2)   /* overrun irq */
#define ICH6_REG_RIRBSIZE		0x5e

#define ICH6_REG_IC			0x60
#define ICH6_REG_IR			0x64
#define ICH6_REG_IRS			0x68
#define   ICH6_IRS_VALID	(1<<1)
#define   ICH6_IRS_BUSY		(1<<0)

#define ICH6_REG_DPLBASE		0x70
#define ICH6_REG_DPUBASE		0x74
#define   ICH6_DPLBASE_ENABLE	0x1	/* Enable position buffer */

/* SD offset: SDI0=0x80, SDI1=0xa0, ... SDO3=0x160 */
enum { SDI0, SDI1, SDI2, SDI3, SDO0, SDO1, SDO2, SDO3 };

/* stream register offsets from stream base */
#define ICH6_REG_SD_CTL			0x00
#define ICH6_REG_SD_STS			0x03
#define ICH6_REG_SD_LPIB		0x04
#define ICH6_REG_SD_CBL			0x08
#define ICH6_REG_SD_LVI			0x0c
#define ICH6_REG_SD_FIFOW		0x0e
#define ICH6_REG_SD_FIFOSIZE		0x10
#define ICH6_REG_SD_FORMAT		0x12
#define ICH6_REG_SD_BDLPL		0x18
#define ICH6_REG_SD_BDLPU		0x1c

/* PCI space */
#define ICH6_PCIREG_TCSEL	0x44

/*
 * other constants
 */

/* max number of SDs */
/* ICH, ATI and VIA have 4 playback and 4 capture */
#define ICH6_NUM_CAPTURE	4
#define ICH6_NUM_PLAYBACK	4

/* ULI has 6 playback and 5 capture */
#define ULI_NUM_CAPTURE		5
#define ULI_NUM_PLAYBACK	6

/* ATI HDMI has 1 playback and 0 capture */
#define ATIHDMI_NUM_CAPTURE	0
#define ATIHDMI_NUM_PLAYBACK	1

/* TERA has 4 playback and 3 capture */
#define TERA_NUM_CAPTURE	3
#define TERA_NUM_PLAYBACK	4

/* this number is statically defined for simplicity */
#define MAX_AZX_DEV		16

/* max number of fragments - we may use more if allocating more pages for BDL */
#define BDL_SIZE		4096
#define AZX_MAX_BDL_ENTRIES	(BDL_SIZE / 16)
#define AZX_MAX_FRAG		32
/* max buffer size - no h/w limit, you can increase as you like */
#define AZX_MAX_BUF_SIZE	(1024*1024*1024)

/* RIRB int mask: overrun[2], response[0] */
#define RIRB_INT_RESPONSE	0x01
#define RIRB_INT_OVERRUN	0x04
#define RIRB_INT_MASK		0x05

/* STATESTS int mask: S3,SD2,SD1,SD0 */
#define AZX_MAX_CODECS		8
#define AZX_DEFAULT_CODECS	4
#define STATESTS_INT_MASK	((1 << AZX_MAX_CODECS) - 1)

/* SD_CTL bits */
#define SD_CTL_STREAM_RESET	0x01	/* stream reset bit */
#define SD_CTL_DMA_START	0x02	/* stream DMA start bit */
#define SD_CTL_STRIPE		(3 << 16)	/* stripe control */
#define SD_CTL_TRAFFIC_PRIO	(1 << 18)	/* traffic priority */
#define SD_CTL_DIR		(1 << 19)	/* bi-directional stream */
#define SD_CTL_STREAM_TAG_MASK	(0xf << 20)
#define SD_CTL_STREAM_TAG_SHIFT	20

/* SD_CTL and SD_STS */
#define SD_INT_DESC_ERR		0x10	/* descriptor error interrupt */
#define SD_INT_FIFO_ERR		0x08	/* FIFO error interrupt */
#define SD_INT_COMPLETE		0x04	/* completion interrupt */
#define SD_INT_MASK		(SD_INT_DESC_ERR|SD_INT_FIFO_ERR|\
				 SD_INT_COMPLETE)

/* SD_STS */
#define SD_STS_FIFO_READY	0x20	/* FIFO ready */

/* INTCTL and INTSTS */
#define ICH6_INT_ALL_STREAM	0xff	   /* all stream interrupts */
#define ICH6_INT_CTRL_EN	0x40000000 /* controller interrupt enable bit */
#define ICH6_INT_GLOBAL_EN	0x80000000 /* global interrupt enable bit */

/* below are so far hardcoded - should read registers in future */
#define ICH6_MAX_CORB_ENTRIES	256
#define ICH6_MAX_RIRB_ENTRIES	256

/* position fix mode */
enum {
	POS_FIX_AUTO,
	POS_FIX_LPIB,
	POS_FIX_POSBUF,
};

/* Defines for ATI HD Audio support in SB450 south bridge */
#define ATI_SB450_HDAUDIO_MISC_CNTR2_ADDR   0x42
#define ATI_SB450_HDAUDIO_ENABLE_SNOOP      0x02

/* Defines for Nvidia HDA support */
#define NVIDIA_HDA_TRANSREG_ADDR      0x4e
#define NVIDIA_HDA_ENABLE_COHBITS     0x0f
#define NVIDIA_HDA_ISTRM_COH          0x4d
#define NVIDIA_HDA_OSTRM_COH          0x4c
#define NVIDIA_HDA_ENABLE_COHBIT      0x01

/* Defines for Intel SCH HDA snoop control */
#define INTEL_SCH_HDA_DEVC      0x78
#define INTEL_SCH_HDA_DEVC_NOSNOOP       (0x1<<11)

/* Define IN stream 0 FIFO size offset in VIA controller */
#define VIA_IN_STREAM0_FIFO_SIZE_OFFSET	0x90
/* Define VIA HD Audio Device ID*/
#define VIA_HDAC_DEVICE_ID		0x3288

/* HD Audio class code */
#define PCI_CLASS_MULTIMEDIA_HD_AUDIO	0x0403

/*
 */

struct azx_dev {
	struct snd_dma_buffer bdl; /* BDL buffer */
	u32 *posbuf;		/* position buffer pointer */

	unsigned int bufsize;	/* size of the play buffer in bytes */
	unsigned int period_bytes; /* size of the period in bytes */
	unsigned int frags;	/* number for period in the play buffer */
	unsigned int fifo_size;	/* FIFO size */
	unsigned long start_wallclk;	/* start + minimum wallclk */
	unsigned long period_wallclk;	/* wallclk for period */

	void __iomem *sd_addr;	/* stream descriptor pointer */

	u32 sd_int_sta_mask;	/* stream int status mask */

	/* pcm support */
	struct snd_pcm_substream *substream;	/* assigned substream,
						 * set in PCM open
						 */
	unsigned int format_val;	/* format value to be set in the
					 * controller and the codec
					 */
	unsigned char stream_tag;	/* assigned stream */
	unsigned char index;		/* stream index */
	int device;			/* last device number assigned to */

	unsigned int opened :1;
	unsigned int running :1;
	unsigned int irq_pending :1;
	/*
	 * For VIA:
	 *  A flag to ensure DMA position is 0
	 *  when link position is not greater than FIFO size
	 */
	unsigned int insufficient :1;
};

/* CORB/RIRB */
struct azx_rb {
	u32 *buf;		/* CORB/RIRB buffer
				 * Each CORB entry is 4byte, RIRB is 8byte
				 */
	dma_addr_t addr;	/* physical address of CORB/RIRB buffer */
	/* for RIRB */
	unsigned short rp, wp;	/* read/write pointers */
	int cmds[AZX_MAX_CODECS];	/* number of pending requests */
	u32 res[AZX_MAX_CODECS];	/* last read value */
};

struct azx {
	struct snd_card *card;
	struct pci_dev *pci;
	struct platform_device *pdev;
	int dev_index;

	/* chip type specific */
	int driver_type;
	int playback_streams;
	int playback_index_offset;
	int capture_streams;
	int capture_index_offset;
	int num_streams;

	/* pci resources */
	unsigned long addr;
	void __iomem *remap_addr;
	void __iomem *pciconfig_addr;
	int irq;

	/* locks */
	spinlock_t reg_lock;
	struct mutex open_mutex;

	/* streams (x num_streams) */
	struct azx_dev *azx_dev;

	/* PCM */
	struct snd_pcm *pcm[HDA_MAX_PCMS];

	/* HD codec */
	unsigned short codec_mask;
	int  codec_probe_mask; /* copied from probe_mask option */
	struct hda_bus *bus;
	unsigned int beep_mode;

	/* CORB/RIRB */
	struct azx_rb corb;
	struct azx_rb rirb;

	/* CORB/RIRB and position buffers */
	struct snd_dma_buffer rb;
	struct snd_dma_buffer posbuf;

	/* flags */
	int position_fix[2]; /* for both playback/capture streams */
	int poll_count;
	unsigned int running :1;
	unsigned int initialized :1;
	unsigned int single_cmd :1;
	unsigned int polling_mode :1;
	unsigned int msi :1;
	unsigned int irq_pending_warned :1;
	unsigned int via_dmapos_patch :1; /* enable DMA-position fix for VIA */
	unsigned int probing :1; /* codec probing phase */

	/* for debugging */
	unsigned int last_cmd[AZX_MAX_CODECS];

	/* for pending irqs */
	struct work_struct irq_pending_work;

	/* reboot notifier (for mysterious hangup problem at power-down) */
	struct notifier_block reboot_notifier;
};

/* driver types */
enum {
	AZX_DRIVER_ICH,
	AZX_DRIVER_PCH,
	AZX_DRIVER_SCH,
	AZX_DRIVER_ATI,
	AZX_DRIVER_ATIHDMI,
	AZX_DRIVER_VIA,
	AZX_DRIVER_SIS,
	AZX_DRIVER_ULI,
	AZX_DRIVER_NVIDIA,
	AZX_DRIVER_TERA,
	AZX_DRIVER_CTX,
	AZX_DRIVER_GENERIC,
	AZX_NUM_DRIVERS, /* keep this as last entry */
};

/*
 * macros for easy use
 */
#define azx_writel(chip,reg,value) \
	writel(value, (chip)->remap_addr + ICH6_REG_##reg)
#define azx_readl(chip,reg) \
	readl((chip)->remap_addr + ICH6_REG_##reg)

#define azx_sd_writel(dev, reg, value) \
	writel(value, (dev)->sd_addr + ICH6_REG_##reg)
#define azx_sd_readl(dev, reg) \
	readl((dev)->sd_addr + ICH6_REG_##reg)

#ifdef CONFIG_SND_HDA_TEGRA
#define MASK_D(reg)	(ICH6_REG_##reg & 0xfc)
#define MASK_W(reg)	(ICH6_REG_##reg & 0x02)
#define MASK_B(reg)	(ICH6_REG_##reg & 0x03)
#define MASK_L_B	0xff
#define MASK_L_W	0xffff
#define reg_mask_byte(idx) \
	((0xff000000 * ((idx == 3) ? 0 : 1)) + \
	(0xff0000 * ((idx == 2) ? 0 : 1)) + (0xff00 * ((idx == 1) ? 0 : 1)) \
					+ (0xff * ((idx == 0) ? 0 : 1)))
#define reg_mask_word(idx) \
	(0xffff0000 * ((idx == 2) ? 0 : 1) + 0xffff * ((idx == 0) ? 0 : 1))

#define nv_readl(chip, reg) \
	readl((chip)->remap_addr + reg)

#define nv_sd_readl(dev, reg) \
	readl((dev)->sd_addr + reg)

#define reg_shift(shiftbits) (shiftbits * 8)

#define azx_writew(chip, reg, value) \
	writel((((value) << reg_shift(MASK_W(reg))) + \
	(nv_readl(chip, MASK_D(reg)) & reg_mask_word(MASK_W(reg)))), \
	((chip)->remap_addr + MASK_D(reg)))

#define azx_readw(chip, reg) \
	((readl((chip)->remap_addr + MASK_D(reg)) >>\
		reg_shift(MASK_W(reg))) & MASK_L_W)

#define azx_writeb(chip, reg, value) \
	writel((((value) << reg_shift(MASK_B(reg))) + \
	(nv_readl(chip, MASK_D(reg)) & reg_mask_byte(MASK_B(reg)))), \
	((chip)->remap_addr + MASK_D(reg)))

#define azx_readb(chip, reg) \
	((readl((chip)->remap_addr + MASK_D(reg)) >>\
		reg_shift(MASK_B(reg))) & MASK_L_B)

#define azx_sd_writew(dev, reg, value) \
	writel((((value) << reg_shift(MASK_W(reg))) + \
	(nv_sd_readl(dev, MASK_D(reg)) & reg_mask_word(MASK_W(reg)))), \
	((dev)->sd_addr + MASK_D(reg)))

#define azx_sd_readw(dev, reg) \
	((readl((dev)->sd_addr + MASK_D(reg)) >>\
		reg_shift(MASK_W(reg))) & MASK_L_W)

#define azx_sd_writeb(dev, reg, value) \
	writel(((value) << reg_shift(MASK_B(reg))) + \
	(nv_sd_readl(dev, MASK_D(reg)) & reg_mask_byte(MASK_B(reg))), \
	((dev)->sd_addr + MASK_D(reg)))

#define azx_sd_readb(dev, reg) \
	((readl((dev)->sd_addr + MASK_D(reg)) >>\
		reg_shift(MASK_B(reg))) & MASK_L_B)

#else	/* CONFIG_SND_HDA_TEGRA */

#define azx_writew(chip,reg,value) \
	writew(value, (chip)->remap_addr + ICH6_REG_##reg)
#define azx_readw(chip,reg) \
	readw((chip)->remap_addr + ICH6_REG_##reg)
#define azx_writeb(chip,reg,value) \
	writeb(value, (chip)->remap_addr + ICH6_REG_##reg)
#define azx_readb(chip,reg) \
	readb((chip)->remap_addr + ICH6_REG_##reg)

#define azx_sd_writew(dev,reg,value) \
	writew(value, (dev)->sd_addr + ICH6_REG_##reg)
#define azx_sd_readw(dev,reg) \
	readw((dev)->sd_addr + ICH6_REG_##reg)
#define azx_sd_writeb(dev,reg,value) \
	writeb(value, (dev)->sd_addr + ICH6_REG_##reg)
#define azx_sd_readb(dev,reg) \
	readb((dev)->sd_addr + ICH6_REG_##reg)
#endif	/* CONFIG_SND_HDA_TEGRA */

/* for pcm support */
#define get_azx_dev(substream) (substream->runtime->private_data)

/*
 * Interface for HD codec
 */

/*
 * CORB / RIRB interface
 */
void azx_init_cmd_io(struct azx *chip);

void azx_free_cmd_io(struct azx *chip);

int azx_corb_send_cmd(struct hda_bus *bus, u32 val);

void azx_update_rirb(struct azx *chip);

unsigned int azx_rirb_get_response(struct hda_bus *bus, unsigned int addr);

int azx_single_send_cmd(struct hda_bus *bus, u32 val);

unsigned int azx_single_get_response(struct hda_bus *bus, unsigned int addr);

int azx_send_cmd(struct hda_bus *bus, unsigned int val);

unsigned int azx_get_response(struct hda_bus *bus, unsigned int addr);

#ifdef CONFIG_SND_HDA_POWER_SAVE
void azx_power_notify(struct hda_bus *bus);
#endif

int azx_reset(struct azx *chip, int full_reset);


void azx_int_enable(struct azx *chip);

void azx_int_disable(struct azx *chip);

void azx_int_clear(struct azx *chip);

void azx_stream_start(struct azx *chip, struct azx_dev *azx_dev);

void azx_stream_stop(struct azx *chip, struct azx_dev *azx_dev);

void azx_init_chip(struct azx *chip, int full_reset);

int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev);

irqreturn_t azx_interrupt(int irq, void *dev_id);

int setup_bdle(struct snd_pcm_substream *substream,
	      struct azx_dev *azx_dev, u32 **bdlp,
	      int ofs, int size, int with_ioc);

int azx_setup_periods(struct azx *chip,
			     struct snd_pcm_substream *substream,
			     struct azx_dev *azx_dev);

int azx_setup_controller(struct azx *chip, struct azx_dev *azx_dev);

int probe_codec(struct azx *chip, int addr);

void azx_stop_chip(struct azx *chip);

inline struct azx_dev *
azx_assign_device(struct azx *chip, struct snd_pcm_substream *substream);

inline void azx_release_device(struct azx_dev *azx_dev);

int azx_pcm_open(struct snd_pcm_substream *substream);

int azx_pcm_close(struct snd_pcm_substream *substream);

int azx_pcm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params);

int azx_pcm_hw_free(struct snd_pcm_substream *substream);

int azx_pcm_prepare(struct snd_pcm_substream *substream);

int azx_pcm_trigger(struct snd_pcm_substream *substream, int cmd);

unsigned int azx_via_get_position(struct azx *chip,
					 struct azx_dev *azx_dev);

unsigned int azx_get_position(struct azx *chip,
				     struct azx_dev *azx_dev);

snd_pcm_uframes_t azx_pcm_pointer(struct snd_pcm_substream *substream);

int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev);

void azx_irq_pending_work(struct work_struct *work);

void azx_clear_irq_pending(struct azx *chip);

void azx_pcm_free(struct snd_pcm *pcm);

int __devinit azx_mixer_create(struct azx *chip);

int __devinit azx_init_stream(struct azx *chip);

#ifdef CONFIG_SND_HDA_POWER_SAVE
void azx_power_notify(struct hda_bus *bus);
#endif /* CONFIG_SND_HDA_POWER_SAVE */

#ifdef CONFIG_PM
int snd_hda_codecs_inuse(struct hda_bus *bus);
#endif /* CONFIG_PM */

int azx_halt(struct notifier_block *nb, unsigned long event, void *buf);

void azx_notifier_register(struct azx *chip);

void azx_notifier_unregister(struct azx *chip);

void power_down_all_codecs(struct azx *chip);

int azx_acquire_irq(struct azx *chip, int do_disconnect);

void azx_bus_reset(struct hda_bus *bus);

int __devinit azx_codec_configure(struct azx *chip);

struct azx_pcm {
	struct azx *chip;
	struct hda_codec *codec;
	struct hda_pcm_stream *hinfo[2];
};

#endif /* __LINUX_LIB_HDA_INTEL_H__ */
