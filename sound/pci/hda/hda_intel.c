/*
 *
 *  hda_intel.c - Implementation of primary alsa driver code base
 *                for Intel HD Audio.
 *
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
 *  CHANGES:
 *
 *  2004.12.01	Major rewrite by tiwai, merged the work of pshou
 *
 */

#include "lib_hda_intel.h"

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static int enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;
static char *model[SNDRV_CARDS];
static int position_fix[SNDRV_CARDS];
static int bdl_pos_adj[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS-1)] = -1};
static int probe_mask[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS-1)] = -1};
static int probe_only[SNDRV_CARDS];
static int single_cmd;
static int enable_msi = -1;
#ifdef CONFIG_SND_HDA_PATCH_LOADER
static char *patch[SNDRV_CARDS];
#endif
#ifdef CONFIG_SND_HDA_INPUT_BEEP
static int beep_mode[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS-1)] =
					CONFIG_SND_HDA_INPUT_BEEP_MODE};
#endif

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for Intel HD audio interface.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for Intel HD audio interface.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable Intel HD audio interface.");
module_param_array(model, charp, NULL, 0444);
MODULE_PARM_DESC(model, "Use the given board model.");
module_param_array(position_fix, int, NULL, 0444);
MODULE_PARM_DESC(position_fix, "Fix DMA pointer "
		 "(0 = auto, 1 = none, 2 = POSBUF).");
module_param_array(bdl_pos_adj, int, NULL, 0644);
MODULE_PARM_DESC(bdl_pos_adj, "BDL position adjustment offset.");
module_param_array(probe_mask, int, NULL, 0444);
MODULE_PARM_DESC(probe_mask, "Bitmask to probe codecs (default = -1).");
module_param_array(probe_only, int, NULL, 0444);
MODULE_PARM_DESC(probe_only, "Only probing and no codec initialization.");
module_param(single_cmd, bool, 0444);
MODULE_PARM_DESC(single_cmd, "Use single command to communicate with codecs "
		 "(for debugging only).");
module_param(enable_msi, int, 0444);
MODULE_PARM_DESC(enable_msi, "Enable Message Signaled Interrupt (MSI)");
#ifdef CONFIG_SND_HDA_PATCH_LOADER
module_param_array(patch, charp, NULL, 0444);
MODULE_PARM_DESC(patch, "Patch file for Intel HD audio interface.");
#endif
#ifdef CONFIG_SND_HDA_INPUT_BEEP
module_param_array(beep_mode, int, NULL, 0444);
MODULE_PARM_DESC(beep_mode, "Select HDA Beep registration mode "
			    "(0=off, 1=on, 2=mute switch on/off) (default=1).");
#endif

#ifdef CONFIG_SND_HDA_POWER_SAVE
static int power_save = CONFIG_SND_HDA_POWER_SAVE_DEFAULT;
module_param(power_save, int, 0644);
MODULE_PARM_DESC(power_save, "Automatic power-saving timeout "
		 "(in second, 0 = disable).");

/* reset the HD-audio controller in power save mode.
 * this may give more power-saving, but will take longer time to
 * wake up.
 */
static int power_save_controller = 1;
module_param(power_save_controller, bool, 0644);
MODULE_PARM_DESC(power_save_controller, "Reset controller in power save mode.");
#endif

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

static char *driver_short_names[] __devinitdata = {
	[AZX_DRIVER_ICH] = "HDA Intel",
	[AZX_DRIVER_PCH] = "HDA Intel PCH",
	[AZX_DRIVER_SCH] = "HDA Intel MID",
	[AZX_DRIVER_ATI] = "HDA ATI SB",
	[AZX_DRIVER_ATIHDMI] = "HDA ATI HDMI",
	[AZX_DRIVER_VIA] = "HDA VIA VT82xx",
	[AZX_DRIVER_SIS] = "HDA SIS966",
	[AZX_DRIVER_ULI] = "HDA ULI M5461",
	[AZX_DRIVER_NVIDIA] = "HDA NVidia",
	[AZX_DRIVER_TERA] = "HDA Teradici", 
	[AZX_DRIVER_CTX] = "HDA Creative",
	[AZX_DRIVER_GENERIC] = "HD-Audio Generic",
};


/*
 * Interface for HD codec
 */

/*
 * CORB / RIRB interface
 */
static int azx_alloc_cmd_io(struct azx *chip)
{
	int err;

	/* single page (at least 4096 bytes) must suffice for both ringbuffes */
	err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  snd_dma_pci_data(chip->pci),
				  PAGE_SIZE, &chip->rb);
	if (err < 0) {
		snd_printk(KERN_ERR SFX "cannot allocate CORB/RIRB\n");
		return err;
	}
	return 0;
}




/*
 * initialize the PCI registers
 */
/* update bits in a PCI register byte */
static void update_pci_byte(struct pci_dev *pci, unsigned int reg,
			    unsigned char mask, unsigned char val)
{
	unsigned char data;

	pci_read_config_byte(pci, reg, &data);
	data &= ~mask;
	data |= (val & mask);
	pci_write_config_byte(pci, reg, data);
}

static void azx_init_pci(struct azx *chip)
{
	unsigned short snoop;

	/* Clear bits 0-2 of PCI register TCSEL (at offset 0x44)
	 * TCSEL == Traffic Class Select Register, which sets PCI express QOS
	 * Ensuring these bits are 0 clears playback static on some HD Audio
	 * codecs
	 */
	update_pci_byte(chip->pci, ICH6_PCIREG_TCSEL, 0x07, 0);

	switch (chip->driver_type) {
	case AZX_DRIVER_ATI:
		/* For ATI SB450 azalia HD audio, we need to enable snoop */
		update_pci_byte(chip->pci,
				ATI_SB450_HDAUDIO_MISC_CNTR2_ADDR, 
				0x07, ATI_SB450_HDAUDIO_ENABLE_SNOOP);
		break;
	case AZX_DRIVER_NVIDIA:
		/* For NVIDIA HDA, enable snoop */
		update_pci_byte(chip->pci,
				NVIDIA_HDA_TRANSREG_ADDR,
				0x0f, NVIDIA_HDA_ENABLE_COHBITS);
		update_pci_byte(chip->pci,
				NVIDIA_HDA_ISTRM_COH,
				0x01, NVIDIA_HDA_ENABLE_COHBIT);
		update_pci_byte(chip->pci,
				NVIDIA_HDA_OSTRM_COH,
				0x01, NVIDIA_HDA_ENABLE_COHBIT);
		break;
	case AZX_DRIVER_SCH:
	case AZX_DRIVER_PCH:
		pci_read_config_word(chip->pci, INTEL_SCH_HDA_DEVC, &snoop);
		if (snoop & INTEL_SCH_HDA_DEVC_NOSNOOP) {
			pci_write_config_word(chip->pci, INTEL_SCH_HDA_DEVC,
				snoop & (~INTEL_SCH_HDA_DEVC_NOSNOOP));
			pci_read_config_word(chip->pci,
				INTEL_SCH_HDA_DEVC, &snoop);
			snd_printdd(SFX "HDA snoop disabled, enabling ... %s\n",
				(snoop & INTEL_SCH_HDA_DEVC_NOSNOOP)
				? "Failed" : "OK");
		}
		break;

        }
}


static int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev);


/*
 * set up BDL entries
 */
int azx_setup_periods(struct azx *chip,
			     struct snd_pcm_substream *substream,
			     struct azx_dev *azx_dev)
{
	u32 *bdl;
	int i, ofs, periods, period_bytes;
	int pos_adj;

	/* reset BDL address */
	azx_sd_writel(azx_dev, SD_BDLPL, 0);
	azx_sd_writel(azx_dev, SD_BDLPU, 0);

	period_bytes = azx_dev->period_bytes;
	periods = azx_dev->bufsize / period_bytes;

	/* program the initial BDL entries */
	bdl = (u32 *)azx_dev->bdl.area;
	ofs = 0;
	azx_dev->frags = 0;
	pos_adj = bdl_pos_adj[chip->dev_index];
	if (pos_adj > 0) {
		struct snd_pcm_runtime *runtime = substream->runtime;
		int pos_align = pos_adj;
		pos_adj = (pos_adj * runtime->rate + 47999) / 48000;
		if (!pos_adj)
			pos_adj = pos_align;
		else
			pos_adj = ((pos_adj + pos_align - 1) / pos_align) *
				pos_align;
		pos_adj = frames_to_bytes(runtime, pos_adj);
		if (pos_adj >= period_bytes) {
			snd_printk(KERN_WARNING SFX "Too big adjustment %d\n",
				   bdl_pos_adj[chip->dev_index]);
			pos_adj = 0;
		} else {
			ofs = setup_bdle(substream, azx_dev,
					 &bdl, ofs, pos_adj, 1);
			if (ofs < 0)
				goto error;
		}
	} else
		pos_adj = 0;
	for (i = 0; i < periods; i++) {
		if (i == periods - 1 && pos_adj)
			ofs = setup_bdle(substream, azx_dev, &bdl, ofs,
					 period_bytes - pos_adj, 0);
		else
			ofs = setup_bdle(substream, azx_dev, &bdl, ofs,
					 period_bytes, 1);
		if (ofs < 0)
			goto error;
	}
	return 0;

 error:
	snd_printk(KERN_ERR SFX "Too many BDL entries: buffer=%d, period=%d\n",
		   azx_dev->bufsize, period_bytes);
	return -EINVAL;
}
	}
}

static int azx_attach_pcm_stream(struct hda_bus *bus, struct hda_codec *codec,
				 struct hda_pcm *cpcm);

/*
 * Codec initialization
 */

/* number of codec slots for each chipset: 0 = default slots (i.e. 4) */
static unsigned int azx_max_codecs[AZX_NUM_DRIVERS] __devinitdata = {
	[AZX_DRIVER_NVIDIA] = 8,
	[AZX_DRIVER_TERA] = 1,
};

static int __devinit azx_codec_create(struct azx *chip, const char *model)
{
	struct hda_bus_template bus_temp;
	int c, codecs, err;
	int max_slots;

	memset(&bus_temp, 0, sizeof(bus_temp));
	bus_temp.private_data = chip;
	bus_temp.modelname = model;
	bus_temp.pci = chip->pci;
	bus_temp.ops.command = azx_send_cmd;
	bus_temp.ops.get_response = azx_get_response;
	bus_temp.ops.attach_pcm = azx_attach_pcm_stream;
	bus_temp.ops.bus_reset = azx_bus_reset;
#ifdef CONFIG_SND_HDA_POWER_SAVE
	bus_temp.power_save = &power_save;
	bus_temp.ops.pm_notify = azx_power_notify;
#endif

	err = snd_hda_bus_new(chip->card, &bus_temp, &chip->bus);
	if (err < 0)
		return err;

	if (chip->driver_type == AZX_DRIVER_NVIDIA)
		chip->bus->needs_damn_long_delay = 1;

	codecs = 0;
	max_slots = azx_max_codecs[chip->driver_type];
	if (!max_slots)
		max_slots = AZX_DEFAULT_CODECS;

	/* First try to probe all given codec slots */
	for (c = 0; c < max_slots; c++) {
		if ((chip->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			if (probe_codec(chip, c) < 0) {
				/* Some BIOSen give you wrong codec addresses
				 * that don't exist
				 */
				snd_printk(KERN_WARNING SFX
					   "Codec #%d probe error; "
					   "disabling it...\n", c);
				chip->codec_mask &= ~(1 << c);
				/* More badly, accessing to a non-existing
				 * codec often screws up the controller chip,
				 * and disturbs the further communications.
				 * Thus if an error occurs during probing,
				 * better to reset the controller chip to
				 * get back to the sanity state.
				 */
				azx_stop_chip(chip);
				azx_init_chip(chip, 1);
			}
		}
	}

	/* Then create codec instances */
	for (c = 0; c < max_slots; c++) {
		if ((chip->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			struct hda_codec *codec;
			err = snd_hda_codec_new(chip->bus, c, &codec);
			if (err < 0)
				continue;
			codec->beep_mode = chip->beep_mode;
			codecs++;
		}
	}
	if (!codecs) {
		snd_printk(KERN_ERR SFX "no codecs initialized\n");
		return -ENXIO;
	}
	return 0;
}
/*
 * Check whether the current DMA position is acceptable for updating
 * periods.  Returns non-zero if it's OK.
 *
 * Many HD-audio controllers appear pretty inaccurate about
 * the update-IRQ timing.  The IRQ is issued before actually the
 * data is processed.  So, we need to process it afterwords in a
 * workqueue.
 */
int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev)
{
	u32 wallclk;
	unsigned int pos;
	int stream;

	wallclk = azx_readl(chip, WALLCLK) - azx_dev->start_wallclk;
	if (wallclk < (azx_dev->period_wallclk * 2) / 3)
		return -1;	/* bogus (too early) interrupt */

	stream = azx_dev->substream->stream;
	pos = azx_get_position(chip, azx_dev);
	if (chip->position_fix[stream] == POS_FIX_AUTO) {
		if (!pos) {
			printk(KERN_WARNING
			       "hda-intel: Invalid position buffer, "
			       "using LPIB read method instead.\n");
			chip->position_fix[stream] = POS_FIX_LPIB;
			pos = azx_get_position(chip, azx_dev);
		} else
			chip->position_fix[stream] = POS_FIX_POSBUF;
	}

	if (WARN_ONCE(!azx_dev->period_bytes,
		      "hda-intel: zero azx_dev->period_bytes"))
		return -1; /* this shouldn't happen! */
	if (wallclk < (azx_dev->period_wallclk * 5) / 4 &&
	    pos % azx_dev->period_bytes > azx_dev->period_bytes / 2)
		/* NG - it's below the first next period boundary */
		return bdl_pos_adj[chip->dev_index] ? 0 : -1;
	azx_dev->start_wallclk += wallclk;
	return 1; /* OK, it's fine */
}

static int
azx_attach_pcm_stream(struct hda_bus *bus, struct hda_codec *codec,
		      struct hda_pcm *cpcm)
{
	struct azx *chip = bus->private_data;
	struct snd_pcm *pcm;
	struct azx_pcm *apcm;
	int pcm_dev = cpcm->device;
	int s, err;

	if (pcm_dev >= HDA_MAX_PCMS) {
		snd_printk(KERN_ERR SFX "Invalid PCM device number %d\n",
			   pcm_dev);
		return -EINVAL;
	}
	if (chip->pcm[pcm_dev]) {
		snd_printk(KERN_ERR SFX "PCM %d already exists\n", pcm_dev);
		return -EBUSY;
	}
	err = snd_pcm_new(chip->card, cpcm->name, pcm_dev,
			  cpcm->stream[SNDRV_PCM_STREAM_PLAYBACK].substreams,
			  cpcm->stream[SNDRV_PCM_STREAM_CAPTURE].substreams,
			  &pcm);
	if (err < 0)
		return err;
	strlcpy(pcm->name, cpcm->name, sizeof(pcm->name));
	apcm = kzalloc(sizeof(*apcm), GFP_KERNEL);
	if (apcm == NULL)
		return -ENOMEM;
	apcm->chip = chip;
	apcm->codec = codec;
	pcm->private_data = apcm;
	pcm->private_free = azx_pcm_free;
	if (cpcm->pcm_type == HDA_PCM_TYPE_MODEM)
		pcm->dev_class = SNDRV_PCM_CLASS_MODEM;
	chip->pcm[pcm_dev] = pcm;
	cpcm->pcm = pcm;
	for (s = 0; s < 2; s++) {
		apcm->hinfo[s] = &cpcm->stream[s];
		if (cpcm->stream[s].substreams)
			snd_pcm_set_ops(pcm, s, &azx_pcm_ops);
	}
	/* buffer pre-allocation */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV_SG,
					      snd_dma_pci_data(chip->pci),
					      1024 * 64, 32 * 1024 * 1024);
	return 0;
}

/*

#ifdef CONFIG_PM
/*
 * power management
 */

static int azx_suspend(struct pci_dev *pci, pm_message_t state)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct azx *chip = card->private_data;
	int i;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	azx_clear_irq_pending(chip);
	for (i = 0; i < HDA_MAX_PCMS; i++)
		snd_pcm_suspend_all(chip->pcm[i]);
	if (chip->initialized)
		snd_hda_suspend(chip->bus);
	azx_stop_chip(chip);
	if (chip->irq >= 0) {
		free_irq(chip->irq, chip);
		chip->irq = -1;
	}
	if (chip->msi)
		pci_disable_msi(chip->pci);
	pci_disable_device(pci);
	pci_save_state(pci);
	pci_set_power_state(pci, pci_choose_state(pci, state));
	return 0;
}

static int azx_resume(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct azx *chip = card->private_data;

	pci_set_power_state(pci, PCI_D0);
	pci_restore_state(pci);
	if (pci_enable_device(pci) < 0) {
		printk(KERN_ERR "hda-intel: pci_enable_device failed, "
		       "disabling device\n");
		snd_card_disconnect(card);
		return -EIO;
	}
	pci_set_master(pci);
	if (chip->msi)
		if (pci_enable_msi(pci) < 0)
			chip->msi = 0;
	if (azx_acquire_irq(chip, 1) < 0)
		return -EIO;
	azx_init_pci(chip);

	if (snd_hda_codecs_inuse(chip->bus))
		azx_init_chip(chip, 1);

	snd_hda_resume(chip->bus);
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	return 0;
}
#endif /* CONFIG_PM */

/*
 * destructor
 */
static int azx_free(struct azx *chip)
{
	int i;

	azx_notifier_unregister(chip);

	if (chip->initialized) {
		azx_clear_irq_pending(chip);
		for (i = 0; i < chip->num_streams; i++)
			azx_stream_stop(chip, &chip->azx_dev[i]);
		azx_stop_chip(chip);
	}

	if (chip->irq >= 0)
		free_irq(chip->irq, (void *)chip);
	if (chip->msi)
		pci_disable_msi(chip->pci);
	if (chip->remap_addr)
		iounmap(chip->remap_addr);

	if (chip->azx_dev) {
		for (i = 0; i < chip->num_streams; i++)
			if (chip->azx_dev[i].bdl.area)
				snd_dma_free_pages(&chip->azx_dev[i].bdl);
	}
	if (chip->rb.area)
		snd_dma_free_pages(&chip->rb);
	if (chip->posbuf.area)
		snd_dma_free_pages(&chip->posbuf);
	pci_release_regions(chip->pci);
	pci_disable_device(chip->pci);
	kfree(chip->azx_dev);
	kfree(chip);

	return 0;
}

static int azx_dev_free(struct snd_device *device)
{
	return azx_free(device->device_data);
}

/*
 * white/black-listing for position_fix
 */
static struct snd_pci_quirk position_fix_list[] __devinitdata = {
	SND_PCI_QUIRK(0x1025, 0x009f, "Acer Aspire 5110", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1025, 0x026f, "Acer Aspire 5538", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1028, 0x01cc, "Dell D820", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1028, 0x01de, "Dell Precision 390", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1028, 0x01f6, "Dell Latitude 131L", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1028, 0x0470, "Dell Inspiron 1120", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x103c, 0x306d, "HP dv3", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1043, 0x813d, "ASUS P5AD2", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1043, 0x81b3, "ASUS", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1043, 0x81e7, "ASUS M2V", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x104d, 0x9069, "Sony VPCS11V9E", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1106, 0x3288, "ASUS M2V-MX SE", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1179, 0xff10, "Toshiba A100-259", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1297, 0x3166, "Shuttle", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1458, 0xa022, "ga-ma770-ud3", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1462, 0x1002, "MSI Wind U115", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1565, 0x820f, "Biostar Microtech", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1565, 0x8218, "Biostar Microtech", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1849, 0x0888, "775Dual-VSTA", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x8086, 0x2503, "DG965OT AAD63733-203", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x8086, 0xd601, "eMachines T5212", POS_FIX_LPIB),
	{}
};

static int __devinit check_position_fix(struct azx *chip, int fix)
{
	const struct snd_pci_quirk *q;

	switch (fix) {
	case POS_FIX_LPIB:
	case POS_FIX_POSBUF:
		return fix;
	}

	/* Check VIA/ATI HD Audio Controller exist */
	switch (chip->driver_type) {
	case AZX_DRIVER_VIA:
	case AZX_DRIVER_ATI:
		chip->via_dmapos_patch = 1;
		/* Use link position directly, avoid any transfer problem. */
		return POS_FIX_LPIB;
	}
	chip->via_dmapos_patch = 0;

	q = snd_pci_quirk_lookup(chip->pci, position_fix_list);
	if (q) {
		printk(KERN_INFO
		       "hda_intel: position_fix set to %d "
		       "for device %04x:%04x\n",
		       q->value, q->subvendor, q->subdevice);
		return q->value;
	}
	return POS_FIX_AUTO;
}

/*
 * black-lists for probe_mask
 */
static struct snd_pci_quirk probe_mask_list[] __devinitdata = {
	/* Thinkpad often breaks the controller communication when accessing
	 * to the non-working (or non-existing) modem codec slot.
	 */
	SND_PCI_QUIRK(0x1014, 0x05b7, "Thinkpad Z60", 0x01),
	SND_PCI_QUIRK(0x17aa, 0x2010, "Thinkpad X/T/R60", 0x01),
	SND_PCI_QUIRK(0x17aa, 0x20ac, "Thinkpad X/T/R61", 0x01),
	/* broken BIOS */
	SND_PCI_QUIRK(0x1028, 0x20ac, "Dell Studio Desktop", 0x01),
	/* including bogus ALC268 in slot#2 that conflicts with ALC888 */
	SND_PCI_QUIRK(0x17c0, 0x4085, "Medion MD96630", 0x01),
	/* forced codec slots */
	SND_PCI_QUIRK(0x1043, 0x1262, "ASUS W5Fm", 0x103),
	SND_PCI_QUIRK(0x1046, 0x1262, "ASUS W5F", 0x103),
	{}
};

#define AZX_FORCE_CODEC_MASK	0x100

static void __devinit check_probe_mask(struct azx *chip, int dev)
{
	const struct snd_pci_quirk *q;

	chip->codec_probe_mask = probe_mask[dev];
	if (chip->codec_probe_mask == -1) {
		q = snd_pci_quirk_lookup(chip->pci, probe_mask_list);
		if (q) {
			printk(KERN_INFO
			       "hda_intel: probe_mask set to 0x%x "
			       "for device %04x:%04x\n",
			       q->value, q->subvendor, q->subdevice);
			chip->codec_probe_mask = q->value;
		}
	}

	/* check forced option */
	if (chip->codec_probe_mask != -1 &&
	    (chip->codec_probe_mask & AZX_FORCE_CODEC_MASK)) {
		chip->codec_mask = chip->codec_probe_mask & 0xff;
		printk(KERN_INFO "hda_intel: codec_mask forced to 0x%x\n",
		       chip->codec_mask);
	}
}

/*
 * white/black-list for enable_msi
 */
static struct snd_pci_quirk msi_black_list[] __devinitdata = {
	SND_PCI_QUIRK(0x1043, 0x81f2, "ASUS", 0), /* Athlon64 X2 + nvidia */
	SND_PCI_QUIRK(0x1043, 0x81f6, "ASUS", 0), /* nvidia */
	SND_PCI_QUIRK(0x1043, 0x822d, "ASUS", 0), /* Athlon64 X2 + nvidia MCP55 */
	SND_PCI_QUIRK(0x1849, 0x0888, "ASRock", 0), /* Athlon64 X2 + nvidia */
	SND_PCI_QUIRK(0xa0a0, 0x0575, "Aopen MZ915-M", 0), /* ICH6 */
	{}
};

static void __devinit check_msi(struct azx *chip)
{
	const struct snd_pci_quirk *q;

	if (enable_msi >= 0) {
		chip->msi = !!enable_msi;
		return;
	}
	chip->msi = 1;	/* enable MSI as default */
	q = snd_pci_quirk_lookup(chip->pci, msi_black_list);
	if (q) {
		printk(KERN_INFO
		       "hda_intel: msi for device %04x:%04x set to %d\n",
		       q->subvendor, q->subdevice, q->value);
		chip->msi = q->value;
		return;
	}

	/* NVidia chipsets seem to cause troubles with MSI */
	if (chip->driver_type == AZX_DRIVER_NVIDIA) {
		printk(KERN_INFO "hda_intel: Disable MSI for Nvidia chipset\n");
		chip->msi = 0;
	}
}


/*
 * constructor
 */
static int __devinit azx_create(struct snd_card *card, struct pci_dev *pci,
				int dev, int driver_type,
				struct azx **rchip)
{
	struct azx *chip;
	int i, err;
	unsigned short gcap;
	static struct snd_device_ops ops = {
		.dev_free = azx_dev_free,
	};

	*rchip = NULL;

	err = pci_enable_device(pci);
	if (err < 0)
		return err;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		snd_printk(KERN_ERR SFX "cannot allocate chip\n");
		pci_disable_device(pci);
		return -ENOMEM;
	}

	spin_lock_init(&chip->reg_lock);
	mutex_init(&chip->open_mutex);
	chip->card = card;
	chip->pci = pci;
	chip->irq = -1;
	chip->driver_type = driver_type;
	check_msi(chip);
	chip->dev_index = dev;
	INIT_WORK(&chip->irq_pending_work, azx_irq_pending_work);

	chip->position_fix[0] = chip->position_fix[1] =
		check_position_fix(chip, position_fix[dev]);
	check_probe_mask(chip, dev);

	chip->single_cmd = single_cmd;

	if (bdl_pos_adj[dev] < 0) {
		switch (chip->driver_type) {
		case AZX_DRIVER_ICH:
		case AZX_DRIVER_PCH:
			bdl_pos_adj[dev] = 1;
			break;
		default:
			bdl_pos_adj[dev] = 32;
			break;
		}
	}

#if BITS_PER_LONG != 64
	/* Fix up base address on ULI M5461 */
	if (chip->driver_type == AZX_DRIVER_ULI) {
		u16 tmp3;
		pci_read_config_word(pci, 0x40, &tmp3);
		pci_write_config_word(pci, 0x40, tmp3 | 0x10);
		pci_write_config_dword(pci, PCI_BASE_ADDRESS_1, 0);
	}
#endif

	err = pci_request_regions(pci, "ICH HD audio");
	if (err < 0) {
		kfree(chip);
		pci_disable_device(pci);
		return err;
	}

	chip->addr = pci_resource_start(pci, 0);
	chip->remap_addr = pci_ioremap_bar(pci, 0);
	if (chip->remap_addr == NULL) {
		snd_printk(KERN_ERR SFX "ioremap error\n");
		err = -ENXIO;
		goto errout;
	}

	if (chip->msi)
		if (pci_enable_msi(pci) < 0)
			chip->msi = 0;

	if (azx_acquire_irq(chip, 0) < 0) {
		err = -EBUSY;
		goto errout;
	}

	pci_set_master(pci);
	synchronize_irq(chip->irq);

	gcap = azx_readw(chip, GCAP);
	snd_printdd(SFX "chipset global capabilities = 0x%x\n", gcap);

	/* disable SB600 64bit support for safety */
	if ((chip->driver_type == AZX_DRIVER_ATI) ||
	    (chip->driver_type == AZX_DRIVER_ATIHDMI)) {
		struct pci_dev *p_smbus;
		p_smbus = pci_get_device(PCI_VENDOR_ID_ATI,
					 PCI_DEVICE_ID_ATI_SBX00_SMBUS,
					 NULL);
		if (p_smbus) {
			if (p_smbus->revision < 0x30)
				gcap &= ~ICH6_GCAP_64OK;
			pci_dev_put(p_smbus);
		}
	}

	/* disable 64bit DMA address for Teradici */
	/* it does not work with device 6549:1200 subsys e4a2:040b */
	if (chip->driver_type == AZX_DRIVER_TERA)
		gcap &= ~ICH6_GCAP_64OK;

	/* allow 64bit DMA address if supported by H/W */
	if ((gcap & ICH6_GCAP_64OK) && !pci_set_dma_mask(pci, DMA_BIT_MASK(64)))
		pci_set_consistent_dma_mask(pci, DMA_BIT_MASK(64));
	else {
		pci_set_dma_mask(pci, DMA_BIT_MASK(32));
		pci_set_consistent_dma_mask(pci, DMA_BIT_MASK(32));
	}

	/* read number of streams from GCAP register instead of using
	 * hardcoded value
	 */
	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	if (!chip->playback_streams && !chip->capture_streams) {
		/* gcap didn't give any info, switching to old method */

		switch (chip->driver_type) {
		case AZX_DRIVER_ULI:
			chip->playback_streams = ULI_NUM_PLAYBACK;
			chip->capture_streams = ULI_NUM_CAPTURE;
			break;
		case AZX_DRIVER_ATIHDMI:
			chip->playback_streams = ATIHDMI_NUM_PLAYBACK;
			chip->capture_streams = ATIHDMI_NUM_CAPTURE;
			break;
		case AZX_DRIVER_GENERIC:
		default:
			chip->playback_streams = ICH6_NUM_PLAYBACK;
			chip->capture_streams = ICH6_NUM_CAPTURE;
			break;
		}
	}
	chip->capture_index_offset = 0;
	chip->playback_index_offset = chip->capture_streams;
	chip->num_streams = chip->playback_streams + chip->capture_streams;
	chip->azx_dev = kcalloc(chip->num_streams, sizeof(*chip->azx_dev),
				GFP_KERNEL);
	if (!chip->azx_dev) {
		snd_printk(KERN_ERR SFX "cannot malloc azx_dev\n");
		goto errout;
	}

	for (i = 0; i < chip->num_streams; i++) {
		/* allocate memory for the BDL for each stream */
		err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
					  snd_dma_pci_data(chip->pci),
					  BDL_SIZE, &chip->azx_dev[i].bdl);
		if (err < 0) {
			snd_printk(KERN_ERR SFX "cannot allocate BDL\n");
			goto errout;
		}
	}
	/* allocate memory for the position buffer */
	err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  snd_dma_pci_data(chip->pci),
				  chip->num_streams * 8, &chip->posbuf);
	if (err < 0) {
		snd_printk(KERN_ERR SFX "cannot allocate posbuf\n");
		goto errout;
	}
	/* allocate CORB/RIRB */
	err = azx_alloc_cmd_io(chip);
	if (err < 0)
		goto errout;

	/* initialize streams */
	azx_init_stream(chip);

	/* initialize chip */
	azx_init_pci(chip);
	azx_init_chip(chip, (probe_only[dev] & 2) == 0);

	/* codec detection */
	if (!chip->codec_mask) {
		snd_printk(KERN_ERR SFX "no codecs found!\n");
		err = -ENODEV;
		goto errout;
	}

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err <0) {
		snd_printk(KERN_ERR SFX "Error creating device [card]!\n");
		goto errout;
	}

	strcpy(card->driver, "HDA-Intel");
	strlcpy(card->shortname, driver_short_names[chip->driver_type],
		sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname),
		 "%s at 0x%lx irq %i",
		 card->shortname, chip->addr, chip->irq);

	*rchip = chip;
	return 0;

 errout:
	azx_free(chip);
	return err;
}

static int __devinit azx_probe(struct pci_dev *pci,
			       const struct pci_device_id *pci_id)
{
	static int dev;
	struct snd_card *card;
	struct azx *chip;
	int err;

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

	err = snd_card_create(index[dev], id[dev], THIS_MODULE, 0, &card);
	if (err < 0) {
		snd_printk(KERN_ERR SFX "Error creating card!\n");
		return err;
	}

	/* set this here since it's referred in snd_hda_load_patch() */
	snd_card_set_dev(card, &pci->dev);

	err = azx_create(card, pci, dev, pci_id->driver_data, &chip);
	if (err < 0)
		goto out_free;
	card->private_data = chip;

#ifdef CONFIG_SND_HDA_INPUT_BEEP
	chip->beep_mode = beep_mode[dev];
#endif

	/* create codec instances */
	err = azx_codec_create(chip, model[dev]);
	if (err < 0)
		goto out_free;
#ifdef CONFIG_SND_HDA_PATCH_LOADER
	if (patch[dev]) {
		snd_printk(KERN_ERR SFX "Applying patch firmware '%s'\n",
			   patch[dev]);
		err = snd_hda_load_patch(chip->bus, patch[dev]);
		if (err < 0)
			goto out_free;
	}
#endif
	if ((probe_only[dev] & 1) == 0) {
		err = azx_codec_configure(chip);
		if (err < 0)
			goto out_free;
	}

	/* create PCM streams */
	err = snd_hda_build_pcms(chip->bus);
	if (err < 0)
		goto out_free;

	/* create mixer controls */
	err = azx_mixer_create(chip);
	if (err < 0)
		goto out_free;

	err = snd_card_register(card);
	if (err < 0)
		goto out_free;

	pci_set_drvdata(pci, card);
	chip->running = 1;
	power_down_all_codecs(chip);
	azx_notifier_register(chip);

	dev++;
	return err;
out_free:
	snd_card_free(card);
	return err;
}

static void __devexit azx_remove(struct pci_dev *pci)
{
	snd_card_free(pci_get_drvdata(pci));
	pci_set_drvdata(pci, NULL);
}

/* PCI IDs */
static DEFINE_PCI_DEVICE_TABLE(azx_ids) = {
	/* ICH 6..10 */
	{ PCI_DEVICE(0x8086, 0x2668), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x27d8), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x269a), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x284b), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x2911), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x293e), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x293f), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x3a3e), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x3a6e), .driver_data = AZX_DRIVER_ICH },
	/* PCH */
	{ PCI_DEVICE(0x8086, 0x3b56), .driver_data = AZX_DRIVER_ICH },
	{ PCI_DEVICE(0x8086, 0x3b57), .driver_data = AZX_DRIVER_ICH },
	/* CPT */
	{ PCI_DEVICE(0x8086, 0x1c20), .driver_data = AZX_DRIVER_PCH },
	/* PBG */
	{ PCI_DEVICE(0x8086, 0x1d20), .driver_data = AZX_DRIVER_PCH },
	/* SCH */
	{ PCI_DEVICE(0x8086, 0x811b), .driver_data = AZX_DRIVER_SCH },
	/* ATI SB 450/600 */
	{ PCI_DEVICE(0x1002, 0x437b), .driver_data = AZX_DRIVER_ATI },
	{ PCI_DEVICE(0x1002, 0x4383), .driver_data = AZX_DRIVER_ATI },
	/* ATI HDMI */
	{ PCI_DEVICE(0x1002, 0x793b), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0x7919), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0x960f), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0x970f), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa00), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa08), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa10), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa18), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa20), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa28), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa30), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa38), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa40), .driver_data = AZX_DRIVER_ATIHDMI },
	{ PCI_DEVICE(0x1002, 0xaa48), .driver_data = AZX_DRIVER_ATIHDMI },
	/* VIA VT8251/VT8237A */
	{ PCI_DEVICE(0x1106, 0x3288), .driver_data = AZX_DRIVER_VIA },
	/* SIS966 */
	{ PCI_DEVICE(0x1039, 0x7502), .driver_data = AZX_DRIVER_SIS },
	/* ULI M5461 */
	{ PCI_DEVICE(0x10b9, 0x5461), .driver_data = AZX_DRIVER_ULI },
	/* NVIDIA MCP */
	{ PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, PCI_ANY_ID),
	  .class = PCI_CLASS_MULTIMEDIA_HD_AUDIO << 8,
	  .class_mask = 0xffffff,
	  .driver_data = AZX_DRIVER_NVIDIA },
	/* Teradici */
	{ PCI_DEVICE(0x6549, 0x1200), .driver_data = AZX_DRIVER_TERA },
	/* Creative X-Fi (CA0110-IBG) */
#if !defined(CONFIG_SND_CTXFI) && !defined(CONFIG_SND_CTXFI_MODULE)
	/* the following entry conflicts with snd-ctxfi driver,
	 * as ctxfi driver mutates from HD-audio to native mode with
	 * a special command sequence.
	 */
	{ PCI_DEVICE(PCI_VENDOR_ID_CREATIVE, PCI_ANY_ID),
	  .class = PCI_CLASS_MULTIMEDIA_HD_AUDIO << 8,
	  .class_mask = 0xffffff,
	  .driver_data = AZX_DRIVER_CTX },
#else
	/* this entry seems still valid -- i.e. without emu20kx chip */
	{ PCI_DEVICE(0x1102, 0x0009), .driver_data = AZX_DRIVER_CTX },
#endif
	/* AMD/ATI Generic, PCI class code and Vendor ID for HD Audio */
	{ PCI_DEVICE(PCI_VENDOR_ID_ATI, PCI_ANY_ID),
	  .class = PCI_CLASS_MULTIMEDIA_HD_AUDIO << 8,
	  .class_mask = 0xffffff,
	  .driver_data = AZX_DRIVER_GENERIC },
	{ PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_ANY_ID),
	  .class = PCI_CLASS_MULTIMEDIA_HD_AUDIO << 8,
	  .class_mask = 0xffffff,
	  .driver_data = AZX_DRIVER_GENERIC },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, azx_ids);

/* pci_driver definition */
static struct pci_driver driver = {
	.name = "HDA Intel",
	.id_table = azx_ids,
	.probe = azx_probe,
	.remove = __devexit_p(azx_remove),
#ifdef CONFIG_PM
	.suspend = azx_suspend,
	.resume = azx_resume,
#endif
};

static int __init alsa_card_azx_init(void)
{
	return pci_register_driver(&driver);
}

static void __exit alsa_card_azx_exit(void)
{
	pci_unregister_driver(&driver);
}

module_init(alsa_card_azx_init)
module_exit(alsa_card_azx_exit)
