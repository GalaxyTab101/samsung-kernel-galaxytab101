/*
 *
 *  hda_tegra.c - Implementation of primary alsa driver code base
 *                for T30 HD Audio.
 *
 *  Copyright(c) 2004 Intel Corporation. All rights reserved.
 *
 *  Copyright (c) 2004 Takashi Iwai <tiwai@suse.de>
 *                     PeiSen Hou <pshou@realtek.com.tw>
 *  Copyright (C) 2010 NVIDIA Corporation.
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

#include "../../pci/hda/hda_codec.h"
#include "../../pci/hda/lib_hda_intel.h"
#include <linux/clk.h>
#include <mach/clk.h>

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static int enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;
static char *model[SNDRV_CARDS];
static int position_fix[SNDRV_CARDS];
static int bdl_pos_adj[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS-1)] = -1};
static int probe_mask[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS-1)] = -1};
static int probe_only[SNDRV_CARDS];
static int single_cmd;
static int enable_msi;

/* Module clock info */
static struct clk *clk_hda,  *clk_hda2codec , *clk_hda2hdmicodec;

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

#define T30SFX	"hda-tegra: "

#define TEGRA_HDA_BAR0_OFFSET           0x8000
#define TEGRA_HDA_BAR0_SIZE             0x4000
#define TEGRA_HDA_CONFIG_OFFSET         0x1000
#define TEGRA_HDA_CONFIG_SIZE           0x1000

#define NV_TEGRA_HDA_CFG_CMD_OFFSET           0x04
#define NV_TEGRA_HDA_CFG_BAR0_OFFSET          0x10

#define NV_TEGRA_HDA_ENABLE_IO_SPACE          (1 << 0)
#define NV_TEGRA_HDA_ENABLE_MEM_SPACE         (1 << 1)
#define NV_TEGRA_HDA_ENABLE_BUS_MASTER        (1 << 2)
#define NV_TEGRA_HDA_ENABLE_SERR              (1 << 8)
#define NV_TEGRA_HDA_DISABLE_INTR             (1 << 10)
#define NV_TEGRA_HDA_BAR0_INIT_PROGRAM        0xFFFFFFFF
#define NV_TEGRA_HDA_BAR0_FINAL_PROGRAM       (1 << 14)

#define CLK_RST_HDA2CODEC  (1<<15)
#define CLK_RST_HDA  (1<<29)

/* Define core/link clocks */
#define NV_TEGRA_HDA_CORE_CLOCK_FREQ_KHZ      (108*1000)
#define NV_TEGRA_HDA_LINK_CLOCK_FREQ_KHZ      (48*1000)

/* IPFS */
#define IPFS_EN_FPCI                          0x1
#define IPFS_HDA_CONFIGURATION_0              0x180
#define IPFS_HDA_FPCI_BAR0                    0x80
#define IPFS_HDA_INTR_MASK                    0x188
#define FPCI_BAR0_START                       0x40

#define TEGRA_HDA_CONFIG_OFFSET         0x1000

#define TIMEOUT_CODEC	50

static u32 alc269_verb_table[] = {
	/* HDA Codec Subsystem ID Verb-table */
	/* HDA Codec Subsystem ID  : 0x00000000 */
	0x10172000, 0x10172100, 0x10172200, 0x10172300,
	/* Pin Widget Verb-table */
	/* Pin Complex (NID 0x12 )*/
	0x11271CF0, 0x11271D01, 0x11271ED3, 0x11271F01,
	/* Pin Complex (NID 0x14 ) */
	0x11471CF0, 0x11471D01, 0x11471E11, 0x11471F90,
	/* Pin Complex (NID 0x21 ) */
	0x12171CF0, 0x12171D01, 0x12171E11, 0x12171F40,
	/* Pin Complex (NID 0x17 ) */
	0x11771CF0, 0x11771D01, 0x11771E11, 0x11771F40,
	/* Pin Complex (NID 0x18 ) */
	0x11871CF0, 0x11871D91, 0x11871EA1, 0x11871F01,
	/* Pin Complex (NID 0x19 ) */
	0x11971CF0, 0x11971D01, 0x11971E13, 0x11971F90,
	/* Pin Complex (NID 0x1A ) */
	0x11A71CF0, 0x11A71D41, 0x11A71E01, 0x11A71F01,
	/* Pin Complex (NID 0x1B ) */
	0x11B71CF0, 0x11B71D31, 0x11B71E81, 0x11B71F01,
	/* Pin Complex (NID 0x1D ) */
	0x11D71CF0, 0x11D71D01, 0x11D71E83, 0x11D71F40,
	/* Pin Complex (NID 0x1E ) */
	0x11E71CF0, 0x11E71D11, 0x11E71E45, 0x11E71F01,
	/* Pin Complex (NID 0x20 ) */
	0x12050011, 0x12040010, 0x12050012, 0x12041901,
	/* Pin Complex (NID 0x20 ) - 1 */
	0x12050002, 0x1204AAB8, 0x12050002, 0x1204AAB8,
	/* Pin Complex (NID 0x20 ) - 2 */
	0x1205000F, 0x1204B60B, 0x1205000F, 0x1204B60B,
	/* Pin Complex (NID 0x20 ) - 3 */
	0x12050007, 0x120400C0, 0x12050007, 0x120400C0,
	/* Pin Complex (NID 0x20 ) - 4 */
	0x12050008, 0x12040300, 0x12050008, 0x12040300,
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
				  &chip->pdev->dev,
				  PAGE_SIZE , &chip->rb);
	if (err < 0) {
		snd_printk(KERN_ERR T30SFX "cannot allocate CORB/RIRB\n");
		return err;
	}
	return 0;
}

static struct snd_pcm_ops azx_pcm_ops = {
	.open = azx_pcm_open,
	.close = azx_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = azx_pcm_hw_params,
	.hw_free = azx_pcm_hw_free,
	.prepare = azx_pcm_prepare,
	.trigger = azx_pcm_trigger,
	.pointer = azx_pcm_pointer,
	.page = snd_pcm_sgbuf_ops_page,
};


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


static int azx_attach_pcm_stream(struct hda_bus *bus, struct hda_codec *codec,
				 struct hda_pcm *cpcm);
 /*
 * Codec initialization
 */

static int __devinit azx_codec_create(struct azx *chip, const char *model)
{
	struct hda_bus_template bus_temp;
	int c, codecs, err;
	int max_slots;

	memset(&bus_temp, 0, sizeof(bus_temp));
	bus_temp.private_data = chip;
	bus_temp.modelname = model;
	bus_temp.pdev = chip->pdev;
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
		max_slots = AZX_DEFAULT_CODECS;

	/* First try to probe all given codec slots */
	for (c = 0; c < max_slots; c++) {
		if ((chip->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			if (probe_codec(chip, c) < 0) {
				/* Some BIOSen give you wrong codec addresses
				 * that don't exist
				 */
				snd_printk(KERN_WARNING T30SFX
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
		snd_printk(KERN_ERR T30SFX "no codecs initialized\n");
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
		snd_printk(KERN_ERR T30SFX "Invalid PCM device number %d\n",
			   pcm_dev);
		return -EINVAL;
	}
	if (chip->pcm[pcm_dev]) {
		snd_printk(KERN_ERR T30SFX "PCM %d already exists\n", pcm_dev);
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
					      &chip->pdev->dev,
					      1024 * 64, 32 * 1024 * 1024);
	return 0;
}



#ifdef CONFIG_PM
/*
 * power management
 */
static int nv_tegra_hda_controller_suspend(struct platform_device *pdev)
{
#if 0
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct azx *chip = card->private_data;
	clk_disable(chip->clk);
#endif
	return 0;
}

static int nv_tegra_hda_controller_resume(struct platform_device *pdev)
{
#if 0
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct azx *chip = card->private_data;
	clk_enable(chip->clk);
#endif
    return 0;
}

static int nv_tegra_azx_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
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

	return nv_tegra_hda_controller_suspend(pdev);
}

static int nv_tegra_azx_resume(struct platform_device *pdev)
{
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct azx *chip = card->private_data;
	int rc;

	rc = nv_tegra_hda_controller_resume(pdev);
	if (rc)
		return rc;

	chip->msi = 0;
	if (azx_acquire_irq(chip, 1) < 0)
		return -EIO;

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
	kfree(chip->azx_dev);
	kfree(chip);

	return 0;
}

static int azx_dev_free(struct snd_device *device)
{
	return azx_free(device->device_data);
}
static int __devinit check_position_fix(struct azx *chip, int fix)
{
	chip->via_dmapos_patch = 0;
	return POS_FIX_AUTO;
}

#define AZX_FORCE_CODEC_MASK	0x100

static void __devinit check_probe_mask(struct azx *chip, int dev)
{
	chip->codec_probe_mask = probe_mask[dev];

	/* check forced option */
	if (chip->codec_probe_mask != -1 &&
	    (chip->codec_probe_mask & AZX_FORCE_CODEC_MASK)) {
		chip->codec_mask = chip->codec_probe_mask & 0xff;
		printk(KERN_INFO "hda_intel: codec_mask forced to 0x%x\n",
		       chip->codec_mask);
	}
}

/*
 * constructor
 */
static int __devinit azx_create(struct snd_card *card,
				struct platform_device *pdev,
				int dev, struct azx **rchip)
{
	struct azx *chip;
	int i, err;
	unsigned short gcap;
	struct resource *res ;
	static struct snd_device_ops ops = {
		.dev_free = azx_dev_free,
	};

	*rchip = NULL;

	/* acquire resource.*/
	/* get mapped config*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -EINVAL;
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		snd_printk(KERN_ERR T30SFX "cannot allocate chip\n");
		return -ENOMEM;
	}

	spin_lock_init(&chip->reg_lock);
	mutex_init(&chip->open_mutex);
	chip->card = card;
	chip->pdev = pdev;
	chip->irq = -1;
	chip->msi = 0;
	chip->dev_index = dev;
	INIT_WORK(&chip->irq_pending_work, azx_irq_pending_work);

	chip->position_fix[0] = chip->position_fix[1] =
		check_position_fix(chip, position_fix[dev]);
	check_probe_mask(chip, dev);
	chip->single_cmd = single_cmd;

	if (bdl_pos_adj[dev] < 0)
		bdl_pos_adj[dev] = 32;
	chip->addr = res->start + TEGRA_HDA_BAR0_OFFSET ;
	chip->remap_addr = devm_ioremap(&pdev->dev, res->start,
				res->end - res->start + 1) + TEGRA_HDA_BAR0_OFFSET;;
	chip->pciconfig_addr = devm_ioremap(&pdev->dev, res->start,
				res->end - res->start + 1) + TEGRA_HDA_CONFIG_OFFSET ;

	if (chip->remap_addr == NULL) {
		snd_printk(KERN_ERR T30SFX "ioremap error\n");
		err = -ENXIO;
		goto errout;
	}

	if (azx_acquire_irq(chip, 0) < 0) {
		err = -EBUSY;
		goto errout;
	}

	synchronize_irq(chip->irq);

	gcap = azx_readw(chip, GCAP);
	snd_printdd("chipset global capabilities = 0x%x\n", gcap);

	/* read number of streams from GCAP register instead of using
	 * hardcoded value
	 */
	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	/* read number of streams from GCAP register instead of using
	 * hardcoded value
	 */
	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	if (!chip->playback_streams && !chip->capture_streams) {
		/* gcap didn't give any info, switching to old method */

		chip->playback_streams = ICH6_NUM_PLAYBACK;
		chip->capture_streams = ICH6_NUM_CAPTURE;
	}
	chip->capture_index_offset = 0;
	chip->playback_index_offset = chip->capture_streams;
	chip->num_streams = chip->playback_streams + chip->capture_streams;
	chip->azx_dev = kcalloc(chip->num_streams, sizeof(*chip->azx_dev),
				GFP_KERNEL);
	if (!chip->azx_dev) {
		snd_printk(KERN_ERR "cannot malloc azx_dev\n");
		goto errout;
	}

	for (i = 0; i < chip->num_streams; i++) {
		/* allocate memory for the BDL for each stream */
		err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
					  &chip->pdev->dev,
					  BDL_SIZE, &chip->azx_dev[i].bdl);
		if (err < 0) {
			snd_printk(KERN_ERR T30SFX "cannot allocate BDL\n");
			goto errout;
		}
	}
	/* allocate memory for the position buffer */
	err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  &chip->pdev->dev,
				  chip->num_streams * 8, &chip->posbuf);
	if (err < 0) {
		snd_printk(KERN_ERR T30SFX "cannot allocate posbuf\n");
		goto errout;
	}
	/* allocate CORB/RIRB */
	err = azx_alloc_cmd_io(chip);
	if (err < 0)
		goto errout;

	/* initialize streams */
	azx_init_stream(chip);

	/* initialize chip */
	azx_init_chip(chip, (probe_only[dev] & 2) == 0);

	/* codec detection */
	if (!chip->codec_mask) {
		snd_printk(KERN_ERR T30SFX "no codecs found!\n");
		err = -ENODEV;
		goto errout;
	}

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0) {
		snd_printk(KERN_ERR T30SFX "Error creating device [card]!\n");
		goto errout;
	}

	strcpy(card->driver, "HDA-Tegra");
	strcpy(card->shortname, "tegra-hda");
	sprintf(card->longname, "%s at 0x%lx irq %i",
	card->shortname, chip->addr, chip->irq);
	*rchip = chip;
	return 0;

 errout:
	azx_free(chip);
	return err;
}

static int nv_tegra_hda_controller_init(struct platform_device *pdev)
{
	void __iomem *mmio = NULL, *hda_config = NULL ,  *hda_reg = NULL ;
	struct resource *regs;
	u32 temp, intr_mask;
	u16 codec_mask;
	int err = 0, count, i;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!regs)
	{
		snd_printk(KERN_ERR T30SFX "no memory resource\n");
		return -ENOMEM;
	}

	/* HDA Reg */
	hda_reg = devm_ioremap(&pdev->dev, regs->start,
			regs->end - regs->start + 1);

	hda_config  =  hda_reg + TEGRA_HDA_CONFIG_OFFSET;
	mmio =  hda_reg + TEGRA_HDA_BAR0_OFFSET;
	clk_hda = clk_get_sys("hda", NULL);
	if (IS_ERR(clk_hda)) {
		snd_printk(KERN_ERR T30SFX "%s: can't get hda clock\n", __func__);
		return -1;
	}
	clk_hda2codec = clk_get_sys("hda2codec_2x", NULL);
	if (IS_ERR(clk_hda2codec)) {
		snd_printk(KERN_ERR T30SFX "%s: can't get hda clock\n", __func__);
		return -1;
	}
	clk_hda2hdmicodec = clk_get_sys("hda2hdmi", NULL);
	if (IS_ERR(clk_hda2codec)) {
		snd_printk(KERN_ERR T30SFX "%s: can't get hda clock\n", __func__);
		return -1;
	}
	clk_enable(clk_hda);
	clk_enable(clk_hda2codec);
	clk_enable(clk_hda2hdmicodec);
	/*Enable the PCI access */
	temp = readl(hda_reg + IPFS_HDA_CONFIGURATION_0);
	temp |= IPFS_EN_FPCI;
	writel(temp, (hda_reg + IPFS_HDA_CONFIGURATION_0));

	/* Program config space registers */
	/* Enable MEM/IO space and bus master */
	temp = readl(hda_config + NV_TEGRA_HDA_CFG_CMD_OFFSET);
	temp |= (NV_TEGRA_HDA_ENABLE_MEM_SPACE | NV_TEGRA_HDA_ENABLE_IO_SPACE |
		NV_TEGRA_HDA_ENABLE_BUS_MASTER | NV_TEGRA_HDA_ENABLE_SERR);
	temp &= (~NV_TEGRA_HDA_DISABLE_INTR);
	writel(temp, hda_config + NV_TEGRA_HDA_CFG_CMD_OFFSET);

	/* program bar0 space */
	/* write 1's to bar0 register */
	writel(NV_TEGRA_HDA_BAR0_INIT_PROGRAM,
		hda_config + NV_TEGRA_HDA_CFG_BAR0_OFFSET);
	/* flush */
	temp = readl(hda_config + NV_TEGRA_HDA_CFG_BAR0_OFFSET);
	writel(NV_TEGRA_HDA_BAR0_FINAL_PROGRAM,
		hda_config + NV_TEGRA_HDA_CFG_BAR0_OFFSET);
	/* flush */
	readl(hda_config + NV_TEGRA_HDA_CFG_BAR0_OFFSET);

	writel(FPCI_BAR0_START, ( hda_reg + IPFS_HDA_FPCI_BAR0) );
	intr_mask = readl((hda_reg+ IPFS_HDA_INTR_MASK));
	intr_mask &=   ~(1 << 16) ;
	intr_mask |=   (1 << 16) ;

	writel(intr_mask, (hda_reg+ IPFS_HDA_INTR_MASK));
	/* Turn on the link by de-asserting the Controller Reset# bit */
	writel(readl(mmio + ICH6_REG_GCTL) |
			ICH6_GCTL_RESET, mmio + ICH6_REG_GCTL);
	count = TIMEOUT_CODEC;
	while (!(readl(mmio + ICH6_REG_GCTL) & 0x0f) && --count)
		msleep(1);
	/* Clear STATESTS bits */
	writel(readl(mmio + ICH6_REG_WAKEEN) | (STATESTS_INT_MASK << 16),
			mmio + ICH6_REG_WAKEEN);
	/* Turn off the link by writing 0 to the Controller Reset# bit */
	writel(readl(mmio + ICH6_REG_GCTL) &
			~ICH6_GCTL_RESET, mmio + ICH6_REG_GCTL);
	count = TIMEOUT_CODEC;
	while ((readl(mmio + ICH6_REG_GCTL) & 0x0f) && --count)
		msleep(1);
	/* Turn on the Link again by writing 1 to the Controller Reset# bit */
	writel(readl(mmio + ICH6_REG_GCTL) |
			ICH6_GCTL_RESET, mmio + ICH6_REG_GCTL);
	count = TIMEOUT_CODEC;
	while (!(readl(mmio + ICH6_REG_GCTL) & 0x0f) && --count)
		msleep(1);
	/* Get the codec mask */
	codec_mask = (readl(mmio + ICH6_REG_WAKEEN) &
				(STATESTS_INT_MASK << 16)) >> 16;
	if (!codec_mask)
		snd_printk(KERN_ERR T30SFX "preinit: no codecs found!\n");
	/* Load Realtek ALC269 verbs table */
	for (i = 0; i < sizeof(alc269_verb_table); i++) {
		count = TIMEOUT_CODEC;
		while (count--) {
			/* check ICB busy bit */
			if (!((readl(mmio + ICH6_REG_IRS) & ICH6_IRS_BUSY))) {
				/* Clear IRV valid bit */
				writel(readl(mmio + ICH6_REG_IRS) |
					ICH6_IRS_VALID,	mmio + ICH6_REG_IRS);
				writel(alc269_verb_table[i],
					mmio + ICH6_REG_IC);
				writel(readl(mmio + ICH6_REG_IRS) |
					ICH6_IRS_BUSY, mmio + ICH6_REG_IRS);
				break;
			}
			udelay(1);
			if (!count) {
				snd_printk(KERN_ERR T30SFX "preinit: send verb \
						table timeout!\n");
				goto fail;
			}
		}
	}

	/* unmap the resources we mapped above */
	if (hda_reg)
		devm_iounmap(&pdev->dev, hda_reg);
	return 0;
fail:
	/* unmap the resources we mapped above */
	if (hda_reg)
		devm_iounmap(&pdev->dev, hda_reg);
	return err;
}

static int __devinit nv_tegra_azx_probe(struct platform_device *pdev)
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
	/* Call tegra init routine */
	err = nv_tegra_hda_controller_init(pdev);

	if (err != 0) {
		dev_printk(KERN_ERR, &pdev->dev, "NV TEGRA HDA init failed\n");
		return err;
	}

	err = snd_card_create(index[dev], id[dev], THIS_MODULE, 0, &card);
	if (err < 0) {
		snd_printk(KERN_ERR T30SFX "Error creating card!\n");
		return err;
	}

	/* set this here since it's referred in snd_hda_load_patch() */
	snd_card_set_dev(card, &pdev->dev);

	err = azx_create(card, pdev, dev, &chip);
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
		snd_printk(KERN_ERR T30SFX "Applying patch firmware '%s'\n",
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

	dev_set_drvdata(&pdev->dev, card);
	chip->running = 1;
	power_down_all_codecs(chip);
	azx_notifier_register(chip);

	dev++;
	return err;
out_free:
	snd_card_free(card);
	return err;
}

static int __devexit nv_tegra_azx_remove(struct platform_device *pdev)
{
	snd_card_free(dev_get_drvdata(&pdev->dev));
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

/* platform_driver definition */
static struct platform_driver tegra_platform_hda_driver = {
	.driver = {
		.name = "tegra-hda",
	},
	.probe = nv_tegra_azx_probe,
	.remove	= __devexit_p(nv_tegra_azx_remove),
#ifdef CONFIG_PM
	.suspend = nv_tegra_azx_suspend,
	.resume = nv_tegra_azx_resume,
#endif
};

static int __init alsa_card_azx_init(void)
{
	return platform_driver_register(&tegra_platform_hda_driver);
}

static void __exit alsa_card_azx_exit(void)
{
	platform_driver_unregister(&tegra_platform_hda_driver);
}

module_init(alsa_card_azx_init)
module_exit(alsa_card_azx_exit)
