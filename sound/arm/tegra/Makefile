#EXTRA_CFLAGS += -DSND_HDA_TEGRA

snd-hda-tegra-objs := ../../pci/hda/lib_hda_intel.o hda_tegra.o

snd-hda-codec-y := ../../pci/hda/hda_codec.o
snd-hda-codec-$(CONFIG_SND_HDA_GENERIC) += ../../pci/hda/hda_generic.o
snd-hda-codec-$(CONFIG_SND_HDA_HWDEP) += ../../pci/hda/hda_hwdep.o
snd-hda-codec-$(CONFIG_SND_HDA_INPUT_BEEP) += ../../pci/hda/hda_beep.o
snd-hda-codec-$(CONFIG_PROC_FS) += ../../pci/hda/hda_proc.o

snd-hda-codec-nvhdmi-objs := 	patch_nvhdmi_tegra.o
snd-hda-codec-realtek-objs := 	../../pci/hda/patch_realtek.o


# common driver
obj-$(CONFIG_SND_HDA_TEGRA) := snd-hda-codec.o

# codec drivers (note: CONFIG_SND_HDA_CODEC_XXX are booleans)
ifdef CONFIG_SND_HDA_CODEC_NVHDMI
#obj-$(CONFIG_SND_HDA_TEGRA) += snd-hda-codec-nvhdmi.o
endif
ifdef CONFIG_SND_HDA_CODEC_REALTEK
obj-$(CONFIG_SND_HDA_TEGRA) += snd-hda-codec-realtek.o
endif

# this must be the last entry after codec drivers;
# otherwise the codec patches won't be hooked before the PCI probe
# when built in kernel
obj-$(CONFIG_SND_HDA_TEGRA) += snd-hda-tegra.o
