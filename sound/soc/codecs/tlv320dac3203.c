/*
 * linux/sound/soc/codecs/tlv320dac3203.c
 *
 *
 * Copyright (C) 2012 Barnes & Noble.com 
 *
 * Based on sound/soc/codecs/tlv320dac3203.c 
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * 
 */

/***************************** INCLUDES ************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "tlv320dac3203.h"
#include <mach/gpio.h>
/*
 ***************************************************************************** 
 * Macros
 ***************************************************************************** 
 */

#ifdef CONFIG_MINI_DSP
extern int dac3203_minidsp_program(struct snd_soc_codec *codec);
extern void dac3203_add_minidsp_controls(struct snd_soc_codec *codec);
#endif

#define SOC_SINGLE_DAC3203(xname) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = __new_control_info, .get = __new_control_get,\
	.put = __new_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
}



#define SOC_DOUBLE_R_DAC3203(xname, reg_left, reg_right, shift, mask, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw_2r_dac3203, \
	.get = snd_soc_get_volsw_2r_dac3203, .put = snd_soc_put_volsw_2r_dac3203, \
	.private_value = (reg_left) | ((shift) << 8)  | \
		((mask) << 12) | ((invert) << 20) | ((reg_right) << 24) }
#define AUDIO_CODEC_POWER_ENABLE_GPIO   103
#define AUDIO_CODEC_RESET_GPIO          36
#define AUDIO_CODEC_IRQ_GPIO            114

/*
 ***************************************************************************** 
 * Function Prototype
 ***************************************************************************** 
 */
static int dac3203_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *tmp);

static int dac3203_mute(struct snd_soc_dai *dai, int mute);

static int dac3203_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir);

static int dac3203_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);

static int dac3203_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level);

static u8 dac3203_read(struct snd_soc_codec *codec, u16 reg);

static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);

static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int __new_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int snd_soc_info_volsw_2r_dac3203(struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_info *uinfo);

static int snd_soc_get_volsw_2r_dac3203(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

static int snd_soc_put_volsw_2r_dac3203(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

/*
 ***************************************************************************** 
 * Global Variable
 ***************************************************************************** 
 */
static u8 dac3203_reg_ctl;

/* whenever aplay/arecord is run, dac3203_hw_params() function gets called. 
 * This function reprograms the clock dividers etc. this flag can be used to 
 * disable this when the clock dividers are programmed by pps config file
 */
static int soc_static_freq_config = 1;

/*
 ***************************************************************************** 
 * Structure Declaration
 ***************************************************************************** 
 */
static struct snd_soc_device *dac3203_socdev;

/*
 ***************************************************************************** 
 * Structure Initialization
 ***************************************************************************** 
 */
static const struct snd_kcontrol_new dac3203_snd_controls[] = {
	/* Output */
	/* sound new kcontrol for PCM Playback volume control */
	SOC_DOUBLE_R_DAC3203("PCM Playback Volume", LDAC_VOL, RDAC_VOL, 0, 0xAf,
			     0),
	/* sound new kcontrol for HP driver gain */
	SOC_DOUBLE_R_DAC3203("HP Driver Gain", HPL_GAIN, HPR_GAIN, 0, 0x23, 0),
	/* sound new kcontrol for HP mute */
	SOC_DOUBLE_R("HP DAC Playback Switch", HPL_GAIN, HPR_GAIN, 6,
		     0x01, 1),
	/* sound new kcontrol for Programming the registers from user space */
	SOC_SINGLE_DAC3203("Program Registers"),

};

/* the sturcture contains the different values for mclk */
static const struct dac3203_rate_divs dac3203_divs[] = {
/* 
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc, blck_N, 
 * codec_speficic_initializations 
 */
	/* 8k rate */
	{12000000, 8000, 1, 7, 6800, 768, 5, 3, 128, 5, 18, 24,
	 {{60, 1}, {61, 1}}},
	{13000000, 8000, 1, 7, 912, 768, 5, 3, 128, 5, 18, 24,
	 {{60, 1}, {61, 1}}},
	{24000000, 8000, 2, 7, 6800, 768, 15, 1, 64, 45, 4, 24,
	 {{60, 1}, {61, 1}}},
	/* 11.025k rate */
	{12000000, 11025, 1, 7, 5264, 512, 8, 2, 128, 8, 8, 16,
	 {{60, 1}, {61, 1}}},
	{13000000, 11025, 1, 6, 9496, 512, 8, 2, 128, 8, 8, 16,
	 {{60, 1}, {61, 1}}},
	{24000000, 11025, 2, 7, 5264, 512, 16, 1, 64, 32, 4, 16,
	 {{60, 1}, {61, 1}}},
	/* 16k rate */
	{12000000, 16000, 1, 7, 6800, 384, 5, 3, 128, 5, 9, 12,
	 {{60, 1}, {61, 1}}},
	{13000000, 16000, 1, 7, 912, 384, 5, 3, 128, 5, 9, 12,
	 {{60, 1}, {61, 1}}},
	{24000000, 16000, 2, 7, 6800, 384, 15, 1, 64, 18, 5, 12,
	 {{60, 1}, {61, 1}}},
	/* 22.05k rate */
	{12000000, 22050, 1, 7, 5264, 256, 4, 4, 128, 4, 8, 8,
	 {{60, 1}, {61, 1}}},
	{13000000, 22050, 1, 6, 9496, 256, 4, 4, 128, 4, 8, 8,
	 {{60, 1}, {61, 1}}},
	{24000000, 22050, 2, 7, 5264, 256, 16, 1, 64, 16, 4, 8,
	 {{60, 1}, {61, 1}}},
	/* 32k rate */
	{12000000, 32000, 1, 7, 1680, 192, 2, 7, 64, 2, 21, 6,
	 {{60, 1}, {61, 1}}},
	{13000000, 32000, 1, 6, 6184, 192, 2, 7, 64, 2, 21, 6,
	 {{60, 1}, {61, 1}}},
	{24000000, 32000, 2, 7, 1680, 192, 7, 2, 64, 7, 6, 6,
	 {{60, 1}, {61, 1}}},
	/* 44.1k rate */
#ifdef  CONFIG_MINI_DSP
	{12000000, 44100, 1, 7, 5264, 128, 2, 8, 128, 2, 8, 4,
	 {}},
	{24000000, 44100, 2, 7, 5264, 128, 8, 2, 64, 8, 4, 4,
	 {}},
#else
	{12000000, 44100, 1, 7, 5264, 128, 2, 8, 128, 2, 8, 4,
	 {{60, 1}, {61, 1}}},
	{13000000, 44100, 1, 6, 9496, 128, 2, 8, 128, 2, 8, 4,
	 {{60, 1}, {61, 1}}},
	{24000000, 44100, 2, 7, 5264, 128, 8, 2, 64, 8, 4, 4,
	 {{60, 1}, {61, 1}}},
#endif
	/* 48k rate */
	{12000000, 48000, 1, 8, 1920, 128, 2, 8, 128, 2, 8, 4,
	 {{60, 1}, {61, 1}}},
	{13000000, 48000, 1, 7, 5640, 128, 2, 8, 28, 2, 8, 4,
	 {{60, 1}, {61, 1}}},
	{24000000, 48000, 2, 8, 1920, 128, 8, 2, 64, 8, 4, 4,
	 {{60, 1}, {61, 1}}},
	/*96k rate */
	{12000000, 96000, 1, 8, 1920, 64, 2, 8, 64, 2, 8, 2,
	 {{60, 7}, {61, 7}}},
	{13000000, 96000, 1, 7, 5640, 64, 2, 8, 64, 2, 8, 2,
	 {{60, 7}, {61, 7}}},
	{24000000, 96000, 2, 8, 1920, 64, 4, 4, 64, 8, 2, 2,
	 {{60, 7}, {61, 7}}},
	/*192k */
	{12000000, 192000, 1, 8, 1920, 32, 2, 8, 32, 2, 8, 1,
	 {{60, 17}, {61, 13}}},
	{13000000, 192000, 1, 7, 5640, 32, 2, 8, 32, 2, 8, 1,
	 {{60, 17}, {61, 13}}},
	{24000000, 192000, 2, 8, 1920, 32, 4, 4, 32, 4, 4, 1,
	 {{60, 17}, {61, 13}}},
};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *          It is SoC Codec DAI structure which has DAI capabilities viz., 
 *          playback and capture, DAI runtime information viz. state of DAI 
 *			and pop wait state, and DAI private data. 
 *          The DAC3203 rates ranges from 8k to 192k
 *          The PCM bit format supported are 16, 20, 24 and 32 bits
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai tlv320dac3203_dai = {
	.name = "dac3203",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = DAC3203_RATES,
		     .formats = DAC3203_FORMATS,},
	.ops = {
		.hw_params = dac3203_hw_params,
		.digital_mute = dac3203_mute,
		.set_sysclk = dac3203_set_dai_sysclk,
		.set_fmt = dac3203_set_dai_fmt,
		}
};

EXPORT_SYMBOL_GPL(tlv320dac3203_dai);

/*
 ***************************************************************************** 
 * Initializations
 ***************************************************************************** 
 */
/*
 * DAC3203 register cache
 * We are caching the registers here.
 * There is no point in caching the reset register.
 * NOTE: In AIC32, there are 127 registers supported in both page0 and page1
 *       The following table contains the page0 and page 1registers values.
 */
static const u8 dac3203_reg[DAC3203_CACHEREGNUM] = {
	0x00, 0x00, 0x50, 0x00,	/* 0 */
	0x00, 0x11, 0x04, 0x00,	/* 4 */
	0x00, 0x00, 0x00, 0x01,	/* 8 */
	0x01, 0x00, 0x80, 0x02,	/* 12 */
	0x00, 0x08, 0x01, 0x01,	/* 16 */
	0x80, 0x01, 0x00, 0x04,	/* 20 */
	0x00, 0x00, 0x01, 0x00,	/* 24 */
	0x00, 0x00, 0x01, 0x00,	/* 28 */
	0x00, 0x00, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x00, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x00,	/* 40 */
	0x00, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
	0x00, 0x42, 0x02, 0x02,	/* 52 */
	0x42, 0x02, 0x02, 0x02,	/* 56 */
	0x00, 0x00, 0x00, 0x01,	/* 60 */
	0x01, 0x00, 0x14, 0x00,	/* 64 */
	0x0C, 0x00, 0x00, 0x00,	/* 68 */
	0x00, 0x00, 0x00, 0xEE,	/* 72 */
	0x10, 0xD8, 0x10, 0xD8,	/* 76 */
	0x00, 0x00, 0x88, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
	0x7F, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x7F, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x00, 0x00,	/* 100 */
	0x00, 0x00, 0x00, 0x00,	/* 104 */
	0x00, 0x00, 0x00, 0x00,	/* 108 */
	0x00, 0x00, 0x00, 0x00,	/* 112 */
	0x00, 0x00, 0x00, 0x00,	/* 116 */
	0x00, 0x00, 0x00, 0x00,	/* 120 */
	0x00, 0x00, 0x00, 0x00,	/* 124 - PAGE0 Registers(127) ends here */
	0x01, 0x00, 0x08, 0x00,	/* 128, PAGE1-0 */
	0x00, 0x00, 0x00, 0x00,	/* 132, PAGE1-4 */
	0x00, 0x00, 0x00, 0x10,	/* 136, PAGE1-8 */
	0x00, 0x00, 0x00, 0x00,	/* 140, PAGE1-12 */
	0x40, 0x40, 0x40, 0x40,	/* 144, PAGE1-16 */
	0x00, 0x00, 0x00, 0x00,	/* 148, PAGE1-20 */
	0x00, 0x00, 0x00, 0x00,	/* 152, PAGE1-24 */
	0x00, 0x00, 0x00, 0x00,	/* 156, PAGE1-28 */
	0x00, 0x00, 0x00, 0x00,	/* 160, PAGE1-32 */
	0x00, 0x00, 0x00, 0x00,	/* 164, PAGE1-36 */
	0x00, 0x00, 0x00, 0x00,	/* 168, PAGE1-40 */
	0x00, 0x00, 0x00, 0x00,	/* 172, PAGE1-44 */
	0x00, 0x00, 0x00, 0x00,	/* 176, PAGE1-48 */
	0x00, 0x00, 0x00, 0x00,	/* 180, PAGE1-52 */
	0x00, 0x00, 0x00, 0x80,	/* 184, PAGE1-56 */
	0x80, 0x00, 0x00, 0x00,	/* 188, PAGE1-60 */
	0x00, 0x00, 0x00, 0x00,	/* 192, PAGE1-64 */
	0x00, 0x00, 0x00, 0x00,	/* 196, PAGE1-68 */
	0x00, 0x00, 0x00, 0x00,	/* 200, PAGE1-72 */
	0x00, 0x00, 0x00, 0x00,	/* 204, PAGE1-76 */
	0x00, 0x00, 0x00, 0x00,	/* 208, PAGE1-80 */
	0x00, 0x00, 0x00, 0x00,	/* 212, PAGE1-84 */
	0x00, 0x00, 0x00, 0x00,	/* 216, PAGE1-88 */
	0x00, 0x00, 0x00, 0x00,	/* 220, PAGE1-92 */
	0x00, 0x00, 0x00, 0x00,	/* 224, PAGE1-96 */
	0x00, 0x00, 0x00, 0x00,	/* 228, PAGE1-100 */
	0x00, 0x00, 0x00, 0x00,	/* 232, PAGE1-104 */
	0x00, 0x00, 0x00, 0x00,	/* 236, PAGE1-108 */
	0x00, 0x00, 0x00, 0x00,	/* 240, PAGE1-112 */
	0x00, 0x00, 0x00, 0x00,	/* 244, PAGE1-116 */
	0x00, 0x00, 0x00, 0x00,	/* 248, PAGE1-120 */
	0x00, 0x00, 0x00, 0x00	/* 252, PAGE1-124 */
};

/* 
 * dac3203 initialization data 
 * This structure initialization contains the initialization required for
 * DAC3203.
 * These registers values (reg_val) are written into the respective DAC3203 
 * register offset (reg_offset) to  initialize DAC3203. 
 * These values are used in dac3203_init() function only. 
 */
static const struct dac3203_configs dac3203_reg_init[] = {
	/* Carry out the software reset */
	{RESET, 0x01},
	/* Disable crude LDO */
	{POW_CFG, 0x08},
	/* Switch on the analog blocks */
	{LDO_CTL, 0x01},
	/* PLL is CODEC_CLKIN */
	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
	/* DAC_MOD_CLK is BCLK source */
	{AIS_REG_3, DAC_MOD_CLK_2_BDIV_CLKIN},
	/* Setting up DAC Channel */
	{DAC_CHN_REG,
	 LDAC_2_LCHN | RDAC_2_RCHN | SOFT_STEP_2WCLK},
	/* Headphone powerup
	   This will control Headphone startup.
	   Use this to reduce the pop-noise during start.
	   Now set to 4 times the constant value calcualted based on
	    6K Resistance, with minimal pop noise and start-up in ~1Sec*/
	{HPHONE_STARTUP_CTRL, 0x21},
	/* Setting REF Chanrging time to 40ms: As per DAC, application note*/
	{REF_PWR_CTRL, 0x01},
	/* Left Channel DAC recons filter's positive terminal is routed to HPL */
	{HPL_ROUTE_CTL, LDAC_CHNL_2_HPL},
	/* Right Channel DAC recons filter's positive terminal is routed to HPR */
	{HPR_ROUTE_CTL, RDAC_CHNL_2_HPR},
	/* HPL unmute and gain 0db */
	{HPL_GAIN, 0x0},
	/* HPR unmute and gain 0db */
	{HPR_GAIN, 0x0},
	/* Unmute DAC Left and Right channels */
	{DAC_MUTE_CTRL_REG, 0x00},
};

/* Left DAC_L Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", HPL_ROUTE_CTL, 3, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", HPR_ROUTE_CTL, 3, 1, 0),
};

static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", LOL_ROUTE_CTL, 3, 1, 0),
};

static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", LOR_ROUTE_CTL, 3, 1, 0),
};


static const struct snd_soc_dapm_widget dac3203_dapm_widgets[] = {
	/* Left DAC to Left Outputs */
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_CHN_REG, 7, 0),
	/* dapm widget (path domain) for left DAC_L Mixer */
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_output_mixer_controls[0],
			   ARRAY_SIZE(hpl_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPL Power", OUT_PWR_CTL, 5, 0, NULL, 0),

	/* Right DAC to Right Outputs */
	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_CHN_REG, 6, 0),
	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_output_mixer_controls[0],
			   ARRAY_SIZE(hpr_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPR Power", OUT_PWR_CTL, 4, 0, NULL, 0),

	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),
	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),
};

static const struct snd_soc_dapm_route dac3203_dapm_routes[] = {
	/* ******** Left Output ******** */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},

	{"HPL Power", NULL, "HPL Output Mixer"},
	{"HPL", NULL, "HPL Power"},

	/* ******** Right Output ******** */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},

	{"HPR Power", NULL, "HPR Output Mixer"},
	{"HPR", NULL, "HPR Power"},

};

#define DAC3203_DAPM_ROUTE_NUM (sizeof(dac3203_dapm_routes)/sizeof(struct snd_soc_dapm_route))

/*
 ***************************************************************************** 
 * Function Definitions
 ***************************************************************************** 
 */
static int snd_soc_info_volsw_2r_dac3203(struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_info *uinfo)
{
    int mask = (kcontrol->private_value >> 12) & 0xff;

    uinfo->type =
        mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = mask;
    return 0;
}
/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_get_volsw_2r_dac3203
 * Purpose  : Callback to get the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_get_volsw_2r_dac3203(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & DAC3203_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & DAC3203_8BITS_MASK;
	int mask;
	int shift;
	unsigned short val, val2;

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		mask = DAC3203_8BITS_MASK;
		shift = 0;
	} else if ( (!strcmp(kcontrol->id.name, "HP Driver Gain")) ) {
		mask = 0x3F;
		shift = 0;
	} else {
		printk("Invalid kcontrol name\n");
		return -1;
	}

	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		ucontrol->value.integer.value[0] =
		    (val <= 48) ? (val + 127) : (val - 129);
		ucontrol->value.integer.value[1] =
		    (val2 <= 48) ? (val2 + 127) : (val2 - 129);
	} else if ((!strcmp(kcontrol->id.name, "HP Driver Gain"))) {
		ucontrol->value.integer.value[0] =
		    (val <= 29) ? (val + 6) : (val - 58);
		ucontrol->value.integer.value[1] =
		    (val2 <= 29) ? (val2 + 6) : (val2 - 58);
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_put_volsw_2r_dac3203
 * Purpose  : Callback to set the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_put_volsw_2r_dac3203(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & DAC3203_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & DAC3203_8BITS_MASK;
	int err;
	unsigned short val, val2, val_mask;

	val = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = DAC3203_8BITS_MASK;	/* 8 bits */
	} else if ((!strcmp(kcontrol->id.name, "HP Driver Gain"))) {
		val = (val >= 6) ? (val - 6) : (val + 58);
		val2 = (val2 >= 6) ? (val2 - 6) : (val2 + 58);
		val_mask = 0x3F;	/* 6 bits */
	} else {
		printk("Invalid control name\n");
		return -1;
	}

	if ((err = snd_soc_update_bits(codec, reg, val_mask, val)) < 0) {
		printk("Error while updating bits\n");
		return err;
	}

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info
 * Purpose  : This function is to initialize data for new control required to 
 *            program the DAC3203 registers.
 *            
 *----------------------------------------------------------------------------
 */
static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get
 * Purpose  : This function is to read data of new control for 
 *            program the DAC3203 registers.
 *            
 *----------------------------------------------------------------------------
 */
static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	val = dac3203_read(codec, dac3203_reg_ctl);
	ucontrol->value.integer.value[0] = val;

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 * 
 *----------------------------------------------------------------------------
 */
static int __new_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dac3203_priv *dac3203 = codec->private_data;

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	dac3203_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0]) {
		dac3203->page_no = data[1];
	}

	printk("reg = %d val = %x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_change_page
 * Purpose  : This function is to switch between page 0 and page 1.
 *            
 *----------------------------------------------------------------------------
 */
int dac3203_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct dac3203_priv *dac3203 = codec->private_data;
	u8 data[2];

	data[0] = 0;
	data[1] = new_page;
	dac3203->page_no = new_page;

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in changing page to 1\n");
		return -1;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_write_reg_cache
 * Purpose  : This function is to write dac3203 register cache
 *            
 *----------------------------------------------------------------------------
 */
static inline void dac3203_write_reg_cache(struct snd_soc_codec *codec,
					   u16 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= DAC3203_CACHEREGNUM) {
		return;
	}
	cache[reg] = value;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_write
 * Purpose  : This function is to write to the dac3203 register space.
 *            
 *----------------------------------------------------------------------------
 */
int dac3203_write(struct snd_soc_codec *codec, u16 reg, u8 value)
{
	struct dac3203_priv *dac3203 = codec->private_data;
	u8 data[2];
	u8 page;

	page = reg / 128;
	data[DAC3203_REG_OFFSET_INDEX] = reg % 128;

	if (dac3203->page_no != page) {
		dac3203_change_page(codec, page);
	}

	/* data is
	 *   D15..D8 dac3203 register offset
	 *   D7...D0 register data
	 */
	data[DAC3203_REG_DATA_INDEX] = value & DAC3203_8BITS_MASK;
#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1)) {
		dac3203_write_reg_cache(codec, reg, value);
	}
#endif
	if (!data[DAC3203_REG_OFFSET_INDEX]) {
		/* if the write is to reg0 update dac3203->page_no */
		dac3203->page_no = value;
	}

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_read
 * Purpose  : This function is to read the dac3203 register space.
 *            
 *----------------------------------------------------------------------------
 */
static u8 dac3203_read(struct snd_soc_codec *codec, u16 reg)
{
	struct dac3203_priv *dac3203 = codec->private_data;
	u8 value;
	u8 page = reg / 128;

	reg = reg % 128;

	if (dac3203->page_no != page) {
		dac3203_change_page(codec, page);
	}

	i2c_master_send(codec->control_data, (char *)&reg, 1);
	i2c_master_recv(codec->control_data, &value, 1);
	return value;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_get_divs
 * Purpose  : This function is to get required divisor from the "dac3203_divs"
 *            table.
 *            
 *----------------------------------------------------------------------------
 */
static inline int dac3203_get_divs(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dac3203_divs); i++) {
		if ((dac3203_divs[i].rate == rate)
		    && (dac3203_divs[i].mclk == mclk)) {
			return i;
		}
	}
	printk("Master clock and sample rate is not supported\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_add_controls
 * Purpose  : This function is to add non dapm kcontrols.  The different 
 *            controls are in "dac3203_snd_controls" table.
 *            The following different controls are supported
 *                # PCM Playback volume control 
 *				  # PCM Playback Volume
 *				  # HP Driver Gain
 *				  # HP DAC Playback Switch
 *				  # PGA Capture Volume
 *				  # Program Registers
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(dac3203_snd_controls); i++) {
		err =
		    snd_ctl_add(codec->card,
				snd_soc_cnew(&dac3203_snd_controls[i], codec,
					     NULL));
		if (err < 0) {
			printk("Invalid control\n");
			return err;
		}
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_add_widgets
 * Purpose  : This function is to add the dapm widgets 
 *            The following are the main widgets supported
 *                # Left DAC to Left Outputs
 *                # Right DAC to Right Outputs
 *		  # Left Inputs to Left ADC
 *		  # Right Inputs to Right ADC
 *
 *----------------------------------------------------------------------------
 */
static int dac3203_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dac3203_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &dac3203_dapm_widgets[i]);
	}

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, &dac3203_dapm_routes[0],
				DAC3203_DAPM_ROUTE_NUM);

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_hw_params
 * Purpose  : This function is to set the hardware parameters for DAC3203.
 *            The functions set the sample rate and audio serial data word 
 *            length.
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, 
			     struct snd_soc_dai *tmp)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct dac3203_priv *dac3203 = codec->private_data;
	int i, j;
	u8 data;

	dac3203_set_bias_level(codec, SNDRV_CTL_POWER_D3hot);

	i = dac3203_get_divs(dac3203->sysclk, params_rate(params));

	if (i < 0) {
		printk("sampling rate not supported\n");
		return i;
	}

	if (soc_static_freq_config) {
		/* We will fix R value to 1 and will make P & J=K.D as varialble */

		/* Setting P & R values */
		dac3203_write(codec, CLK_REG_2,
			      ((dac3203_divs[i].p_val << 4) | 0x01));

		/* J value */
		dac3203_write(codec, CLK_REG_3, dac3203_divs[i].pll_j);

		/* MSB & LSB for D value */
		dac3203_write(codec, CLK_REG_4, (dac3203_divs[i].pll_d >> 8));
		dac3203_write(codec, CLK_REG_5,
			      (dac3203_divs[i].pll_d & DAC3203_8BITS_MASK));

		/* NDAC divider value */
		dac3203_write(codec, NDAC_CLK_REG_6, dac3203_divs[i].ndac);

		/* MDAC divider value */
		dac3203_write(codec, MDAC_CLK_REG_7, dac3203_divs[i].mdac);

		/* DOSR MSB & LSB values */
		dac3203_write(codec, DAC_OSR_MSB, dac3203_divs[i].dosr >> 8);
		dac3203_write(codec, DAC_OSR_LSB,
			      dac3203_divs[i].dosr & DAC3203_8BITS_MASK);

		/* NADC divider value */
		dac3203_write(codec, NADC_CLK_REG_8, dac3203_divs[i].nadc);

		/* MADC divider value */
		dac3203_write(codec, MADC_CLK_REG_9, dac3203_divs[i].madc);

		/* AOSR value */
		dac3203_write(codec, ADC_OSR_REG, dac3203_divs[i].aosr);
	}
	/* BCLK N divider */
	dac3203_write(codec, CLK_REG_11, dac3203_divs[i].blck_N);

	dac3203_set_bias_level(codec, SNDRV_CTL_POWER_D0);

	data = dac3203_read(codec, INTERFACE_SET_REG_1);

	data = data & ~(3 << 4);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (DAC3203_WORD_LEN_20BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (DAC3203_WORD_LEN_24BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (DAC3203_WORD_LEN_32BITS << DAC_OSR_MSB_SHIFT);
		break;
	}

	dac3203_write(codec, INTERFACE_SET_REG_1, data);


//	dac3203_write(codec, OUT_PWR_CTL, 0x30); //Power Up HPL and HPR


//	dac3203_write(codec, DAC_CHN_REG, 0xD6); //Power Up HPL and HPR
//	dac3203_write(codec, DAC_MUTE_CTRL_REG, 0x00);
#if 0
	for (j = 0; j < NO_FEATURE_REGS; j++) {
		dac3203_write(codec,
			      dac3203_divs[i].codec_specific_regs[j].reg_offset,
			      dac3203_divs[i].codec_specific_regs[j].reg_val);
	}
#endif

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_mute
 * Purpose  : This function is to mute or unmute the left and right DAC
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 dac_reg;

	dac_reg = dac3203_read(codec, DAC_MUTE_CTRL_REG) & ~MUTE_ON;
	if (mute)
		dac3203_write(codec, DAC_MUTE_CTRL_REG, dac_reg | MUTE_ON);
	else
		dac3203_write(codec, DAC_MUTE_CTRL_REG, dac_reg);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_set_dai_sysclk
 * Purpose  : This function is to set the DAI system clock
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct dac3203_priv *dac3203 = codec->private_data;

	switch (freq) {
	case DAC3203_FREQ_12000000:
	case DAC3203_FREQ_13000000:
	case DAC3203_FREQ_24000000:
		dac3203->sysclk = freq;
		return 0;
	}
	printk("Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_set_dai_fmt
 * Purpose  : This function is to set the DAI format
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct dac3203_priv *dac3203 = codec->private_data;
	u8 iface_reg;

	iface_reg = dac3203_read(codec, INTERFACE_SET_REG_1);
	iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		dac3203->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		dac3203->master = 0;
		break;
	default:
		printk("Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (DAC3203_DSP_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (DAC3203_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (DAC3203_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	default:
		printk("Invalid DAI interface format\n");
		return -EINVAL;
	}

	dac3203_write(codec, INTERFACE_SET_REG_1, iface_reg);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_set_bias_level
 * Purpose  : This function is to get triggered when dapm events occurs.
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct dac3203_priv *dac3203 = codec->private_data;
	u8 value;

	switch (level) {
		/* full On */
	case SND_SOC_BIAS_ON:
		printk("Setting BIAS to SND_SOC_BIAS_ON:\n");
		/* all power is driven by DAPM system */
		if (dac3203->master) {
			/* Switch on PLL */
			value = dac3203_read(codec, CLK_REG_2);
			dac3203_write(codec, CLK_REG_2, (value | ENABLE_PLL));

			/* Switch on NDAC Divider */
			value = dac3203_read(codec, NDAC_CLK_REG_6);
			dac3203_write(codec, NDAC_CLK_REG_6,
				      value | ENABLE_NDAC);

			/* Switch on MDAC Divider */
			value = dac3203_read(codec, MDAC_CLK_REG_7);
			dac3203_write(codec, MDAC_CLK_REG_7,
				      value | ENABLE_MDAC);

			/* Switch on NADC Divider */
			value = dac3203_read(codec, NADC_CLK_REG_8);
			dac3203_write(codec, NADC_CLK_REG_8,
				      value | ENABLE_MDAC);

			/* Switch on MADC Divider */
			value = dac3203_read(codec, MADC_CLK_REG_9);
			dac3203_write(codec, MADC_CLK_REG_9,
				      value | ENABLE_MDAC);

			/* Switch on BCLK_N Divider */
			value = dac3203_read(codec, CLK_REG_11);
			dac3203_write(codec, CLK_REG_11, value | ENABLE_BCLK);
		}
		break;

		/* partial On */
	case SND_SOC_BIAS_PREPARE:
		break;

		/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		printk("Setting BIAS to SND_SOC_BIAS_STANDBY:\n");
		/*
		 * all power is driven by DAPM system,
		 * so output power is safe if bypass was set
		 */
		if (dac3203->master) {
			/* Switch off PLL */
			value = dac3203_read(codec, CLK_REG_2);
			dac3203_write(codec, CLK_REG_2, (value & ~ENABLE_PLL));

			/* Switch off NDAC Divider */
			value = dac3203_read(codec, NDAC_CLK_REG_6);
			dac3203_write(codec, NDAC_CLK_REG_6,
				      value & ~ENABLE_NDAC);

			/* Switch off MDAC Divider */
			value = dac3203_read(codec, MDAC_CLK_REG_7);
			dac3203_write(codec, MDAC_CLK_REG_7,
				      value & ~ENABLE_MDAC);

			/* Switch off NADC Divider */
			value = dac3203_read(codec, NADC_CLK_REG_8);
			dac3203_write(codec, NADC_CLK_REG_8,
				      value & ~ENABLE_NDAC);

			/* Switch off MADC Divider */
			value = dac3203_read(codec, MADC_CLK_REG_9);
			dac3203_write(codec, MADC_CLK_REG_9,
				      value & ~ENABLE_MDAC);
			value = dac3203_read(codec, CLK_REG_11);

			/* Switch off BCLK_N Divider */
			dac3203_write(codec, CLK_REG_11, value & ~ENABLE_BCLK);
		}
		break;

		/* Off, without power */
	case SND_SOC_BIAS_OFF:
		printk("Setting BIAS to SND_SOC_BIAS_OFF:\n");
		/* force all power off */
		break;
	}
	codec->bias_level = level;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_suspend
 * Purpose  : This function is to suspend the DAC3203 driver.
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	dac3203_set_bias_level(codec, SNDRV_CTL_POWER_D3cold);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_resume
 * Purpose  : This function is to resume the DAC3203 driver
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u8 *cache = codec->reg_cache;

	dac3203_change_page(codec, 0);
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(dac3203_reg); i++) {
		data[0] = i % 128;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, 2);
	}
	dac3203_change_page(codec, 0);
	dac3203_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_init
 * Purpose  : This function is to initialise the DAC3203 driver
 *            register the mixer and dsp interfaces with the kernel.
 *            
 *----------------------------------------------------------------------------
 */
static int tlv320dac3203_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	struct dac3203_priv *dac3203 = codec->private_data;
	int ret = 0;
	int i = 0;

	codec->name = "dac3203";
	codec->owner = THIS_MODULE;
	codec->read = dac3203_read;
	codec->write = dac3203_write;
	codec->set_bias_level = dac3203_set_bias_level;
	codec->dai = &tlv320dac3203_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(dac3203_reg);
	codec->reg_cache =
	    kmemdup(dac3203_reg, sizeof(dac3203_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		printk(KERN_ERR "dac3203: kmemdup failed\n");
		return -ENOMEM;
	}

	dac3203->page_no = 0;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "dac3203: failed to create pcms\n");
		goto pcm_err;
	}

	for (i = 0;
	     i < sizeof(dac3203_reg_init) / sizeof(struct dac3203_configs);
	     i++) {
		dac3203_write(codec, dac3203_reg_init[i].reg_offset,
			      dac3203_reg_init[i].reg_val);
	}

	/* off, with power on */
	dac3203_set_bias_level(codec, SNDRV_CTL_POWER_D3hot);
	dac3203_add_controls(codec);
	dac3203_add_widgets(codec);

#ifdef CONFIG_MINI_DSP
	dac3203_add_minidsp_controls(codec);
#endif
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "dac3203: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 *----------------------------------------------------------------------------
 * Function : dac3203_codec_probe
 * Purpose  : This function attaches the i2c client and initializes 
 *				DAC3203 CODEC.
 *            NOTE:
 *            This function is called from i2c core when the I2C address is
 *            valid.
 *            If the i2c layer weren't so broken, we could pass this kind of 
 *            data around
 *            
 *----------------------------------------------------------------------------
 */
static int tlv320dac3203_codec_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = dac3203_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = tlv320dac3203_init(socdev);
	if (ret < 0) {
		printk(KERN_ERR "dac3203: failed to attach codec at addr\n");
		return -1;
	}
#ifdef CONFIG_MINI_DSP
	/* Program MINI DSP for ADC and DAC */
	dac3203_minidsp_program(codec);
	dac3203_change_page(codec, 0x0);
#endif

	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : tlv320dac3203_i2c_remove
 * Purpose  : This function removes the i2c client and uninitializes 
 *                              DAC3203 CODEC.
 *            NOTE:
 *            This function is called from i2c core 
 *            If the i2c layer weren't so broken, we could pass this kind of 
 *            data around
 *            
 *----------------------------------------------------------------------------
 */

static int __exit tlv320dac3203_i2c_remove(struct i2c_client *i2c)
{
        put_device(&i2c->dev);
        return 0;
}

static const struct i2c_device_id tlv320dac3203_id[] = {
        {"tlv320dac3203", 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, tlv320dac3203_id);

static struct i2c_driver tlv320dac3203_i2c_driver = {
	.driver = {
		.name = "tlv320dac3203",
	},
	.probe = tlv320dac3203_codec_probe,
	.remove = __exit_p(tlv320dac3203_i2c_remove),
	.id_table = tlv320dac3203_id,
};

#endif //#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_probe
 * Purpose  : This is first driver function called by the SoC core driver.
 *            
 *----------------------------------------------------------------------------
 */
static void audio_dac_3203_dev_init(void)
{
        printk("board-3621_gossamera.c: audio_dac_3203_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_RESET_GPIO, "AUDIO_CODEC_RESET_GPIO") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_RESET_GPIO \n");
                return;
        }

        printk("board-3621_gossamera.c: audio_dac_3203_dev_init > set AUDIO_CODEC_RESET_GPIO to output Low!\n");
        gpio_direction_output(AUDIO_CODEC_RESET_GPIO, 0);
        gpio_set_value(AUDIO_CODEC_RESET_GPIO, 0);

        printk("board-3621_gossamera.c: audio_dac_3203_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_POWER_ENABLE_GPIO, "AUDIO DAC3203 POWER ENABLE") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_POWER_ENABLE_GPIO \n");
                return;
        }

        printk("board-3621_gossamera.c: audio_dac_3203_dev_init > set AUDIO_CODEC_POWER_ENABLE_GPIO to output and value high!\n");
        gpio_direction_output(AUDIO_CODEC_POWER_ENABLE_GPIO, 0);
        gpio_set_value(AUDIO_CODEC_POWER_ENABLE_GPIO, 1);

        /* 1 msec delay needed after PLL power-up */
        mdelay (10);

        printk("board-3621_gossamera.c: audio_dac_3203_dev_init > set AUDIO_CODEC_RESET_GPIO to output and value high!\n");
        gpio_set_value(AUDIO_CODEC_RESET_GPIO, 1);

}

static int dac3203_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct dac3203_priv *dac3203;
	int ret = 0;

	printk(KERN_INFO "DAC3203 Audio Codec %s\n", DAC3203_VERSION);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if(codec == NULL)
		return -ENOMEM;

	dac3203 = kzalloc(sizeof(struct dac3203_priv), GFP_KERNEL);
	if (dac3203 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = dac3203;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	audio_dac_3203_dev_init();
	dac3203_socdev = socdev;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
		codec->hw_write = (hw_write_t) i2c_master_send;
		codec->hw_read = (hw_read_t) i2c_master_recv;
		ret = i2c_add_driver(&tlv320dac3203_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
#else
	/* Add other interfaces here */
#endif
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3203_remove
 * Purpose  : to remove dac3203 soc device 
 *            
 *----------------------------------------------------------------------------
 */
static int dac3203_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	/* power down chip */
	if (codec->control_data)
		dac3203_set_bias_level(codec, SNDRV_CTL_POWER_D3);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&tlv320dac3203_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_device |
 *          This structure is soc audio codec device sturecute which pointer
 *          to basic functions dac3203_probe(), dac3203_remove(),  
 *          dac3203_suspend() and dac3203_resume()
 *
 */
struct snd_soc_codec_device soc_codec_dev_dac3203 = {
	.probe = dac3203_probe,
	.remove = dac3203_remove,
	.suspend = dac3203_suspend,
	.resume = dac3203_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_dac3203);

static int __init tlv320dac3203_modinit(void)
{
	return snd_soc_register_dai(&tlv320dac3203_dai);
}

module_init(tlv320dac3203_modinit);

static void __exit tlv320dac3203_exit(void)
{
	snd_soc_unregister_dai(&tlv320dac3203_dai);
}

module_exit(tlv320dac3203_exit);

MODULE_DESCRIPTION("ASoC TLV320DAC3203 codec driver");
MODULE_AUTHOR("fpaul@book.com");
MODULE_LICENSE("GPL");
