/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>

#include "adau1x61.h"

LOG_MODULE_REGISTER(adau1x61, CONFIG_ADAU1X61_LOG_LEVEL);

struct adau1x61_driver_config {
	struct i2c_dt_spec bus;
	const char *adau_model;
};

struct adau1x61_driver_data {
	bool dsp_model;
};

#define DEV_CFG(dev)  ((struct adau1x61_driver_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct adau1x61_driver_data *const)(dev)->data)

static int adau1x61_write_reg(const struct device *dev, uint16_t reg_addr, uint8_t reg_val);
static int adau1x61_read_reg(const struct device *dev, uint16_t reg_addr, uint8_t *reg_val);
static int adau1x61_update_reg(const struct device *dev, uint16_t reg_addr, uint8_t mask,
			       uint8_t reg_val);
static int adau1x61_configure_dai(const struct device *dev, audio_dai_cfg_t *cfg);
static int adau1x61_configure_clocks(const struct device *dev, struct audio_codec_cfg *cfg);
static int adau1x61_set_output_volume(const struct device *dev, audio_channel_t channel, int vol);
static int adau1x61_set_output_mute(const struct device *dev, audio_channel_t channel, bool mute);
static int adau1x61_set_input_volume(const struct device *dev, audio_channel_t channel, int vol);
static int adau1x61_set_input_mute(const struct device *dev, audio_channel_t channel, bool mute);
static void adau1x61_configure_output(const struct device *dev);
static void adau1x61_configure_input(const struct device *dev);

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void adau1x61_read_all_regs(const struct device *dev);
#define ADAU1X61_DUMP_REGS(dev) adau1x61_read_all_regs((dev))
#else
#define ADAU1X61_DUMP_REGS(dev)
#endif

static int adau1x61_initialize(const struct device *dev)
{
	struct adau1x61_driver_config *const dev_cfg = DEV_CFG(dev);
	struct adau1x61_driver_data *const dev_data = DEV_DATA(dev);

	if (strcmp(dev_cfg->adau_model, "ADAU1761") == 0) {
		dev_data->dsp_model = true;
	} else {
		dev_data->dsp_model = false;
	}
	LOG_DBG("Connected ADAU model: %s", dev_cfg->adau_model);

	return 0;
}

static int adau1x61_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	int ret;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("dai_type must be AUDIO_DAI_TYPE_I2S");
		return -EINVAL;
	}

	ret = adau1x61_configure_clocks(dev, cfg);
	if (ret) {
		return -EIO;
	}

	ret = adau1x61_configure_dai(dev, &cfg->dai_cfg);
	if (ret) {
		return -EIO;
	}

	adau1x61_configure_output(dev);
	adau1x61_configure_input(dev);

	return 0;
}

static int adau1x61_set_property(const struct device *dev, audio_property_t property,
				 audio_channel_t channel, audio_property_value_t val)
{
	int ret;

	LOG_DBG("Apply property");

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		ret = adau1x61_set_output_volume(dev, channel, val.vol);
		break;
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		ret = adau1x61_set_output_mute(dev, channel, val.mute);
		break;
	case AUDIO_PROPERTY_INPUT_VOLUME:
		ret = adau1x61_set_input_volume(dev, channel, val.vol);
		break;
	case AUDIO_PROPERTY_INPUT_MUTE:
		ret = adau1x61_set_input_mute(dev, channel, val.mute);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int adau1x61_apply_properties(const struct device *dev)
{
	/* nothing to do because there is nothing cached */
	return 0;
}

static int adau1x61_write_reg(const struct device *dev, uint16_t reg_addr, uint8_t reg_val)
{
	struct adau1x61_driver_config *const dev_cfg = DEV_CFG(dev);
	uint8_t tx_buf[3] = {reg_addr >> 8, reg_addr, reg_val};

	LOG_DBG("WR REG:0x%04x VAL:0x%02x", reg_addr, reg_val);

	return i2c_write_dt(&dev_cfg->bus, tx_buf, 3);
}

static int adau1x61_read_reg(const struct device *dev, uint16_t reg_addr, uint8_t *reg_val)
{
	struct adau1x61_driver_config *const dev_cfg = DEV_CFG(dev);
	uint8_t tx_buf[2] = {reg_addr >> 8, reg_addr};
	int ret;

	ret = i2c_write_read_dt(&dev_cfg->bus, &tx_buf, sizeof(tx_buf), reg_val, sizeof(*reg_val));

	LOG_DBG("RD REG:0x%04x VAL:0x%02x", reg_addr, *reg_val);

	return ret;
}

static int adau1x61_update_reg(const struct device *dev, uint16_t reg_addr, uint8_t mask,
			       uint8_t reg_val)
{
	uint8_t old_val, new_val;
	int ret;

	ret = adau1x61_read_reg(dev, reg_addr, &old_val);
	if (ret) {
		return ret;
	}

	new_val = (old_val & ~mask) | (reg_val & mask);
	if (new_val == old_val) {
		return 0;
	}

	return adau1x61_write_reg(dev, reg_addr, new_val);
}

static int adau1x61_configure_dai(const struct device *dev, audio_dai_cfg_t *cfg)
{
	struct adau1x61_driver_config *const dev_cfg = DEV_CFG(dev);
	struct adau1x61_driver_data *const dev_data = DEV_DATA(dev);
	struct i2s_config *i2s_cfg = &cfg->i2s;
	uint8_t reg0 = 0, reg1 = 0;

	/* configure DAI format */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		reg1 |= ADAU1X61_SERIAL_PORT1_DELAY1;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		reg0 |= ADAU1X61_SERIAL_PORT0_PULSE_MODE | ADAU1X61_SERIAL_PORT0_LRCLK_POL;
		reg1 |= ADAU1X61_SERIAL_PORT1_DELAY1;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		reg0 |= ADAU1X61_SERIAL_PORT0_LRCLK_POL;
		reg1 |= ADAU1X61_SERIAL_PORT1_DELAY0;
		break;
	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	/* configure word size */
	switch (i2s_cfg->word_size) {
	case AUDIO_PCM_WIDTH_16_BITS:
		if (strcmp(dev_cfg->adau_model, "ADAU1761") == 0) {
			goto fail;
		}
		reg1 |= ADAU1X61_SERIAL_PORT1_BCLK32;
		break;
	case AUDIO_PCM_WIDTH_24_BITS:
		reg1 |= ADAU1X61_SERIAL_PORT1_BCLK48;
		break;
	case AUDIO_PCM_WIDTH_32_BITS:
		reg1 |= ADAU1X61_SERIAL_PORT1_BCLK64;
		break;
	fail:
	default:
		LOG_ERR("Unsupported PCM sample bit width %u", cfg->i2s.word_size);
		return -EINVAL;
	}

	/* configure CLK format */
	switch (i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) {
	case I2S_FMT_CLK_NF_NB:
		break;
	case I2S_FMT_CLK_NF_IB:
		reg0 |= ADAU1X61_SERIAL_PORT0_BCLK_POL;
		break;
	case I2S_FMT_CLK_IF_NB:
		reg0 ^= ADAU1X61_SERIAL_PORT0_LRCLK_POL;
		break;
	case I2S_FMT_CLK_IF_IB:
		reg0 |= ADAU1X61_SERIAL_PORT0_BCLK_POL;
		reg0 ^= ADAU1X61_SERIAL_PORT0_LRCLK_POL;
		break;
	default:
		LOG_ERR("Unsupported I2S clock format");
		return -EINVAL;
	}

	/* configure CLK slave/master */
	if ((i2s_cfg->options & I2S_OPT_BIT_CLK_MASTER) &&
	    (i2s_cfg->options & I2S_OPT_FRAME_CLK_MASTER)) {
		reg0 |= ADAU1X61_SERIAL_PORT0_MASTER;
	} else {
		reg0 &= ~(ADAU1X61_SERIAL_PORT0_MASTER);
	}

	adau1x61_write_reg(dev, ADAU1X61_SERIAL_PORT0, reg0);
	adau1x61_write_reg(dev, ADAU1X61_SERIAL_PORT1, reg1);

	if (dev_data->dsp_model) {
		adau1x61_write_reg(dev, ADAU1761_SERIAL_INPUT_ROUTE,
				   ADAU1761_SERIAL_INPUT_ROUTE_SI_TO_DAC);
		adau1x61_write_reg(dev, ADAU1761_SERIAL_OUTPUT_ROUTE,
				   ADAU1761_SERIAL_OUTPUT_ROUTE_ADC_TO_SI);
		adau1x61_write_reg(dev, ADAU1761_CLK_ENABLE0,
				   ADAU1761_CLK_ENABLE0_SPPD | ADAU1761_CLK_ENABLE0_SINPD |
					   ADAU1761_CLK_ENABLE0_INTPD |
					   ADAU1761_CLK_ENABLE0_SOUTPD |
					   ADAU1761_CLK_ENABLE0_DECPD | ADAU1761_CLK_ENABLE0_ALCPD |
					   ADAU1761_CLK_ENABLE0_SLEWPD);
		adau1x61_write_reg(dev, ADAU1761_CLK_ENABLE1,
				   ADAU1761_CLK_ENABLE1_CLK0 | ADAU1761_CLK_ENABLE1_CLK1);
	}
	return 0;
}

static int adau1x61_configure_clocks(const struct device *dev, struct audio_codec_cfg *cfg)
{
	struct i2s_config *i2s_cfg;
	uint8_t infreq;

	i2s_cfg = &cfg->dai_cfg.i2s;
	LOG_DBG("MCLK Rate: %u Hz - PCM Rate: %u Hz", cfg->mclk_freq, i2s_cfg->frame_clk_freq);

	if (cfg->mclk_freq % i2s_cfg->frame_clk_freq != 0) {
		LOG_ERR("Wrong MCLK or PCM rate");
		return -EINVAL;
	}

	switch (cfg->mclk_freq / i2s_cfg->frame_clk_freq) {
	case 256:
		infreq = 0;
		break;
	case 512:
		infreq = 1;
		break;
	case 768:
		infreq = 2;
		break;
	case 1024:
		infreq = 3;
		break;
	default:
		return -EINVAL;
	}

	adau1x61_write_reg(dev, ADAU1X61_CLOCK_CONTROL,
			   ADAU1X61_CLOCK_CONTROL_COREN | (infreq << 1));
	return 0;
}

static void adau1x61_configure_output(const struct device *dev)
{
	/* Playback power management - Enable PREN & PLEN */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_POWER_MGMT,
			   ADAU1X61_PLAY_POWER_MGMT_PREN | ADAU1X61_PLAY_POWER_MGMT_PLEN);

	/* Configure LHP & RHP outputs in Headphone mode */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_HP_LEFT_VOL, ADAU1X61_PLAY_HP_LEFT_VOL_HPEN);
	adau1x61_write_reg(dev, ADAU1X61_PLAY_HP_RIGHT_VOL, ADAU1X61_PLAY_HP_RIGHT_VOL_HPMODE);
	adau1x61_write_reg(dev, ADAU1X61_PLAY_MONO_OUTPUT_VOL,
			   ADAU1X61_PLAY_MONO_OUTPUT_VOL_MODE_HP |
				   ADAU1X61_PLAY_MONO_OUTPUT_VOL_UNMUTE);

	/* Configure LOUT & ROUT in Line out mode */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_LINE_LEFT_VOL, 0);
	adau1x61_write_reg(dev, ADAU1X61_PLAY_LINE_RIGHT_VOL, 0);
}

static void adau1x61_configure_input(const struct device *dev)
{
	/* Microphone input (Stereo Differential Microphone Configuration) */
	adau1x61_write_reg(dev, ADAU1X61_LEFT_DIFF_INPUT_VOL, ADAU1X61_DIFF_INPUT_VOL_EN);
	adau1x61_write_reg(dev, ADAU1X61_RIGHT_DIFF_INPUT_VOL, ADAU1X61_DIFF_INPUT_VOL_EN);
	adau1x61_write_reg(dev, ADAU1X61_REC_MIXER_LEFT1,
			   ADAU1X61_REC_MIXER_RDBOOST(INPUT_PGA_BOOST_0DB));
	adau1x61_write_reg(dev, ADAU1X61_REC_MIXER_RIGHT1,
			   ADAU1X61_REC_MIXER_RDBOOST(INPUT_PGA_BOOST_0DB));
	adau1x61_write_reg(dev, ADAU1X61_MICBIAS,
			   ADAU1X61_MICBIAS_MBIEN | ADAU17X1_MICBIAS_0_65_AVDD);

	/* Aux input (Stereo Single-Ended) */
	adau1x61_update_reg(dev, ADAU1X61_REC_MIXER_LEFT1, 0x7,
			    ADAU1X61_REC_MIXER_AUX_GAIN(INPUT_GAIN_0DB));
	adau1x61_update_reg(dev, ADAU1X61_REC_MIXER_RIGHT1, 0x7,
			    ADAU1X61_REC_MIXER_AUX_GAIN(INPUT_GAIN_0DB));
}

static void adau1x61_start_output(const struct device *dev)
{
	/* powerup DAC channels */
	adau1x61_update_reg(dev, ADAU1X61_DAC_CONTROL0, ADAU1X61_DAC_CONTROL0_DACEN_MASK,
			    ADAU1X61_DAC_CONTROL0_DACEN_LEFT_DAC |
				    ADAU1X61_DAC_CONTROL0_DACEN_RIGHT_DAC);

	/* unmute left DAC mixer MX3 */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_MIXER_LEFT0,
			   ADAU1X61_PLAY_MIXER_LEFT0_MIXER_EN |
				   ADAU1X61_PLAY_MIXER_LEFT0_LEFT_MUTE);
	/* unmute right DAC mixer MX4 */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_MIXER_RIGHT0,
			   ADAU1X61_PLAY_MIXER_RIGHT0_MIXER_EN |
				   ADAU1X61_PLAY_MIXER_RIGHT0_RIGHT_MUTE);
	/* unmute left Line out mixer MX5 */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_LR_MIXER_LEFT,
			   ADAU1X61_PLAY_LR_MIXER_LEFT_MIXER_EN |
				   ADAU1X61_PLAY_LR_MIXER_LEFT_MX5G3_6DB);
	/* unmute right line out mixer MX6 */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_LR_MIXER_RIGHT,
			   ADAU1X61_PLAY_LR_MIXER_RIGHT_MIXER_EN |
				   ADAU1X61_PLAY_LR_MIXER_RIGHT_MX6G4_6DB);
}

static void adau1x61_stop_output(const struct device *dev)
{
	/* mute DAC channels */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_MIXER_LEFT0, 0);
	adau1x61_write_reg(dev, ADAU1X61_PLAY_MIXER_RIGHT0, 0);

	/* powerdown DAC channels */
	adau1x61_write_reg(dev, ADAU1X61_DAC_CONTROL0, 0);
}

static void adau1x61_start_input(const struct device *dev)
{
	/* powerup ADC channels */
	adau1x61_update_reg(dev, ADAU1X61_ADC_CONTROL, ADAU1X61_ADC_CONTROL_ADCEN_MASK,
			    ADAU1X61_ADC_CONTROL_ADCEN_LEFT_ADC |
				    ADAU1X61_ADC_CONTROL_ADCEN_RIGHT_ADC);

	/* unmute left ADC mixer MX1 */
	adau1x61_update_reg(dev, ADAU1X61_REC_MIXER_LEFT0, ADAU1X61_REC_MIXER_LEFT0_MIXER_EN,
			    ADAU1X61_REC_MIXER_LEFT0_MIXER_EN);
	/* unmute right ADC mixer MX2 */
	adau1x61_update_reg(dev, ADAU1X61_REC_MIXER_RIGHT0, ADAU1X61_REC_MIXER_RIGHT0_MIXER_EN,
			    ADAU1X61_REC_MIXER_RIGHT0_MIXER_EN);
}

static void adau1x61_stop_input(const struct device *dev)
{
	/* mute left ADC mixer MX1 */
	adau1x61_update_reg(dev, ADAU1X61_REC_MIXER_LEFT0, ADAU1X61_REC_MIXER_LEFT0_MIXER_EN, 0);
	/* mute right ADC mixer MX2 */
	adau1x61_update_reg(dev, ADAU1X61_REC_MIXER_RIGHT0, ADAU1X61_REC_MIXER_RIGHT0_MIXER_EN, 0);

	/* powerdown ADC channels */
	adau1x61_update_reg(dev, ADAU1X61_ADC_CONTROL, ADAU1X61_ADC_CONTROL_ADCEN_MASK, 0);
}

static int adau1x61_set_output_volume(const struct device *dev, audio_channel_t channel, int vol)
{
	switch (channel) {
	case AUDIO_CHANNEL_LINE_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_LEFT_VOL, ADAU1X61_PLAY_VOL_MASK,
				    vol << 2);
		break;
	case AUDIO_CHANNEL_LINE_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_RIGHT_VOL, ADAU1X61_PLAY_VOL_MASK,
				    vol << 2);
		break;
	case AUDIO_CHANNEL_HEADPHONE_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_LEFT_VOL, ADAU1X61_PLAY_VOL_MASK,
				    vol << 2);
		break;
	case AUDIO_CHANNEL_HEADPHONE_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_RIGHT_VOL, ADAU1X61_PLAY_VOL_MASK,
				    vol << 2);
		break;
	case AUDIO_CHANNEL_MONOOUT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_MONO_OUTPUT_VOL, ADAU1X61_PLAY_VOL_MASK,
				    vol << 2);
		break;
	case AUDIO_CHANNEL_DIG_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_DAC_CONTROL1, DIGITAL_VOLUME_MASK, vol);
		break;
	case AUDIO_CHANNEL_DIG_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_DAC_CONTROL2, DIGITAL_VOLUME_MASK, vol);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int adau1x61_set_output_mute(const struct device *dev, audio_channel_t channel, bool mute)
{
	uint8_t reg;

	if (mute) {
		reg = 0;
	} else {
		reg = ADAU1X61_PLAY_MUTE_MASK;
	}

	switch (channel) {
	case AUDIO_CHANNEL_LINE_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_LEFT_VOL, ADAU1X61_PLAY_MUTE_MASK, reg);
		break;
	case AUDIO_CHANNEL_LINE_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_RIGHT_VOL, ADAU1X61_PLAY_MUTE_MASK,
				    reg);
		break;
	case AUDIO_CHANNEL_HEADPHONE_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_LEFT_VOL, ADAU1X61_PLAY_MUTE_MASK, reg);
		break;
	case AUDIO_CHANNEL_HEADPHONE_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_RIGHT_VOL, ADAU1X61_PLAY_MUTE_MASK, reg);
		break;
	case AUDIO_CHANNEL_MONOOUT:
		adau1x61_update_reg(dev, ADAU1X61_PLAY_MONO_OUTPUT_VOL, ADAU1X61_PLAY_MUTE_MASK,
				    reg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int adau1x61_set_input_volume(const struct device *dev, audio_channel_t channel, int vol)
{
	switch (channel) {
	case AUDIO_CHANNEL_MIC_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_LEFT_DIFF_INPUT_VOL, ADAU1X61_DIFF_INPUT_VOL_MASK,
				    vol << 2);
		break;
	case AUDIO_CHANNEL_MIC_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_RIGHT_DIFF_INPUT_VOL,
				    ADAU1X61_DIFF_INPUT_VOL_MASK, vol << 2);
		break;
	case AUDIO_CHANNEL_DIG_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_LEFT_INPUT_DIGITAL_VOL, DIGITAL_VOLUME_MASK, vol);
		break;
	case AUDIO_CHANNEL_DIG_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_RIGHT_INPUT_DIGITAL_VOL, DIGITAL_VOLUME_MASK, vol);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int adau1x61_set_input_mute(const struct device *dev, audio_channel_t channel, bool mute)
{
	uint8_t reg;

	if (mute) {
		reg = 0;
	} else {
		reg = ADAU1X61_PLAY_MUTE_MASK;
	}

	switch (channel) {
	case AUDIO_CHANNEL_MIC_LEFT:
		adau1x61_update_reg(dev, ADAU1X61_LEFT_DIFF_INPUT_VOL, ADAU1X61_PLAY_MUTE_MASK,
				    reg);
		break;
	case AUDIO_CHANNEL_MIC_RIGHT:
		adau1x61_update_reg(dev, ADAU1X61_RIGHT_DIFF_INPUT_VOL, ADAU1X61_PLAY_MUTE_MASK,
				    reg);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void adau1x61_read_all_regs(const struct device *dev)
{
	struct adau1x61_driver_data *const dev_data = DEV_DATA(dev);
	uint8_t val;

	adau1x61_read_reg(dev, ADAU1X61_CLOCK_CONTROL, &val);
	adau1x61_read_reg(dev, ADAU1X61_SERIAL_PORT0, &val);
	adau1x61_read_reg(dev, ADAU1X61_SERIAL_PORT1, &val);
	adau1x61_read_reg(dev, ADAU1X61_CONVERTER0, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_MIXER_LEFT0, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_MIXER_RIGHT0, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_HP_LEFT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_HP_RIGHT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_LINE_LEFT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_LINE_RIGHT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_DAC_CONTROL0, &val);
	adau1x61_read_reg(dev, ADAU1X61_DAC_CONTROL1, &val);
	adau1x61_read_reg(dev, ADAU1X61_DAC_CONTROL2, &val);
	if (dev_data->dsp_model) {
		adau1x61_read_reg(dev, ADAU1761_SERIAL_INPUT_ROUTE, &val);
		adau1x61_read_reg(dev, ADAU1761_SERIAL_OUTPUT_ROUTE, &val);
		adau1x61_read_reg(dev, ADAU1761_SERIAL_SAMPLING_RATE, &val);
		adau1x61_read_reg(dev, ADAU1761_CLK_ENABLE0, &val);
		adau1x61_read_reg(dev, ADAU1761_CLK_ENABLE1, &val);
	}
}
#endif

static const struct audio_codec_api adau1x61_driver_api = {
	.configure = adau1x61_configure,
	.start_output = adau1x61_start_output,
	.stop_output = adau1x61_stop_output,
	.start_input = adau1x61_start_input,
	.stop_input = adau1x61_stop_input,
	.set_property = adau1x61_set_property,
	.apply_properties = adau1x61_apply_properties,
};

#define ADAU1x61_INIT(n, model)                                                                    \
	static struct adau1x61_driver_data adau1x61_device_data_##n;                               \
                                                                                                   \
	static struct adau1x61_driver_config adau1x61_device_config_##n = {                        \
		.bus = I2C_DT_SPEC_INST_GET(n), model};                                            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, adau1x61_initialize, NULL, &adau1x61_device_data_##n,             \
			      &adau1x61_device_config_##n, POST_KERNEL,                            \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY, &adau1x61_driver_api);

#define ADAU1361_INIT(n) ADAU1x61_INIT(n, "ADAU1361")
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adau1361
DT_INST_FOREACH_STATUS_OKAY(ADAU1361_INIT)

#define ADAU1761_INIT(n) ADAU1x61_INIT(n, "ADAU1761")
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adau1761
DT_INST_FOREACH_STATUS_OKAY(ADAU1761_INIT)

#define ADAU1961_INIT(n) ADAU1x61_INIT(n, "ADAU1961")
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adau1961
DT_INST_FOREACH_STATUS_OKAY(ADAU1961_INIT)
