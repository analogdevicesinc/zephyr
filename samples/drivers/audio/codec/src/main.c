/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/logging/log_ctrl.h>

#define SAMPLE_FREQUENCY    48000
#define SAMPLE_BIT_WIDTH    32
#define BYTES_PER_SAMPLE    sizeof(int32_t)
#define NUMBER_OF_CHANNELS  2
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK   ((SAMPLE_FREQUENCY / 10) * NUMBER_OF_CHANNELS)
#define INITIAL_BLOCKS      2
#define TIMEOUT             1000

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT (INITIAL_BLOCKS + 2)
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static K_SEM_DEFINE(toggle_transfer, 1, 1);

int main(void)
{
	struct i2s_config i2s_cfg;
	struct audio_codec_cfg codec_cfg;
	int ret;
	const struct device *dev_i2s = DEVICE_DT_GET(DT_NODELABEL(i2s_rxtx));
	const struct device *dev_codec = DEVICE_DT_GET(DT_NODELABEL(codec));
	audio_property_value_t prop_val;

	if (!device_is_ready(dev_i2s)) {
		printf("I2S device not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(dev_codec)) {
		printf("Audio codec device not ready\n");
		return -ENODEV;
	}

	/* Configure I2S stream */
	i2s_cfg.word_size = SAMPLE_BIT_WIDTH;
	i2s_cfg.channels = NUMBER_OF_CHANNELS;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = SAMPLE_FREQUENCY;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = TIMEOUT;
	/* Configure the Transmit port as Master */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER
			| I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &mem_slab;
	ret = i2s_configure(dev_i2s, I2S_DIR_BOTH, &i2s_cfg);
	if (ret < 0) {
		printf("Failed to configure I2S stream\n");
		return ret;
	}

	/* Configure Audio codec */
	codec_cfg.dai_cfg.i2s = i2s_cfg;
	codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	codec_cfg.mclk_freq = 12288000;
	ret = audio_codec_configure(dev_codec, &codec_cfg);
	if (ret < 0) {
		printf("Failed to configure codec\n");
		return ret;
	}

	audio_codec_start_output(dev_codec);
	audio_codec_start_input(dev_codec);
	prop_val.vol = 7;
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_HEADPHONE_LEFT, prop_val);
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_HEADPHONE_RIGHT, prop_val);
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_LINE_LEFT, prop_val);
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_LINE_RIGHT, prop_val);
	prop_val.mute = 0;
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_HEADPHONE_LEFT, prop_val);
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_HEADPHONE_RIGHT, prop_val);
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_LINE_LEFT, prop_val);
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_LINE_RIGHT, prop_val);

	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
		void *mem_block;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate TX block %d: %d\n", i, ret);
			return false;
		}

		memset(mem_block, 0, BLOCK_SIZE);

		ret = i2s_write(dev_i2s, mem_block, BLOCK_SIZE);
		if (ret < 0) {
			printk("Failed to write block %d: %d\n", i, ret);
			return false;
		}
	}

	for (;;) {
		k_sem_take(&toggle_transfer, K_FOREVER);

		/* Trigger the I2S transmission */
		ret = i2s_trigger(dev_i2s, I2S_DIR_BOTH, I2S_TRIGGER_START);
		if (ret < 0) {
			printf("Could not trigger I2S tx\n");
			return ret;
		}

		printk("Streams started\n");

		while (k_sem_take(&toggle_transfer, K_NO_WAIT) != 0) {
			void *mem_block;
			uint32_t block_size;
			int ret;

			ret = i2s_read(dev_i2s, &mem_block, &block_size);
			if (ret < 0) {
				printk("Failed to read data: %d\n", ret);
				break;
			}

			ret = i2s_write(dev_i2s, mem_block, block_size);
			if (ret < 0) {
				printk("Failed to write data: %d\n", ret);
				break;
			}
		}

		/* Trigger the I2S transmission */
		ret = i2s_trigger(dev_i2s, I2S_DIR_BOTH, I2S_TRIGGER_DROP);
		if (ret < 0) {
			printf("Could not trigger I2S tx\n");
			return ret;
		}

		printk("Streams stopped\n");
	}

	audio_codec_stop_output(dev_codec);

	return 0;
}
