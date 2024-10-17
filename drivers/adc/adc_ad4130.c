/*
 * Copyright (c) 2023 Mustafa Abdullah Kus, Sparse Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad4130_adc

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_ad4130, CONFIG_ADC_LOG_LEVEL);

#define AD4130_CONFIG_PGA(x)     BIT(x)
#define AD4130_CONFIG_CHANNEL(x) ((x) << 5)
#define AD4130_CONFIG_CHMAP(x)   ((x << 2) | BIT(1))
#define AD4130_REG_DATA(x)       (AD4130_REG_DATA0 + (x << 1))

#define AD4130_CMD_READ        0xC1
#define AD4130_CMD_WRITE       0xC0
#define AD4130_CMD_CONV        0x80
#define AD4130_CMD_CALIBRATION 0x20
#define AD4130_CMD_SEQUENCER   0x30

enum ad4130_mode {
	AD4130_MODE_POWERDOWN = 0x01,
	AD4130_MODE_CALIBRATION = 0x02,
	AD4130_MODE_SEQUENCER = 0x03,
};

enum {
	AD4130_CONFIG_RATE_1_9 = 0x00,
	AD4130_CONFIG_RATE_3_9 = 0x01,
	AD4130_CONFIG_RATE_7_8 = 0x02,
	AD4130_CONFIG_RATE_15_6 = 0x03,
	AD4130_CONFIG_RATE_31_2 = 0x04,
	AD4130_CONFIG_RATE_62_5 = 0x05,
	AD4130_CONFIG_RATE_125 = 0x06,
	AD4130_CONFIG_RATE_250 = 0x07,
	AD4130_CONFIG_RATE_500 = 0x08,
	AD4130_CONFIG_RATE_1000 = 0x09,
	AD4130_CONFIG_RATE_2000 = 0x0A,
	AD4130_CONFIG_RATE_4000 = 0x0B,
	AD4130_CONFIG_RATE_8000 = 0x0C,
	AD4130_CONFIG_RATE_16000 = 0x0D,
	AD4130_CONFIG_RATE_32000 = 0x0E,
	AD4130_CONFIG_RATE_64000 = 0x0F,
};

enum ad4130_reg {
	AD4130_REG_STAT = 0x00,
	AD4130_REG_CTRL1 = 0x02,
	AD4130_REG_CTRL2 = 0x04,
	AD4130_REG_CTRL3 = 0x06,
	AD4130_REG_GPIO_CTRL = 0x08,
	AD4130_REG_DELAY = 0x0A,
	AD4130_REG_CHMAP1 = 0x0C,
	AD4130_REG_CHMAP0 = 0x0E,
	AD4130_REG_SEQ = 0x10,
	AD4130_REG_GPO_DIR = 0x12,
	AD4130_REG_SOC = 0x14,
	AD4130_REG_SGC = 0x16,
	AD4130_REG_SCOC = 0x18,
	AD4130_REG_SCGC = 0x1A,
	AD4130_REG_DATA0 = 0x1C,
	AD4130_REG_DATA1 = 0x1E,
	AD4130_REG_DATA2 = 0x20,
	AD4130_REG_DATA3 = 0x22,
	AD4130_REG_DATA4 = 0x24,
	AD4130_REG_DATA5 = 0x26,
};

enum {
	AD4130_REG_STAT_LEN = 3,
	AD4130_REG_CTRL1_LEN = 1,
	AD4130_REG_CTRL2_LEN = 1,
	AD4130_REG_CTRL3_LEN = 1,
	AD4130_REG_GPIO_CTRL_LEN = 1,
	AD4130_REG_DELAY_LEN = 2,
	AD4130_REG_CHMAP1_LEN = 3,
	AD4130_REG_CHMAP0_LEN = 3,
	AD4130_REG_SEQ_LEN = 1,
	AD4130_REG_GPO_DIR_LEN = 1,
	AD4130_REG_SOC_LEN = 3,
	AD4130_REG_SGC_LEN = 3,
	AD4130_REG_SCOC_LEN = 3,
	AD4130_REG_SCGC_LEN = 3,
};

enum {
	AD4130_CTRL1_CAL_SELF = 0,
	AD4130_CTRL1_CAL_OFFSET = 1,
	AD4130_CTRL1_CAL_FULLSCALE = 2,
};

enum {
	AD4130_CTRL1_PD_NOP = 0,
	AD4130_CTRL1_DP_SLEEP = 1,
	AD4130_CTRL1_DP_STANDBY = 2,
	AD4130_CTRL1_DP_RESET = 3,
};

enum {
	AD4130_CTRL1_CONTSC = 0,
	AD4130_CTRL1_SCYCLE = 1,
	AD4130_CTRL1_FORMAT = 2,
	AD4130_CTRL1_UBPOLAR = 3,
};

enum {
	AD4130_CTRL2_PGA_GAIN_1 = 0,
	AD4130_CTRL2_PGA_GAIN_2 = 1,
	AD4130_CTRL2_PGA_GAIN_4 = 2,
	AD4130_CTRL2_PGA_GAIN_8 = 3,
	AD4130_CTRL2_PGA_GAIN_16 = 4,
	AD4130_CTRL2_PGA_GAIN_32 = 5,
	AD4130_CTRL2_PGA_GAIN_64 = 6,
	AD4130_CTRL2_PGA_GAIN_128 = 7,
};

enum {
	AD4130_CTRL2_PGAEN = 3,
	AD4130_CTRL2_LPMODE = 4,
	AD4130_CTRL2_LDOEN = 5,
	AD4130_CTRL2_CSSEN = 6,
	AD4130_CTRL2_EXTCLK = 7,
};

enum {
	AD4130_CTRL3_NOSCO = 0,
	AD4130_CTRL3_NOSCG = 1,
	AD4130_CTRL3_NOSYSO = 2,
	AD4130_CTRL3_NOSYSG = 3,
	AD4130_CTRL3_CALREGSEL = 4,
	AD4130_CTRL3_SYNC_MODE = 5,
	AD4130_CTRL3_GPO_MODE = 6,
};

enum {
	AD4130_GPIO_CTRL_DIO0 = 0,
	AD4130_GPIO_CTRL_DIO1 = 1,
	AD4130_GPIO_CTRL_DIRO = 3,
	AD4130_GPIO_CTRL_DIR1 = 4,
	AD4130_GPIO_CTRL_GPIO0_EN = 6,
	AD4130_GPIO_CTRL_GPIO1_EN = 7,
};

enum {
	AD4130_SEQ_RDYBEN = 0,
	AD4130_SEQ_MDREN = 1,
	AD4130_SEQ_GPODREN = 2,
	AD4130_SEQ_MODE0 = 3,
	AD4130_SEQ_MODE1 = 4,
	AD4130_SEQ_MUX0 = 5,
	AD4130_SEQ_MUX1 = 6,
	AD4130_SEQ_MUX2 = 7,
};

enum {
	AD4130_GPO_DIR_GPO0 = 0,
	AD4130_GPO_DIR_GPO1 = 1,
};

enum {
	AD4130_CMD_RATE0 = 0,
	AD4130_CMD_RATE1 = 1,
	AD4130_CMD_RATE2 = 2,
	AD4130_CMD_RATE3 = 3,
};

enum {
	AD4130_CHANNEL_0 = 0x0,
	AD4130_CHANNEL_1 = 0x1,
	AD4130_CHANNEL_2 = 0x2,
	AD4130_CHANNEL_3 = 0x3,
	AD4130_CHANNEL_4 = 0x4,
	AD4130_CHANNEL_5 = 0x5,
};

enum {
	AD4130_CMD_MODE0 = 4,
	AD4130_CMD_MODE1 = 5,
};

struct ad4130_gpio_ctrl {
	bool gpio0_enable;
	bool gpio1_enable;
	bool gpio0_direction;
	bool gpio1_direction;
};

struct ad4130_gpo_ctrl {
	bool gpo0_enable;
	bool gpo1_enable;
};

struct ad4130_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec drdy_gpio;
	const uint32_t odr_delay[16];
	uint8_t resolution;
	bool multiplexer;
	bool pga;
	bool self_calibration;
	struct ad4130_gpio_ctrl gpio;
	struct ad4130_gpo_ctrl gpo;
};

struct ad4130_data {
	const struct device *dev;
	struct adc_context ctx;
	uint8_t rate;
	struct gpio_callback callback_data_ready;
	struct k_sem acq_sem;
	struct k_sem data_ready_signal;
	int32_t *buffer;
	int32_t *repeat_buffer;
	struct k_thread thread;
	bool differential;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_AD4130_ACQUISITION_THREAD_STACK_SIZE);
};

static void ad4130_data_ready_handler(const struct device *dev, struct gpio_callback *gpio_cb,
				      uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct ad4130_data *data = CONTAINER_OF(gpio_cb, struct ad4130_data, callback_data_ready);

	k_sem_give(&data->data_ready_signal);
}

static int ad4130_read_reg(const struct device *dev, enum ad4130_reg reg_addr, uint8_t *buffer,
			   size_t reg_size)
{
	int ret;
	const struct ad4130_config *config = dev->config;
	uint8_t buffer_tx[3];
	uint8_t buffer_rx[ARRAY_SIZE(buffer_tx)];
	const struct spi_buf tx_buf[] = {{
		.buf = buffer_tx,
		.len = ARRAY_SIZE(buffer_tx),
	}};
	const struct spi_buf rx_buf[] = {{
		.buf = buffer_rx,
		.len = ARRAY_SIZE(buffer_rx),
	}};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};
	buffer_tx[0] = AD4130_CMD_READ | reg_addr;
	/* read one register */
	buffer_tx[1] = 0x00;

	ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret != 0) {
		LOG_ERR("AD4130: error writing register 0x%X (%d)", reg_addr, ret);
		return ret;
	}
	*buffer = buffer_rx[1];
	LOG_DBG("read from register 0x%02X value 0x%02X", reg_addr, *buffer);

	return 0;
}

static int ad4130_write_reg(const struct device *dev, enum ad4130_reg reg_addr, uint8_t *reg_val,
			    size_t reg_size)
{
	int ret;
	const struct ad4130_config *config = dev->config;
	uint8_t command = AD4130_CMD_WRITE | reg_addr;

	const struct spi_buf spi_buf[2] = {{.buf = &command, .len = sizeof(command)},
					   {.buf = reg_val, .len = reg_size}};
	const struct spi_buf_set tx = {.buffers = spi_buf, .count = ARRAY_SIZE(spi_buf)};

	ret = spi_write_dt(&config->bus, &tx);
	if (ret != 0) {
		LOG_ERR("AD4130: error writing register 0x%X (%d)", reg_addr, ret);
		return ret;
	}

	return 0;
}

static int ad4130_send_command(const struct device *dev, enum ad4130_mode mode, uint8_t rate)
{
	int ret;
	const struct ad4130_config *config = dev->config;
	uint8_t command = AD4130_CMD_CONV | mode | rate;
	const struct spi_buf spi_buf = {.buf = &command, .len = sizeof(command)};
	const struct spi_buf_set tx = {.buffers = &spi_buf, .count = 1};

	ret = spi_write_dt(&config->bus, &tx);
	if (ret != 0) {
		LOG_ERR("AD4130: error writing register 0x%X (%d)", rate, ret);
		return ret;
	}

	return 0;
}

static int ad4130_start_conversion(const struct device *dev)
{
	const struct ad4130_data *data = dev->data;

	return ad4130_send_command(dev, AD4130_CMD_SEQUENCER, data->rate);
}

static inline int ad4130_acq_time_to_dr(const struct device *dev, uint16_t acq_time)
{
	struct ad4130_data *data = dev->data;
	const struct ad4130_config *config = dev->config;
	const uint32_t *odr_delay = config->odr_delay;
	uint32_t odr_delay_us = 0;
	uint16_t acq_value = ADC_ACQ_TIME_VALUE(acq_time);
	int odr = -EINVAL;

	if (acq_time != ADC_ACQ_TIME_DEFAULT && ADC_ACQ_TIME_UNIT(acq_time) != ADC_ACQ_TIME_TICKS) {
		LOG_ERR("AD4130: invalid acq time value (%d)", acq_time);
		return -EINVAL;
	}

	if (acq_value < AD4130_CONFIG_RATE_1_9 || acq_value > AD4130_CONFIG_RATE_64000) {
		LOG_ERR("AD4130: invalid acq value (%d)", acq_value);
		return -EINVAL;
	}

	odr = acq_value;
	odr_delay_us = odr_delay[acq_value];

	data->rate = odr;

	return odr;
}

static int ad4130_wait_data_ready(const struct device *dev)
{
	struct ad4130_data *data = dev->data;

	return k_sem_take(&data->data_ready_signal, ADC_CONTEXT_WAIT_FOR_COMPLETION_TIMEOUT);
}

static int ad4130_read_sample(const struct device *dev)
{
	const struct ad4130_config *config = dev->config;
	struct ad4130_data *data = dev->data;
	bool is_positive;
	uint8_t buffer_tx[(config->resolution / 8) + 1];
	uint8_t buffer_rx[ARRAY_SIZE(buffer_tx)];
	uint8_t current_channel = find_msb_set(data->ctx.sequence.channels) - 1;
	int rc;

	const struct spi_buf tx_buf[] = {{
		.buf = buffer_tx,
		.len = ARRAY_SIZE(buffer_tx),
	}};
	const struct spi_buf rx_buf[] = {{
		.buf = buffer_rx,
		.len = ARRAY_SIZE(buffer_rx),
	}};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	buffer_tx[0] = AD4130_CMD_READ | AD4130_REG_DATA(current_channel);

	rc = spi_transceive_dt(&config->bus, &tx, &rx);
	if (rc != 0) {
		LOG_ERR("spi_transceive failed with error %i", rc);
		return rc;
	}

	/* The data format while in unipolar mode is always offset binary.
	 * In offset binary format the most negative value is 0x000000,
	 * the midscale value is 0x800000 and the most positive value is
	 * 0xFFFFFF. In bipolar mode if the FORMAT bit = ‘1’ then the
	 * data format is offset binary. If the FORMAT bit = ‘0’, then
	 * the data format is two’s complement. In two’s complement the
	 * negative full-scale value is 0x800000, the midscale is 0x000000
	 * and the positive full scale is 0x7FFFFF. Any input exceeding
	 * the available input range is limited to the minimum or maximum
	 * data value.
	 */
	is_positive = buffer_rx[(config->resolution / 8)] >> 7;
	if (is_positive) {
		*data->buffer++ = sys_get_be24(buffer_rx) - (1 << (config->resolution - 1));
	} else {
		*data->buffer++ = sys_get_be24(buffer_rx + 1);
	}

	adc_context_on_sampling_done(&data->ctx, dev);

	return rc;
}

static int ad4130_configure_chmap(const struct device *dev, const uint8_t channel_id)
{
	uint8_t last_order = 0;
	uint8_t chmap1_register[3] = {0};
	uint8_t chmap0_register[3] = {0};

	if (channel_id > 6) {
		LOG_ERR("AD4130: invalid channel (%u)", channel_id);
		return -EINVAL;
	}

	ad4130_read_reg(dev, AD4130_REG_CHMAP1, chmap1_register, AD4130_REG_CHMAP1_LEN);
	for (int index = 0; index < 3; index++) {
		if ((chmap1_register[index] >> 2) >= last_order) {
			last_order = chmap1_register[index] >> 2;
		} else {
			continue;
		}
	}

	ad4130_read_reg(dev, AD4130_REG_CHMAP0, chmap0_register, AD4130_REG_CHMAP0_LEN);
	for (int index = 0; index < 3; index++) {
		if ((chmap0_register[index] >> 2) >= last_order) {
			last_order = chmap0_register[index] >> 2;
		} else {
			continue;
		}
	}

	last_order++;

	switch (channel_id) {
	case AD4130_CHANNEL_0:
		chmap0_register[2] = AD4130_CONFIG_CHMAP(last_order);
		break;
	case AD4130_CHANNEL_1:
		chmap0_register[1] = AD4130_CONFIG_CHMAP(last_order);
		break;
	case AD4130_CHANNEL_2:
		chmap0_register[0] = AD4130_CONFIG_CHMAP(last_order);
		break;
	case AD4130_CHANNEL_3:
		chmap1_register[2] = AD4130_CONFIG_CHMAP(last_order);
		break;
	case AD4130_CHANNEL_4:
		chmap1_register[1] = AD4130_CONFIG_CHMAP(last_order);
		break;
	case AD4130_CHANNEL_5:
		chmap1_register[0] = AD4130_CONFIG_CHMAP(last_order);
		break;
	default:
		break;
	}

	if (channel_id > 3) {
		/* CHMAP 1 register configuration */
		ad4130_write_reg(dev, AD4130_REG_CHMAP1, chmap1_register, AD4130_REG_CHMAP1_LEN);
	} else {
		/* CHMAP 0 register configuration */
		ad4130_write_reg(dev, AD4130_REG_CHMAP0, chmap0_register, AD4130_REG_CHMAP0_LEN);
	}

	return 0;
}

static int ad4130_self_calibration(const struct device *dev)
{
	uint8_t seq_register = 0;
	uint8_t ctrl1_register = BIT(AD4130_CTRL1_SCYCLE);

	ad4130_write_reg(dev, AD4130_REG_SEQ, &seq_register, AD4130_REG_SEQ_LEN);
	ad4130_write_reg(dev, AD4130_REG_CTRL1, &ctrl1_register, AD4130_REG_CTRL1_LEN);
	ad4130_send_command(dev, AD4130_CMD_CALIBRATION, 0x00);

	k_sleep(K_MSEC(200));
	return 0;
}

static int ad4130_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
	const struct ad4130_config *config = dev->config;
	uint8_t seq_register = 0;
	uint8_t ctrl2_register = 0;
	uint8_t gpio_reg = 0;
	uint8_t gpo_reg = 0;

	/* sequencer register configuration */
	ad4130_read_reg(dev, AD4130_REG_SEQ, &seq_register, AD4130_REG_SEQ_LEN);
	seq_register |= BIT(AD4130_SEQ_MDREN);
	seq_register |= BIT(AD4130_SEQ_MODE0);
	ad4130_write_reg(dev, AD4130_REG_SEQ, &seq_register, AD4130_REG_SEQ_LEN);

	/* configuration multiplexer */
	if (config->multiplexer) {
		if (!channel_cfg->differential) {
			LOG_ERR("6 channel fully supported only supported differential "
				"differemtial option %i",
				channel_cfg->differential);
			return -ENOTSUP;
		}
	}

	ad4130_acq_time_to_dr(dev, channel_cfg->acquisition_time);

	/* ctrl2 register configuration */
	if (config->pga) {
		/* programmable gain amplifier support */
		ctrl2_register |= AD4130_CONFIG_PGA(AD4130_CTRL2_PGAEN);
		switch (channel_cfg->gain) {
		case ADC_GAIN_1:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_1;
			break;
		case ADC_GAIN_2:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_2;
			break;
		case ADC_GAIN_4:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_4;
			break;
		case ADC_GAIN_8:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_8;
			break;
		case ADC_GAIN_16:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_16;
			break;
		case ADC_GAIN_32:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_32;
			break;
		case ADC_GAIN_64:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_64;
			break;
		case ADC_GAIN_128:
			ctrl2_register |= AD4130_CTRL2_PGA_GAIN_128;
			break;
		default:
			LOG_ERR("AD4130: unsupported channel gain '%d'", channel_cfg->gain);
			return -ENOTSUP;
		}
	}

	if (channel_cfg->reference == ADC_REF_INTERNAL) {
		ctrl2_register |= BIT(AD4130_CTRL2_LDOEN);

	} else if (channel_cfg->reference == ADC_REF_EXTERNAL1) {
		ctrl2_register &= ~BIT(AD4130_CTRL2_LDOEN);
	} else {
		LOG_ERR("AD4130: unsupported channel reference type '%d'", channel_cfg->reference);
		return -ENOTSUP;
	}
	ad4130_write_reg(dev, AD4130_REG_CTRL2, &ctrl2_register, AD4130_REG_CTRL2_LEN);

	/* GPIO_CTRL register configuration */
	gpio_reg |= config->gpio.gpio0_enable << AD4130_GPIO_CTRL_GPIO0_EN;
	gpio_reg |= config->gpio.gpio1_enable << AD4130_GPIO_CTRL_GPIO1_EN;
	gpio_reg |= config->gpio.gpio0_direction << AD4130_GPIO_CTRL_DIRO;
	gpio_reg |= config->gpio.gpio1_direction << AD4130_GPIO_CTRL_DIR1;
	ad4130_write_reg(dev, AD4130_REG_GPIO_CTRL, &gpio_reg, AD4130_REG_GPIO_CTRL_LEN);

	/* GPO_DIR register configuration */
	gpo_reg |= config->gpo.gpo0_enable << AD4130_GPO_DIR_GPO0;
	gpo_reg |= config->gpo.gpo1_enable << AD4130_GPO_DIR_GPO1;
	ad4130_write_reg(dev, AD4130_REG_GPO_DIR, &gpo_reg, AD4130_REG_GPO_DIR_LEN);

	/* configuration of channel order */
	ad4130_configure_chmap(dev, channel_cfg->channel_id);

	return 0;
}

static int ad4130_validate_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = sizeof(uint8_t) * (sequence->resolution / 8);

	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int ad4130_validate_sequence(const struct device *dev, const struct adc_sequence *sequence)
{
	int err;

	if (sequence->oversampling) {
		LOG_ERR("AD4130: oversampling not supported");
		return -ENOTSUP;
	}

	err = ad4130_validate_buffer_size(sequence);
	if (err) {
		LOG_ERR("AD4130: buffer size too small");
		return -ENOTSUP;
	}

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ad4130_data *data = CONTAINER_OF(ctx, struct ad4130_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ad4130_data *data = CONTAINER_OF(ctx, struct ad4130_data, ctx);

	data->repeat_buffer = data->buffer;

	ad4130_start_conversion(data->dev);

	k_sem_give(&data->acq_sem);
}

static int ad4130_adc_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	int rc;
	struct ad4130_data *data = dev->data;

	rc = ad4130_validate_sequence(dev, sequence);
	if (rc != 0) {
		return rc;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int ad4130_adc_read_async(const struct device *dev, const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	int rc;
	struct ad4130_data *data = dev->data;

	adc_context_lock(&data->ctx, async ? true : false, async);
	rc = ad4130_adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, rc);

	return rc;
}

static int ad4130_adc_perform_read(const struct device *dev)
{
	struct ad4130_data *data = dev->data;
	int rc;

	rc = ad4130_read_sample(dev);
	if (rc != 0) {
		LOG_ERR("reading sample failed (err %d)", rc);
		adc_context_complete(&data->ctx, rc);
		return rc;
	}

	return rc;
}

static int ad4130_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return ad4130_adc_read_async(dev, sequence, NULL);
}

static void ad4130_acquisition_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ad4130_data *data = dev->data;
	int rc;

	while (true) {
		k_sem_take(&data->acq_sem, K_FOREVER);

		rc = ad4130_wait_data_ready(dev);
		if (rc != 0) {
			LOG_ERR("AD4130: failed to get ready status (err %d)", rc);
			adc_context_complete(&data->ctx, rc);
			break;
		}

		ad4130_adc_perform_read(dev);
	}
}

static int ad4130_init(const struct device *dev)
{
	int err;
	const struct ad4130_config *config = dev->config;
	struct ad4130_data *data = dev->data;

	data->dev = dev;

	k_sem_init(&data->acq_sem, 0, 1);
	k_sem_init(&data->data_ready_signal, 0, 1);

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("spi bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	if (config->self_calibration) {
		LOG_INF("performing self calibration process");
		ad4130_self_calibration(dev);
	}

	err = gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT);
	if (err != 0) {
		LOG_ERR("failed to initialize GPIO for data ready (err %d)", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		LOG_ERR("failed to configure data ready interrupt (err %d)", err);
		return -EIO;
	}

	gpio_init_callback(&data->callback_data_ready, ad4130_data_ready_handler,
			   BIT(config->drdy_gpio.pin));
	err = gpio_add_callback(config->drdy_gpio.port, &data->callback_data_ready);
	if (err != 0) {
		LOG_ERR("failed to add data ready callback (err %d)", err);
		return -EIO;
	}

	k_tid_t tid =
		k_thread_create(&data->thread, data->stack, K_KERNEL_STACK_SIZEOF(data->stack),
				ad4130_acquisition_thread, (void *)dev, NULL, NULL,
				CONFIG_ADC_AD4130_ACQUISITION_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(tid, "adc_ad4130");

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api adc_ad4130_driver_api = {
	.channel_setup = ad4130_channel_setup,
	.read = ad4130_read,
	.ref_internal = 2048,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ad4130_adc_read_async,
#endif
};

#define AD4130_ADC_INIT(_num)                                                                      \
	static const struct ad4130_config ad4130_config_##_num = {                                 \
		.bus = SPI_DT_SPEC_GET(DT_INST(_num, adi_ad4130_adc),                              \
				       SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,    \
				       1),                                                         \
		.odr_delay = 0,                                                                    \
		.resolution = 0,                                                                   \
		.multiplexer = 0,                                                                  \
		.pga = 0,                                                                          \
		.drdy_gpio = GPIO_DT_SPEC_GET_OR(DT_INST(_num, adi_ad4130_adc), drdy_gpios, {0}),  \
		.self_calibration = DT_INST_PROP_OR(_num, self_calibration, 0),                    \
		.gpio.gpio0_enable = DT_INST_PROP_OR(_num, gpio0_enable, 1),                       \
		.gpio.gpio1_enable = DT_INST_PROP_OR(_num, gpio1_enable, 0),                       \
		.gpio.gpio0_direction = DT_INST_PROP_OR(_num, gpio0_direction, 0),                 \
		.gpio.gpio1_direction = DT_INST_PROP_OR(_num, gpio1_direction, 0),                 \
		.gpo.gpo0_enable = DT_INST_PROP_OR(_num, gpo1_enable, 0),                          \
		.gpo.gpo1_enable = DT_INST_PROP_OR(_num, gpo1_enable, 0),                          \
	};                                                                                         \
	static struct ad4130_data ad4130_data_##_num = {                                           \
		ADC_CONTEXT_INIT_LOCK(ad4130_data_##_num, ctx),                                    \
		ADC_CONTEXT_INIT_TIMER(ad4130_data_##_num, ctx),                                   \
		ADC_CONTEXT_INIT_SYNC(ad4130_data_##_num, ctx),                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_num, &ad4130_init, NULL, &ad4130_data_##_num,                       \
			      &ad4130_config_##_num, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,        \
			      &adc_ad4130_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AD4130_ADC_INIT)

/* Each data register is a 16-bit read-only register. Any attempt to write
 * data to this location will have no effect. The data read from these
 * registers is clocked out MSB first. The result is stored in a format
 * according to the FORMAT bit in the CTRL1 register. The data format
 * while in unipolar mode is always offset binary. In offset binary
 * format the most negative value is 0x0000, the midscale value is 0x8000 and
 *  the most positive value is 0xFFFF. In bipolar mode if the FORMAT
 * bit = ‘1’ then the data format is offset binary. If the FORMAT
 * bit= ‘0’, then the data format is two’s complement. In two’s
 * complement the negative full-scale value is 0x8000, the midscale is 0x0000
 * and the positive full scale is 0x7FFF. Any input exceeding the available
 * input range is limited to the minimum or maximum data value.
 */

#define AD4130_RESOLUTION 24

/*
 * Approximated AD4130 acquisition times in microseconds. These are
 * used for the initial delay when polling for data ready.
 *
 * {1.9 SPS, 3.9 SPS, 7.8 SPS, 15.6 SPS, 31.2 SPS, 62.5 SPS, 125 SPS, 250 SPS, 500 SPS,
 * 1000 SPS, 2000 SPS, 4000 SPS, 8000 SPS, 16000 SPS, 32000 SPS, 64000 SPS}
 */
#define AD4130_ODR_DELAY_US                                                                        \
	{                                                                                          \
		526315, 256410, 128205, 64102, 32051, 16000, 8000, 4000, 2000, 1000, 500, 250,     \
			125, 62, 31, 15                                                            \
	}
