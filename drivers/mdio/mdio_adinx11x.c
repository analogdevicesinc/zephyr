/*
 * Copyright (c) 2023 PHOENIX CONTACT Electronics GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_adinx11x, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT adi_adinx11x_mdio

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/mdio/mdio_adinx11x.h>
#include <zephyr/drivers/ethernet/eth_adinx11x.h>

/* MDIO ready check retry delay */
#define ADINX11X_MDIO_READY_AWAIT_DELAY_POLL_US		5U
/* Number of retries for MDIO ready check */
#define ADINX11X_MDIO_READY_AWAIT_RETRY_COUNT		10U

/* MDIO Access Register 1 */
#define ADINX11X_MDIOACC0				0x20U
/* MDIO Access Register 2 */
#define ADINX11X_MDIOACC1				0x21U

/* MDIO MDIOACC Transaction Done */
#define ADINX11X_MDIOACC_MDIO_TRDONE			BIT(31)

struct mdio_adinx11x_config {
	const struct device *adin;
};

static int mdio_adinx11x_wait_ready(const struct device *dev, uint16_t reg,
				    uint32_t *out)
{
	const struct mdio_adinx11x_config *const cfg = dev->config;
	uint32_t count;
	int ret;

	for (count = 0U; count < ADINX11X_MDIO_READY_AWAIT_RETRY_COUNT; ++count) {
		ret = eth_adinx11x_reg_read(cfg->adin, reg, out);
		if (ret >= 0) {
			if ((*out) & ADINX11X_MDIOACC_MDIO_TRDONE) {
				break;
			}
			ret = -ETIMEDOUT;
		}
		k_sleep(K_USEC(ADINX11X_MDIO_READY_AWAIT_DELAY_POLL_US));
	}

	return ret;
}


int adinx11x_mdio_c45_read(const struct device *dev, uint8_t prtad,
			   uint8_t devad, uint16_t regad,
			   uint16_t *data)
{
	const struct mdio_adinx11x_config *const cfg = dev->config;
	uint32_t rdy;
	uint32_t cmd;
	int ret;

	/* address op */
	cmd = (prtad & 0x1FU) << 21;
	cmd |= (devad & 0x1FU) << 16;
	cmd |= regad;

	ret = eth_adinx11x_reg_write(cfg->adin, ADINX11X_MDIOACC0, cmd);
	if (ret < 0) {
		return ret;
	}

	/* read op */
	cmd = (cmd & ~UINT16_MAX) | (0x3U << 26);

	ret = eth_adinx11x_reg_write(cfg->adin, ADINX11X_MDIOACC1, cmd);
	if (ret < 0) {
		return ret;
	}

	ret = mdio_adinx11x_wait_ready(dev, ADINX11X_MDIOACC1, &rdy);
	if (ret < 0) {
		return ret;
	}

	/* read out */
	ret = eth_adinx11x_reg_read(cfg->adin, ADINX11X_MDIOACC1, &cmd);

	*data = cmd & UINT16_MAX;

	return ret;
}

int adinx11x_mdio_c45_write(const struct device *dev, uint8_t prtad,
			    uint8_t devad, uint16_t regad,
			    uint16_t data)
{
	const struct mdio_adinx11x_config *const cfg = dev->config;

	uint32_t rdy;
	uint32_t cmd;
	int ret;

	/* address op */
	cmd = (prtad & 0x1FU) << 21;
	cmd |= (devad & 0x1FU) << 16;
	cmd |= regad;

	ret = eth_adinx11x_reg_write(cfg->adin, ADINX11X_MDIOACC0, cmd);
	if (ret < 0) {
		return ret;
	}

	/* write op */
	cmd |= BIT(26);
	cmd = (cmd & ~UINT16_MAX) | data;

	ret = eth_adinx11x_reg_write(cfg->adin, ADINX11X_MDIOACC1, cmd);
	if (ret < 0) {
		return ret;
	}

	ret = mdio_adinx11x_wait_ready(dev, ADINX11X_MDIOACC1, &rdy);

	return ret;
}

static int mdio_adinx11x_read(const struct device *dev, uint8_t prtad,
			      uint8_t devad, uint16_t *data)
{
	const struct mdio_adinx11x_config *const cfg = dev->config;
	uint32_t read;
	uint32_t cmd;
	int ret;

	cmd = BIT(28);
	cmd |= 0x3U << 26;
	cmd |= (prtad & 0x1FU) << 21;
	cmd |= (devad & 0x1FU) << 16;

	ret = eth_adinx11x_reg_write(cfg->adin, ADINX11X_MDIOACC0, cmd);
	if (ret >= 0) {
		ret = mdio_adinx11x_wait_ready(dev, ADINX11X_MDIOACC0, &read);
		*data = read & UINT16_MAX;
	}

	return ret;
}

static int mdio_adinx11x_write(const struct device *dev, uint8_t prtad,
			       uint8_t devad, uint16_t data)
{
	const struct mdio_adinx11x_config *const cfg = dev->config;
	uint32_t cmd;
	uint32_t rdy;
	int ret;

	cmd = BIT(28);
	cmd |= BIT(26);
	cmd |= (prtad & 0x1FU) << 21;
	cmd |= (devad & 0x1FU) << 16;
	cmd |= data;

	ret = eth_adinx11x_reg_write(cfg->adin, ADINX11X_MDIOACC0, cmd);
	if (ret >= 0) {
		ret = mdio_adinx11x_wait_ready(dev, ADINX11X_MDIOACC0, &rdy);
	}

	return ret;
}

static void mdio_adinx11x_bus_enable(const struct device *dev)
{
	const struct mdio_adinx11x_config *const cfg = dev->config;

	eth_adinx11x_lock(cfg->adin, K_FOREVER);
}

static void mdio_adinx11x_bus_disable(const struct device *dev)
{
	const struct mdio_adinx11x_config *const cfg = dev->config;

	eth_adinx11x_unlock(cfg->adin);
}

static const struct mdio_driver_api mdio_adinx11x_api = {
	.read = mdio_adinx11x_read,
	.write = mdio_adinx11x_write,
	.bus_enable = mdio_adinx11x_bus_enable,
	.bus_disable = mdio_adinx11x_bus_disable
};

#define ADINX11X_MDIO_INIT(n)							\
	static const struct mdio_adinx11x_config mdio_adinx11x_config_##n = {	\
		.adin = DEVICE_DT_GET(DT_INST_BUS(n)),				\
	};									\
	DEVICE_DT_INST_DEFINE(n, NULL, NULL,					\
			      NULL, &mdio_adinx11x_config_##n,			\
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,		\
			      &mdio_adinx11x_api);

DT_INST_FOREACH_STATUS_OKAY(ADINX11X_MDIO_INIT)
