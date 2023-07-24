/*
 * Copyright (c) 2023 PHOENIX CONTACT Electronics GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_adinx11x, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>

#if CONFIG_ETH_ADINX11X_SPI_CFG0
#include <zephyr/sys/crc.h>
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */
#include <string.h>
#include <errno.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/drivers/ethernet/eth_adinx11x.h>

#include "phy/phy_adinx11x_priv.h"
#include "eth_adinx11x_priv.h"

/* SPI Communication check retry delay */
#define ADINX11X_DEV_AWAIT_DELAY_POLL_US	100U
/* Number of retries SPI Communication check */
#define ADINX11X_DEV_AWAIT_RETRY_COUNT		200U

/* ADIN RESETC check retry delay */
#define ADINX11X_RESETC_AWAIT_DELAY_POLL_US	100U
/* Number of retries for ADIN RESETC check */
#define ADINX11X_RESETC_AWAIT_RETRY_COUNT	200U

/* Boot delay for clocks stabilisation (maximum 90ms) */
#define ADINX11X_HW_BOOT_DELAY_MS		100

/* MAC Address Rule and DA Filter multicast slot/idx */
#define ADINX11X_MULTICAST_ADDR_SLOT		0U
/* MAC Address Rule and DA Filter broadcast slot/idx */
#define ADINX11X_BROADCAST_ADDR_SLOT		1U
/* MAC Address Rule and DA Filter Port 1 slot/idx */
#define ADINX11X_UNICAST_P1_ADDR_SLOT		2U
/* MAC Address Rule and DA Filter Port 2 slot/idx */
#define ADINX11X_UNICAST_P2_ADDR_SLOT		3U

int eth_adinx11x_lock(const struct device *dev, k_timeout_t timeout)
{
	struct adinx11x_data *ctx = dev->data;

	return k_mutex_lock(&ctx->lock, timeout);
}

int eth_adinx11x_unlock(const struct device *dev)
{
	struct adinx11x_data *ctx = dev->data;

	return k_mutex_unlock(&ctx->lock);
}

int eth_adinx11x_reg_write(const struct device *dev, const uint16_t reg,
			   const uint32_t val)
{
	const struct adinx11x_config *cfg = dev->config;
	size_t header_size = ADINX11X_WRITE_HEADER_SIZE;
	size_t data_size = sizeof(uint32_t);
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	uint8_t buf[ADINX11X_REG_WRITE_BUF_SIZE_CRC] = { 0 };
#else
	uint8_t buf[ADINX11X_REG_WRITE_BUF_SIZE] = { 0 };
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	/* spi header */
	*(uint16_t *)buf = htons((ADINX11X_WRITE_TXN_CTRL | reg));
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	buf[2] = crc8_ccitt(0, buf, header_size);
	++header_size;
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	/* reg */
	*(uint32_t *)(buf + header_size) = htonl(val);
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	buf[header_size + data_size] = crc8_ccitt(0, &buf[header_size], data_size);
	++data_size;
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	const struct spi_buf spi_tx_buf = {
		.buf = buf,
		.len = header_size + data_size
	};
	const struct spi_buf_set tx = { .buffers = &spi_tx_buf, .count = 1U };

	return spi_write_dt(&cfg->spi, &tx);
}

int eth_adinx11x_reg_read(const struct device *dev, const uint16_t reg,
			  uint32_t *val)
{
	const struct adinx11x_config *cfg = dev->config;
	size_t header_len = ADINX11X_READ_HEADER_SIZE;
	size_t read_len = sizeof(uint32_t);
	int ret;
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	uint8_t rcv_crc;
	uint8_t comp_crc;
	uint8_t buf[ADINX11X_REG_READ_BUF_SIZE_CRC] = { 0 };
#else
	uint8_t buf[ADINX11X_REG_READ_BUF_SIZE] = { 0 };
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	/* spi header */
	*(uint16_t *)buf = htons((ADINX11X_READ_TXN_CTRL | reg));
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	buf[2] = crc8_ccitt(0, buf, ADINX11X_SPI_HEADER_SIZE);
	/* TA */
	buf[3] = 0U;
	++header_len;
	++read_len;
#else
	/* TA */
	buf[2] = 0U;
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	const struct spi_buf tx_buf = { .buf = buf, .len = header_len + read_len };
	const struct spi_buf rx_buf = { .buf = buf, .len = header_len + read_len };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1U };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1U };

	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
	if (ret < 0) {
		return ret;
	}

#if CONFIG_ETH_ADINX11X_SPI_CFG0
	comp_crc = crc8_ccitt(0, &buf[header_len], sizeof(uint32_t));
	rcv_crc = buf[header_len + sizeof(uint32_t)];

	if (rcv_crc != comp_crc) {
		/* invalid crc */
		return -EIO;
	}
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	*val = ntohl((*(uint32_t *)(&buf[header_len])));

	return ret;
}

static int adinx11x_read_fifo(const struct device *dev, const uint8_t port)
{
	const struct adinx11x_config *cfg = dev->config;
	struct adinx11x_data *ctx = dev->data;
	struct net_if *iface;
	struct net_pkt *pkt;
	size_t header_len = ADINX11X_READ_HEADER_SIZE;
	uint16_t fsize_reg = ((port == 0U) ? ADINX11X_P1_RX_FSIZE : ADINX11X_P2_RX_FSIZE);
	uint16_t rx_reg = ((port == 0U) ? ADINX11X_P1_RX : ADINX11X_P2_RX);
	uint32_t fsize;
	uint32_t fsize_real;
	uint32_t padding_len;
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	uint8_t cmd_buf[ADINX11X_FIFO_READ_CMD_BUF_SIZE_CRC] = { 0 };
#else
	uint8_t cmd_buf[ADINX11X_FIFO_READ_CMD_BUF_SIZE] = { 0 };
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */
	int ret;

	iface = ((struct adinx11x_port_data *)ctx->port[port]->data)->iface;

	/* get received frame size in bytes */
	ret = eth_adinx11x_reg_read(dev, fsize_reg, &fsize);
	if (ret < 0) {
		eth_stats_update_errors_rx(iface);
		LOG_ERR("Port %u failed to read RX FSIZE, %d", port, ret);
		return ret;
	}

	/* burst read must be in multiples of 4 */
	padding_len = ((fsize % 4) == 0) ? 0U : (ROUND_UP(fsize, 4U) - fsize);
	/* actual frame length is FSIZE - FRAME HEADER */
	fsize_real = fsize - ADINX11X_FRAME_HEADER_SIZE;

	/* spi header */
	*(uint16_t *)cmd_buf = htons((ADINX11X_READ_TXN_CTRL | rx_reg));
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	cmd_buf[2] = crc8_ccitt(0, cmd_buf, ADINX11X_SPI_HEADER_SIZE);
	/* TA */
	cmd_buf[3] = 0U;
	++header_len;
#else
	/* TA */
	cmd_buf[2] = 0U;
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	const struct spi_buf tx_buf = { .buf = cmd_buf, .len = sizeof(cmd_buf) };
	const struct spi_buf rx_buf[3] = {
		{.buf = NULL, .len = sizeof(cmd_buf) + ADINX11X_FRAME_HEADER_SIZE},
		{.buf = ctx->buf, .len = fsize_real},
		{.buf = NULL, .len = padding_len }
	};
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1U };
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ((padding_len == 0U) ? 2U : 3U)
	};

	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
	if (ret < 0) {
		eth_stats_update_errors_rx(iface);
		LOG_ERR("Port %u failed to read RX FIFO, %d", port, ret);
		return ret;
	}

	pkt = net_pkt_rx_alloc_with_buffer(iface, fsize_real, AF_UNSPEC, 0,
					   K_MSEC(CONFIG_ETH_ADINX11X_TIMEOUT));
	if (!pkt) {
		eth_stats_update_errors_rx(iface);
		LOG_ERR("Port %u failed to alloc frame RX buffer, %u bytes",
			port, fsize_real);
		return -ENOMEM;
	}

	ret = net_pkt_write(pkt, ctx->buf, fsize_real);
	if (ret < 0) {
		eth_stats_update_errors_rx(iface);
		net_pkt_unref(pkt);
		LOG_ERR("Port %u failed to fill RX frame, %d", port, ret);
		return ret;
	}

	ret = net_recv_data(iface, pkt);
	if (ret < 0) {
		eth_stats_update_errors_rx(iface);
		net_pkt_unref(pkt);
		LOG_ERR("Port %u failed to enqueue frame to RX queue, %d",
			port, ret);
		return ret;
	}

	eth_stats_update_bytes_rx(iface, fsize_real);
	eth_stats_update_pkts_rx(iface);

	return ret;
}

static inline void adinx11x_port_on_phyint(const struct device *dev)
{
	const struct adinx11x_port_config *cfg = dev->config;
	struct adinx11x_port_data *data = dev->data;
	struct phy_link_state state;

	if (phy_adinx11x_handle_phy_irq(cfg->phy, &state) < 0) {
		/* no change or error */
		return;
	}

	if (state.is_up) {
		net_eth_carrier_on(data->iface);
	} else {
		net_eth_carrier_off(data->iface);
	}
}

static void adinx11x_offload_thread(const struct device *dev)
{
	struct adinx11x_data *ctx = dev->data;
	const struct adinx11x_port_config *pcfg = dev->config;
	const struct adinx11x_config *adin_cfg = pcfg->adin->config;
	uint32_t status0;
	uint32_t status1;
	int ret;

	for (;;) {
		/* await INT */
		k_sem_take(&ctx->offload_sem, K_FOREVER);

		/* lock device */
		eth_adinx11x_lock(dev, K_FOREVER);

		/* disable interrupts */
		ret = eth_adinx11x_reg_write(dev, ADINX11X_IMASK0, UINT32_MAX);
		if (ret < 0) {
			goto continue_unlock;
		}
		ret = eth_adinx11x_reg_write(dev, ADINX11X_IMASK1, UINT32_MAX);
		if (ret < 0) {
			goto continue_unlock;
		}

		/* read interrupts */
		ret = eth_adinx11x_reg_read(dev, ADINX11X_STATUS0, &status0);
		if (ret < 0) {
			goto continue_unlock;
		}
		ret = eth_adinx11x_reg_read(dev, ADINX11X_STATUS1, &status1);
		if (ret < 0) {
			goto continue_unlock;
		}

#if CONFIG_ETH_ADINX11X_SPI_CFG0
		if (status0 & ADINX11X_STATUS1_SPI_ERR) {
			LOG_WRN("Detected TX SPI CRC error");
		}
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

		/* handle port 1 phy interrupts */
		if (status0 & ADINX11X_STATUS0_PHYINT) {
			adinx11x_port_on_phyint(ctx->port[0]);
		}

		/* handle port 2 phy interrupts */
		if ((status1 & ADINX11X_STATUS1_PHYINT) && (adin_cfg->id == ADIN2111_MAC)) {
			adinx11x_port_on_phyint(ctx->port[1]);
		}

		/* handle port 1 rx */
		if (status1 & ADINX11X_STATUS1_P1_RX_RDY) {
			do {
				ret = adinx11x_read_fifo(dev, 0U);
				if (ret < 0) {
					break;
				}

				ret = eth_adinx11x_reg_read(dev, ADINX11X_STATUS1, &status1);
				if (ret < 0) {
					goto continue_unlock;
				}
			} while (!!(status1 & ADINX11X_STATUS1_P1_RX_RDY));
		}

		/* handle port 2 rx */
		if ((status1 & ADINX11X_STATUS1_P2_RX_RDY) && (adin_cfg->id == ADIN2111_MAC)) {
			do {
				ret = adinx11x_read_fifo(dev, 1U);
				if (ret < 0) {
					break;
				}

				ret = eth_adinx11x_reg_read(dev, ADINX11X_STATUS1, &status1);
				if (ret < 0) {
					goto continue_unlock;
				}
			} while (!!(status1 & ADINX11X_STATUS1_P2_RX_RDY));
		}

continue_unlock:
		/* clear interrupts */
		ret = eth_adinx11x_reg_write(dev, ADINX11X_STATUS0, ADINX11X_STATUS0_CLEAR);
		if (ret < 0) {
			LOG_ERR("Failed to clear STATUS0, %d", ret);
		}
		ret = eth_adinx11x_reg_write(dev, ADINX11X_STATUS1, ADINX11X_STATUS1_CLEAR);
		if (ret < 0) {
			LOG_ERR("Failed to clear STATUS1, %d", ret);
		}
		/* enable interrupts */
		ret = eth_adinx11x_reg_write(dev, ADINX11X_IMASK0, ctx->imask0);
		if (ret < 0) {
			LOG_ERR("Failed to write IMASK0, %d", ret);
		}
		ret = eth_adinx11x_reg_write(dev, ADINX11X_IMASK1, ctx->imask1);
		if (ret < 0) {
			LOG_ERR("Failed to write IMASK1, %d", ret);
		}
		eth_adinx11x_unlock(dev);
	};
}

static void adinx11x_int_callback(const struct device *dev,
				  struct gpio_callback *cb,
				  uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct adinx11x_data *ctx = CONTAINER_OF(cb, struct adinx11x_data, gpio_int_callback);

	k_sem_give(&ctx->offload_sem);
}

static int adinx11x_read_tx_space(const struct device *dev, uint32_t *space)
{
	uint32_t val;
	int ret;

	ret = eth_adinx11x_reg_read(dev, ADINX11X_TX_SPACE, &val);
	if (ret < 0) {
		return ret;
	}

	/* tx space is a number of halfwords (16-bits), multiply by 2 for bytes */
	*space = val * 2;

	return ret;
}

static int adinx11x_port_send(const struct device *dev, struct net_pkt *pkt)
{
	const struct adinx11x_port_config *cfg = dev->config;
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct adinx11x_port_data *data = dev->data;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
	const struct device *adin = cfg->adin;
	struct adinx11x_data *ctx = cfg->adin->data;
	size_t pkt_len = net_pkt_get_len(pkt);
	size_t header_size = ADINX11X_WRITE_HEADER_SIZE;
	size_t padded_size;
	size_t burst_size;
	uint32_t tx_space;
	int ret;

	eth_adinx11x_lock(adin, K_FOREVER);

	/* query remaining tx fifo space */
	ret = adinx11x_read_tx_space(adin, &tx_space);
	if (ret < 0) {
		eth_stats_update_errors_tx(data->iface);
		LOG_ERR("Failed to read TX FIFO space, %d", ret);
		goto end_unlock;
	}

	/**
	 * verify that there is space for the frame
	 * (frame + 2b header + 2b size field)
	 */
	if (tx_space <
	   (pkt_len + ADINX11X_FRAME_HEADER_SIZE + ADINX11X_INTERNAL_HEADER_SIZE)) {
		/* tx buffer is full */
		eth_stats_update_errors_tx(data->iface);
		ret = -EBUSY;
		goto end_unlock;
	}

	/**
	 * pad to 64 bytes, otherwise MAC/PHY has to do it
	 * internally MAC adds 4 bytes for forward error correction
	 */
	if ((pkt_len + ADINX11X_TX_FIFO_BUFFER_MARGIN) < 64) {
		padded_size = pkt_len
			+ (64 - (pkt_len + ADINX11X_TX_FIFO_BUFFER_MARGIN))
			+ ADINX11X_FRAME_HEADER_SIZE;
	} else {
		padded_size = pkt_len + ADINX11X_FRAME_HEADER_SIZE;
	}

	/* prepare burst write (write data must be in multiples of 4) */
	burst_size = ROUND_UP(padded_size, 4);
	if ((burst_size + ADINX11X_WRITE_HEADER_SIZE) > CONFIG_ETH_ADINX11X_BUFFER_SIZE) {
		ret = -ENOMEM;
		eth_stats_update_errors_tx(data->iface);
		goto end_unlock;
	}

	/* prepare tx buffer */
	memset(ctx->buf, 0, burst_size + ADINX11X_WRITE_HEADER_SIZE);

	/* spi header */
	*(uint16_t *)ctx->buf = htons(ADINX11X_TXN_CTRL_TX_REG);
#if CONFIG_ETH_ADINX11X_SPI_CFG0
	ctx->buf[2] = crc8_ccitt(0, ctx->buf, header_size);
	++header_size;
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	/* frame header */
	*(uint16_t *)(ctx->buf + header_size) = htons(cfg->port_idx);

	/* read pkt into tx buffer */
	ret = net_pkt_read(pkt,
			   (ctx->buf + header_size + ADINX11X_FRAME_HEADER_SIZE),
			   pkt_len);
	if (ret < 0) {
		eth_stats_update_errors_tx(data->iface);
		LOG_ERR("Port %u failed to read PKT into TX buffer, %d",
			cfg->port_idx, ret);
		goto end_unlock;
	}

	/* write transmit size */
	ret = eth_adinx11x_reg_write(adin, ADINX11X_TX_FSIZE, padded_size);
	if (ret < 0) {
		eth_stats_update_errors_tx(data->iface);
		LOG_ERR("Port %u write FSIZE failed, %d", cfg->port_idx, ret);
		goto end_unlock;
	}

	/* write transaction */
	const struct spi_buf buf = {
		.buf = ctx->buf,
		.len = header_size + burst_size
	};
	const struct spi_buf_set tx = { .buffers = &buf, .count = 1U };

	ret = spi_write_dt(&((const struct adinx11x_config *) adin->config)->spi,
			   &tx);
	if (ret < 0) {
		eth_stats_update_errors_tx(data->iface);
		LOG_ERR("Port %u frame SPI write failed, %d", cfg->port_idx, ret);
		goto end_unlock;
	}

	eth_stats_update_bytes_tx(data->iface, pkt_len);
	eth_stats_update_pkts_tx(data->iface);

end_unlock:
	eth_adinx11x_unlock(adin);
	return ret;
}

static int adinx11x_config_sync(const struct device *dev)
{
	int ret;
	uint32_t val;

	ret = eth_adinx11x_reg_read(dev, ADINX11X_CONFIG0, &val);
	if (ret < 0) {
		return ret;
	}

	val |= ADINX11X_CONFIG0_SYNC;

	ret = eth_adinx11x_reg_write(dev, ADINX11X_CONFIG0, val);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int adinx11x_write_filter_address(const struct device *dev,
					 uint8_t *addr, uint8_t *mask,
					 uint32_t rules, uint16_t slot)
{
	uint16_t offset = slot * 2U;
	int ret;

	ret = eth_adinx11x_reg_write(dev, ADINX11X_ADDR_FILT_UPR + offset,
				     rules | sys_get_be16(&addr[0]));
	if (ret < 0) {
		return ret;
	}

	ret = eth_adinx11x_reg_write(dev, ADINX11X_ADDR_FILT_LWR  + offset,
				     sys_get_be32(&addr[2]));
	if (ret < 0) {
		return ret;
	}

	if (offset > 2U) {
		/* mask filter addresses are limited to 2 */
		return 0;
	}

	ret = eth_adinx11x_reg_write(dev, ADINX11X_ADDR_MSK_UPR + offset,
				     sys_get_be16(&mask[0]));
	if (ret < 0) {
		return ret;
	}

	ret = eth_adinx11x_reg_write(dev, ADINX11X_ADDR_MSK_LWR + offset,
				     sys_get_be32(&mask[2]));
	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int adinx11x_filter_multicast(const struct device *dev)
{
	const struct adinx11x_config *cfg = dev->config;
	uint8_t mm[6] = {BIT(0), 0U,  0U, 0U, 0U, 0U};
	uint32_t rules = ADINX11X_ADDR_APPLY2PORT1 |
			 (cfg->id == ADIN2111_MAC ? ADINX11X_ADDR_APPLY2PORT2 : 0) |
			 ADINX11X_ADDR_TO_HOST |
			 ADINX11X_ADDR_TO_OTHER_PORT;

	return adinx11x_write_filter_address(dev, mm, mm, rules,
					     ADINX11X_MULTICAST_ADDR_SLOT);
}

static int adinx11x_filter_broadcast(const struct device *dev)
{
	const struct adinx11x_config *cfg = dev->config;
	uint8_t mac[] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
	uint32_t rules = ADINX11X_ADDR_APPLY2PORT1 |
			 (cfg->id == ADIN2111_MAC ? ADINX11X_ADDR_APPLY2PORT2 : 0) |
			 ADINX11X_ADDR_TO_HOST |
			 ADINX11X_ADDR_TO_OTHER_PORT;

	return adinx11x_write_filter_address(dev, mac, mac, rules,
					     ADINX11X_BROADCAST_ADDR_SLOT);
}

static int adinx11x_filter_unicast(const struct device *dev, uint8_t *addr,
				   uint8_t port_idx)
{
	uint32_t rules = (port_idx == 0 ? ADINX11X_ADDR_APPLY2PORT1
					: ADINX11X_ADDR_APPLY2PORT2)
			 | ADINX11X_ADDR_TO_HOST;
	uint16_t slot = (port_idx == 0 ? ADINX11X_UNICAST_P1_ADDR_SLOT
				       : ADINX11X_UNICAST_P2_ADDR_SLOT);

	return adinx11x_write_filter_address(dev, addr, NULL, rules, slot);
}

static void adinx11x_port_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct adinx11x_port_config *cfg = dev->config;
	struct adinx11x_port_data *data = dev->data;
	const struct device *adin = cfg->adin;
	struct adinx11x_data *ctx = adin->data;
	int ret;

	if (!device_is_ready(adin)) {
		LOG_ERR("ADIN %s is not ready, can't init port %u iface",
			cfg->adin->name, cfg->port_idx);
		return;
	}

	if (!device_is_ready(cfg->phy)) {
		LOG_ERR("PHY %u is not ready, can't init port %u iface",
			cfg->phy_addr, cfg->port_idx);
		return;
	}

	ctx->port[cfg->port_idx] = dev;
	data->iface = iface;

	ret = adinx11x_filter_unicast(adin, data->mac_addr, cfg->port_idx);
	if (ret < 0) {
		LOG_ERR("Port %u, failed to set unicast filter, %d",
			cfg->port_idx, ret);
		return;
	}
	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	ethernet_init(iface);
	net_if_carrier_off(iface);

	--ctx->ifaces_left_to_init;

	/* if all ports are initialized */
	if (ctx->ifaces_left_to_init == 0U) {
		/* setup rx filters */
		ret = adinx11x_filter_multicast(adin);
		if (ret < 0) {
			LOG_ERR("Couldn't set multicast filter, %d", ret);
			return;
		}
		ret = adinx11x_filter_broadcast(adin);
		if (ret < 0) {
			LOG_ERR("Couldn't set broadcast filter, %d", ret);
			return;
		}

		/* sync */
		ret = adinx11x_config_sync(adin);
		if (ret < 0) {
			LOG_ERR("Failed to write CONFIG0 SYNC, %d", ret);
			return;
		}

		/* all ifaces are done, start INT processing */
		k_thread_create(&ctx->rx_thread, ctx->rx_thread_stack,
				CONFIG_ETH_ADINX11X_IRQ_THREAD_STACK_SIZE,
				(k_thread_entry_t)adinx11x_offload_thread,
				(void *)adin, NULL, NULL,
				CONFIG_ETH_ADINX11X_IRQ_THREAD_PRIO,
				K_ESSENTIAL, K_NO_WAIT);
		k_thread_name_set(&ctx->rx_thread, "eth_adinx11x_offload");
	}
}

static enum ethernet_hw_caps adinx11x_port_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE_T
#if defined(CONFIG_NET_LLDP)
		| ETHERNET_LLDP
#endif
		;
}

static int adinx11x_port_set_config(const struct device *dev,
				    enum ethernet_config_type type,
				    const struct ethernet_config *config)
{
	const struct adinx11x_port_config *cfg = dev->config;
	struct adinx11x_port_data *data = dev->data;
	const struct device *adin = cfg->adin;
	int ret = -ENOTSUP;

	if (type == ETHERNET_CONFIG_TYPE_MAC_ADDRESS) {
		ret = adinx11x_filter_unicast(adin, data->mac_addr, cfg->port_idx);
		if (ret < 0) {
			return ret;
		}

		memcpy(data->mac_addr, config->mac_address.addr, sizeof(data->mac_addr));

		net_if_set_link_addr(data->iface, data->mac_addr,
				     sizeof(data->mac_addr),
				     NET_LINK_ETHERNET);
	}

	return ret;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *adinx11x_port_get_stats(const struct device *dev)
{
	struct adinx11x_port_data *data = dev->data;

	return &data->stats;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static int adinx11x_check_spi(const struct device *dev)
{
	uint32_t count;
	uint32_t val;
	int ret;

	/* check SPI communication by reading PHYID */
	for (count = 0U; count < ADINX11X_DEV_AWAIT_RETRY_COUNT; ++count) {
		ret = eth_adinx11x_reg_read(dev, ADINX11X_PHYID, &val);
		LOG_INF("SPI READ ID: %x\n", val);
		if (ret >= 0) {
			if (val == ADIN2111_PHYID_RST_VAL || val == ADIN1110_PHYID_RST_VAL) {
				break;
			}
			ret = -ETIMEDOUT;
		}
		k_sleep(K_USEC(ADINX11X_DEV_AWAIT_DELAY_POLL_US));
	}

	return ret;
}

static int adinx11x_await_device(const struct device *dev)
{
	uint32_t count;
	uint32_t val;
	int ret;

	/* await reset complete (RESETC) and clear it */
	for (count = 0U; count < ADINX11X_RESETC_AWAIT_RETRY_COUNT; ++count) {
		ret = eth_adinx11x_reg_read(dev, ADINX11X_STATUS0, &val);
		if (ret >= 0) {
			/* if out of reset */
			if (val & ADINX11X_STATUS0_RESETC) {
				/* clear RESETC */
				ret = eth_adinx11x_reg_write(dev, ADINX11X_STATUS0,
							 ADINX11X_STATUS0_RESETC);
				if (ret >= 0) {
					break;
				}
			}
			ret = -ETIMEDOUT;
		}
		k_sleep(K_USEC(ADINX11X_RESETC_AWAIT_DELAY_POLL_US));
	}

	return ret;
}

static int adinx11x_init(const struct device *dev)
{
	const struct adinx11x_config *cfg = dev->config;
	struct adinx11x_data *ctx = dev->data;
	int ret;
	uint32_t val;

	LOG_INF("DEVICE ID, %d", cfg->id);

	__ASSERT(cfg->spi.config.frequency <= ADINX11X_SPI_MAX_FREQUENCY,
		 "SPI frequency exceeds supported maximum\n");

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus %s not ready", cfg->spi.bus->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->interrupt)) {
		LOG_ERR("Interrupt GPIO device %s is not ready",
			cfg->interrupt.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt GPIO, %d", ret);
		return ret;
	}

	if (cfg->reset.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("Reset GPIO device %s is not ready",
			cfg->reset.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure reset GPIO, %d", ret);
			return ret;
		}

		/* perform hard reset */
		/* assert pin low for 16 µs (10 µs min) */
		gpio_pin_set_dt(&cfg->reset, 1);
		k_busy_wait(16U);
		/* deassert and wait for 90 ms (max) for clocks stabilisation */
		gpio_pin_set_dt(&cfg->reset, 0);
		k_msleep(ADINX11X_HW_BOOT_DELAY_MS);
	}

	gpio_init_callback(&(ctx->gpio_int_callback),
			   adinx11x_int_callback,
			   BIT(cfg->interrupt.pin));

	ret = gpio_add_callback(cfg->interrupt.port, &ctx->gpio_int_callback);
	if (ret < 0) {
		LOG_ERR("Failed to add INT callback, %d", ret);
		return ret;
	}

	ret = adinx11x_check_spi(dev);
	if (ret < 0) {
		LOG_ERR("Failed to communicate over SPI, %d", ret);
		return ret;
	}

	/* perform MACPHY soft reset */
	ret = eth_adinx11x_reg_write(dev, ADINX11X_RESET, ADINX11X_RESET_SWRESET);
	if (ret < 0) {
		LOG_ERR("MACPHY software reset failed, %d", ret);
		return ret;
	}

	ret = adinx11x_await_device(dev);
	if (ret < 0) {
		LOG_ERR("ADIN did't come out of the reset, %d", ret);
		return ret;
	}

	/* CONFIG 0 */
	/* disable Frame Check Sequence validation on the host */
	/* if that is enabled, then CONFIG_ETH_ADINX11X_SPI_CFG0 must be off */
	ret = eth_adinx11x_reg_read(dev, ADINX11X_CONFIG0, &val);
	if (ret < 0) {
		LOG_ERR("Failed to read CONFIG0, %d", ret);
		return ret;
	}

	/* RXCTE must be disabled for Generic SPI */
	val &= ~ADINX11X_CONFIG0_RXCTE;
	val &= ~(ADINX11X_CONFIG0_TXCTE | ADINX11X_CONFIG0_TXFCSVE);

	ret = eth_adinx11x_reg_write(dev, ADINX11X_CONFIG0, val);
	if (ret < 0) {
		LOG_ERR("Failed to write CONFIG0, %d", ret);
		return ret;
	}

	/* CONFIG 2 */
	ret = eth_adinx11x_reg_read(dev, ADINX11X_CONFIG2, &val);
	if (ret < 0) {
		LOG_ERR("Failed to read CONFIG2, %d", ret);
		return ret;
	}

#if CONFIG_ETH_ADINX11X_SPI_CFG0
	val |= ADINX11X_CONFIG2_CRC_APPEND;
#else
	val &= ~ADINX11X_CONFIG2_CRC_APPEND;
#endif /* CONFIG_ETH_ADINX11X_SPI_CFG0 */

	/* configure forwarding of frames with unknown destination address */
	/* to the other port. This forwarding is done in hardware.         */
	/* The setting will take effect after the ports                    */
	/* are out of software powerdown.                                  */
	val |= (ADINX11X_CONFIG2_PORT_CUT_THRU_EN |
		(cfg->id == ADIN2111_MAC ? ADINX11X_CONFIG2_P1_FWD_UNK2P2 : 0) |
		(cfg->id == ADIN2111_MAC ?ADINX11X_CONFIG2_P2_FWD_UNK2P1 : 0));

	ret = eth_adinx11x_reg_write(dev, ADINX11X_CONFIG2, val);
	if (ret < 0) {
		LOG_ERR("Failed to write CONFIG2, %d", ret);
		return ret;
	}

	/* configure interrupt masks */
	ctx->imask0 = ~ADINX11X_IMASK0_PHYINTM;
	ctx->imask1 = ~(ADINX11X_IMASK1_TX_RDY_MASK |
			ADINX11X_IMASK1_P1_RX_RDY_MASK |
			ADINX11X_IMASK1_SPI_ERR_MASK |
			(cfg->id == ADIN2111_MAC ? ADINX11X_IMASK1_P2_RX_RDY_MASK : 0) |
			(cfg->id == ADIN2111_MAC ? ADINX11X_IMASK1_P2_PHYINT_MASK : 0));

	/* enable interrupts */
	ret = eth_adinx11x_reg_write(dev, ADINX11X_IMASK0, ctx->imask0);
	if (ret < 0) {
		LOG_ERR("Failed to write IMASK0, %d", ret);
		return ret;
	}
	ret = eth_adinx11x_reg_write(dev, ADINX11X_IMASK1, ctx->imask1);
	if (ret < 0) {
		LOG_ERR("Failed to write IMASK1, %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt,
						GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to enable INT, %d", ret);
		return ret;
	}

	return ret;
}

static const struct ethernet_api adinx11x_port_api = {
	.iface_api.init = adinx11x_port_iface_init,
	.get_capabilities = adinx11x_port_get_capabilities,
	.set_config = adinx11x_port_set_config,
	.send = adinx11x_port_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = adinx11x_port_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

#define ADINX11X_STR(x)		#x
#define ADINX11X_XSTR(x)	ADINX11X_STR(x)

#define ADINX11X_MDIO_PHY_BY_ADDR(adin_n, phy_addr)						\
	DEVICE_DT_GET(DT_CHILD(DT_INST_CHILD(adin_n, mdio), phy_##phy_addr))

#define ADINX11X_PORT_MAC(adin_n, port_n)							\
	DT_PROP(DT_CHILD(DT_DRV_INST(adin_n), port##port_n), local_mac_address)

#define ADINX11X_PORT_DEVICE_INIT_INSTANCE(parent_n, port_n, phy_n)				\
	static struct adinx11x_port_data adinx11x_port_data_##port_n = {			\
		.mac_addr = ADINX11X_PORT_MAC(parent_n, phy_n),					\
	};											\
	static const struct adinx11x_port_config adinx11x_port_config_##port_n = {		\
		.adin = DEVICE_DT_INST_GET(parent_n),						\
		.phy = ADINX11X_MDIO_PHY_BY_ADDR(parent_n, phy_n),				\
		.port_idx = port_n,								\
		.phy_addr = phy_n,								\
	};											\
	NET_DEVICE_INIT_INSTANCE(adinx11x_port_##port_n, "port_" ADINX11X_XSTR(port_n), port_n,	\
				 NULL, NULL, &adinx11x_port_data_##port_n,			\
				 &adinx11x_port_config_##port_n, CONFIG_ETH_INIT_PRIORITY,	\
				 &adinx11x_port_api, ETHERNET_L2,				\
				 NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

#define ADINX11X_SPI_OPERATION ((uint16_t)(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)))

#define ADINX11X_MAC_INITIALIZE(inst, dev_id, ifaces)						\
	static uint8_t __aligned(4) adinx11x_buffer_##inst[CONFIG_ETH_ADINX11X_BUFFER_SIZE];	\
	static const struct adinx11x_config adinx11x_config_##inst = {				\
		.id = dev_id,									\
		.spi = SPI_DT_SPEC_INST_GET(inst, ADINX11X_SPI_OPERATION, 1),			\
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),				\
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, { 0 }),			\
	};											\
	static struct adinx11x_data adinx11x_data_##inst = {					\
		.ifaces_left_to_init = ifaces,							\
		.port = {},									\
		.offload_sem = Z_SEM_INITIALIZER(adinx11x_data_##inst.offload_sem, 0, 1),	\
		.lock = Z_MUTEX_INITIALIZER(adinx11x_data_##inst.lock),				\
		.buf = adinx11x_buffer_##inst,							\
	};											\
	/* adin */										\
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), adinx11x_init, NULL,				\
			 &adinx11x_data_##inst, &adinx11x_config_##inst,			\
			 POST_KERNEL, CONFIG_ETH_ADINX11X_INIT_PRIORITY,			\
			 NULL);

#define ADIN1110_MAC_INITIALIZE(inst) ADINX11X_MAC_INITIALIZE(inst, ADIN1110_MAC, 1)		\
	/* ports */										\
	ADINX11X_PORT_DEVICE_INIT_INSTANCE(inst, 0, 1)

#define ADIN2111_MAC_INITIALIZE(inst) ADINX11X_MAC_INITIALIZE(inst, ADIN2111_MAC, 2)		\
	/* ports */										\
	ADINX11X_PORT_DEVICE_INIT_INSTANCE(inst, 0, 1)						\
	ADINX11X_PORT_DEVICE_INIT_INSTANCE(inst, 1, 2)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adin1110
DT_INST_FOREACH_STATUS_OKAY(ADIN1110_MAC_INITIALIZE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adin2111
DT_INST_FOREACH_STATUS_OKAY(ADIN2111_MAC_INITIALIZE)
