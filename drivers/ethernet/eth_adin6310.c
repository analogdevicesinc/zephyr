/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_adin6310, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "zephyr/sys/util.h"

#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>

#include "eth_adin6310.h"
#include "SMP_stack_api.h"
#include "SES_port_api.h"
#include "SES_codes.h"
#include "SES_switch.h"
#include "SES_event.h"
#include "SES_frame_api.h"
#include "SES_interface_management.h"

const SES_portInit_t initializePorts_p[] = {
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 0, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 1, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1300, {true, 0, 6, SES_phySpeed1000, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1300, {true, 0, 5, SES_phySpeed1000, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 2, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 3, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}}
};

void* SES_PORT_CreateSemaphore(int initCount, int maxCount)
{
	struct k_sem *sem = NULL;

	sem = k_calloc(1, sizeof(*sem));
	if (!sem)
		return NULL;

	k_sem_init(sem, initCount, maxCount);

	return sem;
}

int32_t SES_PORT_WaitSemaphore(int* semaphore_p, int timeout)
{
	struct k_sem *sem = (struct k_sem *)semaphore_p;
	int ret;

	ret = k_sem_take(sem, K_MSEC(timeout));
	if (ret == -ETIMEDOUT)
		return SES_PORT_TIMEOUT;

	return SES_PORT_OK;
}

int32_t SES_PORT_SignalSemaphore(int* semaphore_p)
{
	struct k_sem *sem = (struct k_sem *)semaphore_p;

	if (sem == NULL)
		return SES_PORT_INVALID_PARAM;

	k_sem_give(sem);

	return 1;
}

int32_t SES_PORT_DeleteSemaphore(int* semaphore_p)
{
	struct k_sem *sem = (struct k_sem *)semaphore_p;

	k_free(sem);

	return 1;
}

int32_t SES_PORT_Malloc(void **buf_pp, int size)
{
	if (!buf_pp)
		return SES_PORT_BUF_ERR;
	
	*buf_pp = k_malloc(size);

	return 0;
}

int32_t SES_PORT_Free(void *buf_p)
{
	if (!buf_p)
		return SES_PORT_BUF_ERR;

	k_free(buf_p);

	return 0;
}

static int adin6310_spi_init(void *param_p,
                      SES_PORT_intfType_t *intfType_p,
                      uint8_t *srcMac_p)
{
	*intfType_p = SES_PORT_spiInterface;

	return (int)param_p;
}

static int adin6310_spi_remove(int intfHandle)
{
	return 0;
}

static int adin6310_spi_write(int intfHandle, int size, void *data_p)
{
	const struct spi_dt_spec dev_spi = SPI_DT_SPEC_GET(DT_NODELABEL(adin6310),
							   SPI_WORD_SET(8) |
							   SPI_OP_MODE_MASTER |
							   SPI_TRANSFER_MSB |
							   SPI_MODE_GET(0), 0);
	struct adin6310_data *priv = (struct adin6310_data *)intfHandle;
	struct spi_buf tx_buf;
	struct spi_buf_set tx;
	int ret;

	k_mutex_lock(&priv->lock, K_FOREVER);
	tx_buf.len = size + 1;

	tx_buf.buf = priv->tx_buf;
	tx.buffers = &tx_buf;
	tx.count = 1;

	priv->tx_buf[0] = ADIN6310_SPI_WR_HEADER;
	memcpy(&priv->tx_buf[1], data_p, size);
	ret = spi_write_dt(&dev_spi, &tx);
	
	SES_PORT_Free(data_p);
	k_mutex_unlock(&priv->lock);

	return ret;
}

static int adin6310_spi_read(struct adin6310_data *priv, uint8_t *data, uint32_t len)
{
	int ret = 0;
	struct spi_buf rx_buf;
	struct spi_buf tx_buf;
	struct spi_buf_set rx_buf_set;
	struct spi_buf_set tx_buf_set;
	const struct spi_dt_spec dev_spi = SPI_DT_SPEC_GET(DT_NODELABEL(adin6310),
							   SPI_WORD_SET(8) |
							   SPI_OP_MODE_MASTER |
							   SPI_TRANSFER_MSB |
							   SPI_MODE_GET(0), 0);

	k_mutex_lock(&priv->lock, K_FOREVER);

	memset(priv->tx_buf, 0, len + 1);
	priv->tx_buf[0] = ADIN6310_SPI_RD_HEADER;

	rx_buf.len = len + 1;
	rx_buf.buf = data;
	tx_buf.len = len + 1;
	tx_buf.buf = priv->tx_buf;

	rx_buf_set.buffers = &rx_buf;
	rx_buf_set.count = 1;
	tx_buf_set.buffers = &tx_buf;
	tx_buf_set.count = 1;

	ret = spi_transceive_dt(&dev_spi, &tx_buf_set, &rx_buf_set);

	k_mutex_unlock(&priv->lock);

	return ret;
}

static int adin6310_read_message(struct adin6310_data *priv, int tbl_index)
{
	uint32_t frame_type;
	uint32_t padded_len;
	uint32_t rx_len;
	int ret;

	ret = adin6310_spi_read(priv, priv->rx_buf, 4);
	if (ret)
		return ret;

	rx_len = ((priv->rx_buf[2] & 0xF) << 8) | priv->rx_buf[1];
	frame_type = (priv->rx_buf[2] & 0xF0) >> 4;

	padded_len = rx_len + 0x3;
	padded_len &= ~GENMASK(1, 0);

	ret = adin6310_spi_read(priv, priv->rx_buf, padded_len);
	if (ret)
		return ret;

	return SES_ReceiveMessage(priv, SES_PORT_spiInterface, rx_len,
				  (void*)&priv->rx_buf[1], -1, 0, NULL);
}

static void adin6310_msg_recv(void *p1, void *p2, void *p3)
{
	struct adin6310_data *priv = p1;
	k_thread_custom_data_set(p1);

	while (1) {
		k_sem_take(&priv->reader_thread_sem, K_FOREVER);
		adin6310_read_message(p1, 0);
	};
}

static void adin6310_int_rdy(const struct device *port,
			     struct gpio_callback *cb,
		      	     gpio_port_pins_t pins)
{
	struct adin6310_data *priv = CONTAINER_OF(cb, struct adin6310_data, gpio_int_callback);

	k_sem_give(&priv->reader_thread_sem);
}

static void adin6310_rx_callback(int32_t frame_len, uint8_t *frame, SES_frameAttributes_t *attrs)
{
	struct net_pkt *pkt;
	struct net_if *iface;
	struct adin6310_data *data = k_thread_custom_data_get();
	struct adin6310_port_data *port = data->port_data[attrs->port];
	int ret;

	iface = port->iface;
	if (!net_if_is_carrier_ok(iface))
		net_eth_carrier_on(iface);

	pkt = net_pkt_rx_alloc_with_buffer(iface, frame_len, AF_UNSPEC, 0, K_FOREVER);
	if (!pkt) {
		LOG_ERR("Error on net_pkt alloc\n");
		return;
	}

	ret = net_pkt_write(pkt, frame, frame_len);
	if (ret) {
		net_pkt_unref(pkt);
		LOG_ERR("Port %u failed to fill RX frame", attrs->port);
		return;
	}

	ret = net_recv_data(iface, pkt);
	if (ret) {
		net_pkt_unref(pkt);
		LOG_ERR("Port %u failed to enqueue frame to RX queue",
			attrs->port);
		return;
	}
}

static int adin6310_port_send(const struct device *dev, struct net_pkt *pkt)
{
	struct adin6310_port_data *data = dev->data;
	struct adin6310_data *priv = data->net_device;
	size_t pkt_len = net_pkt_get_len(pkt);
	int ret;

	SES_transmitFrameData_t txData_p = {
		.frameType = SES_standardFrame,
		.byteCount = pkt_len,
		.cb_p = 0,
		.cbParam_p = 0,
		.pad = 0,
		.ses = {
			.generateFcs = 1,
			.xmitPriority = 0,
			.egressPortMap = BIT(data->id),
			.transformId = SES_NO_TRANSFORM,
			.attributeRequest = 0,
		}
	};

	if (!net_if_is_carrier_ok(pkt->iface))
		net_if_carrier_on(pkt->iface);

	txData_p.data_p = priv->frame_buf;
	ret = net_pkt_read(pkt, priv->frame_buf, pkt_len);
	if (ret < 0) {
		LOG_ERR("Port %d failed to read PKT into TX buffer", data->id);
		return ret;
	}

	ret = SES_XmitFrame(&txData_p);
	if (ret != SES_OK) {
		LOG_ERR("Port %d failed to send packet", data->id);
		return -1;
	}

	return 0;
}

static int adin6310_add_filter(uint8_t mac_addr[6])
{
	return SES_RxSesFramesByMac(mac_addr, SES_REQUEST_PORT_ATTRIBUTE, adin6310_rx_callback);
}

static enum ethernet_hw_caps adin6310_port_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_1000BASE_T | ETHERNET_LINK_100BASE_T | ETHERNET_LINK_10BASE_T;
}

static int adin6310_port_set_config(const struct device *dev,
				    enum ethernet_config_type type,
				    const struct ethernet_config *config)
{
	int ret = -ENOTSUP;

	if (type == ETHERNET_CONFIG_TYPE_MAC_ADDRESS) {
		ret = adin6310_add_filter((uint8_t *)&config->mac_address.addr[0]);
		if (ret)
			return ret;
	}

	return ret;
}

static void adin6310_port_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct adin6310_port_data *data = dev->data;
	const struct adin6310_port_config *config = dev->config;
	struct adin6310_data *adin_priv = config->net_device;
	int ret;

	if (!device_is_ready(config->adin)) {
		LOG_ERR("ADIN is not ready, can't init port %u iface",
			config->id);
		return;
	}

	data->adin = config->adin;
	data->id = config->id;
	data->net_device = config->net_device;
	data->name = config->name;
	data->initialized = false;
	data->cpu_port = config->cpu_port;
	memcpy(data->mac_addr, config->mac_addr, 6);

	net_if_set_link_addr(iface, data->mac_addr, 6, NET_LINK_ETHERNET);
	ethernet_init(iface);
	net_if_carrier_off(iface);

	ret = SES_RxSesFramesByMac(data->mac_addr, SES_REQUEST_PORT_ATTRIBUTE, adin6310_rx_callback);
	if (ret) {
		LOG_ERR("Error (%d) configuring MAC filter for port %d", ret, config->id);
		return;
	}

	data->initialized = true;
	data->iface = iface;
	adin_priv->port_data[data->id] = data;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *adin6310_port_get_stats(const struct device *dev)
{
	struct adin6310_data *data = dev->data;

	return NULL;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static int adin6310_set_broadcast_route(const struct device *dev)
{
	uint8_t lookupPriority = 1;
	uint8_t macAddr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t macMask[6] = { 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };
	int16_t vlanId = SES_DYNTBL_NO_VLAN;
	uint16_t vlanMask = 0xFFF;
	uint8_t sourceEntry = 0;
	uint8_t override = 0;
	uint8_t sourceOverride = 0;
	uint8_t hsrOrPrpSupervisory = 0;
	SES_dynTblLookup_t lookupType = SES_dynamicTblLookupBasic;
	uint8_t portMap = 0x3F;
	uint8_t sendToAE = 1;
	uint8_t danp = 0;
	uint8_t cutThrough = 0;
	uint8_t syntTimestamp = 0;
	uint8_t localTimestamp = 0;
	SES_dynTblSeqMgmt_t sequenceMgmt = dynamicTblNoSequenceOp;
	int rxSequenceSet = 0;
	int transmitFilter = SES_IGNORE_FIELD;
	int receiveFilter = SES_IGNORE_FIELD;
	int index;
	int ret;

	ret = SES_RxSesFramesByMac(macAddr, SES_REQUEST_PORT_ATTRIBUTE, adin6310_rx_callback);
	if (ret) {
		LOG_ERR("RX MAC config error\n");
		return ret;
	}

	return SES_AddStaticTableEntryEx(lookupPriority,
					macAddr,
					macMask,
					vlanId,
					vlanMask,
					sourceEntry,
					override,
					sourceOverride,
					hsrOrPrpSupervisory,
					lookupType,
					portMap,
					sendToAE,
					danp,
					cutThrough,
					syntTimestamp,
					localTimestamp,
					sequenceMgmt,
					rxSequenceSet,
					transmitFilter,
					receiveFilter,
					&index);
}

static int adin6310_get_cpu_port(const struct device *dev,
				 const struct adin6310_port_config **port)
{
	const struct adin6310_config *config = dev->config;

	for (int i = 0; i < ADIN6310_NUM_PORTS; i++) {
		if (config->port_config[i].cpu_port) {
			*port = &config->port_config[i];

			return 0;
		}
	}

	return -ENODEV;
}

static int adin6310_init(const struct device *dev)
{
        const struct adin6310_config *cfg = dev->config;
	const struct adin6310_port_config *cpu_port;
	struct adin6310_data *priv = dev->data;
	sesID_t dev_id;
	int iface;
	int ret;

	SES_driverFunctions_t comm_callbacks = {
		.init_p = adin6310_spi_init,
		.release_p = adin6310_spi_remove,
		.sendMessage_p = adin6310_spi_write,
	};

        if (!spi_is_ready_dt(&cfg->spi)) {
                LOG_ERR("SPI bus %s not ready", cfg->spi.bus->name);
		return -ENODEV;
        }

        if (!gpio_is_ready_dt(&cfg->interrupt)) {
		LOG_ERR("Interrupt GPIO device %s is not ready",
			cfg->interrupt.port->name);
		return -ENODEV;
        }

        if (!gpio_is_ready_dt(&cfg->reset)) {
		LOG_ERR("Interrupt GPIO device %s is not ready",
			cfg->reset.port->name);
		return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
        if (ret) {
        	LOG_ERR("Failed to configure interrupt GPIO, %d", ret);
		return ret;
        }

        ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
        if (ret) {
        	LOG_ERR("Failed to configure reset GPIO, %d", ret);
		return ret;
        }

        gpio_pin_set_dt(&cfg->reset, 1);
        k_busy_wait(100);
        gpio_pin_set_dt(&cfg->reset, 0);
        k_busy_wait(100);

	k_mutex_init(&priv->lock);
	k_sem_init(&priv->reader_thread_sem, 0, 1);
	priv->offload_thread_id = k_thread_create(&priv->rx_thread, priv->rx_thread_stack,
						  K_KERNEL_STACK_SIZEOF(priv->rx_thread_stack),
						  adin6310_msg_recv, priv, NULL, NULL,
						  CONFIG_ETH_ADIN6310_IRQ_THREAD_PRIO,
						  K_ESSENTIAL, K_FOREVER);
	k_thread_name_set(priv->offload_thread_id, "adin6310_rx_offload");

	gpio_init_callback(&priv->gpio_int_callback, adin6310_int_rdy, BIT(cfg->interrupt.pin));
	gpio_add_callback(cfg->interrupt.port, &priv->gpio_int_callback);

	k_thread_start(priv->offload_thread_id);

	priv->interrupt = &cfg->interrupt;
	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret)
		return ret;

	ret = adin6310_get_cpu_port(dev, &cpu_port);
	if (ret)
		return ret;

	ret = SES_Init();
	if (ret) {
		LOG_ERR("SES_Init error %d\n", ret);
		return ret;
	}

	ret = SES_AddHwInterface(priv, &comm_callbacks, &iface);
	if (ret) {
		LOG_ERR("SES_AddHwInterface error %d\n", ret);
		return ret;
	}

	ret = SES_AddDevice(iface, (uint8_t *)cpu_port->mac_addr, &dev_id);
	if (ret) {
		LOG_ERR("SES_AddDevice error %d\n", ret);
		return ret;
	}

	ret = SES_MX_InitializePorts(dev_id, ADIN6310_NUM_PORTS, initializePorts_p);
	if (ret) {
		LOG_ERR("SES_MX_InitializePorts error %d\n", ret);
		return ret;
	}

	return adin6310_set_broadcast_route(dev);
}

static const struct ethernet_api adin6310_port_api = {
	.iface_api.init = adin6310_port_iface_init,
	.get_capabilities = adin6310_port_get_capabilities,
	.set_config = adin6310_port_set_config,
	.send = adin6310_port_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = adin6310_port_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

#define ADIN6310_SPI_OPERATION ((uint16_t)(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)))

#define ADIN6310_PRIV_PORT_CONFIG(parent, inst, idx)	\
	adin6310_port_config_##parent##_##inst

#define ADIN6310_PORT_INIT(parent, inst, idx)								\
	static struct adin6310_port_data adin6310_port_data_##parent##_##inst = {0};			\
	static const struct adin6310_port_config adin6310_port_config_##parent##_##inst = {		\
		.adin = DEVICE_DT_INST_GET(parent),							\
		.id = idx,										\
		.net_device = &adin6310_data_##parent,							\
		.name = "port_" #idx,									\
		.mac_addr = DT_PROP(DT_CHILD(DT_DRV_INST(parent), inst), local_mac_address),		\
		.cpu_port = DT_PROP(DT_CHILD(DT_DRV_INST(parent), inst), cpu_port),			\
	}; 												\
	NET_DEVICE_INIT_INSTANCE(parent##_port_##idx, "port_" #idx, idx,				\
				 NULL, NULL, &adin6310_port_data_##parent##_##inst,			\
				 &adin6310_port_config_##parent##_##inst, CONFIG_ETH_INIT_PRIORITY,	\
				 &adin6310_port_api, ETHERNET_L2,					\
				 NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

#define ADIN6310_DT_INST(inst)									\
	static struct adin6310_data adin6310_data_##inst = {0};					\
	static const struct adin6310_config adin6310_config_##inst;				\
	DEVICE_DT_DEFINE	(DT_DRV_INST(inst), adin6310_init, NULL,			\
				&adin6310_data_##inst, &adin6310_config_##inst,			\
				POST_KERNEL, CONFIG_ETH_INIT_PRIORITY,				\
				NULL);								\
	ADIN6310_PORT_INIT(inst, port0, 0)							\
	ADIN6310_PORT_INIT(inst, port1, 1)							\
	ADIN6310_PORT_INIT(inst, port2, 2)							\
	ADIN6310_PORT_INIT(inst, port3, 3)							\
	ADIN6310_PORT_INIT(inst, port4, 4)							\
	ADIN6310_PORT_INIT(inst, port5, 5)							\
	static const struct adin6310_config adin6310_config_##inst = {				\
		.id = ADIN6310,									\
		.spi = SPI_DT_SPEC_INST_GET(inst, ADIN6310_SPI_OPERATION, 1),			\
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),				\
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),			\
		.port_config = {								\
			ADIN6310_PRIV_PORT_CONFIG(inst, port0, 0),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port1, 1),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port2, 2),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port3, 3),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port4, 4),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port5, 5),				\
		}										\
	};
	

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adin6310 
DT_INST_FOREACH_STATUS_OKAY(ADIN6310_DT_INST)
