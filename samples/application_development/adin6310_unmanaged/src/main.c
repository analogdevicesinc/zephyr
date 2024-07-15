/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_adin6310, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/net/net_config.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "SMP_stack_api.h"
#include "SES_port_api.h"
#include "SES_switch.h"
#include "SES_frame_api.h"
#include "SES_interface_management.h"
#include "zephyr/sys/util.h"

#define ADIN6310_SPI_WR_HEADER	0x40
#define ADIN6310_SPI_RD_HEADER	0x80

static struct k_sem semaphores[50];
K_THREAD_STACK_DEFINE(stack_area, 2000);
K_SEM_DEFINE(reader_thread_sem, 0, 1);
K_MUTEX_DEFINE(spi_mutex);

void* SES_PORT_CreateSemaphore(int initCount, int maxCount)
{
	struct k_sem *ret_sem;
	struct k_sem sem;

	for (int i = 0; i < 50; i++) {
		if (semaphores[i].limit == 0) {
			k_sem_init(&semaphores[i], initCount, maxCount);

			return &semaphores[i];
		}
	}

	return 0;
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

	memset(sem, 0x0, sizeof(*sem));

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

int SES_PORT_SPI_Init(void *param_p,
                      SES_PORT_intfType_t *intfType_p,
                      uint8_t *srcMac_p)
{
	*intfType_p = SES_PORT_spiInterface;

	return 0;
}

int SES_PORT_SPI_Release(int intfHandle)
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
	struct spi_buf tx_buf;
	struct spi_buf_set tx;
	uint8_t *txb;
	int ret = 0;

	tx_buf.len = size + 1;
	txb = k_calloc(size + 1, 1);
	if (!txb) {
		LOG_ERR("Calloc error %d txb", (uint32_t)txb);
		ret = -ENOMEM;
		goto free_txb;
	}

	tx_buf.buf = txb;
	tx.buffers = &tx_buf;
	tx.count = 1;

	txb[0] = ADIN6310_SPI_WR_HEADER;
	memcpy(&txb[1], data_p, size);
	ret = spi_write_dt(&dev_spi, &tx);
	
	SES_PORT_Free(data_p);
free_txb:
	k_free(txb);

	return ret;
}

int adin6310_read(uint8_t *buf, uint32_t len)
{
	int ret;
	uint8_t *xfer_buf_tx;
	uint8_t *xfer_buf_rx;
	struct spi_buf rx_buf;
	struct spi_buf tx_buf;
	struct spi_buf_set rx_buf_set;
	struct spi_buf_set tx_buf_set;
	const struct spi_dt_spec dev_spi = SPI_DT_SPEC_GET(DT_NODELABEL(adin6310),
							   SPI_WORD_SET(8) |
							   SPI_OP_MODE_MASTER |
							   SPI_TRANSFER_MSB |
							   SPI_MODE_GET(0), 0);

	xfer_buf_rx = k_calloc(len + 1, sizeof(xfer_buf_rx));
	if (!xfer_buf_rx)
		return -ENOMEM;

	xfer_buf_tx = k_calloc(len + 1, sizeof(*xfer_buf_tx));
	if (!xfer_buf_tx)
		goto free_rx;

	xfer_buf_tx[0] = 0x80;

	rx_buf.len = len + 1;
	rx_buf.buf = xfer_buf_rx;
	tx_buf.len = len + 1;
	tx_buf.buf = xfer_buf_tx;

	rx_buf_set.buffers = &rx_buf;
	rx_buf_set.count = 1;
	tx_buf_set.buffers = &tx_buf;
	tx_buf_set.count = 1;

	ret = spi_transceive_dt(&dev_spi, &tx_buf_set, &rx_buf_set);
	if (ret)
		goto free_tx;

	memcpy(buf, &xfer_buf_rx[1], len);

free_tx:
	k_free(xfer_buf_tx);
free_rx:
	k_free(xfer_buf_rx);

	return ret;
}

int adin6310_spi_read(int tbl_index)
{
	uint32_t frame_type;
	uint8_t *frame_buf;
	uint32_t padded_len;
	uint32_t rx_len;
	uint8_t header[4];
	uint32_t pad;
	int ret;

	ret = adin6310_read(header, 4);
	if (ret)
		return ret;

	rx_len = ((header[1] & 0xF) << 8) | header[0];
	frame_type = (header[1] & 0xF0) >> 4;

	padded_len = rx_len + 0x3;
	padded_len &= ~GENMASK(1, 0);

	frame_buf = k_calloc(padded_len, sizeof(*frame_buf));
	if (!frame_buf)
		return -ENOMEM;

	ret = adin6310_read(frame_buf, padded_len);
	if (ret)
		return ret;

	ret = SES_ReceiveMessage(tbl_index,
			SES_PORT_spiInterface,
			rx_len,
			(void*)frame_buf,
			(-1),
			(0),
			NULL);

	k_free(frame_buf);

	return ret;
}

void adin6310_msg_recv(void *p1, void *p2, void *p3)
{
	int ret;

	printf("Reader thread start\n");

	while (1) {
		k_sem_take(&reader_thread_sem, K_FOREVER);
		ret = adin6310_spi_read(0);
		if (ret)
			return;
	};
}

void adin6310_int_rdy()
{
	k_sem_give(&reader_thread_sem);
}

int main(void)
{
	int32_t ret;
	int iface;
	sesID_t dev_id;
	struct k_thread thread_data;
	struct gpio_callback cb_data;
	uint8_t mac_addr[6] = {0x00, 0x18, 0x80, 0x03, 0x25, 0x60};

	const SES_portInit_t initializePorts_p[] = {
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 0, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 1, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1300, {true, 0, 6, SES_phySpeed1000, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1300, {true, 0, 5, SES_phySpeed1000, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 2, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 3, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}}
	};

	SES_driverFunctions_t comm_callbacks = {
		.init_p = SES_PORT_SPI_Init,
		.release_p = SES_PORT_SPI_Release,
		.sendMessage_p = adin6310_spi_write,
	};

	struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(adin6310), reset_gpios);
	struct gpio_dt_spec irq_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(adin6310), int_gpios);

	ret = gpio_pin_configure_dt(&irq_gpio, GPIO_INPUT);
	if (ret)
		return ret;

	ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret) {
        	LOG_ERR("Failed to configure reset GPIO, %d", ret);
		return ret;
        }

	gpio_pin_set_dt(&reset_gpio, 1);
        k_busy_wait(1000);
        gpio_pin_set_dt(&reset_gpio, 0);
        k_busy_wait(1000);

	k_tid_t spi_read_tid = k_thread_create(&thread_data, stack_area,
					       K_THREAD_STACK_SIZEOF(stack_area),
					       adin6310_msg_recv, NULL, NULL, NULL,
					       K_PRIO_PREEMPT(0), 0, K_FOREVER);

	k_thread_name_set(spi_read_tid, "ADIN6310 SPI reader");
	/* The SPI is not initialized, so we may start the thread. */
	k_thread_start(spi_read_tid);
	gpio_init_callback(&cb_data, adin6310_int_rdy, BIT(irq_gpio.pin));
	gpio_add_callback(irq_gpio.port, &cb_data);
	ret = gpio_pin_interrupt_configure_dt(&irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret)
		return ret;

	ret = SES_Init();
	if (ret)
		printf("SES_Init() error\n");

	ret = SES_AddHwInterface(NULL, &comm_callbacks, &iface);
	if (ret)
		printf("SES_AddHwInterface() error\n");

	ret = SES_AddDevice(iface, mac_addr, &dev_id);
	if (ret)
		printf("SES_AddDevice() error %d\n", ret);

	ret = SES_MX_InitializePorts(dev_id, 6, initializePorts_p);
	if (ret)
		printf("SES_MX_InitializePorts() error %d\n", ret);

	while (1) {
		k_sleep(K_MSEC(10));
	}

	return 0;
}
