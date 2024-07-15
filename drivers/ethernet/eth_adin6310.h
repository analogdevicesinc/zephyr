#ifndef _ETH_ADIN6310_PRIV_H_
#define _ETH_ADIN6310_PRIV_H_

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "zephyr/sys/util.h"
#include "SES_port_api.h"

#define ADIN6310_NUM_PORTS	6
#define ADIN6310_SPI_WR_HEADER	0x40
#define ADIN6310_SPI_RD_HEADER	0x80

struct adin6310_data;

enum adin6310_chip_id {
	ADIN3310 = 0,
	ADIN6310 = 1,
};

struct adin6310_phy_info {
	struct device *dev;
	uint32_t phy_addr;
	bool enabled;
};

struct adin6310_port_config {
	const struct device *adin;
	const struct device *mdio;
	struct device *phy;
	uint32_t phy_addr;
	uint32_t id;
	struct adin6310_data *net_device;
	uint8_t mac_addr[6];
	char *name;
	bool cpu_port;
};

struct adin6310_port_data {
	const struct device *adin;
	struct device *phy;
	uint32_t phy_addr;
	uint32_t id;
	struct adin6310_data *net_device;
	struct net_if *iface;
	uint8_t mac_addr[6];
	char *name;
	bool cpu_port;
};

struct adin6310_data {
	const struct device *port[ADIN6310_NUM_PORTS];
	struct adin6310_port_data *port_data[ADIN6310_NUM_PORTS];
	struct gpio_callback gpio_int_callback;
	const struct gpio_dt_spec *interrupt;
	struct k_mutex lock;
	struct k_mutex spi_lock;
	uint8_t frame_buf[CONFIG_ETH_ADIN6310_BUFFER_SIZE];
	uint8_t rx_buf[CONFIG_ETH_ADIN6310_BUFFER_SIZE];
	uint8_t tx_buf[CONFIG_ETH_ADIN6310_BUFFER_SIZE];
	struct k_sem semaphores[CONFIG_ETH_ADIN6310_SEMAPHORE_COUNT];
	bool ses_configured;

	k_tid_t offload_thread_id;
	struct k_sem offload_thread_sem;
	K_KERNEL_STACK_MEMBER(offload_thread_stack, CONFIG_ETH_ADIN6310_IRQ_THREAD_STACK_SIZE);
	struct k_thread offload_thread;
};

struct adin6310_config {
	enum adin6310_chip_id id;
	struct spi_dt_spec spi;
	const struct gpio_dt_spec interrupt;
	const struct gpio_dt_spec reset;
	struct adin6310_port_config port_config[ADIN6310_NUM_PORTS];
	const struct adin6310_phy_info *phys;
};

#endif /* _ETH_ADIN6310_PRIV_H_ */
