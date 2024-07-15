/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/net/net_config.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "SMP_stack_api.h"
#include "SES_port_api.h"
#include "SES_switch.h"
#include "SES_frame_api.h"
#include "SES_interface_management.h"
#include "zephyr/sys/util.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static void print_netifs(struct net_if *iface, void *user_data)
{
	struct net_if_addr *if_addr;
	struct in_addr src = { { { 192, 168, 1, 5 } } };

	if_addr = net_if_ipv4_addr_add(iface, &src, NET_ADDR_MANUAL, 0);
	if (!if_addr){
		printf("Error adding ip\n");
	} else {
		printf("Success adding ip\n");
	}
}

int main(void)
{	
	int ret;
	int srv_fd;
	int counter = 0;
	struct sockaddr_in bind_addr;

	net_if_foreach(print_netifs, NULL);

	return 0;
}
