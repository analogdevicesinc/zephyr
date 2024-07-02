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
#include "SES_PORT_interface.h"
#include "SES_interface_management.h"
#include "zephyr/sys/util.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define DISABLE_AUTOMATIC_FW_UPDATE

void print_ipv4(struct net_if *iface, struct net_if_addr *addr, void *user_data)
{
	printf("Iface ip addr: 0x%X\n", addr->address.in_addr.s_addr);
}

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

static const char content[] = {
	// "HTTP/1.0 200 OK"
	"HTTP/1.0 200 OK; "	\
	"Content-Type: text/html; charset=utf-8\r\n\r\n" \
	"<html>" \
	"<head>" \
	"<title>DWSampleFiles</title>" \
	"<meta charset=\"utf-8\"/>" \
	"<meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\"/>" \
	"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/>" \
	"<style type=\"text/css\">* {font-family: Inter;} body {padding: 2em;}</style>" \
	"</head>" \
	"<body>" \
	"<div>" \
	"<h1>Hello world!</h1>" \
	"<p>This is sample 1.</p>" \
	"<!-- Hi! -->" \
	"<p>" \
	"<a href=\"https://www.dwsamplefiles.com/\">Learn More</a>" \
	"</p>" \
	"</div>" \
	"</body>" \
	"</html>"
};

int main(void)
{	
	int ret;
	int srv_fd;
	int counter = 0;
	struct sockaddr_in bind_addr;

	net_if_foreach(print_netifs, NULL);

	srv_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (srv_fd < 0)
		return -1;

	bind_addr.sin_family = AF_INET;
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bind_addr.sin_port = htons(8080);

	ret = bind(srv_fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
	if (ret)
		return ret;

	ret = listen(srv_fd, 5);
	if (ret)
		return ret;

	printf("Single-threaded dumb HTTP server waits for a connection on "
	       "port 8080...\n");

while (1) {
		struct sockaddr_in client_addr;
		socklen_t client_addr_len = sizeof(client_addr);
		char addr_str[32];
		int req_state = 0;
		const char *data;
		size_t len;

		int client = accept(srv_fd, (struct sockaddr *)&client_addr,
				    &client_addr_len);
		if (client < 0) {
			printf("Error in accept: %d - continuing\n", errno);
			continue;
		}

		inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
			  addr_str, sizeof(addr_str));
		printf("Connection #%d from %s\n", counter++, addr_str);

		/* Discard HTTP request (or otherwise client will get
		 * connection reset error).
		 */
		while (1) {
			ssize_t r;
			char c;

			r = recv(client, &c, 1, 0);
			if (r == 0) {
				goto close_client;
			}

			if (r < 0) {
				if (errno == EAGAIN || errno == EINTR) {
					continue;
				}

				printf("Got error %d when receiving from "
				       "socket\n", errno);
				goto close_client;
			}
			if (req_state == 0 && c == '\r') {
				req_state++;
			} else if (req_state == 1 && c == '\n') {
				req_state++;
			} else if (req_state == 2 && c == '\r') {
				req_state++;
			} else if (req_state == 3 && c == '\n') {
				break;
			} else {
				req_state = 0;
			}
		}

		data = content;
		len = sizeof(content);
		while (len) {
			int sent_len = send(client, data, len, 0);

			if (sent_len == -1) {
				printf("Error sending data to peer, errno: %d\n", errno);
				break;
			}
			data += sent_len;
			len -= sent_len;
		}

close_client:
		ret = close(client);
		if (ret == 0) {
			printf("Connection from %s closed\n", addr_str);
		} else {
			printf("Got error %d while closing the "
			       "socket\n", errno);
		}
	}

	return 0;
}
