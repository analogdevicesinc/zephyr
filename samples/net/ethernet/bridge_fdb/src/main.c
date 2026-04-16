/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_bridge_fdb_sample, LOG_LEVEL_INF);

#include <zephyr/net/ethernet_bridge.h>
#include <zephyr/net/ethernet_bridge_fdb.h>
#include <zephyr/net/ethernet_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/kernel.h>

#define BRIDGE_IP_ADDR    "192.168.80.50"
#define BRIDGE_NETMASK    "255.255.255.0"

/* IEEE 802 Local Experimental EtherType — easy to filter in Wireshark */
#define TEST_ETHERTYPE 0x88B5

/* ---- FDB event handler ------------------------------------------------- */

static struct net_mgmt_event_callback fdb_evt_cb;

static void fdb_event_handler(struct net_mgmt_event_callback *cb,
			      uint64_t mgmt_event, struct net_if *iface)
{
	const struct eth_bridge_fdb_entry *entry = cb->info;

	ARG_UNUSED(cb);

	if (entry == NULL) {
		return;
	}

	if (mgmt_event == NET_EVENT_ETHERNET_FDB_ADD) {
		LOG_INF("FDB_ADD  %02x:%02x:%02x:%02x:%02x:%02x iface %d flags 0x%02x",
			entry->mac.addr[0], entry->mac.addr[1],
			entry->mac.addr[2], entry->mac.addr[3],
			entry->mac.addr[4], entry->mac.addr[5],
			net_if_get_by_iface(entry->iface),
			entry->flags);
	} else if (mgmt_event == NET_EVENT_ETHERNET_FDB_DEL) {
		LOG_INF("FDB_DEL  %02x:%02x:%02x:%02x:%02x:%02x iface %d flags 0x%02x",
			entry->mac.addr[0], entry->mac.addr[1],
			entry->mac.addr[2], entry->mac.addr[3],
			entry->mac.addr[4], entry->mac.addr[5],
			net_if_get_by_iface(entry->iface),
			entry->flags);
	}
}

/* ---- Bridge setup (reused pattern from bridge sample) ------------------- */

struct bridge_ctx {
	struct net_if *bridge;
	struct net_if *ifaces[CONFIG_NET_ETHERNET_BRIDGE_ETH_INTERFACE_COUNT];
	int iface_cnt;
};

static struct bridge_ctx g_ctx;

static void bridge_find_cb(struct eth_bridge_iface_context *br, void *user_data)
{
	struct bridge_ctx *ctx = user_data;

	if (ctx->bridge == NULL) {
		ctx->bridge = br->iface;
		LOG_INF("Found bridge iface %d", net_if_get_by_iface(br->iface));
	}
}

static void bridge_add_iface_cb(struct net_if *iface, void *user_data)
{
	struct bridge_ctx *ctx = user_data;
	int ret;

	if (ctx->iface_cnt >= CONFIG_NET_ETHERNET_BRIDGE_ETH_INTERFACE_COUNT) {
		return;
	}

	if (net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET)) {
		return;
	}

	ctx->ifaces[ctx->iface_cnt] = iface;
	ctx->iface_cnt++;

	LOG_INF("Adding iface %d to bridge", net_if_get_by_iface(iface));

	ret = eth_bridge_iface_add(ctx->bridge, iface);
	if (ret < 0) {
		LOG_ERR("eth_bridge_iface_add failed: %d", ret);
	}
}

/* ---- FDB table dump helper --------------------------------------------- */

static void fdb_dump_cb(struct eth_bridge_fdb_entry *entry, void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_INF("  FDB: %02x:%02x:%02x:%02x:%02x:%02x -> iface %d  flags 0x%02x",
		entry->mac.addr[0], entry->mac.addr[1],
		entry->mac.addr[2], entry->mac.addr[3],
		entry->mac.addr[4], entry->mac.addr[5],
		net_if_get_by_iface(entry->iface),
		entry->flags);
}

/* ---- Raw socket RX test ------------------------------------------------- */

#define RX_THREAD_STACK_SIZE 2048
#define RX_THREAD_PRIORITY   7
#define RX_BUF_SIZE          128
#define RX_TIMEOUT_SEC       30

static K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
static struct k_thread rx_thread_data;

struct raw_rx_ctx {
	struct net_if *iface;
	int port_idx;
};

static struct raw_rx_ctx rx_ctx;

static void raw_rx_thread(void *p1, void *p2, void *p3)
{
	struct raw_rx_ctx *ctx = p1;
	int ifindex = net_if_get_by_iface(ctx->iface);
	struct sockaddr_ll addr = {
		.sll_family = AF_PACKET,
		.sll_protocol = htons(ETH_P_ALL),
		.sll_ifindex = ifindex,
	};
	uint8_t buf[RX_BUF_SIZE];
	int sock, ret;
	struct zsock_timeval tv = {
		.tv_sec = RX_TIMEOUT_SEC,
		.tv_usec = 0,
	};

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	sock = zsock_socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (sock < 0) {
		LOG_ERR("RX Port %d: socket() failed: %d", ctx->port_idx, errno);
		return;
	}

	ret = zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
		LOG_ERR("RX Port %d: bind() failed: %d", ctx->port_idx, errno);
		zsock_close(sock);
		return;
	}

	ret = zsock_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	if (ret < 0) {
		LOG_WRN("RX Port %d: setsockopt(SO_RCVTIMEO) failed: %d",
			ctx->port_idx, errno);
	}

	LOG_INF("RX Port %d (iface %d): listening for %d seconds...",
		ctx->port_idx, ifindex, RX_TIMEOUT_SEC);

	while (true) {
		ret = zsock_recv(sock, buf, sizeof(buf), 0);
		if (ret < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				LOG_INF("RX Port %d: timeout, stopping", ctx->port_idx);
				break;
			}
			LOG_ERR("RX Port %d: recv() failed: %d", ctx->port_idx, errno);
			break;
		}

		if (ret >= (int)sizeof(struct net_eth_hdr)) {
			struct net_eth_hdr *hdr = (struct net_eth_hdr *)buf;
			uint16_t etype = ntohs(hdr->type);

			/* Only log our test frames to avoid flooding the console */
			if (etype == TEST_ETHERTYPE) {
				int payload_len = ret - sizeof(struct net_eth_hdr);

				LOG_INF("RX Port %d: %d bytes, src %02x:%02x:%02x:%02x:%02x:%02x, "
					"ethertype 0x%04X, payload[0..4]: %c%c%c%c%c",
					ctx->port_idx, ret,
					hdr->src.addr[0], hdr->src.addr[1],
					hdr->src.addr[2], hdr->src.addr[3],
					hdr->src.addr[4], hdr->src.addr[5],
					etype,
					payload_len > 0 ? buf[14] : '?',
					payload_len > 1 ? buf[15] : '?',
					payload_len > 2 ? buf[16] : '?',
					payload_len > 3 ? buf[17] : '?',
					payload_len > 4 ? buf[18] : '?');
			}
		}
	}

	zsock_close(sock);
	LOG_INF("RX Port %d: done", ctx->port_idx);
}

/* ---- Raw socket TX test ------------------------------------------------- */

struct raw_test_frame {
	struct net_eth_hdr eth;
	uint8_t payload[32];
} __packed;

static void wait_for_iface(struct net_if *iface, int port_idx)
{
	uint8_t timeout = 5;
	
	while (!net_if_is_up(iface) && timeout) {
		LOG_INF("Port %d: waiting for link...", port_idx);
		k_sleep(K_SECONDS(1));
		timeout--;
	}

	if (!net_if_is_up(iface))
		LOG_INF("Port %d: link timeout...", port_idx);
}

static void raw_tx_test(struct bridge_ctx *ctx)
{
	static const struct net_eth_addr dst = {
		.addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
	};

	for (int i = 0; i < ctx->iface_cnt; i++) {
		struct net_if *iface = ctx->ifaces[i];
		int ifindex = net_if_get_by_iface(iface);

		wait_for_iface(iface, i);
		struct net_linkaddr *ll = net_if_get_link_addr(iface);
		struct sockaddr_ll addr = {
			.sll_family = AF_PACKET,
			.sll_protocol = htons(TEST_ETHERTYPE),
			.sll_ifindex = ifindex,
		};
		struct raw_test_frame frame = {0};
		int sock, ret;

		sock = zsock_socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
		if (sock < 0) {
			LOG_ERR("Port %d: socket() failed: %d", i, errno);
			continue;
		}

		ret = zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr));
		if (ret < 0) {
			LOG_ERR("Port %d: bind() failed: %d", i, errno);
			zsock_close(sock);
			continue;
		}

		/* Build raw Ethernet frame */
		memcpy(frame.eth.dst.addr, dst.addr, sizeof(dst));
		memcpy(frame.eth.src.addr, ll->addr, sizeof(frame.eth.src));
		frame.eth.type = htons(TEST_ETHERTYPE);

		/* Payload: port index + marker so frames are distinguishable */
		memset(frame.payload, 0, sizeof(frame.payload));
		frame.payload[0] = 'P';
		frame.payload[1] = 'O';
		frame.payload[2] = 'R';
		frame.payload[3] = 'T';
		frame.payload[4] = '0' + i;

		ret = zsock_sendto(sock, &frame, sizeof(frame), 0,
				   (struct sockaddr *)&addr, sizeof(addr));
		if (ret < 0) {
			LOG_ERR("Port %d: sendto() failed: %d", i, errno);
		} else {
			LOG_INF("Port %d (iface %d): sent %d bytes, ethertype 0x%04X",
				i, ifindex, ret, TEST_ETHERTYPE);
		}

		zsock_close(sock);
	}
}

/* ---- Main --------------------------------------------------------------- */

int main(void)
{
	struct net_eth_addr mac1 = { .addr = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01} };
	struct net_eth_addr mac2 = { .addr = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02} };
	int ret;

	/* 1. Register FDB event listener */
	net_mgmt_init_event_callback(&fdb_evt_cb, fdb_event_handler,
				     NET_EVENT_ETHERNET_FDB_ADD |
				     NET_EVENT_ETHERNET_FDB_DEL);
	net_mgmt_add_event_callback(&fdb_evt_cb);
	LOG_INF("FDB event listener registered");

	/* 2. Set up bridge */
	net_eth_bridge_foreach(bridge_find_cb, &g_ctx);
	if (g_ctx.bridge == NULL) {
		LOG_ERR("No bridge interface found");
		return -1;
	}

	net_if_foreach(bridge_add_iface_cb, &g_ctx);
	net_if_up(g_ctx.bridge);
	LOG_INF("Bridge is up with %d port(s)", g_ctx.iface_cnt);

	/* Assign a static IP to the bridge interface so the board is pingable */
	{
		struct in_addr addr, netmask;

		net_addr_pton(AF_INET, BRIDGE_IP_ADDR, &addr);
		net_addr_pton(AF_INET, BRIDGE_NETMASK, &netmask);

		net_if_ipv4_addr_add(g_ctx.bridge, &addr, NET_ADDR_MANUAL, 0);
		net_if_ipv4_set_netmask_by_addr(g_ctx.bridge, &addr, &netmask);

		LOG_INF("Bridge IP: %s / %s", BRIDGE_IP_ADDR, BRIDGE_NETMASK);
	}

	if (g_ctx.iface_cnt < 2) {
		LOG_WRN("Need at least 2 ports for FDB test, have %d", g_ctx.iface_cnt);
		return 0;
	}

	/* 3. Automated FDB test */
	LOG_INF("--- Adding static FDB entries ---");

	ret = eth_bridge_fdb_add(&mac1, g_ctx.ifaces[0]);
	LOG_INF("fdb_add(AA:BB:CC:DD:EE:01, iface %d) = %d",
		net_if_get_by_iface(g_ctx.ifaces[0]), ret);

	ret = eth_bridge_fdb_add(&mac2, g_ctx.ifaces[1]);
	LOG_INF("fdb_add(AA:BB:CC:DD:EE:02, iface %d) = %d",
		net_if_get_by_iface(g_ctx.ifaces[1]), ret);

	LOG_INF("--- FDB table after add ---");
	eth_bridge_fdb_foreach(fdb_dump_cb, NULL);

	LOG_INF("--- Deleting static FDB entries ---");

	ret = eth_bridge_fdb_del(&mac1, g_ctx.ifaces[0]);
	LOG_INF("fdb_del(AA:BB:CC:DD:EE:01, iface %d) = %d",
		net_if_get_by_iface(g_ctx.ifaces[0]), ret);

	ret = eth_bridge_fdb_del(&mac2, g_ctx.ifaces[1]);
	LOG_INF("fdb_del(AA:BB:CC:DD:EE:02, iface %d) = %d",
		net_if_get_by_iface(g_ctx.ifaces[1]), ret);

	LOG_INF("--- FDB table after delete ---");
	eth_bridge_fdb_foreach(fdb_dump_cb, NULL);

	LOG_INF("FDB test complete. Use 'net bridge fdb' shell commands for interactive testing.");

	/* 4. Raw socket RX test — listen on port 0 for frames from PC.
	 * Send from PC with:
	 *   sudo python3 -c "from scapy.all import *; \
	 *     sendp(Ether(type=0x88b5)/b'HELLO_FROM_PC', iface='eth0')"
	 * The RX thread filters for ethertype 0x88B5 and logs matching frames.
	 */
	LOG_INF("--- Raw socket RX test (port 0) ---");
	rx_ctx.iface = g_ctx.ifaces[0];
	rx_ctx.port_idx = 0;
	k_thread_create(&rx_thread_data, rx_thread_stack,
			K_THREAD_STACK_SIZEOF(rx_thread_stack),
			raw_rx_thread, &rx_ctx, NULL, NULL,
			RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	/* 5. Raw socket TX test — send one frame per port.
	 * Use Wireshark filter: eth.type == 0x88b5
	 * Each frame payload starts with "PORT0" or "PORT1" to identify the source port.
	 * If the bridge is NOT leaking, you should only see the frame from the port
	 * connected to your PC — not from the other port.
	 */
	LOG_INF("--- Raw socket TX test ---");
	raw_tx_test(&g_ctx);
	LOG_INF("Raw TX test done. Check Wireshark for ethertype 0x88B5 frames.");

	return 0;
}
