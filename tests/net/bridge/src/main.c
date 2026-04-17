/*
 * Copyright (c) 2021 BayLibre SAS
 * Copyright (c) 2024 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define NET_LOG_LEVEL CONFIG_NET_ETHERNET_BRIDGE_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_test, NET_LOG_LEVEL);

#include <zephyr/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>

#include <zephyr/ztest.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ethernet_bridge.h>
#include <zephyr/net/ethernet_bridge_fdb.h>
#include <zephyr/net/virtual.h>
#include <zephyr/net/promiscuous.h>

#if NET_LOG_LEVEL >= LOG_LEVEL_DBG
#define DBG(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

static struct net_if *bridge;

struct eth_fake_context {
	struct net_if *iface;
	struct net_pkt *sent_pkt;
	uint8_t mac_address[6];
	bool promisc_mode;
};

static void eth_fake_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_fake_context *ctx = dev->data;

	ctx->iface = iface;

	ctx->mac_address[0] = 0xc2;
	ctx->mac_address[1] = 0xaa;
	ctx->mac_address[2] = 0xbb;
	ctx->mac_address[3] = 0xcc;
	ctx->mac_address[4] = 0xdd;
	ctx->mac_address[5] = 0xee;

	net_if_set_link_addr(iface, ctx->mac_address,
			     sizeof(ctx->mac_address),
			     NET_LINK_ETHERNET);

	ethernet_init(iface);
}

static int eth_fake_send(const struct device *dev,
			 struct net_pkt *pkt)
{
	struct eth_fake_context *ctx = dev->data;
	struct net_eth_hdr *eth_hdr = NET_ETH_HDR(pkt);

	/*
	 * Ignore packets we don't care about for this test, like
	 * the IP autoconfig related ones, etc.
	 */
	if (eth_hdr->type != net_htons(NET_ETH_PTYPE_ALL)) {
		DBG("Fake send ignoring pkt %p\n", pkt);
		return 0;
	}

	if (ctx->sent_pkt != NULL) {
		DBG("Fake send found pkt %p while sending %p\n",
		    ctx->sent_pkt, pkt);
		return -EBUSY;
	}
	ctx->sent_pkt = net_pkt_shallow_clone(pkt, K_NO_WAIT);
	if (ctx->sent_pkt == NULL) {
		DBG("Fake send out of mem while sending pkt %p\n", pkt);
		return -ENOMEM;
	}

	DBG("Fake send pkt %p kept locally as %p\n", pkt, ctx->sent_pkt);
	return 0;
}

static enum ethernet_hw_caps eth_fake_get_capabilities(const struct device *dev)
{
	return ETHERNET_PROMISC_MODE;
}

static int eth_fake_set_config(const struct device *dev,
			       enum ethernet_config_type type,
			       const struct ethernet_config *config)
{
	struct eth_fake_context *ctx = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		if (config->promisc_mode == ctx->promisc_mode) {
			return -EALREADY;
		}

		ctx->promisc_mode = config->promisc_mode;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct ethernet_api eth_fake_api_funcs = {
	.iface_api.init = eth_fake_iface_init,
	.get_capabilities = eth_fake_get_capabilities,
	.set_config = eth_fake_set_config,
	.send = eth_fake_send,
};

static int eth_fake_init(const struct device *dev)
{
	struct eth_fake_context *ctx = dev->data;

	ctx->promisc_mode = false;

	return 0;
}

static struct eth_fake_context eth_fake_data[3];

ETH_NET_DEVICE_INIT(eth_fake0, "eth_fake0",
		    eth_fake_init, NULL,
		    &eth_fake_data[0], NULL, CONFIG_ETH_INIT_PRIORITY,
		    &eth_fake_api_funcs, NET_ETH_MTU);

ETH_NET_DEVICE_INIT(eth_fake1, "eth_fake1",
		    eth_fake_init, NULL,
		    &eth_fake_data[1], NULL, CONFIG_ETH_INIT_PRIORITY,
		    &eth_fake_api_funcs, NET_ETH_MTU);

ETH_NET_DEVICE_INIT(eth_fake2, "eth_fake2",
		    eth_fake_init, NULL,
		    &eth_fake_data[2], NULL, CONFIG_ETH_INIT_PRIORITY,
		    &eth_fake_api_funcs, NET_ETH_MTU);

static struct net_if *fake_iface[3];

static void iface_cb(struct net_if *iface, void *user_data)
{
	static int if_count;

	if (if_count >= ARRAY_SIZE(fake_iface)) {
		return;
	}

	DBG("Interface %p [%d]\n", iface, net_if_get_by_iface(iface));

	if (net_if_l2(iface) == &NET_L2_GET_NAME(ETHERNET)) {
		const struct ethernet_api *api =
			net_if_get_device(iface)->api;

		/*
		 * We want to only use struct net_if devices defined in
		 * this test as board on which it is run can have its
		 * own set of interfaces.
		 */
		if (api->get_capabilities ==
		    eth_fake_api_funcs.get_capabilities) {
			fake_iface[if_count++] = iface;
		}
	}

	if (net_if_l2(iface) == &NET_L2_GET_NAME(VIRTUAL)) {
		enum virtual_interface_caps caps;

		caps = net_virtual_get_iface_capabilities(iface);
		if (caps & VIRTUAL_INTERFACE_BRIDGE) {
			bridge = iface;
		}
	}
}

static int orig_rx_num_blocks;
static int orig_tx_num_blocks;

static void get_free_packet_count(void)
{
	struct k_mem_slab *rx, *tx;

	net_pkt_get_info(&rx, &tx, NULL, NULL);
	orig_rx_num_blocks = rx->info.num_blocks;
	orig_tx_num_blocks = tx->info.num_blocks;
}

static void check_free_packet_count(void)
{
	struct k_mem_slab *rx, *tx;

	net_pkt_get_info(&rx, &tx, NULL, NULL);
	zassert_equal(rx->info.num_blocks, orig_rx_num_blocks, "");
	zassert_equal(tx->info.num_blocks, orig_tx_num_blocks, "");
}

static void test_iface_setup(void)
{
	net_if_foreach(iface_cb, NULL);

	zassert_not_null(fake_iface[0], "");
	zassert_not_null(fake_iface[1], "");
	zassert_not_null(fake_iface[2], "");

	DBG("Interfaces: [%d] iface0 %p, [%d] iface1 %p, [%d] iface2 %p\n",
	    net_if_get_by_iface(fake_iface[0]), fake_iface[0],
	    net_if_get_by_iface(fake_iface[1]), fake_iface[1],
	    net_if_get_by_iface(fake_iface[2]), fake_iface[2]);

	net_if_up(fake_iface[0]);
	net_if_up(fake_iface[1]);
	net_if_up(fake_iface[2]);

	/* Remember the initial number of free packets in the pool. */
	get_free_packet_count();
}

/*
 * Simulate a packet reception from the outside world
 */
static void _recv_data(struct net_if *iface)
{
	struct net_pkt *pkt;
	struct net_eth_hdr eth_hdr;
	static uint8_t data[] = { 't', 'e', 's', 't', '\0' };
	int ret;

	pkt = net_pkt_rx_alloc_with_buffer(iface, sizeof(eth_hdr) + sizeof(data),
					   NET_AF_UNSPEC, 0, K_FOREVER);
	zassert_not_null(pkt, "");

	/*
	 * The source and destination MAC addresses are completely arbitrary
	 * except for the U/L and I/G bits. However, the index of the faked
	 * incoming interface is mixed in as well to create some variation,
	 * and to help with validation on the transmit side.
	 */

	eth_hdr.dst.addr[0] = 0xb2;
	eth_hdr.dst.addr[1] = 0x11;
	eth_hdr.dst.addr[2] = 0x22;
	eth_hdr.dst.addr[3] = 0x33;
	eth_hdr.dst.addr[4] = net_if_get_by_iface(iface);
	eth_hdr.dst.addr[5] = 0x55;

	eth_hdr.src.addr[0] = 0xa2;
	eth_hdr.src.addr[1] = 0x11;
	eth_hdr.src.addr[2] = 0x22;
	eth_hdr.src.addr[3] = net_if_get_by_iface(iface);
	eth_hdr.src.addr[4] = 0x77;
	eth_hdr.src.addr[5] = 0x88;

	eth_hdr.type = net_htons(NET_ETH_PTYPE_ALL);

	ret = net_pkt_write(pkt, &eth_hdr, sizeof(eth_hdr));
	zassert_equal(ret, 0, "");

	ret = net_pkt_write(pkt, data, sizeof(data));
	zassert_equal(ret, 0, "");

	DBG("[%d] Fake recv pkt %p\n", net_if_get_by_iface(iface), pkt);
	ret = net_recv_data(iface, pkt);
	zassert_equal(ret, 0, "");
}

static void test_recv_before_bridging(void)
{
	/* fake some packet reception */
	_recv_data(fake_iface[0]);
	_recv_data(fake_iface[1]);
	_recv_data(fake_iface[2]);

	/* give time to the processing threads to run */
	k_sleep(K_MSEC(100));

	/* nothing should have been transmitted at this point */
	zassert_is_null(eth_fake_data[0].sent_pkt, "");
	zassert_is_null(eth_fake_data[1].sent_pkt, "");
	zassert_is_null(eth_fake_data[2].sent_pkt, "");

	/* and everything already dropped. */
	check_free_packet_count();
}

static void test_setup_bridge(void)
{
	int ret;

	/* add our interfaces to the bridge */
	ret = eth_bridge_iface_add(bridge, fake_iface[0]);
	zassert_equal(ret, 0, "");
	ret = eth_bridge_iface_add(bridge, fake_iface[1]);
	zassert_equal(ret, 0, "");

	/* Try to add the bridge twice, there should be no error */
	ret = eth_bridge_iface_add(bridge, fake_iface[1]);
	zassert_equal(ret, 0, "");

	ret = eth_bridge_iface_add(bridge, fake_iface[2]);
	zassert_equal(ret, 0, "");

	ret = net_if_up(bridge);
	zassert_equal(ret, 0, "");
}

static void test_recv_with_bridge(void)
{
	int i, j;

	for (i = 0; i < 3; i++) {
		int src_if_idx = net_if_get_by_iface(fake_iface[i]);

		/* fake reception of packets */
		_recv_data(fake_iface[i]);

		/* give time to the processing threads to run */
		k_sleep(K_MSEC(100));

		/*
		 * fake_iface[0] and fake_iface[2] should have sent the packet
		 * but only if it didn't come from them.
		 * We skip fake_iface[1] handled above.
		 */
		for (j = 0; j < 3; j += 2) {
			struct net_pkt *pkt = eth_fake_data[j].sent_pkt;
			struct net_eth_hdr *hdr;

			if (eth_fake_data[j].iface == fake_iface[i]) {
				zassert_is_null(pkt, "");
				continue;
			}

			eth_fake_data[j].sent_pkt = NULL;
			zassert_not_null(pkt, "");

			/* make sure nothing messed up our ethernet header */
			hdr = NET_ETH_HDR(pkt);

			zassert_equal(hdr->dst.addr[0], 0xb2, "");
			zassert_equal(hdr->src.addr[0], 0xa2, "");
			zassert_equal(hdr->dst.addr[4], src_if_idx, "");
			zassert_equal(hdr->src.addr[3], src_if_idx, "");

			net_pkt_unref(pkt);
		}
	}

	check_free_packet_count();
}

static void test_recv_with_bridge_fdb(void)
{
	struct net_eth_addr mac;
	struct net_pkt *pkt;
	struct net_eth_hdr *hdr;
	uint8_t iface0_index = net_if_get_by_iface(fake_iface[0]);
	int ret;

	/* Add FDB entry: forward fake_iface[0] rx pkt to fake_iface[1] tx path */
	mac.addr[0] = 0xb2;
	mac.addr[1] = 0x11;
	mac.addr[2] = 0x22;
	mac.addr[3] = 0x33;
	mac.addr[4] = iface0_index;
	mac.addr[5] = 0x55;

	ret = eth_bridge_fdb_add(&mac, fake_iface[1]);
	zassert_equal(ret, 0, "");

	/* fake reception of packets */
	_recv_data(fake_iface[0]);

	/* give time to the processing threads to run */
	k_sleep(K_MSEC(100));

	/* check fake_iface[0] tx path */
	pkt = eth_fake_data[0].sent_pkt;
	zassert_is_null(pkt, "");

	/* check fake_iface[2] tx path */
	pkt = eth_fake_data[2].sent_pkt;
	zassert_is_null(pkt, "");

	/* check fake_iface[1] tx path */
	pkt = eth_fake_data[1].sent_pkt;

	eth_fake_data[1].sent_pkt = NULL;
	zassert_not_null(pkt, "");

	/* make sure nothing messed up our ethernet header */
	hdr = NET_ETH_HDR(pkt);

	zassert_equal(hdr->dst.addr[0], 0xb2, "");
	zassert_equal(hdr->src.addr[0], 0xa2, "");
	zassert_equal(hdr->dst.addr[4], iface0_index, "");
	zassert_equal(hdr->src.addr[3], iface0_index, "");

	net_pkt_unref(pkt);

	check_free_packet_count();
}

static void _recv_data_broadcast(struct net_if *iface)
{
	struct net_pkt *pkt;
	struct net_eth_hdr eth_hdr;
	static uint8_t data[] = { 't', 'e', 's', 't', '\0' };
	int ret;

	pkt = net_pkt_rx_alloc_with_buffer(iface, sizeof(eth_hdr) + sizeof(data),
					   NET_AF_UNSPEC, 0, K_FOREVER);
	zassert_not_null(pkt, "");

	/* Broadcast destination */
	memset(eth_hdr.dst.addr, 0xff, sizeof(eth_hdr.dst.addr));

	eth_hdr.src.addr[0] = 0xa2;
	eth_hdr.src.addr[1] = 0x11;
	eth_hdr.src.addr[2] = 0x22;
	eth_hdr.src.addr[3] = net_if_get_by_iface(iface);
	eth_hdr.src.addr[4] = 0x77;
	eth_hdr.src.addr[5] = 0x88;

	eth_hdr.type = net_htons(NET_ETH_PTYPE_ALL);

	ret = net_pkt_write(pkt, &eth_hdr, sizeof(eth_hdr));
	zassert_equal(ret, 0, "");

	ret = net_pkt_write(pkt, data, sizeof(data));
	zassert_equal(ret, 0, "");

	DBG("[%d] Fake recv broadcast pkt %p\n", net_if_get_by_iface(iface), pkt);
	ret = net_recv_data(iface, pkt);
	zassert_equal(ret, 0, "");
}

/*
 * Simulate a packet reception with specific src and dst MACs
 */
static void _recv_data_with_macs(struct net_if *iface,
				 struct net_eth_addr *src,
				 struct net_eth_addr *dst)
{
	struct net_pkt *pkt;
	struct net_eth_hdr eth_hdr;
	static uint8_t data[] = { 't', 'e', 's', 't', '\0' };
	int ret;

	pkt = net_pkt_rx_alloc_with_buffer(iface, sizeof(eth_hdr) + sizeof(data),
					   NET_AF_UNSPEC, 0, K_FOREVER);
	zassert_not_null(pkt, "");

	memcpy(&eth_hdr.dst, dst, sizeof(struct net_eth_addr));
	memcpy(&eth_hdr.src, src, sizeof(struct net_eth_addr));
	eth_hdr.type = net_htons(NET_ETH_PTYPE_ALL);

	ret = net_pkt_write(pkt, &eth_hdr, sizeof(eth_hdr));
	zassert_equal(ret, 0, "");

	ret = net_pkt_write(pkt, data, sizeof(data));
	zassert_equal(ret, 0, "");

	DBG("[%d] Fake recv pkt %p (src %02x:%02x, dst %02x:%02x)\n",
	    net_if_get_by_iface(iface), pkt,
	    src->addr[4], src->addr[5],
	    dst->addr[4], dst->addr[5]);
	ret = net_recv_data(iface, pkt);
	zassert_equal(ret, 0, "");
}

struct fdb_lookup_data {
	struct net_eth_addr *mac;
	struct net_if *iface;
	uint8_t flags;
	bool found;
};

static void fdb_lookup_cb(struct eth_bridge_fdb_entry *entry, void *user_data)
{
	struct fdb_lookup_data *data = user_data;

	if (memcmp(&entry->mac, data->mac, sizeof(struct net_eth_addr)) == 0) {
		data->iface = entry->iface;
		data->flags = entry->flags;
		data->found = true;
	}
}

static void test_recv_with_bridge_fdb_dynamic(void)
{
	struct net_eth_addr src_mac = {
		.addr = {0xa2, 0xdd, 0xee, 0xff, 0x11, 0x22}
	};
	struct net_eth_addr dst_mac_unknown = {
		.addr = {0xb2, 0x99, 0x88, 0x77, 0x66, 0x55}
	};
	struct fdb_lookup_data lookup = {0};

	/* First, delete the static FDB entry from the previous test so it
	 * doesn't interfere with forwarding.
	 */
	struct net_eth_addr static_mac;
	uint8_t iface0_index = net_if_get_by_iface(fake_iface[0]);

	static_mac.addr[0] = 0xb2;
	static_mac.addr[1] = 0x11;
	static_mac.addr[2] = 0x22;
	static_mac.addr[3] = 0x33;
	static_mac.addr[4] = iface0_index;
	static_mac.addr[5] = 0x55;
	(void)eth_bridge_fdb_del(&static_mac, fake_iface[1]);

	/*
	 * Step 1: Send a packet on fake_iface[0] with a known source MAC.
	 * This should cause the bridge to learn src_mac -> fake_iface[0].
	 */
	_recv_data_with_macs(fake_iface[0], &src_mac, &dst_mac_unknown);

	k_sleep(K_MSEC(100));

	/* Clean up forwarded packets from the unknown-dst flood */
	for (int i = 0; i < 3; i++) {
		if (eth_fake_data[i].sent_pkt != NULL) {
			net_pkt_unref(eth_fake_data[i].sent_pkt);
			eth_fake_data[i].sent_pkt = NULL;
		}
	}

	/*
	 * Step 2: Verify the FDB now contains a DYNAMIC entry for src_mac
	 * pointing to fake_iface[0].
	 */
	lookup.mac = &src_mac;
	lookup.found = false;
	eth_bridge_fdb_foreach(fdb_lookup_cb, &lookup);

	zassert_true(lookup.found, "Dynamic FDB entry not found for src_mac");
	zassert_equal(lookup.iface, fake_iface[0],
		      "Dynamic entry points to wrong interface");
	zassert_equal(lookup.flags, ETHERNET_BRIDGE_FDB_FLAG_DYNAMIC,
		      "Entry should be dynamic");

	/*
	 * Step 3: Send a packet on fake_iface[1] with src_mac as the
	 * destination. The FDB should forward it only to fake_iface[0].
	 */
	struct net_eth_addr other_src = {
		.addr = {0xa2, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc}
	};

	_recv_data_with_macs(fake_iface[1], &other_src, &src_mac);

	k_sleep(K_MSEC(100));

	/*
	 * Step 4: Verify forwarding: fake_iface[0] should have received
	 * the packet, but fake_iface[2] should NOT (no flooding).
	 */
	zassert_not_null(eth_fake_data[0].sent_pkt,
			 "Packet should be forwarded to fake_iface[0]");
	zassert_is_null(eth_fake_data[2].sent_pkt,
			"Packet should NOT be flooded to fake_iface[2]");

	/* fake_iface[1] is the source, so it should not receive */
	zassert_is_null(eth_fake_data[1].sent_pkt,
			"Packet should NOT be sent back to source");

	net_pkt_unref(eth_fake_data[0].sent_pkt);
	eth_fake_data[0].sent_pkt = NULL;

	/*
	 * Step 5: Verify static takes precedence. Add a static entry for
	 * src_mac pointing to fake_iface[2], and verify it overwrites
	 * the dynamic one.
	 */
	int ret = eth_bridge_fdb_add(&src_mac, fake_iface[2]);

	zassert_equal(ret, 0, "Failed to add static FDB entry");

	lookup.found = false;
	eth_bridge_fdb_foreach(fdb_lookup_cb, &lookup);

	zassert_true(lookup.found, "FDB entry not found after static add");
	zassert_equal(lookup.flags, ETHERNET_BRIDGE_FDB_FLAG_STATIC,
		      "Entry should now be static");
	zassert_equal(lookup.iface, fake_iface[2],
		      "Static entry should point to fake_iface[2]");

	/* Clean up the static entry */
	(void)eth_bridge_fdb_del(&src_mac, fake_iface[2]);

	/* Clean up any remaining dynamic entries from learning */
	(void)eth_bridge_fdb_del(&other_src, fake_iface[1]);

	check_free_packet_count();
}

static void test_recv_with_bridge_allow_tx(void)
{
	int i, j, ret;

	/* Enable allow_tx (dynamic forwarding) */
	ret = eth_bridge_set_allow_tx(bridge, true);
	zassert_equal(ret, 0, "");
	zassert_true(eth_bridge_get_allow_tx(bridge), "");

	/*
	 * Test unicast with allow_tx: packets should be forwarded to other
	 * bridge ports AND continue processing on the original interface
	 * (returned as NET_CONTINUE). Since the destination MAC doesn't match
	 * any individual interface MAC, the ethernet layer will drop them
	 * after the bridge processes them. But the bridge forwarding should
	 * still work.
	 */
	for (i = 0; i < 3; i++) {
		_recv_data(fake_iface[i]);

		k_sleep(K_MSEC(100));

		/* Packets should still be forwarded to other bridge ports */
		for (j = 0; j < 3; j += 2) {
			struct net_pkt *pkt = eth_fake_data[j].sent_pkt;

			if (eth_fake_data[j].iface == fake_iface[i]) {
				zassert_is_null(pkt, "");
				continue;
			}

			eth_fake_data[j].sent_pkt = NULL;
			zassert_not_null(pkt, "");

			net_pkt_unref(pkt);
		}
	}

	/*
	 * Test broadcast with allow_tx: packets should be forwarded to other
	 * bridge ports. With allow_tx, broadcast is handled on the original
	 * interface (NET_CONTINUE) instead of the bridge interface.
	 */
	for (i = 0; i < 3; i++) {
		_recv_data_broadcast(fake_iface[i]);

		k_sleep(K_MSEC(100));

		/* Broadcast should be forwarded to other bridge ports */
		for (j = 0; j < 3; j += 2) {
			struct net_pkt *pkt = eth_fake_data[j].sent_pkt;

			if (eth_fake_data[j].iface == fake_iface[i]) {
				zassert_is_null(pkt, "");
				continue;
			}

			eth_fake_data[j].sent_pkt = NULL;
			zassert_not_null(pkt, "");

			net_pkt_unref(pkt);
		}
	}

	/* Disable allow_tx */
	ret = eth_bridge_set_allow_tx(bridge, false);
	zassert_equal(ret, 0, "");
	zassert_false(eth_bridge_get_allow_tx(bridge), "");

	check_free_packet_count();
}

static void test_recv_after_bridging(void)
{
	int ret;

	ret = net_if_down(bridge);
	zassert_equal(ret, 0, "");

	/* remove our interfaces from the bridge */
	ret = eth_bridge_iface_remove(bridge, fake_iface[0]);
	zassert_equal(ret, 0, "");
	ret = eth_bridge_iface_remove(bridge, fake_iface[1]);
	zassert_equal(ret, 0, "");
	ret = eth_bridge_iface_remove(bridge, fake_iface[2]);
	zassert_equal(ret, 0, "");

	/* If there are not enough interfaces in the bridge, it is not created */
	ret = net_if_up(bridge);
	zassert_equal(ret, -ENOENT, "");

	eth_fake_data[0].sent_pkt = eth_fake_data[1].sent_pkt =
		eth_fake_data[2].sent_pkt = NULL;

	/* things should have returned to the pre-bridging state */
	test_recv_before_bridging();
}

/* Make sure bridge interface support promiscuous API */
ZTEST(net_eth_bridge, test_verify_promisc_mode)
{
	int ret;

	ret = net_promisc_mode_on(bridge);
	zassert_equal(ret, 0, "");
}

ZTEST(net_eth_bridge, test_net_eth_bridge)
{
	DBG("Before bridging\n");
	test_iface_setup();
	test_recv_before_bridging();
	DBG("With bridging\n");
	test_setup_bridge();
	test_recv_with_bridge();
	test_recv_with_bridge_fdb();
	DBG("With dynamic FDB learning\n");
	test_recv_with_bridge_fdb_dynamic();
	DBG("With allow_tx (dynamic forwarding)\n");
	test_recv_with_bridge_allow_tx();
	DBG("After bridging\n");
	test_recv_after_bridging();
}

ZTEST_SUITE(net_eth_bridge, NULL, NULL, NULL, NULL, NULL);
