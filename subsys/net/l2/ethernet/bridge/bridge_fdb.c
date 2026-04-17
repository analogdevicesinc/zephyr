/*
 * SPDX-FileCopyrightText: Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_eth_bridge_fdb, CONFIG_NET_ETHERNET_BRIDGE_LOG_LEVEL);

#include <zephyr/net/net_log.h>
#include <zephyr/net/ethernet_bridge.h>
#include <zephyr/net/ethernet_bridge_fdb.h>
#include <zephyr/kernel.h>

/* FDB table */
static sys_slist_t fdb_entries = SYS_SLIST_STATIC_INIT(&fdb_entries);
static K_MUTEX_DEFINE(fdb_lock);
static size_t fdb_count;

/* Memory pool for FDB entries */
K_MEM_SLAB_DEFINE_STATIC(fdb_slab, sizeof(struct eth_bridge_fdb_entry),
	CONFIG_NET_ETHERNET_BRIDGE_FDB_MAX_ENTRIES, 4);

int eth_bridge_fdb_add(struct net_eth_addr *mac, struct net_if *iface)
{
	struct eth_bridge_fdb_entry *entry;
	int ret = 0;

	/* Check mac and iface is valid or not */
	if (mac == NULL || iface == NULL) {
		return -EINVAL;
	}

	if (!net_eth_iface_is_bridged(net_if_l2_data(iface))) {
		return -EINVAL;
	}

	k_mutex_lock(&fdb_lock, K_FOREVER);

	/* Check if the mac already exists in FDB table */
	SYS_SLIST_FOR_EACH_CONTAINER(&fdb_entries, entry, node) {
		if (memcmp(&entry->mac, mac, sizeof(struct net_eth_addr)) == 0) {

			if (entry->iface == iface) {
				NET_DBG("FDB entry exists");
				goto out;
			}

			/* For unicast address, just update it */
			if (net_eth_is_addr_valid(mac)) {
				entry->iface = iface;
				NET_DBG("FDB entry updated: %02x:%02x:%02x:%02x:%02x:%02x -> iface "
					"%d",
					mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3],
					mac->addr[4], mac->addr[5], net_if_get_by_iface(iface));
				goto out;
			}
		}
	}

	/* Check if we have space */
	if (fdb_count >= CONFIG_NET_ETHERNET_BRIDGE_FDB_MAX_ENTRIES) {
		NET_ERR("FDB table full");
		ret = -ENOMEM;
		goto out;
	}

	/* Allocate new entry */
	if (k_mem_slab_alloc(&fdb_slab, (void **)&entry, K_NO_WAIT) != 0) {
		NET_ERR("Failed to allocate FDB entry");
		ret = -ENOMEM;
		goto out;
	}

	/* Initialize entry */
	memcpy(&entry->mac, mac, sizeof(struct net_eth_addr));
	entry->iface = iface;
	entry->flags = ETHERNET_BRIDGE_FDB_FLAG_STATIC;
	entry->last_seen = k_uptime_get();

	/* Add to list */
	sys_slist_prepend(&fdb_entries, &entry->node);
	fdb_count++;

	NET_DBG("FDB entry added: %02x:%02x:%02x:%02x:%02x:%02x -> iface %d", mac->addr[0],
		mac->addr[1], mac->addr[2], mac->addr[3], mac->addr[4], mac->addr[5],
		net_if_get_by_iface(iface));
out:
	k_mutex_unlock(&fdb_lock);
	return ret;
}

int eth_bridge_fdb_del(struct net_eth_addr *mac, struct net_if *iface)
{
	struct eth_bridge_fdb_entry *entry;
	sys_snode_t *node, *prev = NULL;
	int ret = -ENOENT;

	/* Check mac and iface is valid or not */
	if (mac == NULL || iface == NULL) {
		return -EINVAL;
	}

	if (net_eth_get_bridge(net_if_l2_data(iface)) == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&fdb_lock, K_FOREVER);

	SYS_SLIST_FOR_EACH_NODE(&fdb_entries, node) {
		entry = CONTAINER_OF(node, struct eth_bridge_fdb_entry, node);
		if (memcmp(&entry->mac, mac, sizeof(struct net_eth_addr)) == 0) {

			if (entry->iface != iface) {
				prev = node;
				continue;
			}

			/* Remove from list */
			sys_slist_remove(&fdb_entries, prev, node);
			fdb_count--;

			/* Free memory */
			k_mem_slab_free(&fdb_slab, (void *)entry);

			NET_DBG("FDB entry deleted: %02x:%02x:%02x:%02x:%02x:%02x -> iface %d",
				mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3],
				mac->addr[4], mac->addr[5], net_if_get_by_iface(iface));

			ret = 0;
			break;
		}

		prev = node;
	}

	k_mutex_unlock(&fdb_lock);
	return ret;
}

int eth_bridge_fdb_del_iface(struct net_if *iface)
{
	struct eth_bridge_fdb_entry *entry;
	sys_snode_t *node, *prev = NULL;

	if (iface == NULL) {
		return -EINVAL;
	}

	if (net_eth_get_bridge(net_if_l2_data(iface)) == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&fdb_lock, K_FOREVER);

	SYS_SLIST_FOR_EACH_NODE(&fdb_entries, node) {
		entry = CONTAINER_OF(node, struct eth_bridge_fdb_entry, node);
		if (entry->iface != iface) {
			prev = node;
			continue;
		}

		/* Remove from list */
		sys_slist_remove(&fdb_entries, prev, node);
		fdb_count--;

		/* Free memory */
		k_mem_slab_free(&fdb_slab, (void *)entry);

		NET_DBG("FDB entry deleted: %02x:%02x:%02x:%02x:%02x:%02x -> iface %d",
			entry->mac.addr[0], entry->mac.addr[1], entry->mac.addr[2],
			entry->mac.addr[3], entry->mac.addr[4], entry->mac.addr[5],
			net_if_get_by_iface(iface));

		break;
	}

	k_mutex_unlock(&fdb_lock);
	return 0;
}

void eth_bridge_fdb_foreach(eth_bridge_fdb_entry_cb_t cb, void *user_data)
{
	struct eth_bridge_fdb_entry *entry, *tmp;

	k_mutex_lock(&fdb_lock, K_FOREVER);

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&fdb_entries, entry, tmp, node) {
		cb(entry, user_data);
	}

	k_mutex_unlock(&fdb_lock);
}

#define FDB_AGING_TIME_MS (CONFIG_NET_ETHERNET_BRIDGE_FDB_AGING_TIME * 1000LL)

static void fdb_aging_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(fdb_aging_work, fdb_aging_work_handler);

static void fdb_aging_work_handler(struct k_work *work)
{
	struct eth_bridge_fdb_entry *entry;
	sys_snode_t *node, *prev = NULL;
	int64_t now = k_uptime_get();
	bool has_dynamic = false;

	ARG_UNUSED(work);

	k_mutex_lock(&fdb_lock, K_FOREVER);

	node = sys_slist_peek_head(&fdb_entries);
	while (node != NULL) {
		sys_snode_t *next = sys_slist_peek_next(node);

		entry = CONTAINER_OF(node, struct eth_bridge_fdb_entry, node);

		if (entry->flags & ETHERNET_BRIDGE_FDB_FLAG_DYNAMIC) {
			if ((now - entry->last_seen) > FDB_AGING_TIME_MS) {
				NET_DBG("FDB aging out: %02x:%02x:%02x:%02x:%02x:%02x",
					entry->mac.addr[0], entry->mac.addr[1],
					entry->mac.addr[2], entry->mac.addr[3],
					entry->mac.addr[4], entry->mac.addr[5]);

				sys_slist_remove(&fdb_entries, prev, node);
				fdb_count--;
				k_mem_slab_free(&fdb_slab, (void *)entry);

				/* prev stays the same after removal */
				node = next;
				continue;
			}

			has_dynamic = true;
		}

		prev = node;
		node = next;
	}

	k_mutex_unlock(&fdb_lock);

	if (has_dynamic) {
		k_work_reschedule(&fdb_aging_work,
				  K_SECONDS(CONFIG_NET_ETHERNET_BRIDGE_FDB_AGING_TIME));
	}
}

int eth_bridge_fdb_learn(struct net_eth_addr *mac, struct net_if *iface)
{
	struct eth_bridge_fdb_entry *entry;
	int ret = 0;

	if (mac == NULL || iface == NULL) {
		return -EINVAL;
	}

	/* Only learn unicast source MACs */
	if (net_eth_is_addr_broadcast(mac) || net_eth_is_addr_multicast(mac)) {
		return 0;
	}

	k_mutex_lock(&fdb_lock, K_FOREVER);

	/* Check if the MAC already exists */
	SYS_SLIST_FOR_EACH_CONTAINER(&fdb_entries, entry, node) {
		if (memcmp(&entry->mac, mac, sizeof(struct net_eth_addr)) == 0) {
			/* Static entries take precedence — never overwrite */
			if (entry->flags & ETHERNET_BRIDGE_FDB_FLAG_STATIC) {
				goto out;
			}

			/* Dynamic entry: update port and/or timestamp */
			entry->iface = iface;
			entry->last_seen = k_uptime_get();
			goto out;
		}
	}

	/* New MAC — allocate a dynamic entry */
	if (fdb_count >= CONFIG_NET_ETHERNET_BRIDGE_FDB_MAX_ENTRIES) {
		NET_DBG("FDB table full, cannot learn");
		ret = -ENOMEM;
		goto out;
	}

	if (k_mem_slab_alloc(&fdb_slab, (void **)&entry, K_NO_WAIT) != 0) {
		NET_DBG("Failed to allocate FDB entry for learning");
		ret = -ENOMEM;
		goto out;
	}

	memcpy(&entry->mac, mac, sizeof(struct net_eth_addr));
	entry->iface = iface;
	entry->flags = ETHERNET_BRIDGE_FDB_FLAG_DYNAMIC;
	entry->last_seen = k_uptime_get();

	sys_slist_prepend(&fdb_entries, &entry->node);
	fdb_count++;

	NET_DBG("FDB learned: %02x:%02x:%02x:%02x:%02x:%02x -> iface %d",
		mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3],
		mac->addr[4], mac->addr[5], net_if_get_by_iface(iface));

	/* Schedule aging work if not already running */
	k_work_reschedule(&fdb_aging_work,
			  K_SECONDS(CONFIG_NET_ETHERNET_BRIDGE_FDB_AGING_TIME));

out:
	k_mutex_unlock(&fdb_lock);
	return ret;
}
