/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2024 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <CANopen.h>
#include <storage/CO_storage.h>
#include "OD.h"

#include <canopennode.h>

LOG_MODULE_REGISTER(canopen_storage, CONFIG_CANOPEN_LOG_LEVEL);

#if CO_CONFIG_STORAGE & CO_CONFIG_STORAGE_ENABLE

static ODR_t store(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule);

static ODR_t restore(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule);

static int canopen_settings_set(const char *key, size_t len_rd, settings_read_cb read_cb,
				void *cb_arg);

/**
 * @brief Global pointer to the canopen instance since we are unable to retrieve it from settings
 * handlers.
 */
static struct canopen *coo;
static bool settings_set_err = false;

SETTINGS_STATIC_HANDLER_DEFINE(canopen, CONFIG_CANOPENNODE_STORAGE_SUBTREE, NULL,
			       canopen_settings_set, NULL, NULL);

int canopen_storage_init(struct canopen *co)
{
	int err;
	CO_t *CO = co->CO;
	OD_entry_t *OD_1010;
	OD_entry_t *OD_1011;

	coo = co;

	co->storage.enabled = false;

	OD_1010 = OD_find(OD, OD_H1010_STORE_PARAMETERS);
	if (OD_1010 == NULL) {
		LOG_ERR("object dictionary error at entry 0x1010");
		return -EINVAL;
	}

	OD_1011 = OD_find(OD, OD_H1011_RESTORE_DEFAULT);
	if (OD_1011 == NULL) {
		LOG_ERR("object dictionary error at entry 0x1011");
		return -EINVAL;
	}

	err = CO_storage_init(&co->storage, CO->CANmodule, OD_1010, OD_1011, store, restore,
			      co->storage_entries, co->storage_entries_count);
	if (err != CO_ERROR_NO) {
		LOG_ERR("CO_storage_init failed (err %d)", err);
		return -EIO;
	}

	err = settings_subsys_init();
	if (err < 0) {
		LOG_ERR("failed to initialize settings subsystem (err %d)", err);
		return err;
	}

	err = settings_load_subtree(CONFIG_CANOPENNODE_STORAGE_SUBTREE);
	if (err < 0) {
		LOG_ERR("failed to load settings subtree (err %d)", err);
		return err;
	}

	// settings_set_err is set if error occurs during settings loading
	if (settings_set_err) {
		LOG_ERR("failed to load some settings data");
		return -EIO;
	}

	co->storage.enabled = true;

	return 0;
}

int canopen_storage_process(struct canopen *co)
{
	int err;

	for (size_t i = 0; i < co->storage_entries_count; i++) {
		if (co->storage_entries[i].attr & CO_storage_auto) {
			err = store(&co->storage_entries[i], co->CO->CANmodule);
			if (err != ODR_OK) {
				return err;
			}
		}
	}

	return 0;
}

static ODR_t store(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
	int err;
	char key[100];

	ARG_UNUSED(CANmodule);

	snprintf(key, sizeof(key), CONFIG_CANOPENNODE_STORAGE_SUBTREE "/%s", entry->key);
	err = settings_save_one(key, entry->addr, entry->len);
	if (err < 0) {
		LOG_ERR("failed to save settings data %s (err %d)", entry->key, err);
		return ODR_HW;
	}

	return ODR_OK;
}

static ODR_t restore(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
	int err;
	char key[100];

	ARG_UNUSED(CANmodule);

	snprintf(key, sizeof(key), CONFIG_CANOPENNODE_STORAGE_SUBTREE "/%s", entry->key);
	err = settings_delete(key);
	if (err < 0) {
		LOG_ERR("failed to delete settings data %s (err %d)", entry->key, err);
		return ODR_HW;
	}

	return ODR_OK;
}

static int canopen_settings_set(const char *key, size_t len_rd, settings_read_cb read_cb,
				void *cb_arg)
{
	const char *next;
	ssize_t len;
	size_t i;

	for (i = 0; i < coo->storage_entries_count; i++) {
		if (settings_name_steq(key, coo->storage_entries[i].key, &next) && !next) {
			len = read_cb(cb_arg, coo->storage_entries[i].addr,
				      coo->storage_entries[i].len);
			if (len < 0) {
				LOG_ERR("failed to load settings data %s (err %d)",
					coo->storage_entries[i].key, len);
				settings_set_err = true;
				return len;
			}

			return 0;
		}
	}

	return -ENOENT;
}

#endif /* CO_CONFIG_STORAGE */
