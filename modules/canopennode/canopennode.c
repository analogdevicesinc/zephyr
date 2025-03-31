/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2024 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <canopennode.h>

#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>

#include <CANopen.h>
#include <storage/CO_storage.h>
#include "OD.h"

LOG_MODULE_REGISTER(canopennode, CONFIG_CANOPEN_LOG_LEVEL);

static void mainline_thread(void *p1, void *p2, void *p3);

/**
 * @brief CANopen sync thread.
 *
 * The CANopen real-time sync thread processes SYNC RPDOs and TPDOs
 * through the CANopenNode stack with an interval of 1 millisecond.
 *
 * @param p1 Unused
 * @param p2 Unused
 * @param p3 Unused
 */
static void sync_thread(void *p1, void *p2, void *p3);

static k_tid_t mainline_tid = NULL;
static k_tid_t sync_tid = NULL;

static struct k_thread mainline_thread_data;
static struct k_thread sync_thread_data;

K_THREAD_STACK_DEFINE(mainline_thread_stack, CONFIG_CANOPENNODE_MAINLINE_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(sync_thread_stack, CONFIG_CANOPENNODE_SYNC_THREAD_STACK_SIZE);

int canopen_init(struct canopen *co)
{
	int err;

	if (!device_is_ready(co->can_dev)) {
		LOG_ERR("CAN device not ready");
		return -ENODEV;
	}

#if CO_CONFIG_LEDS & CO_CONFIG_LEDS_ENABLE
	if (!gpio_is_ready_dt(&co->green_led)) {
		LOG_ERR("green LED device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&co->red_led)) {
		LOG_ERR("red LED device not ready");
		return -ENODEV;
	}

	gpio_pin_set_dt(&co->green_led, 0);
	gpio_pin_set_dt(&co->red_led, 0);
#endif /* CO_CONFIG_LEDS */

	co->CO = CO_new(NULL, NULL);

#if CO_CONFIG_STORAGE & CO_CONFIG_STORAGE_ENABLE
	err = canopen_storage_init(co);
	if (err < 0) {
		LOG_ERR("CO_storage_init failed (err %d)", err);
		return -EIO;
	}
#endif /* CO_CONFIG_STORAGE */

	err = canopen_reset_communication(co);
	if (err < 0) {
		LOG_ERR("failed to reset canopen communication (err %d)", err);
		return -EIO;
	}

	return 0;
}

int canopen_reset_communication(struct canopen *co)
{
	int err;
	uint32_t error_info;
	CO_t *CO = co->CO;
	uint16_t first_hb_time_ms = 0;
#if OD_CNT_NMT == 1
	OD_entry_t *first_hb_time;
#endif /* OD_CNT_NMT */
#if CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE
	OD_entry_t *identity;
	uint32_t vendor_id;
	uint32_t product_code;
	uint32_t revision_number;
	uint32_t serial_number;
#endif /* CO_CONFIG_LSS */

	if (mainline_tid != NULL) {
		/* lock all mutecies before aborting threads to ensure they are not locked when
		 * aborted */
		while (true) {
			if (k_mutex_lock(&CO->CANmodule->can_send_mutex, K_MSEC(100)) < 0 ||
			    k_mutex_lock(&CO->CANmodule->od_mutex, K_MSEC(100)) < 0 ||
			    k_mutex_lock(&CO->CANmodule->emcy_mutex, K_MSEC(100)) < 0) {
				continue;
			} else {
				break;
			}
		}

		k_thread_abort(mainline_tid);
		k_thread_abort(sync_tid);
		mainline_tid = NULL;
		sync_tid = NULL;

		k_mutex_unlock(&CO->CANmodule->can_send_mutex);
		k_mutex_unlock(&CO->CANmodule->od_mutex);
		k_mutex_unlock(&CO->CANmodule->emcy_mutex);
	}

	CO_CANmodule_disable(CO->CANmodule);
	err = CO_CANinit(CO, (void *)co->can_dev, co->bitrate);
	if (err != CO_ERROR_NO) {
		LOG_ERR("CO_CANinit failed (err %d)", err);
		return -EIO;
	}

#if CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE
	if ((identity = OD_find(OD, OD_H1018_IDENTITY_OBJECT)) == NULL ||
	    OD_get_u32(identity, 1, &vendor_id, true) != ODR_OK ||
	    OD_get_u32(identity, 2, &product_code, true) != ODR_OK ||
	    OD_get_u32(identity, 3, &revision_number, true) != ODR_OK ||
	    OD_get_u32(identity, 4, &serial_number, true) != ODR_OK) {
		LOG_ERR("object dictionary error at entry 0x1018");
		return -EINVAL;
	}

	CO_LSS_address_t lssAddress = {
		.identity =
			{
				.vendorID = vendor_id,
				.productCode = product_code,
				.revisionNumber = revision_number,
				.serialNumber = serial_number,
			},
	};

	err = CO_LSSinit(CO, &lssAddress, &co->node_id, &co->bitrate);
	if (err != CO_ERROR_NO) {
		LOG_ERR("CO_LSSinit failed (err %d)", err);
		return -EIO;
	}
#endif /* CO_CONFIG_LSS */

#if OD_CNT_NMT == 1
	if ((first_hb_time = OD_find(OD, OD_H1017_PRODUCER_HB_TIME)) == NULL ||
	    OD_get_u16(first_hb_time, 0, &first_hb_time_ms, true) != ODR_OK) {
		LOG_ERR("object dictionary error at entry 0x1017");
		return -EINVAL;
	}
#endif /* OD_CNT_NMT */

	err = CO_CANopenInit(
		CO, NULL, NULL, OD,
#if CO_CONFIG_EM & CO_CONFIG_EM_STATUS_BITS
		co->status_bits,
#else
		NULL,
#endif /* CO_CONFIG_EM */
		co->nmt_control, first_hb_time_ms, CONFIG_CANOPENNODE_SDO_SRV_TIMEOUT_TIME,
		CONFIG_CANOPENNODE_SDO_CLI_TIMEOUT_TIME,
		IS_ENABLED(CONFIG_CANOPENNODE_SDO_CLI_BLOCK), co->node_id, &error_info);
	if (err == CO_ERROR_OD_PARAMETERS) {
		LOG_ERR("object dictionary error at entry 0x%X", error_info);
		return -EINVAL;
	} else if (err != CO_ERROR_NO && (!(CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE) ||
					  err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)) {
		LOG_ERR("CO_CANopenInit failed (err %d)", err);
		return -EIO;
	}

	err = CO_CANopenInitPDO(CO, CO->em, OD, co->node_id, &error_info);
	if (err == CO_ERROR_OD_PARAMETERS) {
		LOG_ERR("object dictionary error at entry 0x%X", error_info);
		return -EINVAL;
	} else if (err != CO_ERROR_NO) {
		LOG_ERR("CO_CANopenInitPDO failed (err %d)", err);
		return -EIO;
	}

	CO_CANsetNormalMode(CO->CANmodule);

	mainline_tid = k_thread_create(&mainline_thread_data, mainline_thread_stack,
				       K_THREAD_STACK_SIZEOF(mainline_thread_stack),
				       mainline_thread, co, NULL, NULL,
				       CONFIG_CANOPENNODE_MAINLINE_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(mainline_tid, "canopen_mainline");
	sync_tid = k_thread_create(&sync_thread_data, sync_thread_stack,
				   K_THREAD_STACK_SIZEOF(sync_thread_stack), sync_thread, co, NULL,
				   NULL, CONFIG_CANOPENNODE_SYNC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(sync_tid, "canopen_sync");

	return 0;
}

static void mainline_thread(void *p1, void *p2, void *p3)
{
	struct canopen *co = (struct canopen *)p1;
	CO_t *CO = co->CO;
	CO_NMT_reset_cmd_t reset;
	uint32_t start;       /* cycles */
	uint32_t stop;        /* cycles */
	uint32_t delta;       /* cycles */
	uint32_t elapsed = 0; /* microseconds */
	uint32_t next;        /* microseconds */

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		next = 10000; /* 10 ms */
		start = k_cycle_get_32();

		reset = CO_process(CO, false, elapsed, &next);

#if CO_CONFIG_LEDS & CO_CONFIG_LEDS_ENABLE
#ifdef CONFIG_CANOPENNODE_LEDS_BICOLOR
		/* flavors red LED when both on */
		gpio_pin_set_dt(&co->->green_led, CO_LED_GREEN(CO->LEDs, CO_LED_CANopen) &&
							  !CO_LED_RED(CO->LEDs, CO_LED_CANopen));
#else
		gpio_pin_set_dt(&co->green_led, CO_LED_GREEN(CO->LEDs, CO_LED_CANopen));
#endif /* CONFIG_CANOPENNODE_LEDS_BICOLOR */
		gpio_pin_set_dt(&co->red_led, CO_LED_RED(CO->LEDs, CO_LED_CANopen));
#endif /* CO_CONFIG_LEDS */

#if CO_CONFIG_STORAGE & CO_CONFIG_STORAGE_ENABLE
		canopen_storage_process(co);
#endif /* CO_CONFIG_STORAGE */

		if (reset == CO_RESET_COMM) {
			LOG_INF("CANopen communication reset");
			canopen_reset_communication(co);
		} else if (reset == CO_RESET_APP) {
			LOG_INF("CANopen application reset");
			// log panic to flush logs before reboot
			log_panic();

			sys_reboot(SYS_REBOOT_COLD);
		}

		k_sleep(K_USEC(next));
		stop = k_cycle_get_32();
		delta = stop - start;
		elapsed = k_cyc_to_us_near32(delta);
	}
}

static void sync_thread(void *p1, void *p2, void *p3)
{
	struct canopen *co = (struct canopen *)p1;
	CO_t *CO = co->CO;
	uint32_t start;       /* cycles */
	uint32_t stop;        /* cycles */
	uint32_t delta;       /* cycles */
	uint32_t elapsed = 0; /* microseconds */
	uint32_t next;        /* microseconds */
	CO_SYNC_status_t sync;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		next = 1000;
		start = k_cycle_get_32();

		CO_LOCK_OD(CO->CANmodule);
		sync = CO_process_SYNC(CO, elapsed, &next);
		CO_process_RPDO(CO, sync == CO_SYNC_RX_TX, elapsed, &next);
		CO_process_TPDO(CO, sync == CO_SYNC_RX_TX, elapsed, &next);
		CO_UNLOCK_OD(CO->CANmodule);

		k_sleep(K_USEC(next));
		stop = k_cycle_get_32();
		delta = stop - start;
		elapsed = k_cyc_to_us_near32(delta);
	}
}
