/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2024 National Taiwan University Racing Team
 * Copyright (c) 2025 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_CANOPENNODE_CO_DRIVER_H
#define ZEPHYR_MODULES_CANOPENNODE_CO_DRIVER_H

/*
 * Zephyr RTOS CAN driver interface and configuration for CANopenNode
 * CANopen protocol stack.
 *
 * See canopennode/301/CO_driver.h for API description.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/dsp/types.h> /* float32_t, float64_t */
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_CANOPENNODE_CONFIG_FILE
#include CONFIG_CANOPENNODE_CONFIG_FILE
#endif /* CONFIG_CANOPENNODE_CONFIG_FILE */

/* Use static variables instead of calloc() */
#define CO_USE_GLOBALS

/* Use Zephyr provided crc16 implementation */
#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE | CO_CONFIG_CRC16_EXTERNAL)

#ifdef CONFIG_LITTLE_ENDIAN
#define CO_LITTLE_ENDIAN
#else
#define CO_BIG_ENDIAN
#endif

#define CO_SWAP_16(x) sys_le16_to_cpu(x)
#define CO_SWAP_32(x) sys_le32_to_cpu(x)
#define CO_SWAP_64(x) sys_le64_to_cpu(x)

typedef bool bool_t;
typedef char char_t;
typedef unsigned char oChar_t;
typedef unsigned char domain_t;

BUILD_ASSERT(sizeof(float32_t) >= 4);
BUILD_ASSERT(sizeof(float64_t) >= 8);

typedef struct canopen_rx_msg {
	uint8_t data[8];
	uint16_t ident;
	uint8_t DLC;
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) (((CO_CANrxMsg_t *)msg)->ident)
#define CO_CANrxMsg_readDLC(msg)   (((CO_CANrxMsg_t *)msg)->DLC)
#define CO_CANrxMsg_readData(msg)  (((CO_CANrxMsg_t *)msg)->data)

typedef void (*CO_CANrxBufferCallback_t)(void *object, void *message);

typedef struct canopen_rx {
	int filter_id;
	void *object;
	CO_CANrxBufferCallback_t pFunct;
	uint16_t ident;
	uint16_t mask;
#ifdef CONFIG_CAN_ACCEPT_RTR
	bool rtr;
#endif /* CONFIG_CAN_ACCEPT_RTR */
} CO_CANrx_t;

typedef struct canopen_tx {
	uint8_t data[8];
	uint16_t ident;
	uint8_t DLC;
	bool_t rtr: 1;
	bool_t bufferFull: 1;
	bool_t syncFlag: 1;
} CO_CANtx_t;

typedef struct canopen_module {
	const struct device *can_dev;
	struct k_mutex can_send_mutex;
	struct k_mutex emcy_mutex;
	struct k_mutex od_mutex;
	struct k_work tx_retry_work;
	CO_CANrx_t *rx_array;
	CO_CANtx_t *tx_array;
	int sw_filter_id;
	uint16_t rx_size;
	uint16_t tx_size;
	uint16_t CANerrorStatus;
	bool_t configured: 1;
	bool_t CANnormal: 1;
	bool_t first_tx_msg: 1;
	bool_t use_software_filters : 1;
} CO_CANmodule_t;

typedef struct canopen_storage_entry {
	const char *key;
	void *addr;
	size_t len;
	uint8_t subIndexOD;
	uint8_t attr;
} CO_storage_entry_t;

#define CO_LOCK_CAN_SEND(mod)   k_mutex_lock(&mod->can_send_mutex, K_FOREVER)
#define CO_UNLOCK_CAN_SEND(mod) k_mutex_unlock(&mod->can_send_mutex)

#define CO_LOCK_EMCY(mod)   k_mutex_lock(&mod->emcy_mutex, K_FOREVER)
#define CO_UNLOCK_EMCY(mod) k_mutex_unlock(&mod->emcy_mutex)

#define CO_LOCK_OD(mod)   k_mutex_lock(&mod->od_mutex, K_FOREVER)
#define CO_UNLOCK_OD(mod) k_mutex_unlock(&mod->od_mutex)

/*
 * CANopenNode RX callbacks run in interrupt context, no memory
 * barrier needed.
 */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                         \
	do {                                                                                       \
		CO_MemoryBarrier();                                                                \
		rxNew = (void *)1L;                                                                \
	} while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                       \
	do {                                                                                       \
		CO_MemoryBarrier();                                                                \
		rxNew = NULL;                                                                      \
	} while (0)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODULES_CANOPENNODE_CO_DRIVER_H */
