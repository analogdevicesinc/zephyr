/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2024 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @defgroup CAN CAN BUS
 * @{
 * @}
 */

/**
 * @brief CANopen Network Stack
 * @defgroup canopen CANopen Network Stack
 * @ingroup CAN
 * @{
 */

#ifndef ZEPHYR_MODULES_CANOPENNODE_CANOPENNODE_H_
#define ZEPHYR_MODULES_CANOPENNODE_CANOPENNODE_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <CANopen.h>
#include <storage/CO_storage.h>
#include "OD.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANOPEN_STORAGE_ENTRY(PARAM, IDX, ATTR)                                                    \
	{                                                                                          \
		.key = #PARAM,                                                                     \
		.addr = (void *)&PARAM,                                                            \
		.len = sizeof(PARAM),                                                              \
		.subIndexOD = IDX,                                                                 \
		.attr = ATTR,                                                                      \
	}

#define CANOPEN_STORAGE_DEFINE(NAME, ...) static CO_storage_entry_t NAME[] = {__VA_ARGS__}

struct canopen {
	const struct device *can_dev;
	uint8_t node_id;
	uint16_t bitrate;
#if CO_CONFIG_EM & CO_CONFIG_EM_STATUS_BITS
	OD_entry_t *status_bits;
#endif /* CO_CONFIG_EM */
#if OD_CNT_NMT == 1
	CO_NMT_control_t nmt_control;
#endif /* OD_CNT_NMT */
#if CO_CONFIG_LEDS & CO_CONFIG_LEDS_ENABLE
	struct gpio_dt_spec green_led;
	struct gpio_dt_spec red_led;
#endif /* CO_CONFIG_LEDS */
#if CO_CONFIG_STORAGE & CO_CONFIG_STORAGE_ENABLE
	CO_storage_t storage; /* should be included in CO_t  */
	CO_storage_entry_t *storage_entries;
	size_t storage_entries_count;
#endif /* CO_CONFIG_STORAGE */
	CO_t *CO;
};

int canopen_init(struct canopen *co);

int canopen_reset_communication(struct canopen *co);

int canopen_storage_init(struct canopen *co);

int canopen_storage_process(struct canopen *co);

/**
 * @brief Attach CANopen object dictionary program download handlers.
 *
 * Attach CANopen program download functions to object dictionary
 * indexes 0x1F50, 0x1F51, 0x1F56, and 0x1F57. This function must be
 * called after calling CANopenNode `CO_init()`.
 *
 * @param nmt CANopenNode NMT object
 * @param sdo CANopenNode SDO server object
 * @param em  CANopenNode Emergency object
 */
void canopen_program_download_attach(CO_NMT_t *nmt, CO_SDOserver_t *sdo, CO_EM_t *em);

/**
 * @brief Indicate CANopen program download in progress
 *
 * Indicate that a CANopen program download is in progress.
 *
 * @param in_progress true if program download is in progress, false otherwise
 */
void canopen_leds_program_download(bool in_progress);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_MODULES_CANOPENNODE_CANOPENNODE_H_ */
