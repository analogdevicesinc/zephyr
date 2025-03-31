/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2024 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <CANopen.h>
#include <301/crc16-ccitt.h>
#include <canopennode.h>

LOG_MODULE_REGISTER(canopen_driver, CONFIG_CANOPEN_LOG_LEVEL);

static void canopen_tx_retry(struct k_work *work);

static void canopen_detach_all_rx_filters(CO_CANmodule_t *CANmodule)
{
	uint16_t i;

	if (!CANmodule || !CANmodule->rx_array || !CANmodule->configured) {
		return;
	}

	for (i = 0U; i < CANmodule->rx_size; i++) {
		if (CANmodule->rx_array[i].filter_id != -ENOSPC) {
			can_remove_rx_filter(CANmodule->can_dev, CANmodule->rx_array[i].filter_id);
			CANmodule->rx_array[i].filter_id = -ENOSPC;
		}
	}
}

static void canopen_rx_callback(const struct device *dev, struct can_frame *frame, void *arg)
{
	CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)arg;
	CO_CANrxMsg_t rxMsg;
	CO_CANrx_t *buffer;
	int i;

	ARG_UNUSED(dev);

	/* Loop through registered rx buffers in priority order */
	for (i = 0; i < CANmodule->rx_size; i++) {
		buffer = &CANmodule->rx_array[i];

		if (buffer->filter_id == -ENOSPC || buffer->pFunct == NULL) {
			continue;
		}

		if (((frame->id ^ buffer->ident) & buffer->mask) == 0U) {
#ifdef CONFIG_CAN_ACCEPT_RTR
			if (buffer->rtr && ((frame->flags & CAN_FRAME_RTR) == 0U)) {
				continue;
			}
#endif /* CONFIG_CAN_ACCEPT_RTR */
			rxMsg.ident = frame->id;
			rxMsg.DLC = frame->dlc;
			memcpy(rxMsg.data, frame->data, frame->dlc);
			buffer->pFunct(buffer->object, &rxMsg);
			break;
		}
	}
}

static void canopen_tx_callback(const struct device *dev, int error, void *arg)
{
	CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)arg;

	ARG_UNUSED(dev);

	if (error == 0) {
		CANmodule->first_tx_msg = false;
	}

	k_work_submit(&CANmodule->tx_retry_work);
}

static void canopen_tx_retry(struct k_work *work)
{
	int err;
	CO_CANmodule_t *CANmodule = CONTAINER_OF(work, CO_CANmodule_t, tx_retry_work);
	struct can_frame frame;
	CO_CANtx_t *buffer;
	uint16_t i;

	ARG_UNUSED(work);

	memset(&frame, 0, sizeof(frame));

	CO_LOCK_CAN_SEND(CANmodule);

	for (i = 0; i < CANmodule->tx_size; i++) {
		buffer = &CANmodule->tx_array[i];
		if (buffer->bufferFull) {
			frame.id = buffer->ident;
			frame.dlc = buffer->DLC;
			frame.flags |= (buffer->rtr ? CAN_FRAME_RTR : 0);
			memcpy(frame.data, buffer->data, buffer->DLC);

			err = can_send(CANmodule->can_dev, &frame, K_NO_WAIT, canopen_tx_callback,
				       CANmodule);
			if (err == -EAGAIN) {
				break;
			} else if (err != 0) {
				LOG_ERR("failed to send CAN frame (err %d)", err);
			}

			buffer->bufferFull = false;
		}
	}

	CO_UNLOCK_CAN_SEND(CANmodule);
}

void CO_CANsetConfigurationMode(void *CANptr)
{
	const struct device *can_dev = CANptr;
	int err;

	err = can_stop(can_dev);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("failed to stop CAN interface (err %d)", err);
	}
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
	int err;

	err = can_start(CANmodule->can_dev);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("failed to start CAN interface (err %d)", err);
		return;
	}

	CANmodule->CANnormal = true;
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[],
				   uint16_t rxSize, CO_CANtx_t txArray[], uint16_t txSize,
				   uint16_t CANbitRate)
{
	struct device *can_dev = (struct device *)CANptr;
	uint16_t i;
	int err;
	int max_filters;

	if (!CANmodule || !rxArray || !txArray || !CANptr) {
		LOG_ERR("failed to initialize CAN module");
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	memset(CANmodule, 0, sizeof(*CANmodule));
	CANmodule->can_dev = can_dev;
	CANmodule->rx_array = rxArray;
	CANmodule->rx_size = rxSize;
	CANmodule->tx_array = txArray;
	CANmodule->tx_size = txSize;
	CANmodule->CANnormal = false;
	CANmodule->first_tx_msg = true;
	CANmodule->CANerrorStatus = 0;

	max_filters = can_get_max_filters(can_dev, false);
	if (max_filters != -ENOSYS) {
		if (max_filters < 0) {
			LOG_ERR("unable to determine number of CAN RX filters");
			return CO_ERROR_SYSCALL;
		}

		if (rxSize > max_filters) {
			LOG_ERR("insufficient number of concurrent CAN RX filters"
				" (needs %d, %d available)",
				rxSize, max_filters);
			return CO_ERROR_OUT_OF_MEMORY;
		} else if (rxSize < max_filters) {
			LOG_DBG("excessive number of concurrent CAN RX filters enabled"
				" (needs %d, %d available)",
				rxSize, max_filters);
		}
	}

	err = can_set_bitrate(can_dev, KHZ(CANbitRate));
	if (err < 0) {
		LOG_ERR("failed to configure CAN bitrate (err %d)", err);
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	err = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (err < 0) {
		LOG_ERR("failed to configure CAN interface (err %d)", err);
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	for (i = 0U; i < rxSize; i++) {
		rxArray[i].ident = 0U;
		rxArray[i].pFunct = NULL;
		rxArray[i].filter_id = -ENOSPC;
	}

	for (i = 0U; i < txSize; i++) {
		txArray[i].bufferFull = false;
	}

	k_mutex_init(&CANmodule->can_send_mutex);
	k_mutex_init(&CANmodule->emcy_mutex);
	k_mutex_init(&CANmodule->od_mutex);
	k_work_init(&CANmodule->tx_retry_work, canopen_tx_retry);

	CANmodule->configured = true;

	return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
	int err;

	if (!CANmodule || !CANmodule->can_dev) {
		return;
	}

	canopen_detach_all_rx_filters(CANmodule);

	err = can_stop(CANmodule->can_dev);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("failed to disable CAN interface (err %d)", err);
	}
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident,
				    uint16_t mask, bool_t rtr, void *object,
				    CO_CANrxBufferCallback_t pFunct)
{
	struct can_filter filter;
	CO_CANrx_t *buffer;

	if (CANmodule == NULL) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	if (!pFunct || (index >= CANmodule->rx_size)) {
		LOG_ERR("failed to initialize CAN rx buffer, illegal argument");
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	buffer = &CANmodule->rx_array[index];
	buffer->object = object;
	buffer->pFunct = pFunct;
	buffer->ident = ident;
	buffer->mask = mask;

#ifndef CONFIG_CAN_ACCEPT_RTR
	if (rtr) {
		LOG_ERR("request for RTR frames, but RTR frames are rejected");
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}
#else  /* !CONFIG_CAN_ACCEPT_RTR */
	buffer->rtr = rtr;
#endif /* CONFIG_CAN_ACCEPT_RTR */

	filter.flags = 0U;
	filter.id = ident;
	filter.mask = mask;

	if (buffer->filter_id != -ENOSPC) {
		can_remove_rx_filter(CANmodule->can_dev, buffer->filter_id);
	}

	buffer->filter_id =
		can_add_rx_filter(CANmodule->can_dev, canopen_rx_callback, CANmodule, &filter);
	if (buffer->filter_id == -ENOSPC) {
		LOG_ERR("failed to add CAN rx callback, no free filter");
		return CO_ERROR_OUT_OF_MEMORY;
	}

	return CO_ERROR_NO;
}

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident,
			       bool_t rtr, uint8_t noOfBytes, bool_t syncFlag)
{
	CO_CANtx_t *buffer;

	if (CANmodule == NULL) {
		return NULL;
	}

	if (index >= CANmodule->tx_size) {
		LOG_ERR("failed to initialize CAN rx buffer, illegal argument");
		return NULL;
	}

	buffer = &CANmodule->tx_array[index];
	buffer->ident = ident;
	buffer->rtr = rtr;
	buffer->DLC = noOfBytes;
	buffer->bufferFull = false;
	buffer->syncFlag = syncFlag;

	return buffer;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	CO_ReturnError_t ret = CO_ERROR_NO;
	struct can_frame frame;
	int err;

	if (!CANmodule || !CANmodule->can_dev || !buffer) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	memset(&frame, 0, sizeof(frame));

	CO_LOCK_CAN_SEND(CANmodule);

	if (buffer->bufferFull) {
		// if (!CANmodule->first_tx_msg) {
		// 	CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
		// }
		// buffer->bufferFull = false;
		// ret = CO_ERROR_TX_OVERFLOW;

		LOG_WRN("TX buffer full, discarding message with ID 0x%03X", buffer->ident);

		CO_UNLOCK_CAN_SEND(CANmodule);
		return CO_ERROR_TX_OVERFLOW;
	}

	frame.id = buffer->ident;
	frame.dlc = buffer->DLC;
	frame.flags = (buffer->rtr ? CAN_FRAME_RTR : 0);
	memcpy(frame.data, buffer->data, buffer->DLC);

	err = can_send(CANmodule->can_dev, &frame, K_NO_WAIT, canopen_tx_callback, CANmodule);
	if (err == -EAGAIN) {
		buffer->bufferFull = true;
	} else if (err != 0) {
		LOG_ERR("failed to send CAN frame (err %d)", err);
		ret = CO_ERROR_TX_UNCONFIGURED;
	}

	CO_UNLOCK_CAN_SEND(CANmodule);

	return ret;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
	bool_t tpdoDeleted = false;
	CO_CANtx_t *buffer;
	uint16_t i;

	if (!CANmodule) {
		return;
	}

	CO_LOCK_CAN_SEND(CANmodule);

	for (i = 0; i < CANmodule->tx_size; i++) {
		buffer = &CANmodule->tx_array[i];
		if (buffer->bufferFull && buffer->syncFlag) {
			buffer->bufferFull = false;
			tpdoDeleted = true;
		}
	}

	CO_UNLOCK_CAN_SEND(CANmodule);

	if (tpdoDeleted) {
		CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
	}
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	int err;

	err = can_get_state(CANmodule->can_dev, &state, &err_cnt);
	if (err != 0) {
		LOG_ERR("failed to get CAN controller state (err %d)", err);
		return;
	}

	if (state == CAN_STATE_BUS_OFF) {
		CANmodule->CANerrorStatus |= CO_CAN_ERRTX_BUS_OFF;
	} else {
		/* clear flags before recaculation */
		CANmodule->CANerrorStatus &=
			~(CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE |
			  CO_CAN_ERRTX_WARNING | CO_CAN_ERRRX_WARNING);

		if (err_cnt.tx_err_cnt >= 96U) {
			CANmodule->CANerrorStatus |= CO_CAN_ERRTX_WARNING;
		}

		if (err_cnt.rx_err_cnt >= 96U) {
			CANmodule->CANerrorStatus |= CO_CAN_ERRRX_WARNING;
		}

		if (err_cnt.tx_err_cnt >= 128U && !CANmodule->first_tx_msg) {
			CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PASSIVE;
		}

		if (err_cnt.rx_err_cnt >= 128U) {
			CANmodule->CANerrorStatus |= CO_CAN_ERRRX_PASSIVE;
		}

		/* TODO: Zephyr lacks an API for reading the rx mailbox overflow counter. */
	}
}
