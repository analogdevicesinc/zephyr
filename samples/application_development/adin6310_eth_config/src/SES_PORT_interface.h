/*
* Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
*
* This software is proprietary and confidential to Analog Devices, Inc.
* and its licensors.
*/

#ifndef _SES_PORT_INTERFACE_H_
#define _SES_PORT_INTERFACE_H_

// Includes *******************************************************************

#include "SES_port_api.h"

typedef struct {
  int drvHdlIdx;
} SES_PORT_threadData_t;

typedef struct {
  uint32_t baseLocationId;     // Base location of the SPI module
  int chipSelect;              // Not used with configuration mode 0
  SES_PORT_SPI_mode_t spiMode; // Type of SPI (Quad, Double or Single)
} SES_PORT_ft4222InitParams_t;

struct SES_PORT_zephyr_spi_ip {
	uint32_t id;
	uint32_t cs;
	SES_PORT_SPI_mode_t spiMode;
};

SES_PORT_init_t SES_PORT_ETH_Init;
SES_PORT_release_t SES_PORT_ETH_Release;
SES_PORT_sendMessage_t SES_PORT_ETH_SendMessage;
SES_PORT_updateFilter_t SES_PORT_ETH_UpdateFilter;

// Forward declarations for SPI driver functions
SES_PORT_init_t SES_PORT_SPI_Init;
SES_PORT_release_t SES_PORT_SPI_Release;
SES_PORT_sendMessage_t SES_PORT_SPI_SendMessage;

#endif /* _SES_PORT_INTERFACE_H_ */
