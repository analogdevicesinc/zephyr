/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_LTC4296_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_LTC4296_H_

#include <zephyr/device.h>

enum ltc4296_state {
	LTC_UNLOCKED = 0,
	LTC_LOCKED
};

enum ltc4296_port {
	LTC_PORT0 = 0,
	LTC_PORT1,
	LTC_PORT2,
	LTC_PORT3,
	LTC_PORT4,
	LTC_NO_PORT
};

enum ltc4296_port_status {
	LTC_PORT_DISABLED = 0,
	LTC_PORT_ENABLED
};

enum ltc4296_pse_status {
	LTC_PSE_STATUS_DISABLED = 0,      /*  000b Port is disabled                */
	LTC_PSE_STATUS_SLEEPING,          /*  001b Port is in sleeping             */
	LTC_PSE_STATUS_DELIVERING,        /*  010b Port is delivering power        */
	LTC_PSE_STATUS_SEARCHING,         /*  011b Port is searching               */
	LTC_PSE_STATUS_ERROR,             /*  100b Port is in error                */
	LTC_PSE_STATUS_IDLE,              /*  101b Port is idle                    */
	LTC_PSE_STATUS_PREPDET,           /*  110b Port is preparing for detection */
	LTC_PSE_STATUS_UNKNOWN            /*  111b Port is in an unknown state     */
};

enum ltc4296_board_class {
	SPOE_CLASS10 = 0,
	SPOE_CLASS11,
	SPOE_CLASS12,
	SPOE_CLASS13,
	SPOE_CLASS14,
	SPOE_CLASS15,
	APL_CLASSA,
	APL_CLASSA_NOAUTONEG,
	APL_CLASSC,
	APL_CLASS3,
	PRODUCTION_POWER_TEST,
	APL_CLASSA_OLD_DEMO,
	SPOE_OFF,
	PRODUCTION_DATA_TEST,
	RESERVED,
	DEBUGMODE
};

enum ltc4296_config {
	LTC_CFG_SCCP_MODE = 0,
	LTC_CFG_APL_MODE,
	LTC_CFG_RESET
};

enum ltc4296_port_reg_offset_e {
	LTC_PORT_EVENTS   = 0,
	LTC_PORT_STATUS   = 2,
	LTC_PORT_CFG0     = 3,
	LTC_PORT_CFG1     = 4,
	LTC_PORT_ADCCFG   = 5,
	LTC_PORT_ADCDAT   = 6,
	LTC_PORT_SELFTEST = 7
};

enum adi_ltc_result
{
	ADI_LTC_SUCCESS = 0,                  /*!< Success                                                    */
	ADI_LTC_DISCONTINUE_SCCP,             /*!< Discontinue the SCCP configuration cycle.                  */
	ADI_LTC_SCCP_COMPLETE,                /*!< Complete SCCP configuration cycle.                         */
	ADI_LTC_SCCP_PD_DETECTION_FAILED,     /*!< PD Detection failed                                        */
	ADI_LTC_SCCP_PD_NOT_PRESENT,          /*!< SCCP  PD not present                                       */
	ADI_LTC_SCCP_PD_RES_INVALID,          /*!< PD Response is invalid                                     */
	ADI_LTC_SCCP_PD_PRESENT,              /*!< PD is present.                                             */
	ADI_LTC_SCCP_PD_CLASS_COMPATIBLE,     /*!< PD Class is compatible                                     */
	ADI_LTC_SCCP_PD_CLASS_NOT_SUPPORTED,  /*!< PD Class is out of range                                   */
	ADI_LTC_SCCP_PD_CLASS_NOT_COMPATIBLE, /*!< PD Class is not compatible                                 */
	ADI_LTC_SCCP_PD_LINE_NOT_HIGH,        /*!< PD line has not gone HIGH                                  */
	ADI_LTC_SCCP_PD_LINE_NOT_LOW,         /*!< PD line has not gone LOW                                   */
	ADI_LTC_SCCP_PD_CRC_FAILED,           /*!< CRC received from PD is incorrect                          */
	ADI_LTC_APL_COMPLETE,                 /*!< Complete APL configuration cycle.                          */
	ADI_LTC_DISCONTINUE_APL,              /*!< Discontinue the APL configuration cycle.                   */
	ADI_LTC_INVALID_ADC_VOLTAGE,          /*!< Invalid ADC Accumulation result.                           */
	ADI_LTC_INVALID_ADC_PORT_CURRENT,     /*!< Invalid ADC Port Current                                   */
	ADI_LTC_TEST_COMPLETE,                /*!< LTC Test complete.                                         */
	ADI_LTC_DISCONTINUE_TEST,             /*!< LTC Discontinue Test.                                      */
	ADI_LTC_TEST_FAILED,                  /*!< LTC Test Failed.                                           */
	ADI_LTC_INVALID_VIN                   /*!< VIN is invalid                                             */
};

struct ltc4296_vi
{
	int ltc4296_vin;
	int ltc4296_vout;
	int ltc4296_iout;
	bool  ltc4296_print_vin;
};


int ltc4296_reg_write(const struct device *dev, uint8_t reg, uint16_t data);
int ltc4296_reg_read(const struct device *dev, uint8_t reg, uint16_t *data);
int ltc4296_reset(const struct device *dev);

int ltc4296_get_port_addr(enum ltc4296_port port_no, enum ltc4296_port_reg_offset_e port_offset,
			  uint8_t *port_addr);

int ltc4296_clear_global_faults(const struct device *dev);
int ltc4296_clear_ckt_breaker(const struct device *dev);
int ltc4296_read_global_faults(const struct device *dev, uint16_t *g_events);
int ltc4296_unlock(const struct device *dev);
int ltc4296_is_locked(const struct device *dev, enum ltc4296_state *state);
int ltc4296_read_gadc(const struct device *dev, int *port_voltage_mv);
int ltc4296_set_gadc_vin(const struct device *dev);
int ltc4296_is_vin_valid(const struct device *dev, int port_vin_mv,
			 enum ltc4296_board_class ltcboard_class, bool *vin_valid);
int ltc4296_is_vout_valid(const struct device *dev, int port_vout_mv,
			  enum ltc4296_board_class ltcboard_class, bool *vout_valid);
int ltc4296_disable_gadc(const struct device *dev);
int ltc4296_read_port_events(const struct device *dev, enum ltc4296_port port_no,
			     uint16_t *port_events);
int ltc4296_clear_port_events(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_read_port_status(const struct device *dev, enum ltc4296_port port_no, uint16_t *port_status);
int ltc4296_is_port_disabled(const struct device *dev, enum ltc4296_port port_no,
			     enum ltc4296_port_status *port_chk);
int ltc4296_port_disable(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_is_port_deliver_pwr(const struct device *dev, enum ltc4296_port port_no,
				enum ltc4296_pse_status *pwr_status);
int ltc4296_is_port_pwr_stable(const struct device *dev, enum ltc4296_port port_no, bool *pwr_status);
int ltc4296_read_port_adc(const struct device *dev, enum ltc4296_port port_no, int *port_i_out_ma);
int ltc4296_port_prebias(const struct device *dev, enum ltc4296_port port_no, enum ltc4296_config mode);
int ltc4296_port_en(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_port_en_and_classification(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_is_pd_compatible(const struct device *dev, enum ltc4296_board_class pse_class,
			     uint16_t sccp_response_data, uint8_t *pd_class);
int ltc4296_set_port_mfvs(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_set_port_pwr(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_force_port_pwr(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_port_pwr_available(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_set_gadc_vout(const struct device *dev, enum ltc4296_port port_no);
int ltc4296_sccp_reset_pulse(const struct device *dev, uint8_t *pd_present);
int ltc4296_sccp_res_pd(const struct device *dev, uint16_t *res_data, uint8_t broadcast_addr,
			uint8_t read_scratchpad);
int ltc4296_sccp_pd(const struct device *dev, uint16_t *res_data, uint8_t broadcast_addr,
		    uint8_t read_scratchpad);
int ltc4296_print_global_faults(uint16_t g_events);
int ltc4296_print_port_events(enum ltc4296_port port_no, uint16_t port_events);
int ltc4296_chk_global_events(const struct device *dev);
int ltc4296_chk_port_events(const struct device *dev, enum ltc4296_port ltc4296_port);
int ltc4296_do_apl(const struct device *dev, enum ltc4296_board_class board_class,
		   enum ltc4296_port ltc4296_port, struct ltc4296_vi *ltc4296_vi);
int ltc4296_do_spoe_sccp(const struct device *dev, enum ltc4296_board_class board_class,
			 enum ltc4296_port ltc4296_port, struct ltc4296_vi *ltc4296_vi);
int ltc4296_retry_spoe_sccp(const struct device *dev, enum ltc4296_port ltc4296_port,
			    struct ltc4296_vi *ltc4296_vi);
int ltc4296_pwr_test(const struct device *dev, enum ltc4296_board_class board_class);

#endif  /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_LTC4296_H_ */
