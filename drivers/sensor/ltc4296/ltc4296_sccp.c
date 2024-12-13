#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor/ltc4296.h>
#include "ltc4296.h"
#include "ltc4296_sccp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LTC4296_SCCP, CONFIG_SENSOR_LOG_LEVEL);

#define SCCP_CLASS_SIZE 16

#define SCCP_TYPE_MASK         0xF000
#define SCCP_CLASS_TYPE_MASK   0x0FFF

const uint8_t class_compatibility[16][16] = {
//PD								PSE
//			0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15
/*0*/		{1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
/*1*/		{0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
/*2*/		{0,	0,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
/*3*/		{0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
/*4*/		{0,	0,	0,	0,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0},
/*5*/		{0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0},
/*6*/		{0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0},
/*7*/		{0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0},
/*8*/		{0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	0,	0,	0,	0},
/*9*/		{0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0},
/*10*/		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0},
/*11*/		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	0},
/*12*/		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0},
/*13*/		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1},
/*14*/		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1},
/*15*/		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1}
};

uint16_t sccp_classes[SCCP_CLASS_SIZE] = {
    0x3FE, // class 0
    0x3FD, // class 1
    0x3FB, // class 2
    0x3F7, // class 3
    0x3F7, // class 4
    0x3EF, // class 5
    0x3DF, // class 6
    0x3BF, // class 7
    0x37F, // class 8
    0x2FF, // class 9
    0x001, // class 10 /* Supported by D2Z Board*/
    0x002, // class 11 /* Supported by D2Z Board*/
    0x003, // class 12 /* Supported by D2Z Board*/
    0x004, // class 13 /* Supported by D2Z Board*/
    0x005, // class 14 /* Supported by D2Z Board*/
    0x006  // class 15 /* Supported by D2Z Board*/
};

const uint8_t sccp_types[5] = { 0xE, 0xD, 0xB, 0x7, 0xC };

static uint8_t get_crc(uint8_t* buf)
{
	uint8_t byte, bit;
	uint8_t x3,x4,x7, in;

	uint8_t crc = 0;
	for(byte = 0; byte < 2; byte++)
	{
		for(bit=0; bit<8; bit++)
		{
			/* save some bits before shifting register */
			x3 = (crc >> 3) & 0x01;
			x4 = (crc >> 4) & 0x01;
			x7 = (crc >> 7) & 0x01;
			in = (buf[byte] >> bit) & 0x01;
			in ^= x7;
			/* shift the register */
			crc  = (crc << 1) | in;

			/* clear bits 4 & 5 */
			crc &= ~(0x30);

			/* replace bits with xor of 'in' and prev bit */
			uint8_t temp = x3 ^ in;
			crc |= (temp << 4);
			temp = x4 ^ in;
			crc |= (temp << 5);
		}
	}
	return crc;
}

uint8_t READ_LINE(const struct device *dev)
{
	const struct ltc4296_dev_config *cfg = dev->config;
	struct ltc4296_data *data = dev->data;

	return gpio_pin_get_dt(&cfg->port_config[data->current_sccp_port].sccpi_gpio);
}

void PULL_DOWN_LINE(const struct device *dev)
{
	const struct ltc4296_dev_config *cfg = dev->config;
	struct ltc4296_data *data = dev->data;

	gpio_pin_set_dt(&cfg->port_config[data->current_sccp_port].sccpo_gpio, 0);
}

void RELEASE_LINE(const struct device *dev)
{
	const struct ltc4296_dev_config *cfg = dev->config;
	struct ltc4296_data *data = dev->data;

	gpio_pin_set_dt(&cfg->port_config[data->current_sccp_port].sccpo_gpio, 1);
}

void write_bit(const struct device *dev, uint8_t bit)
{
	PULL_DOWN_LINE(dev);
	if (bit)
	{
		k_busy_wait(300);
		RELEASE_LINE(dev);
		k_busy_wait(2150);//T_WRITESLOT-T_REC-T_W1L = 2.15
	}
	else
	{
		k_busy_wait(2450); //TW0L + 0.45 = 2.45
		RELEASE_LINE(dev);
	}

	/*Recovery time after every bit transmit */
	k_busy_wait(320);//0.32
	return;
}

void transmit_byte(const struct device *dev, uint8_t tx_byte)
{
	uint8_t bit_pos = 0;
	while (bit_pos < 8)
	{
		uint8_t bit = (tx_byte>>bit_pos) & 0x01;
		write_bit(dev, bit);
		bit_pos++;
	}
	return;
}


uint8_t read_bit(const struct device *dev)
{
	uint8_t bit;

	PULL_DOWN_LINE(dev);
	k_busy_wait(300); //T_W1L =0.3

	RELEASE_LINE(dev);

	k_busy_wait(700); //T_MSR-T_W1L = 1.225-0.3 = 700

	bit = READ_LINE(dev);

	k_busy_wait(2000); //T_READSLOT-T_MSR = 3-1 =2

	k_busy_wait(320); //T_REC
	return bit;
}

void receive_response(const struct device *dev, uint8_t* buf)
{
	volatile uint8_t rx_byte = 0;
	volatile uint8_t bytes_rxd = 0;
	volatile uint8_t bit_pos = 0;
    	uint8_t sccp_buf[3] = {0, 0, 0};

	while (bytes_rxd < 3)
	{
		rx_byte = 0;
		bit_pos = 0;
		while(bit_pos < 8)
		{
			uint8_t bit = read_bit(dev);
			rx_byte |= (bit<<bit_pos);
			bit_pos++;
		}
		sccp_buf[bytes_rxd] = rx_byte;
		bytes_rxd++;
		k_sleep(K_MSEC(5));
	}
	buf[0]=sccp_buf[0];
	buf[1]=sccp_buf[1];
	buf[2]=sccp_buf[2];
	return;
}

int sccp_reset_pulse(const struct device *dev)
{
	uint8_t level = 0;
	//uint32_t i=0,j,k;
	int ret= ADI_LTC_SCCP_PD_PRESENT;

	/* check if the line is high before reset pulse */
	if(!READ_LINE(dev))
	{
		return ADI_LTC_SCCP_PD_LINE_NOT_HIGH;
	}

	/* assert pulse */
	PULL_DOWN_LINE(dev);

	/* check to make sure line is actually getting pulled down (protect pull down fet) */
	k_sleep(K_MSEC(3));

	if(READ_LINE(dev))
	{
		/* release because fet must be pulling down against stronger source than a classification v source */
		RELEASE_LINE(dev);
		return ADI_LTC_SCCP_PD_LINE_NOT_LOW;
	}

	k_sleep(K_MSEC(T_RSTL_NOM / 1000 - 3));

	RELEASE_LINE(dev);

	k_sleep(K_MSEC(T_MSP / 1000));

	/* look for presence pulse */
	level = READ_LINE(dev);

	k_sleep(K_MSEC(4));

	if(level == HIGH)
		ret = ADI_LTC_SCCP_PD_NOT_PRESENT;
	else if(level == LOW)
		ret = ADI_LTC_SCCP_PD_PRESENT;

	return ret;
}

int sccp_read_write_pd(const struct device *dev, uint8_t addr, uint8_t cmd, uint8_t* buf)
{
	int ret;

	ret = sccp_reset_pulse(dev);
	if(ret == ADI_LTC_SCCP_PD_NOT_PRESENT)
		return ADI_LTC_SCCP_PD_NOT_PRESENT; //PD is not present
	else if(ret == ADI_LTC_SCCP_PD_LINE_NOT_LOW)
		return ADI_LTC_SCCP_PD_LINE_NOT_LOW;
	else if(ret == ADI_LTC_SCCP_PD_LINE_NOT_HIGH)
		return ADI_LTC_SCCP_PD_LINE_NOT_HIGH;

	k_sleep(K_MSEC(5));
	transmit_byte(dev, addr);
	k_sleep(K_MSEC(5));
	transmit_byte(dev, cmd);
	k_sleep(K_MSEC(5));
	receive_response(dev, buf);

	/* Check if the received data from PD is valid */
	if (get_crc(buf) != buf[2] )
	{
		LOG_ERR("PD CRC Error, CRC is 0x%x \n\r", buf[2]);
		return ADI_LTC_SCCP_PD_CRC_FAILED; /* Wrong CRC*/
	}
	return ADI_LTC_SCCP_PD_PRESENT;
}

int sccp_is_pd(const struct device *dev, uint8_t pse_class, uint16_t sccp_response_data, uint8_t *pd_class)
{
	uint8_t i,sccp_type;
	uint16_t val, pd_type=0;
	int ret;

	/* bottom 12 bits of CLASS_TYPE_INFO codes for PD power classes 0 through 15 */
	val = (sccp_response_data & SCCP_CLASS_TYPE_MASK);

	/*we support classes 10- Classes15*/
	for(i = 0; i < SCCP_CLASS_SIZE; i++)
	{
		if(sccp_classes[i] == val)
		{
			pd_type = i;
			break;
		}
	}

	/* Check for SCCP type; MSB 4 bits */
	sccp_type = ( (sccp_response_data & SCCP_TYPE_MASK) >> 12);
	if(sccp_type != sccp_types[4])
	{
		*pd_class = i;
		return ADI_LTC_SCCP_PD_CLASS_NOT_SUPPORTED;
	}

	/* PD Type supported are Class10 to Class15*/
	if( (pd_type < 10) || (pd_type > 15) )
	{
		/* The PD class is not supported by the D2Z board*/
		*pd_class = i;
		return ADI_LTC_SCCP_PD_CLASS_NOT_SUPPORTED;
	}

	if( class_compatibility[pd_type][pse_class+10] == 1)
	{
		ret = ADI_LTC_SCCP_PD_CLASS_COMPATIBLE;
		*pd_class = i;
	}
	else
	{
		ret = ADI_LTC_SCCP_PD_CLASS_NOT_SUPPORTED;
		*pd_class = i;
	}

   	return ret;
}
