/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sensor.h>
#include <stdio.h>

#define pow2(x) ((x) * (x))

#define SENSOR_READINGS		50
#define WAKEUP_THRESH 		0 /* (35 * SENSOR_G) */

static struct sensor_trigger trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

static double sqrt(double value)
{
	int i;
	double sqrt = value / 3;

	if (value <= 0) {
		return 0;
	}

	for (i = 0; i < 6; i++) {
		sqrt = (sqrt + value / sqrt) / 2;
	}

	return sqrt;
}

static int sensor_set_attribute(struct device *dev, enum sensor_channel chan,
				enum sensor_attribute attr, int value)
{
	struct sensor_value sensor_val;
	int ret;

	if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
		sensor_val.val1 = value;
	} else {
		sensor_val.val1 = value / 1000000;
		sensor_val.val2 = value % 1000000;
	}

	ret = sensor_attr_set(dev, chan, attr, &sensor_val);
	if (ret) {
		printf("sensor_attr_set failed ret %d\n", ret);
	}

	return ret;
}

K_SEM_DEFINE(sem, 0, 1);

static void trigger_handler(struct device *dev, struct sensor_trigger *trigger)
{
	static int cnt = SENSOR_READINGS;
	ARG_UNUSED(trigger);


	if (sensor_sample_fetch(dev)) {
		printf("sensor_sample_fetch failed\n");
		return;
	}

	if (WAKEUP_THRESH && (cnt-- == 0)) {
		cnt = SENSOR_READINGS;
		sensor_trigger_set(dev, &trig, NULL);

		sensor_set_attribute(dev, SENSOR_CHAN_ACCEL_XYZ,
				     SENSOR_ATTR_WAKEUP_THRESH, WAKEUP_THRESH);

		printf("\n --- Waiting for threshold wakeup event ---\n");

		if (sensor_trigger_set(dev, &trig, trigger_handler)) {
			printf("Could not set trigger\n");

		}
	}

	k_sem_give(&sem);
}

void main(void)
{
	struct sensor_value accel[3];
	double mag;
	int i;
	char meter[200];

	struct device *dev = device_get_binding(CONFIG_ADXL372_DEV_NAME);

	if (dev == NULL) {
		printf("Could not get %s device\n", CONFIG_ADXL372_DEV_NAME);
		return;
	}

	if (IS_ENABLED(CONFIG_ADXL372_PEAK_DETECT_MODE)) {
		trig.type = SENSOR_TRIG_THRESHOLD;
	}

	if (WAKEUP_THRESH) {
		sensor_set_attribute(dev, SENSOR_CHAN_ACCEL_XYZ,
				SENSOR_ATTR_SAMPLING_FREQUENCY,
				400);
	}

	if (IS_ENABLED(CONFIG_ADXL372_TRIGGER)) {
		if (sensor_trigger_set(dev, &trig, trigger_handler)) {
			printf("Could not set trigger\n");
			return;
		}
	}

	while (1) {
		if (IS_ENABLED(CONFIG_ADXL372_TRIGGER)) {
			if (IS_ENABLED(CONFIG_ADXL372_PEAK_DETECT_MODE)) {
				printf("Waiting for a threshold event\n");
			}
			k_sem_take(&sem, K_FOREVER);
		} else {
			if (sensor_sample_fetch(dev)) {
				printf("sensor_sample_fetch failed\n");
			}
		}

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

		if (IS_ENABLED(CONFIG_ADXL372_PEAK_DETECT_MODE)) {
			mag = sqrt(pow2(sensor_ms2_to_g(&accel[0])) +
				pow2(sensor_ms2_to_g(&accel[1])) +
				pow2(sensor_ms2_to_g(&accel[2])));

			for (i = 0; i <= mag && i < (sizeof(meter) - 1); i++) {
				meter[i] = '#';
			}

			meter[i] = '\0';

			printf("%6.2f g: %s\n", mag, meter);
		} else {
			printf("AX=%10.2f AY=%10.2f AZ=%10.2f (m/s^2)\n",
				sensor_value_to_double(&accel[0]),
				sensor_value_to_double(&accel[1]),
				sensor_value_to_double(&accel[2]));
		}

		if (!IS_ENABLED(CONFIG_ADXL372_TRIGGER)) {
			k_sleep(2000);
		}
	}
}
