/*
 * Copyright (c) 2024 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_SENSOR_LOG_LEVEL);

int main(void)
{
	int rc;

	const struct device *dev;
	struct sensor_value uv;

	dev = DEVICE_DT_GET_ONE(ams_as7331);
	if (!device_is_ready(dev)) {
		LOG_INF("Sensor: device not ready.");
		// return 0;
	}

	LOG_INF("Device is %p, Name is %s", dev, dev->name);

	while (1) {
		rc = sensor_sample_fetch(dev);
		if (rc == 0) {
			sensor_channel_get(dev, SENSOR_CHAN_UV, &uv);
			LOG_INF("Uv: %d.%06d", uv.val1, uv.val2);
		}

		k_sleep(K_MSEC(1000));
	}
	return 0;
}
