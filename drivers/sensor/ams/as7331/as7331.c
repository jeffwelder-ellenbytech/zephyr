/*
 * Copyright (c) 2024 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Datasheet
 * https://look.ams-osram.com/m/1856fd2c69c35605/original/AS7331-Spectral-UVA-B-C-Sensor.pdf
 * Linux Driver
 * https://github.com/torvalds/linux/blob/ed30a4a51bb196781c8058073ea720133a65596f/drivers/iio/light/as73211.c
 * Sparkfun
 * https://github.com/sparkfun/SparkFun_AS7331_Arduino_Library/tree/main
 */

#define DT_DRV_COMPAT ams_as7331

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/sensor/as7331.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(as7331, CONFIG_SENSOR_LOG_LEVEL);

// SENSOR_CHAN_UV

static int as7331_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	int ret;
	const struct as7331_config *config = dev->config;
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = val;

	ret = i2c_write_dt(&config->i2c, buf, 2U);
	if (ret < 0) {
		LOG_ERR("Failed writing register 0x%02x", reg);
		return ret;
	}

	return 0;
}

static int as7331_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t size)
{
	int ret;
	const struct as7331_config *config = dev->config;

	ret = i2c_write_read_dt(&config->i2c, (uint8_t *)&reg, 1U, buf, size);
	if (ret < 0) {
		LOG_ERR("Failed reading register 0x%02x", reg);
		return ret;
	}

	return 0;
}

static int as7331_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as7331_data *data = dev->data;
	int ret;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_UV: {
		data->osr.dos = AS7331_DEVICE_MODE_NOP;
		data->osr.ss = 1; /* Turn on Start State */
		ret = as7331_reg_write(dev, AS7331_OSR, data->osr.reg);
		if (ret < 0) {
			return -EIO;
		}
		// k_msleep(3); // TODO FIX
		k_msleep(100); // TODO FIX
		uint8_t buf[2];

		as7331_reg_output_osr_status_t osr_status = {0};
		ret = as7331_reg_read(dev, AS7331_OSR, (uint8_t *)&osr_status.reg,
				      sizeof(osr_status));
		if (ret < 0) {
			return -EIO;
		}

		if (!osr_status.ndata || osr_status.osr.dos != AS7331_DEVICE_MODE_MEAS) {
			if (osr_status.osr.ss) {
				LOG_ERR("%d Measurement has not stopped", osr_status.reg);
				return -ETIME;
			}
			if (osr_status.notready) {
				LOG_ERR("%d Data is not ready", osr_status.reg);
				return -ENODATA;
			}
			if (osr_status.ndata) {
				LOG_ERR("%d No new data available", osr_status.reg);
				return -ENODATA;
			}
			if (osr_status.ldata) {
				LOG_ERR("%d Result buffer overrun", osr_status.reg);
				return -ENOBUFS;
			}
			if (osr_status.adcof) {
				LOG_ERR("%d ADC overflow", osr_status.reg);
				return -EOVERFLOW;
			}
			if (osr_status.mresof) {
				LOG_ERR("%d Measurement result overflow", osr_status.reg);
				return -EOVERFLOW;
			}
			if (osr_status.outconvof) {
				LOG_ERR("%d Timer overflow", osr_status.reg);
				return -EOVERFLOW;
			}
			return -EIO;
		}

		/* Read UV_A */
		ret = as7331_reg_read(dev, AS7331_MRES1, buf, sizeof(buf));
		if (ret < 0) {
			return -EIO;
		}
		data->uv_a = (uint16_t)((buf[1] << 8) | buf[0]);

		/* Read UV_B */
		ret = as7331_reg_read(dev, AS7331_MRES2, buf, sizeof(buf));
		if (ret < 0) {
			return -EIO;
		}
		data->uv_b = (uint16_t)((buf[1] << 8) | buf[0]);

		/* Read UV_C */
		ret = as7331_reg_read(dev, AS7331_MRES3, buf, sizeof(buf));
		if (ret < 0) {
			return -EIO;
		}
		data->uv_c = (uint16_t)((buf[1] << 8) | buf[0]);
		LOG_DBG("UV: %d, %d, %d", data->uv_a, data->uv_b, data->uv_c);
	} break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int as7331_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	if (val == NULL) {
		LOG_ERR("Argument of type sensor_value* cannot be null ");
		return -EINVAL;
	}

	struct as7331_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_UV: {
		val->val1 = (data->uv_a);
		val->val2 = (data->uv_b);

		// val->val1 = (data->uv_a) / 1000000;
		// val->val2 = (data->uv_a) % 1000000;
	} break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int as7331_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	// struct as7331_data *data = dev->data;
	return 0;
}

static int as7331_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, struct sensor_value *val)
{
	return 0;
}

#ifdef CONFIG_PM_DEVICE

static int as7331_pm_action(const struct device *dev, enum pm_device_action action)
{
	// const struct as7331_config *conf = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		// return as7331_set_shutdown_flag(dev, 1);

	case PM_DEVICE_ACTION_RESUME:
		// return as7331_set_shutdown_flag(dev, 0);

	default:
		return -ENOTSUP;
	}

	return 0;
}

#endif /* CONFIG_PM_DEVICE */

static int as7331_init(const struct device *dev)
{
	const struct as7331_config *config = dev->config;
	struct as7331_data *data = dev->data;
	int ret;

	// LOG_INF("as7331_init");
	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("Device not ready");
		return -ENODEV;
	}

#if CONFIG_AS7331_TRIGGER
	if (as7331_trigger_init(dev)) {
		LOG_ERR("Could not initialise interrupts");
		return -EIO;
	}
#endif

	/* Send a Software Reset */
	ret = as7331_reg_read(dev, AS7331_OSR, &data->osr.reg, sizeof(data->osr));
	if (ret < 0) {
		return ret;
	}
	data->osr.sw_res = 1; /* Set Software Reset */
	ret = as7331_reg_write(dev, AS7331_OSR, data->osr.reg);
	if (ret < 0) {
		return ret;
	}

	/* Wait for Power On (Using TSTARTPD from Datasheet) */
	k_msleep(2);

	// TODO Check if order of operations matters

	/* Set Configuration Mode */
	data->osr.pd = 0; /* Turn off Powerdown */
	data->osr.dos = AS7331_DEVICE_MODE_CFG;
	ret = as7331_reg_write(dev, AS7331_OSR, data->osr.reg);

	/* Verify Device ID */
	ret = as7331_reg_read(dev, AS7331_AGEN, &data->agen.reg, sizeof(data->agen));
	if (ret < 0 || data->agen.devid != AS7331_DEVICE_ID_NIBBLE) {
		LOG_ERR("Device ID Read Failed %d %d", ret, data->agen.devid);
		return -ENODEV;
	}

	/* Read CREG[0:3] */
	ret = as7331_reg_read(dev, AS7331_CREG1, &data->creg1.reg, sizeof(data->creg1));
	if (ret < 0) {
		return -EIO;
	}

	ret = as7331_reg_read(dev, AS7331_CREG2, &data->creg2.reg, sizeof(data->creg2));
	if (ret < 0) {
		return -EIO;
	}

	ret = as7331_reg_read(dev, AS7331_CREG3, &data->creg3.reg, sizeof(data->creg3));
	if (ret < 0) {
		return -EIO;
	}

	/* Set Measurement Mode to CMD Mode */
	data->creg3.mmode = AS7331_CMD_MODE; /* Command Mode (1-shot) */
	data->creg3.sb = 0;                  /* Turn off Standby*/
	ret = as7331_reg_write(dev, AS7331_CREG3, data->creg3.reg);
	if (ret < 0) {
		return ret;
	}

	ret = as7331_reg_read(dev, AS7331_OSR, &data->osr.reg, sizeof(data->osr));
	if (ret < 0) {
		return ret;
	}
	data->osr.pd = 0;                        /* Turn off Powerdown */
	data->osr.ss = 1;                        /* Turn on Start State */
	data->osr.dos = AS7331_DEVICE_MODE_MEAS; /* Switch to Measurement */
	ret = as7331_reg_write(dev, AS7331_OSR, data->osr.reg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static const struct sensor_driver_api as7331_api = {.sample_fetch = as7331_sample_fetch,
						    .channel_get = as7331_channel_get,
						    .attr_set = as7331_attr_set,
						    .attr_get = as7331_attr_get,
#ifdef CONFIG_AS7331_TRIGGER
						    .trigger_set = as7331_trigger_set
#endif
};

#define AS7331_DEFINE(n)                                                                           \
	static struct as7331_data as7331_data_##n;                                                 \
                                                                                                   \
	static const struct as7331_config as7331_config_##n = {                                    \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.ready_gpio = GPIO_DT_SPEC_INST_GET_OR(n, ready_gpios, {0}),                       \
		.sync_gpio = GPIO_DT_SPEC_INST_GET_OR(n, ready_gpios, {0}),                        \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(n, as7331_pm_action);                                             \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(n, as7331_init, PM_DEVICE_DT_INST_GET(n), &as7331_data_##n,   \
				     &as7331_config_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				     &as7331_api);

DT_INST_FOREACH_STATUS_OKAY(AS7331_DEFINE)
