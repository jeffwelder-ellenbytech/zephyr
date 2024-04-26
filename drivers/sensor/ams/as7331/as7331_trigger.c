#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/drivers/sensor/as7331.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(as7331_trigger, CONFIG_SENSOR_LOG_LEVEL);

static void as7331_handle_int_cb(const struct device *dev)
{
	const struct as7331_config *config = dev->config;

	LOG_INF("Got GPIO interrupt");

	// gpio_pin_interrupt_configure_dt(&config->ready_gpio, GPIO_INT_DISABLE);
	// k_sleep(K_MSEC(500));
	// gpio_pin_interrupt_configure_dt(&config->ready_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	// #if defined(CONFIG_AS7331_TRIGGER_OWN_THREAD)
	// 	k_sem_give(&data->trig_sem);
	// #elif defined(CONFIG_AS7331_TRIGGER_GLOBAL_THREAD)
	// 	k_work_schedule(&data->int_work, BQ25792_ADC_CONV_TIME_ALL);
	// #endif
}

static void as7331_int_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				     uint32_t pin_mask)
{
	// struct as7331_data *data = dev->data;
	const struct as7331_config *config = dev->config;

	// if ((pin_mask & BIT(config->ready_gpio.pin)) == 0U) {
	// 	return;
	// }

	as7331_handle_int_cb(dev);
}

int as7331_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct as7331_data *data = dev->data;

	if (SENSOR_TRIG_DATA_READY != trig->type) {
		return -ENOTSUP;
	}

	if (handler == NULL) {
		return -ENOTSUP;
	}

	data->trigger_handler = handler;
	return 0;
}

int as7331_trigger_init(const struct device *dev)
{
	const struct as7331_config *config = dev->config;
	struct as7331_data *data = dev->data;
	int ret;

	// data->dev = dev;

	if (config->ready_gpio.port == NULL) {
		LOG_DBG("instance '%s' doesn't support trigger mode", dev->name);
		return 0;
	}

	/* Get the GPIO device */
	if (!device_is_ready(config->ready_gpio.port)) {
		LOG_ERR("%s: device %s is not ready", dev->name, config->ready_gpio.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->ready_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure GPIO (ret %d)", ret);
		return -ENODEV;
	}
	// data->cb = NULL;

	// #if defined(CONFIG_AS7331_TRIGGER_OWN_THREAD)
	// 	k_sem_init(&data->trig_sem, 0, K_SEM_MAX_LIMIT);
	// 	k_thread_create(&data->thread, data->thread_stack, CONFIG_AS7331_THREAD_STACK_SIZE,
	// 			(k_thread_entry_t)as7331_thread_main, data, NULL, NULL,
	// 			K_PRIO_COOP(CONFIG_AS7331_THREAD_PRIORITY), 0, K_NO_WAIT);
	// 	k_thread_name_set(&data->thread, "AS7331 trigger");
	// #elif defined(CONFIG_AS7331_TRIGGER_GLOBAL_THREAD)
	// #if defined(CONFIG_AS7331_TRIGGER_GLOBAL_THREAD)
	// 	k_work_init_delayable(&data->int_work, as7331_int_work_handler);
	// #endif

	gpio_init_callback(&data->ready_gpio_cb, as7331_int_gpio_callback,
			   BIT(config->ready_gpio.pin));

	// if (gpio_add_callback(config->ready_gpio.port, &data->ready_gpio_cb) < 0) {
	if (gpio_add_callback_dt(&config->ready_gpio, &data->ready_gpio_cb) < 0) {
		LOG_ERR("Failed to set int gpio callback");
		return -EIO;
	}

	gpio_pin_interrupt_configure_dt(&config->ready_gpio, GPIO_INT_EDGE_TO_ACTIVE);

	if (gpio_pin_get_dt(&config->ready_gpio) > 0) {
		as7331_handle_int_cb(dev);
	}

	return 0;
}
