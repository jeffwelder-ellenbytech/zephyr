/*
 * Copyright (c) 2024 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_AS7331_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_AS7331_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Register Definitions
 */

/* Configuration Registers */
#define AS7331_OSR    (0x00)
#define AS7331_AGEN   (0x02)
#define AS7331_CREG1  (0x06)
#define AS7331_CREG2  (0x07)
#define AS7331_CREG3  (0x08)
#define AS7331_BREAK  (0x09)
#define AS7331_EDGES  (0x0A)
#define AS7331_OPTREG (0x0B)

/* Measurement Registers */
#define AS7331_STATUS   (0x00)
#define AS7331_TEMP     (0x01)
#define AS7331_MRES1    (0x02)
#define AS7331_MRES2    (0x03)
#define AS7331_MRES3    (0x04)
#define AS7331_OUTCONVL (0x05)
#define AS7331_OUTCONVH (0x06)

/* Device Info */
#define AS7331_ADDRESS          (0x74) /* If A0 = A1 = LOW */
#define AS7331_DEVICE_ID        (0x20) /* AGEN Register Upper Nibble */
#define AS7331_DEVICE_ID_NIBBLE (0x2)  /* AGEN Register Upper Nibble */

#define AS7331_DEVICE_MODE_NOP  (0x0)
#define AS7331_DEVICE_MODE_CFG  (0x2)
#define AS7331_DEVICE_MODE_MEAS (0x3)

/* Measurement Modes */
#define AS7331_CONT_MODE (0x00)
#define AS7331_CMD_MODE  (0x01)
#define AS7331_SYNS_MODE (0x02)
#define AS7331_SYND_MODE (0x03)

/* Operational State Register - 0x00 */
typedef union {
	uint8_t reg;
	struct {
		uint8_t dos: 3;      // Device Operating State   - OSR[2:0]
		uint8_t sw_res: 1;   // Software Reset           - OSR[3]
		uint8_t reserved: 2; // Reserved, don't write    - OSR[4:5]
		uint8_t pd: 1;       // Power Down Enabled       - OSR[6]
		uint8_t ss: 1;       // Start State              - OSR[7]
	} __packed;
} as7331_reg_cfg_osr_t;

/* AGEN (API Generation Register) - 0x02 */
typedef union {
	uint8_t reg;
	struct {
		uint8_t mut: 4;   // Increments when changes are made to the control registers.
				  // Defaults to 0b0001            - MUT[3:0]
		uint8_t devid: 4; // Always equals 0b0010  - DEVID[4:7]
	} __packed;
} as7331_reg_cfg_agen_t;

/* CREG1 (Configuration Register 1) - 0x06 */
typedef union {
	uint8_t reg;
	struct {
		uint8_t time: 4; // Conversion time  - CREG1[3:0]
		uint8_t gain: 4; // Sensor gain      - CREG1[7:4]
	} __packed;
} as7331_reg_cfg_creg1_t;

/* CREG2 (Configuration Register 2) - 0x07 */
typedef union {
	uint8_t reg;
	struct {
		uint8_t div: 3;    // Digital Divider value                             - CREG2[2:0]
		uint8_t en_div: 1; // Whether the digital divider is enabled            - CREG2[3]
		uint8_t reserved: 2; // Reserved                                        - CREG2[5:4]
		uint8_t en_tm: 1;    // Whether temperature is calculated in SYND mode. - CREG2[6]
		uint8_t reserved1: 1; // Reserved                                       - CREG2[7]
	} __packed;
} as7331_reg_cfg_creg2_t;

/* CREG3 (Configuration Register 3) - 0x08 */
typedef union {
	uint8_t reg;
	struct {
		uint8_t cclk: 2;      // Conversion clock selection       - CREG3[1:0]
		uint8_t reserved: 1;  // Reserved                         - CREG3[2]
		uint8_t rdyod: 1;     // Output mode of the ready pin     - CREG3[3]
		uint8_t sb: 1;        // Standby mode                     - CREG3[4]
		uint8_t reserved1: 1; // Reserved                         - CREG3[5]
		uint8_t mmode: 2;     // Measurement mode selection       - CREG3[7:6]
	} __packed;
} as7331_reg_cfg_creg3_t;

/* BREAK (Time Between Consecutive Measurements) - 0x09 */
/* EDGES (Number of SYN falling edges) - 0x0A */

/* OPTREG (Option Register) - 0x0B */
typedef union {
	uint8_t reg;
	struct {
		uint8_t init_idx: 1; // I2C repeat start mode flag   - OPTREG[0]
		uint8_t reserved: 7; // Reserved                     - OPTREG[7:1]
	} __packed;
} as7331_reg_cfg_optreg_t;

/*
 * Output Registers (16-bit)
 */

/* OSR/RO (Operational State/Status Register) - 0x00 */
typedef union {
	uint16_t reg;
	struct {
		as7331_reg_cfg_osr_t osr; // See OSR configuration register above - OSRSTAT[7:0]
		uint8_t powerstate: 1;    // Power down state.                         - OSRSTAT[8]
		uint8_t standbystate: 1;  // Standby mode state.                       - OSRSTAT[9]
		uint8_t notready: 1;  // Inverted value of the ready pin.             - OSRSTAT[10]
		uint8_t ndata: 1;     // Indicates new data available.                - OSRSTAT[11]
		uint8_t ldata: 1;     // Indicates data overwrite prior to retrieval. - OSRSTAT[12]
		uint8_t adcof: 1;     // OVF of at least one ADC channel.             - OSRSTAT[13]
		uint8_t mresof: 1;    // OVF of at least one of the MRES registers.   - OSRSTAT[14]
		uint8_t outconvof: 1; // OVF of the internal 24-bit time reference.   - OSRSTAT[15]
	} __packed;
} as7331_reg_output_osr_status_t;

/* TEMP  (Temperature Register)       - 0x01 */
/* MRES1 (UV-A Measurement Register)  - 0x02 */
/* MRES2 (UV-B Measurement Register)  - 0x03 */
/* MRES3 (UV-C Measurement Register)  - 0x04 */
/* OUTCONVL (Time Reference Register) - 0x05 */
/* OUTCONVH (Time Reference Register) - 0x06 */

// typedef enum {
// 	AS7331_CONT_MODE = 0x00, // continuous mode
// 	AS7331_CMD_MODE = 0x01,  // force mode, one-time measurement
// 	AS7331_SYNS_MODE = 0x02,
// 	AS7331_SYND_MODE = 0x03
// } MMODE;

// typedef enum {
// 	AS7331_1024 = 0x00, // internal clock frequency, 1.024 MHz, etc
// 	AS7331_2048 = 0x01,
// 	AS7331_4096 = 0x02,
// 	AS7331_8192 = 0x03
// } CCLK;

struct as7331_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec ready_gpio;
	struct gpio_dt_spec sync_gpio;
};

struct as7331_data {
	as7331_reg_cfg_osr_t osr;
	as7331_reg_cfg_agen_t agen;
	as7331_reg_cfg_creg1_t creg1;
	as7331_reg_cfg_creg2_t creg2;
	as7331_reg_cfg_creg3_t creg3;

	uint16_t uv_a;
	uint16_t uv_b;
	uint16_t uv_c;

	struct gpio_callback ready_gpio_cb;
	sensor_trigger_handler_t trigger_handler;

#ifdef CONFIG_AS7331_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_AS7331_TRIGGER_GLOBAL_THREAD
	struct k_work stat_work;
	struct k_work_delayable int_work;
#endif
};

#ifdef CONFIG_AS7331_TRIGGER
int as7331_trigger_init(const struct device *dev);
#endif
int as7331_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_AS7331_H_ */
