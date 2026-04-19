/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * VL53L5CX Platform Adaptation Layer for Zephyr
 *
 * This file implements the platform functions required by the ST VL53L5CX
 * ULD (Ultra Lite Driver). The ULD expects the customer to provide I2C
 * read/write, delay, and byte-swap functions.
 *
 * Key difference from generic platform.h:
 *   VL53L5CX uses 16-bit register addresses (not 8-bit like VL53L0X).
 *   So we must send 2 address bytes before data in every I2C transaction.
 */

#ifndef VL53L5CX_PLATFORM_ZEPHYR_H_
#define VL53L5CX_PLATFORM_ZEPHYR_H_

#include <stdint.h>
#include <string.h>
#include <zephyr/drivers/i2c.h>

/**
 * VL53L5CX_Platform: Zephyr-specific platform structure.
 * The ULD driver requires this to contain at least the I2C address.
 * We extend it with the Zephyr I2C device pointer.
 */
typedef struct {
	uint16_t address;            /* 8-bit I2C address (0x52 default) */
	const struct device *i2c_dev; /* Zephyr I2C controller device */
} VL53L5CX_Platform;

/*
 * Sensor configuration defines used by the ULD.
 * VL53L5CX_NB_TARGET_PER_ZONE: number of targets per zone (1-4).
 * Lower values reduce I2C bandwidth and RAM usage.
 */
#define VL53L5CX_NB_TARGET_PER_ZONE    1U

/* All outputs enabled by default. Uncomment to disable specific outputs. */
/* #define VL53L5CX_DISABLE_AMBIENT_PER_SPAD */
/* #define VL53L5CX_DISABLE_NB_SPADS_ENABLED */
/* #define VL53L5CX_DISABLE_NB_TARGET_DETECTED */
/* #define VL53L5CX_DISABLE_SIGNAL_PER_SPAD */
/* #define VL53L5CX_DISABLE_RANGE_SIGMA_MM */
/* #define VL53L5CX_DISABLE_DISTANCE_MM */
/* #define VL53L5CX_DISABLE_REFLECTANCE_PERCENT */
/* #define VL53L5CX_DISABLE_TARGET_STATUS */
/* #define VL53L5CX_DISABLE_MOTION_INDICATOR */

/* Platform I2C functions required by VL53L5CX ULD */
uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform,
			 uint16_t register_address, uint8_t *p_value);

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform,
			 uint16_t register_address, uint8_t value);

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform,
			  uint16_t register_address, uint8_t *p_values,
			  uint32_t size);

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform,
			  uint16_t register_address, uint8_t *p_values,
			  uint32_t size);

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform);

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t time_ms);

#endif /* VL53L5CX_PLATFORM_ZEPHYR_H_ */
