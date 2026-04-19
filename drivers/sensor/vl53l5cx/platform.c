/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * VL53L5CX Platform Implementation for Zephyr
 *
 * Implements I2C read/write, delay, and byte-swap using Zephyr APIs.
 * The VL53L5CX uses 16-bit register addresses, so each I2C write must
 * prepend 2 address bytes (MSB first) before the data payload.
 */

#include "platform.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(vl53l5cx_platform, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Maximum chunk size for multi-byte I2C writes.
 * VL53L5CX firmware upload requires large transfers (~95KB total).
 * We split into chunks to avoid stack overflow and I2C controller limits.
 * 1024 is safe for most Zephyr I2C controllers.
 */
#define VL53L5CX_I2C_CHUNK_SIZE  1024

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform,
			 uint16_t register_address, uint8_t *p_value)
{
	uint8_t addr_buf[2];
	int ret;

	addr_buf[0] = (uint8_t)(register_address >> 8);
	addr_buf[1] = (uint8_t)(register_address & 0xFF);

	ret = i2c_write_read(p_platform->i2c_dev,
			     p_platform->address >> 1,
			     addr_buf, 2,
			     p_value, 1);
	if (ret < 0) {
		LOG_ERR("RdByte(0x%04X) failed: %d", register_address, ret);
		return 1;
	}
	return 0;
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform,
			 uint16_t register_address, uint8_t value)
{
	uint8_t buf[3];
	int ret;

	buf[0] = (uint8_t)(register_address >> 8);
	buf[1] = (uint8_t)(register_address & 0xFF);
	buf[2] = value;

	ret = i2c_write(p_platform->i2c_dev, buf, 3,
			p_platform->address >> 1);
	if (ret < 0) {
		LOG_ERR("WrByte(0x%04X) failed: %d", register_address, ret);
		return 1;
	}
	return 0;
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform,
			  uint16_t register_address, uint8_t *p_values,
			  uint32_t size)
{
	uint8_t addr_buf[2];
	int ret;

	addr_buf[0] = (uint8_t)(register_address >> 8);
	addr_buf[1] = (uint8_t)(register_address & 0xFF);

	ret = i2c_write_read(p_platform->i2c_dev,
			     p_platform->address >> 1,
			     addr_buf, 2,
			     p_values, size);
	if (ret < 0) {
		LOG_ERR("RdMulti(0x%04X, %u) failed: %d",
			register_address, size, ret);
		return 1;
	}
	return 0;
}

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform,
			  uint16_t register_address, uint8_t *p_values,
			  uint32_t size)
{
	/*
	 * The firmware upload sends very large buffers (up to 32KB).
	 * We chunk them to stay within I2C controller limits and avoid
	 * large stack allocations.
	 */
	uint8_t buf[VL53L5CX_I2C_CHUNK_SIZE + 2];
	uint32_t offset = 0;
	uint32_t chunk;
	int ret;

	while (offset < size) {
		chunk = size - offset;
		if (chunk > VL53L5CX_I2C_CHUNK_SIZE) {
			chunk = VL53L5CX_I2C_CHUNK_SIZE;
		}

		uint16_t addr = register_address + offset;
		buf[0] = (uint8_t)(addr >> 8);
		buf[1] = (uint8_t)(addr & 0xFF);
		memcpy(&buf[2], &p_values[offset], chunk);

		ret = i2c_write(p_platform->i2c_dev, buf, chunk + 2,
				p_platform->address >> 1);
		if (ret < 0) {
			LOG_ERR("WrMulti(0x%04X+%u, %u) failed: %d",
				register_address, offset, chunk, ret);
			return 1;
		}
		offset += chunk;
	}
	return 0;
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform)
{
	/* Optional: implement HW reset via LPN GPIO if available */
	VL53L5CX_WaitMs(p_platform, 100);
	return 0;
}

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
	uint32_t i, tmp;

	for (i = 0; i < size; i += 4) {
		tmp = ((uint32_t)buffer[i] << 24)
		    | ((uint32_t)buffer[i + 1] << 16)
		    | ((uint32_t)buffer[i + 2] << 8)
		    | ((uint32_t)buffer[i + 3]);
		memcpy(&buffer[i], &tmp, 4);
	}
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t time_ms)
{
	(void)p_platform;
	k_msleep(time_ms);
	return 0;
}
