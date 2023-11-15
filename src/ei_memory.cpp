/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ei_memory, CONFIG_EI_LOG_LEVEL);
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/devicetree.h>
#include "ei_memory.h"

#define STORAGE_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(storage_partition)

uint32_t EiFlashMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    if (num_bytes > memory_size - address) {
        num_bytes = memory_size - address;
    }

    memcpy(data, &ram_memory[address], num_bytes);

    return num_bytes;
}

uint32_t EiFlashMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    if (num_bytes > memory_size - address) {
        num_bytes = memory_size - address;
    }

    memcpy(&ram_memory[address], data, num_bytes);

    return num_bytes;
}

uint32_t EiFlashMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
    if (num_bytes > memory_size - address) {
        num_bytes = memory_size - address;
    }

    memset(&ram_memory[address], 0, num_bytes);

    return num_bytes;
}

bool EiFlashMemory::save_config(const uint8_t *config, uint32_t config_size)
{
    int ret;
    uint32_t bytes_to_erase = 0;

    // calculate how many blocks to eras based on config_size and flash_block_size
    // but block _size is for Flash memory (separate) not the one configured for EiDeviceMemory
    if(config_size <= flash_block_size) {
        bytes_to_erase = flash_block_size;
    }
    else {
        bytes_to_erase = config_size % flash_block_size == 0 ? config_size : config_size + (flash_block_size - (config_size % flash_block_size));
    }
    LOG_DBG("Erasing %u bytes for config", bytes_to_erase);

    ret = flash_erase(this->flash_dev, STORAGE_PARTITION_OFFSET, bytes_to_erase);
    if(ret) {
        LOG_ERR("Erase config memory failed: %d", ret);
        return false;
    }

    ret = flash_write(this->flash_dev, STORAGE_PARTITION_OFFSET, config, config_size);
    if(ret) {
        LOG_ERR("Failed to save config: %d", ret);
        return false;
    }

    LOG_DBG("Saved config (%u bytes)", config_size);

    return true;
}

bool EiFlashMemory::load_config(uint8_t *config, uint32_t config_size)
{
    int ret = flash_read(this->flash_dev, STORAGE_PARTITION_OFFSET, config, config_size);
    if(ret) {
        LOG_ERR("Failed to read config: %d\n", ret);
        return false;
    }

    return true;
}

EiFlashMemory::EiFlashMemory(uint32_t config_size):
    EiDeviceMemory(
        0, // 0 means samples will be stored from block 0
        0,
        RAM_TOTAL_SIZE,
        RAM_BLOCK_SIZE),
    flash_dev(FIXED_PARTITION_DEVICE(storage_partition))
{
    struct flash_pages_info info;

    if (!device_is_ready(this->flash_dev)) {
		LOG_ERR("%s: device not ready.", this->flash_dev->name);
	}

	int rc = flash_get_page_info_by_offs(flash_dev, STORAGE_PARTITION_OFFSET, &info);
    if (rc) {
        LOG_ERR("Unable to get page info for offset %d (err: %d)", STORAGE_PARTITION_OFFSET, rc);
        this->flash_block_size = 0;
    }
    else {
        this->flash_block_size = info.size;
    }
}
