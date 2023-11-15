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

#ifndef EI_FLASH_MEMORY_H
#define EI_FLASH_MEMORY_H

#include "firmware-sdk/ei_device_memory.h"
#define RAM_BLOCK_SIZE  (1024)
#define RAM_BLOCK_NUM   (84)
#define RAM_TOTAL_SIZE  (RAM_BLOCK_SIZE * RAM_BLOCK_NUM)

class EiFlashMemory : public EiDeviceMemory {
private:
    const struct device *flash_dev;
protected:
    uint32_t read_data(uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t erase_data(uint32_t address, uint32_t num_bytes);
    // RAM memory is for sample only, not for config
    uint8_t ram_memory[RAM_TOTAL_SIZE];
    // used for save/load config operations only because base block_size is defined for RAM
    uint32_t flash_block_size;
public:
    EiFlashMemory(uint32_t config_size);
    // override default implementation to use Flash memory, while RAM is for samples
    bool save_config(const uint8_t *config, uint32_t config_size) override;
    bool load_config(uint8_t *config, uint32_t config_size) override;
};

#endif /* EI_FLASH_MEMORY_H */