
/* Include ----------------------------------------------------------------- */
#include <zephyr.h>
#include <device.h>
#include "ei_zephyr_flash_commands.h"
#include "ei_device_nordic_nrf91.h"

#define RAM_SAMPLE_BLOK         9
#define RAM_SAMPLE_BLOK_SIZE    4096

#define SIZE_RAM_BUFFER         (RAM_SAMPLE_BLOK_SIZE * RAM_SAMPLE_BLOK)


/* Private function prototypes --------------------------------------------- */
static uint8_t ram_memory[SIZE_RAM_BUFFER];

/** 32-bit align write buffer size */
#define WORD_ALIGN(a)	((a & 0x3) ? (a & ~0x3) + 0x4 : a)


/**
 * @brief      Copy configuration data to config pointer
 *
 * @param      config       Destination pointer for config
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_zephyr_ret_t enum
 */
int ei_zephyr_flash_load_config(uint32_t *config, uint32_t config_size)
{
    if(config == NULL) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    return ZEPHYR_FLASH_CMD_OK;
}


/**
 * @brief      Write config to Flash
 *
 * @param[in]  config       Pointer to configuration data
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_zephyr_ret_t enum
 */
int ei_zephyr_flash_save_config(const uint32_t *config, uint32_t config_size)
{
    int ret_val = ZEPHYR_FLASH_CMD_OK;

    if(config == NULL) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    return ret_val;
}


/**
 * @brief      Erase blocks in sample data space
 *
 * @param[in]  start_block  The start block
 * @param[in]  end_address  The end address
 *
 * @return     ei_zephyr_ret_t
 */
int ei_zephyr_flash_erase_sampledata(uint32_t start_block,
                                     uint32_t end_address)
{
    return ZEPHYR_FLASH_CMD_OK;
}


/**
 * @brief      Write sample data to flash
 *
 * @param[in]  buffer          Buffer holding data that we want to write
 * @param[in]  address_offset  The offset that we want to write from
 * @param[in]  num_write_bytes The number of bytes that we want to write
 *
 * @return     ei_zephyr_ret_t
 */
int ei_zephyr_flash_write_samples(const void *sample_buffer,
                                  uint32_t address_offset,
                                  uint32_t num_write_bytes)
{
    uint32_t aligned_num_write_bytes = WORD_ALIGN(num_write_bytes);

    if((address_offset + aligned_num_write_bytes) > SIZE_RAM_BUFFER) {
        return ZEPHYR_FLASH_CMD_WRITE_ERROR;
    }
    else if(sample_buffer == 0) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    for(uint32_t i = 0;  i < aligned_num_write_bytes; i++) {
        ram_memory[address_offset + i] = *((char *)sample_buffer + i);
    }
    return ZEPHYR_FLASH_CMD_OK;
}


/**
 * @brief      Read sample data from flash
 *
 * @param      sample_buffer   The sample buffer
 * @param[in]  address_offset  The address offset
 * @param[in]  n_read_bytes    The n read bytes
 *
 * @return     ei_zephyr_ret_t
 */
int ei_zephyr_flash_read_samples(void *sample_buffer,
                                 uint32_t address_offset,
                                 uint32_t num_read_bytes)
{
    uint32_t aligned_num_read_bytes = WORD_ALIGN(num_read_bytes);

    if((address_offset + num_read_bytes) > SIZE_RAM_BUFFER) {
        return ZEPHYR_FLASH_CMD_READ_ERROR;
    }
    else if(sample_buffer == 0) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    for(uint32_t i = 0;  i < aligned_num_read_bytes; i++) {
        *((char *)sample_buffer + i) = ram_memory[address_offset + i];
    }
    return ZEPHYR_FLASH_CMD_OK;
}


/**
 * @brief      Get block size (Smallest erasble block).
 *
 * @return     Length of 1 block
 */
uint32_t ei_zephyr_flash_get_block_size(void)
{
    return RAM_SAMPLE_BLOK_SIZE;
}

/**
 * @brief      Get available sample blocks
 *
 * @return     Sample memory size / block size
 */
uint32_t ei_zephyr_flash_get_n_available_sample_blocks(void)
{
    return RAM_SAMPLE_BLOK;
}
