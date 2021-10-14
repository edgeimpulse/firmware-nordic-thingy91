#ifndef EI_ZEPHYR_FLASH_COMMANDS_H
#define EI_ZEPHYR_FLASH_COMMANDS_H

/* Include ----------------------------------------------------------------- */
#include <stdint.h>

#define ZEPHYR_FLASH_BLOCK_ERASE_TIME_MS    90

/** Eta fs return values */
typedef enum
{
    ZEPHYR_FLASH_CMD_OK = 0,                /**!< All is well                */
    ZEPHYR_FLASH_CMD_NOT_INIT,              /**!< FS is not initialised      */
    ZEPHYR_FLASH_CMD_READ_ERROR,            /**!< Error occured during read  */
    ZEPHYR_FLASH_CMD_WRITE_ERROR,           /**!< Error occured during write */
    ZEPHYR_FLASH_CMD_ERASE_ERROR,           /**!< Erase error occured        */
    ZEPHYR_FLASH_CMD_NULL_POINTER,          /**!< Null pointer parsed        */

}ei_zephyr_ret_t;

/* Prototypes -------------------------------------------------------------- */
int ei_zephyr_flash_load_config(uint32_t *config, uint32_t config_size);
int ei_zephyr_flash_save_config(const uint32_t *config, uint32_t config_size);
int ei_zephyr_flash_erase_sampledata(uint32_t start_block,
                                     uint32_t end_address);
int ei_zephyr_flash_read_samples(void *sample_buffer,
                                     uint32_t address_offset, 
                                     uint32_t num_read_bytes);
int ei_zephyr_flash_write_samples(const void *sample_buffer, 
                                  uint32_t address_offset, 
                                  uint32_t num_samples);
uint32_t ei_zephyr_flash_get_block_size(void);
uint32_t ei_zephyr_flash_get_n_available_sample_blocks(void);

#endif
