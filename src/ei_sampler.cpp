/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sampler, CONFIG_EI_LOG_LEVEL);
#include <stdint.h>
#include <stdlib.h>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_memory.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "ei_sampler.h"
#include "ei_ws_client.h"
#include <zephyr/kernel.h>

/* Forward declarations ---------------------------------------------------- */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *);
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin);
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght);
static bool create_header(sensor_aq_payload_info *payload);

/* Private variables ------------------------------------------------------- */
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t sample_buffer_size;
static uint32_t headerOffset = 0;
static uint8_t write_word_buf[4];
static int write_addr = 0;
EI_SENSOR_AQ_STREAM stream;
static EiDeviceMemory* memory = EiDeviceInfo::get_device()->get_memory();

static unsigned char ei_samples_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_samples_signing_ctx;
static sensor_aq_ctx ei_samples_ctx = {
    { ei_samples_ctx_buffer, 1024 },
    &ei_samples_signing_ctx,
    &ei_write,
    &ei_seek,
    nullptr,
};

/**
 * @brief      Write sample data to FLASH
 * @details    Write size is always 4 bytes to keep alignment
 *
 * @param[in]  buffer     The buffer
 * @param[in]  size       The size
 * @param[in]  count      The count
 * @param      EI_SENSOR_AQ_STREAM file pointer (not used)
 *
 * @return     number of bytes handled
 */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *)
{
    for (size_t i = 0; i < count; i++) {
        write_word_buf[write_addr & 0x3] = *((char *)buffer + i);

        if ((++write_addr & 0x03) == 0x00) {
            memory->write_sample_data(write_word_buf, (write_addr - 4) + headerOffset, 4);
        }
    }

    return count;
}

/**
 * @brief      File handle seed function. Not used
 */
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin)
{
    return 0;
}

/**
 * @brief      Write out remaining data in word buffer to FLASH.
 *             And append CBOR end character.
 */
static void ei_write_last_data(void)
{
    uint8_t fill = ((uint8_t)write_addr & 0x03);
    uint8_t insert_end_address = 0;

    if (fill != 0x00) {
        for (uint8_t i = fill; i < 4; i++) {
            write_word_buf[i] = 0xFF;
        }

        memory->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, 4);
        insert_end_address = 4;
    }

    /* Write appending word for end character */
    for (uint8_t i = 0; i < 4; i++) {
        write_word_buf[i] = 0xFF;
    }
    memory->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset + insert_end_address, 4);
}

bool ei_sampler_start_sampling(void *v_ptr_payload, starter_callback ei_sample_start, uint32_t sample_size)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    sensor_aq_payload_info *payload = (sensor_aq_payload_info *)v_ptr_payload;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %u ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());
    sample_buffer_size = (samples_required * sample_size) * 4;
    current_sample = 0;

    // Minimum delay of 2000 ms for daemon
    uint32_t delay_time_ms = ((sample_buffer_size / memory->block_size) + 1) * memory->block_erase_time;
    ei_printf("Starting in %u ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);

    dev->set_state(eiStateErasingFlash);

    if(memory->erase_sample_data(0, sample_buffer_size) != (sample_buffer_size)) {
        return false;
    }

    // if erasing took less than 2 seconds, wait additional time
    if(delay_time_ms < 2000) {
        ei_sleep(2000 - delay_time_ms);
    }

    if (create_header(payload) == false) {
        return false;
    }

    if (ei_sample_start(&sample_data_callback, dev->get_sample_interval_ms()) == false) {
        return false;
    }

#ifdef CONFIG_REMOTE_INGESTION
    if(ei_ws_get_connection_status()) {
        ei_ws_send_msg(TxMsgType::SampleStartedMsg);
    }
#endif

    ei_printf("Sampling...\n");

    dev->set_state(eiStateSampling);

    while (current_sample < samples_required) {
        k_yield();
    }

    dev->stop_sample_thread();

    ei_write_last_data();
    write_addr++;

    ei_printf("Done sampling, total bytes collected: %u\n", samples_required);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
#ifdef CONFIG_REMOTE_INGESTION
    if(ei_ws_get_connection_status()) {
        ei_printf("Used buffer, from=0, to=%u.\n", write_addr + headerOffset);
        ei_ws_send_msg(TxMsgType::SampleUploadingMsg);
        ei_ws_send_sample(0, write_addr + headerOffset);
        ei_ws_send_msg(TxMsgType::SampleFinishedMsg);
    }
    else
#endif
    {
        ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%u.\n", write_addr + headerOffset);
    }
    ei_printf("OK\n");

    return true;
}

/**
 * @brief      Create and write the CBOR header to FLASH
 *
 * @param      payload  The payload
 *
 * @return     True on success
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    sensor_aq_init_none_context(&ei_samples_signing_ctx);

    int tr = sensor_aq_init(&ei_samples_ctx, payload, NULL, true);

    if (tr != AQ_OK) {
        LOG_ERR("sensor_aq_init failed (%d)", tr);
        return false;
    }
    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_samples_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t *)ei_samples_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        LOG_ERR("Failed to find end of header");
        return false;
    }

    // Write to blockdevice
    tr = memory->write_sample_data((uint8_t*)ei_samples_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)tr != end_of_header_ix) {
        LOG_ERR("Failed to write to header blockdevice (%d)", tr);
        return false;
    }

    ei_samples_ctx.stream = &stream;

    headerOffset = end_of_header_ix;
    write_addr = 0;

    return true;
}

static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght)
{
    sensor_aq_add_data(&ei_samples_ctx, (float *)sample_buf, byteLenght / sizeof(float));

    if (++current_sample >= samples_required) {
        return true;
    }
    else {
        return false;
    }
}
