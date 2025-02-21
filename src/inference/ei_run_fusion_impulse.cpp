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

#include "model-parameters/model_metadata.h"
#if defined(EI_CLASSIFIER_SENSOR) && ((EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_FUSION) || (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER))
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "firmware-sdk/ei_fusion.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "ei_run_impulse.h"

typedef enum {
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

static int print_results;
static uint16_t samples_per_inference;
static inference_state_t state = INFERENCE_STOPPED;
static uint64_t last_inference_ts = 0;
static bool continuous_mode = false;
static bool debug_mode = false;
static bool is_fusion = false;
static float samples_circ_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int samples_wr_index = 0;

/**
 * @brief Called for each single sample
 *
 */
bool samples_callback(const void *raw_sample, uint32_t raw_sample_size)
{
    if(state != INFERENCE_SAMPLING) {
        // stop collecting samples if we are not in SAMPLING state
        return true;
    }

    for(int i = 0; i < (int)(raw_sample_size / sizeof(float)); i++) {
        samples_circ_buff[samples_wr_index++] = ((float*)raw_sample)[i];
        if(samples_wr_index > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            /* start from beginning of the circular buffer */
            samples_wr_index = 0;
        }
        if(samples_wr_index >= samples_per_inference) {
            state = INFERENCE_DATA_READY;
            return true;
        }
    }

    return false;
}

void ei_run_impulse(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    switch(state) {
        case INFERENCE_STOPPED:
            // nothing to do
            return;
        case INFERENCE_WAITING:
            if(ei_read_timer_ms() < (last_inference_ts + 2000)) {
                return;
            }

#if MULTI_FREQ_ENABLED == 1
            if (is_fusion) {
                ei_multi_fusion_sample_start(&samples_callback, EI_CLASSIFIER_INTERVAL_MS);
            }
            else {
                ei_fusion_sample_start(&samples_callback, EI_CLASSIFIER_INTERVAL_MS);
            }
#else
            ei_fusion_sample_start(&samples_callback, EI_CLASSIFIER_INTERVAL_MS);
#endif
            state = INFERENCE_SAMPLING;
            dev->set_state(eiStateSampling);
            ei_printf("Sampling...\n");
            return;
        case INFERENCE_SAMPLING:
            // wait for data to be collected through callback
            return;
        case INFERENCE_DATA_READY:
            dev->set_state(eiStateIdle);
            // nothing to do, just continue to inference provcessing below
            break;
        default:
            break;
    }

    signal_t signal;

    // shift circular buffer, so the newest data will be the first
    // if samples_wr_index is 0, then roll is immediately returning
    numpy::roll(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, (-samples_wr_index));
    /* reset wr index, the oldest data will be overwritten */
    samples_wr_index = 0;

    // Create a data structure to represent this window of data
    int err = numpy::signal_from_buffer(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("ERR: signal_from_buffer failed (%d)\n", err);
    }

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_error;
    if(continuous_mode == true) {
        ei_error = run_classifier_continuous(&signal, &result, debug_mode);
    }
    else {
        ei_error = run_classifier(&signal, &result, debug_mode);
    }

    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("Failed to run impulse (%d)", ei_error);
        return;
    }

    if(continuous_mode == true) {
        if(++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW >> 1)) {
            display_results(&ei_default_impulse, &result);
            print_results = 0;
        }
    }
    else {
        display_results(&ei_default_impulse, &result);
    }

    if(continuous_mode == true) {
        state = INFERENCE_SAMPLING;
    }
    else {
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();

    const char *axis_name = EI_CLASSIFIER_FUSION_AXES_STRING;
    if (!ei_connect_fusion_list(axis_name, AXIS_FORMAT)) {
        ei_printf("ERR: Failed to find sensor '%s' in the sensor list\n", axis_name);
        return;
    }

    continuous_mode = continuous;
    debug_mode = debug;
#if MULTI_FREQ_ENABLED == 1
    is_fusion = ei_is_fusion();
#endif

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.04fms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %.02f ms.\n", (float)(EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_INTERVAL_MS));
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));
    ei_printf("Starting inferencing, press 'b' to break\n");

    dev->set_sample_length_ms(EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_INTERVAL_MS);
    dev->set_sample_interval_ms(EI_CLASSIFIER_INTERVAL_MS);

    if (continuous == true) {
        samples_per_inference = EI_CLASSIFIER_SLICE_SIZE * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // In order to have meaningful classification results, continuous inference has to run over
        // the complete model window. So the first iterations will print out garbage.
        // We now use a fixed length moving average filter of half the slices per model window and
        // only print when we run the complete maf buffer to prevent printing the same classification multiple times.
        print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
        run_classifier_init();
        state = INFERENCE_SAMPLING;
    }
    else {
        samples_per_inference = EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // it's time to prepare for sampling
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

void ei_stop_impulse(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();

    if(state != INFERENCE_STOPPED) {
        state = INFERENCE_STOPPED;
        ei_printf("Inferencing stopped by user\r\n");
        dev->set_state(eiStateFinished);
        /* reset samples buffer */
        samples_wr_index = 0;
        run_classifier_deinit();
    }
}

bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

#endif /* defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_FUSION */
