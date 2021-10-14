/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
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

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_fusion.h"
#include "ei_inertialsensor.h"
#include "ei_environmentsensor.h"
#include "ei_lightsensor.h"

#include "ei_config_types.h"
#include "ei_device_nordic_nrf91.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_classifier_porting.h"

#include <cmath>
#include <stdio.h>
#include <device.h>
#include <drivers/sensor.h>
#include <zephyr.h>
#include <logging/log.h>

#define LOG_MODULE_NAME ei_fusion
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

extern ei_config_t *ei_config_get_config();
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

/* Private functions declaration */
void ei_fusion_read_data(void);
bool ei_fusion_sample_start(sampler_callback callsampler, float sample_interval_ms);

/*
** @brief fixes CBOR header padding issue
*/
#define CBOR_HEADER_OFFSET      0x02 // offset for unknown header size (can be 0-3)
#define SENSORS_BYTE_OFFSET     14 // number of CBOR bytes for sensor (ie {"name": "...", "units": "..."}) 

sampler_callback  fusion_cb_sampler;

/*
** @brief list of fusable sensors
*/
const ei_device_fusion_sensor_t fusable_sensor_list[NUM_FUSION_SENSORS] = { inertial_sensor, environment_sensor, light_sensor }; // #include "sensor.h" & add ei_device_fusion_sensor_t struct

/*
** @brief list of sensors to fuse
*/
ei_device_fusion_sensor_t *fusion_sensors[NUM_FUSION_SENSORS];
static char fusion_buffer[NUM_MAX_FUSIONS * SIZEOF_SENSOR_NAME];
static int num_fusions, num_fusion_axis;

/**
 * @brief   Gets list of possible sensor combinations from fusable_sensor_list
 */
void ei_get_sensor_fusion_list(void)
{
    num_fusion_axis = 0;
    fusion_buffer[0] = '\0';
    bool check[NUM_FUSION_SENSORS];  

    for(int i = 0; i < NUM_FUSION_SENSORS; i++) {
        check[i] = false;
    }

    for(int i = NUM_MIN_FUSIONS; i <= NUM_MAX_FUSIONS; i++) {
        create_fusion_list(i, 0, 0, check, NUM_FUSION_SENSORS);
    }
}

/**
 * @brief      Check if requested sensor list is valid sensor fusion, create sensor buffer
 *
 * @param[in]  sensor_list      Sensor list to sample (ie. "Inertial + Environmental")
 * 
 * @retval  false if invalid sensor_list
 */
bool ei_is_fusion(char *sensor_list) 
{
    char *buff;
    bool is_fusion, added_loc;
    
    num_fusions = 0;
    num_fusion_axis = 0;

    // clear fusion list
    for (int i = 0; i < NUM_FUSION_SENSORS; i++) {
        fusion_sensors[i] = NULL;
    }
    
    // copy sensor_list to buffer to keep for error msgs
    strcpy(fusion_buffer, sensor_list);
    buff = strtok(fusion_buffer, " +");
    // while there is sensors names in sensor list buffer
    while (buff != NULL) {
        is_fusion = false;
        // check for sensor name in list of fusable sensors
        for (int i = 0; i < NUM_FUSION_SENSORS; i++) {
            // is a matching sensor
            if (strstr(buff, fusable_sensor_list[i].name)) {
                added_loc = false;
                for (int j = 0; j < num_fusions; j++) { 
                    // has already been added to sampling list
                    if (strstr(buff, fusion_sensors[j]->name)) {
                        added_loc = true;
                        break;
                    }
                }
                if (!added_loc) {
                    fusion_sensors[num_fusions++] = (ei_device_fusion_sensor_t *)&fusable_sensor_list[i];
                    num_fusion_axis += fusable_sensor_list[i].num_axis;
                }
                is_fusion = true; 
            }
        }
        // no matching sensors in sensor_list
        if (!is_fusion) {
            return false;
        }
        buff = strtok(NULL, " +");
    }
    return true;
}

/**
 * @brief   Read from selected sensors, combine and send to callback sampler
 * 
 * @todo    Allow sampling at multiple frequencies
 */
void ei_fusion_read_data(void)
{
    uint32_t start = ei_read_timer_ms();
    int32_t remaining;
    float *buff;
    float data[num_fusion_axis];
    int loc = 0;

    for (int i = 0; i < num_fusions; i++) {
        // read sensor data from sensor
        buff = fusion_sensors[i]->read_data();
        for (int j = 0; j < fusion_sensors[i]->num_axis; j++) {
            // add sensor data to fusion data
            data[j+loc] = *(buff+j);
        }

        loc += fusion_sensors[i]->num_axis;
    }
    
    // send fusion data to sampler
    fusion_cb_sampler((const void *)&data[0], sizeof(sample_format_t) * num_fusion_axis);

    remaining = ei_config_get_config()->sample_interval_ms - (ei_read_timer_ms() - start);
    if (remaining > 0) {
        ei_sleep(remaining);
    }
}

/**
 * @brief      Create list of all sensor combinations
 *
 * @param[in]  callsampler          callback function from ei_sampler
 * @param[in]  sample_interval_ms   sample interval from ei_sampler
 * 
 * @retval  false if initialisation failed
 */
bool ei_fusion_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    fusion_cb_sampler = callsampler; // connect cb sampler (used in ei_fusion_read_data())

    //if one of the sensos is light sensor, do not use LED indication for sampling
    for (int i = 0; i < num_fusions; i++) {
        LOG_INF("num: %d, sens: %s", num_fusions, fusion_sensors[i]->name);
        if (strcmp(fusion_sensors[i]->name, "Light")) {
            EiDevice.set_state(eiStateSampling);
        }
        else {
            EiDevice.set_state(eiStateIdle);
            //Delay due to afect of LEDs to the light sensor 
            k_sleep(K_MSEC(500));
        }
    }
    return true;
}

/**
 * @brief      Create payload for sampling list, pad, start sampling
 */
bool ei_fusion_setup_data_sampling(void) 
{
    uint32_t available_bytes;
    uint32_t requested_bytes;
    int payload_bytes = 0; // counts bytes sensor fusion adds
    char fill;
    int index = 0;

    // Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead
    available_bytes = (ei_zephyr_flash_get_n_available_sample_blocks()-1) * ei_zephyr_flash_get_block_size();

    // Check available sample size before sampling for the selected frequency
    requested_bytes = ceil((ei_config_get_config()->sample_length_ms / ei_config_get_config()->sample_interval_ms) * (sizeof(sample_format_t) * num_fusion_axis) * 2);
    if(requested_bytes > available_bytes) {
        ei_printf("ERR: Sample length is too long. Maximum allowed is %ims at %.1fHz.\r\n", 
            (int)floor(available_bytes / (((sizeof(sample_format_t) * num_fusion_axis) * 2) / ei_config_get_config()->sample_interval_ms)),
            (1000 / ei_config_get_config()->sample_interval_ms));
        return false;
    }
    
    // create header payload from individual sensors
    sensor_aq_payload_info payload = { EiDevice.get_id_pointer(), EiDevice.get_type_pointer(), ei_config_get_config()->sample_interval_ms, NULL };
    for (int i = 0; i < num_fusions; i++) {
        for (int j = 0; j < fusion_sensors[i]->num_axis; j++){
            payload.sensors[index].name = fusion_sensors[i]->sensors[j].name;
            payload.sensors[index++].units = fusion_sensors[i]->sensors[j].units;

            payload_bytes += strlen(fusion_sensors[i]->sensors[j].name) + strlen(fusion_sensors[i]->sensors[j].units) + SENSORS_BYTE_OFFSET;
        }
    }

    // counts bytes payload adds, pads if not 32 bits
    fill = (CBOR_HEADER_OFFSET + payload_bytes) & 0x03;
    if(fill != 0x00) {
        strcpy(fusion_buffer, payload.sensors[num_fusion_axis-1].units);
        for(int i=fill; i<4; i++) {
            strcat(fusion_buffer, " ");
        }
        payload.sensors[num_fusion_axis-1].units = fusion_buffer;
    }

    EiDevice.set_state(eiStateErasingFlash);
    ei_sampler_start_sampling(&payload, &ei_fusion_sample_start, &ei_fusion_read_data, (sizeof(sample_format_t) * num_fusion_axis));
    EiDevice.set_state(eiStateIdle);

    return true;
}

/**
 * @brief      Create list of all sensor combinations
 *
 * @param[in]  min_length       min number of items to combine
 * @param[in]  i                iterator
 * @param[in]  curr_length      current combination length
 * @param[in]  check            bitmap of select pattern
 * @param[in]  max_length       total items to combine
 */
void create_fusion_list(int min_length, int i, int curr_length, bool check[], int max_length)
{
    uint32_t available_bytes;

    int curr_pos = 0;

    if(curr_length > min_length) {
        return;
    }
    else if (curr_length == min_length) {
        /* Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead */
        available_bytes = (ei_zephyr_flash_get_n_available_sample_blocks()-1) * ei_zephyr_flash_get_block_size();
        for (int i = 0; i < max_length; i++) {
            if (check[i]) {
                curr_pos++;
                strcat(fusion_buffer, fusable_sensor_list[i].name);
                num_fusion_axis += fusable_sensor_list[i].num_axis;
                if (curr_pos != min_length) {
                    strcat(fusion_buffer, " + ");
                }
                else {
                    if (min_length == 1) // no fusion, use regular freq
                    {
                        ei_printf("Name: %s, Max sample length: %hus, Frequencies: [", fusion_buffer, (int)(available_bytes / (fusable_sensor_list[i].frequencies[0] * (sizeof(sample_format_t) * num_fusion_axis) * 2)));
                        for (int j = 0; j < EI_MAX_FREQUENCIES; j++) {
                            if (fusable_sensor_list[i].frequencies[j] != 0.0f) {
                                if (j != 0) {
                                    ei_printf(", ");
                                }
                                ei_printf_float(fusable_sensor_list[i].frequencies[j]);
                                ei_printf("Hz");
                            }
                        }
                    } 
                    else { // fusion, use set freq
                        ei_printf("Name: %s, Max sample length: %hus, Frequencies: [", fusion_buffer, (int)(available_bytes / (FUSION_FREQUENCY * (sizeof(sample_format_t) * num_fusion_axis) * 2)));
                        ei_printf_float(FUSION_FREQUENCY);
                        ei_printf("Hz");
                    }
                    ei_printf("]\n");
                }

            }
        }

        /* Done, get next fusion */
        num_fusion_axis = 0;
        fusion_buffer[0] = '\0';
        return;
    }

    if (i == max_length) {
        return;
    }

    check[i] = true;
    create_fusion_list(min_length, i + 1, curr_length + 1, check, max_length);
    check[i] = false;
    create_fusion_list(min_length, i + 1, curr_length, check, max_length);
}
