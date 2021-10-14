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

#ifndef _EI_FUSION_H
#define _EI_FUSION_H

/* Include ----------------------------------------------------------------- */
#include "ei_sampler.h"
#include "sensor_aq.h"

#define NUM_FUSION_SENSORS       3  // number of fusable sensor modules
#define NUM_MIN_FUSIONS          1  // min number of sensor module combinations
#define NUM_MAX_FUSIONS          3  // max number of sensor module combinations
#define FUSION_FREQUENCY         10.0f // sampling frequency for fusion samples
#define NUM_MAX_FUSION_AXIS      20 // max number of axis to sample
#define SIZEOF_SENSOR_NAME       20 // char alloc for sensor module name

#define EI_MAX_FREQUENCIES  5
/**
 * Information about the fusion structure, name, number of axis, sampling frequencies, axis name, and reference to read sensor function
 */
typedef struct {
    // Name of sensor to show up in Studio
    const char *name;
    // Number of sensor axis to sample
    int num_axis;
    // List of sensor sampling frequencies
    float frequencies[EI_MAX_FREQUENCIES];
    // Sensor axes, note that I declare this not as a pointer to have a more fluent interface
    sensor_aq_sensor sensors[EI_MAX_SENSOR_AXES];
    // Reference to read sensor function that should return pointer to float array of raw sensor data
    float* (*read_data)();
 } ei_device_fusion_sensor_t;

typedef float sample_format_t;

 /* Function prototypes ----------------------------------------------------- */
void ei_get_sensor_fusion_list(void);
bool ei_is_fusion(char *sensor_list);
bool ei_fusion_setup_data_sampling(void);
//TODO: do we need it?
void create_fusion_list(int min_length, int i, int curr_length, bool check[], int max_length);

#endif
