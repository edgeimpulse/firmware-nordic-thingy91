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


#ifndef _EI_ENVIRONMENTSENSOR_H
#define _EI_ENVIRONMENTSENSOR_H

/* Include ----------------------------------------------------------------- */
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include "ei_sampler.h"
#include "ei_fusion.h"

/** Number of axis used and sample data format */
#define ENVIRONMENT_VALUES_IN_SAMPLE			4
#define SIZEOF_ENVIRONMENT_VALUES_IN_SAMPLE   (sizeof(float) * ENVIRONMENT_VALUES_IN_SAMPLE)

/* Function prototypes ----------------------------------------------------- */
bool ei_environment_init(void);
bool ei_environment_sample_start(sampler_callback callback, float sample_interval_ms);
void ei_environment_read_data(void);

/* sensor fusion related functions and data structs */
float *ei_fusion_environment_read_data(void);

static const ei_device_fusion_sensor_t environment_sensor = {
    // name of sensor module to be displayed in fusion list
    "Environment",
    // number of sensor module axis
    ENVIRONMENT_VALUES_IN_SAMPLE,
    // sampling frequencies
    { 0.25f },
    // axis name and units payload (must be same order as read in)
    { {"temperature", "degC"}, {"pressure", "kPa"}, {"humidity", "%"}, {"gas res", "MOhm"}, },
    // reference to read data function
    &ei_fusion_environment_read_data
};

#endif
