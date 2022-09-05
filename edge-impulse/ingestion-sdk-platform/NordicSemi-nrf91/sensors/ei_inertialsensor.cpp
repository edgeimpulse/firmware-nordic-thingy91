/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
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
#include <stdio.h>
#include <device.h>
#include <drivers/sensor.h>
#include <zephyr.h>
#include "ei_inertialsensor.h"
#include "ei_config_types.h"
#include "ei_device_nordic_nrf91.h"
#include "sensor_aq.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include <logging/log.h>
#define LOG_MODULE_NAME ei_accel
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

//TODO: missing header for that function?
extern ei_config_t *ei_config_get_config();
//TODO: missing header for that function?
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

static sampler_callback  cb_sampler;
const struct device *accel_dev;
static float sample[INERTIAL_VALUES_IN_SAMPLE];

/* private functions */
bool ei_inertial_fetch_sample(void);

/**
 * @brief      Setup accelerometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_inertial_init(void)
{
	accel_dev =  device_get_binding("ADXL362");
	if (!accel_dev) {
		LOG_ERR("Devicetree has no ADXL362 node");
		return false;
	}

	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Device %s is not ready", accel_dev->name);
		return false;
	}

    return true;
}

/**
 * @brief      Setup timing and data handle callback function
 *
 * @param[in]  callsampler         Function to handle the sampled data
 * @param[in]  sample_interval_ms  The sample interval milliseconds
 *
 * @return     true
 */
bool ei_inertial_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    cb_sampler = callsampler;

    LOG_INF("sample_interval_ms = %f us\n", sample_interval_ms);

    ei_config_set_sample_interval(sample_interval_ms);

    EiDevice.set_state(eiStateSampling);

    return true;
}

bool ei_inertial_fetch_sample(void)
{
    struct sensor_value raw_sample[INERTIAL_VALUES_IN_SAMPLE];

    if (sensor_sample_fetch(accel_dev) < 0) {
        LOG_ERR("Sample fetch error");
        return false;
    }

    if (sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, raw_sample) < 0) {
        LOG_ERR("Failed to get accel readings\n");
        return false;
    }

    for(int i=0; i<INERTIAL_VALUES_IN_SAMPLE; i++) {
        sample[i] = (float)sensor_value_to_double(&raw_sample[i]);
    }

    return true;
}

/**
 * @brief      Get data from sensor, convert and call callback to handle
 */
void ei_inertial_read_data(void)
{
    uint32_t start = ei_read_timer_ms();
    int32_t remaining;

    ei_inertial_fetch_sample();

    cb_sampler((const void *)&sample[0], SIZEOF_INERTIAL_VALUES_IN_SAMPLE);

    remaining = (ei_config_get_config()->sample_interval_ms) - (ei_read_timer_ms() - start);
    LOG_DBG("remaining = %d", remaining);
    if (remaining > 0) {
        ei_sleep(remaining);
    }
}

float *ei_fusion_inertial_read_data(void)
{
    ei_inertial_fetch_sample();

    return sample;
}