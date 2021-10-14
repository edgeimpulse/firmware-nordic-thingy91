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
#include <stdio.h>
#include <device.h>
#include <drivers/sensor.h>
#include <zephyr.h>
#include "ei_config_types.h"
#include "ei_lightsensor.h"
#include "ei_device_nordic_nrf91.h"
#include "sensor_aq.h"
#include "ei_classifier_porting.h"

#include <logging/log.h>
#define LOG_MODULE_NAME ei_light
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

//TODO: missing header for that function?
extern ei_config_t *ei_config_get_config();
//TODO: missing header for that function?
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

static sampler_callback  cb_sampler;
const struct device *light_dev;
static float sample[LIGHTSENSOR_VALUES_IN_SAMPLE];

/* private functions */
bool ei_light_fetch_sample(void);

/**
 * @brief      Setup lightneetometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_lightsensor_init(void)
{
	light_dev = device_get_binding("BH1749");
	if (!light_dev) {
		LOG_ERR("Devicetree has no BH1749 node");
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
bool ei_lightsensor_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    cb_sampler = callsampler;

    LOG_INF("sample_interval_ms = %f us\n", sample_interval_ms);

    ei_config_set_sample_interval(sample_interval_ms);

    //Disabled because LEDs are interfering with raw_sample sensor
    EiDevice.set_state(eiStateIdle);//eiStateSampling);

    //Delay due to afect of LEDs to the raw_sample sensor 
    k_sleep(K_MSEC(500));
    
    return true;
}

bool ei_light_fetch_sample(void)
{
    struct sensor_value raw_sample[LIGHTSENSOR_VALUES_IN_SAMPLE];

    if (sensor_sample_fetch(light_dev) < 0) {
        LOG_ERR("Sample fetch error");
        return false;
    }

    if (sensor_channel_get(light_dev, SENSOR_CHAN_RED, &raw_sample[0])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_RED");
        return false;
    }

    if (sensor_channel_get(light_dev, SENSOR_CHAN_GREEN, &raw_sample[1])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_GREEN");
        return false;
    }

    if (sensor_channel_get(light_dev, SENSOR_CHAN_BLUE, &raw_sample[2])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_BLUE");
        return false;
    }

    if (sensor_channel_get(light_dev, SENSOR_CHAN_IR, &raw_sample[3])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_IR");
        return false;
    }

    for(int i=0; i<LIGHTSENSOR_VALUES_IN_SAMPLE; i++) {
        sample[i] = (float)sensor_value_to_double(&raw_sample[i]);
    }

    return true;
}

/**
 * @brief      Get data from sensor, convert and call callback to handle
 */
void ei_lightsensor_read_data(void)
{
    uint32_t start = ei_read_timer_ms();
    int32_t remaining;

    ei_light_fetch_sample();

    cb_sampler((const void *)&sample[0], SIZEOF_LIGHTSENSOR_VALUES_IN_SAMPLE);

    remaining = (ei_config_get_config()->sample_interval_ms)-(ei_read_timer_ms()-start);
    LOG_DBG("remaining = %d", remaining);
    if (remaining > 0) {
        ei_sleep(remaining);
    }
}

float *ei_fusion_light_read_data(void)
{
    ei_light_fetch_sample();

    return sample;
}
