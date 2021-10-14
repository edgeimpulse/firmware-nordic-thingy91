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
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAenvironmentES OR OTHER
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
#include "ei_environmentsensor.h"
#include "ei_config_types.h"
#include "ei_device_nordic_nrf91.h"
#include "sensor_aq.h"
#include "ei_classifier_porting.h"

#include <logging/log.h>
#define LOG_MODULE_NAME ei_environment
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

//TODO: missing header for that function?
extern ei_config_t *ei_config_get_config();
//TODO: missing header for that function?
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

static sampler_callback  cb_sampler;
static const struct device *sensor_dev;
static float sample[ENVIRONMENT_VALUES_IN_SAMPLE];

static void env_timer_handler(struct k_timer *dummy);
static void env_work_handler(struct k_work *work);

K_TIMER_DEFINE(env_timer, env_timer_handler, NULL);
K_WORK_DEFINE(env_work, env_work_handler);

/* private functions */
bool ei_environment_fetch_sample(void);

/**
 * @brief      Setup environmentneetometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_environment_init(void)
{
	sensor_dev = device_get_binding("BME680");
	if (!sensor_dev) {
		LOG_ERR("Devicetree has no BME680 node");
		return false;
	}

    k_timer_start(&env_timer, K_MSEC(80), K_MSEC(80));

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
bool ei_environment_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    cb_sampler = callsampler;

    LOG_INF("sample_interval_ms = %f us\n", sample_interval_ms);

    ei_config_set_sample_interval(sample_interval_ms);

    EiDevice.set_state(eiStateSampling);

    return true;
}

bool ei_environment_fetch_sample(void)
{
    struct sensor_value raw_sample[ENVIRONMENT_VALUES_IN_SAMPLE];

    /* 
     * Sample fetching is done by env_timer in the background to provide
     * stable readings. It is required due to heating element in the gas sensor that
     * has direct impact on temp and hum readings.
     */

    //TODO: add channel IDs and coefficients to some array and do readings in a loop
    if (sensor_channel_get(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &raw_sample[0])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_AMBIENT_TEMP");
        return false;
    }

    if (sensor_channel_get(sensor_dev, SENSOR_CHAN_PRESS, &raw_sample[1])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_PRESS");
        return false;
    }

    if (sensor_channel_get(sensor_dev, SENSOR_CHAN_HUMIDITY, &raw_sample[2])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_HUMIDITY");
        return false;
    }

    if (sensor_channel_get(sensor_dev, SENSOR_CHAN_GAS_RES, &raw_sample[3])) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_GAS_RES");
        return false;
    }

    for(int i=0; i<ENVIRONMENT_VALUES_IN_SAMPLE; i++) {
        sample[i] = (float)sensor_value_to_double(&raw_sample[i]);
    }
    // sample 3 is SENSOR_CHAN_GAS_RES and we have to convert it to Mohm
    sample[3] = sample[3] / 100000.0f;

    return true;
}

/**
 * @brief      Get data from sensor, convert and call callback to handle
 */
void ei_environment_read_data(void)
{
    uint32_t start = ei_read_timer_ms();
    int32_t remaining;

    ei_environment_fetch_sample();

    cb_sampler((const void *)&sample[0], SIZEOF_ENVIRONMENT_VALUES_IN_SAMPLE);

    remaining = (ei_config_get_config()->sample_interval_ms)-(ei_read_timer_ms()-start);
    LOG_DBG("remaining = %d", remaining);
    if (remaining > 0) {
        ei_sleep(remaining);
    }
}

float *ei_fusion_environment_read_data(void)
{
    ei_environment_fetch_sample();

    return sample;
}

static void env_timer_handler(struct k_timer *dummy)
{
    /* fetching sample can't be done on interrupt level
     * so we have to submit work to thread level worker
     */
    k_work_submit(&env_work);
}

static void env_work_handler(struct k_work *work)
{
    if (sensor_sample_fetch(sensor_dev) < 0) {
        LOG_ERR("Sample fetch error");
    }
}