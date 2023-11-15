/* Edge Impulse ingestion SDK
 * Copyright (c) 2023 EdgeImpulse Inc.
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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(environment, CONFIG_EI_LOG_LEVEL);
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "ei_environment.h"

#define CHANNELS_NUM    4

const struct device *envsens_dev;
const enum sensor_channel channels[CHANNELS_NUM] = {
    SENSOR_CHAN_AMBIENT_TEMP,
    SENSOR_CHAN_PRESS,
    SENSOR_CHAN_HUMIDITY,
    SENSOR_CHAN_GAS_RES
};

bool ei_environment_init(void)
{
	envsens_dev = DEVICE_DT_GET_ONE(bosch_bme680);
	if (!envsens_dev) {
		LOG_ERR("device %s not ready.", envsens_dev->name);
		return false;
	}

    if(ei_add_sensor_to_fusion_list(environment_sensor) == false) {
        LOG_ERR("failed to register environment sensor!");
        return false;
    }

    return true;
}

float *ei_fusion_environment_read_data(int n_samples)
{
    int ret;
    struct sensor_value raw_sample[ENVIRONMENT_AXIS_SAMPLED];
    static float out_sample[ENVIRONMENT_AXIS_SAMPLED];

    memset(out_sample, 0, sizeof(float) * ENVIRONMENT_AXIS_SAMPLED);

    ret = sensor_sample_fetch(envsens_dev);
    if (ret) {
        LOG_ERR("Sample fetch error: %d", ret);
        return out_sample;
    }

    for(unsigned int i = 0; i < CHANNELS_NUM; i++) {
        ret = sensor_channel_get(envsens_dev, channels[i], &raw_sample[i]);
        if (ret) {
            LOG_ERR("Channel (%d) read error: %d", channels[i], ret);
            return out_sample;
        }
        out_sample[i] = (float)sensor_value_to_double(&raw_sample[i]);
    }

    return out_sample;
}