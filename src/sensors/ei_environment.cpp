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