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
LOG_MODULE_REGISTER(light, CONFIG_EI_LOG_LEVEL);
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "ei_light.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"

#define CHANNELS_NUM    4

const struct device *lightsens_dev;
const enum sensor_channel channels[CHANNELS_NUM] = {
    SENSOR_CHAN_RED,
    SENSOR_CHAN_GREEN,
    SENSOR_CHAN_BLUE,
    SENSOR_CHAN_IR
};

bool ei_light_init(void)
{
	lightsens_dev = DEVICE_DT_GET_ONE(rohm_bh1749);
	if (!lightsens_dev) {
		LOG_ERR("device %s not ready.", lightsens_dev->name);
		return false;
	}

    if(ei_add_sensor_to_fusion_list(light_sensor) == false) {
        LOG_ERR("failed to register light sensor!");
        return false;
    }

    return true;
}

float *ei_fusion_light_read_data(int n_samples)
{
    // EiDeviceNRF91* dev = static_cast<EiDeviceNRF91*>(EiDeviceInfo::get_device());

    int ret;
    struct sensor_value raw_sample[LIGHT_AXIS_SAMPLED];
    static float out_sample[LIGHT_AXIS_SAMPLED];
    EiDeviceInfo *dev = EiDeviceInfo::get_device();

    dev->set_state(eiStateIdle);

    memset(out_sample, 0, sizeof(float) * LIGHT_AXIS_SAMPLED);

    ret = sensor_sample_fetch(lightsens_dev);
    if (ret) {
        LOG_ERR("Sample fetch error: %d", ret);
        return out_sample;
    }

    for(unsigned int i = 0; i < CHANNELS_NUM; i++) {
        ret = sensor_channel_get(lightsens_dev, channels[i], &raw_sample[i]);
        if (ret) {
            LOG_ERR("Channel (%d) read error: %d", channels[i], ret);
            return out_sample;
        }
        out_sample[i] = (float)sensor_value_to_double(&raw_sample[i]);
    }

    return out_sample;
}
