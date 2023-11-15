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
