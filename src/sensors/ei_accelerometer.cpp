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
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(accelerometer, CONFIG_EI_LOG_LEVEL);
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "ei_accelerometer.h"

const struct device *accel_dev;

bool ei_accelerometer_init(void)
{
	struct sensor_value odr_attr, fs_attr;

    accel_dev = DEVICE_DT_GET(DT_ALIAS(accelerometer));
    if (!device_is_ready(accel_dev))
    {
        LOG_ERR("device %s not ready.", accel_dev->name);
        return false;
    }

    if(ei_add_sensor_to_fusion_list(accelerometer_sensor) == false) {
        LOG_ERR("failed to register accelerometer sensor!");
        return false;
    }

    return true;
}


float *ei_fusion_accelerometer_read_data(int n_samples)
{
    struct sensor_value accel_raw[ACCEL_AXIS_SAMPLED];
    static float accel_mg[ACCEL_AXIS_SAMPLED];

    memset(accel_mg, 0, ACCEL_AXIS_SAMPLED * sizeof(float));

    if (sensor_sample_fetch(accel_dev) < 0) {
        LOG_ERR("IIS2DLPC Sensor sample update error");
    }
    else {
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_raw);
        for (unsigned int i = 0; i < ACCEL_AXIS_SAMPLED; i++) {
            accel_mg[i] = sensor_value_to_double(&accel_raw[i]);
        }
    }

    return accel_mg;
}

