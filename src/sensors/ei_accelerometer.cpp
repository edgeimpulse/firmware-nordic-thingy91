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

