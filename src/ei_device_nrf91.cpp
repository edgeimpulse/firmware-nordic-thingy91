/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
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
LOG_MODULE_REGISTER(ei_device, CONFIG_EI_LOG_LEVEL);
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <zephyr/kernel.h>
#include <modem/modem_info.h>
#include <zephyr/drivers/gpio.h>
#include "ei_device_nrf91.h"
#include "ei_memory.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

using namespace std;
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static void led_timer_handler(struct k_timer *dummy);
static void led_work_handler(struct k_work *work);
static void sampler_timer_handler(struct k_timer *dummy);
static void sampler_work_handler(struct k_work *work);
static void set_all_leds(uint8_t state);

K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
K_WORK_DEFINE(led_work, led_work_handler);
K_TIMER_DEFINE(sampler_timer, sampler_timer_handler, NULL);
K_WORK_DEFINE(sampler_work, sampler_work_handler);

static void led_work_handler(struct k_work *work)
{
    EiDeviceNRF91 *dev = static_cast<EiDeviceNRF91*>(EiDeviceInfo::get_device());
    EiState state = dev->get_state();
    static uint8_t animation = 0;
    static uint8_t blink;

    switch(state)
    {
        case eiStateErasingFlash:
            gpio_pin_set_dt(&led0, blink++ % 2);
            break;
        case eiStateSampling:
            gpio_pin_set_dt(&led1, blink++ % 2);
            break;
        case eiStateUploading:
            gpio_pin_set_dt(&led2, blink++ % 2);
            break;
        case eiStateFinished:
            blink = 0;
            if(animation == 0) {
                animation = 0x24;
            }
            break;
        default:
            break;
    }

    if(animation == 0) {
        return;
    }

    set_all_leds(animation & 0x0f);
    animation >>= 1;

    if(!animation) {
        dev->set_state(eiStateIdle);
    }
}

static void led_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&led_work);
}

static void sampler_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&sampler_work);
}

static void sampler_work_handler(struct k_work *work)
{
    EiDeviceNRF91 *dev = static_cast<EiDeviceNRF91*>(EiDeviceInfo::get_device());

#if MULTI_FREQ_ENABLED == 1
    if (dev->get_fusioning() == 1){
        dev->sample_read_callback();
    }
    else {
        uint8_t flag = 0;
        uint8_t i = 0;
        
        dev->actual_timer += dev->get_sample_interval();

        for (i = 0; i < dev->get_fusioning(); i++){
            if (((uint32_t)(dev->actual_timer % (uint32_t)dev->multi_sample_interval.at(i))) == 0) {
                flag |= (1<<i);
            }
        }
        if (dev->sample_multi_read_callback != nullptr)
        {
            dev->sample_multi_read_callback(flag);

        }
    }
#else
    dev->sample_read_callback();
#endif
}

/*
    state encodes state of LEDs
    bit 0 -> LED0
    bit 1 -> LED1
    ...
 */
static void set_all_leds(uint8_t state)
{
    gpio_pin_set_dt(&led0, state & (1 << 0));
    gpio_pin_set_dt(&led1, state & (1 << 1));
    gpio_pin_set_dt(&led2, state & (1 << 2));
}


EiDeviceNRF91::EiDeviceNRF91(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    load_config();

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);

    device_type = "NORDIC_THINGY91";
}

EiDeviceNRF91::~EiDeviceNRF91()
{

}

string EiDeviceNRF91::get_mac_address(void)
{
    return mac_address;
}

void EiDeviceNRF91::init_device_id(void)
{
    char temp[18];
    char buf[32];

    /* Read IMEI from modem since ID registers are not accessible from non-secure */
    int err = modem_info_init();
    if(err == 0) {
        err = modem_info_string_get(MODEM_INFO_IMEI, buf, sizeof(buf));
        if(err > 0) {
            // prepare buffer (fill with ':' and end with null char)
            memset(temp, 0, 18);
            memset(temp, ':', 17);
            // copy last 12 digits of IMEI to MAC-like string
            for(int i=0; i<6; i++) {
                memcpy(&temp[i*3], &buf[3 + i*2], 2);
            }
            device_id = string(temp);
            mac_address = string(temp);
            return;
        }
        else {
            LOG_ERR("Failed to get IMEI (%d)", err);
        }
    }
    else {
        LOG_ERR("Failed to initialize modem info");
    }

    // fallback to zero only ID
    device_id = string("00:00:00:00:00:00");
    mac_address = string("00:00:00:00:00:00");

    save_config();
}

/**
 * @brief get_device is a static method of EiDeviceInfo class
 * It is used to implement singleton paradigm, so we are returning
 * here pointer always to the same object (dev)
 *
 * @return EiDeviceInfo*
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    static EiFlashMemory memory(sizeof(EiConfig));
    static EiDeviceNRF91 dev(&memory);

    return &dev;
}

bool EiDeviceNRF91::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    this->sample_read_callback = sample_read_cb;

#if MULTI_FREQ_ENABLED == 1
    this->actual_timer = 0;
    this->fusioning = 1;
#endif

    k_timer_start(&sampler_timer, K_MSEC(sample_interval_ms), K_MSEC(sample_interval_ms));

    return true;
}

#if MULTI_FREQ_ENABLED == 1
bool EiDeviceNRF91::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;
    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++){
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }
    /* to improve, we consider just a 2 sensors case for now */
    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++){
        flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;

    k_timer_start(&sampler_timer, K_MSEC(sample_interval_ms), K_MSEC(sample_interval_ms));

    return true;
}
#endif

bool EiDeviceNRF91::stop_sample_thread(void)
{
    k_timer_stop(&sampler_timer);

#if MULTI_FREQ_ENABLED == 1
    this->actual_timer = 0;
    this->fusioning = 0;
#endif

    return true;
}

void EiDeviceNRF91::set_state(EiState state)
{
    this->state = state;

    set_all_leds(0x00);

    switch(state) {
        case eiStateErasingFlash:
        case eiStateSampling:
        case eiStateUploading:
        case eiStateFinished:
            k_timer_start(&led_timer, K_MSEC(250), K_MSEC(250));
            break;
        case eiStateIdle:
        default:
            k_timer_stop(&led_timer);
            set_all_leds(0x00);
            break;
    }
}

EiState EiDeviceNRF91::get_state(void)
{
    return this->state;
}

const std::string& EiDeviceNRF91::get_device_id(void)
{
    static bool initialized = false;

    if(!initialized) {
        init_device_id();
        initialized = true;
    }

    return device_id;
}

bool EiDeviceNRF91::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list = this->standalone_sensor_list;
    *sensor_list_size = this->standalone_sensor_num;

    return true;
}

uint32_t EiDeviceNRF91::get_data_output_baudrate(void)
{
    return 115200;
}
