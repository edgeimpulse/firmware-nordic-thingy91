#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "ei_run_impulse_thread.h"
#include "ei_inertialsensor.h"
#include "ei_environmentsensor.h"
#include "ei_lightsensor.h"
#include "ei_device_nordic_nrf91.h"
#include <zephyr.h>
extern "C" {
    #include "connectivity.h"
};

#include <logging/log.h>
#define LOG_MODULE_NAME run_impulse
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static volatile bool inference_active = false;
static float samples_circ_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int samples_wr_index = 0;

bool samples_callback(const void *raw_sample, uint32_t raw_sample_size)
{
    float *sample = (float *)raw_sample;

    for(int i = 0; i < (int)(raw_sample_size / sizeof(float)); i++) {
        samples_circ_buff[samples_wr_index + i] = sample[i];
    }   

    samples_wr_index += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
    if(samples_wr_index >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        /* start from beginning of the circular buffer */
        samples_wr_index = 0;
    }

    return true;
}

static void display_results(ei_impulse_result_t* result)
{
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
        result->timing.dsp, result->timing.classification, result->timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {            
        ei_printf("    %s: \t", result->classification[ix].label);
        ei_printf_float(result->classification[ix].value);
        ei_printf("\r\n");
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: ");
    ei_printf_float(result->anomaly);
    ei_printf("\r\n");        
#endif
}

static void send_results(ei_impulse_result_t* result)
{
    float max = 0.0f;
    size_t max_ix;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {       
        if (result->classification[ix].value > max) {
            max = result->classification[ix].value;
            max_ix = ix;
        }
    }

    ei_mqtt_publish(result->classification[max_ix].label);
}

void ei_inference_thread(void* param1, void* param2, void* param3)
{
    signal_t signal;
    const float sample_length = 1000.0f * static_cast<float>(EI_CLASSIFIER_RAW_SAMPLE_COUNT) /
                        (1000.0f / static_cast<float>(EI_CLASSIFIER_INTERVAL_MS));

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER
    const uint32_t delay = sample_length < 2000 ? sample_length : 2000;
#elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ENVIRONMENTAL
    const uint32_t delay = 5000;
// #elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_LIGHT
    // const uint32_t delay = 2000;
#endif

    LOG_INF("Sample length: %f ms", sample_length);
    LOG_INF("Delay time: %d ms", delay);

    while (1) {
        if (!inference_active) {
            k_sleep(K_MSEC(200));
            // LOG_INF("Waiting for running...");
            continue;
        }

        // summary of inferencing settings (from model_metadata.h)
        ei_printf("Inferencing settings:\n");
        ei_printf("\tInterval: ");
        ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
        ei_printf("ms.\n");
        ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        ei_printf("\tSample length: ");
        ei_printf_float(sample_length);
        ei_printf("ms.\n");
        ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                                sizeof(ei_classifier_inferencing_categories[0]));
        ei_printf("Starting inferencing, press 'b' to break\n");

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER
        ei_inertial_sample_start(&samples_callback, EI_CLASSIFIER_INTERVAL_MS);
#elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ENVIRONMENTAL
        ei_environment_sample_start(&samples_callback, EI_CLASSIFIER_INTERVAL_MS);
// #elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_LIGHT
//         ei_lightsensor_sample_start(&samples_callback, EI_CLASSIFIER_INTERVAL_MS);
#endif

        /* Fill samples buffer */
        ei_printf("Starting inferencing in %d seconds...\n", (EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_INTERVAL_MS / 1000));
        for(int i = 0; i < EI_CLASSIFIER_RAW_SAMPLE_COUNT; i++) {
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER
            ei_inertial_read_data();
#elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ENVIRONMENTAL
            ei_environment_read_data();
// #elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_LIGHT
//                 ei_lightsensor_read_data();
#endif
            // check just to be sure nobody stopped us during samples collecting
            if(!inference_active) {
                break;
            }
        }

        while (inference_active) {
            /* shift circular buffer, so the newest data will be the first */
            numpy::roll(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, (-samples_wr_index));
            /* reset wr index, the oldest data will be overwritten */
            samples_wr_index = 0;

            // Create a data structure to represent this window of data
            int err = numpy::signal_from_buffer(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
            if (err != 0) {
                LOG_ERR("signal_from_buffer failed (%d)\n", err); 
            }

            // run the impulse: DSP, neural network and the Anomaly algorithm
            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result);
            if (ei_error != EI_IMPULSE_OK) {
                LOG_ERR("Failed to run impulse (%d)", ei_error);
                break;
            }

            /* check if user stopped inferencing */
            if(inference_active) {
                display_results(&result);
                send_results(&result);
            }

            /* wait for a new data to be added to circular buffer */
            ei_printf("Starting inferencing in %d seconds...\n", (int)(delay / 1000));
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER
            for(uint32_t i = 0; i < EI_CLASSIFIER_RAW_SAMPLE_COUNT; i++) {
                ei_inertial_read_data();
#elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ENVIRONMENTAL
            //TODO: change fixed 5 seconds into parameter from model (EI_CLASSIFIER_SLICE_SIZE?)
            for(uint32_t i = 0; i < 5; i++) {
                ei_environment_read_data();
// #elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_LIGHT
                // ei_lightsensor_read_data();
#endif
                // check just to be sure nobody stopped us during samples collecting
                if(!inference_active) {
                    break;
                }
            }
        }
        ei_stop_impulse();
    }
}

void ei_start_impulse(void) 
{
    LOG_INF("Starting impulse!");

    inference_active = true;
}

void ei_stop_impulse(void) 
{
    inference_active = false;

    /* reset samples buffer */
    samples_wr_index = 0;

    EiDevice.set_state(eiStateFinished);
    ei_printf("Inferencing stopped by user\r\n");
}

bool ei_is_impulse_running(void)
{
    return inference_active;
}

K_THREAD_DEFINE(ei_inference, CONFIG_EI_INFERENCE_THREAD_STACK,
                ei_inference_thread, NULL, NULL, NULL,
                CONFIG_EI_INFERENCE_THREAD_PRIO, 0, 0);
