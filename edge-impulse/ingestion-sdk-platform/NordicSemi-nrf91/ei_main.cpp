#include <stdio.h>

#include "ei_device_nordic_nrf91.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_classifier_porting.h"
#include "ei_main.h"
#include "ei_run_impulse_thread.h"
#include "../../repl/at_cmds.h"

#include "ei_inertialsensor.h"
#include "ei_lightsensor.h"
#include "ei_environmentsensor.h"
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
extern "C"
{
#include <modem/modem_key_mgmt.h>
};
#include <logging/log.h>

#define LOG_MODULE_NAME ei_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* help function for list files which is not currently implemented */
void list_files_help_fn(void (*data_fn)(char *))
{
    char no_files[] = "No files";
    data_fn(no_files);
}

void ei_init(void)
{
    int err;

    ei_printf("Hello from Edge Impulse Device SDK.\r\n"
              "Compiled on %s %s\r\n", __DATE__, __TIME__);

    /* Setup the inertial sensor */
    if(ei_inertial_init() == false) {
        LOG_ERR("Accelerometer sensor communication error occured\r\n");
    }

    /* Setup the light sensor */
    if(ei_lightsensor_init() == false) {
        LOG_ERR("Light sensor communication error occured");
    }

    /* Setup the env sensor */
    if(ei_environment_init() == false) {
        LOG_ERR("Environemt sensor communication error occured");
    }

    /* Intialize configuration */
    static ei_config_ctx_t config_ctx = { 0 };
    config_ctx.get_device_id = EiDevice.get_id_function();
    config_ctx.get_device_type = EiDevice.get_type_function();
    config_ctx.wifi_connection_status = EiDevice.get_wifi_connection_status_function(); 
    config_ctx.wifi_present = EiDevice.get_wifi_present_status_function();
    config_ctx.load_config = &ei_zephyr_flash_load_config;
    config_ctx.save_config = &ei_zephyr_flash_save_config;
    config_ctx.list_files = list_files_help_fn;
    config_ctx.read_buffer = EiDevice.get_read_sample_buffer_function();
    config_ctx.mqtt_connect = EiDevice.mqtt_connect();
    config_ctx.get_mqtt_connect = EiDevice.get_mqtt_status();


    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        ei_printf("Failed to initialize configuration (%d)\n", cr);
    }
    else {
        ei_printf("Loaded configuration\n");
    }

    /* Setup the command line commands */
    ei_at_register_generic_cmds();
    ei_at_cmd_register("RUNIMPULSE", "Run the impulse", ei_start_impulse);
    //ei_at_cmd_register("RUNIMPULSEDEBUG", "Run the impulse with extra debug output", run_nn_debug);
    // ei_at_cmd_register("RUNIMPULSECONT", "Run the impulse continuously", run_nn_continuous_normal);
    ei_printf("Type AT+HELP to see a list of commands.\r\n> ");

    EiDevice.set_state(eiStateFinished);

    err = nrf_modem_lib_init(NORMAL_MODE);
    if (err)
    {
        ei_printf("ERR: Failed to initialize modem library!");
    }

    err = at_cmd_init();
    if (err)
    {
        ei_printf("ERR: Failed to initialize AT commands, err %d\n", err);
    }

    err = at_notif_init();
    if (err)
    {
        ei_printf("ERR: Failed to initialize AT notifications, err %d\n", err);
    }
}

void ei_main(void)
{
    ei_command_line_handle();
    k_sleep(K_MSEC(10));
}
