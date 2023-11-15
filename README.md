# Edge Impulse firmware for Nordic Semiconductor Thingy:91

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Nordic Semiconductor Thingy91 development board. This combination supports all Edge Impulse device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board with Edge Impulse? No need to build this firmware. See the instructions for the [Thingy:91](https://docs.edgeimpulse.com/docs/development-platforms/officially-supported-mcu-targets/nordic-semi-thingy91) for a prebuilt image and instructions, or use the [data forwarder](https://docs.edgeimpulse.com/docs/tools/edge-impulse-cli/cli-data-forwarder) to capture data from any sensor.

## Building the device firmware (native)

1. Install the [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-2.4.0/page/nrf/getting_started/installing.html) in a *separate* folder from this repository (e.g. `~/repos/ncs`).

2. Clone this repository:

    ```bash
    $ git clone https://github.com/edgeimpulse/firmware-nordic-thingy91
    ```

3. (optional) Build the `connectivity bridge`:
    > **Note:** If your Thingy:91 is not registering two serial ports after connecting to PC through Micro USB cable, then you have to flash the `connectivty bridge` firmware. Otherwise, go to the next step.

    ```bash
    $ cd connectivity_bridge/
    $ west build -b thingy91_nrf52840
    ```

4. Go back to the `firmware-nordic-thingy91` directory (don't run this command from the `connectivity_bridge` folder).
5. Build the application:

    ```bash
    $ west build -b thingy91_nrf9160_ns
    ```

## Building the device firmware (Docker)

1. Clone this repository:

    ```bash
    $ git clone https://github.com/edgeimpulse/firmware-nordic-thingy91
    ```

2. Build the Docker container:

    ```bash
    $ docker build -t edge-impulse-nordic .
    ```

3. (optional) Build the `connectivity_bridge`:
    > **Note:** If your Thingy:91 is not registering two serial ports after connecting to PC through Micro USB cable, then you have to flash the `connectivty bridge` firmware. Otherwise, go to the next step.

    ```bash
    $ docker run --rm -v $PWD:/app -w /app/connectivity_bridge edge-impulse-nordic-thingy91 west build -b thingy91_nrf52840
    ```

4. Build the application:

    ```
    $ docker run --rm -v $PWD:/app edge-impulse-nordic-thingy91 west build -b thingy91_nrf9160_ns
    ```

## Flashing

1. (optional) To flash the `connectivity bridge` follow [this guide](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/device_guides/working_with_nrf/nrf91/thingy91_gsg.html#updating_the_firmware_in_the_nrf52840_soc) and use the file: `connectivity_bridge/build/zephyr/app_signed.hex`.

2. To flash the application follow [this guide](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/device_guides/working_with_nrf/nrf91/thingy91_gsg.html#program_the_nrf9160_sip_application) and use the file `build/zephyr/app_signed.hex`.

That's it. You can now connect to the device through the Edge Impulse CLI or through WebUSB, as described [here](https://docs.edgeimpulse.com/docs/development-platforms/officially-supported-mcu-targets/nordic-semi-nrf5340-dk#4.-setting-keys).

## Updating your ML model

1. [Train a model in Edge Impulse](https://docs.edgeimpulse.com).
2. On the **Deployment** page in the Studio, export as a C++ library.
3. Remove the content of `ei-model` directory in this repository.
4. Extract the downloaded zip with the C++ library into `ei-model` directory.
5. Rebuild the application.

## Using Remote Ingestion

This firmware is equipped with the Remote Ingestion functionality. It allows you to connect your device to the internet using the LTE connection and start ingesting data remotely from the Edge Impulse Studio!

To build the firmware with the Remote Ingestion, follow the steps above but insted of command:

```
west build -b thingy91_nrf9160_ns
```

Run:

```
west build -b thingy91_nrf9160_ns -- -DEXTRA_CONF_FILE=overlay-remote-ingestion.conf
```

And then:
1. Flash the board and connect for the first time to studio.
2. Power off the board and insert the SIM card.
    > **Note:** Make sure your SIM card is active. You can use the iBasis SIM card shipped with the Thingy:91 or any other SIM card that supports LTE-M and/or NB-IoT.
3. Power on the board and wait for the connection to be established (Thingy will blink RGB LED).
4. Go to your project in Studio and click on **Devices** tab, you should see your device with green mark.

## Troubleshooting

1. In case of any issues, the Thingy:91 with Edge Impulse firmware is exposing a serial console on the first UART port (connection parameters 115200bps 8N1). Open the serial port and reset the device to see the boot messages.
2. If you are using the Remote Ingestion functionality and have problems with connection, in the `overlay-remote-ingestion.conf` file change:

    ```
    CONFIG_REMOTE_INGESTION_LOG_LEVEL_WRN=y
    ```

    to:

    ```
    CONFIG_REMOTE_INGESTION_LOG_LEVEL_DBG=y
    ```

    After rebuilding the firmware and flashing, you should see more detailed logs in the serial console.

3. Remember that device configuration is stored in the nRF9160 SoC Flash, so after every flashing your device is loosing API KEY and other settings. You have to connect the device to Edge Impulse through `edge-impulse-daemon` once again to restore the configuration.