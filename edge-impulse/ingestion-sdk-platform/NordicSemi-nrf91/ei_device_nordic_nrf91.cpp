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
#include "ei_device_nordic_nrf91.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_inertialsensor.h"
#include "ei_lightsensor.h"
#include "ei_environmentsensor.h"
#include "repl.h"
#include <cstdarg>
#include "math.h"
#include <sys/printk.h>
#include <drivers/uart.h>
#include <modem/modem_info.h>
#include "ei_run_impulse_thread.h"

extern "C" {
    #include "connectivity.h"
};

extern "C" void ei_led_state_control(void);

/* Constants --------------------------------------------------------------- */
#define EI_LED_RED      BOARD_ledSetLedOn(1, 0, 0)
#define EI_LED_GREEN    BOARD_ledSetLedOn(0, 1, 0)
#define EI_LED_BLUE     BOARD_ledSetLedOn(0, 0, 1)
#define EI_LED_YELLOW   BOARD_ledSetLedOn(0, 1, 1)
#define EI_LED_OFF      BOARD_ledSetLedOn(0, 0, 0)

/** Led device struct */
const struct device *led_dev;
const struct device *uart;

/** Device type */
static const char *ei_device_type = "NORDIC       ";

/** Device object, for this class only 1 object should exist */
EiDeviceNRF91 EiDevice;

static tEiState ei_program_state = eiStateIdle;

/* Private function declarations ------------------------------------------- */
static int get_id_c(uint8_t out_buffer[32], size_t *out_size);
static int get_type_c(uint8_t out_buffer[32], size_t *out_size);
static bool get_wifi_connection_status_c(void);
static bool get_wifi_present_status_c(void);
static bool read_sample_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t));
static bool mqtt_connect_c(void);
static bool get_mqtt_status_c(void);
static void zephyr_timer_handler(struct k_timer *dummy);

/** Zephyr timer */
K_TIMER_DEFINE(led_timer, zephyr_timer_handler, NULL);

/* Public functions -------------------------------------------------------- */
int str2int(const char* str, int len);


EiDeviceNRF91::EiDeviceNRF91(void)
{
    /* init device name to null only, it means it is not initialized so we have
    to do that on first access */
    memset(ei_device_id, 0, DEVICE_ID_MAX_SIZE);
    memset(ei_imei, 0, DEVICE_ID_MAX_SIZE);
}

/**
 * @brief      Get the device ID (last 7 digits of IMEI)
 *
 * @param      out_buffer  Destination array for id string
 * @param      out_size    Length of id string
 *
 * @return     0 on success, negtive number on error
 */
int EiDeviceNRF91::get_id(uint8_t out_buffer[32], size_t *out_size)
{
    int ret;

    if(!this->ei_device_id[0])
    {
        ret = this->id_init();
        if(ret)
            return ret;
    }

    size_t length = strlen(this->ei_device_id);

    if (length < 32)
    {
        memcpy(out_buffer, this->ei_device_id, length);

        *out_size = length;
        return 0;
    }
    else
    {
        *out_size = 0;
        return -1;
    }
}

/**
 * @brief      Gets the identifier pointer.
 *
 * @return     The identifier pointer.
 */
const char *EiDeviceNRF91::get_id_pointer(void)
{
    /* init ei_device_id if set to null bytes */
    if(!this->ei_device_id[0]) {
        this->id_init();
    }
    return (const char *)this->ei_device_id;
}

/**
 * @brief      Gets the pointer to IMEI string.
 *
 * @return     The pointer to IMEI string.
 */
const char *EiDeviceNRF91::get_imei_pointer(void)
{
    /* init ei_device_id if set to null bytes */
    if(!this->ei_imei[0]) {
        this->id_init();
    }
    return (const char *)this->ei_imei;
}

/**
 * @brief      Copy device type in out_buffer & update out_size
 *
 * @param      out_buffer  Destination array for device type string
 * @param      out_size    Length of string
 *
 * @return     -1 if device type string exceeds out_buffer
 */
int EiDeviceNRF91::get_type(uint8_t out_buffer[32], size_t *out_size)
{
    return get_type_c(out_buffer, out_size);
}

/**
 * @brief      Gets the type pointer.
 *
 * @return     The type pointer.
 */
const char *EiDeviceNRF91::get_type_pointer(void)
{
    return (const char *)ei_device_type;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceNRF91::get_wifi_connection_status(void)
{
    return false;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceNRF91::get_wifi_present_status(void)
{
    return false;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceNRF91::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    /* all sensors are handled by sensors fusion subsystem */
    *sensor_list      = NULL;
    *sensor_list_size = 0;

    return false;
}

/**
 * @brief      Device specific delay ms implementation
 *
 * @param[in]  milliseconds  The milliseconds
 */
void EiDeviceNRF91::delay_ms(uint32_t milliseconds)
{
    k_msleep((int32_t)milliseconds);
}

/**
 * @brief      Device specific get ms form application start
 *
 * @return    The milliseconds form start
 */
uint64_t EiDeviceNRF91::get_ms(void)
{
    return (uint64_t)k_uptime_get();
}

void EiDeviceNRF91::set_state(tEiState state)
{
    ei_program_state = state;

    if(state == eiStateFinished) {
        EI_LED_OFF;
        delay_ms(300);
        EI_LED_BLUE;
        delay_ms(300);
        EI_LED_GREEN;
        delay_ms(300);
        EI_LED_YELLOW;
        delay_ms(300);
        EI_LED_RED;
        delay_ms(300);
        EI_LED_OFF;

        ei_program_state = eiStateIdle;
    }
}

/**
 * @brief      Fetch device ID and parse to string
 *
 * @return     0 If successful
 * @return     -ENXIO If not successful
 *
 */
int EiDeviceNRF91::id_init(void)
{
    /* Read IMEI from modem since ID registers are not accessible from non-secure */
    int err = modem_info_init();
    if (err)
    {
        return -ENXIO;
    }

    err = modem_info_string_get(MODEM_INFO_IMEI, this->ei_imei, DEVICE_ID_MAX_SIZE);
    if (err < 0)
    {
        return -ENXIO;
    }

    /* Setup device ID from last 7 IMEI digits, which are unique per device */
    snprintf(&ei_device_id[0], DEVICE_ID_MAX_SIZE,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             str2int(&this->ei_imei[8 + 0], 2),
             str2int(&this->ei_imei[8 + 2], 1),
             str2int(&this->ei_imei[8 + 3], 1),
             str2int(&this->ei_imei[8 + 4], 1),
             str2int(&this->ei_imei[8 + 5], 1),
             str2int(&this->ei_imei[8 + 6], 1));

    return 0;
}

/**
 * @brief      Connect to MQTT broker
 *
 */
c_callback_status EiDeviceNRF91::mqtt_connect()
{
    return &mqtt_connect_c;
}

/**
 * @brief      Get connection status to MQQTT broker
 *
 */
c_callback_status EiDeviceNRF91::get_mqtt_status()
{
    return &get_mqtt_status_c;
}

/**
 * @brief      Get a C callback for the get_id method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceNRF91::get_id_function(void)
{
    return &get_id_c;
}

/**
 * @brief      Get a C callback for the get_type method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceNRF91::get_type_function(void)
{
    return &get_type_c;
}

/**
 * @brief      Get a C callback for the get_wifi_connection_status method
 *
 * @return     Pointer to c get function
 */
c_callback_status EiDeviceNRF91::get_wifi_connection_status_function(void)
{
    return &get_wifi_connection_status_c;
}

/**
 * @brief      Get a C callback for the wifi present method
 *
 * @return     The wifi present status function.
 */
c_callback_status EiDeviceNRF91::get_wifi_present_status_function(void)
{
    return &get_wifi_present_status_c;
}

/**
 * @brief      Get a C callback to the read sample buffer function
 *
 * @return     The read sample buffer function.
 */
c_callback_read_sample_buffer EiDeviceNRF91::get_read_sample_buffer_function(void)
{
    return &read_sample_buffer;
}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length) 
{
    for( int i = 0; i < length; i++) {
        printf("%c", *(data++));
    }
}


void ei_printfloat(int n_decimals, int n, ...)
{
    int i;
    double val;

    char buffer[32];

    sprintf(buffer, "%%.%df", n_decimals);

    va_list vl;
    va_start(vl,n);
    for (i=0;i<n;i++){
        val=va_arg(vl,double);
        ei_printf(buffer, val);
    }
    va_end(vl);
}

void ei_printf_float(float f)
{
    float n = f;

    static double PRECISION = 0.00001;
    static int MAX_NUMBER_STRING_SIZE = 32;

    char s[MAX_NUMBER_STRING_SIZE];

    if (n == 0.0) {
        ei_printf("0.00000");
    }
    else {
        int digit, m;//, m1;
        char *c = s;
        int neg = (n < 0);
        if (neg) {
            n = -n;
        }
        // calculate magnitude
        m = log10(n);
        if (neg) {
            *(c++) = '-';
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0) {
                *(c++) = '.';
            }
            m--;
        }
        *(c) = '\0';
        ei_write_string(s, c - s);
    }    
}

/**
 * @brief      Get characters for uart pheripheral and send to repl
 */
void ei_command_line_handle(void)
{
    uint8_t data; 
    int num_bytes = uart_fifo_read(uart, &data, 1);
    while(num_bytes == 1) {
        if(ei_is_impulse_running() && data == 'b') {
            ei_stop_impulse();
        }
        else {
            rx_callback(data);
        }
        num_bytes = uart_fifo_read(uart, &data, 1);
    }
}

bool ei_user_invoke_stop(void)
{
    bool stop_found = false;
    char data = uart_getchar();
    
    while(data != 0xFF) {
        if(data == 'b') {
            stop_found = true;
            break;
        }
        data = uart_getchar();
    }

    return stop_found;
}

/**
 * @brief      Check if new serial data is available
 *
 * @return     Returns number of available bytes
 */
int ei_get_serial_available(void) {
    return 0;//Serial.available();
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void) {
    return 0;//Serial.read();
}

/* Private functions ------------------------------------------------------- */
static int get_id_c(uint8_t out_buffer[32], size_t *out_size)
{
    return EiDevice.get_id(out_buffer, out_size);
}

static int get_type_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_type);

    if(length < 32) {
        memcpy(out_buffer, ei_device_type, length);

        *out_size = length;
        return 0;
    }
    else {
        *out_size = 0;
        return -1;
    }
}

static bool get_wifi_connection_status_c(void)
{
    return false;
}

static bool get_wifi_present_status_c(void)
{
    return false;
}

static bool mqtt_connect_c()
{
    const char *imei = EiDevice.get_imei_pointer();

    if(strlen(imei) < 8) {
        ei_printf("ERR: ID too short!\n");
        return false;
    }

    if(ei_mqtt_connect(&imei[8])) {
        return false;
    }
    else {
        return true;
    }
}

static bool get_mqtt_status_c()
{
    return ei_get_mqtt_connected();
}

/**
 * @brief      Read samples from sample memory and send to data_fn function
 *
 * @param[in]  begin    Start address
 * @param[in]  length   Length of samples in bytes
 * @param[in]  data_fn  Callback function for sample data
 *
 * @return     false on flash read function
 */
static bool read_sample_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t))
{
    size_t pos = begin;
    size_t bytes_left = length;
    bool retVal;

    EiDevice.set_state(eiStateUploading);

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            retVal = true;
            break;
        }

        int r = ei_zephyr_flash_read_samples(buffer, pos, bytes_to_read);
        if (r != 0) {
            retVal = false;
            break;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    EiDevice.set_state(eiStateFinished);

    return retVal;
}

void ei_led_state_control(void)
{
    static char toggle = 0;


    if(toggle) {
        switch(ei_program_state)
        {
            case eiStateErasingFlash:   EI_LED_RED;     break;
            case eiStateSampling:       EI_LED_BLUE;    break;
            case eiStateUploading:      EI_LED_GREEN;   break;
            default: break;
        }
    }
    else {
        if(ei_program_state != eiStateFinished) {
            EI_LED_OFF;
        }
    }
    toggle ^= 1;
}

/**
 * @brief      Peridioc (200ms) handler for the LED's
 *
 * @param      dummy  The dummy
 */
static void zephyr_timer_handler(struct k_timer *dummy)
{
    ei_led_state_control();
}

/**
 * @brief      Sets development kit LEDs on and off
 *
 * @param[in]  led1     set LED1 on and off (true/false)
 * @param[in]  led2     set LED2 on and off (true/false)
 * @param[in]  led3     set LED3 on and off (true/false)
 *
 */
void BOARD_ledSetLedOn(uint8_t led1, uint8_t led2, uint8_t led3)
{
    /** set led1 */
    if(led1 < 1) gpio_pin_set(led_dev, PIN_LED0, 0); else gpio_pin_set(led_dev, PIN_LED0, 1);
    /** set led2 */
    if(led2 < 1) gpio_pin_set(led_dev, PIN_LED1, 0); else gpio_pin_set(led_dev, PIN_LED1, 1);
    /** set led3 */
    if(led3 < 1) gpio_pin_set(led_dev, PIN_LED2, 0); else gpio_pin_set(led_dev, PIN_LED2, 1);
}

/**
 * @brief      Init development kit LEDs
 *
 * @return     0 If successful
 * @return     EIO If not successful
 *
 */
int BOARD_ledInit(void)
{
    int ret = 0;

    led_dev = device_get_binding(LED_DEVICE);
    if(led_dev == NULL)
    {
        return EIO;
    }
    /** init gpio for led0 */
    ret = gpio_pin_configure(led_dev, PIN_LED0, GPIO_OUTPUT_INACTIVE | FLAGS_LED0);
    if(ret < 0)
    {
        return EIO;
    }
    /** init gpio for led1 */
    ret = gpio_pin_configure(led_dev, PIN_LED1, GPIO_OUTPUT_INACTIVE | FLAGS_LED1);
    if(ret < 0)
    {
        return EIO;
    }
    /** init gpio for led2 */
    ret = gpio_pin_configure(led_dev, PIN_LED2, GPIO_OUTPUT_INACTIVE | FLAGS_LED2);
    if(ret < 0)
    {
        return EIO;
    }

    /* start periodic timer that expires once every 200 ms */
    k_timer_start(&led_timer, K_MSEC(200), K_MSEC(200));

    return ret;
}

/**
 * @brief      Init development kit UART
 *
 * @return     0 If successful
 * @return     -ENXIO If not successful
 *
 */
int uart_init(void)
{
	int err = 0;

	uart = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
	if (!uart) {
		return -ENXIO;
	}
    
    return err;
}

/**
 * @brief      Get char from UART
 *
 * @return     rcv_char If successful
 * @return     0xFF If not successful
 *
 */
char uart_getchar(void)
{
    unsigned char rcv_char;

    if (!uart_poll_in(uart, &rcv_char)) {
        return rcv_char;
    }
    else{
        return 0xFF;
    }
}

/**
 * @brief      Get char from UART
 *
 * @param[in] send_char Character to be sent over UART
 *
 */
void uart_putchar(char send_char)
{
    uart_poll_out(uart, send_char);
}

// helper for bellow
int str2int(const char* str, int len)
{
    int i;
    int ret = 0;
    for(i = 0; i < len; ++i)
    {
        ret = ret * 10 + (str[i] - '0');
    }
    return ret;
}
