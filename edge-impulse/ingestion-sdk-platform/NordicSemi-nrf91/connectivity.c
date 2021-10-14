#include <zephyr.h>
#include "connectivity.h"
#include <stdio.h>
#include <drivers/uart.h>
#include <string.h>
#include <random/rand32.h>
#include <net/mqtt.h>
#include <net/socket.h>
#include <modem/at_cmd.h>
#include <modem/lte_lc.h>

#define MQTT_MESSAGE_BUFFER_SIZE    128
#define MQTT_PAYLOAD_BUFFER_SIZE    128
#define MQTT_MSG                    "Hello from Thingy91"
#define STACKSIZE                   1024
#define PRIORITY                    7

/* The mqtt client struct */
static struct mqtt_client client;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

/* File descriptor */
static struct pollfd fds;
K_THREAD_STACK_DEFINE(mqtt_handler_stack, STACKSIZE);
// struct k_thread mqtt_handler_data;
// k_tid_t mqtt_handler_id;
void mqtt_handler(void *p1, void *p2, void *p3);
static bool mqtt_active = false;
static bool mqtt_connected = false;

/* Buffers for MQTT client. */
static uint8_t rx_buffer[MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t tx_buffer[MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t payload_buf[MQTT_PAYLOAD_BUFFER_SIZE];

#if defined(CONFIG_NRF_MODEM_LIB)
#define IMEI_LEN 15
#define CGSN_RESPONSE_LENGTH 19
#define CLIENT_ID_LEN sizeof("nrf-") + IMEI_LEN
#else
#define RANDOM_LEN 10
#define CLIENT_ID_LEN sizeof(CONFIG_BOARD) + 1 + RANDOM_LEN
#endif /* defined(CONFIG_NRF_MODEM_LIB) */

/**@brief Function to read the published payload.
 */
static int publish_get_payload(struct mqtt_client *c, size_t length)
{
    if (length > sizeof(payload_buf))
    {
        return -EMSGSIZE;
    }

    return mqtt_readall_publish_payload(c, payload_buf, length);
}

/**@brief Function to publish data on the configured topic
 */
static int data_publish(struct mqtt_client *c, enum mqtt_qos qos,
                        uint8_t *data, size_t len)
{
    struct mqtt_publish_param param;

    param.message.topic.qos = qos;
    param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC_PREFIX;
    param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC_PREFIX);
    param.message.payload.data = data;
    param.message.payload.len = len;
    param.message_id = sys_rand32_get();
    param.dup_flag = 0;
    param.retain_flag = 0;

    return mqtt_publish(c, &param);
}

/**@brief MQTT client event handler
 */
void mqtt_evt_handler(struct mqtt_client *const c,
                      const struct mqtt_evt *evt)
{
    int err;

    switch (evt->type)
    {
    case MQTT_EVT_CONNACK:
        if (evt->result != 0)
        {
            printk("ERR: MQTT connect failed: %d\n", evt->result);
            break;
        }

        break;

    case MQTT_EVT_DISCONNECT:
        printk("ERR: MQTT client disconnected: %d\n", evt->result);
        break;

    case MQTT_EVT_PUBLISH:
    {
        const struct mqtt_publish_param *p = &evt->param.publish;

        err = publish_get_payload(c, p->message.payload.len);

        if (p->message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE)
        {
            const struct mqtt_puback_param ack = {
                .message_id = p->message_id};

            /* Send acknowledgment. */
            mqtt_publish_qos1_ack(&client, &ack);
        }

        if (err >= 0)
        {
            // data_print("Received: ", payload_buf,
            //            p->message.payload.len);
            /* Echo back received data */
            data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
                         payload_buf, p->message.payload.len);
        }
        else
        {
            printk("ERR: publish_get_payload failed: %d\n", err);
            printk("ERR: Disconnecting MQTT client...\n");

            err = mqtt_disconnect(c);
            if (err)
            {
                printk("ERR: Could not disconnect: %d\n", err);
            }
        }
    }
    break;

    case MQTT_EVT_PUBACK:
        if (evt->result != 0)
        {
            printk("MQTT PUBACK error: %d\n", evt->result);
            break;
        }

        break;

    case MQTT_EVT_SUBACK:
        if (evt->result != 0)
        {
            printk("MQTT SUBACK error: %d\n", evt->result);
            break;
        }

        break;

    case MQTT_EVT_PINGRESP:
        if (evt->result != 0)
        {
            printk("ERR: MQTT PINGRESP error: %d\n", evt->result);
        }
        break;

    default:
        printk("ERR: Unhandled MQTT event type: %d\n", evt->type);
        break;
    }
}

/**@brief Initialize the file descriptor structure used by poll.
 */
static int fds_init(struct mqtt_client *c)
{
    if (c->transport.type == MQTT_TRANSPORT_NON_SECURE)
    {
        fds.fd = c->transport.tcp.sock;
    }
    else
    {
        return -ENOTSUP;
    }

    fds.events = POLLIN;

    return 0;
}

/**@brief Resolves the configured hostname and
 * initializes the MQTT broker structure
 */
static int broker_init(void)
{
    int err;
    struct addrinfo *result;
    struct addrinfo *addr;
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM};

    err = getaddrinfo(CONFIG_MQTT_BROKER_HOSTNAME, NULL, &hints, &result);
    if (err)
    {
        printk("ERR: getaddrinfo failed: %d\n", err);
        return -ECHILD;
    }

    addr = result;

    /* Look for address of the broker. */
    while (addr != NULL)
    {
        /* IPv4 Address. */
        if (addr->ai_addrlen == sizeof(struct sockaddr_in))
        {
            struct sockaddr_in *broker4 =
                ((struct sockaddr_in *)&broker);
            char ipv4_addr[NET_IPV4_ADDR_LEN];

            broker4->sin_addr.s_addr =
                ((struct sockaddr_in *)addr->ai_addr)
                    ->sin_addr.s_addr;
            broker4->sin_family = AF_INET;
            broker4->sin_port = htons(CONFIG_MQTT_BROKER_PORT);

            inet_ntop(AF_INET, &broker4->sin_addr.s_addr,
                      ipv4_addr, sizeof(ipv4_addr));

            break;
        }
        else
        {
            printk("ERR: ai_addrlen = %u should be %u or %u\n",
                    (unsigned int)addr->ai_addrlen,
                    (unsigned int)sizeof(struct sockaddr_in),
                    (unsigned int)sizeof(struct sockaddr_in6));
        }

        addr = addr->ai_next;
    }

    /* Free the address. */
    freeaddrinfo(result);

    return err;
}

/**@brief Initialize the MQTT client structure
 */
static int client_init(struct mqtt_client *client)
{
    int err;

    mqtt_client_init(client);

    err = broker_init();
    if (err)
    {
        printk("ERR: Failed to initialize broker connection\n");
        return err;
    }

    /* MQTT client configuration */
    client->broker = &broker;
    client->evt_cb = mqtt_evt_handler;
    client->password = NULL;
    client->user_name = NULL;
    client->protocol_version = MQTT_VERSION_3_1_1;

    /* MQTT buffers configuration */
    client->rx_buf = rx_buffer;
    client->rx_buf_size = sizeof(rx_buffer);
    client->tx_buf = tx_buffer;
    client->tx_buf_size = sizeof(tx_buffer);

    /* MQTT transport configuration */
    client->transport.type = MQTT_TRANSPORT_NON_SECURE;

    return err;
}

int ei_mqtt_connect(const char* client_id)
{
    int err;
    static char client_name[32] = "nRF91-";

    if(mqtt_active) {
        return 0;
    }

    if(strlen(client_id) > 24) {
        printk("ERR: client_id too long! (%d > 24)\n", strlen(client_id));
        return -51;
    }

    err = lte_lc_init_and_connect();
    if (err) {
        printk("ERR: Failed to connect to the LTE network, err %d\n", err);
        return -52;
    }

    err = client_init(&client);
    if (err) {
        printk("ERR: client_init: %d\n", err);
        return -53;
    }

    /* append client ID to known prefix */
    strncpy(&client_name[6], client_id, strlen(client_id));

    client.client_id.utf8 = client_name;
    client.client_id.size = strlen(client_name);

    mqtt_active = true;

    return 0;
}

void mqtt_handler(void * p1, void * p2, void * p3)
{
    int err;

    while(1) {
        if(!mqtt_active) {
            k_sleep(K_MSEC(100));
            // printk("MQTT not active...\n");
            continue;
        }

        err = mqtt_connect(&client);
        if (err != 0)
        {
            printk("ERR: mqtt_connect %d\n", err);
            printk("Reconnecting...");
            k_sleep(K_SECONDS(2));
            /* try again */
            continue;
        }

        err = fds_init(&client);
        if (err != 0)
        {
            printk("ERR: fds_init: %d\n", err);
            /* try again */
            continue;
        }

        mqtt_connected = true;
        printk("MQTT: connected, client_id=%s\n", client.client_id.utf8);

        while (1)
        {
            err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
            if (err < 0)
            {
                printk("ERR: poll: %d\n", errno);
                break;
            }

            err = mqtt_live(&client);
            if ((err != 0) && (err != -EAGAIN))
            {
                printk("ERR: mqtt_live: %d\n", err);
                break;
            }

            if ((fds.revents & POLLIN) == POLLIN)
            {
                err = mqtt_input(&client);
                if (err != 0)
                {
                    printk("ERR: mqtt_input: %d\n", err);
                    break;
                }
            }

            if ((fds.revents & POLLERR) == POLLERR)
            {
                printk("ERR: POLLERR\n");
                break;
            }

            if ((fds.revents & POLLNVAL) == POLLNVAL)
            {
                printk("ERR: POLLNVAL\n");
                break;
            }
            k_sleep(K_MSEC(1000));
        }
        mqtt_connected = false;

        printk("ERR: Error during connection, reconnecting...\n");

        err = mqtt_disconnect(&client);
        if (err)
        {
            printk("ERR: Could not disconnect MQTT client: %d", err);
        }
    }
    printk("ERR: Ooops! Shouldn't be here!\n");
}

void ei_mqtt_publish(const char *value)
{
    int err;

    err = data_publish(&client,
                       MQTT_QOS_1_AT_LEAST_ONCE,
                       (uint8_t*) value,
                       strlen(value));
    if (err) {
        printk("ERR: Publish failed: %d\n", err);
    }
}

bool ei_get_mqtt_connected(void)
{
    return mqtt_connected;
}

K_THREAD_DEFINE(mqtt_handler_id, STACKSIZE, mqtt_handler, NULL, NULL, NULL,
                PRIORITY, 0, 0);