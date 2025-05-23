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
LOG_MODULE_REGISTER(ei_ws_client, CONFIG_REMOTE_INGESTION_LOG_LEVEL);

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "ei_ws_client.h"
#include "remote-mgmt.h"
#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/websocket.h>
#include <climits>
#include <string>
#include <errno.h>

#define REMOTE_MGMT_HOST "remote-mgmt.edgeimpulse.com"
#define INGESTION_HOST "ingestion.edgeimpulse.com"
#define REMOTE_MGMT_PORT "80"
#define INGESTION_PORT "80"

using namespace std;

static int remote_mgmt_socket = -1;
static int ingestion_socket = -1;
static bool is_connected = false;
static struct k_thread ws_read_thread_data;
static EiDeviceInfo *device;
static struct addrinfo *ingestion_addrinfo;
bool (*sample_start_handler)(const char **, const int);
void ws_ping_work_handler(struct k_work *work);
void ws_ping_timer_handler(struct k_timer *dummy);

K_THREAD_STACK_DEFINE(ws_read_stack, 8192);
K_WORK_DEFINE(ws_ping_work, ws_ping_work_handler);
K_TIMER_DEFINE(ws_ping_timer, ws_ping_timer_handler, NULL);

void ws_ping_work_handler(struct k_work *work)
{
    int ret;

    if(remote_mgmt_socket < 0) {
        LOG_WRN("Remote Management service not connected!");
        return;
    }

    LOG_DBG("Ping!");
    ret = websocket_send_msg(remote_mgmt_socket, NULL, 0, WEBSOCKET_OPCODE_PING, true, true, 100);
    if (ret < 0) {
        LOG_ERR("Failed to send ping! (%d)", ret);
    }
}

void ws_ping_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&ws_ping_work);
}

static bool server_connect(void)
{
    static int remote_mgmt_http_socket = -1;
	int32_t timeout = 3 * MSEC_PER_SEC;
	static struct addrinfo hints;
	struct addrinfo *remote_mgmt_addrinfo;
	int ret;
	static struct websocket_request req;
	static uint8_t temp_recv_buf_ipv4[512];
    char peer_addr[INET6_ADDRSTRLEN];

	LOG_DBG("Resolving address: %s", REMOTE_MGMT_HOST);
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	ret = zsock_getaddrinfo(REMOTE_MGMT_HOST, REMOTE_MGMT_PORT, &hints, &remote_mgmt_addrinfo);
	if (ret != 0) {
		LOG_ERR("Unable to resolve address, quitting (%d)", ret);
		return false;
	}
    inet_ntop(remote_mgmt_addrinfo->ai_family, &((struct sockaddr_in *)(remote_mgmt_addrinfo->ai_addr))->sin_addr, peer_addr, INET6_ADDRSTRLEN);
    LOG_DBG("Resolved %s (%s)", peer_addr, net_family2str(remote_mgmt_addrinfo->ai_family));
	LOG_DBG("Resolving address OK");

	LOG_DBG("Resolving address: %s", INGESTION_HOST);
	ret = zsock_getaddrinfo(INGESTION_HOST, INGESTION_PORT, &hints, &ingestion_addrinfo);
	if (ret != 0) {
		LOG_ERR("Unable to resolve address (%d). Samples won't be uploaded to ingestion service!", ret);
	}
    inet_ntop(ingestion_addrinfo->ai_family, &((struct sockaddr_in *)(ingestion_addrinfo->ai_addr))->sin_addr, peer_addr, INET6_ADDRSTRLEN);
    LOG_DBG("Resolved %s (%s)", peer_addr, net_family2str(ingestion_addrinfo->ai_family));
	LOG_DBG("Resolving address OK");

	LOG_DBG("Connecting to remote management server...");
	remote_mgmt_http_socket = zsock_socket(remote_mgmt_addrinfo->ai_family, remote_mgmt_addrinfo->ai_socktype, remote_mgmt_addrinfo->ai_protocol);
    if(remote_mgmt_http_socket < 0) {
        LOG_ERR("Failed to create socket! (%d)", errno);
        return false;
    }

	ret = zsock_connect(remote_mgmt_http_socket, remote_mgmt_addrinfo->ai_addr, remote_mgmt_addrinfo->ai_addrlen);
	if (ret < 0) {
		LOG_ERR("Cannot connect to remote management service! (%d)", errno);
		return false;
	}
	LOG_DBG("Connecting to remote management server... OK");

	LOG_DBG("Establishing websocket connection to remote management server...");
	memset(&req, 0, sizeof(req));
	req.host = REMOTE_MGMT_HOST;
	req.url = "/";
	req.cb = nullptr;
	req.tmp_buf = temp_recv_buf_ipv4;
	req.tmp_buf_len = sizeof(temp_recv_buf_ipv4);

	remote_mgmt_socket = websocket_connect(remote_mgmt_http_socket, &req, timeout, nullptr);
	if (remote_mgmt_socket < 0) {
		LOG_ERR("Cannot connect to %s:%s", REMOTE_MGMT_HOST, REMOTE_MGMT_PORT);
		close(remote_mgmt_http_socket);
		return false;
	}
	LOG_DBG("Establishing websocket connection to remote management server... OK");

	return true;
}

static void parse_received_msg(const uint8_t *buf, size_t buf_len)
{
    auto decoded_message = decode_message(buf, buf_len, device);

    if (decoded_message->getType() == MessageType::DecoderErrorType) {
        auto msg = static_cast<DecoderError*>(decoded_message.get());
        LOG_DBG("Error decoding message (%d): %s", msg->err_code, msg->err_message.c_str());
    } else if (decoded_message->getType() == MessageType::HelloResponseType) {
        auto msg = static_cast<HelloResponse*>(decoded_message.get());
        LOG_DBG("Hello response: %s", msg->status ? "OK" : "ERROR");
        is_connected = msg->status;
    } else if (decoded_message->getType() == MessageType::ErrorResponseType) {
        auto msg = static_cast<ErrorResponse*>(decoded_message.get());
        LOG_DBG("Error response: %s", msg->err_message.c_str());
    } else if (decoded_message->getType() == MessageType::SampleRequestType) {
        auto msg = static_cast<SampleRequest*>(decoded_message.get());
        LOG_DBG("Sample request: %s", msg->sensor.c_str());
        const char* sensor = msg->sensor.c_str();
        if(sample_start_handler) {
            sample_start_handler(&sensor, 1);
        }
    } else if (decoded_message->getType() == MessageType::StreamingStartRequestType) {
        // auto msg = static_cast<StreamingStartRequest*>(decoded_message.get());
        LOG_DBG("Streaming Start request");
        // not supported yet
    } else if (decoded_message->getType() == MessageType::StreamingStopRequestType) {
        // auto msg = static_cast<StreamingStopRequest*>(decoded_message.get());
        LOG_DBG("Streaming Stop request");
        // not supported yet
    }
    else {
        LOG_WRN("Unknown message type!");
    }
}

void ws_read_handler(void *arg1, void *arg2, void *arg3)
{
    uint64_t remaining;
    uint32_t message_type;
    uint8_t buf[512];
    uint32_t total_read;
    int read_pos;
    int ret;

	if(!server_connect()) {
		LOG_ERR("Server connection failed!");
		k_sleep(K_FOREVER);
	}

    // send hello
    ei_ws_send_msg(TxMsgType::HelloMsg);

    while(true) {
        remaining = ULLONG_MAX;
        read_pos = 0;
        total_read = 0;

        while (remaining > 0) {
            ret = websocket_recv_msg(remote_mgmt_socket, buf + read_pos,
                        sizeof(buf) - read_pos,
                        &message_type,
                        &remaining,
                        SYS_FOREVER_MS);
            if (ret < 0) {
                if (ret == -EAGAIN) {
                    k_sleep(K_MSEC(50));
                    continue;
                }

                LOG_DBG("connection closed while waiting (%d/%d)", ret, errno);
                //TODO: reconnect in case of websocket closed
                is_connected = false;
                k_sleep(K_SECONDS(1));
                continue;
            }

            read_pos += ret;
            total_read += ret;
        }

        if (remaining != 0) {
            LOG_ERR("data recv failure (remaining %" PRId64 ")", remaining);
            // TODO: reconnect in case of websocket closed
            continue;
        }

        if(message_type & WEBSOCKET_FLAG_PING) {
            LOG_DBG("PING received, sending PONG");
            websocket_send_msg(remote_mgmt_socket, buf, total_read, WEBSOCKET_OPCODE_PONG, true, true, SYS_FOREVER_MS);
            continue;
        }
        else if((message_type & WEBSOCKET_FLAG_BINARY) && (message_type & WEBSOCKET_FLAG_FINAL)) {
            parse_received_msg(buf, total_read);
            continue;
        }
        else {
            LOG_DBG("recv %d bytes (type: 0x%02x)", total_read, message_type);
            LOG_HEXDUMP_DBG(buf, total_read, "received ws buf");
        }
    }
}

bool ei_ws_send_msg(TxMsgType msg_type, const char* data)
{
    const size_t tx_msg_len = 1024;
    uint8_t tx_msg_buf[tx_msg_len];
    int ret;
    uint32_t act_msg_len = 0;
    string msg_name;

    switch (msg_type) {
        case TxMsgType::HelloMsg:
            act_msg_len = get_hello_msg(tx_msg_buf, tx_msg_len, device);
            msg_name = "Hello";
        break;
        case TxMsgType::SampleStartMsg:
            act_msg_len = get_sample_start_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Start";
        break;
        case TxMsgType::SampleFailedMsg:
            act_msg_len = get_sample_failed_msg(tx_msg_buf, tx_msg_len, data);
            msg_name = "Sample Failed";
        break;
        case TxMsgType::SampleStartedMsg:
            act_msg_len = get_sample_started_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Started";
        break;
        case TxMsgType::SampleProcessingMsg:
            act_msg_len = get_sample_processing_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Processing";
        break;
        case TxMsgType::SampleUploadingMsg:
            act_msg_len = get_sample_uploading_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Uploading";
        break;
        case TxMsgType::SampleFinishedMsg:
            act_msg_len = get_sample_finished_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Finished";
        break;
        case TxMsgType::SnapshotFrameMsg:
            act_msg_len = get_snapshot_frame_msg(tx_msg_buf, tx_msg_len, data);
            msg_name = "Snapshot Frame";
        break;
    }

    ret = websocket_send_msg(remote_mgmt_socket, (const uint8_t*)tx_msg_buf, act_msg_len, WEBSOCKET_OPCODE_DATA_BINARY,
                          true, true, SYS_FOREVER_MS);
    if(ret < 0) {
        LOG_ERR("Failed to send %s message! (%d)", msg_name.c_str(), ret);
        return false;
    }

    LOG_DBG("Message %s len = %d", msg_name.c_str(), act_msg_len);
    LOG_HEXDUMP_DBG(tx_msg_buf, act_msg_len, msg_name.c_str());

    return true;
}

void ei_ws_client_start(EiDeviceInfo *dev, bool (*handler)(const char **, const int))
{
    device = dev;

    sample_start_handler = handler;

    k_timer_start(&ws_ping_timer, K_SECONDS(15), K_SECONDS(15));

    k_thread_create(&ws_read_thread_data, ws_read_stack,
                    K_THREAD_STACK_SIZEOF(ws_read_stack),
                    ws_read_handler,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
}

bool ei_ws_get_connection_status(void)
{
    return is_connected;
}

static void response_cb(struct http_response *rsp, enum http_final_call final_data, void *user_data)
{
    int ret;
	if (final_data == HTTP_DATA_MORE) {
		LOG_DBG("Partial data received (%zd bytes)", rsp->data_len);
	} else if (final_data == HTTP_DATA_FINAL) {
		LOG_DBG("All the data received (%zd bytes)", rsp->data_len);
        ret = zsock_close(ingestion_socket);
        if(ret < 0) {
            LOG_ERR("Failed to close ingestion socket! (%d)", errno);
        }
	}

    LOG_HEXDUMP_DBG(rsp->recv_buf, rsp->recv_buf_len, "rx http buf");
	LOG_DBG("Response status %s", rsp->http_status);
}

bool ei_ws_send_sample(size_t address, size_t length)
{
    int ret;
    char api_key_header[128];
    char label_header[128];
    static uint8_t temp_recv_buf_ipv4[512];
    int32_t timeout = 3 * MSEC_PER_SEC;
    EiDeviceMemory *memory = device->get_memory();
    uint8_t* buffer = (uint8_t*)ei_malloc(length);
    struct http_request req;

	LOG_DBG("Connecting to ingestion service...");
	ingestion_socket = zsock_socket(ingestion_addrinfo->ai_family, ingestion_addrinfo->ai_socktype, ingestion_addrinfo->ai_protocol);
    if(ingestion_socket < 0) {
        LOG_ERR("Failed to create socket! (%d)", errno);
        return false;
    }
	ret = zsock_connect(ingestion_socket, ingestion_addrinfo->ai_addr, ingestion_addrinfo->ai_addrlen);
	if (ret < 0) {
		LOG_ERR("Cannot create HTTP connection.");
		return false;
	}
	LOG_DBG("Connecting to ingestion service... OK");

    snprintf(api_key_header, sizeof(api_key_header), "x-api-key: %s\r\n", device->get_upload_api_key().c_str());
    snprintf(label_header, sizeof(label_header), "x-file-name: %s\r\n", device->get_sample_label().c_str());

    const char *extra_headers[] = {
		api_key_header,
        label_header,
        "x-disallow-duplicates\r\n",
        "Content-type: application/cbor\r\n",
		NULL
	};

    memory->read_sample_data(buffer, address, length);

    memset(&req, 0, sizeof(req));

    req.method = HTTP_POST;
    req.url = "/api/training/data";
    req.host = INGESTION_HOST;
    req.protocol = "HTTP/1.1";
    req.optional_headers = extra_headers;
    req.payload = (const char*)buffer;
    req.payload_len = length;
    req.response = response_cb;
    req.recv_buf = temp_recv_buf_ipv4;
    req.recv_buf_len = sizeof(temp_recv_buf_ipv4);

    ret = http_client_req(ingestion_socket, &req, timeout, nullptr);
    if(ret <0) {
        LOG_ERR("Failed to send sample! (%d)", ret);
        return false;
    }

    return true;
}