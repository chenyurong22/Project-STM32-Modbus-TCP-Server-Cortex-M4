#include "modbus_tcp.h"

#include "modbus_protocol.h"
#include "platform_port.h"

#include "lwip/tcp.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    struct tcp_pcb *pcb;
    uint8_t rx_buffer[MODBUS_TCP_ADU_MAX_SIZE];
    size_t rx_count;
} mb_client_t;

static struct tcp_pcb *listen_pcb;
static mb_client_t clients[MBTCP_MAX_CLIENTS];

static err_t mb_accept(void *arg, struct tcp_pcb *new_pcb, err_t error);
static err_t mb_receive(void *arg, struct tcp_pcb *pcb, struct pbuf *packet, err_t error);
static void mb_connection_error(void *arg, err_t error);
static err_t mb_data_sent(void *arg, struct tcp_pcb *pcb, u16_t length);

static mb_client_t *allocate_client(struct tcp_pcb *pcb)
{
    for (size_t i = 0u; i < MBTCP_MAX_CLIENTS; ++i) {
        if (clients[i].pcb == NULL) {
            clients[i].pcb = pcb;
            clients[i].rx_count = 0u;
            return &clients[i];
        }
    }
    return NULL;
}

static void release_client(mb_client_t *client)
{
    if (client != NULL) {
        client->pcb = NULL;
        client->rx_count = 0u;
    }
}

static int expected_adu_size(const uint8_t *buffer, size_t available, size_t *expected)
{
    uint16_t length_field;

    if (available < MBTCP_MBAP_HEADER_SIZE) {
        return 0;
    }
    length_field = (uint16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
    if (length_field < 2u || (6u + (size_t)length_field) > MODBUS_TCP_ADU_MAX_SIZE) {
        return -1;
    }
    *expected = 6u + (size_t)length_field;
    return 1;
}

static int process_buffered_frames(mb_client_t *client)
{
    for (;;) {
        uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
        size_t expected = 0u;
        size_t response_len = 0u;
        size_t remaining;
        int state = expected_adu_size(client->rx_buffer, client->rx_count, &expected);

        if (state < 0) {
            return -1;
        }
        if (state == 0 || client->rx_count < expected) {
            return 0;
        }
        if (mbtcp_process_adu(client->rx_buffer,
                              expected,
                              response,
                              sizeof(response),
                              &response_len) != 0) {
            return -1;
        }
        if (tcp_write(client->pcb,
                      response,
                      (u16_t)response_len,
                      TCP_WRITE_FLAG_COPY) != ERR_OK) {
            return -1;
        }
        (void)tcp_output(client->pcb);

        remaining = client->rx_count - expected;
        if (remaining > 0u) {
            memmove(client->rx_buffer, &client->rx_buffer[expected], remaining);
        }
        client->rx_count = remaining;
    }
}

static int feed_received_bytes(mb_client_t *client, const uint8_t *data, size_t length)
{
    while (length > 0u) {
        size_t free_space = sizeof(client->rx_buffer) - client->rx_count;
        size_t copy_length;

        if (free_space == 0u) {
            if (process_buffered_frames(client) != 0 ||
                client->rx_count == sizeof(client->rx_buffer)) {
                return -1;
            }
            free_space = sizeof(client->rx_buffer) - client->rx_count;
        }

        copy_length = length < free_space ? length : free_space;
        memcpy(&client->rx_buffer[client->rx_count], data, copy_length);
        client->rx_count += copy_length;
        data += copy_length;
        length -= copy_length;

        if (process_buffered_frames(client) != 0) {
            return -1;
        }
    }
    return 0;
}

void mbtcp_init(void)
{
    err_t bind_result;

    memset(clients, 0, sizeof(clients));
    listen_pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (listen_pcb == NULL) {
        return;
    }

    bind_result = tcp_bind(listen_pcb, IP_ANY_TYPE, (u16_t)MBTCP_SERVER_PORT);
    if (bind_result != ERR_OK) {
        (void)tcp_close(listen_pcb);
        listen_pcb = NULL;
        return;
    }

    {
        struct tcp_pcb *new_listen_pcb =
            tcp_listen_with_backlog(listen_pcb, (u8_t)MBTCP_MAX_CLIENTS);
        if (new_listen_pcb == NULL) {
            (void)tcp_close(listen_pcb);
            listen_pcb = NULL;
            return;
        }
        listen_pcb = new_listen_pcb;
    }
    tcp_accept(listen_pcb, mb_accept);
}

void mbtcp_poll(void)
{
    /* Reserved for application-level periodic work. */
}

static err_t mb_accept(void *arg, struct tcp_pcb *new_pcb, err_t error)
{
    mb_client_t *client;
    (void)arg;

    if (error != ERR_OK || new_pcb == NULL) {
        return error;
    }

    client = allocate_client(new_pcb);
    if (client == NULL) {
        tcp_abort(new_pcb);
        return ERR_ABRT;
    }

    tcp_arg(new_pcb, client);
    tcp_recv(new_pcb, mb_receive);
    tcp_err(new_pcb, mb_connection_error);
    tcp_sent(new_pcb, mb_data_sent);
    return ERR_OK;
}

static void mb_connection_error(void *arg, err_t error)
{
    (void)error;
    release_client((mb_client_t *)arg);
}

static err_t mb_data_sent(void *arg, struct tcp_pcb *pcb, u16_t length)
{
    (void)arg;
    (void)pcb;
    (void)length;
    return ERR_OK;
}

static err_t mb_receive(void *arg, struct tcp_pcb *pcb, struct pbuf *packet, err_t error)
{
    mb_client_t *client = (mb_client_t *)arg;
    u16_t received_length;

    if (client == NULL) {
        if (packet != NULL) {
            pbuf_free(packet);
        }
        return ERR_ARG;
    }

    if (packet == NULL) {
        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_sent(pcb, NULL);
        release_client(client);
        if (tcp_close(pcb) != ERR_OK) {
            tcp_abort(pcb);
            return ERR_ABRT;
        }
        return ERR_OK;
    }

    received_length = packet->tot_len;
    if (error != ERR_OK) {
        pbuf_free(packet);
        return error;
    }

    for (struct pbuf *part = packet; part != NULL; part = part->next) {
        if (feed_received_bytes(client, (const uint8_t *)part->payload, part->len) != 0) {
            pbuf_free(packet);
            tcp_abort(pcb);
            release_client(client);
            return ERR_ABRT;
        }
    }

    tcp_recved(pcb, received_length);
    pbuf_free(packet);
    return ERR_OK;
}
