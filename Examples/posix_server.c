#define _POSIX_C_SOURCE 200809L

#include "modbus.h"
#include "modbus_protocol.h"

#include <arpa/inet.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

static volatile sig_atomic_t keep_running = 1;

static void stop_server(int signal_number)
{
    (void)signal_number;
    keep_running = 0;
}

static int send_all(int socket_fd, const uint8_t *data, size_t length)
{
    while (length > 0u) {
        ssize_t sent = send(socket_fd, data, length, 0);
        if (sent < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        data += (size_t)sent;
        length -= (size_t)sent;
    }
    return 0;
}

static int process_client(int client_fd)
{
    uint8_t buffer[MODBUS_TCP_ADU_MAX_SIZE];
    size_t buffered = 0u;

    while (keep_running) {
        ssize_t received = recv(client_fd,
                                &buffer[buffered],
                                sizeof(buffer) - buffered,
                                0);
        if (received == 0) {
            return 0;
        }
        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        buffered += (size_t)received;

        for (;;) {
            uint16_t length_field;
            size_t expected;
            uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
            size_t response_len = 0u;

            if (buffered < MBTCP_MBAP_HEADER_SIZE) {
                break;
            }
            length_field = (uint16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
            expected = 6u + (size_t)length_field;
            if (length_field < 2u || expected > sizeof(buffer)) {
                return -1;
            }
            if (buffered < expected) {
                break;
            }
            if (mbtcp_process_adu(buffer,
                                  expected,
                                  response,
                                  sizeof(response),
                                  &response_len) != 0 ||
                send_all(client_fd, response, response_len) != 0) {
                return -1;
            }

            buffered -= expected;
            if (buffered > 0u) {
                memmove(buffer, &buffer[expected], buffered);
            }
        }

        if (buffered == sizeof(buffer)) {
            return -1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    long requested_port = argc > 1 ? strtol(argv[1], NULL, 10) : 1502L;
    int server_fd;
    int reuse = 1;
    struct sockaddr_in address;

    if (requested_port < 1L || requested_port > 65535L) {
        fprintf(stderr, "invalid port: %ld\n", requested_port);
        return EXIT_FAILURE;
    }

    signal(SIGINT, stop_server);
    signal(SIGTERM, stop_server);
    mb_init();

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        return EXIT_FAILURE;
    }
    (void)setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons((uint16_t)requested_port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        listen(server_fd, 4) < 0) {
        perror("bind/listen");
        close(server_fd);
        return EXIT_FAILURE;
    }

    printf("Modbus TCP demo listening on 127.0.0.1:%ld\n", requested_port);
    fflush(stdout);

    while (keep_running) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("accept");
            break;
        }
        (void)process_client(client_fd);
        close(client_fd);
    }

    close(server_fd);
    return EXIT_SUCCESS;
}
