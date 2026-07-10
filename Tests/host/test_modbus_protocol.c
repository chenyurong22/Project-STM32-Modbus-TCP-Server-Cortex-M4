#include "modbus.h"
#include "modbus_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

static size_t make_request(uint16_t transaction,
                           uint8_t unit,
                           const uint8_t *pdu,
                           size_t pdu_len,
                           uint8_t *adu)
{
    uint16_t length = (uint16_t)(1u + pdu_len);
    adu[0] = (uint8_t)(transaction >> 8);
    adu[1] = (uint8_t)transaction;
    adu[2] = 0u;
    adu[3] = 0u;
    adu[4] = (uint8_t)(length >> 8);
    adu[5] = (uint8_t)length;
    adu[6] = unit;
    memcpy(&adu[7], pdu, pdu_len);
    return 7u + pdu_len;
}

static int transact(const uint8_t *pdu,
                    size_t pdu_len,
                    uint8_t *response,
                    size_t *response_len)
{
    uint8_t request[MODBUS_TCP_ADU_MAX_SIZE];
    size_t request_len = make_request(0x1234u, 1u, pdu, pdu_len, request);
    return mbtcp_process_adu(request,
                             request_len,
                             response,
                             MODBUS_TCP_ADU_MAX_SIZE,
                             response_len);
}

static int test_register_map(void)
{
    mb_init();
    mb_set_hreg(7u, 0xBEEFu);
    mb_set_ireg(3u, 0x1234u);
    mb_set_coil(9u, 1u);
    mb_set_dinput(4u, 1u);

    CHECK(mb_get_hreg(7u) == 0xBEEFu);
    CHECK(mb_get_ireg(3u) == 0x1234u);
    CHECK(mb_get_coil(9u) == 1u);
    CHECK(mb_get_dinput(4u) == 1u);
    CHECK(mb_get_hreg((uint16_t)MB_MAX_HREGS) == 0u);
    CHECK(mb_get_coil((uint16_t)MB_MAX_COILS) == 0u);
    return EXIT_SUCCESS;
}

static int test_write_and_read_holding_registers(void)
{
    const uint8_t write_pdu[] = {0x10u, 0x00u, 0x02u, 0x00u, 0x02u,
                                 0x04u, 0x12u, 0x34u, 0xABu, 0xCDu};
    const uint8_t read_pdu[] = {0x03u, 0x00u, 0x02u, 0x00u, 0x02u};
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
    size_t response_len = 0u;

    CHECK(transact(write_pdu, sizeof(write_pdu), response, &response_len) == 0);
    CHECK(response_len == 12u);
    CHECK(memcmp(&response[7], (const uint8_t[]){0x10u, 0x00u, 0x02u, 0x00u, 0x02u}, 5u) == 0);

    CHECK(transact(read_pdu, sizeof(read_pdu), response, &response_len) == 0);
    CHECK(response_len == 13u);
    CHECK(memcmp(&response[7],
                 (const uint8_t[]){0x03u, 0x04u, 0x12u, 0x34u, 0xABu, 0xCDu},
                 6u) == 0);
    return EXIT_SUCCESS;
}

static int test_coils(void)
{
    const uint8_t write_pdu[] = {0x0Fu, 0x00u, 0x00u, 0x00u, 0x0Au,
                                 0x02u, 0x4Du, 0x03u};
    const uint8_t read_pdu[] = {0x01u, 0x00u, 0x00u, 0x00u, 0x0Au};
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
    size_t response_len = 0u;

    CHECK(transact(write_pdu, sizeof(write_pdu), response, &response_len) == 0);
    CHECK(response_len == 12u);
    CHECK(transact(read_pdu, sizeof(read_pdu), response, &response_len) == 0);
    CHECK(response_len == 11u);
    CHECK(response[7] == 0x01u);
    CHECK(response[8] == 0x02u);
    CHECK(response[9] == 0x4Du);
    CHECK((response[10] & 0x03u) == 0x03u);
    return EXIT_SUCCESS;
}

static int test_exceptions_and_malformed_frames(void)
{
    const uint8_t illegal_function[] = {0x7Fu};
    const uint8_t illegal_address[] = {0x03u, 0x00u, 0xFFu, 0x00u, 0x02u};
    const uint8_t illegal_value[] = {0x05u, 0x00u, 0x00u, 0x12u, 0x34u};
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t request[MODBUS_TCP_ADU_MAX_SIZE];
    size_t response_len = 0u;
    size_t request_len;

    CHECK(transact(illegal_function, sizeof(illegal_function), response, &response_len) == 0);
    CHECK(response[7] == 0xFFu && response[8] == 0x01u);

    CHECK(transact(illegal_address, sizeof(illegal_address), response, &response_len) == 0);
    CHECK(response[7] == 0x83u && response[8] == 0x02u);

    CHECK(transact(illegal_value, sizeof(illegal_value), response, &response_len) == 0);
    CHECK(response[7] == 0x85u && response[8] == 0x03u);

    request_len = make_request(1u, 1u, illegal_function, sizeof(illegal_function), request);
    request[5] = (uint8_t)(request[5] + 1u);
    CHECK(mbtcp_process_adu(request,
                            request_len,
                            response,
                            sizeof(response),
                            &response_len) < 0);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_register_map() == EXIT_SUCCESS);
    CHECK(test_write_and_read_holding_registers() == EXIT_SUCCESS);
    CHECK(test_coils() == EXIT_SUCCESS);
    CHECK(test_exceptions_and_malformed_frames() == EXIT_SUCCESS);
    puts("modbus protocol tests: PASS");
    return EXIT_SUCCESS;
}
