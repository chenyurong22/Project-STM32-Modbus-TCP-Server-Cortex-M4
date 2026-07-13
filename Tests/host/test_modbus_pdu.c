#include "modbus.h"
#include "modbus_pdu.h"
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

static int process_expect(const uint8_t *request,
                          size_t request_len,
                          const uint8_t *expected,
                          size_t expected_len)
{
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_len = 0u;

    CHECK(mb_process_pdu(request,
                         request_len,
                         response,
                         sizeof(response),
                         &response_len) == 0);
    CHECK(response_len == expected_len);
    CHECK(memcmp(response, expected, expected_len) == 0);
    return EXIT_SUCCESS;
}

static size_t make_tcp_request(const uint8_t *pdu,
                               size_t pdu_len,
                               uint8_t *adu)
{
    uint16_t length = (uint16_t)(1u + pdu_len);

    adu[0] = 0x12u;
    adu[1] = 0x34u;
    adu[2] = 0u;
    adu[3] = 0u;
    adu[4] = (uint8_t)(length >> 8);
    adu[5] = (uint8_t)length;
    adu[6] = 1u;
    memcpy(&adu[7], pdu, pdu_len);
    return MBTCP_MBAP_HEADER_SIZE + pdu_len;
}

static int test_read_bits(void)
{
    const uint8_t read_coils[] = {0x01u, 0x00u, 0x00u, 0x00u, 0x0Au};
    const uint8_t read_discretes[] = {0x02u, 0x00u, 0x00u, 0x00u, 0x08u};
    const uint8_t expected_coils[] = {0x01u, 0x02u, 0x4Du, 0x03u};
    const uint8_t expected_discretes[] = {0x02u, 0x01u, 0xA6u};

    mb_init();
    for (uint16_t i = 0u; i < 10u; ++i) {
        mb_set_coil(i, (uint8_t)((0x34Du >> i) & 1u));
    }
    for (uint16_t i = 0u; i < 8u; ++i) {
        mb_set_dinput(i, (uint8_t)((0xA6u >> i) & 1u));
    }

    CHECK(process_expect(read_coils,
                         sizeof(read_coils),
                         expected_coils,
                         sizeof(expected_coils)) == EXIT_SUCCESS);
    CHECK(process_expect(read_discretes,
                         sizeof(read_discretes),
                         expected_discretes,
                         sizeof(expected_discretes)) == EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static int test_read_registers(void)
{
    const uint8_t read_holding[] = {0x03u, 0x00u, 0x02u, 0x00u, 0x02u};
    const uint8_t read_input[] = {0x04u, 0x00u, 0x04u, 0x00u, 0x02u};
    const uint8_t expected_holding[] = {0x03u, 0x04u, 0x12u, 0x34u, 0xABu, 0xCDu};
    const uint8_t expected_input[] = {0x04u, 0x04u, 0x0Bu, 0xADu, 0xBEu, 0xEFu};

    mb_init();
    mb_set_hreg(2u, 0x1234u);
    mb_set_hreg(3u, 0xABCDu);
    mb_set_ireg(4u, 0x0BADu);
    mb_set_ireg(5u, 0xBEEFu);

    CHECK(process_expect(read_holding,
                         sizeof(read_holding),
                         expected_holding,
                         sizeof(expected_holding)) == EXIT_SUCCESS);
    CHECK(process_expect(read_input,
                         sizeof(read_input),
                         expected_input,
                         sizeof(expected_input)) == EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static int test_single_writes(void)
{
    const uint8_t write_coil_on[] = {0x05u, 0x00u, 0x07u, 0xFFu, 0x00u};
    const uint8_t write_coil_off[] = {0x05u, 0x00u, 0x07u, 0x00u, 0x00u};
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu};

    mb_init();
    CHECK(process_expect(write_coil_on,
                         sizeof(write_coil_on),
                         write_coil_on,
                         sizeof(write_coil_on)) == EXIT_SUCCESS);
    CHECK(mb_get_coil(7u) == 1u);
    CHECK(process_expect(write_coil_off,
                         sizeof(write_coil_off),
                         write_coil_off,
                         sizeof(write_coil_off)) == EXIT_SUCCESS);
    CHECK(mb_get_coil(7u) == 0u);

    CHECK(process_expect(write_register,
                         sizeof(write_register),
                         write_register,
                         sizeof(write_register)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(9u) == 0xCAFEu);
    return EXIT_SUCCESS;
}

static int test_multiple_writes(void)
{
    const uint8_t write_coils[] = {0x0Fu, 0x00u, 0x03u, 0x00u, 0x0Au,
                                   0x02u, 0x4Du, 0x03u};
    const uint8_t expected_coils[] = {0x0Fu, 0x00u, 0x03u, 0x00u, 0x0Au};
    const uint8_t write_registers[] = {0x10u, 0x00u, 0x0Au, 0x00u, 0x03u,
                                       0x06u, 0x11u, 0x11u, 0x22u, 0x22u,
                                       0x33u, 0x33u};
    const uint8_t expected_registers[] = {0x10u, 0x00u, 0x0Au, 0x00u, 0x03u};

    mb_init();
    CHECK(process_expect(write_coils,
                         sizeof(write_coils),
                         expected_coils,
                         sizeof(expected_coils)) == EXIT_SUCCESS);
    for (uint16_t i = 0u; i < 10u; ++i) {
        CHECK(mb_get_coil((uint16_t)(3u + i)) ==
              (uint8_t)((0x34Du >> i) & 1u));
    }

    CHECK(process_expect(write_registers,
                         sizeof(write_registers),
                         expected_registers,
                         sizeof(expected_registers)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(10u) == 0x1111u);
    CHECK(mb_get_hreg(11u) == 0x2222u);
    CHECK(mb_get_hreg(12u) == 0x3333u);
    return EXIT_SUCCESS;
}


static int test_write_capacity_is_transactional(void)
{
    const uint8_t write_coil[] = {0x05u, 0x00u, 0x07u, 0xFFu, 0x00u};
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu};
    const uint8_t write_coils[] = {0x0Fu, 0x00u, 0x03u, 0x00u, 0x0Au,
                                   0x02u, 0x4Du, 0x03u};
    const uint8_t write_registers[] = {0x10u, 0x00u, 0x0Au, 0x00u, 0x03u,
                                       0x06u, 0x11u, 0x11u, 0x22u, 0x22u,
                                       0x33u, 0x33u};
    const uint8_t *requests[] = {
        write_coil,
        write_register,
        write_coils,
        write_registers
    };
    const size_t request_lengths[] = {
        sizeof(write_coil),
        sizeof(write_register),
        sizeof(write_coils),
        sizeof(write_registers)
    };
    const uint8_t expected_functions[] = {0x85u, 0x86u, 0x8Fu, 0x90u};
    uint8_t response[2];

    mb_init();
    mb_set_hreg(9u, 0x1111u);
    mb_set_hreg(10u, 0xAAAAu);
    mb_set_hreg(11u, 0xBBBBu);
    mb_set_hreg(12u, 0xCCCCu);

    for (size_t request_index = 0u;
         request_index < sizeof(requests) / sizeof(requests[0]);
         ++request_index) {
        size_t response_len = 0u;

        CHECK(mb_process_pdu(requests[request_index],
                             request_lengths[request_index],
                             response,
                             sizeof(response),
                             &response_len) == 0);
        CHECK(response_len == 2u);
        CHECK(response[0] == expected_functions[request_index]);
        CHECK(response[1] == 0x04u);
    }

    CHECK(mb_get_coil(7u) == 0u);
    for (uint16_t i = 0u; i < 10u; ++i) {
        CHECK(mb_get_coil((uint16_t)(3u + i)) == 0u);
    }
    CHECK(mb_get_hreg(9u) == 0x1111u);
    CHECK(mb_get_hreg(10u) == 0xAAAAu);
    CHECK(mb_get_hreg(11u) == 0xBBBBu);
    CHECK(mb_get_hreg(12u) == 0xCCCCu);
    return EXIT_SUCCESS;
}

static int test_exception_pdus(void)
{
    const uint8_t illegal_function[] = {0x7Fu};
    const uint8_t illegal_address[] = {0x03u, 0x00u, 0xFFu, 0x00u, 0x02u};
    const uint8_t illegal_value[] = {0x05u, 0x00u, 0x00u, 0x12u, 0x34u};
    const uint8_t malformed_multiple[] = {0x10u, 0x00u, 0x00u, 0x00u, 0x02u,
                                          0x02u, 0x12u, 0x34u};
    const uint8_t expected_function[] = {0xFFu, 0x01u};
    const uint8_t expected_address[] = {0x83u, 0x02u};
    const uint8_t expected_value[] = {0x85u, 0x03u};
    const uint8_t expected_malformed[] = {0x90u, 0x03u};
    const uint8_t read_holding[] = {0x03u, 0x00u, 0x00u, 0x00u, 0x01u};
    uint8_t response[2];
    size_t response_len = 0u;

    mb_init();
    CHECK(process_expect(illegal_function,
                         sizeof(illegal_function),
                         expected_function,
                         sizeof(expected_function)) == EXIT_SUCCESS);
    CHECK(process_expect(illegal_address,
                         sizeof(illegal_address),
                         expected_address,
                         sizeof(expected_address)) == EXIT_SUCCESS);
    CHECK(process_expect(illegal_value,
                         sizeof(illegal_value),
                         expected_value,
                         sizeof(expected_value)) == EXIT_SUCCESS);
    CHECK(process_expect(malformed_multiple,
                         sizeof(malformed_multiple),
                         expected_malformed,
                         sizeof(expected_malformed)) == EXIT_SUCCESS);

    CHECK(mb_process_pdu(read_holding,
                         sizeof(read_holding),
                         response,
                         sizeof(response),
                         &response_len) == 0);
    CHECK(response_len == 2u);
    CHECK(response[0] == 0x83u && response[1] == 0x04u);
    return EXIT_SUCCESS;
}

static int test_api_validation(void)
{
    const uint8_t request[] = {0x03u, 0x00u, 0x00u, 0x00u, 0x01u};
    uint8_t oversized[MODBUS_PDU_MAX_SIZE + 1u] = {0u};
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_len = 99u;

    oversized[0] = 0x03u;
    CHECK(mb_process_pdu(NULL,
                         sizeof(request),
                         response,
                         sizeof(response),
                         &response_len) < 0);
    CHECK(mb_process_pdu(request,
                         sizeof(request),
                         NULL,
                         sizeof(response),
                         &response_len) < 0);
    CHECK(mb_process_pdu(request,
                         sizeof(request),
                         response,
                         sizeof(response),
                         NULL) < 0);
    CHECK(mb_process_pdu(request, 0u, response, sizeof(response), &response_len) < 0);
    CHECK(response_len == 0u);
    response_len = 99u;
    CHECK(mb_process_pdu(oversized,
                         sizeof(oversized),
                         response,
                         sizeof(response),
                         &response_len) < 0);
    CHECK(response_len == 0u);
    response_len = 99u;
    CHECK(mb_process_pdu(request, sizeof(request), response, 1u, &response_len) < 0);
    CHECK(response_len == 0u);
    return EXIT_SUCCESS;
}

static int test_tcp_wrapper_uses_same_pdu_result(void)
{
    const uint8_t requests[][5] = {
        {0x03u, 0x00u, 0x02u, 0x00u, 0x02u},
        {0x03u, 0x00u, 0xFFu, 0x00u, 0x02u}
    };
    const size_t request_lengths[] = {5u, 5u};
    uint8_t direct_response[MODBUS_PDU_MAX_SIZE];
    uint8_t tcp_request[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t tcp_response[MODBUS_TCP_ADU_MAX_SIZE];

    mb_init();
    mb_set_hreg(2u, 0x1234u);
    mb_set_hreg(3u, 0xABCDu);

    for (size_t i = 0u; i < 2u; ++i) {
        size_t direct_len = 0u;
        size_t tcp_request_len;
        size_t tcp_response_len = 0u;

        CHECK(mb_process_pdu(requests[i],
                             request_lengths[i],
                             direct_response,
                             sizeof(direct_response),
                             &direct_len) == 0);
        tcp_request_len = make_tcp_request(requests[i], request_lengths[i], tcp_request);
        CHECK(mbtcp_process_adu(tcp_request,
                                tcp_request_len,
                                tcp_response,
                                sizeof(tcp_response),
                                &tcp_response_len) == 0);
        CHECK(tcp_response_len == MBTCP_MBAP_HEADER_SIZE + direct_len);
        CHECK(memcmp(&tcp_response[MBTCP_MBAP_HEADER_SIZE],
                     direct_response,
                     direct_len) == 0);
    }
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_read_bits() == EXIT_SUCCESS);
    CHECK(test_read_registers() == EXIT_SUCCESS);
    CHECK(test_single_writes() == EXIT_SUCCESS);
    CHECK(test_multiple_writes() == EXIT_SUCCESS);
    CHECK(test_write_capacity_is_transactional() == EXIT_SUCCESS);
    CHECK(test_exception_pdus() == EXIT_SUCCESS);
    CHECK(test_api_validation() == EXIT_SUCCESS);
    CHECK(test_tcp_wrapper_uses_same_pdu_result() == EXIT_SUCCESS);
    puts("modbus PDU tests: PASS");
    return EXIT_SUCCESS;
}
