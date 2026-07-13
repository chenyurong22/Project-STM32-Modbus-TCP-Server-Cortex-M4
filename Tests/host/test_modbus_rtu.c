#include "modbus.h"
#include "modbus_crc16.h"
#include "modbus_rtu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

static size_t make_request(uint8_t address,
                           const uint8_t *pdu,
                           size_t pdu_len,
                           uint8_t *adu)
{
    uint16_t crc;
    size_t without_crc_len = 1u + pdu_len;

    adu[0] = address;
    memcpy(&adu[1], pdu, pdu_len);
    crc = mb_crc16(adu, without_crc_len);
    adu[without_crc_len] = (uint8_t)crc;
    adu[without_crc_len + 1u] = (uint8_t)(crc >> 8u);
    return without_crc_len + MODBUS_RTU_CRC_SIZE;
}

static int response_crc_is_valid(const uint8_t *response, size_t response_len)
{
    return response_len >= MODBUS_RTU_ADU_MIN_SIZE &&
           mb_crc16(response, response_len) == 0u;
}

static int process_expect(uint8_t slave_address,
                          const uint8_t *request_pdu,
                          size_t request_pdu_len,
                          const uint8_t *expected_pdu,
                          size_t expected_pdu_len)
{
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_len;
    size_t response_len = 0u;

    request_len = make_request(slave_address,
                               request_pdu,
                               request_pdu_len,
                               request);
    CHECK(mbrtu_process_adu(slave_address,
                            request,
                            request_len,
                            response,
                            sizeof(response),
                            &response_len) == MBRTU_RESPONSE_READY);
    CHECK(response_len == 1u + expected_pdu_len + MODBUS_RTU_CRC_SIZE);
    CHECK(response[0] == slave_address);
    CHECK(memcmp(&response[1], expected_pdu, expected_pdu_len) == 0);
    CHECK(response_crc_is_valid(response, response_len));
    return EXIT_SUCCESS;
}

static int process_expect_no_response(uint8_t slave_address,
                                      uint8_t request_address,
                                      const uint8_t *request_pdu,
                                      size_t request_pdu_len)
{
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE] = {0u};
    size_t request_len;
    size_t response_len = 99u;

    request_len = make_request(request_address,
                               request_pdu,
                               request_pdu_len,
                               request);
    CHECK(mbrtu_process_adu(slave_address,
                            request,
                            request_len,
                            response,
                            sizeof(response),
                            &response_len) == MBRTU_NO_RESPONSE);
    CHECK(response_len == 0u);
    return EXIT_SUCCESS;
}

static int test_read_functions(void)
{
    const uint8_t read_coils[] = {0x01u, 0x00u, 0x00u, 0x00u, 0x0Au};
    const uint8_t read_discretes[] = {0x02u, 0x00u, 0x00u, 0x00u, 0x08u};
    const uint8_t read_holding[] = {0x03u, 0x00u, 0x02u, 0x00u, 0x02u};
    const uint8_t read_input[] = {0x04u, 0x00u, 0x04u, 0x00u, 0x02u};
    const uint8_t expected_coils[] = {0x01u, 0x02u, 0x4Du, 0x03u};
    const uint8_t expected_discretes[] = {0x02u, 0x01u, 0xA6u};
    const uint8_t expected_holding[] = {0x03u, 0x04u, 0x12u, 0x34u, 0xABu, 0xCDu};
    const uint8_t expected_input[] = {0x04u, 0x04u, 0x0Bu, 0xADu, 0xBEu, 0xEFu};

    mb_init();
    for (uint16_t i = 0u; i < 10u; ++i) {
        mb_set_coil(i, (uint8_t)((0x34Du >> i) & 1u));
    }
    for (uint16_t i = 0u; i < 8u; ++i) {
        mb_set_dinput(i, (uint8_t)((0xA6u >> i) & 1u));
    }
    mb_set_hreg(2u, 0x1234u);
    mb_set_hreg(3u, 0xABCDu);
    mb_set_ireg(4u, 0x0BADu);
    mb_set_ireg(5u, 0xBEEFu);

    CHECK(process_expect(1u, read_coils, sizeof(read_coils),
                         expected_coils, sizeof(expected_coils)) == EXIT_SUCCESS);
    CHECK(process_expect(1u, read_discretes, sizeof(read_discretes),
                         expected_discretes, sizeof(expected_discretes)) == EXIT_SUCCESS);
    CHECK(process_expect(1u, read_holding, sizeof(read_holding),
                         expected_holding, sizeof(expected_holding)) == EXIT_SUCCESS);
    CHECK(process_expect(1u, read_input, sizeof(read_input),
                         expected_input, sizeof(expected_input)) == EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static int test_write_functions(void)
{
    const uint8_t write_coil_on[] = {0x05u, 0x00u, 0x07u, 0xFFu, 0x00u};
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu};
    const uint8_t write_coils[] = {0x0Fu, 0x00u, 0x03u, 0x00u, 0x0Au,
                                   0x02u, 0x4Du, 0x03u};
    const uint8_t expected_coils[] = {0x0Fu, 0x00u, 0x03u, 0x00u, 0x0Au};
    const uint8_t write_registers[] = {0x10u, 0x00u, 0x0Au, 0x00u, 0x03u,
                                       0x06u, 0x11u, 0x11u, 0x22u, 0x22u,
                                       0x33u, 0x33u};
    const uint8_t expected_registers[] = {0x10u, 0x00u, 0x0Au, 0x00u, 0x03u};

    mb_init();
    CHECK(process_expect(1u, write_coil_on, sizeof(write_coil_on),
                         write_coil_on, sizeof(write_coil_on)) == EXIT_SUCCESS);
    CHECK(mb_get_coil(7u) == 1u);

    CHECK(process_expect(1u, write_register, sizeof(write_register),
                         write_register, sizeof(write_register)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(9u) == 0xCAFEu);

    CHECK(process_expect(1u, write_coils, sizeof(write_coils),
                         expected_coils, sizeof(expected_coils)) == EXIT_SUCCESS);
    for (uint16_t i = 0u; i < 10u; ++i) {
        CHECK(mb_get_coil((uint16_t)(3u + i)) ==
              (uint8_t)((0x34Du >> i) & 1u));
    }

    CHECK(process_expect(1u, write_registers, sizeof(write_registers),
                         expected_registers, sizeof(expected_registers)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(10u) == 0x1111u);
    CHECK(mb_get_hreg(11u) == 0x2222u);
    CHECK(mb_get_hreg(12u) == 0x3333u);
    return EXIT_SUCCESS;
}

static int test_exception_responses(void)
{
    const uint8_t illegal_function[] = {0x7Fu};
    const uint8_t illegal_address[] = {0x03u, 0x00u, 0xFFu, 0x00u, 0x02u};
    const uint8_t illegal_value[] = {0x05u, 0x00u, 0x00u, 0x12u, 0x34u};
    const uint8_t extra_byte[] = {0x06u, 0x00u, 0x08u, 0x12u, 0x34u, 0x00u};
    const uint8_t expected_function[] = {0xFFu, 0x01u};
    const uint8_t expected_address[] = {0x83u, 0x02u};
    const uint8_t expected_value[] = {0x85u, 0x03u};
    const uint8_t expected_extra[] = {0x86u, 0x03u};

    mb_init();
    mb_set_hreg(8u, 0xBEEFu);
    CHECK(process_expect(1u, illegal_function, sizeof(illegal_function),
                         expected_function, sizeof(expected_function)) == EXIT_SUCCESS);
    CHECK(process_expect(1u, illegal_address, sizeof(illegal_address),
                         expected_address, sizeof(expected_address)) == EXIT_SUCCESS);
    CHECK(process_expect(1u, illegal_value, sizeof(illegal_value),
                         expected_value, sizeof(expected_value)) == EXIT_SUCCESS);
    CHECK(process_expect(1u, extra_byte, sizeof(extra_byte),
                         expected_extra, sizeof(expected_extra)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(8u) == 0xBEEFu);
    return EXIT_SUCCESS;
}

static int test_address_crc_and_broadcast_rules(void)
{
    const uint8_t write_coil[] = {0x05u, 0x00u, 0x07u, 0xFFu, 0x00u};
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu};
    const uint8_t write_coils[] = {0x0Fu, 0x00u, 0x03u, 0x00u, 0x0Au,
                                   0x02u, 0x4Du, 0x03u};
    const uint8_t write_registers[] = {0x10u, 0x00u, 0x0Au, 0x00u, 0x03u,
                                       0x06u, 0x11u, 0x11u, 0x22u, 0x22u,
                                       0x33u, 0x33u};
    const uint8_t read_holding[] = {0x03u, 0x00u, 0x09u, 0x00u, 0x01u};
    const uint8_t unsupported[] = {0x7Fu};
    const uint8_t invalid_broadcast_write[] = {0x05u, 0x00u, 0x08u, 0x12u, 0x34u};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE] = {0u};
    size_t request_len;
    size_t response_len;

    mb_init();
    mb_set_hreg(9u, 0x1111u);

    CHECK(process_expect_no_response(1u, 2u, write_register,
                                     sizeof(write_register)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(9u) == 0x1111u);

    request_len = make_request(1u, write_register, sizeof(write_register), request);
    request[request_len - 1u] ^= 0x01u;
    response_len = 99u;
    CHECK(mbrtu_process_adu(1u, request, request_len, response,
                            sizeof(response), &response_len) == MBRTU_NO_RESPONSE);
    CHECK(response_len == 0u);
    CHECK(mb_get_hreg(9u) == 0x1111u);

    CHECK(process_expect_no_response(1u, 0u, read_holding,
                                     sizeof(read_holding)) == EXIT_SUCCESS);
    CHECK(process_expect_no_response(1u, 0u, unsupported,
                                     sizeof(unsupported)) == EXIT_SUCCESS);
    CHECK(process_expect_no_response(1u, 0u, invalid_broadcast_write,
                                     sizeof(invalid_broadcast_write)) == EXIT_SUCCESS);
    CHECK(mb_get_coil(8u) == 0u);

    CHECK(process_expect_no_response(1u, 0u, write_coil,
                                     sizeof(write_coil)) == EXIT_SUCCESS);
    CHECK(mb_get_coil(7u) == 1u);
    CHECK(process_expect_no_response(1u, 0u, write_register,
                                     sizeof(write_register)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(9u) == 0xCAFEu);
    CHECK(process_expect_no_response(1u, 0u, write_coils,
                                     sizeof(write_coils)) == EXIT_SUCCESS);
    for (uint16_t i = 0u; i < 10u; ++i) {
        CHECK(mb_get_coil((uint16_t)(3u + i)) ==
              (uint8_t)((0x34Du >> i) & 1u));
    }
    CHECK(process_expect_no_response(1u, 0u, write_registers,
                                     sizeof(write_registers)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(10u) == 0x1111u);
    CHECK(mb_get_hreg(11u) == 0x2222u);
    CHECK(mb_get_hreg(12u) == 0x3333u);
    return EXIT_SUCCESS;
}

static int test_maximum_read_response(void)
{
    const uint8_t read_holding[] = {0x03u, 0x00u, 0x00u, 0x00u, 0x7Du};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_len;
    size_t response_len = 0u;

    mb_init();
    for (uint16_t i = 0u; i < 125u; ++i) {
        mb_set_hreg(i, (uint16_t)(0x1000u + i));
    }

    request_len = make_request(1u, read_holding, sizeof(read_holding), request);
    CHECK(mbrtu_process_adu(1u, request, request_len, response,
                            254u, &response_len) == MBRTU_RESPONSE_READY);
    CHECK(response_len == 5u);
    CHECK(response[0] == 1u);
    CHECK(response[1] == 0x83u && response[2] == 0x04u);
    CHECK(response_crc_is_valid(response, response_len));

    CHECK(mbrtu_process_adu(1u, request, request_len, response,
                            255u, &response_len) == MBRTU_RESPONSE_READY);
    CHECK(response_len == 255u);
    CHECK(response[0] == 1u);
    CHECK(response[1] == 0x03u);
    CHECK(response[2] == 250u);
    CHECK(response[3] == 0x10u && response[4] == 0x00u);
    CHECK(response[251] == 0x10u && response[252] == 0x7Cu);
    CHECK(response_crc_is_valid(response, response_len));
    return EXIT_SUCCESS;
}

static int test_frame_boundaries_and_api_validation(void)
{
    const uint8_t unsupported[] = {0x7Fu};
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu};
    const uint8_t illegal_address[] = {0x03u, 0x00u, 0xFFu, 0x00u, 0x02u};
    const uint8_t expected_function[] = {0xFFu, 0x01u};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE + 1u] = {0u};
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE] = {0u};
    uint8_t maximum_pdu[MODBUS_PDU_MAX_SIZE] = {0u};
    size_t request_len;
    size_t response_len = 99u;

    maximum_pdu[0] = 0x7Fu;
    CHECK(process_expect(1u, unsupported, sizeof(unsupported),
                         expected_function, sizeof(expected_function)) == EXIT_SUCCESS);
    CHECK(process_expect(247u, unsupported, sizeof(unsupported),
                         expected_function, sizeof(expected_function)) == EXIT_SUCCESS);
    CHECK(process_expect(1u, maximum_pdu, sizeof(maximum_pdu),
                         expected_function, sizeof(expected_function)) == EXIT_SUCCESS);

    CHECK(mbrtu_process_adu(1u, request, 3u, response, sizeof(response),
                            &response_len) == MBRTU_NO_RESPONSE);
    CHECK(response_len == 0u);
    response_len = 99u;
    CHECK(mbrtu_process_adu(1u, request, sizeof(request), response,
                            sizeof(response), &response_len) == MBRTU_NO_RESPONSE);
    CHECK(response_len == 0u);

    request_len = make_request(248u, unsupported, sizeof(unsupported), request);
    response_len = 99u;
    CHECK(mbrtu_process_adu(1u, request, request_len, response,
                            sizeof(response), &response_len) == MBRTU_NO_RESPONSE);
    CHECK(response_len == 0u);

    request_len = make_request(1u, illegal_address, sizeof(illegal_address), request);
    response_len = 99u;
    CHECK(mbrtu_process_adu(1u, request, request_len, response, 5u,
                            &response_len) == MBRTU_RESPONSE_READY);
    CHECK(response_len == 5u);
    CHECK(response[0] == 1u);
    CHECK(response[1] == 0x83u && response[2] == 0x02u);
    CHECK(response_crc_is_valid(response, response_len));

    request_len = make_request(1u, write_register, sizeof(write_register), request);
    mb_init();
    CHECK(mbrtu_process_adu(1u, request, request_len, response, 4u,
                            &response_len) == MBRTU_ERROR_RESPONSE_CAPACITY);
    CHECK(response_len == 0u);
    CHECK(mb_get_hreg(9u) == 0u);

    CHECK(mbrtu_process_adu(0u, request, request_len, response,
                            sizeof(response), &response_len) == MBRTU_ERROR_SLAVE_ADDRESS);
    CHECK(mbrtu_process_adu(248u, request, request_len, response,
                            sizeof(response), &response_len) == MBRTU_ERROR_SLAVE_ADDRESS);
    response_len = 99u;
    CHECK(mbrtu_process_adu(1u, NULL, request_len, response,
                            sizeof(response), &response_len) == MBRTU_ERROR_ARGUMENT);
    CHECK(response_len == 0u);
    response_len = 99u;
    CHECK(mbrtu_process_adu(1u, request, request_len, NULL,
                            sizeof(response), &response_len) == MBRTU_ERROR_ARGUMENT);
    CHECK(response_len == 0u);
    CHECK(mbrtu_process_adu(1u, request, request_len, response,
                            sizeof(response), NULL) == MBRTU_ERROR_ARGUMENT);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_read_functions() == EXIT_SUCCESS);
    CHECK(test_write_functions() == EXIT_SUCCESS);
    CHECK(test_exception_responses() == EXIT_SUCCESS);
    CHECK(test_address_crc_and_broadcast_rules() == EXIT_SUCCESS);
    CHECK(test_maximum_read_response() == EXIT_SUCCESS);
    CHECK(test_frame_boundaries_and_api_validation() == EXIT_SUCCESS);
    puts("modbus RTU ADU tests: PASS");
    return EXIT_SUCCESS;
}
