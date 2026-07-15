#include "modbus_crc16.h"
#include "modbus_rtu_master.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

static size_t append_crc(uint8_t *adu, size_t length_without_crc)
{
    uint16_t crc = mb_crc16(adu, length_without_crc);
    adu[length_without_crc] = (uint8_t)crc;
    adu[length_without_crc + 1u] = (uint8_t)(crc >> 8u);
    return length_without_crc + MODBUS_RTU_CRC_SIZE;
}

static int crc_is_valid(const uint8_t *adu, size_t adu_length)
{
    return mb_crc16(adu, adu_length) == 0u;
}

static int test_read_request_builders(void)
{
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 99u;

    CHECK(mbrtum_build_read_bits_request(1u, MBRTUM_FC_READ_COILS,
                                         0x0013u, 0x0025u, &request,
                                         adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 8u);
    CHECK(memcmp(adu, (const uint8_t[]){0x01u, 0x01u, 0x00u, 0x13u, 0x00u, 0x25u}, 6u) == 0);
    CHECK(crc_is_valid(adu, adu_length));
    CHECK(request.slave_address == 1u);
    CHECK(request.function == MBRTUM_FC_READ_COILS);
    CHECK(request.start_address == 0x0013u);
    CHECK(request.quantity == 0x0025u);
    CHECK(request.value == 0u);
    CHECK(request.expects_response == 1u);

    CHECK(mbrtum_build_read_bits_request(247u, MBRTUM_FC_READ_DISCRETE_INPUTS,
                                         0u, 2000u, &request,
                                         adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu[0] == 247u && adu[1] == MBRTUM_FC_READ_DISCRETE_INPUTS);
    CHECK(crc_is_valid(adu, adu_length));

    CHECK(mbrtum_build_read_registers_request(17u, MBRTUM_FC_READ_HOLDING_REGISTERS,
                                              0x006Bu, 3u, &request,
                                              adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 8u);
    CHECK(memcmp(adu, (const uint8_t[]){0x11u, 0x03u, 0x00u, 0x6Bu, 0x00u, 0x03u}, 6u) == 0);
    CHECK(crc_is_valid(adu, adu_length));

    CHECK(mbrtum_build_read_registers_request(1u, MBRTUM_FC_READ_INPUT_REGISTERS,
                                              0u, 125u, &request,
                                              adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu[1] == MBRTUM_FC_READ_INPUT_REGISTERS);
    return EXIT_SUCCESS;
}

static int test_single_write_request_builders(void)
{
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    CHECK(mbrtum_build_write_single_coil_request(1u, 7u, 1u, &request,
                                                 adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 8u);
    CHECK(memcmp(adu, (const uint8_t[]){0x01u, 0x05u, 0x00u, 0x07u, 0xFFu, 0x00u}, 6u) == 0);
    CHECK(request.quantity == 1u && request.value == 0xFF00u);
    CHECK(request.expects_response == 1u);
    CHECK(crc_is_valid(adu, adu_length));

    CHECK(mbrtum_build_write_single_coil_request(0u, 8u, 0u, &request,
                                                 adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu[0] == 0u && adu[4] == 0u && adu[5] == 0u);
    CHECK(request.expects_response == 0u);

    CHECK(mbrtum_build_write_single_register_request(1u, 9u, 0xCAFEu, &request,
                                                     adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(memcmp(adu, (const uint8_t[]){0x01u, 0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu}, 6u) == 0);
    CHECK(request.value == 0xCAFEu && request.quantity == 1u);
    CHECK(crc_is_valid(adu, adu_length));
    return EXIT_SUCCESS;
}

static int test_multiple_write_request_builders(void)
{
    const uint8_t packed_coils[] = {0x4Du, 0xFFu};
    const uint16_t registers[] = {0x1111u, 0x2222u, 0x3333u};
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    CHECK(mbrtum_build_write_multiple_coils_request(1u, 3u, 10u,
                                                    packed_coils, &request,
                                                    adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 11u);
    CHECK(memcmp(adu, (const uint8_t[]){0x01u, 0x0Fu, 0x00u, 0x03u,
                                                0x00u, 0x0Au, 0x02u, 0x4Du, 0x03u}, 9u) == 0);
    CHECK(request.quantity == 10u && request.value == 0u);
    CHECK(crc_is_valid(adu, adu_length));

    CHECK(mbrtum_build_write_multiple_registers_request(1u, 10u, 3u,
                                                        registers, &request,
                                                        adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 15u);
    CHECK(memcmp(adu, (const uint8_t[]){0x01u, 0x10u, 0x00u, 0x0Au,
                                                0x00u, 0x03u, 0x06u,
                                                0x11u, 0x11u, 0x22u, 0x22u,
                                                0x33u, 0x33u}, 13u) == 0);
    CHECK(crc_is_valid(adu, adu_length));
    return EXIT_SUCCESS;
}

static int test_read_response_decoding(void)
{
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;
    uint8_t bit = 0u;
    uint16_t value = 0u;

    CHECK(mbrtum_build_read_bits_request(1u, MBRTUM_FC_READ_COILS,
                                         0u, 10u, &request, request_adu,
                                         sizeof(request_adu), &request_length) == MBRTUM_OK);
    memcpy(response_adu, (const uint8_t[]){0x01u, 0x01u, 0x02u, 0x4Du, 0x03u}, 5u);
    response_length = append_crc(response_adu, 5u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.function == MBRTUM_FC_READ_COILS);
    CHECK(response.exception_code == 0u);
    CHECK(response.data_length == 2u);
    CHECK(mbrtum_get_bit(&request, &response, 0u, &bit) == MBRTUM_OK && bit == 1u);
    CHECK(mbrtum_get_bit(&request, &response, 1u, &bit) == MBRTUM_OK && bit == 0u);
    CHECK(mbrtum_get_bit(&request, &response, 9u, &bit) == MBRTUM_OK && bit == 1u);
    CHECK(mbrtum_get_bit(&request, &response, 10u, &bit) == MBRTUM_ERROR_INDEX);

    memcpy(response_adu,
           (const uint8_t[]){0x01u, 0x01u, 0x02u, 0x4Du, 0x83u},
           5u);
    response_length = append_crc(response_adu, 5u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_MALFORMED_RESPONSE);

    CHECK(mbrtum_build_read_registers_request(17u, MBRTUM_FC_READ_HOLDING_REGISTERS,
                                              0x006Bu, 3u, &request, request_adu,
                                              sizeof(request_adu), &request_length) == MBRTUM_OK);
    memcpy(response_adu, (const uint8_t[]){0x11u, 0x03u, 0x06u,
                                                   0x02u, 0x2Bu,
                                                   0x00u, 0x00u,
                                                   0x00u, 0x64u}, 9u);
    response_length = append_crc(response_adu, 9u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.data_length == 6u);
    CHECK(mbrtum_get_register(&request, &response, 0u, &value) == MBRTUM_OK && value == 0x022Bu);
    CHECK(mbrtum_get_register(&request, &response, 1u, &value) == MBRTUM_OK && value == 0x0000u);
    CHECK(mbrtum_get_register(&request, &response, 2u, &value) == MBRTUM_OK && value == 0x0064u);
    CHECK(mbrtum_get_register(&request, &response, 3u, &value) == MBRTUM_ERROR_INDEX);
    return EXIT_SUCCESS;
}

static int test_write_acknowledgements(void)
{
    const uint8_t coils[] = {0x4Du, 0x03u};
    const uint16_t registers[] = {0x1111u, 0x2222u, 0x3333u};
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(mbrtum_build_write_single_coil_request(1u, 7u, 1u, &request,
                                                 request_adu, sizeof(request_adu),
                                                 &request_length) == MBRTUM_OK);
    CHECK(mbrtum_process_response(&request, request_adu, request_length,
                                  &response) == MBRTUM_OK);

    CHECK(mbrtum_build_write_single_register_request(1u, 9u, 0xCAFEu, &request,
                                                     request_adu, sizeof(request_adu),
                                                     &request_length) == MBRTUM_OK);
    CHECK(mbrtum_process_response(&request, request_adu, request_length,
                                  &response) == MBRTUM_OK);

    CHECK(mbrtum_build_write_multiple_coils_request(1u, 3u, 10u, coils,
                                                    &request, request_adu,
                                                    sizeof(request_adu), &request_length) == MBRTUM_OK);
    memcpy(response_adu, (const uint8_t[]){0x01u, 0x0Fu, 0x00u, 0x03u, 0x00u, 0x0Au}, 6u);
    response_length = append_crc(response_adu, 6u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_OK);

    CHECK(mbrtum_build_write_multiple_registers_request(1u, 10u, 3u, registers,
                                                        &request, request_adu,
                                                        sizeof(request_adu), &request_length) == MBRTUM_OK);
    memcpy(response_adu, (const uint8_t[]){0x01u, 0x10u, 0x00u, 0x0Au, 0x00u, 0x03u}, 6u);
    response_length = append_crc(response_adu, 6u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.data == NULL && response.data_length == 0u);

    response_adu[5] ^= 0x01u;
    response_length = append_crc(response_adu, 6u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) ==
          MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH);

    CHECK(mbrtum_build_write_single_register_request(1u, 9u, 0xCAFEu, &request,
                                                     request_adu,
                                                     sizeof(request_adu),
                                                     &request_length) ==
          MBRTUM_OK);
    memcpy(response_adu, request_adu, request_length);
    response_adu[5] ^= 0x01u;
    response_length = append_crc(response_adu, 6u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) ==
          MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH);
    return EXIT_SUCCESS;
}

static int test_exception_and_validation_errors(void)
{
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(mbrtum_build_read_registers_request(1u, MBRTUM_FC_READ_HOLDING_REGISTERS,
                                              0u, 2u, &request, request_adu,
                                              sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(mbrtum_process_response(&request, response_adu, 4u,
                                  &response) == MBRTUM_ERROR_RESPONSE_LENGTH);

    memcpy(response_adu,
           (const uint8_t[]){0x01u, 0x83u, 0x02u, 0x00u},
           4u);
    response_length = append_crc(response_adu, 4u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_RESPONSE_LENGTH);

    memcpy(response_adu, (const uint8_t[]){0x01u, 0x83u, 0x02u}, 3u);
    response_length = append_crc(response_adu, 3u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_EXCEPTION_RESPONSE);
    CHECK(response.function == MBRTUM_FC_READ_HOLDING_REGISTERS);
    CHECK(response.exception_code == MBRTUM_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    CHECK(response.data == NULL && response.data_length == 0u);

    response_adu[2] = 0u;
    response_length = append_crc(response_adu, 3u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_MALFORMED_RESPONSE);

    memcpy(response_adu, (const uint8_t[]){0x01u, 0x03u, 0x04u, 0x12u, 0x34u, 0xABu, 0xCDu}, 7u);
    response_length = append_crc(response_adu, 7u);
    response_adu[response_length - 1u] ^= 0x01u;
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_CRC);

    response_adu[0] = 2u;
    response_length = append_crc(response_adu, 7u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_ADDRESS_MISMATCH);

    memcpy(response_adu, (const uint8_t[]){0x01u, 0x04u, 0x04u, 0x12u, 0x34u, 0xABu, 0xCDu}, 7u);
    response_length = append_crc(response_adu, 7u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_FUNCTION_MISMATCH);

    memcpy(response_adu, (const uint8_t[]){0x01u, 0x03u, 0x02u, 0x12u, 0x34u, 0xABu, 0xCDu}, 7u);
    response_length = append_crc(response_adu, 7u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_MALFORMED_RESPONSE);
    return EXIT_SUCCESS;
}

static int test_broadcast_and_api_validation(void)
{
    mbrtum_request_t request;
    mbrtum_response_t response = {99u, 99u, (const uint8_t *)1, 99u};
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 99u;

    CHECK(mbrtum_build_write_single_register_request(0u, 1u, 2u, &request,
                                                     adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(request.expects_response == 0u);
    CHECK(mbrtum_process_response(&request, adu, adu_length,
                                  &response) == MBRTUM_ERROR_RESPONSE_NOT_EXPECTED);
    CHECK(response.function == 0u && response.exception_code == 0u &&
          response.data == NULL && response.data_length == 0u);

    CHECK(mbrtum_build_read_bits_request(0u, MBRTUM_FC_READ_COILS, 0u, 1u,
                                         &request, adu, sizeof(adu), &adu_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(adu_length == 0u);
    CHECK(mbrtum_build_read_bits_request(1u, MBRTUM_FC_READ_HOLDING_REGISTERS,
                                         0u, 1u, &request, adu, sizeof(adu),
                                         &adu_length) == MBRTUM_ERROR_FUNCTION);
    CHECK(mbrtum_build_read_bits_request(1u, MBRTUM_FC_READ_COILS, 0u, 0u,
                                         &request, adu, sizeof(adu),
                                         &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_bits_request(1u, MBRTUM_FC_READ_COILS,
                                         UINT16_MAX, 2u, &request, adu,
                                         sizeof(adu), &adu_length) ==
          MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_bits_request(1u, MBRTUM_FC_READ_COILS, 0u, 1u,
                                         &request, adu, 7u,
                                         &adu_length) == MBRTUM_ERROR_CAPACITY);
    CHECK(mbrtum_build_write_single_coil_request(1u, 0u, 2u, &request,
                                                 adu, sizeof(adu),
                                                 &adu_length) == MBRTUM_ERROR_VALUE);
    CHECK(mbrtum_build_write_multiple_registers_request(
              1u, UINT16_MAX, 2u, (const uint16_t[]){1u, 2u},
              &request, adu, sizeof(adu), &adu_length) ==
          MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_bits_request(1u, MBRTUM_FC_READ_COILS, 0u, 1u,
                                         NULL, adu, sizeof(adu),
                                         &adu_length) == MBRTUM_ERROR_ARGUMENT);
    CHECK(mbrtum_process_response(NULL, adu, 5u, &response) == MBRTUM_ERROR_ARGUMENT);
    return EXIT_SUCCESS;
}

static int test_maximum_request_sizes(void)
{
    uint8_t coils[246u];
    uint16_t registers[123u];
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    memset(coils, 0xA5, sizeof(coils));
    for (uint16_t i = 0u; i < 123u; ++i) {
        registers[i] = (uint16_t)(0x1000u + i);
    }

    CHECK(mbrtum_build_write_multiple_coils_request(1u, 0u, 1968u, coils,
                                                    &request, adu, sizeof(adu),
                                                    &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 255u && crc_is_valid(adu, adu_length));
    CHECK(mbrtum_build_write_multiple_coils_request(1u, 0u, 1968u, coils,
                                                    &request, adu, 254u,
                                                    &adu_length) == MBRTUM_ERROR_CAPACITY);

    CHECK(mbrtum_build_write_multiple_registers_request(1u, 0u, 123u, registers,
                                                        &request, adu, sizeof(adu),
                                                        &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 255u && crc_is_valid(adu, adu_length));
    CHECK(mbrtum_build_write_multiple_registers_request(1u, 0u, 123u, registers,
                                                        &request, adu, 254u,
                                                        &adu_length) == MBRTUM_ERROR_CAPACITY);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_read_request_builders() == EXIT_SUCCESS);
    CHECK(test_single_write_request_builders() == EXIT_SUCCESS);
    CHECK(test_multiple_write_request_builders() == EXIT_SUCCESS);
    CHECK(test_read_response_decoding() == EXIT_SUCCESS);
    CHECK(test_write_acknowledgements() == EXIT_SUCCESS);
    CHECK(test_exception_and_validation_errors() == EXIT_SUCCESS);
    CHECK(test_broadcast_and_api_validation() == EXIT_SUCCESS);
    CHECK(test_maximum_request_sizes() == EXIT_SUCCESS);
    puts("modbus RTU master tests: PASS");
    return EXIT_SUCCESS;
}
