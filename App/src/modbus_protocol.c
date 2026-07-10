#include "modbus_protocol.h"

#include "modbus.h"
#include "platform_port.h"

#include <string.h>

#define MB_EX_ILLEGAL_FUNCTION     0x01u
#define MB_EX_ILLEGAL_DATA_ADDRESS 0x02u
#define MB_EX_ILLEGAL_DATA_VALUE   0x03u
#define MB_EX_SERVER_FAILURE       0x04u

static uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static int range_is_valid(uint16_t first, uint16_t quantity, uint32_t limit)
{
    return quantity > 0u && (uint32_t)first < limit &&
           (uint32_t)quantity <= (limit - (uint32_t)first);
}

static uint8_t process_pdu(const uint8_t *pdu,
                           size_t pdu_len,
                           uint8_t *response,
                           size_t response_capacity,
                           size_t *response_len)
{
    uint8_t function;
    uint16_t address;
    uint16_t quantity;
    size_t byte_count;

    if (pdu == NULL || response == NULL || response_len == NULL || pdu_len == 0u) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    function = pdu[0];

    switch (function) {
    case 0x01u: /* Read Coils */
    case 0x02u: /* Read Discrete Inputs */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 2000u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity,
                            function == 0x01u ? MB_MAX_COILS : MB_MAX_DISCRETE_INPUTS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = ((size_t)quantity + 7u) / 8u;
        if (response_capacity < 2u + byte_count) {
            return MB_EX_SERVER_FAILURE;
        }
        response[0] = function;
        response[1] = (uint8_t)byte_count;
        memset(&response[2], 0, byte_count);
        for (uint16_t i = 0; i < quantity; ++i) {
            uint8_t bit = function == 0x01u ? mb_get_coil((uint16_t)(address + i))
                                            : mb_get_dinput((uint16_t)(address + i));
            response[2u + (i / 8u)] |= (uint8_t)((bit & 1u) << (i % 8u));
        }
        *response_len = 2u + byte_count;
        return 0u;

    case 0x03u: /* Read Holding Registers */
    case 0x04u: /* Read Input Registers */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 125u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity,
                            function == 0x03u ? MB_MAX_HREGS : MB_MAX_IREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = (size_t)quantity * 2u;
        if (response_capacity < 2u + byte_count) {
            return MB_EX_SERVER_FAILURE;
        }
        response[0] = function;
        response[1] = (uint8_t)byte_count;
        for (uint16_t i = 0; i < quantity; ++i) {
            uint16_t value = function == 0x03u ? mb_get_hreg((uint16_t)(address + i))
                                               : mb_get_ireg((uint16_t)(address + i));
            write_be16(&response[2u + (2u * i)], value);
        }
        *response_len = 2u + byte_count;
        return 0u;

    case 0x05u: /* Write Single Coil */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        if (!range_is_valid(address, 1u, MB_MAX_COILS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        quantity = read_be16(&pdu[3]);
        if (quantity == 0xFF00u) {
            mb_set_coil(address, 1u);
        } else if (quantity == 0x0000u) {
            mb_set_coil(address, 0u);
        } else {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        memcpy(response, pdu, 5u);
        *response_len = 5u;
        return 0u;

    case 0x06u: /* Write Single Holding Register */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        if (!range_is_valid(address, 1u, MB_MAX_HREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        mb_set_hreg(address, read_be16(&pdu[3]));
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        memcpy(response, pdu, 5u);
        *response_len = 5u;
        return 0u;

    case 0x0Fu: /* Write Multiple Coils */
        if (pdu_len < 6u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 1968u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity, MB_MAX_COILS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = ((size_t)quantity + 7u) / 8u;
        if (pdu[5] != byte_count || pdu_len != 6u + byte_count) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        for (uint16_t i = 0; i < quantity; ++i) {
            uint8_t value = (uint8_t)(((uint32_t)pdu[6u + (i / 8u)] >> (i % 8u)) & 1u);
            mb_set_coil((uint16_t)(address + i), value);
        }
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        response[0] = function;
        write_be16(&response[1], address);
        write_be16(&response[3], quantity);
        *response_len = 5u;
        return 0u;

    case 0x10u: /* Write Multiple Holding Registers */
        if (pdu_len < 6u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 123u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity, MB_MAX_HREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = (size_t)quantity * 2u;
        if (pdu[5] != byte_count || pdu_len != 6u + byte_count) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        for (uint16_t i = 0; i < quantity; ++i) {
            mb_set_hreg((uint16_t)(address + i), read_be16(&pdu[6u + (2u * i)]));
        }
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        response[0] = function;
        write_be16(&response[1], address);
        write_be16(&response[3], quantity);
        *response_len = 5u;
        return 0u;

    default:
        return MB_EX_ILLEGAL_FUNCTION;
    }
}

int mbtcp_process_adu(const uint8_t *request,
                      size_t request_len,
                      uint8_t *response,
                      size_t response_capacity,
                      size_t *response_len)
{
    uint16_t length_field;
    size_t expected_len;
    size_t response_pdu_len = 0u;
    uint8_t exception;

    if (request == NULL || response == NULL || response_len == NULL) {
        return -1;
    }
    *response_len = 0u;

    if (request_len < MBTCP_MBAP_HEADER_SIZE + 1u ||
        request_len > MODBUS_TCP_ADU_MAX_SIZE) {
        return -2;
    }
    if (request[2] != 0u || request[3] != 0u) {
        return -3;
    }

    length_field = read_be16(&request[4]);
    expected_len = 6u + (size_t)length_field;
    if (length_field < 2u || expected_len != request_len) {
        return -4;
    }
    if (response_capacity < MBTCP_MBAP_HEADER_SIZE + 2u) {
        return -5;
    }

    exception = process_pdu(&request[7],
                            request_len - MBTCP_MBAP_HEADER_SIZE,
                            &response[7],
                            response_capacity - MBTCP_MBAP_HEADER_SIZE,
                            &response_pdu_len);

    memcpy(response, request, MBTCP_MBAP_HEADER_SIZE);
    if (exception != 0u) {
        response[7] = (uint8_t)(request[7] | 0x80u);
        response[8] = exception;
        response_pdu_len = 2u;
    }

    length_field = (uint16_t)(1u + response_pdu_len);
    write_be16(&response[4], length_field);
    *response_len = MBTCP_MBAP_HEADER_SIZE + response_pdu_len;
    return 0;
}
