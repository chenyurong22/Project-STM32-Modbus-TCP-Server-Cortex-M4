#include "modbus_rtu.h"

#include "modbus_crc16.h"
#include "modbus_pdu.h"

static uint16_t read_crc_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8u));
}

static void write_crc_le(uint8_t *p, uint16_t crc)
{
    p[0] = (uint8_t)crc;
    p[1] = (uint8_t)(crc >> 8u);
}

static int is_supported_write_function(uint8_t function)
{
    return function == 0x05u || function == 0x06u ||
           function == 0x0Fu || function == 0x10u;
}

int mbrtu_process_adu(uint8_t slave_address,
                      const uint8_t *request_adu,
                      size_t request_adu_len,
                      uint8_t *response_adu,
                      size_t response_capacity,
                      size_t *response_adu_len)
{
    uint8_t request_address;
    size_t request_pdu_len;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (response_adu_len == NULL) {
        return MBRTU_ERROR_ARGUMENT;
    }
    *response_adu_len = 0u;

    if (request_adu == NULL || response_adu == NULL) {
        return MBRTU_ERROR_ARGUMENT;
    }

    if (slave_address < MODBUS_RTU_ADDRESS_MIN ||
        slave_address > MODBUS_RTU_ADDRESS_MAX) {
        return MBRTU_ERROR_SLAVE_ADDRESS;
    }

    if (request_adu_len < MODBUS_RTU_ADU_MIN_SIZE ||
        request_adu_len > MODBUS_RTU_ADU_MAX_SIZE) {
        return MBRTU_NO_RESPONSE;
    }

    request_address = request_adu[0];
    if (request_address != MODBUS_RTU_BROADCAST_ADDRESS &&
        request_address != slave_address) {
        return MBRTU_NO_RESPONSE;
    }

    received_crc = read_crc_le(&request_adu[request_adu_len - MODBUS_RTU_CRC_SIZE]);
    calculated_crc = mb_crc16(request_adu,
                              request_adu_len - MODBUS_RTU_CRC_SIZE);
    if (received_crc != calculated_crc) {
        return MBRTU_NO_RESPONSE;
    }

    request_pdu_len = request_adu_len - 1u - MODBUS_RTU_CRC_SIZE;

    if (request_address == MODBUS_RTU_BROADCAST_ADDRESS) {
        uint8_t discarded_response[5];
        size_t discarded_response_len = 0u;
        int pdu_result;

        if (!is_supported_write_function(request_adu[1])) {
            return MBRTU_NO_RESPONSE;
        }

        pdu_result = mb_process_pdu(&request_adu[1],
                                    request_pdu_len,
                                    discarded_response,
                                    sizeof(discarded_response),
                                    &discarded_response_len);
        if (pdu_result < 0) {
            return MBRTU_ERROR_PDU;
        }
        return MBRTU_NO_RESPONSE;
    }

    if (response_capacity < 1u + 2u + MODBUS_RTU_CRC_SIZE) {
        return MBRTU_ERROR_RESPONSE_CAPACITY;
    }

    {
        size_t response_pdu_len = 0u;
        size_t response_without_crc_len;
        int pdu_result;

        pdu_result = mb_process_pdu(&request_adu[1],
                                    request_pdu_len,
                                    &response_adu[1],
                                    response_capacity - 1u - MODBUS_RTU_CRC_SIZE,
                                    &response_pdu_len);
        if (pdu_result < 0) {
            return MBRTU_ERROR_PDU;
        }

        response_adu[0] = request_address;
        response_without_crc_len = 1u + response_pdu_len;
        calculated_crc = mb_crc16(response_adu, response_without_crc_len);
        write_crc_le(&response_adu[response_without_crc_len], calculated_crc);
        *response_adu_len = response_without_crc_len + MODBUS_RTU_CRC_SIZE;
    }

    return MBRTU_RESPONSE_READY;
}
