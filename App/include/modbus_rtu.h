#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "modbus_pdu.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_RTU_ADDRESS_MIN 1u
#define MODBUS_RTU_ADDRESS_MAX 247u
#define MODBUS_RTU_BROADCAST_ADDRESS 0u
#define MODBUS_RTU_CRC_SIZE 2u
#define MODBUS_RTU_ADU_MIN_SIZE 4u
#define MODBUS_RTU_ADU_MAX_SIZE (1u + MODBUS_PDU_MAX_SIZE + MODBUS_RTU_CRC_SIZE)

#define MBRTU_NO_RESPONSE 0
#define MBRTU_RESPONSE_READY 1
#define MBRTU_ERROR_ARGUMENT (-1)
#define MBRTU_ERROR_SLAVE_ADDRESS (-2)
#define MBRTU_ERROR_RESPONSE_CAPACITY (-3)
#define MBRTU_ERROR_PDU (-4)

/**
 * Validate and process exactly one complete Modbus RTU Application Data Unit.
 *
 * request_adu contains: slave address, PDU, CRC low byte, CRC high byte.
 * request_adu and response_adu must not overlap. response_adu receives the
 * same slave address, the shared PDU response, and
 * a newly calculated CRC in low-byte/high-byte wire order.
 *
 * Frames with an invalid CRC, a different slave address, or an invalid RTU
 * length are silently ignored. Address zero is treated as a broadcast:
 * supported write requests are applied without a response, while broadcast
 * reads and unsupported functions are ignored.
 *
 * @return MBRTU_RESPONSE_READY when response_adu is ready to transmit,
 *         MBRTU_NO_RESPONSE when the frame is intentionally ignored or is a
 *         broadcast, or one of the MBRTU_ERROR_* values.
 */
int mbrtu_process_adu(uint8_t slave_address,
                      const uint8_t *request_adu,
                      size_t request_adu_len,
                      uint8_t *response_adu,
                      size_t response_capacity,
                      size_t *response_adu_len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
