#ifndef MODBUS_PDU_H
#define MODBUS_PDU_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_PDU_MAX_SIZE 253u

/**
 * Process exactly one Modbus Protocol Data Unit (PDU).
 *
 * A request with a valid function byte always produces either a normal Modbus
 * response PDU or a two-byte Modbus exception response PDU. Transport framing
 * such as an MBAP header, RTU slave address, or CRC is intentionally excluded.
 *
 * The request and response buffers must not overlap.
 *
 * @return 0 when a response PDU was produced, negative for invalid arguments,
 *         an empty/oversized request PDU, or insufficient exception capacity.
 */
int mb_process_pdu(const uint8_t *request_pdu,
                   size_t request_pdu_len,
                   uint8_t *response_pdu,
                   size_t response_capacity,
                   size_t *response_pdu_len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_PDU_H */
