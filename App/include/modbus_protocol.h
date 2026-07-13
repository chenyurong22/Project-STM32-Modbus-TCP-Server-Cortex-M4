#ifndef MODBUS_PROTOCOL_H
#define MODBUS_PROTOCOL_H

#include "modbus_pdu.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MBTCP_MBAP_HEADER_SIZE 7u
#define MODBUS_TCP_ADU_MAX_SIZE (MBTCP_MBAP_HEADER_SIZE + MODBUS_PDU_MAX_SIZE)

/**
 * Process exactly one Modbus TCP Application Data Unit (ADU).
 *
 * A valid request always produces either a normal Modbus response or a Modbus
 * exception response. Malformed MBAP frames return a negative value and do not
 * produce a response.
 *
 * @return 0 on success, negative on malformed input or insufficient output space.
 */
int mbtcp_process_adu(const uint8_t *request,
                      size_t request_len,
                      uint8_t *response,
                      size_t response_capacity,
                      size_t *response_len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_PROTOCOL_H */
