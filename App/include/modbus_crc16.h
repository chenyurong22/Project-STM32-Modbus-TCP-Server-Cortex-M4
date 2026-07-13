#ifndef MODBUS_CRC16_H
#define MODBUS_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Calculate the Modbus serial-line CRC-16 value.
 *
 * The returned 16-bit value is transmitted least-significant byte first on
 * the wire. data must be non-NULL when length is nonzero. An empty input
 * produces the Modbus initial value 0xFFFF.
 */
uint16_t mb_crc16(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_CRC16_H */
