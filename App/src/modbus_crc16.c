#include "modbus_crc16.h"

uint16_t mb_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0u; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 1u) != 0u) {
                crc = (uint16_t)((crc >> 1u) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1u);
            }
        }
    }

    return crc;
}
