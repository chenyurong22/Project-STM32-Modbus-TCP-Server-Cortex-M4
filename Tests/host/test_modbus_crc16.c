#include "modbus_crc16.h"

#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

static int test_known_vectors(void)
{
    const uint8_t read_holding[] = {0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x0Au};
    const uint8_t read_coils[] = {0x01u, 0x01u, 0x00u, 0x13u, 0x00u, 0x25u};
    const uint8_t serial_guide_example[] = {0x11u, 0x03u, 0x00u, 0x6Bu, 0x00u, 0x03u};
    const uint8_t write_register[] = {0x01u, 0x06u, 0x00u, 0x01u, 0x00u, 0x03u};

    CHECK(mb_crc16(NULL, 0u) == 0xFFFFu);
    CHECK(mb_crc16(read_holding, sizeof(read_holding)) == 0xCDC5u);
    CHECK(mb_crc16(read_coils, sizeof(read_coils)) == 0x140Cu);
    CHECK(mb_crc16(serial_guide_example, sizeof(serial_guide_example)) == 0x8776u);
    CHECK(mb_crc16(write_register, sizeof(write_register)) == 0x0B98u);
    return EXIT_SUCCESS;
}

static int test_wire_order_and_corruption(void)
{
    uint8_t frame[] = {0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x0Au, 0xC5u, 0xCDu};

    CHECK(mb_crc16(frame, sizeof(frame)) == 0u);
    frame[3] ^= 0x01u;
    CHECK(mb_crc16(frame, sizeof(frame)) != 0u);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_known_vectors() == EXIT_SUCCESS);
    CHECK(test_wire_order_and_corruption() == EXIT_SUCCESS);
    puts("modbus CRC-16 tests: PASS");
    return EXIT_SUCCESS;
}
