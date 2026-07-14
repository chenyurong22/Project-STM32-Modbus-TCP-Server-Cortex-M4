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

typedef struct {
    mbrtu_context_t context;
    uint8_t receive_a[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t receive_b[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t transmitted[MODBUS_RTU_ADU_MAX_SIZE];
    size_t transmitted_length;
    uint32_t transmit_count;
    int transmit_result;
} fixture_t;

static int capture_transmit(void *user_context,
                            const uint8_t *data,
                            size_t length)
{
    fixture_t *fixture = (fixture_t *)user_context;

    if (length > sizeof(fixture->transmitted)) {
        return -1;
    }

    memcpy(fixture->transmitted, data, length);
    fixture->transmitted_length = length;
    ++fixture->transmit_count;
    return fixture->transmit_result;
}

static int fixture_init(fixture_t *fixture,
                        uint32_t baud_rate,
                        size_t receive_capacity)
{
    mbrtu_config_t config;

    memset(fixture, 0, sizeof(*fixture));
    config.slave_address = 1u;
    config.baud_rate = baud_rate;
    config.data_bits = 8u;
    config.parity_bits = 1u;
    config.stop_bits = 1u;
    config.receive_buffer_a = fixture->receive_a;
    config.receive_buffer_b = fixture->receive_b;
    config.receive_buffer_capacity = receive_capacity;
    config.response_buffer = fixture->response;
    config.response_buffer_capacity = sizeof(fixture->response);
    config.transmit = capture_transmit;
    config.user_context = fixture;

    return mbrtu_init(&fixture->context, &config);
}

static size_t make_request(uint8_t address,
                           const uint8_t *pdu,
                           size_t pdu_length,
                           uint8_t *adu)
{
    uint16_t crc;
    size_t length_without_crc = 1u + pdu_length;

    adu[0] = address;
    memcpy(&adu[1], pdu, pdu_length);
    crc = mb_crc16(adu, length_without_crc);
    adu[length_without_crc] = (uint8_t)crc;
    adu[length_without_crc + 1u] = (uint8_t)(crc >> 8u);
    return length_without_crc + MODBUS_RTU_CRC_SIZE;
}

static void advance_ticks(mbrtu_context_t *context, uint32_t tick_count)
{
    for (uint32_t tick = 0u; tick < tick_count; ++tick) {
        mbrtu_on_50us_tick_isr(context);
    }
}

static void receive_bytes(mbrtu_context_t *context,
                          const uint8_t *data,
                          size_t length,
                          uint32_t inter_byte_ticks)
{
    for (size_t index = 0u; index < length; ++index) {
        mbrtu_on_rx_byte_isr(context, data[index]);
        if (index + 1u < length) {
            advance_ticks(context, inter_byte_ticks);
        }
    }
}

static int transmitted_crc_is_valid(const fixture_t *fixture)
{
    return fixture->transmitted_length >= MODBUS_RTU_ADU_MIN_SIZE &&
           mb_crc16(fixture->transmitted,
                    fixture->transmitted_length) == 0u;
}

static int test_timing_calculation(void)
{
    mbrtu_timing_t timing;

    CHECK(mbrtu_calculate_timing(9600u, 8u, 1u, 1u, &timing) == 0);
    CHECK(timing.character_ticks == 23u);
    CHECK(timing.t1_5_ticks == 35u);
    CHECK(timing.t3_5_ticks == 81u);
    CHECK(timing.byte_complete_t1_5_ticks == 58u);
    CHECK(timing.byte_complete_t3_5_ticks == 104u);

    CHECK(mbrtu_calculate_timing(19200u, 8u, 1u, 1u, &timing) == 0);
    CHECK(timing.character_ticks == 12u);
    CHECK(timing.t1_5_ticks == 18u);
    CHECK(timing.t3_5_ticks == 41u);
    CHECK(timing.byte_complete_t1_5_ticks == 29u);
    CHECK(timing.byte_complete_t3_5_ticks == 52u);

    CHECK(mbrtu_calculate_timing(115200u, 8u, 1u, 1u, &timing) == 0);
    CHECK(timing.character_ticks == 2u);
    CHECK(timing.t1_5_ticks == 15u);
    CHECK(timing.t3_5_ticks == 35u);
    CHECK(timing.byte_complete_t1_5_ticks == 17u);
    CHECK(timing.byte_complete_t3_5_ticks == 37u);

    CHECK(mbrtu_calculate_timing(9600u, 8u, 0u, 1u, &timing) == 0);
    CHECK(timing.character_ticks == 21u);
    CHECK(timing.t1_5_ticks == 32u);
    CHECK(timing.t3_5_ticks == 73u);
    CHECK(timing.byte_complete_t1_5_ticks == 53u);
    CHECK(timing.byte_complete_t3_5_ticks == 94u);

    CHECK(mbrtu_calculate_timing(0u, 8u, 1u, 1u, &timing) ==
          MBRTU_ERROR_SERIAL_CONFIG);
    CHECK(mbrtu_calculate_timing(9600u, 4u, 1u, 1u, &timing) ==
          MBRTU_ERROR_SERIAL_CONFIG);
    CHECK(mbrtu_calculate_timing(9600u, 8u, 2u, 1u, &timing) ==
          MBRTU_ERROR_SERIAL_CONFIG);
    CHECK(mbrtu_calculate_timing(9600u, 8u, 1u, 0u, &timing) ==
          MBRTU_ERROR_SERIAL_CONFIG);
    CHECK(mbrtu_calculate_timing(9600u, 8u, 1u, 1u, NULL) ==
          MBRTU_ERROR_ARGUMENT);
    return EXIT_SUCCESS;
}

static int test_valid_frame_and_exact_t1_5_gap(void)
{
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    fixture_t fixture;

    CHECK(fixture_init(&fixture, 115200u,
                       sizeof(fixture.receive_a)) == 0);
    mb_init();
    request_length = make_request(1u, write_register,
                                  sizeof(write_register), request);

    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.byte_complete_t1_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_IDLE);

    advance_ticks(&fixture.context,
                  fixture.context.timing.byte_complete_t3_5_ticks - 1u);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_IDLE);
    advance_ticks(&fixture.context, 1u);

    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(fixture.transmit_count == 1u);
    CHECK(fixture.transmitted_length == request_length);
    CHECK(memcmp(fixture.transmitted, request, request_length) == 0);
    CHECK(transmitted_crc_is_valid(&fixture));
    CHECK(mb_get_hreg(9u) == 0xCAFEu);
    return EXIT_SUCCESS;
}

static int test_invalid_inter_character_gap_and_recovery(void)
{
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0x12u, 0x34u};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    fixture_t fixture;
    mbrtu_stats_t stats;

    CHECK(fixture_init(&fixture, 115200u,
                       sizeof(fixture.receive_a)) == 0);
    mb_init();
    request_length = make_request(1u, write_register,
                                  sizeof(write_register), request);

    receive_bytes(&fixture.context, request, 3u,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context,
                  fixture.context.timing.byte_complete_t1_5_ticks + 1u);
    receive_bytes(&fixture.context, &request[3], request_length - 3u,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);

    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_FRAME_DROPPED);
    CHECK(fixture.transmit_count == 0u);
    CHECK(mb_get_hreg(9u) == 0u);

    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(mb_get_hreg(9u) == 0x1234u);

    mbrtu_get_stats(&fixture.context, &stats);
    CHECK(stats.invalid_gap_events == 1u);
    CHECK(stats.responses_sent == 1u);
    return EXIT_SUCCESS;
}

static int test_byte_complete_t3_5_boundaries(void)
{
    const uint8_t write_register_a[] = {0x06u, 0x00u, 0x0Au, 0x11u, 0x11u};
    const uint8_t write_register_b[] = {0x06u, 0x00u, 0x0Bu, 0x22u, 0x22u};
    uint8_t request_a[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t request_b[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_a_length;
    size_t request_b_length;
    fixture_t fixture;

    CHECK(fixture_init(&fixture, 115200u,
                       sizeof(fixture.receive_a)) == 0);
    mb_init();
    request_a_length = make_request(1u, write_register_a,
                                    sizeof(write_register_a), request_a);
    request_b_length = make_request(1u, write_register_b,
                                    sizeof(write_register_b), request_b);

    /* A byte completing just before the adjusted T3.5 deadline belongs to
     * the invalid T1.5-to-T3.5 interval, so the combined frame is dropped. */
    receive_bytes(&fixture.context, request_a, request_a_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context,
                  fixture.context.timing.byte_complete_t3_5_ticks - 1u);
    receive_bytes(&fixture.context, request_b, request_b_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context,
                  fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_FRAME_DROPPED);
    CHECK(mb_get_hreg(10u) == 0u);
    CHECK(mb_get_hreg(11u) == 0u);

    /* At the adjusted T3.5 deadline, the first frame is complete and the
     * next receive-complete event starts a new frame. */
    receive_bytes(&fixture.context, request_a, request_a_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context,
                  fixture.context.timing.byte_complete_t3_5_ticks);
    receive_bytes(&fixture.context, request_b, request_b_length,
                  fixture.context.timing.character_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(mb_get_hreg(10u) == 0x1111u);

    advance_ticks(&fixture.context,
                  fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(mb_get_hreg(11u) == 0x2222u);
    return EXIT_SUCCESS;
}

static int test_two_buffer_handoff(void)
{
    const uint8_t write_register_a[] = {0x06u, 0x00u, 0x0Au, 0x11u, 0x11u};
    const uint8_t write_register_b[] = {0x06u, 0x00u, 0x0Bu, 0x22u, 0x22u};
    uint8_t request_a[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t request_b[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_a_length;
    size_t request_b_length;
    fixture_t fixture;

    CHECK(fixture_init(&fixture, 19200u,
                       sizeof(fixture.receive_a)) == 0);
    mb_init();
    request_a_length = make_request(1u, write_register_a,
                                    sizeof(write_register_a), request_a);
    request_b_length = make_request(1u, write_register_b,
                                    sizeof(write_register_b), request_b);

    receive_bytes(&fixture.context, request_a, request_a_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);

    receive_bytes(&fixture.context, request_b, request_b_length,
                  fixture.context.timing.character_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(mb_get_hreg(10u) == 0x1111u);

    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(mb_get_hreg(11u) == 0x2222u);
    CHECK(fixture.transmit_count == 2u);
    return EXIT_SUCCESS;
}

static int test_pending_frame_overrun_and_recovery(void)
{
    const uint8_t write_register_a[] = {0x06u, 0x00u, 0x0Au, 0x11u, 0x11u};
    const uint8_t write_register_b[] = {0x06u, 0x00u, 0x0Bu, 0x22u, 0x22u};
    const uint8_t write_register_c[] = {0x06u, 0x00u, 0x0Cu, 0x33u, 0x33u};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    fixture_t fixture;
    mbrtu_stats_t stats;

    CHECK(fixture_init(&fixture, 115200u,
                       sizeof(fixture.receive_a)) == 0);
    mb_init();

    request_length = make_request(1u, write_register_a,
                                  sizeof(write_register_a), request);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);

    request_length = make_request(1u, write_register_b,
                                  sizeof(write_register_b), request);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);

    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(mb_get_hreg(10u) == 0x1111u);
    CHECK(mb_get_hreg(11u) == 0u);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_IDLE);

    request_length = make_request(1u, write_register_c,
                                  sizeof(write_register_c), request);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(mb_get_hreg(12u) == 0x3333u);

    mbrtu_get_stats(&fixture.context, &stats);
    CHECK(stats.overrun_frames == 1u);
    return EXIT_SUCCESS;
}

static int test_overflow_and_recovery(void)
{
    const uint8_t write_registers[] = {0x10u, 0x00u, 0x0Au, 0x00u, 0x02u,
                                       0x04u, 0x11u, 0x11u, 0x22u, 0x22u};
    const uint8_t unsupported[] = {0x7Fu};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    fixture_t fixture;
    mbrtu_stats_t stats;

    CHECK(fixture_init(&fixture, 9600u, 8u) == 0);
    mb_init();

    request_length = make_request(1u, write_registers,
                                  sizeof(write_registers), request);
    CHECK(request_length > 8u);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_FRAME_DROPPED);
    CHECK(mb_get_hreg(10u) == 0u);
    CHECK(mb_get_hreg(11u) == 0u);

    request_length = make_request(1u, unsupported,
                                  sizeof(unsupported), request);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(fixture.transmitted[1] == 0xFFu);
    CHECK(fixture.transmitted[2] == 0x01u);

    mbrtu_get_stats(&fixture.context, &stats);
    CHECK(stats.overflow_events == 1u);
    return EXIT_SUCCESS;
}

static int test_broadcast_wrong_address_and_crc(void)
{
    const uint8_t write_register[] = {0x06u, 0x00u, 0x09u, 0xCAu, 0xFEu};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    fixture_t fixture;

    CHECK(fixture_init(&fixture, 115200u,
                       sizeof(fixture.receive_a)) == 0);
    mb_init();

    request_length = make_request(0u, write_register,
                                  sizeof(write_register), request);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_NO_RESPONSE);
    CHECK(mb_get_hreg(9u) == 0xCAFEu);
    CHECK(fixture.transmit_count == 0u);

    request_length = make_request(2u, write_register,
                                  sizeof(write_register), request);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_NO_RESPONSE);
    CHECK(fixture.transmit_count == 0u);

    request_length = make_request(1u, write_register,
                                  sizeof(write_register), request);
    request[request_length - 1u] ^= 0x01u;
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_NO_RESPONSE);
    CHECK(fixture.transmit_count == 0u);
    return EXIT_SUCCESS;
}

static int test_transmit_error_and_api_validation(void)
{
    const uint8_t unsupported[] = {0x7Fu};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    fixture_t fixture;
    mbrtu_config_t config;
    mbrtu_context_t context;
    mbrtu_stats_t stats;

    CHECK(fixture_init(&fixture, 115200u,
                       sizeof(fixture.receive_a)) == 0);
    fixture.transmit_result = -1;
    request_length = make_request(1u, unsupported,
                                  sizeof(unsupported), request);
    receive_bytes(&fixture.context, request, request_length,
                  fixture.context.timing.character_ticks);
    advance_ticks(&fixture.context, fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_ERROR_TRANSMIT);
    mbrtu_get_stats(&fixture.context, &stats);
    CHECK(stats.transmit_errors == 1u);

    memset(&config, 0, sizeof(config));
    CHECK(mbrtu_init(NULL, &config) == MBRTU_ERROR_ARGUMENT);
    CHECK(mbrtu_init(&context, NULL) == MBRTU_ERROR_ARGUMENT);

    config.slave_address = 1u;
    config.baud_rate = 19200u;
    config.data_bits = 8u;
    config.parity_bits = 1u;
    config.stop_bits = 1u;
    config.receive_buffer_a = fixture.receive_a;
    config.receive_buffer_b = fixture.receive_b;
    config.receive_buffer_capacity = sizeof(fixture.receive_a);
    config.response_buffer = fixture.response;
    config.response_buffer_capacity = sizeof(fixture.response);
    config.transmit = capture_transmit;
    config.user_context = &fixture;

    config.slave_address = 0u;
    CHECK(mbrtu_init(&context, &config) == MBRTU_ERROR_SLAVE_ADDRESS);
    config.slave_address = 248u;
    CHECK(mbrtu_init(&context, &config) == MBRTU_ERROR_SLAVE_ADDRESS);
    config.slave_address = 1u;

    config.receive_buffer_capacity = 3u;
    CHECK(mbrtu_init(&context, &config) == MBRTU_ERROR_ARGUMENT);
    config.receive_buffer_capacity = MODBUS_RTU_ADU_MAX_SIZE + 1u;
    CHECK(mbrtu_init(&context, &config) == MBRTU_ERROR_ARGUMENT);
    config.receive_buffer_capacity = sizeof(fixture.receive_a);

    config.receive_buffer_b = fixture.receive_a;
    CHECK(mbrtu_init(&context, &config) == MBRTU_ERROR_ARGUMENT);
    config.receive_buffer_b = fixture.receive_b;
    config.response_buffer_capacity = 4u;
    CHECK(mbrtu_init(&context, &config) == MBRTU_ERROR_ARGUMENT);
    config.response_buffer_capacity = sizeof(fixture.response);
    config.transmit = NULL;
    CHECK(mbrtu_init(&context, &config) == MBRTU_ERROR_ARGUMENT);

    CHECK(mbrtu_poll(NULL) == MBRTU_ERROR_ARGUMENT);
    mbrtu_on_rx_byte_isr(NULL, 0u);
    mbrtu_on_50us_tick_isr(NULL);
    mbrtu_get_stats(NULL, &stats);
    mbrtu_get_stats(&fixture.context, NULL);

    advance_ticks(&fixture.context, 100000u);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_IDLE);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_timing_calculation() == EXIT_SUCCESS);
    CHECK(test_valid_frame_and_exact_t1_5_gap() == EXIT_SUCCESS);
    CHECK(test_invalid_inter_character_gap_and_recovery() == EXIT_SUCCESS);
    CHECK(test_byte_complete_t3_5_boundaries() == EXIT_SUCCESS);
    CHECK(test_two_buffer_handoff() == EXIT_SUCCESS);
    CHECK(test_pending_frame_overrun_and_recovery() == EXIT_SUCCESS);
    CHECK(test_overflow_and_recovery() == EXIT_SUCCESS);
    CHECK(test_broadcast_wrong_address_and_crc() == EXIT_SUCCESS);
    CHECK(test_transmit_error_and_api_validation() == EXIT_SUCCESS);
    puts("modbus RTU timing tests: PASS");
    return EXIT_SUCCESS;
}
