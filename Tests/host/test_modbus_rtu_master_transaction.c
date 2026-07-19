#include "modbus_crc16.h"
#include "modbus_rtu_master_transaction.h"

#include <stdio.h>
#include <string.h>

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    return 1; } } while (0)

typedef struct {
    unsigned calls;
    int result;
    uint8_t last_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t last_length;
} fake_transport_t;

static int fake_transmit(void *context, const uint8_t *adu, size_t length)
{
    fake_transport_t *transport = (fake_transport_t *)context;
    ++transport->calls;
    transport->last_length = length;
    memcpy(transport->last_adu, adu, length);
    return transport->result;
}

static size_t append_crc(uint8_t *adu, size_t length)
{
    uint16_t crc = mb_crc16(adu, length);
    adu[length++] = (uint8_t)(crc & 0xFFu);
    adu[length++] = (uint8_t)(crc >> 8u);
    return length;
}

static void replace_crc(uint8_t *adu, size_t length)
{
    uint16_t crc = mb_crc16(adu, length - 2u);
    adu[length - 2u] = (uint8_t)(crc & 0xFFu);
    adu[length - 1u] = (uint8_t)(crc >> 8u);
}

static int build_fc03(mbrtum_request_t *request, uint8_t *adu, size_t *length)
{
    return mbrtum_build_read_registers_request(1u,
                                                MBRTUM_FC_READ_HOLDING_REGISTERS,
                                                0u,
                                                2u,
                                                request,
                                                adu,
                                                MODBUS_RTU_ADU_MAX_SIZE,
                                                length);
}

static size_t fc03_response(uint8_t *adu, uint8_t address)
{
    adu[0] = address;
    adu[1] = MBRTUM_FC_READ_HOLDING_REGISTERS;
    adu[2] = 4u;
    adu[3] = 0x12u;
    adu[4] = 0x34u;
    adu[5] = 0xABu;
    adu[6] = 0xCDu;
    return append_crc(adu, 7u);
}

static mbrtum_transaction_config_t config(uint16_t retries, uint32_t delay)
{
    mbrtum_transaction_config_t value;
    value.response_timeout_ticks = 100u;
    value.retry_delay_ticks = delay;
    value.max_retries = retries;
    value.retry_transport_errors = 1u;
    return value;
}

static int test_initialization_and_config_guards(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(0u, 0u);
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t length = 0u;

    memset(&txn, 0, sizeof(txn));
    CHECK(build_fc03(&request, adu, &length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 0u) ==
          MBRTUM_TXN_ERROR_NOT_INITIALIZED);
    CHECK(mbrtum_transaction_poll(&txn, 0u) ==
          MBRTUM_TXN_ERROR_NOT_INITIALIZED);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 0u) ==
          MBRTUM_TXN_ERROR_NOT_INITIALIZED);
    CHECK(mbrtum_transaction_is_active(&txn) == 0);

    cfg.retry_transport_errors = 2u;
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) ==
          MBRTUM_TXN_ERROR_CONFIG);
    cfg.retry_transport_errors = 1u;
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) ==
          MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 0u) ==
          MBRTUM_TXN_OK);
    return 0;
}

static int test_request_adu_validation(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(0u, 0u);
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t length = 0u;

    CHECK(build_fc03(&request, adu, &length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) ==
          MBRTUM_TXN_OK);

    adu[0] = 2u;
    replace_crc(adu, length);
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 0u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    CHECK(transport.calls == 0u);

    CHECK(build_fc03(&request, adu, &length) == MBRTUM_OK);
    adu[length - 1u] ^= 1u;
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 0u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    CHECK(transport.calls == 0u);

    CHECK(mbrtum_transaction_start(&txn, &request, adu, 3u, 0u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    return 0;
}

static int test_success_and_immutable_copy(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(0u, 0u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;
    uint16_t value = 0u;

    CHECK(build_fc03(&request, request_adu, &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 10u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_TRANSMITTING);
    CHECK(transport.calls == 1u);

    request.slave_address = 9u;
    request_adu[0] = 9u;
    CHECK(txn.request.slave_address == 1u);
    CHECK(txn.request_adu[0] == 1u);

    CHECK(mbrtum_transaction_on_tx_complete(&txn, 20u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_WAITING_RESPONSE);
    response_length = fc03_response(response_adu, 1u);
    CHECK(mbrtum_transaction_on_response(&txn, response_adu, response_length, 21u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(txn.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(mbrtum_get_register(&txn.request, &txn.response, 1u, &value) == MBRTUM_OK);
    CHECK(value == 0xABCDu);
    CHECK(txn.diagnostics.transactions_started == 1u);
    CHECK(txn.diagnostics.successful_transactions == 1u);
    return 0;
}

static int test_timeout_retry_then_success(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(1u, 10u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(build_fc03(&request, request_adu, &request_length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 0u) == 0);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 1u) == 0);
    CHECK(mbrtum_transaction_poll(&txn, 100u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_WAITING_RESPONSE);
    CHECK(mbrtum_transaction_poll(&txn, 101u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_RETRY_DELAY);
    CHECK(mbrtum_transaction_poll(&txn, 110u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_RETRY_DELAY);
    CHECK(mbrtum_transaction_poll(&txn, 111u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_TRANSMITTING);
    CHECK(transport.calls == 2u);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 112u) == 0);
    response_length = fc03_response(response_adu, 1u);
    CHECK(mbrtum_transaction_on_response(&txn, response_adu, response_length, 113u) == 0);
    CHECK(txn.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(txn.diagnostics.timeouts == 1u);
    CHECK(txn.diagnostics.retries_performed == 1u);
    return 0;
}

static int test_zero_delay_retry_is_poll_driven(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, -1, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(3u, 0u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;

    CHECK(build_fc03(&request, request_adu, &request_length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 7u) == 0);
    CHECK(transport.calls == 1u);
    CHECK(txn.state == MBRTUM_TXN_STATE_RETRY_DELAY);

    CHECK(mbrtum_transaction_poll(&txn, 7u) == 0);
    CHECK(transport.calls == 2u);
    CHECK(txn.state == MBRTUM_TXN_STATE_RETRY_DELAY);
    CHECK(mbrtum_transaction_poll(&txn, 7u) == 0);
    CHECK(transport.calls == 3u);
    CHECK(txn.state == MBRTUM_TXN_STATE_RETRY_DELAY);
    CHECK(mbrtum_transaction_poll(&txn, 7u) == 0);
    CHECK(transport.calls == 4u);
    CHECK(txn.state == MBRTUM_TXN_STATE_FAILED);
    CHECK(txn.result == MBRTUM_TXN_RESULT_TRANSPORT_ERROR);
    CHECK(txn.diagnostics.retries_performed == 3u);
    CHECK(txn.diagnostics.transport_errors == 4u);
    return 0;
}

static int test_retry_exhaustion(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(1u, 0u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;

    CHECK(build_fc03(&request, request_adu, &request_length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 0u) == 0);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 0u) == 0);
    CHECK(mbrtum_transaction_poll(&txn, 100u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_RETRY_DELAY);
    CHECK(mbrtum_transaction_poll(&txn, 100u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_TRANSMITTING);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 101u) == 0);
    CHECK(mbrtum_transaction_poll(&txn, 201u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_FAILED);
    CHECK(txn.result == MBRTUM_TXN_RESULT_TIMEOUT);
    CHECK(txn.diagnostics.timeouts == 2u);
    return 0;
}

static int test_malformed_then_response_error(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(0u, 0u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(build_fc03(&request, request_adu, &request_length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 0u) == 0);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 0u) == 0);
    response_length = fc03_response(response_adu, 1u);
    response_adu[response_length - 1u] ^= 1u;
    CHECK(mbrtum_transaction_on_response(&txn, response_adu, response_length, 1u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_WAITING_RESPONSE);
    CHECK(txn.diagnostics.malformed_responses == 1u);
    CHECK(mbrtum_transaction_poll(&txn, 100u) == 0);
    CHECK(txn.result == MBRTUM_TXN_RESULT_RESPONSE_ERROR);
    return 0;
}

static int test_late_response_is_not_accepted(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(0u, 0u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(build_fc03(&request, request_adu, &request_length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 0u) == 0);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 10u) == 0);
    response_length = fc03_response(response_adu, 1u);
    CHECK(mbrtum_transaction_on_response(&txn, response_adu, response_length, 110u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_FAILED);
    CHECK(txn.result == MBRTUM_TXN_RESULT_TIMEOUT);
    CHECK(txn.diagnostics.timeouts == 1u);
    CHECK(txn.diagnostics.late_responses == 1u);
    CHECK(txn.diagnostics.successful_transactions == 0u);
    return 0;
}

static int test_unrelated_ignored(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(0u, 0u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(build_fc03(&request, request_adu, &request_length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 0u) == 0);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 0u) == 0);
    response_length = fc03_response(response_adu, 2u);
    CHECK(mbrtum_transaction_on_response(&txn, response_adu, response_length, 1u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_WAITING_RESPONSE);
    CHECK(txn.diagnostics.unrelated_responses == 1u);
    return 0;
}

static int test_exception_completes_without_retry(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(3u, 0u);
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[5];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(build_fc03(&request, request_adu, &request_length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, request_adu, request_length, 0u) == 0);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 0u) == 0);
    response_adu[0] = 1u;
    response_adu[1] = (uint8_t)(MBRTUM_FC_READ_HOLDING_REGISTERS | 0x80u);
    response_adu[2] = MBRTUM_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    response_length = append_crc(response_adu, 3u);
    CHECK(mbrtum_transaction_on_response(&txn, response_adu, response_length, 1u) == 0);
    CHECK(txn.result == MBRTUM_TXN_RESULT_MODBUS_EXCEPTION);
    CHECK(txn.exception_code == MBRTUM_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    CHECK(transport.calls == 1u);
    return 0;
}

static int test_broadcast_and_transport_errors(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(1u, 0u);
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t length = 0u;

    CHECK(mbrtum_build_write_single_register_request(0u, 3u, 9u,
                                                      &request, adu, sizeof(adu), &length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 0u) == 0);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 1u) == 0);
    CHECK(txn.result == MBRTUM_TXN_RESULT_SUCCESS);

    transport.result = -1;
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 2u) == 0);
    CHECK(transport.calls == 2u);
    CHECK(txn.state == MBRTUM_TXN_STATE_RETRY_DELAY);
    CHECK(mbrtum_transaction_poll(&txn, 2u) == 0);
    CHECK(transport.calls == 3u);
    CHECK(txn.state == MBRTUM_TXN_STATE_FAILED);
    CHECK(txn.result == MBRTUM_TXN_RESULT_TRANSPORT_ERROR);
    CHECK(txn.diagnostics.transport_errors == 2u);
    return 0;
}

static int test_cancel_busy_state_and_wrap(void)
{
    mbrtum_transaction_t txn;
    fake_transport_t transport = {0u, 0, {0u}, 0u};
    mbrtum_transaction_config_t cfg = config(0u, 0u);
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t length = 0u;

    CHECK(build_fc03(&request, adu, &length) == 0);
    CHECK(mbrtum_transaction_init(&txn, &cfg, fake_transmit, &transport) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 0xFFFFFFF0u) == 0);
    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 0u) == MBRTUM_TXN_ERROR_BUSY);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 0xFFFFFFF5u) == 0);
    CHECK(mbrtum_transaction_poll(&txn, 0x00000058u) == 0);
    CHECK(txn.state == MBRTUM_TXN_STATE_WAITING_RESPONSE);
    CHECK(mbrtum_transaction_poll(&txn, 0x00000059u) == 0);
    CHECK(txn.result == MBRTUM_TXN_RESULT_TIMEOUT);

    CHECK(mbrtum_transaction_start(&txn, &request, adu, length, 10u) == 0);
    CHECK(mbrtum_transaction_cancel(&txn) == 0);
    CHECK(txn.result == MBRTUM_TXN_RESULT_CANCELLED);
    CHECK(mbrtum_transaction_on_tx_complete(&txn, 11u) == MBRTUM_TXN_ERROR_STATE);
    return 0;
}

int main(void)
{
    CHECK(test_initialization_and_config_guards() == 0);
    CHECK(test_request_adu_validation() == 0);
    CHECK(test_success_and_immutable_copy() == 0);
    CHECK(test_timeout_retry_then_success() == 0);
    CHECK(test_zero_delay_retry_is_poll_driven() == 0);
    CHECK(test_retry_exhaustion() == 0);
    CHECK(test_malformed_then_response_error() == 0);
    CHECK(test_late_response_is_not_accepted() == 0);
    CHECK(test_unrelated_ignored() == 0);
    CHECK(test_exception_completes_without_retry() == 0);
    CHECK(test_broadcast_and_transport_errors() == 0);
    CHECK(test_cancel_busy_state_and_wrap() == 0);
    puts("modbus RTU master transaction tests: PASS");
    return 0;
}
