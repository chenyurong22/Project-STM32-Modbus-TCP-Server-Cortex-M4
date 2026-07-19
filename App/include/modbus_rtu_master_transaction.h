#ifndef MODBUS_RTU_MASTER_TRANSACTION_H
#define MODBUS_RTU_MASTER_TRANSACTION_H

#include "modbus_rtu_master.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MBRTUM_TXN_TRANSMIT_ACCEPTED 0

#define MBRTUM_TXN_OK 0
#define MBRTUM_TXN_ERROR_ARGUMENT (-1)
#define MBRTUM_TXN_ERROR_BUSY (-2)
#define MBRTUM_TXN_ERROR_STATE (-3)
#define MBRTUM_TXN_ERROR_CAPACITY (-4)
#define MBRTUM_TXN_ERROR_CONFIG (-5)
#define MBRTUM_TXN_ERROR_NOT_INITIALIZED (-6)
#define MBRTUM_TXN_ERROR_REQUEST (-7)

#define MBRTUM_TXN_PROTOCOL_NONE 0

typedef enum {
    MBRTUM_TXN_STATE_IDLE = 0,
    MBRTUM_TXN_STATE_TRANSMITTING,
    MBRTUM_TXN_STATE_WAITING_RESPONSE,
    MBRTUM_TXN_STATE_RETRY_DELAY,
    MBRTUM_TXN_STATE_COMPLETE,
    MBRTUM_TXN_STATE_FAILED,
    MBRTUM_TXN_STATE_CANCELLED
} mbrtum_transaction_state_t;

typedef enum {
    MBRTUM_TXN_RESULT_NONE = 0,
    MBRTUM_TXN_RESULT_SUCCESS,
    MBRTUM_TXN_RESULT_MODBUS_EXCEPTION,
    MBRTUM_TXN_RESULT_TIMEOUT,
    MBRTUM_TXN_RESULT_TRANSPORT_ERROR,
    MBRTUM_TXN_RESULT_RESPONSE_ERROR,
    MBRTUM_TXN_RESULT_CANCELLED
} mbrtum_transaction_result_t;

typedef int (*mbrtum_transaction_transmit_fn)(void *context,
                                              const uint8_t *adu,
                                              size_t adu_length);

typedef struct {
    uint32_t response_timeout_ticks;
    uint32_t retry_delay_ticks;
    uint16_t max_retries;
    uint8_t retry_transport_errors;
} mbrtum_transaction_config_t;

typedef struct {
    uint32_t transactions_started;
    uint32_t successful_transactions;
    uint32_t exception_responses;
    uint32_t transmission_attempts;
    uint32_t retries_performed;
    uint32_t timeouts;
    uint32_t transport_errors;
    uint32_t malformed_responses;
    uint32_t unrelated_responses;
    uint32_t late_responses;
    uint32_t cancelled_transactions;
} mbrtum_transaction_diagnostics_t;

typedef struct {
    mbrtum_transaction_state_t state;
    mbrtum_transaction_result_t result;
    int protocol_result;
    uint8_t exception_code;

    mbrtum_transaction_config_t config;
    mbrtum_transaction_transmit_fn transmit;
    void *transport_context;

    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_adu_length;

    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t response_adu_length;
    mbrtum_response_t response;

    uint32_t deadline;
    uint32_t initialization_cookie;
    uint16_t retries_used;
    uint8_t saw_response_error;

    mbrtum_transaction_diagnostics_t diagnostics;
} mbrtum_transaction_t;

int mbrtum_transaction_init(mbrtum_transaction_t *transaction,
                            const mbrtum_transaction_config_t *config,
                            mbrtum_transaction_transmit_fn transmit,
                            void *transport_context);

int mbrtum_transaction_start(mbrtum_transaction_t *transaction,
                             const mbrtum_request_t *request,
                             const uint8_t *request_adu,
                             size_t request_adu_length,
                             uint32_t now);

int mbrtum_transaction_poll(mbrtum_transaction_t *transaction,
                            uint32_t now);

int mbrtum_transaction_on_tx_complete(mbrtum_transaction_t *transaction,
                                      uint32_t now);

int mbrtum_transaction_on_tx_error(mbrtum_transaction_t *transaction,
                                   uint32_t now);

int mbrtum_transaction_on_response(mbrtum_transaction_t *transaction,
                                   const uint8_t *response_adu,
                                   size_t response_adu_length,
                                   uint32_t now);

int mbrtum_transaction_cancel(mbrtum_transaction_t *transaction);

int mbrtum_transaction_is_active(const mbrtum_transaction_t *transaction);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_MASTER_TRANSACTION_H */
