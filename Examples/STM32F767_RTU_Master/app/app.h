#ifndef APP_H_
#define APP_H_

#include "main.h"
#include "usart.h"

#include "modbus_rtu.h"
#include "modbus_rtu_master.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODBUS_MASTER_STATE_NOT_INITIALIZED = 0,
    MODBUS_MASTER_STATE_IDLE = 1,
    MODBUS_MASTER_STATE_WAITING_RESPONSE = 2
} modbus_master_state_t;

typedef enum {
    MODBUS_MASTER_RESULT_NONE = 0,
    MODBUS_MASTER_RESULT_OK = 1,
    MODBUS_MASTER_RESULT_EXCEPTION = 2,
    MODBUS_MASTER_RESULT_BUILD_ERROR = 3,
    MODBUS_MASTER_RESULT_TRANSMIT_ERROR = 4,
    MODBUS_MASTER_RESULT_TIMEOUT = 5,
    MODBUS_MASTER_RESULT_RESPONSE_ERROR = 6,
    MODBUS_MASTER_RESULT_RX_INVALID_GAP = 7,
    MODBUS_MASTER_RESULT_RX_OVERFLOW = 8,
    MODBUS_MASTER_RESULT_UART_ERROR = 9
} modbus_master_result_t;

/* Keil Watch-friendly master diagnostics. */
extern volatile uint32_t systick_count;
extern volatile modbus_master_state_t modbus_master_state;
extern volatile modbus_master_result_t modbus_master_last_result;
extern volatile int modbus_master_last_protocol_result;
extern volatile uint8_t modbus_master_last_exception_code;
extern volatile uint16_t modbus_master_last_response_length;
extern volatile uint8_t modbus_master_auto_poll_enabled;

extern volatile uint32_t modbus_master_requests_sent;
extern volatile uint32_t modbus_master_valid_responses;
extern volatile uint32_t modbus_master_exception_responses;
extern volatile uint32_t modbus_master_timeouts;
extern volatile uint32_t modbus_master_transmit_errors;
extern volatile uint32_t modbus_master_response_errors;
extern volatile uint32_t modbus_master_received_bytes;
extern volatile uint32_t modbus_master_detected_frames;
extern volatile uint32_t modbus_master_invalid_gap_frames;
extern volatile uint32_t modbus_master_overflow_frames;
extern volatile uint32_t modbus_master_overrun_frames;
extern volatile uint32_t modbus_master_unexpected_bytes;
extern volatile uint32_t modbus_uart_error_count;
extern volatile uint32_t modbus_uart_recovery_count;

/* FC03 demo result: holding registers 0..4 from slave address 1. */
extern uint16_t modbus_master_holding_registers[5];

void app(void);
int modbus_master_init(void);
void modbus_master_process(void);
void modbus_master_request_now(void);
void systick_50us_tick_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
