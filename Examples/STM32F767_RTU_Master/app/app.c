#include "app.h"

#include <stddef.h>
#include <string.h>

/*
 * STM32F767 Modbus RTU master hardware-validation harness.
 *
 * This file intentionally keeps the portable complete-frame master core
 * unchanged. It supplies only the board-specific test glue needed to:
 *   - build one immutable FC03 request;
 *   - transmit it on USART3;
 *   - collect one bounded RTU response using 50 us timing;
 *   - validate the complete response with mbrtum_process_response();
 *   - expose deterministic diagnostics in Keil Watch.
 *
 * It is a hardware smoke-test adapter, not the future portable transaction
 * engine. There is no retry policy and no RS-485 DE/RE control in this board.
 */

#define MODBUS_MASTER_SLAVE_ADDRESS             1u
#define MODBUS_MASTER_BAUD_RATE                 115200u
#define MODBUS_MASTER_DATA_BITS                 8u
#define MODBUS_MASTER_PARITY_BITS               0u
#define MODBUS_MASTER_STOP_BITS                 1u
#define MODBUS_MASTER_START_ADDRESS             0u
#define MODBUS_MASTER_REGISTER_COUNT            5u
#define MODBUS_MASTER_RESPONSE_TIMEOUT_MS       1000u
#define MODBUS_MASTER_REQUEST_PERIOD_MS         1000u
#define MODBUS_MASTER_FIRST_REQUEST_DELAY_MS    500u
#define MODBUS_MASTER_UART_TIMEOUT_MS           100u

#define MODBUS_MASTER_RX_BUFFER_CAPACITY MODBUS_RTU_ADU_MAX_SIZE

#if MODBUS_MASTER_REGISTER_COUNT != 5u
#error "Update modbus_master_holding_registers when changing the demo quantity"
#endif

typedef struct {
    uint8_t buffers[2][MODBUS_MASTER_RX_BUFFER_CAPACITY];
    mbrtu_timing_t timing;

    volatile uint32_t gap_ticks;
    volatile uint16_t active_length;
    volatile uint16_t pending_length;
    volatile uint8_t active_buffer_index;
    volatile uint8_t pending_buffer_index;
    volatile uint8_t frame_active;
    volatile uint8_t active_invalid_gap;
    volatile uint8_t active_overflow;
    volatile uint8_t pending_ready;
    volatile uint8_t pending_invalid_gap;
    volatile uint8_t pending_overflow;
} modbus_master_rx_t;

static modbus_master_rx_t modbus_master_rx;
static mbrtum_request_t modbus_master_request;
static uint8_t modbus_master_request_adu[MODBUS_RTU_ADU_MAX_SIZE];
static size_t modbus_master_request_adu_length;
static uint8_t modbus_master_uart_rx_byte;
static uint32_t modbus_master_response_deadline_ms;
static uint32_t modbus_master_next_request_ms;
static volatile uint8_t modbus_master_initialized;
static volatile uint8_t modbus_master_force_request;
static volatile uint8_t modbus_master_uart_error_pending;

volatile uint32_t systick_count = 0u;
volatile modbus_master_state_t modbus_master_state =
    MODBUS_MASTER_STATE_NOT_INITIALIZED;
volatile modbus_master_result_t modbus_master_last_result =
    MODBUS_MASTER_RESULT_NONE;
volatile int modbus_master_last_protocol_result = MBRTUM_OK;
volatile uint8_t modbus_master_last_exception_code = 0u;
volatile uint16_t modbus_master_last_response_length = 0u;
volatile uint8_t modbus_master_auto_poll_enabled = 1u;

volatile uint32_t modbus_master_requests_sent = 0u;
volatile uint32_t modbus_master_valid_responses = 0u;
volatile uint32_t modbus_master_exception_responses = 0u;
volatile uint32_t modbus_master_timeouts = 0u;
volatile uint32_t modbus_master_transmit_errors = 0u;
volatile uint32_t modbus_master_response_errors = 0u;
volatile uint32_t modbus_master_received_bytes = 0u;
volatile uint32_t modbus_master_detected_frames = 0u;
volatile uint32_t modbus_master_invalid_gap_frames = 0u;
volatile uint32_t modbus_master_overflow_frames = 0u;
volatile uint32_t modbus_master_overrun_frames = 0u;
volatile uint32_t modbus_master_unexpected_bytes = 0u;
volatile uint32_t modbus_uart_error_count = 0u;
volatile uint32_t modbus_uart_recovery_count = 0u;

uint16_t modbus_master_holding_registers[MODBUS_MASTER_REGISTER_COUNT];

static int modbus_master_time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static uint32_t modbus_master_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void modbus_master_exit_critical(uint32_t primask)
{
    if ((primask & 1u) == 0u) {
        __enable_irq();
    }
}

static void modbus_master_rx_reset_active_isr(void)
{
    modbus_master_rx.gap_ticks = 0u;
    modbus_master_rx.active_length = 0u;
    modbus_master_rx.frame_active = 0u;
    modbus_master_rx.active_invalid_gap = 0u;
    modbus_master_rx.active_overflow = 0u;
}

static void modbus_master_rx_reset_all(void)
{
    uint32_t primask = modbus_master_enter_critical();

    modbus_master_rx.active_buffer_index = 0u;
    modbus_master_rx.pending_buffer_index = 0u;
    modbus_master_rx.pending_length = 0u;
    modbus_master_rx.pending_ready = 0u;
    modbus_master_rx.pending_invalid_gap = 0u;
    modbus_master_rx.pending_overflow = 0u;
    modbus_master_rx_reset_active_isr();

    modbus_master_exit_critical(primask);
}

static void modbus_master_rx_complete_frame_isr(void)
{
    if (modbus_master_rx.frame_active == 0u) {
        return;
    }

    ++modbus_master_detected_frames;

    if (modbus_master_rx.pending_ready == 0u) {
        /* Publish metadata first and the ready flag last. */
        modbus_master_rx.pending_buffer_index =
            modbus_master_rx.active_buffer_index;
        modbus_master_rx.pending_length = modbus_master_rx.active_length;
        modbus_master_rx.pending_invalid_gap =
            modbus_master_rx.active_invalid_gap;
        modbus_master_rx.pending_overflow = modbus_master_rx.active_overflow;
        modbus_master_rx.active_buffer_index ^= 1u;
        modbus_master_rx.pending_ready = 1u;
    } else {
        ++modbus_master_overrun_frames;
    }

    modbus_master_rx_reset_active_isr();
}

static void modbus_master_on_rx_byte_isr(uint8_t byte)
{
    uint8_t *active_buffer;

    if (modbus_master_initialized == 0u ||
        modbus_master_state != MODBUS_MASTER_STATE_WAITING_RESPONSE) {
        ++modbus_master_unexpected_bytes;
        return;
    }

    ++modbus_master_received_bytes;

    if (modbus_master_rx.frame_active != 0u) {
        if (modbus_master_rx.gap_ticks >=
            modbus_master_rx.timing.byte_complete_t3_5_ticks) {
            /* This byte belongs to a new frame after a valid T3.5 boundary. */
            modbus_master_rx_complete_frame_isr();
        } else if (modbus_master_rx.gap_ticks >
                       modbus_master_rx.timing.byte_complete_t1_5_ticks &&
                   modbus_master_rx.active_invalid_gap == 0u) {
            /* A gap longer than T1.5 but shorter than T3.5 invalidates the ADU. */
            modbus_master_rx.active_invalid_gap = 1u;
            ++modbus_master_invalid_gap_frames;
        }
    }

    modbus_master_rx.gap_ticks = 0u;
    modbus_master_rx.frame_active = 1u;

    if (modbus_master_rx.active_invalid_gap != 0u ||
        modbus_master_rx.active_overflow != 0u) {
        return;
    }

    if (modbus_master_rx.active_length >= MODBUS_MASTER_RX_BUFFER_CAPACITY) {
        modbus_master_rx.active_overflow = 1u;
        ++modbus_master_overflow_frames;
        return;
    }

    active_buffer =
        modbus_master_rx.buffers[modbus_master_rx.active_buffer_index];
    active_buffer[modbus_master_rx.active_length] = byte;
    ++modbus_master_rx.active_length;
}

static void modbus_master_on_50us_tick_isr(void)
{
    if (modbus_master_initialized == 0u ||
        modbus_master_state != MODBUS_MASTER_STATE_WAITING_RESPONSE ||
        modbus_master_rx.frame_active == 0u) {
        return;
    }

    if (modbus_master_rx.gap_ticks < modbus_master_rx.timing.t3_5_ticks) {
        ++modbus_master_rx.gap_ticks;
    }

    if (modbus_master_rx.gap_ticks >= modbus_master_rx.timing.t3_5_ticks) {
        modbus_master_rx_complete_frame_isr();
    }
}

static void modbus_master_finish_transaction(modbus_master_result_t result)
{
    modbus_master_state = MODBUS_MASTER_STATE_IDLE;
    modbus_master_last_result = result;
    modbus_master_next_request_ms =
        HAL_GetTick() + MODBUS_MASTER_REQUEST_PERIOD_MS;
    modbus_master_rx_reset_all();
}

static void modbus_master_ensure_receive_armed(void)
{
    if (huart3.RxState == HAL_UART_STATE_READY) {
        if (HAL_UART_Receive_IT(&huart3, &modbus_master_uart_rx_byte, 1u) ==
            HAL_OK) {
            ++modbus_uart_recovery_count;
        } else {
            ++modbus_uart_error_count;
        }
    }
}

static void modbus_master_start_request(void)
{
    HAL_StatusTypeDef transmit_result;
    uint32_t now;

    if (modbus_master_initialized == 0u ||
        modbus_master_state != MODBUS_MASTER_STATE_IDLE) {
        return;
    }

    modbus_master_rx_reset_all();
    modbus_master_last_result = MODBUS_MASTER_RESULT_NONE;
    modbus_master_last_protocol_result = MBRTUM_OK;
    modbus_master_last_exception_code = 0u;
    modbus_master_last_response_length = 0u;

    now = HAL_GetTick();
    modbus_master_response_deadline_ms =
        now + MODBUS_MASTER_RESPONSE_TIMEOUT_MS;
    modbus_master_state = MODBUS_MASTER_STATE_WAITING_RESPONSE;

    transmit_result = HAL_UART_Transmit(
        &huart3,
        modbus_master_request_adu,
        (uint16_t)modbus_master_request_adu_length,
        MODBUS_MASTER_UART_TIMEOUT_MS);

    if (transmit_result != HAL_OK) {
        ++modbus_master_transmit_errors;
        modbus_master_finish_transaction(MODBUS_MASTER_RESULT_TRANSMIT_ERROR);
        return;
    }

    ++modbus_master_requests_sent;
}

static void modbus_master_process_response_frame(void)
{
    const uint8_t *response_adu;
    uint16_t response_length;
    uint8_t invalid_gap;
    uint8_t overflow;
    mbrtum_response_t response;
    uint16_t register_index;
    int process_result;

    if (modbus_master_rx.pending_ready == 0u) {
        return;
    }

    response_adu =
        modbus_master_rx.buffers[modbus_master_rx.pending_buffer_index];
    response_length = modbus_master_rx.pending_length;
    invalid_gap = modbus_master_rx.pending_invalid_gap;
    overflow = modbus_master_rx.pending_overflow;
    modbus_master_last_response_length = response_length;

    if (modbus_master_state != MODBUS_MASTER_STATE_WAITING_RESPONSE) {
        modbus_master_rx.pending_ready = 0u;
        modbus_master_rx_reset_all();
        return;
    }

    if (overflow != 0u) {
        ++modbus_master_response_errors;
        modbus_master_finish_transaction(MODBUS_MASTER_RESULT_RX_OVERFLOW);
        return;
    }

    if (invalid_gap != 0u) {
        ++modbus_master_response_errors;
        modbus_master_finish_transaction(MODBUS_MASTER_RESULT_RX_INVALID_GAP);
        return;
    }

    process_result = mbrtum_process_response(
        &modbus_master_request,
        response_adu,
        (size_t)response_length,
        &response);
    modbus_master_last_protocol_result = process_result;

    if (process_result == MBRTUM_OK) {
        for (register_index = 0u;
             register_index < MODBUS_MASTER_REGISTER_COUNT;
             ++register_index) {
            int decode_result = mbrtum_get_register(
                &modbus_master_request,
                &response,
                register_index,
                &modbus_master_holding_registers[register_index]);

            if (decode_result != MBRTUM_OK) {
                modbus_master_last_protocol_result = decode_result;
                ++modbus_master_response_errors;
                modbus_master_finish_transaction(
                    MODBUS_MASTER_RESULT_RESPONSE_ERROR);
                return;
            }
        }

        ++modbus_master_valid_responses;
        modbus_master_finish_transaction(MODBUS_MASTER_RESULT_OK);
        return;
    }

    if (process_result == MBRTUM_EXCEPTION_RESPONSE) {
        modbus_master_last_exception_code = response.exception_code;
        ++modbus_master_exception_responses;
        modbus_master_finish_transaction(MODBUS_MASTER_RESULT_EXCEPTION);
        return;
    }

    ++modbus_master_response_errors;
    modbus_master_finish_transaction(MODBUS_MASTER_RESULT_RESPONSE_ERROR);
}

int modbus_master_init(void)
{
    int timing_result;
    int build_result;

    memset(&modbus_master_rx, 0, sizeof(modbus_master_rx));
    memset(&modbus_master_request, 0, sizeof(modbus_master_request));
    memset(modbus_master_request_adu, 0, sizeof(modbus_master_request_adu));
    memset(modbus_master_holding_registers,
           0,
           sizeof(modbus_master_holding_registers));

    timing_result = mbrtu_calculate_timing(
        MODBUS_MASTER_BAUD_RATE,
        MODBUS_MASTER_DATA_BITS,
        MODBUS_MASTER_PARITY_BITS,
        MODBUS_MASTER_STOP_BITS,
        &modbus_master_rx.timing);
    if (timing_result != 0) {
        modbus_master_last_protocol_result = timing_result;
        modbus_master_last_result = MODBUS_MASTER_RESULT_BUILD_ERROR;
        return timing_result;
    }

    build_result = mbrtum_build_read_registers_request(
        MODBUS_MASTER_SLAVE_ADDRESS,
        MBRTUM_FC_READ_HOLDING_REGISTERS,
        MODBUS_MASTER_START_ADDRESS,
        MODBUS_MASTER_REGISTER_COUNT,
        &modbus_master_request,
        modbus_master_request_adu,
        sizeof(modbus_master_request_adu),
        &modbus_master_request_adu_length);
    if (build_result != MBRTUM_OK) {
        modbus_master_last_protocol_result = build_result;
        modbus_master_last_result = MODBUS_MASTER_RESULT_BUILD_ERROR;
        return build_result;
    }

    if (modbus_master_request_adu_length == 0u ||
        modbus_master_request_adu_length > (size_t)UINT16_MAX) {
        modbus_master_last_protocol_result = MBRTUM_ERROR_CAPACITY;
        modbus_master_last_result = MODBUS_MASTER_RESULT_BUILD_ERROR;
        return MBRTUM_ERROR_CAPACITY;
    }

    modbus_master_rx_reset_all();
    modbus_master_uart_error_pending = 0u;
    modbus_master_force_request = 0u;
    modbus_master_state = MODBUS_MASTER_STATE_IDLE;
    modbus_master_last_result = MODBUS_MASTER_RESULT_NONE;
    modbus_master_last_protocol_result = MBRTUM_OK;
    modbus_master_initialized = 1u;
    modbus_master_next_request_ms =
        HAL_GetTick() + MODBUS_MASTER_FIRST_REQUEST_DELAY_MS;

    if (HAL_UART_Receive_IT(&huart3, &modbus_master_uart_rx_byte, 1u) !=
        HAL_OK) {
        ++modbus_uart_error_count;
        modbus_master_initialized = 0u;
        modbus_master_state = MODBUS_MASTER_STATE_NOT_INITIALIZED;
        modbus_master_last_result = MODBUS_MASTER_RESULT_UART_ERROR;
        return -1;
    }

    return 0;
}

void modbus_master_request_now(void)
{
    modbus_master_force_request = 1u;
}

void modbus_master_process(void)
{
    uint32_t now;

    if (modbus_master_initialized == 0u) {
        return;
    }

    modbus_master_ensure_receive_armed();

    if (modbus_master_uart_error_pending != 0u) {
        uint32_t primask = modbus_master_enter_critical();
        modbus_master_uart_error_pending = 0u;
        modbus_master_exit_critical(primask);

        if (modbus_master_state == MODBUS_MASTER_STATE_WAITING_RESPONSE) {
            ++modbus_master_response_errors;
            modbus_master_finish_transaction(MODBUS_MASTER_RESULT_UART_ERROR);
        }
    }

    /* A completed response always wins over a timeout on the same loop pass. */
    modbus_master_process_response_frame();

    now = HAL_GetTick();
    if (modbus_master_state == MODBUS_MASTER_STATE_WAITING_RESPONSE) {
        if (modbus_master_time_reached(
                now,
                modbus_master_response_deadline_ms)) {
            ++modbus_master_timeouts;
            modbus_master_finish_transaction(MODBUS_MASTER_RESULT_TIMEOUT);
        }
        return;
    }

    if (modbus_master_force_request != 0u) {
        modbus_master_force_request = 0u;
        modbus_master_start_request();
        return;
    }

    if (modbus_master_auto_poll_enabled != 0u &&
        modbus_master_time_reached(now, modbus_master_next_request_ms)) {
        modbus_master_start_request();
    }
}

void app(void)
{
    int init_result;
    uint32_t last_reported_request_count = 0u;
    uint32_t last_reported_completion_count = 0u;

    printf("Modbus RTU master smoke test for STM32F767\r\n");
    printf("USART3: 115200 8N1, full-duplex TTL UART\r\n");
    printf("Request: slave 1, FC03, start 0, quantity 5\r\n");

    init_result = modbus_master_init();
    if (init_result != 0) {
        printf("Modbus master initialization failed: %d\r\n", init_result);
        Error_Handler();
    }

    printf("Modbus master initialized; first request in 500 ms\r\n");

    while (1) {
        uint32_t completion_count;

        modbus_master_process();

        if (modbus_master_requests_sent != last_reported_request_count) {
            last_reported_request_count = modbus_master_requests_sent;
            printf("Master request sent: %lu\r\n",
                   (unsigned long)last_reported_request_count);
        }

        completion_count = modbus_master_valid_responses +
                           modbus_master_exception_responses +
                           modbus_master_timeouts +
                           modbus_master_transmit_errors +
                           modbus_master_response_errors;

        if (completion_count != last_reported_completion_count) {
            last_reported_completion_count = completion_count;

            if (modbus_master_last_result == MODBUS_MASTER_RESULT_OK) {
                printf("Valid FC03 response: %u, %u, %u, %u, %u\r\n",
                       (unsigned int)modbus_master_holding_registers[0],
                       (unsigned int)modbus_master_holding_registers[1],
                       (unsigned int)modbus_master_holding_registers[2],
                       (unsigned int)modbus_master_holding_registers[3],
                       (unsigned int)modbus_master_holding_registers[4]);
            } else if (modbus_master_last_result ==
                       MODBUS_MASTER_RESULT_EXCEPTION) {
                printf("Modbus exception response: 0x%02X\r\n",
                       (unsigned int)modbus_master_last_exception_code);
            } else {
                printf("Master transaction failed: result=%d protocol=%d len=%u\r\n",
                       (int)modbus_master_last_result,
                       modbus_master_last_protocol_result,
                       (unsigned int)modbus_master_last_response_length);
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART3) {
        modbus_master_on_rx_byte_isr(modbus_master_uart_rx_byte);

        if (HAL_UART_Receive_IT(&huart3,
                                &modbus_master_uart_rx_byte,
                                1u) != HAL_OK) {
            ++modbus_uart_error_count;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART3) {
        ++modbus_uart_error_count;
        modbus_master_uart_error_pending = 1u;
    }
}

void systick_50us_tick_isr(void)
{
    modbus_master_on_50us_tick_isr();
}
