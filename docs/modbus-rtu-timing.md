# Portable Modbus RTU byte and timing layer

This layer turns single-byte UART receive events and a fixed 50 microsecond timer tick into complete Modbus RTU frames. It is portable C11 and contains no STM32 HAL, DMA, UART idle-line, RTOS, or transceiver dependency.

<p align="center">
  <img src="images/rtu-timing.svg" alt="Modbus RTU T1.5 and T3.5 receive timing" width="96%">
</p>

## Public API

```c
int mbrtu_init(mbrtu_context_t *ctx, const mbrtu_config_t *config);
void mbrtu_on_rx_byte_isr(mbrtu_context_t *ctx, uint8_t byte);
void mbrtu_on_50us_tick_isr(mbrtu_context_t *ctx);
int mbrtu_poll(mbrtu_context_t *ctx);
```

`mbrtu_on_rx_byte_isr()` and `mbrtu_on_50us_tick_isr()` are constant-time entry points intended for interrupt context. They do not calculate CRC, execute function codes, call application hooks, or transmit.

The receive callback is defined as a **byte-complete** event: call it after the UART has received the complete character, including the stop bit. The timing calculation includes one additional character duration when comparing successive byte-complete callbacks. This avoids mistaking a legal silent interval for an illegal gap merely because the newly received character itself took time on the wire.

`mbrtu_poll()` is called from the bare-metal main loop. It validates a published frame through `mbrtu_process_adu()` and invokes the configured transmit callback when a response is ready.

## Static storage

The application supplies:

- two non-overlapping receive buffers
- one non-overlapping response buffer
- a synchronous transmit callback
- serial format and slave-address configuration

No dynamic allocation is used. Two receive buffers allow the ISR to begin receiving a new frame while the main loop processes the previously completed frame.

A second frame that completes before the pending frame is consumed is dropped and counted as an overrun. This is deliberate and explicit; unread data is never silently overwritten. A later queue extension can add more frame slots without changing the complete-frame ADU API.

## Timing calculation

The timing source is fixed at:

```text
50 microseconds per tick
20,000 ticks per second
```

For baud rates at or below 19200, the complete character length is:

```text
1 start bit + data bits + optional parity bit + stop bits
```

T1.5 and T3.5 are converted directly to 50 microsecond ticks with ceiling division. The interval therefore never expires earlier than required.

Because ordinary UART receive interrupts report a byte only after the character completes, the implementation also calculates two byte-complete deadlines:

```text
byte-complete T1.5 deadline = T1.5 silence + one character time
byte-complete T3.5 deadline = T3.5 silence + one character time
```

The state machine uses those adjusted deadlines for byte arrival classification and frame publication. This adds at most one character time of latency after the final byte but preserves the required serial-line silence semantics without requiring start-bit detection or UART idle-line hardware.

For baud rates above 19200, the recommended fixed values are used:

```text
T1.5 = 750 microseconds  = 15 ticks
T3.5 = 1750 microseconds = 35 ticks
```

Examples for 8E1:

| Baud | Character ticks | T1.5 ticks | T3.5 ticks | Byte-complete T1.5 | Byte-complete T3.5 |
|---:|---:|---:|---:|---:|---:|
| 9600 | 23 | 35 | 81 | 58 | 104 |
| 19200 | 12 | 18 | 41 | 29 | 52 |
| 115200 | 2 | 15 | 35 | 17 | 37 |

## State behavior

1. The first byte-complete event starts an active frame and resets the gap counter.
2. Back-to-back characters produce callback spacing of approximately one character time.
3. A byte completing at or before the adjusted T1.5 deadline remains part of the frame.
4. A byte completing after the adjusted T1.5 deadline but before the adjusted T3.5 deadline marks the whole active frame invalid.
5. Reaching the adjusted T3.5 deadline publishes the active buffer as one completed frame.
6. The alternate receive buffer becomes active immediately.
7. Invalid-gap and overflow frames are discarded by `mbrtu_poll()` without protocol processing.

The gap counter never wraps: it is bounded by T3.5 and reset when the frame boundary is published.

## Example configuration

```c
#include "modbus.h"
#include "modbus_rtu.h"

static mbrtu_context_t rtu;
static uint8_t rtu_rx_a[MODBUS_RTU_ADU_MAX_SIZE];
static uint8_t rtu_rx_b[MODBUS_RTU_ADU_MAX_SIZE];
static uint8_t rtu_response[MODBUS_RTU_ADU_MAX_SIZE];

static int uart_transmit(void *user_context,
                         const uint8_t *data,
                         size_t length)
{
    (void)user_context;
    return board_uart_transmit(data, length); /* Return 0 on success. */
}

void app_init(void)
{
    const mbrtu_config_t config = {
        .slave_address = 1u,
        .baud_rate = 19200u,
        .data_bits = 8u,
        .parity_bits = 1u,
        .stop_bits = 1u,
        .receive_buffer_a = rtu_rx_a,
        .receive_buffer_b = rtu_rx_b,
        .receive_buffer_capacity = sizeof(rtu_rx_a),
        .response_buffer = rtu_response,
        .response_buffer_capacity = sizeof(rtu_response),
        .transmit = uart_transmit,
        .user_context = NULL
    };

    mb_init();
    (void)mbrtu_init(&rtu, &config);
}
```

UART receive ISR/callback:

```c
mbrtu_on_rx_byte_isr(&rtu, received_byte);
```

50 microsecond timer ISR:

```c
mbrtu_on_50us_tick_isr(&rtu);
```

Bare-metal superloop:

```c
for (;;) {
    (void)mbrtu_poll(&rtu);
    /* Other application and network polling. */
}
```

The transmit callback must either complete transmission synchronously or copy the response into independently owned transmit storage before returning. It must not retain the response-buffer pointer for later interrupt- or DMA-driven transmission because the next `mbrtu_poll()` call may reuse that buffer.

## Interrupt-priority requirement

The UART receive-complete callback and the 50 microsecond tick callback both modify the same context. The board adapter must serialize them. Configure both interrupt sources at the same preemption priority, or protect the two calls with a very small critical section. Do not allow one callback to preempt the other while it is updating RTU state.

The UART callback should run before the timer callback when both become pending at the same instant. This preserves the intended boundary behavior for a character completing exactly at an adjusted deadline.

## STM32 time-base note

The portable layer does not configure SysTick or an STM32 timer. If a CubeMX reference uses a 50 microsecond SysTick, preserve the HAL millisecond time base by calling `HAL_IncTick()` every 20 interrupts. The final board adapter must also preserve any generated SysTick callback behavior and verify HAL timeouts, delays, Ethernet, and lwIP operation.

A dedicated hardware timer can call the same `mbrtu_on_50us_tick_isr()` API if changing SysTick is undesirable.

## Diagnostics

`mbrtu_get_stats()` provides a best-effort snapshot of:

- received bytes
- detected frame boundaries
- queued frames
- invalid-gap events
- buffer overflows
- pending-frame overruns
- no-response frames
- responses sent
- processing and transmit errors

The snapshot is intended for diagnostics; values can change while interrupts remain enabled.

## Stage boundary

Implemented now:

- single-byte receive ISR API
- fixed 50 microsecond tick API
- configurable serial-character timing
- correct T1.5/T3.5 state handling
- two-buffer ownership handoff
- overflow, invalid-gap, and overrun recovery
- main-loop ADU processing and transmit callback
- deterministic host tests with a fake tick source

Still future work:

- STM32CubeMX/CubeIDE reference project
- actual UART interrupt re-arming
- 50 microsecond SysTick or hardware-timer adapter
- HAL 1 ms divider integration
- physical full-duplex UART testing
- RS-485 DE/RE control and transmission-complete handling
- optional multi-frame descriptor queue
