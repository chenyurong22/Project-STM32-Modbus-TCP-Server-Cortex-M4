# STM32F767 Modbus RTU slave hardware validation

## Purpose

This document records the external physical-hardware validation of the
portable Modbus RTU slave implementation contained in this repository.

The validation was performed by integrating the portable RTU library into an
STM32CubeMX and Keil MDK-ARM project targeting an STM32F767IGTx-based board.

The hardware-specific integration remained separate from the portable RTU
protocol and timing core.

## Validation environment

The reported hardware configuration was:

- MCU: STM32F767IGTx
- CPU core: Arm Cortex-M7
- System clock: 216 MHz
- Development configuration: STM32CubeMX
- Target toolchain: Keil MDK-ARM with ArmClang
- Modbus serial peripheral: USART3
- Debug serial peripheral: USART1
- Modbus baud rate: 115200 baud
- Data bits: 8
- Parity: none
- Stop bits: 1
- Serial format: 115200 8N1
- Electrical interface: 3.3 V TTL UART
- Modbus slave address: 1
- Receive method: single-byte interrupt reception
- RTU timing source: fixed 50 microsecond SysTick callback
- HAL time base: 1 millisecond using a 20:1 divider from the 50 microsecond
  SysTick interrupt
- Test application: Modbus Poll on Windows
- Scan interval used during stress testing: 1000 milliseconds

No RS-485 driver-enable or receiver-enable control was required because the
test connection used full-duplex TTL UART.

## STM32 integration

The external STM32F767 project integrated the following portable APIs:

```c
mbrtu_init()
mbrtu_poll()
mbrtu_on_rx_byte_isr()
mbrtu_on_50us_tick_isr()
mbrtu_get_stats()
```

The integration included:

- one `mbrtu_context_t` instance;
- two RTU receive buffers;
- one RTU response buffer;
- an `mbrtu_config_t` configuration;
- USART3 blocking response transmission;
- single-byte interrupt reception and re-arming;
- RTU polling from the main application loop;
- a 50 microsecond RTU timing callback;
- preservation of the STM32 HAL 1 millisecond time base;
- UART diagnostic and recovery counters;
- RTU diagnostic statistics.

The RTU configuration used:

```text
Slave address: 1
Baud rate:     115200
Data bits:     8
Parity bits:   0
Stop bits:     1
```

In the portable API, `parity_bits` represents the number of parity bits in one
serial character:

- `0` means no parity bit;
- `1` means one parity bit.

The selection of odd or even parity remains the responsibility of the
hardware UART configuration.

## Validated function codes

The following Modbus RTU slave function codes were tested successfully on
physical STM32F767 hardware:

- FC01 — Read Coils
- FC02 — Read Discrete Inputs
- FC03 — Read Holding Registers
- FC04 — Read Input Registers
- FC05 — Write Single Coil
- FC06 — Write Single Holding Register
- FC0F — Write Multiple Coils
- FC10 — Write Multiple Holding Registers

In Modbus Poll, FC0F and FC10 may be displayed as decimal function numbers 15
and 16.

The tests demonstrated:

- correct slave-address handling;
- correct CRC-16 validation;
- correct request decoding;
- correct shared PDU processing;
- correct coil and register access;
- correct exception and response framing;
- correct UART response transmission;
- correct repeated request processing.

## Initial test data

The application initialized example Modbus data for physical testing.

### Coils

The first five coils were initialized to `1`.

### Discrete inputs

The first five discrete inputs were initialized to `1`.

### Holding registers

The first five holding registers were initialized to:

```text
1
2
3
4
5
```

### Input registers

The first five input registers were initialized to:

```text
10
20
30
40
50
```

These values were read successfully through Modbus Poll.

Holding-register and coil writes were also confirmed through subsequent read
operations.

## Multiple-register validation

FC10, Write Multiple Holding Registers, was tested with several value sets,
including positive and negative values displayed as signed 16-bit quantities
by Modbus Poll.

The RTU protocol transports register values as unsigned 16-bit words.
Negative values shown by Modbus Poll use two's-complement representation.

Repeated FC10 requests produced the expected acknowledgement containing:

- the requested starting address;
- the requested register quantity;
- a valid Modbus CRC.

## Stress-test results

Long-duration stress testing was performed using repeated Modbus Poll
requests at a 1000 millisecond scan interval.

The reported tests included:

- more than 1000 repeated FC03 Read Holding Registers transactions;
- more than 1000 repeated FC10 Write Multiple Holding Registers transactions;
- continuous testing over periods exceeding ten minutes.

The final tests completed with:

```text
Modbus Poll errors: 0
```

During successful testing, the RTU diagnostic counters showed that:

- detected frame boundaries matched queued frames;
- queued frames matched successfully transmitted responses;
- invalid inter-character-gap events remained zero;
- receive-buffer overflow events remained zero;
- pending-frame overrun events remained zero;
- RTU processing errors remained zero;
- RTU transmit errors remained zero.

This demonstrates that the two-buffer RTU receive design kept pace with the
test traffic and that every complete request delivered to the portable RTU
layer was processed and answered.

## Intermittent timeout investigation

An intermittent timeout and Modbus Poll communication error were initially
observed during external testing.

The investigation included:

- RTU frame counters;
- receive-byte counters;
- frame-boundary counters;
- queued-frame counters;
- response counters;
- invalid-gap counters;
- overflow counters;
- overrun counters;
- processing-error counters;
- transmit-error counters;
- STM32 UART error and receive-state diagnostics;
- Modbus Poll communication-traffic captures.

For requests that reached the RTU library, the diagnostics consistently showed
successful frame detection, processing, and response transmission.

No evidence was found of:

- an FC03 processing defect;
- an FC10 processing defect;
- invalid RTU frame timing;
- receive-buffer overflow;
- pending-frame overrun;
- PDU-processing failure;
- UART response-transmission failure.

The external tester later identified the cause as an unstable PC USB serial
connection. The USB connection could reset or interrupt the PC serial path,
causing Modbus Poll write errors and timeouts.

After correcting the USB serial connection, repeated FC03 and FC10 stress
tests completed without communication errors.

The intermittent timeout was therefore not attributed to the portable Modbus
RTU slave core.

## Modbus Poll error interpretation

During the investigation, Modbus Poll sometimes displayed a message such as
`Write error` even while FC03 Read Holding Registers was selected.

In that context, `Write error` referred to Modbus Poll failing to write a
request to the Windows COM port. It did not indicate that the STM32 failed to
write a Modbus register.

This distinction was important when separating PC-side serial failures from
Modbus protocol-processing failures.

## Diagnostic statistics

The portable RTU layer exposes diagnostic counters through:

```c
void mbrtu_get_stats(const mbrtu_context_t *ctx,
                     mbrtu_stats_t *stats);
```

This function copies a best-effort snapshot of the current RTU counters.

The available counters include:

- `received_bytes`
- `detected_frame_boundaries`
- `queued_frames`
- `invalid_gap_events`
- `overflow_events`
- `overrun_frames`
- `no_response_frames`
- `responses_sent`
- `processing_errors`
- `transmit_errors`

The function does not process requests, modify the register map, or reset the
counters.

A nonzero `no_response_frames` value is not automatically an error. A frame
may intentionally produce no response when, for example:

- it is a broadcast request;
- it is addressed to another slave;
- it fails address or CRC validation;
- protocol rules require the request to be ignored.

One `no_response_frames` event was observed during manual diagnostic testing.
It did not correspond to a failed normal request.

## Buffering architecture

The portable RTU implementation uses two complete-frame receive buffers:

- one active buffer owned by the receive and timing interrupt path;
- one completed buffer published to the main-loop polling function.

This design keeps protocol and response processing outside the interrupt
context while permitting reception to continue in the alternate buffer.

The physical hardware tests reported:

```text
overflow_events = 0
overrun_frames  = 0
```

The original intermittent timeout was not caused by a missing circular byte
queue.

A byte-oriented circular queue may still be considered later as an optional
hardware-adapter enhancement. Such an adapter must preserve the timing
information required for Modbus RTU T1.5 and T3.5 detection.

A plain FIFO without timing or timestamp information must not replace the
validated RTU timing state machine.

## Validation scope and limitations

The physical STM32F767 tests were performed externally by a project
collaborator using their own:

- STM32F767IGTx board;
- CubeMX configuration;
- Keil toolchain;
- USB-to-TTL adapter;
- Windows computer;
- Modbus Poll installation;
- wiring and power environment.

The test artifacts included screenshots, communication-traffic captures,
diagnostic values, and long-duration test results.

The validation was not independently reproduced in the repository
maintainer's local laboratory.

The results validate the portable RTU slave implementation on the reported
STM32F767 hardware, but they do not guarantee identical behavior for every
MCU, board, USB serial adapter, operating system, driver, or physical wiring
configuration.

## Current project status

The following stages are complete:

- shared transport-independent Modbus PDU engine;
- Modbus TCP ADU processing;
- Modbus CRC-16;
- complete-frame Modbus RTU processing;
- portable RTU byte-receive and timing state machine;
- host-based RTU tests;
- continuous-integration verification;
- external STM32F767 physical RTU slave validation;
- successful testing of all supported RTU slave function codes;
- successful long-duration FC03 and FC10 stress testing.

The Modbus RTU slave implementation is considered functionally validated for
the reported STM32F767 test configuration.

## Next development stage

The next planned major development stage is the portable Modbus RTU master
transaction engine.

The planned master work includes:

- master request ADU construction;
- response address validation;
- response CRC validation;
- response function-code matching;
- Modbus exception-response handling;
- response timeouts;
- retry handling;
- broadcast-write handling;
- host-based tests;
- support for FC01, FC02, FC03, FC04, FC05, FC06, FC0F, and FC10;
- independence from STM32 HAL, CubeMX, Keil, and a particular UART peripheral.

STM32-specific master integration and physical master testing will follow
after the portable master engine has passed host tests and continuous
integration.
