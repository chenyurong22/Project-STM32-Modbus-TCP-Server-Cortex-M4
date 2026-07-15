# Verification report

Verification performed on 2026-07-14.

## Successful checks

| Check | Result |
|---|---|
| GCC strict C11 build | Pass |
| Clang strict C11 build | Pass |
| AddressSanitizer + UndefinedBehaviorSanitizer | Pass |
| Register-map self-test | 5/5 pass |
| Modbus CRC-16 known-vector tests | Pass |
| Shared PDU unit tests | Pass |
| Modbus RTU ADU tests | Pass |
| Modbus RTU 50 us timing/state tests | Pass |
| Modbus TCP ADU regression tests | Pass |
| POSIX TCP integration smoke test | Pass |
| lwIP transport compile check | Pass |
| CMake configure/build/CTest (6 tests) | Pass |
| SVG XML validation and PNG rendering | Pass |

The strict warning set is:

```text
-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror
```

## RTU coverage

The host RTU suite verifies:

- CRC calculation, wire byte order, and corruption rejection
- all eight supported function codes through RTU framing
- normal and exception response CRCs
- configurable slave addressing
- silent discard of wrong-address and invalid-CRC frames
- broadcast writes with no response
- broadcast read and unsupported-function suppression
- truncated, oversized, minimum-size, and maximum-size frames
- output-capacity failure without applying a write
- 9600, 19200, and 115200 timing thresholds
- exact T1.5 acceptance and T1.5-to-T3.5 rejection
- T3.5 frame publication and two-buffer handoff
- pending-frame overrun reporting
- receive-buffer overflow and recovery
- main-loop transmit dispatch and transmit-error reporting
- API argument and configured-address validation

## Scope

The shared Modbus PDU core, backward-compatible TCP ADU wrapper, portable CRC-16 implementation, complete-frame RTU ADU wrapper, host demonstration, tests, and lwIP-facing application sources are verified.

The portable RTU byte-receive/timing state machine, fixed 50 microsecond tick API, T1.5/T3.5 frame detection, buffering, and recovery are host verified. The STM32 UART/timer adapter and physical hardware validation remain outside this stage.

A final STM32 firmware ELF/BIN/HEX cannot be produced without board-specific STM32CubeMX output for the selected MCU and board, including startup code, linker script, HAL/CMSIS, Ethernet MAC/PHY configuration, UART configuration, and generated lwIP port files.

Integration and current RTU scope are documented in `README.md`, `docs/modbus-rtu-core.md`, `docs/modbus-rtu-timing.md`, and `Examples/stm32_cube_main.c`.

## External STM32F767 hardware validation

The portable Modbus RTU slave was integrated into an STM32F767IGTx
CubeMX/Keil project and tested on physical hardware using USART3 at
115200 baud, 8N1.

All supported RTU slave function codes were demonstrated successfully:
FC01, FC02, FC03, FC04, FC05, FC06, FC0F, and FC10.

Long-duration FC03 and FC10 stress testing completed without RTU processing,
buffering, or transmission errors after an unstable PC USB serial connection
was identified and corrected.

See [`docs/stm32f767-rtu-validation.md`](docs/stm32f767-rtu-validation.md)
for the detailed external validation record.
