# Verification report

Verification performed on 2026-07-13.

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
| Modbus TCP ADU regression tests | Pass |
| POSIX TCP integration smoke test | Pass |
| lwIP transport compile check | Pass |
| CMake configure/build/CTest | Pass |
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
- API argument and configured-address validation

## Scope

The shared Modbus PDU core, backward-compatible TCP ADU wrapper, portable CRC-16 implementation, complete-frame RTU ADU wrapper, host demonstration, tests, and lwIP-facing application sources are verified.

The RTU byte-receive/timing state machine, STM32 UART adapter, 50 microsecond tick integration, T1.5/T3.5 frame detection, and physical hardware validation are intentionally outside this stage.

A final STM32 firmware ELF/BIN/HEX cannot be produced without board-specific STM32CubeMX output for the selected MCU and board, including startup code, linker script, HAL/CMSIS, Ethernet MAC/PHY configuration, UART configuration, and generated lwIP port files.

Integration and current RTU scope are documented in `README.md`, `docs/modbus-rtu-core.md`, and `Examples/stm32_cube_main.c`.
