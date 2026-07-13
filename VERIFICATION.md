# Verification report

Verification performed on 2026-07-12.

## Successful checks

| Check | Result |
|---|---|
| GCC 14.2 strict C11 build | Pass |
| Clang 17 strict C11 build | Pass |
| AddressSanitizer + UndefinedBehaviorSanitizer | Pass |
| Register-map self-test | 5/5 pass |
| Shared PDU unit tests | Pass |
| Modbus TCP ADU regression tests | Pass |
| POSIX TCP integration smoke test | Pass |
| lwIP transport compile check | Pass |
| CMake configure/build/CTest | Pass |
| SVG XML validation and rendering preview | Pass |

The strict warning set is:

```text
-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror
```

## Scope

The shared Modbus PDU core, backward-compatible TCP ADU wrapper, host demonstration, tests, and lwIP-facing application sources are verified. A final STM32 firmware ELF/BIN/HEX cannot be produced without the board-specific STM32CubeMX output for the selected MCU and board, including startup code, linker script, HAL/CMSIS, Ethernet MAC/PHY configuration, and generated lwIP port files.

The integration procedure is documented in `README.md` and `Examples/stm32_cube_main.c`.
