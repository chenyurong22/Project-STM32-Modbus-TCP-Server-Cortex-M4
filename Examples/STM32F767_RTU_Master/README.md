# STM32F767 Modbus RTU master example

This directory contains the externally validated STM32F767IGTx bare-metal
hardware example for the repository's portable Modbus RTU master core.

The example keeps board-specific STM32 HAL code separate from the canonical
portable sources in the repository root. The Keil project references:

- `../../../App/include`
- `../../../App/src`

Do not copy a second Modbus implementation into this directory.

## Validated configuration

- MCU: STM32F767IGTx
- Toolchain: Keil µVision 5.41, ArmClang 6.22
- Operating system: none (bare metal)
- UART: USART3
- Pins: PB10 TX, PB11 RX
- Serial format: 115200 baud, 8 data bits, no parity, 1 stop bit
- Electrical interface tested: 3.3 V TTL UART with crossed TX/RX and common GND
- Modbus role: RTU master
- Test request: FC03, slave 1, start address 0, quantity 5

The project name `CAN` is inherited from the supplied CubeMX/Keil board project.
It is retained to avoid changing the externally tested build configuration.

## Build

Open:

```text
MDK-ARM/CAN.uvprojx
```

Perform a full **Rebuild** before flashing.

The example depends on the repository's portable files, including
`modbus_rtu_master.c` and `modbus_rtu_master.h`. Therefore, keep this directory
inside the repository at its current relative path.

## Scope

This example validates request generation, USART3 transmission,
interrupt-driven reception, RTU T1.5/T3.5 frame timing, bounded double
buffering, response timeout reporting, strict complete-frame validation, and
FC03 register decoding.

It is not yet the reusable master transaction engine and does not implement
RS-485 DE/RE direction control. Physical RS-485 validation requires an external
transceiver and a later transport adapter.

See `BUILD_AND_TEST.md` and `validation/EXTERNAL_VALIDATION.md`.

## Clean-project note

The original validation archive compiled a local `platform_stm32.c` helper even
though the RTU master harness did not use its `sys_now()` function. The
repository's canonical `App/src/platform_stm32.c` is the TCP/lwIP adapter and
must not be pulled into this bare-metal RTU target. The clean Keil project
therefore omits that unrelated source file.
