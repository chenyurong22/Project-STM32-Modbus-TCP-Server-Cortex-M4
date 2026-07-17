# Build and hardware smoke test

## Keil build

1. Open `MDK-ARM/CAN.uvprojx` in Keil MDK-ARM.
2. Confirm the selected device is STM32F767IGTx.
3. Perform a full **Rebuild**.
4. Flash the resulting image to the target board.

The externally validated build used µVision 5.41 and ArmClang 6.22 and reported
zero errors and zero warnings.

## Serial connection

USART3 is configured as 115200 baud, 8N1:

- PB10 (TX) to slave RX
- PB11 (RX) to slave TX
- GND to GND

Use compatible 3.3 V TTL UART levels. Do not connect true RS-232 voltage levels
directly to STM32 GPIO pins. This example does not provide an RS-485
transceiver or DE/RE control.

## Automatic request

After a short startup delay, the firmware sends FC03 once per second:

```text
01 03 00 00 00 05 85 C9
```

This requests five holding registers from slave address 1, starting at address
0. A normal 15-byte response has this shape:

```text
01 03 0A RR RR RR RR RR RR RR RR RR RR CRC CRC
```

## Watch variables

Healthy operation is indicated by:

```text
modbus_master_requests_sent == modbus_master_valid_responses
modbus_master_timeouts == 0
modbus_master_transmit_errors == 0
modbus_master_response_errors == 0
modbus_master_invalid_gap_frames == 0
modbus_master_overflow_frames == 0
modbus_master_overrun_frames == 0
modbus_master_unexpected_bytes == 0
modbus_uart_error_count == 0
```

The decoded values are available in `modbus_master_holding_registers[5]`.

## Architecture boundary

`app/app.c` is a board-specific validation harness. Protocol building and
validation remain in the portable repository sources. Retry policy and a
reusable transaction engine are separate future stages.

## Portable-source verification

From the repository root, verify that the scoped portable sources match the
externally tested source content:

```text
python3 Examples/STM32F767_RTU_Master/validation/verify_tested_core.py .
```

The check normalizes Windows and Linux line endings before hashing.
`App/src/platform_stm32.c` is intentionally outside this RTU-master validation
scope because it is the repository's TCP/lwIP platform helper.
