<p align="center">
  <img src="docs/images/hero.svg" alt="STM32 Modbus TCP Server" width="100%">
</p>

# STM32 Modbus TCP Server for Cortex‑M4

A compact Modbus TCP server written in portable C11, with an STM32/lwIP raw-API transport, a deterministic in-memory register map, strict request validation, and no heap allocation in the request path.

The repository builds and tests on a normal Linux/macOS development machine. For deployment, copy the application sources into the STM32CubeMX project generated for your exact MCU, Ethernet PHY, clocks, pins, and memory layout.

## Build status

The default build verifies all portable code and compile-checks the lwIP transport against API-compatible headers:

```bash
make
make test
```

`make test` runs:

- C unit tests for the register map and Modbus protocol engine
- the on-device register self-test as a host executable
- strict `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror` compile checks
- a real TCP socket integration test against the included POSIX demo server

A GitHub Actions workflow and a Docker build are included.

## Architecture

<p align="center">
  <img src="docs/images/architecture.svg" alt="Layered project architecture" width="95%">
</p>

The design separates the pure protocol engine from the network transport:

- `modbus_protocol.c` validates MBAP/PDU fields, dispatches function codes, and creates normal or exception responses.
- `modbus_tcp.c` handles lwIP TCP callbacks, fragmented/coalesced stream data, client slots, and response transmission.
- `modbus.c` provides the default fixed-size coils and register banks.
- weak write hooks connect Modbus writes to relays, PWM, configuration storage, or other application logic.

## Supported function codes

| Function | Code | Maximum per request | Access |
|---|---:|---:|---|
| Read Coils | `0x01` | 2,000 bits | Read |
| Read Discrete Inputs | `0x02` | 2,000 bits | Read |
| Read Holding Registers | `0x03` | 125 registers | Read |
| Read Input Registers | `0x04` | 125 registers | Read |
| Write Single Coil | `0x05` | 1 bit | Write |
| Write Single Register | `0x06` | 1 register | Write |
| Write Multiple Coils | `0x0F` | 1,968 bits | Write |
| Write Multiple Registers | `0x10` | 123 registers | Write |

Illegal functions, addresses, quantities, byte counts, and values produce standard Modbus exception responses.

## Repository layout

```text
App/
├── include/
│   ├── modbus.h              Register-map API and write hooks
│   ├── modbus_protocol.h     Portable ADU processor
│   ├── modbus_tcp.h          lwIP server API
│   └── platform_port.h       Compile-time configuration
└── src/
    ├── modbus.c              Coils and register storage
    ├── modbus_protocol.c     Transport-independent Modbus engine
    ├── modbus_tcp.c          lwIP raw-API transport
    └── platform_stm32.c      HAL tick and optional RTOS locking

Examples/
├── posix_server.c            Local functional demo
└── stm32_cube_main.c         CubeMX integration reference

Tests/
├── host/                     Unit and socket-level tests
├── mocks/lwip/               Compile-check headers only
└── stm32/                    Register-map self-test for a target board
```

## Quick start on a development machine

Requirements: a C11 compiler, Make, and Python 3.

```bash
git clone <your-repository-url>
cd stm32-modbus-tcp-server
make test
```

Expected result:

```text
modbus protocol tests: PASS
Modbus SelfTest: total=5, passed=5, failed=0, first_err=0
Modbus TCP smoke test: PASS (127.0.0.1:15020)
```

### CMake

```bash
cmake -S . -B build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-cmake --parallel
ctest --test-dir build-cmake --output-on-failure
```

### Docker

```bash
docker build -t stm32-modbus-tcp .
docker run --rm stm32-modbus-tcp
```

## Run the local Modbus TCP demo

The POSIX demo makes it possible to test the same protocol core before hardware is available. It listens on loopback only and defaults to unprivileged port `1502`.

Terminal 1:

```bash
make demo
./build/modbus_posix_server 1502
```

Terminal 2:

```bash
python3 Tests/host/ci_modbus_smoke.py 127.0.0.1 1502
```

The smoke test writes and reads holding registers and coils, then verifies an illegal-address exception.

## STM32CubeMX integration

This repository intentionally does not ship fabricated board startup code, a linker script, PHY configuration, or Ethernet pin assignments. Those settings must match the selected STM32 and board.

### 1. Generate the board project

In STM32CubeMX or STM32CubeIDE:

1. Select the exact MCU or development board.
2. Enable Ethernet and configure the correct RMII/MII pins and PHY address.
3. Enable lwIP.
4. Select the lwIP raw API. The included transport does not require Netconn or BSD sockets.
5. For a no-RTOS project, ensure the generated main loop calls `MX_LWIP_Process()`.
6. Generate the project with its HAL, CMSIS, startup, linker, Ethernet, and lwIP files.

### 2. Add the application sources

Add these files to the generated build:

```text
App/src/modbus.c
App/src/modbus_protocol.c
App/src/modbus_tcp.c
App/src/platform_stm32.c
```

Add this include directory:

```text
App/include
```

For a CubeIDE project, copy the `App` directory into the project, refresh the workspace, and ensure the source folder is not excluded from the active build configuration.

### 3. Initialize the server

Merge the relevant lines from `Examples/stm32_cube_main.c` into the generated `main.c`:

```c
#include "modbus.h"
#include "modbus_tcp.h"

/* After MX_LWIP_Init(): */
mb_init();
mbtcp_init();

/* In a no-RTOS superloop: */
MX_LWIP_Process();
mbtcp_poll();
```

Initialize the server only after the Ethernet interface and lwIP are initialized.

### 4. Configure the data model

Edit `App/include/platform_port.h` or pass equivalent compiler definitions:

```c
#define MBTCP_SERVER_PORT       502u
#define MBTCP_MAX_CLIENTS       3u
#define MB_MAX_COILS            256u
#define MB_MAX_DISCRETE_INPUTS  256u
#define MB_MAX_HREGS            256u
#define MB_MAX_IREGS            256u
```

<p align="center">
  <img src="docs/images/register-map.svg" alt="Default Modbus register map" width="95%">
</p>

### 5. Connect writes to hardware

Override either weak hook in an application source file:

```c
void mb_on_write_coil(uint16_t address, uint8_t value)
{
    if (address == 0u) {
        HAL_GPIO_WritePin(RELAY_GPIO_Port,
                          RELAY_Pin,
                          value != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void mb_on_write_hreg(uint16_t address, uint16_t value)
{
    if (address == 0u) {
        set_pwm_duty(value);
    }
}
```

Update read-only values from the application:

```c
mb_set_ireg(0u, adc_sample);
mb_set_dinput(0u, digital_input_state);
```

### 6. FreeRTOS locking

Define `WITH_RTOS` when multiple tasks access the map. `platform_stm32.c` then uses a FreeRTOS mutex. For stricter real-time requirements, create the mutex during application initialization and replace the default lazy initialization with your project’s synchronization policy.

## Packet processing

<p align="center">
  <img src="docs/images/request-flow.svg" alt="Modbus TCP request and response flow" width="95%">
</p>

The lwIP transport supports:

- TCP frames split across multiple pbufs or receive callbacks
- multiple Modbus ADUs arriving in one TCP stream
- a fixed maximum ADU of 260 bytes
- up to `MBTCP_MAX_CLIENTS` active connections
- no dynamic allocation by the application code

## On-device self-test

Add these files to a temporary target build:

```text
Tests/stm32/register_selftest.c
Tests/stm32/register_selftest.h
```

Then run:

```c
MbSelfTestResult result;
char line[128];

mb_selftest_run(&result);
mb_selftest_print(line, sizeof(line), &result);
printf("%s", line);
```

The self-test exercises all four data banks, boundary behavior, and deterministic bulk patterns without requiring Ethernet.

## Test a physical board

After flashing and assigning an IP address:

```bash
python3 Tests/host/ci_modbus_smoke.py 192.168.1.50 502
```

Or use a third-party Modbus client. Protocol addresses are zero-based in requests; some tools display holding register address `0` as `40001`.

Useful Wireshark filters are in `tools/wireshark_display_filter.md`.

## Memory use

With the default map sizes, data storage uses approximately:

- 256 bytes for coils
- 256 bytes for discrete inputs
- 512 bytes for holding registers
- 512 bytes for input registers
- 260 bytes of receive buffering per active lwIP client

The exact flash and RAM totals depend on compiler settings, lwIP configuration, HAL, PHY driver, and the chosen STM32.

## Security

Modbus TCP provides no authentication, confidentiality, or authorization. Do not expose port 502 directly to the public internet. Use network segmentation, firewall rules, a VPN, or a security gateway, and validate all hardware actions in the write hooks.

## Troubleshooting

**The host build passes, but the STM32 project does not compile.**  
Check that CubeMX generated lwIP raw-API headers, `App/include` is in the include path, and all four `App/src` files are part of the active target.

**The server does not accept connections.**  
Verify PHY link status, MAC/PHY address configuration, IP address assignment, and that `MX_LWIP_Process()` runs continuously in no-RTOS projects.

**Port 502 cannot be opened on a desktop.**  
Ports below 1024 may require elevated privileges. Use the included demo on port 1502 instead.

**A client reports an illegal data address.**  
The requested address range exceeds the configured bank size. Remember that protocol addresses are zero-based.

## License

MIT — see [LICENSE](LICENSE).
