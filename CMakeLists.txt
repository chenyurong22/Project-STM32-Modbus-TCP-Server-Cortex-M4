cmake_minimum_required(VERSION 3.16)
project(stm32_modbus_tcp_server VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
endif()

add_library(modbus_tcp_core STATIC
    App/src/modbus.c
    App/src/modbus_protocol.c
)
target_include_directories(modbus_tcp_core PUBLIC App/include)

add_executable(modbus_posix_server Examples/posix_server.c)
target_link_libraries(modbus_posix_server PRIVATE modbus_tcp_core)

add_executable(modbus_pdu_tests Tests/host/test_modbus_pdu.c)
target_link_libraries(modbus_pdu_tests PRIVATE modbus_tcp_core)

add_executable(modbus_protocol_tests Tests/host/test_modbus_protocol.c)
target_link_libraries(modbus_protocol_tests PRIVATE modbus_tcp_core)

add_executable(register_selftest
    Tests/host/register_selftest_runner.c
    Tests/stm32/register_selftest.c
)
target_include_directories(register_selftest PRIVATE Tests/stm32)
target_link_libraries(register_selftest PRIVATE modbus_tcp_core)

add_library(lwip_transport_compile_check OBJECT
    App/src/modbus_tcp.c
    App/src/platform_stm32.c
)
target_include_directories(lwip_transport_compile_check PRIVATE
    App/include
    Tests/mocks
)

enable_testing()
add_test(NAME pdu COMMAND modbus_pdu_tests)
add_test(NAME protocol COMMAND modbus_protocol_tests)
add_test(NAME register_map COMMAND register_selftest)
