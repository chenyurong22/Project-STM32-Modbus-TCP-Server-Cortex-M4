# Useful Wireshark display filters

```text
# All Modbus TCP traffic
tcp.port == 502

# Decoded Modbus function codes
modbus.func_code

# Modbus exceptions
modbus.exception_code

# Requests carrying payload
modbus && tcp.len > 0
```

For the local POSIX demo, replace port `502` with `1502` (or the port passed on the command line).
