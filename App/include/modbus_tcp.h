// App/include/modbus_tcp.h
#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void mbtcp_init(void);
void mbtcp_poll(void); // optional, for future extensions

#ifdef __cplusplus
}
#endif

#endif // MODBUS_TCP_H

