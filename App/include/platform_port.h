#ifndef PLATFORM_PORT_H
#define PLATFORM_PORT_H

#include <stdint.h>

#ifndef MBTCP_SERVER_PORT
#define MBTCP_SERVER_PORT 502u
#endif

#ifndef MBTCP_MAX_CLIENTS
#define MBTCP_MAX_CLIENTS 3u
#endif

#ifndef MB_MAX_COILS
#define MB_MAX_COILS 256u
#endif

#ifndef MB_MAX_DISCRETE_INPUTS
#define MB_MAX_DISCRETE_INPUTS 256u
#endif

#ifndef MB_MAX_HREGS
#define MB_MAX_HREGS 256u
#endif

#ifndef MB_MAX_IREGS
#define MB_MAX_IREGS 256u
#endif

#if MBTCP_SERVER_PORT > 65535u
#error "MBTCP_SERVER_PORT must fit in a TCP port number"
#endif

#if MBTCP_MAX_CLIENTS < 1u || MBTCP_MAX_CLIENTS > 255u
#error "MBTCP_MAX_CLIENTS must be between 1 and 255"
#endif

#if MB_MAX_COILS < 1u || MB_MAX_COILS > 65536u || \
    MB_MAX_DISCRETE_INPUTS < 1u || MB_MAX_DISCRETE_INPUTS > 65536u || \
    MB_MAX_HREGS < 1u || MB_MAX_HREGS > 65536u || \
    MB_MAX_IREGS < 1u || MB_MAX_IREGS > 65536u
#error "Modbus bank sizes must be between 1 and 65536"
#endif

/* Define WITH_RTOS when the register map is accessed from multiple tasks. */
#ifdef WITH_RTOS
void mb_lock(void);
void mb_unlock(void);
#else
static inline void mb_lock(void) {}
static inline void mb_unlock(void) {}
#endif

#endif /* PLATFORM_PORT_H */
