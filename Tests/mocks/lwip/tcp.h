#ifndef TEST_LWIP_TCP_H
#define TEST_LWIP_TCP_H

#include <stdint.h>

typedef int err_t;
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;

struct tcp_pcb { int unused; };
struct pbuf {
    struct pbuf *next;
    void *payload;
    u16_t tot_len;
    u16_t len;
};

#define ERR_OK   0
#define ERR_MEM -1
#define ERR_ABRT -2
#define ERR_ARG  -3
#define IPADDR_TYPE_ANY 0
#define IP_ANY_TYPE ((const void *)0)
#define TCP_WRITE_FLAG_COPY 1u

struct tcp_pcb *tcp_new_ip_type(u8_t type);
err_t tcp_bind(struct tcp_pcb *pcb, const void *ipaddr, u16_t port);
struct tcp_pcb *tcp_listen_with_backlog(struct tcp_pcb *pcb, u8_t backlog);
void tcp_accept(struct tcp_pcb *pcb, err_t (*accept)(void *, struct tcp_pcb *, err_t));
void tcp_arg(struct tcp_pcb *pcb, void *arg);
void tcp_recv(struct tcp_pcb *pcb, err_t (*recv)(void *, struct tcp_pcb *, struct pbuf *, err_t));
void tcp_err(struct tcp_pcb *pcb, void (*errf)(void *, err_t));
void tcp_sent(struct tcp_pcb *pcb, err_t (*sent)(void *, struct tcp_pcb *, u16_t));
err_t tcp_write(struct tcp_pcb *pcb, const void *data, u16_t length, u8_t flags);
err_t tcp_output(struct tcp_pcb *pcb);
void tcp_recved(struct tcp_pcb *pcb, u16_t length);
err_t tcp_close(struct tcp_pcb *pcb);
void tcp_abort(struct tcp_pcb *pcb);
u8_t pbuf_free(struct pbuf *p);

#endif
