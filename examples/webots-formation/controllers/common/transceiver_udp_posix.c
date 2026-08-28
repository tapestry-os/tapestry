/*
 * transceiver_udp_posix.c — see transceiver_udp_posix.h
 */

#include "transceiver_udp_posix.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

static uint8_t  g_element_id;
static uint8_t  g_n_elements;
static uint16_t g_base_port;
static int      g_sock = -1;
static int      g_dir_sock = -1;   /* directive frames — own port range, see .h */

void udp_posix_configure(uint8_t element_id, uint8_t n_elements, uint16_t base_port)
{
    g_element_id = element_id;
    g_n_elements = n_elements;
    g_base_port  = base_port;
}

/* Shared by udp_posix_init()'s two sockets: create, SO_REUSEADDR, bind to
 * loopback:port, set non-blocking.  Returns the fd, or -1 (closed) on any
 * failure. */
static int bind_nonblocking(uint16_t port, const char *what)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror(what);
        return -1;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror(what);
        close(fd);
        return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

static int udp_posix_init(void)
{
    g_sock = bind_nonblocking((uint16_t)(g_base_port + g_element_id),
                              "transceiver_udp_posix: gossip bind");
    if (g_sock < 0) {
        return -1;
    }

    /* Best-effort: a directive-bind failure must not take gossip down with
     * it — g_dir_sock stays -1 and tx_directive/rx_directive below become
     * no-ops, same degradation as gossip's own metric_sock pattern in
     * transceiver_udp.c. */
    g_dir_sock = bind_nonblocking(
        (uint16_t)(g_base_port + TAPESTRY_UDP_POSIX_DIRECTIVE_PORT_OFFSET
                   + g_element_id),
        "transceiver_udp_posix: directive bind");

    return 0;
}

static int udp_posix_tx(const uint8_t *data, uint16_t len)
{
    if (g_sock < 0) {
        return -1;
    }

    for (uint8_t peer = 0; peer < g_n_elements; peer++) {
        if (peer == g_element_id) {
            continue;
        }

        struct sockaddr_in dst = {0};
        dst.sin_family      = AF_INET;
        dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        dst.sin_port        = htons((uint16_t)(g_base_port + peer));

        sendto(g_sock, data, len, 0, (struct sockaddr *)&dst, sizeof(dst));
    }

    return 0;
}

static int udp_posix_rx(uint8_t *buf, uint16_t max_len)
{
    if (g_sock < 0) {
        return 0;
    }

    ssize_t n = recvfrom(g_sock, buf, max_len, 0, NULL, NULL);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }

    return (int)n;
}

static void udp_posix_set_power(float level)
{
    (void)level;   /* no-op — unsupported on this transceiver */
}

/* ── Directive frames (wire.h v5) — see the .h comment for why these use a
 * separate port range instead of the gossip socket. ─────────────────────── */

static int udp_posix_tx_directive(const uint8_t *data, uint16_t len)
{
    if (g_dir_sock < 0) {
        return -1;
    }

    for (uint8_t peer = 0; peer < g_n_elements; peer++) {
        if (peer == g_element_id) {
            continue;
        }

        struct sockaddr_in dst = {0};
        dst.sin_family      = AF_INET;
        dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        dst.sin_port        = htons((uint16_t)
            (g_base_port + TAPESTRY_UDP_POSIX_DIRECTIVE_PORT_OFFSET + peer));

        sendto(g_dir_sock, data, len, 0, (struct sockaddr *)&dst, sizeof(dst));
    }

    return 0;
}

static int udp_posix_rx_directive(uint8_t *buf, uint16_t max_len)
{
    if (g_dir_sock < 0) {
        return 0;
    }

    ssize_t n = recvfrom(g_dir_sock, buf, max_len, 0, NULL, NULL);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }

    return (int)n;
}

const tapestry_transceiver_t transceiver_udp_posix = {
    .type         = TRANSCEIVER_TYPE_UDP,
    .init         = udp_posix_init,
    .tx           = udp_posix_tx,
    .rx           = udp_posix_rx,
    .set_power    = udp_posix_set_power,
    .tx_directive = udp_posix_tx_directive,
    .rx_directive = udp_posix_rx_directive,
};
