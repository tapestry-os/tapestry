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

void udp_posix_configure(uint8_t element_id, uint8_t n_elements, uint16_t base_port)
{
    g_element_id = element_id;
    g_n_elements = n_elements;
    g_base_port  = base_port;
}

static int udp_posix_init(void)
{
    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) {
        perror("transceiver_udp_posix: socket");
        return -1;
    }

    int reuse = 1;
    setsockopt(g_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons((uint16_t)(g_base_port + g_element_id));

    if (bind(g_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("transceiver_udp_posix: bind");
        close(g_sock);
        g_sock = -1;
        return -1;
    }

    int flags = fcntl(g_sock, F_GETFL, 0);
    fcntl(g_sock, F_SETFL, flags | O_NONBLOCK);

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

const tapestry_transceiver_t transceiver_udp_posix = {
    .type      = TRANSCEIVER_TYPE_UDP,
    .init      = udp_posix_init,
    .tx        = udp_posix_tx,
    .rx        = udp_posix_rx,
    .set_power = udp_posix_set_power,
};
