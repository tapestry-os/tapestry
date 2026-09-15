/*
 * status_tx.c — see status_tx.h
 */

#include "status_tx.h"

#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

static int      g_sock = -1;
static uint16_t g_port;

void status_tx_init(uint16_t base_port)
{
    g_port = (uint16_t)(base_port + WH_STATUS_PORT_OFFSET);

    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) {
        return;
    }
    /* Non-blocking: a supervisor that is not listening must never stall a
     * control loop, and on some platforms an unread local datagram socket
     * can push back. */
    int flags = fcntl(g_sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(g_sock, F_SETFL, flags | O_NONBLOCK);
    }
}

void status_tx_send(const wh_status_t *st)
{
    if (g_sock < 0 || st == NULL) {
        return;
    }

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family      = AF_INET;
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    dst.sin_port        = htons(g_port);

    /* Return value deliberately ignored — see status_tx.h. */
    (void)sendto(g_sock, st, sizeof(*st), 0,
                 (struct sockaddr *)&dst, sizeof(dst));
}
