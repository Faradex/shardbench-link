#include "sbl_tunnel.h"

#include <string.h>

#define SEQ_MASK 0x0Fu

void sbl_tunnel_init(sbl_tunnel *t)
{
    memset(t, 0, sizeof(*t));
}

size_t sbl_tunnel_receive(sbl_tunnel *t, const uint8_t *frame, size_t len,
                          uint32_t now_ms, uint8_t *out)
{
    size_t n = 0;
    uint8_t seq;

    if (len == 0) {
        return 0;
    }
    t->heard = 1;
    t->last_rx_ms = now_ms;
    if (len == 1) {
        return 0;                                 /* a keepalive */
    }
    if (len > SBL_TUNNEL_FRAME_MAX) {
        len = SBL_TUNNEL_FRAME_MAX;
    }
    seq = (uint8_t)(frame[0] & SEQ_MASK);
    if (t->rx_synced && seq != t->rx_expected) {
        out[n++] = 0x00;                          /* drop the link frame in progress */
        t->gaps++;
    }
    t->rx_synced = 1;
    t->rx_expected = (uint8_t)((seq + 1u) & SEQ_MASK);
    /* One byte is the 0x00 at most, and 7 the stream: SBL_TUNNEL_FRAME_MAX holds both. */
    memcpy(out + n, frame + 1, len - 1);
    return n + len - 1;
}

int sbl_tunnel_attached(const sbl_tunnel *t, uint32_t now_ms)
{
    /* Unsigned subtraction: right across the tick counter wrapping. */
    return t->heard && (uint32_t)(now_ms - t->last_rx_ms) < SBL_TUNNEL_ATTACHED_MS;
}

size_t sbl_tunnel_next_frame(sbl_tunnel *t, const uint8_t *data, size_t len,
                             uint8_t *frame, size_t *used)
{
    size_t take = len < SBL_TUNNEL_CHUNK ? len : SBL_TUNNEL_CHUNK;

    *used = take;
    if (take == 0) {
        return 0;
    }
    frame[0] = t->tx_seq;
    memcpy(frame + 1, data, take);
    t->tx_seq = (uint8_t)((t->tx_seq + 1u) & SEQ_MASK);
    return take + 1;
}

size_t sbl_tunnel_presence(const sbl_tunnel *t, uint32_t now_ms, uint8_t *frame)
{
    if (sbl_tunnel_attached(t, now_ms)) {
        return 0;                    /* the host is listening: it has the stream */
    }
    if (t->presence_sent && (uint32_t)(now_ms - t->presence_ms) < SBL_TUNNEL_PRESENCE_MS) {
        return 0;
    }
    frame[0] = t->tx_seq;            /* a keepalive: the sequence does not move */
    return 1;
}

void sbl_tunnel_presence_sent(sbl_tunnel *t, uint32_t now_ms)
{
    t->presence_sent = 1;
    t->presence_ms = now_ms;
}
