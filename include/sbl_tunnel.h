/*
 * The ShardBench Link byte stream over CAN, through a Crystal (ShardBench docs:
 * architecture.md 7.1, decisions.md D-7).
 *
 * Plain C, no CAN driver underneath: the firmware sends and receives the frames, this
 * decides what goes in them. The Crystal's service implements the same rules in Python.
 *
 * - host to node on paramId 0x7A0, node to host on 0x7A1, as (localId << 11) | param;
 * - data byte 0 is a sequence number (low four bits), bytes 1-7 up to 7 stream bytes;
 * - byte 0 alone is a keepalive: no bytes, and the sequence does not move;
 * - an unexpected sequence means a lost frame: a 0x00 goes to the decoder first, which
 *   drops the link frame in progress -- the host's retry recovers it;
 * - a node is attached while it has heard on 0x7A0 within SBL_TUNNEL_ATTACHED_MS, and
 *   sends on 0x7A1 only then.
 */
#ifndef SBL_TUNNEL_H
#define SBL_TUNNEL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SBL_TUNNEL_TO_NODE      0x7A0u
#define SBL_TUNNEL_TO_HOST      0x7A1u
#define SBL_TUNNEL_CHUNK        7u
#define SBL_TUNNEL_FRAME_MAX    8u
#define SBL_TUNNEL_ATTACHED_MS  3000u

typedef struct {
    uint8_t  tx_seq;
    uint8_t  rx_expected;
    uint8_t  rx_synced;      /* 0 until the first frame: any sequence is accepted then */
    uint8_t  heard;          /* 0 until anything has arrived */
    uint32_t last_rx_ms;
    uint32_t gaps;           /* lost frames noticed, for a stats line */
} sbl_tunnel;

void sbl_tunnel_init(sbl_tunnel *t);

/**
 * One frame received on 0x7A0 for this node. Writes the stream bytes it carries to
 * `out` (room for SBL_TUNNEL_FRAME_MAX) and returns how many: a 0x00 first when frames
 * were lost, nothing for a keepalive. Either way the node now counts as attached.
 */
size_t sbl_tunnel_receive(sbl_tunnel *t, const uint8_t *frame, size_t len,
                          uint32_t now_ms, uint8_t *out);

/** Whether the host has been heard recently enough to be spoken to. */
int sbl_tunnel_attached(const sbl_tunnel *t, uint32_t now_ms);

/**
 * Cut the next frame for 0x7A1 out of `data`: writes it to `frame` (room for
 * SBL_TUNNEL_FRAME_MAX), sets `*used` to the stream bytes it took, returns its length.
 * Returns 0 when `len` is 0.
 */
size_t sbl_tunnel_next_frame(sbl_tunnel *t, const uint8_t *data, size_t len,
                             uint8_t *frame, size_t *used);

#ifdef __cplusplus
}
#endif

#endif /* SBL_TUNNEL_H */
