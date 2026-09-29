/*
 * Header packing and whole-frame encode/decode.
 *
 * The header is written and read byte by byte rather than by casting a struct over the
 * buffer: the wire is little-endian and packed, and a struct is neither of those things
 * in any way the C standard promises. One memcpy of a struct would work on this Cortex-M
 * and break the day the protocol meets a different compiler.
 */
#include "sbl_internal.h"

static void put_u16(uint8_t *at, uint16_t value)
{
    at[0] = (uint8_t)(value & 0xFFu);
    at[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *at)
{
    return (uint16_t)((uint16_t)at[0] | ((uint16_t)at[1] << 8));
}

long sbl_encode(const sbl_frame *frame, uint8_t *out, size_t cap)
{
    uint8_t header[SBL_HEADER_SIZE];
    sbl_cobs_enc enc;
    uint16_t crc;
    uint16_t i;
    long encoded;

    if (frame->len > SBL_MAX_PAYLOAD) {
        return SBL_E_TOO_BIG;
    }

    header[0] = frame->ver ? frame->ver : SBL_VERSION;
    header[1] = frame->addr;
    header[2] = frame->type;
    header[3] = frame->flags;
    header[4] = frame->seq;
    put_u16(&header[5], frame->id);
    put_u16(&header[7], frame->len);

    crc = sbl_crc16(header, SBL_HEADER_SIZE, 0xFFFFu);
    if (frame->len) {
        crc = sbl_crc16(frame->payload, frame->len, crc);
    }

    /* Encoded straight into the caller's buffer: the body is never assembled, so this
       costs nine bytes of stack instead of the whole frame. */
    sbl_cobs_begin(&enc, out, cap);
    for (i = 0; i < SBL_HEADER_SIZE; i++) {
        sbl_cobs_put(&enc, header[i]);
    }
    for (i = 0; i < frame->len; i++) {
        sbl_cobs_put(&enc, frame->payload[i]);
    }
    sbl_cobs_put(&enc, (uint8_t)(crc & 0xFFu));
    sbl_cobs_put(&enc, (uint8_t)(crc >> 8));

    encoded = sbl_cobs_end(&enc);
    if (encoded < 0) {
        return encoded;
    }
    if ((size_t)encoded + 1 > cap) {
        return SBL_E_TOO_BIG;
    }
    out[encoded] = 0x00;            /* the delimiter is part of the frame */
    return encoded + 1;
}

int sbl_decode_block(const uint8_t *block, size_t len,
                     uint8_t *scratch, size_t scratch_cap, sbl_frame *out)
{
    long raw_len;
    size_t body_len;
    uint16_t stated, computed;

    raw_len = sbl_cobs_decode(block, len, scratch, scratch_cap);
    if (raw_len < 0) {
        return (int)raw_len;
    }
    if ((size_t)raw_len < (size_t)(SBL_HEADER_SIZE + SBL_CRC_SIZE)) {
        return SBL_E_SHORT;
    }

    body_len = (size_t)raw_len - SBL_CRC_SIZE;
    stated = get_u16(&scratch[body_len]);
    computed = sbl_crc16(scratch, body_len, 0xFFFFu);
    if (stated != computed) {
        return SBL_E_CRC;
    }

    out->ver   = scratch[0];
    out->addr  = scratch[1];
    out->type  = scratch[2];
    out->flags = scratch[3];
    out->seq   = scratch[4];
    out->id    = get_u16(&scratch[5]);
    out->len   = get_u16(&scratch[7]);

    /* The header's own length has to agree with how many bytes actually arrived,
       or the frame is not the one the sender built. */
    if ((size_t)out->len != body_len - SBL_HEADER_SIZE) {
        return SBL_E_LENGTH;
    }
    out->payload = out->len ? &scratch[SBL_HEADER_SIZE] : NULL;
    return SBL_OK;
}
