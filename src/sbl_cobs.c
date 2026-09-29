/*
 * Consistent Overhead Byte Stuffing.
 *
 * COBS removes every zero byte from the body, which leaves 0x00 free to mean "frame
 * ends here". That is what lets a listener that joined the line halfway through find
 * the start of the next frame without knowing anything about the one it missed.
 *
 * The encoder is written as a byte-at-a-time state machine, and the bulk entry point
 * is built on top of it, so there is one implementation of the algorithm rather than
 * two that have to agree.
 */
#include "sbl_internal.h"

void sbl_cobs_begin(sbl_cobs_enc *e, uint8_t *out, size_t cap)
{
    e->out = out;
    e->cap = cap;
    e->at = 1;          /* out[0] is the first count, written when its run ends */
    e->code_at = 0;
    e->code = 1;
    e->failed = (uint8_t)(cap < 1);
}

/** Close the run in progress and open another. */
static void start_new_run(sbl_cobs_enc *e)
{
    e->out[e->code_at] = e->code;
    e->code_at = e->at;
    e->code = 1;
    if (e->at + 1 > e->cap) {
        e->failed = 1;
        return;
    }
    e->at++;
}

void sbl_cobs_put(sbl_cobs_enc *e, uint8_t byte)
{
    if (e->failed) {
        return;
    }
    if (byte == 0) {
        /* The run ends here, and the zero itself is what the count stands for. */
        start_new_run(e);
        return;
    }
    if (e->at + 1 > e->cap) {
        e->failed = 1;
        return;
    }
    e->out[e->at++] = byte;
    e->code++;
    if (e->code == 0xFF) {
        /* A run cannot describe more than 254 bytes. */
        start_new_run(e);
    }
}

long sbl_cobs_end(sbl_cobs_enc *e)
{
    if (e->failed) {
        return SBL_E_TOO_BIG;
    }
    e->out[e->code_at] = e->code;
    return (long)e->at;
}

long sbl_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t cap)
{
    sbl_cobs_enc enc;
    size_t i;

    sbl_cobs_begin(&enc, out, cap);
    for (i = 0; i < len; i++) {
        sbl_cobs_put(&enc, in[i]);
    }
    return sbl_cobs_end(&enc);
}

long sbl_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t cap)
{
    size_t i = 0;
    size_t at = 0;

    while (i < len) {
        uint8_t code = in[i];
        size_t end;

        /* A count of zero describes a run of no length: no encoder produces that. */
        if (code == 0) {
            return SBL_E_COBS;
        }
        i++;
        end = i + (size_t)code - 1;
        if (end > len) {
            return SBL_E_COBS;
        }
        if (at + (end - i) > cap) {
            return SBL_E_TOO_BIG;
        }
        while (i < end) {
            out[at++] = in[i++];
        }
        /* Every run but a full one, and never the last, stood for a zero byte. */
        if (code != 0xFF && i < len) {
            if (at + 1 > cap) {
                return SBL_E_TOO_BIG;
            }
            out[at++] = 0;
        }
    }
    return (long)at;
}
