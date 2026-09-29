/*
 * Turning a byte stream into frames.
 *
 * The UART hands over whatever happened to be in the FIFO, so a frame arrives in any
 * number of pieces and two frames can arrive in one. Everything between two 0x00 bytes
 * is a candidate block; one that does not decode is dropped and counted, and the next
 * delimiter starts a fresh one.
 */
#include "shardbench_link.h"

void sbl_decoder_init(sbl_decoder *d, sbl_frame_fn on_frame, void *user)
{
    d->fill = 0;
    d->overflowed = 0;
    d->errors = 0;
    d->on_frame = on_frame;
    d->user = user;
}

static void finish_block(sbl_decoder *d)
{
    sbl_frame frame;
    int status;

    if (d->overflowed) {
        /* Counted once, where it was noticed, not again here. */
        d->overflowed = 0;
        d->fill = 0;
        return;
    }
    if (d->fill == 0) {
        return;          /* two delimiters in a row, or idle line: nothing to report */
    }

    status = sbl_decode_block(d->block, d->fill, d->body, sizeof(d->body), &frame);
    if (status == SBL_OK) {
        if (d->on_frame) {
            d->on_frame(&frame, d->user);
        }
    } else {
        d->errors++;
    }
    d->fill = 0;
}

void sbl_decoder_feed(sbl_decoder *d, const uint8_t *data, size_t len)
{
    size_t i;

    for (i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (byte == 0x00) {
            finish_block(d);
            continue;
        }
        if (d->overflowed) {
            continue;    /* already lost: wait for the delimiter */
        }
        if (d->fill >= (uint16_t)sizeof(d->block)) {
            /* Longer than any legal frame, so the delimiter was missed or the line is
               noise. Count it now and swallow the rest. */
            d->overflowed = 1;
            d->errors++;
            continue;
        }
        d->block[d->fill++] = byte;
    }
}
