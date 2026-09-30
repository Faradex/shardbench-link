/*
 * Shared between the library's own translation units. Not part of the public API.
 */
#ifndef SBL_INTERNAL_H
#define SBL_INTERNAL_H

#include "shardbench_link.h"

/**
 * COBS encoder fed one byte at a time.
 *
 * It exists so that a frame can be written straight into the caller's output buffer
 * without ever assembling the body anywhere else. Building the body first would cost
 * half a kilobyte of stack in whichever thread happens to send -- and on this device
 * that is the measurement thread, whose stack we do not own.
 *
 * Once a write would overrun the output, the encoder latches failed and quietly
 * swallows the rest: the caller finds out once, at the end, instead of every byte.
 */
typedef struct {
    uint8_t *out;
    size_t   cap;
    size_t   at;        /**< next free byte in out */
    size_t   code_at;   /**< where the count for the run in progress belongs */
    uint8_t  code;      /**< bytes in this run, the count itself included */
    uint8_t  failed;
} sbl_cobs_enc;

void sbl_cobs_begin(sbl_cobs_enc *e, uint8_t *out, size_t cap);
void sbl_cobs_put(sbl_cobs_enc *e, uint8_t byte);
long sbl_cobs_end(sbl_cobs_enc *e);

/**
 * Encode a frame whose payload arrives in two pieces, `prefix` first.
 *
 * `frame->len` is the length of both together. It exists for STREAM, whose payload is a
 * small header this library writes followed by samples the application owns: copying the
 * two into one buffer would put half a kilobyte on the measurement thread's stack, which
 * is the cost the incremental encoder was built to avoid.
 */
long sbl_encode_parts(const sbl_frame *frame,
                      const uint8_t *prefix, uint16_t prefix_len,
                      uint8_t *out, size_t cap);

#endif /* SBL_INTERNAL_H */
