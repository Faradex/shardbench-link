/*
 * ShardBench Link — wire format, device side.
 *
 * Faradex. The same protocol the host speaks, in C99 with no platform underneath it:
 * no mbed, no printf, no malloc, no floating point. That is not tidiness for its own
 * sake — it means these sources also compile on a PC, so the host's test suite exercises
 * the firmware's own framing code rather than a second implementation that has to be
 * kept in agreement with it.
 *
 * Wire format:  COBS( header[9] | payload[len] | crc16_le[2] ) 0x00
 * Header, little-endian: ver u8, addr u8, type u8, flags u8, seq u8, id u16, len u16
 * CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over header + payload.
 */
#ifndef SHARDBENCH_LINK_H
#define SHARDBENCH_LINK_H

#include <stddef.h>
#include <stdint.h>

#include "sbl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SBL_VERSION 0x01

/** Point-to-point uses address 0. The byte exists for a future multi-drop bus. */
#define SBL_ADDR_P2P 0x00

typedef enum {
    SBL_GET      = 0x01,
    SBL_SET      = 0x02,
    SBL_CALL     = 0x03,
    SBL_RSP      = 0x10,
    SBL_ERR      = 0x11,
    SBL_ACCEPTED = 0x12,
    SBL_EVT      = 0x20,
    SBL_STREAM   = 0x21,
    SBL_CHUNK    = 0x22,
    SBL_LOG      = 0x30
} sbl_msg_type;

#define SBL_FLAG_NONE  0x00
#define SBL_FLAG_MORE  0x01
#define SBL_FLAG_ASYNC 0x02

typedef enum {
    SBL_ERR_UNKNOWN_ID      = 0x01,
    SBL_ERR_BAD_TYPE        = 0x02,
    SBL_ERR_BAD_LEN         = 0x03,
    SBL_ERR_BAD_VALUE       = 0x04,
    SBL_ERR_BUSY            = 0x05,
    SBL_ERR_NOT_READY       = 0x06,
    SBL_ERR_HW_FAIL         = 0x07,
    SBL_ERR_TIMEOUT         = 0x08,
    SBL_ERR_UNSUPPORTED_VER = 0x09
} sbl_err_code;

/** Core resource ids. Device resources start at 0x0100. */
typedef enum {
    SBL_ID_PING       = 0x0001,
    SBL_ID_INFO       = 0x0002,
    SBL_ID_TIME       = 0x0003,
    SBL_ID_STATUS     = 0x0004,
    SBL_ID_RESET      = 0x0005,
    SBL_ID_STREAM_CFG = 0x0006,
    SBL_ID_LOG_LEVEL  = 0x0007
} sbl_core_id;

/** Negative values are failures; zero and above are byte counts or success. */
typedef enum {
    SBL_OK        =  0,
    SBL_E_SHORT   = -1,   /**< fewer bytes than a frame needs */
    SBL_E_CRC     = -2,   /**< the CRC does not match the body */
    SBL_E_COBS    = -3,   /**< malformed COBS block */
    SBL_E_LENGTH  = -4,   /**< the header's length disagrees with the block */
    SBL_E_TOO_BIG = -5    /**< payload over the limit, or the output buffer is too small */
} sbl_status;

/**
 * One frame. `payload` points into a buffer the caller owns and must outlive the frame:
 * nothing here copies, because on a device the copy is the expensive part.
 */
typedef struct {
    uint8_t        ver;
    uint8_t        addr;
    uint8_t        type;
    uint8_t        flags;
    uint8_t        seq;
    uint16_t       id;
    uint16_t       len;
    const uint8_t *payload;
} sbl_frame;

/* --- primitives ------------------------------------------------------------ */

/** CRC-16/CCITT-FALSE. Pass 0xFFFF as `crc` to start; feed it back to continue. */
uint16_t sbl_crc16(const uint8_t *data, size_t len, uint16_t crc);

/** Returns the encoded length, or SBL_E_TOO_BIG if `cap` cannot hold it. */
long sbl_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t cap);

/** Returns the decoded length, SBL_E_COBS if malformed, SBL_E_TOO_BIG if it will not fit. */
long sbl_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t cap);

/* --- frames ---------------------------------------------------------------- */

/**
 * Encode one frame, delimiter included, ready to be written to the wire.
 * Returns the number of bytes written, or a negative sbl_status.
 */
long sbl_encode(const sbl_frame *frame, uint8_t *out, size_t cap);

/**
 * Decode one COBS block (the bytes between two delimiters).
 *
 * `scratch` receives the decoded body and must stay alive for as long as `out` is
 * read, because `out->payload` points into it.
 */
int sbl_decode_block(const uint8_t *block, size_t len,
                     uint8_t *scratch, size_t scratch_cap, sbl_frame *out);

/* --- incremental decoding -------------------------------------------------- */

typedef void (*sbl_frame_fn)(const sbl_frame *frame, void *user);

/**
 * Byte-stream decoder. Feed it whatever the UART gave you, in any sized pieces.
 *
 * A block that is malformed, fails its CRC or outgrows the buffer is dropped and
 * counted, and the stream picks up again at the next delimiter: a device that has been
 * powered up mid-transmission, or a line with noise on it, must recover by itself
 * rather than needing someone to restart it.
 */
typedef struct {
    uint8_t      block[SBL_MAX_ENCODED];
    uint8_t      body[SBL_MAX_BODY];
    uint16_t     fill;
    uint8_t      overflowed;      /**< drop the rest of this block */
    uint32_t     errors;          /**< blocks thrown away since init */
    sbl_frame_fn on_frame;
    void        *user;
} sbl_decoder;

void sbl_decoder_init(sbl_decoder *d, sbl_frame_fn on_frame, void *user);
void sbl_decoder_feed(sbl_decoder *d, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* SHARDBENCH_LINK_H */
