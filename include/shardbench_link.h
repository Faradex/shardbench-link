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

/* --- the link -------------------------------------------------------------- */

/**
 * What the library needs from the platform, and all it needs.
 *
 * `write` must put every byte it is given on the wire. `lock`/`unlock` are held around
 * a whole frame, because a frame interleaved with another is lost to both. `tick_ms` is
 * a free-running millisecond counter; only differences matter, so it may wrap.
 */
typedef struct {
    void     (*write)(void *io, const uint8_t *data, size_t len);
    void     (*lock)(void *io);
    void     (*unlock)(void *io);
    uint32_t (*tick_ms)(void *io);
    void      *io;
} sbl_port;

/** What INFO tells the host. Pointed to by the context; must outlive it. */
typedef struct {
    uint16_t    type_id;
    uint32_t    def_hash;
    uint8_t     uid[12];
    uint8_t     fw_version[4];
    const char *sdk_version;
    uint8_t     app_slot;
} sbl_device_info;

/** Which message types a resource answers. */
#define SBL_CAN_GET  0x01
#define SBL_CAN_SET  0x02
#define SBL_CAN_CALL 0x04

/** One request, and the buffer its answer goes in. */
typedef struct {
    uint8_t        type;        /**< SBL_GET, SBL_SET or SBL_CALL */
    /** The host is holding this open. A handler that returns SBL_DEFER must keep it,
        because that is what sbl_resolve() matches the answer to. */
    uint8_t        seq;
    uint16_t       id;
    const uint8_t *payload;     /**< what arrived; NULL when len is 0 */
    uint16_t       len;
    uint8_t       *reply;       /**< write the answer here */
    uint16_t       reply_cap;
    uint16_t       reply_len;   /**< how much of it you wrote */
    uint8_t        error;       /**< an sbl_err_code, when refusing */
} sbl_request;

typedef enum {
    SBL_REPLY  = 0,   /**< reply_len bytes are the answer */
    SBL_DEFER  = 1,   /**< too slow to answer now: ACCEPTED goes out, then sbl_resolve() */
    SBL_REFUSE = 2    /**< set req->error first */
} sbl_action;

typedef sbl_action (*sbl_handler_fn)(sbl_request *req, void *user);

typedef struct {
    uint16_t       id;
    uint8_t        kinds;
    sbl_handler_fn fn;
    void          *user;
} sbl_resource;

/** A stream's configuration, as the host last set it. */
typedef struct {
    uint16_t id;
    uint8_t  enabled;
    uint16_t decimation;
} sbl_stream_cfg;

typedef struct {
    const sbl_port        *port;
    const sbl_device_info *info;
    sbl_decoder            decoder;
    sbl_resource           table[SBL_MAX_RESOURCES];
    uint8_t                resources;
    sbl_stream_cfg         streams[SBL_MAX_STREAMS];
    uint8_t                stream_count;
    uint8_t                log_level;
    uint8_t                state;
    uint32_t               alarms;
    uint32_t               started_ms;
    uint8_t                tx[SBL_MAX_ENCODED];   /**< held under port->lock */
    uint8_t                reply[SBL_MAX_PAYLOAD];
    uint32_t               sent;
    uint32_t               dropped;               /**< frames that would not encode */
    uint32_t               received;              /**< requests dispatched */
    uint32_t               slowest_ms;            /**< longest a request took to answer */
} sbl_ctx;

/** Default log level: 0 off, 1 error, 2 warn, 3 info, 4 debug. */
#define SBL_LOG_INFO 3

void sbl_init(sbl_ctx *ctx, const sbl_port *port, const sbl_device_info *info);

/** Register a resource. Returns 0, or SBL_E_TOO_BIG when the table is full. */
int sbl_add(sbl_ctx *ctx, uint16_t id, uint8_t kinds, sbl_handler_fn fn, void *user);

/** Feed bytes from the wire. Replies are sent from inside this call. */
void sbl_feed(sbl_ctx *ctx, const uint8_t *data, size_t len);

/** Answer a deferred CALL: an EVT carrying ASYNC and the seq of the request. */
void sbl_resolve(sbl_ctx *ctx, uint16_t id, uint8_t seq,
                 const uint8_t *payload, uint16_t len);

/**
 * Give up on a deferred CALL: an ERR carrying ASYNC and the seq of the request.
 *
 * The counterpart of sbl_resolve(). A handler that returned SBL_DEFER has promised an
 * answer, and the host is holding the sequence number until one arrives; a failure that
 * is simply not reported keeps that promise open forever.
 */
void sbl_fail(sbl_ctx *ctx, uint16_t id, uint8_t seq, uint8_t code);

/** A spontaneous event. */
void sbl_emit(sbl_ctx *ctx, uint16_t id, const uint8_t *payload, uint16_t len);

/**
 * Stream samples, already thinned by the configured decimation.
 *
 * `tick_ms` is when the *first* sample of the batch was taken, not when the batch is
 * being sent: the host spaces the rest out by `period_us` from there, so stamping at
 * send time would put the whole batch late by the time it took to fill.
 *
 * Samples are counted, not measured: `count` of them, `sample_bytes` each. The header
 * the host expects in front of them is written here rather than by the application.
 */
void sbl_stream(sbl_ctx *ctx, uint16_t id, uint32_t tick_ms, uint32_t period_us,
                const void *samples, uint16_t count, uint8_t sample_bytes);

/** A log line. Dropped if the host has asked for a quieter level. */
void sbl_log(sbl_ctx *ctx, uint8_t level, const char *text, uint16_t len);

/** What STATUS reports, and what the host sees in the session pill. */
void sbl_set_state(sbl_ctx *ctx, uint8_t state, uint32_t alarms);

/** How the host last configured a stream. Applying it is the application's job. */
const sbl_stream_cfg *sbl_stream_config(const sbl_ctx *ctx, uint16_t id);

/* --- the console ----------------------------------------------------------- */

/** One writer's line, as far as it has got. */
typedef struct {
    const void *owner;
    uint16_t    fill;
    char        text[SBL_CONSOLE_LINE];
} sbl_line;

typedef struct {
    sbl_line lines[SBL_CONSOLE_SLOTS];
} sbl_console;

/** Called with each finished line. The text no longer carries its level prefix. */
typedef void (*sbl_console_fn)(uint8_t level, const char *text, uint16_t len, void *user);

void sbl_console_init(sbl_console *c);
void sbl_console_reset(sbl_console *c);

/**
 * Feed one write. `owner` identifies the writer -- a thread id on a device -- and
 * exists because printf is not atomic: without it, two writers building a line at the
 * same time produce text belonging to neither.
 */
void sbl_console_write(sbl_console *c, const void *owner,
                       const char *data, size_t len,
                       sbl_console_fn out, void *user);

/** The level the line announces, and how many bytes of prefix to drop. */
uint8_t sbl_console_level(const char *line, uint16_t len, uint16_t *skip);

#ifdef __cplusplus
}
#endif

#endif /* SHARDBENCH_LINK_H */
