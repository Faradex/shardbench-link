/*
 * The link: what to do with a frame once it has been decoded.
 *
 * Core resources are answered here, because every firmware answers them identically and
 * the handshake depends on it. Everything from 0x0100 up belongs to the device and goes
 * to a handler the application registered, which is what keeps this file free of any
 * knowledge about what a BMU is.
 *
 * Sending is serialised by the port's lock rather than by a queue. A queue would need a
 * worker, a buffer sized for the worst burst, and a policy for what to drop when it
 * fills; the lock costs one mutex and cannot lose anything.
 */
#include "sbl_internal.h"

static void put_u16(uint8_t *at, uint16_t v)
{
    at[0] = (uint8_t)(v & 0xFFu);
    at[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *at, uint32_t v)
{
    at[0] = (uint8_t)(v & 0xFFu);
    at[1] = (uint8_t)((v >> 8) & 0xFFu);
    at[2] = (uint8_t)((v >> 16) & 0xFFu);
    at[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *at)
{
    return (uint16_t)((uint16_t)at[0] | ((uint16_t)at[1] << 8));
}

static uint16_t text_len(const char *s, uint16_t cap)
{
    uint16_t n = 0;
    while (s && s[n] && n < cap) {
        n++;
    }
    return n;
}

/* --- sending --------------------------------------------------------------- */

static void send_parts(sbl_ctx *ctx, uint8_t type, uint16_t id, uint8_t seq,
                       uint8_t flags, const uint8_t *prefix, uint16_t prefix_len,
                       const uint8_t *payload, uint16_t len)
{
    sbl_frame frame;
    long written;

    frame.ver = SBL_VERSION;
    frame.addr = SBL_ADDR_P2P;
    frame.type = type;
    frame.flags = flags;
    frame.seq = seq;
    frame.id = id;
    frame.len = (uint16_t)(prefix_len + len);
    frame.payload = payload;

    ctx->port->lock(ctx->port->io);
    written = sbl_encode_parts(&frame, prefix, prefix_len, ctx->tx, sizeof(ctx->tx));
    if (written > 0) {
        ctx->port->write(ctx->port->io, ctx->tx, (size_t)written);
        ctx->sent++;
    } else {
        /* Only a payload over the limit gets here, which is a bug in the caller
           rather than anything the wire did. Counted, not hidden. */
        ctx->dropped++;
    }
    ctx->port->unlock(ctx->port->io);
}

static void send(sbl_ctx *ctx, uint8_t type, uint16_t id, uint8_t seq, uint8_t flags,
                 const uint8_t *payload, uint16_t len)
{
    send_parts(ctx, type, id, seq, flags, NULL, 0, payload, len);
}

static void send_err(sbl_ctx *ctx, uint16_t id, uint8_t seq, uint8_t code)
{
    send(ctx, SBL_ERR, id, seq, SBL_FLAG_NONE, &code, 1);
}

void sbl_fail(sbl_ctx *ctx, uint16_t id, uint8_t seq, uint8_t code)
{
    send(ctx, SBL_ERR, id, seq, SBL_FLAG_ASYNC, &code, 1);
}

void sbl_emit(sbl_ctx *ctx, uint16_t id, const uint8_t *payload, uint16_t len)
{
    send(ctx, SBL_EVT, id, 0, SBL_FLAG_NONE, payload, len);
}

void sbl_resolve(sbl_ctx *ctx, uint16_t id, uint8_t seq,
                 const uint8_t *payload, uint16_t len)
{
    /* The seq of the CALL that started it: that is how the host matches the answer
       to the request it is still holding open. */
    send(ctx, SBL_EVT, id, seq, SBL_FLAG_ASYNC, payload, len);
}

void sbl_stream(sbl_ctx *ctx, uint16_t id, uint32_t tick_ms, uint32_t period_us,
                const void *samples, uint16_t count, uint8_t sample_bytes)
{
    uint8_t head[SBL_STREAM_HEADER_SIZE];
    uint32_t bytes = (uint32_t)count * (uint32_t)sample_bytes;

    /* The samples are the application's; the header in front of them is the protocol's,
       and belongs here so there is one place that can get it wrong. */
    if (bytes + SBL_STREAM_HEADER_SIZE > SBL_MAX_PAYLOAD) {
        ctx->dropped++;
        return;
    }

    put_u32(&head[0], tick_ms);
    put_u32(&head[4], period_us);
    put_u16(&head[8], count);

    send_parts(ctx, SBL_STREAM, id, 0, SBL_FLAG_NONE,
               head, SBL_STREAM_HEADER_SIZE,
               (const uint8_t *)samples, (uint16_t)bytes);
}

void sbl_log(sbl_ctx *ctx, uint8_t level, const char *text, uint16_t len)
{
    uint8_t line[SBL_MAX_PAYLOAD];
    uint16_t i;

    if (level == 0 || level > ctx->log_level) {
        return;      /* the host asked for a quieter board */
    }
    if (len > (uint16_t)(SBL_MAX_PAYLOAD - 1)) {
        len = (uint16_t)(SBL_MAX_PAYLOAD - 1);
    }
    line[0] = level;
    for (i = 0; i < len; i++) {
        line[1 + i] = (uint8_t)text[i];
    }
    send(ctx, SBL_LOG, 0, 0, SBL_FLAG_NONE, line, (uint16_t)(len + 1));
}

/* --- core resources -------------------------------------------------------- */

static uint32_t uptime_ms(const sbl_ctx *ctx)
{
    /* Unsigned subtraction, so a wrapping tick counter still gives the right span. */
    return ctx->port->tick_ms(ctx->port->io) - ctx->started_ms;
}

static uint16_t build_info(const sbl_ctx *ctx, uint8_t *out)
{
    const sbl_device_info *info = ctx->info;
    uint16_t sdk_len = text_len(info->sdk_version, 64);
    uint16_t at = 0;
    uint16_t i;

    put_u16(&out[at], info->type_id);      at = (uint16_t)(at + 2);
    put_u32(&out[at], info->def_hash);     at = (uint16_t)(at + 4);
    for (i = 0; i < 12; i++) {
        out[at++] = info->uid[i];
    }
    for (i = 0; i < 4; i++) {
        out[at++] = info->fw_version[i];
    }
    out[at++] = (uint8_t)sdk_len;
    for (i = 0; i < sdk_len; i++) {
        out[at++] = (uint8_t)info->sdk_version[i];
    }
    out[at++] = SBL_VERSION;
    out[at++] = info->app_slot;
    return at;
}

static sbl_stream_cfg *stream_slot(sbl_ctx *ctx, uint16_t id)
{
    uint8_t i;

    for (i = 0; i < ctx->stream_count; i++) {
        if (ctx->streams[i].id == id) {
            return &ctx->streams[i];
        }
    }
    if (ctx->stream_count >= SBL_MAX_STREAMS) {
        return NULL;
    }
    ctx->streams[ctx->stream_count].id = id;
    ctx->streams[ctx->stream_count].enabled = 0;
    ctx->streams[ctx->stream_count].decimation = 1;
    return &ctx->streams[ctx->stream_count++];
}

const sbl_stream_cfg *sbl_stream_config(const sbl_ctx *ctx, uint16_t id)
{
    uint8_t i;

    for (i = 0; i < ctx->stream_count; i++) {
        if (ctx->streams[i].id == id) {
            return &ctx->streams[i];
        }
    }
    return NULL;
}

/** Returns 1 when the frame was a core resource and has been answered. */
static int handle_core(sbl_ctx *ctx, const sbl_frame *f)
{
    uint8_t out[128];

    switch (f->id) {
    case SBL_ID_PING:
        if (f->type != SBL_CALL) {
            break;
        }
        /* Echoed back exactly: the host times the round trip with it. */
        send(ctx, SBL_RSP, f->id, f->seq, SBL_FLAG_NONE, f->payload, f->len);
        return 1;

    case SBL_ID_INFO:
        if (f->type != SBL_GET) {
            break;
        }
        send(ctx, SBL_RSP, f->id, f->seq, SBL_FLAG_NONE, out, build_info(ctx, out));
        return 1;

    case SBL_ID_TIME:
        if (f->type != SBL_GET) {
            break;
        }
        put_u32(out, uptime_ms(ctx));
        send(ctx, SBL_RSP, f->id, f->seq, SBL_FLAG_NONE, out, 4);
        return 1;

    case SBL_ID_STATUS:
        if (f->type != SBL_GET) {
            break;
        }
        out[0] = ctx->state;
        put_u32(&out[1], ctx->alarms);
        put_u32(&out[5], uptime_ms(ctx) / 1000u);
        send(ctx, SBL_RSP, f->id, f->seq, SBL_FLAG_NONE, out, 9);
        return 1;

    case SBL_ID_LOG_LEVEL:
        if (f->type != SBL_SET) {
            break;
        }
        if (f->len < 1) {
            send_err(ctx, f->id, f->seq, SBL_ERR_BAD_LEN);
            return 1;
        }
        ctx->log_level = f->payload[0];
        send(ctx, SBL_RSP, f->id, f->seq, SBL_FLAG_NONE, NULL, 0);
        return 1;

    case SBL_ID_STREAM_CFG: {
        sbl_stream_cfg *cfg;

        if (f->type != SBL_SET) {
            break;
        }
        if (f->len < 5) {                      /* u16 id, u8 enable, u16 decimation */
            send_err(ctx, f->id, f->seq, SBL_ERR_BAD_LEN);
            return 1;
        }
        cfg = stream_slot(ctx, get_u16(&f->payload[0]));
        if (cfg == NULL) {
            send_err(ctx, f->id, f->seq, SBL_ERR_BUSY);
            return 1;
        }
        cfg->enabled = f->payload[2] ? 1 : 0;
        cfg->decimation = get_u16(&f->payload[3]);
        if (cfg->decimation == 0) {
            cfg->decimation = 1;               /* one in nothing is not a rate */
        }
        send(ctx, SBL_RSP, f->id, f->seq, SBL_FLAG_NONE, NULL, 0);
        return 1;
    }

    default:
        return 0;
    }

    /* A core id exists but not for this message type. */
    send_err(ctx, f->id, f->seq, SBL_ERR_BAD_TYPE);
    return 1;
}

/* --- dispatch -------------------------------------------------------------- */

static uint8_t kind_of(uint8_t type)
{
    if (type == SBL_GET) {
        return SBL_CAN_GET;
    }
    if (type == SBL_SET) {
        return SBL_CAN_SET;
    }
    if (type == SBL_CALL) {
        return SBL_CAN_CALL;
    }
    return 0;
}

/**
 * How long this took, from the frame being decoded to its answer being written.
 *
 * Measured on the device because measuring from the host cannot tell a slow board from
 * a slow line: both look like a late reply.
 */
static void note_time(sbl_ctx *ctx, uint32_t started)
{
    uint32_t took = ctx->port->tick_ms(ctx->port->io) - started;

    ctx->received++;
    if (took > ctx->slowest_ms) {
        ctx->slowest_ms = took;
    }
}

static void dispatch(const sbl_frame *f, void *user)
{
    sbl_ctx *ctx = (sbl_ctx *)user;
    uint8_t kind = kind_of(f->type);
    uint32_t started = ctx->port->tick_ms(ctx->port->io);
    uint8_t i;

    /* Anything that is not a request is the host talking out of turn: a device never
       has to interpret RSP, EVT or STREAM, and quietly ignoring them keeps a confused
       host from being able to confuse the board as well. */
    if (kind == 0) {
        return;
    }
    if (f->ver != SBL_VERSION) {
        send_err(ctx, f->id, f->seq, SBL_ERR_UNSUPPORTED_VER);
        return;
    }
    if (f->id < 0x0100u) {
        if (handle_core(ctx, f)) {
            note_time(ctx, started);
            return;
        }
    }

    for (i = 0; i < ctx->resources; i++) {
        sbl_resource *res = &ctx->table[i];
        sbl_request req;
        sbl_action action;

        if (res->id != f->id) {
            continue;
        }
        if ((res->kinds & kind) == 0) {
            send_err(ctx, f->id, f->seq, SBL_ERR_BAD_TYPE);
            return;
        }

        req.type = f->type;
        req.seq = f->seq;
        req.id = f->id;
        req.payload = f->payload;
        req.len = f->len;
        req.reply = ctx->reply;
        req.reply_cap = (uint16_t)sizeof(ctx->reply);
        req.reply_len = 0;
        req.error = SBL_ERR_HW_FAIL;

        action = res->fn(&req, res->user);
        if (action == SBL_REPLY) {
            send(ctx, SBL_RSP, f->id, f->seq, SBL_FLAG_NONE, req.reply, req.reply_len);
        } else if (action == SBL_DEFER) {
            /* The host starts a timer on this and waits for the EVT. */
            send(ctx, SBL_ACCEPTED, f->id, f->seq, SBL_FLAG_NONE, NULL, 0);
        } else {
            send_err(ctx, f->id, f->seq, req.error);
        }
        note_time(ctx, started);
        return;
    }

    send_err(ctx, f->id, f->seq, SBL_ERR_UNKNOWN_ID);
    note_time(ctx, started);
}

/* --- lifecycle ------------------------------------------------------------- */

void sbl_init(sbl_ctx *ctx, const sbl_port *port, const sbl_device_info *info)
{
    ctx->port = port;
    ctx->info = info;
    ctx->resources = 0;
    ctx->stream_count = 0;
    ctx->log_level = SBL_LOG_INFO;
    ctx->state = 0;
    ctx->alarms = 0;
    ctx->sent = 0;
    ctx->dropped = 0;
    ctx->received = 0;
    ctx->slowest_ms = 0;
    ctx->started_ms = port->tick_ms(port->io);
    sbl_decoder_init(&ctx->decoder, dispatch, ctx);
}

int sbl_add(sbl_ctx *ctx, uint16_t id, uint8_t kinds, sbl_handler_fn fn, void *user)
{
    if (ctx->resources >= SBL_MAX_RESOURCES) {
        return SBL_E_TOO_BIG;
    }
    ctx->table[ctx->resources].id = id;
    ctx->table[ctx->resources].kinds = kinds;
    ctx->table[ctx->resources].fn = fn;
    ctx->table[ctx->resources].user = user;
    ctx->resources++;
    return SBL_OK;
}

void sbl_feed(sbl_ctx *ctx, const uint8_t *data, size_t len)
{
    sbl_decoder_feed(&ctx->decoder, data, len);
}

void sbl_set_state(sbl_ctx *ctx, uint8_t state, uint32_t alarms)
{
    ctx->state = state;
    ctx->alarms = alarms;
}
