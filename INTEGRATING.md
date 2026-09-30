# Integrating shardbench-link

Faradex · how to make a firmware speak ShardBench Link.

`README.md` says what the library is and what it costs. This says what you have to write,
in the order you have to write it, with the mistakes that are worth not repeating. The
examples are real: they are the ones from the BMU-1ch integration, trimmed.

The whole integration is four things, and only the third is much work:

1. a **port** — four functions that know about your hardware
2. **bring-up** — construct, register, feed
3. **resources** — one handler per thing the board exposes
4. optionally, a **console bridge**, so existing `printf`s become log frames

---

## 1. The port

Everything the library needs from a platform is this:

```c
typedef struct {
    void     (*write)(void *io, const uint8_t *data, size_t len);
    void     (*lock)(void *io);
    void     (*unlock)(void *io);
    uint32_t (*tick_ms)(void *io);
    void      *io;
} sbl_port;
```

- **`write`** must put *every* byte on the wire before returning. A partial write
  truncates a frame, and a truncated frame is one the host throws away.
- **`lock` / `unlock`** are held around a whole frame. A frame interleaved with another
  is lost to both. If only one context ever sends, they may be empty — but read the
  warning below before deciding that.
- **`tick_ms`** is a free-running millisecond counter. Only differences matter, so it may
  wrap.

### A bare-metal port, no RTOS

```c
static void port_write(void *io, const uint8_t *data, size_t len)
{
    (void)io;
    for (size_t i = 0; i < len; i++) {
        while (!uart_tx_ready()) { }
        uart_tx_byte(data[i]);
    }
}

/* No threads here, but there is an interrupt that also sends -- so the lock is real. */
static void port_lock(void *io)   { (void)io; __disable_irq(); }
static void port_unlock(void *io) { (void)io; __enable_irq();  }

static uint32_t port_tick_ms(void *io) { (void)io; return g_systick_ms; }

static const sbl_port PORT = {
    port_write, port_lock, port_unlock, port_tick_ms, NULL
};
```

**Do not leave the lock empty just because you have no RTOS.** Ask instead: can two
things send at once? A timer interrupt that emits an event while the main loop is
answering a request is two things. If the answer is no, empty is honest; if you are not
sure, it is not.

`port/sbl_port_mbed.cpp` is the same four functions over `BufferedSerial` and an
`rtos::Mutex`, plus a receive thread. Read it as a worked example, not as a requirement.

---

## 2. Bringing it up

```c
static sbl_ctx ctx;

static const sbl_device_info INFO = {
    /* type_id    */ 0x0101,          /* which kind of board this is           */
    /* def_hash   */ 0xD964F0ADu,     /* which Definition describes it         */
    /* uid        */ {0},             /* filled from the MCU's unique id       */
    /* fw_version */ {1, 4, 0, 0},
    /* sdk_version*/ "1.2.3",
    /* app_slot   */ 0
};

void link_start(void)
{
    sbl_init(&ctx, &PORT, &INFO);
    register_resources();             /* section 3 */
}

/* Wherever bytes arrive. Replies are sent from inside this call. */
void on_uart_rx(const uint8_t *data, size_t len)
{
    sbl_feed(&ctx, data, len);
}
```

`sbl_ctx` is about 1.3 kB and the caller owns it. Nothing here allocates, ever.

### Two things about `sbl_device_info`

It must **outlive the context** — the context keeps a pointer, it does not copy. A local
is a bug that works until it doesn't.

`def_hash` is the fingerprint of the Definition's wire layout, and it is how the host
decides which Definition describes this board. **Do not type it by hand.** Generate it:

```sh
make def-hash                              # from the ShardBench repository
make def-hash CHECK=path/to/your/main.c    # verify a firmware still agrees
```

`--check` exits non-zero on a mismatch, so a build that runs it cannot ship a firmware
describing a Definition it no longer has. Without that, the only symptom of drift is the
session going UNMATCHED — which names neither side of the disagreement.

### Start it early

Start the link **before** anything that can block: a cable wait, a startup delay, a bus
retry loop. Any of those can hold a board indefinitely, and a bench that cannot see a
board stuck in one of them is no use. Started first, the host sees it and the log says
where it stopped.

---

## 3. Resources

A resource is one thing the board exposes, identified by a 16-bit id. Core ids `0x0001`
to `0x00FF` belong to the protocol (PING, INFO, TIME, STATUS, RESET, STREAM_CFG,
LOG_LEVEL) and are answered inside the library. **Yours start at `0x0100`.**

```c
sbl_add(&ctx, id, kinds, handler, user);
```

`kinds` is any of `SBL_CAN_GET | SBL_CAN_SET | SBL_CAN_CALL`. A request of a kind the
resource does not declare is refused before your handler ever sees it.

### A property (GET)

Payloads are packed little-endian structs, described by the Definition. There is no
schema on the wire — both sides already agree, which is what `def_hash` guarantees.

```c
static uint16_t put_f32(uint8_t *at, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));   /* not a cast: that is aliasing UB */
    at[0] = (uint8_t)(bits & 0xFF);
    at[1] = (uint8_t)((bits >> 8) & 0xFF);
    at[2] = (uint8_t)((bits >> 16) & 0xFF);
    at[3] = (uint8_t)(bits >> 24);
    return 4;
}

static sbl_action get_v_batt(sbl_request *req, void *user)
{
    (void)user;
    req->reply_len = put_f32(req->reply, read_battery_voltage());
    return SBL_REPLY;
}

sbl_add(&ctx, 0x0100, SBL_CAN_GET, get_v_batt, NULL);
```

Write bytes one at a time rather than memcpy'ing a struct: the wire is packed and
little-endian, and a struct is neither of those in any way C promises.

### A parameter (GET and SET)

Validate, and refuse with a code rather than clamping — a board that quietly accepts a
different value than it was given is worse than one that says no.

```c
static sbl_action period_param(sbl_request *req, void *user)
{
    volatile uint32_t *slot = (volatile uint32_t *)user;

    if (req->type == SBL_SET) {
        if (req->len != 4) {
            req->error = SBL_ERR_BAD_LEN;
            return SBL_REFUSE;
        }
        uint32_t value = get_u32(req->payload);
        if (value == 0) {
            req->error = SBL_ERR_BAD_VALUE;     /* would busy-loop whatever uses it */
            return SBL_REFUSE;
        }
        *slot = value;
        req->reply_len = 0;
        return SBL_REPLY;
    }
    req->reply_len = put_u32(req->reply, *slot);
    return SBL_REPLY;
}

sbl_add(&ctx, 0x0200, SBL_CAN_GET | SBL_CAN_SET, period_param, &g_period_s);
```

One handler, two resources, different `user` pointers: that is what `user` is for.

### A quick command (CALL)

If it finishes now, answer now. `SBL_REPLY` with `reply_len = 0` is a perfectly good
"done".

---

## 4. Long commands, and the rule that matters most

A handler that cannot answer immediately returns `SBL_DEFER`. The library sends ACCEPTED,
which frees the line, and the host keeps the sequence number open waiting for the result.

**Keep `req->seq`.** It is what the answer is matched to.

```c
static sbl_action call_measure(sbl_request *req, void *user)
{
    (void)user;
    lock();
    if (g_pending) {
        unlock();
        req->error = SBL_ERR_BUSY;
        return SBL_REFUSE;
    }
    g_pending = true;
    g_pending_seq = req->seq;
    g_pending_since_ms = tick_ms();
    unlock();

    start_measurement();
    return SBL_DEFER;
}
```

Later, from wherever the work finishes:

```c
sbl_resolve(&ctx, 0x0300, seq, payload, len);   /* it worked */
sbl_fail(&ctx, 0x0300, seq, SBL_ERR_HW_FAIL);   /* it did not */
```

### Three rules, each of which cost a day

**A deferred CALL must always end.** If nothing resolves it, the resource is wedged: every
later call is refused BUSY and only a reset clears it. Carry a deadline and answer
`SBL_ERR_TIMEOUT` when it passes — a failure is always better than silence.

**The deadline must be below the host's timeout.** The Definition declares how long the
host will wait. If the board's deadline is longer, the host gives up first while the board
still believes the command is running, and a retry in that gap is refused BUSY for reasons
the operator has no way to see. Shorter, and the host always receives a real error code
instead of a timeout.

**Do not infer completion by polling a transient state.** The first BMU integration
watched for the board leaving its `RESISTANCE_MEAS` state. That state lived about 200 ms
and the poller ran every 100 ms — so it usually worked, and occasionally did not, and when
it did not the command never ended. If the code doing the work knows when it finished,
have it say so with a callback. If it genuinely cannot, poll *and* keep the deadline.

**And report a failure as a failure.** A measurement that bailed out early usually leaves
the previous result in place. Answering with it hands back stale numbers as though they
were fresh — worse than an error, because nothing looks wrong.

---

## 5. Events

Something happened that nobody asked about:

```c
uint8_t payload[4];
put_u32(payload, alarms);
sbl_emit(&ctx, 0x0401, payload, 4);
```

Events carry sequence number 0, which is reserved for exactly this. If a board has no
callback for the thing you want to report, a small polling thread that emits on *change*
is fine — that is what BMU-1ch does for state and alarms. Polling for a **change** is safe;
polling for a transient **state** is the trap in section 4.

`sbl_set_state(&ctx, state, alarms)` updates what STATUS reports. That is what the host
shows in its session indicator, and it is separate from emitting an event about it.

---

## 6. Streams

For sampled data: many values in one frame, sent as they are produced.

```c
sbl_stream(&ctx, id, tick_ms, period_us, samples, count, sample_bytes);
```

The library writes the header the host expects — `tick_ms u32, period_us u32, count u16`
— in front of your samples. You provide only the samples.

Three things are easy to get wrong, and all three were:

**`tick_ms` is the first sample's time, not the send time.** The host spaces the rest out
from there by `period_us`. Stamping at send puts the whole batch late by however long it
took to fill.

**Measure `period_us`, do not declare it.** Unless a timer drives your sampling, the rate
is whatever the hardware and the scheduler allow, and a constant is a guess the host will
plot as fact. Record the time of the first sample and of the last, and divide by the gaps
between them — `count - 1`, not `count`.

**A batch may be flushed long after its last sample.** If a timer flushes short batches,
measuring the span at flush time counts the idle wait as if samples were spread across it.
Track the time of the last *sample*.

```c
/* Called when the batch is full, or by a timer for a short one. Lock held. */
static void flush_locked(void)
{
    if (g_filled == 0) return;

    uint32_t gaps = g_filled - 1u;
    /* A lone sample has no gap to measure. Zero tells the host it stands alone. */
    uint32_t period_us = gaps ? (g_last_sample_us - g_first_sample_us) / gaps : 0u;

    sbl_stream(&ctx, ID_RAW, g_first_tick_ms, period_us,
               g_batch, g_filled, (uint8_t)sizeof(int16_t));
    g_filled = 0;
}
```

### Decimation is yours to apply

The host asks for one sample in N through STREAM_CFG. The library records the request; the
application honours it:

```c
const sbl_stream_cfg *cfg = sbl_stream_config(&ctx, ID_RAW);
if (cfg == NULL || !cfg->enabled) return;
if (cfg->decimation > 1) {
    if (++g_skipped < cfg->decimation) return;   /* dropped here, never sent */
    g_skipped = 0;
}
```

Dropping on the device is the whole point of asking: what is decimated never costs any
bandwidth. Do not send everything and let the host thin it.

Size the batch so that `count * sample_bytes + 10` stays within `SBL_MAX_PAYLOAD` (512).
Sixty-four `int16_t` is 138 bytes, comfortable. A batch that does not fit is **dropped
whole** and counted, not truncated — a half-sent batch would decode into samples that were
never taken.

---

## 7. The console bridge

If the project already logs with `printf`, every line can become a LOG frame without
touching a single call site. That includes libraries you only have as binaries.

On Mbed it is one function:

```cpp
FileHandle *mbed::mbed_override_console(int fd) { return &link.console(); }
```

Elsewhere, hook whatever your platform calls for stdout — `_write` on newlib — and feed
`sbl_console_write()`. It assembles lines per writer, parses the level out of a
`[ERROR]`/`[WARNING]`/`[INFO]`/`[DEBUG]` prefix, and hands each finished line to you.

**Before the link is up, and during a crash, write the raw text to the same port.** A
board that dies early must not be silent.

### Unbuffered stdout is not optional

```c
setvbuf(stdout, NULL, _IONBF, 0);
```

newlib keeps **one** stdout buffer shared by every thread that prints. Two threads inside
`printf` at once corrupt it — on BMU-1ch, one line in twelve arrived damaged, with another
thread's text spliced into it. Line buffering made it **worse**, by creating a shared
buffer where the damage had been smaller. With no buffer there is nothing to corrupt, and
the per-writer assembly in `sbl_console` puts the lines back together. The cost is more
UART writes, which on a debug console is nothing.

This is worth doing in production firmware too. The corruption is there either way; on a
terminal it is just easier not to notice.

---

## 8. Configuration

Everything in `include/sbl_config.h` is overridable with `-D`:

| | default | |
|---|---|---|
| `SBL_MAX_PAYLOAD` | 512 | protocol maximum; lowering it saves RAM on both buffers |
| `SBL_MAX_RESOURCES` | 24 | the registration table |
| `SBL_MAX_STREAMS` | 4 | how many stream configs are remembered |
| `SBL_CONSOLE_SLOTS` | 4 | concurrent printing threads before lines can interleave |
| `SBL_CONSOLE_LINE` | 160 | longest log line kept whole |
| `SBL_CONSOLE_TRACE` | 0 | report every write stdout makes — bring-up only |
| `SBL_LINK_STATS` | 0 | ms between self-reports; see below |

---

## 9. Bring-up

In the order that finds problems fastest.

**Listen before transmitting.** `tools/probe.py <port> --listen` sends nothing, so it
cannot disturb anything, and it separates the four failure modes:

| what you see | what it is |
|---|---|
| nothing at all | wiring, baud, or a board that never reached `main()` |
| bytes, no delimiters | almost always the wrong baud rate |
| delimiters, bad CRC | a baud close but not exact, or a noisy line |
| frames that decode | the link is fine — a terminal was showing you COBS |

**Then the handshake.** `tools/probe.py <port>` runs it and reports attempts per request.
All ones is what healthy looks like.

**When something is wrong, instrument at the second failed hypothesis, not the fifth.**
Two flags exist for exactly this and both pay for themselves in minutes:

`-D SBL_LINK_STATS=5000` makes the board report every five seconds how many requests it
decoded and the longest it took to answer one. `rx` climbing with `slowest=0ms` while the
host sees a second of delay proves the board innocent and sends you to look at the line.
That is how a four-channel USB-serial adapter — which was delaying frames by up to 1.5 s —
was finally caught, after six firmware hypotheses had been tested and disproved.

`-D SBL_CONSOLE_TRACE=1` reports every write stdout makes, before the library touches it.
It settled the stdout corruption above in ten minutes, after an afternoon of guessing, by
showing the text already damaged on arrival.

**Suspect the cable early.** It is not usually the cable. But it costs one minute to rule
out and an afternoon not to.

---

## 10. Testing without hardware

`src/` compiles on a PC — no platform underneath it — so the host test suite compiles
these exact sources and drives them through `ctypes`, checking the bytes against the
host's own implementation. See `backend/tests/test_firmware_link.py` in the ShardBench
repository: 60-odd tests, including the full session handshake run against the C library
through an in-memory transport.

That is the point of the no-platform rule. Wire compatibility stops being a claim about
two implementations agreeing and becomes a test of the code that will actually run.

**If you add to the protocol surface, add the cross-check in the same commit.** A stream
test once compared the payload the library produced with the samples that went in — with
itself, in other words — and passed happily while the firmware was sending batches with no
header at all. Every frame was rejected on the bench and the test said nothing. Compare
against the *host's* decoder, never against your own input.

---

## 11. Checklist

- [ ] port: `write` sends everything; `lock`/`unlock` real if two contexts can send
- [ ] `sbl_device_info` outlives the context
- [ ] `def_hash` generated, not typed, and checked by the build
- [ ] link started before anything that can block
- [ ] resource ids from `0x0100`; `kinds` matches what each handler accepts
- [ ] every `SBL_DEFER` path resolves or fails, and has a deadline below the host's
- [ ] failures answered with `sbl_fail`, never with the previous result
- [ ] stream `tick_ms` is the first sample's; `period_us` measured over `count - 1`
- [ ] decimation applied on the device
- [ ] `setvbuf(stdout, NULL, _IONBF, 0)` if anything shares stdout
- [ ] raw-text fallback before the link starts and during a crash
- [ ] cross-checks against the host decoder for anything new on the wire
