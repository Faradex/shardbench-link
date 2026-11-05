#include "sbl_port_mbed.h"

#include <stdio.h>

namespace sbl {

/*
 * No buffer for stdout, deliberately -- and this is the opposite of what it looks like.
 *
 * newlib keeps one buffer for stdout, shared by every thread that prints. Two threads
 * in printf at once corrupt it: the tracing build showed a 62-byte write arriving cut
 * off mid-word, "[WARNING][BMUSensing]: Positive temperature s", with another thread's
 * line spliced on after it. The damage is done before any of this code is reached, so
 * no amount of care in the FileHandle can undo it.
 *
 * Unbuffered, newlib has nothing to corrupt: every character is handed straight over,
 * and sbl_console then reassembles the lines per writing thread, which it is tested to
 * do correctly. Buffering here would be an optimisation that costs correctness.
 */

/** Bytes pulled off the UART per read. A STREAM frame is a few hundred. */
static const size_t RX_CHUNK = 128;
/** Enough for the link's own work; the application's handlers run on this stack too. */
static const uint32_t RX_STACK = 2048;
/** The flag that wakes the link thread: the UART has bytes, or the CAN tunnel has. */
static const uint32_t WAKE = 1u;
/** The flag that wakes the CAN sending thread: there is something to send. */
static const uint32_t CAN_TX = 2u;
/** The CAN sending thread's stack. Not small: the SDK's CAN driver prints an error with
    printf when no mailbox is free, and newlib's printf is deep. At 1.5 kB that overflowed
    and corrupted the heap beside it -- the board reset a moment after each answer, and
    the UART repeated one log line for ever (bench, 2026-11-05). Mbed's own default. */
static const uint32_t CAN_TX_STACK = 4096;
static const size_t CAN_TX_CAPACITY = 1024;
/** CAN frames sent back to back before letting the controller's three mailboxes drain. */
static const unsigned CAN_BURST = 3;
/** How often, and how many times, a CAN frame is offered again when the driver has no
    room: 100 us apart, 20 ms in all, then it is dropped and the host's retry recovers.
    Short, because an adapter that keeps one frame in the controller at a time (to keep
    them in order) says "no room" for every frame still on the wire. */
static const unsigned CAN_SEND_RETRY_US = 100;
static const unsigned CAN_SEND_TRIES = 200;

// --- the console ------------------------------------------------------------

ConsoleToLog::ConsoleToLog(mbed::BufferedSerial &serial)
    : _serial(serial), _ctx(nullptr)
{
    sbl_console_init(&_console);
}

void ConsoleToLog::attach(sbl_ctx *ctx)
{
    _lock.lock();
    _ctx = ctx;
    sbl_console_reset(&_console);   /* half-written text belongs to the raw era */
    _lock.unlock();
}

void ConsoleToLog::on_line(uint8_t level, const char *text, uint16_t len, void *user)
{
    sbl_log(static_cast<sbl_ctx *>(user), level, text, len);
}

#if SBL_CONSOLE_TRACE
/**
 * Report every write stdout hands over, before anything is done with it.
 *
 * Temporary, and deliberately crude: it answers one question -- what does printf
 * actually deliver, and in what pieces -- which no amount of reasoning about newlib
 * has managed to settle.
 */
void ConsoleToLog::trace(const char *data, size_t size)
{
    static const char HEX[] = "0123456789abcdef";
    char out[96];
    size_t n = 0;
    size_t i;

    out[n++] = 'w';
    out[n++] = HEX[(size >> 4) & 0xF];
    out[n++] = HEX[size & 0xF];
    out[n++] = ':';
    for (i = 0; i < size && n + 2 < sizeof(out); i++) {
        out[n++] = HEX[((unsigned char)data[i] >> 4) & 0xF];
        out[n++] = HEX[(unsigned char)data[i] & 0xF];
    }
    sbl_log(_ctx, SBL_LOG_INFO, out, (uint16_t)n);
}
#endif

ssize_t ConsoleToLog::write(const void *buffer, size_t size)
{
    /* Before the link exists, and from an interrupt, straight to the port. Mbed's
       error handler prints the crash through here, and it must survive. */
    if (_ctx == nullptr || core_util_is_isr_active() || osThreadGetId() == _raw_thread) {
        return _serial.write(buffer, size);
    }

#if SBL_CONSOLE_TRACE
    trace(static_cast<const char *>(buffer), size);
#endif

    _lock.lock();
    /* The thread id is what tells one printf from another: they interleave, because
       printf is not atomic and this console is not buffered. */
    sbl_console_write(&_console, osThreadGetId(),
                      static_cast<const char *>(buffer), size,
                      &ConsoleToLog::on_line, _ctx);
    _lock.unlock();
    return static_cast<ssize_t>(size);
}

ssize_t ConsoleToLog::read(void *, size_t)
{
    /* stdin does not exist: the port belongs to the protocol. Reporting end of file
       rather than blocking keeps anything that reads stdin from hanging the board. */
    return 0;
}

off_t ConsoleToLog::seek(off_t, int)
{
    return -ESPIPE;
}

int ConsoleToLog::close()
{
    return 0;
}

int ConsoleToLog::isatty()
{
    return 1;
}

// --- the port ---------------------------------------------------------------

void Link::port_write(void *io, const uint8_t *data, size_t len)
{
    Link *self = static_cast<Link *>(io);
    if (self->_can != nullptr && self->_channel == CAN) {
        if (self->can_attached()) {
            /* Called under the link's transmit lock, so one frame's bytes are queued
               together. A frame that does not fit is dropped whole -- a cut one would
               only be dropped by the host's decoder -- and the host's retry recovers. */
            if (CAN_TX_CAPACITY - self->_can_tx.size() < len) {
                return;
            }
            for (size_t i = 0; i < len; i++) {
                self->_can_tx.push(data[i]);
            }
            self->_wake.set(CAN_TX);
            return;
        }
        self->_channel = UART;           /* the host on the tunnel stopped listening */
    }
    self->_serial.write(data, len);
}

void Link::port_lock(void *io)
{
    static_cast<Link *>(io)->_tx_lock.lock();
}

void Link::port_unlock(void *io)
{
    static_cast<Link *>(io)->_tx_lock.unlock();
}

uint32_t Link::port_tick_ms(void *)
{
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(rtos::Kernel::Clock::now().time_since_epoch()).count());
}

Link::Link(mbed::BufferedSerial &serial, const sbl_device_info &info)
    : _serial(serial), _console(serial), _thread(osPriorityAboveNormal, RX_STACK),
      _running(false), _can(nullptr), _channel(UART),
      _can_thread(osPriorityAboveNormal, CAN_TX_STACK)
{
    sbl_tunnel_init(&_tunnel);
    _port.write = &Link::port_write;
    _port.lock = &Link::port_lock;
    _port.unlock = &Link::port_unlock;
    _port.tick_ms = &Link::port_tick_ms;
    _port.io = this;
    sbl_init(&_ctx, &_port, &info);
}

void Link::start(osPriority priority)
{
    if (_running) {
        return;
    }
    _running = true;
    _thread.set_priority(priority);
    _thread.start(mbed::callback(this, &Link::rx_thread));
    if (_can != nullptr) {
        _can_thread.start(mbed::callback(this, &Link::can_tx_thread));
    }
    /* Only now: a log frame sent before the thread exists would still go out, but the
       host would have nothing listening for a board it has not probed yet. */
    _console.attach(&_ctx);

    /* Forced, not merely left alone: what Mbed leaves stdout set to varies, and a
       shared buffer is what breaks the log. See the note at the top of this file. */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* The marker goes out from the thread, a moment from now: sent here it would land
       in the line transient that follows a reset, which is where the first frame of
       every session has been disappearing. */
}

// --- the CAN tunnel (D-7) ----------------------------------------------------

void Link::attach_can(CanBus &bus)
{
    _can = &bus;
}

void Link::can_received(const uint8_t *data, uint8_t len)
{
    uint8_t bytes[SBL_TUNNEL_FRAME_MAX];
    _tunnel_lock.lock();
    size_t n = sbl_tunnel_receive(&_tunnel, data, len, port_tick_ms(nullptr), bytes);
    _tunnel_lock.unlock();
    for (size_t i = 0; i < n; i++) {
        /* Full means the link thread has fallen behind by half a kilobyte; what is lost
           is a frame the host will ask for again. */
        if (_can_rx.full()) {
            break;
        }
        _can_rx.push(bytes[i]);
    }
    /* A keepalive carries no bytes but still matters: it keeps the node attached. */
    if (n > 0) {
        _wake.set(WAKE);
    }
}

bool Link::can_attached()
{
    _tunnel_lock.lock();
    bool attached = sbl_tunnel_attached(&_tunnel, port_tick_ms(nullptr)) != 0;
    _tunnel_lock.unlock();
    return attached;
}

void Link::can_tx_thread()
{
    uint8_t chunk[SBL_TUNNEL_CHUNK];
    uint8_t frame[SBL_TUNNEL_FRAME_MAX];
    unsigned burst = 0;

    /* This thread's own output -- the CAN driver printing an error from inside send()
       -- goes to the UART as raw text: through the link it would come back here. */
    _console.raw_for(osThreadGetId());
    while (_running) {
        if (_can_tx.empty()) {
            burst = 0;
            _wake.wait_any_for(CAN_TX, std::chrono::milliseconds(100));
            continue;
        }
        if (!can_attached()) {
            _can_tx.reset();             /* nobody is listening any more */
            continue;
        }
        size_t n = 0;
        while (n < sizeof(chunk) && _can_tx.pop(chunk[n])) {
            n++;
        }
        size_t used;
        _tunnel_lock.lock();
        size_t len = sbl_tunnel_next_frame(&_tunnel, chunk, n, frame, &used);
        _tunnel_lock.unlock();
        unsigned tries = 0;
        while (!_can->send(SBL_TUNNEL_TO_HOST, frame, static_cast<uint8_t>(len))) {
            if (++tries >= CAN_SEND_TRIES) {
                break;                   /* lost: the host's retry recovers it */
            }
            wait_us(CAN_SEND_RETRY_US);
        }
        if (++burst >= CAN_BURST) {
            burst = 0;
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Link::on_serial_event()
{
    _wake.set(WAKE);                     /* from an interrupt: EventFlags allow it */
}

void Link::feed(Channel from, const uint8_t *data, size_t len)
{
    _rx_lock.lock();
    if (from != _channel) {
        /* Half a frame from one channel must never be completed by bytes from the
           other: a lone 0x00 makes the decoder drop whatever it was holding. */
        static const uint8_t delimiter = 0x00;
        sbl_feed(&_ctx, &delimiter, 1);
        _channel = from;
    }
    sbl_feed(&_ctx, data, len);
    _rx_lock.unlock();
}

void Link::rx_thread()
{
    uint8_t chunk[RX_CHUNK];

    _serial.sigio(mbed::callback(this, &Link::on_serial_event));

    /* Past the reset transient, and sent with sbl_log directly rather than through the
       console: if this arrives while ordinary log lines are still damaged, the fault is
       in the console and not in the link underneath it. */
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(300));
    sbl_log(&_ctx, SBL_LOG_INFO, "[shardbench-link] " SBL_BUILD_MARKER,
            (uint16_t)(sizeof("[shardbench-link] " SBL_BUILD_MARKER) - 1));

#if SBL_LINK_STATS
    uint32_t next_report = port_tick_ms(nullptr) + SBL_LINK_STATS;
#endif

    while (_running) {
#if SBL_LINK_STATS
        /* A count of what has been asked and how long the slowest answer took, seen
           from inside. The host cannot tell a slow board from a slow line; this can,
           and it is what proved the board innocent when a USB bridge was delaying
           replies by a second. Off by default: it is noise on a working bench. */
        if ((int32_t)(port_tick_ms(nullptr) - next_report) >= 0) {
            char note[64];
            int n = snprintf(note, sizeof(note), "[link] rx=%lu tx=%lu slowest=%lums",
                             (unsigned long)_ctx.received, (unsigned long)_ctx.sent,
                             (unsigned long)_ctx.slowest_ms);
            if (n > 0) {
                sbl_log(&_ctx, SBL_LOG_INFO, note, (uint16_t)n);
            }
            _ctx.slowest_ms = 0;
            next_report += SBL_LINK_STATS;
        }
#endif

        /* Both ways in, from one thread: handlers run here, on this stack, whichever
           channel the request came by -- so they must not block for long. */
        bool worked = false;
        size_t m = 0;
        while (m < sizeof(chunk) && _can_rx.pop(chunk[m])) {
            m++;
        }
        if (m > 0) {
            feed(CAN, chunk, m);
            worked = true;
        }
        if (!_serial.readable()) {
            if (!worked) {
                _wake.wait_any_for(WAKE, std::chrono::milliseconds(50));
            }
            continue;
        }
        ssize_t n = _serial.read(chunk, sizeof(chunk));
        if (n > 0) {
            feed(UART, chunk, static_cast<size_t>(n));
        } else if (n < 0) {
            /* A read error on a UART is not something a device can fix. Pause rather
               than spin, and carry on: the line may come back when a cable does. */
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// --- wrappers ---------------------------------------------------------------

void Link::add(uint16_t id, uint8_t kinds, sbl_handler_fn fn, void *user)
{
    sbl_add(&_ctx, id, kinds, fn, user);
}

void Link::emit(uint16_t id, const void *payload, uint16_t len)
{
    sbl_emit(&_ctx, id, static_cast<const uint8_t *>(payload), len);
}

void Link::resolve(uint16_t id, uint8_t seq, const void *payload, uint16_t len)
{
    sbl_resolve(&_ctx, id, seq, static_cast<const uint8_t *>(payload), len);
}

void Link::fail(uint16_t id, uint8_t seq, uint8_t code)
{
    sbl_fail(&_ctx, id, seq, code);
}

void Link::stream(uint16_t id, uint32_t tick_ms, uint32_t period_us,
                  const void *samples, uint16_t count, uint8_t sample_bytes)
{
    sbl_stream(&_ctx, id, tick_ms, period_us, samples, count, sample_bytes);
}

void Link::set_state(uint8_t state, uint32_t alarms)
{
    sbl_set_state(&_ctx, state, alarms);
}

uint32_t Link::tick_ms() const
{
    return port_tick_ms(nullptr);
}

const sbl_stream_cfg *Link::stream_config(uint16_t id) const
{
    return sbl_stream_config(&_ctx, id);
}

}  // namespace sbl
