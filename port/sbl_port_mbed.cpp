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
    if (_ctx == nullptr || core_util_is_isr_active()) {
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
    static_cast<Link *>(io)->_serial.write(data, len);
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
      _running(false)
{
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

void Link::rx_thread()
{
    uint8_t chunk[RX_CHUNK];

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

        ssize_t n = _serial.read(chunk, sizeof(chunk));
        if (n > 0) {
            /* Handlers run here, on this stack, so they must not block for long. */
            sbl_feed(&_ctx, chunk, static_cast<size_t>(n));
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
