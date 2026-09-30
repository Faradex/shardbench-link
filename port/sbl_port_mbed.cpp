#include "sbl_port_mbed.h"

namespace sbl {

/** Bytes pulled off the UART per read. A STREAM frame is a few hundred. */
static const size_t RX_CHUNK = 128;
/** Enough for the link's own work; the application's handlers run on this stack too. */
static const uint32_t RX_STACK = 2048;

// --- the console ------------------------------------------------------------

ConsoleToLog::ConsoleToLog(mbed::BufferedSerial &serial)
    : _serial(serial), _ctx(nullptr), _fill(0), _owner(nullptr)
{
}

/**
 * Read the level out of the text, because printf cannot carry one.
 *
 * Every line the SDK and the application produce starts "[LEVEL][MODULE]: ", so the
 * level is there to be had. Taking it means the host can colour the line, filter on it,
 * and honour SET LOG_LEVEL properly -- and that the log does not read "[INFO] [ERROR]".
 * The module tag stays: it is the useful half.
 */
static uint8_t level_from(const char *line, uint16_t len, uint16_t *skip)
{
    static const struct { const char *name; uint8_t len; uint8_t level; } LEVELS[] = {
        {"[ERROR]",   7, 1},
        {"[WARNING]", 9, 2},
        {"[INFO]",    6, 3},
        {"[DEBUG]",   7, 4},
    };
    unsigned i, c;

    *skip = 0;
    for (i = 0; i < sizeof(LEVELS) / sizeof(LEVELS[0]); i++) {
        if (len < LEVELS[i].len) {
            continue;
        }
        for (c = 0; c < LEVELS[i].len; c++) {
            if (line[c] != LEVELS[i].name[c]) {
                break;
            }
        }
        if (c == LEVELS[i].len) {
            *skip = LEVELS[i].len;
            return LEVELS[i].level;
        }
    }
    return SBL_LOG_INFO;
}

void ConsoleToLog::attach(sbl_ctx *ctx)
{
    _lock.lock();
    _ctx = ctx;
    _fill = 0;              /* whatever was half-written belongs to the raw era */
    _owner = nullptr;
    _lock.unlock();
}

void ConsoleToLog::flush_line()
{
    uint16_t skip = 0;
    uint8_t level;

    if (_fill == 0 || _ctx == nullptr) {
        _fill = 0;
        _owner = nullptr;
        return;
    }
    level = level_from(_line, _fill, &skip);
    sbl_log(_ctx, level, &_line[skip], (uint16_t)(_fill - skip));
    _fill = 0;
    _owner = nullptr;
}

ssize_t ConsoleToLog::write(const void *buffer, size_t size)
{
    const char *text = static_cast<const char *>(buffer);

    /* Before the link exists, and from an interrupt, straight to the port. Mbed's
       error handler prints the crash through here, and it must survive. */
    if (_ctx == nullptr || core_util_is_isr_active()) {
        return _serial.write(buffer, size);
    }

    _lock.lock();

    /* A different thread starting to print means the line in the buffer will never be
       finished by its owner. Sending it now costs a truncated line; letting the two be
       spliced together costs both of them. */
    osThreadId_t me = osThreadGetId();
    if (_fill > 0 && _owner != me) {
        flush_line();
    }
    _owner = me;

    for (size_t i = 0; i < size; i++) {
        char c = text[i];
        if (c == '\r') {
            continue;                       /* the "\n\r" the debug macro appends */
        }
        if (c == '\n') {
            flush_line();
            continue;
        }
        if (_fill >= sizeof(_line)) {
            flush_line();                   /* a line longer than the buffer splits */
        }
        _line[_fill++] = c;
    }
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
    : _serial(serial), _console(serial), _thread(osPriorityBelowNormal, RX_STACK),
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
}

void Link::rx_thread()
{
    uint8_t chunk[RX_CHUNK];

    while (_running) {
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

void Link::stream(uint16_t id, const void *samples, uint16_t len)
{
    sbl_stream(&_ctx, id, static_cast<const uint8_t *>(samples), len);
}

void Link::set_state(uint8_t state, uint32_t alarms)
{
    sbl_set_state(&_ctx, state, alarms);
}

const sbl_stream_cfg *Link::stream_config(uint16_t id) const
{
    return sbl_stream_config(&_ctx, id);
}

}  // namespace sbl
