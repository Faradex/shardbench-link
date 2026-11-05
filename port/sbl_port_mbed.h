/*
 * Mbed OS port of ShardBench Link.
 *
 * Everything that knows about a UART, a thread or a mutex is here; `src/` stays plain
 * C99 so it can be tested on a PC.
 *
 * A second way in, beside the UART: the CAN tunnel through a Crystal (`sbl_tunnel.h`).
 * The firmware supplies a `CanBus` -- "send this frame" -- and hands over what arrives
 * on 0x7A0 for its node; the link does the rest, with one set of resources for both.
 */
#ifndef SBL_PORT_MBED_H
#define SBL_PORT_MBED_H

#include <mbed.h>

#include "sbl_tunnel.h"
#include "shardbench_link.h"

namespace sbl {

/**
 * What the link needs of a CAN driver, and nothing more: the library stays free of any
 * SDK. The firmware's adapter builds the extended id from its own node id.
 */
class CanBus {
public:
    virtual ~CanBus() {}
    /** Send one frame on `param` from this node. False when the controller has no
        room right now: the link waits a little and tries again. */
    virtual bool send(uint16_t param, const uint8_t *data, uint8_t len) = 0;
};

/**
 * Turns everything written to stdout into LOG frames.
 *
 * `main.cpp` hands this to `mbed_override_console`, which is the single place all
 * output already passes through: `LOG_INFO` expands to `printf`, and so does the
 * prebuilt vitroio-sdk. Nothing else has to be touched to put the board's log on the
 * wire.
 *
 * Before the link starts, and from inside an interrupt, bytes go straight to the serial
 * port instead. That is not a nicety: Mbed's own error handler prints the crash through
 * this same path, and a message that explains why the board died must not be swallowed
 * by a mutex the fault has already made unusable.
 */
class ConsoleToLog : public mbed::FileHandle {
public:
    ConsoleToLog(mbed::BufferedSerial &serial);

    /** Called once the link is running; before this, output is passed through raw. */
    void attach(sbl_ctx *ctx);

    /**
     * Pass one thread's output straight to the UART, as text, while it is set: the
     * thread sending a CAN frame, whose driver may print an error. Through the link it
     * would need the console's lock while holding the transmit lock -- which another
     * printing thread may hold the other way round.
     */
    void raw_for(osThreadId_t thread) { _raw_thread = thread; }

    ssize_t write(const void *buffer, size_t size) override;
    ssize_t read(void *buffer, size_t size) override;
    off_t seek(off_t offset, int whence = SEEK_SET) override;
    int close() override;
    int isatty() override;

private:
    /** Where a finished line goes. Static so the C core can call back into it. */
    static void on_line(uint8_t level, const char *text, uint16_t len, void *user);
#if SBL_CONSOLE_TRACE
    void trace(const char *data, size_t size);
#endif

    mbed::BufferedSerial &_serial;
    sbl_ctx    *_ctx;
    rtos::Mutex _lock;
    volatile osThreadId_t _raw_thread = nullptr;
    /* The line assembly itself lives in the platform-free half, under test: it is the
       part that got the interleaving wrong twice, and here it cannot be tested. */
    sbl_console _console;
};

/**
 * The link, its thread and its serial port.
 *
 * Construct it next to the BufferedSerial the board already has, register resources,
 * then `start()`. Start it before anything that can block -- on this firmware that is
 * the cable wait, the random startup delay and the CAN retry loop, any of which can hold
 * a board for ever. Started first, ShardBench sees a board stuck in one of them and its
 * log says which.
 */
class Link {
public:
    Link(mbed::BufferedSerial &serial, const sbl_device_info &info);

    /**
     * Above the measurement threads, and this is worth arguing about.
     *
     * Below them looks right -- a late log matters less than a late sample -- and it is
     * wrong. The thread does almost nothing: read a few bytes, answer, sleep again. But
     * it is the only thread that answers the host, and the monitors here bit-bang a
     * 1-Wire bus with busy waits that never yield, so at a lower priority it simply
     * never runs. The board keeps logging, because logs are written by whichever thread
     * called printf, and stops answering anything: it looks dead while sounding alive.
     *
     * Lower it if the microseconds it steals ever disturb a measurement.
     */
    void start(osPriority priority = osPriorityAboveNormal);

    sbl_ctx *ctx() { return &_ctx; }
    ConsoleToLog &console() { return _console; }

    /** Give the link the Crystal's CAN tunnel as a second way in (D-7). Before start(). */
    void attach_can(CanBus &bus);

    /**
     * The data of a frame on 0x7A0 addressed to this node. From any thread -- the CAN
     * driver's event queue, typically -- but not from an interrupt. The bytes are
     * handled on the link's own thread, like the UART's.
     */
    void can_received(const uint8_t *data, uint8_t len);

    /** Convenience wrappers; the C API underneath takes ctx() directly. */
    void add(uint16_t id, uint8_t kinds, sbl_handler_fn fn, void *user = nullptr);
    void emit(uint16_t id, const void *payload, uint16_t len);
    void resolve(uint16_t id, uint8_t seq, const void *payload, uint16_t len);
    void fail(uint16_t id, uint8_t seq, uint8_t code);
    void stream(uint16_t id, uint32_t tick_ms, uint32_t period_us,
                const void *samples, uint16_t count, uint8_t sample_bytes);
    void set_state(uint8_t state, uint32_t alarms);
    /** The link's own millisecond clock, so a timestamp an application hands back --
        a stream batch's tick -- is on the same time base the link runs on. */
    uint32_t tick_ms() const;
    const sbl_stream_cfg *stream_config(uint16_t id) const;

private:
    enum Channel { UART, CAN };

    void rx_thread();
    void on_serial_event();
    void feed(Channel from, const uint8_t *data, size_t len);
    bool can_attached();
    void can_write(const uint8_t *data, size_t len);

    static void port_write(void *io, const uint8_t *data, size_t len);
    static void port_lock(void *io);
    static void port_unlock(void *io);
    static uint32_t port_tick_ms(void *io);

    mbed::BufferedSerial &_serial;
    ConsoleToLog   _console;
    rtos::Mutex    _tx_lock;
    rtos::Thread   _thread;
    sbl_port       _port;
    sbl_ctx        _ctx;
    volatile bool  _running;

    /* The CAN tunnel. `_channel` is where the last request came from, and where
       answers, events and LOG frames go while the host there is still listening. */
    CanBus        *_can;
    sbl_tunnel     _tunnel;
    volatile Channel _channel;
    rtos::Mutex    _rx_lock;             /* sbl_feed is not re-entrant */
    rtos::Mutex    _tunnel_lock;
    mbed::CircularBuffer<uint8_t, 512> _can_rx;
    rtos::EventFlags _wake;
    bool           _in_can_write;
};

}  // namespace sbl

#endif /* SBL_PORT_MBED_H */
