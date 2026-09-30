/*
 * Mbed OS port of ShardBench Link.
 *
 * Everything that knows about a UART, a thread or a mutex is here; `src/` stays plain
 * C99 so it can be tested on a PC. A different transport -- CAN, or a socket -- is
 * another file beside this one rather than a change to anything above it.
 */
#ifndef SBL_PORT_MBED_H
#define SBL_PORT_MBED_H

#include <mbed.h>

#include "shardbench_link.h"

namespace sbl {

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

    ssize_t write(const void *buffer, size_t size) override;
    ssize_t read(void *buffer, size_t size) override;
    off_t seek(off_t offset, int whence = SEEK_SET) override;
    int close() override;
    int isatty() override;

private:
    /** Where a finished line goes. Static so the C core can call back into it. */
    static void on_line(uint8_t level, const char *text, uint16_t len, void *user);

    mbed::BufferedSerial &_serial;
    sbl_ctx    *_ctx;
    rtos::Mutex _lock;
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

    /** Below the measurement threads: a late log matters less than a late sample. */
    void start(osPriority priority = osPriorityBelowNormal);

    sbl_ctx *ctx() { return &_ctx; }
    ConsoleToLog &console() { return _console; }

    /** Convenience wrappers; the C API underneath takes ctx() directly. */
    void add(uint16_t id, uint8_t kinds, sbl_handler_fn fn, void *user = nullptr);
    void emit(uint16_t id, const void *payload, uint16_t len);
    void resolve(uint16_t id, uint8_t seq, const void *payload, uint16_t len);
    void stream(uint16_t id, const void *samples, uint16_t len);
    void set_state(uint8_t state, uint32_t alarms);
    const sbl_stream_cfg *stream_config(uint16_t id) const;

private:
    void rx_thread();

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
};

}  // namespace sbl

#endif /* SBL_PORT_MBED_H */
