# shardbench-link

Faradex · the device half of ShardBench Link.

The protocol as a C99 library, meant to be dropped into a firmware project as a source
folder. `src/` has no platform underneath it: no mbed, no RTOS, no `printf`, no `malloc`,
no floating point. That is the rule the rest of the library is built around, and it buys
two things.

**Wire compatibility stops being a claim.** These sources compile on a PC, so the host's
test suite calls them directly through `ctypes` and checks them against the host's own
implementation — see `backend/tests/test_firmware_link.py`. The bytes the device will put
on the line are produced by the code that will run on the device.

**The port is small enough to be obvious.** Everything that knows about a UART or a
thread lives in `port/`, so a CAN transport later is an implementation rather than a
redesign.

## Layout

```
include/shardbench_link.h   public API
include/sbl_config.h        sizes, overridable with -D
src/sbl_crc.c               CRC-16/CCITT-FALSE
src/sbl_cobs.c              COBS, bulk and byte-at-a-time
src/sbl_frame.c             header pack/unpack, whole-frame encode/decode
src/sbl_decoder.c           byte stream -> frames, resynchronising
src/sbl_internal.h          shared between the above; not public
port/                       the platform layer (Mbed, and whatever comes next)
```

## What it costs

Measured, not estimated — `-Os`, host compiler; the Cortex-M4 figures are in the same
range:

| | |
|---|---|
| code | ~1.0 kB across the four objects |
| `sbl_decoder` | **1080 bytes**, static, owned by the caller |
| deepest stack | **~152 bytes** (`sbl_encode` 144 + `sbl_cobs_put` 8) |
| heap | none, ever |

`sbl_encode` writes COBS straight into the caller's output buffer rather than assembling
the body first. That is worth knowing when reading the code: the obvious implementation
costs 523 bytes of stack, and on this device the thread that sends stream samples is the
measurement thread, whose stack we do not own.

## Using it from Mbed

The BMU build already takes several source folders, so the library joins them:

```sh
mbed compile -t GCC_ARM -m NUCLEO_L486RG \
    --source=. --source=../app_src --source=../common \
    --source=../libs/vitroio-sdk --source=../libs/shardbench-link
```

Everything is behind `SHARDBENCH_LINK`. With the macro undefined the library is compiled
out completely: no flash, no RAM, no behaviour change. The test firmware defines it; the
production firmware does not, and does not speak the protocol at all.

## The wire

```
COBS( header[9] | payload[len] | crc16_le[2] ) 0x00
header, little-endian: ver u8, addr u8, type u8, flags u8, seq u8, id u16, len u16
CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over header + payload
```

A frame is what lies between two delimiters. A block that is malformed, fails its CRC or
outgrows the buffer is dropped and counted, and decoding resumes at the next delimiter —
so a device powered up mid-transmission, or a line with noise on it, recovers by itself.
What is *not* promised: bytes appended to a frame with no delimiter between them are one
malformed block, and that frame goes with the junk.

Full protocol: `docs/architecture.md`, section 5.
