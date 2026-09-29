/*
 * Compile-time sizes for ShardBench Link.
 *
 * Every buffer in the library is sized from these, and every one of them is static:
 * the library never allocates. On a board that runs for days on a bench, a heap is a
 * source of failures that arrive late and cannot be reproduced.
 *
 * Override any of these by defining it before including, or with -D on the compiler
 * command line.
 */
#ifndef SBL_CONFIG_H
#define SBL_CONFIG_H

/** Largest payload the protocol allows (architecture doc, section 5). */
#ifndef SBL_MAX_PAYLOAD
#define SBL_MAX_PAYLOAD 512
#endif

#define SBL_HEADER_SIZE 9
#define SBL_CRC_SIZE    2

/** Header + payload + CRC: what the CRC is computed over, plus the CRC itself. */
#define SBL_MAX_BODY (SBL_HEADER_SIZE + SBL_MAX_PAYLOAD + SBL_CRC_SIZE)

/*
 * COBS adds one code byte per 254 bytes of data, plus one leading code byte, and the
 * frame ends with the 0x00 delimiter. 523 -> 527 at the worst.
 */
#define SBL_MAX_ENCODED (SBL_MAX_BODY + (SBL_MAX_BODY / 254) + 2)

/** Resources the device may register. Costs 16 bytes each on a 32-bit target. */
#ifndef SBL_MAX_RESOURCES
#define SBL_MAX_RESOURCES 24
#endif

/** Streams whose configuration is remembered. BMU-1ch declares one. */
#ifndef SBL_MAX_STREAMS
#define SBL_MAX_STREAMS 4
#endif

#endif /* SBL_CONFIG_H */
