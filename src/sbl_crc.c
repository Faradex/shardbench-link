/*
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
 *
 * Bitwise rather than table-driven. A table costs 512 bytes of flash to save a few
 * microseconds per frame, and at 921600 baud a frame still arrives far more slowly than
 * this loop runs. If a profile ever says otherwise, the table goes in here and nothing
 * else changes.
 */
#include "shardbench_link.h"

uint16_t sbl_crc16(const uint8_t *data, size_t len, uint16_t crc)
{
    size_t i;
    int bit;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}
