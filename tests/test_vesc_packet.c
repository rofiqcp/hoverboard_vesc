#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vesc_packet.h"
#include "vesc_buffer.h"

static uint8_t got[768];
static uint16_t got_len;
static int callbacks;
static void cb(const uint8_t *p, uint16_t n) {
    assert(n <= sizeof(got));
    memcpy(got, p, n);
    got_len = n;
    callbacks++;
}

int main(void) {
    /* Canonical CRC-CCITT/XMODEM check used by the VESC packet layer. */
    static const uint8_t s[] = "123456789";
    assert(vesc_crc16(s, 9) == 0x31C3U);

    const uint8_t small[] = {0, 6, 0x12, 0x34, 0x56, 0x78};
    uint8_t frame[VESC_PACKET_MAX_FRAME];
    uint16_t n = VescPacket_Encode(small, sizeof(small), frame, sizeof(frame));
    assert(n == sizeof(small) + 5U);
    assert(frame[0] == 2U && frame[1] == sizeof(small) && frame[n - 1] == 3U);

    VescPacketParser p;
    VescPacket_Init(&p);
    /* Noise before a frame must be ignored. */
    VescPacket_Feed(&p, 0x55, cb);
    VescPacket_Feed(&p, 0xAA, cb);
    for (uint16_t i = 0; i < n; ++i) VescPacket_Feed(&p, frame[i], cb);
    assert(callbacks == 1 && got_len == sizeof(small));
    assert(memcmp(got, small, sizeof(small)) == 0);

    /* Broken CRC must not dispatch. */
    uint8_t broken[VESC_PACKET_MAX_FRAME];
    memcpy(broken, frame, n);
    broken[n - 3] ^= 1U;
    for (uint16_t i = 0; i < n; ++i) VescPacket_Feed(&p, broken[i], cb);
    assert(callbacks == 1);

    /* Exercise the VESC long-frame prefix (start byte 3). */
    uint8_t large[300];
    for (uint16_t i = 0; i < sizeof(large); ++i) large[i] = (uint8_t)(i * 17U + 3U);
    n = VescPacket_Encode(large, sizeof(large), frame, sizeof(frame));
    assert(n == sizeof(large) + 6U);
    assert(frame[0] == 3U && frame[1] == 1U && frame[2] == 44U);
    for (uint16_t i = 0; i < n; ++i) VescPacket_Feed(&p, frame[i], cb);
    assert(callbacks == 2 && got_len == sizeof(large));
    assert(memcmp(got, large, sizeof(large)) == 0);

    puts("VESC_PACKET_TESTS_PASS");
    return 0;
}
