#ifndef VESC_PACKET_H
#define VESC_PACKET_H
#include <stdint.h>
#include <stdbool.h>
#define VESC_PACKET_MAX_PAYLOAD 768U
#define VESC_PACKET_MAX_FRAME (VESC_PACKET_MAX_PAYLOAD + 6U)
typedef void (*VescPacketRxCb)(const uint8_t *payload,uint16_t len);
typedef struct {uint8_t payload[VESC_PACKET_MAX_PAYLOAD];uint16_t expected;uint16_t pos;uint16_t rx_crc;uint8_t state;uint8_t long_len_hi;} VescPacketParser;
void VescPacket_Init(VescPacketParser *p);
void VescPacket_Feed(VescPacketParser *p,uint8_t byte,VescPacketRxCb cb);
uint16_t VescPacket_Encode(const uint8_t *payload,uint16_t len,uint8_t *frame,uint16_t cap);
#endif
