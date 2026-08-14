#include "vesc_packet.h"
#include "vesc_buffer.h"
#include <string.h>
enum{P_WAIT=0,P_LEN8,P_LEN16_HI,P_LEN16_LO,P_DATA,P_CRC_HI,P_CRC_LO,P_STOP};
void VescPacket_Init(VescPacketParser*p){memset(p,0,sizeof(*p));}
static void reset(VescPacketParser*p){p->state=P_WAIT;p->expected=0;p->pos=0;p->rx_crc=0;}
void VescPacket_Feed(VescPacketParser*p,uint8_t x,VescPacketRxCb cb){
 switch(p->state){
 case P_WAIT: if(x==2U)p->state=P_LEN8;else if(x==3U)p->state=P_LEN16_HI;break;
 case P_LEN8:p->expected=x;if(p->expected==0||p->expected>VESC_PACKET_MAX_PAYLOAD)reset(p);else{p->pos=0;p->state=P_DATA;}break;
 case P_LEN16_HI:p->long_len_hi=x;p->state=P_LEN16_LO;break;
 case P_LEN16_LO:p->expected=(uint16_t)(((uint16_t)p->long_len_hi<<8)|x);if(p->expected==0||p->expected>VESC_PACKET_MAX_PAYLOAD)reset(p);else{p->pos=0;p->state=P_DATA;}break;
 case P_DATA:p->payload[p->pos++]=x;if(p->pos>=p->expected)p->state=P_CRC_HI;break;
 case P_CRC_HI:p->rx_crc=(uint16_t)x<<8;p->state=P_CRC_LO;break;
 case P_CRC_LO:p->rx_crc|=x;p->state=P_STOP;break;
 case P_STOP:
  /* 0x03 is the normal VESC end marker as well as the long-frame start
   * marker. When it terminates the current frame it must NOT be consumed a
   * second time as the start of the next frame. Only a non-stop byte is
   * considered for immediate resynchronisation. */
  if(x==3U&&p->rx_crc==vesc_crc16(p->payload,p->expected)&&cb)cb(p->payload,p->expected);
  reset(p);
  if(x!=3U){if(x==2U)p->state=P_LEN8;}
  break;
 default:reset(p);break;}}
uint16_t VescPacket_Encode(const uint8_t*payload,uint16_t len,uint8_t*frame,uint16_t cap){if(!payload||!frame||len==0||len>VESC_PACKET_MAX_PAYLOAD)return 0;uint16_t h=(len<=255U)?2U:3U;uint16_t total=(uint16_t)(h+len+3U);if(cap<total)return 0;uint16_t i=0;if(len<=255U){frame[i++]=2U;frame[i++]=(uint8_t)len;}else{frame[i++]=3U;frame[i++]=(uint8_t)(len>>8);frame[i++]=(uint8_t)len;}memcpy(frame+i,payload,len);i+=len;uint16_t crc=vesc_crc16(payload,len);frame[i++]=(uint8_t)(crc>>8);frame[i++]=(uint8_t)crc;frame[i++]=3U;return i;}
