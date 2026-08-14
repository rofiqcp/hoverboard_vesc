/* GPL-3.0-or-later. Wire representation follows vedderb/bldc VESC 6.00. */
#include "vesc_buffer.h"
#include <math.h>

void vesc_buf_append_i16(uint8_t *b,int16_t v,int32_t *i){b[(*i)++]=(uint8_t)((uint16_t)v>>8);b[(*i)++]=(uint8_t)v;}
void vesc_buf_append_u16(uint8_t *b,uint16_t v,int32_t *i){b[(*i)++]=(uint8_t)(v>>8);b[(*i)++]=(uint8_t)v;}
void vesc_buf_append_i32(uint8_t *b,int32_t v,int32_t *i){uint32_t u=(uint32_t)v;b[(*i)++]=(uint8_t)(u>>24);b[(*i)++]=(uint8_t)(u>>16);b[(*i)++]=(uint8_t)(u>>8);b[(*i)++]=(uint8_t)u;}
void vesc_buf_append_u32(uint8_t *b,uint32_t v,int32_t *i){b[(*i)++]=(uint8_t)(v>>24);b[(*i)++]=(uint8_t)(v>>16);b[(*i)++]=(uint8_t)(v>>8);b[(*i)++]=(uint8_t)v;}
void vesc_buf_append_float16(uint8_t *b,float v,float scale,int32_t *i){vesc_buf_append_i16(b,(int16_t)(v*scale),i);}
void vesc_buf_append_float32(uint8_t *b,float v,float scale,int32_t *i){vesc_buf_append_i32(b,(int32_t)(v*scale),i);}
void vesc_buf_append_float32_auto(uint8_t *b,float v,int32_t *i){if(fabsf(v)<1.5e-38f)v=0.0f;int e=0;float sig=frexpf(v,&e);float a=fabsf(sig);uint32_t si=0;if(a>=0.5f){si=(uint32_t)((a-0.5f)*2.0f*8388608.0f);e+=126;}uint32_t r=((uint32_t)(e&0xFF)<<23)|(si&0x7FFFFFU);if(sig<0.0f)r|=1UL<<31;vesc_buf_append_u32(b,r,i);}
int16_t vesc_buf_get_i16(const uint8_t*b,int32_t*i){uint16_t u=((uint16_t)b[*i]<<8)|b[*i+1];*i+=2;return(int16_t)u;}
uint16_t vesc_buf_get_u16(const uint8_t*b,int32_t*i){uint16_t u=((uint16_t)b[*i]<<8)|b[*i+1];*i+=2;return u;}
int32_t vesc_buf_get_i32(const uint8_t*b,int32_t*i){uint32_t u=((uint32_t)b[*i]<<24)|((uint32_t)b[*i+1]<<16)|((uint32_t)b[*i+2]<<8)|b[*i+3];*i+=4;return(int32_t)u;}
uint32_t vesc_buf_get_u32(const uint8_t*b,int32_t*i){uint32_t u=((uint32_t)b[*i]<<24)|((uint32_t)b[*i+1]<<16)|((uint32_t)b[*i+2]<<8)|b[*i+3];*i+=4;return u;}
float vesc_buf_get_float16(const uint8_t*b,float scale,int32_t*i){return(float)vesc_buf_get_i16(b,i)/scale;}
float vesc_buf_get_float32(const uint8_t*b,float scale,int32_t*i){return(float)vesc_buf_get_i32(b,i)/scale;}
float vesc_buf_get_float32_auto(const uint8_t*b,int32_t*i){uint32_t r=vesc_buf_get_u32(b,i);int e=(int)((r>>23)&0xFFU);uint32_t si=r&0x7FFFFFU;bool neg=(r&(1UL<<31))!=0;float sig=0.0f;if(e!=0||si!=0){sig=(float)si/(8388608.0f*2.0f)+0.5f;e-=126;}if(neg)sig=-sig;return ldexpf(sig,e);}
uint16_t vesc_crc16(const uint8_t *data,uint16_t len){uint16_t c=0;for(uint16_t n=0;n<len;n++){c^=(uint16_t)data[n]<<8;for(uint8_t b=0;b<8;b++)c=(c&0x8000U)?(uint16_t)((c<<1)^0x1021U):(uint16_t)(c<<1);}return c;}
