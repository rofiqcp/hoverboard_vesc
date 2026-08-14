#ifndef VESC_BUFFER_H
#define VESC_BUFFER_H
#include <stdint.h>
#include <stdbool.h>

void vesc_buf_append_i16(uint8_t *b, int16_t v, int32_t *i);
void vesc_buf_append_u16(uint8_t *b, uint16_t v, int32_t *i);
void vesc_buf_append_i32(uint8_t *b, int32_t v, int32_t *i);
void vesc_buf_append_u32(uint8_t *b, uint32_t v, int32_t *i);
void vesc_buf_append_float16(uint8_t *b, float v, float scale, int32_t *i);
void vesc_buf_append_float32(uint8_t *b, float v, float scale, int32_t *i);
void vesc_buf_append_float32_auto(uint8_t *b, float v, int32_t *i);
int16_t vesc_buf_get_i16(const uint8_t *b, int32_t *i);
uint16_t vesc_buf_get_u16(const uint8_t *b, int32_t *i);
int32_t vesc_buf_get_i32(const uint8_t *b, int32_t *i);
uint32_t vesc_buf_get_u32(const uint8_t *b, int32_t *i);
float vesc_buf_get_float16(const uint8_t *b, float scale, int32_t *i);
float vesc_buf_get_float32(const uint8_t *b, float scale, int32_t *i);
float vesc_buf_get_float32_auto(const uint8_t *b, int32_t *i);
uint16_t vesc_crc16(const uint8_t *data, uint16_t len);
#endif
