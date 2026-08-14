#ifndef TEST_CROSS_STUB_STRING_H
#define TEST_CROSS_STUB_STRING_H
#include <stddef.h>
void *memset(void *s, int c, size_t n);
void *memcpy(void *dst, const void *src, size_t n);
int memcmp(const void *a, const void *b, size_t n);
#endif
