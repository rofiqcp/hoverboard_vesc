#ifndef LEFT_ENCODER_H
#define LEFT_ENCODER_H
#include <stdint.h>
#include <stdbool.h>
void LeftEncoder_Init(void);
void LeftEncoder_SetMode(bool encoder_mode);
bool LeftEncoder_IsEnabled(void);
int32_t LeftEncoder_GetCount(void);
void LeftEncoder_ZeroMechanical(void);
void LeftEncoder_IndexIrq(void);
bool LeftEncoder_IndexSeen(void);
int32_t LeftEncoder_GetIndexCount(void);
#endif
