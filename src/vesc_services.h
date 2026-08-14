#ifndef VESC_SERVICES_H
#define VESC_SERVICES_H

#include <stdint.h>

typedef struct {
    uint32_t scheduler_passes;
    uint32_t control_updates;
    uint32_t app_updates;
    uint32_t status_updates;
    uint16_t max_control_gap_ms;
    uint8_t status_code;
} VescServiceStats;

/* Compatibility only. Main scheduler lives in src/main.c. */
void VescServices_Init(void);
void VescServices_Run(void);
void VescServices_GetStats(VescServiceStats *out);

#endif
