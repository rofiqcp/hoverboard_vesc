/*
 * Deprecated V22 compatibility shim.
 *
 * The real cooperative VESC task scheduler now lives explicitly in main.c.
 * Keeping only these symbols prevents stale external test code from failing to
 * link while guaranteeing that no second hidden scheduler can compete with main.
 */
#include "vesc_services.h"
#include <stddef.h>
#include <string.h>

static VescServiceStats s_compat_stats;

void VescServices_Init(void)
{
    memset(&s_compat_stats, 0, sizeof(s_compat_stats));
}

void VescServices_Run(void)
{
    /* Intentionally empty. main.c owns every task. */
}

void VescServices_GetStats(VescServiceStats *out)
{
    if (out == NULL) return;
    *out = s_compat_stats;
}
