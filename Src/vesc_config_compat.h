#ifndef VESC_CONFIG_COMPAT_H
#define VESC_CONFIG_COMPAT_H
#include <stdint.h>
#include <stdbool.h>

#define VESC6_MCCONF_SIGNATURE 776184161UL
#define VESC6_APPCONF_SIGNATURE 486554156UL

int32_t VescConfig_SerializeMc(uint8_t *buffer, bool right_motor, bool defaults);
bool VescConfig_DeserializeMc(const uint8_t *buffer, uint32_t len, bool right_motor, bool store);
int32_t VescConfig_SerializeApp(uint8_t *buffer, bool right_motor, bool defaults);
bool VescConfig_DeserializeApp(const uint8_t *buffer, uint32_t len, bool right_motor, bool store);
#endif
