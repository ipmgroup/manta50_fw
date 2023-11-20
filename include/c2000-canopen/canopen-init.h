#ifndef __CANOPEN_INIT_H__
#define __CANOPEN_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t controlword;
extern uint16_t statusword;
extern int32_t target_rpm;
extern int32_t current_rpm;

extern uint32_t accel_rpm;
extern uint16_t current;
extern uint16_t temperature;
extern uint16_t vol_hum[2];
extern uint16_t fault[4];
//extern uint16_t pid_k[2];
extern float pid_k[2];

#ifdef EEPROM_25AA02
extern uint32_t profile_save;
extern uint16_t profile_select;
extern float profile[17];
#endif

void canopen_init(void);

#ifdef __cplusplus
}
#endif

#endif
