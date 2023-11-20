/*
 * eeprom-pm.h
 *
 *  Created on: 15.04.2016
 *      Author: Dmitri Ranfft
 */

#ifndef INCLUDE_EEPROM_PM_EEPROM_PM_H_
#define INCLUDE_EEPROM_PM_EEPROM_PM_H_
#ifdef EEPROM_25AA02

#include "main_env.h"
//#include "hal.h"
//#include "Microchip_25AA02E48.h"

//###############################################################

#define EEPROM_SIZE                          256 // Byte.
#define EEPROM_BLOCK_SIZE                     16 // Block size in bytes.
#define EEPROM_PROFILE_START_BLOCK             1 // Block 0 will be reserved for metadata.
#define EEPROM_PROFILE_SIZE                    3 // In blocks.
#define EEPROM_PROFILE_COUNT                   5 // (((EEPROM_SIZE/EEPROM_BLOCK_SIZE)-EEPROM_PROFILE_START_BLOCK)/EEPROM_PROFILE_SIZE) // Number of profiles based on sizes.
#define EEPROM_PROFILE_SELECT_ADDRESS       0x04 // Address of the byte containing the id of the target profile.

#define EEPROM_VERIFICATION_NUMBER    0x13117532 // EEPROM verification number.
#define EEPROM_VERIFICATION_LENGTH             4 // Length of the verification number in bytes.
#define EEPROM_VERIFICATION_ADDRESS         0x00 // Starting address of the verification number.

//###############################################################

extern uint8_t node_id;
extern uint16_t rpmLimit;
extern uint16_t rpmpsLimit;

//###############################################################

void EEPROM_init_pm(HAL_Handle halHandle);
void EEPROM_disableWP(HAL_Handle halHandle);
void EEPROM_enableWP(HAL_Handle halHandle);
void EEPROM_save(HAL_Handle halHandle, uint16_t addr, uint_least8_t *data, int len);
void EEPROM_read(HAL_Handle halHandle, uint16_t addr, uint_least8_t *buf, int len);
uint32_t EEPROM_getVerificationNumber(HAL_Handle halHandle);

uint_least8_t EEPROM_getTargetProfile(HAL_Handle halHandle);
void EEPROM_setTargetProfile(HAL_Handle halHandle, uint_least8_t pn);
int EEPROM_loadProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params);
int EEPROM_saveProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params, uint_least8_t nodeid);

void EEPROM_initMem(HAL_Handle halHandle, USER_Params *params, MOTOR_Vars_t *m_params);
void EEPROM_startupProfileLoad(HAL_Handle halHandle, USER_Params *u_params, MOTOR_Vars_t *m_params);

#endif //EEPROM_25AA02
#endif /* INCLUDE_EEPROM_PM_EEPROM_PM_H_ */
