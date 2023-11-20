/*
 * eeprom-pm.c
 *
 *  Created on: 15.04.2016
 *      Author: Dmitri Ranfft
 */
#ifdef EEPROM_25AA02

#include <math.h> // Needed for sqrt();
#include "eeprom-pm.h"

//###############################################################

// These are the conversion factors for saving/reading corresponding values from EEPROM.
// They are used to fit numbers over 255 or under 1 into 1 byte of memory with sufficient precision.
#define RPM_LIMIT_STEP     (200)
#define RPMPS_LIMIT_STEP    (50)
#define KP_FACTOR           (10.0)
#define KI_FACTOR         (1000.0)

//###############################################################

void EEPROM_init_pm(HAL_Handle halHandle){
	HAL_setupEEPROM25AA02(halHandle);
}

void EEPROM_disableWP(HAL_Handle halHandle){
	HAL_setupSpi_25AA02(halHandle);

	// Disable write protection on the whole array: 0x00.
	// Enable  write protection on 0xC0 - 0xFF:     0x04.
	// Enable  write protection on 0x80 - 0xFF:     0x08.
	// Enable  write protection on the whole array: 0x0C.
	EEPROM25AA02_writeStatus(halHandle->eeprom25aa02Handle, 0x00);

	HAL_setupSpiA(halHandle);
}

void EEPROM_enableWP(HAL_Handle halHandle){
	HAL_setupSpi_25AA02(halHandle);

	// Disable write protection on the whole array: 0x00.
	// Enable  write protection on 0xC0 - 0xFF:     0x04.
	// Enable  write protection on 0x80 - 0xFF:     0x08.
	// Enable  write protection on the whole array: 0x0C.
	EEPROM25AA02_writeStatus(halHandle->eeprom25aa02Handle, 0x0C);

	HAL_setupSpiA(halHandle);
}

void EEPROM_save(HAL_Handle halHandle, uint16_t addr, uint_least8_t *data, int len){
	HAL_setupSpi_25AA02(halHandle);
	EEPROM25AA02_writeRegisterN(halHandle->eeprom25aa02Handle, addr, data, len);
	HAL_setupSpiA(halHandle);
}

void EEPROM_read(HAL_Handle halHandle, uint16_t addr, uint_least8_t *buf, int len){
	HAL_setupSpi_25AA02(halHandle);
	EEPROM25AA02_readRegisterN(halHandle->eeprom25aa02Handle, addr, buf, len);
	HAL_setupSpiA(halHandle);
}

uint32_t EEPROM_getVerificationNumber(HAL_Handle halHandle){
	int i = 0;
	uint32_t verNum = 0;
	uint_least8_t buf[EEPROM_VERIFICATION_LENGTH];
	EEPROM_read(halHandle, EEPROM_VERIFICATION_ADDRESS, buf, EEPROM_VERIFICATION_LENGTH);

	for(i = 0; i < EEPROM_VERIFICATION_LENGTH; i++){
		volatile uint16_t shift = ((EEPROM_VERIFICATION_LENGTH - 1) - i) << 3; // Number of bits to shift when saving data from the buffer to the variable.
		verNum |= (((uint32_t)buf[i]) & 0xFF) << shift;
	}

	return verNum;
}

void EEPROM_setVerificationNumber(HAL_Handle halHandle, uint32_t verNum){
	int i = 0;
	uint_least8_t buf[EEPROM_VERIFICATION_LENGTH];

	for(i = 0; i < EEPROM_VERIFICATION_LENGTH; i++){
		uint16_t shift = ((EEPROM_VERIFICATION_LENGTH - 1) - i) << 3; // Number of bits to shift when saving data from the buffer to the variable.
		buf[i] = (verNum >> shift) & 0xFF;
	}

	EEPROM_save(halHandle, EEPROM_VERIFICATION_ADDRESS, buf, EEPROM_VERIFICATION_LENGTH);
}

int EEPROM_isNew(HAL_Handle halHandle){
	uint32_t verNum = EEPROM_getVerificationNumber(halHandle);
	return !(verNum == EEPROM_VERIFICATION_NUMBER);
}

uint_least8_t EEPROM_getTargetProfile(HAL_Handle halHandle){
	uint_least8_t profileNumber = 0;
	EEPROM_read(halHandle, EEPROM_PROFILE_SELECT_ADDRESS, &profileNumber, 1);
	return profileNumber;
}

void EEPROM_setTargetProfile(HAL_Handle halHandle, uint_least8_t pn){
	EEPROM_save(halHandle, EEPROM_PROFILE_SELECT_ADDRESS, &pn, 1);
}

int EEPROM_loadProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params){
	// Checking if a profile with such a number can exist.
	if(pn < 0 || pn >= EEPROM_PROFILE_COUNT){
		return -1;
	}
	// Creating a buffer long enough to hold one profile.
	uint_least8_t buf[EEPROM_PROFILE_SIZE * EEPROM_BLOCK_SIZE];
	// Calculating start address for the requested profile.
	uint16_t startAddress = (EEPROM_PROFILE_SIZE * pn + EEPROM_PROFILE_START_BLOCK) * EEPROM_BLOCK_SIZE;

	int i = 0;
	// Pointer to the place in the buffer where the next page should be saved.
	uint_least8_t *bufPtr = buf;
	// Reading each page of the profile and saving it to the buffer.
	for(i = 0; i < EEPROM_PROFILE_SIZE; i++){
		EEPROM_read(halHandle, startAddress, bufPtr, EEPROM_BLOCK_SIZE);
		startAddress += EEPROM_BLOCK_SIZE;
		bufPtr += EEPROM_BLOCK_SIZE;
	}

	node_id = buf[0] & 0x7F;

	u_params->motor_type = (MOTOR_Type_e)(buf[1] & 0xFF);

	u_params->motor_numPolePairs = ((buf[2] & 0xFF) << 8);
	u_params->motor_numPolePairs |= (buf[3] & 0xFF);

	*(uint32_t*)&u_params->motor_Rr = (((uint32_t)(buf[4] & 0xFF)) << 24);
	*(uint32_t*)&u_params->motor_Rr |= (((uint32_t)(buf[5] & 0xFF)) << 16);
	*(uint32_t*)&u_params->motor_Rr |= (((uint32_t)(buf[6] & 0xFF)) << 8);
	*(uint32_t*)&u_params->motor_Rr |= ((uint32_t)(buf[7] & 0xFF));

	*(uint32_t*)&u_params->motor_Rs = (((uint32_t)(buf[8] & 0xFF)) << 24);
	*(uint32_t*)&u_params->motor_Rs |= (((uint32_t)(buf[9] & 0xFF)) << 16);
	*(uint32_t*)&u_params->motor_Rs |= (((uint32_t)(buf[10] & 0xFF)) << 8);
	*(uint32_t*)&u_params->motor_Rs |= ((uint32_t)(buf[11] & 0xFF));

	*(uint32_t*)&u_params->motor_Ls_d = (((uint32_t)(buf[12] & 0xFF)) << 24);
	*(uint32_t*)&u_params->motor_Ls_d |= (((uint32_t)(buf[13] & 0xFF)) << 16);
	*(uint32_t*)&u_params->motor_Ls_d |= (((uint32_t)(buf[14] & 0xFF)) << 8);
	*(uint32_t*)&u_params->motor_Ls_d |= ((uint32_t)(buf[15] & 0xFF));

	*(uint32_t*)&u_params->motor_Ls_q = (((uint32_t)(buf[16] & 0xFF)) << 24);
	*(uint32_t*)&u_params->motor_Ls_q |= (((uint32_t)(buf[17] & 0xFF)) << 16);
	*(uint32_t*)&u_params->motor_Ls_q |= (((uint32_t)(buf[18] & 0xFF)) << 8);
	*(uint32_t*)&u_params->motor_Ls_q |= ((uint32_t)(buf[19] & 0xFF));

	*(uint32_t*)&u_params->motor_ratedFlux = (((uint32_t)(buf[20] & 0xFF)) << 24);
	*(uint32_t*)&u_params->motor_ratedFlux |= (((uint32_t)(buf[21] & 0xFF)) << 16);
	*(uint32_t*)&u_params->motor_ratedFlux |= (((uint32_t)(buf[22] & 0xFF)) << 8);
	*(uint32_t*)&u_params->motor_ratedFlux |= ((uint32_t)(buf[23] & 0xFF));

	*(uint32_t*)&u_params->IdRated = (((uint32_t)(buf[24] & 0xFF)) << 24);
	*(uint32_t*)&u_params->IdRated |= (((uint32_t)(buf[25] & 0xFF)) << 16);
	*(uint32_t*)&u_params->IdRated |= (((uint32_t)(buf[26] & 0xFF)) << 8);
	*(uint32_t*)&u_params->IdRated |= ((uint32_t)(buf[27] & 0xFF));

	*(uint32_t*)&u_params->maxCurrent_resEst = (((uint32_t)(buf[28] & 0xFF)) << 24);
	*(uint32_t*)&u_params->maxCurrent_resEst |= (((uint32_t)(buf[29] & 0xFF)) << 16);
	*(uint32_t*)&u_params->maxCurrent_resEst |= (((uint32_t)(buf[30] & 0xFF)) << 8);
	*(uint32_t*)&u_params->maxCurrent_resEst |= ((uint32_t)(buf[31] & 0xFF));

	*(uint32_t*)&u_params->maxCurrent_indEst = (((uint32_t)(buf[32] & 0xFF)) << 24);
	*(uint32_t*)&u_params->maxCurrent_indEst |= (((uint32_t)(buf[33] & 0xFF)) << 16);
	*(uint32_t*)&u_params->maxCurrent_indEst |= (((uint32_t)(buf[34] & 0xFF)) << 8);
	*(uint32_t*)&u_params->maxCurrent_indEst |= ((uint32_t)(buf[35] & 0xFF));

	*(uint32_t*)&u_params->maxCurrent = (((uint32_t)(buf[36] & 0xFF)) << 24);
	*(uint32_t*)&u_params->maxCurrent |= (((uint32_t)(buf[37] & 0xFF)) << 16);
	*(uint32_t*)&u_params->maxCurrent |= (((uint32_t)(buf[38] & 0xFF)) << 8);
	*(uint32_t*)&u_params->maxCurrent |= ((uint32_t)(buf[39] & 0xFF));

	m_params->RsOnLineCurrent_A = _IQ(0.1 * u_params->maxCurrent);

	*(uint32_t*)&u_params->fluxEstFreq_Hz = (((uint32_t)(buf[40] & 0xFF)) << 24);
	*(uint32_t*)&u_params->fluxEstFreq_Hz |= (((uint32_t)(buf[41] & 0xFF)) << 16);
	*(uint32_t*)&u_params->fluxEstFreq_Hz |= (((uint32_t)(buf[42] & 0xFF)) << 8);
	*(uint32_t*)&u_params->fluxEstFreq_Hz |= ((uint32_t)(buf[43] & 0xFF));

	rpmLimit = ((uint16_t)buf[44]) * RPM_LIMIT_STEP;
	rpmpsLimit = ((uint16_t)buf[45]) * RPMPS_LIMIT_STEP;

	m_params->Kp_spd = _IQ(((float)buf[46]) / KP_FACTOR);
	m_params->Ki_spd = _IQ(((float)buf[47]) / KI_FACTOR);

	if((u_params->motor_Rr > (float_t)0.0) && (u_params->motor_Rs > (float_t)0.0)){
		u_params->powerWarpGain = sqrt((float_t)1.0 + u_params->motor_Rr/u_params->motor_Rs);
	}else{
		u_params->powerWarpGain = USER_POWERWARP_GAIN;
	}

	return 0;
}

int EEPROM_saveProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params, uint_least8_t nodeid){
	// Checking if a profile with such a number can exist.
	if(pn < 0 || pn >= EEPROM_PROFILE_COUNT){
		return -1;
	}
	// Creating a buffer long enough to hold one profile.
	uint_least8_t buf[EEPROM_PROFILE_SIZE * EEPROM_BLOCK_SIZE];

	buf[0] = nodeid & 0x7F; //0x06;

	buf[1] = u_params->motor_type;

	buf[2] = ((u_params->motor_numPolePairs >> 8) & 0xFF);
	buf[3] = (u_params->motor_numPolePairs & 0xFF);

	buf[4] = ((*((uint32_t*)&u_params->motor_Rr) >> 24) & 0xFF);
	buf[5] = ((*((uint32_t*)&u_params->motor_Rr) >> 16) & 0xFF);
	buf[6] = ((*((uint32_t*)&u_params->motor_Rr) >> 8) & 0xFF);
	buf[7] = (*((uint32_t*)&u_params->motor_Rr) & 0xFF);

	buf[8] = ((*((uint32_t*)&u_params->motor_Rs) >> 24) & 0xFF);
	buf[9] = ((*((uint32_t*)&u_params->motor_Rs) >> 16) & 0xFF);
	buf[10] = ((*((uint32_t*)&u_params->motor_Rs) >> 8) & 0xFF);
	buf[11] = (*((uint32_t*)&u_params->motor_Rs) & 0xFF);

	buf[12] = ((*((uint32_t*)&u_params->motor_Ls_d) >> 24) & 0xFF);
	buf[13] = ((*((uint32_t*)&u_params->motor_Ls_d) >> 16) & 0xFF);
	buf[14] = ((*((uint32_t*)&u_params->motor_Ls_d) >> 8) & 0xFF);
	buf[15] = (*((uint32_t*)&u_params->motor_Ls_d) & 0xFF);

	buf[16] = ((*((uint32_t*)&u_params->motor_Ls_q) >> 24) & 0xFF);
	buf[17] = ((*((uint32_t*)&u_params->motor_Ls_q) >> 16) & 0xFF);
	buf[18] = ((*((uint32_t*)&u_params->motor_Ls_q) >> 8) & 0xFF);
	buf[19] = (*((uint32_t*)&u_params->motor_Ls_q) & 0xFF);

	buf[20] = ((*((uint32_t*)&u_params->motor_ratedFlux) >> 24) & 0xFF);
	buf[21] = ((*((uint32_t*)&u_params->motor_ratedFlux) >> 16) & 0xFF);
	buf[22] = ((*((uint32_t*)&u_params->motor_ratedFlux) >> 8) & 0xFF);
	buf[23] = (*((uint32_t*)&u_params->motor_ratedFlux) & 0xFF);

	buf[24] = ((*((uint32_t*)&u_params->IdRated) >> 24) & 0xFF);
	buf[25] = ((*((uint32_t*)&u_params->IdRated) >> 16) & 0xFF);
	buf[26] = ((*((uint32_t*)&u_params->IdRated) >> 8) & 0xFF);
	buf[27] = (*((uint32_t*)&u_params->IdRated) & 0xFF);

	buf[28] = ((*((uint32_t*)&u_params->maxCurrent_resEst) >> 24) & 0xFF);
	buf[29] = ((*((uint32_t*)&u_params->maxCurrent_resEst) >> 16) & 0xFF);
	buf[30] = ((*((uint32_t*)&u_params->maxCurrent_resEst) >> 8) & 0xFF);
	buf[31] = (*((uint32_t*)&u_params->maxCurrent_resEst) & 0xFF);

	buf[32] = ((*((uint32_t*)&u_params->maxCurrent_indEst) >> 24) & 0xFF);
	buf[33] = ((*((uint32_t*)&u_params->maxCurrent_indEst) >> 16) & 0xFF);
	buf[34] = ((*((uint32_t*)&u_params->maxCurrent_indEst) >> 8) & 0xFF);
	buf[35] = (*((uint32_t*)&u_params->maxCurrent_indEst) & 0xFF);

	buf[36] = ((*((uint32_t*)&u_params->maxCurrent) >> 24) & 0xFF);
	buf[37] = ((*((uint32_t*)&u_params->maxCurrent) >> 16) & 0xFF);
	buf[38] = ((*((uint32_t*)&u_params->maxCurrent) >> 8) & 0xFF);
	buf[39] = (*((uint32_t*)&u_params->maxCurrent) & 0xFF);

	buf[40] = ((*((uint32_t*)&u_params->fluxEstFreq_Hz) >> 24) & 0xFF);
	buf[41] = ((*((uint32_t*)&u_params->fluxEstFreq_Hz) >> 16) & 0xFF);
	buf[42] = ((*((uint32_t*)&u_params->fluxEstFreq_Hz) >> 8) & 0xFF);
	buf[43] = (*((uint32_t*)&u_params->fluxEstFreq_Hz) & 0xFF);

	buf[44] = (rpmLimit/RPM_LIMIT_STEP) & 0xFF;
	buf[45] = (rpmpsLimit/RPMPS_LIMIT_STEP) & 0xFF;

	buf[46] = ((uint_least8_t)(_IQtoF(m_params->Kp_spd) * KP_FACTOR + 0.2)) & 0xFF; //turning xx.x to xxx.0
	buf[47] = ((uint_least8_t)(_IQtoF(m_params->Ki_spd) * KI_FACTOR + 0.2)) & 0xFF; //turning 0.xxx to xxx.0

	// Calculating start address for the requested profile.
	uint16_t startAddress = (EEPROM_PROFILE_SIZE * pn + EEPROM_PROFILE_START_BLOCK) * EEPROM_BLOCK_SIZE;

	int i = 0;
	uint_least8_t *bufPtr = buf;
	for(i = 0; i < EEPROM_PROFILE_SIZE; i++){
		EEPROM_save(halHandle, startAddress, bufPtr, EEPROM_BLOCK_SIZE);
		startAddress += EEPROM_BLOCK_SIZE;
		bufPtr += EEPROM_BLOCK_SIZE;
	}
	return 0;
}

void EEPROM_initMem(HAL_Handle halHandle, USER_Params *u_params, MOTOR_Vars_t *m_params){
	int i = 0;
	for(i = 0; i < EEPROM_PROFILE_COUNT; i++){
		EEPROM_saveProfile(halHandle, i, u_params, m_params, NODE_ID);
	}
}

/*
 * If EEPROM is verified, loads profile from EEPROM. Otherwise it
 * initializes EEPROM with the default parameters.
 */
void EEPROM_startupProfileLoad(HAL_Handle halHandle, USER_Params *u_params, MOTOR_Vars_t *m_params){
	if(EEPROM_isNew(halHandle)){
		EEPROM_initMem(halHandle, u_params, m_params);
		EEPROM_setTargetProfile(halHandle, 0);
		EEPROM_setVerificationNumber(halHandle, EEPROM_VERIFICATION_NUMBER);
	}else{
		uint_least8_t lastProfile = EEPROM_getTargetProfile(halHandle);
		EEPROM_loadProfile(halHandle, (int)lastProfile, u_params, m_params);
	}
}

#endif //EEPROM_25AA02
