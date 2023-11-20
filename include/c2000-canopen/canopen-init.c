#include <canopen.h>
#include "canopen-pdo.h"
#include "canopen-init.h"
#include "main_env.h" // MOTOR_Vars_t
#ifdef EEPROM_25AA02
#include "eeprom-pm.h"
#endif

uint16_t controlword = 0, statusword = 0;
int32_t target_rpm = 0, current_rpm = 0;
uint32_t accel_rpm = 1000;

uint16_t rpmLimit = 50000;
uint16_t rpmpsLimit = 5000;

uint16_t current = 0, temperature = 0;
uint16_t vol_hum[2] = {0, 0};
uint16_t fault[4] = {0, 0, 0, 0};
//uint16_t pid_k[2] = {0, 0}; // kp, ki
float pid_k[2] = {0, 0}; // kp, ki

static uint8_t temp_len[] = { 0, 0, 2 };
static uint8_t warr_len[] = { 0, 2, 2, 2, 2 };
static uint8_t warr_len4[] = { 0, 4, 4 };

#ifdef EEPROM_25AA02
extern uint8_t node_id;
uint32_t profile_save = 0;
uint16_t profile_select = 0;
float profile[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t prof_save_len[2] = {0, 4};
static uint8_t prof_len[] = {0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
#endif

static pdo_map_t pdo_map[] = {
    {
         PDO_DEFAULT(1)
    }, {
         PDO_DEFAULT(2)
    }, {
         PDO_DEFAULT(3)
    }, {
         PDO_DEFAULT(4)
    }
};

static void map_pdo(void);
static void set_descr(void);
static void set_model(void);
static void set_custom(void);
static void *pdo_genmap(uint16_t index, int16_t subindex, int16_t len);

static uint8_t setcw_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t getsw_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t setrpm_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t getrpm_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t setaccel_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t kpki_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t gettemp_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t getcurrent_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t volhum_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t getstatus_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);

#ifdef EEPROM_25AA02
static uint8_t prof_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t profselect_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
static uint8_t profsave_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);
#endif

void canopen_init(void) {
#ifdef EEPROM_25AA02
	// For this to work make sure that EEPROM_startupProfileLoad() or EEPROM_loadProfile() is called before this function.
	// If it isn't done, node_id will be 0.
    co_init(node_id, 1, pdo_map, sizeof(pdo_map) / sizeof(pdo_map[0]));
#else
    co_init(NODE_ID, 1, pdo_map, sizeof(pdo_map) / sizeof(pdo_map[0]));
#endif

    map_pdo();
    set_descr();
    set_model();
    set_custom();
}

static void map_pdo(void) {
    uint16_t num, mode;

    // RxPDO1
    num = 1; co_memcpy_pack(dict_data(0x1600, 0), &num, 1);
    co_memcpy_pack(dict_data(0x1600, 1), pdo_genmap(0x6040, 0, 16), 4);

    // RxPDO2
    num = 2; co_memcpy_pack(dict_data(0x1601, 0), &num, 1);
    co_memcpy_pack(dict_data(0x1601, 1), pdo_genmap(0x60FF, 0, 32), 4);
    co_memcpy_pack(dict_data(0x1601, 2), pdo_genmap(0x6083, 0, 32), 4);

    // TxPDO1
    num = 1; co_memcpy_pack(dict_data(0x1A00, 0), &num, 1);
    co_memcpy_pack(dict_data(0x1A00, 1), pdo_genmap(0x6041, 0, 16), 4);
    mode = 1; co_memcpy_pack(dict_data(0x1800, 2), &mode, 1);

    // TxPDO2
    num = 1; co_memcpy_pack(dict_data(0x1A01, 0), &num, 1);
    co_memcpy_pack(dict_data(0x1A01, 1), pdo_genmap(0x606C, 0, 32), 4);
    mode = 1; co_memcpy_pack(dict_data(0x1801, 2), &mode, 1);

    // TxPDO3
    num = 4; co_memcpy_pack(dict_data(0x1A02, 0), &num, 1);
    co_memcpy_pack(dict_data(0x1A02, 1), pdo_genmap(0x2313, 1, 16), 4);
    co_memcpy_pack(dict_data(0x1A02, 2), pdo_genmap(0x6078, 0, 16), 4);
    co_memcpy_pack(dict_data(0x1A02, 3), pdo_genmap(0x2323, 2, 16), 4);
    co_memcpy_pack(dict_data(0x1A02, 4), pdo_genmap(0x2313, 2, 16), 4);
    mode = 1; co_memcpy_pack(dict_data(0x1802, 2), &mode, 1);

    // TxPDO4
    num = 4; co_memcpy_pack(dict_data(0x1A03, 0), &num, 1);
    co_memcpy_pack(dict_data(0x1A03, 1), pdo_genmap(0x2320, 1, 16), 4);
    co_memcpy_pack(dict_data(0x1A03, 2), pdo_genmap(0x2320, 2, 16), 4);
    co_memcpy_pack(dict_data(0x1A03, 3), pdo_genmap(0x2320, 3, 16), 4);
    co_memcpy_pack(dict_data(0x1A03, 4), pdo_genmap(0x2320, 4, 16), 4);
    mode = 1; co_memcpy_pack(dict_data(0x1803, 2), &mode, 1);
}

static void set_descr(void) {
    uint8_t data[4];

    memcpy(data, "IpmL", 4);
    co_memcpy_pack(dict_data(0x1018, 1), data, 4); // Vendor-ID

    memcpy(data, "ROVD", 4);
    co_memcpy_pack(dict_data(0x1018, 2), data, 4); // Product-ID

    memset(data, '\0', 4);
    data[0] = 1;
    co_memcpy_pack(dict_data(0x1018, 3), data, 4); // Revision-Num

    data[0] = 0xEF;
    data[1] = 0xBE;
    data[2] = 0xAD;
    data[3] = 0xDE;
    co_memcpy_pack(dict_data(0x1018, 4), data, 4); // Serial-Number
}

static void set_model(void) {
    static const char cname[] = "Manta50";
    static const size_t cnlen = sizeof(cname) - 1;

    static char name[6];
    static uint8_t len = 10;
    static uint32_t model = 0xC001BABE;

	dict_add(0x1008, name, &len, 1, NULL, CODF_READ);
	dict_add(0x1000, &model, &dict_dword, 1, NULL, CODF_READ);
    co_memcpy_pack(dict_data(0x1008, 0), cname, cnlen);
}

static void set_custom(void) {
	dict_add(0x6040, &controlword, &dict_word, 1, &setcw_hook, CODF_RW);
    dict_add(0x6041, &statusword, &dict_word, 1, &getsw_hook, CODF_READ);

    dict_add(0x60FF, &target_rpm, &dict_dword, 1, &setrpm_hook, CODF_RW);
    dict_add(0x606C, &current_rpm, &dict_dword, 1, &getrpm_hook, CODF_READ);
    dict_add(0x6083, &accel_rpm, &dict_dword, 1, &setaccel_hook, CODF_RW);

    dict_add(0x6078, &current, &dict_word, 1, &getcurrent_hook, CODF_READ);
    dict_add(0x2323, &temperature, temp_len, 3, &gettemp_hook, CODF_READ);
    dict_add(0x2313, vol_hum, warr_len, 3, &volhum_hook, CODF_READ);
    dict_add(0x2320, fault,   warr_len, 5, &getstatus_hook, CODF_READ);
    dict_add(0x2331, pid_k,   warr_len4, 3, &kpki_hook, CODF_RW);

#ifdef EEPROM_25AA02
    dict_add(0x1010, &profile_save, prof_save_len, 2, &profsave_hook, CODF_RW);
    dict_add(0x2000, &profile_select, &dict_word, 1, &profselect_hook, CODF_RW);
    dict_add(0x2001, profile, prof_len, 18, &prof_hook, CODF_RW);
#endif
}

static void *pdo_genmap(uint16_t index, int16_t subindex, int16_t len) {
    static uint16_t rs[4];

    rs[0] = len & 0xFF;
    rs[1] = subindex & 0xFF;
    rs[2] = index & 0xFF;
    rs[3] = index >> 8;

    return rs;
}

static uint8_t setcw_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len) {
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern MOTOR_Vars_t gMotorVars;

    if (op == CO_WRITE) {
        if (len != 2){
            return 1; // REJECT
        }

        uint16_t cw;
        co_mbyte = 0; // to co_memcpy_pack() start from bit 0, not bit 8
        co_memcpy_pack(&cw, data, 2);

		gMotorVars.Flag_enableSys = cw & 0x01;
		gMotorVars.Flag_Run_Identify = (cw>>1) & 0x01;
		gMotorVars.Flag_enableFieldWeakening = (cw>>2) & 0x01;
		gMotorVars.Flag_enableForceAngle = (cw>>3) & 0x01;
		gMotorVars.Flag_enableRsRecalc = (cw>>4) & 0x01;
		gMotorVars.Flag_enablePowerWarp = (cw>>5) & 0x01;
		gMotorVars.Flag_enableUserParams = (~(cw>>15)) & 0x01; // Automatically determine the motor parameters.
    }

    return 0;
}

static uint8_t getsw_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern MOTOR_Vars_t gMotorVars;

    if(op == CO_READ){
		statusword = (gMotorVars.Flag_enableSys & 0x01) | ((gMotorVars.Flag_Run_Identify & 0x01) << 1)
						| ((gMotorVars.Flag_enableFieldWeakening & 0x01) << 2) | ((gMotorVars.Flag_enableForceAngle & 0x01) << 3)
						| ((gMotorVars.Flag_enableRsRecalc & 0x01) << 4) | ((gMotorVars.Flag_enablePowerWarp & 0x01) << 5)
						| (((~gMotorVars.Flag_enableUserParams) & 0x01) << 15);
	}

    return 0;
}

static uint8_t setrpm_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len) {
    (void)idx;
    (void)sidx;
    extern MOTOR_Vars_t gMotorVars;

    if(op == CO_WRITE){
        if (len != 4)
            return 1; // REJECT

        int32_t rpm;
        co_mbyte = 0; // to co_memcpy_pack() start from bit 0, not bit 8
        co_memcpy_pack(&rpm, data, 4);

#ifdef EEPROM_25AA02
        if (abs(rpm) > rpmLimit)
            return 1; // REJECT
#endif

    	float krpm = ((float)rpm)/1000.0;
    	gMotorVars.SpeedRef_krpm = _IQ(krpm);
    }else if(op == CO_READ){
    	target_rpm = (int32_t)(_IQtoF(gMotorVars.SpeedRef_krpm) * 1000);
    }

    return 0;
}

static uint8_t getrpm_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern MOTOR_Vars_t gMotorVars;

    if(op == CO_READ){
    	current_rpm = (int32_t)(_IQtoF(gMotorVars.Speed_krpm) * 1000);
	}

    return 0;
}

static uint8_t setaccel_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len) {
    (void)idx;
    (void)sidx;
    extern MOTOR_Vars_t gMotorVars;

    if(op == CO_WRITE){
        if (len != 4)
            return 1; // REJECT

        int32_t rpmps;
        co_mbyte = 0; // to co_memcpy_pack() start from bit 0, not bit 8
        co_memcpy_pack(&rpmps, data, 4);

#ifdef EEPROM_25AA02
        if (abs(rpmps) > rpmpsLimit)
            return 1; // REJECT
#endif

    	float krpmps = ((float)rpmps)/1000.0;
    	gMotorVars.MaxAccel_krpmps = _IQ(krpmps);
    }else if(op == CO_READ){
    	accel_rpm = (int32_t)(_IQtoF(gMotorVars.MaxAccel_krpmps) * 1000);
    }

    return 0;
}

static uint8_t kpki_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern MOTOR_Vars_t gMotorVars;

    if(op == CO_WRITE){
    	if(len != 4){
    		return 1; // REJECT
    	}

    	float value = 0.0;
    	co_mbyte = 0;
    	//co_memcpy_pack(&value, data, 4);
    	co_memcpy_pack(&value, data, sizeof(value) << 1);

    	if(sidx == 1){
    		gMotorVars.Kp_spd = _IQ(value);
    	}else if(sidx == 2){
    		gMotorVars.Ki_spd = _IQ(value);
    	}
    }else if(op == CO_READ){
    	pid_k[0] = /*(uint16_t)*/_IQtoF(gMotorVars.Kp_spd);
    	pid_k[1] = /*(uint16_t)*/_IQtoF(gMotorVars.Ki_spd);
    }

    return 0;
}

static uint8_t gettemp_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern int measureTemperatureC();

    if(op == CO_READ){
    	temperature = (uint16_t)measureTemperatureC();
    }

    return 0;
}

static uint8_t getcurrent_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern int calcAvgCurrent();

    if(op == CO_READ){
    	current = (uint16_t)calcAvgCurrent();
    }

    return 0;
}

static uint8_t volhum_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern MOTOR_Vars_t gMotorVars;
    extern int calcHumidity();

    if(op == CO_READ){
    	if(sidx == 1){
    		vol_hum[0] = (uint16_t)(_IQtoF(gMotorVars.VdcBus_kV)*10000);
    	}else if(sidx == 2){
    		vol_hum[1] = (uint16_t)calcHumidity();
    	}
    }

    return 0;
}

static uint8_t getstatus_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    //extern void getStatRegData();
    extern HAL_Handle halHandle;

    if(op == CO_READ){
#ifdef DRV8305
    	HAL_setupSpiA(halHandle);
    	//getStatRegData();
    	fault[0] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_1);
    	fault[1] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_2);
    	fault[2] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_3);
    	fault[3] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_4);
    	HAL_setupSpi_MCP2515(halHandle);
#else
    	fault[0] = 1;
    	fault[1] = 1;
    	fault[2] = 1;
    	fault[3] = 1;
#endif

    }

    return 0;
}

#ifdef EEPROM_25AA02
static uint8_t prof_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    //extern volatile HAL_Handle halHandle;
    extern USER_Params gUserParams;
    extern MOTOR_Vars_t gMotorVars;
    extern CTRL_Handle ctrlHandle;
    extern void copyMotorParams();

    //extern uint16_t rpmLimit;
    //extern uint16_t rpmpsLimit;

    if(op == CO_WRITE){
    	if(len != 4 || gMotorVars.Flag_Run_Identify != 0){ // If the length is wrong or the motor is running, reject.
    		return 1;
    	}

        float pdata = 0.0;
        co_mbyte = 0; // to co_memcpy_pack() start from bit 0, not bit 8
        co_memcpy_pack(&pdata, data, sizeof(pdata) << 1);

    	switch(sidx){
    		case 1:
				node_id = (uint8_t)pdata;
				break;
			case 2:
				gUserParams.motor_type = (MOTOR_Type_e)pdata;
				break;
			case 3:
				gUserParams.motor_numPolePairs = (uint_least16_t)pdata;
				break;
			case 4:
				gUserParams.motor_Rr = (float_t)pdata;
				break;
			case 5:
				gUserParams.motor_Rs = (float_t)pdata;
				break;
			case 6:
				gUserParams.motor_Ls_d = (float_t)pdata;
				break;
			case 7:
				gUserParams.motor_Ls_q = (float_t)pdata;
				break;
			case 8:
				gUserParams.motor_ratedFlux = (float_t)pdata;
				break;
			case 9:
				gUserParams.IdRated = (float_t)pdata;
				break;
			case 10:
				gUserParams.maxCurrent_resEst = (float_t)pdata;
				break;
			case 11:
				gUserParams.maxCurrent_indEst = (float_t)pdata;
				break;
			case 12:
				gUserParams.maxCurrent = (float_t)pdata;
				break;
			case 13:
				gUserParams.fluxEstFreq_Hz = (float_t)pdata;
				break;
			case 14:
				rpmLimit = (uint16_t)((float_t)pdata);
				break;
			case 15:
				rpmpsLimit = (uint16_t)((float_t)pdata);
				break;
			case 16:
				gMotorVars.Kp_spd = _IQ(pdata);
				break;
			case 17:
				gMotorVars.Ki_spd = _IQ(pdata);
				break;
    	}

    	if((gUserParams.motor_Rr > (float_t)0.0) && (gUserParams.motor_Rs > (float_t)0.0)){
    		gUserParams.powerWarpGain = sqrt((float_t)1.0 + gUserParams.motor_Rr/gUserParams.motor_Rs);
    	}
    	else{
    		gUserParams.powerWarpGain = USER_POWERWARP_GAIN;
    	}
    }else if(op == CO_READ){
		profile[0] = (float)node_id;
		profile[1] = (float)gUserParams.motor_type;
		profile[2] = (float)gUserParams.motor_numPolePairs;
		profile[3] = (float)gUserParams.motor_Rr;
		profile[4] = (float)gUserParams.motor_Rs;
		profile[5] = (float)gUserParams.motor_Ls_d;
		profile[6] = (float)gUserParams.motor_Ls_q;
		profile[7] = (float)gUserParams.motor_ratedFlux;
		profile[8] = (float)gUserParams.IdRated;
		profile[9] = (float)gUserParams.maxCurrent_resEst;
		profile[10] = (float)gUserParams.maxCurrent_indEst;
		profile[11] = (float)gUserParams.maxCurrent;
		profile[12] = (float)gUserParams.fluxEstFreq_Hz;
		profile[13] = (float)rpmLimit;
		profile[14] = (float)rpmpsLimit;
		profile[15] = _IQtoF(gMotorVars.Kp_spd);
		profile[16] = _IQtoF(gMotorVars.Ki_spd);
    }

    return 0;
}
#endif

#ifdef EEPROM_25AA02
static uint8_t profselect_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern HAL_Handle halHandle;

    if(op == CO_WRITE){
    	if(len != 2){
    		return 1;
    	}

    	EEPROM_setTargetProfile(halHandle, *(uint_least8_t*)data);
    	HAL_setupSpi_MCP2515(halHandle);
    }else if(op == CO_READ){
    	HAL_setupSpi_25AA02(halHandle);
    	profile_select = (uint16_t)EEPROM_getTargetProfile(halHandle);
    	HAL_setupSpi_MCP2515(halHandle);
    	//HAL_setupSpiA(halHandle);
    }

    return 0;
}
#endif

#ifdef EEPROM_25AA02
static uint8_t profsave_hook(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len){
    (void)idx;
    (void)sidx;
    (void)ptype;
    extern HAL_Handle halHandle;
    extern USER_Params gUserParams;
    extern MOTOR_Vars_t gMotorVars;

    if(op == CO_WRITE){
    	if(len != 4){
    		return 1;
    	}

        int32_t val;
        co_mbyte = 0; // to co_memcpy_pack() start from bit 0, not bit 8
        co_memcpy_pack(&val, data, 4);

    	if((val & 0x01) != 1){
    		return 1;
    	}

    	HAL_setupSpi_25AA02(halHandle);
    	int pn = (int)EEPROM_getTargetProfile(halHandle);
    	EEPROM_saveProfile(halHandle, pn, &gUserParams, &gMotorVars, node_id);
    	HAL_setupSpi_MCP2515(halHandle);
    	//HAL_setupSpiA(halHandle);
    }

    return 0;
}
#endif

#ifdef EEPROM_25AA02
uint16_t lss_store_config(uint8_t node_id){
    extern HAL_Handle halHandle;
    extern USER_Params gUserParams;
    extern MOTOR_Vars_t gMotorVars;

	HAL_setupSpi_25AA02(halHandle);
	int pn = (int)EEPROM_getTargetProfile(halHandle);
	int success = EEPROM_saveProfile(halHandle, pn, &gUserParams, &gMotorVars, node_id);
	HAL_setupSpi_MCP2515(halHandle);

	return (success == -1 ? 2 : 0);
}
#endif
