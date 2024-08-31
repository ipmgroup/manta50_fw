/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   base on solutions/instaspin_foc/src/proj_lab09.c
//! \brief Automatic field weakening
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB09 PROJ_LAB09
//@{

//! \defgroup PROJ_LAB09_OVERVIEW Project Overview
//!
//! Experimentation with Field Weakening
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main_env.h"

//C Standard libs
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef MCP2515
// #include "mcp2515.h"
#include "can.h"
#include "can-fifo.h"
#endif

//Canopen
#ifdef CANOPEN
#include "canopen-init.h"
//#include "common.h"
#include "canopen.h"
#include "canopen-pdo.h"
#endif



//Other includes
#include "ADC_temp.h"
//#include "Microchip_25AA02E48.h"
#ifdef EEPROM_25AA02
#include "eeprom-pm.h"
#endif

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function


// **************************************************************************
// the defines

#define CURRENT_BUF_LEN 32
#define CURRENT_ADJUSTMENT_INTERCEPT 0
#define CURRENT_ADJUSTMENT_SLOPE     1
int raw_current = 0;

#define LED_BLINK_FREQ_Hz   5
#define nFault_FREQ_Hz   1

#define SPEED_BASE_KRPM     3.5// T200_R1 //7.7  // Max motor speed for DJI E300 920 Kv at 11 V

// **************************************************************************
// the globals

#ifdef CANOPEN
extern uint16_t fault[4];
#endif

int currentBuf[CURRENT_BUF_LEN] = {0};
uint8_t cPos = 0;

int resetTrigger = 0;

#ifdef MCP2515
uint8_t SPI_Buf_R[8] = {0};//{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t SPI_Buf_W[8] = {0};
uint8_t SPI_ReadByte = 0;
#endif //MCP2515

#ifdef DEBUG
int ISRCount = 0;
int repeatReadCounter = 0;
int maxRepeatRead = 0;
bool interruptPinState = 0;
int canFrameCounter = 0;
int mcpError = 0;
int emptyRead = 0;
int mcpResetCounter = 0;

int canWriteErrorCounter = 0;
long mainIsrCounter = 0;
long syncTimes[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int nextSyncPos = -1;
long maxSyncTime = 0;
long diff = 0;
long averageSyncDiff = 0;
#endif

uint32_t canInactivityCounter = 0;
int resetMcpTrigger = 0;
float mcpInactivityResetFreq = 0.3;

#ifdef nFault
bool Flag_nFault = 0;
#endif

// **************************************************************************

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

#ifdef F2802xF
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif
HAL_Handle halHandle;

#ifdef F2802xF
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif
USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
#ifdef F2802xF
#pragma DATA_SECTION(ctrl,"rom_accessed_data");
#endif
CTRL_Obj ctrl;				//v1p7 format
#endif

#ifdef LED_ON
uint16_t gLEDcnt = 0;
#endif

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef F2802xF
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif

#endif

FW_Obj fw;
FW_Handle fwHandle;

_iq Iq_Max_pu;


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;


int isr_failsafe_count = 0;
char devId = '0';
bool failsafe = 1;

#ifdef nFault
uint16_t gFaultcnt = 0;
#endif

// **************************************************************************
// the functions

#ifdef CANOPEN
void saveCanData();
void processCanData();
void sendPdoOnFault();
#endif

#ifdef MCP2515
void configMCP2515();
#endif

void copyMotorParams();

int measureTemperatureC() {
    uint16_t chanBak = halHandle->adcHandle->ADCSOCxCTL[ADC_SocNumber_0];

    // connect channel A5 internally to the temperature sensor
    ADC_setTempSensorSrc(halHandle->adcHandle, ADC_TempSensorSrc_Int);
    // set SOC0 channel select to ADCINA5
    ADC_setSocChanNumber(halHandle->adcHandle, ADC_SocNumber_0, ADC_SocChanNumber_A5);

    uint16_t temp = ADC_readResult(halHandle->adcHandle, ADC_ResultNumber_0);

    halHandle->adcHandle->ADCSOCxCTL[ADC_SocNumber_0] = chanBak;

    return (int)ADC_getTemperatureC(halHandle->adcHandle, temp);
}

int calcAvgCurrent() {
    long currentValue = 0;

    int i = 0;
    for(i = 0; i < CURRENT_BUF_LEN; i++) {
        currentValue += currentBuf[i];// * current[i];
    }
    currentValue = currentValue / CURRENT_BUF_LEN;
    return (int)currentValue;
}

int calcHumidity() {
    uint16_t raw = ADC_readResult(halHandle->adcHandle,ADC_ResultNumber_8); //3.3v = 4095
    float V = (3.300/4096.0 * raw);

    int tempC = measureTemperatureC();
    float humidity = -23.82075472 + 47.64627406 * V;
    humidity = humidity/(1.0546-0.00216*tempC); //adjusted for temperature
    //sprintf(dataBuffer, "%d", (int)humidity);
    return (int)humidity;
}

int initReset() {
    resetTrigger = 1;
    gMotorVars.Flag_enableSys = 0;
    return 0;
}

int resetDevice() {
#ifdef DEBUG
    int i;
    for (i = 1; i < 12; ++i) {
        syncTimes[i - 1] = syncTimes[i] - syncTimes[i - 1];
    }
#endif

    //gMotorVars.Flag_enableSys = 0; // Disable system if reset is requested.
    MCP2515_reset(halHandle->mcp2515Handle);

    WDOG_disable(halHandle->wdogHandle);

    EALLOW;
    halHandle->wdogHandle->SCSR = 0x0;    // Set wdog overflow for device reset.
    halHandle->wdogHandle->WDCR = 0x0;    // Set WDCHK 000 to trigger reset.
    EDIS;

    WDOG_enable(halHandle->wdogHandle);

    while(1) {
    }

    return 0;
}

int resetMcp() {
#ifdef DEBUG
    mcpResetCounter++;
#endif

    HAL_setupSpi_MCP2515(halHandle);
    MCP2515_reset(halHandle->mcp2515Handle);
    configMCP2515();
    HAL_setupSpiA(halHandle);

    resetMcpTrigger = 0;

    return 0;
}

// **************************************************************************

void main(void)
{
    uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
    uint_least8_t ctrlNumber = 0;
#endif

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the project
#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);

#ifdef F2802xF
    //copy .econst to unsecure RAM
    if(*econst_end - *econst_start)
    {
        memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
    }

    //copy .switch ot unsecure RAM
    if(*switch_end - *switch_start)
    {
        memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
    }
#endif

#endif

    // initialize the hardware abstraction layer
    halHandle = HAL_init(&hal,sizeof(hal));


    // check for errors in user parameters
    USER_checkForErrors(&gUserParams);


    // store user parameter error in global variable
    gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


    // do not allow code execution if there is a user parameter error
    if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
        for(;;)
        {
            gMotorVars.Flag_enableSys = false;
        }
    }

    // initialize the user parameters
    USER_setParams(&gUserParams);


    // set the hardware abstraction layer parameters
    HAL_setParams(halHandle,&gUserParams);


    // initialize the controller
#ifdef FAST_ROM_V1p6
    ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
    controller_obj = (CTRL_Obj *)ctrlHandle;
#else
    ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif


    {
        CTRL_Version version;

        // get the version number
        CTRL_getVersion(ctrlHandle,&version);

        gMotorVars.CtrlVersion = version;
    }

    // ILM70-10 custom values for speed controller
    gMotorVars.Kp_spd = _IQ(3.500);//_IQ(2.000);
    gMotorVars.Ki_spd = _IQ(0.059);

#ifdef EEPROM_25AA02
    EEPROM_init_pm(halHandle);
    EEPROM_disableWP(halHandle);
    EEPROM_startupProfileLoad(halHandle, &gUserParams, &gMotorVars);
#endif


    // set the default controller parameters
    CTRL_setParams(ctrlHandle,&gUserParams);


    // Initialize field weakening
    fwHandle = FW_init(&fw,sizeof(fw));


    // Disable field weakening
    FW_setFlag_enableFw(fwHandle, false);


    // Clear field weakening counter
    FW_clearCounter(fwHandle);


    // Set the number of ISR per field weakening ticks
    FW_setNumIsrTicksPerFwTick(fwHandle, FW_NUM_ISR_TICKS_PER_CTRL_TICK);


    // Set the deltas of field weakening
    FW_setDeltas(fwHandle, FW_INC_DELTA, FW_DEC_DELTA);


    // Set initial output of field weakening to zero
    FW_setOutput(fwHandle, _IQ(0.0));


    // Set the field weakening controller limits
    FW_setMinMax(fwHandle,_IQ(USER_MAX_NEGATIVE_ID_REF_CURRENT_A/USER_IQ_FULL_SCALE_CURRENT_A),_IQ(0.0));


    // setup faults
    HAL_setupFaults(halHandle);


    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);


    // enable the ADC interrupts
    HAL_enableAdcInts(halHandle);


    // enable global interrupts
    HAL_enableGlobalInts(halHandle);


    // enable debug interrupts
    HAL_enableDebugInt(halHandle);


    // disable the PWM
    HAL_disablePwm(halHandle);


#ifdef DRV8301_SPI
    // turn on the DRV8301 if present
    HAL_enableDrv(halHandle);
    // initialize the DRV8301 interface
    HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif

#ifdef DRV8305_SPI
    // turn on the DRV8305 if present
    HAL_enableDrv(halHandle);
    // initialize the DRV8305 interface
    HAL_setupDrvSpi(halHandle,&gDrvSpi8305Vars);
#endif

    // enable DC bus compensation
    CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


    // compute scaling factors for flux and torque calculations
    gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
    gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
    gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
    gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

    //Default values
    gMotorVars.Flag_enableSys = 1;
    gMotorVars.Flag_Run_Identify = 0;
    gMotorVars.Flag_enableFieldWeakening = 1;
    gMotorVars.Flag_enableForceAngle = 1;
    gMotorVars.Flag_enableRsRecalc = 1;
    gMotorVars.Flag_enablePowerWarp = 1;
    gMotorVars.MaxAccel_krpmps = _IQ(1.0);
    gMotorVars.SpeedRef_krpm = _IQ(0.0);
    // ILM70-10 custom values for speed controller. Has to be set before the values will be loaded from EEPROM.
    //gMotorVars.Kp_spd = _IQ(3.500);//_IQ(2.000);
    //gMotorVars.Ki_spd = _IQ(0.059);

#ifdef DEBUG
    interruptPinState = GPIO_read(halHandle->gpioHandle, halHandle->mcp2515Handle->gpio_INT); // 1 = no data.
#endif

#ifdef MCP2515
    HAL_setupSpi_MCP2515(halHandle);
    configMCP2515();
#ifdef DEBUG
    mcpError = MCP2515_checkError(halHandle->mcp2515Handle);
#endif
#ifdef CANOPEN
    canopen_init(); // CAN message is sent from this function
    initCanBuffer();
#endif
    HAL_setupSpiA(halHandle);
#endif

    for(;;)
    {
        // Waiting for enable system flag to be set
        while(!(gMotorVars.Flag_enableSys)) {
            if(resetMcpTrigger) {
                gMotorVars.SpeedRef_krpm = _IQ(0.0);
                resetMcp();
            }
            if(resetTrigger) {
                resetDevice();
            }
#ifdef CANOPEN
            HAL_setupSpi_MCP2515(halHandle);
            if (getRcvFlag(halHandle->mcp2515Handle)) {
                saveCanData();
            }
            processCanData();
            HAL_setupSpiA(halHandle);
#endif
        }

        Flag_Latch_softwareUpdate = true;

        // Enable the Library internal PI.  Iq is referenced by the speed PI now
        CTRL_setFlag_enableSpeedCtrl(ctrlHandle, true);

        // loop while the enable system flag is true
        while(gMotorVars.Flag_enableSys)
        {
            if(resetMcpTrigger) {
                gMotorVars.SpeedRef_krpm = _IQ(0.0);
                resetMcp();
            }
#ifdef DEBUG_CURRENT
            raw_current = calcAvgCurrent();
#endif
#ifdef CANOPEN
            HAL_setupSpi_MCP2515(halHandle);
            if (getRcvFlag(halHandle->mcp2515Handle)) {
                saveCanData();
            }
            processCanData();
            HAL_setupSpiA(halHandle);
#endif

            CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

            // increment counters
            gCounter_updateGlobals++;

            // enable/disable the use of motor parameters being loaded from user.h
            CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);

            // enable/disable Rs recalibration during motor startup
            EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

            // enable/disable automatic calculation of bias values
            CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


            if(CTRL_isError(ctrlHandle))
            {
                // set the enable controller flag to false
                CTRL_setFlag_enableCtrl(ctrlHandle,false);

                // set the enable system flag to false
                gMotorVars.Flag_enableSys = false;

                // disable the PWM
                HAL_disablePwm(halHandle);
            }
            else
            {
                // update the controller state
                bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

                // enable or disable the control
                CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

                if(flag_ctrlStateChanged)
                {
                    CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                    if(ctrlState == CTRL_State_OffLine)
                    {
                        // enable the PWM
                        HAL_enablePwm(halHandle);
                    }
                    else if(ctrlState == CTRL_State_OnLine)
                    {
                        if(gMotorVars.Flag_enableOffsetcalc == true)
                        {
                            // update the ADC bias values
                            HAL_updateAdcBias(halHandle);
                        }
                        else
                        {
                            // set the current bias
                            HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                            HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                            HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

                            // set the voltage bias
                            HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                            HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                            HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                        }

                        // Return the bias value for currents
                        gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0);
                        gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1);
                        gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2);

                        // Return the bias value for voltages
                        gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0);
                        gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1);
                        gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2);

                        // enable the PWM
                        HAL_enablePwm(halHandle);
                    }
                    else if(ctrlState == CTRL_State_Idle)
                    {
                        // disable the PWM
                        HAL_disablePwm(halHandle);
                        gMotorVars.Flag_Run_Identify = false;
                    }

                    if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                            (ctrlState > CTRL_State_Idle) &&
                            (gMotorVars.CtrlVersion.minor == 6))
                    {
                        // call this function to fix 1p6
                        USER_softwareUpdate1p6(ctrlHandle);
                    }

                }
            }


            if(EST_isMotorIdentified(obj->estHandle))
            {
                _iq Is_Max_squared_pu = _IQ((USER_MOTOR_MAX_CURRENT*USER_MOTOR_MAX_CURRENT)/  \
                                            (USER_IQ_FULL_SCALE_CURRENT_A*USER_IQ_FULL_SCALE_CURRENT_A));
                _iq Id_squared_pu = _IQmpy(CTRL_getId_ref_pu(ctrlHandle),CTRL_getId_ref_pu(ctrlHandle));

                // Take into consideration that Iq^2+Id^2 = Is^2
                Iq_Max_pu = _IQsqrt(Is_Max_squared_pu-Id_squared_pu);

                //Set new max trajectory
                CTRL_setSpdMax(ctrlHandle, Iq_Max_pu);

                // set the current ramp
                EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
                gMotorVars.Flag_MotorIdentified = true;

                // set the speed reference
                CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

                // set the speed acceleration
                CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

                if(Flag_Latch_softwareUpdate)
                {
                    Flag_Latch_softwareUpdate = false;

                    USER_calcPIgains(ctrlHandle);

                    // initialize the watch window kp and ki current values with pre-calculated values
                    gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle,CTRL_Type_PID_Id);
                    gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle,CTRL_Type_PID_Id);

                    copyMotorParams();
                }

            }
            else
            {
                Flag_Latch_softwareUpdate = true;

                // initialize the watch window kp and ki values with pre-calculated values
                //gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle,CTRL_Type_PID_spd); //Interferes with the control of these variables via can.
                //gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle,CTRL_Type_PID_spd);


                // the estimator sets the maximum current slope during identification
                gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
            }


            // when appropriate, update the global variables
            if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
            {
                // reset the counter
                gCounter_updateGlobals = 0;

                updateGlobalVariables_motor(ctrlHandle);
            }


            // update Kp and Ki gains
            updateKpKiGains(ctrlHandle);

            // set field weakening enable flag depending on user's input
            FW_setFlag_enableFw(fwHandle,gMotorVars.Flag_enableFieldWeakening);

            // enable/disable the forced angle
            EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

            // enable or disable power warp
            CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
            HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

            HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI

            if(Flag_nFault) {
                #ifdef CANOPEN
                sendPdoOnFault();
                #endif

                Flag_nFault = 0;
            }

            HAL_writeDrvData(halHandle,&gDrvSpi8305Vars);

            HAL_readDrvData(halHandle,&gDrvSpi8305Vars);
#endif
        } // end of while(gFlag_enableSys) loop


        // disable the PWM
        HAL_disablePwm(halHandle);

        // set the default controller parameters (Reset the control to re-identify the motor)
        CTRL_setParams(ctrlHandle,&gUserParams);
        gMotorVars.Flag_Run_Identify = false;

    } // end of for(;;) loop

} // end of main() function


interrupt void mainISR(void)
{
#ifdef LED_ON
    // toggle status LED
    if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
    {
        HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
        gLEDcnt = 0;
    }
#endif

#ifdef DEBUG
    mainIsrCounter++;
#endif


#ifdef nFault
    if(gFaultcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / nFault_FREQ_Hz))
    {
        //gDrvSpi8305Vars.ReadCmd = true;
        Flag_nFault = 1;

        gFaultcnt = 0;
    }
#endif

    canInactivityCounter++;
    // If MCP reset frequency is greater than 0, activate it.
    if(mcpInactivityResetFreq > 0) {
        uint_least32_t mcpResetCycle = (uint_least32_t)(((float)USER_ISR_FREQ_Hz) / ((float)mcpInactivityResetFreq)); // Number of the cycle at which the MCP has to be reset.
        if(((canInactivityCounter % mcpResetCycle) + 1) >= mcpResetCycle && resetMcpTrigger == 0) { // Increase counter, and if it's time, set flag to reset MCP.
            resetMcpTrigger = 1;
        }
    }

    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


    // convert the ADC data
    HAL_readAdcData(halHandle,&gAdcData);


    // run the controller
    CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData);


    // write the PWM compare values
    HAL_writePwmData(halHandle,&gPwmData);


    if(FW_getFlag_enableFw(fwHandle) == true)
    {
        FW_incCounter(fwHandle);

        if(FW_getCounter(fwHandle) > FW_getNumIsrTicksPerFwTick(fwHandle))
        {
            _iq refValue;
            _iq fbackValue;
            _iq output;

            FW_clearCounter(fwHandle);

            refValue = gMotorVars.VsRef;

            fbackValue = gMotorVars.Vs;

            FW_run(fwHandle, refValue, fbackValue, &output);

            CTRL_setId_ref_pu(ctrlHandle, output);

            gMotorVars.IdRef_A = _IQmpy(CTRL_getId_ref_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
        }
    }
    else
    {
        CTRL_setId_ref_pu(ctrlHandle, _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));
    }

    // setup the controller
    CTRL_setup(ctrlHandle);


    return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    // get the speed estimate
    gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

    // get the real time speed reference coming out of the speed trajectory generator
    gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

    // get the torque estimate
    gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

    // get the magnetizing current
    gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

    // get the rotor resistance
    gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

    // get the stator resistance
    gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

    // get the stator inductance in the direct coordinate direction
    gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

    // get the stator inductance in the quadrature coordinate direction
    gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

    // get the flux in V/Hz in floating point
    gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

    // get the flux in Wb in fixed point
    gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

    // get the controller state
    gMotorVars.CtrlState = CTRL_getState(handle);

    // get the estimator state
    gMotorVars.EstState = EST_getState(obj->estHandle);

    // read Vd and Vq vectors per units
    gMotorVars.Vd = CTRL_getVd_out_pu(ctrlHandle);
    gMotorVars.Vq = CTRL_getVq_out_pu(ctrlHandle);

    // calculate vector Vs in per units
    gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

    // read Id and Iq vectors in amps
    gMotorVars.Id_A = _IQmpy(CTRL_getId_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
    gMotorVars.Iq_A = _IQmpy(CTRL_getIq_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

    float Id_A = _IQtoF(gMotorVars.Id_A);
    float Iq_A = _IQtoF(gMotorVars.Iq_A);
    float Is_A = sqrt((Id_A * Id_A) + (Iq_A * Iq_A));

    // calculate vector Is in amps
    gMotorVars.Is_A = _IQ(Is_A); //_IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));

    // Get the DC buss voltage
    gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));


    float Ifloat = (_IQtoF(gMotorVars.Is_A)/((float)1.414214));
    currentBuf[cPos] = (int)(CURRENT_ADJUSTMENT_INTERCEPT + CURRENT_ADJUSTMENT_SLOPE * (Ifloat * 1000.0));
    if(cPos < CURRENT_BUF_LEN-1) {
        cPos++;
    } else {
        cPos = 0;
    }

    return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
    if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
        // set the kp and ki speed values from the watch window
        CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
        CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

        // set the kp and ki current values for Id and Iq from the watch window
        CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
        CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
        CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
        CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
    }

    return;
} // end of updateKpKiGains() function


#ifdef MCP2515
/*********************************************************************************************************
** Function name:           MCP2515_rcvISR
** Descriptions:            sets canRcvFlag to true when there is data in the rcv buffer
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
__interrupt void MCP2515_rcvISR(void) {
#ifdef DEBUG
    ISRCount++;
#endif
    HAL_Obj *obj = (HAL_Obj *) halHandle;
    setRcvFlag(obj->mcp2515Handle, true);

    canInactivityCounter = 0;

    //PIE_clearInt(obj->pieHandle,PIE_GroupNumber_1);
}
#endif

#ifdef CANOPEN
void getStatRegData() {
    fault[0] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_1);
    fault[1] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_2);
    fault[2] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_3);
    fault[3] = DRV8305_readSpi(halHandle->drv8305Handle, Address_Status_4);
}
#endif

#ifdef CANOPEN
void saveCanData() {
    canMessage *msg_h;
    uint8_t msgRead;
    int i;

    //HAL_setupSpi_MCP2515(halHandle);

    // As long as data is available and the buffer isn't full, attempt reading the data.
    for(i = 0; (MCP2515_checkReceive(halHandle->mcp2515Handle) == CAN_MSGAVAIL) && i < BUFFER_SIZE; i++) {
        msg_h = getHeadAdv();
        // If the buffer is full, abort.
        if(msg_h == NULL) {
            break;
        }
        msgRead = can_read(&(msg_h->id), msg_h->msg, &(msg_h->len));
        // If reading was unsuccessful, mark the entry as invalid.
        // TODO: Check if higher 8 bits are 0 in msg_h->msg[i].
        if(!msgRead || (msg_h->id == 0 && msg_h->msg[1] != node_id && msg_h->msg[1] != 0)) {
            releaseHead();
            //msg_h->valid = 0;
        }
    }

    //HAL_setupSpiA(halHandle);

    if(GPIO_read(halHandle->gpioHandle, halHandle->mcp2515Handle->gpio_INT)) { //If MCP signals no more data in the buffer.
        setRcvFlag(halHandle->mcp2515Handle, 0);
        PIE_clearInt(((HAL_Obj *)halHandle)->pieHandle, PIE_GroupNumber_1);
#ifdef DEBUG
        if(repeatReadCounter > maxRepeatRead) maxRepeatRead = repeatReadCounter;
        repeatReadCounter = 0;
#endif
    } else {
        setRcvFlag(halHandle->mcp2515Handle, 1);
#ifdef DEBUG
        repeatReadCounter++;
#endif
    }
}

void processCanData() {
    canMessage *msg_t = getTailAdv();
    // If there is valid data in the buffer, pass it to the canopen library.
    if((msg_t != NULL)/* && msg_t->valid*/) {
        //HAL_setupSpi_MCP2515(halHandle);
        co_dispatch(msg_t->id, msg_t->msg, msg_t->len);
        //HAL_setupSpiA(halHandle);
    }
}
#endif

#ifdef CANOPEN
// TODO: Replace this function with an emergency message.
void sendPdoOnFault() {
    if(DRV8305_isFault(halHandle->drv8305Handle)) {
        //Flag_nFault = 1;
        gMotorVars.Flag_Run_Identify = 0;

        //gDrvSpi8305Vars.ReadCmd = true;

        getStatRegData();

        //Reset fault.
        gDrvSpi8305Vars.ManWriteAddr = 0x09;
        gDrvSpi8305Vars.ManWriteData = 0x22;
        gDrvSpi8305Vars.ManWriteCmd = true;

        //HAL_setupSpi_MCP2515(halHandle);
        //pdo_write(3);
        //HAL_setupSpiA(halHandle);
    }
}
#endif

#ifdef MCP2515
void configMCP2515() {
    //HAL_setupSpi_MCP2515(halHandle);
    SPI_ReadByte = MCP2515_begin(halHandle->mcp2515Handle, CAN_500KBPS);

    MCP2515_init_Mask(halHandle->mcp2515Handle, 0, 0, 0x007F);
#ifdef EEPROM_25AA02
    MCP2515_init_Filt(halHandle->mcp2515Handle, 0, 0, node_id); // RxPDO and SDO
#else
    MCP2515_init_Filt(halHandle->mcp2515Handle, 0, 0, NODE_ID); // RxPDO and SDO
#endif
    MCP2515_init_Filt(halHandle->mcp2515Handle, 1, 0, 0x0000);

    MCP2515_init_Mask(halHandle->mcp2515Handle, 1, 0, 0x07FF); //0x007F
#ifdef EEPROM_25AA02
    MCP2515_init_Filt(halHandle->mcp2515Handle, 2, 0, node_id);
#else
    MCP2515_init_Filt(halHandle->mcp2515Handle, 2, 0, NODE_ID);
#endif
    MCP2515_init_Filt(halHandle->mcp2515Handle, 3, 0, 0x0000);
    MCP2515_init_Filt(halHandle->mcp2515Handle, 4, 0, 0x07E5);
    MCP2515_init_Filt(halHandle->mcp2515Handle, 5, 0, 0x0080);

    //HAL_setupSpiA(halHandle);
}
#endif


void copyMotorParams() {
//		gUserParams.motor_type = gMotorVars.???;  // Set only manually.
//		gUserParams.motor_numPolePairs = gMotorVars.???; // Set only manually.
    gUserParams.motor_Rr = gMotorVars.Rr_Ohm;
    gUserParams.motor_Rs = gMotorVars.Rs_Ohm;
    gUserParams.motor_Ls_d = gMotorVars.Lsd_H;
    gUserParams.motor_Ls_q = gMotorVars.Lsq_H;
    gUserParams.motor_ratedFlux = gMotorVars.Flux_VpHz;
    gUserParams.IdRated = gMotorVars.IdRef_A;
//		gUserParams.maxCurrent_resEst = gMotorVars.???; // Set only manually.
//		gUserParams.maxCurrent_indEst = gMotorVars.???; // Set only manually.
//		gUserParams.maxCurrent = gMotorVars.???; // Set only manually.
//		gUserParams.fluxEstFreq_Hz = gMotorVars.???; // Set only manually.
}

//@} //defgroup
// end of file



