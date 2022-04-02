/*
 * main_2enc.h
 *
 *  Created on: Oct 7, 2021
 *      Author: eleme
 */

#ifndef MAIN_2ENC_H_
#define MAIN_2ENC_H_

#include "hal_2mtr.h"
#include "user1.h"
#include "user2.h"
#include "canapi.h"

#include "sw/modules/enc/src/32b/enc.h"

#define CAN_TRANSMISSION_TIMER_FREQ_Hz 100

#define WINDOW_SIZE 10

#define TIMER0_FREQ_Hz CAN_TRANSMISSION_TIMER_FREQ_Hz
#define TIMER0_PERIOD_S (1.0 / TIMER0_FREQ_Hz)
#define CAN_STATUSMSG_TRANS_FREQ_Hz 1

#define PI 3.141592

void PIE_registerTimer0IntHandler(PIE_Handle pieHandle, PIE_IntVec_t isr);

interrupt void can1_ISR();

interrupt void timer0_ISR();

void setEncoderStatus(const HAL_MtrSelect_e mtrNum);

void maybeSendCanStatusMsg();

void PIE_registerCan1IntHandler(PIE_Handle pieHandle, PIE_IntVec_t isr)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    pie->ECAN1INT = isr;
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
}

void setupCan(HAL_Handle halHandle, PIE_IntVec_t can1_ISR)
{
    HAL_Obj *hal = (HAL_Obj*)halHandle;

    // Setup CAN interface
    CAN_initECanaGpio(halHandle);
    CAN_initECana();
    CAN_setupMboxes();

    // Set ISR for CAN interrupt
    PIE_registerCan1IntHandler(hal->pieHandle, can1_ISR);
    // Enable the ECANA1 interrupt in PIE group 9
    PIE_enableInt(hal->pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_ECANA1);
    // Enable the cpu interrupt for PIE group 9 interrupts
    CPU_enableInt(hal->cpuHandle, CPU_IntNumber_9);
}

void overwriteSetupTimer0(HAL_Handle handle, const uint32_t timerPeriod_counts)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  // use timer 0 for CAN transmissions
  TIMER_setDecimationFactor(obj->timerHandle[0], 0);
  TIMER_setEmulationMode(obj->timerHandle[0], TIMER_EmulationMode_RunFree);
  TIMER_setPeriod(obj->timerHandle[0], timerPeriod_counts);
  TIMER_setPreScaler(obj->timerHandle[0], 0);

  return;
}  // end of HAL_setupTimers() function

#endif /* MAIN_2ENC_H_ */
