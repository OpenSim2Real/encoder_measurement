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

#define CAN_TRANSMISSION_TIMER_FREQ_Hz 1000
#define CAN_STATUSMSG_TRANS_FREQ_Hz 10

#define TIMER0_FREQ_Hz CAN_TRANSMISSION_TIMER_FREQ_Hz

void PIE_registerTimer0IntHandler(PIE_Handle pieHandle, PIE_IntVec_t isr);

interrupt void can1_ISR();

interrupt void timer0_ISR();

void maybeSendCanStatusMsg();

/*
interrupt void qep1IndexISR();

interrupt void qep2IndexISR();

inline void genericQepIndexISR(const HAL_MtrSelect_e mtrNum);
*/

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


/*
void setupQepIndexInterrupt(HAL_Handle halHandle, HAL_Handle_mtr halHandleMtr[2],
        PIE_IntVec_t qep1IndexIsr, PIE_IntVec_t qep2IndexIsr)
{
    uint_least8_t mtrNum;
    HAL_Obj *halObj = (HAL_Obj *)halHandle;
    HAL_Obj_mtr *halMtrObj;
    PIE_Obj *pie = (PIE_Obj *)halObj->pieHandle;

    // specify ISRs
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    pie->EQEP1_INT = qep1IndexIsr;
    pie->EQEP2_INT = qep2IndexIsr;
    //PIE_registerPieIntHandler(obj->pieHandle, PIE_GroupNumber_5, PIE_InterruptSource_EQEP1, &qepIndexISR);
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    for(mtrNum=HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++)
    {
        halMtrObj = (HAL_Obj_mtr *)halHandleMtr[mtrNum];

        // enable QEP interrupt for index
        QEP_clear_all_interrupt_flags(halMtrObj->qepHandle);
        QEP_enable_interrupt(halMtrObj->qepHandle, QEINT_Iel);
    }

    // enable the corresponding interrupts in PIE (group 5)
    PIE_enableInt(halObj->pieHandle, PIE_GroupNumber_5, PIE_InterruptSource_EQEP1);
    PIE_enableInt(halObj->pieHandle, PIE_GroupNumber_5, PIE_InterruptSource_EQEP2);

    // finally enable the CPU interrupt for PIE group 5 interrupts
    CPU_enableInt(halObj->cpuHandle, CPU_IntNumber_5);
}
*/

#endif /* MAIN_2ENC_H_ */
