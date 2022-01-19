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
//! \file   solutions/instaspin_foc/src/proj_lab01.c
//! \brief CPU and Inverter Set-up and introduction to interfacing to the ROM library
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//
// **************************************************************************
// the includes

// system includes
#include <math.h>

#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/memCopy/src/memCopy.h"

#include "canapi.h"
#include "main_2enc.h"

#ifdef FLASH
#pragma CODE_SECTION(motor1_ISR, "ramfuncs");
#pragma CODE_SECTION(motor2_ISR, "ramfuncs");
#endif

#pragma DATA_SECTION(ECanaRegs, "ECanaRegsFile");
volatile struct ECAN_REGS ECanaRegs;

#pragma DATA_SECTION(ECanaMboxes, "ECanaMboxesFile");
volatile struct ECAN_MBOXES ECanaMboxes;

uint32_t gCanLastStatusMsgTime = 0;

uint32_t gTimer0_stamp = 0;

HAL_Handle halHandle;                         // Handle to the Inverter hardware abstraction layer
HAL_Obj hal;

//! \brief The handles for the encoder
ENC_Handle encHandle[2];
//! \brief The encoder objects
ENC_Obj enc[2];

//! \brief The handles for the hardware abstraction layer specific to the motor
//!     board.
HAL_Handle_mtr halHandleMtr[2];
//! \brief The hardware abstraction layer object specific to the motor board.
HAL_Obj_mtr halMtr[2];

USER_Params gUserParams[2];                      // Contains the user.h settings

uint32_t previous_index_pos_1 = 0;
uint32_t previous_index_pos_2 = 0;

int32_t num_revs_1 = 0;
int32_t num_revs_2 = 0;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#endif

// **************************************************************************
// the functions

void main(void)
{
  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
      // Copy time critical code and Flash setup code to RAM
      // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
      // symbols are created by the linker. Refer to the linker files.
      memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,
              (uint16_t *)&RamfuncsRunStart);
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));

  // initialize the user parameters
  USER_setParamsMtr1(&gUserParams[HAL_MTR1]);
  USER_setParamsMtr2(&gUserParams[HAL_MTR2]);

  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams[HAL_MTR1]);

  // GPIO 16-23 (covering eQEP1)
  GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_16,
                              11);  // GPIO16-23
  // GPIO 50-55 and 56-58 (covering eQEP2)
  GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_50,
                              11);  // GPIO50-55
  GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_56,
                              11);  // GPIO56-58

  // Overwrite the settings for timer0 (we want it faster)
  uint32_t timerPeriod_cnts =
      ((uint32_t)gUserParams[0].systemFreq_MHz * 1e6l) / TIMER0_FREQ_Hz - 1;
  overwriteSetupTimer0(halHandle, timerPeriod_cnts);

  // initialize the individual motor hal files
  halHandleMtr[HAL_MTR1] = HAL_init_mtr(&halMtr[HAL_MTR1], sizeof(halMtr[HAL_MTR1]), (HAL_MtrSelect_e)HAL_MTR1);
  halHandleMtr[HAL_MTR2] = HAL_init_mtr(&halMtr[HAL_MTR2], sizeof(halMtr[HAL_MTR2]), (HAL_MtrSelect_e)HAL_MTR2);

  // Setup each motor board to its specific setting
  HAL_setParamsMtr(halHandleMtr[HAL_MTR1], halHandle, &gUserParams[HAL_MTR1]);
  HAL_setParamsMtr(halHandleMtr[HAL_MTR2], halHandle, &gUserParams[HAL_MTR2]);

  HAL_setupQEP(halHandleMtr[HAL_MTR1], 200);
  HAL_setupQEP(halHandleMtr[HAL_MTR2], 200);

  HAL_setGpioLow(halHandle, (GPIO_Number_e)HAL_Gpio_LED3);
  HAL_setGpioLow(halHandle, (GPIO_Number_e)HAL_Gpio_LED2);

  // initialize the encoder module
  encHandle[HAL_MTR1] = ENC_init(&enc[HAL_MTR1], sizeof(enc[HAL_MTR1]));
  encHandle[HAL_MTR2] = ENC_init(&enc[HAL_MTR2], sizeof(enc[HAL_MTR2]));

  // setup the encoder module
  ENC_setup(encHandle[HAL_MTR1], 1, USER_MOTOR_NUM_POLE_PAIRS,
            200, 0, USER_IQ_FULL_SCALE_FREQ_Hz,
            USER_ISR_FREQ_Hz, 8000.0);
  ENC_setup(encHandle[HAL_MTR2], 1, USER_MOTOR_NUM_POLE_PAIRS_2,
            200, 0, USER_IQ_FULL_SCALE_FREQ_Hz_2,
            USER_ISR_FREQ_Hz_2, 8000.0);

  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);

  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);

  // enable global interrupts
  HAL_enableGlobalInts(halHandle);

  // enable debug interrupts
  HAL_enableDebugInt(halHandle);

  // enable the Timer 0 interrupts
  HAL_enableTimer0Int(halHandle);
  PIE_registerTimer0IntHandler(hal.pieHandle, &timer0_ISR);

  // disable the PWM
  HAL_disablePwm(halHandleMtr[HAL_MTR1]);
  HAL_disablePwm(halHandleMtr[HAL_MTR2]);

  // Setup everything related to CAN communication
  setupCan(halHandle, &can1_ISR);


  // For ever loop
  while(true)
  {
      maybeSendCanStatusMsg();
  }

} // end of main() function


interrupt void timer0_ISR() {
    ++gTimer0_stamp;

    // acknowledge interrupt
    HAL_acqTimer0Int(halHandle);
}


void PIE_registerTimer0IntHandler(PIE_Handle pieHandle, PIE_IntVec_t isr)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    pie->TINT0 = isr;
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
}

void setCanStatusMsg() {
    CAN_StatusMsg_t status;

    // Send status message via CAN
    status.all = 0;

    CAN_setStatusMsg(status);
}


void maybeSendCanStatusMsg() {
    if (gCanLastStatusMsgTime <
        (gTimer0_stamp - TIMER0_FREQ_Hz / CAN_STATUSMSG_TRANS_FREQ_Hz)) {
        // If there is still an old message waiting for transmission, abort it
        if (CAN_checkTransmissionPending(CAN_MBOX_OUT_PIVOTENC)) {
            CAN_abort(CAN_MBOX_OUT_PIVOTENC);
        }

        //setCanStatusMsg();
        //CAN_send(CAN_MBOX_OUT_PIVOTEN;C);

        // how do i count encoder revolutions?
        //  first cast the encoder position to a signed 32-bit integer
        //  if encoder is >= 0 previously, but is <0 now, negative revolution
        //  if encoder is <0 previously, but is >=0 now, positive revolution
        //  we must send the float number of encoder revolutions. This means that
        //      we take the number of complete encoder revolutions, then add the
        //      current recorded encoder revolutions number to it.

        //HAL_Obj_mtr *halMtrObj = (HAL_Obj_mtr *)halHandleMtr[HAL_MTR1];
        //uint32_t index_posn = QEP_read_posn_count(halMtrObj->qepHandle);
        //QEP_Obj *qep = (QEP_Obj *)halMtrObj->qepHandle;
        //_iq enc_pos_1 = _IQ((float_t) qep->QPOSMAX / qep->QPOSMAX);

        //HAL_Obj_mtr *halMtrObj2 = (HAL_Obj_mtr *)halHandleMtr[HAL_MTR2];
        //uint32_t index_posn2 = QEP_read_posn_count(halMtrObj2->qepHandle);
        //QEP_Obj *qep2 = (QEP_Obj *)halMtrObj2->qepHandle;
        //_iq enc_pos_2 = _IQ((float_t) index_posn2 / qep2->QPOSMAX);


        HAL_Obj_mtr *halMtrObj = (HAL_Obj_mtr *)halHandleMtr[HAL_MTR1];
        uint32_t index_posn = QEP_read_posn_count(halMtrObj->qepHandle);
        QEP_Obj *qep = (QEP_Obj *)halMtrObj->qepHandle;

        _iq enc_pos_1;
        if ((previous_index_pos_1 >= 600 && previous_index_pos_1 <= 800)
                && (index_posn >= 0 && index_posn <= 200))
        {
            num_revs_1++;
        }
        else if ((previous_index_pos_1 >= 0 && previous_index_pos_1 <= 200)
                && (index_posn >= 600 && index_posn <=800))
        {
            num_revs_1--;
        }
        enc_pos_1 = _IQ((float_t) (num_revs_1 + ((float_t) index_posn / qep->QPOSMAX)));
        previous_index_pos_1 = index_posn;

        HAL_Obj_mtr *halMtrObj2 = (HAL_Obj_mtr *)halHandleMtr[HAL_MTR2];
        uint32_t index_posn2 = QEP_read_posn_count(halMtrObj2->qepHandle);
        QEP_Obj *qep2 = (QEP_Obj *)halMtrObj2->qepHandle;

        _iq enc_pos_2;
        if ((previous_index_pos_2 >= 600 && previous_index_pos_2 <= 800)
                && (index_posn2 >= 0 && index_posn2 <= 200))
        {
            num_revs_2++;
        }
        else if ((previous_index_pos_2 >= 0 && previous_index_pos_2 <= 200)
                && (index_posn2 >= 600 && index_posn2 <=800))
        {
            num_revs_2--;
        }
        enc_pos_2 = _IQ((float_t) (num_revs_2 + ((float_t) index_posn2 / qep2->QPOSMAX)));
        previous_index_pos_2 = index_posn2;

        CAN_setDataPivotEnc(enc_pos_1, enc_pos_2);
        CAN_send(CAN_MBOX_OUT_PIVOTENC);

        gCanLastStatusMsgTime = gTimer0_stamp;
    }
}

interrupt void motor1_ISR(void) {
    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

    return;
}  // end of motor1_ISR() function

interrupt void motor2_ISR(void) {
    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle, ADC_IntNumber_2);

    return;
}  // end of motor2_ISR() function

interrupt void can1_ISR() {
    // The same ISR is used by the eCAN module, independent of the source of the
    // interrupt.  This means, we have to distinguish the various cases here,
    // based on the values of certain registers (see SPRUH18f, section 16.13)

    // This ISR is used by GIF1

    // NOTE: SPRU074F, sec. 3.4.3.2 describes how to correctly handle all cases
    // (I don't fully understand what is meant by a "half-word read", though).

    // Since this ISR is currently only used for mailbox 0 receives, we only
    // check for this here and simply ignore other cases that call this ISR
    // (there shouldn't be any).
    // Note: ECanaRegs.CANGIF1.bit.MIV1 contains the number of the mailbox that
    // caused this interrupt (this should always be 0 for now).
    if (CAN_checkReceivedMessagePending(CAN_MBOX_IN_COMMANDS)) {

        // Acknowledge interrupt
        CAN_clearReceivedMessagePending(CAN_MBOX_IN_COMMANDS);
    }

    // acknowledge interrupt from PIE
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);
}

/*
interrupt void qep1IndexISR() { genericQepIndexISR(HAL_MTR1); }

interrupt void qep2IndexISR() { genericQepIndexISR(HAL_MTR2); }

inline void genericQepIndexISR(const HAL_MtrSelect_e mtrNum) {
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    HAL_Obj_mtr *halMtrObj = (HAL_Obj_mtr *)halHandleMtr[mtrNum];

    uint32_t index_posn = QEP_read_posn_index_latch(halMtrObj->qepHandle);

    HAL_toggleGpio(halHandle, (GPIO_Number_e)HAL_Gpio_LED3);
    HAL_toggleGpio(halHandle, (GPIO_Number_e)HAL_Gpio_LED2);

    // Convert index position from counts to mrev by dividing by the max.
    // number of counts.
    QEP_Obj *qep = (QEP_Obj *)halMtrObj->qepHandle;
    _iq index_pos_mrev = _IQ((float_t) index_posn / qep->QPOSMAX);

    CAN_setEncoderIndex(mtrNum, index_pos_mrev);
    CAN_send(CAN_MBOX_OUT_ENC_INDEX);

    // acknowledge QEP interrupt
    // for some reason I have to clear *all* flags, not only Iel
    QEP_clear_all_interrupt_flags(halMtrObj->qepHandle);
    // acknowledge interrupt from PIE group 5
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_5);
}
*/







