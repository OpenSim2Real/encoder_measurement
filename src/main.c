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

int32_t gNumRevs[2];

uint32_t gPreviousEncPos[2];
float gPreviousEncPosition[2][3];
uint32_t gQepPosMax[2];

float gEncPosQueue[2][WINDOW_SIZE];
uint32_t gEncPosQueueIndex[2];
float gEncPosRolling[2];

float gEncVelQueue[2][WINDOW_SIZE];
uint32_t gEncVelQueueIndex[2];
float gEncVelRolling[2];

float gEncAccQueue[2][WINDOW_SIZE];
uint32_t gEncAccQueueIndex[2];
float gEncAccRolling[2];

uint32_t index;

//! Last time a status message was sent via CAN (based on gTimer0_stamp).
uint32_t gCanLastStatusMsgTime = 0;

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

  HAL_setupQEP(halHandleMtr[HAL_MTR1], 5000);
  HAL_setupQEP(halHandleMtr[HAL_MTR2], 5000);

  HAL_setGpioLow(halHandle, (GPIO_Number_e)HAL_Gpio_LED3);
  HAL_setGpioLow(halHandle, (GPIO_Number_e)HAL_Gpio_LED2);

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

  gQepPosMax[HAL_MTR1] = halHandleMtr[HAL_MTR1]->qepHandle->QPOSMAX;
  gQepPosMax[HAL_MTR2] = halHandleMtr[HAL_MTR2]->qepHandle->QPOSMAX;

  gPreviousEncPosition[HAL_MTR1][0] = 0.0;
  gPreviousEncPosition[HAL_MTR1][1] = 0.0;
  gPreviousEncPosition[HAL_MTR1][2] = 0.0;
  gPreviousEncPosition[HAL_MTR2][0] = 0.0;
  gPreviousEncPosition[HAL_MTR2][1] = 0.0;
  gPreviousEncPosition[HAL_MTR2][2] = 0.0;

  gPreviousEncPos[HAL_MTR1] = 0;
  gPreviousEncPos[HAL_MTR2] = 0;

  gNumRevs[HAL_MTR1] = 0;
  gNumRevs[HAL_MTR2] = 0;

  // Initialize queues
  for (index = 0; index < WINDOW_SIZE; index++)
  {
      gEncPosQueue[HAL_MTR1][index] = 0.0;
      gEncPosQueue[HAL_MTR2][index] = 0.0;

      gEncVelQueue[HAL_MTR1][index] = 0.0;
      gEncVelQueue[HAL_MTR2][index] = 0.0;

      gEncAccQueue[HAL_MTR1][index] = 0.0;
      gEncAccQueue[HAL_MTR2][index] = 0.0;
  }
  gEncPosQueueIndex[HAL_MTR1] = 0;
  gEncPosQueueIndex[HAL_MTR2] = 0;
  gEncVelQueueIndex[HAL_MTR1] = 0;
  gEncVelQueueIndex[HAL_MTR2] = 0;
  gEncAccQueueIndex[HAL_MTR1] = 0;
  gEncAccQueueIndex[HAL_MTR2] = 0;

  gEncPosRolling[HAL_MTR1] = 0.0;
  gEncPosRolling[HAL_MTR2] = 0.0;
  gEncVelRolling[HAL_MTR1] = 0.0;
  gEncVelRolling[HAL_MTR2] = 0.0;
  gEncAccRolling[HAL_MTR1] = 0.0;
  gEncAccRolling[HAL_MTR2] = 0.0;

  // For ever loop
  while(true)
  {
      maybeSendCanStatusMsg();
  }

} // end of main() function


interrupt void timer0_ISR() {
    ++gTimer0_stamp;

    // If there is still an old message waiting for transmission, abort it
    if (CAN_checkTransmissionPending(CAN_MBOX_ALL)) {
        CAN_abort(CAN_MBOX_ALL);
    }

    setEncoderStatus(HAL_MTR1);
    setEncoderStatus(HAL_MTR2);

    CAN_send((uint32_t) CAN_MBOX_OUT_PIVOTENC \
             | CAN_MBOX_OUT_PIVOTENCVEL \
             | CAN_MBOX_OUT_PIVOTENCACC);

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

void setEncoderStatus(const HAL_MtrSelect_e mtrNum) {

    // get the encoder position first

    uint32_t index_posn = QEP_read_posn_count(halHandleMtr[mtrNum]->qepHandle);

    float encPos, encVel, encAcc;
    if ((gPreviousEncPos[mtrNum] >= (600*25) && gPreviousEncPos[mtrNum] <= (800*25))
            && (index_posn <= (200*25)))
    {
        gNumRevs[mtrNum]++;
    }
    else if ((gPreviousEncPos[mtrNum] <= (25*200))
            && (index_posn >= (25*600) && index_posn <= (25*800)))
    {
        gNumRevs[mtrNum]--;
    }
    gPreviousEncPos[mtrNum] = index_posn;
    encPos = (float_t) (gNumRevs[mtrNum] + ((float_t) index_posn / gQepPosMax[mtrNum]));

    gPreviousEncPosition[mtrNum][0] = gPreviousEncPosition[mtrNum][1];
    gPreviousEncPosition[mtrNum][1] = gPreviousEncPosition[mtrNum][2];
    gPreviousEncPosition[mtrNum][2] = encPos;

    // Add encoder position to queue and update rolling average
    gEncPosRolling[mtrNum] = gEncPosRolling[mtrNum] + (1.0 / WINDOW_SIZE)
            * (encPos - gEncPosQueue[mtrNum][gEncPosQueueIndex[mtrNum]]);
    gEncPosQueue[mtrNum][gEncPosQueueIndex[mtrNum]] = encPos;
    gEncPosQueueIndex[mtrNum] = (gEncPosQueueIndex[mtrNum] + 1) % WINDOW_SIZE;

    // get the encoder velocity and convert to krpm
    encVel = ((gPreviousEncPosition[mtrNum][2] - gPreviousEncPosition[mtrNum][1])
                 / (TIMER0_PERIOD_S)) * (60.0 / 1000.0);

    // Add encoder velocity to queue
    gEncVelRolling[mtrNum] = gEncVelRolling[mtrNum] + (1.0 / WINDOW_SIZE)
            * (encVel - gEncVelQueue[mtrNum][gEncVelQueueIndex[mtrNum]]);
    gEncVelQueue[mtrNum][gEncVelQueueIndex[mtrNum]] = encVel;
    gEncVelQueueIndex[mtrNum] = (gEncVelQueueIndex[mtrNum] + 1) % WINDOW_SIZE;

    // get the encoder acceleration in rad/s
    encAcc = (gPreviousEncPosition[mtrNum][0] - 2 * gPreviousEncPosition[mtrNum][1]
         + gPreviousEncPosition[mtrNum][2]) / (2 * PI * TIMER0_PERIOD_S * TIMER0_PERIOD_S);

    // Add encoder acceleration to queue
    gEncAccRolling[mtrNum] = gEncAccRolling[mtrNum] + (1.0 / WINDOW_SIZE)
            * (encAcc - gEncAccQueue[mtrNum][gEncAccQueueIndex[mtrNum]]);
    gEncAccQueue[mtrNum][gEncAccQueueIndex[mtrNum]] = encAcc;
    gEncAccQueueIndex[mtrNum] = (gEncAccQueueIndex[mtrNum] + 1) % WINDOW_SIZE;

    if (mtrNum == HAL_MTR1)
    {
        CAN_setEnc1(_IQ(gEncPosRolling[mtrNum]), _IQ(gEncVelRolling[mtrNum]), _IQ(gEncAccRolling[mtrNum]));
    }
    else
    {
        CAN_setEnc2(_IQ(gEncPosRolling[mtrNum]), _IQ(gEncVelRolling[mtrNum]), _IQ(gEncAccRolling[mtrNum]));
    }

}

interrupt void can1_ISR() {

    // acknowledge interrupt from PIE
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);

}

void maybeSendCanStatusMsg() {
    if (gCanLastStatusMsgTime <
        (gTimer0_stamp - TIMER0_FREQ_Hz / CAN_STATUSMSG_TRANS_FREQ_Hz)) {
        // If there is still an old message waiting for transmission, abort it
        if (CAN_checkTransmissionPending(CAN_MBOX_OUT_STATUSMSG)) {
            CAN_abort(CAN_MBOX_OUT_STATUSMSG);
        }

        CAN_setStatusMsg();
        CAN_send(CAN_MBOX_OUT_STATUSMSG);

        gCanLastStatusMsgTime = gTimer0_stamp;
    }
}
