// BSD 3-Clause License
//
// Copyright (c) 2019, Max Planck Gesellschaft, New York University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * \brief API for more convenient use of the eCAN module
 *
 * The goal of this API is to provide a simple, clear interface to control the
 * motors via the eCAN module that wraps all the ugly register stuff. I.e. it
 * should be understandable by everyone, even if they did not read the eCAN
 * documentation.
 *
 * \author Felix Widmaier <fwidmaier@tue.mpg.de>
 */

#ifndef SRC_CANAPI_H_
#define SRC_CANAPI_H_

// **************************************************************************
// the includes
#include "sw/drivers/can/src/32b/f28x/f2806x/can.h"
#include "hal_2mtr.h"


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \name Mailbox Bitmasks
//! \brief Bit masks specifying the mailboxes used for the various types of
//! messages.
//!
//! We have enough mailboxes available so that we can use a separate one for
//! every message type.
//! `CAN_MBOX_IN|OUT_XY = 1 << n` means that mailbox n is used for message type
//! XY.
//!
//! NOTE: Changing the mailbox number of a message also requires some
//! adjustments in the code below!
//! \{
#define CAN_MBOX_OUT_PIVOTENC   (uint32_t) 1 << 9
#define CAN_MBOX_OUT_PIVOTENCVEL (uint32_t) 1 << 8
#define CAN_MBOX_OUT_PIVOTENCACC (uint32_t) 1 << 7

#define CAN_MBOX_OUT_STATUSMSG (uint32_t) 1 << 15

#define CAN_MBOX_ALL CAN_MBOX_OUT_PIVOTENC \
    | CAN_MBOX_OUT_PIVOTENCVEL \
    | CAN_MBOX_OUT_PIVOTENCACC \
    | CAN_MBOX_OUT_STATUSMSG
//! \}

//#define CAN_MBOX_ALL CAN_MBOX_OUT_ENC_POS


//! \name Arbitration IDs
//! \brief Arbitration IDs of the different message types
//! \{
#define CAN_ID_PIVOTENC     0x31
#define CAN_ID_PIVOTENCVEL  0x41
#define CAN_ID_PIVOTENCACC  0x71
#define CAN_ID_STATUSMSG 0x11
//! \}


// **************************************************************************
// the typedefs

//! \brief Status message bits.

struct CAN_STATUSMSG_BITS
{                              // bits
   uint16_t system_enabled:1;  // 0
   uint16_t motor1_enabled:1;  // 1
   uint16_t motor1_ready:1;    // 2
   uint16_t motor2_enabled:1;  // 3
   uint16_t motor2_ready:1;    // 4
   //! \see \ref ErrorCodes
   uint16_t error_code:3;      // 5-7
   uint16_t rsvd:8;            // 8-15
};


//! \brief Status message that allows integer or bit access.

typedef union _CAN_StatusMsg_t_
{
   uint16_t              all;
   struct CAN_STATUSMSG_BITS  bit;
} CAN_StatusMsg_t;



//! \brief Command message.
/*
typedef struct _CAN_Command_t_
{
	//! \brief Command ID
	uint32_t id;
	//! \brief Value of the command
	uint32_t value;
} CAN_Command_t;
*/

// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief Initialize the eCAN-A GPIOs
extern void CAN_initECanaGpio(HAL_Handle halHandle);


//! \brief Initialize eCAN-A module
extern void CAN_initECana();


//! \brief Setup mailboxes
extern void CAN_setupMboxes();


//! \brief Write status message to the corresponding transmission mailbox.

inline void CAN_setStatusMsg()
{

    CAN_StatusMsg_t status;

    status.all = 0;

	ECanaMboxes.MBOX15.MDL.byte.BYTE0 = status.all;

	return;
}


inline void CAN_setEnc1(_iq encPos, _iq encVel, _iq encAcc)
{
    ECanaMboxes.MBOX9.MDL.all = encPos;
    ECanaMboxes.MBOX8.MDL.all = encVel;
    ECanaMboxes.MBOX7.MDL.all = encAcc;
}


inline void CAN_setEnc2(_iq encPos, _iq encVel, _iq encAcc)
{
    ECanaMboxes.MBOX9.MDH.all = encPos;
    ECanaMboxes.MBOX8.MDH.all = encVel;
    ECanaMboxes.MBOX7.MDH.all = encAcc;
}


//! \brief Send data of the specified mailboxes
//!
//! To specify the mailboxes, use the CAN_MBOX_OUT_xy defines for this. Example:
//! to send current, position and velocity of motor 1 call
//! `CAN_send(CAN_MBOX_OUT_IqPos_mtr1 | CAN_MBOX_OUT_SPEED_mtr1);`
//!
//! \param mailboxes  A bitmap specifying the mailboxes to be send.
inline void CAN_send(uint32_t mailboxes)
{
	// Always access whole register, not bitfield.  Therefore no shadow register
	// should be necessary.

	// Set TRS for all specified mailboxes
	ECanaRegs.CANTRS.all |= mailboxes;
	// Wait for all TAn bits of specified mailboxes to be set
//	while ((ECanaRegs.CANTA.all & mailboxes) != mailboxes);
//	// Clear all TAn (we have to set it to 1 so it becomes 0...)
//	ECanaRegs.CANTA.all = mailboxes;
//	// wait for all TAn bits to become zero
//	while ((ECanaRegs.CANTA.all & mailboxes) != 0);

	return;
}


//! \brief Abort pending transmissions of the specified mailboxes.
//!
//! \param mailboxes  A bitmap specifying the mailboxes to be send.
inline void CAN_abort(uint32_t mailboxes)
{
	// request transmission reset
	ECanaRegs.CANTRR.all |= mailboxes;
	// wait until TRS is cleared
	//while ((ECanaRegs.CANTRS.all & mailboxes) != 0);

	return;
}


//! \brief Check if received message pending flag is set.
//!
//! Check if the "received message pending" flag is set for the specified
//! mailbox.  To clear the flag, use CAN_clearReceivedMessagePending().
//!
//! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
//! 	than one mailbox in the mask, this function will return true if a
//! 	message is pending in at least one of the mailboxes.
//! \returns True if a new message is pending in the specified mailbox.
inline bool CAN_checkReceivedMessagePending(uint32_t mailbox_mask)
{
	return ECanaRegs.CANRMP.all & mailbox_mask;
}


//! \brief Clear received message pending flag.
//!
//! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
//! 	than one mailbox in the mask, the flag is reset for each of them.
inline void CAN_clearReceivedMessagePending(uint32_t mailbox_mask)
{
	// reset bit (have to write a 1 to get a 0)
	ECanaRegs.CANRMP.all = mailbox_mask;
}


//! \brief Check if new message arrived and acknowledge if yes.
//!
//! Check if the "received message pending" flag for the specified mailbox.  If
//! yes, the message is acknowledged (i.e. the flag is reset).
//!
//! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
//! 	than one mailbox in the mask, this function will return true if a
//! 	message is pending in at least one of the mailboxes.
//! \returns True if a new message is pending in the specified mailbox.
inline bool CAN_checkAndClearRMP(uint32_t mailbox_mask)
{
	if (CAN_checkReceivedMessagePending(mailbox_mask))
	{
		CAN_clearReceivedMessagePending(mailbox_mask);
		return true;
	}
	else
	{
		return false;
	}
}


//! \brief Check if a message is waiting for transmission
//!
//! Check if a message is waiting for transmission (i.e. the TRS bit is set) in
//! the specified mailbox.  If more then one mailbox is specified, it is checked
//! if at least one of them has a pending message.
//!
//! \param mailbox_mask Bitmask that specifies the mailbox(es).  See the
//! 	CAN_MBOX_OUT_* defines.
//! \returns True if a message is waiting for transmission in one of the
//! 	specified mailboxes.
inline bool CAN_checkTransmissionPending(uint32_t mailbox_mask)
{
	return ECanaRegs.CANTRS.all & mailbox_mask;
}


#ifdef __cplusplus
}
#endif // extern "C"


#endif /* SRC_CANAPI_H_ */
