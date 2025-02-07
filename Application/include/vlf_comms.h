/**************************************************************************************************

  Copyright (c) 2004 Embedded IQ cc. All rights reserved.

  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR EMBEDDED IQ BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL,  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**************************************************************************************************/

/**
 * \file vlf_comms.h
 * \brief Header file for VLF communicatons functionality.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005

	06.01.2011 Igors Zalts
 */

#ifndef VLF_COMMS_H__
#define VLF_COMMS_H__


#include <htc.h>
#include "types.h"
#include "time.h"
#include "vlf_pkt.h"

// Externals

extern const BYTE su2mu_preamble[VLF_SU_TO_MU_PREAMBLE_SIZE];
extern const BYTE mu2su_preamble[VLF_MU_TO_SU_PREAMBLE_SIZE];

extern bank1 struct Pkt_tx_buf_st  vtx;
extern bank1 Pkt_rx_buf_st vrx;
extern volatile BYTE vlf_state;


// Hardware constants

// Note Timer 2 is used as the LF transmit PWM period and receive subperiod clock.
#define  TMR2_PRESC_RX  	1              // The TMR2 prescaler in RX mode

#define  TMR2_PRESC_TX      0              // The TMR2 prescaler in TX mode
#define  TMR2_POSC          4              // The TMR2 postscaler
#define  PR2_RX             ((SYS_FREQ/(1<<(2*TMR2_PRESC_RX))/(TMR2_POSC+1)/(VLF_RX_SYMBOL_FREQUENCY)/(VLF_NUM_RX_SYMBOL_SUBPERIODS)) - 1) // The PR2 value in RX mode
#define  PR2_TX             ((SYS_FREQ/(1<<(2*TMR2_PRESC_TX))/(VLF_TX_FREQUENCY)) - 1) 	// The PR2 value in TX mode
#define  TX_FULL_POWER      (SYS_FREQ/(1<<(2*TMR2_PRESC_TX))*4/(VLF_TX_FREQUENCY)/2) 	// The full TX power duty cycle

// Note Timer 0 is used as a RX signal crossing counter and a TX symbol rate timer
#define  TMR0_PRESC                          5              // The TMR0 prescaler
#define  TMR0_START                          ((BYTE)((ulong)257 - (ulong)SYS_FREQ/(ulong)VLF_TX_SYMBOL_FREQUENCY/((ulong)1<<((ulong)TMR0_PRESC+(ulong)1)))) // Add 1 to make sure we're less than the required amount

/*
#define  VLF_RAND_MULTIPLIER                 106
#define  VLF_RAND_DIVIDER                    255
*/

// An enumeration describing the states on the VLF interface
enum VLF_STATE {
	VLF_STATE_IDLE,
	VLF_STATE_READBIT,
	VLF_STATE_PACKET_TRANSMITTED
};

// An enumeration describing the RX states on the VLF interface
enum VLF_RX_STATES {
	VLF_RX_APS,					//seeking an APS preambule from ExZMTU/combine
   	VLF_RX_SEEKING,            	// Seeking an opening flag sequence in the incoming stream
   	VLF_RX_LOCKED,             	// Locked to an incoming packet
   	VLF_RX_RECEIVING,          	// Receiving an incoming packet
   	VLF_PACKET_READY        	// A packet is ready
   };

// An enumeration describing the TX states on the VLF interface
enum TX_STATE {
   	VLF_TX_IDLE,               // The TX is in an IDLE state
   	VLF_TX_LOCATE,             // The TX is currently transmitting a location signal
   	VLF_TX_PACKET_PREAMBLE,    // The TX is currently transmitting the packet preamble
   	VLF_TX_PACKET_TYPE,        // The TX is currently transmitting the packet type
   	VLF_TX_PACKET_DATA,        // The TX is currently transmitting the packet data
   	VLF_TX_PACKET_DONE         // The TX has completed transmission
   };



/*
// An enumeration describing the location modes on the VLF interface
enum VLF_LOC_MODES {
   	VLF_LOC_NONE = 0,          // Not in location mode
   	VLF_LOC_ONESHOT,           // Transmits a location signal of specified duration once
   	VLF_LOC_CONTINUOUS         // Transmits a number location signals, terminated by the search unit
};

// An enumeration describing the location modes on the VLF interface
enum VLF_LOC_CONT_STATE {
   	VLF_LOC_STATE_RESET = 0,   // Doing nothing
   	VLF_LOC_STATE_INIT,        // Initial signature search
   	VLF_LOC_STATE_MARK,        // Transmitting location signal
   	VLF_LOC_STATE_SPACE        // Waiting for location signature
};
*/


//extern bank1 BYTE  detect_seq;
//extern bank1 ushort su_id;
extern bank1 BYTE sid;


BYTE VLF_Busy(void);
void VLF_SendGeneric(void);

extern void VLF_StartReceiver(void);
extern void VLF_StopReceiver(void);
extern char VLF_ProcessBitRead(void);
extern void VLF_ReplyToBroadcastRequest( BYTE intervals );
//extern void VLF_ReplyToBroadcastRequest(void);
extern BYTE VLF_ProcessPacket(void);
extern void VLF_ReplyLocateRequest( uchar locate_seconds );

#define VLF_START_SCORE 	0xFFFF


#define SEARCH_ID_INTERVALS 	10
#define SEARCH_QUICK_INTERVALS 	5

// These defines are used for antenna delays since we send the command on X,Y and then Z from the SU to the MU
// So the delays correspond to the time it takes to send the variable-sized command
// CAUTION: These timeouts are assumed constant in the SU so any changes should be made to both sides and this
//			side should always be about 10ms more than the other

#define QUICKSEARCH_RX_DELAY_MS ((3*85)+20)
#define DETECT_RX_DELAY_MS 		((5*85)+20)

#define LOCATE_RX_DELAY_MS  	((7*85)+20)
#define MASK_RX_DELAY_MS 		((8*85)+20)


#endif
