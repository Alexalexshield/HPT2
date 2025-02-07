/**************************************************************************************************
  Copyright (c) 2008 Embedded IQ cc. All rights reserved.

  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR EMBEDDED IQ BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL,  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/

#ifndef __VLF_PKT_H
#define __VLF_PKT_H

#include <htc.h>
#include "types.h"

// Hardware constants

// Frequencies and periods
#define  VLF_TX_FREQUENCY                    (35714.2857)		// The VLF TX frequency
#define  VLF_TX_SYMBOL_FREQUENCY             (  190.7349)      	// The VLF TX symbol frequency


#define  VLF_RX_FREQUENCY                    (8000)           	// The VLF RX frequency
#define  VLF_RX_SYMBOL_FREQUENCY             (100)            	// The VLF RX symbol frequency
#define  VLF_NUM_RX_SYMBOL_SUBPERIODS        (4)              	// The number of periods in a RX bit
#define  VLF_RX_HIGH_THRESHOLD               (VLF_RX_FREQUENCY/VLF_RX_SYMBOL_FREQUENCY/VLF_NUM_RX_SYMBOL_SUBPERIODS/2 - 1) // The threshold between a high and a low bit
#define  VLF_LOC_ONESHOT_TX_PERIOD_MS        ((ulong)1000)    	// The period for which to transmit a location signal in one shot mode
#define  VLF_LOC_CONT_TX_PERIOD_MS           ((ulong)300)		// The period for which to transmit a location signal in continuous mode
#define  VLF_LOC_CONT_WAIT_PERIOD_MS         ((ulong)101)       // The period for which to wait for a location signature in continuous mode (+1 to correct for TX symbol timer granularity)
#define  VLF_LOC_CONT_INIT_TX_HOLDOFF_MS     ((ulong)10)
#define  VLF_LOC_CONT_INIT_WAIT_PERIOD_MS    ((ulong)1000)
#define  VLF_LOC_CONT_MISSED_SIGS            ((ulong)6)         // The number of missed signatures before cancelling continuous location mode


// Packet attributes
#define  SU_TO_MU_PREAMBLE                   0x5A5             	// The packet preamble for SU to MU messages
#define  VLF_SU_TO_MU_PREAMBLE_SIZE          12                 // The size in symbols of the packet preamble
#define  SU_TO_MU_PREAMBLE_CRC               0xF5

#define  MU_TO_SU_PREAMBLE                   0xA5             	// The preamble value for MU to SU messages
#define  VLF_MU_TO_SU_PREAMBLE_SIZE          8                  // The size in bits of the packet preamble
#define  MU_TO_SU_PREAMBLE_CRC               0x28

#define  APS_PREAMBLE                        0x2C0             	// The preamble for APS messages
#define  APS_PREAMBLE_SIZE                   10                 // The size in bits of the packet preamble
#define  APS_SUBPERIOD_THRESH				 22
#define  APS_MARK_THRESH					 70
#define  APS_TAIL_THRESH1					 70
#define  APS_TAIL_THRESH0					 2*APS_SUBPERIOD_THRESH
#define  APS_SPACE_THRESH					 25
#define  APS_TIMEOUT						 (120*(TICKS_PER_SEC))
#define  APS_PREAMBLE_SUBP_SIZE             (APS_PREAMBLE_SIZE *VLF_NUM_RX_SYMBOL_SUBPERIODS)  	// The PREAMBLE size in QUARTER BITS

#define  LOC_SIGNATURE                       0x6B               // The location signature value
#define  LOC_SIGNTAURE_SIZE                  7                  // The size in symbols of the location signature
#define  EVAC_SIGNATURE                      0xBEAF             // The evacuation signature

// Packet sizes
#define  PCKT_PREAMBLE_SUBP_SIZE             (VLF_SU_TO_MU_PREAMBLE_SIZE*VLF_NUM_RX_SYMBOL_SUBPERIODS)  	// The PREAMBLE size in QUARTER BITS
#define  PCKT_LOC_SIG_SIZE                   7                  // The size in symbols of the location signature
#define  PCKT_TYPE_SIZE                      4                  // The size in symbols of the packet type


// Convolutional encoding and Viterbi decoding
#define  VITERBI_N                           2                  // The rate of the convolutional code used
#define  VITERBI_K                           5                  // The constraint length of the code
#define  SCORE_METRIC_SHIFT                  3                  // The viterbi metric score shift value
#define  SCORE_METRIC_FS                     ((VLF_RX_FREQUENCY/VLF_RX_SYMBOL_FREQUENCY) >> SCORE_METRIC_SHIFT) // A full scale metric
#define  SCORE_METRIC_LOW_THRESH             2//(SCORE_METRIC_FS*4/5)		// The maximum score metric for a low bit
#define  SCORE_METRIC_HIGH_THRESH            3//(SCORE_METRIC_FS*2/5)  	// The minimum score metric for a high bit
#define  SCORE_METRIC_MID_THRESH             (SCORE_METRIC_FS*3/5)  	// Somewhere in between
#define  CODING_TAIL_SIZE                    ((VITERBI_K-1)*VITERBI_N) 	// The length of the coding tail


//#define MU_ID_3BYTE		1

#define MU_PROTOCOL2_RESPONSE_PAYLOADSIZE 4


enum VLF_COMMS_PACKET_TYPES {
   VLF_MU_DETECT = 0,
   VLF_MU_MASK,             // 01
   VLF_MU_QUICK_SEARCH_Y,   // 02
   VLF_MU_QUICK_SEARCH_Z,   // 03
   VLF_MU_LOCATE,           // 04
   VLF_MU_VLF_RX_CONTROL,   // 05 ����������� ����������/������������� ������ VLF �������( ������������ �� ���������� 433MHz )
   VLF_MU_QUICK_SEARCH_X,	// 06 Formerly VLF_MU_EVACUATE,
   VLF_MU_TEST,             // 07 ��� �������� ����� � ��������� ��������. ����������� �������� ��� �������������� ��������.
   VLF_MU_MASK_FULL,        // 08 VLF Only ������ ������������ TAG-�� ��������� ������.
//   VLF_MU_FLEXALERT,      	// 09 "VLF OnlyEvacuate" message from FlexAlert
//   VLF_MU_CALL_1,      		// 10 "VLF OnlyCall 1" 	 message from FlexAlert
//   VLF_MU_CALL_2,      		// 11 "VLF OnlyCall 2" 	 message from FlexAlert
   VLF_NO_PACKET
};


//---------------------------------------------------------------------------
typedef struct {            // Used to initiate a search for tags.
	BYTE sid;          		// Contains a unique SU ID
   	BYTE seq;            	// Gives a unique sequence for the detect operation
} DetectCommand_P2;


//---------------------------------------------------------------------------
typedef struct {            // Used to mask the tag from responding to the current detect request
	mid24_s mid_s;			// Contains the tag's 24-bit ID
   	BYTE sid;          		// Unique SU ID
   	BYTE seq;           	// Detect sequence to mask
} MaskCommand_P2;

//---------------------------------------------------------------------------
typedef struct {            // Used to mask the tag from responding to the current detect request
	mid24_s mid_s;			// Contains the tag's 24-bit ID
   	BYTE timer;           	// Mask timeout
} MaskFullCommand_P3;

//---------------------------------------------------------------------------
typedef struct {            // Instruct the tag to transmit a continuous carrier signal for location purposes
	mid24_s mid_s;	  		// Contains the tag's 24-bit ID
   	BYTE  duration;         // Seconds to transmit LOC signal for. A value of zero indicates a transmission
                            // that is prolonged by the SU periodically transmitting a signature as long as it
                            // wants the MU to carry on transmitting
} LocateCommand_P2;


/*
//---------------------------------------------------------------------------
typedef struct {        	// Used to initiate a QuickSearch for tags.
   	BYTE 	sid;     		// Contains a unique SU ID
   	BYTE  	seq;       		// Gives a unique sequence for the detect operation
} QuickSearchCommand_P2;
*/


//---------------------------------------------------------------------------
typedef struct {         	// This is the generic response to all unicast commands
	mid24_s mid_s;	 	// Contains the tag's 24-bit ID

	union {
		struct {
				BYTE mobility:4;
				BYTE bit5:1;
				BYTE bit6:1;
				BYTE batteryproblem:1;
				BYTE masked:1;
		} s ;
		BYTE status;
	} m ;
} GenericResponse_P2;


//---------------------------------------------------------------------------
typedef struct Pkt_tx_buf_st {
   	volatile BYTE          	state;
   	BYTE 	cnt;
	BYTE	idx;
	BYTE	shifter;
	BYTE	seq;
	BYTE	sid; 			// Search Unit ID
	BYTE  	type;
   union {
      BYTE                  data[8];
      DetectCommand_P2	    det_cmd;
      LocateCommand_P2		loc_cmd;
      MaskCommand_P2		msk_cmd;
      MaskFullCommand_P3 	msk_full_cmd;
  	  GenericResponse_P2   	responce;
   	} u;
} Pkt_tx_buf_st;

//---------------------------------------------------------------------------
typedef struct Pkt_rx_buf_st {
   	volatile BYTE state;
   	BYTE  cnt;
	BYTE  len;
	BYTE  *ptr;
	BYTE  shifter;
	ushort preamble_score;
	ushort prev_preamble_score;
	BYTE  score;
	BYTE  unlocked;

	BYTE  preamble[VLF_SU_TO_MU_PREAMBLE_SIZE*VLF_NUM_RX_SYMBOL_SUBPERIODS];
#if( USE_APS )
	union flags {
		struct aps{
		BYTE  tail		:1;
		BYTE  enable	:1;
		BYTE  bit_score	:4;
		BYTE  unlock	:1;
		BYTE  combiner	:1;
		} aps;
		BYTE aps_state;
	} flags;
	BYTE   aps_lock_cnt;
	BYTE aps_score;
	ushort aps_timeout;
#endif
} Pkt_rx_buf_st;


extern const BYTE uncoded_rx_data_size[];
#define UNCODED_RX_DATA_SIZE( type )      ( (BYTE)(uncoded_rx_data_size[type]    ) )
#define UNCODED_RX_PAYLOAD_SIZE( type )   ( (BYTE)(uncoded_rx_data_size[type] + 1) )


#endif
