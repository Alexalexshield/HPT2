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

#ifndef __UHF_PKT
#define __UHF_PKT

/**
 * \file uhf_pkt.h
 * \brief Header file for high frequency communicatons protocol.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
 
 * BYTE   bat_error   	:1;		->	BYTE	trap_8khz	:1:
 * #define BIR_CMD_BATT_ERR	(4) -> #define BIR_CMD_TRAP_8KHZ	(4)

 */

#include <htc.h>
#include "types.h"

// Packet sizes
#define  UHF_PREAMBLE_SIZE                   10    	// The number of preamble bytes in a UHF packet
#define  UHF_UART_RESYNC_SIZE                2     	// The number of UART resynch bytes in a UHF packet
#define  UHF_TRAILER_SIZE                    1     	// The number of trailer bytes in a UHF packet

// Packet attributes
#define  UHF_PREAMBLE_CHAR                   0x55  	// The preamble character used
#define  UHF_UART_RESYNC_CHAR                0xFF  	// The resynch character used
#define  UHF_STX_CHAR                        0x5A  	// The STX field character used
#define  UHF_TRAILER_CHAR                    0x5C  	// The trailer character used


//#if( DEBUG_SU_UHF_RX )
//
//#define  UHF_PREMABLE_RX_THRESHOLD           4     	// The number of valid preamble characters required
//#define  UHF_UART_STX_THRESHOLD            	 9    	// The number of characters to wait for an STX

//#else

#define  UHF_PREMABLE_RX_THRESHOLD           4     	// The number of valid preamble characters required
#define  UHF_UART_STX_THRESHOLD            	 8    	// The number of characters to wait for an STX

//#endif

#define  UHF_TIMESLOT_PERIOD_US              5500  	// The length of a UHF timeslot

#define  UHF_CRC_POLYNOMIAL                  0xA001 // The CRC polynimial
#define  UHF_CRC_INIT                        0xFFFF // The initial CRC value

#define  UHF_DL_DATA_BOCK_SIZE               0x20   // Size of download block in words
#define  UHF_CONTROL_SEARCH_REPLIES          7      // Number of replies to give to the UHF control search packets


#define CMD_MS								0x80
#define CMD_ACK								0x01
#define CMD_GAS_ALARM	  					0x02
#define CMD_BAT_LOW	  						0x04



// The structure used to receive ILB to MU BI packets
typedef struct {
	BYTE	mark;
   	BYTE   	no_slt  	:3;	//NO_SLT contains the number of slots that the ILB will use in its Broadcast Invitation (BI) message
   	BYTE   	type       	:4;	//The ILB uses this to determine what type of message is being sent, i.e. Beacon Configuration.
   	BYTE   	ms    		:1;	//The M/S bit is used in conjunction to identify if the message is from the master (HEC, 1), or from the slave
//  short_u bid;
   	bid_u 	bid;
   	BYTE   	cur_grp		:7;
   	BYTE   	reserv		:1;
   	BYTE   	no_grp_u3   :3;
	BYTE   	ext         :3;   	// ��������� �������� "���������", "�����", "test"
	BYTE   	ext_reserv  :2;
   	short_u sum_u;
} UHF_BI_s;


#if( USE_UHF_TO_VLF )
// The structure used to receive ILB to MU Pseudo VLF packets
typedef struct {
  	BYTE   	mark;

   	BYTE   	vlf_cmd	:3;	 	// VLF command
	BYTE   	type    :4;		// The ILB uses this to determine what type of message is being sent, i.e. VLF Command.
   	BYTE   	ms   	:1;		// The M/S bit is used in conjunction to identify if the message is from the master (HEC, 1), or from the slave

	BYTE	sid;			// Search Unit ID
	BYTE	seq_dur;		// Gives a unique sequence for the detect operation / TX Duration
	mid24_s mid_s;			// Contains the tag's 24-bit ID
   	short_u sum_u;
} UHF_VLF_s;
#endif

// The structure used to receive ILB to MU BIRC packets
typedef struct {
   	BYTE 	mark;
   	BYTE    ext        	:3;
   	BYTE   	type       	:4;
   	BYTE   	ms    		:1;
   	long_u 	mid;
   	short_u	sum_u;
} UHF_BIRC_s;


// The structure used to receive ILB to MU BIRC_E packets
typedef struct {
   	BYTE   	mark;
   	BYTE    ext     	  :3;
   	BYTE   	type       :4;
   	BYTE   	ms    :1;
   	long_u 	mid;
	BYTE	ext_data;
   short_u sum_u;
} UHF_BIRC_E_s;


// The structure used to receive ILB to MU II packets
typedef struct {
	BYTE		mark;
   	BYTE        ext   	:3;
   	BYTE   		type    :4;
   	BYTE   		ms  	:1;
   	bid_u 		bid;
   	long_u  	mid;
   	short_u 	sum_u;
} UHF_II_s;



typedef struct {
   	BYTE   ack        	:1;
   	BYTE   ack_manual   :1;
   	//BYTE   bat_error   	:1;
	BYTE	trap_8khz	:1;
   	BYTE   type      	:4;
   	BYTE   ms   		:1;
} bir_cmd_s;

#define BIR_CMD_ACK			(1)
#define BIR_GAS_ALARM		(2)
#define BIR_CMD_ACK_MANUAL 	(BIR_GAS_ALARM)
//#define BIR_CMD_BATT_ERR	(4)
#define BIR_CMD_TRAP_8KHZ	(4)

// The structure used to send MU to ILB BIR packets
typedef struct {
 	BYTE   mark;

	union{
	bir_cmd_s	c;
	BYTE		b;
	}cmd;

   long_u  mid;
   short_u  sum_u;
} UHF_BIR_s;


// The structure used to send Controller to MU control search packets
typedef struct {
	BYTE	mark;
   	BYTE   	gmsk_u3       :3;
   	BYTE   	type       :4;
   	BYTE   	ms    :1;
   	BYTE   	type_ctl;
   	short_u sidx;
   	BYTE   	grp_u8;
   	short_u sum_u;
} UhfCtrlSearchCTSt;


// The structure used to send Controller to MU control query packets
typedef struct {
	BYTE   	mark;
   	BYTE            	:3;
   	BYTE   	type     :4;
   	BYTE   	ms  :1;
   	BYTE   	type_ctl;
   	long_u 	mid;
   	short_u	sum_u;
} UhfCtrlQueryCTSt;


// (11) Reply
// The structure used by MU to reply to Controller search and query packets
typedef struct {
   BYTE   mark;
   BYTE         :3;
   BYTE   type  :4;
   BYTE   ms    :1;
   BYTE   type_ctl;
   long_u mid;
   BYTE   state;
   BYTE   wdog_resets;
   BYTE   vers_boot;
   BYTE   vers;
   short_u sum_u;
} UhfCtrlSearchQueryRTSt;


// The structure used to send Controller to MU control boot mode packets
typedef struct {
   BYTE   	mark;
   BYTE           :3;
   BYTE   	type  :4;
   BYTE   	ms    :1;
   BYTE   	type_ctl;
   long_u 	mid;
   short_u	sum_u;
} UhfCtrlBootModeCTSt;


// The structure used to send Controller to MU control download init packets
typedef struct {
   BYTE   	mark;
   BYTE     	  :3;
   BYTE   	type  :4;
   BYTE   	ms    :1;
   BYTE   	type_ctl;
   long_u	mid;
   short_u 	sum_u;
} UhfCtrlDLInitTSt;


// The structure used to send Controller to MU control download data packets
typedef struct {
   BYTE   	mark;
   BYTE           :3;
   BYTE   	type  :4;
   BYTE   	ms    :1;

   BYTE   	type_ctl;
   BYTE   	len;
   ushort  	seq;
   ushort  	address;
   ushort  	data[UHF_DL_DATA_BOCK_SIZE];
   short_u 	sum_u;
} UhfCtrlDLDataTSt;


// The structure used to send Controller to MU control download end packets
typedef struct {
   BYTE   	mark;
   BYTE           :3;
   BYTE   	type  :4;
   BYTE   	ms    :1;
   BYTE   	type_ctl;
   long_u 	mid;
   ushort  	seq;
   short_u 	sum_u;
} UhfCtrlDLEndCTSt;



// The structure used to send Controller to MU control Test Mode packet
typedef struct {
   BYTE   	mark;
   BYTE           :3;
   BYTE   	type  :4;
   BYTE   	ms    :1;
   BYTE   	type_ctl;
   long_u 	mid;
   ushort  	test_mode;
   short_u 	sum_u;
} UHF_CRTL_TEST_s;





// (6) Reply
// The structure used by MU to reply to Controller MU download end query packets
typedef struct {
   BYTE   mark;
   BYTE             :3;
   BYTE   type      :4;
   BYTE   ms    	:1;
   BYTE   type_ctl;
   BYTE   status;
   short_u sum_u;
} UhfCtrlDLEndRTSt;


// The structure used to send Controller to MU set ID control packets
typedef struct {
   BYTE   	mark;
   BYTE                	:3;
   BYTE   	type       	:4;
   BYTE   	ms   		:1;
   BYTE   	type_ctl;
   long_u 	curr_mid;
   ushort  	data_au16[4];
   short_u 	sum_u;
} UhfCtrlSetIdCTSt;


// (6) Reply
// The structure used by MU to reply to Controller Set ID
typedef struct {
   BYTE   mark;
   BYTE             :3;
   BYTE   type      :4;
   BYTE   ms    	:1;
   BYTE   type_ctl;
   BYTE   success;
   short_u  sum_u;
} UhfCtrlSetIdRTSt;


// A union of all the packets possible
typedef union {
   	UHF_BI_s                	bi;
   	UHF_BIRC_s              	birc;
   	UHF_BIRC_E_s            	birc_e;
   	UHF_II_s                	ii;
   	UHF_BIR_s               	bir;

   	UhfCtrlSearchCTSt       	csc;
   	UhfCtrlQueryCTSt        	cqc;
   	UhfCtrlSearchQueryRTSt  	csqr;
   	UhfCtrlBootModeCTSt     	cbmc;
   	UhfCtrlDLInitTSt        	cdi;
   	UhfCtrlDLDataTSt        	cdd;
   	UhfCtrlDLEndCTSt        	cdec;
   	UhfCtrlDLEndRTSt        	cder;
   	UhfCtrlSetIdCTSt        	csic;
   	UhfCtrlSetIdRTSt        	csir;
	UHF_CRTL_TEST_s				ctst;

#if( USE_UHF_TO_VLF )
   	UHF_VLF_s        			vlf;
#endif

   BYTE                    	data[sizeof(UhfCtrlDLDataTSt)];         // Longest packet


} UhfPktTU;

// The various UHF packet types
enum UHF_PKT_TYPES {
   	UHF_BI_PKT = 0,
   	UHF_BIRC_PKT,
   	UHF_II_PKT,
#if( USE_UHF_TO_VLF )
   	UHF_RESERV,
	UHF_VLF_PKT,
#endif
   	UHF_CONTROL_PKT = 0x0F
};

enum UHF_CONTROL_TYPES {
   	UHFC_SEARCH = 0,
  	UHFC_QUERY,
   	UHFC_BOOTMODE,
	UHFC_3,
	UHFC_4,
	UHFC_TESTMODE	//��������! ��������� ����� ����������
};

// Monitor State
// This is the state of operation the tag enters after power-up, reset, and after an ILB communication timeout.
// The tag monitors its RF channel for messages from any ILB.  Specifically, the tag is waiting to receive
// a BI (Broadcast Invitation) from any ILB.  Upon receiving a BI, the tag will set a flag indicating that it
// has received a BI from a particular ILB (note that a tag will store the ILB�s ID so that it can identify
// which ILB it is communicating with).  If the tag receives a second BI from the same ILB before receiving
// a BI from any other ILB, the tag is said to be bonded to the ILB and will enter the Discovery State.

// Discovery State
// This is the state of operation that the tag enters after receiving two consecutive BIs from the same ILB.
// The purpose of having the tag wait for two consecutive BIs from the same ILB is to help prevent the tag
// from �bouncing� back and forth between multiple ILBs in the case where ILBs have overlapping coverage areas.
// The tag will issue a BIR (Broadcast Invitation Reply) in a randomly selected timeslot within the required
// group (if there is more than one group).  The tag will now wait for an acknowledgement from the ILB in the
// form of a BIRC (Broadcast Invitation Reply Confirmation).  If a BIRC is not received, the tag will remain
// in the Discovery State and respond to the next BI (if it comes from the same ILB).

// Poll State
// If a BIRC is received from an ILB, the tag will enter the Poll State.  Once in this state, the tag
// will only listen but not respond to a specific ILB during timeout period. The tag exits Poll State once
// timeout period expired or after receiving a BI from a new ILB.  If the tag receives a second consecutive
// BI from the new ILB before receiving a BI from any other ILB, the tag will enter the Discovery State.

/*
enum UHF_STATES {
   UHF_MONITOR,         // The tag is waiting to receive two consecutive BIs (Broadcast Invitations) from any ILB
   UHF_DISCOVERY,       // The tag responds to an ILB with its ID
   UHF_POLL             // The tag will only listen but not respond to a specific ILB during timeout period.
};
*/


//---------------------------------------------------------------------------
typedef struct Uhf_pkt_s {

	BYTE		rxt_state;
	BYTE		idx;
	BYTE		len;
	BYTE 		brx_pkt_cnt;
	UhfPktTU 	pkt;

} Uhf_pkt_s;




#endif
