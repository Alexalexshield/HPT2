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
 * \file vlf_comms.c
 * \brief Source file for MU VLF comms functionality, etc.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005

	09.01.2011 Igors Zalts
 */

#include <htc.h>
#include "stdlib.h"
//#include "math.h"
#include "types.h"
#include "init30a.h"
#include "mu.h"
#include "mobility.h"
#include "vlf_comms.h"
#include "uhf_comms.h"
#include "cc1000.h"
#include "time.h"
#include "wait.h"
//#include "string.h"
#include "vlf_pkt.h"



#if( HPT4_MODE )
#else
bank1 Pkt_rx_buf_st vrx;
#endif

bank1 Pkt_tx_buf_st vtx;

volatile BYTE vlf_state = VLF_STATE_IDLE;	// Set to no VLF event


// Bit array is ordered so that byte[0] = LSB and byte[VLF_SU_TO_MU_PREAMBLE_SIZE] = MSB
const BYTE su2mu_preamble[VLF_SU_TO_MU_PREAMBLE_SIZE] = {
   (SU_TO_MU_PREAMBLE      ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 1 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 2 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 3 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 4 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 5 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 6 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 7 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 8 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 9 ) & 0x01,
   (SU_TO_MU_PREAMBLE >> 10) & 0x01,
 #if( VLF_SU_TO_MU_PREAMBLE_SIZE == 16 )
   (SU_TO_MU_PREAMBLE >> 11) & 0x01,
   (SU_TO_MU_PREAMBLE >> 12) & 0x01,
   (SU_TO_MU_PREAMBLE >> 13) & 0x01,
   (SU_TO_MU_PREAMBLE >> 14) & 0x01,
   (SU_TO_MU_PREAMBLE >> 15) & 0x01
 #elif( VLF_SU_TO_MU_PREAMBLE_SIZE == 20 )
   (SU_TO_MU_PREAMBLE >> 11) & 0x01,
   (SU_TO_MU_PREAMBLE >> 12) & 0x01,
   (SU_TO_MU_PREAMBLE >> 13) & 0x01,
   (SU_TO_MU_PREAMBLE >> 14) & 0x01,
   (SU_TO_MU_PREAMBLE >> 15) & 0x01,
   (SU_TO_MU_PREAMBLE >> 16) & 0x01,
   (SU_TO_MU_PREAMBLE >> 17) & 0x01,
   (SU_TO_MU_PREAMBLE >> 18) & 0x01,
   (SU_TO_MU_PREAMBLE >> 19) & 0x01
 #elif( VLF_SU_TO_MU_PREAMBLE_SIZE == 12 )
   (SU_TO_MU_PREAMBLE >> 11) & 0x01
 #else
 #error "Invalid SU2MU_PREAMBLE"
 #endif
};

#if( USE_APS )
const BYTE aps_preamble[APS_PREAMBLE_SIZE] = {
   (APS_PREAMBLE >> 4 ) & 0x01,		// 1
   (APS_PREAMBLE >> 3 ) & 0x01,		// 0
   (APS_PREAMBLE >> 2 ) & 0x01,		// 1
   (APS_PREAMBLE >> 1 ) & 0x01,		// 1
   (APS_PREAMBLE >> 0 ) & 0x01,		// 0
};
#endif

 #if( MU_VLF_TX_TESTING == 2 )

const BYTE mu2su_preamble[VLF_MU_TO_SU_PREAMBLE_SIZE] = {
   0x01,
   0x01,
   0x01,
   0x01,
   0x01,
   0x01,
   0x01,
   0x01
};

 #else

const BYTE mu2su_preamble[VLF_MU_TO_SU_PREAMBLE_SIZE] = {
   (MU_TO_SU_PREAMBLE     ) & 0x01,
   (MU_TO_SU_PREAMBLE >> 1) & 0x01,
   (MU_TO_SU_PREAMBLE >> 2) & 0x01,
   (MU_TO_SU_PREAMBLE >> 3) & 0x01,
   (MU_TO_SU_PREAMBLE >> 4) & 0x01,
   (MU_TO_SU_PREAMBLE >> 5) & 0x01,
   (MU_TO_SU_PREAMBLE >> 6) & 0x01,
   (MU_TO_SU_PREAMBLE >> 7) & 0x01
};
 #endif

// PROTOCOL 2 - New protocol SU to MU
// Add 12 bits for the preamble + 4 bits for the type and 8 bits for the CRC8.
// Each byte takes 80 ms to send  to the MU note that there are 4 bits for type = 16
// combinations of which we currently only use 4

const BYTE uncoded_rx_data_size[] = {
   2, // VLF_MU_DETECT,         Sent to MU with 1 byte SU ID and 1 byte sequence = 5*80=400ms
   5, // VLF_MU_MASK,           Sent to MU with 3 byte MU ID and 1 byte suid and 1 byte sequence = 8*80=640ms
   0, // VLF_MU_QUICK_SEARCH_Y, Sent to MU WITH NO PAYLOAD 3*80=400ms
   0, // VLF_MU_QUICK_SEARCH_Z, Sent to MU WITH NO PAYLOAD 5*80=400ms
   4, // VLF_MU_LOCATE,         Sent to MU with 3 byte MU ID and 1 byte for number of seconds = 7*80=560ms
   0, // VLF_MU_VLF_RX_CONTROL  NOT USED
   0, // VLF_MU_QUICK_SEARCH_X  Sent to MU with NO PAYLOAD 3*80=400ms
   0, // VLF_MU_TEST
   4  // VLF_MU_MASK_FULL  		Sent to MU with 3 byte MU ID and 1 byte for number of seconds
//   0, // VLF_MU_FLEXALERT
//   0, // VLF_MU_CALL_1
//   0  // VLF_MU_CALL_2
};





static void VLF_SendPacket( void );
static BYTE buffcrc( BYTE *ptr, BYTE crc, uchar len );
static BYTE get_crc( BYTE crc, BYTE data );

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
static BYTE get_crc( BYTE crc, BYTE data )
{
register uchar bit_cnt;

	for( bit_cnt = 0; bit_cnt < 8; bit_cnt++ )
	{	if( crc & 0x80 )
		{   crc <<= 1;
         	if( data & 0x80 )
            	crc |= 0x01;
	       	crc = crc ^ 0x85;
      	}
		else
		{  	crc <<= 1;
        	if( data & 0x80 )
            	crc |= 0x01;
      	}
      	data <<= 1;
   	}
   	return( crc );
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
static BYTE buffcrc( BYTE *ptr, BYTE crc, uchar len )
{
	while( len-- )
    	crc = get_crc( crc, *ptr++ );
   	return( crc );
}


#if( HPT4_MODE )
#else

//---------------------------------------------------------------------------
// Returns nonzero if the VLF interface is busy
//---------------------------------------------------------------------------
uchar VLF_Busy(void)
{
   if( ( vrx.state == VLF_RX_SEEKING )&&( vtx.state == VLF_TX_IDLE ) )
      return( FALSE );

   return( TRUE );
}



//---------------------------------------------------------------------------
// Handles a packet RX'd on the VLF interface.
// This function is called when a packet is received on the vLF interface. The packet is assembled
// from the Viterbi traceback buffer in EEPROM and checked for validity. Finally, it is processed and
// a repsonse generated if need be.
//---------------------------------------------------------------------------
BYTE VLF_ProcessPacket(void)
{
BYTE crc;

#if( DEBUG_LEVEL >= 2 )
	TEST_LED5_VLF_RX = TEST_LEDX_ON;
#endif

	if( vtx.type >= VLF_NO_PACKET  )
	{   VLF_StartReceiver(); // Not processed so just go back to waiting for another packet

#if( DEBUG_LEVEL >= 2 )
	TEST_LED5_VLF_RX = TEST_LEDX_OFF;
#endif

		return( 1 );
	}

	crc = get_crc( SU_TO_MU_PREAMBLE_CRC, (vtx.type << 4) );
	crc = buffcrc( &vtx.u.data[0], crc, UNCODED_RX_PAYLOAD_SIZE( vtx.type ) - 1 );
   	crc = get_crc( crc, 0x00 );


	if( vtx.u.data[UNCODED_RX_DATA_SIZE( vtx.type )] != crc )
	{ 	VLF_StartReceiver(); // Not processed so just go back to waiting for another packet
#if( DEBUG_LEVEL >= 2 )
		TEST_LED5_VLF_RX = TEST_LEDX_OFF;
#endif
		return( 2 );
	}


	switch( vtx.type )
	{
	case VLF_MU_DETECT:
		// Only respond if the SU ID or sequence are different - i.e. not masked
		if( ( vtx.u.det_cmd.sid !=  vtx.sid )||( vtx.u.det_cmd.seq != vtx.seq ) )
		{
			// Since this is a new sequence or SU-ID, unmask ourselves if we were masked
			if( lamp_state == LAMP_SEARCH_IN_PROGRESS )
		  	{  	lamp_state = LAMP_NORMAL_STATE;

#if( USE_INVERT_LAMP )
				LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON; 	// On
#else
	      		LAMP_FET = LAMP_ON;	  									// On
#endif
		    }
			else if( lamp_state == LAMP_VLF_BLOCKED_STATE )
			{
/*
				if( vtx.u.det_cmd.seq != vtx.seq )
			  	{  	lamp_state = LAMP_NORMAL_STATE;

#if( USE_INVERT_LAMP )
					LAMP_FET =  ( mu_flags & FL_LAMP_INV ) ? 1 : LAMP_ON; 	// On
#else
	      			LAMP_FET = LAMP_ON;	  							// On
#endif
				}
				else
*/
				{	VLF_StartReceiver(); 	// Not processed so just go back to waiting for another packet
					return( 0 );
				}
			}


  		  	delay_ms( DETECT_RX_DELAY_MS * 2 );	// Delay to allow Y signal to come through in worst case of X
           	//EvtQueueWriteFromMain(  VLF_REPLY_BROADCAST );
           	VLF_ReplyToBroadcastRequest( SEARCH_ID_INTERVALS );
           	return( 0 );		// Return without calling WaitForPacket();
		}
      	break;


	case VLF_MU_MASK:
		if(
			( vtx.u.msk_cmd.mid_s.val[2] == mid.b[1] )&&
	    	( vtx.u.msk_cmd.mid_s.val[1] == mid.b[2] )&&
	    	( vtx.u.msk_cmd.mid_s.val[0] == mid.b[3] ) ) 	// If it is for us
  	    {
	    	delay_ms( MASK_RX_DELAY_MS * 2 );	// Delay to allow Y & Z signal to come through in worst case
         	//EvtQueueWriteFromMain( VLF_REPLY_UNICAST );
			if( lamp_state != LAMP_SEARCH_IN_PROGRESS )
			{  	lamp_state = LAMP_SEARCH_IN_PROGRESS;
	            lamp_timeout = get_timeout( LAMP_SEARCH_STATE_TIMEOUT );
			}
			vtx.seq = vtx.u.msk_cmd.seq;
			vtx.sid = vtx.u.msk_cmd.sid;
			VLF_SendGeneric();
           	return( 0 );		// Return without calling WaitForPacket();
		}
        break;

	case VLF_MU_QUICK_SEARCH_X:
   		delay_ms( QUICKSEARCH_RX_DELAY_MS );	// Delay to allow Y signal to come through

  	case VLF_MU_QUICK_SEARCH_Y:
   	    delay_ms( QUICKSEARCH_RX_DELAY_MS );	// Delay to allow Z signal to come through

    case VLF_MU_QUICK_SEARCH_Z:

		if( lamp_state == LAMP_VLF_BLOCKED_STATE )
			VLF_StartReceiver(); 	// Not processed so just go back to waiting for another packet
		else
       		VLF_ReplyToBroadcastRequest( SEARCH_QUICK_INTERVALS );
	   	return( 0 );	// Return without calling WaitForPacket();

    case VLF_MU_LOCATE:
	  	if(
			( vtx.u.msk_cmd.mid_s.val[2] == mid.b[1]) &&
		   	( vtx.u.msk_cmd.mid_s.val[1] == mid.b[2]) &&
		   	( vtx.u.msk_cmd.mid_s.val[0] == mid.b[3])  ) 	// If it is for us
	    {   delay_ms( LOCATE_RX_DELAY_MS * 2 );	// Delay to allow Y signal to come through in worst case of X
        	VLF_ReplyLocateRequest( vtx.u.loc_cmd.duration );
			VLF_StartReceiver();	// Return to VLF receive mode whereby we listen for incoming messages
           	return( 0 );			// Return without calling WaitForPacket();
        }
      	break;

	case VLF_MU_TEST:

		VLF_SendGeneric();

        return( 0 );		// Return without calling WaitForPacket();


	case VLF_MU_MASK_FULL:
		if(
			( vtx.u.msk_full_cmd.mid_s.val[2] == mid.b[1] )&&
	    	( vtx.u.msk_full_cmd.mid_s.val[1] == mid.b[2] )&&
	    	( vtx.u.msk_full_cmd.mid_s.val[0] == mid.b[3] ) ) 	// If it is for us
  	    {
	    	delay_ms( MASK_RX_DELAY_MS * 2 );					// Delay to allow Y & Z signal to come through in worst case

			lamp_min_cnt = vtx.u.msk_full_cmd.timer;
			if( lamp_min_cnt )
			{   lamp_state = LAMP_VLF_BLOCKED_STATE;
	            lamp_timeout = get_timeout( LAMP_60SEC_TIMEOUT );
			}
			else
			{	lamp_state = LAMP_NORMAL_STATE;
#if( USE_INVERT_LAMP )
				LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON; 	// On
#else
	      		LAMP_FET = LAMP_ON;	  			 					// On
#endif
			}
			VLF_SendGeneric();
           	return( 0 );		// Return without calling WaitForPacket();
		}
        break;
/*
	//----------------
	case VLF_MU_FLEXALERT:
		// "Evacuate" Message from FlexAlert
		if( lamp_state >= LAMP_PAGER_ALARM )
		{   lamp_state = LAMP_PAGER_ALARM;
			lamp_timeout = get_timeout( LAMP_ALARM_UHF_TIMEOUT );
			mu_flags |= FL_ACK;
		}
      	break;

	//----------------
	case VLF_MU_CALL_1:
		// "Call" Message from FlexAlert
		if( lamp_state >= LAMP_PAGER_ALARM )
		{	lamp_state =  LAMP_PAGER_CALL_1;
   	 		lamp_timeout = get_timeout( LAMP_CALL_STATE_TIMEOUT );
			mu_flags |= FL_ACK;
		}
      	break;

	//----------------
	case VLF_MU_CALL_2:
		// "Call" Message from FlexAlert
		if( lamp_state >= LAMP_PAGER_ALARM )
		{	lamp_state =  LAMP_PAGER_CALL_2;
   	 		lamp_timeout = get_timeout( LAMP_CALL_STATE_TIMEOUT );
			mu_flags |= FL_ACK;
		}
      	break;
*/
	//----------------
	default:
		VLF_StartReceiver(); 	// Not processed so just go back to waiting for another packet
		return( 3 );

	}
	VLF_StartReceiver(); 	 	// Not processed so just go back to waiting for another packet
	return( 0 );
}

//---------------------------------------------------------------------------
// The routine that handles an RX signal crossing threshold reading
// The handler goes through a number of states.
//  - When seeking, the handler will try to match last number of symbols received to the opening flag.
//  - If this succeeds, the handler will try to determine a valid packet type for the incoming packet.
//  - When receiving, the handler will post a VLF_RX_DECODE event after every symbol is received.
//    After the last symbol is decoded, a VLF_PACKET_RXD will be posted.
// Returns TRUE if packet ready
//---------------------------------------------------------------------------
char VLF_ProcessBitRead(void)
{
register BYTE  ii;

#ifdef VLF_TEST_TAG
	vrx.state =  VLF_RX_APS;
#endif 

	switch( vrx.state )
	{
#if( USE_APS )

	case VLF_RX_APS:
{
 		if( vrx.cnt >= APS_PREAMBLE_SUBP_SIZE ) 	// Make sure it is primed before checking for sync
 		{
	    	for( ii = 0, vrx.unlocked = 0, vrx.preamble_score = 0;
	               ( ii < APS_PREAMBLE_SIZE )&&( !vrx.unlocked ); ii++ )
			{
	        	vrx.score = vrx.preamble[ (ii<<2)     ] +   // Unroll preamble loop
	                        vrx.preamble[ (ii<<2) + 1 ] +
	                        vrx.preamble[ (ii<<2) + 2 ] +
	                        vrx.preamble[ (ii<<2) + 3 ];

	            vrx.score >>= SCORE_METRIC_SHIFT;

	            if( aps_preamble[ii] )
	          	{   if( vrx.score <= SCORE_METRIC_HIGH_THRESH )
	                  	vrx.unlocked = 1;
	               	else
	                  	vrx.preamble_score += (SCORE_METRIC_FS - vrx.score );
	          	}
	            else
	          	{   if( vrx.score >= SCORE_METRIC_LOW_THRESH )
	                  	vrx.unlocked = 1;
	               	else
	                  	vrx.preamble_score += vrx.score;
	         	}
	    	}

	    	if( vrx.prev_preamble_score != VLF_START_SCORE )
	      	{  	if( ( vrx.unlocked )||( vrx.preamble_score > vrx.prev_preamble_score ) )
	            { 	//preambule detected
	               	vrx.cnt = 1;
	               	vrx.prev_preamble_score = VLF_START_SCORE;
					LED_GREEN ^= 1;

					if(vrx.flags.aps.combiner == FALSE)
					{
					//set flag TRAP_8KHZ
						mu_flags |= FL_8KHZ_TRAP;
					// для укорачивания режима моргания будем брать таймаут ~1 сек
						if( lamp_state == LAMP_NORMAL_STATE )
						{
							lamp_state = LAMP_IDENTIFY;
	        				lamp_timeout = get_timeout( LAMP_APS_TIMEOUT );
						}
					}
					mu_flags |= FL_8KHZ_TRAP;
	         	}
	            else
	                vrx.prev_preamble_score = vrx.preamble_score;
			}
	        else if( !vrx.unlocked )
	       	{
				vrx.prev_preamble_score = vrx.preamble_score;	
			}
		}
	    else
	    	vrx.cnt++;

#if( DEBUG_LEVEL >= 2 )
		TEST_LED5_VLF_RX = TEST_LEDX_OFF;
#endif
}
		break;
#endif
	//---------------------------------------------------------------------------
	case VLF_RX_SEEKING:       						// Attempt to lock onto the opening flag of the incoming signal

{
 		if( vrx.cnt >= PCKT_PREAMBLE_SUBP_SIZE ) 	// Make sure it is primed before checking for sync
 		{
	    	for( ii = 0, vrx.unlocked = 0, vrx.preamble_score = 0;
	               ( ii < VLF_SU_TO_MU_PREAMBLE_SIZE )&&( !vrx.unlocked ); ii++ )
			{

/*
	        	vrx.score = vrx.preamble[ ii*VLF_NUM_RX_SYMBOL_SUBPERIODS     ] +   // Unroll preamble loop
	                        vrx.preamble[ ii*VLF_NUM_RX_SYMBOL_SUBPERIODS + 1 ] +
	                        vrx.preamble[ ii*VLF_NUM_RX_SYMBOL_SUBPERIODS + 2 ] +
	                        vrx.preamble[ ii*VLF_NUM_RX_SYMBOL_SUBPERIODS + 3 ];
*/

	        	vrx.score = vrx.preamble[ (ii<<2)     ] +   // Unroll preamble loop
	                        vrx.preamble[ (ii<<2) + 1 ] +
	                        vrx.preamble[ (ii<<2) + 2 ] +
	                        vrx.preamble[ (ii<<2) + 3 ];

	            vrx.score >>= SCORE_METRIC_SHIFT;

/*
	            if( su2mu_preamble[ii] == 0 )
	          	{   if( vrx.score >= SCORE_METRIC_LOW_THRESH )
	                  	vrx.unlocked = 1;
	               	else
	                  	vrx.preamble_score += vrx.score;
	         	}
	            else //if ( su2mu_preamble[ii] == 1 )
	          	{   if( vrx.score <= SCORE_METRIC_HIGH_THRESH )
	                  	vrx.unlocked = 1;
	               	else
	                  	vrx.preamble_score += (SCORE_METRIC_FS - vrx.score );
	          	}
*/
	            if( su2mu_preamble[ii] )
	          	{   if( vrx.score <= SCORE_METRIC_HIGH_THRESH )
	                  	vrx.unlocked = 1;
	               	else
	                  	vrx.preamble_score += (SCORE_METRIC_FS - vrx.score );
	          	}
	            else
	          	{   if( vrx.score >= SCORE_METRIC_LOW_THRESH )
	                  	vrx.unlocked = 1;
	               	else
	                  	vrx.preamble_score += vrx.score;
	         	}
	    	}

	    	if( vrx.prev_preamble_score != VLF_START_SCORE )
	      	{  	if( ( vrx.unlocked )||( vrx.preamble_score > vrx.prev_preamble_score ) )
	            {  	vrx.state = VLF_RX_LOCKED;
	               	vrx.cnt = 1;
	               	vrx.prev_preamble_score = VLF_START_SCORE;
	         	}
	            else
	                vrx.prev_preamble_score = vrx.preamble_score;
			}
	        else if( !vrx.unlocked )
	       	{
				vrx.prev_preamble_score = vrx.preamble_score;
			}
		}
	    else
	    	vrx.cnt++;


//#if( MU_VLF_RX_ENABLE_LED_FEEDBACK )
//		if( vrx.state == VLF_RX_LOCKED )
//		{
//   			TESTPT29 = 1;		// signal that the preamble is locked
//   		}
//#endif

#if( DEBUG_LEVEL >= 2 )
		TEST_LED5_VLF_RX = TEST_LEDX_OFF;
#endif
}


      	break;

	//---------------------------------------------------------------------------
    case VLF_RX_LOCKED:
      	vtx.type = 0x00;

        if( (vrx.cnt & ((VLF_NUM_RX_SYMBOL_SUBPERIODS*PCKT_TYPE_SIZE) - 1) ) ==
                             ((VLF_NUM_RX_SYMBOL_SUBPERIODS*PCKT_TYPE_SIZE) - 1) )
		{
        	for( ii = 0; (ii < PCKT_TYPE_SIZE); ii++ )
            {
               	vrx.score = vrx.preamble[sizeof(vrx.preamble) - ((ii << 2) + 1)] +   // Unroll preamble loop
                	        vrx.preamble[sizeof(vrx.preamble) - ((ii << 2) + 2)] +
                            vrx.preamble[sizeof(vrx.preamble) - ((ii << 2) + 3)] +
                            vrx.preamble[sizeof(vrx.preamble) - ((ii << 2) + 4)];

               	vrx.score >>= SCORE_METRIC_SHIFT;

               	vtx.type <<= 1;
               	if( vrx.score >= SCORE_METRIC_MID_THRESH )
                  	vtx.type |= 0x01;
          	}

            if( vtx.type < VLF_NO_PACKET )
            {
				vrx.len = UNCODED_RX_PAYLOAD_SIZE( vtx.type );
			   	vrx.ptr = &vtx.u.data[0];
			   	vrx.shifter = 0x01;
               	vrx.state = VLF_RX_RECEIVING;
               	vrx.cnt = 1;
            }
            else
            {  	vrx.state = VLF_RX_SEEKING;
               	vrx.cnt = 0;

#if( DEBUG_LEVEL >= 2 )
				TEST_LED5_VLF_RX = TEST_LEDX_OFF;
#endif
			}
  		}
        else
        	vrx.cnt++;
    	break;

	//---------------------------------------------------------------------------
      case VLF_RX_RECEIVING:
         if( ( vrx.cnt & (VLF_NUM_RX_SYMBOL_SUBPERIODS - 1) ) == (VLF_NUM_RX_SYMBOL_SUBPERIODS-1) )
         {
            vrx.score = vrx.preamble[sizeof(vrx.preamble) - 1] +   // Average the samples
                        vrx.preamble[sizeof(vrx.preamble) - 2] +
                        vrx.preamble[sizeof(vrx.preamble) - 3] +
                        vrx.preamble[sizeof(vrx.preamble) - 4];
            vrx.score >>= SCORE_METRIC_SHIFT;

            // If the average of the 4 reads is greater than or equal to the the mid-threshold, then set the bit
            if ( vrx.score >= SCORE_METRIC_MID_THRESH )
           		*vrx.ptr |=  vrx.shifter;
            else
            	*vrx.ptr &= ~vrx.shifter;	// Otherwise clear it

			if( vrx.shifter < 0x80 )
				vrx.shifter <<= 1;
			else
			{	vrx.shifter = 0x01;
				vrx.ptr++;

	            if( !(--vrx.len ) )			// If count completed, Disable TMR2 interrupt
    	        {   TMR2IE = 0; 		   	// PIE1 &= ~0x02  Disable TMR2 interrupt for VLF-BIT reading
        	    	TMR2ON = 0;
            		vrx.state = VLF_PACKET_READY;
            		return( TRUE );
            	}
			}
/*
            if( --vrx.len == 0 )		// If count completed, Disable TMR2 interrupt
            {   TMR2IE = 0; 					// PIE1 &= ~0x02  Disable TMR2 interrupt for VLF-BIT reading
            	TMR2ON = 0;
            	vrx.state = VLF_PACKET_READY;
            	return( TRUE );
            }
*/
        }
      	vrx.cnt++;
      	break;

	case VLF_PACKET_READY:
		return( TRUE );
	}

	// Shift the preamble array
   	for( ii = 0; ii < sizeof(vrx.preamble) - 1; ii++ )
    	vrx.preamble[ii] = vrx.preamble[ii+1];

   	return( FALSE );
}
#endif


//---------------------------------------------------------------------------
// Handles a reply broadcast command
// Send in timeslots from 0...intervals-1 in multiples of 300 ms
//---------------------------------------------------------------------------
void VLF_ReplyToBroadcastRequest( BYTE intervals )
{
BYTE rand_time_u8;
	rand_time_u8 = rand() % intervals;
	while( rand_time_u8-- )
		delay_ms( 290 );
	VLF_SendGeneric();	// Send a generic response
}

//---------------------------------------------------------------------------
// Send generic responses for the duration of the locate request
//---------------------------------------------------------------------------
void VLF_ReplyLocateRequest( uchar locate_seconds )
{

/*
ushort delay_u16;
	TMR1IE = 0;		// Disable the interrupt for Timer1 to prevent accidental changes to this countdown timer
	countdown_timer_ticks_u16 = ( (ushort)locate_seconds * TICKS_PER_SEC );
	TMR1IE = 1;		// Reenable the interrupt

	while( countdown_timer_ticks_u16 ) 	// Note: countdown_timer_ticks_u16 is decremented on the timer interrupt
	{
		CLRWDT();
		delay_u16 = countdown_timer_ticks_u16;

		MeasureMobility();
		VLF_SendGeneric();
		while( vtx.state != VLF_TX_IDLE );	// Wait until the packet has been transmitted


		// Wait for about 1/2 second between transmissions
		while( ( countdown_timer_ticks_u16 )&&( (delay_u16 - countdown_timer_ticks_u16) <= TICKS_PER_SEC/2) )
		{
			MeasureMobility();	// Take multiple mobility measurements while waiting
		}
	}
*/
ushort timeout;

	locate_seconds <<= 1;

	while( locate_seconds-- )
	{ 	// Wait for about 1/2 second between transmissions
		timeout = get_timeout( (TICKS_PER_SEC + 1) / 2 );
//		MeasureMobility();
		VLF_SendGeneric();
		while( vtx.state != VLF_TX_IDLE );		// Wait until the packet has been transmitted

		while( chk_timeout( timeout ) == FALSE )
		{
//			MeasureMobility();					// Take multiple mobility measurements while waiting
		}
	}

}



#if( HPT4_MODE )
#else
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void VLF_StartReceiver(void)
{
	vtx.type = VLF_NO_PACKET;
	vrx.len = 0;
	vrx.cnt = 0;			// Reset count to wait until the preamble is primed
	vtx.state = VLF_TX_IDLE;
   	vrx.state = VLF_RX_SEEKING;
   	vlf_state = VLF_STATE_IDLE;

   	vrx.prev_preamble_score = VLF_START_SCORE;

   	SWDTEN = 0;             // Disable Watchdog timer to avoid a WDT reset

//  OPTION = 0xE8;      	// 11101000 PORTB pull-ups are disabled, Transition on low-to-high transition ON T0CKI pin
   	OPTION = 0x68;      	// 11101000 PORTB pull-ups are Enabled, Transition on low-to-high transition ON T0CKI pin
   							// Interrupt on rising edge of INT pin, Prescaler is assigned to the WDT
   	CLRWDT();
   	SWDTEN = 1;             // Re-enable WDT Timer0 is used as a comparator pulse counter

// OPTION = 0b11100000;     // 11100000 PORTB pull-ups are disabled,
 							// Transition on low-to-high transition ON T0CKI pin
 							// Interrupt on rising edge of INT pin
 							// Prescaler is assigned to the Timer0 module
 							// Prescaler is 1:1
   	TMR0 = 0;
   	T2CON = ( TMR2_POSC << 3 ) | TMR2_PRESC_RX;
   	PR2  = PR2_RX;
   	TMR2 = 0;
   	TMR2ON = 1;

   	TMR2IF = 0; 			// PIR1 &= ~0x02;  Clear the TMR2 interrupt flag
   	TMR2IE = 1; 			// PIE1 |= 0x02;   Enable TMR2 interrupt for VLF-BIT reading
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void VLF_StopReceiver(void)
{
	TMR2IE = 0; 		   	// PIE1 &= ~0x02  Disable TMR2 interrupt for VLF-BIT reading
    TMR2ON = 0;
}
#endif


//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
static void VLF_SendPacket(void)
{
#if( DEBUG_LEVEL >= 2 )
	TEST_LED4_VLF_TX = TEST_LEDX_ON;
#endif

 	TMR2IE = 0;  	// Disable Timer2 interrupt
   	TMR2ON = 0;		// TURN off Timer2
   	TMR2   = 0;		// Reset Timer2

	// Setup timer 2 to the transmit frequency
   	T2CON 	= (TMR2_POSC << 3) | TMR2_PRESC_TX;
   	PR2  	= (BYTE)PR2_TX;

/*
   	CCPR1L 	=   tx_duty_u8 >> 2;
   	CCP1CON = ((tx_duty_u8 & 3 ) << 4);
*/
   	CCPR1L 	=     (BYTE)TX_FULL_POWER >> 2;
   	CCP1CON = ( ( (BYTE)TX_FULL_POWER &  3 ) << 4 );

   	vtx.cnt = 0;
   	vtx.state = VLF_TX_PACKET_PREAMBLE;

   	SWDTEN = 0;               	// Disable Watchdog timer to avoid a WDT reset
   	OPTION = 0xC0 | TMR0_PRESC; // 0xC0 = 0b11000000 =
   								// 1 = PORTB pull-ups are disabled,
   								// 1 = Interrupt on rising edge of INT pin
   								// PRESCALER IS FOR Timer0
   	CLRWDT();
   	SWDTEN = 1;               	// Re-enable WDT Timer0 is used as a comparator pulse counter
	// Reset Timer0
   	T0IF = 0;
   	TMR0 = 0xFF;
   	T0IE = 1;		// Enable Timer0 interrupt to transmit a bit pattern
   	TMR2ON = 1;		// Enable Timer2
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void VLF_SendGeneric(void)
{
BYTE crc;		// Preamble MU to SU is only 8 bits long and there is no type
	vtx.type = VLF_MU_DETECT;
	vtx.u.responce.mid_s.val[0] = mid.b[3];
    vtx.u.responce.mid_s.val[1] = mid.b[2];
    vtx.u.responce.mid_s.val[2] = mid.b[1];
	vtx.u.responce.m.status = 0;
#if( USE_MOBILITY )
    vtx.u.responce.m.s.mobility = GetMobilityValue();
#else
    vtx.u.responce.m.s.mobility = 1;
#endif
    vtx.u.responce.m.s.masked = ( lamp_state == LAMP_SEARCH_IN_PROGRESS ) ? 1 : 0;

	crc = buffcrc( vtx.u.data, MU_TO_SU_PREAMBLE_CRC, MU_PROTOCOL2_RESPONSE_PAYLOADSIZE );
	crc = get_crc( crc, 0x00 );

	// If the checksum looks like the preamble, then modify one of the bits to force it to be different
	if( crc == MU_TO_SU_PREAMBLE )
	{   vtx.u.responce.m.s.bit6 = 1;
		crc = buffcrc( &vtx.u.data[0], MU_TO_SU_PREAMBLE_CRC, MU_PROTOCOL2_RESPONSE_PAYLOADSIZE );
		crc = get_crc( crc, 0x00 );
	}
  	vtx.u.data[MU_PROTOCOL2_RESPONSE_PAYLOADSIZE] = crc;
    VLF_SendPacket();
}


#if( MU_VLF_TX_TESTING )
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void TestVlfTx( ushort times )
{
(void)times;
//	while( times )
//	{
		CLRWDT();
//		MeasureMobility();
//		TEST_LED_PCB = TEST_LEDX_ON;
		VLF_SendGeneric();
		while( vtx.state != VLF_TX_IDLE );	// Wait until the packet has been transmitted
//		TEST_LED_PCB = TEST_LEDX_OFF;

//		WaitMs( 30 );

//#if( MU_VLF_TX_TESTING == 1 )
		delay_ms( 2000 );
//#else
//		delay_ms( 2000 );
//#endif
//
//	    times--;
//	}
}
#endif

