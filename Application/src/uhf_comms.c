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
 * \file uhf_comms.c
 * \brief Source file for high frequency communicatons functionality.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
	10.01.2011 Igors Zalts
 */

#include <htc.h>
#include "mu.h"
#include "eeprom.h"
#include "types.h"
#include "time.h"
#include "wait.h"
#include "uhf_cmn.h"
#include "uhf_comms.h"
#include "cc1000.h"
#include "stdlib.h"
#include "init30a.h"


#include "vlf_comms.h"
#include "vlf_pkt.h"


bank2 Uhf_pkt_s	uhf;

ushort      uhf_registration_timeout;

bank3 bid_u		  bb[BB_LIST_SIZE];		// Bounded beacons list
bank3 bid_u		  bi[BB_LIST_SIZE];		// Invitation beacons list
bank3 ushort	  combine_id = 0;			// ID combine 
bank3 ushort 	  bid_temp = 0;

bank3 BYTE 		bl_version = 0;

static ushort  	uhf_ctrl_sidx = 0;
static BYTE    	uhf_ctrl_sctr = 0;

static void SendUHFPkt( void );

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
static void PrepareSearchQueryReply(void)
{
   	uhf.pkt.csqr.mid = mid;

	if( ( lamp_state == LAMP_NORMAL_STATE )&&( mu_flags & FL_LAMP_INV ) )
		uhf.pkt.csqr.state = LAMP_INVERT_STATE;
	else
		uhf.pkt.csqr.state = lamp_state;

//#if( ENABLE_DEBUG_FLAGS )
//	uhf.pkt.csqr.wdog_resets = dflags.f.value;
//#else

  #if( ALLOW_UHF_BOOTMODE )
	uhf.pkt.csqr.wdog_resets = eeprom_read( EEPROM_SD_OFFSETOF( wdog_resets ) );
  #else
	uhf.pkt.csqr.wdog_resets = 0;
  #endif
//#endif


#if( ALLOW_UHF_BOOTMODE )
	uhf.pkt.csqr.vers_boot = bl_version;
#else
   	uhf.pkt.csqr.vers_boot = 0;
#endif
   	uhf.pkt.csqr.vers = APP_FIRMWARE_VERSION;

}

//---------------------------------------------------------------------------
// UHF considered busy if in the process of either decoding a packet or sending a packet
// In rare circumstances, we could be inbetween the two states so it is important that the
// calling function also checks the uhf_packet_ready variable to make sure it is ok to do what it is about to do
//---------------------------------------------------------------------------
BYTE UHFBusy(void)
{
   if( uhf.rxt_state == UHF_RXTX_PREAMBLE )	// If we're still looking for a preamble
      return( 0 ); 								// Then we're not busy on the UHF

   return( 1 );						   			// Otherwise we're busy
}

//---------------------------------------------------------------------------
// The process routine for a received UHF packet which is stored in the uhf_pkt
// structure
//---------------------------------------------------------------------------
BYTE ProcessUHFPacket(void)
{
	// Skip the packet if it is not from an ILB device or if the checksum is bad
 	if( uhf.pkt.bi.ms == CMD_MASTER )
	{ 	if( CalcUHFChecksum( 0 ) )
		{   CREN = 1;				// Enable the UHF UART RX
      		RCIE = 1;       		// Enable interrupt on UHF RX
      		return( 1 );
		}
	}
	else
	{	CREN = 1;
      	RCIE = 1;
      	return( RESULT_OK );
	}



	// Service the message type
	switch( uhf.pkt.bi.type )
	{
	// Broadcast Invitation (BI) ----------------------------------------
    case UHF_BI_PKT:
		// This message is sent from an ILB to all Transponders that may have recently entered the Beacon’s area
		// and have not been accounted for yet.  Transponders new to the area reply back with the transponder’s
		// ID and status information. The BI message contains “NO_SLT” (number of timeslots)
		// and “NO_GRP” (number of groups).  These are used by the tag to select a random timeslot
		// to respond in and to determine which group it should respond in.
		// These are used to help reduce the chance of multiple transponders responding to the ILB
 		// at the same time (i.e. to reduce the possibility of message collisions).

		// Broadcast Invitation Reply (BIR)
		// If Beacon ID is received Broadcast Invitations and it differs from what the Transponder has in its memory,
		// the Transponder will respond with the Broadcast Invitation Reply.
		// It will choose a random time slot in which to send the BIR back to the ILB.


#if( ( HPT3_MODE == 4 )||( HPT3_MODE == 6 ) )

#endif
#if(USE_APS)
		if(uhf.pkt.bi.ext_reserv & BIT1)		
		{
			vrx.state = VLF_RX_APS;
			//LED_GREEN = TEST_LEDX_ON;
			vrx.flags.aps.enable = TRUE;
			vrx.aps_timeout = get_timeout( APS_TIMEOUT);
			// запомним id бикона комбайна
			combine_id = uhf.pkt.bi.bid.w;
			combine_id = NTOHS(combine_id) & 0x0FFF;
		}

		//service-state check-out
		//prevent TRAP_8KHZ lamp blinking
		bid_temp = uhf.pkt.bi.bid.w;
		bid_temp = NTOHS(bid_temp) & 0x0FFF;
		if ( (combine_id == bid_temp ) && ((uhf.pkt.bi.ext_reserv & BIT1) == FALSE) )
		{	
		//	vrx.aps_timeout = get_timeout( 15 );
			LED_GREEN = TEST_LEDX_OFF;
			vrx.state = VLF_RX_SEEKING;
			vrx.flags.aps.enable = FALSE;
			vrx.flags.aps.combiner = FALSE;
			mu_flags &=~(FL_8KHZ_TRAP);			
		}


#endif

		if( uhf.pkt.bi.ext &&( ( mu_flags & FL_ACK_FORCED ) == 0  ) )
		{	mu_flags |= FL_ACK_FORCED;
			bb[uhf.pkt.bi.bid.ho.lo & BB_LIST_MASK].w  = 0;			// Clear Index Table
		}


//флаг FL_8KHZ_TRAP сбрысывает при получении ответа UHF_BIRC_PKT
// Таг комбайнера
#if(USE_APS)
		if((vrx.flags.aps.combiner == FALSE)&&(	vrx.flags.aps.enable == TRUE))
		{	
			if( mu_flags & FL_8KHZ_TRAP )		
			{	//перерегистрация
				char i;
				mu_flags &= (~(FL_REGISTRATION|FL_ACK_FORCED|FL_QUIET_PAGER) );

				for( i=0; i<BB_LIST_SIZE; i++ )
					bb[i].w = bi[i].w = 0;			
			}
		}
#endif
    	if( bb[uhf.pkt.bi.bid.ho.lo & BB_LIST_MASK].w != uhf.pkt.bi.bid.w )		// В идексной таблице еще нет?
		{	// If its a new beacon, then reset our mode to Discovery mode
			// If we're in the correct group, send a a Broadcast Invitation Reply
			// Groups are powers of 2 - 1 as in ie 0,1,3,7,15,31,63,127 for the 8 possible choices
			// 0=General, 1=IPT, 2=ISPT, 3=HPT, 4=HSPT, 5=Vehicle, 6=Reserved, 7=Beacon Event Address,
			// 8=HPI1, 9=HPT2

            if( uhf.pkt.bi.cur_grp == ( mid.b[3] &( (1 << uhf.pkt.bi.no_grp_u3)-1)) )
			{
				// Запомнили в массиве полный Beacon ID, поскольку при подтверждении адрес не передается ;(
				bi[uhf.pkt.bi.bid.ho.lo & BB_LIST_MASK].w = uhf.pkt.bi.bid.w;

				switch( uhf.pkt.bi.ext )  
				{
//#if( HPT3_DISABLE_CALL || DEBUG_BLINK )
//#else
				case 1:
					// Общий вызов
					if( lamp_state <= LAMP_NORMAL_STATE )
					{   lamp_state = LAMP_PAGER_CALL_1;
        			    lamp_timeout = get_timeout( LAMP_CALL_STATE_TIMEOUT );
						mu_flags |= FL_ACK;
					}
					break;

				case 2:
					// Общая Эвакуация
					if( lamp_state >= LAMP_PAGER_ALARM )
					{   lamp_state = LAMP_PAGER_ALARM;
					    lamp_timeout = get_timeout( LAMP_ALARM_UHF_TIMEOUT );
						mu_flags |= FL_ACK;
					}
					break;

				case 3:
					// Просто активизация перерегистрации
					mu_flags |= FL_ACK;
					break;
//#endif
				}

				// Backoff for a random period specified by the message, then transmit it
				// The random time-slot is based on the no_slt variable
				// which is simply a power of 2 -1 mask, ie 0,1,3,7,15,31,63,127 for the 8 possible choices
				// anded with a random number from 0 to 127
				delay_uhf_ts( (BYTE)( rand() &( (1 << uhf.pkt.bi.no_slt ) - 1 ) ) );
				uhf.pkt.bir.mid = mid;					// Insert our ID into the response

				uhf.pkt.bir.cmd.b = (UHF_BI_PKT<<3);  	// Заново сформировали заголовок
				if( mu_flags & FL_ACK )
					uhf.pkt.bir.cmd.b |= BIR_CMD_ACK;
#if( USE_LAMP_KEY )
				if( mu_flags & FL_ACK_MANUAL )
					uhf.pkt.bir.cmd.b |= BIR_CMD_ACK_MANUAL;
#endif
#if( USE_APS )
				if( mu_flags & FL_8KHZ_TRAP)
					uhf.pkt.bir.cmd.b |= BIR_CMD_TRAP_8KHZ;
#endif

				uhf_registration_timeout = get_timeout( REGISTRATION_TIMEOUT );
				mu_flags |= (FL_SEND|FL_REGISTRATION);
           } 
        }
		break;
	//-----------------------------------------------------------------------
	// Broadcast Invitation Reply Confirmation (BIRC)
	case UHF_BIRC_PKT:
		// This request is sent to a Transponder once the ILB receives a Broadcast Invitation Reply.
		// The message will tell that particular transponder to stop replying to Broadcast Invitations
		// from this same Beacon. To be able to do so, the Transponder will store the Beacon’s ID in its memory.
		// If this is for me then the Broadcast Invitation Reply was confirmed so change to POLL state
	    if( uhf.pkt.birc.mid.w == mid.w )
	    {	ushort bid;
			{
				// Запомнили все активные Beacon ID в массиве подтвержденных
				char i;
				for( i=0; i<BB_LIST_SIZE; i++ )
				{   bid_u bid_cur;
					bid_cur.w = bi[i].w;
					if( bid_cur.w )
						bb[i].w = bid = bid_cur.w;
				}
			}

			bid = NTOHS(bid) & 0x0FFF;

#if( DEBUG_LEVEL )
			TEST_LED_PCB ^= 1;
#endif


#if( TEST_BEACON_REGISTRATION )

 #if( TEST_BEACON_REGISTRATION == 2 )
 		{
 #else
		if( bid <= 8  )
		{
 #endif
			if( lamp_state == LAMP_NORMAL_STATE )
			{   lamp_state = __LAMP_PAGER_CALL_1;
               	lamp_timeout = get_timeout( LAMP_CHECK_STATE_TIMEOUT );
			}
		}
		else
#endif


//#if( HPT3_DISABLE_CALL )
//#else
        if( uhf.pkt.birc.ext & BIT0 )
 		{	
			// Индивидуальный вызов
			if( lamp_state == LAMP_NORMAL_STATE )
			{   lamp_state = LAMP_PAGER_CALL_1;
            	lamp_timeout = get_timeout( LAMP_CALL_STATE_TIMEOUT );
			}
		}
//#endif
#if( USE_APS )

		if( bid == combine_id )
		{
	        if( uhf.pkt.birc.ext & BIT2 )
	        {
			//комбайнер
	       		vrx.flags.aps.combiner = TRUE;
			}
			else
			{
	        	vrx.flags.aps.combiner = FALSE;
			}
		}
		
#endif

		if( ( bid <= 4 )||( ( bid >= 8 )&&( bid <= 12 ) ) )
			uhf_registration_timeout = get_timeout( REGISTRATION_TIMEOUT * 4 );
		else
			uhf_registration_timeout = get_timeout( REGISTRATION_TIMEOUT     );  	// Set a 3-minute reset counter for the bonding

		if(vrx.flags.aps.combiner == TRUE)
		{	
			if (bid == combine_id )
			{
					uhf_registration_timeout = get_timeout( COMBINE_REGISTRATION_TIMEOUT     );  	// Set a 15-sec reset counter for the bonding
			}
		}


		mu_flags |= FL_REGISTRATION;

#if( USE_LAMP_KEY )
		mu_flags &= (~(FL_ACK|FL_ACK_MANUAL|FL_8KHZ_TRAP) );
#else
		mu_flags &= (~(FL_ACK|FL_8KHZ_TRAP));
#endif
	  	}
      	break;

	//-----------------------------------------------------------------------
	case UHF_II_PKT:
		// If this is for me and I am already bonded to it then respond immediately with my ID

//LAMP_FET ^= 1;

//TEST_LED_PCB ^= 1;


    	if( uhf.pkt.ii.mid.w == mid.w )
    	{
    		uhf.pkt.bir.mid = mid;

			// Set a 3-minute reset counter for the bonding
			uhf_registration_timeout = get_timeout( REGISTRATION_TIMEOUT );
#if( DEBUG_II )
   			LAMP_FET ^= 1;
//			TEST_LED_PCB ^= 1;
			timer_ii = 0;
#endif


			switch( uhf.pkt.ii.ext )
			{
			case 1:
				mu_flags |= (FL_QUIET_PAGER|FL_SEND);
				break;

	  		case 5:

  				VLF_SendGeneric();
				mu_flags |= FL_SEND;
				break;

			// Более позднее добавление
			case 7:
				if( lamp_state == LAMP_NORMAL_STATE )
				{   lamp_state = LAMP_PAGER_CALL_1;
        	    	lamp_timeout = get_timeout( LAMP_CALL_STATE_TIMEOUT );
				}
				mu_flags |= FL_SEND;
				break;

			default:

				mu_flags |= FL_SEND;

			}
			uhf.pkt.bir.cmd.b = (UHF_II_PKT<<3);  	// Заново сформировали заголовок

      	}
      	break;

#if( USE_UHF_TO_VLF )
	//-----------------------------------------------------------------------
	case UHF_VLF_PKT:


		switch( uhf.pkt.vlf.vlf_cmd )
		{
		//----------------
		case VLF_MU_DETECT:
			// Only respond if the SU ID or sequence are different - i.e. not masked
			if( ( uhf.pkt.vlf.sid !=  vtx.sid )||( uhf.pkt.vlf.seq_dur != vtx.seq ) )
			{ 	// Since this is a new sequence or SU-ID, unmask ourselves if we were masked
				if( lamp_state == LAMP_SEARCH_IN_PROGRESS )
			  	{  	lamp_state = LAMP_NORMAL_STATE;

#if( USE_INVERT_LAMP )
					LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON; 	// On
#else
	      			LAMP_FET = LAMP_ON;	  									// On
#endif
			    }

				if( lamp_state != LAMP_VLF_BLOCKED_STATE )
				{	delay_ms( DETECT_RX_DELAY_MS * 2 );	// Delay to allow Y signal to come through in worst case of X
					VLF_ReplyToBroadcastRequest( SEARCH_ID_INTERVALS );
				}
			}
      		break;
		//----------------
		case VLF_MU_MASK:
			if(	( uhf.pkt.vlf.mid_s.val[2] == mid.b[1] )&&
		    	( uhf.pkt.vlf.mid_s.val[1] == mid.b[2] )&&
		    	( uhf.pkt.vlf.mid_s.val[0] == mid.b[3] ) ) 	// If it is for us
  		    {
		    	delay_ms( MASK_RX_DELAY_MS * 2 );			// Delay to allow Y & Z signal to come through in worst case

				if( uhf.pkt.vlf.sid == 0x00 )
				{   // Маскирование на 30 минут перенесено
					lamp_min_cnt = uhf.pkt.vlf.seq_dur;
					if( lamp_min_cnt )
					{   lamp_state = LAMP_VLF_BLOCKED_STATE;
	            		lamp_timeout = get_timeout( LAMP_60SEC_TIMEOUT );
					}
					else
					{	lamp_state = LAMP_NORMAL_STATE;
	#if( USE_INVERT_LAMP )
						LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON; 	// On
	#else
	      				LAMP_FET = LAMP_ON;	  										// On
	#endif
					}
				}
				else
				{	if( lamp_state != LAMP_SEARCH_IN_PROGRESS )
					{  	lamp_state = LAMP_SEARCH_IN_PROGRESS;
		        	    lamp_timeout = get_timeout( LAMP_SEARCH_STATE_TIMEOUT );
					}
					vtx.seq = uhf.pkt.vlf.seq_dur;
					vtx.sid = uhf.pkt.vlf.sid;
				}

				VLF_SendGeneric();
			}
    	    break;

#if( HPT4_MODE )
#else
		//----------------
	   	case VLF_MU_VLF_RX_CONTROL:
			if( uhf.pkt.vlf.seq_dur == 0x5A )
		   		VLF_StopReceiver();
			else
		   		VLF_StartReceiver();		// Return to VLF receive mode whereby we listen for incoming messages

//			// Передача пакета для тестовых целей
//			VLF_SendGeneric();

 #if( DEBUG_SU_UHF_RX == 1 )
 				LAMP_FET ^= 1;
 				TEST_LED_PCB ^= 1;
				timer_ii = 0;
 #endif
			break;
#endif

		//----------------
		case VLF_MU_QUICK_SEARCH_X:
   			delay_ms( QUICKSEARCH_RX_DELAY_MS );	// Delay to allow Y signal to come through

  		case VLF_MU_QUICK_SEARCH_Y:
   		    delay_ms( QUICKSEARCH_RX_DELAY_MS );	// Delay to allow Z signal to come through

    	case VLF_MU_QUICK_SEARCH_Z:    // 03

			if( lamp_state != LAMP_VLF_BLOCKED_STATE )
    	   		VLF_ReplyToBroadcastRequest( SEARCH_QUICK_INTERVALS );

		   	break;

		//----------------
   	  	case VLF_MU_LOCATE:           // 04
		  	if(
				( uhf.pkt.vlf.mid_s.val[2] == mid.b[1]) &&
			   	( uhf.pkt.vlf.mid_s.val[1] == mid.b[2]) &&
			   	( uhf.pkt.vlf.mid_s.val[0] == mid.b[3])  ) 	// If it is for us
		    {
				delay_ms( LOCATE_RX_DELAY_MS * 2 );	// Delay to allow Y signal to come through in worst case of X
				VLF_ReplyLocateRequest( uhf.pkt.vlf.seq_dur );

    	    }

    	  	break;

		//----------------
   	  	case VLF_MU_TEST:

#if( DEBUG_SU_UHF_RX == 1 )
 			LAMP_FET ^= 1;
 			TEST_LED_PCB ^= 1;
			timer_ii = 0;
#else
			VLF_SendGeneric();
#endif
			break;

/* Нельзя 08 - 3 бита всего :( Подменяем на расширение команды MASK
		//----------------
   	  	case VLF_MU_MASK_FULL: 		// 08

		  	if(
				( uhf.pkt.vlf.mid_s.val[2] == mid.b[1]) &&
			   	( uhf.pkt.vlf.mid_s.val[1] == mid.b[2]) &&
			   	( uhf.pkt.vlf.mid_s.val[0] == mid.b[3])  ) 	// If it is for us
  	    	{

	    		delay_ms( MASK_RX_DELAY_MS * 2 );	   		// Delay to allow Y & Z signal to come through in worst case

				lamp_min_cnt = uhf.pkt.vlf.seq_dur;
				if( lamp_min_cnt )
				{   lamp_state = LAMP_VLF_BLOCKED_STATE;
	            	lamp_timeout = get_timeout( LAMP_60SEC_TIMEOUT );
				}
				else
				{	lamp_state = LAMP_NORMAL_STATE;
#if( USE_INVERT_LAMP )
					LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON; 	// On
#else
	      			LAMP_FET = LAMP_ON;	  									// On
#endif
				}
				VLF_SendGeneric();
			}
			break;
*/
		}
		break;
#endif
		//-----------------------------------------------------------------------
	case UHF_CONTROL_PKT:

    	switch( uhf.pkt.cqc.type_ctl )
        {
		case UHFC_SEARCH:
			// When the host is searching for MU's, we reply to each unique search index a number of times
			// in a random time slot of one in 256. The intent of this is to give the host a chance to catch
			// our reply if the channel is busy.
			// If the beacon is different, then reset the counter for how many times we will reply to a search request
        	if( uhf_ctrl_sidx != uhf.pkt.csc.sidx.w )
			{  	uhf_ctrl_sidx = uhf.pkt.csc.sidx.w;
        	    uhf_ctrl_sctr = 0;
        	}
			// If we haven't sent the maximum relies(3 in this case) then send another reply but only if the request
			// is for our group - group is based on a mask in the packet which is applied to the low byte of our ID
        	if( ( uhf_ctrl_sctr < UHF_CONTROL_SEARCH_REPLIES )
//				&&
//        	    ( uhf.pkt.csc.grp_u8 ==
//        	                ( mid.b[3] &( (1 << uhf.pkt.csc.gmsk_u3) - 1 ) ))

																			)
			{	uhf_ctrl_sctr++;
				// Backoff for a random period, then transmit reply
				delay_uhf_ts( (BYTE)rand() );
				PrepareSearchQueryReply();
				mu_flags |= FL_SEND;
			}
        	break;

	//-----------------------------------------------------------------------
	// Respond immediately to query request if it is for me
	case UHFC_QUERY:

//uhf.pkt.cbmc.mid.b[0] = 0x03;	// Для программатора тип подменить на HPT

            if( uhf.pkt.cqc.mid.w == mid.w )
			{	PrepareSearchQueryReply();
				mu_flags |= FL_SEND;
            }
        	break;

#if( ALLOW_UHF_BOOTMODE  )
	case UHFC_BOOTMODE:

//uhf.pkt.cbmc.mid.b[0] = 0x03;	// Для программатора тип подменить на HPT

        	if( uhf.pkt.cbmc.mid.w == mid.w )
			{  	di();
                EEPROM_WRITE( EEPROM_SD_OFFSETOF( warm_boot ), 1 );
                
				SWDTEN = 1;		// WDTCON |= 0x1;
                for( ; ; );     // Wait for WATCHDOG TO TIMEOUT CAUSING A RESET
          	}
            break;
#endif

	//-----------------------------------------------------------------------
	case UHFC_TESTMODE:

		   if( uhf.pkt.ctst.mid.w == mid.w )
		   {
				switch( uhf.pkt.ctst.test_mode )
				{
				case 0x8000:
					if( lamp_state == LAMP_TEST_STATE )
						lamp_state = LAMP_NORMAL_STATE;
#if( USE_INVERT_LAMP )
					LAMP_FET =  ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON; 	// On
#else
	      			LAMP_FET = LAMP_ON;	  									// On
#endif
					mu_flags |= FL_SEND;
					break;

				case 0x8001:
					// Перевод в тестовый режим
					lamp_state = LAMP_TEST_STATE;
#if( USE_INVERT_LAMP )
					LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_ON : LAMP_OFF; // Off
#else
	      			LAMP_FET = LAMP_OFF;	  									// Off
#endif
					mu_flags |= FL_SEND;
					break;

#if( USE_INVERT_LAMP )
				case 0x8002:
					mu_flags ^= FL_LAMP_INV;
					// Write Signature TO EERROM
					eeprom_write( EEPROM_SD_OFFSETOF( lamp_mode ), ( mu_flags & FL_LAMP_INV ) ? EEPROM_LAMP_MODE_SIGN1 : EEPROM_LAMP_MODE_SIGN0 );
					LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON; 	// Lamp On
					mu_flags |= FL_SEND;
					break;
#endif
				}

				if( mu_flags & FL_SEND )
				{	uhf.pkt.cqc.type_ctl = UHFC_QUERY;	// Подмена команды
					PrepareSearchQueryReply();
				}
           }
		}
    	break;
	}

   	if( mu_flags & FL_SEND )
    {  	SendUHFPkt();
		mu_flags &= (~FL_SEND);
	}
   	else
	{  	CREN = 1;			// Enable the UHF UART RX
      	RCIE = 1;			// Enable interrupt on UHF RX
   	}

//TEST_LED_PCB = TEST_LEDX_OFF;


	return( RESULT_OK );
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
static void SendUHFPkt(void)
{
#if( DEBUG_LEVEL )
	TEST_LED1_TX = 0; 				// On
#endif

	// Reset TRX state machine to preamble
	uhf.rxt_state = UHF_RXTX_PREAMBLE;
	uhf.idx = 0;


	if( SetUHFMode( UHF_TX_MODE ) )	// Set the UHF transceiver to TX mode
	{
		lamp_state 	 = LAMP_POST_ERROR;
		lamp_timeout = get_timeout( 32000 );

	}

	uhf.pkt.data[0] = UHF_STX_CHAR;
	uhf.pkt.data[1] &= (~CMD_MS);

	CalcUHFChecksum( 1 );		// Calculate and Set checksum of the packet

	TXEN = 1;
	TXIE = 1;					// Enable UHF UART TX interrupt
}


#if( MU_433_TX_TESTING  )
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void Test433tx(void)
{
	// Reset TRX state machine to preamble
	uhf.rxt_state = UHF_RXTX_PREAMBLE;
	uhf.idx = 0;

	SetUHFMode( UHF_TX_MODE );	// Set the UHF transceiver to TX mode

//	TXEN = 1;
//	TXIE = 1;					// Enable UHF UART TX interrupt

	for( ; ; )
	{	uhf.rxt_state = UHF_RXTX_PREAMBLE;
		uhf.idx = 0;

		CLRWDT();

	}
}
#endif



