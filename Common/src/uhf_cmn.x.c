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

/**
 * \file uhf_cmn.c
 * \brief Source file for common UHF communicatons functionality.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/03/2008

	09.01.2011 Igors Zalts

 */


#include <htc.h>
#include "mu.h"
#include "types.h"
#include "uhf_pkt.h"
#include "uhf_cmn.h"
#include "cc1000.h"
#include "uhf_comms.h"
#include "init30a.h"


//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
// Lookup tables for standard packet size, indexed by pkt_type & ms_u8
static const short_u pkt_len_au[] = {
   { 	sizeof(UHF_BIR_s), 	sizeof(UHF_BI_s)   },
   { 	0xFF,              	sizeof(UHF_BIRC_s) },
   { 	sizeof(UHF_BIR_s), 	sizeof(UHF_II_s)   },
#if( USE_UHF_TO_VLF )
   { 	0xFF, 				sizeof(UHF_VLF_s)  }
#endif
};

// Lookup tables for control packet size, indexed by ctype & ms_u8
static const short_u pkt_ctrl_len_au[] = {
   { sizeof(UhfCtrlSearchQueryRTSt),	sizeof(UhfCtrlSearchCTSt) 		},
   { sizeof(UhfCtrlSearchQueryRTSt),	sizeof(UhfCtrlQueryCTSt) 		},
   { 0xFF, 						  		sizeof(UhfCtrlBootModeCTSt) 	},
   { 0xFF, 						  		sizeof(UhfCtrlDLInitTSt) 		},
   { 0xFF, 						  		sizeof(UhfCtrlDLDataTSt) 		},
   { sizeof(UhfCtrlDLEndRTSt), 			sizeof(UhfCtrlDLEndCTSt) 		},
   { sizeof(UhfCtrlSetIdRTSt), 			sizeof(UhfCtrlSetIdCTSt) 		}
};

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void UHFCommsInit(void)
{
	// Disable receiver before initialization in case this function is
	// called from any place other than a reset (which it is)
	CREN = 0;	// Disable UART RECEIVER
	RCIE = 0;	// Disable UART RX INTERRUPT
	TXEN = 0;	// Disable UHF UART TX
    TXIE = 0;	// Disable UHF UART TX interrupt

	uhf.rxt_state = UHF_RXTX_PREAMBLE;
	uhf.idx = 0;

  	uhf_registration_timeout = get_timeout( REGISTRATION_TIMEOUT );

	InitUHFTranceiver();

	// Initialise UHF interface for RX mode
	SetUHFMode( UHF_RX_MODE ); 	// Turn on receive mode
	CREN = 1;					// Enable the UHF UART RX
	RCIE = 1;					// Enable interrupt on UHF rx
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
BYTE CalcUHFChecksum( uchar set_flag )
{
ushort crc = UHF_CRC_INIT;
BYTE num = get_pkt_len();
BYTE ii, jj;

   if( num == 0xFF )
      return( 0xFF );
   else
      num -= 2;

	// Using the CRC generation algorithm takes longer than the lookup table, but saves us a lot of code space
   	for( ii = 0; ii < num; ii++ )
	{  	crc ^= uhf.pkt.data[ii];
      	for( jj = 0; jj < 8; jj++ )
		{  	if( crc & 1 )
            	crc = (crc >> 1) ^ UHF_CRC_POLYNOMIAL;
         	else
            	crc >>= 1;
      	}
   	}

   	if( set_flag )
	{   uhf.pkt.data[num  ] = (BYTE)(crc >> 8 );
  		uhf.pkt.data[num+1] = (BYTE)(crc      );
		return( RESULT_OK );
	}
	else
	{ 	if( crc == ((ushort)(uhf.pkt.data[num  ] << 8) | (ushort)(uhf.pkt.data[num+1])) )
			return( RESULT_OK );
	}
	return( 0xFF );
}


//---------------------------------------------------------------------------
// Gets packet length according to type of packet
//---------------------------------------------------------------------------
BYTE get_pkt_len(void)
{



   	if( uhf.pkt.bi.type == UHF_CONTROL_PKT )
	{


//LAMP_FETn = 1;


		if( uhf.pkt.cqc.ctype < (sizeof (pkt_ctrl_len_au)/2) )
         	return( pkt_ctrl_len_au[uhf.pkt.cqc.type ].b[uhf.pkt.cqc.ms] );
		return( 0xFF );

  	}

//LAMP_FETn = 0;


   	if( uhf.pkt.bi.type < (sizeof (pkt_len_au)/2) )
      	return( pkt_len_au[uhf.pkt.bi.type].b[uhf.pkt.bi.ms] );

/*
   	else if( uhf.pkt.bi.type == UHF_CONTROL_PKT )
	{



		if( uhf.pkt.cqc.ctype < (sizeof (pkt_ctrl_len_au)/2) )
         	return( pkt_ctrl_len_au[uhf.pkt.cqc.type ].b[uhf.pkt.cqc.ms] );
  	}
*/

	return( 0xFF );
}
