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
 * \file uhf_comms.h
 * \brief Header file for high frequency communicatons functionality.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
 */

#ifndef __UHF_COMMS_H
#define __UHF_COMMS_H


#include <htc.h>
#include "types.h"
#include "time.h"
#include "uhf_pkt.h"


// Timeouts
//#define  UHF_RX_TIMEOUT_PERIOD  		(TICKS_PER_SEC*10)  // The UHF RX timeout period before looking for another beacon
//#define  UHF_RX_TIMEOUT_MULTIPLIER      18                 	// The multiplier for the timeout period


#if( DEBUG_LEVEL >= 2 )

 #define REGISTRATION_TIMEOUT    		(  10*(TICKS_PER_SEC))

#else

 #if( DEBUG_TST_TIMEOUT || DEBUG_BLINK )
  #define REGISTRATION_TIMEOUT    		(  5*(TICKS_PER_SEC))
 #else
  #if( IVT_SUBMODE )
   #define REGISTRATION_TIMEOUT    		(  62*(TICKS_PER_SEC))
  #else

   #define REGISTRATION_TIMEOUT    		(3*60*(TICKS_PER_SEC))

//   #define REGISTRATION_TIMEOUT    		(1*60*(TICKS_PER_SEC))

  #endif
 #endif

#endif

#define COMBINE_REGISTRATION_TIMEOUT 	( 15 * (TICKS_PER_SEC))


#define UHF_RESET_TIMEOUT  				(2*60*(TICKS_PER_SEC))

extern ushort  uhf_registration_timeout;
extern short_u uhf_bonded_bid;
extern short_u uhf_last_bid;

extern bank2 Uhf_pkt_s 	uhf;

BYTE ProcessUHFPacket(void);

BYTE UHFBusy(void);
//extern void TestUHFTx();

#define BB_LIST_SIZE 	(8)
#define BB_LIST_MASK 	( (BB_LIST_SIZE) - 1 )

extern bank3 bid_u	bb[BB_LIST_SIZE];		// Bounded beacons ID list
extern bank3 bid_u	bi[BB_LIST_SIZE];		//

#endif
