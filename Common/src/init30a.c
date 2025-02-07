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
 * \file init30a.c
 * \brief Common board related functions for Rev30A
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/03/2008
 */


#include <htc.h>
#include "types.h"
#include "wait.h"
#include "init30a.h"
#include "mu.h"

/*
__CONFIG( XT & WDTDIS & PWRTEN & MCLRDIS & UNPROTECT & BORDIS & IESODIS & FCMDIS & LVPDIS & DEBUGDIS & BORV21);
*/

// Note: For devices that have more than one configuration word location, each subsequent
// invocation of __CONFIG() will modify the next configuration word in sequence.

__CONFIG( XT & WDTDIS & PWRTEN & MCLRDIS & UNPROTECT &          IESODIS & FCMDIS & LVPDIS & DEBUGDIS  );
__CONFIG( BORV21 );


//---------------------------------------------------------------------------
// Initialises the MU REV30A board.
// Performs initialisations of the registers, ports & peripherals.
//---------------------------------------------------------------------------
void init_board( void )
{
/*
volatile ulong ii;

   	for( ii = 0; ii < 100000; ii++ );	// Delay? before? initializing?
*/

  	OPTION = 0x60;	// 01100000 PORTB pull-ups are Enabled,
   					// Transition on low-to-high transition ON T0CKI pin
   					// Interrupt on rising edge of INT pin
   					// Prescaler is assigned to the Timer0 module
   					// Prescaler is 1:1

	WPUB  = (BIT2|BIT5); // Pager + Iridium   Pull-Up enabled


#if( DEBUG_LEVEL > 1 )
   	PORTB = BIT2;
   	TRISB = 0x1A;	// 0001 1010  B4,B3,B1 are inputs - the rest are outputs
#else

   	PORTB = (BIT2|BIT5);

 #if( HPT3_MODE )
   	TRISB = 0x3A;	// 0001 1010  B5,B4,B3,B1 are inputs - the rest are ts outputs
 #elif( HPT4_MODE )
   	TRISB = 0x1A;	// 0001 1010  B4,B3,B1 are inputs - the rest are ts outputs
 #else
   	TRISB = 0x1E;	// 0001 1110  B4,B3,B2,B1 are inputs - the rest are outputs
 #endif

#endif

  	PORTA = 0x08;
   	TRISA = 0x03;	// 0000 0011  A1,A0 are inputs - the rest are outputs


   	PORTC  = 0x41;  // Turn the Camp lamp on by default

	TRISC  = 0x90;		// 1001 0000 C7 and C4 are inputs the reset are outputs

#if( HPT4_MODE )
   	ANSEL  = 0x00;
   	ANSELH = 0x0A;
#else
   	ANSEL  = 0x03;
   	ANSELH = 0x0A;
#endif

	// Setup A/D module
   	ADCON0 = 0x81;     	// Tad of 4us, ON
   	ADCON1 = 0x00;     	// Left justified, full VCC range

	// Timer1 config   01 = 1:2 Prescale Value
	T1CON = 0x11;      	// Timer is started, 1 ticks/us

	// Comparator config
   	VRCON 	 = 0xE8;//0xE7 = 0.875 V 0xE8 1.0V, low range 0xE9;   	// Ref on, routed to C1, at 1.125V, Low range
   	CM2CON1 |= 0x20;
   	CM1CON0  = 0xA5;   	// Comparator 1 on, OE, connected to CVref


	// SSP port config
   	SSPSTAT = 0x40;    	// Tx data on rising edge, read on falling edge

//   	SSPCON  = 0x30;    	// SSP Master is on and SCK defaults high
   	SSPCON  = 0x31;    	// SSP Master is on and SCK defaults high  FOSC/16
//   	SSPCON  = 0x32;    	// SSP Master is on and SCK defaults high  FOSC/64

	// UART config
   	SPBRG = 12;   		// 38400 baud
   	TXSTA = 0x04;      	// TX disable, Async, 8 data bits, high baud
	RCSTA = 0x80;      	// RX off, SP enabled



}

//---------------------------------------------------------------------------
// Reads a channel of the A/D.
// Performs a read of the specified A/D channel.
// @param channel_u8 The channel to read
// @return The value read
//---------------------------------------------------------------------------
BYTE ReadADChannel( BYTE ch )
{
volatile BYTE ii;
	ADCON0 = 0x00;				// Reset the AD while we modify the registers
   	ADCON1 = 0x00;             	// ADFM = 0 for Left justified, full VCC range
   	ADCON0 = 0x81 | ch;  		// Tad of 4us, and turn on the A/D
   	for( ii = 0; ii < 10; ++ii )
      ;              			// Delay for at least 10us to setup A/D
   	GODONE = 1;					// Start conversion
   	NOP();
   	while( GODONE );  			// Wait for conversion to be done
   	return( ADRESH );  			// Being left justified, we only need to read the high register to get the top 8 bits of the 10 bit value
}

