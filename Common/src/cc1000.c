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
 * \file CC1000.c
 * \brief Source file for CC1000 communications transceiver driver.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
 */

#include <htc.h>
#include "types.h"
#include "mu.h"
#include "cc1000.h"
#include "init30a.h"
#include "wait.h"


static const struct {
   BYTE main_u8;
   BYTE current_u8;
   BYTE pll_u8;
   BYTE power_u8;
} c_uhf_modes_ast[] = {
// RX mode settings
#if( DEBUG_SU_UHF_RX )
   	{ 0x11, 0x44, 0x60, 0 },
#else
   	{ 0x11, 0x44, 0x60, 0 },    		// RX mode settings  (Changed current from 0x40 to 0x44 - Datasheet & Smart RF Studio recommended
#endif
// TX mode settings
#if( MU_433_TX_TESTING  )
   	{ 0xE1, 0x81, 0x48, 0xC0}//0x01 }    		// TX mode settings  -20dB
#elif( HPT4_MODE )
	{ 0xE1, 0x81, 0x48, 0x80 }          // TX mode settings  +6dB
#else
   	{ 0xE1, 0x81, 0x48, 0xC0}//0xCO 8 dB    0x0F } 	// TX mode settings  +0dB
#endif
};


//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void cc1000_wr( BYTE addr_u8, BYTE val_u8 )
{
	addr_u8 <<= 1;
   	addr_u8 |= 1;

   	di();		// Disable interrupts during write to CC1000
   	UHF_PALE = 0;
   	SSPBUF = addr_u8;
   	while ( !BF );
   	UHF_PALE = 1;

   	SSPBUF = val_u8;
   	while ( !BF );
   	ei();
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
BYTE cc1000_rd( BYTE addr_u8 )
{
	addr_u8 <<= 1;

   	di();
   	UHF_PALE = 0;
   	SSPBUF = addr_u8;
   	while ( !BF );

	// Tristate SDO to receive incoming data
   	TRISC |= 0x20;
   	UHF_PALE = 1;

   	SSPBUF = 0x00;
   	while ( !BF );
   	TRISC &= ~0x20;

   	addr_u8 = SSPBUF;
   	ei();

   	return( addr_u8 );
}

#if( HPT3_MODE )
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void pager_write( BYTE value )
{
   	di();		// Disable interrupts during write to CC1000
   	SSP_PAGER_CS = 0;
	NOP();
   	SSPBUF = value;
   	while ( !BF );
	NOP();
   	SSP_PAGER_CS = 1;
   	ei();
}
#endif



//---------------------------------------------------------------------------
// Initialises the CC1000 communications transceiver.
// This initialisation is performed in accordance with that observed for that of
// the MRS INsite transponder
//---------------------------------------------------------------------------
BYTE InitUHFTranceiver( void )
{
// 	0x3A =  RXTX = 0, F_REG = 0, RX_PD = 1, TX_PD = 1, FS_PD = 1, CORE_PD = 0, BIAS_PD = 1, RESET_N = 0
   	cc1000_wr( CC1000_MAIN, 0x3A );   // Reset & set OSC
//  WaitMs( 10 );   // ???? This time is not in the flowchart on page 29
   	delay_ms( 10 );   // ???? This time is not in the flowchart on page 29


// 	0x3B =  RXTX = 0, F_REG = 0, RX_PD = 1, TX_PD = 1, FS_PD = 1, CORE_PD = 0, BIAS_PD = 1, RESET_N = 1
   	cc1000_wr( CC1000_MAIN, 0x3B );   // RESET_N = 1
//  WaitMs( 10 );	// 2ms - Time to wait depends on the crystal frequency and the load capacitance
   	delay_ms( 10 );	// 2ms - Time to wait depends on the crystal frequency and the load capacitance


	// Program all registers except MAIN. Values used by MRS INsite transponder
   	cc1000_wr( CC1000_FREQ_2A, 0x58 );	// FREQ_A[23:16]
   	cc1000_wr( CC1000_FREQ_1A, 0x20 );	// FREQ_A[15:8]
   	cc1000_wr( CC1000_FREQ_0A, 0    );	// FREQ_A[7:0]


   	if( cc1000_rd( CC1000_FREQ_2A ) !=  0x58 )
		return( 1 );
   	if( cc1000_rd( CC1000_FREQ_1A ) != 0x20  )
		return( 1 );
   	if( cc1000_rd( CC1000_FREQ_0A ) != 0     )
		return( 1 );


   	cc1000_wr( CC1000_FREQ_2B, 0x42 );
   	cc1000_wr( CC1000_FREQ_1B, 0x14 );
   	cc1000_wr( CC1000_FREQ_0B, 0x9c );


   	if( cc1000_rd( CC1000_FREQ_2B ) != 0x42 )
		return( 1 );
   	if( cc1000_rd( CC1000_FREQ_1B ) != 0x14 )
		return( 1 );
   	if( cc1000_rd( CC1000_FREQ_0B ) != 0x9c )
		return( 1 );


//#if( DEBUG_SU_UHF_RX )
//   	cc1000_wr( CC1000_FSEP1,   0x05 );		// 3 MSB of frequency separation control   128K
//   	cc1000_wr( CC1000_FSEP0,   0x00 );		// 8 LSB of frequency separation control
//#else
   	cc1000_wr( CC1000_FSEP1,   0x02 );		// 3 MSB of frequency separation control   64K
   	cc1000_wr( CC1000_FSEP0,   0x80 );		// 8 LSB of frequency separation control
//#endif


//#if( DEBUG_SU_UHF_RX )
//   	cc1000_wr( CC1000_FRONT_END, 0x12 );  //(; LNA-1.4mA, RSSI-active) to match the values given by Smart RF Studio
//#else
   	cc1000_wr( CC1000_FRONT_END, 0x12 );  //(; LNA-1.4mA, RSSI-active) to match the values given by Smart RF Studio
//#endif

   	cc1000_wr( CC1000_PA_POW,  0 );

   	cc1000_wr( CC1000_PLL, c_uhf_modes_ast[ UHF_RX_MODE ].pll_u8 );
//#if( DEBUG_SU_UHF_RX )

//#if( HPT4_MODE && MU_433_TX_TESTING	)
//  	cc1000_wr( CC1000_LOCK,    0x18 ); 	// 00010000 : LOCK_CONTINUOUS (active high) + PLL Wide
//#else
  	cc1000_wr( CC1000_LOCK,    0x10 ); 	// 00010000 : LOCK_CONTINUOUS (active high)      // Changed from 0xD0
//#endif

   	cc1000_wr( CC1000_CAL,     0x26 );	// Calibration-inactive,

//#if( DEBUG_SU_UHF_RX )
//   	cc1000_wr( CC1000_MODEM2,  0xA0 );	// Peak detector and remover is enabled    128KHz
//   	cc1000_wr( CC1000_MODEM1,  0x6B );	// Average filter is free-running,Lock AF is controlled by LOCK_AVG_IN, 22bit
//   	cc1000_wr( CC1000_MODEM0,  0x5B );	//??? this is 5B instead of 59??? - ;59 = Baudrate-38.4kBaud, XTAL-14.7456MHz
//#else
   	cc1000_wr( CC1000_MODEM2,  0x9C );	// Peak detector and remover is enabled
   	cc1000_wr( CC1000_MODEM1,  0x6B );	// Average filter is free-running,Lock AF is controlled by LOCK_AVG_IN, 22bit
   	cc1000_wr( CC1000_MODEM0,  0x5B );	//??? this is 5B instead of 59??? - ;59 = Baudrate-38.4kBaud, XTAL-14.7456MHz
//#endif
   	cc1000_wr( CC1000_MATCH,   0x70 );	// Match capacitor: RX - 2.8pF,TX - 0pF
   	cc1000_wr( CC1000_FSCTRL,  0x01 );	// Enable data shaping
   	cc1000_wr( CC1000_PRESCALER, 0 );		// Nominal setting
   	cc1000_wr( CC1000_TEST6,   0x10 );
   	cc1000_wr( CC1000_TEST5,   0x08 );
   	cc1000_wr( CC1000_TEST4,   0x3F );	// Constant for 38.4 and 76.8 kBaud
   	cc1000_wr( CC1000_TEST3,   0x04 );
   	cc1000_wr( CC1000_TEST2,   0    );
   	cc1000_wr( CC1000_TEST1,   0    );
   	cc1000_wr( CC1000_TEST0,   0    );

	// Calibrate VCO and PLL
	// Rx calibration routine - update current and PLL for RX mode
   	cc1000_wr( CC1000_MAIN,    c_uhf_modes_ast[ UHF_RX_MODE ].main_u8 );

   	cc1000_wr( CC1000_CURRENT, c_uhf_modes_ast[ UHF_RX_MODE ].current_u8 );

   	cc1000_wr( CC1000_PLL    , c_uhf_modes_ast[ UHF_RX_MODE ].pll_u8 );
	cc1000_wr( CC1000_CAL, 0xA6 );	// Turn on the CAL_START bit to start calibration
 	while ( (cc1000_rd( CC1000_CAL ) & 0x08 ) == 0 );

//  delay_ms( 34 );		// Calibration can take up to 34 ms at slowest 1 MHz PLL freq.
// 	delay_ms( 68 );		// Calibration can take up to 34 ms at slowest 1 MHz PLL freq.

   	cc1000_wr( CC1000_CAL, 0x26 ); // The CAL_START bit must be set to 0 by the microcontroller after the calibration is done.

	// Tx calibration routine - update current and PLL for TX mode
   	cc1000_wr( CC1000_MAIN, c_uhf_modes_ast[ UHF_TX_MODE ].main_u8 );
   	cc1000_wr( CC1000_CURRENT, c_uhf_modes_ast[ UHF_TX_MODE ].current_u8 );
   	cc1000_wr( CC1000_PLL    , c_uhf_modes_ast[ UHF_TX_MODE ].pll_u8 );
   	cc1000_wr( CC1000_CAL, 0xA6 );  // Turn on the CAL_START bit to start calibration
 	while ( ( cc1000_rd( CC1000_CAL ) & 0x08 ) == 0 );
//  delay_ms( 34 );		// Calibration can take up to 34 ms at slowest 1 MHz PLL freq.
// 	delay_ms( 68 );		// Calibration can take up to 34 ms at slowest 1 MHz PLL freq.
   	cc1000_wr( CC1000_CAL, 0x26 );	// The CAL_START bit must be set to 0 by the microcontroller after the calibration is done.

	// Power down CC1000
   	// 0x3F =  RXTX = 0, F_REG = 0, RX_PD = 1, TX_PD = 1, FS_PD = 1, CORE_PD = 1, BIAS_PD = 1, RESET_N = 1
   	cc1000_wr( CC1000_MAIN, 0x3F );	// ;FULL POWER DOWN mode, FS, XOSC, BIAS


   	cc1000_wr( CC1000_PA_POW, 0x55 );
	if( cc1000_rd( CC1000_PA_POW ) !=  0x55 )
		return( 1 );

   	cc1000_wr( CC1000_PA_POW, 0xAA );
	if( cc1000_rd( CC1000_PA_POW ) !=  0xAA )
		return( 2 );

   	cc1000_wr( CC1000_PA_POW, 0 ); // PA_POW should be set to 00h before power down mode to ensure lowest possible leakage current.

	// Turn on OSC core
   	// 0x7B =  RXTX = 0, F_REG = 1, RX_PD = 1, TX_PD = 1, FS_PD = 1, CORE_PD = 0, BIAS_PD = 1, RESET_N = 1
   	cc1000_wr( CC1000_MAIN, 0x7B );	// CORE_PD = 0
  	delay_ms( 2 );		// Wait 2ms - Time to wait depends on the crystal frequency and the load capacitance
   	cc1000_wr( CC1000_MAIN, 0x79 );	// BIAS_PD = 0
   	Wait250Us();		//WaitUs( 200 );		// wait 200 usec ACCORDING TO data sheet

	return( 0 );

}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
static BYTE uhf_mode_u8 = UHF_RX_MODE;
BYTE GetUHFMode(void)
{
	return( uhf_mode_u8 );
};

//---------------------------------------------------------------------------
// Sets the mode of the CC1000 communications transceiver to be in RX or TX mode.
// mode_u8 specifies the rx or tx mode of the transceiver
// This function assumes the oscillator is always turned on and we are simply switching between RX and TX modes
// This is different from the data sheet which powers down the CC1000 at the end of an RX or TX session
//---------------------------------------------------------------------------
BYTE SetUHFMode( BYTE mode_u8 )
{
	CLRWDT();
	// The order here lines up more with the CC1000 datasheet flowcharts
    uhf_mode_u8 = mode_u8;		// store the current mode

	//  Always set the power to 0 before configuring for RX or TX
//  cc1000_wr( CC1000_PA_POW , 0x00 );	// turn off power


   	// Setup the RX or TX control bits, current and PLL setting
	cc1000_wr( CC1000_MAIN,    c_uhf_modes_ast[ mode_u8 ].main_u8 );
   	cc1000_wr( CC1000_CURRENT, c_uhf_modes_ast[ mode_u8 ].current_u8 );
   	cc1000_wr( CC1000_PLL    , c_uhf_modes_ast[ mode_u8 ].pll_u8 );
	if( mode_u8 == UHF_RX_MODE )
	{
    	cc1000_wr( CC1000_PA_POW , 0x00 );	// Turn off power
 	}
  	Wait250Us();
	// Set TX OUTPUT POWER
   	if( mode_u8 == UHF_TX_MODE )
	{
    	cc1000_wr( CC1000_PA_POW , c_uhf_modes_ast[mode_u8].power_u8 );
	   	Wait25Us();		// WaitUs( 20 );
   	}

/*
   	delay_ms( 34 );		// Calibration can take up to 34 ms at slowest 1 MHz PLL freq.

   	// Check to see if the PLL is still locked - if not then we might have to recalibrate - FOR NOW SET A FLAG THAT IT HAPPENED
//#if( ENABLE_DEBUG_FLAGS )
	if( ( cc1000_rd( CC1000_LOCK ) & 0x01 ) == 0 )
	{
//		dflags.f.b.cc1000_lockfail = 1;		// Might have to recalibrate
		return( 1 );
	}
//#endif
*/


	return( 0 );

}


//---------------------------------------------------------------------------
// Locks the symbol threshold of the CC1000 communications transceiver. Called from interrupt,
// so cc1000_wr inlined here.
//---------------------------------------------------------------------------
void LockUHFThresh(void)
{
	UHF_PALE = 0;
   	SSPBUF = (CC1000_MODEM1 << 1) | 0x01;
   	while( !BF );
   	UHF_PALE = 1;
   	SSPBUF =  0x7B;		// Average filter is locked, LOCK_AVG_IN = 1
   	while( !BF );
}


//---------------------------------------------------------------------------
// Unlocks the symbol threshold of the CC1000 communications transceiver. Called from interrupt,
// so cc1000_wr inlined here.
//---------------------------------------------------------------------------
void UnlockUHFThresh(void)
{
	UHF_PALE = 0;
   	SSPBUF = (CC1000_MODEM1 << 1) | 0x01;
   	while( !BF );
   	UHF_PALE = 1;

   	SSPBUF = 0x6B;		// Average filter is free-running,LOCK_AVG_IN = 0
   	while( !BF );
}
