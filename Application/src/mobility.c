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


#include <htc.h>
#include "stdlib.h"
//#include "math.h"
#include "types.h"
#include "mu.h"
#include "mobility.h"
#include "vlf_comms.h"
#include "uhf_comms.h"
#include "init30a.h"


/*
BYTE bank2            		m_history_au8[ MOBILITY_HISTORY_SIZE ] = {0, 0, 0, 0, 0, 0};
static bank2 ushort         mobility_cnt_u16 = 0;
bank2 BYTE min_mobility;
bank2 BYTE max_mobility;
*/


#if( USE_MOBILITY )

static BYTE     mob_history[MOBILITY_HISTORY_SIZE] = { 0, 0, 0, 0, 0, 0 };
static BYTE   	mob_cnt = 0;
static BYTE 	mob_min;
static BYTE 	mob_max;


//---------------------------------------------------------------------------
// Performs a mobility measurement
// Note: this function should not be called when busy decoding, transmitting or
// communicating on the UHF
//---------------------------------------------------------------------------
void MeasureMobility(void)
{
BYTE diff = 0;
BYTE value;
BYTE count;

//	TESTPT30 = 1;		// signal that we are measuring the mobility

	value = ReadADChannel( MOBILITY_AD_CHANNEL );			// Read the accelerometer

	if( mob_cnt == 0 )
	{  	mob_min = mob_max = value;						    // The first time, set a reference
        diff = 0;
	}
	else
	{  	mob_min = min( mob_min, value );  	// Keep a minhold
		mob_max = max( mob_max, value );  	// Keep a maxhold
     	diff = mob_max - mob_min;
	}

	if( diff > ACCELEROMETER_NOISE_FLOOR )
	{   diff -= ACCELEROMETER_NOISE_FLOOR;				// Subtract the noise floor from the signal
		for( count = 0; count < MOBILITY_HISTORY_SIZE; count++ )
		{	if( diff > mob_history[count] )
				mob_history[count] = diff;
	    }
	}

    mob_cnt++;

    // Every period, roll the data through a history buffer
    if( mob_cnt >= MOBILITY_30SEC_OF_MEASUREMENTS )
   	{   // Roll the mobility history
    	mob_history[5] = mob_history[4];
       	mob_history[4] = mob_history[3];
       	mob_history[3] = mob_history[2];
       	mob_history[2] = mob_history[1];
       	mob_history[1] = mob_history[0];
       	mob_history[0] = 0;
       	mob_cnt = 0;
	}

//	mobility_countdown_ticks_u16 = MOBILITY_MEASURE_PERIOD;	// reset the timer to take the next measurement in about 330 ms

//	TESTPT30 = 0;		// signal that mobility measurement is finished

}


//---------------------------------------------------------------------------
// Calulate the current mobility value using an exponentially weighted average of the mobility
// history giving the most recent the highest weight. Scale it to 1 to 15
//---------------------------------------------------------------------------
BYTE GetMobilityValue( void )
{
     BYTE mob_val = 	(mob_history[0] >> 1) +
                   		(mob_history[1] >> 2) +
                   		(mob_history[2] >> 3) +
                   		(mob_history[3] >> 4) +
                   		(mob_history[4] >> 5) +
                   		(mob_history[5] >> 6);

    return( min( (mob_val>>2 ), 15 ) );
}


//---------------------------------------------------------------------------
// Low priority Mobility Routine that is called whenever a system tick occurs.
//---------------------------------------------------------------------------
static BYTE mobility_countdown_ticks = MOBILITY_WAKE_UP_DELAY_BEFORE_MEASUREMENTS; // schedule first mobility measurement


void MobilityTickHandler( void )
{
   	if( mobility_countdown_ticks )
		mobility_countdown_ticks--;
	else if( ( !VLF_Busy() )&&( !UHFBusy() ) )
   	{  	MeasureMobility();
		mobility_countdown_ticks = MOBILITY_MEASURE_PERIOD;	// Reset the timer to take the next measurement in about 330 ms
   	}
}

#endif
