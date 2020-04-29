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
 * \file mobility.h
 * \brief Header file for mobility functionality.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
 */

#ifndef MOBILITY_H
#define MOBILITY_H

#include <htc.h>
#include "types.h"
#include "time.h"


#define  MOBILITY_MEASUREMENTS_PER_SECOND 	 3
//#define  MOBILITY_5MIN_OF_MEASUREMENTS 		 (5*60*MOBILITY_MEASUREMENTS_PER_SECOND)
#define  MOBILITY_30SEC_OF_MEASUREMENTS 	 ( 30*MOBILITY_MEASUREMENTS_PER_SECOND )
//#define  MOBILITY_10SEC_OF_MEASUREMENTS 	 (10*MOBILITY_MEASUREMENTS_PER_SECOND)
#define  MOBILITY_MEASURE_PERIOD             (TICKS_PER_SEC/MOBILITY_MEASUREMENTS_PER_SECOND)    // The time between alive sensor measurements

#define  MOBILITY_WAKE_UP_DELAY_BEFORE_MEASUREMENTS (TICKS_PER_SEC*5)
#define  ACCELEROMETER_NOISE_FLOOR  0x10	// Noise floor based on measurments on one of the units
#define  MOBILITY_HISTORY_SIZE 		6



void MeasureMobility( void );
BYTE   GetMobilityValue( void );
void MobilityTickHandler( void );

#endif
