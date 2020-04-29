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
 * \file CC1000.h
 * \brief Header file for CC1000 communications transceiver.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
 */

#ifndef __CC1000_H
#define __CC1000_H


#include <htc.h>
#include "types.h"

#define CC1000_MAIN               0    // MAIN Register
#define CC1000_FREQ_2A            0x01 // Frequency Register 2A
#define CC1000_FREQ_1A            0x02 // Frequency Register 1A
#define CC1000_FREQ_0A            0x03 // Frequency Register 0A
#define CC1000_FREQ_2B            0x04 // Frequency Register 2B
#define CC1000_FREQ_1B            0x05 // Frequency Register 1B
#define CC1000_FREQ_0B            0x06 // Frequency Register 0B


#define CC1000_FSEP1              0x07 // Frequency Separation Register 1
#define CC1000_FSEP0              0x08 // Frequency Separation Register 0

#define CC1000_CURRENT            0x09 // Current Consumption Control Register
#define CC1000_FRONT_END          0x0A // Front End Control Register
#define CC1000_PA_POW             0x0B // PA Output Power Control Register
#define CC1000_PLL                0x0C // PLL Control Register
#define CC1000_LOCK               0x0D // LOCK Status Register and signal select to CHP_OUT (LOCK) pin
#define CC1000_CAL                0x0E // VCO Calibration Control and Status Register
#define CC1000_MODEM2             0x0F // Modem Control Register 2
#define CC1000_MODEM1             0x10 // Modem Control Register 1
#define CC1000_MODEM0             0x11 // Modem Control Register 0
#define CC1000_MATCH              0x12 // Match Capacitor Array Control Register for RX and TX impedance matching
#define CC1000_FSCTRL             0x13 // Frequency Synthesiser Control Register
#define CC1000_PRESCALER          0x1C // Prescaler and IF-strip test control register
#define CC1000_TEST6              0x40 // Test register for PLL LOOP
#define CC1000_TEST5              0x41 // Test register for PLL LOOP
#define CC1000_TEST4              0x42 // Test register for PLL LOOP (must be updated as specified)
#define CC1000_TEST3              0x43 // Test register for VCO
#define CC1000_TEST2              0x44 // Test register for Calibration
#define CC1000_TEST1              0x45 // Test register for Calibration
#define CC1000_TEST0              0x46 // Test register for Calibration

// An enumeration describing the various modes of the CC1000 transceiver
enum UHF_MODES {
	UHF_RX_MODE = 0,
	UHF_TX_MODE
};

extern BYTE cc1000_rd( BYTE addr );
extern void cc1000_wr( BYTE addr, BYTE val );
extern void LockUHFThresh(void);
extern void UnlockUHFThresh(void);
extern BYTE InitUHFTranceiver(void);
extern BYTE SetUHFMode( BYTE mode );
extern BYTE GetUHFMode(void);

void pager_write( BYTE value );

#endif
