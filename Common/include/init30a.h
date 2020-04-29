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
 * \file mu.h
 * \brief Header file for MU REV30A board.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/03/2008

	03.01.2011 Igors Zalts
 */

#ifndef __INIT30A_H
#define __INIT30A_H


#include <htc.h>
#include "types.h"
#include "mu.h"



#define  PAGER_VLF                      RA0


#define  LED_GREEN                   	RB0
#define  UHF_LOCK                       RB1
#define  SSP_PAGER_CS                   RB2
#define  LF_TX_SIG                      RB3
#define  UHF_RSSI                       RB4
#define  UHF_PALE                       RC0
#define  LAMP_FET	                    RC1
#define  TX_PWM                         RC2
#define  SSP_CLK                        RC3
#define  SSP_SDI                        RC4
#define  SSP_SDO                        RC5
#define  TEST_POINT30                   RB5
#define  TEST_POINT25                   RA3



#if( HPT4_MODE  )

#define TEST_LEDX_OFF 	1
#define TEST_LEDX_ON 	0

#elif( HPT3_MODE )

#define TEST_LEDX_OFF 	1
#define TEST_LEDX_ON 	0
#define INP_IRIDIUM                   	TEST_POINT30

#else

#define  PAGER_ALARM					SSP_PAGER_CS

#if( DEBUG_LEVEL )
#define  TEST_POINT29                   RA4
#define  TEST_LED2_VLF_COMP            	TEST_POINT29
#define  TEST_LED1_TX                   TEST_POINT30
#define  TEST_LED0_RX_CRC_ERROR         TEST_POINT25
#endif

#if( DEBUG_LEVEL > 1 )
#define  TEST_LED4_VLF_TX              	RB7
#define  TEST_LED5_VLF_RX              	SSP_PAGER_CS
#define  TEST_LED6              		RB6
#endif

#if( DEBUG_LEVEL )
#define  TEST_LED3_RX                  	LAMP_FET
#endif


#define TEST_LEDX_OFF 	1
#define TEST_LEDX_ON 	0

#endif


void init_board(void);

BYTE ReadADChannel( BYTE channel );

#endif
