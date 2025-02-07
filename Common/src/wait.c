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
 * \file wait.c
 * \brief Source file for wait functions.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/03/2008

	V 2.0 	03.01.2011	I.Zalts

 */

#include <htc.h>
#include "mu.h"
#include "types.h"
#include "time.h"
#include "wait.h"
#include "uhf_pkt.h"
#include "init30a.h"


static volatile unsigned short TMR1  @ 0x00E;


typedef struct tmr_s {
   BYTE   lo;
   BYTE   hi;
} tmr_s;



typedef union tmr_u {

	tmr_s  dat;
	ushort cnt;

} tmr_u;


//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void delay_uhf_ts( BYTE ts )
{
ushort tmr;
ushort us;

	while( ts-- )
	{
 		do{ tmr = TMR1H;
			tmr = tmr<<8 | TMR1L;
		}while( TMR1H != (BYTE)(tmr>>8) );

    	us  = tmr + ( UHF_TIMESLOT_PERIOD_US - 34 );

		CLRWDT();

		for( ; ; )
		{
			if( tmr >= us )
			{  	if( ( tmr - us ) < 0x8000 )
					break;
			}
			else
			{   if( ( us - tmr ) > 0x8000 )
					break;
			}

	 		do{	tmr = TMR1H;
				tmr = tmr<<8 | TMR1L;
			}while( TMR1H != (BYTE)(tmr>>8) );

		}
	}
}


//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void delay_ms( ushort ms )
{
tmr_u t;
ushort us;

	while( ms-- )
	{
 		do{ t.dat.hi = TMR1H;
			t.dat.lo = TMR1L;
		}while( TMR1H != t.dat.hi );

		CLRWDT();
    	us  = t.cnt + ( 1000 - 34 );
		for( ; ; )
		{   if( t.cnt >= us )
			{  	if( ( t.cnt - us ) < 0x8000 )
					break;
			}
			else
			{   if( ( us - t.cnt ) > 0x8000 )
					break;
			}

	 		do{	t.dat.hi = TMR1H;
				t.dat.lo = TMR1L;
			}while( TMR1H != t.dat.hi );

		}
	}
}


/*
This function cannot be used - it is unstable and can provide unpredictable delays because of timer overflow conditions which cannot be
be properly detected due to a variable amount of time that an interrupt can steal from this countdown. This can cause multiple wrap-arounds
resulting in unpredictable timeouts every so often.

void delay_us_old( ushort us_u16 )
{
   ushort tmr1_u16;
   do {
      CLRWDT();
      tmr1_u16 = (ushort)(TMR1H << 8) | (ushort)(TMR1L);
   } while ( (((ushort)(TMR1H << 8) | (ushort)(TMR1L)) - tmr1_u16) > 0x7F );	// while the difference is greater than 127

   us_u16 += tmr1_u16 + 4;

   while ( (((ushort)TMR1H << 8) | (ushort)TMR1L) > us_u16 );  CLRWDT();     // Wraparound condition
   while ( (((ushort)TMR1H << 8) | (ushort)TMR1L) < us_u16 );  CLRWDT();

}
*/



// This timeout is approximate since frequency background interrupts will slow it down and delays vary slightly with size
// error is on the side of too long - all the routines that call this function can tolerate this
// a clearing of the watchdog timeout is included in case the timeout is very long
#define	XTAL_FREQ		8		// Crystal frequency in MHz
#define	XTAL_FREQ_REF	12
#define MAXIMUM_DELAYUS 400

void Wait250Us()
{
volatile unsigned char dcnt = 125;
	while(dcnt-- != 0);
}

// This timeout is approximate since frequent background interrupts from the VLF or UHF can slow it down
// error is on the side of too long - make sure that all the routines that call this function can tolerate this
// The watchdog has to be cleared since this routine is mostly called to implement timouts for random responses to UHF and VLF
// note that WaitMS doesn't use WaitUs to avoid an unnecessary a stack depth
/*
#define USEC250_CONSTANT 248
void WaitMs( ushort ms6 )
{
volatile ushort dcnt;

	while( ms6-- )
	{ 	CLRWDT();
		// Do multiples of MAXIMUM_DELAYUS
		dcnt = 220;
		while( --dcnt != 0 );
	}
}


void Wait5500Us(void)
{
volatile ushort i;
	for( i=0; i < 950; i++ );
}
*/

void Wait25Us(void)
{
volatile BYTE dcnt = 13;
	while( --dcnt != 0 );
}



#if( USE_IRIDIUM )
#define  UART_TX	LAMP_FET
#define  OUT_HI		LAMP_OFF
#define  OUT_LO		LAMP_ON
#define  SUART_STOP		(0xF0)


BYTE byte_cnt = 0;
BYTE byte_bit_cnt = 0;

#define  UART_TX	LAMP_FET
#define  OUT_HI		LAMP_OFF
#define  OUT_LO		LAMP_ON

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void suart_send_id( void )
{
	byte_cnt     = 1;
	byte_bit_cnt = 0;
}


//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
BYTE suart_send_process( void )
{
BYTE data;

	if( byte_cnt )
	{
		if( byte_cnt > 3 )
		{	byte_cnt = byte_bit_cnt = 0;
			// �������� ���������. ����� ����������� ������� �������
			return( 1 );
		}

		data = mid.b[byte_cnt];

		switch( byte_bit_cnt )
		{
		case 0:
		   	UART_TX = OUT_LO;     // Start State
			byte_bit_cnt++;
			break;

		case 9:
		   	UART_TX = OUT_HI;     // Stop State
			byte_bit_cnt = 0;
			byte_cnt++;
			break;

		default:
			// Data Bit
 			if( ( data >>( byte_bit_cnt-1 )) & 1 )
			   	UART_TX = OUT_HI;
			else
			   	UART_TX = OUT_LO;
			byte_bit_cnt++;
		}

		// ���� ������� ��������
		return( 0 );

	}
	// �������� ���. ����� ����������� ������� �������
	return( 1 );
}


#endif


/*
#if( USE_IRIDIUM )
//---------------------------------------------------------------------------
// Software UART transmit function
// Timing dependent loop!!!

#define  UART_TX	LAMP_FET
#define  OUT_HI		LAMP_OFF
#define  OUT_LO		LAMP_ON
//---------------------------------------------------------------------------
void suart_send_id( void )
{
volatile uchar i;
volatile uchar j;
volatile uchar data;

   	UART_TX = OUT_HI;     // Stop State

	LED_GREEN = TEST_LEDX_ON;  	// Test LED On

    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );
    asm( " nop" );

	di();
 	for( i = 0; i < 4; i++ )         // 174 cycles round trip
 	{  	data = mid.b[i];
      	UART_TX = OUT_LO;   		  // Start bit
      	j = 0;
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
     	asm( " nop" );
      	asm( " nop" );
//---  34 �����
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
//---


      	for( ; ; )           // 17 cycles round trip
 		{   if( data & 1 )
            {   asm( " nop" );
            	UART_TX = OUT_HI;
            }
         	else
            {   UART_TX = OUT_LO;
            	asm( " nop" );
            	asm( " nop" );
            }

         	data >>= 1;
	       	asm( " nop" );
         	asm( " nop" );
         	asm( " nop" );

//---  34 �����
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
//---




         	if( ++j == 8 )
            	break;
    	}
		asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );


//---  34 �����
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
//---
      	UART_TX = OUT_HI;           // Stop bit
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

//---  34 �����
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );

      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
      	asm( " nop" );
//---


	}

	LED_GREEN = TEST_LEDX_OFF;  	// Test LED Off
	ei();
}


#endif
*/


