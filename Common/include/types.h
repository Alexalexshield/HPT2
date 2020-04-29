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
 * \file types.h
 * \brief Header file for all data types used on the PIC.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
	03.01.2011 Igors Zalts
 */

#ifndef __TYPES_H
#define __TYPES_H

typedef unsigned short ushort;      // A 16 bit unsigned value
typedef unsigned long ulong;
typedef unsigned char BYTE;        	// An 8 bit unsigned value
typedef unsigned char uchar;


#define BIT(n)           ( 1U <<   (n) )
#define SETBIT( p, n )   ( (p)|= BIT(n) )
#define CLRBIT( p, n )   ( (p)&=~BIT(n) )
#define XORBIT( p, n )   ( (p)=(p)^BIT(n) )
#define TSTBIT( p, n )   ( (p)&  BIT(n) )

#define BIT0		(BIT(0))
#define BIT1		(BIT(1))
#define BIT2		(BIT(2))
#define BIT3		(BIT(3))
#define BIT4		(BIT(4))
#define BIT5		(BIT(5))
#define BIT6		(BIT(6))
#define BIT7		(BIT(7))


// A union to used to access a 16 bit value as all 16 bits or its indivudual bytes
typedef union {
   BYTE                 b[2];
   ushort               w;
   struct {
      BYTE            	lo;
      BYTE              hi;
   } 	st;
} short_u;

typedef union {
   BYTE                 b[2];
   ushort               w;
   struct {
      BYTE            	hi; 		// From Nenwork to Host
      BYTE              lo;         // From Nenwork to Host
   } ho;
} bid_u;


// A union to used to access a 32 bit value as all 32 bits, its indivudual 16 bit words, or its 8 bit bytes
typedef union {
   BYTE           		b[4];
   ulong                w;
//   struct {
//      short_u             lo_word;
//      short_u             hi_word;
//   }  st;
} long_u;

#define	TRUE	(1)
#define	FALSE	(0)
#define RESULT_OK	(0)

//---------------------------------------------------------------------------
typedef struct mid24
{
	BYTE val[3];
} mid24_s;




#define NTOHS( x )	( ( (x) >> 8 )|( (x) << 8 ) )

#endif
