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
 * \file time.h
 * \brief Header file for wait functions.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/03/2008

   03.01.2011	I.Zalts

 */

#ifndef __WAIT_H
#define __WAIT_H


#include <htc.h>
#include "types.h"


//void delay_us( ushort us );
void delay_us_old( ushort us );
void delay_ms( ushort ms );
void delay_uhf_ts( BYTE ts );

//void WaitMs( ushort us_u16 );
extern void Wait250Us(void);
extern void Wait25Us(void);
extern void Wait5500Us(void);


#if( USE_IRIDIUM )
void suart_send_id( void );
BYTE suart_send_process( void );
#endif



#endif
