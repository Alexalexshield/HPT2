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
 * \brief Header file for time related structures and global time functions.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/03/2008

	07.01.2011 Igors Zalts
 */

#ifndef TIME_H
#define TIME_H

#include <htc.h>
#include "types.h"

#define  SYS_FREQ                            2000000 	// The system frequency in Hz
#define  TICKS_PER_SEC                       15.2588    // The number of clock ticks per second


typedef union {
   BYTE       cnt_byte;
   struct {
   BYTE       lo;
   BYTE       hi;
   }s;
   ushort     cnt;
} Timer_u;


extern volatile Timer_u  timer;
extern BYTE timer_last_tick;
extern BYTE timer_ii;
//extern BYTE state_ii;

BYTE chk_timeout( ushort endcount );
#define get_timeout(x) 	(timer.cnt + (ushort)(x))


//extern volatile BYTE           timer_ticks_u8;
//extern volatile ushort countdown_timer_ticks_u16;	// Caution! used as a countdown timer - can only be used by one function at a time


#endif
