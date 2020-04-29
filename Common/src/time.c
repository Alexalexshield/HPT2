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
 * \file time.c
 * \brief Source file for time related structures and global time functions.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005

	06.01.2011 Igors Zalts
 */

#include <htc.h>
#include "mu.h"
#include "stdlib.h"
#include "types.h"
#include "time.h"


volatile Timer_u timer;		// The global tick timer counter
BYTE timer_last_tick;
#if( DEBUG_II || DEBUG_SU_UHF_RX )
BYTE timer_ii;
//BYTE state_ii;
#endif
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
BYTE chk_timeout( ushort counter )
{
	di();
	if(  timer.cnt >= counter )
	{	if( ( timer.cnt - counter ) < 0x8000U )
		{	ei();
			return( TRUE );    	// Expiry
		}
	}
	ei();
	return( FALSE );
}

