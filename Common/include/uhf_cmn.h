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
 * \file uhf_cmn.h
 * \brief Header file for common UHF communicatons functionality.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/03/2008
 */

#ifndef UHF_CMN_H
#define UHF_CMN_H


#include <htc.h>
#include "types.h"
#include "uhf_pkt.h"


// The various RX and TX states the MU can be in
enum UHF_RXTX_STATES  {
   UHF_RXTX_PREAMBLE,
   UHF_RXTX_RESYNC,
   UHF_RX_HDR,
   UHF_RXTX_PKT,
   UHF_RX_COMPLETE,
   UHF_TX_TRAILER,
   UHF_TX_END,
   UHF_TX_COMPLETE
};


BYTE UHFCommsInit(void);
BYTE ProcessUHFChar( BYTE ch_u8 );
BYTE SendUHFPacketChar(void);
BYTE CalcUHFChecksum( uchar set_flag );
BYTE get_pkt_len(void);


#define CMD_MASTER	1
#define CMD_SLAVE	0

#endif
