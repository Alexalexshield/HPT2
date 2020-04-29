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
 * \file eeprom.h
 * \brief Header file for EEPROM map.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005
 */

#ifndef __EEPROM_H
#define __EEPROM_H

#include <htc.h>
#include "types.h"


#define  EEPROM_SD_OFFSET                    0
//#define  EEPROM_VLF_COMMS_OFFSET             0x10

#define  FLASH_ID_WORD_ADDR                  0x1FFC

#define  EEPROM_LAMP_MODE_SIGN1              0x77
#define  EEPROM_LAMP_MODE_SIGN0              0x00

#define  EEPROM_SD_OFFSETOF( member )        ((BYTE)(&((EeSharedDataTSt*)EEPROM_SD_OFFSET)->member))

typedef struct {
   BYTE    warm_boot;
   BYTE    valid_img_u8;
   BYTE    wdog_resets;
   BYTE    bl_ver;
   BYTE    app_ver_u8;
   BYTE    tx_hw_power_u8;
   BYTE    lamp_mode;
   BYTE	   d7;

   BYTE    warm_boot_sec;

} EeSharedDataTSt;

#endif
