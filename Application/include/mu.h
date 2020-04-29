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
 * \file mu.h
 * \brief Header file for MU firmware application.
 * \author Ryan Lishman
 * \version 1.0
 * \date 01/01/2005

	03.01.2011 Igors Zalts
	12.08.2011
 */

#ifndef __MU_H_
#define __MU_H_

#include <htc.h>
#include "types.h"
#include "time.h"


//-----------------------------------------------------------------------------------------------------------
//#define HPT5_MODE						0		// Усеченный вариант без поиска, но с прямой работой с пейдером
//#define HPT5_MODE_TST  				0      	// Времянка для отладки Soft UART

#define HPT4_MODE						0		// Усеченный вариант без приемника 8KHz, но со входом датчика



#define HPT3_MODE						6		// 1 - Пейджер с индивидуальными вызовами
												// 2 - Казахский с LEXT
												// 3 - Казахский с Iridium (Обратое считывание)
												// 4 - Комбайнер для блокировок + подтверждение (13-я прошивка))
												// 5 - Кнопка подтверждения
												// 6 - Комбайнер для блокировок, без кнопки подтверждения с переключение инв/ не инвернсый (14-я прошивка)
												// пока отключаю пейджер...

#define MU_VLF_TX_TESTING   			0 		// 1=Normal, 2=Preamble 0xFF 3=Full 4=0/1 непрерывно
#define USE_8KHZ_FILTER 				1




#define	HPT2_MODE						0       // Стандартный пейджер


/*
#define IPTX_MODE						0		// IPTx    1 - Cо светодиодами,
												//         2 - LED заблокированы + вход управления лампой.
												//         3 - LED заблокирован+xx - пока не используется

#define IVTX_MODE						0		// IVTx
*/

#define USE_APS							1		// Функция регистрации перепроезда и нахождения в запрещённой зоне

#define USE_PAGER						1		// Использование пейджера
//-----------------------------------------------------------------------------------------------------------



#define DEBUG_LEVEL		 	0

#define DEBUG_II 			0					//
#define DEBUG_SU_UHF_RX		0					//
#define DEBUG_RX_VLF 	 	0


 // Enable sending a test signal
#define MU_433_TX_TESTING	0
//#define MU_VLF_TX_TESTING   0					// 1=Normal, 2=Preamble 0xFF 3=Full 4=0/1 непрерывно
#define MU_PAGER_TESTING	0
#define MU_R8KHZ_TESTING	0

#define VLF_TEST_TAG		0					// 1- special tag for testing vlf

#define USE_MOBILITY 		0					//

#if( DEBUG_LEVEL )
#define TEST_BEACON_REGISTRATION 		0		//
#else
#define TEST_BEACON_REGISTRATION 		0		//
#endif


//#define HPT3_DISABLE_CALL				0       // 1 - Отключить оповещение при регистрации


//#if( ( HPT4_MODE == 0 )&&( HPT3_MODE == 0 ) )
//#define	HPT2_MODE						1
//#else
//#define	HPT2_MODE						0
//#endif


#if ( HPT3_MODE == 5 )||( HPT3_MODE == 4 )       // С подтверждением, или комбайнер
 #define USE_LAMP_KEY 						1
 #define USE_IRIDIUM 						0
#elif( HPT3_MODE == 3 )
 #define USE_LAMP_KEY 						0
 #define USE_IRIDIUM 						1
#elif ( HPT3_MODE == 6 )
 #define USE_LAMP_KEY						0
 #define USE_IRIDIUM						0
#else
 #define USE_LAMP_KEY 						0
 #define USE_IRIDIUM 						0
#endif




#if( HPT4_MODE )

 #define USE_INVERT_LAMP					0
 #define USE_UHF_TO_VLF 					1

#else

 #if( HPT3_MODE == 2 )
  #define USE_INVERT_LAMP					2	// 2 - Invert Mode & Set sign
 #elif( HPT3_MODE == 3 )
  #define USE_INVERT_LAMP					2	// 2 - Invert Mode & Set sign
 #elif (HPT3_MODE ==4)
  #define USE_INVERT_LAMP					0	// 2 - Invert Mode & Set sign)
#elif (HPT3_MODE == 6)
  #define USE_INVERT_LAMP					2	// 2 - Invert Mode & Set sign)
 #else
  #define USE_INVERT_LAMP					0	// 0 - No Invert Mode & Clear sign
												// 1 - Use Invert Sign
												// 2 - Invert Mode & Set sign
 #endif

 #define USE_UHF_TO_VLF 					1	// UHF SU Mode support



#endif


#if( HPT3_MODE )
//#define APP_FIRMWARE_VERSION          0x70 	// Поддержка другого пейджера с тремя сообщениями
//#define APP_FIRMWARE_VERSION          0x71 	// VLF Блокировка исправления
//#define APP_FIRMWARE_VERSION          0x72 	// VLF FlexAlert
//#define APP_FIRMWARE_VERSION          0x73 	// Patch Pager CRC :(
//#define APP_FIRMWARE_VERSION          0x74 	// Убран patch и добавлены персональные вызовы во FlexAlert
//#define APP_FIRMWARE_VERSION          0x75 	// Тест  Эфир инверсия
//#define APP_FIRMWARE_VERSION          0x77 	// Воркута вариант 3
//#define APP_FIRMWARE_VERSION          0x78 	// Воркута PLL и в HPT3 пока отключено оповещение при регистрации

//#if( DEBUG_BLINK )
//
//#define APP_FIRMWARE_VERSION            0x7D 	// Полное маскирование через 433 изменено и в HPT3 пока отключено оповещение при регистрации
//
//#else

//#define APP_FIRMWARE_VERSION            0x79 	// Полное маскирование через 433 изменено и в HPT3 пока отключено оповещение при регистрации
//#define APP_FIRMWARE_VERSION            0x7A 	// Полное маскирование через 433 изменено и в HPT3 пока отключено оповещение при регистрации
//#define APP_FIRMWARE_VERSION            0x7B 	// Тест VLF по команде II работает. Заблокирован вход Reset
//#define APP_FIRMWARE_VERSION            0x7C 	// Чистка исходников от лишних сборок и LEXT
//#define APP_FIRMWARE_VERSION            0x7D 	// Поддержка Iridium
//#define APP_FIRMWARE_VERSION            0x7E 	// Начало поддержки кнопки подтверждения
//#define APP_FIRMWARE_VERSION            0x7F 	// Фильтрация импульсов от пейджера
//#define APP_FIRMWARE_VERSION            0x80 	// Вариант Комбайнера с подтверждением добавлен
//#define APP_FIRMWARE_VERSION            0x88	// Тест антипроезда
//#define APP_FIRMWARE_VERSION            0x89	// Блокировка комбайна по низкой частоте 
//#define APP_FIRMWARE_VERSION            0x8A	// Блокировка комбайна по низкой частоте РЕЛИЗ
//#define APP_FIRMWARE_VERSION              0x8B	// Блокировка комбайна по низкой частоте. Исправлен момент с незабыванием комбайна.
//#define APP_FIRMWARE_VERSION              0x8C	// Комбайнер перерегистрируется на биконе комбайна каждые 15 секунд.
  #if( HPT3_MODE == 4 )
	#define APP_FIRMWARE_VERSION              0x8D	// Если включен сервисный режим, метка смотрит, что 1 БИТ комбайна сброшен и перестает обрабатывать 8 кГц. подтверждение кнопки
  #elif (HPT3_MODE == 6)
	#define APP_FIRMWARE_VERSION              0x8E	//Без подтверждения,  настройка инверсного/не инверсного режима лампы
  #endif
#endif

#if( HPT4_MODE )

#define APP_FIRMWARE_VERSION            0x7B 	// Тест VLF по команде II работает. Заблокирован вход Reset

#endif

#if( HPT2_MODE )
//#define APP_FIRMWARE_VERSION          0x60
//#define APP_FIRMWARE_VERSION          0x61 	// Усечена таблица размеров Ctrl фреймов. Максимальный размер фрейма
//												// оставлен без изменений
//#define APP_FIRMWARE_VERSION          0x62 	// Фатальная ошибка при VLF
//#define APP_FIRMWARE_VERSION         	0x63 	// Совместимость с 7.00
//#define APP_FIRMWARE_VERSION          0x64
//#define APP_FIRMWARE_VERSION         	0x65  	// Режим IVT поддерживается
//#define APP_FIRMWARE_VERSION         	0x66  	// PLL
//#define APP_FIRMWARE_VERSION          0x67  	// Полное маскирование через 433 изменено
#define APP_FIRMWARE_VERSION          	0x68  	// Test МДА по команде II работает. Заблокирован вход Reset
#endif


// Allow a UHF BOOTLOAD COMMAND
#define ALLOW_UHF_BOOTMODE		1

// Enable debugging flags
//#define ENABLE_DEBUG_FLAGS		0

#define  MOBILITY_AD_CHANNEL 	(0<<2)	// The A/D channel to take mobility readings from
#define  TXSIG_AD_CHANNEL       (9<<2) 	// The A/D channel to take TX readings from

//zlt[
#define  LAMP_SEARCH_STATE_TIMEOUT     	(ushort)( 3*60*(TICKS_PER_SEC))
#define  LAMP_60SEC_TIMEOUT     		(ushort)(   60*(TICKS_PER_SEC))

#define  LAMP_CHECK_STATE_TIMEOUT     	(ushort)(   90*(TICKS_PER_SEC))
#define  LAMP_APS_TIMEOUT				(ushort)(    1*(TICKS_PER_SEC))    

//#if( DEBUG_BLINK )
//
//#define  LAMP_CALL_STATE_TIMEOUT     	(ushort)( 500*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5
//#define  LAMP_CALL_STATE_TIMEOUT_ALT   	(ushort)( 5*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5
//
//#else

#define  LAMP_CALL_STATE_TIMEOUT     	(ushort)( 5*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5

//#endif


#define  LAMP_EVACUATE_STATE_TICK      	(ushort)(30*60*(TICKS_PER_SEC)) // The timeout for exitting an evacuate state
#define  LAMP_IDENTIFY_STATE_TIMEOUT    (ushort)(10*(TICKS_PER_SEC)) 	// The timeout for exitting an identify state


//#if( DEBUG_BLINK )
//#define  LAMP_ALARM_STATE_TIMEOUT       (ushort)( 500*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5
//#define  LAMP_ALARM_STATE_TIMEOUT_ALT   (ushort)( 5*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5
//#else
#define  LAMP_ALARM_STATE_TIMEOUT       (ushort)( 5*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5
//#endif


#define  LAMP_ALARM_UHF_TIMEOUT       	(ushort)(60*(TICKS_PER_SEC))    // Pager timeout
#define  LAMP_SCHEDULE_SIZE             ( 8 )                           // Size of the flash schedule for each state
#define  LAMP_SCHEDULE_MASK            	((LAMP_SCHEDULE_SIZE)-1)

/*
#if( ENABLE_DEBUG_FLAGS  )
typedef struct {              // This is the generic response to all unicast commands
	union {
		struct {
				BYTE missed_tick:1;
				BYTE got_uhf_preamble:1;
				BYTE got_uhf_stx:1;
				BYTE watchdog:1;
				BYTE invalid_rx_uhf_state:1;
				BYTE cc1000_lockfail:1;
				BYTE bit6:1;
				BYTE bit7:1;
		}b;
		BYTE value;
	}f;
} DebugFlags;

extern DebugFlags dflags;
#endif
*/

// An enumeration of the various states that the MU may be in

enum LMP_STATES {
	LAMP_POST_ERROR = 0,
    LAMP_FIRMWARE_disabled,
    LAMP_SEARCH_IN_PROGRESS,    	// A trapped miner search is being conducted
    LAMP_EVACUATE_IN_PROGRESS_disabled,      // An evacuation has been signaled
	//LAMP_IDENTIFY,         			// MU must identify itself
	LAMP_IDENTIFY,         			// Blocked the harvester

    LAMP_PAGER_ALARM,              	// MU must Pager Alarm
	LAMP_PAGER_CALL_1,
	LAMP_PAGER_CALL_2,
	LAMP_VLF_BLOCKED_STATE,
    LAMP_STATES_NUM,               	// Total number of abnormal MU states

	LAMP_NORMAL_STATE = 0xF0,		// The MU is in a default state
	LAMP_INVERT_STATE = 0xF1,		// The MU is in a default state (INVERT)
	LAMP_TEST_STATE   = 0xF2,	 	// The MU is in a Test state
};


extern BYTE     lamp_state;
//extern BYTE     lamp_mode;
extern ushort 	lamp_timeout;
extern BYTE 	lamp_timeout_loops;
extern BYTE		mu_flags;
#define	FL_SEND				(0x01)
#define FL_REGISTRATION		(0x02)
#define FL_LAMP_INV			(0x04)
#define FL_ACK				(0x08)
#define FL_ACK_FORCED		(0x10)
#define FL_QUIET_PAGER      (0x20)
#define FL_ACK_MANUAL		(0x40)
//#define FL_OK				(0x80)
#define FL_8KHZ_TRAP		(0x80)

BYTE ReadADChannel( BYTE channel );

extern long_u    mid;
//extern ulong    mu_state_timeout_u32;
extern BYTE lamp_min_cnt;

#if( HPT4_MODE )

 #define LAMP_ON		1
 #define LAMP_OFF		0

#else

 #define LAMP_ON		0
 #define LAMP_OFF		1

#endif


#endif
