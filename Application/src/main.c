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

  Copyright Mine Radio Systems

**************************************************************************************************/
/*
	06.01.2011 Igors Zalts
*/

// Максимально точная аппаратная реализация миллисекундных и таймслотовых задержек с минимизированной
// зависимостью от наличия источников прерываний;
// Исключен обработчик прерывания EEPROM;
// Оптимизирован обработчик прерывания приема тела UHF пакета;
// Исключено ожидание trailer bytes при приеме UHF пакета;
// Быстрый переход UHF приемника в состояние `Preamble` при Frame & Overrun error;
// Работа с UHF CRC16 оптимизирована;
// Переменные обслуживающие RX/TX VLF об]единены в структуры и помещены в общий банк памяти - оптимизация
// по производительности;
// Оптимизирован по времени обработчик прерывания передачи VLF пакета;
// Вычисление VLF CRC8 оптимизировано;
// Восстановлен контроль VLF CRC8 при приеме команд "MU Quick Search";
// Ранний отсев VLF пакетов по типу команды до контроля CRC8;
// Радикально укорочен обработчик 65,535 таймерного прерывания;
// Убраны несуразности srand()/rand();
// Унифицировано управление Cap Lamp и исключено безумное количество переменных и ресурсов ранее используемых
// для этого;
// Вместо большого количества разрозненных счетчиков унифицированная служба 65,535 ms таймера;
// Разумное использование watcdog, вместо тупого сброса в Main loop;
// Переинициализация СС1000 только в случае отсутствия приема пакетов вместо тупо-периодического;
// Блокировка непрерывной передачи подтверждений в случае неответа Beacon;
// Переменные и буфер обслуживающий RX/TX UHF об]единены в структуры и помещены в общий банк памяти - оптимизация
// по производительности;
// Использована стандартная функция rand() для UHFC_SEARCH;
// Исключена посылка VLF пакета по включению питания;
// Баг в VLF_SendGeneric() при повторном подсчете CRC8

#include <htc.h>
#include "stdlib.h"
//#include "math.h"
#include "types.h"
#include "init30a.h"
#include "eeprom.h"
#include "mu.h"
#include "time.h"
#include "wait.h"
#include "cc1000.h"
#include "uhf_cmn.h"
#include "vlf_comms.h"
#include "uhf_comms.h"
#include "mobility.h"

long_u 	mid;

BYTE	lamp_state;
BYTE	mu_flags;
BYTE 	lamp_min_cnt;
BYTE   	lamp_tick;
#if( HPT3_MODE )
 BYTE	pager_cnt = 0;
 #if( USE_LAMP_KEY )
 BYTE	lampk_cnt = 0;
 BYTE	lampo_cnt = 0;
 #endif
 #if( USE_IRIDIUM )
 BYTE	lampk_cnt = 0;
 #endif
#endif
ushort 	lamp_timeout;


static ushort uhf_reset_timeout;
extern bank3 BYTE bl_version;
extern bank3 ushort combine_id;

BYTE eeprom_tmp;

//#if( HPT4_MODE )
//BYTE pager_vlf_state;
//#endif


void Test433tx(void);

//static void LampTickHandler(void);

//---------------------------------------------------------------------------
static const BYTE lamp_schedule[LAMP_STATES_NUM][LAMP_SCHEDULE_SIZE] = {


{  	0xCC,  		// POST Error
   	0xCC,
   	0xCC,
   	0xCC,
   	0xCC,
   	0xCC,
   	0xCC,
   	0xCC     },

{  	0xFF,       // ZLT Fifmware Subversion
   	0x00,		// Long->Trembling   - disabled
   	0xFF,
   	0x00,
   	0xFF,
   	0x00,
   	0xFF,
   	0x00     },

{  	0xFF,       // Search  - мигание отключено, но этo состояние используется
	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF		},


{  	0xFF,       // Evacuate_disabled
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF     },

{  	0xF9,//0xF9 = 130 ms   0xFD = 65 ms,       // Identify  - одиночный импульс в ответ на тестовый
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF     },

#if( HPT3_MODE == 2 )

{  	0x11,       // Pager Alarm
   	0x11,
   	0x51,
   	0x55,
   	0x00,
   	0x00,
   	0x00,
   	0x00     },

{  	0x41,  		// Beacon registred/Personal Call 1
   	0x10,
   	0x00,
   	0x00,
   	0x00,
   	0x00,
   	0x00,
   	0x00     },

{  	0x41,  		// Beacon Personal Call 2
   	0x10,
   	0x04,
   	0x41,
   	0x00,
   	0x00,
   	0x00,
   	0x00     },

#else

{  	0xEE,       // Pager Alarm
   	0xEE,
   	0xAA,
   	0xAA,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF     },

{  	0xCC,  		// Beacon registred/Personal Call 1
   	0xFC,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF     },

{  	0xCC,  		// Beacon Personal Call 2
   	0xCC,
   	0xCC,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF     },

#endif

{  	0xFF,  		// VLF Blocked State - Состояние по определению без мигания
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF,
   	0xFF     }

};


#if( MU_VLF_TX_TESTING )
extern void TestVlfTx( ushort times );
#endif

// Test delay routines
#define TEST_DELAY_ROUTINES 	0

#if( TEST_DELAY_ROUTINES )
void TestDelayRoutines()
{
	TRISA = 0x03;		// 0000 0011  A1,A0 are inputs - the rest are outputs
	TRISB = 0x1A;		// 0001 1010  B4,B3,B1 are inputs - the rest are outputs
    T1CON = 0x11;       // Timer is started, 1 ticks/us

	for( ; ; )
	{
		TESTPT30 = 1;
		delay_uhf_ts( 2 );
		TESTPT30 = 0;
	}
}
#endif



//---------------------------------------------------------------------------
// The main routine.
// This routine gets called by the C runtime environment startup function.
// It never returns
//---------------------------------------------------------------------------
void main( void )
{

 	PCON |= 0x3;		   	// Set brownout and reset flags so we can tell if the next one is a watchdog

   	// Enable Watchdog timeout for about 2.1 seconds
   	WDTCON = 0b00010110;    // 1011 = 1:65536 Prescale Value which is about 2.1 seconds
	CLRWDT();
	SWDTEN = 1;				// Enable watchdog

	init_board();

#if( HPT4_MODE )
	mid.b[0] = 0x0B;  	// Change to "HPT4"
#elif( HPT3_MODE )
 #if( USE_IRIDIUM )
	mid.b[0] = 0x12;  	// Change to "HPT3-Iridium"
 #elif( HPT3_MODE == 4 )
	mid.b[0] = 0x16;//0x14;  	// Change to "HPT3-Комбайнер"
 #elif( HPT3_MODE == 6 )
	mid.b[0] = 0x16;//А0x14;  	// Change to "HPT3-Комбайнер" без подтверждения кнопки с переключением инверсный/не инверсный

// #elif( USE_LAMP_KEY )
//	mid.b[0] = 0x15;  	// Change to "HPT3-Key"
 #else
	mid.b[0] = 0x0A;  	// Change to "HPT3"
 #endif
#elif( HPT2_MODE )
	mid.b[0] = 0x09;  	// Change to "HPT2"
#else
	mid.b[0] = 0x00;  	// Change to "Generic"
#endif
	


	mid.b[1] = FLASH_READ( (ushort)FLASH_ID_WORD_ADDR + 1 );
//fist bit in MSB set to 1 (131072)
//	mid.b[1] |= 0x02;

	mid.b[2] = FLASH_READ( (ushort)FLASH_ID_WORD_ADDR + 2 );
	mid.b[3] = FLASH_READ( (ushort)FLASH_ID_WORD_ADDR + 3 );		// LSB

#if( TEST_DELAY_ROUTINES  )
	TestDelayRoutines();
#endif


#if( USE_LAMP_KEY || USE_IRIDIUM )

	mu_flags = 0;		// Гарантированно не инверсный режим

 #if( USE_LAMP_KEY )
	lampo_cnt = 0;
 #endif

#else

#if( USE_INVERT_LAMP == 1 )
	eeprom_tmp = eeprom_read( EEPROM_SD_OFFSETOF( lamp_mode ) );
	if( eeprom_tmp != EEPROM_LAMP_MODE_SIGN1 )
	{ 	NOP();
		NOP();
		NOP();
		NOP();
		eeprom_write( EEPROM_SD_OFFSETOF( lamp_mode ), EEPROM_LAMP_MODE_SIGN1 );   	// Write Signature TO EERROM
	}
	mu_flags = FL_LAMP_INV;
#endif
#if( USE_INVERT_LAMP == 2 )
	eeprom_tmp = eeprom_read( EEPROM_SD_OFFSETOF( lamp_mode ) );
	if( eeprom_tmp == EEPROM_LAMP_MODE_SIGN1 )
		mu_flags = FL_LAMP_INV;
	else
	{ 	mu_flags = 0;
		if( eeprom_tmp != EEPROM_LAMP_MODE_SIGN0 )
		{ 	NOP();
			NOP();
			NOP();
			NOP();
			eeprom_write( EEPROM_SD_OFFSETOF( lamp_mode ), EEPROM_LAMP_MODE_SIGN0 );   		// Clear Invert Signature
		}
	}
#endif
#if( USE_INVERT_LAMP == 0 )
	eeprom_tmp = eeprom_read( EEPROM_SD_OFFSETOF( lamp_mode ) );
	if( eeprom_tmp != EEPROM_LAMP_MODE_SIGN0 )
	{ 	NOP();
		NOP();
		NOP();
		NOP();
		eeprom_write( EEPROM_SD_OFFSETOF( lamp_mode ), EEPROM_LAMP_MODE_SIGN0 );   		// Clear Invert Signature
	}
	mu_flags = 0;
#endif
#endif


#if( USE_LAMP_KEY || USE_IRIDIUM )
	LAMP_FET = LAMP_OFF;  		// Off FET
#elif( USE_INVERT_LAMP )
	LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON;	// On
#else
	LAMP_FET = LAMP_ON;	  										// On
#endif

	{
	ushort uid;
	uid  = mid.b[2];
	uid <<= 8;
	uid	|= mid.b[3];
	srand( uid );
	}

	INTCON = 0xC0;          // Global interrupt enable & PERIPHERAL INTERRUPT enable

#if( ALLOW_UHF_BOOTMODE )
// Bug Patch
	bl_version = eeprom_read( EEPROM_SD_OFFSETOF( bl_ver ) );

	if( ( bl_version == 0xFF )||( bl_version == 0x1F ) )
	{   NOP();
		NOP();
		NOP();
		bl_version = 0x21;
		eeprom_write( EEPROM_SD_OFFSETOF( bl_ver ), bl_version );
	}

	// Application version is hardcoded in UHF_COMMS.C	 - will need to reenable write to eeprom if bootloader is enabled
	eeprom_tmp = eeprom_read( EEPROM_SD_OFFSETOF( app_ver_u8 ) );	// If app version not up to date
	if( eeprom_tmp != APP_FIRMWARE_VERSION )	// If app version not up to date
	{   NOP();
		NOP();
		NOP();
		NOP();
		eeprom_write( EEPROM_SD_OFFSETOF( app_ver_u8 ), APP_FIRMWARE_VERSION );		// Write APP VERSION TO EERROM
	}
#endif





#if( HPT3_MODE )
	// Инициализация пейджера идентификатором тага


//#if( HPT3_MODE == 5 )
//
//   	SSP_PAGER_CS = 1;
//
//	lamp_state 	= LAMP_PAGER_ALARM;
//    lamp_timeout = get_timeout( LAMP_ALARM_UHF_TIMEOUT );
//
//	lamp_state 	= LAMP_NORMAL_STATE;		// The MU is in a default state
//
//#else
#if( USE_PAGER )
	eeprom_tmp = eeprom_read( EEPROM_SD_OFFSETOF( warm_boot_sec ) );
	if( eeprom_tmp !=0x00)//== 0x7A )
	{  	// Произошел Soft Reset - признак выставил загрузчик
		// Программирование и тестирование пейджера не производится, поскольку он находится уже в рабочем режиме ;(
		eeprom_write( EEPROM_SD_OFFSETOF( warm_boot_sec ), 0x00 );
		lamp_state = LAMP_NORMAL_STATE;		// The MU is in a default state

		TRISB |= (BIT2|BIT5);  					// SSP_PAGER_CS to Input

	}
	else
	{	// Сброс по включению питания
		pager_write( 0x00 );  		// Dummy
		delay_ms( 50 );

		pager_write( 0xAA );        // Start
		delay_ms( 50 );

 #if( APP_FIRMWARE_VERSION == 0x73 )
		{
		unsigned int crc;
		crc = ( mid.b[1] + mid.b[2] + mid.b[3] );
			if( crc >= 0x100 )
			{	pager_write( mid.b[3] );		// Check Summ
				delay_ms( 50 );
				pager_write( 0 );
				delay_ms( 50 );
				pager_write( 0 );
				delay_ms( 50 );
				pager_write( mid.b[3] );
			}
			else
			{	pager_write( (BYTE)crc );  		// Check Summ
				delay_ms( 50 );
				pager_write( mid.b[1] );
				delay_ms( 50 );
				pager_write( mid.b[2] );
				delay_ms( 50 );
				pager_write( mid.b[3] );
			}
		}
 #else
		pager_write( (BYTE)( mid.b[1] + mid.b[2] + mid.b[3] ) );			// Check Summ
		delay_ms( 50 );

		pager_write( mid.b[1] );
		delay_ms( 50 );

		pager_write( mid.b[2] );
		delay_ms( 50 );

		pager_write( mid.b[3] );
 #endif

		delay_ms( 10 );
		TRISB |= BIT2;			// SSP_PAGER_CS to Input
		delay_ms( 50 );

		if( SSP_PAGER_CS == 0 )
		{	// CS = 0 Pager failed or Soft Restart
   			lamp_state 	= LAMP_POST_ERROR;
			lamp_timeout = get_timeout( 32000 );
		}
		else
		{	// CS = 1  Wait
			delay_ms( 350 );       	// Задержка для на время разборок субмодуля с полученной информацией

			// Контроль окончания импульса
			if( SSP_PAGER_CS == 1 )
			{	// Pager failed
				lamp_state 	= LAMP_POST_ERROR;
				lamp_timeout = get_timeout( 32000 );
			}
			else
			{

//				lamp_state 	= LAMP_FIRMWARE;
//				lamp_timeout = get_timeout( LAMP_SCHEDULE_SIZE * 8 );

				lamp_state 	= LAMP_NORMAL_STATE;
			}
		}
	}
#endif

#elif( HPT4_MODE || HPT2_MODE )

	TRISB |= BIT2;								// SSP_PAGER_CS to Input

	lamp_state 	= LAMP_NORMAL_STATE;		// The MU is in a default state

#else

	lamp_state 	= LAMP_NORMAL_STATE;		// The MU is in a default state

#endif

   	if( UHFCommsInit() )	// Init the UHF communications via the CC1000 module
	{
   		lamp_state 	= LAMP_POST_ERROR;
		lamp_timeout = get_timeout( 32000 );
	}

   	TMR1IE = 1;				// Enable Timer1 tick interrupt for a ~66 ms tick

#if( MU_VLF_TX_TESTING )

	for( ; ; )
	{
		TestVlfTx( 300 );
	}
#endif

	uhf_reset_timeout = get_timeout( UHF_RESET_TIMEOUT );


#if( DEBUG_LEVEL > 1 )
	TEST_LED3_RX     = TEST_LEDX_OFF;
	TEST_LED1_TX     = TEST_LEDX_OFF;
	TEST_LED0_RX_CRC_ERROR = TEST_LEDX_OFF;

	TEST_LED4_VLF_TX = TEST_LEDX_OFF;
	TEST_LED5_VLF_RX = TEST_LEDX_OFF;
	TEST_LED6 		 = TEST_LEDX_OFF;
#endif


#if( MU_433_TX_TESTING  )

	Test433tx();

#endif


#if( HPT4_MODE )
#else
	VLF_StartReceiver();				// This STATEMENT MUST BE CALLED TO START THE VLF
#endif

	LED_GREEN = TEST_LEDX_OFF;		// LED Off


//#if( HPT4_MODE )
//	pager_vlf_state = PAGER_VLF;
//#endif

/*
if( RA0 == 0 )
	TEST_LED_PCB = 0;
else
	TEST_LED_PCB = 1;
*/


	//-----------------------------------------------------------------------
	// Main Loop
	for( ; ; )
   	{

#if( MU_PAGER_TESTING  )

#else

#if( ( DEBUG_LEVEL > 1 )||( DEBUG_II )||( DEBUG_SU_UHF_RX ) )
#else




 #if( HPT3_MODE )

 #if( USE_LAMP_KEY )
		if( lampk_cnt )
		{  
			if( INP_IRIDIUM == 1 ) 	// Импульс закончился
			{
/*
				if(      lampk_cnt >= (9-1) )
				{	lamp_state =  LAMP_PAGER_ALARM;
					lamp_tick = 0;
					lampo_cnt = 4;
	        		lamp_timeout = get_timeout( LAMP_ALARM_STATE_TIMEOUT );
				}
				else
*/
				if( lampk_cnt >= (6-1) )
				{   // Импульс тестовый
					lamp_state =  LAMP_IDENTIFY;
					lamp_tick = 0;
					lampo_cnt = 4;
	        		lamp_timeout = get_timeout( 16 );    // На 16 тактов длительность
				}
				else 

				if( lampk_cnt >= (3-1) )
				{   lamp_state =  LAMP_NORMAL_STATE;
					mu_flags |= (FL_ACK_MANUAL|FL_ACK_FORCED);
					lamp_tick = 0;
					lampo_cnt = 0;
					lamp_timeout = get_timeout( 0 );
			   		LAMP_FET = LAMP_OFF;  // Off FET

				}

				lampk_cnt = 0;
				LED_GREEN = TEST_LEDX_OFF;		// LED Off
			}
		}

#endif


 #if( USE_IRIDIUM )
		if( lampk_cnt )
		{   if( INP_IRIDIUM == 1 ) 	// Импульс закончился
			{
				LED_GREEN = TEST_LEDX_OFF;		// LED Off

				if( lampk_cnt >= (6-1) )
				{   // Импульс запроса идентификатора
					suart_send_id();
				}
				else if( lampk_cnt >= (3-1) )
				{
					mu_flags |= (FL_ACK_MANUAL|FL_ACK_FORCED);
				}

				lampk_cnt = 0;
			}
		}
 #endif
		if( pager_cnt )
		{   if( SSP_PAGER_CS == 0 ) 	// Импульс закончился
			{   if( lamp_state >= LAMP_PAGER_ALARM )
		 		{
					if( pager_cnt >= (12-1) )
					{
						// Ошибочный импульс
					}
					else if( pager_cnt >= (9-1) )
					{	lamp_state =  LAMP_PAGER_ALARM;
#if( USE_LAMP_KEY )
	        	 		lamp_timeout = get_timeout( 32000 );
#else
	        	 		lamp_timeout = get_timeout( LAMP_ALARM_STATE_TIMEOUT );
#endif
						mu_flags |= FL_ACK;
					}
					else if( pager_cnt >= (6-1) )
					{	lamp_state =  LAMP_PAGER_CALL_1;
#if( USE_LAMP_KEY )
	        	 		lamp_timeout = get_timeout( 32000 );
#else
	        	 		lamp_timeout = get_timeout( LAMP_CALL_STATE_TIMEOUT );
#endif
						mu_flags |= FL_ACK;
					}
					else if( pager_cnt >= (3-1) )
					{	lamp_state =  LAMP_PAGER_CALL_2;
#if( USE_LAMP_KEY )
	        	 		lamp_timeout = get_timeout( 32000 );
#else
	        	 		lamp_timeout = get_timeout( LAMP_CALL_STATE_TIMEOUT );
#endif
						mu_flags |= FL_ACK;
					}

/*
					else
					{	// Ошибочный импульс
						lamp_state =  LAMP_POST_ERROR;
	        	 		lamp_timeout = get_timeout( LAMP_ALARM_STATE_TIMEOUT );
					}

					mu_flags |= FL_ACK;
*/

				}
				pager_cnt = 0;
			}
		}

 #elif( HPT2_MODE )       	// HPT2  Mode
		// Check to see if the pager bit is set in which case we start flashing the cap lamp
		if( PAGER_ALARM ==  0 )
	  	{ 	if( lamp_state >= LAMP_PAGER_ALARM )
		 	{	lamp_state =  LAMP_PAGER_ALARM;
         		lamp_timeout = get_timeout( LAMP_ALARM_STATE_TIMEOUT );
				mu_flags |= FL_ACK;
      		}
		}
 #endif
#endif
#endif


#if( HPT4_MODE )
#else
      	// Service any VLF changes in states
		if( vlf_state != VLF_STATE_IDLE )
		{   if( vlf_state == VLF_STATE_READBIT ) 	// A quarter bit was recorded
      		{   vlf_state = VLF_STATE_IDLE;			// Reset state to indicate it has been processed
       			if( VLF_ProcessBitRead() == TRUE ) 	// If packet ready
					VLF_ProcessPacket(); 			// A packet was received on the VLF interface
       		}
       		else
       		{
 #if( DEBUG_LEVEL >= 2  )
				TEST_LED4_VLF_TX = TEST_LEDX_OFF;
 #endif
				VLF_StartReceiver();	// Start VLF receiver and wait for packet and set event mode to VLF_STATE_IDLE
			}
		}
#endif

		// Check to see if a UHF frame is ready
		if( uhf.rxt_state == UHF_RX_COMPLETE )
		{  	if( ProcessUHFPacket() == RESULT_OK ) 				// Process the packet that was received on the UHF interface
			{   if( ++uhf.brx_pkt_cnt == 0 )
					uhf.brx_pkt_cnt++; 			// Heartbeat
#if( DEBUG_LEVEL )
				TEST_LED0_RX_CRC_ERROR = 1; 	// Off
#endif
			}

#if( DEBUG_LEVEL )
			else
			{
				TEST_LED0_RX_CRC_ERROR = 0;   // Rx CRC error splash
			}
			TEST_LED3_RX = TEST_LEDX_OFF; 	  // Rx Off
#endif

			uhf.rxt_state = UHF_RXTX_PREAMBLE;	// Reset packet RX and TX state to `Preamble`
		}

		// Check flag to see if a packet has been sent so we can revert back to RX mode on the UHF
		//		if( uhf_packet_sent )
		else if( uhf.rxt_state == UHF_TX_COMPLETE )
		{   // Return UHF to RX mode, enable receive interrupts
			SetUHFMode( UHF_RX_MODE );	// Reenable listening for packets
			CREN = 1;					// Reenable RECEIVER
			RCIE = 1;					// Reenable RX INTERRUPT
#if( DEBUG_LEVEL )
			TEST_LED1_TX = 1; 	// TX Led OFF
#endif

			uhf.rxt_state = UHF_RXTX_PREAMBLE;		// Set Packet RX to preamble
		}

        // A timer tick occurred
        if( timer_last_tick != timer.cnt_byte )
        {

			CLRWDT();

#if( ( DEBUG_II )||( DEBUG_SU_UHF_RX ) )
			if( ++timer_ii ==  30 )
			{  	timer_ii  = 0;
				LAMP_FET = LAMP_ON;   			// On

		TEST_LED_PCB = TEST_LEDX_ON;

			}
#endif


//#if( ENABLE_DEBUG_FLAGS	)
//	        if( ( timer.cnt_byte != 0 )&&( abs(timer.cnt_byte - timer_last_tick) > 1 ) )
//	        	dflags.f.b.missed_tick = 1;
//#endif
	        timer_last_tick = timer.cnt_byte;

#if( USE_MOBILITY )
			MobilityTickHandler();
#endif
	   		if( ( mu_flags & FL_REGISTRATION ) == 0 )
			{	if( chk_timeout( uhf_reset_timeout ) )
				{	uhf_reset_timeout = get_timeout( UHF_RESET_TIMEOUT );
					if( uhf.brx_pkt_cnt )
						uhf.brx_pkt_cnt = 0;
					else
	      				UHFCommsInit();
				}
			}
			else
			{	if( chk_timeout( uhf_registration_timeout ) )				//important for re_registation!
				{   char i;
					mu_flags &= (~(FL_REGISTRATION|FL_ACK_FORCED|FL_QUIET_PAGER) );
					//reregistration 
					//при перерегистрации забываем, что комбайнер
					//vrx.flags.aps.combiner = FALSE;
					for( i=0; i<BB_LIST_SIZE; i++ )
						bb[i].w = bi[i].w = 0;
      			}
			}

// Обработка флага ловушки 8 кГц
#if( USE_APS )
	if (vrx.flags.aps.enable==TRUE)
	{
		if( chk_timeout(vrx.aps_timeout) )
		{
			LED_GREEN = TEST_LEDX_OFF;
			vrx.state = VLF_RX_SEEKING;
			vrx.flags.aps.enable = FALSE;
			vrx.flags.aps.combiner = FALSE;
			mu_flags &=~(FL_8KHZ_TRAP);
			combine_id = 0;
		}
	}	
#endif



	if( lamp_state < LAMP_NORMAL_STATE )
	{
		if( lamp_state > LAMP_STATES_NUM )
			lamp_state = LAMP_NORMAL_STATE;

		if( chk_timeout( lamp_timeout ) )
			{ 	lamp_tick = 0; 		// Scheduler to start

#if( USE_LAMP_KEY )
				lampo_cnt = 0;
#endif
				if( lamp_state == LAMP_SEARCH_IN_PROGRESS )
				{	vtx.sid = 0;
					lamp_state = LAMP_NORMAL_STATE;
				}
				else
				if( lamp_state == LAMP_VLF_BLOCKED_STATE )
				{	if(  lamp_min_cnt )
					{ 	if( --lamp_min_cnt == 0 )
							lamp_state = LAMP_NORMAL_STATE;
						else
				       		lamp_timeout = get_timeout( LAMP_60SEC_TIMEOUT );
					}
				}
				else
					lamp_state = LAMP_NORMAL_STATE;

#if( USE_LAMP_KEY || USE_IRIDIUM )
			   		LAMP_FET = LAMP_OFF;  // Off FET
#elif( USE_INVERT_LAMP )
			   		LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON;	// On
#else
			   		LAMP_FET = LAMP_ON;											// On
#endif
				}

			}
#if( HPT4_MODE  )

			// В нормальном режиме смотрим за аварией
			else
   			{   if( PAGER_VLF )      // Инверсный!
				{	// Авария. Просто транслируем.
	   				LAMP_FET = LAMP_ON;
				}
				else
				{ 	LAMP_FET = LAMP_OFF;

				}
			}
#endif

	   	}

	}	// End Main Loop
}


//---------------------------------------------------------------------------
#if( USE_8KHZ_FILTER )
#define FILTER_8KHZ		(2) 	// 1-Hard 2-Soft
#else
#define FILTER_8KHZ		(0) 	// 0-Off
#endif

#define VRXDAT		 	(sizeof(vrx.preamble)-1)

//---------------------------------------------------------------------------
// The system interrupt handler
// Determines the cause of interrupt and dispatches the thread of control to that interrupt routine
//---------------------------------------------------------------------------
void interrupt int_handler(void)
{

#if( HPT4_MODE )

	// Поддержки приемника поиска в завале нет

#else
   	// VLF RX BIT INTERRUPT -------------------------------------------------
	if( TMR2IE && TMR2IF )
   	{   // Note: TMR2IE is only set if we have deliberately entered VLF receiver mode
		// TM2IE is only disabled when we send a response VLF packet - after which it should be reenabled.
		// This handler gets called as a result of the TMR2 interrupt. By examining the TMR0 counter, the number of
		// zero crossings of the RX signal for that 1/4 bit are determined. The number of zero crossings in a 1/4 determines
		// whether the 1/4 is a likely a 0 or a 1

		vrx.preamble[VRXDAT] = TMR0;
		TMR0 	= 0;
		TMR2IF 	= 0;

#if( MU_R8KHZ_TESTING )

		if( (vrx.preamble[VRXDAT] <= 21 )&&(vrx.preamble[VRXDAT] >= 19 ) )
			LED_GREEN = TEST_LEDX_ON;
		else
			LED_GREEN = TEST_LEDX_OFF;
#else

  #if( DEBUG_LEVEL >= 2 )
		if( vrx.preamble[VRXDAT] > 1 )
			TEST_LED6 = 0;		// On
		else
			TEST_LED6 = 1;		// Off
  #endif

		if( lamp_state == LAMP_TEST_STATE )
		{	if( vrx.preamble[VRXDAT] == 20 )
				LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON;  // On
			else
				LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_ON : LAMP_OFF;  // Off
		}

 #if( FILTER_8KHZ )
		if( vrx.preamble[VRXDAT] < 20 - FILTER_8KHZ )
		{	vrx.preamble[VRXDAT] = 0;

		}
		else
		if( vrx.preamble[VRXDAT] > 20 + FILTER_8KHZ )
		{
	  #if( FILTER_8KHZ >= 2 )
				vrx.preamble[VRXDAT] = 8;
	  #else
				vrx.preamble[VRXDAT] = 0;
	  #endif
		}

 #endif
//	LED_GREEN ^= 1;
		vlf_state = VLF_STATE_READBIT;	// Flag the event so that we do the
										// processing in the main thread so we don't miss interrupts
#endif
   	}

	else
#endif

   	// VLF TX BIT INTERRUPT -------------------------------------------------
	if( T0IE && T0IF ) 	   		// Note: TOIE is only ever turned on if we are in VLF TX mode
   	{  	TMR0 += TMR0_START;		// Finetune timing. Timer granularity is not good due to prescalar.


		// The VLF TX interrupt handler
		// This handler gets called as a result of the TMR0 interrupt. The essence of the function is that PWM pin
		// is turned on and off, depending on whether a zero or a one must be transmitted.
		// The handler goes through a number of states to determine where it is in the packet


		switch( vtx.state )
		{
		case VLF_TX_PACKET_PREAMBLE:

#if( MU_VLF_TX_TESTING == 3 )
			CCP1CON |=  0x0C;
			CLRWDT();

#elif( MU_VLF_TX_TESTING == 4 )
			vtx.cnt ^= 1;

			if( vtx.cnt & 0x01 )
			{
				CCP1CON |=  0x0C;
			}
			else
			{  	TX_PWM = 0;      	// Turn off TX
				CCP1CON &= ~0x0C;
			}
			CLRWDT();

#else
			if( mu2su_preamble[vtx.cnt] == 1 )
				CCP1CON |=  0x0C;
			else
			{  	TX_PWM = 0;      	// Turn off TX
				CCP1CON &= ~0x0C;
			}
			if( ++vtx.cnt == VLF_MU_TO_SU_PREAMBLE_SIZE )
			{ 	vtx.cnt = MU_PROTOCOL2_RESPONSE_PAYLOADSIZE + 1;
				vtx.shifter = 0x01;
				vtx.idx = 0;
				vtx.state = VLF_TX_PACKET_DATA;
			}
#endif
		   	break;

	   case VLF_TX_PACKET_DATA:

#if( MU_VLF_TX_TESTING ==  2 )
			CLRWDT();
			CCP1CON |=  0x00C;
#else
			if( vtx.u.data[vtx.idx] & vtx.shifter )
				CCP1CON |=  0x00C;
			else
			{ 	TX_PWM = 0;    		// Turn off TX
				CCP1CON &= ~0x0C;
			}

			if( vtx.shifter < 0x80 )
				vtx.shifter <<= 1;
			else
			{	vtx.shifter = 0x01;
				vtx.idx++;
				if( !( --vtx.cnt ) )
					vtx.state = VLF_TX_PACKET_DONE;
			}
#endif
	   		break;

	   case VLF_TX_PACKET_DONE:
		   	CCP1CON &= ~0x0C;
		   	TX_PWM 	= 0;            // Turn off TX
		   	T0IE 	= 0;			// Packet done so disable Timer0 interrupt
		   	vtx.state = VLF_TX_IDLE;
		   	vlf_state = VLF_STATE_PACKET_TRANSMITTED;

			break;
		}
      	T0IF = 0;
	}	// End OF if( T0IF && T0IE )


   	// UHF RX CHAR INTERRUPT ------------------------------------------------
	// Note RCIE, the UHF UART RX character-ready-interrupt is only ever enabled if we are in UHF receive mode - it is disabled when sending
	// this forces it to act in half duplex
	// once a packet has been received, the interrupt is disabled until the packet has been processed
	// NOTE: RCIF is read only and is only cleared by reading the RCREG
	if( RCIE && RCIF )
  	{
// debug for counting characters so you see how long it takes to reach the limit using a breakpoint
// tests show that we get about 2800 characters a second [every 200 usec] so this routine has to be very fast
//	  	static ushort debug_uhf_count = 0;
//		debug_uhf_count++;
//		if (debug_uhf_count > 50000)
//			debug_uhf_count = 0;
// end of debug character counter




		{
		register BYTE uhf_char = RCREG;	// Read the char and also clear the buffer and clear RCIF

	 	if( FERR || OERR )		// Ignore any characters with framing or overrun errors
	  	{	if( OERR )
	    	{  	CREN = 0;
	           	NOP();
	       		CREN = 1;
			}

			if( uhf.rxt_state >= UHF_RX_HDR )
			{	// Unlock AVG filter and start again
				UnlockUHFThresh();
				uhf.idx = 0;
				if( uhf.rxt_state != UHF_RX_COMPLETE )
					uhf.rxt_state = UHF_RXTX_PREAMBLE;
			}
	  	}
	  	else 	// NO OVERRUN OR FRAMING ERROR WITH CHARACTER SO PROCESS IT - even if we are busy on the VLF
	  	{

			switch( uhf.rxt_state )
		    {
		    case UHF_RXTX_PREAMBLE:
				if( uhf_char == UHF_PREAMBLE_CHAR )
			    {   if( ++uhf.idx == UHF_PREMABLE_RX_THRESHOLD )
			        {	// Valid preamble, lock the signal threshold
			            LockUHFThresh();	// Locks the symbol threshold of the CC1000 communications transceiver.
			            uhf.idx = 0;
//#if( ENABLE_DEBUG_FLAGS )
//						dflags.f.b.got_uhf_preamble = 1;
//#endif
			            uhf.rxt_state = UHF_RXTX_RESYNC;

#if( DEBUG_SU_UHF_RX == 2 )

		LAMP_FET ^= 1;
		TEST_LED_PCB ^= 1;
		timer_ii = 0;

#elif( DEBUG_SU_UHF_RX == 1 )

			TEST_LED_PCB ^= 1;
#endif

			         }
			     }
				 else
				     uhf.idx = 0;
		      	break;

		    case UHF_RXTX_RESYNC:
				if( uhf_char == UHF_STX_CHAR )
			    {	uhf.pkt.data[0] = UHF_STX_CHAR;
					uhf.idx = 1;
//#if( ENABLE_DEBUG_FLAGS )
//					dflags.f.b.got_uhf_stx = 1;
//#endif
					uhf.rxt_state 	= UHF_RX_HDR;	// Move on to next stage of building packet

			  	}
			    else if( ++uhf.idx == UHF_UART_STX_THRESHOLD )
			   	{ 	// No STX char, unlock the threshold and start again
				  	UnlockUHFThresh();
				    uhf.idx = 0;
				    uhf.rxt_state = UHF_RXTX_PREAMBLE;
			 	}
		 		break;

			case UHF_RX_HDR:
				// If we've received a enough chars for the corresponding packet, check header.
				uhf.pkt.data[uhf.idx++] = uhf_char;
			    if( uhf.idx == 4 )
				{
					uhf.len = get_pkt_len();
			        if( uhf.len == 0xFF ) 	// If packet type is invalid, reset back to looking for preamble
				  	{ 	UnlockUHFThresh();
			            uhf.idx = 0;
			            uhf.rxt_state = UHF_RXTX_PREAMBLE;
			      	}
					else
			            uhf.rxt_state = UHF_RXTX_PKT;
				}
				break;

		  	case UHF_RXTX_PKT:
				uhf.pkt.data[uhf.idx++] = uhf_char;
			    if( uhf.idx >= uhf.len )
				{
#if( DEBUG_LEVEL )
					TEST_LED3_RX = TEST_LEDX_ON;  	// Rx On/splash
#endif
					CREN = 0;						// Disable UART receiver
					RCIE = 0;						// Disable UART RX interrupt
			        uhf.idx = 0;
			   		UnlockUHFThresh();
					uhf.rxt_state = UHF_RX_COMPLETE;
			    }
		      	break;

			case UHF_RX_COMPLETE:

				break;

		  	default:
//#if( ENABLE_DEBUG_FLAGS )
//				dflags.f.b.invalid_rx_uhf_state = 1;
//#endif
			   	UnlockUHFThresh();
			    uhf.idx = 0;
				uhf.rxt_state = UHF_RXTX_PREAMBLE;
		      	break;
			}

		}
		}
   	}   // End of if( RCIE && RCIF )

   	// UHF TX buffer ready interrupt ----------------------------------------
	else if( TXIE && TXIF )
	{  	// Note TXIE, the UHF UART TX interrupt is only ever enabled if there is a packet in the process of being sent
		// once the packet has been completely sent, this interrupt is disabled
		// Check to see if any UHF characters need sending
		switch( uhf.rxt_state )
		{
		case UHF_RXTX_PREAMBLE:
		    TXREG = UHF_PREAMBLE_CHAR;
		    if( ++uhf.idx == UHF_PREAMBLE_SIZE )
		    {   uhf.idx = 0;
		       	uhf.rxt_state = UHF_RXTX_RESYNC;
		    }
			break;

		case UHF_RXTX_RESYNC:
		 	TXREG = UHF_UART_RESYNC_CHAR;
		    if( ++uhf.idx == UHF_UART_RESYNC_SIZE )
		  	{   uhf.len = get_pkt_len();
		        uhf.idx = 0;
		  		uhf.rxt_state = UHF_RXTX_PKT;
		 	}
			break;

		case UHF_RXTX_PKT:
		 	TXREG = uhf.pkt.data[uhf.idx++];
		    if( uhf.idx >= uhf.len )
		 	{ 	uhf.idx = 0;
		 	 	uhf.rxt_state = UHF_TX_TRAILER;
		 	}
			break;

		case UHF_TX_TRAILER:
		    TXREG = UHF_TRAILER_CHAR;
		    if( ++uhf.idx == UHF_TRAILER_SIZE )
		   		uhf.rxt_state = UHF_TX_END;
		    break;

		case UHF_TX_END:
			TXEN = 0;		// If done sending then disable UHF UART TX
		    TXIE = 0;		// Disable UHF UART TX interrupt
		    uhf.idx = 0;
		    uhf.rxt_state = UHF_TX_COMPLETE;
			break;

		}
		TXIF = 0;
	}	// End of if( TXIE && TXIF )

   	// Timer1 tick interrupt ------------------------------------------------
	if( TMR1IF && TMR1IE )
   	{
		timer.cnt++;

#if( HPT3_MODE )
		if( SSP_PAGER_CS )
			++pager_cnt;
#endif


#if( USE_LAMP_KEY )
		if( lampo_cnt == 0 )
		{  
			//LED_GREEN = LED_GREEN ^= 1;
			if( INP_IRIDIUM == 0 )
			{  
				//LED_GREEN = LED_GREEN ^= 1;
				//LED_GREEN = TEST_LEDX_ON;
				++lampk_cnt;
			}
		}
#endif


#if( USE_IRIDIUM )
		if( suart_send_process() )
		{  	if( ( INP_IRIDIUM == 0 )&&( lamp_state == LAMP_NORMAL_STATE ) )
			{   LED_GREEN = TEST_LEDX_ON;
				++lampk_cnt;
			}
		}
#endif

		// Cap Lamp flashing - high priority
		if( lamp_state < LAMP_NORMAL_STATE )
		{	if( ( lamp_schedule[lamp_state][(lamp_tick >> 3) & LAMP_SCHEDULE_MASK]	) &
					( 1 << ( lamp_tick & 0x7 ) ) )
#if( USE_LAMP_KEY )
			{	LAMP_FET = LAMP_OFF;   // FET Off
	   			if( lampo_cnt )
					lampo_cnt--;
			}
			else
			{   LAMP_FET = LAMP_ON;
				lampo_cnt = 4;   	// Счетчик активности выхода лампы вне фазы активности производится чтение
									// входа лампы
			}
#elif( USE_IRIDIUM )
			{
				LAMP_FET = LAMP_OFF;   // FET Off
			}
			else
			{
				LAMP_FET = LAMP_ON;
			}
#elif( USE_INVERT_LAMP )
				LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_OFF : LAMP_ON;	// On
			else
				LAMP_FET = ( mu_flags & FL_LAMP_INV ) ? LAMP_ON : LAMP_OFF;	// Off

#else
				LAMP_FET = LAMP_ON;		// On
			else
        		LAMP_FET = LAMP_OFF;  	// Off
#endif

			lamp_tick++;
		}


#if( MU_PAGER_TESTING  )
		if( SSP_PAGER_CS )
		{
       		LAMP_FET ^= 1;
		}
#endif

/*
if( RA0 == 0 )
	TEST_LED_PCB = 0;
else
	TEST_LED_PCB = 1;
*/


      	TMR1IF = 0;		// Clear Timer interrupt flag
   	}
}

