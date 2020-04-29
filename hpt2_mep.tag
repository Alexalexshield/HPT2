ÄÄ´ mobility.c    ÃÄÄ´ 08-12-16 14:39 ÃþDT=1233679611þPATH=D:\Work_PIC\HPT2\Application\src\
mob_history	BYTE_static_[]	static BYTE     mob_history[MOBILITY_HISTORY_SIZE] = { 0, 0, 0, 0, 0, 0 };
mob_cnt	BYTE_static	static BYTE   	mob_cnt = 0;
mob_min	BYTE_static	static BYTE 	mob_min;
mob_max	BYTE_static	static BYTE 	mob_max;
MeasureMobility	void_proc	void MeasureMobility(void)
GetMobilityValue	BYTE_proc	BYTE GetMobilityValue( void )
mobility_countdown_ticks	BYTE_static	static BYTE mobility_countdown_ticks = MOBILITY_WAKE_UP_DELAY_BEFORE_MEASUREMENTS; // schedule first mobility measurement
MobilityTickHandler	void_proc	void MobilityTickHandler( void )
 þ
ÄÄ´ vlf_comms2.c  ÃÄÄ´ 27-07-18 13:06 ÃþDT=1291544771þPATH=D:\Work_PIC\HPT2\Application\src\
vtx	bank1	bank1 Pkt_tx_buf_st vtx;
vlf_state	BYTE_volatile	volatile BYTE vlf_state = VLF_STATE_IDLE;	// Set to no VLF event
su2mu_preamble	BYTE_const_[]	const BYTE su2mu_preamble[VLF_SU_TO_MU_PREAMBLE_SIZE] = {
mu2su_preamble	BYTE_const_[]	const BYTE mu2su_preamble[VLF_MU_TO_SU_PREAMBLE_SIZE] = {
uncoded_rx_data_size	BYTE_const_[]	const BYTE uncoded_rx_data_size[] = {
get_crc	BYTE_static_proc	static BYTE get_crc( BYTE crc, BYTE data )
buffcrc	BYTE_static_proc	static BYTE buffcrc( BYTE *ptr, BYTE crc, uchar len )
if	if_proc	if( vtx.type >= VLF_NO_PACKET  )
crc	crc	crc = get_crc( SU_TO_MU_PREAMBLE_CRC, (vtx.type << 4) );
crc	crc	crc = buffcrc( &vtx.u.data[0], crc, UNCODED_RX_PAYLOAD_SIZE( vtx.type ) - 1 );
crc	crc	crc = get_crc( crc, 0x00 );
if	if_proc	if( vtx.u.data[UNCODED_RX_DATA_SIZE( vtx.type )] != crc )
switch	switch_proc	switch( vtx.type )
VLF_ProcessBitRead	char_proc	char VLF_ProcessBitRead(void)
VLF_ReplyToBroadcastRequest	void_proc	void VLF_ReplyToBroadcastRequest( BYTE intervals )
VLF_ReplyLocateRequest	void_proc	void VLF_ReplyLocateRequest( uchar locate_seconds )
VLF_SendPacket	void_static_proc	static void VLF_SendPacket(void)
VLF_SendGeneric	void_proc	void VLF_SendGeneric(void)
TestVlfTx	void_proc	void TestVlfTx( ushort times )
 þ
ÄÄ´ mobility.h    ÃÄÄ´ 10-06-13 12:29 ÃþDT=1120560032þPATH=D:\Work_PIC\HPT2\Application\include\
MOBILITY_H	#define	#define MOBILITY_H
MOBILITY_MEASUREMENTS_PER_SECOND	#define	#define  MOBILITY_MEASUREMENTS_PER_SECOND 	 3
MOBILITY_30SEC_OF_MEASUREMENTS	#define	#define  MOBILITY_30SEC_OF_MEASUREMENTS 	 ( 30*MOBILITY_MEASUREMENTS_PER_SECOND )
MOBILITY_MEASURE_PERIOD	#define	#define  MOBILITY_MEASURE_PERIOD             (TICKS_PER_SEC/MOBILITY_MEASUREMENTS_PER_SECOND)    // The time between alive senso
MOBILITY_WAKE_UP_DELAY_BEFORE_MEASUREMENTS	#define	#define  MOBILITY_WAKE_UP_DELAY_BEFORE_MEASUREMENTS (TICKS_PER_SEC*5)
ACCELEROMETER_NOISE_FLOOR	#define	#define  ACCELEROMETER_NOISE_FLOOR  0x10	// Noise floor based on measurments on one of the units
MOBILITY_HISTORY_SIZE	#define	#define  MOBILITY_HISTORY_SIZE 		6
 þ
ÄÄ´ uhf_comms.h   ÃÄÄ´ 02-10-14 11:20 ÃþDT=1161976452þPATH=D:\Work_PIC\HPT2\Application\include\
__UHF_COMMS_H	#define	#define __UHF_COMMS_H
REGISTRATION_TIMEOUT	#define	#define REGISTRATION_TIMEOUT    		(  10*(TICKS_PER_SEC))
UHF_RESET_TIMEOUT	#define	#define UHF_RESET_TIMEOUT  				(2*60*(TICKS_PER_SEC))
uhf_registration_timeout	ushort	extern ushort  uhf_registration_timeout;
uhf_bonded_bid	short_u	extern short_u uhf_bonded_bid;
uhf_last_bid	short_u	extern short_u uhf_last_bid;
uhf	bank2	extern bank2 Uhf_pkt_s 	uhf;
BB_LIST_SIZE	#define	#define BB_LIST_SIZE 	(8)
BB_LIST_MASK	#define	#define BB_LIST_MASK 	( (BB_LIST_SIZE) - 1 )
bb	bank3_[]	extern bank3 bid_u	bb[BB_LIST_SIZE];		// Bounded beacons ID list
bi	bank3_[]	extern bank3 bid_u	bi[BB_LIST_SIZE];		//
 þ
ÄÄ´ vlf_comms.h   ÃÄÄ´ 17-06-15 17:50 ÃþDT=1188138585þPATH=D:\Work_PIC\HPT2\Application\include\
VLF_COMMS_H__	#define	#define VLF_COMMS_H__
su2mu_preamble	const_[]	extern const BYTE su2mu_preamble[VLF_SU_TO_MU_PREAMBLE_SIZE];
mu2su_preamble	const_[]	extern const BYTE mu2su_preamble[VLF_MU_TO_SU_PREAMBLE_SIZE];
vtx	bank1	extern bank1 struct Pkt_tx_buf_st  vtx;
vrx	bank1	extern bank1 Pkt_rx_buf_st vrx;
vlf_state	volatile	extern volatile BYTE vlf_state;
TMR2_PRESC_RX	#define	#define  TMR2_PRESC_RX  	1              // The TMR2 prescaler in RX mode
TMR2_PRESC_TX	#define	#define  TMR2_PRESC_TX      0              // The TMR2 prescaler in TX mode
TMR2_POSC	#define	#define  TMR2_POSC          4              // The TMR2 postscaler
PR2_RX	#define	#define  PR2_RX             ((SYS_FREQ/(1<<(2*TMR2_PRESC_RX))/(TMR2_POSC+1)/(VLF_RX_SYMBOL_FREQUENCY)/(VLF_NUM_RX_SYMBOL_SUBPERI
PR2_TX	#define	#define  PR2_TX             ((SYS_FREQ/(1<<(2*TMR2_PRESC_TX))/(VLF_TX_FREQUENCY)) - 1) 	// The PR2 value in TX mode
TX_FULL_POWER	#define	#define  TX_FULL_POWER      (SYS_FREQ/(1<<(2*TMR2_PRESC_TX))*4/(VLF_TX_FREQUENCY)/2) 	// The full TX power duty cycle
TMR0_PRESC	#define	#define  TMR0_PRESC                          5              // The TMR0 prescaler
TMR0_START	#define	#define  TMR0_START                          ((BYTE)((ulong)257 - (ulong)SYS_FREQ/(ulong)VLF_TX_SYMBOL_FREQUENCY/((ulong)1<<((ul
VLF_STATE	enum_type	enum VLF_STATE {
VLF_STATE_IDLE	enum_item	enum VLF_STATE {
VLF_STATE_READBIT	enum_item	enum VLF_STATE {
VLF_STATE_PACKET_TRANSMITTED	enum_item	enum VLF_STATE {
VLF_RX_STATES	enum_type	enum VLF_RX_STATES {
VLF_RX_SEEKING	enum_item	enum VLF_RX_STATES {
VLF_RX_LOCKED	enum_item	enum VLF_RX_STATES {
VLF_RX_RECEIVING	enum_item	enum VLF_RX_STATES {
VLF_PACKET_READY	enum_item	enum VLF_RX_STATES {
TX_STATE	enum_type	enum TX_STATE {
VLF_TX_IDLE	enum_item	enum TX_STATE {
VLF_TX_LOCATE	enum_item	enum TX_STATE {
VLF_TX_PACKET_PREAMBLE	enum_item	enum TX_STATE {
VLF_TX_PACKET_TYPE	enum_item	enum TX_STATE {
VLF_TX_PACKET_DATA	enum_item	enum TX_STATE {
VLF_TX_PACKET_DONE	enum_item	enum TX_STATE {
sid	bank1	extern bank1 BYTE sid;
VLF_START_SCORE	#define	#define VLF_START_SCORE 	0xFFFF
SEARCH_ID_INTERVALS	#define	#define SEARCH_ID_INTERVALS 	10
SEARCH_QUICK_INTERVALS	#define	#define SEARCH_QUICK_INTERVALS 	5
QUICKSEARCH_RX_DELAY_MS	#define	#define QUICKSEARCH_RX_DELAY_MS ((3*85)+20)
DETECT_RX_DELAY_MS	#define	#define DETECT_RX_DELAY_MS 		((5*85)+20)
LOCATE_RX_DELAY_MS	#define	#define LOCATE_RX_DELAY_MS  	((7*85)+20)
MASK_RX_DELAY_MS	#define	#define MASK_RX_DELAY_MS 		((8*85)+20)
 þ
ÄÄ´ cc1000.c      ÃÄÄ´ 08-12-16 15:01 ÃþDT=1233680440þPATH=D:\Work_PIC\HPT2\Common\src\
c_uhf_modes_ast	struct_[]	} c_uhf_modes_ast[] = {
#if	struct_proc	#if( DEBUG_SU_UHF_RX )
{	struct	{ 0x11, 0x44, 0x60, 0 },
0x11	struct	{ 0x11, 0x44, 0x60, 0 },    		// RX mode settings  (Changed current from 0x40 to 0x44 - Datasheet & Smart RF Studio recomm
0x44	struct	{ 0x11, 0x44, 0x60, 0 },    		// RX mode settings  (Changed current from 0x40 to 0x44 - Datasheet & Smart RF Studio recomm
0x60	struct	{ 0x11, 0x44, 0x60, 0 },    		// RX mode settings  (Changed current from 0x40 to 0x44 - Datasheet & Smart RF Studio recomm
}	struct	{ 0x11, 0x44, 0x60, 0 },    		// RX mode settings  (Changed current from 0x40 to 0x44 - Datasheet & Smart RF Studio recomm
#if	struct_proc	#if( MU_433_TX_TESTING  )
#elif	struct_proc	#elif( HPT4_MODE )
0xE1	struct	{ 0xE1, 0x81, 0x48, 0x0F }         	// TX mode settings  +0dB
0x81	struct	{ 0xE1, 0x81, 0x48, 0x0F }         	// TX mode settings  +0dB
0x48	struct	{ 0xE1, 0x81, 0x48, 0x0F }         	// TX mode settings  +0dB
}	struct	};
cc1000_wr	void_proc	void cc1000_wr( BYTE addr_u8, BYTE val_u8 )
cc1000_rd	BYTE_proc	BYTE cc1000_rd( BYTE addr_u8 )
pager_write	void_proc	void pager_write( BYTE value )
InitUHFTranceiver	BYTE_proc	BYTE InitUHFTranceiver( void )
uhf_mode_u8	BYTE_static	static BYTE uhf_mode_u8 = UHF_RX_MODE;
GetUHFMode	BYTE_proc	BYTE GetUHFMode(void)
SetUHFMode	BYTE_proc	BYTE SetUHFMode( BYTE mode_u8 )
LockUHFThresh	void_proc	void LockUHFThresh(void)
UnlockUHFThresh	void_proc	void UnlockUHFThresh(void)
 þ
ÄÄ´ init30a.c     ÃÄÄ´ 22-08-17 13:23 ÃþDT=1259760371þPATH=D:\Work_PIC\HPT2\Common\src\
init_board	void_proc	void init_board( void )
ReadADChannel	BYTE_proc	BYTE ReadADChannel( BYTE ch )
 þ
ÄÄ´ time.c        ÃÄÄ´ 10-06-13 12:29 ÃþDT=1120560032þPATH=D:\Work_PIC\HPT2\Common\src\
timer	Timer_u_volatile	volatile Timer_u timer;		// The global tick timer counter
timer_last_tick	BYTE	BYTE timer_last_tick;
timer_ii	BYTE	BYTE timer_ii;
chk_timeout	BYTE_proc	BYTE chk_timeout( ushort counter )
 þ
ÄÄ´ uhf_cmn.c     ÃÄÄ´ 08-12-16 15:01 ÃþDT=1233680440þPATH=D:\Work_PIC\HPT2\Common\src\
pkt_len_au	short_u_static_const_[]	static const short_u pkt_len_au[] = {
pkt_ctl_len_au	short_u_static_const_[]	static const short_u pkt_ctl_len_au[] = {
UHFCommsInit	BYTE_proc	BYTE UHFCommsInit(void)
CalcUHFChecksum	BYTE_proc	BYTE CalcUHFChecksum( uchar set_flag )
get_pkt_len	BYTE_proc	BYTE get_pkt_len(void)
 þ
ÄÄ´ wait.c        ÃÄÄ´ 02-03-18 14:56 ÃþDT=1281521433þPATH=D:\Work_PIC\HPT2\Common\src\
@0x00E	short_static_volatile_unsigned	static volatile unsigned short TMR1  @ 0x00E;
tmr_s	typedef_struct_type	typedef struct tmr_s {
tmr_s	typedef_struct	} tmr_s;
tmr_u	typedef_union_type	typedef union tmr_u {
tmr_u	typedef_union	} tmr_u;
delay_uhf_ts	void_proc	void delay_uhf_ts( BYTE ts )
delay_ms	void_proc	void delay_ms( ushort ms )
XTAL_FREQ	#define	#define	XTAL_FREQ		8		// Crystal frequency in MHz
XTAL_FREQ_REF	#define	#define	XTAL_FREQ_REF	12
MAXIMUM_DELAYUS	#define	#define MAXIMUM_DELAYUS 400
Wait250Us	void_proc	void Wait250Us()
Wait25Us	void_proc	void Wait25Us(void)
UART_TX	#define	#define  UART_TX	LAMP_FET
OUT_HI	#define	#define  OUT_HI		LAMP_OFF
OUT_LO	#define	#define  OUT_LO		LAMP_ON
SUART_STOP	#define	#define  SUART_STOP		(0xF0)
byte_cnt	BYTE	BYTE byte_cnt = 0;
byte_bit_cnt	BYTE	BYTE byte_bit_cnt = 0;
UART_TX	#define	#define  UART_TX	LAMP_FET
OUT_HI	#define	#define  OUT_HI		LAMP_OFF
OUT_LO	#define	#define  OUT_LO		LAMP_ON
suart_send_id	void_proc	void suart_send_id( void )
suart_send_process	BYTE_proc	BYTE suart_send_process( void )
 þ
ÄÄ´ cc1000.h      ÃÄÄ´ 10-06-13 12:29 ÃþDT=1120560032þPATH=D:\Work_PIC\HPT2\Common\include\
__CC1000_H	#define	#define __CC1000_H
CC1000_MAIN	#define	#define CC1000_MAIN               0    // MAIN Register
CC1000_FREQ_2A	#define	#define CC1000_FREQ_2A            0x01 // Frequency Register 2A
CC1000_FREQ_1A	#define	#define CC1000_FREQ_1A            0x02 // Frequency Register 1A
CC1000_FREQ_0A	#define	#define CC1000_FREQ_0A            0x03 // Frequency Register 0A
CC1000_FREQ_2B	#define	#define CC1000_FREQ_2B            0x04 // Frequency Register 2B
CC1000_FREQ_1B	#define	#define CC1000_FREQ_1B            0x05 // Frequency Register 1B
CC1000_FREQ_0B	#define	#define CC1000_FREQ_0B            0x06 // Frequency Register 0B
CC1000_FSEP1	#define	#define CC1000_FSEP1              0x07 // Frequency Separation Register 1
CC1000_FSEP0	#define	#define CC1000_FSEP0              0x08 // Frequency Separation Register 0
CC1000_CURRENT	#define	#define CC1000_CURRENT            0x09 // Current Consumption Control Register
CC1000_FRONT_END	#define	#define CC1000_FRONT_END          0x0A // Front End Control Register
CC1000_PA_POW	#define	#define CC1000_PA_POW             0x0B // PA Output Power Control Register
CC1000_PLL	#define	#define CC1000_PLL                0x0C // PLL Control Register
CC1000_LOCK	#define	#define CC1000_LOCK               0x0D // LOCK Status Register and signal select to CHP_OUT (LOCK) pin
CC1000_CAL	#define	#define CC1000_CAL                0x0E // VCO Calibration Control and Status Register
CC1000_MODEM2	#define	#define CC1000_MODEM2             0x0F // Modem Control Register 2
CC1000_MODEM1	#define	#define CC1000_MODEM1             0x10 // Modem Control Register 1
CC1000_MODEM0	#define	#define CC1000_MODEM0             0x11 // Modem Control Register 0
CC1000_MATCH	#define	#define CC1000_MATCH              0x12 // Match Capacitor Array Control Register for RX and TX impedance matching
CC1000_FSCTRL	#define	#define CC1000_FSCTRL             0x13 // Frequency Synthesiser Control Register
CC1000_PRESCALER	#define	#define CC1000_PRESCALER          0x1C // Prescaler and IF-strip test control register
CC1000_TEST6	#define	#define CC1000_TEST6              0x40 // Test register for PLL LOOP
CC1000_TEST5	#define	#define CC1000_TEST5              0x41 // Test register for PLL LOOP
CC1000_TEST4	#define	#define CC1000_TEST4              0x42 // Test register for PLL LOOP (must be updated as specified)
CC1000_TEST3	#define	#define CC1000_TEST3              0x43 // Test register for VCO
CC1000_TEST2	#define	#define CC1000_TEST2              0x44 // Test register for Calibration
CC1000_TEST1	#define	#define CC1000_TEST1              0x45 // Test register for Calibration
CC1000_TEST0	#define	#define CC1000_TEST0              0x46 // Test register for Calibration
UHF_MODES	enum_type	enum UHF_MODES {
UHF_RX_MODE	enum_item	enum UHF_MODES {
UHF_TX_MODE	enum_item	enum UHF_MODES {
 þ
ÄÄ´ init30a.h     ÃÄÄ´ 21-08-17 15:39 ÃþDT=1259699430þPATH=D:\Work_PIC\HPT2\Common\include\
__INIT30A_H	#define	#define __INIT30A_H
PAGER_VLF	#define	#define  PAGER_VLF                      RA0
LED_GREEN	#define	#define  LED_GREEN                   	RB0
UHF_LOCK	#define	#define  UHF_LOCK                       RB1
SSP_PAGER_CS	#define	#define  SSP_PAGER_CS                   RB2
LF_TX_SIG	#define	#define  LF_TX_SIG                      RB3
UHF_RSSI	#define	#define  UHF_RSSI                       RB4
UHF_PALE	#define	#define  UHF_PALE                       RC0
LAMP_FET	#define	#define  LAMP_FET	                    RC1
TX_PWM	#define	#define  TX_PWM                         RC2
SSP_CLK	#define	#define  SSP_CLK                        RC3
SSP_SDI	#define	#define  SSP_SDI                        RC4
SSP_SDO	#define	#define  SSP_SDO                        RC5
TEST_POINT30	#define	#define  TEST_POINT30                   RB5
TEST_POINT25	#define	#define  TEST_POINT25                   RA3
TEST_LEDX_OFF	#define	#define TEST_LEDX_OFF 	1
TEST_LEDX_ON	#define	#define TEST_LEDX_ON 	0
TEST_LEDX_OFF	#define	#define TEST_LEDX_OFF 	1
TEST_LEDX_ON	#define	#define TEST_LEDX_ON 	0
INP_IRIDIUM	#define	#define INP_IRIDIUM                   	TEST_POINT30
TEST_LED4_VLF_TX	#define	#define  TEST_LED4_VLF_TX              	RB7
TEST_LED5_VLF_RX	#define	#define  TEST_LED5_VLF_RX              	SSP_PAGER_CS
TEST_LED6	#define	#define  TEST_LED6              		RB6
TEST_LED3_RX	#define	#define  TEST_LED3_RX                  	LAMP_FET
TEST_LEDX_OFF	#define	#define TEST_LEDX_OFF 	1
TEST_LEDX_ON	#define	#define TEST_LEDX_ON 	0
 þ
ÄÄ´ time.h        ÃÄÄ´ 10-06-13 12:29 ÃþDT=1120560032þPATH=D:\Work_PIC\HPT2\Common\include\
TIME_H	#define	#define TIME_H
SYS_FREQ	#define	#define  SYS_FREQ                            2000000 	// The system frequency in Hz
TICKS_PER_SEC	#define	#define  TICKS_PER_SEC                       15.2588    // The number of clock ticks per second
Timer_u	typedef_union	} Timer_u;
timer	volatile	extern volatile Timer_u  timer;
timer_last_tick	BYTE	extern BYTE timer_last_tick;
timer_ii	BYTE	extern BYTE timer_ii;
get_timeout	#define	#define get_timeout(x) 	(timer.cnt + (x))
 þ
ÄÄ´ types.h       ÃÄÄ´ 10-06-13 12:29 ÃþDT=1120560032þPATH=D:\Work_PIC\HPT2\Common\include\
__TYPES_H	#define	#define __TYPES_H
ushort	typedef	typedef unsigned short ushort;      // A 16 bit unsigned value
ulong	typedef	typedef unsigned long ulong;
BYTE	typedef	typedef unsigned char BYTE;        	// An 8 bit unsigned value
uchar	typedef	typedef unsigned char uchar;
BIT	#define	#define BIT(n)           ( 1U <<   (n) )
SETBIT	#define	#define SETBIT( p, n )   ( (p)|= BIT(n) )
CLRBIT	#define	#define CLRBIT( p, n )   ( (p)&=~BIT(n) )
XORBIT	#define	#define XORBIT( p, n )   ( (p)=(p)^BIT(n) )
TSTBIT	#define	#define TSTBIT( p, n )   ( (p)&  BIT(n) )
BIT0	#define	#define BIT0		(BIT(0))
BIT1	#define	#define BIT1		(BIT(1))
BIT2	#define	#define BIT2		(BIT(2))
BIT3	#define	#define BIT3		(BIT(3))
BIT4	#define	#define BIT4		(BIT(4))
BIT5	#define	#define BIT5		(BIT(5))
BIT6	#define	#define BIT6		(BIT(6))
BIT7	#define	#define BIT7		(BIT(7))
short_u	typedef_union	} short_u;
bid_u	typedef_union	} bid_u;
long_u	typedef_union	} long_u;
TRUE	#define	#define	TRUE	(1)
FALSE	#define	#define	FALSE	(0)
RESULT_OK	#define	#define RESULT_OK	(0)
mid24	typedef_struct_type	typedef struct mid24
mid24_s	typedef_struct	} mid24_s;
NTOHS	#define	#define NTOHS( x )	( ( (x) >> 8 )|( (x) << 8 ) )
 þ
ÄÄ´ uhf_cmn.h     ÃÄÄ´ 10-06-13 12:29 ÃþDT=1120560032þPATH=D:\Work_PIC\HPT2\Common\include\
UHF_CMN_H	#define	#define UHF_CMN_H
UHF_RXTX_STATES	enum_type	enum UHF_RXTX_STATES  {
UHF_RXTX_PREAMBLE	enum_item	enum UHF_RXTX_STATES  {
UHF_RXTX_RESYNC	enum_item	enum UHF_RXTX_STATES  {
UHF_RX_HDR	enum_item	enum UHF_RXTX_STATES  {
UHF_RXTX_PKT	enum_item	enum UHF_RXTX_STATES  {
UHF_RX_COMPLETE	enum_item	enum UHF_RXTX_STATES  {
UHF_TX_TRAILER	enum_item	enum UHF_RXTX_STATES  {
UHF_TX_END	enum_item	enum UHF_RXTX_STATES  {
UHF_TX_COMPLETE	enum_item	enum UHF_RXTX_STATES  {
CMD_MASTER	#define	#define CMD_MASTER	1
CMD_SLAVE	#define	#define CMD_SLAVE	0
 þ
ÄÄ´ vlf_pkt.h     ÃÄÄ´ 06-08-18 14:21 ÃþDT=1292268217þPATH=D:\Work_PIC\HPT2\Common\include\
__VLF_PKT_H	#define	#define __VLF_PKT_H
VLF_TX_FREQUENCY	#define	#define  VLF_TX_FREQUENCY                    (35714.2857)		// The VLF TX frequency
VLF_TX_SYMBOL_FREQUENCY	#define	#define  VLF_TX_SYMBOL_FREQUENCY             (  190.7349)      	// The VLF TX symbol frequency
VLF_RX_FREQUENCY	#define	#define  VLF_RX_FREQUENCY                    (8000)           	// The VLF RX frequency
VLF_RX_SYMBOL_FREQUENCY	#define	#define  VLF_RX_SYMBOL_FREQUENCY             (100)            	// The VLF RX symbol frequency
VLF_NUM_RX_SYMBOL_SUBPERIODS	#define	#define  VLF_NUM_RX_SYMBOL_SUBPERIODS        (4)              	// The number of periods in a RX bit
VLF_RX_HIGH_THRESHOLD	#define	#define  VLF_RX_HIGH_THRESHOLD               (VLF_RX_FREQUENCY/VLF_RX_SYMBOL_FREQUENCY/VLF_NUM_RX_SYMBOL_SUBPERIODS/2 - 1) // Th
VLF_LOC_ONESHOT_TX_PERIOD_MS	#define	#define  VLF_LOC_ONESHOT_TX_PERIOD_MS        ((ulong)1000)    	// The period for which to transmit a location signal in one sho
VLF_LOC_CONT_TX_PERIOD_MS	#define	#define  VLF_LOC_CONT_TX_PERIOD_MS           ((ulong)300)		// The period for which to transmit a location signal in continu
VLF_LOC_CONT_WAIT_PERIOD_MS	#define	#define  VLF_LOC_CONT_WAIT_PERIOD_MS         ((ulong)101)       // The period for which to wait for a location signature in cont
VLF_LOC_CONT_INIT_TX_HOLDOFF_MS	#define	#define  VLF_LOC_CONT_INIT_TX_HOLDOFF_MS     ((ulong)10)
VLF_LOC_CONT_INIT_WAIT_PERIOD_MS	#define	#define  VLF_LOC_CONT_INIT_WAIT_PERIOD_MS    ((ulong)1000)
VLF_LOC_CONT_MISSED_SIGS	#define	#define  VLF_LOC_CONT_MISSED_SIGS            ((ulong)6)         // The number of missed signatures before cancelling continuous
SU_TO_MU_PREAMBLE	#define	#define  SU_TO_MU_PREAMBLE                   0x5A5             	// The packet preamble for SU to MU messages
VLF_SU_TO_MU_PREAMBLE_SIZE	#define	#define  VLF_SU_TO_MU_PREAMBLE_SIZE          12                 // The size in symbols of the packet preamble
SU_TO_MU_PREAMBLE_CRC	#define	#define  SU_TO_MU_PREAMBLE_CRC               0xF5
MU_TO_SU_PREAMBLE	#define	#define  MU_TO_SU_PREAMBLE                   0xA5             	// The preamble value for MU to SU messages
VLF_MU_TO_SU_PREAMBLE_SIZE	#define	#define  VLF_MU_TO_SU_PREAMBLE_SIZE          8                  // The size in bits of the packet preamble
MU_TO_SU_PREAMBLE_CRC	#define	#define  MU_TO_SU_PREAMBLE_CRC               0x28
LOC_SIGNATURE	#define	#define  LOC_SIGNATURE                       0x6B               // The location signature value
LOC_SIGNTAURE_SIZE	#define	#define  LOC_SIGNTAURE_SIZE                  7                  // The size in symbols of the location signature
EVAC_SIGNATURE	#define	#define  EVAC_SIGNATURE                      0xBEAF             // The evacuation signature
PCKT_PREAMBLE_SUBP_SIZE	#define	#define  PCKT_PREAMBLE_SUBP_SIZE             (VLF_SU_TO_MU_PREAMBLE_SIZE*VLF_NUM_RX_SYMBOL_SUBPERIODS)  	// The PREAMBLE size
PCKT_LOC_SIG_SIZE	#define	#define  PCKT_LOC_SIG_SIZE                   7                  // The size in symbols of the location signature
PCKT_TYPE_SIZE	#define	#define  PCKT_TYPE_SIZE                      4                  // The size in symbols of the packet type
VITERBI_N	#define	#define  VITERBI_N                           2                  // The rate of the convolutional code used
VITERBI_K	#define	#define  VITERBI_K                           5                  // The constraint length of the code
SCORE_METRIC_SHIFT	#define	#define  SCORE_METRIC_SHIFT                  3                  // The viterbi metric score shift value
SCORE_METRIC_FS	#define	#define  SCORE_METRIC_FS                     ((VLF_RX_FREQUENCY/VLF_RX_SYMBOL_FREQUENCY) >> SCORE_METRIC_SHIFT) // A full scale
SCORE_METRIC_LOW_THRESH	#define	#define  SCORE_METRIC_LOW_THRESH             (SCORE_METRIC_FS*4/5)		// The maximum score metric for a low bit
SCORE_METRIC_HIGH_THRESH	#define	#define  SCORE_METRIC_HIGH_THRESH            (SCORE_METRIC_FS*2/5)  	// The minimum score metric for a high bit
SCORE_METRIC_MID_THRESH	#define	#define  SCORE_METRIC_MID_THRESH             (SCORE_METRIC_FS*3/5)  	// Somewhere in between
CODING_TAIL_SIZE	#define	#define  CODING_TAIL_SIZE                    ((VITERBI_K-1)*VITERBI_N) 	// The length of the coding tail
MU_PROTOCOL2_RESPONSE_PAYLOADSIZE	#define	#define MU_PROTOCOL2_RESPONSE_PAYLOADSIZE 4
VLF_COMMS_PACKET_TYPES	enum_type	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_DETECT	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_MASK	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_QUICK_SEARCH_Y	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_QUICK_SEARCH_Z	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_LOCATE	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_VLF_RX_CONTROL	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_QUICK_SEARCH_X	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_TEST	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_MU_MASK_FULL	enum_item	enum VLF_COMMS_PACKET_TYPES {
VLF_NO_PACKET	enum_item	enum VLF_COMMS_PACKET_TYPES {
DetectCommand_P2	typedef_struct	} DetectCommand_P2;
MaskCommand_P2	typedef_struct	} MaskCommand_P2;
MaskFullCommand_P3	typedef_struct	} MaskFullCommand_P3;
LocateCommand_P2	typedef_struct	} LocateCommand_P2;
GenericResponse_P2	typedef_struct	} GenericResponse_P2;
Pkt_tx_buf_st	typedef_struct_type	typedef struct Pkt_tx_buf_st {
Pkt_tx_buf_st	typedef_struct	} Pkt_tx_buf_st;
Pkt_rx_buf_st	typedef_struct_type	typedef struct Pkt_rx_buf_st {
Pkt_rx_buf_st	typedef_struct	} Pkt_rx_buf_st;
uncoded_rx_data_size	const_[]	extern const BYTE uncoded_rx_data_size[];
UNCODED_RX_DATA_SIZE	#define	#define UNCODED_RX_DATA_SIZE( type )      ( (BYTE)(uncoded_rx_data_size[type]    ) )
UNCODED_RX_PAYLOAD_SIZE	#define	#define UNCODED_RX_PAYLOAD_SIZE( type )   ( (BYTE)(uncoded_rx_data_size[type] + 1) )
 þ
ÄÄ´ wait.h        ÃÄÄ´ 01-03-18 16:18 ÃþDT=1281458780þPATH=D:\Work_PIC\HPT2\Common\include\
__WAIT_H	#define	#define __WAIT_H
 þ
ÄÄ´ eeprom.h      ÃÄÄ´ 10-06-13 12:29 ÃþDT=1120560032þPATH=D:\Work_PIC\HPT2\Common\include\
__EEPROM_H	#define	#define __EEPROM_H
EEPROM_SD_OFFSET	#define	#define  EEPROM_SD_OFFSET                    0
FLASH_ID_WORD_ADDR	#define	#define  FLASH_ID_WORD_ADDR                  0x1FFC
EEPROM_LAMP_MODE_SIGN1	#define	#define  EEPROM_LAMP_MODE_SIGN1              0x77
EEPROM_LAMP_MODE_SIGN0	#define	#define  EEPROM_LAMP_MODE_SIGN0              0x00
EEPROM_SD_OFFSETOF	#define	#define  EEPROM_SD_OFFSETOF( member )        ((BYTE)(&((EeSharedDataTSt*)EEPROM_SD_OFFSET)->member))
EeSharedDataTSt	typedef_struct	} EeSharedDataTSt;
 þ
ÄÄ´ pic16f887.h   ÃÄÄ´ 12-01-09 20:32 ÃþDT=976004118þPATH=D:\PICC\std\9.60\include\
__PIC16F887_H	#define	#define	__PIC16F887_H
@0x00	char_static_volatile_unsigned	static volatile       unsigned char	INDF		@ 0x00;
@0x001	char_static_volatile_unsigned	static volatile       unsigned char	TMR0		@ 0x001;
@0x002	char_static_volatile_unsigned	static volatile       unsigned char	PCL		@ 0x002;
@0x003	char_static_volatile_unsigned	static volatile       unsigned char	STATUS		@ 0x003;
@0x004	char_static_unsigned	static                unsigned char	FSR		@ 0x004;
@0x005	char_static_volatile_unsigned	static volatile       unsigned char	PORTA		@ 0x005;
@0x006	char_static_volatile_unsigned	static volatile       unsigned char	PORTB		@ 0x006;
@0x007	char_static_volatile_unsigned	static volatile       unsigned char	PORTC		@ 0x007;
@0x008	char_static_volatile_unsigned	static volatile       unsigned char	PORTD		@ 0x008;
@0x009	char_static_volatile_unsigned	static volatile       unsigned char	PORTE		@ 0x009;
@0x00A	char_static_volatile_unsigned	static volatile       unsigned char	PCLATH		@ 0x00A;
@0x00B	char_static_volatile_unsigned	static volatile       unsigned char	INTCON		@ 0x00B;
@0x00C	char_static_volatile_unsigned	static volatile       unsigned char	PIR1		@ 0x00C;
@0x00D	char_static_volatile_unsigned	static volatile       unsigned char	PIR2		@ 0x00D;
@0x00E	char_static_volatile_unsigned	static volatile       unsigned char	TMR1L		@ 0x00E;
@0x00F	char_static_volatile_unsigned	static volatile       unsigned char	TMR1H		@ 0x00F;
@0x010	char_static_unsigned	static                unsigned char	T1CON		@ 0x010;
@0x011	char_static_volatile_unsigned	static volatile       unsigned char	TMR2		@ 0x011;
@0x012	char_static_unsigned	static                unsigned char	T2CON		@ 0x012;
@0x013	char_static_volatile_unsigned	static volatile       unsigned char	SSPBUF		@ 0x013;
@0x014	char_static_volatile_unsigned	static volatile       unsigned char	SSPCON		@ 0x014;
@0x015	char_static_volatile_unsigned	static volatile       unsigned char	CCPR1L		@ 0x015;
@0x016	char_static_volatile_unsigned	static volatile       unsigned char	CCPR1H		@ 0x016;
@0x017	char_static_volatile_unsigned	static volatile       unsigned char	CCP1CON		@ 0x017;
@0x018	char_static_volatile_unsigned	static volatile       unsigned char	RCSTA		@ 0x018;
@0x019	char_static_volatile_unsigned	static volatile       unsigned char	TXREG		@ 0x019;
@0x01A	char_static_volatile_unsigned	static volatile       unsigned char	RCREG		@ 0x01A;
@0x01B	char_static_volatile_unsigned	static volatile       unsigned char	CCPR2L		@ 0x01B;
@0x01C	char_static_volatile_unsigned	static volatile       unsigned char	CCPR2H		@ 0x01C;
@0x01D	char_static_volatile_unsigned	static volatile       unsigned char	CCP2CON		@ 0x01D;
@0x01E	char_static_volatile_unsigned	static volatile       unsigned char	ADRESH		@ 0x01E;
@0x01F	char_static_volatile_unsigned	static volatile       unsigned char	ADCON0		@ 0x01F;
@0x081	bank1_static	static          bank1 unsigned char	OPTION		@ 0x081;
@0x085	bank1_static_volatile	static volatile bank1 unsigned char	TRISA		@ 0x085;
@0x086	bank1_static_volatile	static volatile bank1 unsigned char	TRISB		@ 0x086;
@0x087	bank1_static_volatile	static volatile bank1 unsigned char	TRISC		@ 0x087;
@0x088	bank1_static_volatile	static volatile bank1 unsigned char	TRISD		@ 0x088;
@0x089	bank1_static_volatile	static volatile bank1 unsigned char	TRISE		@ 0x089;
@0x08C	bank1_static	static          bank1 unsigned char	PIE1		@ 0x08C;
@0x08D	bank1_static	static          bank1 unsigned char	PIE2		@ 0x08D;
@0x08E	bank1_static_volatile	static volatile bank1 unsigned char	PCON		@ 0x08E;
@0x08F	bank1_static_volatile	static volatile bank1 unsigned char	OSCCON		@ 0x08F;
@0x090	bank1_static	static          bank1 unsigned char	OSCTUNE		@ 0x090;
@0x091	bank1_static_volatile	static volatile bank1 unsigned char	SSPCON2		@ 0x091;
@0x092	bank1_static	static          bank1 unsigned char	PR2		@ 0x092;
@0x093	bank1_static	static          bank1 unsigned char	SSPADD		@ 0x093;
@0x093	bank1_static	static          bank1 unsigned char	SSPMSK		@ 0x093;
@0x094	bank1_static_volatile	static volatile bank1 unsigned char	SSPSTAT		@ 0x094;
@0x095	bank1_static	static          bank1 unsigned char	WPUB		@ 0x095;
@0x096	bank1_static	static          bank1 unsigned char	IOCB		@ 0x096;
@0x097	bank1_static	static          bank1 unsigned char	VRCON		@ 0x097;
@0x098	bank1_static_volatile	static volatile bank1 unsigned char	TXSTA		@ 0x098;
@0x099	bank1_static	static          bank1 unsigned char	SPBRG		@ 0x099;
@0x09A	bank1_static	static          bank1 unsigned char	SPBRGH		@ 0x09A;
@0x09B	bank1_static_volatile	static volatile bank1 unsigned char	PWM1CON		@ 0x09B;
@0x09C	bank1_static_volatile	static volatile bank1 unsigned char	ECCPAS		@ 0x09C;
@0x09D	bank1_static_volatile	static volatile bank1 unsigned char	PSTRCON		@ 0x09D;
@0x09E	bank1_static_volatile	static volatile bank1 unsigned char	ADRESL		@ 0x09E;
@0x09F	bank1_static	static          bank1 unsigned char	ADCON1		@ 0x09F;
@0x105	bank2_static_volatile	static volatile bank2 unsigned char	WDTCON		@ 0x105;
@0x107	bank2_static_volatile	static volatile bank2 unsigned char	CM1CON0		@ 0x107;
@0x108	bank2_static_volatile	static volatile bank2 unsigned char	CM2CON0		@ 0x108;
@0x109	bank2_static_volatile	static volatile bank2 unsigned char	CM2CON1		@ 0x109;
@0x10C	bank2_static_volatile	static volatile bank2 unsigned char	EEDAT		@ 0x10C;
@0x10C	bank2_static_volatile	static volatile bank2 unsigned char	EEDATA		@ 0x10C;
@0x10D	bank2_static	static          bank2 unsigned char	EEADR		@ 0x10D;
@0x10D	bank2_static	static          bank2 unsigned char	EEADRL		@ 0x10D;
@0x10E	bank2_static_volatile	static volatile bank2 unsigned char	EEDATH		@ 0x10E;
@0x10F	bank2_static	static          bank2 unsigned char	EEADRH		@ 0x10F;
@0x185	bank3_static_volatile	static volatile bank3 unsigned char	SRCON		@ 0x185;
@0x187	bank3_static_volatile	static volatile bank3 unsigned char	BAUDCTL		@ 0x187;
@0x188	bank3_static	static          bank3 unsigned char	ANSEL		@ 0x188;
@0x189	bank3_static	static          bank3 unsigned char	ANSELH		@ 0x189;
@0x18C	bank3_static_volatile	static volatile bank3 unsigned char	EECON1		@ 0x18C;
@0x18D	bank3_static_volatile	static volatile bank3 unsigned char	EECON2		@ 0x18D;
 þ
ÄÄ´ htc.h         ÃÄÄ´ 12-01-09 20:32 ÃþDT=976004118þPATH=D:\PICC\std\9.60\include\
_HTC_H_	#define	#define _HTC_H_
__LITE__	#define	#define __LITE__ 0
__STD__	#define	#define __STD__ 1
__PRO__	#define	#define __PRO__ 2
___mkstr1	#define	#define	___mkstr1(x)	#x
___mkstr	#define	#define	___mkstr(x)	___mkstr1(x)
 þ
ÄÄ´ pic.h         ÃÄÄ´ 12-01-09 20:32 ÃþDT=976004118þPATH=D:\PICC\std\9.60\include\
_PIC_H_	#define	#define	_PIC_H_
CLRWDT	#define	#define	CLRWDT()	asm("clrwdt")
SLEEP	#define	#define	SLEEP()		asm("sleep")
NOP	#define	#define NOP()		asm("nop")
__CONFIG	#define	#define	__CONFIG(x)	asm("\tpsect config,class=CONFIG,delta=2");\
__IDLOC	#define	#define __IDLOC(w)       asm("\tpsect idloc,class=IDLOC,delta=2");\
__IDLOC7	#define	#define __IDLOC7(a,b,c,d) asm("\tpsect idloc,class=IDLOC,delta=2");\
__EEPROM_DATA	#define	#define __EEPROM_DATA(a, b, c, d, e, f, g, h) \
FLASH_READ	#define	#define FLASH_READ(addr) \
FLASH_WRITE	#define	#define FLASH_WRITE(addr,data)	\
FLASH_WRITE	#define	#define FLASH_WRITE(addr, value) \
FLASH_ERASE	#define	#define FLASH_ERASE(addr) \
EEPROM_WRITE	#define	#define	EEPROM_WRITE(addr, value) \
EEPROM_READ	#define	#define	EEPROM_READ(addr) ((EEADR=(addr)),(EECON1&=0x7F),(RD=1),EEDATA)
ei	#define	#define	ei()	(GLINTD = 0)	// interrupt disable bit
di	#define	#define di()	{ do { GLINTD = 1; } while ( GLINTD == 0 ); }	// disable interrupt bit
ei	#define	#define	ei()	(GIE = 1)	// interrupt enable bit
di	#define	#define di()	{ do { GIE = 0; } while ( GIE == 1 ); }	// disable interrupt bit
__timeout	bit	extern bit __timeout, __powerdown;
__powerdown	bit	extern bit __timeout, __powerdown;
 þ
ÄÄ´ uhf_pkt.h     ÃÄÄ´ 30-01-19 06:48 ÃþDT=1312699917þPATH=D:\Work_PIC\HPT2\Common\include\
__UHF_PKT	#define	#define __UHF_PKT
UHF_PREAMBLE_SIZE	#define	#define  UHF_PREAMBLE_SIZE                   10    	// The number of preamble bytes in a UHF packet
UHF_UART_RESYNC_SIZE	#define	#define  UHF_UART_RESYNC_SIZE                2     	// The number of UART resynch bytes in a UHF packet
UHF_TRAILER_SIZE	#define	#define  UHF_TRAILER_SIZE                    1     	// The number of trailer bytes in a UHF packet
UHF_PREAMBLE_CHAR	#define	#define  UHF_PREAMBLE_CHAR                   0x55  	// The preamble character used
UHF_UART_RESYNC_CHAR	#define	#define  UHF_UART_RESYNC_CHAR                0xFF  	// The resynch character used
UHF_STX_CHAR	#define	#define  UHF_STX_CHAR                        0x5A  	// The STX field character used
UHF_TRAILER_CHAR	#define	#define  UHF_TRAILER_CHAR                    0x5C  	// The trailer character used
UHF_PREMABLE_RX_THRESHOLD	#define	#define  UHF_PREMABLE_RX_THRESHOLD           4     	// The number of valid preamble characters required
UHF_UART_STX_THRESHOLD	#define	#define  UHF_UART_STX_THRESHOLD            	 8    	// The number of characters to wait for an STX
UHF_TIMESLOT_PERIOD_US	#define	#define  UHF_TIMESLOT_PERIOD_US              5500  	// The length of a UHF timeslot
UHF_CRC_POLYNOMIAL	#define	#define  UHF_CRC_POLYNOMIAL                  0xA001 // The CRC polynimial
UHF_CRC_INIT	#define	#define  UHF_CRC_INIT                        0xFFFF // The initial CRC value
UHF_DL_DATA_BOCK_SIZE	#define	#define  UHF_DL_DATA_BOCK_SIZE               0x20   // Size of download block in words
UHF_CONTROL_SEARCH_REPLIES	#define	#define  UHF_CONTROL_SEARCH_REPLIES          7      // Number of replies to give to the UHF control search packets
CMD_MS	#define	#define CMD_MS								0x80
CMD_ACK	#define	#define CMD_ACK								0x01
CMD_GAS_ALARM	#define	#define CMD_GAS_ALARM	  					0x02
CMD_BAT_LOW	#define	#define CMD_BAT_LOW	  						0x04
UHF_BI_s	typedef_struct	} UHF_BI_s;
UHF_VLF_s	typedef_struct	} UHF_VLF_s;
UHF_BIRC_s	typedef_struct	} UHF_BIRC_s;
UHF_BIRC_E_s	typedef_struct	} UHF_BIRC_E_s;
UHF_II_s	typedef_struct	} UHF_II_s;
bir_cmd_s	typedef_struct	} bir_cmd_s;
BIR_CMD_ACK	#define	#define BIR_CMD_ACK			(1)
BIR_GAS_ALARM	#define	#define BIR_GAS_ALARM		(2)
BIR_CMD_ACK_MANUAL	#define	#define BIR_CMD_ACK_MANUAL 	(BIR_GAS_ALARM)
BIR_CMD_BATT_ERR	#define	#define BIR_CMD_BATT_ERR	(4)
UHF_BIR_s	typedef_struct	} UHF_BIR_s;
UhfCtrlSearchCTSt	typedef_struct	} UhfCtrlSearchCTSt;
UhfCtrlQueryCTSt	typedef_struct	} UhfCtrlQueryCTSt;
UhfCtrlSearchQueryRTSt	typedef_struct	} UhfCtrlSearchQueryRTSt;
UhfCtrlBootModeCTSt	typedef_struct	} UhfCtrlBootModeCTSt;
UhfCtrlDLInitTSt	typedef_struct	} UhfCtrlDLInitTSt;
UhfCtrlDLDataTSt	typedef_struct	} UhfCtrlDLDataTSt;
UhfCtrlDLEndCTSt	typedef_struct	} UhfCtrlDLEndCTSt;
UHF_CRTL_TEST_s	typedef_struct	} UHF_CRTL_TEST_s;
UhfCtrlDLEndRTSt	typedef_struct	} UhfCtrlDLEndRTSt;
UhfCtrlSetIdCTSt	typedef_struct	} UhfCtrlSetIdCTSt;
UhfCtrlSetIdRTSt	typedef_struct	} UhfCtrlSetIdRTSt;
UhfPktTU	typedef_union	} UhfPktTU;
UHF_PKT_TYPES	enum_type	enum UHF_PKT_TYPES {
UHF_BI_PKT	enum_item	enum UHF_PKT_TYPES {
UHF_BIRC_PKT	enum_item	enum UHF_PKT_TYPES {
UHF_II_PKT	enum_item	enum UHF_PKT_TYPES {
#if	enum_item	enum UHF_PKT_TYPES {
UHF_VLF_PKT	enum_item	enum UHF_PKT_TYPES {
#endif	enum_item	enum UHF_PKT_TYPES {
UHF_CONTROL_TYPES	enum_type	enum UHF_CONTROL_TYPES {
UHFC_SEARCH	enum_item	enum UHF_CONTROL_TYPES {
UHFC_QUERY	enum_item	enum UHF_CONTROL_TYPES {
UHFC_BOOTMODE	enum_item	enum UHF_CONTROL_TYPES {
UHFC_3	enum_item	enum UHF_CONTROL_TYPES {
UHFC_4	enum_item	enum UHF_CONTROL_TYPES {
UHFC_TESTMODE	enum_item	enum UHF_CONTROL_TYPES {
Uhf_pkt_s	typedef_struct_type	typedef struct Uhf_pkt_s {
Uhf_pkt_s	typedef_struct	} Uhf_pkt_s;
 þ
ÄÄ´ uhf_comms.c   ÃÄÄ´ 28-02-19 05:00 ÃþDT=1314662410þPATH=D:\Work_PIC\HPT2\Application\src\
uhf	bank2	bank2 Uhf_pkt_s	uhf;
uhf_registration_timeout	ushort	ushort      uhf_registration_timeout;
bb	bank3_[]	bank3 bid_u		  bb[BB_LIST_SIZE];		// Bounded beacons list
bi	bank3_[]	bank3 bid_u		  bi[BB_LIST_SIZE];		// Invitation beacons list
bl_version	bank3	bank3 BYTE 		bl_version = 0;
uhf_ctrl_sidx	ushort_static	static ushort  	uhf_ctrl_sidx = 0;
uhf_ctrl_sctr	BYTE_static	static BYTE    	uhf_ctrl_sctr = 0;
PrepareSearchQueryReply	void_static_proc	static void PrepareSearchQueryReply(void)
UHFBusy	BYTE_proc	BYTE UHFBusy(void)
ProcessUHFPacket	BYTE_proc	BYTE ProcessUHFPacket(void)
SendUHFPkt	void_static_proc	static void SendUHFPkt(void)
Test433tx	void_proc	void Test433tx(void)
 þ
ÄÄ´ mu.h          ÃÄÄ´ 30-05-19 08:32 ÃþDT=1321092121þPATH=D:\Work_PIC\HPT2\Application\include\
__MU_H_	#define	#define __MU_H_
HPT4_MODE	#define	#define HPT4_MODE						0		// Óñå÷åííûé âàðèàíò áåç ïðèåìíèêà 8KHz, íî ñî âõîäîì äàò÷èêà
HPT3_MODE	#define	#define HPT3_MODE						4		// 1 - Ïåéäæåð ñ èíäèâèäóàëüíûìè âûçîâàìè
MU_VLF_TX_TESTING	#define	#define MU_VLF_TX_TESTING   			0 		// 1=Normal, 2=Preamble 0xFF 3=Full 4=0/1 íåïðåðûâíî
USE_8KHZ_FILTER	#define	#define USE_8KHZ_FILTER 				1
HPT2_MODE	#define	#define	HPT2_MODE						0       // Ñòàíäàðòíûé ïåéäæåð
DEBUG_LEVEL	#define	#define DEBUG_LEVEL		 	0
DEBUG_II	#define	#define DEBUG_II 			0					//
DEBUG_SU_UHF_RX	#define	#define DEBUG_SU_UHF_RX		0					//
DEBUG_RX_VLF	#define	#define DEBUG_RX_VLF 	 	0
MU_433_TX_TESTING	#define	#define MU_433_TX_TESTING	0
MU_PAGER_TESTING	#define	#define MU_PAGER_TESTING	0
MU_R8KHZ_TESTING	#define	#define MU_R8KHZ_TESTING	0
USE_MOBILITY	#define	#define USE_MOBILITY 		0					//
TEST_BEACON_REGISTRATION	#define	#define TEST_BEACON_REGISTRATION 		0		//
USE_LAMP_KEY	#define	#define USE_LAMP_KEY 						1
USE_IRIDIUM	#define	#define USE_IRIDIUM 						0
USE_LAMP_KEY	#define	#define USE_LAMP_KEY 						0
USE_IRIDIUM	#define	#define USE_IRIDIUM 						1
USE_INVERT_LAMP	#define	#define USE_INVERT_LAMP					0
USE_UHF_TO_VLF	#define	#define USE_UHF_TO_VLF 					1
USE_UHF_TO_VLF	#define	#define USE_UHF_TO_VLF 					1	// UHF SU Mode support
APP_FIRMWARE_VERSION	#define	#define APP_FIRMWARE_VERSION            0x80 	// Âàðèàíò Êîìáàéíåðà ñ ïîäòâåðæäåíèåì äîáàâëåí
APP_FIRMWARE_VERSION	#define	#define APP_FIRMWARE_VERSION            0x7B 	// Òåñò VLF ïî êîìàíäå II ðàáîòàåò. Çàáëîêèðîâàí âõîä Reset
APP_FIRMWARE_VERSION	#define	#define APP_FIRMWARE_VERSION          	0x68  	// Test ÌÄÀ ïî êîìàíäå II ðàáîòàåò. Çàáëîêèðîâàí âõîä Reset
ALLOW_UHF_BOOTMODE	#define	#define ALLOW_UHF_BOOTMODE		1
MOBILITY_AD_CHANNEL	#define	#define  MOBILITY_AD_CHANNEL 	(0<<2)	// The A/D channel to take mobility readings from
TXSIG_AD_CHANNEL	#define	#define  TXSIG_AD_CHANNEL       (9<<2) 	// The A/D channel to take TX readings from
LAMP_SEARCH_STATE_TIMEOUT	#define	#define  LAMP_SEARCH_STATE_TIMEOUT     	(ushort)( 3*60*(TICKS_PER_SEC))
LAMP_60SEC_TIMEOUT	#define	#define  LAMP_60SEC_TIMEOUT     		(ushort)(   60*(TICKS_PER_SEC))
LAMP_CHECK_STATE_TIMEOUT	#define	#define  LAMP_CHECK_STATE_TIMEOUT     	(ushort)(   90*(TICKS_PER_SEC))
LAMP_CALL_STATE_TIMEOUT	#define	#define  LAMP_CALL_STATE_TIMEOUT     	(ushort)( 5*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5
LAMP_EVACUATE_STATE_TICK	#define	#define  LAMP_EVACUATE_STATE_TICK      	(ushort)(30*60*(TICKS_PER_SEC)) // The timeout for exitting an evacuate state
LAMP_IDENTIFY_STATE_TIMEOUT	#define	#define  LAMP_IDENTIFY_STATE_TIMEOUT    (ushort)(10*(TICKS_PER_SEC)) 	// The timeout for exitting an identify state
LAMP_ALARM_STATE_TIMEOUT	#define	#define  LAMP_ALARM_STATE_TIMEOUT       (ushort)( 5*(LAMP_SCHEDULE_SIZE * 8))    // Pager timeout = 5
LAMP_ALARM_UHF_TIMEOUT	#define	#define  LAMP_ALARM_UHF_TIMEOUT       	(ushort)(60*(TICKS_PER_SEC))    // Pager timeout
LAMP_SCHEDULE_SIZE	#define	#define  LAMP_SCHEDULE_SIZE             ( 8 )                           // Size of the flash schedule for each state
LAMP_SCHEDULE_MASK	#define	#define  LAMP_SCHEDULE_MASK            	((LAMP_SCHEDULE_SIZE)-1)
LMP_STATES	enum_type	enum LMP_STATES {
LAMP_POST_ERROR	enum_item	enum LMP_STATES {
LAMP_FIRMWARE_disabled	enum_item	enum LMP_STATES {
LAMP_SEARCH_IN_PROGRESS	enum_item	enum LMP_STATES {
LAMP_EVACUATE_IN_PROGRESS_disabled	enum_item	enum LMP_STATES {
LAMP_IDENTIFY	enum_item	enum LMP_STATES {
LAMP_PAGER_ALARM	enum_item	enum LMP_STATES {
LAMP_PAGER_CALL_1	enum_item	enum LMP_STATES {
LAMP_PAGER_CALL_2	enum_item	enum LMP_STATES {
LAMP_VLF_BLOCKED_STATE	enum_item	enum LMP_STATES {
LAMP_STATES_NUM	enum_item	enum LMP_STATES {
LAMP_NORMAL_STATE	enum_item	enum LMP_STATES {
LAMP_INVERT_STATE	enum_item	enum LMP_STATES {
LAMP_TEST_STATE	enum_item	enum LMP_STATES {
lamp_state	BYTE	extern BYTE     lamp_state;
lamp_timeout	ushort	extern ushort 	lamp_timeout;
lamp_timeout_loops	BYTE	extern BYTE 	lamp_timeout_loops;
mu_flags	BYTE	extern BYTE		mu_flags;
FL_SEND	#define	#define	FL_SEND				(0x01)
FL_REGISTRATION	#define	#define FL_REGISTRATION		(0x02)
FL_LAMP_INV	#define	#define FL_LAMP_INV			(0x04)
FL_ACK	#define	#define FL_ACK				(0x08)
FL_ACK_FORCED	#define	#define FL_ACK_FORCED		(0x10)
FL_QUIET_PAGER	#define	#define FL_QUIET_PAGER      (0x20)
FL_ACK_MANUAL	#define	#define FL_ACK_MANUAL		(0x40)
mid	long_u	extern long_u    mid;
lamp_min_cnt	BYTE	extern BYTE lamp_min_cnt;
LAMP_ON	#define	#define LAMP_ON		1
LAMP_OFF	#define	#define LAMP_OFF		0
 þ
ÄÄ´ main.c        ÃÄÄ´ 30-05-19 10:56 ÃþDT=1321096971þPATH=D:\Work_PIC\HPT2\Application\src\
mid	long_u	long_u 	mid;
lamp_state	BYTE	BYTE	lamp_state;
mu_flags	BYTE	BYTE	mu_flags;
lamp_min_cnt	BYTE	BYTE 	lamp_min_cnt;
lamp_tick	BYTE	BYTE   	lamp_tick;
pager_cnt	BYTE	BYTE	pager_cnt = 0;
lampk_cnt	BYTE	BYTE	lampk_cnt = 0;
lampo_cnt	BYTE	BYTE	lampo_cnt = 0;
lampk_cnt	BYTE	BYTE	lampk_cnt = 0;
lamp_timeout	ushort	ushort 	lamp_timeout;
uhf_reset_timeout	ushort_static	static ushort uhf_reset_timeout;
bl_version	bank3	extern bank3 BYTE bl_version;
eeprom_tmp	BYTE	BYTE eeprom_tmp;
lamp_schedule	BYTE_static_const_[][]	static const BYTE lamp_schedule[LAMP_STATES_NUM][LAMP_SCHEDULE_SIZE] = {
TEST_DELAY_ROUTINES	#define	#define TEST_DELAY_ROUTINES 	0
TestDelayRoutines	void_proc	void TestDelayRoutines()
main	void_proc	void main( void )
FILTER_8KHZ	#define	#define FILTER_8KHZ		(2) 	// 1-Hard 2-Soft
VRXDAT	#define	#define VRXDAT		 	(sizeof(vrx.preamble)-1)
int_handler	void_proc	void interrupt int_handler(void)
