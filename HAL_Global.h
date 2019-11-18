#ifndef HAL_GLOBAL
#define HAL_GLOBAL

#define EXTRACT_LSB_16 0x0000FFFF
#define EXTRACT_MSB_16 0xFFFF0000
#define EXTRACT_LSB_4  0x0000000F
#define EXTRACT_LSB_16_RESET 0x00000000

#define MINOR_CYCLE_TERMINAL_COUNT 0x00008000u
#define MAJOR_CYCLE_TERMINAL_COUNT 0x00004000u
#define MINOR_CYCLE_RESET 		   0x00000001u
#define MAJOR_CYCLE_RESET 		   0x00000002u
#define SA1_DEPLOY_ST              0x00000200u
#define SA2_DEPLOY_ST              0x00000100u

#pragma pack(1)
union Out_Latch_1
{
	unsigned short data;
	struct
	{
		unsigned filler			:4;
		unsigned FP_CTRL_MTR3_N2:1;
		unsigned FP_CTRL_MTR3_N1:1;
		unsigned FP_CTRL_MTR3_P2:1;
		unsigned FP_CTRL_MTR3_P1:1;
		unsigned FP_CTRL_MTR2_N2:1;
		unsigned FP_CTRL_MTR2_N1:1;
		unsigned FP_CTRL_MTR2_P2:1;
		unsigned FP_CTRL_MTR2_P1:1;
		unsigned FP_CTRL_MTR1_N2:1;
		unsigned FP_CTRL_MTR1_N1:1;
		unsigned FP_CTRL_MTR1_P2:1;
		unsigned FP_CTRL_MTR1_P1:1;
	};
}Out_Latch_1;

#pragma pack(1)
union Out_Latch_2
{
	unsigned short data;
	struct
	{
		unsigned filler		:4;
		unsigned SPARE2_ON_OFF:1;
		unsigned RF_Tx_ON_OFF:1;
		unsigned X_Tx_ON_OFF:1;
		unsigned MTR_ON_OFF	:1;
		unsigned GPS2_RESET	:1;
		unsigned GPS2_ON_OFF:1;
		unsigned GPS1_RESET	:1;
		unsigned GPS1_ON_OFF:1;
		unsigned IMU2_RESET	:1;
		unsigned IMU2_ON_OFF:1;
		unsigned IMU1_RESET	:1;
		unsigned IMU1_ON_OFF:1;
	};
}Out_Latch_2;

#pragma pack(1)
union Out_Latch_3
{
	unsigned short data;
	struct
	{
		unsigned filler		   :3;
		unsigned TM_DS_EN      :1;
		unsigned PL2_ON_OFF	   :1;
		unsigned PL1_ON_OFF	   :1;
		unsigned RW4_ON_OFF	   :1;
		unsigned RW3_ON_OFF	   :1;
        unsigned RW2_ON_OFF	   :1;
		unsigned RW1_ON_OFF	   :1;
		unsigned Heater6_ON_OFF:1;
		unsigned Heater5_ON_OFF:1;
		unsigned Heater4_ON_OFF:1;
		unsigned Heater3_ON_OFF:1;
		unsigned Heater2_ON_OFF:1;
		unsigned Heater1_ON_OFF:1;
	};
}Out_Latch_3;

//-----------latch_4--------//
#pragma pack(1)
union Out_latch_4
{
	unsigned short data;
	struct
	{
		unsigned EEPROM_RESET   :1;
		unsigned filler2		:7;
		unsigned MajorCycle_RST	:1;
		unsigned filler1		:5;
		unsigned OBT_Reset		:1;
		unsigned MinorCycle_RST	:1;
	};
}Out_latch_4;

//----------latch_5------//
#pragma pack(1)
union Out_latch_5
{
unsigned short data;
	struct
	{
		unsigned filler			:6;
		unsigned ANTENNA_DEPLOY	:1;
		unsigned SPARE1_ON_OFF	:1;
		unsigned SA2_DEPLOY		:1;
		unsigned SA1_DEPLOY		:1;
		unsigned SA3_ON_OFF		:1;
		unsigned SA2_ON_OFF		:1;
		unsigned SA1_ON_OFF		:1;
		unsigned SEL_2			:1;
		unsigned SEL_1			:1;
		unsigned SEL_0			:1;
	};
}Out_latch_5;

#pragma pack(1)
union input_latch_1
{
unsigned short data;
	struct
	{
		unsigned FP_MTR1_mon1_mon2	:2;
		unsigned FP_MTR2_mon1_mon2	:2;
		unsigned FP_MTR3_mon1_mon2	:2;
		unsigned VOBC_TM		    :1;
		unsigned Demod_lock_Ind	    :1;
		unsigned filler			    :7;

	};
}input_latch_1;

#pragma pack(1)
union GPIO
{
	unsigned short data;
	struct
	{
		unsigned PIO_15 :1;
		unsigned PIO_14 :1;
		unsigned PIO_13 :1;
		unsigned PIO_12 :1;
		unsigned PIO_11 :1;
		unsigned PIO_10 :1;
		unsigned PIO_9  :1;
		unsigned PIO_8  :1;
		unsigned PIO_7  :1;
		unsigned PIO_6  :1;
		unsigned PIO_5  :1;
		unsigned PIO_4  :1;
		unsigned PIO_3  :1;
		unsigned PIO_2  :1;
		unsigned PIO_1  :1;
		unsigned PIO_0  :1;
	};
}GPIO_pins;

#define FDI_NMI_INT_LEVEL 4
unsigned char FDI_NMI_Count;
//unsigned short TC_count;

unsigned long int IO_In_Latch_Register_4_Data;
unsigned long int IO_Latch_Register_4_Data;

unsigned long int inter_at697f_count;
unsigned long int inter_at697f_checksum;
unsigned long int inter_at697_var;

//TC
unsigned long int TC_input_cmd[2];
#define TC_MSB TC_input_cmd[0]
#define TC_LSB TC_input_cmd[1]

//Function Declarations
void Init_Memory(void);
unsigned long int checksum_u32(unsigned long int *db_start_address,unsigned long int size_of_db);
unsigned char checksum_u8(unsigned char* db_start_address,unsigned int size_of_db);
void rOutput_Latch_Update();
void EEPROM_RST();
void GPIO();
void EEPROM_RES();

//GPS
#define GPS_1 1
#define GPS_2 0

#define NMEA_GSV_Enable  0
#define NMEA_GSV_Disable 1
#define NMEA_GGA_Enable  2
#define NMEA_GGA_Disable 3
#define NMEA_RMC_Enable  4
#define NMEA_RMC_Disable 5
#define FACTORY_RESET    6
#define COLD_START       7
#define NMEA_ZDA_Enable  8
#define NMEA_ZDA_Disable 9
#define NMEA_VTG_Enable	 10
#define NMEA_VTG_Disable 11
#define NMEA_GSA_Enable  12
#define NMEA_GSA_Disable 13
#define NMEA_GLL_Enable  14
#define NMEA_GLL_Disable 15

extern unsigned short gps_pulse_mic_cnt;
extern unsigned int GPS_pulse_rcvd;
extern unsigned long int gps_obt_counter;
extern unsigned long int gps_obt_count_prev;

//IO OUTPUT LATCH DATA
unsigned long int IO_Latch_Register_2_Data;
unsigned long int IO_Latch_Register_3_Data;
unsigned long int IO_Latch_Register_4_Data;
unsigned long int IO_Latch_Register_5_Data;

extern void rGPS_pulsecheck();

#endif //HAL_GLOBAL
