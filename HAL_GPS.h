#ifndef HAL_GPS
#define HAL_GPS

#define GPS_1_ON 	0x00000010u
#define GPS_1_RESET 0x00000020u
#define GPS_2_ON 	0x00000040u
#define GPS_2_RESET 0x00000080u
#define GPS_1_OFF   0x0000FFEFu
#define GPS_2_OFF   0x0000FFBFu

#define Buffer_inc(a) a = a + 0x00000004

unsigned int GPS_msg_header;
unsigned int GPS_msg_header_MSB;
unsigned int GPS_msg_header_LSB;

unsigned short  gps_pulse_mic_cnt;
unsigned int  GPS_pulse_rcvd;
unsigned long int gps_obt_counter;
unsigned long int gps_obt_count_prev;

unsigned long int GPS_Status_Data;
unsigned long int GPS_Data;
unsigned long int GPS_Buffer_Data[300];
unsigned long int GPS_Addr_Count;
unsigned long int GPS_Buffer_Addr;
unsigned long int GPS_Data_Read_Status;
unsigned long int GPS_Config_Status;
unsigned long int GPS_Locations;

void rHAL_GPS_POWER(unsigned long int GPS_No,unsigned long int GPS_Power);
unsigned long int rHAL_GPS_Read(unsigned long int GPS_No,unsigned long int* GPS_Addr,unsigned long int No_of_Bytes);
unsigned long int rHAL_GPS_Config(unsigned long int GPS_No,unsigned long int Config_Type);
void rGPS_Buffer_Init();
void GPS_1_DATA();
void rGPS_pulsecheck();

#pragma pack(1)
union GPS_Config_Message
{
	unsigned char data[12];
	struct
	{
		unsigned char Initial_msg[7];
		unsigned char Msg_index;
		unsigned char E_D;
		unsigned char Chk_sum;
		unsigned char Carriage_return;
		unsigned char Line_feed;
	};
}GPS_Config_Msg;

#pragma pack(1)
struct GPS_Buff
{
	unsigned int Config_1;
	unsigned int Config_2;
	unsigned int Config_3;
	unsigned int Config_4;
	unsigned int Config_5;
	unsigned int Config_6;
	unsigned int Config_7;
	unsigned int Config_8;
	unsigned int Config_9;
	unsigned int Config_10;
	unsigned int Config_11;
	unsigned int Config_12;

	unsigned int Status_1;
	unsigned int Status_2;
	unsigned int Buffer;

}GPS1_Buffer,GPS2_Buffer;

#pragma pack(1)
union GPS_Status2_Message
{
	unsigned short data;
	struct
	{
		unsigned status2_filler1	:3;
		unsigned status2_NOCB		:4;
		unsigned status2_filler2	:2;
		unsigned status2_config_ED	:1;
		unsigned status2_filler3	:6;
	};
}GPS_Status2_Msg;

int i_GPS_B_Count;

#define GPS_BUFFER_COPY_LIMIT 12

unsigned char* GPS_Config_ptr;
unsigned int* GPS1_Buffer_ptr;
unsigned int* GPS2_Buffer_ptr;

#ifndef OBC_GPS
#define OBC_GPS

unsigned char* GPS_TM_Buffer_Addr_USC;

unsigned long int GPS_RCVD_DATA[60];

//Function Declarations
void rGPS_TM_Extract(void);

unsigned char GPS_obc_checkum;
unsigned int f_GPS_Valid_Data;



#endif // OBC_GPS



#endif // HAL_GPS
