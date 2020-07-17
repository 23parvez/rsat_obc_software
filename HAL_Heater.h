
#ifndef HAL_HEATER
#define HAL_HEATER

#define MAX_VALUE    16
#define AUTO_MODE    1
#define MANUAL_MODE  0

#define Heater_1_manual 1
#define Heater_2_manual 2
#define Heater_3_manual 3
#define Heater_4_manual 4
#define Heater_5_manual 5
#define Heater_6_manual 6
#define Heater_7_manual 0x00000007
#define Heater_8_manual 0x00000008
#define Heater_9_manual 0x00000009
#define Heater_10_manual 0x0000000A
#define Heater_11_manual 0x0000000B
#define Heater_12_manual 0x0000000C
#define Heater_13_manual 0x0000000D
#define Heater_14_manual 0x0000000E
#define Heater_15_manual 0x0000000F

#define Thermistor_MAX_LIMIT 14

unsigned long int Thermistor_Selection[14];
unsigned long int Thermister_index_select[14]={0,1,6,4,11,9,2,3,7,8,10,5,12,13};
unsigned long int heater_update_buffer[16];
unsigned short UTP_data;
unsigned short LTP_data;
//void rHAL_Heater_Power(unsigned int Heater_No,unsigned int Heater_Power);


union heaters_auto_data
{		unsigned short data;
		struct
		{
			unsigned heater_16  :1;
			unsigned heater_15  :1;
			unsigned heater_14  :1;
			unsigned heater_13  :1;
			unsigned heater_12  :1;
			unsigned heater_11  :1;
			unsigned heater_10  :1;
			unsigned heater_9  :1;
			unsigned heater_8  :1;
			unsigned heater_7  :1;
			unsigned heater_6  :1;
			unsigned heater_5  :1;
			unsigned heater_4  :1;
			unsigned heater_3  :1;
			unsigned heater_2  :1;
			unsigned heater_1  :1;
		};
}heaters_auto;

union heaters_manual_on_off_data
{		unsigned short data;
		struct
		{
			unsigned heater_16  :1;
			unsigned heater_15  :1;
			unsigned heater_14  :1;
			unsigned heater_13  :1;
			unsigned heater_12  :1;
			unsigned heater_11  :1;
			unsigned heater_10  :1;
			unsigned heater_9  :1;
			unsigned heater_8  :1;
			unsigned heater_7  :1;
			unsigned heater_6  :1;
			unsigned heater_5  :1;
			unsigned heater_4  :1;
			unsigned heater_3  :1;
			unsigned heater_2  :1;
			unsigned heater_1  :1;
		};
}manual_on_off;

/*union heaters_auto_manual
{		unsigned short data;
		struct
		{
		unsigned int  heater_13_on_off[16]:1;
		};
}heaters;*/

unsigned long int LTP[16]={0x000001c1,0x000001c1,0x000001c1,0x000001c1,0x000001c1,0x000001c1,0x000001c1,0x000001c1,0,0,0,0,0,0};   // 20 deg
            //{0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0,0,0,0,0,0};  // 25 deg
		//{0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0,0,0,0,0,0};  // 25 deg
           //
unsigned long int UTP[16] = {0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0x0000017c,0,0,0,0,0,0};    //25 deg

                         //{0x00000172,0x00000172,0x00000172,0x00000172,0x00000172,0x00000172,0x00000172,0x00000172,0,0,0,0,0,0};  //30 deg


unsigned long int  IMU_temperature;

unsigned long int  Auto_manual = 0x0000007f;

// manual heater previous telecommand memory before load shedding
unsigned short heater_manual_tc;

void Heater_control_auto_manual();
void Heater_1_auto_control();
void Heater_2_auto_control();
void Heater_3_auto_control();
void Heater_4_auto_control();
void Heater_5_auto_control();
void Heater_6_auto_control();
void Heater_7_auto_control();
void Heater_8_auto_control();
void Heater_9_auto_control();
void Heater_10_auto_control();
void Heater_11_auto_control();
void Heater_12_auto_control();
void Heater_13_auto_control();
void Heater_14_auto_control();

void Heater_1_manual_control();
void Heater_2_manual_control();
void Heater_3_manual_control();
void Heater_4_manual_control();
void Heater_5_manual_control();
void Heater_6_manual_control();
void Heater_7_manual_control();
void Heater_8_manual_control();
void Heater_9_manual_control();
void Heater_10_manual_control();
void Heater_11_manual_control();
void Heater_12_manual_control();
void Heater_13_manual_control();
void Heater_14_manual_control();
void Heater_15_manual_control();

void UTP_selection();
void LTP_selection();

#endif // HAL_HEATER
