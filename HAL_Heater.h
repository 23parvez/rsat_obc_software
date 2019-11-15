
#ifndef HAL_HEATER
#define HAL_HEATER

#define MAX_VALUE    16
#define AUTO_MODE    1
#define MANUAL_MODE  0

#define Heater_1_manual 0x00000001
#define Heater_2_manual 0x00000002
#define Heater_3_manual 0x00000003
#define Heater_4_manual 0x00000004
#define Heater_5_manual 0x00000005
#define Heater_6_manual 0x00000006
#define Heater_7_manual 0x00000007





//void rHAL_Heater_Power(unsigned int Heater_No,unsigned int Heater_Power);
union heaters_auto_manual
{		unsigned short data;
		struct
		{
			unsigned filler    :8;
	        unsigned heater_8  :1;
			unsigned heater_7  :1;
			unsigned heater_6  :1;
			unsigned heater_5  :1;
			unsigned heater_4  :1;
			unsigned heater_3  :1;
			unsigned heater_2  :1;
			unsigned heater_1  :1;
		};
}heaters;

union heaters_manual_data
{		unsigned short data;
		struct
		{
			unsigned filler    :8;
			unsigned heater_8  :1;
			unsigned heater_7  :1;
			unsigned heater_6  :1;
			unsigned heater_5  :1;
			unsigned heater_4  :1;
			unsigned heater_3  :1;
			unsigned heater_2  :1;
			unsigned heater_1  :1;
		};
}heaters_manual;


/*union heaters_auto_manual
{		unsigned short data;
		struct
		{
		unsigned int  heater_13_on_off[16]:1;
		};
}heaters;*/

unsigned long int LTP[16]={0x00000204,0x00000204,0x00000204,0x00000204,0x00000204,0x00000204,0x00000204};
unsigned long int UTP[16]={0x000001b4,0x000001b4,0x000001b4,0x000001b4,0x000001b4,0x000001b4,0x000001b4};




unsigned long int  Auto_manual = 0x0000007f;

void Heater_control_auto_manual();
void Heater_1_on_off();
void Heater_2_on_off();
void Heater_3_on_off();
void Heater_4_on_off();
void Heater_5_on_off();
void Heater_6_on_off();

void Heater_manual();
void Heater_1_manuals();
void Heater_2_manuals();
void Heater_3_manuals();
void Heater_4_manuals();
void Heater_5_manuals();
void Heater_6_manuals();

#endif // HAL_HEATER
