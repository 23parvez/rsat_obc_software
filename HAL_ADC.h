#ifndef HAL_ADC
#define HAL_ADC

#define ADC_READ_DISABLE 0x0000FFFE
#define EXTRACT_LSB_12   0x00000FFF
#define ADC_READ_ENABLE  0x00000001
#define SA1_SW_UTP       0x0130
#define SA1_SW_LTP       0x00bf
#define SA2_SW_UTP       0x0080
#define SA2_SW_LTP       0x000f
#define SA3_SW_UTP       0x0030
#define SA3_SW_LTP       0x000f

unsigned long int ADC_Status_Data;
unsigned long int inter_HAL_ADC_Data_Ready;

unsigned long int* ADC_Buffer_Addr;
unsigned long int ADC_Addr_Count;
unsigned long int ADC_Locations;
unsigned long int ADC_Data;
unsigned long int inter_Update_TM_With_ADC_offset;
unsigned long int inter_Update_TM_With_ADC_byte_offset;
unsigned long int battery_temp_1;
unsigned long int battery_temp_2;
unsigned long int thermistor_15;
unsigned long int SA_thermistor1;
unsigned long int SA_thermistor2;

#pragma pack(1)
union ADC_thermistor_lsb_msb
{
	unsigned char data[23];
	struct
	{

	   unsigned short Thermister_1  :12;
	   unsigned short Thermister_2  :12;
	   unsigned short Thermister_3  :12;
	   unsigned short Thermister_4  :12;
	   unsigned short Thermister_5  :12;
	   unsigned short Thermister_6  :12;
	   unsigned short Thermister_7  :12;
	   unsigned short Thermister_8  :12;
	   unsigned short Thermister_9  :12;
	   unsigned short Thermister_10 :12;
	   unsigned short Thermister_11 :12;
	   unsigned short Thermister_12 :12;
	   unsigned short Thermister_13 :12;
	   unsigned short Thermister_14 :12;
	   unsigned short Thermister_15 :12;
	   unsigned short filler        :4;

	};
}thermistor_lsb;

union SA_SW_STS
{
	unsigned char sa_data;
	struct
	{
		unsigned SA1_Shunt_sw:1;
		unsigned SA2_Shunt_sw:1;
		unsigned SA3_Shunt_sw:1;
		unsigned filler      :5;
	};
}SA_STS;

//Function Declarations
void rHAL_ADC_Read(unsigned long int* ADC_Addr);
void rHAL_ADC_TM_Copy(unsigned long int* ADC_Addr);
void sun_Sensor_data();
void rHAL_ADC_StatusREG_Enable();


#endif // HAL_ADC

