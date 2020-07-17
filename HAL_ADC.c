#include "HAL_ADC.h"
#include "Global.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Telemetry.h"
#include "adcs_VarDeclarations.h"

void rHAL_ADC_Read(uint32* ADC_Addr)
{
	ADC_Locations   =  ADC_MAX_LIMIT - 1;
	ADC_Buffer_Addr = (uint32*) ADC_BUFFER_BASE;
	for(ADC_Addr_Count = 0;ADC_Addr_Count<=ADC_Locations;ADC_Addr_Count++)
	{
		REG32(ADC_Addr++) = REG32(ADC_Buffer_Addr++) & EXTRACT_LSB_12;
	}
}

unsigned short agc_test;
unsigned char agc_test1;
unsigned int test_agc;
unsigned short ss_data[15];
unsigned short test_thermistor_1;
void rHAL_ADC_TM_Copy(uint32* ADC_Addr)
{
	unsigned char AGC_DATA;
	unsigned int  temp_data,temp_data_th2,temp_data_th3,temp_data_th4,temp_data_th5;
	unsigned int AGC_Data;
	unsigned short tempdata;
	unsigned int Thermister_1_2,Thermister_2_3,Thermister_3,Thermister_4,Thermister_5,Thermister_6,Thermister_7,Thermister_8,Thermister_9,Thermister_10,Thermister_11,Thermister_12,Thermister_13,Thermister_14,Thermister_15,Thermister_16;
	unsigned short temp_data_0,temp_data_1,temp_data_2,temp_data_3,temp_data_4,temp_data_5,temp_data_6,temp_data_7,temp_data_8,temp_data_9,temp_data_10,temp_data_11,temp_data_12,temp_data_13,temp_data_14,temp_data_15;

		battery_temp_1 = ADC_Buffer[0];
		battery_temp_2 = ADC_Buffer[1];
		thermistor_15 = ADC_Buffer[46];
		TM.Buffer.TM_ADC_Thermistor_15 = thermistor_15;
		SA_thermistor1 = ADC_Buffer[40];
		SA_thermistor2 = ADC_Buffer[41];
		TM.Buffer.TM_SA_thermistor1 = (short)(SA_thermistor1 & 0x0FFF);
		TM.Buffer.TM_SA_thermistor2 = (short)(SA_thermistor2 & 0x0FFF);
		TM.Buffer.TM_battery_temp_1 = (short)(battery_temp_1 & 0x0FFF);
		TM.Buffer.TM_battery_temp_2 = (short)(battery_temp_2 & 0x0FFF);
		ST_normal.ST_NM_Buffer.TM_battery_temp_1 = (short)(battery_temp_1 & 0x0FFF);
		ST_normal.ST_NM_Buffer.TM_battery_temp_2 = (short)(battery_temp_2 & 0x0FFF);

		ADC_Locations   =  ADC_Thermistor - 1;
		for(ADC_Addr_Count = 0;ADC_Addr_Count<=ADC_Locations;ADC_Addr_Count++)
		{
			tempdata =  (short)(*ADC_Addr++ ); // Update to TM Global Buffer
			TM.Buffer.TM_ADC_Thermistor_Data[ADC_Addr_Count] = tempdata;
			ss_data[ADC_Addr_Count] = tempdata;
			ST_normal.ST_NM_Buffer.TM_ADC_Data[ADC_Addr_Count] = tempdata;

		}

	AGC_Data = ADC_Buffer[31];
	TM.Buffer.TM_AGC = (unsigned short)(AGC_Data & 0x00000FFF);
	temp_data = AGC_Data;
	ST_normal.ST_NM_Buffer.TM_AGC = (unsigned short)(AGC_Data & 0x00000FFF);


	temp_data_th2 = ADC_Buffer[2];
	temp_data_th3 = ADC_Buffer[3];
	temp_data_th4 = ADC_Buffer[4];
	temp_data_th5 = ADC_Buffer[5];

	ST_normal.ST_NM_Buffer.TM_Thermistor_2 = temp_data_th2;
	ST_normal.ST_NM_Buffer.TM_Thermistor_3 = temp_data_th3;
	ST_normal.ST_NM_Buffer.TM_Thermistor_4 = temp_data_th4;
	ST_normal.ST_NM_Buffer.TM_Thermistor_5 = temp_data_th5;
	TM.Buffer.SA1_Shunt_sw = ADC_Buffer[35];
	TM.Buffer.SA2_Shunt_sw = ADC_Buffer[36];
	TM.Buffer.SA3_Shunt_sw = ADC_Buffer[37];
}

unsigned char ss_data_2[16];
void sun_Sensor_data()
{
	unsigned char tempdata;
	unsigned int tempdata1;;
	unsigned int *Sun_Sensor_Addr;
	Sun_Sensor_Addr = &ADC_Buffer[14];

	ADC_Locations   =  ADC_SUN_SENSOR_MAX_LIMIT - 1;
	for(ADC_Addr_Count = 0;ADC_Addr_Count<=ADC_Locations;ADC_Addr_Count++)
	{
		tempdata =  (char)(*Sun_Sensor_Addr++ >>3); // Update to TM Global Buffer
		TM.Buffer.TM_ADC_Data[ADC_Addr_Count] = tempdata;
		ss_data_2[ADC_Addr_Count] = TM.Buffer.TM_ADC_Data[ADC_Addr_Count];
		ST_normal.ST_NM_Buffer.TM_ADC_Data[ADC_Addr_Count] = tempdata;

	}

	TM.Buffer.TM_SA_current 	= ADC_Buffer[32];      //SA_current
	TM.Buffer.TM_Bus_current 	= ADC_Buffer[33];      //Bus_Current
	TM.Buffer.TM_Bus_voltage2 	= ADC_Buffer[39];      //38 //Bus_voltage
	TM.Buffer.TM_Bus_voltage 	= ADC_Buffer[38];      //34 //Bus_voltage
	ST_normal.ST_NM_Buffer.TM_Bus_voltage = ADC_Buffer[34];      //Bus_voltage
	ST_normal.ST_NM_Buffer.TM_Bus_current = ADC_Buffer[33];      //Bus_Current

	tempdata1 = ADC_Buffer[32];
	ST_normal.ST_NM_Buffer.TM_SA_current = (unsigned short)(tempdata1 & 0xFFFF);

}

void rHAL_ADC_StatusREG_Enable()
{
	uint16 tempdata;
	tempdata = 0x00000001;
	ADC_STATUS_REGISTER = tempdata;
}

