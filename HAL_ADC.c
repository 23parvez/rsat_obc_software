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
		REG32(ADC_Addr++) = REG32(ADC_Buffer_Addr++) & EXTRACT_LSB_11;
	}
}

void rHAL_ADC_TM_Copy(uint32* ADC_Addr)
{
	ADC_Locations   =  ADC_MAX_LIMIT - 1;
	for(ADC_Addr_Count = 0;ADC_Addr_Count<=ADC_Locations;ADC_Addr_Count++)
	{
		TM.Buffer.TM_ADC_Data[ADC_Addr_Count]= (char)(*ADC_Addr++ >> 3);//Update to TM Global Buffer
	}
}

void rHAL_ADC_StatusREG_Enable()
{
	uint16 tempdata;
	tempdata = 0x00000001;
	ADC_STATUS_REGISTER = tempdata;
}

