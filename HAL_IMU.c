#include "HAL_IMU.h"
#include "Global.h"
#include "HAL_Address.h"
#include "HAL_Global.h"

#include "adcs_SensorDataProcs.h"
#include "adcs_VarDeclarations.h"

#include "TM_Global_Buffer.h"
#include "Telemetry.h"

//IMU STATUS REGISTER:
//[15:12] - Configure Bytes
//[11:7]  - NA
//[6]     - Data Not Available Status
//[5]     - Sensor Enable
//[4]     - Diagnostics Enable
//[3]     - Configure Enable
//[2]     - Buffer Read Enable
//[1]     - Data Not Ready
//[0]     - Data Ready

void rIMU_Init(void)
{
	//Initialize IMU1 Registers
	IMU_1.IMU_Status_Register_1    = IMU_1_STATUS_REGISTER_1;
	IMU_1.IMU_Status_Register_2    = IMU_1_STATUS_REGISTER_2;
	IMU_1.IMU_Buffer_Address       = IMU_1_BUFFER_BASE;
	IMU_1.IMU_Configure_Register   = IMU_1_CONFIG_BASE;

	//Initialize IMU2 Registers
	IMU_2.IMU_Status_Register_1    = IMU_2_STATUS_REGISTER_1;
	IMU_2.IMU_Status_Register_2    = IMU_2_STATUS_REGISTER_2;
	IMU_2.IMU_Buffer_Address       = IMU_2_BUFFER_BASE;
	IMU_2.IMU_Configure_Register   = IMU_2_CONFIG_BASE;
}

unsigned long int HAL_IMU_Read(struct HAL_IMU_Data_Structure IMU_No, unsigned long int *IMU_Addr)
{
	inter_HAL_IMU_Buffer_Addr = (unsigned long int*)IMU_No.IMU_Buffer_Address;
	inter_HAL_IMU_Locations   = IMU_DATA_BUFFER_MAX_LIMIT - 1;
	inter_HAL_IMU_Status_Data = REG32(IMU_No.IMU_Status_Register_2) & EXTRACT_LSB_16;

	if((inter_HAL_IMU_Status_Data & IMU_DATA_READY) == IMU_DATA_READY)            // Check if Data Ready Bit is Set
	{
		TM.Buffer.TM_inter_HAL_IMU_Status_Data_2 = inter_HAL_IMU_Status_Data & EXTRACT_LSB_16;
		REG32(IMU_No.IMU_Status_Register_1) = inter_HAL_IMU_Status_Data | IMU_SENSOR_ENABLE | IMU_READ_ENABLE; //  Set Read Enable Bit
		for(inter_HAL_IMU_Addr_Count = 0;inter_HAL_IMU_Addr_Count<=inter_HAL_IMU_Locations;inter_HAL_IMU_Addr_Count++)
		{
			REG32(IMU_Addr++) = REG32(inter_HAL_IMU_Buffer_Addr++) & EXTRACT_LSB_16;
		}
	    REG32(IMU_No.IMU_Status_Register_1) = inter_HAL_IMU_Status_Data & IMU_READ_DATA_READY_DISABLE ; // Reset Read Enable Bit and Data ready bit
	    IMU_Data_Available = TRUE;
	}
	else
	{
		for(inter_HAL_IMU_Addr_Count = 0;inter_HAL_IMU_Addr_Count<=inter_HAL_IMU_Locations;inter_HAL_IMU_Addr_Count++)
		{
			REG32(IMU_Addr++) = REG32(inter_HAL_IMU_Buffer_Addr++) & EXTRACT_LSB_16_RESET;
		}

		IMU_Data_Available = FALSE;
	}
    return IMU_Data_Available;
}
