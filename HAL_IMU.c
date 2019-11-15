#include "HAL_IMU.h"
#include "Global.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "adcs_VarDeclarations.h"

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
	IMU_1.IMU_Status_Register    = IMU_1_STATUS_REGISTER;
	IMU_1.IMU_Buffer_Address     = IMU_1_BUFFER_BASE;
	IMU_1.IMU_Configure_Register = IMU_1_CONFIG_BASE;
	//Initialize IMU2 Registers
	IMU_2.IMU_Status_Register    = IMU_2_STATUS_REGISTER;
	IMU_2.IMU_Buffer_Address     = IMU_2_BUFFER_BASE;
	IMU_2.IMU_Configure_Register = IMU_2_CONFIG_BASE;
}

int imu_data_ready_count;
unsigned long int HAL_IMU_Read(struct HAL_IMU_Data_Structure IMU_No, unsigned long int *IMU_Addr)
{
	inter_HAL_IMU_Buffer_Addr = (unsigned long int*)IMU_No.IMU_Buffer_Address;
	inter_HAL_IMU_Locations   = IMU_DATA_BUFFER_MAX_LIMIT - 1;
	inter_HAL_IMU_Status_Data = REG32(IMU_No.IMU_Status_Register) & EXTRACT_LSB_16;
	if((inter_HAL_IMU_Status_Data & IMU_DATA_READY) == IMU_DATA_READY)            // Check if Data Ready Bit is Set
	{
		tempat = IMU_2.IMU_Status_Register;
		REG32(IMU_No.IMU_Status_Register) = inter_HAL_IMU_Status_Data | IMU_SENSOR_ENABLE | IMU_READ_ENABLE; //  Set Read Enable Bit
		for(inter_HAL_IMU_Addr_Count = 0;inter_HAL_IMU_Addr_Count<=inter_HAL_IMU_Locations;inter_HAL_IMU_Addr_Count++)
		{
			REG32(IMU_Addr++) = REG32(inter_HAL_IMU_Buffer_Addr++) & EXTRACT_LSB_16;
		}
	    REG32(IMU_No.IMU_Status_Register) = inter_HAL_IMU_Status_Data & IMU_READ_DATA_READY_DISABLE ; // Reset Read Enable Bit and Data ready bit
	    IMU_Data_Available = TRUE;
	}
	else
	{
		imu_data_ready_count++;
		for(inter_HAL_IMU_Addr_Count = 0;inter_HAL_IMU_Addr_Count<=inter_HAL_IMU_Locations;inter_HAL_IMU_Addr_Count++)
		{
			REG32(IMU_Addr++) = REG32(inter_HAL_IMU_Buffer_Addr++) & EXTRACT_LSB_16_RESET;
		}

		IMU_Data_Available = FALSE;
	}
    return IMU_Data_Available;
}


unsigned long int HAL_IMU_Config(struct HAL_IMU_Data_Structure* IMU_No, unsigned long int *IMU_Configure_Addr,unsigned long int IMU_Configure_Words)
{

	if(IMU_Configure_Words >= 1 & IMU_Configure_Words <= IMU_CONFIG_BUFFER_MAX_LIMIT) // FPGA Configure Buffer Size is 15 Words
	{
		inter_HAL_IMU_Status_Data = (REG32(IMU_No->IMU_Status_Register) & EXTRACT_LSB_16); // Read Status Register Data

		if((inter_HAL_IMU_Status_Data & IMU_CONFIG_ENABLE) == IMU_CONFIG_DISABLE) // Check if Configure Enable Bit is Reset
		{
			inter_HAL_IMU_Locations   = IMU_Configure_Words - 1;
			inter_HAL_IMU_Configure_Address = (unsigned long int*)(IMU_No->IMU_Configure_Register);

			REG32(IMU_No->IMU_Status_Register) = inter_HAL_IMU_Status_Data & IMU_SENSOR_DISABLE; // Reset Sensor Enable
			for(inter_HAL_IMU_Addr_Count = 0;inter_HAL_IMU_Addr_Count<= inter_HAL_IMU_Locations;inter_HAL_IMU_Addr_Count++)
			{
				REG32(inter_HAL_IMU_Configure_Address++) = REG32(IMU_Configure_Addr++) & EXTRACT_LSB_16;
			}

			REG32(IMU_No->IMU_Status_Register) = IMU_CONFIG_ENABLE | ((IMU_Configure_Words & EXTRACT_LSB_4) << 12); // Set Configure Enable Bit & No of Config Words
			IMU_Config_Done = TRUE;
	    }
		else
		{
			IMU_Config_Done = FALSE;
		}
	}

	else
	{
		IMU_Config_Done = FALSE;
	}
	return IMU_Config_Done;
}

unsigned long int HAL_IMU_Diag(struct HAL_IMU_Data_Structure* IMU_No, unsigned short IMU_Diag_Data) // IMU_Daignostics Data is the Address of the Channel to be read
{
	inter_HAL_IMU_Status_Data = (REG32(IMU_No->IMU_Status_Register) & EXTRACT_LSB_16); // Read Status Register of IMU

	if((inter_HAL_IMU_Status_Data & IMU_DIAG_ENABLE) == IMU_DIAG_DISABLE) // Check if Diagnosis Enable Bit is Reset
	{
		REG32(IMU_No->IMU_Configure_Register) = (unsigned long int)IMU_Diag_Data; // Write IMU Diagnostics Address to Configuration Register
		REG32(IMU_No->IMU_Status_Register)    = inter_HAL_IMU_Status_Data | IMU_DIAG_ENABLE; // Set Diagnostics Enable Bit
		IMU_Diag_Done = TRUE;
	}
	else
	{
		IMU_Diag_Done = FALSE;
	}
    return IMU_Diag_Done;
}

void rHAL_IMU_POWER(unsigned long int IMU_No,unsigned long int IMU_Power)
{
	unsigned short tempdata;
	if(IMU_No == IMU1)
	{
		if(IMU_Power == ON)
		{
			Out_Latch_2.IMU1_ON_OFF = 1;
			Out_Latch_2.IMU1_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
			REG32(IMU_1_STATUS_REGISTER) = 0x00000020;
		}

		else if(IMU_Power == OFF)
		{
			Out_Latch_2.IMU1_ON_OFF = 0;
			Out_Latch_2.IMU1_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
			REG32(IMU_2_STATUS_REGISTER) = 0x00000000;
		}

		else
		{
			//
		}
	}
	else if(IMU_No == IMU2)
	{
		if(IMU_Power == ON)
		{
			Out_Latch_2.IMU2_ON_OFF = 1;
			Out_Latch_2.IMU2_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
			REG32(IMU_2_STATUS_REGISTER) = 0x00000020;
		}
		else if(IMU_Power == OFF)
		{
			Out_Latch_2.IMU2_ON_OFF = 0;
			Out_Latch_2.IMU2_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
			REG32(IMU_2_STATUS_REGISTER) = 0x00000000;
		}

		else
		{
			//
		}
	}
	else
	{
		//
	}
}
