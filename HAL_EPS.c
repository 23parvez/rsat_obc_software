#include "HAL_EPS.h"
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "Telecommand.h"
#include "TC_List.h"

void rHAL_EPS_shunt_switch()
{
	unsigned short tempdata;
        //ADC data should be available in ADC_Buffer
		//if((EPS_RAW_BUS_VOLTAGE >= 0x00000517))//16.4

	    //if((EPS_RAW_BUS_VOLTAGE >= 0x0000042a))//16.4
	    if((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA1_SHUNT_UTP))
		{

			//IO_Latch_Register_2_Data = IO_Latch_Register_2_Data | 0x00001000;//c
			Out_latch_5.SA1_ON_OFF	= 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}
		if((EPS_RAW_BUS_VOLTAGE <= 0x00000410))//16//change to 4E7
		//if((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA1_SHUNT_LTP))
		{
//			IO_Latch_Register_2_Data = IO_Latch_Register_2_Data & 0x0000EFFF;
//			IO_LATCH_REGISTER_2 = IO_Latch_Register_2_Data;
			Out_latch_5.SA1_ON_OFF	= 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}
		if((EPS_RAW_BUS_VOLTAGE >= 0x00000444))//16.8
		//if((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA2_SHUNT_UTP))
		{
//			IO_Latch_Register_2_Data = IO_Latch_Register_2_Data | 0x00002000;//c
//			IO_LATCH_REGISTER_2 = IO_Latch_Register_2_Data;
			Out_latch_5.SA2_ON_OFF	= 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}
		if((EPS_RAW_BUS_VOLTAGE <= 0x0000042a))//16.4//chnage to 507
		//if((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA2_SHUNT_LTP))
		{
//			IO_Latch_Register_2_Data = IO_Latch_Register_2_Data & 0x0000DFFF;
//			IO_LATCH_REGISTER_2 = IO_Latch_Register_2_Data;
			Out_latch_5.SA2_ON_OFF	= 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;

		}
		if((EPS_RAW_BUS_VOLTAGE >= 0x00000451))//17 //chnage to 547
		//if((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA3_SHUNT_UTP))
		{
//			IO_Latch_Register_2_Data = IO_Latch_Register_2_Data | 0x00000800;//c
//			IO_LATCH_REGISTER_2 = IO_Latch_Register_2_Data;
			Out_latch_5.SA3_ON_OFF	= 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}
		if((EPS_RAW_BUS_VOLTAGE <= 0x00000437))//16.6//chnage to 517
			//if((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA3_SHUNT_LTP))
		{
//			IO_Latch_Register_2_Data = IO_Latch_Register_2_Data & 0x0000F7FF;
//			IO_LATCH_REGISTER_2 = IO_Latch_Register_2_Data;
			Out_latch_5.SA3_ON_OFF	= 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}
}

void rHAL_EPS_Hardware_Status()
{
	unsigned short tempdata;
     for(inter_HAL_EPS_Hardware_Status_count = 0;inter_HAL_EPS_Hardware_Status_count<=7;inter_HAL_EPS_Hardware_Status_count++)
     {
    	 IO_Latch_Register_5_Data = IO_Latch_Register_5_Data & 0xFFFFFFF8;
    	 IO_Latch_Register_5_Data = IO_Latch_Register_5_Data | inter_HAL_EPS_Hardware_Status_SEL[inter_HAL_EPS_Hardware_Status_count];
    	 tempdata = IO_Latch_Register_5_Data;
    	 IO_LATCH_REGISTER_5 = tempdata;


     }
}
