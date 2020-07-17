#include "HAL_EPS.h"
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "Telemetry.h"
#include "TM_Global_Buffer.h"


void rHAL_EPS_shunt_switch()
{
	unsigned short tempdata;
        //ADC data should be available in ADC_Buffer
		//if((EPS_RAW_BUS_VOLTAGE >= 0x00000517))//16.4

	    if ((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA1_SHUNT_UTP))
		{
			Out_latch_5.SA1_ON_OFF	= 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}
		if ((EPS_RAW_BUS_VOLTAGE <= TC_data_command_Table.SA1_SHUNT_LTP))         // 16 // change to 4E7
		{
			Out_latch_5.SA1_ON_OFF	= 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}

		if ((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA2_SHUNT_UTP))      //16.8
		{
			Out_latch_5.SA2_ON_OFF	= 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}

		if ((EPS_RAW_BUS_VOLTAGE <= TC_data_command_Table.SA2_SHUNT_LTP))          // 16.4// chnage to 507
		{
			Out_latch_5.SA2_ON_OFF	= 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}

		if ((EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.SA3_SHUNT_UTP))             // 17 //chnage to 547
		{
			Out_latch_5.SA3_ON_OFF	= 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}

		if ((EPS_RAW_BUS_VOLTAGE <= TC_data_command_Table.SA3_SHUNT_LTP))          //16.6//chnage to 517
		{
			Out_latch_5.SA3_ON_OFF	= 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5 ;
			IO_LATCH_REGISTER_5 = tempdata;
		}

		/** Battery safe mode detection **/
			if (EPS_RAW_BUS_VOLTAGE <= TC_data_command_Table.TC_power_safe_LTP)
			{
				f_battery_safemode  = True;
				//heaters_manual.f_battery_safemode = f_battery_safemode;
				low_battery_voltage = True;
			}
			else if (EPS_RAW_BUS_VOLTAGE >= TC_data_command_Table.TC_power_safe_UTP)
			{
				f_battery_safemode  = False;
				//heaters_manual.f_battery_safemode = f_battery_safemode;
				low_battery_voltage = False;
				load_shedding_flag  = False;
			}

			if (f_battery_safemode && low_battery_voltage && (!load_shedding_flag))
				// perform load shedding on low battery voltage
				rHAL_Battery_load_shedding();
			// Load shedding flag update to TM
			TM.Buffer.TM_heaters_manual_control = ((load_shedding_flag << 6) |  heaters_manual.data);
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

void rHAL_Battery_load_shedding()
{
	// Turn off loads

	// X-Band TX
	rHAL_X_Tx_OFF();

	// Payload
	if (Out_Latch_3.PL1_ON_OFF == 1)
		Out_Latch_3.PL1_ON_OFF = 0;

	else if (Out_Latch_3.PL2_ON_OFF == 1)
		Out_Latch_3.PL2_ON_OFF = 0;

	load_shedding_flag = True;

}
