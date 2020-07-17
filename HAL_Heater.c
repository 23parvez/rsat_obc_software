
#include "HAL_Heater.h"
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "Telemetry.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "TM_Global_Buffer.h"

unsigned int Manual_flag;

void Thermister_select()
{
	int tempdata,Thermistor_Count;

	for(Thermistor_Count = 0; Thermistor_Count < Thermistor_MAX_LIMIT; Thermistor_Count++)
	{
		tempdata = Thermister_index_select[Thermistor_Count];
		Thermistor_Selection[Thermistor_Count]  = ADC_Buffer[tempdata];
	}
}

void UTP_selection()
{
	unsigned short tempdata1;
	unsigned int tempdata_utp;
	unsigned short index_utp ,index_utp1;
	tempdata_utp = TC_data_command_Table.BATTERY_HEATER1_UTP;
	UTP_data = tempdata_utp;

	index_utp = UTP_data ;
	index_utp1 = ((index_utp & 0xF000)>>12);
	tempdata1 = (UTP_data & 0x0FFF);
	UTP[index_utp1] = tempdata1;
}

unsigned int ltp12;
void LTP_selection()
{
	unsigned short tempdata2,index_ltp,index_ltp1;
	unsigned int tempdata_ltp;

	tempdata_ltp = TC_data_command_Table.BATTERY_HEATER1_LTP;
	LTP_data = tempdata_ltp;
	index_ltp = LTP_data;
	index_ltp1 = (index_ltp & 0xF000)>>12;
	ltp12 = index_ltp1;
	tempdata2 = (LTP_data & 0x0FFF);
	LTP[index_ltp1] = tempdata2;
}

void Heater_control_auto_manual()
{
	 unsigned int tempdata;

	 Thermister_select();

	 if(load_shedding_flag)
	 {
		 // putting heater in manual control
		 tempdata = TC_data_command_Table.TC_heaters_auto_manual;
		 heaters.data = tempdata & 0x00000003;

		 // manual mode heater on off control
		 tempdata = TC_data_command_Table.TC_heaters_manual;
		 heaters_manual.data = (unsigned short)tempdata & 0x3;
	 }
	 else
	 {
		 // auto - manual control
		 tempdata = TC_data_command_Table.TC_heaters_auto_manual;
		 heaters.data = (unsigned short)tempdata;

		 // manual mode heater on off control
		 tempdata = TC_data_command_Table.TC_heaters_manual;
		 heaters_manual.data = (unsigned short)tempdata;

	 }

	 //TM.Buffer.TM_auto_manual=heaters.data;
	 // add heaters_manual.data in TM

	if(heaters.heater_1 == AUTO_MODE)
	{

		Heater_1_auto_control();
		//TM.Buffer.TM_heaters_auto_manual = heaters.data;

	}

	else
	{
		Heater_1_manual_control();
		//TM.Buffer.TM_heaters_auto_manual = heaters.data;


	}


	if(heaters.heater_2 == AUTO_MODE)
	{

		Heater_2_auto_control();

	}

	else
	{
		Heater_2_manual_control();

	}

   if (heaters.heater_3 == AUTO_MODE)
	{

		Heater_3_auto_control();

	}

   else
   {
		Heater_3_manual_control();
	}

    if (heaters.heater_4 == AUTO_MODE)
	{
		Heater_4_auto_control();
	}

	else
	{
		Heater_4_manual_control();

	}

    if(heaters.heater_5 == AUTO_MODE)
	{
		Heater_5_auto_control();

	}

	else
	{
		Heater_5_manual_control();

	}

    if(heaters.heater_6 == AUTO_MODE)
	{

		Heater_6_auto_control();

	}

	else
	{
		Heater_6_manual_control();

	}

    if(heaters.heater_7 == AUTO_MODE)
   	{

		Heater_7_auto_control();

   	}

   	else
   	{
		Heater_7_manual_control();

   	}

    if(heaters.heater_8 == AUTO_MODE)
      	{

			Heater_8_auto_control();

      	}

      	else
      	{
    		Heater_8_manual_control();

      	}
    if(heaters.heater_9 == AUTO_MODE)
		{

			Heater_9_auto_control();

		}

		else
		{
			Heater_9_manual_control();

		}

    if(heaters.heater_10 == AUTO_MODE)
	{

		Heater_10_auto_control();

	}

	else
	{
		Heater_10_manual_control();

	}

    if(heaters.heater_11 == AUTO_MODE)
   	{

		Heater_11_auto_control();

   	}

   	else
   	{
		Heater_11_manual_control();

   	}

    if(heaters.heater_12 == AUTO_MODE)
	{

		Heater_12_auto_control();

	}

	else
	{
		Heater_12_manual_control();

	}

    if(heaters.heater_13 == AUTO_MODE)
      	{

    		Heater_13_auto_control();

      	}

      	else
      	{
    		Heater_13_manual_control();

      	}

    if(heaters.heater_14 == AUTO_MODE)
	{

    	Heater_14_auto_control();

	}

	else
	{
		Heater_14_manual_control();

	}
    TM.Buffer.TM_heaters_auto_manual    = heaters.data;
    TM.Buffer.TM_heaters_manual_control = heaters_manual.data;

}
void Heater_1_auto_control()
{
	unsigned short tempdata;

	if (Thermistor_Selection[0] >= LTP[0])                                            // 20 degree = 1.1 volt
	{

		Out_Latch_3.Heater1_ON_OFF           = 1;
		tempdata                             = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                  = tempdata;


	}

	else if(Thermistor_Selection[0] <= UTP[0])                                         // 25 degree = 0.922509225 volt
	{

		Out_Latch_3.Heater1_ON_OFF           = 0;
		tempdata                             = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                  = tempdata;

	}

	else
	{
		//
	}

}

void Heater_2_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[1] >= LTP[1])
	{

		Out_Latch_3.Heater2_ON_OFF            = 1;
		tempdata                              = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                   = tempdata;

	}

	else if(Thermistor_Selection[1] <= UTP[1])
	{

		Out_Latch_3.Heater2_ON_OFF            = 0;
		tempdata                              = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                   = tempdata;

	}

	else
	{
		//
	}

}

void Heater_3_auto_control()
{
	unsigned short tempdata;

	if (Thermistor_Selection[2] >= LTP[2])
	{

		Out_Latch_3.Heater3_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}
	else if (Thermistor_Selection[2] <= UTP[2])
	{

		Out_Latch_3.Heater3_ON_OFF        = 0;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}
	else
	{
		//
	}
}

void Heater_4_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[3] >= LTP[3])
	{

		Out_Latch_3.Heater4_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}

	else if(Thermistor_Selection[3] <= UTP[3])
	{

		Out_Latch_3.Heater4_ON_OFF        = 0;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}

	else
	{
		//
	}
}

void Heater_5_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[4] >= LTP[4])
	{

		Out_Latch_3.Heater5_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}
	else if(Thermistor_Selection[4] <= UTP[4])
	{

		Out_Latch_3.Heater5_ON_OFF        = 0;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}
	else
	{
		//
	}
}

void Heater_6_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[5] >= LTP[5])
	{

		Out_Latch_3.Heater6_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}

	else if(Thermistor_Selection[5] <= UTP[5])
	{

		Out_Latch_3.Heater6_ON_OFF         = 0;
		tempdata                           = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                = tempdata;

	}

	else
	{
		//
	}

}

void Heater_7_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[6] >= LTP[6])
	{

		heaters_auto.heater_7= 1;
	}

	else if(Thermistor_Selection[6] <= UTP[6])
	{

		heaters_auto.heater_7= 0;

	}

	else
	{
		//
	}

}

void Heater_8_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[7] >= LTP[7])
	{

		heaters_auto.heater_8 = 1;

	}

	else if(Thermistor_Selection[7] <= UTP[7])
	{

		heaters_auto.heater_8 = 0;
	}

	else
	{
		//
	}

}

void Heater_9_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[8] >= LTP[8])
	{
		heaters_auto.heater_9 = 1;

	}

	else if(Thermistor_Selection[8] <= UTP[8])
	{


		heaters_auto.heater_9 = 0;
	}

	else
	{
		//
	}

}

void Heater_10_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[9] >= LTP[9])
	{

		heaters_auto.heater_10 =1;

	}

	else if(Thermistor_Selection[9] <= UTP[9])
	{

		heaters_auto.heater_10 =0;

	}

	else
	{
		//
	}

}

void Heater_11_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[10] >= LTP[10])
	{

		heaters_auto.heater_11 = 1;
	}

	else if(Thermistor_Selection[10] <= UTP[10])
	{

		heaters_auto.heater_11 = 0;

	}

	else
	{
		//
	}

}


void Heater_12_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[11] >= LTP[11])
	{

		heaters_auto.heater_12 = 1;

	}

	else if(Thermistor_Selection[11] <= UTP[11])
	{
		heaters_auto.heater_12 = 0;

	}

	else
	{
		//
	}

}

void Heater_13_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[12] >= LTP[12])
	{
		heaters_auto.heater_13 = 1;

	}

	else if(Thermistor_Selection[12] <= UTP[12])
	{

		heaters_auto.heater_13 = 0;
	}

	else
	{
		//
	}

}

void Heater_14_auto_control()
{
	unsigned short tempdata;

	if(Thermistor_Selection[13] >= LTP[13])
	{

		heaters_auto.heater_14 = 1;

	}

	else if(Thermistor_Selection[13] <= UTP[13])
	{

		heaters_auto.heater_14 = 0;

	}

	else
	{
		//
	}

}

/*void Heater_15_on_off()
{
	unsigned short tempdata;

	if(ADC_Buffer[5] >= LTP[5])
	{

		Out_Latch_3.Heater6_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}

	else if(ADC_Buffer[5] <= UTP[5])
	{

		Out_Latch_3.Heater6_ON_OFF         = 0;
		tempdata                           = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                = tempdata;

	}

	else
	{
		//
	}

}

void Heater_16_on_off()
{
	unsigned short tempdata;

	if(ADC_Buffer[5] >= LTP[5])
	{

		Out_Latch_3.Heater6_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}

	else if(ADC_Buffer[5] <= UTP[5])
	{

		Out_Latch_3.Heater6_ON_OFF         = 0;
		tempdata                           = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                = tempdata;

	}

	else
	{
		//
	}

}*/

void  Heater_1_manual_control()
{

	unsigned short tempdata;

	if(heaters_manual.heater_1)
	{

		Out_Latch_3.Heater1_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}

	else
	{

		Out_Latch_3.Heater1_ON_OFF        = 0;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}
}


void Heater_2_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_2)
	{

		Out_Latch_3.Heater2_ON_OFF         = 1;
		tempdata                           = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                = tempdata;

	}

	else
	{

		Out_Latch_3.Heater2_ON_OFF         = 0;
		tempdata                           = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                = tempdata;

	}

}

void Heater_3_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_3)
	{

		Out_Latch_3.Heater3_ON_OFF         = 1;
		tempdata                           = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                = tempdata;

	}

	else
	{

		Out_Latch_3.Heater3_ON_OFF         = 0;
		tempdata                           = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                = tempdata;

	}

}

void Heater_4_manual_control()
{

	unsigned short tempdata;

	if(heaters_manual.heater_4)
	{

		Out_Latch_3.Heater4_ON_OFF          = 1;
		tempdata                            = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                 = tempdata;

	}

	else
	{

		Out_Latch_3.Heater4_ON_OFF          = 0;
		tempdata                            = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                 = tempdata;

	}
}


void Heater_5_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_5)
	{
		Out_Latch_3.Heater5_ON_OFF          = 1;
		tempdata                            = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                 = tempdata;
	}

	else
	{
		Out_Latch_3.Heater5_ON_OFF          = 0;
		tempdata                            = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                 = tempdata;

	}
}

void Heater_6_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_6)
	{
		Out_Latch_3.Heater6_ON_OFF          = 1;
		tempdata                            = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                 = tempdata;

	}

	else
	{
		Out_Latch_3.Heater6_ON_OFF          = 0;
		tempdata                            = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                 = tempdata;

	}
}

void Heater_7_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_7)
	{

		manual_on_off.heater_7 = 1;
	}

	else
	{

		manual_on_off.heater_7 = 0;
	}
}

void Heater_8_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_8)
	{

		manual_on_off.heater_8 = 1;

	}

	else
	{
		manual_on_off.heater_8 = 0;
	}
}

void Heater_9_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_9)
	{

		manual_on_off.heater_9 = 1;
	}

	else
	{
		manual_on_off.heater_9 = 0;
	}
}

void Heater_10_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_10)
	{

		manual_on_off.heater_10 = 1;

	}

	else
	{
		manual_on_off.heater_10 = 0;

	}
}

void Heater_11_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_11)
	{

		manual_on_off.heater_11 = 1;

	}

	else
	{

		manual_on_off.heater_11 = 0;
	}
}

void Heater_12_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_12)
	{

		manual_on_off.heater_12 = 1;

	}

	else
	{
		manual_on_off.heater_12 = 0;
	}
}

void Heater_13_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_13)
	{

		manual_on_off.heater_13 = 1;

	}

	else
	{
		manual_on_off.heater_13 = 0;

	}
}

void Heater_14_manual_control()
{
	unsigned short tempdata;

	if(heaters_manual.heater_14)
	{

		manual_on_off.heater_14 = 1;

	}

	else
	{
		manual_on_off.heater_14 = 0;

	}
}

/*

void rHAL_Heater_Power(unsigned int Heater_No,unsigned int Heater_Power)
{

	unsigned short tempdata;
	if(Heater_Power == ON)
	{
		switch(Heater_No)
		{
			case 1:	Out_Latch_3.Heater1_ON_OFF = 1;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 2:	Out_Latch_3.Heater2_ON_OFF = 1;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 3:	Out_Latch_3.Heater3_ON_OFF = 1;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 4:	Out_Latch_3.Heater4_ON_OFF = 1;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 5:	Out_Latch_3.Heater5_ON_OFF = 1;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 6:	Out_Latch_3.Heater6_ON_OFF = 1;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;
			default:
					break;
		}
	}
	else if(Heater_Power == OFF)
	{
		switch(Heater_No)
		{
			case 1:	Out_Latch_3.Heater1_ON_OFF = 0;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 2:	Out_Latch_3.Heater2_ON_OFF = 0;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 3:	Out_Latch_3.Heater3_ON_OFF = 0;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 4:	Out_Latch_3.Heater4_ON_OFF = 0;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 5:	Out_Latch_3.Heater5_ON_OFF = 0;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;

			case 6:	Out_Latch_3.Heater6_ON_OFF = 0;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
					break;
			default:
					break;
		}
	}
}
*/



/*void thermistor_swaping()
{
	unsigned short tempdata;
	unsigned char thermistor_index;
	unsigned char thermistor_value;

	tempdata = TC_data_command_Table.thermistor_no;
	thermistor_index = (char)(tempdata >> 8);
	thermistor_value = (char)(tempdata & 0x00FF);

	thermistor_select[thermistor_index] = thermistor_value;

}
*/
