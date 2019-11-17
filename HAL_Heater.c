#include "HAL_Heater.h"
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "Telemetry.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "TM_Global_Buffer.h"

unsigned int Manual_flag ;

void Heater_control_auto_manual()
{
	//heaters.data=TC_data_command_Table.TC_heaters_auto_manual;
	heaters.data= 0x002A;

	if(heaters.heater_1 == AUTO_MODE)
	{

		Heater_1_on_off();

	}

	else if (heaters.heater_1 == MANUAL_MODE)
	{

		Manual_flag = Heater_1_manual;
		Heater_manual();

	}

	else
	{
		//
	}

	if(heaters.heater_2 == AUTO_MODE)
	{

		Heater_2_on_off();

	}

	else
	{

		Manual_flag = Heater_2_manual;
		Heater_manual();

	}

   if(heaters.heater_3 == AUTO_MODE)
	{

	   Heater_3_on_off();

	}

   else
   {

	   Manual_flag = Heater_3_manual;
	   Heater_manual();
	}

    if(heaters.heater_4 == AUTO_MODE)
	{

		Heater_4_on_off();
	}

	else
	{

		Manual_flag = Heater_4_manual;
		Heater_manual();

	}

    if(heaters.heater_5 == AUTO_MODE)
	{

		Heater_5_on_off();

	}

	else
	{

		Manual_flag = Heater_5_manual;
		Heater_manual();

	}

    if(heaters.heater_6 == AUTO_MODE)
	{

		Heater_6_on_off();

	}

	else
	{

		Manual_flag = Heater_6_manual;
		Heater_manual();

	}
}
void Heater_1_on_off()
{
	unsigned short tempdata;

	if(ADC_Buffer[0] >= LTP[0])                                              // 20 degree = 1.1 volt
	{

		Out_Latch_3.Heater1_ON_OFF           = 1;
		tempdata                             = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                  = tempdata;

	}

	else if(ADC_Buffer[0] <= UTP[0])                                         // 25 degree = 0.922509225 volt
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

void Heater_2_on_off()
{
	unsigned short tempdata;

	if(ADC_Buffer[1] >= LTP[1])
	{

		Out_Latch_3.Heater2_ON_OFF            = 1;
		tempdata                              = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3                   = tempdata;

	}

	else if(ADC_Buffer[1] <= UTP[1])
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

void Heater_3_on_off()
{
	unsigned short tempdata;

	if (ADC_Buffer[2] >= LTP[2])
	{

		Out_Latch_3.Heater3_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}
	else if (ADC_Buffer[2] <= UTP[2])
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

void Heater_4_on_off()
{
	unsigned short tempdata;

	if(ADC_Buffer[3] >= LTP[3])
	{

		Out_Latch_3.Heater4_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}

	else if(ADC_Buffer[3] <= UTP[3])
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

void Heater_5_on_off()
{
	unsigned short tempdata;

	if(ADC_Buffer[4] >= LTP[4])
	{

		Out_Latch_3.Heater5_ON_OFF        = 1;
		tempdata                          = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3               = tempdata;

	}
	else if(ADC_Buffer[4] <= UTP[4])
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

void Heater_6_on_off()
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

int ck;
void Heater_manual()
{
	unsigned short tempdata;

	heaters_manual.data = 0x0015;

	/*
	heaters_manual.data = TC_data_command_Table.TC_heaters_manual;
*/

	switch(Manual_flag)
	{
		ck = 1;
		case 1:Heater_1_manuals();
				break;
		case 2:Heater_2_manuals();
				break;
		case 3: Heater_3_manuals();
				break;
		case 4:Heater_4_manuals();
				break;
		case 5:Heater_5_manuals();
				break;
		case 6: Heater_6_manuals();
				break;
		default:
				break;
	}


/*	if(Manual_flag ==Heater_1_manual)
	{

		ck = 1;
		Heater_1_manuals();

	}

	else if (Manual_flag ==Heater_2_manual)
	{

		ck = 2;
		Heater_2_manuals();

	}

	else if (Manual_flag ==Heater_3_manual)
	{

		ck = 3;
		Heater_3_manuals();

	}

	else if (Manual_flag ==Heater_4_manual)
	{

		ck = 4;
		Heater_4_manuals();

	}

	else if (Manual_flag ==Heater_5_manual)
	{

		ck = 5;
		Heater_5_manuals();

	}

	else if (Manual_flag ==Heater_6_manual)
	{

		ck = 6;
		Heater_6_manuals();

	}

	else
	{
		ck = 7;
	}*/

}


void  Heater_1_manuals()
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


void  Heater_2_manuals()
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

void  Heater_3_manuals()
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

void  Heater_4_manuals()
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


void  Heater_5_manuals()
{
	unsigned short tempdata;

	if(heaters_manual.heater_5 == 1)
	{

		ck                                  = 8;
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

void  Heater_6_manuals()
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
