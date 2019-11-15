
#include "HAL_GPS.h"
#include "TM_Global_Buffer.h"
#include "HAL_Global.h"
#include "Global.h"
#include "Telemetry.h"
#include "HAL_Address.h"
#include "TC_List.h"
#include "adcs_VarDeclarations.h"

void rHAL_GPS_POWER(unsigned long int GPS_No,unsigned long int GPS_Power)
{
	unsigned short tempdata;
	if(GPS_No == GPS_1)
	{
		if(GPS_Power == ON)
		{
			Out_Latch_2.GPS1_ON_OFF = 1;
			Out_Latch_2.GPS1_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
		}

		else if(GPS_Power == OFF)
		{
			Out_Latch_2.GPS1_ON_OFF = 0;
			Out_Latch_2.GPS1_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
		}

		else
		{
			//
		}
	}
	else if(GPS_No == GPS_2)
	{
		if(GPS_Power == ON)
		{
			Out_Latch_2.GPS2_ON_OFF = 1;
			Out_Latch_2.GPS2_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
		}
		else if(GPS_Power == OFF)
		{
			Out_Latch_2.GPS2_ON_OFF = 0;
			Out_Latch_2.GPS2_RESET = 1;
			tempdata =  Out_Latch_2.data;
			IO_LATCH_REGISTER_2;
			IO_LATCH_REGISTER_2 = tempdata;
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

void GPS_1_DATA()
{
	if(TC_boolean_u.TC_Boolean_Table.TC_GPS12_Select == GPS_1)
	{
		rHAL_GPS_Read(GPS_1,GPS_RCVD_DATA,212);
	}
	else if(TC_boolean_u.TC_Boolean_Table.TC_GPS12_Select == GPS_2)
	{
		rHAL_GPS_Read(GPS_2,GPS_RCVD_DATA,212);
	}
	else
	{
		//
	}
}

void rGPS_pulsecheck()
{
  gps_obt_counter = 0;
  gps_obt_counter = (REG32(0x200009A0) & 0x0000FFFF) << 16;
  gps_obt_counter = gps_obt_counter | (REG32(0x200009A4) & 0x0000FFFF);
  // pps_deltaT = gps_obt_counter;
  TM.Buffer.gps_pulse_mic_counter = (short)(gps_obt_counter & 0x0000FFFF);

  if(gps_obt_count_prev != gps_obt_counter)
  {
	  // gps_obt_count_prev = gps_obt_counter;
	  GPS_pulse_rcvd = 1;
	  // gps_pulse_mic_cnt = 0;

  }

  if(GPS_pulse_rcvd == 1)
  {
	  gps_pulse_mic_cnt++;
	 // TM.Buffer.gps_pulse_mic_counter = gps_pulse_mic_cnt;
  }
}

//unsigned long int rHAL_GPS_Read(unsigned long int GPS_No, unsigned long int* GPS_Addr,unsigned long int No_of_Bytes)
//{
//
//  if(No_of_Bytes %2 == 0)//Check for Even Or Odd
//  {
//      GPS_Locations = No_of_Bytes / 2 ;
//      GPS_Locations = GPS_Locations - 1;
//  }
//  else if(No_of_Bytes %2 == 1)
//  {
//	  GPS_Locations = (No_of_Bytes + 1) / 2 ;
//	  GPS_Locations = GPS_Locations - 1;
//  }
//  else
//  {
//	  GPS_Locations = 0;
//  }
//
//  if(GPS_No == GPS_1)
//  {
//
//	  GPS_Buffer_Addr = GPS1_BUFFER_BASE;
//	  GPS_Status_Data = GPS1_STATUS_REGISTER;
//
//	  if((GPS_Status_Data & 0x00000048) == 0x00000008)//0b0000_0000_0000_1000 ------Check Data Ready Bit
//	  {
//		  GPS1_STATUS_REGISTER = (GPS_Status_Data | 0x00000020);//0b0000_0000_0010_0000 ---------Set Read Enable Bit
//
//		  for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
//		  {
//			  GPS_Data = REG32(GPS_Buffer_Addr);
//			  GPS_Data = GPS_Data & 0x0000FFFF;
//			  GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
//			  GPS_Buffer_Addr = GPS_Buffer_Addr + 0x00000004;
//		  }
//
//		  GPS1_STATUS_REGISTER = (GPS_Status_Data & 0x0000FFDF);//0b1111_1111_1101_1111------------Reset Read Enable Bit
//
//		  GPS_Buffer_Addr = 0x20000800;//Reset GPS Buffer
//
//		  //------------------------------Data Formating--------------------------------------------
//		  for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
//		  {
//			  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count];
//			  GPS_Data = (( GPS_Data & 0x000000FF) << 8);
//			  GPS_Data = ((( GPS_Buffer_Data[GPS_Addr_Count] & 0x0000FF00) >> 8) | GPS_Data);
//			  GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
//		  }
//		  for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count=GPS_Addr_Count+2)
//		  {
//			  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count];
//			  GPS_Data = ((GPS_Data & 0x0000FFFF) << 16);
//			  GPS_Data = ((GPS_Buffer_Data[GPS_Addr_Count+1] & 0x0000FFFF) | GPS_Data);
//			  REG32(GPS_Addr) = GPS_Data;
//			  GPS_Addr =  GPS_Addr + 0x00000001;
//		  }
//		  //------------------------------Data Formating--------------------------------------------
//
//		  GPS_Data_Read_Status = TRUE;
//	  }
//	  else
//	  {
//		  GPS_Data_Read_Status = FALSE;
//	  }
//  }
//  else if(GPS_No == GPS_2)
//  {
//	  GPS_Buffer_Addr = GPS2_BUFFER_BASE;
//	  GPS_Status_Data = GPS2_STATUS_REGISTER;
//
//	  if((GPS_Status_Data & 0x00000048) == 0x00000008)//0b0000_0000_0000_1000 ------Check Data Ready Bit
//	  {
//	  	 GPS2_STATUS_REGISTER = (GPS_Status_Data | 0x00000020);//0b0000_0000_0010_0000 ---------Set Read Enable Bit
//
//	  	 for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
//	  	 {
//	  		 GPS_Data = REG32(GPS_Buffer_Addr);
//	  		 GPS_Data = GPS_Data & 0x0000FFFF;
//	  		 GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
//	  		 GPS_Buffer_Addr = GPS_Buffer_Addr + 0x00000004;
//	  	 }
//
//	  	 GPS2_STATUS_REGISTER = (GPS_Status_Data & 0x0000FFDF);//0b1111_1111_1101_1111------------Reset Read Enable Bit
//
//	  	 GPS_Buffer_Addr = 0x20000800;//Reset GPS Buffer
//
//	  	 //------------------------------Data Formating--------------------------------------------
////	  	 for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
////	  	 {
////	  		  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count];
////	  		  GPS_Data = (( GPS_Data & 0x000000FF) << 8);
////	  		  GPS_Data = ((( GPS_Buffer_Data[GPS_Addr_Count] & 0x0000FF00) >> 8) | GPS_Data);
////	  		  GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
////	  	}
////	  	 for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count=GPS_Addr_Count+2)
////	  	 {
////	  		  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count];
////	  		GPS_Data = ((GPS_Data & 0x0000FFFF) << 16);
////	  		GPS_Data = ((GPS_Buffer_Data[GPS_Addr_Count+1] & 0x0000FFFF) | GPS_Data);
////	  		 REG32(GPS_Addr) = GPS_Data;
////	  		 GPS_Addr =  GPS_Addr + 0x00000001;
////	  }
//	  	//------------------------------Data Formating--------------------------------------------
//
//	     GPS_Data_Read_Status = TRUE;
//	 }
//	 else
//	 {
//	  	 GPS_Data_Read_Status = FALSE;
//	 }
//  }
//
//  return GPS_Data_Read_Status;
//
//}

//TODO:Variables to Be removed only for Test : Issue with GPS Data Ready

unsigned int gps_ready_bit = 0;
unsigned long int rHAL_GPS_Read(unsigned long int GPS_No, unsigned long int* GPS_Addr,unsigned long int No_of_Bytes)
{

  if(No_of_Bytes %2 == 0)//Check for Even Or Odd
  {
      GPS_Locations = No_of_Bytes / 2 ;
      GPS_Locations = GPS_Locations - 1;
  }
  else if(No_of_Bytes %2 == 1)
  {
	  GPS_Locations = (No_of_Bytes + 1) / 2 ;
	  GPS_Locations = GPS_Locations - 1;
  }
  else
  {
	  GPS_Locations = 0;
  }

  if(GPS_No == GPS_1)
  {
	  GPS_Buffer_Addr = GPS1_BUFFER_BASE;
	  GPS_Status_Data = GPS1_STATUS_REGISTER;
//TODO: To be removed : Issue with GPS Data Ready
	  /*gps_obt_counter = 0;
	  gps_obt_counter = (REG32(0x200009A0) & 0x0000FFFF) << 16;
	  gps_obt_counter = gps_obt_counter | (REG32(0x200009A4) & 0x0000FFFF);
	  pps_deltaT = gps_obt_counter;*/

	  if((GPS_Status_Data & 0x00000048) == 0x00000008)//0b0000_0000_0000_1000 ------Check Data Ready Bit
	  {
	  /*if(gps_obt_count_prev != gps_obt_counter) //TODO: To be removed : Issue with GPS Data Ready
	  {
		  gps_obt_count_prev = gps_obt_counter;//TODO: To be removed : Issue with GPS Data Ready*/
		  gps_ready_bit = GPS_Status_Data;
		  GPS1_STATUS_REGISTER = (GPS_Status_Data | 0x00000020);//0b0000_0000_0010_0000 ---------Set Read Enable Bit
		  for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
		  {
			  //aaaa= 0xabcd;
			  GPS_Data = REG32(GPS_Buffer_Addr);
			  GPS_Data = GPS_Data & 0x0000FFFF;
			  GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
			  GPS_Buffer_Addr = GPS_Buffer_Addr + 0x00000004;
		  }

		  GPS1_STATUS_REGISTER = (GPS_Status_Data & 0x0000FFDF);//0b1111_1111_1101_1111------------Reset Read Enable Bit

		  GPS_Buffer_Addr = 0x20000800;//Reset GPS Buffer

		  //------------------------------Data Formating--------------------------------------------
//		  for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
//		  {
//			  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count];
//			  GPS_Data = (( GPS_Data & 0x000000FF) << 8);
//			  GPS_Data = ((( GPS_Buffer_Data[GPS_Addr_Count] & 0x0000FF00) >> 8) | GPS_Data);
//			  GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
//		  }
		  for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
		  {
			  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count++];
			  GPS_Data = ((GPS_Data & 0x0000FFFF) << 16);

			  GPS_Data = (GPS_Data |( GPS_Buffer_Data[GPS_Addr_Count] & 0x0000FFFF));
			  REG32(GPS_Addr) = GPS_Data;
			  GPS_Addr++;
		  }
		  //------------------------------Data Formating--------------------------------------------

		  if((GPS_Buffer_Data[0] == 0x00003F3F) && (GPS_Buffer_Data[1] == 0x0000AF01)) //Check for Valid Message header(0x3F3F) and Identifier(0xAF01)
			  GPS_Data_Read_Status = TRUE;

		  else
			  GPS_Data_Read_Status = FALSE;

		  GPSDataReady = 1;
	  }
	  else
	  {
		  GPS_Data_Read_Status = FALSE;
	  }

  }
  else if(GPS_No == GPS_2)
  {
	  GPS_Buffer_Addr = GPS2_BUFFER_BASE;
	  GPS_Status_Data = GPS2_STATUS_REGISTER;

	  if((GPS_Status_Data & 0x00000048) == 0x00000000)//0b0000_0000_0000_1000 ------Check Data Ready Bit
	  {
	  	 GPS2_STATUS_REGISTER = (GPS_Status_Data | 0x00000020);//0b0000_0000_0010_0000 ---------Set Read Enable Bit

	  	 for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
	  	 {
	  		 GPS_Data = REG32(GPS_Buffer_Addr);
	  		 GPS_Data = GPS_Data & 0x0000FFFF;
	  		 GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
	  		 GPS_Buffer_Addr = GPS_Buffer_Addr + 0x00000004;
	  	 }

	  	 GPS2_STATUS_REGISTER = (GPS_Status_Data & 0x0000FFDF);//0b1111_1111_1101_1111------------Reset Read Enable Bit

	  	 GPS_Buffer_Addr = 0x20000800;//Reset GPS Buffer

	  	 //------------------------------Data Formating--------------------------------------------
//	  	 for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count++)
//	  	 {
//	  		  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count];
//	  		  GPS_Data = (( GPS_Data & 0x000000FF) << 8);
//	  		  GPS_Data = ((( GPS_Buffer_Data[GPS_Addr_Count] & 0x0000FF00) >> 8) | GPS_Data);
//	  		  GPS_Buffer_Data[GPS_Addr_Count] = GPS_Data;
//	  	}
	  	 for(GPS_Addr_Count=0;GPS_Addr_Count<=GPS_Locations;GPS_Addr_Count=GPS_Addr_Count+2)
	  	 {
	  		  GPS_Data =  GPS_Buffer_Data[GPS_Addr_Count];
	  		GPS_Data = ((GPS_Data & 0x0000FFFF) << 16);
	  		GPS_Data = ((GPS_Buffer_Data[GPS_Addr_Count+1] & 0x0000FFFF) | GPS_Data);
	  		 REG32(GPS_Addr) = GPS_Data;
	  		 GPS_Addr =  GPS_Addr + 0x00000001;
	  }
	  	//------------------------------Data Formating--------------------------------------------

		  if((GPS_Buffer_Data[0] == 0x00003F3F) && (GPS_Buffer_Data[1] == 0x0000AF01)) //Check for Valid Message header(0x3F3F) and Identifier(0xAF01)
			  GPS_Data_Read_Status = TRUE;

		  else
			  GPS_Data_Read_Status = FALSE;
	  }
	  else
	  {
		  GPS_Data_Read_Status = FALSE;
	  }
  }

  return GPS_Data_Read_Status;

}

unsigned long int rHAL_GPS_Config(unsigned long int GPS_No,unsigned long int Config_Type)
{
  if(GPS_No == GPS_1)
  {
	 GPS_Status_Data = GPS1_STATUS_REGISTER;
	 if((GPS_Status_Data & 0x00000040) == 0x00000000)
	 {
		 if(Config_Type == NMEA_RMC_Enable)
		 {

             #ifdef ASCII
			 GPS1_CONFIG_REGISTER_1 = '$A';
			 GPS1_CONFIG_REGISTER_2 = 'CM';
			 GPS1_CONFIG_REGISTER_3 = 'CC';
			 GPS1_CONFIG_REGISTER_4 = ',3';
			 GPS1_CONFIG_REGISTER_5 = ',0';
			 GPS1_CONFIG_REGISTER_6 = 0x0000310d;
			 GPS1_CONFIG_REGISTER_7 = 0x00000a00;

			 GPS1_STATUS_REGISTER   = 0x00007A40;//0b_0111_1010_0100_0000
             #endif

             #ifndef ASCII
			 GPS1_CONFIG_REGISTER_1 = 0x00003f3f;
			 GPS1_CONFIG_REGISTER_2 = 0x00000A0C;
			 GPS1_CONFIG_REGISTER_3 = 0x0000A2EA;
			 GPS1_CONFIG_REGISTER_4 = 0x0000CF03;
			 GPS1_CONFIG_REGISTER_5 = 0x00000183;
			 GPS1_CONFIG_REGISTER_6 = 0x00000d0a;

			 GPS1_STATUS_REGISTER2  = 207+80;
			 GPS1_STATUS_REGISTER   = 0x00001840;//0x00007840;//0b_0111_1000_0100_0000
             #endif


		   }

		 else if(Config_Type == NMEA_RMC_Disable)
		 	{

			 #ifdef ASCII
		 	 GPS1_CONFIG_REGISTER_1 = '$A';
		 	 GPS1_CONFIG_REGISTER_2 = 'CM';
		 	 GPS1_CONFIG_REGISTER_3 = 'CC';
		 	 GPS1_CONFIG_REGISTER_4 = ',3';
		 	 GPS1_CONFIG_REGISTER_5 = ',0';
		 	 GPS1_CONFIG_REGISTER_6 = 0x0000300d;
		 	 GPS1_CONFIG_REGISTER_7 = 0x00000a00;

		 	 GPS1_STATUS_REGISTER   = 0x0000FA40;//0b_0111_1010_0100_0000
            #endif

            #ifndef ASCII

		 	GPS1_CONFIG_REGISTER_1 = 0x00003f3f;

		 	GPS1_CONFIG_REGISTER_2 = 0x00000A0C;

		 	GPS1_CONFIG_REGISTER_3 = 0x0000A2EA;

		 	GPS1_CONFIG_REGISTER_4 = 0x0000CF03;

		 	GPS1_CONFIG_REGISTER_5 = 0x00000082;

		 	GPS1_CONFIG_REGISTER_6 = 0x00000d0a;


		 	GPS1_STATUS_REGISTER2 = 207;

		 	GPS1_STATUS_REGISTER   = 0x00001840;//0b_0111_1000_0100_0000


            #endif

		   }

		 else if(Config_Type == NMEA_GGA_Enable)
		 {
            #ifdef ASCII
			GPS1_CONFIG_REGISTER_1 = '$A';
			GPS1_CONFIG_REGISTER_2 = 'CM';
			GPS1_CONFIG_REGISTER_3 = 'CC';
		    GPS1_CONFIG_REGISTER_4 = ',1';
			GPS1_CONFIG_REGISTER_5 = ',0';
			GPS1_CONFIG_REGISTER_6 = 0x0000310d;
			GPS1_CONFIG_REGISTER_7 = 0x00000a00;

			GPS1_STATUS_REGISTER   = 0x00003A40;//0b_0011_1010_0100_0000
            #endif

            #ifndef ASCII

			GPS1_CONFIG_REGISTER_1 = 0x3f;

			GPS1_CONFIG_REGISTER_2 = 0x3f;

			GPS1_CONFIG_REGISTER_3 = 0x0A;

			GPS1_CONFIG_REGISTER_4 = 0x0C;

			GPS1_CONFIG_REGISTER_5 = 0xA2;

			GPS1_CONFIG_REGISTER_6 = 0xEA;

			GPS1_CONFIG_REGISTER_7 = 0xCF;

			GPS1_CONFIG_REGISTER_8 = 0x01;

			GPS1_CONFIG_REGISTER_9 = 0x01;

			GPS1_CONFIG_REGISTER_10 = 0x81;

			GPS1_CONFIG_REGISTER_11 = 0x0d;

			GPS1_CONFIG_REGISTER_12 = 0x0a;


		 	GPS1_STATUS_REGISTER   = 0x00003840;//0b_0011_1000_0100_0000

		 	GPS1_STATUS_REGISTER2 = 83+207;

            #endif

		 }
		 else if(Config_Type == NMEA_GGA_Disable)
		 {
		    #ifdef ASCII
		 	GPS1_CONFIG_REGISTER_1 = '$A';
		 	GPS1_CONFIG_REGISTER_2 = 'CM';
		 	GPS1_CONFIG_REGISTER_3 = 'CC';
		 	GPS1_CONFIG_REGISTER_4 = ',1';
		 	GPS1_CONFIG_REGISTER_5 = ',0';
		 	GPS1_CONFIG_REGISTER_6 = 0x0000300d;
		 	GPS1_CONFIG_REGISTER_7 = 0x00000a00;

		 	GPS1_STATUS_REGISTER   = 0x0000FA40;//0b_1111_1010_0100_0000
		    #endif

		    #ifndef ASCII

			GPS1_CONFIG_REGISTER_1  = 0x3f;

			GPS1_CONFIG_REGISTER_2  = 0x3f;

			GPS1_CONFIG_REGISTER_3  = 0x0A;

			GPS1_CONFIG_REGISTER_4  = 0x0C;

			GPS1_CONFIG_REGISTER_5  = 0xA2;

			GPS1_CONFIG_REGISTER_6  = 0xEA;

			GPS1_CONFIG_REGISTER_7  = 0xCF;

			GPS1_CONFIG_REGISTER_8  = 0x01;

			GPS1_CONFIG_REGISTER_9  = 0x00;

			GPS1_CONFIG_REGISTER_10 = 0x80;

			GPS1_CONFIG_REGISTER_11 = 0x0d;

			GPS1_CONFIG_REGISTER_12 = 0x0a;


		    GPS1_STATUS_REGISTER2 = 207;

		    GPS1_STATUS_REGISTER   = 0x00001840;//0b_1111_1000_0100_0000


		    #endif

		 }
		 else if(Config_Type == NMEA_GSV_Enable)
		 {
            #ifdef ASCII
			GPS1_CONFIG_REGISTER_1 = '$A';
			GPS1_CONFIG_REGISTER_2 = 'CM';
			GPS1_CONFIG_REGISTER_3 = 'CC';
		    GPS1_CONFIG_REGISTER_4 = ',5';
			GPS1_CONFIG_REGISTER_5 = ',0';
			GPS1_CONFIG_REGISTER_6 = 0x0000310d;
			GPS1_CONFIG_REGISTER_7 = 0x00000a00;

			GPS1_STATUS_REGISTER   = 0x0000BA40;//0b_1011_1010_0100_0000
            #endif

            #ifndef ASCII

			GPS1_CONFIG_REGISTER_1 = 0x00003f3f;

            GPS1_CONFIG_REGISTER_2 = 0x00000A0C;

			GPS1_CONFIG_REGISTER_3 = 0x0000A2EA;

			GPS1_CONFIG_REGISTER_4 = 0x0000CF05;

			GPS1_CONFIG_REGISTER_5 = 0x00000185;

			GPS1_CONFIG_REGISTER_6 = 0x00000d0a;


		    GPS1_STATUS_REGISTER   = 0x0000B840;//0b_1011_1000_0100_0000

		    GPS1_STATUS_REGISTER2 =72 + 207;

            #endif

		 }
		 else if(Config_Type == NMEA_GSV_Disable)
		 {

            #ifdef ASCII
	        GPS1_CONFIG_REGISTER_1 = '$A';
	        GPS1_CONFIG_REGISTER_2 = 'CM';
	        GPS1_CONFIG_REGISTER_3 = 'CC';
            GPS1_CONFIG_REGISTER_4 = ',5';
	        GPS1_CONFIG_REGISTER_5 = ',0';
	        GPS1_CONFIG_REGISTER_6 = 0x0000300d;
	        GPS1_CONFIG_REGISTER_7 = 0x00000a00;

	        GPS1_STATUS_REGISTER   = 0x0000FA40;//0b_1111_1010_0100_0000
            #endif


            #ifndef ASCII

			GPS1_CONFIG_REGISTER_1 = 0x00003f3f;

		    GPS1_CONFIG_REGISTER_2 = 0x00000A0C;

			GPS1_CONFIG_REGISTER_3 = 0x0000A2EA;

	        GPS1_CONFIG_REGISTER_4 = 0x0000CF05;

			GPS1_CONFIG_REGISTER_5 = 0x00000084;

			GPS1_CONFIG_REGISTER_6 = 0x00000d0a;


		 	GPS1_STATUS_REGISTER   = 0x0000F840;//0b_1111_1000_0100_0000

		 	GPS1_STATUS_REGISTER2 = 207;

            #endif


		 }
		 else if(Config_Type == NMEA_ZDA_Enable)
		 {

		    #ifdef ASCII
		 	GPS1_CONFIG_REGISTER_1 = '$A';
		 	GPS1_CONFIG_REGISTER_2 = 'CM';
		 	GPS1_CONFIG_REGISTER_3 = 'CC';
		    GPS1_CONFIG_REGISTER_4 = ',0';
		 	GPS1_CONFIG_REGISTER_5 = ',0';
		 	GPS1_CONFIG_REGISTER_6 = 0x0000310d;
		 	GPS1_CONFIG_REGISTER_7 = 0x00000a00;

		 	GPS1_STATUS_REGISTER   = 0x00001A40;//0b_0001_1010_0100_0000
		    #endif


		    #ifndef ASCII

		    GPS1_CONFIG_REGISTER_1 = 0x00003f3f;

		    GPS1_CONFIG_REGISTER_2 = 0x00000A0C;

		 	GPS1_CONFIG_REGISTER_3 = 0x0000A2EA;

		 	GPS1_CONFIG_REGISTER_4 = 0x0000CF00;

		 	GPS1_CONFIG_REGISTER_5 = 0x00000180;

		 	GPS1_CONFIG_REGISTER_6 = 0x00000d0a;


		 	GPS1_STATUS_REGISTER   = 0x00001840;//0b_0001_1000_0100_0000

		 	GPS1_STATUS_REGISTER2 = 39 + 207;

		    #endif


		 }
		 else if(Config_Type == NMEA_ZDA_Disable)
		 {

		    #ifdef ASCII
		 	GPS1_CONFIG_REGISTER_1 = '$A';
		    GPS1_CONFIG_REGISTER_2 = 'CM';
		 	GPS1_CONFIG_REGISTER_3 = 'CC';
		 	GPS1_CONFIG_REGISTER_4 = ',0';
		 	GPS1_CONFIG_REGISTER_5 = ',0';
		 	GPS1_CONFIG_REGISTER_6 = 0x0000300d;
		 	GPS1_CONFIG_REGISTER_7 = 0x00000a00;

		 	GPS1_STATUS_REGISTER   = 0x0000FA40;//0b_1111_1010_0100_0000
		    #endif


		    #ifndef ASCII

		    GPS1_CONFIG_REGISTER_1 = 0x00003f3f;

		    GPS1_CONFIG_REGISTER_2 = 0x00000A0C;

		 	GPS1_CONFIG_REGISTER_3 = 0x0000A2EA;

		    GPS1_CONFIG_REGISTER_4 = 0x0000CF00;

		 	GPS1_CONFIG_REGISTER_5 = 0x00000081;

		 	GPS1_CONFIG_REGISTER_6 = 0x00000d0a;


		 	GPS1_STATUS_REGISTER   = 0x0000F840;//0b_1111_1000_0100_0000

		 	GPS1_STATUS_REGISTER2 = 207;

		    #endif


		 }
		 else if(Config_Type == FACTORY_RESET)
		 {
            #ifdef ASCII
			GPS1_CONFIG_REGISTER_1 = '$A';
		    GPS1_CONFIG_REGISTER_2 = 'CF';
		    GPS1_CONFIG_REGISTER_3 = 'RM';
		 	GPS1_CONFIG_REGISTER_4 = 0x00000d0a;

		 	GPS1_STATUS_REGISTER   = 0x00001040;//0b_0001_0000_0100_0000
            #endif

            #ifndef ASCII

			GPS1_CONFIG_REGISTER_1 = 0x00003f3f;

			GPS1_CONFIG_REGISTER_2 = 0x00000a0c;

			GPS1_CONFIG_REGISTER_3 = 0x0000a2b0;

			GPS1_CONFIG_REGISTER_4 = 0x00000014;

			GPS1_CONFIG_REGISTER_5 = 0x00000d0a;


			GPS1_STATUS_REGISTER   = 0x00001440;//0b_0001_0100_0100_0000

			GPS1_STATUS_REGISTER2 = 207;

            #endif
		 }
		 else if(Config_Type == COLD_START)
		 {
            #ifdef ASCII
			GPS1_CONFIG_REGISTER_1 = '$A';
		    GPS1_CONFIG_REGISTER_2 = 'CC';
		    GPS1_CONFIG_REGISTER_3 = 'SM';
		    GPS1_CONFIG_REGISTER_4 = 0x00000d0a;

		    GPS1_STATUS_REGISTER   = 0x00001040;//0b_0001_0000_0100_0000
            #endif

            #ifndef ASCII

			GPS1_CONFIG_REGISTER_1 = 0x00003f3f;

			GPS1_CONFIG_REGISTER_2 = 0x00000a0c;

			GPS1_CONFIG_REGISTER_3 = 0x0000a2b0;

			GPS1_CONFIG_REGISTER_4 = 0x00000115;

			GPS1_CONFIG_REGISTER_5 = 0x00000d0a;


			GPS1_STATUS_REGISTER   = 0x00001440;//0b_0001_0100_0100_0000

			GPS1_STATUS_REGISTER2 = 207;

            #endif

		 }
	 }
  }
  else if(GPS_No == GPS_2)
  {
	  GPS_Status_Data = GPS2_STATUS_REGISTER;
	  if((GPS_Status_Data & 0x00000040) == 0x00000000)
	  {
	    if(Config_Type == NMEA_RMC_Enable)
	  	{

	        #ifdef ASCII
	    	GPS2_CONFIG_REGISTER_1 = '$A';
	  		GPS2_CONFIG_REGISTER_2 = 'CM';
	  		GPS2_CONFIG_REGISTER_3 = 'CC';
	  		GPS2_CONFIG_REGISTER_4 = ',3';
	  		GPS2_CONFIG_REGISTER_5 = ',0';
	  		GPS2_CONFIG_REGISTER_6 = 0x0000310d;
	  		GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  		GPS2_STATUS_REGISTER   = 0x00007A40;//0b_0111_1010_0100_0000
	        #endif

	        #ifndef ASCII

	  	    GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  		GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  		GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  		GPS2_CONFIG_REGISTER_4 = 0x0000CF03;

	  		GPS2_CONFIG_REGISTER_5 = 0x00000183;

	  		GPS2_CONFIG_REGISTER_6 = 0x00000d0a;


	  		GPS2_STATUS_REGISTER   = 0x00007840;//0b_0111_1000_0100_0000

	        #endif


	  	}

	  	else if(Config_Type == NMEA_RMC_Disable)
	  	{

	  	     #ifdef ASCII
	  		 GPS2_CONFIG_REGISTER_1 = '$A';
	  		 GPS2_CONFIG_REGISTER_2 = 'CM';
	  		 GPS2_CONFIG_REGISTER_3 = 'CC';
	  		 GPS2_CONFIG_REGISTER_4 = ',3';
	  		 GPS2_CONFIG_REGISTER_5 = ',0';
	 	 	 GPS2_CONFIG_REGISTER_6 = 0x0000300d;
	 		 GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  	 	 GPS2_STATUS_REGISTER   = 0x0000FA40;//0b_0111_1010_0100_0000
	         #endif

	         #ifndef ASCII

	  		 GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  		 GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  		 GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  		 GPS2_CONFIG_REGISTER_4 = 0x0000CF03;

	  		 GPS2_CONFIG_REGISTER_5 = 0x00000082;

	  		 GPS2_CONFIG_REGISTER_6 = 0x00000d0a;

  		 	 GPS2_STATUS_REGISTER   = 0x0000F840;//0b_0111_1000_0100_0000

	         #endif

	  	}



	  		 else if(Config_Type == NMEA_GGA_Enable)
	  		 {
	              #ifdef ASCII
	  			GPS2_CONFIG_REGISTER_1 = '$A';
	  			GPS2_CONFIG_REGISTER_2 = 'CM';
	  			GPS2_CONFIG_REGISTER_3 = 'CC';
	  		    GPS2_CONFIG_REGISTER_4 = ',1';
	  			GPS2_CONFIG_REGISTER_5 = ',0';
	  			GPS2_CONFIG_REGISTER_6 = 0x0000310d;
	  			GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  			GPS2_STATUS_REGISTER   = 0x00003A40;//0b_0011_1010_0100_0000
	              #endif

	              #ifndef ASCII

	  			GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  			GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  			GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  			GPS2_CONFIG_REGISTER_4 = 0x0000CF01;

	  			GPS2_CONFIG_REGISTER_5 = 0x00000181;

	  			GPS2_CONFIG_REGISTER_6 = 0x00000d0a;


	  		 	GPS2_STATUS_REGISTER   = 0x00003840;//0b_0011_1000_0100_0000

	              #endif

	  		 }
	  		 else if(Config_Type == NMEA_GGA_Disable)
	  		 {
	  		    #ifdef ASCII
	  		 	GPS2_CONFIG_REGISTER_1 = '$A';
	  		 	GPS2_CONFIG_REGISTER_2 = 'CM';
	  		 	GPS2_CONFIG_REGISTER_3 = 'CC';
	  		 	GPS2_CONFIG_REGISTER_4 = ',1';
	  		 	GPS2_CONFIG_REGISTER_5 = ',0';
	  		 	GPS2_CONFIG_REGISTER_6 = 0x0000300d;
	  		 	GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  		 	GPS2_STATUS_REGISTER   = 0x0000FA40;//0b_1111_1010_0100_0000
	  		    #endif

	  		    #ifndef ASCII

	  		    GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  		 	GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  		 	GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  		 	GPS2_CONFIG_REGISTER_4 = 0x0000CF01;

	  		 	GPS2_CONFIG_REGISTER_5 = 0x00000080;

	  		    GPS2_CONFIG_REGISTER_6 = 0x00000d0a;


	  		    GPS2_STATUS_REGISTER   = 0x0000F840;//0b_1111_1000_0100_0000
	  		    #endif

	  		 }
	  		 else if(Config_Type == NMEA_GSV_Enable)
	  		 {
	              #ifdef ASCII
	  			GPS2_CONFIG_REGISTER_1 = '$A';
	  			GPS2_CONFIG_REGISTER_2 = 'CM';
	  			GPS2_CONFIG_REGISTER_3 = 'CC';
	  		    GPS2_CONFIG_REGISTER_4 = ',5';
	  			GPS2_CONFIG_REGISTER_5 = ',0';
	  			GPS2_CONFIG_REGISTER_6 = 0x0000310d;
	  			GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  			GPS2_STATUS_REGISTER   = 0x0000BA40;//0b_1011_1010_0100_0000
	              #endif

	              #ifndef ASCII

	  			GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	            GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  			GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  			GPS2_CONFIG_REGISTER_4 = 0x0000CF05;

	  			GPS2_CONFIG_REGISTER_5 = 0x00000185;

	  			GPS2_CONFIG_REGISTER_6 = 0x00000d0a;


	  		    GPS2_STATUS_REGISTER   = 0x0000B840;//0b_1011_1000_0100_0000

	              #endif

	  		 }
	  		 else if(Config_Type == NMEA_GSV_Disable)
	  		 {

	              #ifdef ASCII
	  	        GPS2_CONFIG_REGISTER_1 = '$A';
	  	        GPS2_CONFIG_REGISTER_2 = 'CM';
	  	        GPS2_CONFIG_REGISTER_3 = 'CC';
	            GPS2_CONFIG_REGISTER_4 = ',5';
	  	        GPS2_CONFIG_REGISTER_5 = ',0';
	  	        GPS2_CONFIG_REGISTER_6 = 0x0000300d;
	  	        GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  	        GPS2_STATUS_REGISTER   = 0x0000FA40;//0b_1111_1010_0100_0000
	              #endif


	              #ifndef ASCII

	  			GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  		    GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  			GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  	        GPS2_CONFIG_REGISTER_4 = 0x0000CF05;

	  			GPS2_CONFIG_REGISTER_5 = 0x00000084;

	  			GPS2_CONFIG_REGISTER_6 = 0x00000d0a;

	  		 	GPS2_STATUS_REGISTER   = 0x0000F840;//0b_1111_1000_0100_0000
	              #endif


	  		 }
	  		 else if(Config_Type == NMEA_ZDA_Enable)
	  		 {

	  		    #ifdef ASCII
	  		 	GPS2_CONFIG_REGISTER_1 = '$A';
	  		 	GPS2_CONFIG_REGISTER_2 = 'CM';
	  		 	GPS2_CONFIG_REGISTER_3 = 'CC';
	  		    GPS2_CONFIG_REGISTER_4 = ',0';
	  		 	GPS2_CONFIG_REGISTER_5 = ',0';
	  		 	GPS2_CONFIG_REGISTER_6 = 0x0000310d;
	  		 	GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  		 	GPS2_STATUS_REGISTER   = 0x00001A40;//0b_0001_1010_0100_0000
	  		    #endif


	  		    #ifndef ASCII

	  		    GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  		    GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  		 	GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  		 	GPS2_CONFIG_REGISTER_4 = 0x0000CF00;

	  		 	GPS2_CONFIG_REGISTER_5 = 0x00000180;

	  		 	GPS2_CONFIG_REGISTER_6 = 0x00000d0a;


	  		 	GPS2_STATUS_REGISTER   = 0x00001840;//0b_0001_1000_0100_0000

	  		    #endif


	  		 }
	  		 else if(Config_Type == NMEA_ZDA_Disable)
	  		 {

	  		    #ifdef ASCII
	  		 	GPS2_CONFIG_REGISTER_1 = '$A';
	  		    GPS2_CONFIG_REGISTER_2 = 'CM';
	  		 	GPS2_CONFIG_REGISTER_3 = 'CC';
	  		 	GPS2_CONFIG_REGISTER_4 = ',0';
	  		 	GPS2_CONFIG_REGISTER_5 = ',0';
	  		 	GPS2_CONFIG_REGISTER_6 = 0x0000300d;
	  		 	GPS2_CONFIG_REGISTER_7 = 0x00000a00;

	  		 	GPS1_STATUS_REGISTER   = 0x0000FA40;//0b_1111_1010_0100_0000
	  		    #endif


	  		    #ifndef ASCII

	  		    GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  		    GPS2_CONFIG_REGISTER_2 = 0x00000A0C;

	  		 	GPS2_CONFIG_REGISTER_3 = 0x0000A2EA;

	  		    GPS2_CONFIG_REGISTER_4 = 0x0000CF00;

	  		 	GPS2_CONFIG_REGISTER_5 = 0x00000081;

	  		 	GPS2_CONFIG_REGISTER_6 = 0x00000d0a;


	  		 	GPS2_STATUS_REGISTER   = 0x0000F840;//0b_1111_1000_0100_0000

	  		    #endif


	  		 }
	  		 else if(Config_Type == FACTORY_RESET)
	  		 {
	              #ifdef ASCII
	  			GPS2_CONFIG_REGISTER_1 = '$A';
	  		    GPS2_CONFIG_REGISTER_2 = 'CF';
	  		    GPS2_CONFIG_REGISTER_3 = 'RM';
	  		 	GPS2_CONFIG_REGISTER_4 = 0x00000d0a;

	  		 	GPS2_STATUS_REGISTER   = 0x00001040;//0b_0001_0000_0100_0000
	              #endif

	              #ifndef ASCII

	  			GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  			GPS2_CONFIG_REGISTER_2 = 0x00000a0c;

	  			GPS2_CONFIG_REGISTER_3 = 0x0000a2b0;

	  			GPS2_CONFIG_REGISTER_4 = 0x00000014;

	  			GPS2_CONFIG_REGISTER_5 = 0x00000d0a;


	  			GPS2_STATUS_REGISTER   = 0x00001440;//0b_0001_0100_0100_0000

	              #endif
	  		 }
	  		 else if(Config_Type == COLD_START)
	  		 {
	              #ifdef ASCII
	  			GPS1_CONFIG_REGISTER_1 = '$A';
	  		    GPS1_CONFIG_REGISTER_2 = 'CC';
	  		    GPS1_CONFIG_REGISTER_3 = 'SM';
	  		    GPS1_CONFIG_REGISTER_4 = 0x00000d0a;

	  		    GPS1_STATUS_REGISTER   = 0x00001040;//0b_0001_0000_0100_0000
	              #endif

	              #ifndef ASCII

	  			GPS2_CONFIG_REGISTER_1 = 0x00003f3f;

	  			GPS2_CONFIG_REGISTER_2 = 0x00000a0c;

	  			GPS2_CONFIG_REGISTER_3 = 0x0000a2b0;

	  			GPS2_CONFIG_REGISTER_4 = 0x00000115;

	  			GPS2_CONFIG_REGISTER_5 = 0x00000d0a;


	  			GPS2_STATUS_REGISTER   = 0x00001440;//0b_0001_0100_0100_0000

	              #endif

	  		 }
	  	 }
  }
  return 0;
}

//unsigned long int rHAL_GPS_Config(unsigned long int GPS_No,unsigned long int Config_Type)
//{
//	GPS_Config_Msg.Initial_msg[0] = 0x3F;
//	GPS_Config_Msg.Initial_msg[1] = 0x3F;
//	GPS_Config_Msg.Initial_msg[2] = 0x0A;
//	GPS_Config_Msg.Initial_msg[3] = 0x0C;
//	GPS_Config_Msg.Initial_msg[4] = 0xA2;
//	GPS_Config_Msg.Initial_msg[5] = 0xEA;
//	GPS_Config_Msg.Initial_msg[6] = 0xCF;
//	GPS_Config_Msg.Carriage_return = 0x0A;
//	GPS_Config_Msg.Line_feed = 0x31;
//
//	GPS_Config_ptr = (unsigned char*)&GPS_Config_Msg;
//	GPS1_Buffer_ptr = (unsigned int*)&GPS1_Buffer;
//	GPS2_Buffer_ptr = (unsigned int*)&GPS1_Buffer;
//
//	if(GPS_No == GPS_1)
//	{
//		if(Config_Type == NMEA_ZDA_Enable)
//		{
//			GPS_Config_Msg.Msg_index = 0x00;
//			GPS_Config_Msg.E_D = 0x01;
//			GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			GPS_Status2_Msg.status2_NOCB = 12;
//			GPS_Status2_Msg.status2_config_ED = 1;
//
//			for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			{
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			}
//
//			GPS1_Buffer.Status_1 = 207 + 39;
//			GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		}
//
//		else if(Config_Type == NMEA_ZDA_Disable)
//		{
//			GPS_Config_Msg.Msg_index = 0x00;
//			GPS_Config_Msg.E_D = 0x00;
//			GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			GPS_Status2_Msg.status2_NOCB = 12;
//			GPS_Status2_Msg.status2_config_ED = 1;
//
//			for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			{
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			}
//
//			GPS1_Buffer.Status_1 = 207;
//			GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		}
//
//		else if(Config_Type == NMEA_GGA_Enable)
//		{
//			GPS_Config_Msg.Msg_index = 0x01;
//			GPS_Config_Msg.E_D = 0x01;
//			GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			GPS_Status2_Msg.status2_NOCB = 12;
//			GPS_Status2_Msg.status2_config_ED = 1;
//
//			for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			{
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			}
//
//			GPS1_Buffer.Status_1 = 207 + 83;
//			GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		}
//
//		else if(Config_Type == NMEA_GGA_Disable)
//		{
//			GPS_Config_Msg.Msg_index = 0x01;
//			GPS_Config_Msg.E_D = 0x01;
//			GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			GPS_Status2_Msg.status2_NOCB = 12;
//			GPS_Status2_Msg.status2_config_ED = 0;
//
//			for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			{
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			}
//
//			GPS1_Buffer.Status_1 = 207;
//			GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		}
//
//        else if(Config_Type == NMEA_GLL_Enable)
//        {
//        	GPS_Config_Msg.Msg_index = 0x02;
//			GPS_Config_Msg.E_D = 0x01;
//			GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			GPS_Status2_Msg.status2_NOCB = 12;
//			GPS_Status2_Msg.status2_config_ED = 1;
//
//			for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			{
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			}
//
//			GPS1_Buffer.Status_1 = 207 + 52;
//			GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		}
//        else if (Config_Type == NMEA_GLL_Disable)
//        {
//			GPS_Config_Msg.Msg_index = 0x02;
//			GPS_Config_Msg.E_D = 0x00;
//			GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			GPS_Status2_Msg.status2_NOCB = 12;
//			GPS_Status2_Msg.status2_config_ED = 1;
//
//			for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			{
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			}
//
//			GPS1_Buffer.Status_1 = 207 ;
//			GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//        }
//
//        else if (Config_Type == NMEA_RMC_Enable)
//        {
//        	GPS_Config_Msg.Msg_index = 0x03;
//			GPS_Config_Msg.E_D = 0x01;
//			GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			GPS_Status2_Msg.status2_NOCB = 12;
//			GPS_Status2_Msg.status2_config_ED = 1;
//
//			for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			{
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			}
//
//			GPS1_Buffer.Status_1 = 207+80 ;
//			GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//		 else if (Config_Type == NMEA_RMC_Disable)
//		 {
//			 GPS_Config_Msg.Msg_index = 0x03;
//			 GPS_Config_Msg.E_D = 0x00;
//			 GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			 GPS_Status2_Msg.status2_NOCB = 12;
//			 GPS_Status2_Msg.status2_config_ED = 1;
//
//			 for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			 {
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			 }
//
//			 GPS1_Buffer.Status_1 = 207+80 ;
//			 GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//		 else if (Config_Type == NMEA_GSA_Enable)
//		 {
//			 GPS_Config_Msg.Msg_index = 0x04;
//			 GPS_Config_Msg.E_D = 0x01;
//			 GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			 GPS_Status2_Msg.status2_NOCB = 12;
//			 GPS_Status2_Msg.status2_config_ED = 1;
//
//			 for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			 {
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			 }
//
//			 GPS1_Buffer.Status_1 = 207+71;
//			 GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//		 else if (Config_Type == NMEA_GSA_Disable)
//		 {
//			 GPS_Config_Msg.Msg_index = 0x04;
//			 GPS_Config_Msg.E_D = 0x00;
//			 GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			 GPS_Status2_Msg.status2_NOCB = 12;
//			 GPS_Status2_Msg.status2_config_ED = 1;
//
//			 for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			 {
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			 }
//
//			 GPS1_Buffer.Status_1 = 207;
//			 GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//		 else if (Config_Type == NMEA_GSV_Enable)
//		 {
//			 GPS_Config_Msg.Msg_index = 0x05;
//			 GPS_Config_Msg.E_D = 0x01;
//			 GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			 GPS_Status2_Msg.status2_NOCB = 12;
//			 GPS_Status2_Msg.status2_config_ED = 1;
//
//			 for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			 {
//				 *GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			 }
//
//			 GPS1_Buffer.Status_1 = 207+72;
//			 GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//		 else if (Config_Type == NMEA_GSV_Disable)
//		 {
//			 GPS_Config_Msg.Msg_index = 0x05;
//			 GPS_Config_Msg.E_D = 0x00;
//			 GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			 GPS_Status2_Msg.status2_NOCB = 12;
//			 GPS_Status2_Msg.status2_config_ED = 1;
//
//			 for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			 {
//				 *GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			 }
//
//			 GPS1_Buffer.Status_1 = 207;
//			 GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//		 else if (Config_Type == NMEA_VTG_Enable)
//		 {
//			 GPS_Config_Msg.Msg_index = 0x06;
//			 GPS_Config_Msg.E_D = 0x01;
//			 GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			 GPS_Status2_Msg.status2_NOCB = 12;
//			 GPS_Status2_Msg.status2_config_ED = 1;
//
//			 for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			 {
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			 }
//
//			 GPS1_Buffer.Status_1 = 207+49;
//			 GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//		 else if (Config_Type == NMEA_VTG_Disable)
//		 {
//			 GPS_Config_Msg.Msg_index = 0x06;
//			 GPS_Config_Msg.E_D = 0x00;
//			 GPS_Config_Msg.Chk_sum = checksum_u8((unsigned char*)&GPS_Config_Msg,9);
//			 GPS_Status2_Msg.status2_NOCB = 12;
//			 GPS_Status2_Msg.status2_config_ED = 1;
//
//			 for(i_GPS_B_Count = 0;i_GPS_B_Count < GPS_BUFFER_COPY_LIMIT;i_GPS_B_Count++)
//			 {
//				*GPS1_Buffer_ptr++ = *GPS_Config_ptr++;
//			 }
//
//			 GPS1_Buffer.Status_1 = 207;
//			 GPS1_Buffer.Status_2 = GPS_Status2_Msg.data;
//		 }
//	}
//	return 0;
//}
//
void rGPS_Buffer_Init()
{

	//Initialization of GPS1 Config Buffer
	GPS1_Buffer.Config_1 = GPS1_CONFIG_REGISTER_1;
	GPS1_Buffer.Config_2 = GPS1_CONFIG_REGISTER_2;
	GPS1_Buffer.Config_3 = GPS1_CONFIG_REGISTER_3;
	GPS1_Buffer.Config_4 = GPS1_CONFIG_REGISTER_4;
	GPS1_Buffer.Config_5 = GPS1_CONFIG_REGISTER_5;
	GPS1_Buffer.Config_6 = GPS1_CONFIG_REGISTER_6;
	GPS1_Buffer.Config_7 = GPS1_CONFIG_REGISTER_7;
	GPS1_Buffer.Config_8 = GPS1_CONFIG_REGISTER_8;
	GPS1_Buffer.Config_9 = GPS1_CONFIG_REGISTER_9;
	GPS1_Buffer.Config_10 = GPS1_CONFIG_REGISTER_10;
	GPS1_Buffer.Config_11 = GPS1_CONFIG_REGISTER_11;
	GPS1_Buffer.Config_12 = GPS1_CONFIG_REGISTER_12;

	//Initialization of GPS1 Status Buffer
	GPS1_Buffer.Status_1 = GPS1_STATUS_REGISTER;
	GPS1_Buffer.Status_1 = GPS1_STATUS_REGISTER2;

	//Initialization of GPS1 Data Buffer
	GPS1_Buffer.Buffer = GPS1_BUFFER_BASE;
	GPS1_STATUS_REGISTER2 = 207;

	//Initialization of GPS1 Config Buffer
	//(To be added here)
}

void rGPS_TM_Extract(void)
{
	//aaaa=0xabcd;
	if(GPS_Data_Read_Status == TRUE) //Transfer to TM Buffer Only after checking the availability and validity of GPS data
	{
		GPS_TM_Buffer_Addr_USC = (unsigned char*)GPS_RCVD_DATA;

			TM.Buffer.TM_GPS.TM_No_Of_Sat = *(GPS_TM_Buffer_Addr_USC+4);
			TM.Buffer.TM_GPS.TM_UTC_Day = *(GPS_TM_Buffer_Addr_USC+150);
			TM.Buffer.TM_GPS.TM_UTC_Month = *(GPS_TM_Buffer_Addr_USC+151);
			TM.Buffer.TM_GPS.TM_UTC_Year = *((unsigned short*)(GPS_TM_Buffer_Addr_USC+152));
			TM.Buffer.TM_GPS.TM_UTC_hour = *(GPS_TM_Buffer_Addr_USC+154);
			TM.Buffer.TM_GPS.TM_UTC_Min = *(GPS_TM_Buffer_Addr_USC+155);
			TM.Buffer.TM_GPS.TM_UTC_Sec = *((unsigned short*)(GPS_TM_Buffer_Addr_USC+156));
			TM.Buffer.TM_GPS.TM_GPSWeek = *((unsigned short*)(GPS_TM_Buffer_Addr_USC+158));
			TM.Buffer.TM_GPS.TM_GPS_Xpos = *((int*)(GPS_TM_Buffer_Addr_USC+160));
			TM.Buffer.TM_GPS.TM_GPS_Ypos = *((int*)(GPS_TM_Buffer_Addr_USC+164));
			TM.Buffer.TM_GPS.TM_GPS_Zpos = *((int*)(GPS_TM_Buffer_Addr_USC+168));
			TM.Buffer.TM_GPS.TM_GPS_XVel = *((int*)(GPS_TM_Buffer_Addr_USC+172));
			TM.Buffer.TM_GPS.TM_GPS_YVel = *((int*)(GPS_TM_Buffer_Addr_USC+176));
			TM.Buffer.TM_GPS.TM_GPS_ZVel = *((int*)(GPS_TM_Buffer_Addr_USC+180));
			TM.Buffer.TM_GPS.TM_No_Of_Sat_InFix = *(GPS_TM_Buffer_Addr_USC+185);
			TM.Buffer.TM_GPS.TM_Pos_Validity = *(GPS_TM_Buffer_Addr_USC+186);
			TM.Buffer.TM_GPS.TM_BIST_Info = *(GPS_TM_Buffer_Addr_USC+203);
			TM.Buffer.TM_GPS.TM_CheckSum = *(GPS_TM_Buffer_Addr_USC+204);

			//Computation of On board Checksum of GPS data
			GPS_obc_checkum = checksum_u8(GPS_TM_Buffer_Addr_USC,204);

			//Check for BIST Information and Checksum for the validation of Data
			//Check the above mentioned info to raise "GPS_Data_Validity_flag" (which will be used for OBC_GPS processing)
			if(GPS_obc_checkum == TM.Buffer.TM_GPS.TM_CheckSum)
			{
				if(TM.Buffer.TM_GPS.TM_BIST_Info == 0x0002)
				{
					f_GPS_Valid_Data = True;
				}

				else
				{
					f_GPS_Valid_Data = False;
				}
			}
			else
			{
				f_GPS_Valid_Data = False;
			}

	}
}

