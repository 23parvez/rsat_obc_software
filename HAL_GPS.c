
#include "TM_Global_Buffer.h"
#include "HAL_Global.h"
#include "Global.h"
#include "HAL_GPS.h"
#include "Telemetry.h"
#include "HAL_Address.h"
#include "TC_List.h"

#include "adcs_Constants.h"
#include "adcs_GPS_OD.h"
#include "adcs_VarDeclarations.h"

void rGPS_Buffer_Init(void)
{
	//Initialization of GPS1 Status Buffer and read Buffer
	GPS1.GPS_Status_Register_1 	= 0x200023FC;
	GPS1.GPS_Status_Register_2 	= 0x20002BF8;
	GPS1.GPS_Buffer_addr 		= GPS1_BUFFER_BASE;

	GPS1.GPS_Config_1  = 0x20003000;
	GPS1.GPS_Config_2  = 0x20003004;
	GPS1.GPS_Config_3  = 0x20003008;
	GPS1.GPS_Config_4  = 0x2000300C;
	GPS1.GPS_Config_5  = 0x20003010;
	GPS1.GPS_Config_6  = 0x20003014;
	GPS1.GPS_Config_7  = 0x20003018;
	GPS1.GPS_Config_8  = 0x2000301C;
	GPS1.GPS_Config_9  = 0x20003020;
	GPS1.GPS_Config_10 = 0x20003024;
	GPS1.GPS_Config_11 = 0x20003028;
	GPS1.GPS_Config_12 = 0x2000302C;
	GPS1.GPS_Config_13 = 0x20003030;


	//initialization of gps2 status register and Read Buffer
	GPS2.GPS_Status_Register_1 	= 0x200043FC;
	GPS2.GPS_Status_Register_2 	= 0x20004BF8;
	GPS2.GPS_Buffer_addr 		= GPS2_BUFFER_BASE;

	GPS2.GPS_Config_1  = 0x20005000;
	GPS2.GPS_Config_2  = 0x20005004;
	GPS2.GPS_Config_3  = 0x20005008;
	GPS2.GPS_Config_4  = 0x2000500c;
	GPS2.GPS_Config_5  = 0x20005010;
	GPS2.GPS_Config_6  = 0x20005014;
	GPS2.GPS_Config_7  = 0x20005018;
	GPS2.GPS_Config_8  = 0x2000501c;
	GPS2.GPS_Config_9  = 0x20005020;
	GPS2.GPS_Config_10 = 0x20005024;
	GPS2.GPS_Config_11 = 0x20005028;
	GPS2.GPS_Config_12 = 0x2000502c;
	GPS2.GPS_Config_13 = 0x20005030;
}

unsigned int gps_ready_bit = 0;
unsigned long int gpstestbuff[17];
unsigned short maha;
unsigned char GPS_GSV_FLAG = 0;
unsigned int rHAL_GPS_Read (struct HAL_GPS_registers GPS_No, unsigned int No_of_Bytes)
{
	unsigned int temp_data;
	GPS_count_TM = 0;

		if(No_of_Bytes %2 == 0)          // Check for Even Or Odd
		{
			  GPS_Locations = No_of_Bytes / 2 ;
			  GPS_Locations = GPS_Locations - 1;
		}
		else
		{
			 GPS_Locations = (No_of_Bytes + 1) / 2 ;
			 GPS_Locations = GPS_Locations - 1;
		}

		GPS_Buffer_Addr = (unsigned long int*)GPS_No.GPS_Buffer_addr;
		GPS_Status_Data = REG32(GPS_No.GPS_Status_Register_1);
		temp_data = GPS_Status_Data;
		TM.Buffer.TM_GPS_Status  =  (unsigned short)temp_data;

		if (GPS_Status_Data & 0x00000008)//0b0000_0000_0000_1000 ------Check Data Ready Bit
		{

			  gps_ready_bit = GPS_Status_Data;
			  REG32(GPS_No.GPS_Status_Register_1) = (GPS_Status_Data | 0x00000400);//0b0000_0000_0010_0000 ---------Set Read Enable Bit

			  if(GPS_GSV_FLAG)
			  {
				  for(GPS_Addr_Count=0; GPS_Addr_Count<=GPS_Locations; GPS_Addr_Count++)
				  {
					  GPS_Data = REG32(GPS_Buffer_Addr);
					  GPS_Data = GPS_Data & 0x0000FFFF;
					  GPS_RCVD_DATA[GPS_Addr_Count] = GPS_Data;
					  GPS_Buffer_Addr++;
				  }
				  REG32(GPS_No.GPS_Status_Register_1) = (GPS_Status_Data & 0x0000FBFF);//0b1111_1111_1101_1111------------Reset Read Enable Bit
				  for(GPS_Addr_Count=0; GPS_Addr_Count<=GPS_Locations; GPS_Addr_Count++)
				  {
					  GPS_counter++;
					  if(GPS_Addr_Count < 108)
					  {
					       if( (GPS_RCVD_DATA[GPS_Addr_Count] == 0x00003F3F) && (GPS_RCVD_DATA[GPS_counter] == 0x0000AF01))//Check for Valid Message header(0x3F3F) and Identifier(0xAF01))
					       {
					    	   GPS_count_TM = GPS_Addr_Count;
					    	   GPS_Data_Read_Status = TRUE;
					       }
					  }
					  else
					  {
						  GPS_Data_Read_Status = FALSE;
					  }
				  }
			  }
			  else
			  {
				  for(GPS_Addr_Count=0; GPS_Addr_Count<=GPS_Locations; GPS_Addr_Count++)
				  {
					  GPS_Data = REG32(GPS_Buffer_Addr);
					  GPS_Data = GPS_Data & 0x0000FFFF;
					  GPS_RCVD_DATA[GPS_Addr_Count] = GPS_Data;
					  GPS_Buffer_Addr++;
				  }

				  REG32(GPS_No.GPS_Status_Register_1) = (GPS_Status_Data & 0x0000FBFF);//0b1111_1111_1101_1111------------Reset Read Enable Bit

				  //------------------------------Data Formating--------------------------------------------
				  if((GPS_RCVD_DATA[0] == 0x00003F3F) && (GPS_RCVD_DATA[1] == 0x0000AF01)) //Check for Valid Message header(0x3F3F) and Identifier(0xAF01)
				  {

					  GPS_Data_Read_Status = TRUE;
				  }
				  else
				  {

					  GPS_Data_Read_Status = FALSE;
				  }

				  GPSDataReady = 1;
			  }

		}
		else
		{
				GPS_Data_Read_Status = FALSE;
		}

		return 0;


}

void GPS_1_DATA(void)
{

	if(TC_boolean_u.TC_Boolean_Table.TC_GPS12_Select == GPS_1)
	{

		rHAL_GPS_Read(GPS1, NO_BYTES);
	}
	else if(TC_boolean_u.TC_Boolean_Table.TC_GPS12_Select == GPS_2)
	{
		rHAL_GPS_Read(GPS2, NO_BYTES);
	}
	else
	{
		//
	}
}

//GPS_OBT_time:This time gets started when OBC is Powered ON and latched when PPS pulse is generated
void rGPS_OBT_timer(void)
{
	unsigned temp_gps;
	GPS_OBT_Latch_enable  = (unsigned short)(GPS_Latch_enable & 0x0000FFFF);
	GPS_OBT_Read_1  =  GPS_OBT_Count_1 & 0x0000FFFF;
	temp_gps = GPS_OBT_Count_2 & 0x0000FFFF;
	GPS_OBT_Read_2 = ((temp_gps << 16) & 0xFFFF0000);
	GPS_READ_OBT = (unsigned int)((GPS_OBT_Read_2 | GPS_OBT_Read_1) & 0xFFFFFFFF);
	TM.Buffer.TM_GPS_OBT_Read_1   = GPS_READ_OBT;
}

unsigned long int rHAL_GPS_Config(struct HAL_GPS_registers GPS_No,unsigned long int Config_Type)
{
		 if(Config_Type == NMEA_GSA_Enable)         //change to GSA
		 {
             #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1)  	= '$';
			 REG32(GPS_No.GPS_Config_2)  	= 'A';
			 REG32(GPS_No.GPS_Config_3)  	= 'C';
			 REG32(GPS_No.GPS_Config_4) 	= 'M';
			 REG32(GPS_No.GPS_Config_5)  	= 'C';
			 REG32(GPS_No.GPS_Config_6)  	= 'C';
			 REG32(GPS_No.GPS_Config_7)  	= ',';
			 REG32(GPS_No.GPS_Config_8)  	= '4';
			 REG32(GPS_No.GPS_Config_9) 	= ',';
			 REG32(GPS_No.GPS_Config_10) 	= '1';
			 REG32(GPS_No.GPS_Config_11) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) 	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x00006316; //0b_0_1100_0_1_100010110
             #endif

             #ifndef ASCII
			 REG32(GPS_No.GPS_Config_1) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_2) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_3) 	= 0x0000000A;
			 REG32(GPS_No.GPS_Config_4) 	= 0x0000000C;
			 REG32(GPS_No.GPS_Config_5) 	= 0x000000A2;
			 REG32(GPS_No.GPS_Config_6) 	= 0x000000EA;
			 REG32(GPS_No.GPS_Config_7) 	= 0x000000CF;
			 REG32(GPS_No.GPS_Config_8) 	= 0x00000004;//GSA
			 REG32(GPS_No.GPS_Config_9) 	= 0x00000001;
			 REG32(GPS_No.GPS_Config_10) 	= 0x00000084;
			 REG32(GPS_No.GPS_Config_11) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) 	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)  = 0x00006316; //0b_0_1100_0_1_100010110
//			 GPS1_STATUS_REGISTER   = 0x00001840;//0x00007840;//0b_0111_1000_0100_0000
//			 GPS1_STATUS_REGISTER   = 0x00006b1f;// 0b_0110_1011_0001_1111


             #endif
		   }

		 else if(Config_Type == NMEA_GSA_Disable)  //GSA_DIS
		 	{

			 #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) 	= '$';
			 REG32(GPS_No.GPS_Config_2) 	= 'A';
			 REG32(GPS_No.GPS_Config_3) 	= 'C';
			 REG32(GPS_No.GPS_Config_4) 	= 'M';
			 REG32(GPS_No.GPS_Config_5) 	= 'C';
			 REG32(GPS_No.GPS_Config_6) 	= 'C';
			 REG32(GPS_No.GPS_Config_7) 	= ',';
			 REG32(GPS_No.GPS_Config_8)		= '4';
			 REG32(GPS_No.GPS_Config_9)		= ',';
			 REG32(GPS_No.GPS_Config_10)	= '0';
			 REG32(GPS_No.GPS_Config_11)	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12)	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x000062CF;//0b_0111_1000_0100_0000
            #endif

            #ifndef ASCII

			 REG32(GPS_No.GPS_Config_1)  	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_2)  	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_3)  	= 0x0000000A;
			 REG32(GPS_No.GPS_Config_4)  	= 0x0000000C;
			 REG32(GPS_No.GPS_Config_5)  	= 0x000000A2;
			 REG32(GPS_No.GPS_Config_6)  	= 0x000000EA;
			 REG32(GPS_No.GPS_Config_7) 	= 0x000000CF;
			 REG32(GPS_No.GPS_Config_8) 	= 0x00000004;
			 REG32(GPS_No.GPS_Config_9)  	= 0x00000000;
			 REG32(GPS_No.GPS_Config_10) 	= 0x00000085;
			 REG32(GPS_No.GPS_Config_11) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) 	= 0x0000000A;

		 	//GPS1_STATUS_REGISTER2 = 207;

			REG32(GPS_No.GPS_Status_Register_2)   = 0x000062CF;//0b_0111_1000_0100_0000

            #endif

		   }

		 else if(Config_Type == NMEA_GGA_Enable)
		 {
            #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) 	= '$';
			 REG32(GPS_No.GPS_Config_2) 	= 'A';
			 REG32(GPS_No.GPS_Config_3) 	= 'C';
			 REG32(GPS_No.GPS_Config_4) 	= 'M';
			 REG32(GPS_No.GPS_Config_5) 	= 'C';
			 REG32(GPS_No.GPS_Config_6) 	= 'C';
			 REG32(GPS_No.GPS_Config_7) 	= ',';
			 REG32(GPS_No.GPS_Config_8) 	= '1';
			 REG32(GPS_No.GPS_Config_9)		= ',';
			 REG32(GPS_No.GPS_Config_10)	= '1';
			 REG32(GPS_No.GPS_Config_11)	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12)	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x00006322;//0b_0_1100_0_1_100100010
            #endif

            #ifndef ASCII

			 REG32(GPS_No.GPS_Config_1)  = 0x0000003f;
			 REG32(GPS_No.GPS_Config_2)  = 0x0000003f;
			 REG32(GPS_No.GPS_Config_3)  = 0x0000000A;
			 REG32(GPS_No.GPS_Config_4)  = 0x0000000C;
			 REG32(GPS_No.GPS_Config_5)  = 0x000000A2;
			 REG32(GPS_No.GPS_Config_6)  = 0x000000EA;
			 REG32(GPS_No.GPS_Config_7)  = 0x000000CF;
			 REG32(GPS_No.GPS_Config_8)  = 0x00000001;
			 REG32(GPS_No.GPS_Config_9)  = 0x00000001;
			 REG32(GPS_No.GPS_Config_10) = 0x00000081;
			 REG32(GPS_No.GPS_Config_11) = 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) = 0x0000000A;


			 REG32(GPS_No.GPS_Status_Register_2)    = 0x00006322;//0b_0011_1000_0100_0000

		 	//GPS1_STATUS_REGISTER2 = 83+207;

            #endif

		 }
		 else if(Config_Type == NMEA_GGA_Disable)
		 {
		    #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) = '$';
			 REG32(GPS_No.GPS_Config_2) = 'A';
			 REG32(GPS_No.GPS_Config_3) = 'C';
			 REG32(GPS_No.GPS_Config_4) = 'M';
			 REG32(GPS_No.GPS_Config_5) = 'C';
			 REG32(GPS_No.GPS_Config_6) = 'C';
			 REG32(GPS_No.GPS_Config_7) = ',';
			 REG32(GPS_No.GPS_Config_8) = '1';
			 REG32(GPS_No.GPS_Config_9) = ',';
			 REG32(GPS_No.GPS_Config_10)= '0';
			 REG32(GPS_No.GPS_Config_11)= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12)= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x000062CF;//0b_0_1100_0_1_011001111
		    #endif

		    #ifndef ASCII

			 REG32(GPS_No.GPS_Config_1)  = 0x0000003f;
			 REG32(GPS_No.GPS_Config_2)  = 0x0000003f;
			 REG32(GPS_No.GPS_Config_3)  = 0x0000000A;
			 REG32(GPS_No.GPS_Config_4)  = 0x0000000C;
			 REG32(GPS_No.GPS_Config_5)  = 0x000000A2;
			 REG32(GPS_No.GPS_Config_6)  = 0x000000EA;
			 REG32(GPS_No.GPS_Config_7)  = 0x000000CF;
			 REG32(GPS_No.GPS_Config_8)  = 0x00000001;
			 REG32(GPS_No.GPS_Config_9)  = 0x00000000;
			 REG32(GPS_No.GPS_Config_10) = 0x00000080;
			 REG32(GPS_No.GPS_Config_11) = 0x0000000d;
			 REG32(GPS_No.GPS_Config_12) = 0x0000000a;

		    //GPS1_STATUS_REGISTER2 = 207;

			 REG32(GPS_No.GPS_Status_Register_2)   = 0x000062CF;//0b_1111_1000_0100_0000


		    #endif

		 }
		 else if(Config_Type == NMEA_GSV_Enable)
		 {
            #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) = '$';
			 REG32(GPS_No.GPS_Config_2) = 'A';
			 REG32(GPS_No.GPS_Config_3) = 'C';
			 REG32(GPS_No.GPS_Config_4) = 'M';
			 REG32(GPS_No.GPS_Config_5) = 'C';
			 REG32(GPS_No.GPS_Config_6) = 'C';
			 REG32(GPS_No.GPS_Config_7)= ',';
			 REG32(GPS_No.GPS_Config_8) = '5';
			 REG32(GPS_No.GPS_Config_9)= ',';
			 REG32(GPS_No.GPS_Config_10)= '1';
			 REG32(GPS_No.GPS_Config_11)= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12)= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x000063A7;//0b_0_1100_0_1_110100111
            #endif

            #ifndef ASCII

			 GPS_GSV_FLAG = 1;

			 REG32(GPS_No.GPS_Config_1) = 0x0000003f;
			 REG32(GPS_No.GPS_Config_2) = 0x0000003f;
			 REG32(GPS_No.GPS_Config_3) = 0x0000000A;
			 REG32(GPS_No.GPS_Config_4) = 0x0000000C;
			 REG32(GPS_No.GPS_Config_5) = 0x000000A2;
			 REG32(GPS_No.GPS_Config_6) = 0x000000EA;
			 REG32(GPS_No.GPS_Config_7) = 0x000000CF;
			 REG32(GPS_No.GPS_Config_8) = 0x00000005;
			 REG32(GPS_No.GPS_Config_9) = 0x00000001;
			 REG32(GPS_No.GPS_Config_10) = 0x00000085;
			 REG32(GPS_No.GPS_Config_11) = 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) = 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)   = 0x000063A7;//0b_0_1100_0_1_110100111

			 //GPS1_STATUS_REGISTER2 = 72*3 + 207 = 423;

            #endif

		 }
		 else if(Config_Type == NMEA_GSV_Disable)
		 {
            #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) 	= '$';
			 REG32(GPS_No.GPS_Config_2) 	= 'A';
			 REG32(GPS_No.GPS_Config_3) 	= 'C';
			 REG32(GPS_No.GPS_Config_4) 	= 'M';
			 REG32(GPS_No.GPS_Config_5) 	= 'C';
			 REG32(GPS_No.GPS_Config_6) 	= 'C';
			 REG32(GPS_No.GPS_Config_7) 	= ',';
			 REG32(GPS_No.GPS_Config_8) 	= '5';
			 REG32(GPS_No.GPS_Config_9)		= ',';
			 REG32(GPS_No.GPS_Config_10)	= '0';
			 REG32(GPS_No.GPS_Config_11)	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12)	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x000062CF;//0b_0_1100_0_1_011001111
            #endif


            #ifndef ASCII
			 GPS_GSV_FLAG = 0;
			 REG32(GPS_No.GPS_Config_1) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_2) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_3) 	= 0x0000000A;
			 REG32(GPS_No.GPS_Config_4)		= 0x0000000C;
			 REG32(GPS_No.GPS_Config_5) 	= 0x000000A2;
			 REG32(GPS_No.GPS_Config_6) 	= 0x000000EA;
			 REG32(GPS_No.GPS_Config_7)		= 0x000000CF;
			 REG32(GPS_No.GPS_Config_8) 	= 0x00000005;
			 REG32(GPS_No.GPS_Config_9) 	= 0x00000000;
			 REG32(GPS_No.GPS_Config_10)	= 0x00000084;
			 REG32(GPS_No.GPS_Config_11) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) 	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)   =0x000062CF;//0b_0_1100_0_1_011001111

		 	//GPS1_STATUS_REGISTER2 = 207;

            #endif


		 }
		 else if(Config_Type == NMEA_VTG_Enable)     //VTG_EN
		 {
		    #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) 	= '$';
			 REG32(GPS_No.GPS_Config_2) 	= 'A';
			 REG32(GPS_No.GPS_Config_3) 	= 'C';
			 REG32(GPS_No.GPS_Config_4) 	= 'M';
			 REG32(GPS_No.GPS_Config_5) 	= 'C';
			 REG32(GPS_No.GPS_Config_6) 	= 'C';
			 REG32(GPS_No.GPS_Config_7)		= ',';
			 REG32(GPS_No.GPS_Config_8) 	= '6';
			 REG32(GPS_No.GPS_Config_9)		= ',';
			 REG32(GPS_No.GPS_Config_10)	= '1';
			 REG32(GPS_No.GPS_Config_11)	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12)	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x00006300; //16'b_0_1100_0_1_100000000‬
		    #endif


		    #ifndef ASCII

			 REG32(GPS_No.GPS_Config_1) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_2) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_3) 	= 0x0000000A;
			 REG32(GPS_No.GPS_Config_4) 	= 0x0000000C;
			 REG32(GPS_No.GPS_Config_5)		= 0x000000A2;
			 REG32(GPS_No.GPS_Config_6) 	= 0x000000EA;
			 REG32(GPS_No.GPS_Config_7) 	= 0x000000CF;
			 REG32(GPS_No.GPS_Config_8) 	= 0x00000006;
			 REG32(GPS_No.GPS_Config_9) 	= 0x00000001;
			 REG32(GPS_No.GPS_Config_10) 	= 0x00000086;
			 REG32(GPS_No.GPS_Config_11) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) 	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)   = 0x00006300; //16'b_0_1100_0_1_100000000‬

		 	//GPS1_STATUS_REGISTER2 = 39 + 207;

		    #endif


		 }
		 else if(Config_Type == NMEA_VTG_Disable) //VTG_DIS
		 {

		    #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) = '$';
			 REG32(GPS_No.GPS_Config_2) = 'A';
			 REG32(GPS_No.GPS_Config_3) = 'C';
			 REG32(GPS_No.GPS_Config_4) = 'M';
			 REG32(GPS_No.GPS_Config_5) = 'C';
			 REG32(GPS_No.GPS_Config_6) = 'C';
			 REG32(GPS_No.GPS_Config_7) = ',';
			 REG32(GPS_No.GPS_Config_8) = '6';
			 REG32(GPS_No.GPS_Config_9)	= ',';
			 REG32(GPS_No.GPS_Config_10)= '0';
			 REG32(GPS_No.GPS_Config_11)= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12)= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2) = 0x000062CF; //0b_0_1100_0_1_011001111
		    #endif


		    #ifndef ASCII

			 REG32(GPS_No.GPS_Config_1) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_2) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_3) 	= 0x0000000A;
			 REG32(GPS_No.GPS_Config_4) 	= 0x0000000C;
			 REG32(GPS_No.GPS_Config_5) 	= 0x000000A2;
			 REG32(GPS_No.GPS_Config_6) 	= 0x000000EA;
			 REG32(GPS_No.GPS_Config_7)		= 0x000000CF;
			 REG32(GPS_No.GPS_Config_8)		= 0x00000006;
			 REG32(GPS_No.GPS_Config_9) 	= 0x00000000;
			 REG32(GPS_No.GPS_Config_10) 	= 0x00000087;
			 REG32(GPS_No.GPS_Config_11) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_12) 	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)   = 0x000062CF; //0b_0_1100_0_1_011001111

		 	//GPS1_STATUS_REGISTER2 = 207;

		    #endif


		 }
		 else if(Config_Type == FACTORY_RESET)
		 {

            #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) = '$';
			 REG32(GPS_No.GPS_Config_2) = 'A';
			 REG32(GPS_No.GPS_Config_3) = 'C';
			 REG32(GPS_No.GPS_Config_4) = 'F';
			 REG32(GPS_No.GPS_Config_5) = 'R';
			 REG32(GPS_No.GPS_Config_6) = 'M';
			 REG32(GPS_No.GPS_Config_7) = 0x0000000D;
			 REG32(GPS_No.GPS_Config_8) = 0x0000000A;
			 REG32(GPS_No.GPS_Status_Register_2)   = 0x000042CF; //0b_0_1000_0_1_011001111
            #endif

            #ifndef ASCII

			 REG32(GPS_No.GPS_Config_1)  	=  0x0000003f;
			 REG32(GPS_No.GPS_Config_2)  	=  0x0000003f;
			 REG32(GPS_No.GPS_Config_3)  	=  0x0000000A;
			 REG32(GPS_No.GPS_Config_4)  	=  0x0000000C;
			 REG32(GPS_No.GPS_Config_5) 	=  0x000000A2;
			 REG32(GPS_No.GPS_Config_6) 	=  0x000000B0;
			 REG32(GPS_No.GPS_Config_7)  	=  0x00000000;
			 REG32(GPS_No.GPS_Config_8)  	=  0x00000014;
			 REG32(GPS_No.GPS_Config_9)  	=  0x0000000D;
			 REG32(GPS_No.GPS_Config_10) 	=  0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)   = 0x000052CF; //0b_0_1010_0_1_011001111

			//GPS1_STATUS_REGISTER2 = 207;

            #endif
		 }
		 else if(Config_Type == COLD_START)
		 {

            #ifdef ASCII
			 REG32(GPS_No.GPS_Config_1) 	= '$';
			 REG32(GPS_No.GPS_Config_2) 	= 'A';
			 REG32(GPS_No.GPS_Config_3) 	= 'C';
			 REG32(GPS_No.GPS_Config_4) 	= 'C';
			 REG32(GPS_No.GPS_Config_5) 	= 'S';
			 REG32(GPS_No.GPS_Config_6) 	= 'M';
			 REG32(GPS_No.GPS_Config_7) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_8) 	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)   = 0x000042CF; //0b_0_1000_0_1_011001111
            #endif

            #ifndef ASCII

			 REG32(GPS_No.GPS_Config_1) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_2) 	= 0x0000003f;
			 REG32(GPS_No.GPS_Config_3) 	= 0x0000000A;
			 REG32(GPS_No.GPS_Config_4) 	= 0x0000000C;
			 REG32(GPS_No.GPS_Config_5) 	= 0x000000A2;
			 REG32(GPS_No.GPS_Config_6) 	= 0x000000B0;
			 REG32(GPS_No.GPS_Config_7) 	= 0x00000001;
			 REG32(GPS_No.GPS_Config_8) 	= 0x00000015;
			 REG32(GPS_No.GPS_Config_9) 	= 0x0000000D;
			 REG32(GPS_No.GPS_Config_10) 	= 0x0000000A;

			 REG32(GPS_No.GPS_Status_Register_2)    = 0x000052CF;//0b_0_1010_0_1_011001111

			//GPS1_STATUS_REGISTER2 = 207;

            #endif

		 }

  return 0;
}

void rGPS_TM_Extract(void)
{
	if(GPS_Data_Read_Status == TRUE) //Transfer to TM Buffer Only after checking the availability and validity of GPS data
	{
//		if (GPS_GSV_FLAG)
//		{
//			GPS_TM_Buffer_Addr_USC = (unsigned char*)&GPS_RCVD_DATA[GPS_count_TM];
//			db_gps_start_address = (unsigned char*)&GPS_RCVD_DATA[GPS_count_TM];
//		}
//		else
//		{
//			GPS_TM_Buffer_Addr_USC = (unsigned char*)GPS_RCVD_DATA;
//			db_gps_start_address = (unsigned char*)GPS_RCVD_DATA;
//		}

		GPS_TM_Buffer_Addr_USC = (unsigned char*)&GPS_RCVD_DATA[0];
		db_gps_start_address = (unsigned char*)&GPS_RCVD_DATA[0];

		TM.Buffer.TM_GPS.TM_No_Of_Sat 					= *(GPS_TM_Buffer_Addr_USC+4);
		ST_normal.ST_NM_Buffer.TM_No_Of_Sat 			= *(GPS_TM_Buffer_Addr_USC+4);
		ST_special.ST_SP_Buffer.TM_GPS.TM_No_Of_Sat 	= *(GPS_TM_Buffer_Addr_USC+4);
		TM.Buffer.TM_GPS.TM_UTC_Day 					= *(GPS_TM_Buffer_Addr_USC+150);
		ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_Day 		= *(GPS_TM_Buffer_Addr_USC+150);
		ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_Day 		= *(GPS_TM_Buffer_Addr_USC+150);
		TM.Buffer.TM_GPS.TM_UTC_Month 					= *(GPS_TM_Buffer_Addr_USC+151);
		ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_month 		= *(GPS_TM_Buffer_Addr_USC+151);
		TM.Buffer.TM_GPS.TM_UTC_Year 					= *((unsigned short*)(GPS_TM_Buffer_Addr_USC+152));
		ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_Year 		= *((unsigned short*)(GPS_TM_Buffer_Addr_USC+152));
		TM.Buffer.TM_GPS.TM_UTC_hour 					= *(GPS_TM_Buffer_Addr_USC+154);
		ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_hour 		= *(GPS_TM_Buffer_Addr_USC+154);
		ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_hour 		= *(GPS_TM_Buffer_Addr_USC+154);
		TM.Buffer.TM_GPS.TM_UTC_Min 					= *(GPS_TM_Buffer_Addr_USC+155);
		ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_Min 		= *(GPS_TM_Buffer_Addr_USC+155);
		ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_Min 		= *(GPS_TM_Buffer_Addr_USC+155);
		TM.Buffer.TM_GPS.TM_UTC_Sec 					= *((unsigned short*)(GPS_TM_Buffer_Addr_USC+156));
		ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_Sec 		= *((unsigned short*)(GPS_TM_Buffer_Addr_USC+156));
		ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_Sec 		= *((unsigned short*)(GPS_TM_Buffer_Addr_USC+156));
		TM.Buffer.TM_GPS.TM_GPSWeek 					= *((unsigned short*)(GPS_TM_Buffer_Addr_USC+158));
		TM.Buffer.TM_GPS.TM_GPS_Xpos 					= *((int*)(GPS_TM_Buffer_Addr_USC+160));
		ST_special.ST_SP_Buffer.TM_GPS.TM_GPS_Xpos 		= *((int*)(GPS_TM_Buffer_Addr_USC+160));
		TM.Buffer.TM_GPS.TM_GPS_Ypos 					= *((int*)(GPS_TM_Buffer_Addr_USC+164));
		ST_special.ST_SP_Buffer.TM_GPS.TM_GPS_Ypos 		= *((int*)(GPS_TM_Buffer_Addr_USC+164));
		TM.Buffer.TM_GPS.TM_GPS_Zpos 					= *((int*)(GPS_TM_Buffer_Addr_USC+168));
		ST_special.ST_SP_Buffer.TM_GPS.TM_GPS_Zpos 		= *((int*)(GPS_TM_Buffer_Addr_USC+168));
		TM.Buffer.TM_GPS.TM_GPS_XVel 					= *((int*)(GPS_TM_Buffer_Addr_USC+172));
		ST_special.ST_SP_Buffer.TM_GPS.TM_GPS_XVel 		= *((int*)(GPS_TM_Buffer_Addr_USC+172));
		TM.Buffer.TM_GPS.TM_GPS_YVel 					= *((int*)(GPS_TM_Buffer_Addr_USC+176));
		ST_special.ST_SP_Buffer.TM_GPS.TM_GPS_YVel 		= *((int*)(GPS_TM_Buffer_Addr_USC+176));
		TM.Buffer.TM_GPS.TM_GPS_ZVel 					= *((int*)(GPS_TM_Buffer_Addr_USC+180));
		ST_special.ST_SP_Buffer.TM_GPS.TM_GPS_ZVel 		= *((int*)(GPS_TM_Buffer_Addr_USC+180));
		TM.Buffer.TM_GPS.TM_No_Of_Sat_InFix 			= *(GPS_TM_Buffer_Addr_USC+185);
		TM.Buffer.TM_GPS.TM_Pos_Validity 				= *(GPS_TM_Buffer_Addr_USC+186);
		ST_special.ST_SP_Buffer.TM_GPS.TM_Pos_Validity 	= *(GPS_TM_Buffer_Addr_USC+186);
		TM.Buffer.TM_GPS.TM_BIST_Info 					= *(GPS_TM_Buffer_Addr_USC+203);
		ST_normal.ST_NM_Buffer.TM_GPS.TM_BIST_Info 		= *(GPS_TM_Buffer_Addr_USC+203);
		bist_data  										= *(GPS_TM_Buffer_Addr_USC+203);
		TM.Buffer.TM_GPS.TM_CheckSum 					= *(GPS_TM_Buffer_Addr_USC+204);

		//Computation of On board Checksum of GPS data
		GPS_obc_checkum = chksum8(db_gps_start_address,204);

		//Check for BIST Information and Checksum for the validation of Data
		//Check the above mentioned info to raise "GPS_Data_Validity_flag" (which will be used for OBC_GPS processing)
		if(GPS_obc_checkum == TM.Buffer.TM_GPS.TM_CheckSum)
		{
			if (TC_boolean_u.TC_Boolean_Table.TC_BIST_override == 1)
			{
				f_GPS_Valid_Data = True;
			}
			else
			{
				if(bist_data == BIST_DATA)
				{
					f_GPS_Valid_Data = True;
				}
				else
				{
					f_GPS_Valid_Data = False;
				}
			}
		}
		else
		{
			f_GPS_Valid_Data = False;
		}
	}
}


unsigned long int gps_data_test[106];
unsigned int gps_data_length;
void ST_TM_gps_data(void)
{
	unsigned int gps_data_copy_index;
	unsigned int tempdata;
	unsigned char *GPS_TM_Buffer_Addr;
	GPS_TM_Buffer_Addr = (unsigned char*)GPS_RCVD_DATA;

	if(TC_boolean_u.TC_Boolean_Table.TC_GPS12_Select == GPS_1)
	{
		gps_data_length = GPS1_STATUS_REGISTER2 & 0x000001FF;
	}
	else
	{
		gps_data_length = GPS2_STATUS_REGISTER2 & 0x000001FF;
	}
	for(gps_data_copy_index = 0; gps_data_copy_index <= gps_data_length; gps_data_copy_index++)
	{
		ST_special.ST_SP_Buffer.ST_TM_GPS_RCVD_DATA[gps_data_copy_index] = *GPS_TM_Buffer_Addr++;
	}

}
