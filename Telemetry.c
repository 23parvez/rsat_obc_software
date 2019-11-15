#include <stdio.h>
#include "Telemetry.h"
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "TM_Global_Buffer.h"
#include "Telecommand.h"
#include "adcs_VarDeclarations.h"

void rHAL_TM_Write(void)
{

	unsigned long int tempdata;
	static unsigned short* inter_TM_Write_Source_Addr = (unsigned short*)TM_Buffer;

		inter_TM_Status_Data = TM_STATUS_REGISTER;              // Read Status Register Data
		inter_TM_DP_Addr = (unsigned long int*)TM_BUFFER_BASE;  // Assign DP Memory pointer

		//Write 32Bytes Data to FPGA Dual Port Memory

		if((inter_TM_Status_Data & TM_DP_MEMORY_FULL) == TM_DP_MEMORY_EMPTY) // Check Dual Port Memory is Empty or Not
		{
			GPIO_pins.PIO_5 = 1;
			IODAT = GPIO_pins.data;
			tempdata = (inter_TM_Status_Data | TM_DP_MEMORY_WRITE_ENABLE);   // Set WR Bit Before Writing TM Buffer
			 TM_STATUS_REGISTER;
			 TM_STATUS_REGISTER = tempdata;

			for(inter_HAL_TM_Write_Word_Count = 0;inter_HAL_TM_Write_Word_Count <= (TM_DP_MEMORY_SIZE-1) ; inter_HAL_TM_Write_Word_Count++)
			{
				tempdata = ~(unsigned long int)*inter_TM_Write_Source_Addr++; // Invert Data (On board having Inverter)
				TM_STATUS_REGISTER;
				*inter_TM_DP_Addr = tempdata;                                 // Invert Data (On board having Inverter)
				inter_TM_DP_Addr++;

			}
			TM_STATUS_REGISTER = (inter_TM_Status_Data & TM_DP_MEMORY_WRITE_DISABLE);//Reset WR Bit After Writing to DPM
			GPIO_pins.PIO_5 = 0;
			IODAT = GPIO_pins.data;
			inter_TM_Minor_Cycle_Count++;//Update TM Minor Cycle Count for next TM Minor Cycle Operation
			if(inter_TM_Minor_Cycle_Count == 8)
			{
				inter_TM_Minor_Cycle_Count = 0;
				inter_TM_Main_Buffer_Empty = TRUE;
				rTM_Copy_Subframe();//Copy Subframe to TM Main Buffer if TM Main Buffer is Empty
				inter_TM_Write_Source_Addr = (unsigned short*)TM_Buffer;
			}
		}


		else
		{
			//
		}
}

void rHAL_TM_HW_Status_Update(void)
{
	//Hardware Status Update
	//IMU,RW,TC,TM,GPS,EPS

	//OBT Update
	TM.Buffer.OBT = Major_Cycle_Count;
	TM.Buffer.TC_cntr2=TC_count;

	//FDI_NMI_Count Update
}

void Update_TM_With_TMTC()
{
    //Update TMTC to TM GBL Buffer
}

void rTM_Address_Table_Init()
{
	//SUB FRAME 0
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;                //Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;                                                   //Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;           //Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;  //Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;  //Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];             //Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];            //Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];         //Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;  //Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];   //Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;             //Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);                      //2 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;    //Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);             //8 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[14];   //Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 16;                                           //16 Sun sensor channels(16 bytes)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_BODY[0];      //Copy address of Magnetic field in body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_BODY);//6 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_EKF[0];      //Copy address of Q_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 16;                  //16 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_EKF_Bias[0]; //Copy address of B_EKF_Bias
	TM_Table[TM_Table_Row_No].Length_Field = 12;             //12 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Error_EKF[0]; //Copy address of Error_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 12;             //12 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_EKF_Drift[0];//Copy address of w_EKF Drift
	TM_Table[TM_Table_Row_No].Length_Field = 12;//12 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[34];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;//1byte(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[33];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;//1byte(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[32];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;//1byte(SA_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[0];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;//1byte(SA_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TC_cntr1;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TC_cntr2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe0
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 1
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF1;//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF1[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Pos_ECI[0];//Copy address of Position in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Pos_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Vel_ECI[0]; // Copy address of Velocity in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Vel_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Latitude;   // Copy address of Latitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Latitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Altitude;   // Copy address of Altitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Altitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Longitude;  // Copy address of Longitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Longitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Period; // Copy address of Orbit Period
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Period);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Tsince;     // Copy address of Tsince
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Tsince);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Elapsd_Time; // Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Elapsd_Time);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_DOT[0]; // Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_DOT);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[31]; // Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = 1; // RX_AGC(pos-226)(added-22/10/19)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.gps_pulse_mic_counter; // Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = 2; // RX_AGC(pos-226)(added-22/10/19)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe1
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 2
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF2;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF2[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Roll_Error;//Copy address of SS_Roll_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Roll_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Yaw_Error;//Copy address of SS_Yaw_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Yaw_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_DeltaSpeed[0];//Copy address of RW_Delta_Speed
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_RW_DeltaSpeed);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_DPM[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_DPM);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Sunmagad[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Sunmagad);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe2
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 3
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF3;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF3[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Thta_BODY[0];//Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Thta_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Output_Latch[0];//Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Output_Latch);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe3
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 4
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF4;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF4[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[14];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 16;//16 Sun sensor channels(16 bytes)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_BODY[0];//Copy address of Magnetic field in body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_EKF[0];//Copy address of Q_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_EKF_Bias[0];//Copy address of B_EKF_Bias
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Error_EKF[0];//Copy address of Error_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_EKF_Drift[0];//Copy address of w_EKF Drift
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[34];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;// 1bytes(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[33];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;// 1bytes(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[32];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;// 1byte(SA_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe4
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 5
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF5;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF5[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Pos_ECI[0];//Copy address of Position in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Pos_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Vel_ECI[0];//Copy address of Velocity in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Vel_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Latitude;//Copy address of Latitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Latitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Altitude;//Copy address of Altitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Altitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Longitude;//Copy address of Longitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Longitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Period;//Copy address of Orbit Period
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Period);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Tsince;//Copy address of Tsince
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Tsince);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Elapsd_Time;//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Elapsd_Time);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_DOT[0];//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_DOT);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe5
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 6
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF6;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF6[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Roll_Error;//Copy address of SS_Roll_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Roll_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Yaw_Error;//Copy address of SS_Yaw_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Yaw_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_DeltaSpeed[0];//Copy address of RW_Delta_Speed
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_RW_DeltaSpeed);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_DPM[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_DPM);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Sunmagad[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Sunmagad);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe6
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 7
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF7;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF7[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Thta_BODY[0];//Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Thta_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_IMU[0];//Copy address of Rates in Sensor Frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_w_IMU);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe7
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 8
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF8;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF8[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[14];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 16;//16 Sun sensor channels(16 bytes)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_BODY[0];//Copy address of Magnetic field in body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_EKF[0];//Copy address of Q_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_EKF_Bias[0];//Copy address of B_EKF_Bias
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Error_EKF[0];//Copy address of Error_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_EKF_Drift[0];//Copy address of w_EKF Drift
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[34];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;//1byte(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[33];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;//1bytes(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[32];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1;//1byte(SA_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe8
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 9
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;


	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF9;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF9[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Pos_ECI[0];//Copy address of Position in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Pos_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Vel_ECI[0];//Copy address of Velocity in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Vel_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Latitude;//Copy address of Latitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Latitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Altitude;//Copy address of Altitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Altitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Longitude;//Copy address of Longitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Longitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Period;//Copy address of Orbit Period
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Period);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Tsince;//Copy address of Tsince
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Tsince);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Elapsd_Time;//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Elapsd_Time);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_DOT[0];//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_DOT);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_S_BODY_Main[0];//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_S_BODY_Red[0];//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe9
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 10
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF10;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF10[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Roll_Error;//Copy address of SS_Roll_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Roll_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Yaw_Error;//Copy address of SS_Yaw_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Yaw_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_DeltaSpeed[0];//Copy address of RW_Delta_Speed
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_RW_DeltaSpeed);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_DPM[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_DPM);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Sunmagad[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Sunmagad);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe10
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 11
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF11;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF11[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Thta_BODY[0];//Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Thta_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Magad[0];//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Magad);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS;//Copy address of GPS Data
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_GPS);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS1_Status;//Copy address of GPS1 Status
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_GPS1_Status);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS2_Status;//Copy address of GPS2 Status
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_GPS2_Status);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe11
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 12
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF12;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF12[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[14];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 16;//16 Sun sensor channels(16 bytes)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_BODY[0];//Copy address of Magnetic field in body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_EKF[0];//Copy address of Q_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_EKF_Bias[0];//Copy address of B_EKF_Bias
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Error_EKF[0];//Copy address of Error_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_EKF_Drift[0];//Copy address of w_EKF Drift
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[34];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1; // 1byte(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[33];//Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1; // 1byte(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[32]; // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 1; // 1byte(SA_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe12
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 13
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF13;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF13[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Pos_ECI[0];//Copy address of Position in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Pos_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Vel_ECI[0];//Copy address of Velocity in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Vel_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Latitude;//Copy address of Latitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Latitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Altitude;//Copy address of Altitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Altitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Longitude;//Copy address of Longitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Longitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Period;//Copy address of Orbit Period
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Period);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Tsince;//Copy address of Tsince
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Tsince);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Elapsd_Time;//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Elapsd_Time);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_DOT[0];//Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_DOT);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_Ref[0];//Copy address of Reference Rate
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_w_Ref);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.TM_ADC_Data[0];//Copy address of Thermistor1
	TM_Table[TM_Table_Row_No].Length_Field = 14;//14 Thermistor channels(14 bytes)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe13
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 14
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;// Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF14;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF14[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Roll_Error;//Copy address of SS_Roll_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Roll_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Yaw_Error;//Copy address of SS_Yaw_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Yaw_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_DeltaSpeed[0];//Copy address of RW_Delta_Speed
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_RW_DeltaSpeed);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_DPM[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_DPM);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Sunmagad[0];//Copy address of SUNMAGAD
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Sunmagad);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Ref[0];//Copy address of Reference Quaternion
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Ref);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Error[0];//Copy address of Error Quaternion
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe14
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 15
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];//Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];//Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF14;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF14[0]; //Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];//Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;//Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;//Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Thta_BODY[0];//Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Thta_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_TC_Buffer[0];//Copy address of TMTC_Buffer
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_TC_Buffer);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe15
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//END OF TABLE
	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;
	TM_Table[TM_Table_Row_No].Length_Field = 0;

}

void rTM_Copy_Subframe()
{
	static struct FRAME_ADDR_TABLE * inter_TM_Table_Addr = TM_Table;
	inter_TM_Dest_Addr   = TM_Buffer;
    inter_TM_Source_Addr = inter_TM_Table_Addr->Addr_Field;

    if(inter_TM_Main_Buffer_Empty == TRUE)
    {
    	GPIO_pins.PIO_11 = 1;
    	IODAT = GPIO_pins.data;
    	for(inter_TM_Byte_Count=0; inter_TM_Byte_Count<TM_BUFFER_LIMIT; inter_TM_Byte_Count++)
    	{
    		TM_Buffer[inter_TM_Byte_Count] = 0x00;
    	}
		while(inter_TM_Source_Addr != NULL){
			for(inter_TM_Byte_Count=0; inter_TM_Byte_Count <= (inter_TM_Table_Addr->Length_Field -1); inter_TM_Byte_Count++){
				*inter_TM_Dest_Addr++ = *inter_TM_Source_Addr++;
			}
			inter_TM_Table_Addr++;
			inter_TM_Source_Addr = inter_TM_Table_Addr->Addr_Field;
		}
		inter_TM_Table_Addr++;
		TM.Buffer.Sub_Frame++;
		if(inter_TM_Table_Addr->Addr_Field == NULL){
			inter_TM_Table_Addr = TM_Table;
			TM.Buffer.Sub_Frame = 0;
		}
		else
		{
			//
		}
		inter_TM_Main_Buffer_Empty = FALSE;
    }

}
