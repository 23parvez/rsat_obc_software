#include <stdio.h>
#include "Global.h"
#include "Telemetry.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "TM_Global_Buffer.h"
#include "Telecommand.h"
#include "adcs_VarDeclarations.h"
#include "TC_List.h"

extern unsigned char NSP_addr_table[4];

unsigned int real_tm_wait = 1;
unsigned int real_tm_finish;
unsigned int flag_for_test;
 // selection between real_TM and storage TM dumping
void rTM_Real_st_write()
{

	if (TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping)      // Enable storage_TM_dumping
	{

		if ((TM.Buffer.Sub_Frame == 1) && (real_tm_wait))
		{
			real_tm_wait          = 0;
			storage_page_count    = 0;
			TM_page_count         = 0;
			real_tm_finish        = 1;
			rt_tm_frame_finish    = 0;

		}

		if (real_tm_finish)
		{
			if(TC_boolean_u.TC_Boolean_Table.TC_dumping)
			{

				TC_Hist_dumping();
			}
			else
			{
				if(TC_boolean_u.TC_Boolean_Table.storage_dump_mode)
				{
					ST_full_dump();
				}
				else
				{
					ST_DUMPING();
				}
			}

		}
			                                      // Function to dump the storage_tm_data

		else
		{
			rHAL_TM_Write();                                       // Real_time TM_dumping
		}
	}

	else
	{
		rHAL_TM_Write();                                           // Real_time TM_dumping
	}
}


//Real_time_TM_write

void rHAL_TM_Write(void)
{
		unsigned long int tempdata;
		static unsigned short* inter_TM_Write_Source_Addr = (unsigned short*)TM_Buffer;

			inter_TM_Status_Data = TM_STATUS_REGISTER;              // Read Status Register Data
			inter_TM_DP_Addr = (unsigned long int*)TM_BUFFER_BASE;  // Assign DP Memory pointer

			//Write 32Bytes Data to FPGA Dual Port Memory

			if((inter_TM_Status_Data & TM_DP_MEMORY_FULL) == TM_DP_MEMORY_EMPTY) // Check Dual Port Memory is Empty or Not
			{

				tempdata = (inter_TM_Status_Data | TM_DP_MEMORY_WRITE_ENABLE);   // Set WR Bit Before Writing TM Buffer
				 TM_STATUS_REGISTER;
				 //TM_STATUS_REGISTER = tempdata;

				for(inter_HAL_TM_Write_Word_Count = 0;inter_HAL_TM_Write_Word_Count <= (TM_DP_MEMORY_SIZE-1) ; inter_HAL_TM_Write_Word_Count++)
				{
					tempdata = ~(unsigned long int)*inter_TM_Write_Source_Addr++; // Invert Data (On board having Inverter)
					TM_STATUS_REGISTER;
					*inter_TM_DP_Addr = tempdata;                                 // Invert Data (On board having Inverter)
					inter_TM_DP_Addr++;

				}
				//TM_STATUS_REGISTER = (inter_TM_Status_Data & TM_DP_MEMORY_WRITE_DISABLE);//  Reset WR Bit After Writing to DPM

				inter_TM_Minor_Cycle_Count++;                                             //   Update TM Minor Cycle Count for next TM Minor Cycle Operation
				if(inter_TM_Minor_Cycle_Count == 4)
				{
					rt_tm_frame_finish = 1;
					inter_TM_Minor_Cycle_Count = 0;
					inter_TM_Main_Buffer_Empty = TRUE;
					rTM_Copy_Subframe();                        //   Copy Subframe to TM Main Buffer if TM Main Buffer is Empty
					TM_page_count++;                            //   Frame_count
					inter_TM_Write_Source_Addr = (unsigned short*)TM_Buffer;
				}
			}

}

void rHAL_TM_HW_Status_Update(void)
{
	//Hardware Status Update
	//IMU,RW,TC,TM,GPS,EPS

	int i;

	//OBT Update
	TM.Buffer.OBT = Major_Cycle_Count;
	ST_special.ST_SP_Buffer.OBT = Major_Cycle_Count;
	ST_normal.ST_NM_Buffer.OBT = Major_Cycle_Count;
	TM.Buffer.TC_rcvd_cntr2 = TC_count;
	TM.Buffer.TC_cmd_executed_cntr2 = TC_cmd_executed;
	TM.Buffer.TC_ATT_cntr2 = ATTC_count;
	TM.Buffer.TC_pending_cntr2 = TC_command_pending;

	ST_normal.ST_NM_Buffer.TC_rcvd_cntr2 = TC_count;




	for(i= 0 ; i<RW_TM_MAX ; i++)
	{
		TM.Buffer.TM_NSP_addr_table[i] = NSP_addr_table[i];
	}

	//FDI_NMI_Count Update
	TM.Buffer.TM_FDI_NMI_Count = FDI_NMI_Count;
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
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;  //Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;  // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];             // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];            // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];         // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;  // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[0]; // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0];   // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR;             // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);                      // 2 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC;    // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);             // 8 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[0];   // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 16;                                           // 16 Sun sensor channels(16 bytes)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_BODY[0];      // Copy address of Magnetic field in body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_BODY);//6 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_EKF[0];      // Copy address of Q_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 16;                  // 16 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_EKF_Bias[0]; // Copy address of B_EKF_Bias
	TM_Table[TM_Table_Row_No].Length_Field = 12;             // 12 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Error_EKF[0]; // Copy address of Error_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 12;             // 12 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_EKF_Drift[0]; // Copy address of w_EKF Drift
	TM_Table[TM_Table_Row_No].Length_Field = 12;//12 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_voltage; // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_current;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SA_current ;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(SA_current)
	TM_Table_Row_No++;


	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TC_rcvd_cntr2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe0
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	// SUB FRAME 1
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[16];//Copy address of RW speed
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Elapsd_Time;
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Elapsd_Time);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_DOT[0];
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_DOT);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_auto_manual;      // to be removed
	TM_Table[TM_Table_Row_No].Length_Field = 1; // 1 bytes for heater control
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_pl_data[0];
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 2 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_pl_data[1];
	TM_Table[TM_Table_Row_No].Length_Field = 16; // 16 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_pl_data[9];
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 2 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_NSP_addr_table[0];
	TM_Table[TM_Table_Row_No].Length_Field = 4; // 4 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_AGC;
	TM_Table[TM_Table_Row_No].Length_Field = 2; // RX_AGC(pos-226)(added-22/10/19)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe1
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 2
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch; // Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED; // Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0];       // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0];       // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0];     // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0; // Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[32]; // Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0]; // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR; // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Roll_Error; // Copy address of SS_Roll_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Roll_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Yaw_Error; //  Copy address of SS_Yaw_Error
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_SunSens_Yaw_Error);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_DeltaSpeed[0];  // Copy address of RW_Delta_Speed
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_RW_DeltaSpeed);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_DPM[0];            // Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_DPM);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Sunmagad[0];     // Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Sunmagad);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_heaters_auto_manual;  // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 4 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_heaters_manual_control;  // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 4 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;                                                               //total no of bytes are 228bytes

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe2
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 3
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch; // Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED; // Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0]; // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0]; // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0]; // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0; // Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[48]; // Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0]; // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR; // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Thta_BODY[0]; // Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Thta_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Output_Latch[0]; // Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Output_Latch);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_battery_temp_1;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_battery_temp_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.SA1_Shunt_sw ;
	TM_Table[TM_Table_Row_No].Length_Field = 2; // SA_1 switch status
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.SA2_Shunt_sw ;
	TM_Table[TM_Table_Row_No].Length_Field = 2; // SA_2 switch status
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.SA3_Shunt_sw ;
	TM_Table[TM_Table_Row_No].Length_Field = 2; // SA_3 switch status
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_voltage2 ;  // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 4 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_pl_ack_count ;  // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 2 bytes                                      //206  bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SA_thermistor1 ;  // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 2 bytes                                      //208  bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SA_thermistor2 ;  // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 2 bytes                                      //210  bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_NMI_fail_count ;  // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 1; // 1 bytes                                      //211  bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr;     // added(26/10/19)
	TM_Table[TM_Table_Row_No].Length_Field = 4; // 4 bytes                                      //215  bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;                                                          //216 bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe3
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 4
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch; // Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;  // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0]; // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0]; // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0]; // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0; // Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[64]; // Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0]; // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR; // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[0]; // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 16; // 16 Sun sensor channels(16 bytes)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_BODY[0]; // Copy address of Magnetic field in body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_EKF[0]; // Copy address of Q_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_EKF_Bias[0]; // Copy address of B_EKF_Bias
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Error_EKF[0]; // Copy address of Error_EKF
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_EKF_Drift[0]; // Copy address of w_EKF Drift
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_voltage; // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_current;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SA_current ;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2; // 1byte(SA_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_FDI_NMI_Count;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_Antenna_ACK;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe4
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 5
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch; // Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED; // Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0]; // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0]; // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0]; // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0; // Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[80]; // Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0]; // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR; // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Pos_ECI[0]; // Copy address of Position in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Pos_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Vel_ECI[0]; // Copy address of Velocity in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Vel_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Latitude; // Copy address of Latitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Latitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Altitude; // Copy address of Altitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Altitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Longitude; // Copy address of Longitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Longitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Period; // Copy address of Orbit Period
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Period);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Tsince; // Copy address of Tsince
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Tsince);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Elapsd_Time; // Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Elapsd_Time);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_B_DOT[0]; // Copy address of Orbit Elapsed time
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_B_DOT);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe5
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 6
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch; // Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED; // Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0]; // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0]; // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0]; // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0; // Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[96]; // Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0]; // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR; // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SunSens_Roll_Error; // Copy address of SS_Roll_Error
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

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_1;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_3;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_4;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_5;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_6;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_7;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_MUX_8;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe6
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 7
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[112]; //Copy address of remote data
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
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Thta_BODY); //12bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_IMU[0];//Copy address of Rates in Sensor Frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_w_IMU);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ATTC_count;//Copy address of Rates in Sensor Frame
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe7
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 8
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[128]; //Copy address of remote data
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[0];//Copy address of Sunsensor1
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_voltage; // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_current;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SA_current ;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(SA_current)
	TM_Table_Row_No++;


	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe8
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 9
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch; // Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[144]; //Copy address of remote data
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

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe9
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 10
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; //Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[160]; //Copy address of remote data
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Thermistor_Data[0];//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = 28;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Thermistor_15;//Copy address of DPM
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;


	/*TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_1; //Copy address of Thermistor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_2; //Copy address of Thermistor2
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_3; //Copy address of Thermistor3
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_4; //Copy address of Thermistor4
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_5; //Copy address of Thermistor5
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_6; //Copy address of Thermistor6
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_7; //Copy address of Thermistor7
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_8; //Copy address of Thermistor8
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_9; //Copy address of Thermistor9
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_10; //Copy address of Thermistor10
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_11; //Copy address of Thermistor11
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_12; //Copy address of Thermistor12
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_13; //Copy address of Thermistor13
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_14; //Copy address of Thermistor14
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = &TM.Buffer.Thermister_15; //Copy address of Thermistor15
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;*/

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL; // End of Subframe10
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 11
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0]; // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0]; // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0]; // Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0; // Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[176]; // Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0]; // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR; // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Thta_BODY[0]; // Copy address of Theta in Body frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Thta_BODY);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_Magad[0]; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Q_Magad);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS; // Copy address of GPS Data
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_GPS);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS1_Status;//Copy address of GPS1 Status
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_GPS1_Status);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS2_Status;//Copy address of GPS2 Status
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_GPS2_Status);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS_OBT_Read_1;//Copy address of GPS2 Status
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_GPS_OBT_Read_2;//Copy address of GPS2 Status
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[192]; //Copy address of remote data
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_ADC_Data[0];//Copy address of Sunsensor1
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_voltage; // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_voltage)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Bus_current;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(bus_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_SA_current ;  // Copy address of Sunsensor1
	TM_Table[TM_Table_Row_No].Length_Field = 2;//1byte(SA_current)
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG; // Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6;// IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Q_BODY[0]; // Copy address of Q_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_w_BODY[0]; // Copy address of w_BODY
	TM_Table[TM_Table_Row_No].Length_Field = 12;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_RW_Speed[0]; //Copy address of RW speed
	TM_Table[TM_Table_Row_No].Length_Field = 16;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0; // Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[208]; // Copy address of remote data
	TM_Table[TM_Table_Row_No].Length_Field = 64;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Input_Latch[0]; // Copy address of Input Latch
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_MTR; // Copy address of MTR
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_MTR);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Last_seen_TC; // Copy address of last seen TC
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Last_seen_TC);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Pos_ECI[0]; // Copy address of Position in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Pos_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Vel_ECI[0]; // Copy address of Velocity in ECI frame
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Vel_ECI);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Latitude; // Copy address of Latitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Latitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Altitude; // Copy address of Altitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Altitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Longitude; // Copy address of Longitude
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Longitude);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Orb_Period; // Copy address of Orbit Period
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_Orb_Period);
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Tsince; // Copy address of Tsince
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_TC_exe;//Copy address of Reference Rate
	TM_Table[TM_Table_Row_No].Length_Field = 8;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe13
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//SUB FRAME 14
	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.FrameSynch;//Copy address of Framesync
	TM_Table[TM_Table_Row_No].Length_Field = 10; // Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU_SELECTED;//Copy address of Selected IMU rawdata
	TM_Table[TM_Table_Row_No].Length_Field = 18; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU1.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_IMU2.TM_IMU_Diag_REG;//Copy address of IMU1 Reg,Dia,Temp
	TM_Table[TM_Table_Row_No].Length_Field = 6; // IMU
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[224]; //Copy address of remote data
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_TC_cmd_executed_count ;//Copy address of Error Quaternion
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
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

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Addr_SF0;//Copy address of Remote Addr
	TM_Table[TM_Table_Row_No].Length_Field = 4;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = (unsigned char*)&TM.Buffer.TM_Remote_Data_SF0[240]; //Copy address of remote data
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
	TM_Table[TM_Table_Row_No].Length_Field = sizeof(TM.Buffer.TM_TC_Buffer);  //64 byte
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field = (unsigned char*)&TM.Buffer.TM_inter_HAL_IMU_Status_Data_2;
	TM_Table[TM_Table_Row_No].Length_Field = 2;
	TM_Table_Row_No++;

	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;//End of Subframe15
	TM_Table[TM_Table_Row_No].Length_Field = 0;
	TM_Table_Row_No++;

	//END OF TABLE
	TM_Table[TM_Table_Row_No].Addr_Field   = NULL;
	TM_Table[TM_Table_Row_No].Length_Field = 0;

}


unsigned int cp_cnt,cp_cnt1;

// Copy the Real_Time_TM to 256 buffer
void rTM_Copy_Subframe()
{
	unsigned char* pl_tm_buffer_ptr;
	unsigned char tempdata;
	static struct FRAME_ADDR_TABLE * inter_TM_Table_Addr = TM_Table;
	inter_TM_Dest_Addr   = TM_Buffer;
    inter_TM_Source_Addr = inter_TM_Table_Addr->Addr_Field;
    pl_tm_buffer_ptr     = &Pl_tm_data.pl_tm_1byte[0];

    if(inter_TM_Main_Buffer_Empty == TRUE)
    {
    	for(inter_TM_Byte_Count= 0; inter_TM_Byte_Count<TM_BUFFER_LIMIT; inter_TM_Byte_Count++)
    	{
    		TM_Buffer[inter_TM_Byte_Count] = 0x00;
    	}
		while(inter_TM_Source_Addr != NULL)
		{
			for(inter_TM_Byte_Count=0; inter_TM_Byte_Count <= (inter_TM_Table_Addr->Length_Field -1); inter_TM_Byte_Count++)
			{
				*inter_TM_Dest_Addr = *inter_TM_Source_Addr++;    // pl_update(17-1-2020)
				tempdata            = *inter_TM_Dest_Addr++ ;
				*pl_tm_buffer_ptr++ = tempdata;
			}
			inter_TM_Pl_Buffer = TRUE;
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

 // Storage_telemetry

void Storage_Telemetry_Write()
{
	//TC_boolean_u.TC_Boolean_Table.TC_NormalStorage_Sampling_Rate_Select = 0;
	if(TC_boolean_u.TC_Boolean_Table.TC_Storage_TM_Enable_Disable == TRUE)                       // enable_disable_storage_telemetry
	{
	   //TM.Buffer.Real_Storage_TM=TC_boolean_u.TC_Boolean_Table.TC_Storage_TM_Enable_Disable;
	   StorageTelemetry();

	}
	else
    {
	  //
    }
}
unsigned int real_storage,TC_OBT_Start;
unsigned int sampling_rate_count;
unsigned int normal_count1;
void StorageTelemetry()
{
	unsigned int Sampling_Rate = Sampling_Rate_Select();
	unsigned int Frame_Address = Frame_Address_Select();  // frame address select

	ST_Copy_Subframe(Frame_Address,Sampling_Rate);
	TC_OBT_Start=1;
}

int Sampling_Rate_Select(  )
{
	unsigned int mode;
	unsigned int normal_sampling;
	unsigned int special_sampling;

	mode             = TC_boolean_u.TC_Boolean_Table.tc_mode;
	normal_sampling  = TC_boolean_u.TC_Boolean_Table.TC_NormalStorage_Sampling_Rate_Select;
	special_sampling = TC_gain_select_u.TC_gain_select_Table.TC_special_Sampling_rate_Select;

	if  ( mode == 1 )                                        // if normal mode of storage (from TC)
	{
		switch ( special_sampling )
		{
		case 0 : return Special_Sampling_Rate_A;
		case 1 : return Special_Sampling_Rate_B;
		case 2 : return Special_Sampling_Rate_C;
		case 3 : return Special_Sampling_Rate_D;
		}

	}
	else                                                      // if special mode of storage (from TC)
	{
		if ( normal_sampling )
		{
			return Normal_Sampling_Rate_A;
		}
		else
		{
			return Normal_Sampling_Rate_B;
		}
	}
	return 0;
}

// Frame Address Selection (based on Storage Telemetry Mode)
int Frame_Address_Select ( )
{
	unsigned int mode;
	unsigned int ST_Format_Selection;
	unsigned int frame_addr;

	mode = TC_boolean_u.TC_Boolean_Table.tc_mode;
	ST_Format_Selection = TC_gain_select_u.TC_gain_select_Table.TC_ST_Format_Selection;

	if( mode == 1 )                                                    // normal storage
	{
		switch( ST_Format_Selection )
		{
		case 0: frame_addr =  0;
				ST_special.ST_SP_Buffer.Sub_Frame = 0;
				break;
		case 1: frame_addr = Special_st_table_page1;
				ST_special.ST_SP_Buffer.Sub_Frame = 1;
				break;
		case 2: frame_addr = Special_st_table_page2;
		    	ST_special.ST_SP_Buffer.Sub_Frame = 2;
		    	break;
		case 3: frame_addr = Special_st_table_page3;
				ST_special.ST_SP_Buffer.Sub_Frame = 3;
				break;
		}
	}
	else															   // special storage
	{

		switch(ST_Format_Selection )
		{

		case 0: 	frame_addr = 0 	;
					ST_normal.ST_NM_Buffer.Sub_Frame = 0;
					break;
		case 1:		frame_addr = Normal_st_table_page1;
					ST_normal.ST_NM_Buffer.Sub_Frame = 1;
					break;
		case 2: 	frame_addr = Normal_st_table_page2;
					ST_normal.ST_NM_Buffer.Sub_Frame = 2;
					break;
		case 3: 	frame_addr = Normal_st_table_page3;
					ST_normal.ST_NM_Buffer.Sub_Frame = 3;
					break;
		}

	}
	return frame_addr;
}

unsigned int repeat_count = 0;

//fetching data from address and storing it in Destination_Buffer
void ST_Copy_Subframe(int frame_addr,unsigned int Sampling_Rate )
{
	struct Norm_ST_ADDR_TABLE *Normal_ST_Table;
	struct Norm_ST_ADDR_TABLE *Normal_ST_Table_Addr;
	struct Spec_ST_ADDR_TABLE *Special_ST_Table;
	struct Spec_ST_ADDR_TABLE *Special_ST_Table_Addr;

	unsigned char* Normal_ST_Source_Addr;
	unsigned char* Special_ST_Source_Addr;

	Normal_ST_Table_Addr    = Norm_ST_Table + frame_addr;
	Special_ST_Table_Addr   = Spec_ST_Table + frame_addr;

	Normal_ST_Source_Addr   = Normal_ST_Table_Addr->Addr_Field;                      //Initializing Source pointer
	Special_ST_Source_Addr  = Special_ST_Table_Addr->Addr_Field;                     //Initializing Source pointer

	// Special storage enable

	if(TC_boolean_u.TC_Boolean_Table.tc_mode == TRUE)
	{
		static int interval_cnt = 1;                         // initializing the interval variable
		if ( interval_cnt >= Sampling_Rate )                 // if interval exceeds sampling rate
		{
			interval_cnt = 0;
			while(Special_ST_Source_Addr != NULL)
			{
				for(long int j=0; j<=(Special_ST_Table_Addr->Length_Field -1); j++)
				{
					repeat_count++;
					*ST_Dest_Addr++ = *Special_ST_Source_Addr++;   				        //fetching data from source and storing in destination
				}

				Special_ST_Table_Addr++;   								            	//incrementing ST_Table_Addr

				Special_ST_Source_Addr = Special_ST_Table_Addr->Addr_Field;             //assigning Source pointer
			}
			if(repeat_count >= 255)
			{
				repeat_count = 0;
				ST_frame_count++;
				ST_normal.ST_NM_Buffer.frame_count = ST_frame_count;
				copy_frame( circuar_spec);
				ST_Dest_Addr = ST_source_Buffer;                                     //Initializing Destination pointer
			}

			if(ST_special.ST_SP_Buffer.Sub_Frame == 2)
			{
				frame_addr = Special_st_table_page3;
				ST_special.ST_SP_Buffer.Sub_Frame = 3;
				Special_ST_Table_Addr   = Spec_ST_Table + frame_addr;
				Special_ST_Source_Addr  = Special_ST_Table_Addr->Addr_Field;                     //Initializing Source pointer

				while(Special_ST_Source_Addr != NULL)
				{
					for(long int j=0; j<=(Special_ST_Table_Addr->Length_Field -1); j++)
					{
						repeat_count++;
						*ST_Dest_Addr++ = *Special_ST_Source_Addr++;   				        //fetching data from source and storing in destination
					}

					Special_ST_Table_Addr++;   								            	//incrementing ST_Table_Addr

					Special_ST_Source_Addr = Special_ST_Table_Addr->Addr_Field;             //assigning Source pointer

				}
				frame_addr = Special_st_table_page2;
				ST_special.ST_SP_Buffer.Sub_Frame = 2;
				if(repeat_count >= 255)
				{
					repeat_count = 0;
					ST_frame_count++;
					ST_normal.ST_NM_Buffer.frame_count = ST_frame_count;
					copy_frame( circuar_spec);
					ST_Dest_Addr = ST_source_Buffer;                                     //Initializing Destination pointer
				}

			}
		}
		interval_cnt++;

	}

	// Normal storage enable

	else
	{

		static int interval_cnt = 1;                         // initializing the interval variable
		if ( interval_cnt >= Sampling_Rate )                 // if interval exceeds sampling rate
		{
			interval_cnt = 0;
			while(Normal_ST_Source_Addr != NULL)
			{
				for(long int j=0; j<=(Normal_ST_Table_Addr->Length_Field -1); j++)
				{
					repeat_count++;
					*ST_Dest_Addr++ = *Normal_ST_Source_Addr++;   				       //fetching data from source and storing in destination
				}

				Normal_ST_Table_Addr++;   								               //incrementing ST_Table_Addr
				Normal_ST_Source_Addr = Normal_ST_Table_Addr->Addr_Field;              //assigning Source pointer
			}
		}
		interval_cnt++;
		if(repeat_count >= 255)
		{
			repeat_count = 0;
			ST_frame_count++;
			ST_normal.ST_NM_Buffer.frame_count = ST_frame_count;
			copy_frame( circuar_norm );
			ST_Dest_Addr = ST_source_Buffer;                                     //Initializing Destination pointer
		}

	}
// --------------------------TC_history logic------------------------------------------------------

//-----------------------------------------------------------------------------
	//Initializing ST_Table_Addr to point to the selected frame_addr
}


// Normal Storage Function


void rTCH_full_dump_cpy_buf()
{

	int i;
	count_tc_hist = 0;
	//------------- TC_history related initialization---------------------------------
	unsigned char* Tc_ST_Source_Addr;
	Tc_ST_Source_Addr = (unsigned char*)TC_storing_buffer;
	//--------------------------------------------------------------------------------

	ST_TC_header_Buffer.FrameSynch = 0xF9A42BB1;
	ST_TC_header_Buffer.Sat_ID = 0x74;
	ST_TC_header_Buffer.Real_Storage_TM= 1;
	ST_TC_header_Buffer.Norm_Spec_TC_hist_TM = 0x02;
	ST_TC_header_Buffer.filler = 0;
	ST_TC_header_Buffer.Sub_Frame = 0;
	ST_TC_header_Buffer.OBT = Major_Cycle_Count;
	ST_TC_header_Buffer.frame_count = ST_frame_count;

	for(i = 0; i < 3;i++)
	{
		TC_storing_buffer[i] = ST_TC_header_Buffer.data[i];
	}

	i = 3;
	while (count_tc_hist < 15)
	{
		count_tc_hist++;
		for(int j = 0;j < 4; j++)
		{
			TC_storing_buffer[i++] = REG32(TCH_read_full_ptr);
			TCH_read_full_ptr = TCH_read_full_ptr+4;

		}
	}
	ST_TCH_frame_count++;

		if (TCH_read_full_ptr >= &TC_hist_data[TC_HISTORY_MAX])
		{
			TCH_read_full_ptr = &TC_hist_data[0];
			TCH_full_dump_finish = 1;
			ST_TCH_frame_count = 0;
		}
}

void rTCH_dump_cpy_buf()
{
	int i;
	count_tc_hist = 0;
	//------------- TC_history related initialization---------------------------------
	unsigned char* Tc_ST_Source_Addr;
	Tc_ST_Source_Addr = (unsigned char*)TC_storing_buffer;
	//--------------------------------------------------------------------------------

	ST_TC_header_Buffer.FrameSynch = 0xF9A42BB1;
	ST_TC_header_Buffer.Sat_ID = 0x74;
	ST_TC_header_Buffer.Real_Storage_TM= 1;
	ST_TC_header_Buffer.Norm_Spec_TC_hist_TM = 0x02;
	ST_TC_header_Buffer.filler = 0;
	ST_TC_header_Buffer.Sub_Frame = 0;
	ST_TC_header_Buffer.OBT = Major_Cycle_Count;
	ST_TC_header_Buffer.frame_count = ST_frame_count;

	for(i = 0; i < 3;i++)
	{
		TC_storing_buffer[i] = ST_TC_header_Buffer.data[i];
	}

	i = 3;
	while (count_tc_hist < 15)
	{

		if (TCH_read_ptr < TC_hist_write_ptr)
		{

			for(int j = 0;j < 4; j++)
			{
				TC_storing_buffer[i++] = REG32(TCH_read_ptr);
				TCH_read_ptr = TCH_read_ptr+4;

				// TCH pointer dump
				if(TCH_read_ptr >= &TC_hist_data[TC_HISTORY_MAX])
					TCH_read_ptr = &TC_hist_data[0];

			}
		}
		else
		{
			TCH_dump_finish = 1;
			for(int j = 0;j < 4; j++)
			{
				TC_storing_buffer[i++] = 0;
			}
		}
		count_tc_hist++;

	}
	ST_TCH_frame_count++;
}

void TC_Hist_dumping()
{

	rt_tm_frame_finish = 0;
	unsigned long int tempdata;

	inter_TM_Status_Data = TM_STATUS_REGISTER;                                  // Read Status Register Data
	inter_TM_DP_Addr = (unsigned long int*)TM_BUFFER_BASE;                      // Assign DP Memory pointer

	//Write 64Bytes Data to FPGA Dual Port Memory
	if ((inter_TM_Status_Data & TM_DP_MEMORY_FULL) == TM_DP_MEMORY_EMPTY) // Check Dual Port Memory is Empty or Not
	{
		if ((TM_page_count % 4) != 0)
		 {
			for (inter_HAL_TM_Write_Word_Count = 0;
				 inter_HAL_TM_Write_Word_Count <= (TM_DP_MEMORY_SIZE-1);
				 inter_HAL_TM_Write_Word_Count++)
			{
				tempdata = ~(unsigned long int)*inter_TM_ST_TC_NS_Write_Source_Addr++; 		// Invert Data (On board having Inverter)
				TM_STATUS_REGISTER;
				*inter_TM_DP_Addr = tempdata;                                			 	// Invert Data (On board having Inverter)
				inter_TM_DP_Addr++;
			}

			inter_ST_TM_Minor_Cycle_Count++;                                             	//   Update TM Minor Cycle Count for next TM Minor Cycle Operation
			if (inter_ST_TM_Minor_Cycle_Count == 4)
			{
				inter_ST_TM_Minor_Cycle_Count = 0;
				inter_TM_Main_Buffer_Empty    = TRUE;
				inter_TM_ST_TC_NS_Write_Source_Addr = (unsigned short*)TC_storing_buffer;
				TM_page_count++;

				if (TM_page_count > 15)
				{
					TM_page_count = 0;
				}

				if (TC_boolean_u.TC_Boolean_Table.ST_dump_abort)
				{
					TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping = 0;
					TC_boolean_u.TC_Boolean_Table.ST_dump_abort = 0;
					real_tm_finish = 0;
					real_tm_wait = 1;
					if(TC_boolean_u.TC_Boolean_Table.TCH_dump_mode)            // TCH full_dump
					{
						TCH_read_full_ptr = &TC_hist_data[0];
					}
				}
				if (TCH_dump_finish == 1)
				{
					TCH_dump_finish = 0;
					TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping = 0;
					real_tm_finish = 0;
					real_tm_wait = 1;
				}
				else if(TCH_full_dump_finish == 1)
				{
					TCH_full_dump_finish = 0;
					TCH_read_full_ptr = &TC_hist_data[0];

					// pointer re-initialization after full_dump
					TCH_read_full_ptr = TC_hist_write_ptr;
					TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping = 0;
					real_tm_finish = 0;
					real_tm_wait = 1;
				}
				else
				{
					if(TC_boolean_u.TC_Boolean_Table.TCH_dump_mode)            // TCH full_dump
						rTCH_full_dump_cpy_buf();
					else
						rTCH_dump_cpy_buf();
				}

			}
		 }
		 else
		 {

			 rHAL_TM_Write();

			if (rt_tm_frame_finish && !(TCH_start_buffer_ready))
			{
				if(TC_boolean_u.TC_Boolean_Table.TCH_dump_mode)            // TCH full_dump
					rTCH_full_dump_cpy_buf();
				else
					rTCH_dump_cpy_buf();

				rt_tm_frame_finish = 0;
				TCH_start_buffer_ready = 1;
			}
		 }
	}
}


// copy frame function
unsigned int copy_frame_count;
int copy_frame(  unsigned int Circular)
{
	unsigned char* src_ptr; 		                            // Initializing a source pointer
	unsigned char* dst_ptr; 								    // initializing a destination pointer
	unsigned char* Source_end_addr;                             // initializing Source end address

	src_ptr = ST_source_Buffer;

	if(Circular == 1)
	{
		for( int k=0; k < ( sizeof( ST_source_Buffer )); k++ )
		{
			*write_str_ptr++ = *src_ptr++;	                            // copying values
			if( write_str_ptr == Dest_end_addr )                            // if dst_ptr reaches the end of storage area
			{
				write_str_ptr = Storage;
			}
		}
	}
	else if(Circular == 0)
	{
		if(write_str_ptr != Dest_end_addr)
		{
			for( int k=0; k< ( sizeof( ST_source_Buffer )); k++ )
			{
				*write_str_ptr++ = *src_ptr++;	                            // copying values
			}
		}
		else
		{
			ST_normal.ST_NM_Buffer.frame_count = 0;
		}
	}
	return 0;
}

unsigned char test_st = 0xaa;
unsigned char filler_byte1 = 0x00;
unsigned char filler_byte2 = 0x00;
unsigned char filler_byte3 = 0x00;
unsigned char test_st_b = 0xbb;
unsigned int count_plus;

 // Storage Telemetry frame table definition containing four normal storage frames
void Norm_ST_1_Table_Init()
{

	//Subframe 0
	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.FrameSynch;                //Copy address of Frame sync
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 10;                                                   //Frame sync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.frame_count;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_B_BODY[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = sizeof(ST_normal.ST_NM_Buffer.TM_B_BODY);//6 bytes
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_IMU_1_Temp;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_IMU_2_Temp;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_IMU_1_Diag_STS;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_IMU_2_Diag_STS;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Q_BODY[0] ;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_w_BODY[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_RW_Speed[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 16;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_MTR;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Last_seen_TC;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 8;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Q_EKF[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 16;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_w_EKF_Drift[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_B_DOT[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Pos_ECI[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Vel_ECI[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Orb_Elapsd_Time;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_SunSens_Roll_Error;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_SunSens_Pitch_Error;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_SunSens_Yaw_Error;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Q_Sunmagad[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_No_Of_Sat;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_S_BODY_Main[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_S_BODY_Red[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_battery_temp_1;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_battery_temp_2;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Thermistor_2;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Thermistor_3;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Thermistor_4;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Thermistor_5;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Q_Ref[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Q_Error[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 12;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.ST_output_latch;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 4;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Bus_voltage;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Bus_current;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_SA_current;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TC_rcvd_cntr2;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_AGC;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.ang_momtm[0];
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 6;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_Year;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_month;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_Day;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_hour;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_Min ;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_GPS.TM_UTC_Sec;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_GPS.TM_BIST_Info;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.TM_Tsince;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& ST_normal.ST_NM_Buffer.TM_wheel_index_ARCsum;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& ST_normal.ST_NM_Buffer.sunlit_eclipse_status;      // Copy address of Magnetic field in body frame
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& filler_byte1;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& filler_byte2;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& filler_byte3;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 1;
	Norm_ST_Table_Row_No++;


	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe 0
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 0;
	Norm_ST_Table_Row_No++;

	//Normal_st_table_page2 = Norm_ST_Table_Row_No;

	//frame_1
	Normal_st_table_page1 = Norm_ST_Table_Row_No;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.FrameSynch;                //Copy address of Frame sync
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 10;                                                   //Frame sync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.frame_count;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	for(int i = 0;i<244;i++)
	{

		Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& filler_byte1;
		Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
		Norm_ST_Table_Row_No++;
	}

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe 0
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 0;
	Norm_ST_Table_Row_No++;


	//frame_2
	Normal_st_table_page2 = Norm_ST_Table_Row_No;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.FrameSynch;                //Copy address of Frame sync
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 10;                                                   //Frame sync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.frame_count;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	for(int i = 0;i<244;i++)
	{

		Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& filler_byte1;
		Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
		Norm_ST_Table_Row_No++;
	}

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe 0
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 0;
	Norm_ST_Table_Row_No++;

	//frame_1
	Normal_st_table_page3 = Norm_ST_Table_Row_No;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.FrameSynch;                //Copy address of Frame sync
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 10;                                                   //Frame sync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Norm_ST_Table_Row_No++;

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_normal.ST_NM_Buffer.frame_count;
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
	Norm_ST_Table_Row_No++;

	for(int i = 0;i<244;i++)
	{

		Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = (unsigned char*)& filler_byte1;
		Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 2;
		Norm_ST_Table_Row_No++;
	}

	Norm_ST_Table[Norm_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe 0
	Norm_ST_Table[Norm_ST_Table_Row_No].Length_Field = 0;
	Norm_ST_Table_Row_No++;

}
void TC_ST_Table_init()
{

}

unsigned char test_st_sp   = 0x00;
unsigned char test_st_sp_b = 0xbb;
unsigned int test_st_sp_c  = 0xcc;
unsigned int test_st_sp_d  = 0xdd;
void Spec_ST_Table_Init()
{
	// SUB FRAME 0
	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.FrameSynch;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.frame_count;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 2;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Pos_ECI[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Vel_ECI[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_hour;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_Min;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_GPS.TM_UTC_Sec;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 2;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Tsince;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 2;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_S_BODY_Main[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_w_BODY[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Q_Sunmagad[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Q_Error[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_B_BODY[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 6;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_MTR;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.cntrl_torque[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_wheel_index_ARCsum;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_RW_Speed[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 16;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&test_st_sp;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&test_st_sp;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe_0
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 0;
	Spec_ST_Table_Row_No++;

	Special_st_table_page1 = Spec_ST_Table_Row_No;

	//frame _1
	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.FrameSynch;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.frame_count;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 2;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_w_BODY[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_B_BODY[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 6;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_S_BODY_Main[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 4;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_S_BODY_Main[2];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 4;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Q_Sunmagad[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Error_EKF[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Q_Ref[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_Q_EKF[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_B_EKF_Bias[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_w_EKF_Drift[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 12;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.TM_RW_Speed[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 16;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&test_st_sp;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&test_st_sp;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe_0
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 0;
	Spec_ST_Table_Row_No++;

	Special_st_table_page2 = Spec_ST_Table_Row_No;

	// frame_2

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.FrameSynch;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.frame_count;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 2;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.ST_TM_GPS_RCVD_DATA[0];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 244;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe_2
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 0;
	Spec_ST_Table_Row_No++;


	Special_st_table_page3 = Spec_ST_Table_Row_No;

	// frame_2

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.FrameSynch;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 10;//Framesync(4), SAT_ID(1), MAIN_FRAME:SUB_FRAME(1), OBT(4) Total: 10 Bytes
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.frame_count;
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 2;
	Spec_ST_Table_Row_No++;

	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&ST_special.ST_SP_Buffer.ST_TM_GPS_RCVD_DATA[244];
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 179;
	Spec_ST_Table_Row_No++;

	for(i=0;i<65;i++)
	{
		Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = (unsigned char*)&test_st_sp;
		Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 1;
		Spec_ST_Table_Row_No++;
	}
	Spec_ST_Table[Spec_ST_Table_Row_No].Addr_Field   = NULL;//End of Subframe_3
	Spec_ST_Table[Spec_ST_Table_Row_No].Length_Field = 0;
	Spec_ST_Table_Row_No++;


	//Special_st_table_page3 = Spec_ST_Table_Row_No;

}


void ST_DUMPING()
{
		unsigned long int tempdata;
		//static unsigned short* inter_TM_ST_NS_Write_Source_Addr = (unsigned short*)Storage;

		inter_TM_Status_Data = TM_STATUS_REGISTER;              // Read Status Register Data
		inter_TM_DP_Addr = (unsigned long int*)TM_BUFFER_BASE;  // Assign DP Memory pointer

		//Write 64Bytes Data to FPGA Dual Port Memory

		if ((inter_TM_Status_Data & TM_DP_MEMORY_FULL) == TM_DP_MEMORY_EMPTY) // Check Dual Port Memory is Empty or Not
		{
			if(!(read_str_ptr > (unsigned short*)write_str_ptr))
			{
					 if ((TM_page_count % 4) != 0)
					 {
							for (inter_HAL_TM_Write_Word_Count = 0;
								 inter_HAL_TM_Write_Word_Count <= (TM_DP_MEMORY_SIZE-1);
								 inter_HAL_TM_Write_Word_Count++)
							 {
								tempdata = ~(unsigned long int)*read_str_ptr++; 		// Invert Data (On board having Inverter)
								TM_STATUS_REGISTER;
								*inter_TM_DP_Addr = tempdata;                           // Invert Data (On board having Inverter)
								inter_TM_DP_Addr++;
							 }

							inter_ST_TM_Minor_Cycle_Count++;                            //   Update TM Minor Cycle Count for next TM Minor Cycle Operation

							if (inter_ST_TM_Minor_Cycle_Count == 4)
							{
								inter_ST_TM_Minor_Cycle_Count = 0;
								inter_TM_Main_Buffer_Empty    = TRUE;
								TM_page_count++;

								if (TM_page_count > 15)
								{
									TM_page_count = 0;

								}

								if (TC_boolean_u.TC_Boolean_Table.ST_dump_abort)
								{
									TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping = 0;
									TC_boolean_u.TC_Boolean_Table.ST_dump_abort = 0;
									real_tm_finish = 0;
									real_tm_wait = 1;
								}

								 if (read_str_ptr == (unsigned short*)Dest_end_addr)
								 {
									 read_str_ptr =(unsigned short*)Storage;
								 }
							}
					 }
					 else
						 rHAL_TM_Write();

			}
			else
			{
				 TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping = 0;
				 real_tm_finish = 0;
				 real_tm_wait = 1;
			}

		}

}

/****************************************************************
 *@function name ST_full_dump()
 *@return type   NONE
 *@Description   This function will dump full storage memory
 ****************************************************************
 */
void ST_full_dump()
{
		unsigned long int tempdata;
		static unsigned short* inter_TM_ST_NS_Write_Source_Addr = (unsigned short*)Storage;

		inter_TM_Status_Data = TM_STATUS_REGISTER;              // Read Status Register Data
		inter_TM_DP_Addr = (unsigned long int*)TM_BUFFER_BASE;  // Assign DP Memory pointer

		//Write 64Bytes Data to FPGA Dual Port Memory

		if ((inter_TM_Status_Data & TM_DP_MEMORY_FULL) == TM_DP_MEMORY_EMPTY) // Check Dual Port Memory is Empty or Not
		{

			 if ((TM_page_count % 4) != 0)
			 {
					for (inter_HAL_TM_Write_Word_Count = 0;
						 inter_HAL_TM_Write_Word_Count <= (TM_DP_MEMORY_SIZE-1);
						 inter_HAL_TM_Write_Word_Count++)
					{

						tempdata = ~(unsigned long int)*inter_TM_ST_NS_Write_Source_Addr++; 		// Invert Data (On board having Inverter)
						TM_STATUS_REGISTER;
						*inter_TM_DP_Addr = tempdata;                                			 	// Invert Data (On board having Inverter)
						inter_TM_DP_Addr++;
					}

					inter_ST_TM_Minor_Cycle_Count++;                                             	//   Update TM Minor Cycle Count for next TM Minor Cycle Operation
					if (inter_ST_TM_Minor_Cycle_Count == 4)
					{
						inter_ST_TM_Minor_Cycle_Count = 0;
						inter_TM_Main_Buffer_Empty    = TRUE;
						TM_page_count++;

						if (TM_page_count > 15)
						{
							TM_page_count = 0;

						}
						storage_page_count++;
						if(storage_page_count == ST_BUFFER_LIMIT_COUNT)
						{
							 storage_page_count = 0;
							 inter_TM_ST_NS_Write_Source_Addr = (unsigned short*)Storage;
							 TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping = 0;
							 real_tm_finish = 0;
							 real_tm_wait = 1;
						}

						if (TC_boolean_u.TC_Boolean_Table.ST_dump_abort)
						{
							inter_TM_ST_NS_Write_Source_Addr = (unsigned short*)Storage;
							TC_boolean_u.TC_Boolean_Table.Storage_TM_dumping = 0;
							TC_boolean_u.TC_Boolean_Table.ST_dump_abort = 0;
							real_tm_finish = 0;
							real_tm_wait = 1;
						}

					}
			 }
			 else
			 {

				 rHAL_TM_Write();


			 }

		}

}

