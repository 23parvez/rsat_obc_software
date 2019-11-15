#include "HAL_RW.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Global.h"
#include "Telemetry.h"
#include "TM_Global_Buffer.h"
#include "adcs_Constants.h"
#include "TC_List.h"

void RW_Init(void)
{
	 //Initialize RW Configuration Registers
	 RW_1.RW_Configure_Register = RW1_CONFIG_REGISTER_1;
	 RW_2.RW_Configure_Register = RW2_CONFIG_REGISTER_1;
	 RW_3.RW_Configure_Register = RW3_CONFIG_REGISTER_1;
	 RW_4.RW_Configure_Register = RW4_CONFIG_REGISTER_1;
	 //Initialize RW Status Registers
	 RW_1.RW_Status_Register      = RW1_STATUS_REGISTER;
	 RW_2.RW_Status_Register      = RW2_STATUS_REGISTER;
	 RW_3.RW_Status_Register      = RW3_STATUS_REGISTER;
	 RW_4.RW_Status_Register      = RW4_STATUS_REGISTER;
	 //Initialize Buffer Base
	 RW_1.RW_Buffer_Register      = RW1_BUFFER_BASE;
	 RW_2.RW_Buffer_Register      = RW2_BUFFER_BASE;
	 RW_3.RW_Buffer_Register      = RW3_BUFFER_BASE;
	 RW_4.RW_Buffer_Register      = RW4_BUFFER_BASE;
}

unsigned short HAL_RW_CRC_Check(unsigned char* RW_Buffer_Addr,unsigned long int Byte_Count)
{
	//Local variables' declaration
	unsigned int inter_HAL_RW_CRC_Check_Count;
	unsigned int inter_HAL_RW_CRC;
	unsigned char inter_HAL_RW_CRC_Check_Byte;

	inter_HAL_RW_CRC = 0xFFFF;

	while(Byte_Count-- > 0)
	{
		inter_HAL_RW_CRC_Check_Byte = *RW_Buffer_Addr++;
		for(inter_HAL_RW_CRC_Check_Count = 0; inter_HAL_RW_CRC_Check_Count <= 7; inter_HAL_RW_CRC_Check_Count++)
		{
			inter_HAL_RW_CRC = (inter_HAL_RW_CRC>>1) ^ (((inter_HAL_RW_CRC_Check_Byte^inter_HAL_RW_CRC)& 0x01)? RW_POLY : 0 );
			inter_HAL_RW_CRC_Check_Byte  >>=1;
		}
	}
	return (short)inter_HAL_RW_CRC;
}

void rHAL_RW_TC_Write(struct HAL_RW_Data_Structure RW_No,
		              union  RW_TC_Command_u RW,
		              float  RW_Speed,
		              uint8  RW_ID)
{

	////Local variables' declaration
	int Byte_count;					//Counter in the loop
	unsigned int inter_RW_Status_Register;

	// Read the RW Status register

	inter_RW_Status_Register = (REG32(RW_No.RW_Status_Register) & 0x0000FFFF);

//	if((inter_RW_Status_Register & 0x00000001) == 0x00000000)
	if (!(inter_RW_Status_Register & RW_TX_BUF_BUSY))
	{
		RW.Start_Byte  = 0xC0;
		RW.Dest_Addr   = NSP_addr_table[RW_ID];
		RW.Source_Addr = 0x11;
		RW.Poll_Bit    = 0x1;
		RW.B           = 0x0;
		RW.ACK         = 0x1;
		RW.Cmd_Code    = 0x08;
		RW.Data_MSB    = 0x00;
		RW.Mode_Type   = 0x05;
		RW.Stop_Byte   = 0xC0;

		USIF_u.float_num = RW_Speed;	//Assigning of reaction wheel speed
		USIF_u.int_num   = reverse_order(USIF_u.int_num);		//Reversing the order of reaction wheel data (byte wise)
		RW.Data_Value    = USIF_u.float_num;		//Assigning reaction wheel speed data back to structure
		NOB_CRC_TC       = (((unsigned char*)&RW.CRC) - (&RW.Dest_Addr)) / sizeof(char);
		NOB_SFC_TC       = (((unsigned char*)&RW.Stop_Byte) - (&RW.Dest_Addr)) / sizeof(char);
		RW.CRC           = HAL_RW_CRC_Check(&RW.Dest_Addr, NOB_CRC_TC);		// Computation of CRC
		RW.CRC           = byte_swap(RW.CRC);

		Byte_count  = 0;
		TC_ptr      = &RW.Data[0];
		TC_data_ptr = &RW_Data_SlipFrame_TC[0];

		while(Byte_count < 13)
		{
			*TC_data_ptr++ = *TC_ptr++;
			Byte_count++;
		}

		rRW_SlipFrame_Check(&RW_No, &RW_Data_SlipFrame_TC[0], NOB_SFC_TC);		//Slip-frame check routine

	}
}

void rHAL_RW_TM_Write(struct HAL_RW_Data_Structure RW_No,
		              union RW_TM_Command_u RW,
		              uint8 RW_ID)
{
	//Local variables' declaration
	int Byte_count;					//Counter in the loop
	unsigned int inter_RW_Status_Register;
	//--------------------------------------------------------------

	// Read the Status Register of Reaction Wheel interface

	inter_RW_Status_Register = (REG32(RW_No.RW_Status_Register) & 0x0000FFFF);

//	if((inter_RW_Status_Register & 0x00000001) == 0x00000000)
	if (!(inter_RW_Status_Register & RW_TX_BUF_BUSY))
	{
		RW.Start_Byte  = 0xC0;
		RW.Dest_Addr   = NSP_addr_table[RW_ID];
		RW.Source_Addr = 0x11;
		RW.Poll_Bit    = 0x1;
		RW.B           = 0x0;
		RW.ACK         = 0x1;
		RW.Cmd_Code    = 0x07;
		RW.Mode_Type   = 0x05;
		RW.Stop_Byte   = 0xC0;

		NOB_CRC_TMC    = (((unsigned char*)&RW.CRC) - (&RW.Dest_Addr)) / sizeof(char);
		NOB_SFC_TMC    = (((unsigned char*)&RW.Stop_Byte) - (&RW.Dest_Addr)) / sizeof(char);
		RW.CRC         = HAL_RW_CRC_Check(&RW.Dest_Addr, NOB_CRC_TMC);		// Computation of CRC
		RW.CRC         = byte_swap(RW.CRC);

		Byte_count     = 0;
		TMC_ptr        = &RW.Data[0];
		TMC_data_ptr   = &RW_Data_SlipFrame_TMC[0];

		while (Byte_count < 8)
		{
			*TMC_data_ptr++ = *TMC_ptr++;
			Byte_count++;
		}

		rRW_SlipFrame_Check(&RW_No, &RW_Data_SlipFrame_TMC[0], NOB_SFC_TMC);		//Slip-frame check routine
	}
}


int rHAL_RW_TM_Read(struct HAL_RW_Data_Structure* RW_No, union RW_TM_Rcvd_u* RW_TM, int wheel_index)
{

	//
	// returns TRUE on the successful availability of wheel_speed data
	// returns FALSE on error condition
	//

	//Local Variables' declaration
	int inter_HAL_RW_count;
	int inter_Buffer_cpy_limit;

	int wheel_speed_data_available = FALSE;
	//-------------------------------------------------------------

	inter_HAL_RW_count = 0;

	//TODO: CHECK MAX LIMIT
	inter_HAL_RW_Status_Register = (REG32(RW_No->RW_Status_Register) & 0x0000FFFF);
	inter_HAL_RW_Read_Addr = RW_No->RW_Buffer_Register;

	if((inter_HAL_RW_Status_Register & 0x00000002))		//Check for Data Ready
	{
		inter_HAL_RW_Read_Limit  = (inter_HAL_RW_Status_Register & 0x0000FF00)>>8;  //Bytes to be read from RW Buffer
		inter_Buffer_cpy_limit = ((inter_HAL_RW_Read_Limit+1)>>1);

		while(inter_HAL_RW_count < inter_Buffer_cpy_limit)
		{
			temp_short = (unsigned short)(REG32(inter_HAL_RW_Read_Addr) & 0x0000FFFF);
			RW_Buffer_u_rx.data_16bit[inter_HAL_RW_count] = byte_swap(temp_short);
			inter_HAL_RW_count++;
			inter_HAL_RW_Read_Addr = inter_HAL_RW_Read_Addr + 4;
		}

		RW_TM_Raw_Data_ptr = &RW_Buffer_u_rx.data_8bit[0];
		RW_TM_ptr = &(RW_TM->Data[0]);
		RW_TM_ptr_init = RW_TM_ptr;//test


		inter_HAL_RW_count = 0;
		while(inter_HAL_RW_count < inter_HAL_RW_Read_Limit)
		{
			if(*RW_TM_Raw_Data_ptr == 0xDB)
			{
				RW_TM_Raw_Data_ptr++;
				inter_HAL_RW_count++;
				if(*RW_TM_Raw_Data_ptr == 0xDC)
				{
					RW_TM_Raw_Data_ptr++;
					inter_HAL_RW_count++;
					*RW_TM_ptr = 0xC0;
					RW_TM_ptr++;
				}

				else if(*RW_TM_Raw_Data_ptr == 0xDD)
				{
					RW_TM_Raw_Data_ptr++;
					inter_HAL_RW_count++;
					*RW_TM_ptr = 0xDB;
					RW_TM_ptr++;
				}

				else{}
			}

			else
			{
				*RW_TM_ptr = *RW_TM_Raw_Data_ptr;
				RW_TM_ptr++;
				RW_TM_Raw_Data_ptr++;
				inter_HAL_RW_count++;
			}
		}

		if(RW_TM->ACK == 1)
		{

			// ACK bit is high. Process the data

			NOB_CRC_TMC = (((unsigned char*)&RW_TM->CRC) - (&RW_TM->Dest_Addr)) / sizeof(char);
			crc_test = HAL_RW_CRC_Check(&RW_TM->Dest_Addr, NOB_CRC_TMC);		// Computation of CRC
			crc_test = byte_swap(crc_test);
			if(crc_test == RW_TM->CRC)
			{

				// Checksum test passed..
				// data is OK! Store it in Global wheel speed buffer

				USIF_u.float_num = RW_TM->Data_Value;
				USIF_u.int_num = reverse_order(USIF_u.int_num);
				RW_TM->Data_Value = USIF_u.float_num;

				wheel_speed_data_available  = TRUE;
				RW_Wheel_Speed[wheel_index] = RW_TM->Data_Value;


			}
			else
			{
				// checksum is not matched. Ignore the data
			}
		}
		else
		{

			// No_Acknowledgement bit is received. Ignore the data

		}

	}

	return wheel_speed_data_available;
}

void rRW_SlipFrame_Check(struct HAL_RW_Data_Structure* RW_No_Addr,
		                 uint8* inter_slipframe_data_addr,
		                 int NOB_Slipframe_Check)
{
	//Local variables' declaration

	static int inter_HAL_Write_Count;					//Local counter variable
	static uint32 inter_HAL_Config_Addr;			//Local variable that stores address of Config buffer
	static uint16* RW_Buffer_cpy_ptr;			//Pointer to the union to copy data to buffer
	static int32 NOB_Write;								//No of bytes that has to be written to the buffer
	static uint8* RW_TC_ptr;					//Pointer to the union (type casted to char*)
	uint32 swap_temp;

	//----------------------------------------------------------------------------

	inter_HAL_Config_Addr = (unsigned int)RW_No_Addr->RW_Configure_Register;
	inter_HAL_Write_Count = 0;
	RW_TC_ptr             = inter_slipframe_data_addr;

	RW_Write_ptr          =
	RW_Buffer_cpy_ptr     = (unsigned short*)(&RW_Buffer_u);

	*RW_Write_ptr++       = *RW_TC_ptr++;				//Copying the start byte

	while(inter_HAL_Write_Count < NOB_Slipframe_Check)
	{
		if(*RW_TC_ptr != 0xC0 && *RW_TC_ptr != 0xDB)
		{
			*RW_Write_ptr++ = *RW_TC_ptr++;
			inter_HAL_Write_Count++;
		}
		else
		{
			if(*RW_TC_ptr == 0xC0)
			{
				*RW_Write_ptr++ = 0xDB;
				*RW_Write_ptr++ = 0xDC;
				RW_TC_ptr++;
				inter_HAL_Write_Count++;
			}
			else if(*RW_TC_ptr == 0xDB)
			{
				*RW_Write_ptr++ = 0xDB;
				*RW_Write_ptr++ = 0xDD;
				RW_TC_ptr++;
				inter_HAL_Write_Count++;
			}
			else{}
		}
	}
	*RW_Write_ptr++ = *RW_TC_ptr++;				//Copying of stop byte

	NOB_Write = RW_Write_ptr - (unsigned char*)&RW_Buffer_u;

	//since FPGA require swapped data
	for(inter_HAL_Write_Count=0;inter_HAL_Write_Count<((NOB_Write + 1)>>1);inter_HAL_Write_Count++)
	{
		swap_temp = byte_swap(*RW_Buffer_cpy_ptr);
		(*RW_Buffer_cpy_ptr++) = swap_temp;
	}

	rHAL_RW_ConfigBuffer_Write(RW_No_Addr, RW_Buffer_u.data_16bit, NOB_Write);

}

void rHAL_RW_ConfigBuffer_Write(struct HAL_RW_Data_Structure* RW_No_Addr,
		                        uint16* inter_Buffer_cpy_addr,
		                        int inter_NOB_Write)
{
	int i;
	uint32 Config_Buffer_Addr, tempdata;

	Config_Buffer_Addr = RW_No_Addr->RW_Configure_Register;

	for (i = 0; i < ((inter_NOB_Write+1) << 1); i++)
	{

		tempdata = *inter_Buffer_cpy_addr++;
		REG32(Config_Buffer_Addr);
		REG32(Config_Buffer_Addr) = tempdata;
		Config_Buffer_Addr = Config_Buffer_Addr + 0x00000004;
	}

	//Set Configure Enable Bit and update number of bytes

	REG32(RW_No_Addr->RW_Status_Register);
	REG32(RW_No_Addr->RW_Status_Register)  = (((inter_NOB_Write & 0x000000FF)<<8) | 0x00000001);
}

void rRW_Data_Write(void)
{

	// set the wheel speeds to commmand speeds

	RWS[0] = TC_data_command_Table.RW1_Speed;
    RWS[1] = TC_data_command_Table.RW2_Speed;
    RWS[2] = TC_data_command_Table.RW3_Speed;
    RWS[3] = TC_data_command_Table.RW4_Speed;

    if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_1_speed_enable)
	{
		rHAL_RW_TC_Write(RW_1, RW_TC, RWS[0], RWHEEL0);	// Command Wheel Speed to RW1
	}
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_2_speed_enable)
	{
		rHAL_RW_TC_Write(RW_2, RW_TC, RWS[1], RWHEEL1);					// Command Wheel Speed to RW1
	}
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_3_speed_enable )
	{
		rHAL_RW_TC_Write(RW_3, RW_TC, RWS[2], RWHEEL2); 					// Command Wheel Speed to RW1
	}
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_4_speed_enable )
	{
		rHAL_RW_TC_Write(RW_4, RW_TC, RWS[3], RWHEEL3); 					// Command Wheel Speed to RW1
	}
}

void rRW_Data_Request(void)
{
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_1_speed_enable )
	{
		rHAL_RW_TM_Write(RW_1, RW_TMC, RWHEEL0);							// Command to request for RW1 speed
	}
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_2_speed_enable )
    {
		rHAL_RW_TM_Write(RW_2, RW_TMC, RWHEEL1);							// Command to request for RW1 speed
    }
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_3_speed_enable )
	{
		rHAL_RW_TM_Write(RW_3, RW_TMC, RWHEEL2);							// Command to request for RW1 speed
	}
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_4_speed_enable )
    {
		rHAL_RW_TM_Write(RW_4, RW_TMC, RWHEEL3);							// Command to request for RW1 speed
    }
}

void rRW_Data_Read()
{
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_1_speed_enable )
	  	  	{
				if (rHAL_RW_TM_Read(&RW_1, &RW_TM, RWHEEL0))
				{
					TM.Buffer.TM_RW_Speed[RWHEEL0] = (unsigned int)(RW_Wheel_Speed[RWHEEL0]/(c_TM_RW_Resol));

				}

	  	  	}
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_2_speed_enable )
	        {
				if (rHAL_RW_TM_Read(&RW_2, &RW_TM, RWHEEL1))
				{
					TM.Buffer.TM_RW_Speed[RWHEEL1] = (unsigned int)(RW_Wheel_Speed[RWHEEL1]/(c_TM_RW_Resol));

				}
	        }
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_3_speed_enable )
	        {
				if (rHAL_RW_TM_Read(&RW_3, &RW_TM, RWHEEL2))
				{
					TM.Buffer.TM_RW_Speed[RWHEEL2] = (unsigned int)(RW_Wheel_Speed[RWHEEL2]/(c_TM_RW_Resol));

				}
	        }
	if(TC_boolean_u.TC_Boolean_Table.Reaction_wheel_4_speed_enable )
	        {
				if (rHAL_RW_TM_Read(&RW_4, &RW_TM, RWHEEL3))
				{
					TM.Buffer.TM_RW_Speed[RWHEEL3] = (unsigned int)(RW_Wheel_Speed[RWHEEL3]/(c_TM_RW_Resol));

				}
	        }

}


//void rRW_Ping(struct HAL_RW_Data_Structure* RW_No)
//{
//	//----------------Declaration of Local variables------------------------
//	unsigned long int inter_RW_Status_Register;
//	int i_rw;
//	int NOB_Write_rw;
//	//----------------------------------------------------------------------
//
//	inter_RW_Status_Register = (REG32(RW_No->RW_Status_Register) & 0x0000FFFF);
//	NOB_Write_rw = 11; 	//Number of bytes to be transfered to RW for Ping
//
//	for(i_rw=0;i_rw<=5;i_rw++)
//	{
//		temp_ping_rw = byte_swap(RW_Ping_cmd[i_rw]);
//		RW_Ping_cmd_internal[i_rw] = temp_ping_rw;
//	}
//
//	if((inter_RW_Status_Register & 0x00000001) == 0x00000000)
//	{
//		rHAL_RW_ConfigBuffer_Write(RW_No,RW_Ping_cmd_internal,NOB_Write_rw);
//	}
//}


void rRW_init_cmd(struct HAL_RW_Data_Structure RW_No, unsigned char RW_ID)
{
	//----------------Declaration of Local variables------------------------
	int Byte_count;					//Counter in the loop
	unsigned long int inter_RW_Status_Register;
//	int i_rw;
	int NOB_Write_rw;
	//----------------------------------------------------------------------

	inter_RW_Status_Register = (REG32(RW_No.RW_Status_Register) & 0x0000FFFF);
	NOB_Write_rw = 11; 	//Number of bytes to be transfered to RW for Ping

	/*********************** Added on 12/10/19 ***********************/
	rw_init.Start_Byte = 0xC0; rw_init.Dest_Addr = NSP_addr_table[RW_ID];
	rw_init.Source_Addr =0x11; rw_init.Poll_Bit = 0x1; rw_init.B = 0x0;
	rw_init.ACK = 0x1; rw_init.Cmd_Code = 0x01; rw_init.Data_value = 0x00100000;
	rw_init.Stop_Byte = 0xC0;

	NOB_CRC_Init = (((unsigned char*)&rw_init.CRC) - (&rw_init.Dest_Addr)) / sizeof(char);
	NOB_SFC_Init = (((unsigned char*)&rw_init.Stop_Byte) - (&rw_init.Dest_Addr)) / sizeof(char);

	rw_init.CRC = HAL_RW_CRC_Check(&rw_init.Dest_Addr, NOB_CRC_Init);		// Computation of CRC
	rw_init.CRC = byte_swap(rw_init.CRC);

	Byte_count = 0;
	Init_ptr = rw_init.Data;
	Init_data_ptr = &RW_Data_SlipFrame_Init[0];

	while(Byte_count < 11)
	{
		*Init_data_ptr++ = *Init_ptr++;
		Byte_count++;
	}

	rRW_SlipFrame_Check(&RW_No,&RW_Data_SlipFrame_Init[0],NOB_SFC_Init);		//Slip-frame check routine
}
void rRW_Ping_TC1()
{
	rRW_init_cmd(RW_1, 0);
}

void rRW_Ping_TC2()
{
	rRW_init_cmd(RW_2, 1);
}

void rRW_Ping_TC3()
{
	rRW_init_cmd(RW_3, 2);
}

void rRW_Ping_TC4()
{
	rRW_init_cmd(RW_4, 3);
}

void rHAL_RW_POWER(unsigned long int RW_No,unsigned long int RW_Status)
{
	unsigned short tempdata;
	if(RW_No == RW1)
	{
		if(RW_Status == ON)
		{
			Out_Latch_3.RW1_ON_OFF = 1;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}

		else if(RW_Status == OFF)
		{
			Out_Latch_3.RW1_ON_OFF = 0;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}
	}

	else if(RW_No == RW2)
	{
		if(RW_Status == ON)
		{
			Out_Latch_3.RW2_ON_OFF = 1;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}

		else if(RW_Status == OFF)
		{
			Out_Latch_3.RW2_ON_OFF = 0;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}
	}

	else if(RW_No == RW3)
	{
		if(RW_Status == ON)
		{
			Out_Latch_3.RW3_ON_OFF = 1;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}

		else if(RW_Status == OFF)
		{
			Out_Latch_3.RW3_ON_OFF = 0;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}
	}

	else if(RW_No == RW4)
	{
		if(RW_Status == ON)
		{
			Out_Latch_3.RW4_ON_OFF = 1;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}

		else if(RW_Status == OFF)
		{
			Out_Latch_3.RW4_ON_OFF = 0;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}
	}

	else
	{
		//
	}
}





