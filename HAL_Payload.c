#include "HAL_Payload.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Global.h"
#include "Telemetry.h"
#include "TM_Global_Buffer.h"
#include "adcs_VarDeclarations.h"

void rHAL_PL_Power(unsigned int Payload_No,unsigned int status)
{
	unsigned short tempdata;
	if(Payload_No == Payload_1)
	{
		if(status == ON)
		{
			Out_Latch_3.PL1_ON_OFF = 1;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}
		else if(status == OFF)
		{
			//bbb=0xabcd;
			Out_Latch_3.PL1_ON_OFF = 0;
			tempdata = Out_Latch_3.data;
			IO_LATCH_REGISTER_3;
			IO_LATCH_REGISTER_3 = tempdata;
		}
	}
	else if(Payload_No == Payload_2)
	{
		if(status == ON)
				{
					Out_Latch_3.PL2_ON_OFF = 1;
					tempdata = Out_Latch_3.data;
					IO_LATCH_REGISTER_3;
					IO_LATCH_REGISTER_3 = tempdata;
				}
				else if(status == OFF)
				{
					Out_Latch_3.PL2_ON_OFF = 0;
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
void rHAL_X_Tx_ON_OFF(unsigned int x_status)
{
	unsigned short tempdata;
	if(x_status == X_Tx_ON)
	{
		Out_Latch_2.X_Tx_ON_OFF = 1;
		tempdata =  Out_Latch_2.data;
		IO_LATCH_REGISTER_2;
		IO_LATCH_REGISTER_2 = tempdata;
		Out_Latch_3.TM_DS_EN = 1;
		tempdata = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3 = tempdata;
	}
	else if(x_status == X_Tx_OFF)
	{
		Out_Latch_2.X_Tx_ON_OFF = 0;
		tempdata =  Out_Latch_2.data;
		IO_LATCH_REGISTER_2;
		IO_LATCH_REGISTER_2 = tempdata;
		Out_Latch_3.TM_DS_EN = 0;
		tempdata = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3 = tempdata;
	}
	else
	{
		//
	}

}
//void S_band_tx_on_off(unsigned int s_status)
//{
//	if(s_status == s_band_on)
//	{
//		//Out_Latch_2.RF_Tx_ON_OFF = 1;
//		IO_LATCH_REGISTER_2 = Out_Latch_2.data;
//	}
//	else if(s_status == s_band_off)
//	{
//		Out_Latch_2.RF_Tx_ON_OFF = 0;
//		IO_LATCH_REGISTER_2 = Out_Latch_2.data;
//	}
//	else
//	{
//		//
//	}
//
//}

void S_band_tx_on_off(unsigned int s_status)
{
	if(s_status == s_band_on)
	{
		GPIO_pins.PIO_5 = 1;
		IODAT = GPIO_pins.data;
	}
	else if(s_status == s_band_off)
	{
		GPIO_pins.PIO_5 = 0;
		IODAT = GPIO_pins.data;
	}
	else
	{
		//
	}

}
/************************************************************
 *@function name PL_TM
 *@return type   NONE
 *@Description   This function receives the payload data and
 *				 updates it in to telemetry frame.
 ************************************************************
 */

void rHAL_PL_STS_Check()
{
	//REG32(0x20007000) = 0x0000FFFF;
	REG32(PAYLOAD_BUFFER_ADDRESS) = 0x00005355;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000001;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000101;//0000_0001_0000_0001
}

void rHAL_PL_CMD_ACQ() 	// PL_CMD_DEBUG
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
//	REG32(PL_CONFIG_Addr_Ptr++) = 0x00005755;
//	REG32(PL_CONFIG_Addr_Ptr++) = 0x00000100;
//	REG32(PL_CONFIG_Addr_Ptr++) = 0x00000101;
//	REG32(PL_CONFIG_Addr_Ptr++) = 0x00000101;
//	REG32(PL_CONFIG_Addr_Ptr++) = 0x00001F01;
//	REG32(PL_CONFIG_Addr_Ptr) 	= 0x00000002;

	/******************************************************/
	/***********************PL_CMD_Debug******************/
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00004555;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x0000A406;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x000000D2;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x0000FFFF;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x0000FFFF;
//	REG32(PL_CONFIG_Addr_Ptr) 	= 0x00000002;
	/*****************************************************/

	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000501;//0000_0101_1000_0001
}

void rHAL_PL_CMD_HLT()
{
	//REG32(0x20007000) = 0x0000FFFF;
	REG32(PAYLOAD_BUFFER_ADDRESS) = 0x00004855;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x0000000E;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000101;//0000_0001_0000_0001
}

uint16 temp_short1, temp_short2;
void PL_TM_read()
{
	//uint16 temp_short1, temp_short2;
	 Payload_status_1_data = REG32(PAYLOAD_STATUS1_ADDRESS);
	if((Payload_status_1_data & Payload_DATA_READY) == 0x00000002)
	{
		// Read Payload status buffer to the telemtry
		pl_buffer_len = PL_TM_BUF_MAX - 1;
		pl_buffer_addr = (uint32*)PAYLOAD_BUFFER_ADDRESS;
		for(PL_Addr_count = 0; PL_Addr_count <= pl_buffer_len; PL_Addr_count++)
		{
			temp_short1 = (unsigned short)(REG32(pl_buffer_addr++) & 0x0000FFFF);
			temp_short2 = byte_swap(temp_short1);
			TM.Buffer.Payload_TM[PL_Addr_count] = temp_short2;
		}
	}
}

void rHAL_PL_DIAG()			// PL_X_TX
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
//	REG32(PL_CONFIG_Addr_Ptr++)  = 0x00004455;
//	REG32(PL_CONFIG_Addr_Ptr) 	 = 0x00000001;
	/*********************************************/
	/****************X_tx************************/
	REG32(PL_CONFIG_Addr_Ptr++)      = 0x00005255;
	REG32(PL_CONFIG_Addr_Ptr++) 	 = 0x00000a09;
	REG32(PL_CONFIG_Addr_Ptr++) 	 = 0x00000c0b;
	REG32(PL_CONFIG_Addr_Ptr++) 	 = 0x0000ffff;
	REG32(PL_CONFIG_Addr_Ptr++) 	 = 0x000004ff;
	REG32(PL_CONFIG_Addr_Ptr) 	     = 0x00000011;
	/********************************************/
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000581;//0000_0101_1000_0001
}

void rHAL_PL_CMD_OFF()
{
	//REG32(0x20007000) = 0x0000FFFF;
	REG32(PAYLOAD_BUFFER_ADDRESS) = 0x00004655;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000101;//0000_0001_0000_0001
}

void rHAL_PL_CMD_ON()
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00005255;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00000300;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x0000FF07;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x0000FFFF;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00001FFF;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000501;//0000_0110_0000_0001
}

void rHAL_PL_DIAG_SDCARD(void)
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++)  = 0x00004455;
	REG32(PL_CONFIG_Addr_Ptr) 	 = 0x00000002;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000181;//0000_0001_1000_0001
}

void rHAL_PL_DIAG_FRAM(void)
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++)  = 0x00004455;
	REG32(PL_CONFIG_Addr_Ptr) 	 = 0x00000003;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000181;//0000_0001_1000_0001
}

void rHAL_PL_DEBUG(void)
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++)  = 0x00004555;
	REG32(PL_CONFIG_Addr_Ptr++)  = PL_Debug_Data[0];
	REG32(PL_CONFIG_Addr_Ptr++)  = PL_Debug_Data[1];
	REG32(PL_CONFIG_Addr_Ptr++)  = PL_Debug_Data[2];
	REG32(PL_CONFIG_Addr_Ptr)    = PL_Debug_Data[3];
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000501;//0000_0101_0000_0001
}


unsigned int hils_datas[63];
void rHILS_payload(union HILS_test* HILS_packets)
{

	int i;
	int temp;
	Hils_ptr = &(HILS_packets->HILS_data_16bit[0]);
	//hils_ptr_sh = &Hils_ptr;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	for(i = 0 ; i<= 31 ; i++)
	{

		//hils_datas[i] = *Hils_ptr++;
		temp =  *Hils_ptr++;
		REG32(PL_CONFIG_Addr_Ptr++) = temp;
		//REG32(PL_CONFIG_Addr_Ptr++) = 0x0000abcd;
		REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000001;
		REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00004001;

	}

}
//void rHAL_PL_TEST(void)
//{
//		TM.Buffer.Debug_Data[0] = PL_Debug_Data[0];
//		TM.Buffer.Debug_Data[1] = PL_Debug_Data[1];
//		TM.Buffer.Debug_Data[2] = PL_Debug_Data[2];
//		TM.Buffer.Debug_Data[3] = PL_Debug_Data[3];
//		if(PL_Exe_Flag ==0)
//		{
//			if((REG32(PAYLOAD_STATUS1_ADDRESS)&0x00000002) == 0x00000002){
//				if(TC_command == 0x9A){
//				TM.Buffer.TM_PAYLOAD_ACK[0] = (short)((REG32(PAYLOAD_BUFFER_ADDRESS)&0x000000FF) <<8) | (short)((REG32(PAYLOAD_BUFFER_ADDRESS)&0x0000FF00) >>8) ;
//				TM.Buffer.TM_PAYLOAD_ACK[1] = (short)((REG32(PAYLOAD_BUFFER_ADDRESS + 4)&0x000000FF) <<8) | (short)((REG32(PAYLOAD_BUFFER_ADDRESS + 4)&0x0000FF00) >>8);
//				}
//				else{
//				TM.Buffer.TM_PAYLOAD_ACK[0] = (short)((REG32(PAYLOAD_BUFFER_ADDRESS)&0x000000FF) <<8) | (short)((REG32(PAYLOAD_BUFFER_ADDRESS)&0x0000FF00) >>8) ;
//				}
//
//					REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000000;
//			}
//		}
//		if(Enable_PL_TM_flag == TRUE)
//		{
//			inter_PL_TM_Source_Addr = (unsigned short*)TM_Buffer;
//			if(PL_TM_write_Flag ==0)
//			{
//				inter_PL_TM_Dest_Addr=(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
//				REG32(0x20007000) = 0x0000FFFF;
//				*inter_PL_TM_Dest_Addr++ = 0x00005055;//0x00005055;
//				for(pl_i=0;pl_i<=127;pl_i++)
//				{
//					*inter_PL_TM_Dest_Addr++ = (unsigned long int)*inter_PL_TM_Source_Addr++;
//				}
//				REG32(PAYLOAD_STATUS2_ADDRESS) = 2;
//				REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00008101;//1000_0001_0000_0001
//
//			}
//			else
//			{
//				if((short)REG32(PAYLOAD_STATUS1_ADDRESS) & 0x0002 == 0x0002)
//				{
//					REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000000;
//					if((short)REG32(PAYLOAD_BUFFER_ADDRESS)==0x0FAA)
//					{
//						inter_PL_TM_Dest_Addr=(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
//						REG32(0x20007000) = 0x0000FFFF;
//						*inter_PL_TM_Dest_Addr++ = 0x00005055;//0x00005055;
//						for(pl_i=0;pl_i<=127;pl_i++)
//						{
//							*inter_PL_TM_Dest_Addr++ = (unsigned long int)*inter_PL_TM_Source_Addr++;
//						}
//						REG32(PAYLOAD_STATUS2_ADDRESS) = 2;
//						REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00008101;//1000_0001_0000_0001
//
//					}
//					else
//					{
//						//
//					}
//				}
//				else
//				{
//					//
//				}
//				PL_TM_write_Flag =1;
//			}
//		}
//}
