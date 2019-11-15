#include "HAL_Payload.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Global.h"
#include "Telemetry.h"
#include "TM_Global_Buffer.h"

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
void S_band_tx_on_off(unsigned int s_status)
{
	if(s_status == s_band_on)
	{
		//GPIO_pins.PIO_5 = 1;
		IODAT = GPIO_pins.data;
	}
	else if(s_status == s_band_off)
	{
		//GPIO_pins.PIO_5 = 0;
		IODAT = GPIO_pins.data;
	}
	else
	{
		//
	}

}

unsigned short Payload_rcv_data;
void rHAL_PL_STS_Check()
{
	//REG32(0x20007000) = 0x0000FFFF;
	REG32(PAYLOAD_BUFFER_ADDRESS) =  0x00005355;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000101;//0000_0001_0000_0001
}

void PL_TM()
{
	if((PAYLOAD_STATUS1_ADDRESS & Payload_DATA_READY) == 0x00000002)
	   {
		Payload_rcv_data =  REG32(PAYLOAD_BUFFER_ADDRESS);
		TM.Buffer.Payload_TM = Payload_rcv_data;
	   }
	   else
	   {
	    //
	   }
}
void rHAL_PL_CMD_ACQ()
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00005755;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00000100;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00000101;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00000101;
	REG32(PL_CONFIG_Addr_Ptr++) = 0x00001F01;
	REG32(PL_CONFIG_Addr_Ptr) 	= 0x00000002;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000581;//0000_0101_1000_0001
}

void rHAL_PL_CMD_HLT()
{
	//REG32(0x20007000) = 0x0000FFFF;
	REG32(PAYLOAD_BUFFER_ADDRESS) = 0x00004855;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000004;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000101;//0000_0001_0000_0001
}

void rHAL_PL_DIAG()
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x00004455;
	REG32(PL_CONFIG_Addr_Ptr) 	   = 0x00000001;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000181;//0000_0001_1000_0001
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
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x00005255;
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x00000300;
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x0000FF07;
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x0000FFFF;
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x00001FFF;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000501;//0000_0110_0000_0001
}

void rHAL_PL_DIAG_SDCARD(void)
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x00004455;
	REG32(PL_CONFIG_Addr_Ptr) 	   = 0x00000002;
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x00000002;
	REG32(PAYLOAD_STATUS1_ADDRESS) = 0x00000181;//0000_0001_1000_0001
}

void rHAL_PL_DIAG_FRAM(void)
{
	//REG32(0x20007000) = 0x0000FFFF;
	PL_CONFIG_Addr_Ptr =(unsigned long int*)PAYLOAD_BUFFER_ADDRESS;
	REG32(PL_CONFIG_Addr_Ptr++)    = 0x00004455;
	REG32(PL_CONFIG_Addr_Ptr) 	   = 0x00000003;
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
