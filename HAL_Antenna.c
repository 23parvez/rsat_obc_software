#include "HAL_Antenna.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Global.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "TC_List.h"


SA1_status_t SA1_status = SA1_NOT_DEPLOYED;
SA2_status_t SA2_status = SA2_NOT_DEPLOYED;
SA1_status_t rHAL_SA1_Deploy_status_check();
SA2_status_t rHAL_SA2_Deploy_status_check();


/*void rHAL_Antenna_Deploy(unsigned int status)
{
	uint32 tempdata;
	if(status == Deploy)
	{
		Out_latch_5.ANTENNA_DEPLOY = 1;
		tempdata = Out_latch_5.data;
		IO_LATCH_REGISTER_5 = tempdata;
	}
	else if(status == Not_Deploy)
	{
		Out_latch_5.ANTENNA_DEPLOY = 0;
		tempdata = Out_latch_5.data;

		IO_LATCH_REGISTER_5 = tempdata;
	}
	else
	{
		//
	}
}*/

/*void rHAL_Antenna_Write(unsigned long int Command)
{
	inter_HAL_Antenna_Addr = ANTENNA_WRITE_DATA_ADDRESS_BASE;
	REG32(inter_HAL_Antenna_Addr) =  ((ANTENNA_SLAVE1 << 1) | I2C_Write);//7 bit Slave Address + R/W Bit
	inter_HAL_Antenna_Addr = inter_HAL_Antenna_Addr + 4;
	REG32(inter_HAL_Antenna_Addr) =  ((ANTENNA_SLAVE1 << 1) | I2C_Write);//7 bit Slave Address + R/W Bit
}*/


void antennaCommand1(void){                                                           //Antenna_ARM_command
	//aaaa=0xabcd;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000AD;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000c3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
    return;
}
void antennaCommand2(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;               //Antenna_Deploy_command
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000A1;
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
	REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
	REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
    return;
}
void antennaCommand3(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;              //Antenna_DISARM_command
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000AC;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
	return;
}

void antennaCommand4(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;             //Antenna_RESET_command
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000AA;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
	return;
}

void antennaCommand5(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;             //Antenna_deploy_with_override
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000BA;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
	return;
}

void antennaCommand6(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;             //Antenna_system_temp
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000C0;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
	return;
}

void antennaCommand7(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;             //Antenna_deploy_status_report
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000C3;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
	return;
}

void antennaCommand8(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;             //Antenna_deploy_activation_count
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000B0;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
	return;
}

void antennaCommand9(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;             //Antenna_deploy_activation_time
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000B4;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;
	return;
}


void rHAL_Antenna_Read()
{
  unsigned int Antenna_status_reg;
  unsigned short tempdata;
  Antenna_status_reg = REG32(ANTENNA_STATUS_REGISTER_1);
  if(Antenna_status_reg & ANTENNA_DATA_READY)
  {
	  tempdata =  (unsigned short)(REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE) & 0x0000FFFF);
	  Antenna_ACK = tempdata;
	  TM.Buffer.TM_Antenna_ACK = Antenna_ACK;

  }
}

//void rHAL_SA_Deploy()
//{
//	if(TC_HAL_SA_Deploy == HAL_SA_Deploy_Enable)
//	{
//		if(TC_HAL_SA_Heater_Timer >= c_HAL_SA_Heater_Timer_Min_Thrsld )
//		{
//			IO_Latch_Register_3_Data = (IO_Latch_Register_3_Data & ~(HAL_SA1_SA2_Deploy)); //Reset SA1_Deploy & SA2_Deploy
//			IO_LATCH_REGISTER_3 = IO_Latch_Register_3_Data;
//			TC_HAL_SA_Deploy = HAL_SA_Deploy_Disable;
//		}
//		else
//		{
//			IO_Latch_Register_3_Data = (IO_Latch_Register_3_Data | HAL_SA1_SA2_Deploy); //Set SA1_Deploy & SA2_Deploy
//			IO_LATCH_REGISTER_3 = IO_Latch_Register_3_Data;
//			TC_HAL_SA_Heater_Timer++;
//		}
//	}
//}

/*void rHAL_SA_Deploy(unsigned int SA_No, unsigned int TC_status)
{
	uint32 tempdata;
	if(SA_No == SA_MAIN)
	{
		if(TC_status == Deploy)
		{
			Out_latch_5.SA1_DEPLOY = 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA1_status = SA1_DEPLOY_CMD_RCVD;
			SA2_status = SA2_DEPLOY_CMD_RCVD;
			SA1_Deploy_cmd_rcvd_time = Major_Cycle_Count;
			SA2_Deploy_cmd_rcvd_time = Major_Cycle_Count;
			SA_Command_Type = SA_MAIN;
		}
		else
		{
			Out_latch_5.SA1_DEPLOY = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
		}

	}
	else if(SA_No == SA_RED)
	{
		if(TC_status == Deploy)
		{
			Out_latch_5.SPARE1_ON_OFF = 1;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA1_status = SA1_DEPLOY_CMD_RCVD;
			SA2_status = SA2_DEPLOY_CMD_RCVD;
			SA1_Deploy_cmd_rcvd_time = Major_Cycle_Count;
			SA2_Deploy_cmd_rcvd_time = Major_Cycle_Count;
			SA_Command_Type = SA_RED;
		}
		else
		{
			Out_latch_5.SPARE1_ON_OFF = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
		}
	}
	else
	{
		//do nothing
	}
}*/
unsigned int sa_test;
unsigned short sa_data;
void rHAL_SA_MAIN_Deploy_on()                  // -ve roll
{
	uint32 tempdata;
	sa_test = 1;

	Out_latch_5.SA1_DEPLOY = 1;
	tempdata = Out_latch_5.data;
	IO_LATCH_REGISTER_5;
	IO_LATCH_REGISTER_5 = tempdata;
	sa_data = IO_LATCH_REGISTER_5;
	SA1_status = SA1_DEPLOY_CMD_RCVD;
	SA2_status = SA2_DEPLOY_CMD_RCVD;
	SA1_Deploy_cmd_rcvd_time = Major_Cycle_Count;
	SA2_Deploy_cmd_rcvd_time = Major_Cycle_Count;
	SA_Command_Type = SA_MAIN;

}


void rHAL_SA_RED_Deploy_on()                      // +ve roll
{
	uint32 tempdata;

	Out_latch_5.SPARE1_ON_OFF = 1;
	tempdata = Out_latch_5.data;
	IO_LATCH_REGISTER_5;
	IO_LATCH_REGISTER_5 = tempdata;
	SA1_status = SA1_DEPLOY_CMD_RCVD;
	SA2_status = SA2_DEPLOY_CMD_RCVD;
	SA1_Deploy_cmd_rcvd_time = Major_Cycle_Count;
	SA2_Deploy_cmd_rcvd_time = Major_Cycle_Count;
	SA_Command_Type = SA_RED;

}
unsigned int panel_test ;
SA1_status_t rHAL_SA1_Deploy_status_check()
{
	SWHW_STATUS.SA_status_1 = (unsigned char)SA1_status;
	if(SA1_status == SA1_DEPLOY_CMD_RCVD)
	{
		if((IO_IN_LATCH_REGISTER_4 & SA1_DEPLOY_DEF_ST) == SA1_DEPLOY_ST)
		{
			SA1_status = SA1_DEPLOYED;
		}
		else if(TC_data_command_Table.SA_PanelHeater_Timeout <= (Major_Cycle_Count - SA1_Deploy_cmd_rcvd_time) )//testing for 10sec
		{
				SA1_status = SA1_HEATER_TIME_OUT;
		}
		else if(ADC_Buffer[40] >= TC_data_command_Table.TC_over_Heat )               // data_command to be added
		{
				SA1_status = SA1_OVER_HEAT;
		}
		else
		{
			//
		}

	}
	return SA1_status;
}

SA2_status_t rHAL_SA2_Deploy_status_check()
{
	SWHW_STATUS.SA_status_2 = (unsigned char)SA2_status;
	if(SA2_status == SA2_DEPLOY_CMD_RCVD)
	{
		if((IO_IN_LATCH_REGISTER_4 & SA2_DEPLOY_DEF_ST) == SA2_DEPLOY_ST)
		{
			SA2_status = SA2_DEPLOYED;
//			Out_latch_5.SA2_DEPLOY = 0;
//			IO_LATCH_REGISTER_5 = Out_latch_5.data;
		}
		else if(TC_data_command_Table.SA_PanelHeater_Timeout <= (Major_Cycle_Count - SA2_Deploy_cmd_rcvd_time) )//testing for 10sec
		{
			SA2_status = SA2_HEATER_TIME_OUT;
//			Out_latch_5.SA2_DEPLOY = 0;
//			IO_LATCH_REGISTER_5 = Out_latch_5.data;
		}
		else if(ADC_Buffer[41] >= TC_data_command_Table.TC_over_Heat )
		{
			SA2_status = SA2_OVER_HEAT;
		}
		else
		{
			//
		}

	}
	return SA2_status;
}

/*void rHAL_SA_Deploy_Status()
{
	uint32 tempdata;
	SA1_status_t SA1_Status;
	SA2_status_t SA2_Status;

	SA2_Status = rHAL_SA2_Deploy_status_check();
	SA1_Status = rHAL_SA1_Deploy_status_check();

	if(SA_Command_Type == SA_MAIN)//check main or red command received
	{
		if((SA1_Status == SA1_DEPLOYED) && (SA2_Status == SA2_DEPLOYED))
		{
			Out_latch_5.SA1_DEPLOY = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}
		else if((SA1_Status == SA1_HEATER_TIME_OUT) || (SA2_Status == SA2_HEATER_TIME_OUT))
		{
			Out_latch_5.SA1_DEPLOY = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}
		else if((SA1_status == SA1_OVER_HEAT) || (SA2_status == SA2_OVER_HEAT))
		{
			Out_latch_5.SA1_DEPLOY = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}
		else
		{
			//
		}
	}
	else if(SA_Command_Type == SA_RED)
	{
		if((SA1_Status == SA1_DEPLOYED) && (SA2_Status == SA2_DEPLOYED))
		{
			Out_latch_5.SPARE1_ON_OFF = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}
		else if((SA1_Status == SA1_HEATER_TIME_OUT) || (SA2_Status == SA2_HEATER_TIME_OUT))
		{
			Out_latch_5.SPARE1_ON_OFF = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}
		else if((SA1_Status == SA1_HEATER_TIME_OUT) || (SA2_Status == SA2_HEATER_TIME_OUT))
		{
			Out_latch_5.SA1_DEPLOY = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}
		else
		{
			//
		}
	}
	else
	{
		//DO NOTHING
	}
}*/


void rHAL_SA_Deploy_Status_new()
{
	uint32 tempdata;
	SA1_status_t SA1_Status;
	SA2_status_t SA2_Status;

	SA2_Status = rHAL_SA2_Deploy_status_check();
	SA1_Status = rHAL_SA1_Deploy_status_check();

	tempdata = IO_IN_LATCH_REGISTER_4;
	panel_deploy_sts= ((tempdata >> 8) & 0x0003);

	if(SA_Command_Type == SA_MAIN)//check main or red command received
	{
		if (((SA1_Status == SA1_DEPLOYED) && (SA2_Status == SA2_DEPLOYED)) ||
			(SA1_Status == SA1_HEATER_TIME_OUT) ||
			(SA2_Status == SA2_HEATER_TIME_OUT) ||
			(SA1_status == SA1_OVER_HEAT) ||
			(SA2_status == SA2_OVER_HEAT))
		{
			// Abort the deployment
			panel_test = 1;
			Out_latch_5.SA1_DEPLOY = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}

		else
		{
			panel_test = 2;
			// continue with deployment
		}
	}
	else if(SA_Command_Type == SA_RED)
	{
		if (((SA1_Status == SA1_DEPLOYED) && (SA2_Status == SA2_DEPLOYED)) ||
			(SA1_Status == SA1_HEATER_TIME_OUT) ||
			(SA2_Status == SA2_HEATER_TIME_OUT) ||
			(SA1_status == SA1_OVER_HEAT) ||
			(SA2_status == SA2_OVER_HEAT))
		{
			// Abort the deployment
			Out_latch_5.SPARE1_ON_OFF = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}

		else
		{
			// continue with deployment
		}
	}
	else
	{
		//DO NOTHING
	}
}
