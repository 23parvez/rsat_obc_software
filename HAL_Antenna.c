#include "HAL_Antenna.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Global.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "TC_List.h"

SA1_status_t SA1_status = SA1_NOT_DEPLOYED;
SA2_status_t SA2_status = SA2_NOT_DEPLOYED;
SA1_status_t rHAL_SA1_Deploy_status_check(void);
SA2_status_t rHAL_SA2_Deploy_status_check(void);

void antennaCommand1(void)
{                                                                             /* Antenna_ARM_command */
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000AD;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000c3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006c01;
    return;
}
void antennaCommand2(void)
{
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;       /* Antenna_Deploy_command */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000A1;
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
	REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
	REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006c01;
    return;
}
void antennaCommand3(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;       /* Antenna_DISARM_command */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000AC;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006c01;
	return;
}

void antennaCommand4(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;       /* Antenna_RESET_command */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000AA;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006c01;
	return;
}

void antennaCommand5(void){
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;       /* Antenna_deploy_with_override */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000BA;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006c01;
	return;
}

void antennaCommand6(void){
	Antenna_ACKf = 1;
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;       /* Antenna_system_temp */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000C0;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;       /* ACK */

	return;
}

void antennaCommand7(void){
	Antenna_ACKf = 2;
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;       /* Antenna_deploy_status_report */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000C3;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006c03;       /* ACK */
	return;
}

void antennaCommand8(void){
	Antenna_ACKf = 3;
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;       /* Antenna_deploy_activation_count */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000B0;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;       /* ACK */
	return;
}

void antennaCommand9(void){
	Antenna_ACKf = 4;
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE)               = 0x00000062;        /* Antenna_deploy_activation_time */
	REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000004 ) = 0x000000B4;
    REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE + 0x00000008 ) = 0x000000C3;
    REG32(ANTENNA_SLAVE_ADDRESS)                         = 0x00000063;
    REG32(ANTENNA_STATUS_REGISTER_2)                     = 0x00006803;        /* ACK */
	return;
}


void rHAL_Antenna_Read(void)
{
  unsigned int Antenna_status_reg;
  unsigned char tempdata2,tempdata1;
  unsigned short antenna_temp,antenna_temp1;

  Antenna_status_reg = REG32(ANTENNA_STATUS_REGISTER_1);

	if (Antenna_status_reg & ANTENNA_DATA_READY)
	{
	  tempdata2 =  (unsigned char)(REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE) & 0x000000FF);
	  antenna_temp = (unsigned short)(REG32(ANTENNA_WRITE_DATA_ADDRESS_BASE+4) & 0x000000FF);

	  antenna_temp = (antenna_temp<<8);
	  antenna_temp1 = ((antenna_temp | tempdata2) & 0xFFFF);
	  Antenna_ACK = antenna_temp1;
	  Antenna_data_raedy = 1;
	}

}

void antenna_TM(void)
{
	if(Antenna_data_raedy)
	{
		switch(Antenna_ACKf)
		{
		case 1:TM.Buffer.TM_Antenna_temp_ACK = Antenna_ACK;
			break;
		case 2:TM.Buffer.Antenna_deploy_sts_ACK = Antenna_ACK;
			break;
		case 3:TM.Buffer.Antenna_deploy_act_count_ACK = Antenna_ACK;
			break;
		case 4:TM.Buffer.Antenna_deploy_act_time_ACK = Antenna_ACK;
			break;
		default:
			break;
		}
		Antenna_data_raedy = 0;
	}
	else
	{
		Antenna_data_raedy = 0;
	}
}

unsigned short sa_data;
void rHAL_SA_MAIN_Deploy_on(void)                  /* -ve roll */
{
	uint32 tempdata;

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


void rHAL_SA_RED_Deploy_on(void)                      /* +ve roll */
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

SA1_status_t rHAL_SA1_Deploy_status_check(void)
{
	SWHW_STATUS.SA_status_1 = (unsigned char)SA1_status;
	if(SA1_status == SA1_DEPLOY_CMD_RCVD)
	{
		if((IO_IN_LATCH_REGISTER_4 & SA1_DEPLOY_DEF_ST) == SA1_DEPLOY_ST)
		{
			SA1_status = SA1_DEPLOYED;
		}
		else if(TC_data_command_Table.SA_PanelHeater_Timeout <= (Major_Cycle_Count - SA1_Deploy_cmd_rcvd_time) )/* testing for 10sec */
		{
				SA1_status = SA1_HEATER_TIME_OUT;
		}
		else if(ADC_Buffer[40] >= TC_data_command_Table.TC_over_Heat )               /* data_command to be added */
		{
				SA1_status = SA1_OVER_HEAT;
		}
		else
		{
			/* */
		}

	}
	return SA1_status;
}

SA2_status_t rHAL_SA2_Deploy_status_check(void)
{
	SWHW_STATUS.SA_status_2 = (unsigned char)SA2_status;
	if(SA2_status == SA2_DEPLOY_CMD_RCVD)
	{
		if((IO_IN_LATCH_REGISTER_4 & SA2_DEPLOY_DEF_ST) == SA2_DEPLOY_ST)
		{
			SA2_status = SA2_DEPLOYED;

		}
		else if(TC_data_command_Table.SA_PanelHeater_Timeout <= (Major_Cycle_Count - SA2_Deploy_cmd_rcvd_time) )//testing for 10sec
		{
			SA2_status = SA2_HEATER_TIME_OUT;

		}
		else if(ADC_Buffer[41] >= TC_data_command_Table.TC_over_Heat )
		{
			SA2_status = SA2_OVER_HEAT;
		}
		else
		{
			/* */
		}

	}
	return SA2_status;
}

void rHAL_SA_Deploy_Status_new(void)
{
	uint32 tempdata;
	SA1_status_t SA1_Status;
	SA2_status_t SA2_Status;

	SA2_Status = rHAL_SA2_Deploy_status_check();
	SA1_Status = rHAL_SA1_Deploy_status_check();

	tempdata = IO_IN_LATCH_REGISTER_4;
	panel_deploy_sts= ((tempdata >> 8) & 0x0003);

	if(SA_Command_Type == SA_MAIN)              /*  check main or red command received */
	{
		if (((SA1_Status == SA1_DEPLOYED) && (SA2_Status == SA2_DEPLOYED)) ||
			(SA1_Status == SA1_HEATER_TIME_OUT) ||
			(SA2_Status == SA2_HEATER_TIME_OUT) ||
			(SA1_status == SA1_OVER_HEAT) ||
			(SA2_status == SA2_OVER_HEAT))
		{
			/* Abort the deployment */
			Out_latch_5.SA1_DEPLOY = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}

		else
		{
			/* continue with deployment */
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
			/* Abort the deployment */
			Out_latch_5.SPARE1_ON_OFF = 0;
			tempdata = Out_latch_5.data;
			IO_LATCH_REGISTER_5;
			IO_LATCH_REGISTER_5 = tempdata;
			SA_Command_Type = SA_CMD_NOT_RCVD;
		}

		else
		{
			/* continue with deployment */
		}
	}
	else
	{
		/* DO NOTHING */
	}
}
