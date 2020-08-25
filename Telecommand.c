#include <math.h>
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_IMU.h"
#include "HAL_Address.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "adcs_VarDeclarations.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "adcs_Constants.h"
#include "adcs_SensorDataProcs.h"
//#include "HAL_Heater.h"



extern unsigned char NSP_addr_table[4];
BlkExe_Stat BlkExe_Status = BLK_Disabled;

//Initialization of Function Table-------------------------
void(*FuncExecute_Table[TC_func_exe_MAX_LIMIT])() = {
		TC_IMU1_On,                                         			/* offset =    0  */
		TC_IMU2_On,    													/* offset =    1  */
		TC_IMU1_Off,													/* offset =    2  */
		TC_IMU2_Off,													/* offset =    3  */
		imu1_db_execute,												/* offset =    4  */ //Remove
		imu2_db_execute,												/* offset =    5  */ //Remove
		imu1_db_checksum,												/* offset =    6  */ //Remove
		imu2_db_checksum,												/* offset =    7  */ //Remove
		imu_test_sys_sel,												/* offset =    8  */
		TC_Drift_Uplink_Compenstation_Update_IMU1,						/* offset =    9  */
		TC_Drift_Uplink_Compenstation_Update_IMU2,						/* offset =    10 */
		TC_Gyro_Misalignment_Update_IMU1,								/* offset =    11 */
		TC_Gyro_Misalignment_Update_IMU2,								/* offset =    12 */
		TC_Gyro_Scale_Factor_Update_IMU1,								/* offset =    13 */
		TC_Gyro_Scale_Factor_Update_IMU2,								/* offset =    14 */
		TC_MagBias_Uplink_Compenstation_Update_IMU1,					/* offset =    15 */
		TC_MagBias_Uplink_Compenstation_Update_IMU2,					/* offset =    16 */
		TC_Mag_Misalignment_Update_IMU1,								/* offset =    17 */
		TC_Mag_Misalignment_Update_IMU2,								/* offset =    18 */
		TC_Mag_Scale_Factor_Update_IMU1,								/* offset =    19 */
		TC_Mag_Scale_Factor_Update_IMU2,								/* offset =    20 */

		rSSmain_ImaxF_update,											/* offset =    21 */ //Renamed from mag LPF 18-08-2020
		rSSredt_ImaxF_update,											/* offset =    22 */ //Renamed from mag LPF 18-08-2020
		TC_Gyro_LPF_Gain_Update_IMU1,									/* offset =    23 */ //Remove
		TC_Gyro_LPF_Gain_Update_IMU2,									/* offset =    24 */ //Remove


		TC_ACC_Ang_RESET,												/* offset =    25 */
		//TC_Panel1_Deploy,
		TC_NMI_count_reset,                                             /* offset =    26 */
		rMag_Refeci_update,												/* offset =    27 */ // Renamed from Panel2 deploy 18-08-2020
		TC_GPS1_ON,														/* offset =    28 */
		TC_GPS1_OFF,													/* offset =    29 */
		TC_GPS1_NMEA_VTG_enable,										/* offset =    30 */
		TC_GPS1_NMEA_VTG_disable,										/* offset =    31 */
		TC_GPS1_NMEA_GGA_enable,										/* offset =    32 */
		TC_GPS1_NMEA_GGA_disable,										/* offset =    33 */
		TC_GPS1_NMEA_GSA_enable,										/* offset =    34 */
		TC_GPS1_NMEA_GSA_disable,										/* offset =    35 */
		TC_GPS1_cold_start,												/* offset =    36 */
		TC_GPS1_factory_reset,											/* offset =    37 */
		TC_GPS2_on,														/* offset =    38 */
		TC_GPS2_off,													/* offset =    39 */
		TC_GPS2_NMEA_VTG_enable,										/* offset =    40 */
		TC_GPS2_NMEA_VTG_disable,										/* offset =    41 */
		TC_GPS2_NMEA_GGA_enable,										/* offset =    42 */
		TC_GPS2_NMEA_GGA_disable,										/* offset =    43 */
		TC_GPS2_NMEA_GSA_enable,										/* offset =    44 */
		TC_GPS2_NMEA_GSA_disable,										/* offset =    45 */
		TC_GPS2_cold_start,												/* offset =    46 */
		TC_GPS2_factory_reset,											/* offset =    47 */
		TC_W1_ON,														/* offset =    48 */
		TC_W2_ON,														/* offset =    49 */
		TC_W3_ON,														/* offset =    50 */
		TC_W4_ON,														/* offset =    51 */
		TC_W1_OFF,														/* offset =    52 */
		TC_W2_OFF,														/* offset =    53 */
		TC_W3_OFF,														/* offset =    54 */
		TC_W4_OFF,														/* offset =    55 */
		TC_Nominal_wheel_speed_execute,									/* offset =    56 */
		TC_MTR_ON,							           					/* offset =    57 */
		Pitch_Torquer_ON,												/* offset =    58 */
		Yaw_Torquer_ON,													/* offset =    59 */
		TC_MTR_OFF,										        		/* offset =    60 */
		rTC_HILS_ENABLE,												/* offset =    61 */// replaced pitch torquer off (not used)
		rTC_HILS_DISABLE,												/* offset =    62 */// replaced yaw torquer off (not used)
		rTC_Detumbling_ModePreprocessing_BDOT,							/* offset =    63 */
		rTC_Detumbling_ModePreprocessing_GYRO,							/* offset =    64 */
		rTC_SunAcquisition_ModePreprocessing,							/* offset =    65 */
		rTC_ThreeAxis_ModePreprocessing,								/* offset =    66 */
		rTC_Suspended_ModePreprocessing,								/* offset =    67 */
		payload_1_on,													/* offset =    68 */
		payload_1_off,													/* offset =    69 */
		rHeater1_on,													/* offset =    70 */
		rHeater1_off,													/* offset =    71 */
		rHeater2_on,													/* offset =    72 */
		rHeater2_off,													/* offset =    73 */
		rHeater3_on,													/* offset =    74 */
		rHeater3_off,													/* offset =    75 */
		rHeater4_on,													/* offset =    76 */
		rHeater4_off,													/* offset =    77 */
		rHeater5_on,													/* offset =    78 */
		rHeater5_off,													/* offset =    79 */
		rHeater6_on,													/* offset =    80 */
		rHeater6_off,													/* offset =    81 */
		RX_TX_deployed,													/* offset =    82 */
		PL_K_CMD_STS,													/* offset =    83 */
		PL_K_CMD_ACQ,													/* offset =    84 */
		PL_K_CMD_HLT,													/* offset =    85 */
		PL_K_DIAG,														/* offset =    86 */
		PL_K_CMD_OFF,													/* offset =    87 */
		PL_K_CMD_ON,													/* offset =    88 */
		rTC_HILS_MODE_IDLE,												/* offset =    89 */ //Renamed from ss 18-08-2020
		rTC_HILS_MODE_START,										    /* offset =    90 */ //Renamed from ss 18-08-2020
		rTC_HILS_MODE_STOP,												/* offset =    91 */ //Renamed from ss 18-08-2020
		rSun_Ephemeris_update,										    /* offset =    92 */ //Renamed from ss 18-08-2020
		rTLE_Update,												    /* offset =    93 */  //Renamed from TC_Qinit 14-08-2020
		TC_init_RW1,													/* offset =    94 */
		TC_init_RW2,													/* offset =    95 */
		TC_init_RW3,													/* offset =    96 */
		TC_init_RW4,													/* offset =    97 */
		TC_MTR_Roll_Positive,											/* offset =    98 */
		TC_MTR_Roll_Negative,											/* offset =    99 */
		TC_MTR_Pitch_Positive,											/* offset =    100*/
		TC_MTR_Pitch_Negative,											/* offset =    101*/
		TC_MTR_Yaw_Positive,											/* offset =    102*/
		TC_MTR_Yaw_Negative,											/* offset =    103*/
		rDifferentialTTC_Execute,										/* offset =    104*/
		OBC_ON,															/* offset =    105*/
		OBC_OFF,														/* offset =    106*/
		Antenna_mechanism_ON,											/* offset =    107*/
		Antenna_mechanism_OFF,											/* offset =    108*/
		payload_2_on,													/* offset =    109*/
		payload_2_off,													/* offset =    110*/
		S_band_tx_on,													/* offset =    111*/
		S_band_tx_off,													/* offset =    112*/
		X_band_tx_on,													/* offset =    113*/
		X_band_tx_off,													/* offset =    114*/
		SA_DEPLOYMENT_MECHANISM_MAIN_ON,                                /* offset =    115*/
		SA_DEPLOYMENT_MECHANISM_MAIN_OFF,                               /* offset =    116*/
		SA_DEPLOYMENT_MECHANISM_RED_ON,                                 /* offset =    117*/
		SA_DEPLOYMENT_MECHANISM_RED_OFF,                                /* offset =    118*/
		SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_ON,       /* offset =    119*/
		SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_OFF,      /* offset =    120*/
		Test_ON,                                                        /* offset =    121*/
		Test_OFF,                                                       /* offset =    122*/
		Antenna_mechanism_arm,                                          /* offset =    123*/
		Antenna_deploy,                                                 /* offset =    124*/
		Antenna_mechanism_disarm,                                       /* offset =    125*/
		sunlit,                                                         /* offset =    126*/
		eclipse,                                                        /* offset =    127*/
		Sunlit_eclipse_both,                                            /*  offset =   128*/
		rSafe_mode_PreProcessing,                                       /* offset =    129*/
		TC_MTR_Roll_No_cuurent,                                         /* offset =    130*/
		TC_MTR_Pitch_No_cuurent,                                        /* offset =    131*/
		TC_MTR_Yaw_No_cuurent,                                          /* offset =   132 */
		TC_pl_debug,                                                    /* offset =   133 */
		TC_pl_tm,                                                       /* offset =  134  */
		TC_TM_DS_EN,                                                     /* offset =  135  */
		Antenna_RESET_command,                                           /* offset =  136  */
		Antenna_deploy_with_override,                                    /* offset =  137  */
		Antenna_system_temp,                                             /* offset =  138  */
		Antenna_deploy_status_report,                                    /* offset =  139  */
		Antenna_deploy_activation_count,                                 /* offset =  140  */
		Antenna_deploy_activation_time,                                  /* offset =  141  */
		eeprom_en,                                                       /* offset =  142  */
		eeprom_dis,                                                      /* offset =  143  */
		TC_GPS1_NMEA_NMEA_GSV_Enable,                                    /* offset = 144   */
		TC_GPS1_NMEA_NMEA_GSV_disable,                                   /* offset = 145   */
		TC_GPS2_NMEA_NMEA_GSV_Enable,                                    /* offset = 146  */
		TC_GPS2_NMEA_NMEA_GSV_disable,                                   /* offset = 147 */
		TC_ATTC_CMD_clear,                                               /* offset = 148 */
		// Added on 05-08-2020
		rTC_ref_snv_bias_q_update,										/* offset = 149 */
		rTC_ref_stn_bias_q_update,										/* offset = 150 */
		rTC_ref_q_gnd_update,											/* offset = 151 */
		TC_q_body_init,													/* offset = 152 */
		rElapsedTimerAssign,											/* offset = 153 */


	};

 // Initialization of Resolution Table-----------------------
float Resol_Table[TC_data_command_MAX_LIMIT] =
       {
			0.076293945,    /*offset 0*/
			0.076293945,    /*offset 1*/
			0.076293945,    /*offset 2*/
			0.076293945,    /*offset 3*/
			1,			    /*offset 4*/
			1,              /*offset 5*/
			1,              /*offset 6*/
			1,              /*offset 7*/
			1, 			    /*offset 8*/
			1,              /*offset 9*/
			1,              /*offset 10*/
			1,              /*offset 11*/
			1, 			    /*offset 12*/
			1,              /*offset 13*/
			1,              /*offset 14*/
			8,              /*offset 15*/
/************** Added on 27 July 2019 **************/
			1.74532925e-6,				/*offset 16*/
			1.74532925e-6,         		/*offset 17*/
			1,              /*offset 18*/
			1,              /*offset 19*/
			1, 			    /*offset 20*/
			1,              /*offset 21*/
       };
// Initialization of Resolution Table-----------------------
float Resol_Table_Adcs[ADCS_TC_data_command_MAX_LIMIT] =
{
		1.74532925e-6,              /*offset 1*/
		1.74532925e-6,              /*offset 2*/
		1.74532925e-6,              /*offset 3*/
		1.74532925e-6, 			    /*offset 4*/
		1.74532925e-6,              /*offset 5*/
		1.74532925e-6,              /*offset 6*/
		1,              /*offset 7*/
		1, 			    /*offset 8*/
		1,              /*offset 9*/
		1,              /*offset 10*/
		1,			    /*offset 11*/
		1,              /*offset 12*/
		1,              /*offset 13*/
		1,              /*offset 14*/
		1, 			    /*offset 15*/
		1,              /*offset 16*/
		1,              /*offset 17*/
		1,              /*offset 18*/
		1, 			    /*offset 19*/
		1,              /*offset 20*/
		1,              /*offset 21*/
		1,              /*offset 22*/
		1, 			    /*offset 23*/
		1,              /*offset 24*/
		1,              /*offset 25*/
		1,              /*offset 26*/
		1,              /*offset 27*/
		1,              /*offset 28*/
		0.01, 			    /*offset 29*/
		1.74532925e-6,              /*offset 30*/
		1.74532925e-6,              /*offset 31 */
		1,              /*offset 32*/
		1,              /*offset 33*/
		1, 			    /*offset 34*/
		1,              /*offset 35*/
		1,             /*offset 36 */
		1,             /*offset 37 */
		1,             /*offset 38 */
		1,             /*offset 39 */
		1,             /*offset 40 */
		1.74532925e-6,             /*offset 41 */
		1.74532925e-6,             /*offset 42 */
		1,             /*offset 43 */
		1,             /*offset 44 */
		0.001,             /*offset 45 */
		0.001,             /*offset 46 */
		0.001,             /*offset 47 */
		0.1,             /*offset 48 */
		0.1,             /*offset 49 */
		1,             /*offset 50 */
		1,             /*offset 51 */
		0.001,             /*offset 52 */
		0.001,             /*offset 53 */
		0.001,             /*offset 54 */
		1,             /*offset 55 */
		1,             /*offset 56 */
		1.74532925e-6,             /*offset 57 */
		1.74532925e-6,             /*offset 58 */
		0.1,             /*offset 59 */
		0.0001,             /*offset 60 */
		0.0001,             /*offset 61 */
		0.0001,             /*offset 62 */
		0.0001,             /*offset 63 */

   };
unsigned int atp;
unsigned short TC_command_pending;
unsigned int *TC_ptr_count;
//TODO: Add these to TM
unsigned char RcvdParity = 0;
unsigned char ComputedParity = 0;
unsigned int tc_cnt_test;
unsigned long long int tc_ecv_test[50];
unsigned int i_tc_test;


uint32 rHAL_TC_Read()
{
	int i_TC;

	TC_Buffer_Addr = TC_BUFFER_BASE;
	TC_Status_Data = TC_STATUS_REGISTER;
	unsigned long int TC_Rcvd_56Bit[2] = {0UL,0UL};                 // {0x00c41000,0x07900000};

	#ifdef QUALIFICATION_MODEL
			TC_STATUS_REGISTER;
	#endif

	if ((TC_Status_Data & (TC_AUTHENTIC_PULSE_MASK | TC_DATA_READY)) == 0x00008002)  // Check Authentic Pulse
	{
		atp++;
		// Enable FPGA DATA Ready Bit
//		TC_STATUS_REGISTER = (TC_Status_Data | TC_WRITE_BIT_MASK); // Set WR Bit Before Reading TC SPC Buffer
		TC_STATUS_REGISTER2 = TC_WRITE_BIT_MASK;             // TC_buffer read_enable
		TC_authentic_pulse_rcvd = TRUE;
		Telecommand_ptr = &(u_TC.rcvd[0]);
		for(i_TC = 0;i_TC < 4;i_TC++)
		    {
				*Telecommand_ptr++ = (uint16)(REG32(TC_Buffer_Addr) & 0x0000FFFF);
				TC_Buffer_Addr += 0x00000004;
		    }
		if(i_tc_test < 50)
		{
			tc_ecv_test[i_tc_test++]= u_TC.cmd_rcvd;
		}


		//Copying to TM Buffer with appropriate index
		TM.Buffer.TM_Last_seen_TC[0] = u_TC.data_rcvd[0];
		TM.Buffer.TM_Last_seen_TC[1] = (u_TC.data_rcvd[1] & 0xFFFFFF00);

		ST_normal.ST_NM_Buffer.TM_Last_seen_TC[0] = u_TC.data_rcvd[0];
		ST_normal.ST_NM_Buffer.TM_Last_seen_TC[1] = (u_TC.data_rcvd[1] & 0xFFFFFF00);

		//TC_STATUS_REGISTER2 = 0x00000000;                         //Clear data ready bit and authentic pulsezz
		//Test : Create reset for FPGA FSM ( on 18_05_2019)
	#ifdef QUALIFICATION_MODEL
		GPIO_pins.PIO_9 = 0;
		IODAT = GPIO_pins.data;
	#endif


	}
	else
	{
		TC_authentic_pulse_rcvd = FALSE;
	}

	/** Computing parity & authenticating again */
	if(TC_authentic_pulse_rcvd)
	{
		TC_Rcvd_56Bit[0] = u_TC.data_rcvd[0] >> 8;
		TC_Rcvd_56Bit[1] = (u_TC.data_rcvd[0] << 24) & 0xFF000000;
		TC_Rcvd_56Bit[1] = TC_Rcvd_56Bit[1] | (u_TC.data_rcvd[1] >> 8);
		BCHEncoder((unsigned long long int*)TC_Rcvd_56Bit);
		RcvdParity = 0U;
		ComputedParity = 0U;

		RcvdParity = (unsigned char)((TC_Status_Data & 0x00007F00) >> 7);
		ComputedParity = TC_Rcvd.parity;
		//TM.Buffer.ParityRcvd = RcvdParity;
		//TM.Buffer.ComputedParity = ComputedParity;
		if (RcvdParity == ComputedParity)
		{
			TC_authentic_pulse_rcvd = True;
		}
		else
		{
			tc_cnt_test++;
			TC_authentic_pulse_rcvd = False;
		}

	}

	return TC_authentic_pulse_rcvd;
}

void rTC_write_tC_history()
{

	if(TC_flag == 1)
	{
		TC_hist_write_ptr -> TC_execute_hist.TC_cmd_executed 	= u_TC.cmd_rcvd;
		TC_hist_write_ptr -> TC_execute_hist.TC_exe_count 		= TC_cmd_executed_count;
		TC_hist_write_ptr -> TC_execute_hist.OBT_time_stamp 	= Major_Cycle_Count;

		TC_flag = 0;
		if(TC_hist_write_ptr < &TC_hist_data[TC_HISTORY_MAX])
			TC_hist_write_ptr++;
		else
			TC_hist_write_ptr = &TC_hist_data[0];
	}
}

void rTelecommand()
{
	if (TC_authentic_pulse_rcvd)								    //Checks whether if Authentic pulse is received
	{
		TC_count++;
		TC_flag = 1;
		switch (u_TC.Frame.command_type_A)
		{
			case 0x0: rReal_Time_TC();							    //Real-time telecommands
					  rTC_write_tC_history();
					  break;

			case 0x1: if(u_TC.Frame.command_type_B == 0x0F)
			          {
						rAbsoluteTTC_Delete();            	  	    //Absolute time-tag Delete telecommand
			          }
					  else
					  {
						rAbsolute_TimeTag_TC();				  	    //Absolute time-tag telecommands
					  }
					  break;

			case 0x2: rDifferential_TimeTag_TC();				//Differential time-tag telecommands
					  break;

			 default: rBlock_TC();								    //Bulk telecommands
					  break;

		}
	}

	TC_authentic_pulse_rcvd = False;
	Absolutetime = Major_Cycle_Count>>4;
	TM.Buffer.TM_ATTC_count = ATTC_count;

	//Remote_address
	TM.Buffer.TM_Remote_Addr = Remote_Addr;
}

void TC_status_reset()
{
	TC_STATUS_REGISTER2 = 0x00000000;                         //Clear data ready bit and authentic pulsezz
}

unsigned short TC_cmd_executed;
void rReal_Time_TC()
{
	switch (u_TC.Frame.command_type_B)
	{
		case 0x1:	rData_TC();					               	//Processing of Data Telecommands
					TC_cmd_executed_count++;
					break;

		case 0x2: 	rBoolean_TC();				              	//Processing of Boolean(ON/OFF) Telecommands
					TC_cmd_executed_count++;
					break;

		case 0x3: 	rGainSelect_TC();			              	//Processing of Gain Select Telecommands
					TC_cmd_executed_count++;
					break;

		case 0x4: 	rFuncExecute_TC();							//Processing of Function Execute Telecommands
					TC_cmd_executed_count++;
					break;

		case 0x5: 	rRemoteProgram_Addr_TC();					//Processing of Remote Program Address Telecommands
					TC_cmd_executed_count++;
					break;

		case 0x6: 	rRemoteProgram_Data_TC();					//Processing of Remote Program Data Telecommands
					TC_cmd_executed_count++;
					break;

		case 0x7:   rADCS_Data_TC();        				   //Processing of ADCS Data Telecommands(float)
					TC_cmd_executed_count++;
					break;

		case 0x8:  rRemote_base_addr_TC();
					TC_cmd_executed_count++;
					break;

		 default: 	rContingency_TC();							//Processing of Contingency Telecommands
					break;
	}
}

void rContingency_TC()
{
	return;														// Decoded in hardware
}

unsigned short temp_data_rw;
unsigned short rw_data;

 // Routine for Data_Commands Processing
void rData_TC()
{
	//TC_cmd_executed++;

	int   RW_number;
	int   iw_speed;    /* value sign extended to 32 bit integer */
	float fw_speed;    /* converted value in float              */
	short tempdata;
	iw_speed = (int) ((short) (u_TC.DataCommand.Data & 0xffff));
	fw_speed = (float) (iw_speed) * Resol_Table[u_TC.DataCommand.offset_addr];

	switch(u_TC.DataCommand.offset_addr)
	{

	/* Following are for setting the Reaction Wheel Speeds by TC */

	case 0:   TC_data_command_Table.RW1_Speed = fw_speed;
			  break;
	case 1:	  TC_data_command_Table.RW2_Speed = fw_speed;
	          break;
	case 2:   TC_data_command_Table.RW3_Speed = fw_speed;
	          break;
	case 3:   TC_data_command_Table.RW4_Speed = fw_speed;
			  break;

	/* following is for setting the delta time tag serial number */

	case 4:  TC_data_command_Table.Differential_srl_num 						= u_TC.DataCommand.Data;
	           break;

	/**************** Below telecommands are not implemented *****************/
	case 5:  TC_data_command_Table.SA1_SHUNT_LTP 								= u_TC.DataCommand.Data;
		       break;
	case 6:  TC_data_command_Table.SA1_SHUNT_UTP 								= u_TC.DataCommand.Data;
			   break;
	case 7:  TC_data_command_Table.SA2_SHUNT_LTP 								= u_TC.DataCommand.Data;
			   break;
	case 8:  TC_data_command_Table.SA2_SHUNT_UTP 								= u_TC.DataCommand.Data;
			   break;
	case 9:  TC_data_command_Table.SA3_SHUNT_LTP 								= u_TC.DataCommand.Data;
			   break;
	case 10: TC_data_command_Table.SA3_SHUNT_UTP 								= u_TC.DataCommand.Data;
			   break;
	case 11: TC_data_command_Table.BATTERY_HEATER1_UTP						    = u_TC.DataCommand.Data;
	          break;
	case 12: TC_data_command_Table.BATTERY_HEATER1_LTP 							= u_TC.DataCommand.Data;
	           break;
	case 13: TC_data_command_Table.SA_PanelHeater_Timeout 						=
			 (unsigned long int)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
			  break;
	/************************** Added on 27 July 2019 **************************/
	case 14: 	//TC_data_command_Table.TC_RW_NO = u_TC.DataCommand.Data;
				temp_data_rw = u_TC.DataCommand.Data;
				rw_data = temp_data_rw;
				RW_number = ((temp_data_rw & 0x0300) >> 8);
				NSP_addr_table[RW_number] = (temp_data_rw & 0x00FF);
				break;


	case 15: TC_data_command_Table.TC_heaters_auto_manual						= u_TC.DataCommand.Data;
				break;
	case 16: TC_data_command_Table.TC_heaters_manual						    = u_TC.DataCommand.Data;
				break;
	case 17: TC_data_command_Table.pl_data_command_1						    = u_TC.DataCommand.Data;
				pl_data_command.Data_command_1                                  = TC_data_command_Table.pl_data_command_1;
				break;

	case 18:TC_data_command_Table.pl_data_command_2						        = u_TC.DataCommand.Data;
			pl_data_command.Data_command_2                                      = TC_data_command_Table.pl_data_command_2;
					break;

	case 19: TC_data_command_Table.pl_data_command_3						    = u_TC.DataCommand.Data;
			pl_data_command.Data_command_3                                      = TC_data_command_Table.pl_data_command_3;
					break;

	case 20: TC_data_command_Table.pl_data_command_4						    = u_TC.DataCommand.Data;
			pl_data_command.Data_command_4                                      = TC_data_command_Table.pl_data_command_4;
					break;

	case 21: TC_data_command_Table.pl_data_command_5						    = u_TC.DataCommand.Data;
			pl_data_command.Data_command_5                                    	= TC_data_command_Table.pl_data_command_5;
					break;

	case 22: TC_data_command_Table.pl_data_command_6						    = u_TC.DataCommand.Data;
			pl_data_command.Data_command_6                                      = TC_data_command_Table.pl_data_command_6;
					break;

	case 23: TC_data_command_Table.BLK_Number_selection                         = u_TC.DataCommand.Data;
				tempdata  = u_TC.DataCommand.Data;
				BlkExe_Status = tempdata;
					break;

	case 24: TC_data_command_Table.TC_power_safe_LTP                            = u_TC.DataCommand.Data;
	         break;

	case 25: TC_data_command_Table.TC_power_safe_UTP                            = u_TC.DataCommand.Data;
		     break;

	case 26: TC_data_command_Table.TC_over_Heat                                 = u_TC.DataCommand.Data;
		     break;

	default:
		      break;
	}
}

void rADCS_Data_TC()
{
	switch(u_TC.DataCommand.offset_addr)
		{

			case 0: ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU1[0]		    =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 1: ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU1[1]		    =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 2: ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU1[2]	        =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 3: ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU2[0]		    =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 4: ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU2[1]	        =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 5: ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU2[2]		    =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 6: ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU1[0] 		=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 7: ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU1[1]		=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 8: ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU1[2] 		=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 9: ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU2[0] 		=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 10: ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU2[1]		=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 11: ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU2[2] 		=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 12: ADCS_TC_data_command_Table.TC_Mag_Misalignment_IMU1 					= u_TC.DataCommand.Data;// REMOVE
						break;
			case 13: ADCS_TC_data_command_Table.TC_Mag_Misalignment_IMU2 					= u_TC.DataCommand.Data;// REMOVE
						break;
			case 14: ADCS_TC_data_command_Table.TC_Mag_Scale_Factor_IMU1 					= u_TC.DataCommand.Data;// REMOVE
						break;
			case 15: ADCS_TC_data_command_Table.TC_Mag_Scale_Factor_IMU2 					= u_TC.DataCommand.Data;// REMOVE
						break;
			case 16: ADCS_TC_data_command_Table.TC_ACC_Ang_RESET  							= u_TC.DataCommand.Data;// REMOVE
						break;
			case 17: ADCS_TC_data_command_Table.TC_SS_misalnCM1256 							= u_TC.DataCommand.Data;// REMOVE
						break;
			case 18: ADCS_TC_data_command_Table.TC_SS_misalnCM2356 							= u_TC.DataCommand.Data;// REMOVE
						break;
			case 19: ADCS_TC_data_command_Table.TC_SS_misalnCM3456 							= u_TC.DataCommand.Data;// REMOVE
						break;
			case 20: ADCS_TC_data_command_Table.TC_SS_misalnCM4156 							= u_TC.DataCommand.Data;// REMOVE
						break;
			case 21: ADCS_TC_data_command_Table.TC_SS_Imax_ALPHA 							= u_TC.DataCommand.Data;  //TO BE ADDED IN DOCUMENT // REMOVE
						break;
			case 22: ADCS_TC_data_command_Table.TC_eclipse_entrytime  						= u_TC.DataCommand.Data;
						break;
			case 23: ADCS_TC_data_command_Table.TC_eclipse_exittime  						= u_TC.DataCommand.Data;
						break;
			case 24: ADCS_TC_data_command_Table.TC_elapsed_orbitTimer  						= u_TC.DataCommand.Data;
						break;
			case 25: ADCS_TC_data_command_Table.TC_Sunlit_detctn_timer  					= u_TC.DataCommand.Data;// REMOVE
						break;
			case 26: ADCS_TC_data_command_Table.TC_Time_GPS2TLE 							= u_TC.DataCommand.Data;
						break;
			case 27: ADCS_TC_data_command_Table.TC_GPS_OFFSET_UTC 							= u_TC.DataCommand.Data;// REMOVE
						break;
			case 28: ADCS_TC_data_command_Table.TC_delUT1_ECEF2ECI 							=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 29: ADCS_TC_data_command_Table.TC_delAT_ECEF2ECI 							=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 30: ADCS_TC_data_command_Table.TC_xp_ECEF2ECI 								=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 31: ADCS_TC_data_command_Table.TC_yp_ECEF2ECI 								=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 32: ADCS_TC_data_command_Table.TC_JulianDay_at_OBT0					    = u_TC.DataCommand.Data;// REMOVE
						break;
			case 33: ADCS_TC_data_command_Table.TC_OBT_Drift_Corr 							= u_TC.DataCommand.Data;// REMOVE
						break;
			case 34: ADCS_TC_data_command_Table.TC_JulianDate_at_OrbitalEpoch 				= u_TC.DataCommand.Data;// REMOVE
						break;
			case 35: ADCS_TC_data_command_Table.TC_OBT_with_TLE_Update 						= u_TC.DataCommand.Data;// REMOVE
						break;
			case 36: ADCS_TC_data_command_Table.TC_Wheel_Configuration_Index 				= u_TC.DataCommand.Data;// REMOVE
						break;
			case 37: ADCS_TC_data_command_Table.TC_Det_Bprev_Count 							= u_TC.DataCommand.Data;
						break;
			case 38: ADCS_TC_data_command_Table.TC_Det_BDOT_Compute_Count 					= u_TC.DataCommand.Data;
						break;
			case 39: ADCS_TC_data_command_Table.TC_Det_GYRO_Compute_Count			 		= u_TC.DataCommand.Data;
						break;
			case 40: ADCS_TC_data_command_Table.TC_Rate_Chk_Safe2Det 						= u_TC.DataCommand.Data;// REMOVE
						break;
			case 41: ADCS_TC_data_command_Table.TC_ECEF_stationlatitude 					=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 42: ADCS_TC_data_command_Table.TC_ECEF_stationLongitude 					=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 43: ADCS_TC_data_command_Table.TC_Error_dev_SunlitAD 						= u_TC.DataCommand.Data;// REMOVE
						break;
			case 44: ADCS_TC_data_command_Table.TC_Error_dev_EclipseAD 						= u_TC.DataCommand.Data;// REMOVE
						break;
			case 45: ADCS_TC_data_command_Table.TC_wAD_BODYmaxThRoll 						=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 46: ADCS_TC_data_command_Table.TC_wAD_BODYmaxThPitch 						=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 47: ADCS_TC_data_command_Table.TC_wAD_BODYmaxThYaw 						=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 48: ADCS_TC_data_command_Table.TC_magMin_angle 							=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 49: ADCS_TC_data_command_Table.TC_magMax_angle 							=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 50: ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh						=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 51: ADCS_TC_data_command_Table.TC_PanelD_Status_Sel					    = u_TC.DataCommand.Data;// REMOVE
						break;
			case 52: ADCS_TC_data_command_Table.TC_wAD_BODYminThRoll						=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						 break;
			case 53: ADCS_TC_data_command_Table.TC_wAD_BODYminThPitch						=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 54: ADCS_TC_data_command_Table.TC_wAD_BODYminThYaw						    =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 55: ADCS_TC_data_command_Table.TC_wAD_updateTimeThresh					    = u_TC.DataCommand.Data;
						break;
			case 56: ADCS_TC_data_command_Table.TC_wp_QDP						            = u_TC.DataCommand.Data;
						break;
			case 57: ADCS_TC_data_command_Table.TC_nut_dpsi						            =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 58: ADCS_TC_data_command_Table.TC_nut_deps						            =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 59: ADCS_TC_data_command_Table.TC_ref_svn_bias_off_deg						=
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 60: ADCS_TC_data_command_Table.TC_q_command_0						        =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 61: ADCS_TC_data_command_Table.TC_q_command_1						        =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 62: ADCS_TC_data_command_Table.TC_q_command_2						        =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
			case 63: ADCS_TC_data_command_Table.TC_q_command_3						        =
					(float)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
						break;
		    default:
			            break;
		}

}

void rRemote_base_addr_TC()
{
	if(u_TC.Remote.command_type_B == 0x8)
	{
		Remote_data_addr = u_TC.Remote.data_addr;
	}
	Remote_minotoring_addr 			= Remote_data_addr;
	TM.Buffer.TM_Remote_Addr_SF0 	= Remote_minotoring_addr;
}


void rRemote_data_view()
{
	unsigned int remote_monitor_index = 0;
	unsigned int remote_blk_select;
	unsigned int* remote_ptr;

	if((Remote_data_addr >= RAM_SEG_START_ADDR) && (Remote_data_addr < RAM_SEG_END_ADDR))
	{
		Remote_minotoring_addr = Remote_data_addr;

		for(remote_monitor_index = 0; remote_monitor_index <= 255; remote_monitor_index++)
		{
		  TM.Buffer.TM_Remote_Data_SF0[remote_monitor_index] 	= REG32(Remote_minotoring_addr);
		  Remote_minotoring_addr +=4;
		}

	}


}

 // Routine for Boolean_Telecommand processing
void rBoolean_TC()
{
	//TC_cmd_executed++;

	if (u_TC.Bool.offset_addr <= sizeof(TC_boolean_u.TC_Boolean_Table)/sizeof(uint8))
	{
		TC_boolean_u.Pos[u_TC.Bool.offset_addr] = (char)u_TC.Bool.decision_bit;
    }
	else
	{
		//
	}
}

 // Routine for Gain_select_telecommand Processing
void rGainSelect_TC()
{
	//TC_cmd_executed++;

	if(u_TC.GainSelect.offset_addr <= sizeof(TC_gain_select_u.TC_gain_select_Table)/sizeof(uint8))
	{
		TC_gain_select_u.Pos[u_TC.GainSelect.offset_addr] = (char)u_TC.GainSelect.gain_set;
	}
	adcsgains();
}

 // Routine for Function_execution_Telecommand Processing
void rFuncExecute_TC()
{
	//TC_cmd_executed++;

	if(u_TC.FuncExecute.offset_addr < TC_func_exe_MAX_LIMIT)
	{
		FuncExecute_Table[u_TC.FuncExecute.offset_addr]();
	}
}

 // Routine for Remote_addr_Telecommand processing
void rRemoteProgram_Addr_TC()
{
	//TC_cmd_executed++;
	if(u_TC.Remote.command_type_B == 0x5)
	{
		Remote_Addr = u_TC.Remote.data_addr;
	}
}

 // Routine for Remote_data_Telecommand processing
void rRemoteProgram_Data_TC()
{
	if(u_TC.Remote.command_type_B == 0x6)
	{
		//TC_cmd_executed++;
		Remote_data = u_TC.Remote.data_addr;
		REG32(Remote_Addr) = Remote_data;
		Remote_Addr += 4;
	}
}

unsigned int test_attc;
 // Routine for Absolute_TimeTag_Telecommand Processing
void rAbsolute_TimeTag_TC()
{

	rAbsoluteTTC_Update();

}

 // Routine for Differential_Timetag_Telecommand Processing
void rDifferential_TimeTag_TC()
{
	rDifferential_TimeTag_Update();
	//rDifferential_TimeTag();
}

 // Routine for Block_Telecommand Processing
void rBlock_TC()
{
	Block_update();
}

//----------------------To be transferred to h files------------------------------
int32  Nodeptrempty_flag = 0;
int32  ATTC_Master_en = 0;
int32  ATTC_exe_en = 0;
int32  tos;
uint32 ATTC_count = 0;
Nodeptrtype head = NULL;

int32  DTTC_count = 0;
uint32 Diff_start_srl_num = 0;
uint32 Diff_stop_srl_num = 0;
uint32 DTTCtime_reference;
//int DTTC_exe_flag = 1;
//----------------------To be transferred to h files (END)--------------------------

void initNodetable()
{
   int32 attc_node_index;
   for (attc_node_index = 0; attc_node_index < MAX_TIMETAG_CMD_LIMIT; attc_node_index++)
     {
       Nodeptr[attc_node_index] = & Nodearray[attc_node_index];
     }

   tos = 0;
}

Nodeptrtype getNode()
{
    if (tos < MAX_TIMETAG_CMD_LIMIT)
	{
    	Top_index = tos++;
		return (Nodeptr[Top_index]);
    }
	else
	{
      return NULL;  // error return
	}
}

void freeNode(Nodeptrtype ptr)
{
	if (tos > 0)
	{
		tos --;
        Nodeptr[tos] = ptr;
		Nodeptrempty_flag = 0;
    }
}

// Routine for insertion of new node to the list
void rAbsoluteTTC_Update()
{

    if (tos < MAX_TIMETAG_CMD_LIMIT)					// Check whether memory exists in the pointer table for the node insertion
	{
    	ATTCformat new_data = u_TC.ATTC_Exe_cmd;
    	Nodeptrtype new_node = getNode();			//Create new node to be used for storing the TC data to the list

    	new_node->command  = new_data;
		new_node->next  = NULL;

		if(head == NULL || new_node->command.TC_time < head->command.TC_time)	// Check whether the list is empty or the node is to be inserted at the beginning of the list
		{
			new_node->next = head;
			head = new_node;
			ATTC_count++;
		}
		else										                // Node insertion in between or at the end
		{
			Nodeptrtype temp = head;
     		Nodeptrtype prev = NULL;

			while((temp != NULL) && (temp->command.TC_time <= new_node->command.TC_time))
			{
				prev = temp;
				temp = temp->next;
			}
				new_node->next = prev->next;
				prev->next = new_node;
				ATTC_count++;

		}
	}

	else 											                // If no memory is available for TC data storage
		return;
}
unsigned int ab_test,ab_test1;
// Execute the first ATTC
void rAbsoluteTTC_Execute()
{
	if (TC_boolean_u.TC_Boolean_Table.ATTC_Master_enable_flag)		// Check if the Manual execution flag is enabled
	{
		if (ATTC_count > 0)							                // ATTC list is empty or not
		{
			//Absolutetime = 0x000FFFFF & Major_Cycle_Count;
			if (head->command.TC_time == Absolutetime) 	             // Check whether the command to be executed has crossed the OBT.
			{
				ab_test = Absolutetime;
				ab_test1 = Major_Cycle_Count;
				u_TC.ATTC_Exe_cmd = head -> command;
				head -> command.TC_time = 0;
				rReal_Time_TC();

				// delete this node from pending command list
				freeNode(head);
				head = head->next;
				ATTC_count--;
			}
			else
			{
				return;
			}
		}

	}
// Fetching the telecommand which gets executed next of the present one
if(head != NULL)
{
	//Next_exe_TC = head -> command;
	//TM.Buffer.TM_Next_exe_TC = Next_exe_TC;
}
}


// Routine for insertion of new node to the array
void rDifferential_TimeTag_Update()
{
	if (DTTC_count >= MAX_TIMETAG_CMD_LIMIT)
		return;
	else
	{
		DTTCarray[u_TC.DTTC_Up_cmd.srl_num] = u_TC.DTTC_Up_cmd;
		DTTC_count++;
	}
}

// Execute the first DTTC
void rDifferential_TimeTag_Execute()
{
	uint32 i_start, i_stop;
	DTTCformat *ptr;
	uint32 ttc_time = 0;

	i_start = ( TC_data_command_Table.Differential_srl_num & 0xFF00) >> 8;
	i_stop  = ( TC_data_command_Table.Differential_srl_num & 0x00FF);
	ttc_time = Major_Cycle_Count ;
	while (i_start <= i_stop )
	{
		ptr = &DTTCarray[i_start];
		ttc_time =  (ttc_time + DTTCarray[i_start].TC_time) & 0x000FFFFF;
		u_TC.ATTC_Exe_cmd.TC_time              = ttc_time ;
		u_TC.ATTC_Exe_cmd.start_bit            = ptr -> start_bit;
		u_TC.ATTC_Exe_cmd.spacecraft_id        = ptr -> spacecraft_id;
		u_TC.ATTC_Exe_cmd.link_id              = ptr -> link_id;
		u_TC.ATTC_Exe_cmd.command_type_A       = ptr -> command_type_A;
		u_TC.ATTC_Exe_cmd.command_type_B       = ptr -> command_type_B;
		u_TC.ATTC_Exe_cmd.command 		       = ptr -> command;
		u_TC.ATTC_Exe_cmd.type_of_data_command = ptr -> type_of_data_command;
		u_TC.ATTC_Exe_cmd.TC_parity            = ptr -> TC_parity;
		u_TC.ATTC_Exe_cmd.TC_appended_bit      = ptr -> TC_appended_bit;
		rAbsoluteTTC_Update();
		i_start++;
	}
}

//Delete the node for the given position
/*void rAbsoluteTTC_Delete()
{
	unsigned short traverse_count = 0;
	Nodeptrtype temp = head;
	Nodeptrtype prev = NULL;
	while (temp->command.TC_time < u_TC.ATTC_Exe_cmd.TC_time)	  // Search for the ATT command to be deleted using the unique TC_time data
	{
		traverse_count++;
		if(traverse_count == MAX_TIMETAG_CMD_LIMIT)
		{
			break;
		}
		prev = temp;
		temp = temp->next;
	}
	if(traverse_count < MAX_TIMETAG_CMD_LIMIT)
	{
		prev->next = temp->next;
		temp->next = NULL;
		temp->command.TC_time = 0;
		freeNode(temp);
		ATTC_count--;
	}

}*/



// Test variables
/*
unsigned int delete_count1,delete_count2;
void rAbsoluteTTC_Delete()
{
	delete_count1 = 0;
	delete_count2 =0;
	Nodeptrtype temp = head;
	Nodeptrtype prev = NULL;

	while ((temp != NULL) && (temp->command.TC_time < u_TC.ATTC_Exe_cmd.TC_time))	  // Search for the ATT command to be deleted using the unique TC_time data
	{
		delete_count1++;
		prev = temp;
		temp = temp->next;
	}
	if(temp == NULL) return;

	prev->next = temp->next;
	temp->next = NULL;
	temp->command.TC_time = 0;
	freeNode(temp);
	ATTC_count--;
	delete_count2++;

}
*/


void rAbsoluteTTC_Delete()
{
	Nodeptrtype temp = head;
	Nodeptrtype prev = NULL;

	// Return if no linked list exists

	if (temp == NULL) return;

	if (temp->command.TC_time == u_TC.ATTC_Exe_cmd.TC_time)
	{
		// Data key match found in head node, so delete the node.

		head = temp->next;
		temp->command.TC_time = 0;
		freeNode(temp);
		ATTC_count--;
	}

	else
	{
		while ((temp != NULL) && (temp->command.TC_time < u_TC.ATTC_Exe_cmd.TC_time))	  // Search for the ATT command to be deleted using the unique TC_time data
		{
			prev = temp;
			temp = temp->next;

		}

		if ((temp != NULL) && (temp->command.TC_time == u_TC.ATTC_Exe_cmd.TC_time))
		{
			prev->next = temp->next;
			temp->next = NULL;
			temp->command.TC_time = 0;
			freeNode(temp);
			ATTC_count--;
		}

	}

}



// Clear the Telecommand data
void rAbsoluteTTC_Clear()		                               // Routine to clear the Linked list data
{

	Nodeptrtype temp = head;
	while(temp != NULL)
	{
		temp -> command.TC_time = 0;
		//temp -> command = NULL;
		freeNode(temp);
		temp = temp->next;
	}
	head = NULL;	// The linked list is now empty.
	ATTC_count = 0;
}

/** Block Command Functions */
/** Initialize the block index array and block command array to zero */
void rInit_Block(void)
{
	unsigned long int BLK_count = 0;
	unsigned long int BLK_index = 0;

	for(BLK_count = 0;BLK_count < MAX_BLKS; BLK_count++)
	{
		/** clear block index array */
		Block_Index[BLK_count] = 0;
		for(BLK_index = 0;BLK_index < (MAX_BLK_CMD_SIZE * 2); BLK_index++)
		{
			/** clear block cmd  array */
			Block_array[BLK_count][BLK_index] = 0;
		}
	}
}


unsigned char BLK_opn;
unsigned char BLK_number;
unsigned char tempdata1,tempdata2,block_test3,block_test1,block_test2, block_test;
/** Block command update
 *  - This function will allow Block TC to be added in incremental order
 *  - This function will allow Block TC to be updated within the index   */

/*
Block operation (BLK_opn): 2Bits
		BLK_opn = 1'b00; //0
		BLK_opn = 1'b01; //1
		BLK_opn = 1'b10; //2
*/

void Block_update(void)
{
	//tempdata1 = (unsigned char)u_TC.BLK_Update_cmd.Cmd_Srl;
	BLK_number = u_TC.BLK_Update_cmd.Blk_No;
	BLK_opn = u_TC.BLK_Update_cmd.Blk_opn;

	//block_test_array[block_test] = u_TC.cmd_rcvd;
	//block_test++;

	if((u_TC.BLK_Update_cmd.Cmd_Srl == 0)|| (u_TC.BLK_Update_cmd.Cmd_Srl == Block_Index[BLK_number]))
		{
			if(BLK_opn == 0x0)          // BLK_update
			{
				Block_array[BLK_number][u_TC.BLK_Update_cmd.Cmd_Srl] = u_TC.cmd_rcvd;
				Block_Index[BLK_number]++;
				block_test1++;
			}
		}
	if((u_TC.BLK_Update_cmd.Cmd_Srl == 0)|| (u_TC.BLK_Update_cmd.Cmd_Srl <= Block_Index[BLK_number]))
	{
			if(BLK_opn == 0x1)     // BLK_modify
			{

				Block_array[BLK_number][u_TC.BLK_Update_cmd.Cmd_Srl] = u_TC.cmd_rcvd;
				block_test2++;
			}
			else if(BLK_opn == 0x2)      // BLK_delete
			{
				block_test3++;
				Block_array[BLK_number][u_TC.BLK_Update_cmd.Cmd_Srl] = 0x00;
			}
			else
			{
				//
			}
		}

}
/*void Block_update(void)
{

	tempdata2 = u_TC.BLK_Update_cmd.Blk_No;
	tempdata1 = u_TC.BLK_Update_cmd.Blk_No;
	BLK_number = (tempdata2 & 0x3f);
	BLK_num = (tempdata1>>6);
	if((BLK_number < MAX_BLKS) && (u_TC.BLK_Update_cmd.Cmd_Srl < MAX_BLK_CMD_SIZE))
	{
		if((u_TC.BLK_Update_cmd.Cmd_Srl == 0)|| (u_TC.BLK_Update_cmd.Cmd_Srl <= Block_Index[BLK_number]))
			{
				if(BLK_num == 00)          // BLK_update
				{
					Block_array[BLK_number][u_TC.BLK_Update_cmd.Cmd_Srl] = u_TC.cmd_rcvd;
					Block_Index[BLK_number]++;
				}
				else if(BLK_num == 01)     // BLK_modify
				{

					Block_array[BLK_number][u_TC.BLK_Update_cmd.Cmd_Srl] = u_TC.cmd_rcvd;
				}
				else if(BLK_num == 10)      // BLK_delete
				{
					Block_array[BLK_number][u_TC.BLK_Update_cmd.Cmd_Srl] = 0x00;
				}
				else
				{
					//
				}
			}
	}
}*/

unsigned long long int test_3;
/** Block command execute */
void rBlockTC_Execute(void)
{
	unsigned short tempdata12;
	tempdata12 =  (unsigned short)BlkExe_Status;
	if (tempdata12 < BLK_Disabled)
	{
		if(BlkCurrent_Cmd < Block_Index[tempdata12])
		{
			u_TC.cmd_rcvd = Block_array[tempdata12][BlkCurrent_Cmd];
			test_3 =  Block_array[tempdata12][BlkCurrent_Cmd];
			rReal_Time_TC();
			BlkCurrent_Cmd++;
		}
		else
		{
			BlkCurrent_Cmd = 0;
			BlkExe_Status = BLK_Disabled;
		}
	}
}


//TMTC Buffer assignments
void TMTC_Assignment()
{
	//Boolean Command TMTC Assignment
	TMTC_boolean_u.Boolean_Table.Pitch_Torquer_Polarity_Reversal               = TC_boolean_u.TC_Boolean_Table.Pitch_Torquer_Polarity_Reversal;
	TMTC_boolean_u.Boolean_Table.Yaw_Torquer_Polarity_Reversal                 = TC_boolean_u.TC_Boolean_Table.Yaw_Torquer_Polarity_Reversal;
	TMTC_boolean_u.Boolean_Table.Roll_Torquer_Polarity_Reversal                = TC_boolean_u.TC_Boolean_Table.Roll_Torquer_Polarity_Reversal;
	TMTC_boolean_u.Boolean_Table.Pitch_Torquer_Enable_or_Disable               = TC_boolean_u.TC_Boolean_Table.Pitch_Torquer_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.Yaw_Torquer_Enable_or_Disable                 = TC_boolean_u.TC_Boolean_Table.Yaw_Torquer_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.Roll_Torquer_Enable_or_Disable                = TC_boolean_u.TC_Boolean_Table.Roll_Torquer_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_Sus2det_transit_en_dis                     = TC_boolean_u.TC_Boolean_Table.TC_Sus2det_transit_en_dis; // Changed on 17-8-2020
	TMTC_boolean_u.Boolean_Table.TC_AutoTransit_Det2SunAquisition              = TC_boolean_u.TC_Boolean_Table.TC_AutoTransit_Det2SunAquisition;
	TMTC_boolean_u.Boolean_Table.TC_SunAq2DetMode_autotransit                  = TC_boolean_u.TC_Boolean_Table.TC_SunAq2DetMode_autotransit;
	TMTC_boolean_u.Boolean_Table.TC_mom_dumping_ang_mom_based                  = TC_boolean_u.TC_Boolean_Table.TC_mom_dumping_ang_mom_based;
	TMTC_boolean_u.Boolean_Table.TC_SunAq2InertialMode_autotransit             = TC_boolean_u.TC_Boolean_Table.TC_SunAq2InertialMode_autotransit; //Remove
	TMTC_boolean_u.Boolean_Table.TC_IMU_Select                                 = TC_boolean_u.TC_Boolean_Table.TC_IMU_Select;
	TMTC_boolean_u.Boolean_Table.TC_SS_Cells_Sel                               = TC_boolean_u.TC_Boolean_Table.TC_SS_Cells_Sel;
	TMTC_boolean_u.Boolean_Table.TC_GPS12_Select                               = TC_boolean_u.TC_Boolean_Table.TC_GPS12_Select;
	TMTC_boolean_u.Boolean_Table.TC_GPS_TLE_Select                             = TC_boolean_u.TC_Boolean_Table.TC_GPS_TLE_Select;
	TMTC_boolean_u.Boolean_Table.TC_GPS_Utility                                = TC_boolean_u.TC_Boolean_Table.TC_GPS_Utility;
	TMTC_boolean_u.Boolean_Table.TC_EKF_Drift_Compensation_Enable_or_Disable   = TC_boolean_u.TC_Boolean_Table.TC_EKF_Drift_Compensation_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_EKF_MagBias_Compensation_Enable_or_Disable = TC_boolean_u.TC_Boolean_Table.TC_EKF_MagBias_Compensation_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_Mag_Torquer_Bias_Enable_or_Disable         = TC_boolean_u.TC_Boolean_Table.TC_Mag_Torquer_Bias_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_Sun_Ephemeris_en_dis                       = TC_boolean_u.TC_Boolean_Table.TC_Sun_Ephemeris_en_dis;
	TMTC_boolean_u.Boolean_Table.TC_Mag_Refeci_en_dis                          = TC_boolean_u.TC_Boolean_Table.TC_Mag_Refeci_en_dis;
	TMTC_boolean_u.Boolean_Table.TC_Wheel_AutoReConfig_Logic                   = TC_boolean_u.TC_Boolean_Table.TC_Wheel_AutoReConfig_Logic;
	TMTC_boolean_u.Boolean_Table.TC_Wheel_SpinUpDown_Logic                     = TC_boolean_u.TC_Boolean_Table.TC_Wheel_SpinUpDown_Logic;
	TMTC_boolean_u.Boolean_Table.TC_ThreeAxis2SafeMode_autotransit             = TC_boolean_u.TC_Boolean_Table.TC_ThreeAxis2SafeMode_autotransit;
	TMTC_boolean_u.Boolean_Table.TC_SunAq2ThreeAxis_autotransit                = TC_boolean_u.TC_Boolean_Table.TC_SunAq2ThreeAxis_autotransit;
	TMTC_boolean_u.Boolean_Table.TC_Det_AutoTransitionBDOTtoGYRO               = TC_boolean_u.TC_Boolean_Table.TC_Det_AutoTransitionBDOTtoGYRO;
	TMTC_boolean_u.Boolean_Table.TC_Detumbling_Logic_select                    = TC_boolean_u.TC_Boolean_Table.TC_Detumbling_Logic_select; //Remove
	TMTC_boolean_u.Boolean_Table.TC_Speed_Dumping                              = TC_boolean_u.TC_Boolean_Table.TC_Speed_Dumping;
	TMTC_boolean_u.Boolean_Table.TC_Sun_Varying_Mode                           = TC_boolean_u.TC_Boolean_Table.TC_Sun_Varying_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Orbit_Reference_Mode                       = TC_boolean_u.TC_Boolean_Table.TC_Orbit_Reference_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Asd_Node_Mode                              = TC_boolean_u.TC_Boolean_Table.TC_Asd_Node_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Des_Node_Mode                              = TC_boolean_u.TC_Boolean_Table.TC_Des_Node_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Station_Tracking_Mode                      = TC_boolean_u.TC_Boolean_Table.TC_Station_Tracking_Mode;
	TMTC_boolean_u.Boolean_Table.TC_QuestUpdate_Enable                         = TC_boolean_u.TC_Boolean_Table.TC_QuestUpdate_Enable;
	TMTC_boolean_u.Boolean_Table.TC_EKFControl_Enable                          = TC_boolean_u.TC_Boolean_Table.TC_EKFControl_Enable;
	TMTC_boolean_u.Boolean_Table.TC_2RW_Control_Mode                           = TC_boolean_u.TC_Boolean_Table.TC_2RW_Control_Mode; //Remove
	TMTC_boolean_u.Boolean_Table.rTC_mag_detumbling_mode_enable                = TC_boolean_u.TC_Boolean_Table.rTC_mag_detumbling_mode_enable; //Remove
	TMTC_boolean_u.Boolean_Table.rTC_gyro_detumbling_mode                      = TC_boolean_u.TC_Boolean_Table.rTC_gyro_detumbling_mode; //Remove
	TMTC_boolean_u.Boolean_Table.rTC_safe_mode                                 = TC_boolean_u.TC_Boolean_Table.rTC_safe_mode; //Remove
	TMTC_boolean_u.Boolean_Table.TC_detumbling_gyro_select_gnd                 = TC_boolean_u.TC_Boolean_Table.TC_detumbling_gyro_select_gnd; //Remove
	TMTC_boolean_u.Boolean_Table.ATTC_Master_enable_flag                       = TC_boolean_u.TC_Boolean_Table.ATTC_Master_enable_flag;
	TMTC_boolean_u.Boolean_Table.DTTC_exe_flag                                 = TC_boolean_u.TC_Boolean_Table.DTTC_exe_flag;
	TMTC_boolean_u.Boolean_Table.Reaction_wheel_1_speed_enable                 = TC_boolean_u.TC_Boolean_Table.Reaction_wheel_1_speed_enable ;
	TMTC_boolean_u.Boolean_Table.Reaction_wheel_2_speed_enable                 = TC_boolean_u.TC_Boolean_Table.Reaction_wheel_2_speed_enable ;
	TMTC_boolean_u.Boolean_Table.Reaction_wheel_3_speed_enable                 = TC_boolean_u.TC_Boolean_Table.Reaction_wheel_3_speed_enable ;
	TMTC_boolean_u.Boolean_Table.Reaction_wheel_4_speed_enable                 = TC_boolean_u.TC_Boolean_Table.Reaction_wheel_4_speed_enable ;
	TMTC_boolean_u.Boolean_Table.SunMagAD                                      = TC_boolean_u.TC_Boolean_Table.SunMagAD;
	TMTC_boolean_u.Boolean_Table.magAD                                         = TC_boolean_u.TC_Boolean_Table.magAD;
	TMTC_boolean_u.Boolean_Table.TC_EKF1_Enable                                = TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable;
	TMTC_boolean_u.Boolean_Table.TC_EKF2_Enable                                = TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable;
	TMTC_boolean_u.Boolean_Table.TC_Storage_TM_Enable_Disable                  = TC_boolean_u.TC_Boolean_Table.TC_Storage_TM_Enable_Disable;
	TMTC_boolean_u.Boolean_Table.TC_ST_mode                                    = TC_boolean_u.TC_Boolean_Table.TC_ST_mode;
	TMTC_boolean_u.Boolean_Table.TC_NormalStorage_Sampling_Rate_Select         = TC_boolean_u.TC_Boolean_Table.TC_NormalStorage_Sampling_Rate_Select;
	TMTC_boolean_u.Boolean_Table.pl_tx_tm_flag                                 = TC_boolean_u.TC_Boolean_Table.pl_tx_tm_flag;
	TMTC_boolean_u.Boolean_Table.TC_Storage_TM_Dump_enable_disable             = TC_boolean_u.TC_Boolean_Table.TC_Storage_TM_Dump_enable_disable;
	TMTC_boolean_u.Boolean_Table.TC_Storage_TM_Special_Normal_Mode_select      = TC_boolean_u.TC_Boolean_Table.TC_Storage_TM_Special_Normal_Mode_select;
	TMTC_boolean_u.Boolean_Table.RW_Speed_Negative                             = TC_boolean_u.TC_Boolean_Table.RW_Speed_Negative;
	TMTC_boolean_u.Boolean_Table.TC_Storage_TM_Full_Segment_Dump_mode_Select   = TC_boolean_u.TC_Boolean_Table.TC_Storage_TM_Full_Segment_Dump_mode_Select;
	TMTC_boolean_u.Boolean_Table.TC_TCH__Storage_TM_Dump_select                = TC_boolean_u.TC_Boolean_Table.TC_TCH__Storage_TM_Dump_select;
	TMTC_boolean_u.Boolean_Table.TC_sram_scrub_enable_disable                  = TC_boolean_u.TC_Boolean_Table.TC_sram_scrub_enable_disable;
	TMTC_boolean_u.Boolean_Table.NMI_test_enable                               = TC_boolean_u.TC_Boolean_Table.NMI_test_enable;
	TMTC_boolean_u.Boolean_Table.ST_dump_abort                                 = TC_boolean_u.TC_Boolean_Table.ST_dump_abort;
	TMTC_boolean_u.Boolean_Table.TC_TCH_Full_Segment_Dump_mode                 = TC_boolean_u.TC_Boolean_Table.TC_TCH_Full_Segment_Dump_mode;
	//added on 05-08-2020
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW1_enable			   = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW1_enable;
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW1_disable			   = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW1_disable; //Remove
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW2_enable			   = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW2_enable;
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW2_disable             = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW2_disable; //Remove
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW3_enable             = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW3_enable;
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW3_disable             = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW3_disable; //Remove
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW4_enable              = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW4_enable;
	TMTC_boolean_u.Boolean_Table.TC_wheel_index_ground_RW4_disable             = TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW4_disable; //Remove
	TMTC_boolean_u.Boolean_Table.TC_GND_Drift_Compensation_Enable_or_Disable   = TC_boolean_u.TC_Boolean_Table.TC_GND_Drift_Compensation_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_GND_MagBias_Compensation_Enable_or_Disable = TC_boolean_u.TC_Boolean_Table.TC_GND_MagBias_Compensation_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_H8Backup_H4_Main                           = TC_boolean_u.TC_Boolean_Table.TC_H8Backup_H4_Main;
	TMTC_boolean_u.Boolean_Table.TC_H8Backup_H4_Redt                           = TC_boolean_u.TC_Boolean_Table.TC_H8Backup_H4_Redt;
	TMTC_boolean_u.Boolean_Table.TC_H7Backup_H4_Main                           = TC_boolean_u.TC_Boolean_Table.TC_H7Backup_H4_Main;
	TMTC_boolean_u.Boolean_Table.TC_H7Backup_H4_Redt                           = TC_boolean_u.TC_Boolean_Table.TC_H7Backup_H4_Redt;
	TMTC_boolean_u.Boolean_Table.TC_MiddayOrbitFrame                           = TC_boolean_u.TC_Boolean_Table.TC_MiddayOrbitFrame;
	TMTC_boolean_u.Boolean_Table.TC_Sunlitdec_sensor_based                     = TC_boolean_u.TC_Boolean_Table.TC_Sunlitdec_sensor_based;
	TMTC_boolean_u.Boolean_Table.TC_Sunlitdec_Orbit_based                      = TC_boolean_u.TC_Boolean_Table.TC_Sunlitdec_Orbit_based;
	TMTC_boolean_u.Boolean_Table.TC_Sunlitdec_timer_based                      = TC_boolean_u.TC_Boolean_Table.TC_Sunlitdec_timer_based;
	TMTC_boolean_u.Boolean_Table.TC_BIST_override							   = TC_boolean_u.TC_Boolean_Table.TC_BIST_override;





	//Gain Select command TMTC Assignment
	TMTC_gain_select_u.gain_select_Table.TC_detumbling_bdot_gain               = TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain;
	TMTC_gain_select_u.gain_select_Table.TC_detumbling_rate_gain               = TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain;
	TMTC_gain_select_u.gain_select_Table.TC_BDOT_Det_Thresh                    = TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh;
	TMTC_gain_select_u.gain_select_Table.TC_GYRO_Det_Min_Thres                 = TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres;
	TMTC_gain_select_u.gain_select_Table.TC_W1_Commanded_Nominal_Speed         = TC_gain_select_u.TC_gain_select_Table.TC_W1_Commanded_Nominal_Speed;
	TMTC_gain_select_u.gain_select_Table.TC_W2_Commanded_Nominal_Speed         = TC_gain_select_u.TC_gain_select_Table.TC_W2_Commanded_Nominal_Speed;
	TMTC_gain_select_u.gain_select_Table.TC_W3_Commanded_Nominal_Speed         = TC_gain_select_u.TC_gain_select_Table.TC_W3_Commanded_Nominal_Speed;
	TMTC_gain_select_u.gain_select_Table.TC_W4_Commanded_Nominal_Speed         = TC_gain_select_u.TC_gain_select_Table.TC_W4_Commanded_Nominal_Speed;
	TMTC_gain_select_u.gain_select_Table.TC_momentum_dumping_gain              = TC_gain_select_u.TC_gain_select_Table.TC_momentum_dumping_gain;
	TMTC_gain_select_u.gain_select_Table.TC_PanelD_Status_Sel                  = TC_gain_select_u.TC_gain_select_Table.TC_PanelD_Status_Sel;
	TMTC_gain_select_u.gain_select_Table.TC_Gyro_LPF_Gain_IMU1                 = TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1;
	TMTC_gain_select_u.gain_select_Table.TC_Gyro_LPF_Gain_IMU2                 = TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2;
	TMTC_gain_select_u.gain_select_Table.TC_Mag_LPF_Gain_IMU1                  = TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1;
	TMTC_gain_select_u.gain_select_Table.TC_Mag_LPF_Gain_IMU2                  = TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2;
	TMTC_gain_select_u.gain_select_Table.TC_SS_Currents_LPF_Gain               = TC_gain_select_u.TC_gain_select_Table.TC_SS_Currents_LPF_Gain; //Remove
	TMTC_gain_select_u.gain_select_Table.TC_GPS_pulse_duration                 = TC_gain_select_u.TC_gain_select_Table.TC_GPS_pulse_duration;
	TMTC_gain_select_u.gain_select_Table.TC_KP                                 = TC_gain_select_u.TC_gain_select_Table.TC_KP;
	TMTC_gain_select_u.gain_select_Table.TC_KR                          	   = TC_gain_select_u.TC_gain_select_Table.TC_KR;
	/************************************************* Added on 26 JULY 2019 *****************************************************************/
	TMTC_gain_select_u.gain_select_Table.TC_GPS_Validity_Altitude_Threshold    = TC_gain_select_u.TC_gain_select_Table.TC_GPS_Validity_Altitude_Threshold; //Remove
	TMTC_gain_select_u.gain_select_Table.TC_Wheel_Cutoff_Threshold			   = TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold;
	TMTC_gain_select_u.gain_select_Table.TC_Wh_SpinUD_Thrsld				   = TC_gain_select_u.TC_gain_select_Table.TC_Wh_SpinUD_Thrsld; //Remove
	TMTC_gain_select_u.gain_select_Table.TC_comd_pitch_rate					   = TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate;
	TMTC_gain_select_u.gain_select_Table.TC_AngDev_SafeModetransit_Thrsld	   = TC_gain_select_u.TC_gain_select_Table.TC_AngDev_SafeModetransit_Thrsld; //Remove
	TMTC_gain_select_u.gain_select_Table.TC_AngMomDump_Thrsld				   = TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld; //Remove
	TMTC_gain_select_u.gain_select_Table.TC_SpeedDump_Thrsld				   = TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld;
	TMTC_gain_select_u.gain_select_Table.TC_SpeedDump_TimeSelect			   = TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect;
	TMTC_gain_select_u.gain_select_Table.TC_special_Sampling_rate_Select       = TC_gain_select_u.TC_gain_select_Table.TC_special_Sampling_rate_Select;
	TMTC_gain_select_u.gain_select_Table.TC_ST_Format_Selection                = TC_gain_select_u.TC_gain_select_Table.TC_ST_Format_Selection;


	//Assigning Boolean Telecommands to TMTC Buffer

	TM.Buffer.TM_TC_Buffer[0] = TMTC_boolean_u.TMTC_Buffer[0];
	TM.Buffer.TM_TC_Buffer[1] = TMTC_boolean_u.TMTC_Buffer[1];

	//Assigning Gain Select Telecommands to TMTC Buffer

	TM.Buffer.TM_TC_Buffer[2] = TMTC_gain_select_u.TMTC_Buffer[0];
	TM.Buffer.TM_TC_Buffer[3] = TMTC_gain_select_u.TMTC_Buffer[1];

	//Absolute TTC Count
	TM.Buffer.TM_TC_Buffer[4] = ATTC_count;

	//Diferential TTC Count
	TM.Buffer.TM_TC_Buffer[5] = DTTC_count;
	TM.Buffer.TM_TC_Buffer[6] = *((unsigned long int*)(&SWHW_STATUS));
}
void invertReverse(unsigned char * startAddr)
{
	int invert_index;
	//unsigned char temp;
	for (invert_index = 0; invert_index <=6; invert_index++)
    {
	   startAddr[invert_index] = startAddr[invert_index] ^ 0x01;
	}
/*	FPGA is reversing the parity & storing in Status register
	for (j=0; j <=2; j++)
    {
	   temp = startAddr[6-j] ;
	   startAddr[6-j] = startAddr[j];
	   startAddr[j] = temp;
	}
*/
}

/** BCHEncoder: Computes the parity & stores at TC_Rcvd.parity */
void BCHEncoder(unsigned long long int * TCAddr)
{
	unsigned long long int randSeq = 0xFF399E5A68e906;
	unsigned long long int randOut = 0;
	unsigned char x[7] = {0,0,0,0,0,0,0};
	unsigned char xp[7] = {0,0,0,0,0,0,0};
	unsigned long long int randOutShift = 0;
	unsigned long long int randOutShiftX = 0;
	unsigned char randOutBit = 0;
	char BCH_index = 0;
	char BCH_index_1 = 0;

	//compute randomized data
	randOut = ((*TCAddr) ^ randSeq);
	randOutShift = randOut << 8;
	randOutShiftX = randOutShift & 0x8000000000000000;
	randOutBit = (unsigned char) (randOutShiftX >> 63);

	for (BCH_index = 0; BCH_index < 56; BCH_index++)
	{
		x[0] = xp[6]^randOutBit;
		x[1] = xp[0];
		x[2] = xp[1]^x[0];
		x[3] = xp[2];
		x[4] = xp[3];
		x[5] = xp[4];
		x[6] = xp[5]^x[0];
        for (BCH_index_1 = 0; BCH_index_1 <= 6; BCH_index_1++)
		{
			xp[BCH_index_1] = x[BCH_index_1];
		}
		randOutShift = randOutShift << 1;
		randOutShiftX = randOutShift & 0x8000000000000000;
		randOutBit = (unsigned char) (randOutShiftX >> 63);
	}

	//invert and reverse the sequence
	invertReverse(x);

	TC_Rcvd.bits.bit7 = x[0];
	TC_Rcvd.bits.bit6 = x[1];
	TC_Rcvd.bits.bit5 = x[2];
	TC_Rcvd.bits.bit4 = x[3];
	TC_Rcvd.bits.bit3 = x[4];
	TC_Rcvd.bits.bit2 = x[5];
	TC_Rcvd.bits.bit1 = x[6];
	TC_Rcvd.bits.fillerBit = 0;
}


//Function Execute commands (Functions' definition)
void TC_IMU1_On()
{
	Out_Latch_2.IMU1_ON_OFF = 1;
	Out_Latch_2.IMU1_RESET = 1;
	IO_LATCH_REGISTER_2 = Out_Latch_2.data;
	REG32(IMU_1_STATUS_REGISTER_1) = IMU_ON;
}
void TC_IMU2_On()
{
	Out_Latch_2.IMU2_ON_OFF = 1;
	Out_Latch_2.IMU2_RESET = 1;
	IO_LATCH_REGISTER_2 =  Out_Latch_2.data;
	REG32(IMU_2_STATUS_REGISTER_1) = IMU_ON;
}
void TC_IMU1_Off()
{
	Out_Latch_2.IMU1_ON_OFF = 0;
	Out_Latch_2.IMU1_RESET = 1;
	IO_LATCH_REGISTER_2 =  Out_Latch_2.data;
	REG32(IMU_2_STATUS_REGISTER_1) = IMU_OFF;
}
void TC_IMU2_Off()
{
	Out_Latch_2.IMU2_ON_OFF = 0;
	Out_Latch_2.IMU2_RESET = 1;
	IO_LATCH_REGISTER_2 = Out_Latch_2.data;
	REG32(IMU_2_STATUS_REGISTER_1) = IMU_OFF;
}
void imu1_db_execute()
{
	//rIMU1_DB_Execute();
}
void imu2_db_execute()
{
	//rIMU2_DB_Execute();
}
void imu1_db_checksum()
{
//	imu1_db_checksum_obc = checksum_u32((unsigned long int *)&IMU_1_DB,80);
	//Add here for telemetry:	imu1_db_checksum_obc;
}
void imu2_db_checksum()
{
//	imu2_db_checksum_obc = checksum_u32((unsigned long int *)&IMU_2_DB,80);
	//Add here for telemetry:	imu2_db_checksum_obc;
}
void imu_test_sys_sel()
{
	return;
}
void TC_Drift_Uplink_Compenstation_Update_IMU1()
{
	TC_drift_compensation_IMU1[0] = ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU1[0];
	TC_drift_compensation_IMU1[1] = ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU1[1];
	TC_drift_compensation_IMU1[2] = ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU1[2];
}
void TC_Drift_Uplink_Compenstation_Update_IMU2()
{
	TC_drift_compensation_IMU2[0] = ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU2[0];
	TC_drift_compensation_IMU2[1] = ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU2[1];
	TC_drift_compensation_IMU2[2] = ADCS_TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU2[2];
}
void TC_Gyro_Misalignment_Update_IMU1()
{
	IMU1_Corr.DB_w_MisCor[0][0] = 1.0;
	IMU1_Corr.DB_w_MisCor[0][1] = (double)TC_w_miscor_IMU1[0];
	IMU1_Corr.DB_w_MisCor[0][2] = (double)TC_w_miscor_IMU1[1];
	IMU1_Corr.DB_w_MisCor[1][0] = (double)TC_w_miscor_IMU1[2];
	IMU1_Corr.DB_w_MisCor[1][1] = 1.0;
	IMU1_Corr.DB_w_MisCor[1][2] = (double)TC_w_miscor_IMU1[3];
	IMU1_Corr.DB_w_MisCor[2][0] = (double)TC_w_miscor_IMU1[4];
	IMU1_Corr.DB_w_MisCor[2][1] = (double)TC_w_miscor_IMU1[5];
	IMU1_Corr.DB_w_MisCor[2][2] = 1.0;
}
void TC_Gyro_Misalignment_Update_IMU2()
{
	IMU2_Corr.DB_w_MisCor[0][0] = 1.0;
	IMU2_Corr.DB_w_MisCor[0][1] = (double)TC_w_miscor_IMU2[0];
	IMU2_Corr.DB_w_MisCor[0][2] = (double)TC_w_miscor_IMU2[1];
	IMU2_Corr.DB_w_MisCor[1][0] = (double)TC_w_miscor_IMU2[2];
	IMU2_Corr.DB_w_MisCor[1][1] = 1.0;
	IMU2_Corr.DB_w_MisCor[1][2] = (double)TC_w_miscor_IMU2[3];
	IMU2_Corr.DB_w_MisCor[2][0] = (double)TC_w_miscor_IMU2[4];
	IMU2_Corr.DB_w_MisCor[2][1] = (double)TC_w_miscor_IMU2[5];
	IMU2_Corr.DB_w_MisCor[2][2] = 1.0;
}
void TC_Gyro_Scale_Factor_Update_IMU1()
{
	IMU1_Corr.DB_wSFC_Neg[0] = (double)TC_w_sf_IMU1[0];
	IMU1_Corr.DB_wSFC_Neg[1] = (double)TC_w_sf_IMU1[1];
	IMU1_Corr.DB_wSFC_Neg[2] = (double)TC_w_sf_IMU1[2];

	IMU1_Corr.DB_wSFC_Pos[0] = (double)TC_w_sf_IMU1[0];
	IMU1_Corr.DB_wSFC_Pos[1] = (double)TC_w_sf_IMU1[1];
	IMU1_Corr.DB_wSFC_Pos[2] = (double)TC_w_sf_IMU1[2];
}
void TC_Gyro_Scale_Factor_Update_IMU2()
{
	IMU2_Corr.DB_wSFC_Neg[0] = (double)TC_w_sf_IMU2[0];
	IMU2_Corr.DB_wSFC_Neg[1] = (double)TC_w_sf_IMU2[1];
	IMU2_Corr.DB_wSFC_Neg[2] = (double)TC_w_sf_IMU2[2];

	IMU2_Corr.DB_wSFC_Pos[0] = (double)TC_w_sf_IMU2[0];
	IMU2_Corr.DB_wSFC_Pos[1] = (double)TC_w_sf_IMU2[1];
	IMU2_Corr.DB_wSFC_Pos[2] = (double)TC_w_sf_IMU2[2];
}
void TC_MagBias_Uplink_Compenstation_Update_IMU1()
{
	TC_magbias_compensation_IMU1[0] = ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU1[0];
	TC_magbias_compensation_IMU1[1] = ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU1[1];
	TC_magbias_compensation_IMU1[2] = ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU1[2];
}
void TC_MagBias_Uplink_Compenstation_Update_IMU2()
{
	TC_magbias_compensation_IMU2[0] = ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU2[0];
	TC_magbias_compensation_IMU2[1] = ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU2[1];
	TC_magbias_compensation_IMU2[2] = ADCS_TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU2[2];
}
void TC_Mag_Misalignment_Update_IMU1()
{
	//to be done
}
void TC_Mag_Misalignment_Update_IMU2()
{
	//to be done
}
void TC_Mag_Scale_Factor_Update_IMU1()
{
	//to be done
}
void TC_Mag_Scale_Factor_Update_IMU2()
{
	//to be done
}


void rSSmain_ImaxF_update()
{
	SS_Main_2Exe_DB.DB_Imax_RPD = (double)TC_SSmain_ImaxF[0];
	SS_Main_2Exe_DB.DB_Imax_RND = (double)TC_SSmain_ImaxF[1];
	SS_Main_2Exe_DB.DB_Imax_RPND = (double)TC_SSmain_ImaxF[2];
	SS_Main_2Exe_DB.DB_Imax_RNND = (double)TC_SSmain_ImaxF[3];
	SS_Main_2Exe_DB.DB_Imax_PP = (double)TC_SSmain_ImaxF[4];
	SS_Main_2Exe_DB.DB_Imax_PN = (double)TC_SSmain_ImaxF[5];
	SS_Main_2Exe_DB.DB_Imax_YP = (double)TC_SSmain_ImaxF[6];
	SS_Main_2Exe_DB.DB_Imax_YN = (double)TC_SSmain_ImaxF[7];
}
void rSSredt_ImaxF_update()
{
	SS_Redundant_2Exe_DB.DB_Imax_RPD = (double)TC_SSredt_ImaxF[0];
	SS_Redundant_2Exe_DB.DB_Imax_RND = (double)TC_SSredt_ImaxF[0];
	SS_Redundant_2Exe_DB.DB_Imax_RPND = (double)TC_SSredt_ImaxF[2];
	SS_Redundant_2Exe_DB.DB_Imax_RNND = (double)TC_SSredt_ImaxF[3];
	SS_Redundant_2Exe_DB.DB_Imax_PP	= (double)TC_SSredt_ImaxF[4];
	SS_Redundant_2Exe_DB.DB_Imax_PN = (double)TC_SSredt_ImaxF[5];
	SS_Redundant_2Exe_DB.DB_Imax_YP = (double)TC_SSredt_ImaxF[6];
	SS_Redundant_2Exe_DB.DB_Imax_YN = (double)TC_SSredt_ImaxF[7];
}
void TC_Gyro_LPF_Gain_Update_IMU1()
{

}
void TC_Gyro_LPF_Gain_Update_IMU2()
{

}


void TC_ACC_Ang_RESET()
{
	rIMU_Angle_Reset();
}
/*the TC_Panel1_Deploy() and TC_Panel2_Deploy() is now
SA_DEPLOYMENT_MECHANISM_MAIN_ON and SA_DEPLOYMENT_MECHANISM_RED_ON*/
/*void TC_Panel1_Deploy()
{
	//rHAL_SA_MAIN_Deploy_on(Deploy);
}*/

// clear NMI counters
void TC_NMI_count_reset()
{
	NMI_fail_count = 0;
	FDI_NMI_Count = 0;
}
void rMag_Refeci_update()
{
	B_ECI[0] = (double)TC_B_ECI[0];
	B_ECI[1] = (double)TC_B_ECI[1];
	B_ECI[2] = (double)TC_B_ECI[2];

	B_ECItesla[0] = B_ECI[0]*1.0E-9;
	B_ECItesla[1] = B_ECI[1]*1.0E-9;
	B_ECItesla[2] = B_ECI[2]*1.0E-9;

	rVectorNorm(B_ECI);
	B_ECIn[0] = Norm_out[0];
	B_ECIn[1] = Norm_out[1];
	B_ECIn[2] = Norm_out[2];
}
void TC_GPS1_ON()
{
	Out_Latch_2.GPS1_ON_OFF = 1;
	Out_Latch_2.GPS1_RESET = 1;
	IO_LATCH_REGISTER_2 = Out_Latch_2.data;
	//rHAL_GPS_POWER(GPS_1,ON);
	return;
}
void TC_GPS1_OFF()
{
	Out_Latch_2.GPS1_ON_OFF = 0;
	Out_Latch_2.GPS1_RESET = 1;
	IO_LATCH_REGISTER_2 = Out_Latch_2.data;
	//rHAL_GPS_POWER(GPS_1,OFF);
	return;
}
void TC_GPS1_NMEA_VTG_enable()
{
	rHAL_GPS_Config(GPS1,NMEA_VTG_Enable);
	return;
}
void TC_GPS1_NMEA_VTG_disable()
{
	rHAL_GPS_Config(GPS1,NMEA_VTG_Disable);
	return;
}
void TC_GPS1_NMEA_GGA_enable()
{
	rHAL_GPS_Config(GPS1,NMEA_GGA_Enable);
	return;
}
void TC_GPS1_NMEA_GGA_disable()
{
	rHAL_GPS_Config(GPS1,NMEA_GGA_Disable);
	return;
}
void TC_GPS1_NMEA_GSA_enable()
{
	rHAL_GPS_Config(GPS1,NMEA_GSA_Enable);
	return;
}
void TC_GPS1_NMEA_GSA_disable()
{
	rHAL_GPS_Config(GPS1,NMEA_GSA_Disable);
	return;
}
void TC_GPS1_cold_start()
{
	rHAL_GPS_Config(GPS1,COLD_START);
	return;
}
void TC_GPS1_factory_reset()
{
	rHAL_GPS_Config(GPS1,FACTORY_RESET);
	return;
}
void TC_GPS2_on()
{
	Out_Latch_2.GPS2_ON_OFF = 1;
	Out_Latch_2.GPS2_RESET = 1;
	IO_LATCH_REGISTER_2 = Out_Latch_2.data;
	//rHAL_GPS_POWER(GPS_2,ON);
	return;
}
void TC_GPS2_off()
{
	Out_Latch_2.GPS2_ON_OFF = 0;
	Out_Latch_2.GPS2_RESET = 1;
	IO_LATCH_REGISTER_2 = Out_Latch_2.data;
	//rHAL_GPS_POWER(GPS_2,OFF);
	return;
}
void TC_GPS2_NMEA_VTG_enable()
{
	rHAL_GPS_Config(GPS2,NMEA_VTG_Enable);
	return;
}
void TC_GPS2_NMEA_VTG_disable()
{
	rHAL_GPS_Config(GPS2,NMEA_VTG_Disable);
	return;
}
void TC_GPS2_NMEA_GGA_enable()
{
	rHAL_GPS_Config(GPS2,NMEA_GGA_Enable);
	return;
}
void TC_GPS2_NMEA_GGA_disable()
{
	rHAL_GPS_Config(GPS2,NMEA_GGA_Disable);
	return;
}
void TC_GPS2_NMEA_GSA_enable()
{
	rHAL_GPS_Config(GPS2,NMEA_GSA_Enable);
	return;
}
void TC_GPS2_NMEA_GSA_disable()
{
	rHAL_GPS_Config(GPS2,NMEA_GSA_Disable);
	return;
}
void TC_GPS2_cold_start()
{
	rHAL_GPS_Config(GPS2,COLD_START);
	return;
}
void TC_GPS2_factory_reset()
{
	rHAL_GPS_Config(GPS2,FACTORY_RESET);
	return;
}
void TC_W1_ON()
{
	Out_Latch_3.RW1_ON_OFF = 1;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_W2_ON()
{
	Out_Latch_3.RW2_ON_OFF = 1;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_W3_ON()
{
	Out_Latch_3.RW3_ON_OFF = 1;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_W4_ON()
{
	Out_Latch_3.RW4_ON_OFF = 1;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_W1_OFF()
{
	Out_Latch_3.RW1_ON_OFF = 0;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_W2_OFF()
{
	Out_Latch_3.RW2_ON_OFF = 0;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_W3_OFF()
{
	Out_Latch_3.RW3_ON_OFF = 0;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_W4_OFF()
{
	Out_Latch_3.RW4_ON_OFF = 0;
	IO_LATCH_REGISTER_3 = Out_Latch_3.data;

}
void TC_Nominal_wheel_speed_execute()
{
	//to be done
}
void TC_MTR_ON()
{
	rHAL_MTR_ON();
	return;
}
void Pitch_Torquer_ON()
{
	//
}
void Yaw_Torquer_ON()
{
	//
}
void TC_MTR_OFF()
{
	rHAL_MTR_OFF();
	return;
}
void Pitch_Torquer_OFF()
{
	//
}
void Yaw_Torquer_OFF()
{
	//
}

void rTC_Detumbling_ModePreprocessing_BDOT()
{
	Spacecraft_Mode = Detumbling_ModePreprocessing_BDOT;
}
void rTC_Detumbling_ModePreprocessing_GYRO()
{
	Spacecraft_Mode = Detumbling_ModePreprocessing_GYRO;
}
void rTC_SunAcquisition_ModePreprocessing()
{
	Spacecraft_Mode = SunAcquisition_ModePreprocessing;
}
void rTC_ThreeAxis_ModePreprocessing()
{
	Spacecraft_Mode = ThreeAxis_ModePreprocessing;
}

void rTC_Suspended_ModePreprocessing()
{
	Spacecraft_Mode = Suspended_ModePreprocessing;
}

void payload_1_on()
{
	if (!load_shedding_flag)
		rHAL_pl1_ON();
	return;
}
void payload_1_off()
{
	rHAL_pl1_OFF();
	return;
}
void rHeater1_on()
{
	//rHAL_Heater_Power(HEATER_1,ON);
	return;
	/*rHAL_Heater_Power(ON);
	return;*/
}
void rHeater1_off()
{
    //rHAL_Heater_Power(HEATER_1,OFF);
	return;
			/*	rHAL_Heater_Power(OFF);
	return;;*/
}
void rHeater2_on()
{
	//rHAL_Heater_Power(HEATER_2,ON);
	return;
	/*rHAL_Heater_Power(ON);
	return;*/
}
void rHeater2_off()
{
	//rHAL_Heater_Power(HEATER_2,OFF);
	return;
	/*rHAL_Heater_Power(OFF);
	return;*/
}
void rHeater3_on()
{
	//rHAL_Heater_Power(HEATER_3,ON);
	return;
	/*rHAL_Heater_Power(ON);
	return;*/
}
void rHeater3_off()
{
	//rHAL_Heater_Power(HEATER_3,OFF);
	return;
	/*rHAL_Heater_Power(OFF);
	return;*/
}
void rHeater4_on()
{
	//rHAL_Heater_Power(HEATER_4,ON);
	return;
	/*rHAL_Heater_Power(ON);
	return;*/
}
void rHeater4_off()
{
	//rHAL_Heater_Power(HEATER_4,OFF);
	return;
	/*rHAL_Heater_Power(OFF);
	return;*/
}
void rHeater5_on()
{
	//rHAL_Heater_Power(HEATER_5,ON);
	return;
	/*rHAL_Heater_Power(ON);
	return;*/
}
void rHeater5_off()
{
	//rHAL_Heater_Power(HEATER_5,OFF);
	return;
	/*rHAL_Heater_Power(OFF);
	return*/;
}
void rHeater6_on()
{
	//rHAL_Heater_Power(HEATER_6,ON);
	return;
	/*rHAL_Heater_Power(ON);
	return;*/
}
void rHeater6_off()
{
	//rHAL_Heater_Power(HEATER_6,OFF);
	return;
	/*rHAL_Heater_Power(OFF);
	return*/;
}
void RX_TX_deployed()
{
	return;
}
void PL_K_CMD_STS()
{
	rHAL_pl_sts_check();
}
void PL_K_CMD_ACQ()
{
	rHAL_pl_cmd_acq();
}
void PL_K_CMD_HLT()
{
	REG32(PAYLOAD_CONFIG_REGISTER) = 0x00004855;
	/*****************************************************/
	/* No of Configure Bytes (15:7) : 2 bytes
	 * No of Receive Bytes (6:1)    : 15 byte
	 * Configure Bytes  (0)         : 1
	 *****************************************************/
	REG32(PAYLOAD_STATUS2_ADDRESS) = 0x0000011F;  	// 0000_0001_0001_1111
	pl_cmd_id = 3;
}
void PL_K_DIAG()
{
	rHAL_pl_diag();
}

void PL_K_CMD_OFF()
{

	rHAL_pl_x_tx_data_off();
}
void PL_K_CMD_ON()
{
	rHAL_pl_x_tx_data_on();
}
void rTC_HILS_MODE_IDLE()
{
	hils_mode_select = 1;
}
void rTC_HILS_MODE_START()
{
	hils_mode_select = 2;
}
void rTC_HILS_MODE_STOP()
{
	hils_mode_select = 3;
}
void rSun_Ephemeris_update()
{
	S_ECI[0] = (double)TC_S_ECI[0];
	S_ECI[1] = (double)TC_S_ECI[1];
	S_ECI[2] = (double)TC_S_ECI[2];

	rVectorNorm(S_ECI);
	S_ECIn[0] = Norm_out[0];
	S_ECIn[1] = Norm_out[1];
	S_ECIn[2] = Norm_out[2];
}

/*void rTLE_Update()
{
	long long int TLE_E_DAY_temp1;
	long long int  no_TLE_temp1;

	Epochyear_TLE_tc = (int)TC_TLE_data[0];

	TLE_E_DAY_temp1 = ((int)(TC_TLE_data[1]) & 0xFFFFFFFF);
	epochdays_TLE_tc = (double)(((TLE_E_DAY_temp1 << 32) | (int)TC_TLE_data[2]) & 0xFFFFFFFFFFFFFFFF);

	ibexp_TLE_tc = (double)TC_TLE_data[3];
	bstar_TLE_tc = (double)TC_TLE_data[4];
	inclination_TLE_tc = (double)TC_TLE_data[5];
	nodeo_TLE_tc = (double)TC_TLE_data[6];
	ecc_TLE_tc = (double)TC_TLE_data[7];
	argpo_TLE_tc = (double)TC_TLE_data[8];
	mo_TLE_tc = (double)TC_TLE_data[9];

	no_TLE_temp1 = ((int)(TC_TLE_data[10]) & 0xFFFFFFFF);
	no_TLE_tc = (double)(((no_TLE_temp1 << 32) | (int)TC_TLE_data[11]) & 0xFFFFFFFFFFFFFFFF);
	Tsince_TLE_tc = (double)TC_TLE_data[12];
}*/

unsigned char TLE_chksum;
void rTLE_Update()
{
	unsigned char* TLE_ptr;
	TLE_ptr = (unsigned char*)&TLE_data;

	Epochyear_TLE_tc = (int)TLE_data.TC_TLE_data1;
	epochdays_TLE_tc = TLE_data.TC_TLE_data2;
	ibexp_TLE_tc = (double)TLE_data.TC_TLE_data3;
	bstar_TLE_tc = (double)TLE_data.TC_TLE_data4;
	inclination_TLE_tc = (double)TLE_data.TC_TLE_data5;
	nodeo_TLE_tc = (double)TLE_data.TC_TLE_data6;
	ecc_TLE_tc = TLE_data.TC_TLE_data7;
	argpo_TLE_tc = (double)TLE_data.TC_TLE_data8;
	mo_TLE_tc = (double)TLE_data.TC_TLE_data9;
	no_TLE_tc = TLE_data.TC_TLE_data10;
	OBT_at_TLE_epoch = TLE_data.TC_TLE_data11;
	chksum_tle = TLE_data.TC_TLE_data12;

	TLE_chksum = chksum8(TLE_ptr,56);
	if(chksum_tle == TLE_chksum)
	{
		TLE_Data_Available = 1;
	}
	else
	{
		TLE_Data_Available = 0;
	}

}


void TC_init_RW1()
{
	RW_Init();
	rRW_init_cmd(RW_1, 0);

}

void TC_init_RW2()
{
	RW_Init();
	rRW_init_cmd(RW_2, 1);
}

void TC_init_RW3()
{
	RW_Init();
	rRW_init_cmd(RW_3, 2);
}

void TC_init_RW4()
{
	RW_Init();
	rRW_init_cmd(RW_4, 3);
}
void TC_MTR_Yaw_Positive()
{
	rHAL_MTR_TC(Yaw, Positive);
}
void TC_MTR_Yaw_Negative()
{
	rHAL_MTR_TC(Yaw, Negative);
}
void TC_MTR_Pitch_Positive()
{
	rHAL_MTR_TC(Pitch, Positive);

}
void TC_MTR_Pitch_Negative()
{
	rHAL_MTR_TC(Pitch, Negative);

}
void TC_MTR_Roll_Positive()
{

	rHAL_MTR_TC(Roll, Positive);
}
void TC_MTR_Roll_Negative()
{
	rHAL_MTR_TC(Roll, Negative);

}
void rDifferentialTTC_Execute()
{
	rDifferential_TimeTag_Execute();
}
void OBC_ON()
{
	//
}
void OBC_OFF()
{
	//
}
void Antenna_mechanism_ON()
{
	Out_latch_5.ANTENNA_DEPLOY = 1;
	IO_LATCH_REGISTER_5 = Out_latch_5.data;
}
void Antenna_mechanism_OFF()
{
	Out_latch_5.ANTENNA_DEPLOY = 0;
	IO_LATCH_REGISTER_5 = Out_latch_5.data;
}
void payload_2_on()
{
	if (!load_shedding_flag)
		rHAL_pl2_ON();
	return;
}
void payload_2_off()
{
	rHAL_pl2_OFF();
	return;
}
void S_band_tx_on()
{
	Out_latch_6.RF_Tx_ON_OFF = 1;
	IO_LATCH_REGISTER_6 = Out_latch_6.data;
}
void S_band_tx_off()
{
	Out_latch_6.RF_Tx_ON_OFF = 0;
	IO_LATCH_REGISTER_6 = Out_latch_6.data;
}
 void X_band_tx_on()
{
	 if (!load_shedding_flag)
		 rHAL_X_Tx_ON();
}
 void X_band_tx_off()
{
	 rHAL_X_Tx_OFF();
}
void SA_DEPLOYMENT_MECHANISM_MAIN_ON()
{
	rHAL_SA_MAIN_Deploy_on();
}
void SA_DEPLOYMENT_MECHANISM_MAIN_OFF()
{
	uint32 tempdata;
	Out_latch_5.SA1_DEPLOY = 0;
	tempdata = Out_latch_5.data;
	IO_LATCH_REGISTER_5;
	IO_LATCH_REGISTER_5 = tempdata;
}
void SA_DEPLOYMENT_MECHANISM_RED_ON()
{
 	 rHAL_SA_RED_Deploy_on();
}
void SA_DEPLOYMENT_MECHANISM_RED_OFF()
{
	uint32 tempdata;
	Out_latch_5.SPARE1_ON_OFF = 0;
	tempdata = Out_latch_5.data;
	IO_LATCH_REGISTER_5;
	IO_LATCH_REGISTER_5 = tempdata;
}
void SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_ON()
{
	  //
}
void SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_OFF()
{
  	  //
}
void Test_ON()
{
	//
}
void Test_OFF()
{
   //
}
void Antenna_mechanism_arm()
{
	antennaCommand1();
}
void Antenna_deploy()
{
	antennaCommand2();
}
void Antenna_mechanism_disarm()
{
	antennaCommand3();
}
void sunlit()
{
	f_Sunlit_Presence = 1;
}
void eclipse()
{
	f_Sunlit_Presence = 0;
}
void Sunlit_eclipse_both()
{
	CB_Sl_Ecl_OnBrd_detection = Enable;
}

void rSafe_mode_PreProcessing()
{
	Spacecraft_Mode = Safe_ModePreprocessing;
}

void TC_MTR_Roll_No_cuurent()                 // TC for No_current as input to MTR(roll)
{
	rHAL_MTR_TC(Roll, No_Current);
}
void TC_MTR_Pitch_No_cuurent()                // TC for No_current as input to MTR(pitch)
{
	rHAL_MTR_TC(Pitch, No_Current);
}
void TC_MTR_Yaw_No_cuurent() 				 // TC for No_current as input to MTR(yaw)
{
	rHAL_MTR_TC(Yaw, No_Current);
}
void TC_pl_debug()
{
	rHAL_pl_debug();
}
void TC_pl_tm()
{
	//rTC_pl_tx_tm();
}
void TC_TM_DS_EN()
{
	rHAL_tm_ds_en();
}
void Antenna_RESET_command()
{
	antennaCommand4();
}

void Antenna_deploy_with_override()
{
	antennaCommand5();
}

void Antenna_system_temp()
{
	antennaCommand6();
}

void Antenna_deploy_status_report()
{
	antennaCommand7();
}

void Antenna_deploy_activation_count()
{
	antennaCommand8();
}

void Antenna_deploy_activation_time()
{
	antennaCommand9();
}
//function execute TC for eeprom enable
void eeprom_en()
{
	// Manuplate the I/O pin based on the requirement
	eeprom_flag = 1;

}

//function execute TC for eeprom disable
void eeprom_dis()
{
	// Manuplate the I/O pin based on the requirement
	eeprom_flag = 0;
}

void TC_GPS1_NMEA_NMEA_GSV_Enable()
{
	rHAL_GPS_Config(GPS1,NMEA_GSV_Enable);
	return;
}
void TC_GPS1_NMEA_NMEA_GSV_disable()
{
	rHAL_GPS_Config(GPS1,NMEA_GSV_Disable);
	return;
}
void TC_GPS2_NMEA_NMEA_GSV_Enable()
{
	rHAL_GPS_Config(GPS2,NMEA_GSV_Enable);
	return;
}
void TC_GPS2_NMEA_NMEA_GSV_disable()
{
	rHAL_GPS_Config(GPS2,NMEA_GSV_Disable);
	return;
}

void TC_ATTC_CMD_clear()
{
	rAbsoluteTTC_Clear();
}

void rTC_ref_snv_bias_q_update()
{
	Q_svn_off[0] = sin(ADCS_TC_data_command_Table.TC_ref_svn_bias_off_deg*c_D2R*0.5);
	Q_svn_off[1] = 0.0;
	Q_svn_off[2] = 0.0;
	Q_svn_off[3] = cos(ADCS_TC_data_command_Table.TC_ref_svn_bias_off_deg*c_D2R*0.5);

	rQs_Normalization(Q_svn_off);
	Q_svn_off[0] = out_Quat_norm[0];
	Q_svn_off[1] = out_Quat_norm[1];
	Q_svn_off[2] = out_Quat_norm[2];
	Q_svn_off[3] = out_Quat_norm[3];
}

void rTC_ref_stn_bias_q_update()
{
	Q_stn_off[0] = ADCS_TC_data_command_Table.TC_q_command_0;
	Q_stn_off[1] = ADCS_TC_data_command_Table.TC_q_command_1;
	Q_stn_off[2] = ADCS_TC_data_command_Table.TC_q_command_2;
	Q_stn_off[3] = ADCS_TC_data_command_Table.TC_q_command_3;

	rQs_Normalization(Q_stn_off);
	Q_stn_off[0] = out_Quat_norm[0];
	Q_stn_off[1] = out_Quat_norm[1];
	Q_stn_off[2] = out_Quat_norm[2];
	Q_stn_off[3] = out_Quat_norm[3];
}

void rTC_ref_q_gnd_update()
{
	Q_REF_GND[0] = ADCS_TC_data_command_Table.TC_q_command_0;
	Q_REF_GND[1] = ADCS_TC_data_command_Table.TC_q_command_1;
	Q_REF_GND[2] = ADCS_TC_data_command_Table.TC_q_command_2;
	Q_REF_GND[3] = ADCS_TC_data_command_Table.TC_q_command_3;

	rQs_Normalization(Q_REF_GND);
	Q_REF_GND[0] = out_Quat_norm[0];
	Q_REF_GND[1] = out_Quat_norm[1];
	Q_REF_GND[2] = out_Quat_norm[2];
	Q_REF_GND[3] = out_Quat_norm[3];
}

void TC_q_body_init()
{
	Qbody[0] = ADCS_TC_data_command_Table.TC_q_command_0;
	Qbody[1] = ADCS_TC_data_command_Table.TC_q_command_1;
	Qbody[2] = ADCS_TC_data_command_Table.TC_q_command_2;
	Qbody[3] = ADCS_TC_data_command_Table.TC_q_command_3;

	rQs_Normalization(Qbody);
	Qbody[0] = out_Quat_norm[0];
	Qbody[1] = out_Quat_norm[1];
	Qbody[2] = out_Quat_norm[2];
	Qbody[3] = out_Quat_norm[3];
}
	
void rElapsedTimerAssign()
{
	elapsed_running_timer = ADCS_TC_data_command_Table.TC_elapsed_orbitTimer;
}

void rTC_HILS_ENABLE()
{
	HILS_mode_enable();
}
void rTC_HILS_DISABLE()
{
	HILS_mode_disable();
}

void adcsgains()
{
	switch(u_TC.GainSelect.offset_addr)
		{
		case 0: TC_detumbling_bdot_gain_1();
		        break;
		case 1: TC_detumbling_rate_gain_1();
		        break;
		case 2: TC_BDOT_Det_Thresh_1();
			 break;
		case 3: TC_GYRO_Det_Min_Thres_1();
			break;
		case 4: rTc_nominal_speed_RW1();
					break;
		case 5:rTc_nominal_speed_RW2();
					break;
		case 6:rTc_nominal_speed_RW3();
					break;
		case 7:rTc_nominal_speed_RW4();
					break;
		case 23: TC_AngMomDump_Thrsld_1();
			break;
		case 24: TC_SpeedDump_Thrsld_1();
			break;
		case 25: TC_SpeedDump_TimeSelect_1();
			break;
		case 21: TC_comd_pitch_rate_1();
			break;
		case 10: TC_Gyro_LPF_Gain_IMU1_1();
			break;
		case 11: TC_Gyro_LPF_Gain_IMU2_1();
			break;
		case 12: TC_Mag_LPF_Gain_IMU1_1();
			break;
		case 13: TC_Mag_LPF_Gain_IMU2_1();
			break;
		case 19: TC_Wheel_Cutoff_Threshold_1();
			break;
		}
}

void TC_detumbling_rate_gain_1()
{
	if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain == 01)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_01;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_01;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_01;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain == 02)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_10;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_10;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_10;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain==03)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_11;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_11;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_11;
	}
	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain==00)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_00;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_00;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_00;
	}
	else
	{
		//
	}
}
	//
void TC_detumbling_bdot_gain_1()
{
	if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==01)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_01;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_01;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_01;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==02)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_10;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_10;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_10;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==03)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_11;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_11;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_11;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==00)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_00;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_00;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_00;
	}
	else
	{
		//
	}
}
//
void TC_BDOT_Det_Thresh_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==01)

	{
	  TC_BDOT_Det_Thresh = GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_01;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==02)
	{
	  TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_10;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==03)
	{
	 TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==00)
	{
	 TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_00;
	}

	else
	{
		//
	}
}
	//
void TC_GYRO_Det_Min_Thres_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==01)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_01;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==02)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_10;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==03)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==00)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_00;
	}
	else
	{
		//
	}
}	 //

void rTc_nominal_speed_RW1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_W1_Commanded_Nominal_Speed == 01)
	{
		TC_RW_Nominal[0] = GAIN_DATA_SET.Tc_nominal_speed_rw1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W1_Commanded_Nominal_Speed==02)
	{
		TC_RW_Nominal[0] =GAIN_DATA_SET.Tc_nominal_speed_rw1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W1_Commanded_Nominal_Speed==03)
	{
		TC_RW_Nominal[0] =GAIN_DATA_SET.Tc_nominal_speed_rw1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_W1_Commanded_Nominal_Speed==00)
	{
		TC_RW_Nominal[0] =GAIN_DATA_SET.Tc_nominal_speed_rw1_00;
	}
	else
	{
		//
	}
}

void rTc_nominal_speed_RW2()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_W2_Commanded_Nominal_Speed == 1)
	{
		TC_RW_Nominal[1] = GAIN_DATA_SET.Tc_nominal_speed_rw1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W2_Commanded_Nominal_Speed==02)
	{
		TC_RW_Nominal[1]=GAIN_DATA_SET.Tc_nominal_speed_rw2_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W2_Commanded_Nominal_Speed==03)
	{
		TC_RW_Nominal[1]=GAIN_DATA_SET.Tc_nominal_speed_rw2_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_W2_Commanded_Nominal_Speed==00)
	{
		TC_RW_Nominal[1]=GAIN_DATA_SET.Tc_nominal_speed_rw2_00;
	}
	else
	{
		//
	}
}

void rTc_nominal_speed_RW3()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_W3_Commanded_Nominal_Speed == 1)
	{
		TC_RW_Nominal[2] = GAIN_DATA_SET.Tc_nominal_speed_rw3_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W3_Commanded_Nominal_Speed==02)
	{
		TC_RW_Nominal[2]=GAIN_DATA_SET.Tc_nominal_speed_rw3_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W3_Commanded_Nominal_Speed==03)
	{
		TC_RW_Nominal[2]=GAIN_DATA_SET.Tc_nominal_speed_rw3_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_W3_Commanded_Nominal_Speed==00)
	{
		TC_RW_Nominal[2]=GAIN_DATA_SET.Tc_nominal_speed_rw3_00;
	}
	else
	{
		//
	}
}

void rTc_nominal_speed_RW4()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_W4_Commanded_Nominal_Speed == 1)
	{
		TC_RW_Nominal[3] = GAIN_DATA_SET.Tc_nominal_speed_rw4_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W4_Commanded_Nominal_Speed == 2)
	{
		TC_RW_Nominal[3] =GAIN_DATA_SET.Tc_nominal_speed_rw4_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_W4_Commanded_Nominal_Speed== 3)
	{
		TC_RW_Nominal[3] =GAIN_DATA_SET.Tc_nominal_speed_rw4_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_W4_Commanded_Nominal_Speed== 0)
	{
		TC_RW_Nominal[3] =GAIN_DATA_SET.Tc_nominal_speed_rw4_00;
	}
	else
	{
		//
	}
}



void TC_AngMomDump_Thrsld_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==01)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==02)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==03)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==00)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_00;
	}
	else
	{
		//
	}
}	//
void TC_SpeedDump_Thrsld_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==01)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==02)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==03)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==00)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_00;
	}
	else
	{
		//
	}
}	//
void TC_SpeedDump_TimeSelect_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==01)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==02)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==03)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==00)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_00;
	}
	else
	{
		//
	}
}
//
void TC_comd_pitch_rate_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==01)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==02)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==03)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==00)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_00;
	}

	else
	{
		//
	}
}
//
void TC_Gyro_LPF_Gain_IMU1_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==01)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_01;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==02)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_10;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==03)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_11;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==00)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_00;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_00;
	}
	else
	{
		//
	}
}
	//
void TC_Gyro_LPF_Gain_IMU2_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==01)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_01;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==02)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_10;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==03)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_11;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==00)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_00;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_00;
	}
	else
	{
		//
	}
}
	//
void TC_Mag_LPF_Gain_IMU1_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==01)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_01;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==02)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_10;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==03)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_11;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==00)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_00;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_00;
	}
	else
	{
		//
	}
}
//
void TC_Mag_LPF_Gain_IMU2_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==01)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_01;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==02)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_10;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==03)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_11;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==00)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_00;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_00;
	}
	else
	{
		//
	}
}
	//
void TC_Wheel_Cutoff_Threshold_1()
{
	if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==01)
	{
		TC_wh_speed_cutoff=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==02)
	{
		TC_wh_speed_cutoff=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==03)
	{
		TC_wh_speed_cutoff=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==00)
	{
		TC_wh_speed_cutoff=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_00;
	}
	else
	{
		//
	}
}

	/*if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain == 01)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_01;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_01;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_01;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain == 02)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_10;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_10;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_10;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain==03)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_11;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_11;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_11;
	}
	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_rate_gain==00)
	{
		TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_00;
		TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_00;
		TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_00;
	}
	else
	{
		//
	}


	//
	if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==01)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_01;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_01;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_01;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==02)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_10;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_10;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_10;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==03)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_11;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_11;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_11;
	}

	else if( TC_gain_select_u.TC_gain_select_Table.TC_detumbling_bdot_gain==00)
	{
		TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_00;
		TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_00;
		TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==01)
	{
	  TC_BDOT_Det_Thresh = GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_01;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==02)
	{
	  TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_10;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==03)
	{
	 TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_BDOT_Det_Thresh==00)
	{
	 TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_00;
	}

	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==01)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_01;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==02)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_10;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==03)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_GYRO_Det_Min_Thres==00)
	{
	  TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_00;
	}
	else
	{
		//
	}
	 //
	if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==01)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==02)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==03)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld==00)
	{
		TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==01)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==02)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==03)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld==00)
	{
		TC_SpeedDumpTime=GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==01)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==02)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==03)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect==00)
	{
		TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==01)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==02)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==03)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate==00)
	{
		TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_00;
	}

	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==01)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_01;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==02)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_10;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==03)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_11;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU1==00)
	{
		IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_00;
		IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==01)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_01;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==02)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_10;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==03)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_11;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Gyro_LPF_Gain_IMU2==00)
	{
		IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_00;
		IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==01)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_01;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==02)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_10;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==03)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_11;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU1==00)
	{
		IMU1_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_00;
		IMU1_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==01)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_01;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==02)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_10;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==03)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_11;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Mag_LPF_Gain_IMU2==00)
	{
		IMU2_Corr.DB_MagLPF[0]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_00;
		IMU2_Corr.DB_MagLPF[1]=GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_00;
	}
	else
	{
		//
	}
	//
	if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==01)
	{
		TC_wh_speed_thres=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_01;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==02)
	{
		TC_wh_speed_thres=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_10;
	}
	else if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==03)
	{
		TC_wh_speed_thres=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_11;
	}

	else if(TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold==00)
	{
		TC_wh_speed_thres=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_00;
	}
	else
	{
		//
	}

/*	//power on init
	TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_00;
	TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_00;
	TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_00;

	TC_KP[0]=GAIN_DATA_SET.TC_KP_0_00;
	TC_KP[1]=GAIN_DATA_SET.TC_KP_1_00;
	TC_KP[2]=GAIN_DATA_SET.TC_KP_2_00;

	TC_KR[0]=GAIN_DATA_SET.TC_KR_0_00;
	TC_KR[1]=GAIN_DATA_SET.TC_KR_1_00;
	TC_KR[2]=GAIN_DATA_SET.TC_KR_2_00;

	TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_00;
	TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_00;
	TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_00;

    IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_00;
	IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_00;

	IMU1_Corr.DB_MagLPF[0] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_00;
	IMU1_Corr.DB_MagLPF[1] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_00;

    IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_00;
	IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_00;

	IMU2_Corr.DB_MagLPF[0] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_00;
	IMU2_Corr.DB_MagLPF[1] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_00;

	TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_00;

	TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_00;

	TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_00;

	TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_00;

	TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_00;

	TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_00;

	TC_wh_speed_thres=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_00;

} */

