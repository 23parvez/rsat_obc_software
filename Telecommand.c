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

BlkExe_Stat BlkExe_Status = BLK_Disabled;

//Initialization of Function Table-------------------------
void(*FuncExecute_Table[TC_func_exe_MAX_LIMIT])() = {
		TC_IMU1_On,                                         			/* offset =    0  */
		TC_IMU2_On,    													/* offset =    1  */
		TC_IMU1_Off,													/* offset =    2  */
		TC_IMU2_Off,													/* offset =    3  */
		imu1_db_execute,												/* offset =    4  */
		imu2_db_execute,												/* offset =    5  */
		imu1_db_checksum,												/* offset =    6  */
		imu2_db_checksum,												/* offset =    7  */
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

		TC_Mag_LPF_Gain_Update_IMU1,									/* offset =    21 */
		TC_Mag_LPF_Gain_Update_IMU2,									/* offset =    22 */
		TC_Gyro_LPF_Gain_Update_IMU1,									/* offset =    23 */
		TC_Gyro_LPF_Gain_Update_IMU2,									/* offset =    24 */


		TC_ACC_Ang_RESET,												/* offset =    25 */
		TC_Panel1_Deploy,												/* offset =    26 */
		TC_Panel2_Deploy,												/* offset =    27 */
		TC_GPS1_ON,														/* offset =    28 */
		TC_GPS1_OFF,													/* offset =    29 */
		TC_GPS1_NMEA_ZDA_enable,										/* offset =    30 */
		TC_GPS1_NMEA_ZDA_disable,										/* offset =    31 */
		TC_GPS1_NMEA_GGA_enable,										/* offset =    32 */
		TC_GPS1_NMEA_GGA_disable,										/* offset =    33 */
		TC_GPS1_NMEA_RMC_enable,										/* offset =    34 */
		TC_GPS1_NMEA_RMC_disable,										/* offset =    35 */
		TC_GPS1_cold_start,												/* offset =    36 */
		TC_GPS1_factory_reset,											/* offset =    37 */
		TC_GPS2_on,														/* offset =    38 */
		TC_GPS2_off,													/* offset =    39 */
		TC_GPS2_NMEA_ZDA_enable,										/* offset =    40 */
		TC_GPS2_NMEA_ZDA_disable,										/* offset =    41 */
		TC_GPS2_NMEA_GGA_enable,										/* offset =    42 */
		TC_GPS2_NMEA_GGA_disable,										/* offset =    43 */
		TC_GPS2_NMEA_RMC_enable,										/* offset =    44 */
		TC_GPS2_NMEA_RMC_disable,										/* offset =    45 */
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
		Roll_Torquer_ON,												/* offset =    57 */
		Pitch_Torquer_ON,												/* offset =    58 */
		Yaw_Torquer_ON,													/* offset =    59 */
		Roll_Torquer_OFF,												/* offset =    60 */
		Pitch_Torquer_OFF,												/* offset =    61 */
		Yaw_Torquer_OFF,												/* offset =    62 */
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
		ss_main_db_execute,												/* offset =    89 */
		ss_redundant_db_execute,										/* offset =    90 */
		ss_main_db_checksum,											/* offset =    91 */
		ss_redundant_db_checksum,										/* offset =    92 */
		TC_Qinit,														/* offset =    93 */
		TC_Ping_RW1,													/* offset =    94 */
		TC_Ping_RW2,													/* offset =    95 */
		TC_Ping_RW3,													/* offset =    96 */
		TC_Ping_RW4,													/* offset =    97 */
		TC_MTR_Roll_Positive,											/* offset =    98 */
		TC_MTR_Roll_Negative,											/* offset =    99 */
		TC_MTR_Pitch_Positive,											/* offset =    100*/
		TC_MTR_Pitch_Negative,											/* offset =    101*/
		TC_MTR_Yaw_Positive,											/* offset =    102*/
		TC_MTR_Yaw_Negative,											/* offset =    103*/
		rDifferentialTTC_Execute,										/* offset =    104*/
		OBC_ON,															/* offset =    105*/
		OBC_OFF,														/* offset =    106*/
		Antenna_Deloy_ON,												/* offset =    107*/
		Antenna_Deloy_OFF,												/* offset =    108*/
		payload_2_on,													/* offset =    109*/
		payload_2_off,													/* offset =    110*/
		EEPROM_CHECK_COMMAND,											/* offset =    111*/
		S_band_tx_off,													/* offset =    112*/
		X_band_tx_on,													/* offset =    113*/
		X_band_tx_off,													/* offset =    114*/
		SA1_DEPLOYMENT_MECHANISM_MAIN_ON,                               /* offset =    115*/
		SA1_DEPLOYMENT_MECHANISM_MAIN_OFF,                              /* offset =    116*/
		SA2_DEPLOYMENT_MECHANISM_MAIN_ON,                               /* offset =    117*/
		SA2_DEPLOYMENT_MECHANISM_MAIN_OFF,                              /* offset =    118*/
		SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_ON,       /* offset =    119*/
		SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_OFF,      /* offset =    120*/
		Test_ON,                                                        /* offset =    121*/
		Test_OFF,                                                       /* offset =    122*/
		Antenna_armed,                                               	/* offset =    123*/
		Antenna_deployed,                                               /* offset =    124*/
		Antenna_dis_armed,                                              /* offset =    125*/
		sunlit,                                                         /* offset =    126*/
		eclipse,                                                        /* offset =    127*/
		Sunlit_eclipse_both                                             /*  offset =   128*/
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
			1,              /*offset 22*/
			1,              /*offset 23*/
			1, 			    /*offset 24*/
			1,              /*offset 25*/
			1,              /*offset 26*/
			1,			    /*offset 27*/
			1,              /*offset 28*/
			1,              /*offset 29*/
			1,              /*offset 30*/
			1, 			    /*offset 31*/
			1,              /*offset 32*/
			1,              /*offset 33*/
			1,              /*offset 34*/
			1, 			    /*offset 35*/
			1,              /*offset 36*/
			1,              /*offset 37*/
			1,			    /*offset 38*/
			1,              /*offset 39*/
			1,              /*offset 40*/
			1,              /*offset 41*/
			1, 			    /*offset 42*/
			1,              /*offset 43*/
			1,              /*offset 44*/
			1,              /*offset 45*/
			1, 			    /*offset 46*/
			1,              /*offset 47*/
			1,              /*offset 48*/
			1,			    /*offset 49*/
			1,              /*offset 50*/
			1,              /*offset 51*/
			1,              /*offset 52*/
			1, 			    /*offset 53*/
			1,              /*offset 54*/
			1,              /*offset 55*/
			1,              /*offset 56*/
			1, 			    /*offset 57*/
			1,              /*offset 58*/
			1,              /*offset 59*/
			1,              /*offset 60*/
			1, 			    /*offset 61*/
			1,              /*offset 62*/
			1               /*offset 63*/

       };

unsigned int atp;
//TODO: Add these to TM
unsigned char RcvdParity = 0;
unsigned char ComputedParity = 0;
uint32 rHAL_TC_Read()
{
	int i_TC;
	TC_Buffer_Addr = TC_BUFFER_BASE;
	TC_Status_Data = TC_STATUS_REGISTER;
	unsigned long int TC_Rcvd_56Bit[2] = {0UL,0UL};                 // {0x00c41000,0x07900000};

	#ifdef ENGINEERING_MODEL
			TC_STATUS_REGISTER;
	#endif

	if ((TC_Status_Data & TC_AUTHENTIC_PULSE_MASK ) == 0x00008000)  // Check Authentic Pulse
	{
		atp++;
		// Enable FPGA DATA Ready Bit
		TC_STATUS_REGISTER = (TC_Status_Data | TC_WRITE_BIT_MASK); // Set WR Bit Before Reading TC SPC Buffer
		TC_authentic_pulse_rcvd = TRUE;
		Telecommand_ptr = &(u_TC.rcvd[0]);
		for(i_TC = 0;i_TC < 4;i_TC++)
		    {
				*Telecommand_ptr++ = (uint16)(REG32(TC_Buffer_Addr) & 0x0000FFFF);
				TC_Buffer_Addr += 0x00000004;
		    }

		//Copying to TM Buffer with appropriate index
		TM.Buffer.TM_Last_seen_TC[0] = u_TC.data_rcvd[0];
		TM.Buffer.TM_Last_seen_TC[1] = (u_TC.data_rcvd[1] & 0xFFFFFF00);
		TC_STATUS_REGISTER = 0x00000000;                         //Clear data ready bit and authentic pulsezz
		//Test : Create reset for FPGA FSM ( on 18_05_2019)
	#ifdef ENGINEERING_MODEL
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

	}
	return TC_authentic_pulse_rcvd;
}


void rTelecommand()
{
	if (TC_authentic_pulse_rcvd)								    //Checks whether if Authentic pulse is received
	{
		TC_count++;
		switch (u_TC.Frame.command_type_A)
		{
			case 0x0: rReal_Time_TC();							    //Real-time telecommands
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

			case 0x2: // rDifferential_TimeTag_TC();				//Differential time-tag telecommands
					  break;

			 default: rBlock_TC();								    //Bulk telecommands
					  break;

		}
	}
}

void rReal_Time_TC()
{
	switch (u_TC.Frame.command_type_B)
	{

		case 0x1:	rData_TC();					               	//Processing of Data Telecommands
					break;

		case 0x2: 	rBoolean_TC();				              	//Processing of Boolean(ON/OFF) Telecommands
					break;

		case 0x3: 	rGainSelect_TC();			              	//Processing of Gain Select Telecommands
					break;

		case 0x4: 	rFuncExecute_TC();							//Processing of Function Execute Telecommands
					break;

		case 0x5: 	rRemoteProgram_Addr_TC();					//Processing of Remote Program Address Telecommands
					break;

		case 0x6: 	rRemoteProgram_Data_TC();					//Processing of Remote Program Data Telecommands
					break;

		case 0x7:   //rData_TC_uint32();        				//Processing of Data Telecommands(float)
					break;

		 default: 	rContingency_TC();							//Processing of Contingency Telecommands
					break;
	}
}

void rContingency_TC()
{
	return;														// Decoded in hardware
}

 // Routine for Data_Commands Processing
void rData_TC()
{
	switch(u_TC.DataCommand.offset_addr)
	{
	case 0:   TC_data_command_Table.RW1_Speed =
			       u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr];
			  break;
	case 1:	  TC_data_command_Table.RW2_Speed =
		           u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr];
	          break;
	case 2:   TC_data_command_Table.RW3_Speed =
		           u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr];
	          break;
	case 3:  TC_data_command_Table.RW4_Speed =
			        u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr];
			  break;
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
	case 10:  TC_data_command_Table.SA3_SHUNT_UTP 								= u_TC.DataCommand.Data;
			   break;
	case 11:  TC_data_command_Table.BATTERY_HEATER1_UTP						    = u_TC.DataCommand.Data;
			   break;
	case 12:  TC_data_command_Table.BATTERY_HEATER1_LTP 						= u_TC.DataCommand.Data;
	           break;
	case 13:  TC_data_command_Table.BATTERY_HEATER2_UTP 						= u_TC.DataCommand.Data;
		       break;
	case 14:  TC_data_command_Table.BATTERY_HEATER2_LTP 						= u_TC.DataCommand.Data;
			   break;
	case 15:  TC_data_command_Table.SA_PanelHeater_Timeout 						=
						(unsigned long int)(u_TC.DataCommand.Data * Resol_Table[u_TC.DataCommand.offset_addr]);
			  break;
	/************************** Added on 27 July 2019 **************************/
	case 16: TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU1[0] 		= u_TC.DataCommand.Data;
			 break;
	case 17: TC_data_command_Table.TC_Drift_Uplink_Compensation_IMU2[0]		    = u_TC.DataCommand.Data;
			     break;
	case 18: TC_data_command_Table.TC_Gyro_Misalignment_IMU2 					= u_TC.DataCommand.Data;
				 break;
	case 19: TC_data_command_Table.TC_Gyro_Scale_Factor_IMU1 					= u_TC.DataCommand.Data;
				 break;
	case 20: TC_data_command_Table.TC_Gyro_Scale_Factor_IMU2 					= u_TC.DataCommand.Data;
				 break;
	case 21: TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU1[0] 		= u_TC.DataCommand.Data;
				 break;
	case 22: TC_data_command_Table.TC_MagBias_Uplink_Compensation_IMU2[0] 		= u_TC.DataCommand.Data;
				 break;
	case 23: TC_data_command_Table.TC_Mag_Misalignment_IMU1 					= u_TC.DataCommand.Data;
			     break;
	case 24: TC_data_command_Table.TC_Mag_Misalignment_IMU2 					= u_TC.DataCommand.Data;
				break;
	case 25: TC_data_command_Table.TC_Mag_Scale_Factor_IMU1  					= u_TC.DataCommand.Data;
				break;
	case 26: TC_data_command_Table.TC_Mag_Scale_Factor_IMU2  					= u_TC.DataCommand.Data;
				break;
	case 27: TC_data_command_Table.TC_ACC_Ang_RESET  							= u_TC.DataCommand.Data;
				break;
	case 28: TC_data_command_Table.TC_SS_misalnCM1256  							= u_TC.DataCommand.Data;
				break;
	case 29: TC_data_command_Table.TC_SS_misalnCM2356  							= u_TC.DataCommand.Data;
				break;
	case 30: TC_data_command_Table.TC_SS_misalnCM3456  							= u_TC.DataCommand.Data;
				break;
	case 31: TC_data_command_Table.TC_SS_misalnCM4156  							= u_TC.DataCommand.Data;
				break;
	case 32: TC_data_command_Table.TC_SS_Imax_ALPHA  							= u_TC.DataCommand.Data;
				break;
	case 33: TC_data_command_Table.TC_eclipse_entrytime  						= u_TC.DataCommand.Data;
				break;
	case 34: TC_data_command_Table.TC_eclipse_exittime  						= u_TC.DataCommand.Data;
				break;
	case 35: TC_data_command_Table.TC_elapsed_orbitTimer  						= u_TC.DataCommand.Data;
				break;
	case 36: TC_data_command_Table.TC_Sunlit_detctn_timer  						= u_TC.DataCommand.Data;
				break;
	case 37: TC_data_command_Table.TC_Time_GPS2TLE 								= u_TC.DataCommand.Data;
				break;
	case 38: TC_data_command_Table.TC_GPS_OFFSET_UTC 							= u_TC.DataCommand.Data;
				break;
	case 39: TC_data_command_Table.TC_delUT1_ECEF2ECI 							= u_TC.DataCommand.Data;
				break;
	case 40: TC_data_command_Table.TC_delAT_ECEF2ECI 							= u_TC.DataCommand.Data;
				break;
	case 41: TC_data_command_Table.TC_xp_ECEF2ECI 								= u_TC.DataCommand.Data;
				break;
	case 42: TC_data_command_Table.TC_yp_ECEF2ECI 								= u_TC.DataCommand.Data;
				break;
	case 43: TC_data_command_Table.TC_JulianDay_at_OBT0						    = u_TC.DataCommand.Data;
				break;
	case 44: TC_data_command_Table.TC_OBT_Drift_Corr 							= u_TC.DataCommand.Data;
				break;
	case 45: TC_data_command_Table.TC_TLE 										= u_TC.DataCommand.Data;
				break;
	case 46: TC_data_command_Table.TC_JulianDate_at_OrbitalEpoch 				= u_TC.DataCommand.Data;
				break;
	case 47: TC_data_command_Table.TC_OBT_with_TLE_Update 						= u_TC.DataCommand.Data;
				break;
	case 48: TC_data_command_Table.TC_Wheel_Configuration_Index 				= u_TC.DataCommand.Data;
				break;
	case 49: TC_data_command_Table.TC_Speed_Based_Dumping_Speed_Upper_Threshold = u_TC.DataCommand.Data;
				break;
	case 50: TC_data_command_Table.TC_Speed_Based_Dumping_Speed_Lower_Threshold = u_TC.DataCommand.Data;
				break;
	case 51: TC_data_command_Table.TC_Det_Bprev_Count 							= u_TC.DataCommand.Data;
				break;
	case 52: TC_data_command_Table.TC_Det_BDOT_Compute_Count 					= u_TC.DataCommand.Data;
				break;
	case 53: TC_data_command_Table.TC_Det_GYRO_Compute_Count			 		= u_TC.DataCommand.Data;
				break;
	case 54: TC_data_command_Table.TC_Rate_Chk_Safe2Det 						= u_TC.DataCommand.Data;
				break;
	case 55: TC_data_command_Table.TC_ECEF_stationlatitude 						= u_TC.DataCommand.Data;
				break;
	case 56: TC_data_command_Table.TC_ECEF_stationLongitude 					= u_TC.DataCommand.Data;
				break;
	case 57: TC_data_command_Table.TC_Error_dev_SunlitAD 						= u_TC.DataCommand.Data;
				break;
	case 58: TC_data_command_Table.TC_Error_dev_EclipseAD 						= u_TC.DataCommand.Data;
				break;
	case 59: TC_data_command_Table.TC_wAD_BODYmaxThRoll 						= u_TC.DataCommand.Data;
				break;
	case 60: TC_data_command_Table.TC_wAD_BODYmaxThPitch 						= u_TC.DataCommand.Data;
				break;
	case 61: TC_data_command_Table.TC_wAD_BODYmaxThYaw 							= u_TC.DataCommand.Data;
				break;
	case 62: TC_data_command_Table.TC_magMin_angle 								= u_TC.DataCommand.Data;
				break;
	case 63: TC_data_command_Table.TC_magMax_angle 								= u_TC.DataCommand.Data;
				break;
	case 64: TC_data_command_Table.TC_GYRO_Det_Max_Thresh						= u_TC.DataCommand.Data;
			    break;
	case 65: TC_data_command_Table.TC_PanelD_Status_Sel						    = u_TC.DataCommand.Data;
				break;
	case 66: TC_data_command_Table.TC_wAD_BODYminThRoll						    = u_TC.DataCommand.Data;
			 	 break;
	case 67: TC_data_command_Table.TC_wAD_BODYminThPitch						= u_TC.DataCommand.Data;
				break;
	case 68: TC_data_command_Table.TC_wAD_BODYminThYaw						    = u_TC.DataCommand.Data;
				break;
	case 69: TC_data_command_Table.TC_wAD_updateTimeThresh					    = u_TC.DataCommand.Data;
				break;
	case 70: TC_data_command_Table.TC_wp_QDP						            = u_TC.DataCommand.Data;
				break;
	case 71: TC_data_command_Table.TC_heaters_auto_manual						= u_TC.DataCommand.Data;
				break;
	case 72: TC_data_command_Table.TC_heaters_manual						    = u_TC.DataCommand.Data;
				break;

	default:
		      break;
	}
}

 // Routine for Boolean_Telecommand processing
void rBoolean_TC()
{
	if (u_TC.Bool.offset_addr <= sizeof(TC_boolean_u.TC_Boolean_Table)/sizeof(uint8))
	{
		/* if (u_TC.Bool.decision_bit)

			TC_boolean_u.Pos[u_TC.Bool.offset_addr] = 0xff;
		else
			TC_boolean_u.Pos[u_TC.Bool.offset_addr] = 0;

		*/

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
	if(u_TC.GainSelect.offset_addr <= sizeof(TC_gain_select_u.TC_gain_select_Table)/sizeof(uint8))
	{
		TC_gain_select_u.Pos[u_TC.GainSelect.offset_addr] = (char)u_TC.GainSelect.gain_set;
	}
	adcsgains();
}

 // Routine for Function_execution_Telecommand Processing
void rFuncExecute_TC()
{
	if(u_TC.FuncExecute.offset_addr < TC_func_exe_MAX_LIMIT)
	{
		FuncExecute_Table[u_TC.FuncExecute.offset_addr]();
	}
}

 // Routine for Remote_addr_Telecommand processing
void rRemoteProgram_Addr_TC()
{
	Remote_Addr = u_TC.Remote.data_addr;
}

 // Routine for Remote_data_Telecommand processing
void rRemoteProgram_Data_TC()
{
	REG32(Remote_Addr) = u_TC.Remote.data_addr;
	Remote_Addr += 4;
}

 // Routine for Absolute_TimeTag_Telecommand Processing
void rAbsolute_TimeTag_TC()
{
	rAbsoluteTTC_Update();
	if(u_TC.Bool. decision_bit == 0x0F )
	{
		rAbsoluteTTC_Execute();
	}
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
uint32 Absolutetime = 0;
Nodeptrtype head = NULL;

int32  DTTC_count = 0;
uint32 Diff_start_srl_num = 0;
uint32 Diff_stop_srl_num = 0;
uint32 DTTCtime_reference;
//int DTTC_exe_flag = 1;
//----------------------To be transferred to h files (END)--------------------------

void initNodetable()
{
   int32 i;
   for (i = 0; i < MAX_TABLE_LIMIT; i++)
     {
       Nodeptr[i] = & Nodearray[i];
     }

   tos = 0;
}

Nodeptrtype getNode()
{
    if (tos < MAX_TABLE_LIMIT)
	{
//		int Top_index = tos;
    	Top_index = tos;
		tos++;
		if (tos >= MAX_TABLE_LIMIT)
		{
			Nodeptrempty_flag = 1;
		}
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
    if ( Nodeptrempty_flag == 0)					// Check whether memory exists in the pointer table for the node insertion
	{
    	ATTCformat new_data = u_TC.ATTC_Exe_cmd;
    	Nodeptrtype new_node = getNode();			//Create new node to be used for storing the TC data to the list
    	new_data = u_TC.ATTC_Exe_cmd;
    	new_node = getNode();			            //Create new node to be used for storing the TC data to the list
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
			temp = head;
			prev = NULL;
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

// Execute the first ATTC
void rAbsoluteTTC_Execute()
{
	if (TC_boolean_u.TC_Boolean_Table.ATTC_Master_enable_flag)		// Check if the Manual execution flag is enabled
	{
		ATTC_exe_en=1;
		if(ATTC_count == 0)							                // ATTC list is empty or not
		{
			return;
		}
		else
		{
			Absolutetime = 0x000FFFFF & Major_Cycle_Count;           // Extract 20 bits of the Major cycle counter to be used as On Board timer
			if (head->command.TC_time <= Absolutetime) 	             // Check whether the command to be executed has crossed the OBT.
			{
				u_TC.ATTC_Exe_cmd = head -> command;
				head -> command.TC_time = 0;
				rReal_Time_TC();
				freeNode(head);
				head = head->next;
				ATTC_count--;
			}
			else
			{
				return;
			}
		}
		ATTC_exe_en = 0;
	}

	else
	{
		ATTC_exe_en = 0;
		return;
	}
}




// Routine for insertion of new node to the array
void rDifferential_TimeTag_Update()
{
	if (DTTC_count >= MAX_TABLE_LIMIT)
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
void rAbsoluteTTC_Delete()
{
	Nodeptrtype temp = head;
	Nodeptrtype prev = NULL;
	while (temp->command.TC_time < u_TC.ATTC_Exe_cmd.TC_time)	// Search for the ATT command to be deleted using the unique TC_time data
	{
		prev = temp;
		temp = temp->next;
	}
	prev->next = temp->next;
	temp->next = NULL;
	temp->command.TC_time = 0;
	freeNode(temp);
	ATTC_count--;
}

// Clear the Telecommand data
void rAbsoluteTTC_Clear()		                               // Routine to clear the Linked list data
{
	Nodeptrtype temp = head;
	while(temp != NULL)
	{
		temp -> command.TC_time = 0;
		freeNode(temp);
		temp = temp->next;
	}
	head = NULL;	// The linked list is now empty.
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

/** Block command update
 *  - This function will allow Block TC to be added in incremental order
 *  - This function will allow Block TC to be updated within the index   */
void Block_update(void)
{
	if((u_TC.BLK_Update_cmd.Blk_No < MAX_BLKS) && (u_TC.BLK_Update_cmd.Cmd_Srl < MAX_BLK_CMD_SIZE))
	{
		if((u_TC.BLK_Update_cmd.Cmd_Srl == 0)
		   || (u_TC.BLK_Update_cmd.Cmd_Srl <= Block_Index[u_TC.BLK_Update_cmd.Blk_No]))
		{
			Block_array[u_TC.BLK_Update_cmd.Blk_No][u_TC.BLK_Update_cmd.Cmd_Srl] = u_TC.cmd_rcvd;
			Block_Index[u_TC.BLK_Update_cmd.Blk_No]++;
		}
	}
}

/** Block command execute */
void rBlockTC_Execute(void)
{
	if (BlkExe_Status < BLK_Disabled)
	{
		if(BlkCurrent_Cmd < Block_Index[(unsigned long int)BlkExe_Status])
		{
			u_TC.cmd_rcvd = Block_array[(unsigned long int)BlkExe_Status][BlkCurrent_Cmd];
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
void rBlock0_exe(void)
{
	if (BlkExe_Status == BLK_Disabled)
	{
		BlkExe_Status = BLK_0;
	}
}
void rBlock1_exe(void)
{
	if (BlkExe_Status == BLK_Disabled)
	{
		BlkExe_Status = BLK_1;
	}
}
void rBlock2_exe(void)
{
	if (BlkExe_Status == BLK_Disabled)
	{
		BlkExe_Status = BLK_2;
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
	TMTC_boolean_u.Boolean_Table.TC_Sun_Acquisition_Mode_select                = TC_boolean_u.TC_Boolean_Table.TC_Sun_Acquisition_Mode_select;
	TMTC_boolean_u.Boolean_Table.TC_AutoTransit_Det2SunAquisition              = TC_boolean_u.TC_Boolean_Table.TC_AutoTransit_Det2SunAquisition;
	TMTC_boolean_u.Boolean_Table.TC_SunAq2DetMode_autotransit                  = TC_boolean_u.TC_Boolean_Table.TC_SunAq2DetMode_autotransit;
	TMTC_boolean_u.Boolean_Table.TC_mom_dumping_ang_mom_based                  = TC_boolean_u.TC_Boolean_Table.TC_mom_dumping_ang_mom_based;
	TMTC_boolean_u.Boolean_Table.TC_SunAq2InertialMode_autotransit             = TC_boolean_u.TC_Boolean_Table.TC_SunAq2InertialMode_autotransit;
	TMTC_boolean_u.Boolean_Table.TC_IMU_Select                                 = TC_boolean_u.TC_Boolean_Table.TC_IMU_Select;
	TMTC_boolean_u.Boolean_Table.TC_SS_Cells_Sel                               = TC_boolean_u.TC_Boolean_Table.TC_SS_Cells_Sel;
	TMTC_boolean_u.Boolean_Table.TC_GPS12_Select                               = TC_boolean_u.TC_Boolean_Table.TC_GPS12_Select;
	TMTC_boolean_u.Boolean_Table.TC_GPS_TLE_Select                             = TC_boolean_u.TC_Boolean_Table.TC_GPS_TLE_Select;
	TMTC_boolean_u.Boolean_Table.TC_GPS_Utility                                = TC_boolean_u.TC_Boolean_Table.TC_GPS_Utility;
	TMTC_boolean_u.Boolean_Table.TC_EKF_Drift_Compensation_Enable_or_Disable   = TC_boolean_u.TC_Boolean_Table.TC_EKF_Drift_Compensation_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_EKF_MagBias_Compensation_Enable_or_Disable = TC_boolean_u.TC_Boolean_Table.TC_EKF_MagBias_Compensation_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_Mag_Torquer_Bias_Enable_or_Disable         = TC_boolean_u.TC_Boolean_Table.TC_Mag_Torquer_Bias_Enable_or_Disable;
	TMTC_boolean_u.Boolean_Table.TC_DFC_Logic                                  = TC_boolean_u.TC_Boolean_Table.TC_DFC_Logic;
	TMTC_boolean_u.Boolean_Table.TC_Dither_Logic                               = TC_boolean_u.TC_Boolean_Table.TC_Dither_Logic;
	TMTC_boolean_u.Boolean_Table.TC_Wheel_AutoReConfig_Logic                   = TC_boolean_u.TC_Boolean_Table.TC_Wheel_AutoReConfig_Logic;
	TMTC_boolean_u.Boolean_Table.TC_Wheel_SpinUpDown_Logic                     = TC_boolean_u.TC_Boolean_Table.TC_Wheel_SpinUpDown_Logic;
	TMTC_boolean_u.Boolean_Table.TC_ThreeAxis2SafeMode_autotransit              = TC_boolean_u.TC_Boolean_Table.TC_ThreeAxis2SafeMode_autotransit;
	TMTC_boolean_u.Boolean_Table.TC_SunAq2ThreeAxis_autotransit                = TC_boolean_u.TC_Boolean_Table.TC_SunAq2ThreeAxis_autotransit;
	TMTC_boolean_u.Boolean_Table.TC_Det_AutoTransitionBDOTtoGYRO               = TC_boolean_u.TC_Boolean_Table.TC_Det_AutoTransitionBDOTtoGYRO;
	TMTC_boolean_u.Boolean_Table.TC_Detumbling_Logic_select                    = TC_boolean_u.TC_Boolean_Table.TC_Detumbling_Logic_select;
	TMTC_boolean_u.Boolean_Table.TC_Speed_Dumping                              = TC_boolean_u.TC_Boolean_Table.TC_Speed_Dumping;
	TMTC_boolean_u.Boolean_Table.TC_Sun_Varying_Mode                           = TC_boolean_u.TC_Boolean_Table.TC_Sun_Varying_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Orbit_Reference_Mode                       = TC_boolean_u.TC_Boolean_Table.TC_Orbit_Reference_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Asd_Node_Mode                              = TC_boolean_u.TC_Boolean_Table.TC_Asd_Node_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Des_Node_Mode                              = TC_boolean_u.TC_Boolean_Table.TC_Des_Node_Mode;
	TMTC_boolean_u.Boolean_Table.TC_Station_Tracking_Mode                      = TC_boolean_u.TC_Boolean_Table.TC_Station_Tracking_Mode;
	TMTC_boolean_u.Boolean_Table.TC_QuestUpdate_Enable                         = TC_boolean_u.TC_Boolean_Table.TC_QuestUpdate_Enable;
	TMTC_boolean_u.Boolean_Table.TC_EKFControl_Enable                          = TC_boolean_u.TC_Boolean_Table.TC_EKFControl_Enable;
	TMTC_boolean_u.Boolean_Table.TC_2RW_Control_Mode                           = TC_boolean_u.TC_Boolean_Table.TC_2RW_Control_Mode;
	TMTC_boolean_u.Boolean_Table.rTC_mag_detumbling_mode_enable                = TC_boolean_u.TC_Boolean_Table.rTC_mag_detumbling_mode_enable;
	TMTC_boolean_u.Boolean_Table.rTC_gyro_detumbling_mode                      = TC_boolean_u.TC_Boolean_Table.rTC_gyro_detumbling_mode;
	TMTC_boolean_u.Boolean_Table.rTC_safe_mode                                 = TC_boolean_u.TC_Boolean_Table.rTC_safe_mode;
	TMTC_boolean_u.Boolean_Table.TC_detumbling_gyro_select_gnd                 = TC_boolean_u.TC_Boolean_Table.TC_detumbling_gyro_select_gnd;
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
	TMTC_gain_select_u.gain_select_Table.TC_SS_Currents_LPF_Gain               = TC_gain_select_u.TC_gain_select_Table.TC_SS_Currents_LPF_Gain;
	TMTC_gain_select_u.gain_select_Table.TC_GPS_pulse_duration                 = TC_gain_select_u.TC_gain_select_Table.TC_GPS_pulse_duration;
	TMTC_gain_select_u.gain_select_Table.TC_KP                                 = TC_gain_select_u.TC_gain_select_Table.TC_KP;
	TMTC_gain_select_u.gain_select_Table.TC_KR                          	   = TC_gain_select_u.TC_gain_select_Table.TC_KR;
	/************************************************* Added on 26 JULY 2019 *****************************************************************/
	TMTC_gain_select_u.gain_select_Table.TC_GPS_Validity_Altitude_Threshold    = TC_gain_select_u.TC_gain_select_Table.TC_GPS_Validity_Altitude_Threshold;
	TMTC_gain_select_u.gain_select_Table.TC_Wheel_Cutoff_Threshold			   = TC_gain_select_u.TC_gain_select_Table.TC_Wheel_Cutoff_Threshold;
	TMTC_gain_select_u.gain_select_Table.TC_Wh_SpinUD_Thrsld				   = TC_gain_select_u.TC_gain_select_Table.TC_Wh_SpinUD_Thrsld;
	TMTC_gain_select_u.gain_select_Table.TC_comd_pitch_rate					   = TC_gain_select_u.TC_gain_select_Table.TC_comd_pitch_rate;
	TMTC_gain_select_u.gain_select_Table.TC_AngDev_SafeModetransit_Thrsld	   = TC_gain_select_u.TC_gain_select_Table.TC_AngDev_SafeModetransit_Thrsld;
	TMTC_gain_select_u.gain_select_Table.TC_AngMomDump_Thrsld				   = TC_gain_select_u.TC_gain_select_Table.TC_AngMomDump_Thrsld;
	TMTC_gain_select_u.gain_select_Table.TC_SpeedDump_Thrsld				   = TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_Thrsld;
	TMTC_gain_select_u.gain_select_Table.TC_SpeedDump_TimeSelect			   = TC_gain_select_u.TC_gain_select_Table.TC_SpeedDump_TimeSelect;

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
	int j;
	unsigned char temp;
	for (j=0; j <=6; j++)
    {
	   startAddr[j] = startAddr[j] ^ 0x01;
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
	char i = 0;
	char j = 0;

	//compute randomized data
	randOut = ((*TCAddr) ^ randSeq);
	randOutShift = randOut << 8;
	randOutShiftX = randOutShift & 0x8000000000000000;
	randOutBit = (unsigned char) (randOutShiftX >> 63);

	for (i = 0;i < 56; i++)
	{
		x[0] = xp[6]^randOutBit;
		x[1] = xp[0];
		x[2] = xp[1]^x[0];
		x[3] = xp[2];
		x[4] = xp[3];
		x[5] = xp[4];
		x[6] = xp[5]^x[0];
        for (j=0; j<=6; j++)
		{
			xp[j] = x[j];
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
	rHAL_IMU_POWER(IMU1,ON);
}
void TC_IMU2_On()
{
	rHAL_IMU_POWER(IMU2,ON);
}
void TC_IMU1_Off()
{
	rHAL_IMU_POWER(IMU1,OFF);
}
void TC_IMU2_Off()
{
	rHAL_IMU_POWER(IMU2,OFF);
}
void imu1_db_execute()
{
	rIMU1_DB_Execute();
}
void imu2_db_execute()
{
	rIMU2_DB_Execute();
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
	//to be done
}
void TC_Drift_Uplink_Compenstation_Update_IMU2()
{
	//to be done
}
void TC_Gyro_Misalignment_Update_IMU1()
{
	//to be done
}
void TC_Gyro_Misalignment_Update_IMU2()
{
	//to be done
}
void TC_Gyro_Scale_Factor_Update_IMU1()
{
	return;
	//to be done
}
void TC_Gyro_Scale_Factor_Update_IMU2()
{
	//to be done
}
void TC_MagBias_Uplink_Compenstation_Update_IMU1()
{
	//to be done
}
void TC_MagBias_Uplink_Compenstation_Update_IMU2()
{
	//to be done
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


void TC_Mag_LPF_Gain_Update_IMU1()
{

}
void TC_Mag_LPF_Gain_Update_IMU2()
{

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
//TODO:Change name to main and red
void TC_Panel1_Deploy()
{
	rHAL_SA_Deploy(SA_MAIN, Deploy);
}
void TC_Panel2_Deploy()
{
	rHAL_SA_Deploy(SA_RED, Deploy);
}
void TC_GPS1_ON()
{
	rHAL_GPS_POWER(GPS_1,ON);
	return;
}
void TC_GPS1_OFF()
{
	rHAL_GPS_POWER(GPS_1,OFF);
	return;
}
void TC_GPS1_NMEA_ZDA_enable()
{
	rHAL_GPS_Config(GPS_1,NMEA_ZDA_Enable);
	return;
}
void TC_GPS1_NMEA_ZDA_disable()
{
	rHAL_GPS_Config(GPS_1,NMEA_ZDA_Disable);
	return;
}
void TC_GPS1_NMEA_GGA_enable()
{
	rHAL_GPS_Config(GPS_1,NMEA_GGA_Enable);
	return;
}
void TC_GPS1_NMEA_GGA_disable()
{
	rHAL_GPS_Config(GPS_1,NMEA_GGA_Disable);
	return;
}
void TC_GPS1_NMEA_RMC_enable()
{
	rHAL_GPS_Config(GPS_1,NMEA_RMC_Enable);
	return;
}
void TC_GPS1_NMEA_RMC_disable()
{
	rHAL_GPS_Config(GPS_1,NMEA_RMC_Disable);
	return;
}
void TC_GPS1_cold_start()
{
	rHAL_GPS_Config(GPS_1,COLD_START);
	return;
}
void TC_GPS1_factory_reset()
{
	rHAL_GPS_Config(GPS_1,FACTORY_RESET);
	return;
}
void TC_GPS2_on()
{
	rHAL_GPS_POWER(GPS_2,ON);
	return;
}
void TC_GPS2_off()
{
	rHAL_GPS_POWER(GPS_2,OFF);
	return;
}
void TC_GPS2_NMEA_ZDA_enable()
{
	rHAL_GPS_Config(GPS_2,NMEA_ZDA_Enable);
	return;
}
void TC_GPS2_NMEA_ZDA_disable()
{
	rHAL_GPS_Config(GPS_2,NMEA_ZDA_Disable);
	return;
}
void TC_GPS2_NMEA_GGA_enable()
{
	rHAL_GPS_Config(GPS_2,NMEA_GGA_Enable);
	return;
}
void TC_GPS2_NMEA_GGA_disable()
{
	rHAL_GPS_Config(GPS_2,NMEA_GGA_Disable);
	return;
}
void TC_GPS2_NMEA_RMC_enable()
{
	rHAL_GPS_Config(GPS_2,NMEA_RMC_Enable);
	return;
}
void TC_GPS2_NMEA_RMC_disable()
{
	rHAL_GPS_Config(GPS_2,NMEA_RMC_Disable);
	return;
}
void TC_GPS2_cold_start()
{
	rHAL_GPS_Config(GPS_2,COLD_START);
	return;
}
void TC_GPS2_factory_reset()
{
	rHAL_GPS_Config(GPS_2,FACTORY_RESET);
	return;
}
void TC_W1_ON()
{

	rHAL_RW_POWER(RW1,ON);
	//rRW_Ping_TC1();
}
void TC_W2_ON()
{
	rHAL_RW_POWER(RW2,ON);
	//rRW_Ping_TC2();
}
void TC_W3_ON()
{
	rHAL_RW_POWER(RW3,ON);
	//rRW_Ping_TC3();
}
void TC_W4_ON()
{
	rHAL_RW_POWER(RW4,ON);
	//rRW_Ping_TC4();
}
void TC_W1_OFF()
{
	rHAL_RW_POWER(RW1,OFF);
}
void TC_W2_OFF()
{
	rHAL_RW_POWER(RW2,OFF);
}
void TC_W3_OFF()
{
	rHAL_RW_POWER(RW3,OFF);
}
void TC_W4_OFF()
{
	rHAL_RW_POWER(RW4,OFF);
}
void TC_Nominal_wheel_speed_execute()
{
	//to be done
}
void Roll_Torquer_ON()
{
	rHAL_MTR_ON();
	return;
}
void Pitch_Torquer_ON()
{
	rHAL_MTR_ON();
	return;
}
void Yaw_Torquer_ON()
{
	rHAL_MTR_ON();
	return;
}
void Roll_Torquer_OFF()
{
	rHAL_MTR_OFF();
	return;
}
void Pitch_Torquer_OFF()
{
	rHAL_MTR_OFF();
	return;
}
void Yaw_Torquer_OFF()
{
	rHAL_MTR_OFF();
	return;
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
	rHAL_PL_Power(Payload_1,ON);
	return;
}
void payload_1_off()
{
	rHAL_PL_Power(Payload_1,OFF);
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
	rHAL_PL_STS_Check();
//		REG32(0x20007000) = 0x0000FFFF;
//		REG32(PAYLOAD_BUFFER_ADDRESS) = 0x00005355;
//		REG32(PAYLOAD_STATUS_ADDRESS) = 0x00001201;
}
void PL_K_CMD_ACQ()
{
	rHAL_PL_CMD_ACQ();
}
void PL_K_CMD_HLT()
{
	rHAL_PL_CMD_HLT();
}
void PL_K_DIAG()
{
	rHAL_PL_DIAG();
}
void PL_K_CMD_OFF()
{
	rHAL_PL_CMD_OFF();
}
void PL_K_CMD_ON()
{
	rHAL_PL_CMD_ON();
}
void ss_main_db_execute()
{
	rSS_Main_DB_Execute();
}
void ss_redundant_db_execute()
{
	rSS_Redundant_DB_Execute();
}
void ss_main_db_checksum()
{
	//ss_main_db_checksum_obc = checksum_u32((unsigned long int *)&SS_Main_DB,160);
		//Add here for telemetry: ss_main_db_checksum_obc;
}
void ss_redundant_db_checksum()
{
	//ss_redundant_db_checksum_obc = checksum_u32((unsigned long int *)&SS_Redundant_DB,160);
		//Add here for telemetry: ss_redundant_db_checksum_obc;
}

void TC_Qinit()
{
	return;
}

void TC_Ping_RW1()
{
	RW_Init();
	rRW_Ping_TC1();
}

void TC_Ping_RW2()
{
	RW_Init();
	rRW_Ping_TC2();
}

void TC_Ping_RW3()
{
	RW_Init();
	rRW_Ping_TC3();
}

void TC_Ping_RW4()
{
	RW_Init();
	rRW_Ping_TC4();
}
void TC_MTR_Yaw_Positive()
{
//	rHAL_MTR(Yaw, Positive);
	MTR_Axis = Yaw;
	MTR_Polarity = Positive;
	MTR_Reset_Flag = 0;

}
void TC_MTR_Yaw_Negative()
{
//	rHAL_MTR(Yaw, Negative);
	MTR_Axis = Yaw;
	MTR_Polarity = Negative;
	MTR_Reset_Flag = 0;
}
void TC_MTR_Pitch_Positive()
{
//	rHAL_MTR(Pitch, Positive);
	MTR_Axis = Pitch;
	MTR_Polarity = Positive;
	MTR_Reset_Flag = 0;
}
void TC_MTR_Pitch_Negative()
{
//	rHAL_MTR(Pitch, Negative);
	MTR_Axis = Pitch;
	MTR_Polarity = Negative;
	MTR_Reset_Flag = 0;
}
void TC_MTR_Roll_Positive()
{
//	rHAL_MTR(Roll, Positive);
	MTR_Axis = Roll;
	MTR_Polarity = Positive;
	MTR_Reset_Flag = 0;
}
void TC_MTR_Roll_Negative()
{
//	rHAL_MTR(Roll, Negative);
	MTR_Axis = Roll;
	MTR_Polarity = Negative;
	MTR_Reset_Flag = 0;
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
void Antenna_Deloy_ON()
{
	rHAL_Antenna_Deploy(Deploy);
}
void Antenna_Deloy_OFF()
{
	rHAL_Antenna_Deploy(Not_Deploy);
}
void payload_2_on()
{
	rHAL_PL_Power(Payload_2,ON);
	return;
}
void payload_2_off()
{
	rHAL_PL_Power(Payload_2,OFF);
	return;
}
void EEPROM_CHECK_COMMAND()
{
	S_band_tx_on_off(s_band_on);
}
void S_band_tx_off()
{
	S_band_tx_on_off(s_band_off);
}
 void X_band_tx_on()
{
	 rHAL_X_Tx_ON_OFF(X_Tx_ON);
}
 void X_band_tx_off()
{
	 rHAL_X_Tx_ON_OFF(X_Tx_OFF);
}
void SA1_DEPLOYMENT_MECHANISM_MAIN_ON()
{
	//
}
void SA1_DEPLOYMENT_MECHANISM_MAIN_OFF()
{
 	 //
}
void SA2_DEPLOYMENT_MECHANISM_MAIN_ON()
{
 	 //
}
void SA2_DEPLOYMENT_MECHANISM_MAIN_OFF()
{
  	 //
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
void Antenna_armed()
{
	antennaCommand1();
}
void Antenna_deployed()
{
	antennaCommand2();
}
void Antenna_dis_armed()
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

