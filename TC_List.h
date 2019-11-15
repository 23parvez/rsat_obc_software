#ifndef TC_LIST
#define TC_LIST

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long int uint32;
typedef unsigned long long int uint64;


#pragma pack(1)
	struct TC_Boolean
	    {
			uint8 Pitch_Torquer_Polarity_Reversal;                           	  /* offset =    0  */
			uint8 Yaw_Torquer_Polarity_Reversal;							 	  /* offset =    1  */
			uint8 Roll_Torquer_Polarity_Reversal;								  /* offset =    2  */
			uint8 Pitch_Torquer_Enable_or_Disable;								  /* offset =    3  */
			uint8 Yaw_Torquer_Enable_or_Disable;								  /* offset =    4  */
			uint8 Roll_Torquer_Enable_or_Disable;								  /* offset =    5  */
			uint8 TC_Sun_Acquisition_Mode_select;								  /* offset =    6  */		//REMOVE
			uint8 TC_AutoTransit_Det2SunAquisition;								  /* offset =    7  */
			uint8 TC_SunAq2DetMode_autotransit;								      /* offset =    8  */
			uint8 TC_mom_dumping_ang_mom_based;									  /* offset =    9  */
			uint8 TC_SunAq2InertialMode_autotransit;							  /* offset =   10  */
			uint8 TC_IMU_Select;												  /* offset =   11  */
			uint8 TC_SS_Cells_Sel;												  /* offset =   12  */
			uint8 TC_GPS12_Select;												  /* offset =   13  */
			uint8 TC_GPS_TLE_Select;											  /* offset =   14  */
			uint8 TC_GPS_Utility;												  /* offset =   15  */
			uint8 TC_EKF_Drift_Compensation_Enable_or_Disable;					  /* offset =   16  */
			uint8 TC_EKF_MagBias_Compensation_Enable_or_Disable;				  /* offset =   17  */
			uint8 TC_Mag_Torquer_Bias_Enable_or_Disable;						  /* offset =   18  */
			uint8 TC_DFC_Logic;													  /* offset =   19  */
			uint8 TC_Dither_Logic;												  /* offset =   20  */
			uint8 TC_Wheel_AutoReConfig_Logic;									  /* offset =   21  */
			uint8 TC_Wheel_SpinUpDown_Logic;									  /* offset =   22  */
			uint8 TC_ThreeAxis2SafeMode_autotransit;							  /* offset =   23  */
			uint8 TC_SunAq2ThreeAxis_autotransit;								  /* offset =   24  */
			uint8 TC_Det_AutoTransitionBDOTtoGYRO;								  /* offset =   25  */
			uint8 TC_Detumbling_Logic_select;									  /* offset =   26  */
			uint8 TC_Speed_Dumping;												  /* offset =   27  */
			uint8 TC_Sun_Varying_Mode;											  /* offset =   28  */
			uint8 TC_Orbit_Reference_Mode;										  /* offset =   29  */
			uint8 TC_Asd_Node_Mode;												  /* offset =   30  */
			uint8 TC_Des_Node_Mode;												  /* offset =   31  */
			uint8 TC_Station_Tracking_Mode;										  /* offset =   32  */
			uint8 TC_QuestUpdate_Enable;										  /* offset =   33  */
			uint8 TC_EKFControl_Enable;											  /* offset =   34  */
			uint8 TC_2RW_Control_Mode;											  /* offset =   35  */
			uint8 rTC_mag_detumbling_mode_enable;								  /* offset =   36  */		//
			uint8 rTC_gyro_detumbling_mode;										  /* offset =   37  */		//
			uint8 rTC_safe_mode;												  /* offset =   38  */		//
			uint8 TC_detumbling_gyro_select_gnd;								  /* offset =   39  */		//
			uint8 ATTC_Master_enable_flag;										  /* offset =   40  */
			uint8 DTTC_exe_flag ;                                                 /* offset =   41  */
			uint8 Reaction_wheel_1_speed_enable;                                  /* offset =   42  */
			uint8 Reaction_wheel_2_speed_enable;                                  /* offset =   43  */
			uint8 Reaction_wheel_3_speed_enable;                                  /* offset =   44  */
			uint8 Reaction_wheel_4_speed_enable;                                  /* offset =   45  */
			uint8 SunMagAD;                                                    /* offset =   46  */
			uint8 magAD;                                                          /* offset =   47  */
			uint8 TC_EKF1_Enable;                                                  /* offset =   48  */
			uint8 TC_EKF2_Enable;                                                  /* offset =   49  */
	    };

#define TC_BOOLEAN_MAX_LIMIT 48
	union TC_boolean_U
	    {
			uint8 Pos[TC_BOOLEAN_MAX_LIMIT];
			struct TC_Boolean TC_Boolean_Table;
	    }TC_boolean_u;

// ------------------------Boolean commands Telemetry------------------------

#pragma pack(1)
	struct TMTC_Boolean
	    {
			uint8 Pitch_Torquer_Polarity_Reversal				  :1;
			uint8 Yaw_Torquer_Polarity_Reversal       			  :1;
			uint8 Roll_Torquer_Polarity_Reversal      			  :1;
			uint8 Pitch_Torquer_Enable_or_Disable                 :1;
			uint8 Yaw_Torquer_Enable_or_Disable                   :1;
			uint8 Roll_Torquer_Enable_or_Disable                  :1;
			uint8 TC_Sun_Acquisition_Mode_select                  :1;
			uint8 TC_AutoTransit_Det2SunAquisition                :1;
			uint8 TC_SunAq2DetMode_autotransit                    :1;
			uint8 TC_mom_dumping_ang_mom_based                    :1;
			uint8 TC_SunAq2InertialMode_autotransit               :1;
			uint8 TC_IMU_Select                                   :1;
			uint8 TC_SS_Cells_Sel                                 :1;
			uint8 TC_GPS12_Select                                 :1;
			uint8 TC_GPS_TLE_Select                               :1;
			uint8 TC_GPS_Utility                                  :1;
			uint8 TC_EKF_Drift_Compensation_Enable_or_Disable     :1;
			uint8 TC_EKF_MagBias_Compensation_Enable_or_Disable   :1;
			uint8 TC_Mag_Torquer_Bias_Enable_or_Disable           :1;
			uint8 TC_DFC_Logic                                    :1;
			uint8 TC_Dither_Logic                                 :1;
			uint8 TC_Wheel_AutoReConfig_Logic                     :1;
			uint8 TC_Wheel_SpinUpDown_Logic                       :1;
			uint8 TC_ThreeAxis2SafeMode_autotransit               :1;
			uint8 TC_SunAq2ThreeAxis_autotransit                  :1;
			uint8 TC_Det_AutoTransitionBDOTtoGYRO                 :1;
			uint8 TC_Detumbling_Logic_select                      :1;
			uint8 TC_Speed_Dumping                                :1;
			uint8 TC_Sun_Varying_Mode                             :1;
			uint8 TC_Orbit_Reference_Mode                         :1;
			uint8 TC_Asd_Node_Mode                                :1;
			uint8 TC_Des_Node_Mode                                :1;
			uint8 TC_Station_Tracking_Mode                        :1;
			uint8 TC_QuestUpdate_Enable                           :1;
			uint8 TC_EKFControl_Enable                            :1;
			uint8 TC_2RW_Control_Mode                             :1;
			uint8 rTC_mag_detumbling_mode_enable                  :1;
			uint8 rTC_gyro_detumbling_mode                        :1;
			uint8 rTC_safe_mode                                   :1;
			uint8 TC_detumbling_gyro_select_gnd					  :1;
			uint8 ATTC_Master_enable_flag                         :1;
			uint8 DTTC_exe_flag                                   :1;
			uint8 Reaction_wheel_1_speed_enable                   :1;
			uint8 Reaction_wheel_2_speed_enable                   :1;
			uint8 Reaction_wheel_3_speed_enable                   :1;
			uint8 Reaction_wheel_4_speed_enable                   :1;
			uint8 SunMagAD                                     :1;
			uint8 magAD                                           :1;
			uint8 TC_EKF1_Enable                                  :1;
			uint8 TC_EKF2_Enable                                  :1;

	    };

	union TMTC_boolean_U
	    {
		uint32 TMTC_Buffer[2]; //TBD
		struct TMTC_Boolean Boolean_Table;
	    }TMTC_boolean_u;
//---------------------------------------------------------------------------

#pragma pack(1)
	struct TC_gain_select
	    {
	    	//Command names to be updated!!!
	    	uint8 TC_detumbling_bdot_gain;								 //offset =    0//
	    	uint8 TC_detumbling_rate_gain;								 //offset =    1//
	    	uint8 TC_BDOT_Det_Thresh;									 //offset =    2//
	    	uint8 TC_GYRO_Det_Min_Thres;						         //offset =    3//
	    	uint8 TC_W1_Commanded_Nominal_Speed;						 //offset =    4//
	    	uint8 TC_W2_Commanded_Nominal_Speed;						 //offset =    5//
	    	uint8 TC_W3_Commanded_Nominal_Speed;						 //offset =    6//
	    	uint8 TC_W4_Commanded_Nominal_Speed;						 //offset =    7//
	    	uint8 TC_momentum_dumping_gain;								 //offset =    8//
	    	uint8 TC_PanelD_Status_Sel;									 //offset =    9//
	    	uint8 TC_Gyro_LPF_Gain_IMU1;								 //offset =    10//
	    	uint8 TC_Gyro_LPF_Gain_IMU2;								 //offset =    11//
	    	uint8 TC_Mag_LPF_Gain_IMU1;									 //offset =    12//
	    	uint8 TC_Mag_LPF_Gain_IMU2;									 //offset =    13//
	    	uint8 TC_SS_Currents_LPF_Gain;								 //offset =    14//
	    	uint8 TC_GPS_pulse_duration;								 //offset =    15//
	    	uint8 TC_KP;										         //offset =    16//
	    	uint8 TC_KR;											     //offset =    17//
	    	/************ Added on 26 JULY 2019 *******************/
	    	uint8 TC_GPS_Validity_Altitude_Threshold;					 //offset =    18//
	    	uint8 TC_Wheel_Cutoff_Threshold;						     //offset =    19//
	    	uint8 TC_Wh_SpinUD_Thrsld;								     //offset =    20//
	    	uint8 TC_comd_pitch_rate;						             //offset =    21//
	    	uint8 TC_AngDev_SafeModetransit_Thrsld;						 //offset =    22//
	    	uint8 TC_AngMomDump_Thrsld;						             //offset =    23//
	    	uint8 TC_SpeedDump_Thrsld;						             //offset =    24//
	    	uint8 TC_SpeedDump_TimeSelect;							     //offset =    25//

	    };

#define TC_gain_select_MAX_LIMIT 26
	union TC_gain_select_U
	    {
		uint8 Pos[TC_gain_select_MAX_LIMIT];
		struct TC_gain_select TC_gain_select_Table;
	    }TC_gain_select_u;

//----------------------------Gain Select commands Telemetry--------------------
#pragma pack(1)
	struct TMTC_gain_select
	    {
	    	//Command names to be updated!!!
	    	uint8 TC_detumbling_bdot_gain           				:2;
	    	uint8 TC_detumbling_rate_gain           				:2;
	    	uint8 TC_BDOT_Det_Thresh								:2;
	    	uint8 TC_GYRO_Det_Min_Thres						        :2;
	    	uint8 TC_W1_Commanded_Nominal_Speed					    :2;
	    	uint8 TC_W2_Commanded_Nominal_Speed					    :2;
	    	uint8 TC_W3_Commanded_Nominal_Speed					    :2;
	    	uint8 TC_W4_Commanded_Nominal_Speed					    :2;
			uint8 TC_momentum_dumping_gain          				:2;
			uint8 TC_PanelD_Status_Sel              				:2;
			uint8 TC_Gyro_LPF_Gain_IMU1             				:2;
			uint8 TC_Gyro_LPF_Gain_IMU2             				:2;
			uint8 TC_Mag_LPF_Gain_IMU1              				:2;
			uint8 TC_Mag_LPF_Gain_IMU2              				:2;
			uint8 TC_SS_Currents_LPF_Gain           				:2;
			uint8 TC_GPS_pulse_duration             				:2;
			uint8 TC_KP                  				            :2;
			uint8 TC_KR                      				        :2;

			/************ Added on 26 JULY 2019 *******************/
			uint8 TC_GPS_Validity_Altitude_Threshold 				:2;
			uint8 TC_Wheel_Cutoff_Threshold							:2;
			uint8 TC_Wh_SpinUD_Thrsld								:2;
			uint8 TC_comd_pitch_rate								:2;
			uint8 TC_AngDev_SafeModetransit_Thrsld					:2;
			uint8 TC_AngMomDump_Thrsld								:2;
			uint8 TC_SpeedDump_Thrsld								:2;
			uint8 TC_SpeedDump_TimeSelect							:2;

	    };

	union TMTC_gain_select_U
	    {
		uint32 TMTC_Buffer[2]; //TBD
		struct TMTC_gain_select gain_select_Table;
	    }TMTC_gain_select_u;
//-------------------------------------------------------------------------------

	    /* double TC_ADCS_gainset[0][4][3]={
	            {{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}},
	    		{{1,2,3},{1,2,3},{1,2,3},{1,2,3}}*/





#pragma pack(1)
	struct TC_data_command
	    {
	    	//Command names to be updated!!!
			float RW1_Speed;				     										/* offset = 0*/
			float RW2_Speed;			         										/* offset = 1*/
			float RW3_Speed;                     										/* offset = 2*/
			float RW4_Speed;                     										/* offset = 3*/
			long int Differential_srl_num;       								    /* offset = 4*/

			/***** Below telecommands are not implemented *****/

			float SA1_SHUNT_LTP;                  										/* offset = 5*/
			float SA1_SHUNT_UTP;                  										/* offset = 6*/
			float SA2_SHUNT_LTP;                  										/* offset = 7*/
			float SA2_SHUNT_UTP;                  										/* offset = 8*/
			float SA3_SHUNT_LTP;                  										/* offset = 9*/
			float SA3_SHUNT_UTP;                 										/* offset = 10*/
			float BATTERY_HEATER1_UTP;           										/* offset = 11*/
			float BATTERY_HEATER1_LTP;           										/* offset = 12*/
			float BATTERY_HEATER2_UTP;           										/* offset = 13*/
			float BATTERY_HEATER2_LTP;            										/* offset = 14*/
			unsigned long int SA_PanelHeater_Timeout;									/*offset = 15*/


			/***************** Added on 27 July 2019 *****************/
			float TC_Drift_Uplink_Compensation_IMU1[3];									/*offset = 16*/
			float TC_Drift_Uplink_Compensation_IMU2[3];									/*offset = 17*/
			float TC_Gyro_Misalignment_IMU2;											/*offset = 18*/
			float TC_Gyro_Scale_Factor_IMU1;											/*offset = 19*/
			float TC_Gyro_Scale_Factor_IMU2;											/*offset = 20*/
			float TC_MagBias_Uplink_Compensation_IMU1[3];							    /*offset = 21*/
			float TC_MagBias_Uplink_Compensation_IMU2[3];								/*offset = 22*/
			float TC_Mag_Misalignment_IMU1;												/*offset = 23*/
			float TC_Mag_Misalignment_IMU2;												/*offset = 24*/
			float TC_Mag_Scale_Factor_IMU1;												/*offset = 25*/
			float TC_Mag_Scale_Factor_IMU2;												/*offset = 26*/
			float TC_ACC_Ang_RESET;														/*offset = 27*/
			float TC_SS_misalnCM1256;													/*offset = 28*/
			float TC_SS_misalnCM2356;													/*offset = 29*/
			float TC_SS_misalnCM3456;													/*offset = 30*/
			float TC_SS_misalnCM4156;													/*offset = 31*/
			float TC_SS_Imax_ALPHA;														/*offset = 32*/
			float TC_eclipse_entrytime;													/*offset = 33*/
			float TC_eclipse_exittime;													/*offset = 34*/
			float TC_elapsed_orbitTimer;												/*offset = 35*/
			float TC_Sunlit_detctn_timer;												/*offset = 36*/
			float TC_Time_GPS2TLE;														/*offset = 37*/
			float TC_GPS_OFFSET_UTC;													/*offset = 38*/
			float TC_delUT1_ECEF2ECI;													/*offset = 39*/
			float TC_delAT_ECEF2ECI;													/*offset = 40*/
			float TC_xp_ECEF2ECI;														/*offset = 41*/
			float TC_yp_ECEF2ECI;														/*offset = 42*/
			float TC_JulianDay_at_OBT0;													/*offset = 43*/
			float TC_OBT_Drift_Corr;													/*offset = 44*/
			float TC_TLE;																/*offset = 45*/
			float TC_JulianDate_at_OrbitalEpoch;										/*offset = 46*/
			float TC_OBT_with_TLE_Update;												/*offset = 47*/
			float TC_Wheel_Configuration_Index;											/*offset = 48*/
			float TC_Speed_Based_Dumping_Speed_Upper_Threshold;							/*offset = 49*/
			float TC_Speed_Based_Dumping_Speed_Lower_Threshold;							/*offset = 50*/
			float TC_Det_Bprev_Count;													/*offset = 51*/
			float TC_Det_BDOT_Compute_Count;											/*offset = 52*/
			float TC_Det_GYRO_Compute_Count;											/*offset = 53*/
			float TC_Rate_Chk_Safe2Det;													/*offset = 54*/
			float TC_ECEF_stationlatitude;												/*offset = 55*/
			float TC_ECEF_stationLongitude;												/*offset = 56*/
			float TC_Error_dev_SunlitAD;												/*offset = 57*/
			float TC_Error_dev_EclipseAD;												/*offset = 58*/
			float TC_wAD_BODYmaxThRoll;													/*offset = 59*/
			float TC_wAD_BODYmaxThPitch;												/*offset = 60*/
			float TC_wAD_BODYmaxThYaw;													/*offset = 61*/
			float TC_magMin_angle;														/*offset = 62*/
			float TC_magMax_angle;														/*offset = 63*/
			float TC_GYRO_Det_Max_Thresh;												/*offset = 64*/
			float TC_PanelD_Status_Sel;                                                 /*offset = 65*/
			float TC_wAD_BODYminThRoll;                                                 /*offset = 66*/
			float TC_wAD_BODYminThPitch;                                                /*offset = 67*/
			float TC_wAD_BODYminThYaw;                                                  /*offset = 68*/
			float TC_wAD_updateTimeThresh;                                              /*offset = 69*/
			float TC_wp_QDP;                                                            /*offset = 70*/
			float TC_heaters_auto_manual;                                               /*offset = 71*/
			float TC_heaters_manual;                                                    /*offset = 72*/

	    } TC_data_command_Table;



#define TC_data_command_MAX_LIMIT 73

//---------------------------Data command telemetry-------------------------------

//TBD

//---------------------------------------------------------------------------------

//Resolution table for Data-commands
float Resol_Table[TC_data_command_MAX_LIMIT];

double gain_set[300];
//List of Function execute commands

#define TC_func_exe_MAX_LIMIT 129

void(*FuncExecute_Table[TC_func_exe_MAX_LIMIT])();

#endif // TC_LIST

/*#pragma pack(1)
typedef struct TC_ADCS_gainset
{
	double TC_detumbling_bdot_gain_0[4][3];
	double TC_detumbling_rate_gain_0[4][3];
	double TC_BDOT_Det_Thresh_0[4][3];
	double TC_GYRO_Det_Min_Thres_0[4][3];
	double TC_W1_Commanded_Nominal_Speed_0[4][3];
	double TC_W2_Commanded_Nominal_Speed_0[4][3];
	double TC_W3_Commanded_Nominal_Speed_0[4][3];
	double TC_W4_Commanded_Nominal_Speed_0[4][3];
	double TC_momentum_dumping_gain_0[4][3];
	double TC_PanelD_Status_Sel_0[4][3];
	double TC_Gyro_LPF_Gain_IMU1_0[4][3];
	double TC_Gyro_LPF_Gain_IMU2_0[4][3];
	double TC_Mag_LPF_Gain_IMU1_0[4][3];
	double TC_Mag_LPF_Gain_IMU2_0[4][3];
	double TC_SS_Currents_LPF_Gain_0[4][3];
	double TC_GPS_pulse_duration_0[4][3];
	double TC_KP_0[4][3];
	double TC_KR_0[4][3];
	double TC_GPS_Validity_Altitude_Threshold_0[4][3];
	double TC_Wheel_Cutoff_Threshold_0[4][3];
	double TC_Wh_SpinUD_Thrsld_0[4][3];
	double TC_comd_pitch_rate_0[4][3];
	double TC_AngDev_SafeModetransit_Thrsld_0[4][3];
	double TC_AngMomDump_Thrsld_0[4][3];
	double TC_SpeedDump_Thrsld_0[4][3];
}TC_ADCS_gainset_select_lists;*/

#pragma pack(1)
 struct gains_set_sel
	{
	// offset 00
	double TC_detumbling_bdot_gain_0_00;
	double TC_detumbling_bdot_gain_1_00;
	double TC_detumbling_bdot_gain_2_00;

	double TC_detumbling_bdot_gain_0_01;
	double TC_detumbling_bdot_gain_1_01;
	double TC_detumbling_bdot_gain_2_01;

	double TC_detumbling_bdot_gain_0_10;
	double TC_detumbling_bdot_gain_1_10;
	double TC_detumbling_bdot_gain_2_10;

	double TC_detumbling_bdot_gain_0_11;
	double TC_detumbling_bdot_gain_1_11;
	double TC_detumbling_bdot_gain_2_11;

	// offset 01
	double TC_detumbling_rate_gain_0_00;
	double TC_detumbling_rate_gain_1_00;
	double TC_detumbling_rate_gain_2_00;

	double TC_detumbling_rate_gain_0_01;
	double TC_detumbling_rate_gain_1_01;
	double TC_detumbling_rate_gain_2_01;

	double TC_detumbling_rate_gain_0_10;
	double TC_detumbling_rate_gain_1_10;
	double TC_detumbling_rate_gain_2_10;

	double TC_detumbling_rate_gain_0_11;
	double TC_detumbling_rate_gain_1_11;
	double TC_detumbling_rate_gain_2_11;

	// offset 02
	double TC_BDOT_Det_Thresh_0_00;
	double TC_BDOT_Det_Thresh_0_01;
	double TC_BDOT_Det_Thresh_0_10;
	double TC_BDOT_Det_Thresh_0_11;

	// offset 03
	double TC_GYRO_Det_Min_Thres_0_00;
	double TC_GYRO_Det_Min_Thres_0_01;
	double TC_GYRO_Det_Min_Thres_0_10;
	double TC_GYRO_Det_Min_Thres_0_11;

	// offset 04
	double TC_momentum_dumping_gain_0_00;
	double TC_momentum_dumping_gain_0_01;
	double TC_momentum_dumping_gain_0_10;
	double TC_momentum_dumping_gain_0_11;

	// offset 05 // TBD
	double TC_PanelD_Status_Sel_0_00;

	double TC_PanelD_Status_Sel_0_01;

	double TC_PanelD_Status_Sel_0_10;

	double TC_PanelD_Status_Sel_0_11;

	// offset 06
	double TC_Gyro_LPF_Gain_IMU1_0_00;
	double TC_Gyro_LPF_Gain_IMU1_1_00;

	double TC_Gyro_LPF_Gain_IMU1_0_01;
	double TC_Gyro_LPF_Gain_IMU1_1_01;

	double TC_Gyro_LPF_Gain_IMU1_0_10;
	double TC_Gyro_LPF_Gain_IMU1_1_10;

	double TC_Gyro_LPF_Gain_IMU1_0_11;
	double TC_Gyro_LPF_Gain_IMU1_1_11;

	// offset 07
	double TC_Gyro_LPF_Gain_IMU2_0_00 ;
	double TC_Gyro_LPF_Gain_IMU2_1_00;

	double TC_Gyro_LPF_Gain_IMU2_0_01;
	double TC_Gyro_LPF_Gain_IMU2_1_01;

	double TC_Gyro_LPF_Gain_IMU2_0_10;
	double TC_Gyro_LPF_Gain_IMU2_1_10;

	double TC_Gyro_LPF_Gain_IMU2_0_11;
	double TC_Gyro_LPF_Gain_IMU2_1_11;

	// offset 08
	double TC_Mag_LPF_Gain_IMU1_0_00;
	double TC_Mag_LPF_Gain_IMU1_1_00;

	double TC_Mag_LPF_Gain_IMU1_0_01;
	double TC_Mag_LPF_Gain_IMU1_1_01;

	double TC_Mag_LPF_Gain_IMU1_0_10;
	double TC_Mag_LPF_Gain_IMU1_1_10;

	double TC_Mag_LPF_Gain_IMU1_0_11;
	double TC_Mag_LPF_Gain_IMU1_1_11;

	// offset 09
	double TC_Mag_LPF_Gain_IMU2_0_00;
	double TC_Mag_LPF_Gain_IMU2_1_00;

	double TC_Mag_LPF_Gain_IMU2_0_01;
	double TC_Mag_LPF_Gain_IMU2_1_01;

	double TC_Mag_LPF_Gain_IMU2_0_10;
	double TC_Mag_LPF_Gain_IMU2_1_10;

	double TC_Mag_LPF_Gain_IMU2_0_11;
	double TC_Mag_LPF_Gain_IMU2_1_11;

	// offset 10
	double TC_SS_Currents_LPF_Gain_0_00;
	double TC_SS_Currents_LPF_Gain_1_00;

	double TC_SS_Currents_LPF_Gain_0_01;
	double TC_SS_Currents_LPF_Gain_1_01;

	double TC_SS_Currents_LPF_Gain_0_10;
	double TC_SS_Currents_LPF_Gain_1_10;

	double TC_SS_Currents_LPF_Gain_0_11;
	double TC_SS_Currents_LPF_Gain_1_11;

	// offset 11
	double TC_GPS_pulse_duration_0_00;
	double TC_GPS_pulse_duration_0_01;
	double TC_GPS_pulse_duration_0_10;
	double TC_GPS_pulse_duration_0_11;

	// offset 12
	double TC_KP_0_00;
	double TC_KP_1_00;
	double TC_KP_2_00;

	double TC_KP_0_01;
	double TC_KP_1_01;
	double TC_KP_2_01;

	double TC_KP_0_10;
	double TC_KP_1_10;
	double TC_KP_2_10;

	double TC_KP_0_11;
	double TC_KP_1_11;
	double TC_KP_2_11;

	//offset 13
	double TC_KR_0_00;
	double TC_KR_1_00;
	double TC_KR_2_00;

	double TC_KR_0_01;
	double TC_KR_1_01;
	double TC_KR_2_01;

	double TC_KR_0_10;
	double TC_KR_1_10;
	double TC_KR_2_10;

	double TC_KR_0_11;
	double TC_KR_1_11;
	double TC_KR_2_11;

	//offset 14
	double TC_GPS_Validity_Altitude_Threshold_0_00;
	double TC_GPS_Validity_Altitude_Threshold_0_01;
	double TC_GPS_Validity_Altitude_Threshold_0_10;
	double TC_GPS_Validity_Altitude_Threshold_0_11;

	//offset 15
	double TC_Wheel_Cutoff_Threshold_0_00;
	double TC_Wheel_Cutoff_Threshold_1_00;
	double TC_Wheel_Cutoff_Threshold_2_00;

	double TC_Wheel_Cutoff_Threshold_0_01;
	double TC_Wheel_Cutoff_Threshold_1_01;
	double TC_Wheel_Cutoff_Threshold_2_01;

	double TC_Wheel_Cutoff_Threshold_0_10;
	double TC_Wheel_Cutoff_Threshold_1_10;
	double TC_Wheel_Cutoff_Threshold_2_10;

	double TC_Wheel_Cutoff_Threshold_0_11;
	double TC_Wheel_Cutoff_Threshold_1_11;
	double TC_Wheel_Cutoff_Threshold_2_11;

	//offset 16
	double TC_Wh_SpinUD_Thrsld_0_00;
	double TC_Wh_SpinUD_Thrsld_1_00;
	double TC_Wh_SpinUD_Thrsld_2_00;

	double TC_Wh_SpinUD_Thrsld_0_01;
	double TC_Wh_SpinUD_Thrsld_1_01;
	double TC_Wh_SpinUD_Thrsld_2_01;

	double TC_Wh_SpinUD_Thrsld_0_10;
	double TC_Wh_SpinUD_Thrsld_1_10;
	double TC_Wh_SpinUD_Thrsld_2_10;

	double TC_Wh_SpinUD_Thrsld_0_11;
	double TC_Wh_SpinUD_Thrsld_1_11;
	double TC_Wh_SpinUD_Thrsld_2_11;


	//offset 17
	double TC_comd_pitch_rate_0_00;

	double TC_comd_pitch_rate_0_01;

	double TC_comd_pitch_rate_0_10;

	double TC_comd_pitch_rate_0_11;

	//offset 18
	double TC_AngDev_SafeModetransit_Thrsld_0_00;

	double TC_AngDev_SafeModetransit_Thrsld_0_01;

	double TC_AngDev_SafeModetransit_Thrsld_0_10;

	double TC_AngDev_SafeModetransit_Thrsld_0_11;

	//offset 19
	double TC_AngMomDump_Thrsld_0_00;

	double TC_AngMomDump_Thrsld_0_01;

	double TC_AngMomDump_Thrsld_0_10;

	double TC_AngMomDump_Thrsld_0_11;


	//offset 20
	double TC_SpeedDump_Thrsld_0_00;

	double TC_SpeedDump_Thrsld_0_01;

	double TC_SpeedDump_Thrsld_0_10;

	double TC_SpeedDump_Thrsld_0_11;

	//offset 21
	double TC_SpeedDump_TimeSelect_0_00;

	double TC_SpeedDump_TimeSelect_0_01;

	double TC_SpeedDump_TimeSelect_0_10;

	double TC_SpeedDump_TimeSelect_0_11;

}GAIN_DATA_SET;


