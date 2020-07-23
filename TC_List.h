#ifndef TC_LIST
#define TC_LIST

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long int uint32;
typedef unsigned long long int uint64;

unsigned int Remote_data_addr;

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
			uint8 SunMagAD;                                                       /* offset =   46  */
			uint8 magAD;                                                          /* offset =   47  */
			uint8 TC_EKF1_Enable;                                                 /* offset =   48  */
			uint8 TC_EKF2_Enable;                                                 /* offset =   49  */
			uint8 TC_Storage_TM_Enable_Disable;                                   /* offset =   50  */
			uint8 TC_ST_mode;                                                     /* offset =   51  */
			uint8 TC_NormalStorage_Sampling_Rate_Select;                          /* offset =   52  */
			uint8 pl_tx_tm_flag;                                                  /* offset =   53  */
			//uint8 Storage_TM_dumping;                                           /* offset =   54  */
			uint8 TC_Storage_TM_Dump_enable_disable;							  /* offset =   54  */
			//uint8 tc_mode;                                                      /* offset =   55  */
			uint8 TC_Storage_TM_Special_Normal_Mode_select;						  /* offset =   55  */
			uint8 RW_Speed_Negative;                                              /* offset =   56  */
			//uint8 storage_dump_mode;                                            /* offset =   57  */
			uint8 TC_Storage_TM_Full_Segment_Dump_mode_Select;					  /* offset =   57  */
			//uint8 TC_dumping;                                                   /* offset =   58  */
			uint8 TC_TCH__Storage_TM_Dump_select;      							  /* offset =   58  */
			uint8 TC_sram_scrub_enable_disable;                                   /* offset =   59  */
			uint8 NMI_test_enable;                                                /* offset =   60  */
			uint8 ST_dump_abort;												  /* offset =   61  */
			//uint8 TCH_dump_mode;                                                /* offset =   62  */
			uint8 TC_TCH_Full_Segment_Dump_mode;   								  /* offset =   62  */
	    };

#define TC_BOOLEAN_MAX_LIMIT 63
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
			uint8 SunMagAD                                        :1;
			uint8 magAD                                           :1;
			uint8 TC_EKF1_Enable                                  :1;
			uint8 TC_EKF2_Enable                                  :1;
			uint8 TC_Storage_TM_Enable_Disable                    :1;
			uint8 TC_ST_mode                                      :1;
			uint8 TC_NormalStorage_Sampling_Rate_Select           :1;
			uint8 pl_tx_tm_flag                                   :1;
			//uint8 Storage_TM_dumping                            :1;
			uint8 TC_Storage_TM_Dump_enable_disable               :1;
			//uint8 tc_mode                                       :1;
			uint8 TC_Storage_TM_Special_Normal_Mode_select 	 	  :1;
			uint8 RW_Speed_Negative                               :1;
			//uint8 storage_dump_mode                             :1;
			uint8 TC_Storage_TM_Full_Segment_Dump_mode_Select     :1;
			//uint8 TC_dumping                                    :1;
			uint8 TC_TCH__Storage_TM_Dump_select                  :1;
			uint8 TC_sram_scrub_enable_disable                    :1;
			uint8 NMI_test_enable                                 :1;
			uint8 ST_dump_abort                                   :1;
			//uint8 TCH_dump_mode                                 :1;
			uint8 TC_TCH_Full_Segment_Dump_mode                   :1;
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
	    	uint8 TC_special_Sampling_rate_Select;                       //offset =    26//
	    	uint8 TC_ST_Format_Selection;                                //offset =    27//

	    };

#define TC_gain_select_MAX_LIMIT 28
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
			uint8 TC_special_Sampling_rate_Select                   :2;
			uint8 TC_ST_Format_Selection                            :2;

	    };

	union TMTC_gain_select_U
	    {
		uint32 TMTC_Buffer[2]; //TBD
		struct TMTC_gain_select gain_select_Table;
	    }TMTC_gain_select_u;

#pragma pack(1)
	struct TC_data_command
	    {
	    	//Command names to be updated!!!
			float RW1_Speed;				     										/* offset = 0*/
			float RW2_Speed;			         										/* offset = 1*/
			float RW3_Speed;                     										/* offset = 2*/
			float RW4_Speed;                     										/* offset = 3*/
			long int Differential_srl_num;       								        /* offset = 4*/

			/***** Below telecommands are not implemented *****/

			int SA1_SHUNT_LTP;                  										/* offset = 5*/
			int SA1_SHUNT_UTP;                  										/* offset = 6*/
			int SA2_SHUNT_LTP;                  										/* offset = 7*/
			int SA2_SHUNT_UTP;                  										/* offset = 8*/
			int SA3_SHUNT_LTP;                  										/* offset = 9*/
			int SA3_SHUNT_UTP;                 										    /* offset = 10*/
			int BATTERY_HEATER1_UTP;           											/* offset = 11*/
			int BATTERY_HEATER1_LTP;           											/* offset = 12*/
			unsigned long int SA_PanelHeater_Timeout;									/*offset =  13*/


			/***************** Added on 27 July 2019 *****************/
			//unsigned long int TC_RW_NO;

			int TC_heaters_auto_manual;                                                 /*offset = 14*/
			int TC_heaters_manual;                                                      /*offset = 15*/
			float pl_data_command_1;                                                    /*offset = 16*/
			float pl_data_command_2;                                                    /*offset = 17*/
			float pl_data_command_3;                                                    /*offset = 18*/
			float pl_data_command_4;                                                    /*offset = 19*/
			float pl_data_command_5;                                                    /*offset = 20*/
			float pl_data_command_6;                                                    /*offset = 21*/
			int BLK_Number_selection;                                                   /*offset = 22*/
			int Remote_blk_select;                                                      /*offset = 23*/
			int TC_power_safe_LTP;                                                      /*offset = 24*/
			int TC_power_safe_UTP;                                                      /*offset = 25*/
			int TC_over_Heat;                                                           /*offset = 26*/
	    } TC_data_command_Table;

#define TC_data_command_MAX_LIMIT 27

#pragma pack(1)
	    struct ADCS_Data_TC
	    {
			float TC_Drift_Uplink_Compensation_IMU1[3];									/*offset = 0*/
			float TC_Drift_Uplink_Compensation_IMU2[3];									/*offset = 1*/
			float TC_MagBias_Uplink_Compensation_IMU1[3];							    /*offset = 2*/
			float TC_MagBias_Uplink_Compensation_IMU2[3];								/*offset = 3*/
			float TC_Mag_Misalignment_IMU1;												/*offset = 4*/
			float TC_Mag_Misalignment_IMU2;												/*offset = 5*/
			float TC_Mag_Scale_Factor_IMU1;												/*offset = 6*/
			float TC_Mag_Scale_Factor_IMU2;												/*offset = 7*/
			float TC_ACC_Ang_RESET;														/*offset = 8*/
			float TC_SS_misalnCM1256;													/*offset = 9*/
			float TC_SS_misalnCM2356;													/*offset = 10*/
			float TC_SS_misalnCM3456;													/*offset = 11*/
			float TC_SS_misalnCM4156;													/*offset = 12*/
			float TC_SS_Imax_ALPHA;														/*offset = 13*/
			float TC_eclipse_entrytime;													/*offset = 14*/
			float TC_eclipse_exittime;													/*offset = 15*/
			float TC_elapsed_orbitTimer;												/*offset = 16*/
			float TC_Sunlit_detctn_timer;												/*offset = 17*/
			float TC_Time_GPS2TLE;														/*offset = 18*/
			float TC_GPS_OFFSET_UTC;													/*offset = 19*/
			float TC_delUT1_ECEF2ECI;													/*offset = 20*/
			float TC_delAT_ECEF2ECI;													/*offset = 21*/
			float TC_xp_ECEF2ECI;														/*offset = 22*/
			float TC_yp_ECEF2ECI;														/*offset = 23*/
			float TC_JulianDay_at_OBT0;													/*offset = 24*/
			float TC_OBT_Drift_Corr;													/*offset = 25*/
			float TC_TLE;																/*offset = 26*/
			float TC_JulianDate_at_OrbitalEpoch;										/*offset = 27*/
			float TC_OBT_with_TLE_Update;												/*offset = 28*/
			float TC_Wheel_Configuration_Index;											/*offset = 29*/
			float TC_Det_Bprev_Count;													/*offset = 30*/
			float TC_Det_BDOT_Compute_Count;											/*offset = 31*/
			float TC_Det_GYRO_Compute_Count;											/*offset = 32*/
			float TC_Rate_Chk_Safe2Det;													/*offset = 33*/
			float TC_ECEF_stationlatitude;												/*offset = 34*/
			float TC_ECEF_stationLongitude;												/*offset = 35*/
			float TC_Error_dev_SunlitAD;												/*offset = 36*/
			float TC_Error_dev_EclipseAD;												/*offset = 37*/
			float TC_wAD_BODYmaxThRoll;													/*offset = 38*/
			float TC_wAD_BODYmaxThPitch;												/*offset = 39*/
			float TC_wAD_BODYmaxThYaw;													/*offset = 40*/
			float TC_magMin_angle;														/*offset = 41*/
			float TC_magMax_angle;														/*offset = 42*/
			float TC_GYRO_Det_Max_Thresh;												/*offset = 43*/
			float TC_PanelD_Status_Sel;                                                 /*offset = 44*/
			float TC_wAD_BODYminThRoll;                                                 /*offset = 45*/
			float TC_wAD_BODYminThPitch;                                                /*offset = 46*/
			float TC_wAD_BODYminThYaw;                                                  /*offset = 47*/
			float TC_wAD_updateTimeThresh;                                              /*offset = 48*/
			float TC_wp_QDP;                                                            /*offset = 49*/

	  	    }ADCS_TC_data_command_Table;
#define ADCS_TC_data_command_MAX_LIMIT 50
//---------------------------Data command telemetry-------------------------------

//TBD

//---------------------------------------------------------------------------------

//Resolution table for Data-commands
float Resol_Table[TC_data_command_MAX_LIMIT];
//float Resol_Table_Adcs[ADCS_TC_data_command_MAX_LIMIT];
double gain_set[300];
//List of Function execute commands

#define TC_func_exe_MAX_LIMIT 150

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


