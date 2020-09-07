#ifndef TELECOMMAND
#define TELECOMMAND

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long int uint32;
typedef unsigned long long int uint64;


#define MAX_TIMETAG_CMD_LIMIT   1000
#define MAX_BLKS  64
#define MAX_BLK_CMD_SIZE 64
#define TC_TLE_MAX_LIMIT 18
#define TC_TLE_MAX_LIMITs 9

//enum {FALSE,TRUE};

#ifndef NULL
#define NULL ((void*)0)
#endif

//mask values

#define TC_AUTHENTIC_PULSE_MASK 0x00008000
#define TC_DATA_READY           0x00000002
#define TC_WRITE_BIT_MASK 		0x00000001

unsigned long int Absolutetime;
unsigned long int TC_Status_Data;
unsigned long int TC_Buffer_Addr;
unsigned long int TC_authentic_pulse_rcvd;
unsigned short* Telecommand_ptr;
unsigned short TC_count;
unsigned short TC_cmd_executed;
unsigned short TC_command_pending;
unsigned int  TC_cmd_executed_count;
unsigned long long int TC_exe;
//unsigned char TC_storing_buffer[256];
unsigned int TC_storing_buffer[256];
unsigned int TC_buffer_count[300];
unsigned int TC_flag;
unsigned short TC_STS_data;

unsigned long int BlkCurrent_Cmd;



//int Nodeptrempty_flag = 0;
//int ATTC_Master_en = 1;
//int ATTC_exe_en = 0;
//int tos;
//int ATTC_count;


unsigned int TC_DATA[2];

unsigned int abc;

//Remote
unsigned int Remote_Addr;
unsigned int Remote_data;
unsigned int Remote_minotoring_addr;

//Differential Time tagged Command Handling

typedef struct S_DTTC		//Differential Time Tagged TC Structure
{
    uint32 start_bit:2;
    uint32 spacecraft_id:4;
    uint32 link_id:2;
    uint32 command_type_A:2;
    uint32 command_type_B:4;
    uint32 command:16;
    uint32 type_of_data_command:6;
    uint32 srl_num:8;
	uint32 TC_time:12;
    uint32 TC_parity:7;
    uint32 TC_appended_bit:1;
}DTTCformat;

typedef struct S_DTTC_Execute		//Differential Time Tagged TC execute Structure
{
    uint32 start_bit:2;
    uint32 spacecraft_id:4;
    uint32 link_id:2;
    uint32 command_type_A:2;
    uint32 command_type_B:4;
    uint32 start_srl_num:8;
    uint32 stop_srl_num:8;
    uint32 type_of_data_command:6;
    uint32 srl_num:8;
	uint32 TC_time:12;
    uint32 TC_parity:7;
    uint32 TC_appended_bit:1;
}DTTCformatExe;

DTTCformat DTTCarray[MAX_TIMETAG_CMD_LIMIT];
void rDifferentialTTC_Update(void);
void rDifferentialTTC_Execute(void);
void rInit_Block(void);
void rBlockTC_Execute(void);
unsigned long long int *commandptr;
//int DTTC_count = 0;
//uint32 Diff_start_srl_num = 0;
//uint32 Diff_stop_srl_num = 0;
//uint32 DTTCtime_reference;
//int DTTC_exe_flag = 1;

uint64 Block_array[MAX_BLKS][2 * MAX_BLK_CMD_SIZE];
uint32 Block_Index[MAX_BLKS];


#pragma pack(1)
    struct S_telecommand_data_generic	//Generic TC Structure
	{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 command:16;
	    uint32 type_of_data_command:6;
	    uint32 TC_time:20;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
	};

#pragma pack(1)
    struct S_telecommand_boolean		//Boolean TC Structure
	{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 offset_addr:11;
	    uint32 decision_bit:1;
	    uint32 TMTC_Buffer_addr:4;
	    uint32 type_of_data_command:6;
	    uint32 TC_time:20;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
	};

#pragma pack(1)
    struct S_telecommand_gain_select	//Gain Select TC Structure
	{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 offset_addr:8;
	    uint32 gain_set:4;
	    uint32 TMTC_Buffer_addr:4;
	    uint32 type_of_data_command:6;
	    uint32 TC_time:20;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
	} ;

#pragma pack(1)
    struct S_telecommand_data			//Data TC Structure
	{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
		uint32 Data:16;
		uint32 offset_addr:6;
	    uint32 TC_time:20;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
	} ;

#pragma pack(1)
    struct S_telecommand_func_exe		//Function-execute TC Structure
	{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
		uint32 offset_addr:22;
	    uint32 TC_time:20;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
	} ;

#pragma pack(1)
    struct S_telecommand_remote_data_addr		//Remote addr
	{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 filler1:2;
	    uint32 data_addr;
	    uint32 filler2:8;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
	} ;

#pragma pack(1)
typedef struct S_ATTC     //Data TC Structure
{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 command:16;
	    uint32 type_of_data_command:6;
	    uint32 TC_time:19;
	    uint32 filler:1;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
}ATTCformat;

// Structure modified on 18_05_2020
#pragma pack(1)
typedef struct TC_Block    //Data TC Structure
{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 command:16;
	    uint32 type_of_data_command:6;
	    uint32 Cmd_Srl:6;
	    uint32 Blk_opn:2;
	    uint32 Blk_No:6;
	    uint32 filler:6;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
} BLKformat;

/*
#pragma pack(1)
typedef struct TC_Block    //Data TC Structure
{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 command:16;
	    uint32 type_of_data_command:6;
	    uint32 Cmd_Srl:12;
	    uint32 Blk_No:8;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
} BLKformat;
*/


union U_telecommand						//Union of Telecommands
	{
    	unsigned long long int cmd_rcvd;
    	uint16 rcvd[4];
	    uint32 data_rcvd[2];
	    struct S_telecommand_data_generic        Frame;
	    struct S_telecommand_boolean             Bool;
	    struct S_telecommand_gain_select   		 GainSelect;
		struct S_telecommand_data 				 DataCommand;
		struct S_telecommand_func_exe			 FuncExecute;
		//struct S_telecommand_remote_addr		Remote_addr;
		struct S_telecommand_remote_data_addr	Remote;
		//struct S_telecommand_remote_data	Remote_data;
		BLKformat                               BLK_Update_cmd;
		ATTCformat          					ATTC_Exe_cmd;
		DTTCformat                              DTTC_Up_cmd;
		DTTCformatExe                           DTTC_Exe_cmd;
	} u_TC,u_DTTC;


union TC_Hist *TC_hist_write_ptr;
union TC_Hist *TC_hist_read_ptr;

typedef struct Node
{
	ATTCformat command;
	struct Node* next;
}Nodetype;
typedef Nodetype* Nodeptrtype ;

unsigned long long int Next_exe_TC;
// Following are data structures required to emulate malloc()
Nodetype Nodearray[MAX_TIMETAG_CMD_LIMIT];
Nodeptrtype Nodeptr[MAX_TIMETAG_CMD_LIMIT];

int Top_index;
/***************************/
union pending_TC_data
{
	unsigned long long int tc_pending_DT;
	ATTCformat command;
}TC_pend_data;

/****************************/
//-----------------------------------
//----------------BCHEncoder-------------
void invertReverse(unsigned char * startAddr);
void BCHEncoder(unsigned long long int * TCAddr);

#pragma pack(1)
typedef struct parityTC_tag
{
unsigned char bit7 : 1;
unsigned char bit6 : 1;
unsigned char bit5 : 1;
unsigned char bit4 : 1;
unsigned char bit3 : 1;
unsigned char bit2 : 1;
unsigned char bit1 : 1;
unsigned char fillerBit : 1;
}parityTC_t;

union parity_tag
{
	unsigned char parity;
    parityTC_t bits;
}TC_Rcvd;

//Function Declarations
void rTelecommand(void);				//Telecommand Routine
void TC_status_reset(void);
void rReal_Time_TC(void);				//Real time command processing
void rAbsolute_TimeTag_TC(void);		//Absolute timetag commmand processing
void rDifferential_TimeTag_TC(void);	//Differential timetag command processing
void rBlock_TC(void);					//Bulk telecommand processing
void rADCS_Data_TC(void);

void rContingency_TC(void);				//Contingency TC processing
void rData_TC(void);					//Data command processing
void rBoolean_TC(void);					//Boolean command processing
void rGainSelect_TC(void);				//Gain select command processing
void rFuncExecute_TC(void);             //Function execute command processing
void rRemoteProgram_Addr_TC(void);      //Remote program address processing
void rRemoteProgram_Data_TC(void);      //Remote program data processing
void TMTC_Assignment(void);             //Assigning of TC to TM
void rRemote_base_addr_TC(void);
void rRemote_data_view(void);


void adcsgains(void);                   //Gain_set_function
void TC_detumbling_bdot_gain_1(void);
void TC_detumbling_rate_gain_1(void);
void TC_BDOT_Det_Thresh_1(void);
void TC_GYRO_Det_Min_Thres_1(void);
void rTc_nominal_speed_RW1(void);
void rTc_nominal_speed_RW2(void);
void rTc_nominal_speed_RW3(void);
void rTc_nominal_speed_RW4(void);
void TC_AngMomDump_Thrsld_1(void);
void TC_SpeedDump_Thrsld_1(void);
void TC_SpeedDump_TimeSelect_1(void);
void TC_comd_pitch_rate_1(void);
void TC_Gyro_LPF_Gain_IMU1_1(void);
void TC_Gyro_LPF_Gain_IMU2_1(void);
void TC_Mag_LPF_Gain_IMU1_1(void);
void TC_Mag_LPF_Gain_IMU2_1(void);
void TC_Wheel_Cutoff_Threshold_1(void);



//Time tagged functions
void initNodetable(void);
Nodeptrtype getNode(void);
void freeNode(Nodeptrtype ptr);
void rAbsoluteTTC_Update(void);
void rAbsoluteTTC_Execute(void);
void rAbsoluteTTC_Delete(void);
void rAbsoluteTTC_Clear(void);
void rDifferential_TimeTag_Update(void);
void rDifferential_TimeTag_Execute(void);
void rPrintList(void);

//Function execute commands
void TC_IMU1_On(void);
void TC_IMU2_On(void);
void TC_IMU1_Off(void);
void TC_IMU2_Off(void);
void imu1_db_execute(void);
void imu2_db_execute(void);
void imu1_db_checksum(void);
void imu2_db_checksum(void);
void imu_test_sys_sel(void);
void TC_Drift_Uplink_Compenstation_Update_IMU1(void);
void TC_Drift_Uplink_Compenstation_Update_IMU2(void);
void TC_Gyro_Misalignment_Update_IMU1(void);
void TC_Gyro_Misalignment_Update_IMU2(void);
void TC_Gyro_Scale_Factor_Update_IMU1(void);
void TC_Gyro_Scale_Factor_Update_IMU2(void);
void TC_MagBias_Uplink_Compenstation_Update_IMU1(void);
void TC_MagBias_Uplink_Compenstation_Update_IMU2(void);
void TC_Mag_Misalignment_Update_IMU1(void);
void TC_Mag_Misalignment_Update_IMU2(void);
void TC_Mag_Scale_Factor_Update_IMU1(void);
void TC_Mag_Scale_Factor_Update_IMU2(void);

void rSSmain_ImaxF_update(void);
void rSSredt_ImaxF_update(void);
void TC_Gyro_LPF_Gain_Update_IMU1(void);
void TC_Gyro_LPF_Gain_Update_IMU2(void);

void TC_ACC_Ang_RESET(void);
//void TC_Panel1_Deploy(void);
void TC_NMI_count_reset(void);
void rMag_Refeci_update(void);
void TC_GPS1_ON(void);
void TC_GPS1_OFF(void);
void TC_GPS1_NMEA_VTG_enable(void);
void TC_GPS1_NMEA_VTG_disable(void);
void TC_GPS1_NMEA_GGA_enable(void);
void TC_GPS1_NMEA_GGA_disable(void);
void TC_GPS1_NMEA_GSA_enable(void);
void TC_GPS1_NMEA_GSA_disable(void);
void TC_GPS1_cold_start(void);
void TC_GPS1_factory_reset(void);
void TC_GPS2_on(void);
void TC_GPS2_off(void);
void TC_GPS2_NMEA_VTG_enable(void);
void TC_GPS2_NMEA_VTG_disable(void);
void TC_GPS2_NMEA_GGA_enable(void);
void TC_GPS2_NMEA_GGA_disable(void);
void TC_GPS2_NMEA_GSA_enable(void);
void TC_GPS2_NMEA_GSA_disable(void);
void TC_GPS2_cold_start(void);
void TC_GPS2_factory_reset(void);
void TC_W1_ON(void);
void TC_W2_ON(void);
void TC_W3_ON(void);
void TC_W4_ON(void);
void TC_W1_OFF(void);
void TC_W2_OFF(void);
void TC_W3_OFF(void);
void TC_W4_OFF(void);
void TC_Nominal_wheel_speed_execute(void);
void TC_MTR_ON(void);
void Pitch_Torquer_ON(void);
void Yaw_Torquer_ON(void);
void TC_MTR_OFF(void);
void Pitch_Torquer_OFF(void);
void Yaw_Torquer_OFF(void);
void rTC_HILS_ENABLE(void);									/* offset =    61 */ // replaced pitch torquer off (not used)
void rTC_HILS_DISABLE(void);
void TC_Detumbling_Mode_Select(void);
void TC_Safe_Mode_Select(void);
void TC_Threeaxis_Mode_Select(void);
void payload_1_on(void);
void payload_1_off(void);
void rHeater1_on(void);
void rHeater1_off(void);
void rHeater2_on(void);
void rHeater2_off(void);
void rHeater3_on(void);
void rHeater3_off(void);
void rHeater4_on(void);
void rHeater4_off(void);
void rHeater5_on(void);
void rHeater5_off(void);
void rHeater6_on(void);
void rHeater6_off(void);
void RX_TX_deployed(void);
void PL_K_CMD_STS(void);
void PL_K_CMD_ACQ(void);
void PL_K_CMD_HLT(void);
void PL_K_DIAG(void);
void PL_K_CMD_OFF(void);
void PL_K_CMD_ON(void);
void rTC_HILS_MODE_IDLE(void);
void rTC_HILS_MODE_START(void);
void rTC_HILS_MODE_STOP(void);
void rSun_Ephemeris_update(void);
void rTLE_Update(void);
void TC_init_RW1(void);
void TC_init_RW2(void);
void TC_init_RW3(void);
void TC_init_RW4(void);
void TC_MTR_Yaw_Positive(void);
void TC_MTR_Yaw_Negative(void);
void TC_MTR_Pitch_Positive(void);
void TC_MTR_Pitch_Negative(void);
void TC_MTR_Roll_Positive(void);
void TC_MTR_Roll_Negative(void);
void rDifferentialTTC_Execute(void);
void OBC_ON(void);
void OBC_OFF(void);
void Antenna_mechanism_ON(void);
void Antenna_mechanism_OFF(void);
void payload_2_on(void);
void payload_2_off(void);
void S_band_tx_on(void);
void S_band_tx_off(void);
void X_band_tx_on(void);
void X_band_tx_off(void);
void SA_DEPLOYMENT_MECHANISM_MAIN_ON(void);
void SA_DEPLOYMENT_MECHANISM_MAIN_OFF(void);
void SA_DEPLOYMENT_MECHANISM_RED_ON(void);
void SA_DEPLOYMENT_MECHANISM_RED_OFF(void);
void SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_ON(void);
void SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_OFF(void);
void Test_ON(void);
void Test_OFF(void);
void Antenna_mechanism_arm(void);
void Antenna_deploy(void);
void Antenna_mechanism_disarm(void);
void rTC_Detumbling_ModePreprocessing_BDOT(void);
void rTC_Detumbling_ModePreprocessing_GYRO(void);
void rTC_SunAcquisition_ModePreprocessing(void);
void rTC_ThreeAxis_ModePreprocessing(void);
void rTC_Suspended_ModePreprocessing(void);
void sunlit(void);
void eclipse(void);
void Sunlit_eclipse_both(void);
void rTC_Safe_mode_PreProcessing(void);
void TC_MTR_Roll_No_cuurent(void);
void TC_MTR_Pitch_No_cuurent(void);
void TC_MTR_Yaw_No_cuurent(void);
void TC_pl_debug(void);
void TC_pl_tm(void);
void TC_TM_DS_EN(void);
void Antenna_RESET_command(void);
void Antenna_deploy_with_override(void);
void Antenna_system_temp(void);
void Antenna_deploy_status_report(void);
void Antenna_deploy_activation_count(void);
void Antenna_deploy_activation_time(void);
void eeprom_en(void);
void eeprom_dis(void);
void TC_GPS1_NMEA_NMEA_GSV_Enable(void);
void TC_GPS1_NMEA_NMEA_GSV_disable(void);
void TC_GPS2_NMEA_NMEA_GSV_Enable(void);
void TC_GPS2_NMEA_NMEA_GSV_disable(void);
void TC_ATTC_CMD_clear(void);
void rTC_ref_snv_bias_q_update(void);
void rTC_ref_stn_bias_q_update(void);
void rTC_ref_q_gnd_update(void);
void TC_q_body_init(void);
void rElapsedTimerAssign(void);

// TC_history_view
void rTC_write_tC_history(void);

//Block Telecommand
void Block_update(void);
void gain_sets(void);
// TeleCommand

// TLE based array
	struct TLE_Update
	{
		int TC_TLE_data1;
		double TC_TLE_data2;
		float TC_TLE_data3;
		float TC_TLE_data4;
		float TC_TLE_data5;
		float TC_TLE_data6;
		double TC_TLE_data7;
		float TC_TLE_data8;
		float TC_TLE_data9;
		double TC_TLE_data10;
		unsigned int TC_TLE_data11;
		char TC_TLE_data12;
	}TLE_data;



	float TC_drift_compensation_IMU1[3], TC_drift_compensation_IMU2[3];
	float TC_magbias_compensation_IMU1[3], TC_magbias_compensation_IMU2[3];
	float TC_w_miscor_IMU1[6], TC_w_miscor_IMU2[6];
	float TC_w_sf_IMU1[3], TC_w_sf_IMU2[3];
	float TC_S_ECI[3], TC_B_ECI[3];
	float TC_SSmain_ImaxF[8], TC_SSredt_ImaxF[8];

// TLE based array
//float TC_TLE_data[TC_TLE_MAX_LIMIT];

#endif // TELECOMMAND
