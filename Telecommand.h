#ifndef TELECOMMAND
#define TELECOMMAND

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long int uint32;
typedef unsigned long long int uint64;


#define MAX_TIMETAG_CMD_LIMIT   1000
#define MAX_BLKS  64
#define MAX_BLK_CMD_SIZE 64
#define TC_TLE_MAX_LIMIT 17

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
void rDifferentialTTC_Update();
void rDifferentialTTC_Execute();
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

/*#pragma pack(1)
    struct S_telecommand_remote_data		//Remote Data
	{
	    uint32 start_bit:2;
	    uint32 spacecraft_id:4;
	    uint32 link_id:2;
	    uint32 command_type_A:2;
	    uint32 command_type_B:4;
	    uint32 filler1:2;
	    float data_addr;
	    uint32 filler2:8;
	    uint32 TC_parity:7;
	    uint32 TC_appended_bit:1;
	} ;*/

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

//Nodeptrtype Next_exe_TC;

unsigned long long int block_test_array[50];


typedef struct Node
{
	ATTCformat command;
	struct Node* next;
}Nodetype;
typedef Nodetype* Nodeptrtype ;

//Nodeptrtype head = NULL;

// Following are data structures required to emulate malloc()
Nodetype Nodearray[MAX_TIMETAG_CMD_LIMIT];
Nodeptrtype Nodeptr[MAX_TIMETAG_CMD_LIMIT];

//-----------testing--------------
//ATTCformat new_data;
//Nodeptrtype new_node;			//Create new node to be used for storing the TC data to the list
//Nodeptrtype temp;
//Nodeptrtype prev;
int Top_index;
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
void rTelecommand();				//Telecommand Routine
void TC_status_reset();
void rReal_Time_TC();				//Real time command processing
void rAbsolute_TimeTag_TC();		//Absolute timetag commmand processing
void rDifferential_TimeTag_TC();	//Differential timetag command processing
void rBlock_TC();					//Bulk telecommand processing
void rADCS_Data_TC();

void rContingency_TC();				//Contingency TC processing
void rData_TC();					//Data command processing
void rBoolean_TC();					//Boolean command processing
void rGainSelect_TC();				//Gain select command processing
void rFuncExecute_TC();             //Function execute command processing
void rRemoteProgram_Addr_TC();      //Remote program address processing
void rRemoteProgram_Data_TC();      //Remote program data processing
void TMTC_Assignment();             //Assigning of TC to TM
void rRemote_base_addr_TC();
void rRemote_data_view();


void adcsgains();                   //Gain_set_function
void TC_detumbling_bdot_gain_1();
void TC_detumbling_rate_gain_1();
void TC_BDOT_Det_Thresh_1();
void TC_GYRO_Det_Min_Thres_1();
void TC_AngMomDump_Thrsld_1();
void TC_SpeedDump_Thrsld_1();
void TC_SpeedDump_TimeSelect_1();
void TC_comd_pitch_rate_1();
void TC_Gyro_LPF_Gain_IMU1_1();
void TC_Gyro_LPF_Gain_IMU2_1();
void TC_Mag_LPF_Gain_IMU1_1();
void TC_Mag_LPF_Gain_IMU2_1();
void TC_Wheel_Cutoff_Threshold_1();



//Time tagged functions
void initNodetable();
Nodeptrtype getNode();
void freeNode(Nodeptrtype ptr);
void rAbsoluteTTC_Update();
void rAbsoluteTTC_Execute();
void rAbsoluteTTC_Delete();
void rAbsoluteTTC_Clear();
void rDifferential_TimeTag_Update();
void rDifferential_TimeTag_Execute();
void rPrintList();

//Function execute commands
void TC_IMU1_On();
void TC_IMU2_On();
void TC_IMU1_Off();
void TC_IMU2_Off();
void imu1_db_execute();
void imu2_db_execute();
void imu1_db_checksum();
void imu2_db_checksum();
void imu_test_sys_sel();
void TC_Drift_Uplink_Compenstation_Update_IMU1();
void TC_Drift_Uplink_Compenstation_Update_IMU2();
void TC_Gyro_Misalignment_Update_IMU1();
void TC_Gyro_Misalignment_Update_IMU2();
void TC_Gyro_Scale_Factor_Update_IMU1();
void TC_Gyro_Scale_Factor_Update_IMU2();
void TC_MagBias_Uplink_Compenstation_Update_IMU1();
void TC_MagBias_Uplink_Compenstation_Update_IMU2();
void TC_Mag_Misalignment_Update_IMU1();
void TC_Mag_Misalignment_Update_IMU2();
void TC_Mag_Scale_Factor_Update_IMU1();
void TC_Mag_Scale_Factor_Update_IMU2();

void TC_Mag_LPF_Gain_Update_IMU1();
void TC_Mag_LPF_Gain_Update_IMU2();
void TC_Gyro_LPF_Gain_Update_IMU1();
void TC_Gyro_LPF_Gain_Update_IMU2();

void TC_ACC_Ang_RESET();
//void TC_Panel1_Deploy();
void TC_NMI_count_reset();
void TC_Panel2_Deploy();
void TC_GPS1_ON();
void TC_GPS1_OFF();
void TC_GPS1_NMEA_VTG_enable();
void TC_GPS1_NMEA_VTG_disable();
void TC_GPS1_NMEA_GGA_enable();
void TC_GPS1_NMEA_GGA_disable();
void TC_GPS1_NMEA_GSA_enable();
void TC_GPS1_NMEA_GSA_disable();
void TC_GPS1_cold_start();
void TC_GPS1_factory_reset();
void TC_GPS2_on();
void TC_GPS2_off();
void TC_GPS2_NMEA_VTG_enable();
void TC_GPS2_NMEA_VTG_disable();
void TC_GPS2_NMEA_GGA_enable();
void TC_GPS2_NMEA_GGA_disable();
void TC_GPS2_NMEA_GSA_enable();
void TC_GPS2_NMEA_GSA_disable();
void TC_GPS2_cold_start();
void TC_GPS2_factory_reset();
void TC_W1_ON();
void TC_W2_ON();
void TC_W3_ON();
void TC_W4_ON();
void TC_W1_OFF();
void TC_W2_OFF();
void TC_W3_OFF();
void TC_W4_OFF();
void TC_Nominal_wheel_speed_execute();
void TC_MTR_ON();
void Pitch_Torquer_ON();
void Yaw_Torquer_ON();
void TC_MTR_OFF();
void Pitch_Torquer_OFF();
void Yaw_Torquer_OFF();
void TC_Detumbling_Mode_Select();
void TC_Safe_Mode_Select();
void TC_Threeaxis_Mode_Select();
void payload_1_on();
void payload_1_off();
void rHeater1_on();
void rHeater1_off();
void rHeater2_on();
void rHeater2_off();
void rHeater3_on();
void rHeater3_off();
void rHeater4_on();
void rHeater4_off();
void rHeater5_on();
void rHeater5_off();
void rHeater6_on();
void rHeater6_off();
void RX_TX_deployed();
void PL_K_CMD_STS();
void PL_K_CMD_ACQ();
void PL_K_CMD_HLT();
void PL_K_DIAG();
void PL_K_CMD_OFF();
void PL_K_CMD_ON();
void ss_main_db_execute();
void ss_redundant_db_execute();
void ss_main_db_checksum();
void ss_redundant_db_checksum();
void TC_Qinit();
void TC_init_RW1();
void TC_init_RW2();
void TC_init_RW3();
void TC_init_RW4();
void TC_MTR_Yaw_Positive();
void TC_MTR_Yaw_Negative();
void TC_MTR_Pitch_Positive();
void TC_MTR_Pitch_Negative();
void TC_MTR_Roll_Positive();
void TC_MTR_Roll_Negative();
void rDifferentialTTC_Execute();
void OBC_ON();
void OBC_OFF();
void Antenna_mechanism_ON();
void Antenna_mechanism_OFF();
void payload_2_on();
void payload_2_off();
void S_band_tx_on();
void S_band_tx_off();
void X_band_tx_on();
void X_band_tx_off();
void SA_DEPLOYMENT_MECHANISM_MAIN_ON();
void SA_DEPLOYMENT_MECHANISM_MAIN_OFF();
void SA_DEPLOYMENT_MECHANISM_RED_ON();
void SA_DEPLOYMENT_MECHANISM_RED_OFF();
void SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_ON();
void SA1_DEPLOYMENT_AND_SA2_DEPLOYMENT_REDUNDANT_SPARE_BUS_OFF();
void Test_ON();
void Test_OFF();
void Antenna_mechanism_arm();
void Antenna_deploy();
void Antenna_mechanism_disarm();
void rTC_Detumbling_ModePreprocessing_BDOT();
void rTC_Detumbling_ModePreprocessing_GYRO();
void rTC_SunAcquisition_ModePreprocessing();
void rTC_ThreeAxis_ModePreprocessing();
void rTC_Suspended_ModePreprocessing();
void sunlit();
void eclipse();
void Sunlit_eclipse_both();
void rSafe_mode_PreProcessing();
void TC_MTR_Roll_No_cuurent();
void TC_MTR_Pitch_No_cuurent();
void TC_MTR_Yaw_No_cuurent();
void TC_pl_debug();
void TC_pl_tm();
void TC_TM_DS_EN();
void Antenna_RESET_command();
void Antenna_deploy_with_override();
void Antenna_system_temp();
void Antenna_deploy_status_report();
void Antenna_deploy_activation_count();
void Antenna_deploy_activation_time();
void eeprom_en();
void eeprom_dis();
void TC_GPS1_NMEA_NMEA_GSV_Enable();
void TC_GPS1_NMEA_NMEA_GSV_disable();
void TC_GPS2_NMEA_NMEA_GSV_Enable();
void TC_GPS2_NMEA_NMEA_GSV_disable();
void TC_ATTC_CMD_clear();

//void ATTC_exe_en_flag();
//void Diff_Funtion_exe();

// TC_history_view
void rTC_write_tC_history();

//Block Telecommand
void Block_update();
void gain_sets();
// TeleCommand

// TLE based array
unsigned long int TC_TLE_data[TC_TLE_MAX_LIMIT];

#endif // TELECOMMAND
