typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long int uint32;
typedef unsigned long long int uint64;

typedef char int8;
typedef short int16;
typedef long int int32;
typedef long long int int64;

#ifndef GLOBAL
#define GLOBAL

#define PROTO_BOARD 4  //OLD_BOARD
//#define QUALIFICATION_MODEL 5  //ENGINEERING_MODEL
//#define FLIGHT_MODEL 6

#ifndef NULL
#define NULL ((void*)0)
#endif



//Generic Definitions
#define True 1
#define False 0
#define Enable 1
#define Disable 0
#define Set 1
#define Reset 0
#define TRUE 1
#define FALSE 0

#define OFF 	0
#define ON 		1
#define RESET 	2

#define Roll  1
#define Pitch 2
#define Yaw   3

#define No_Current 0
#define Positive   1
#define Negative   -1

#define RW1 1
#define RW2 2
#define RW3 3
#define RW4 4

#define SA_MAIN 1
#define SA_RED 2
#define SA_CMD_NOT_RCVD 0
#define cPANEL_HEATER_TIMEOUT_CONST 16U // 2sec      //   78U // 10 sec

#define Deploy		1
#define Not_Deploy	0

#define REG16(a) *((volatile uint16*) (a))   //For 16 Bit Addressing
#define REG8(a)  *((volatile uint8*)  (a))   //For  8 Bit Addressin
#define REG32(a) *((volatile uint32*) (a))   //For 32 Bit Addressing
#define REG32f(a) *((volatile float*) (a))   //For 32 Bit Addressing

#define TC_HISTORY_MAX 4000

#define byte_swap(a) (unsigned short)(((a&0xFF00)>>8)|((a&0x00FF)<<8))

//Heater
#define HEATER_1 			1
#define HEATER_2 			2
#define HEATER_3 			3
#define HEATER_4 			4
#define HEATER_5 			5
#define HEATER_6 			6


/************************* RAM Scrubbing ***************************/
/************RAM_SEG_START_ADDR and RAM_SEG_END_ADDR means data segment of RAM*********/
#define RAM_SEG_START_ADDR     0x40042d70	    // Scrub start address
#define RAM_SEG_END_ADDR       0x40043294		// Scrub end address
#define RAM_END_ADDR           0x4FFFFFFF		// SRAM end address Note: complete memory scrub is not required.
#define BLOCK_SIZE 	           4096/4 		// 4K Bytes
void s_ram_scrub();

/********************************************************************/


/************************* EEPROM CHECKSUM ************************/
// EEPROM Size: (128k * 32)/8    = 512KB
//EEPROM used: 279KB(without compressing)

#define	EEPROM_START_ADDR    0x00000000
#define	EEPROM_END_ADDR      0x0001F400                //***0x000441D8 = 279KB***//                           //***0x0001F400 = 128KB***//
#define	EEPROM_BLOCK_SIZE    64000                      //***883B = 34875***//
void prom_chksum(void);

/**************************************************/

typedef enum SA1_status_enum
{
	SA1_NOT_DEPLOYED = 0,
	SA1_DEPLOY_CMD_RCVD,
	SA1_DEPLOYED,
	SA1_HEATER_TIME_OUT,
	SA1_OVER_HEAT
}SA1_status_t;

typedef enum SA2_status_enum
{
	SA2_NOT_DEPLOYED = 0,
	SA2_DEPLOY_CMD_RCVD,
	SA2_DEPLOYED,
	SA2_HEATER_TIME_OUT,
	SA2_OVER_HEAT
}SA2_status_t;

#pragma(1)                    //gps_initialize structure
struct HAL_GPS_registers
{
	unsigned long int GPS_Buffer_addr;
	unsigned long int GPS_Status_Register_1;
	unsigned long int GPS_Status_Register_2;

	unsigned long int GPS_Config_1;
	unsigned long int GPS_Config_2;
	unsigned long int GPS_Config_3;
	unsigned long int GPS_Config_4;
	unsigned long int GPS_Config_5;
	unsigned long int GPS_Config_6;
	unsigned long int GPS_Config_7;
	unsigned long int GPS_Config_8;
	unsigned long int GPS_Config_9;
	unsigned long int GPS_Config_10;
	unsigned long int GPS_Config_11;
	unsigned long int GPS_Config_12;
	unsigned long int GPS_Config_13;

}GPS1,GPS2;


extern void rHAL_Heater_Power(unsigned int Heater_No,unsigned int Heater_Power);
//Function Declarations
void rPOR_Init(void);
unsigned long int Minor_Cycle_Count;
unsigned long int Major_Cycle_Count;
extern unsigned short TC_count;
extern unsigned short TC_cmd_executed;
extern uint32 ATTC_count;
extern unsigned short TC_command_pending;
extern void rPOR_Init();
extern void rTM_Copy_Subframe();
//extern unsigned long int Major_Cycle_Count;

//Telemetry
extern unsigned short* inter_TM_ST_TC_NS_Write_Source_Addr;
extern unsigned long int inter_TM_Main_Buffer_Empty;
extern unsigned long int inter_TM_Byte_Count;
extern unsigned long int* addr_pointer_TM;
extern unsigned long int IO_In_Latch_Register_4_Data;
extern unsigned long int IO_Latch_Register_4_Data;
extern char* inter_TM_Dest_Addr;
extern char* inter_TM_Source_Addr;
extern unsigned long int inter_Update_TM_With_ADC_offset;
extern unsigned long int inter_Update_TM_With_ADC_byte_offset;
void GPIO();
extern unsigned char* write_str_ptr;
//extern unsigned char* TC_write_str_ptr;
extern unsigned char* Dest_end_addr; 							    // initializing Destination end address
extern unsigned char* Dest_TC_final_end_addr;
extern unsigned int Normal_st_table_page1, Normal_st_table_page2, Normal_st_table_page3;
extern unsigned int Special_st_table_page1, Special_st_table_page2, Special_st_table_page3;
extern unsigned char* ST_Dest_Addr;
//extern unsigned char TC_storing_buffer[256];
extern unsigned int TCH_read_ptr;
extern unsigned int TCH_read_full_ptr;
extern unsigned int TCH_cpy_ptr;
extern unsigned int TC_storing_buffer[256];
extern unsigned int TC_buffer_count[300];
extern unsigned short ST_frame_count;
extern unsigned short ST_TCH_frame_count;

extern void rHAL_TM_HW_Status_Update(void);
extern void rTM_Address_Table_Init(void);
extern void Storage_Telemetry_Write();
extern void rTCH_full_dump_cpy_buf();
extern void rTCH_dump_cpy_buf();
extern void Norm_ST_1_Table_Init();
extern void TC_hist_view();
extern void Spec_ST_Table_Init();
extern void ST_DUMPING();
extern void ST_full_dump() ;
extern void TC_Hist_dumping();
extern void rTM_Real_st_write();

//IMU
#define IMU1 1
#define IMU2 0

extern void rIMU_Init(void);
extern void rIMU_1_DB_Init(void);
extern void rIMU_2_DB_Init(void);
extern void rPOR_IMU_Parameters_Init(void);
extern void rIMU_1_DB_Copy(void);
extern void rIMU_2_DB_Copy(void);
extern void rIMU_Angle_Reset(void);

extern unsigned long int inter_HAL_IMU_Data;
extern unsigned long int inter_HAL_IMU_Addr_Count;
extern unsigned long int inter_HAL_IMU_Locations;
extern unsigned long int inter_HAL_IMU_Status_Data;
extern unsigned long int IMU_Config_Done;
extern unsigned long int IMU_Data_Available;
extern unsigned long int IMU_Diag_Done;
extern unsigned long int  IMU_temperature;

//ADC
extern void rHAL_ADC_Read(unsigned long int* ADC_Addr);
extern void rHAL_ADC_TM_Copy(unsigned long int* ADC_Addr);
extern void sun_Sensor_data();
extern void rHAL_ADC_StatusREG_Enable();
extern unsigned short UTP_data;
extern unsigned short LTP_data;

#define ADC_MAX_LIMIT 47
#define ADC_Thermistor 14
#define ADC_SUN_SENSOR_MAX_LIMIT 16
unsigned long int ADC_Buffer[ADC_MAX_LIMIT];
#define EPS_RAW_BUS_VOLTAGE ADC_Buffer[34]
#define rBattery_temperature_1  ADC_Buffer[0]

extern unsigned long int inter_HAL_ADC_Data_Ready;
extern unsigned long int ADC_Status_Data;
extern unsigned long int ADC_Addr_Count;
extern unsigned long int ADC_Locations;
extern unsigned long int ADC_Data;
extern  void rHAL_EPS_shunt_switch();

//TC
typedef enum BlkExe_Stat_tag
{
	BLK_0 = 0,
	BLK_1 = 1,
	BLK_2 = 2,
	BLK_3 = 3,
	BLK_4 = 4,
	BLK_5 = 5,
	BLK_6 = 6,
	BLK_7 = 7,
	BLK_Disabled = 64,
} BlkExe_Stat;

struct TC_history
 {
 	unsigned long long int  TC_cmd_executed;
 	unsigned long int TC_exe_count;
 	unsigned long int OBT_time_stamp;
 };

/* Update the tch memory init as following */
 union TC_Hist{
	 // unsigned char data[16];
	  struct TC_history TC_execute_hist;
	  unsigned char TC_execute_hist_1[sizeof(struct TC_history)];
 }TC_hist_data[TC_HISTORY_MAX];

extern union TC_Hist *TC_hist_write_ptr;
extern union TC_Hist *TC_hist_read_ptr;
//extern union TC_Hist *TCH_write_ptr;
extern BlkExe_Stat BlkExe_Status;
extern void rBlockTC_Execute(void);
extern void rBlock0_exe(void);
extern uint32 BlkCurrent_Cmd;
extern uint32 rHAL_TC_Read();
extern void rTelecommand(void);
extern void TC_status_reset();
extern unsigned int  TC_cmd_executed_count;
extern union TC_Hist *TC_hist_read_ptr;

extern unsigned long long int block_test_array[50];  //to be removed

//extern void rReal_Time_TC();				//Real time command processing
//extern void rAbsolute_TimeTag_TC();		//Absolute timetag commmand processing
//extern void rDifferential_TimeTag_TC();	//Differential timetag command processing
//extern void rBlock_TC();					//Bulk telecommand processing


#define TC_data_command_MAX_LIMIT 27
#define ADCS_TC_data_command_MAX_LIMIT 57
#define TC_func_exe_MAX_LIMIT 150
extern float Resol_Table[TC_data_command_MAX_LIMIT];
extern float Resol_Table_Adcs[ADCS_TC_data_command_MAX_LIMIT];
extern void(*FuncExecute_Table[TC_func_exe_MAX_LIMIT])(void);
extern void TMTC_Assignment();             //Assigning of TC to TM
extern void rAbsoluteTTC_Execute();
extern int Storage_TM_dumping;
extern void rRemote_data_view();

//extern int TC_detumbling_bdot_gain_set;
extern void Block_update();
extern void rInit_Block();

//SunSensors
extern void rSS_Main_DB_Init(void);
extern void rSS_Redundant_DB_Init(void);
extern void rSS_Main_DB_Copy(void);
extern void rSS_Redundant_DB_Copy(void);

//GPS
extern unsigned long int GPS_Data_Read_Status;
extern unsigned long int GPS_Buffer_Data[106];
extern unsigned long int GPS_RCVD_DATA[106];
extern void rGPS_TM_Extract(void);
//extern void rHAL_GPS_POWER(unsigned long int GPS_No,unsigned long int GPS_Power);
extern void rGPS_Buffer_Init();
extern void GPS_1_DATA();
extern void ST_TM_gps_data();


//MTR
//#define MTR_Axis
//#define MTR_Polarity
extern unsigned int MTR_Reset_Flag;
extern unsigned int MTR_Axis;
extern unsigned int MTR_Polarity;
extern unsigned long int MTR_Current_Data;
void rHAL_MTR_TC(unsigned long int MTR_Axis,int MTR_Polarity);

extern void rHAL_MTR(void);
extern void rHAL_MTR_ON(void);
extern void rHAL_MTR_OFF(void);
extern void MTR_Monitoring();

//RW
extern void RW_Init(void);
extern void rRW_Data_Write();
extern void rRW_Data_Request();
extern void rRW_Data_Read();

extern struct HAL_RW_Data_Structure RW_1,RW_2,RW_3,RW_4;

extern void rRW_init_cmd(struct HAL_RW_Data_Structure RW_No, unsigned char RW_ID);

//NMI
extern unsigned char NMI_fail_count;

//Solar Arrays
extern unsigned long int TC_HAL_SA_Heater_Timer;
extern unsigned long int SA1_Deploy_cmd_rcvd_time;
extern unsigned long int SA2_Deploy_cmd_rcvd_time;
extern unsigned long int SA1_PanelHeater_Timeout;
extern unsigned long int SA2_PanelHeater_Timeout;

extern void rHAL_SA_MAIN_Deploy_on();
extern void rHAL_SA_RED_Deploy_on();
extern SA1_status_t SA1_status;
extern SA2_status_t SA2_status;
extern SA1_status_t rHAL_SA1_Deploy_status_check();
extern SA2_status_t rHAL_SA2_Deploy_status_check();
extern void rHAL_SA_Deploy_Status();
extern void rHAL_SA_Deploy_Status_new();

//Payload
#define PAYLOAD_1	0
#define PAYLOAD_2	1
#define X_Tx_ON     1
#define X_Tx_OFF    0
#define s_band_on   1
#define s_band_off  0
//extern void payload_2_on();
extern void PL_TM();
//extern void payload_2_off();
extern void rHAL_pl_sts_check();
extern void rHAL_pl_cmd_acq();
extern void rHAL_pl_cmd_hlt();
extern void rHAL_pl_diag(void);
extern void rHAL_pl_x_tx_data_on();
extern void rHAL_pl_x_tx_data_off();
extern void rHAL_pl_debug(void);
extern void TC_tm_ds_en();
//extern void rTC_pl_tx_tm();
extern void rHAL_tm_ds_en();
extern void pl_tx_tm();

extern unsigned int pl_cmd_id;

extern void rHAL_pl1_ON();
extern void rHAL_pl2_ON();
extern void rHAL_pl1_OFF();
extern void rHAL_pl2_OFF();

extern void rHAL_X_Tx_ON();
extern void rHAL_X_Tx_OFF();
extern void rHAL_Antenna_Deploy(unsigned int status);
extern void antennaCommand1(void);
extern void antennaCommand2(void);
extern void antennaCommand3(void);
extern void antennaCommand4(void);
extern void antennaCommand5(void);
extern void antennaCommand6(void);
extern void antennaCommand7(void);
extern void antennaCommand8(void);
extern void antennaCommand9(void);
extern void rHAL_Antenna_Read();

extern void rpl_read();
extern void pl_tx_tm_2();
extern void rpl_tm_write();
extern void rpl_init();
extern void rpl_sts();
extern unsigned long int inter_TM_Pl_Buffer;

extern void MUX_Output();        //EPS_card_MUX_switch_status

//TC_Definitions
unsigned long int rHAL_GPS_Config(struct HAL_GPS_registers GPS_No,unsigned long int Config_Type);
//extern void rHAL_GPS_POWER(unsigned long int GPS_No,unsigned long int GPS_Power);
extern void rIMU1_DB_Execute();
extern void rIMU2_DB_Execute();

//SS
extern void rSS_Main_DB_Execute();
extern void rSS_Redundant_DB_Execute();

extern void rOutput_Latch_Update();
extern void ST_output_update();
extern void NMI_interrupt_test();
extern void EEPROM_RES();
extern void EEPROM_RST();

//heater
extern void Heater_control_auto_manual();
extern void Thermister_select();

unsigned long int aaaa; // testing
float bbbb; // testing
unsigned short bbb; // testing
unsigned short ccc; // testing

extern unsigned int Remote_data_addr;    // Remote_addr_data_variable
extern unsigned short PL_TM_Status_flag; // payload TM_transmit flag

extern unsigned int m;

extern void gain_sets();//array for gain_sets

//Adcs_testing(To be removed)
extern void inputs(void);

// manual heater previous telecommand memory before load shedding
extern unsigned char load_shedding_flag;
extern unsigned char low_battery_voltage;
extern int f_battery_safemode;

union heaters_manual_data
{		unsigned short data;
		struct
		{
				unsigned heater_16  :1;
				unsigned heater_15  :1;
				unsigned heater_14  :1;
				unsigned heater_13  :1;
				unsigned heater_12  :1;
				unsigned heater_11  :1;
				unsigned heater_10  :1;
				unsigned heater_9  	:1;
				unsigned heater_8  	:1;
				unsigned heater_7  	:1;
				unsigned heater_6  	:1;
				unsigned heater_5  	:1;
				unsigned heater_4  	:1;
				unsigned heater_3  	:1;
				unsigned heater_2  	:1;
				unsigned heater_1  	:1;
		};
}heaters_manual;

extern union heaters_manual_data heaters_manual;

extern unsigned short pl_ack_count; //Payload acknowledgement count




/****************** Remote Patch function declaration *************/

/***************** Remote Patch hook function *******************/
extern void rSus_mode_remote_entry_hook();

extern void rSus_mode_remote_exit_hook();

extern void rDBDOT_mode_remote_entry_hook();

extern void rDBDOT_mode_remote_entry_hook();

extern void rDBDOT_mode_remote_exit_hook();

extern void rDGYRO_mode_remote_entry_hook();

extern void rDGYRO_mode_remote_exit_hook();

extern void rSACQ_mode_remote_entry_hook();

extern void rSACQ_mode_remote_exit_hook();

extern void r3AXIS_mode_remote_entry_hook();

extern void r3AXIS_mode_remote_exit_hook();

extern void rSAFE_mode_remote_entry_hook();

extern void rSAFE_mode_remote_exit_hook();


/*************************************************/
/************* Patch area declaration ************/

extern void rRemote_patch_area_1();

extern void rRemote_patch_area_2();

extern void rRemote_patch_area_3();

extern void rRemote_patch_area_4();

extern void rRemote_patch_area_5();

extern void rRemote_patch_area_6();

/*************************************************/
#endif // GLOBAL
