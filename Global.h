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

//#define OLD_BOARD 4
#define ENGINEERING_MODEL 5
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
#define cPANEL_HEATER_TIMEOUT_CONST 78U //10 sec

#define Deploy		1
#define Not_Deploy	0

#define REG16(a) *((volatile uint16*) (a))   //For 16 Bit Addressing
#define REG8(a)  *((volatile uint8*)  (a))   //For  8 Bit Addressin
#define REG32(a) *((volatile uint32*) (a))   //For 32 Bit Addressing
#define REG32f(a) *((volatile float*) (a))   //For 32 Bit Addressing

//Heater
#define HEATER_1 			1
#define HEATER_2 			2
#define HEATER_3 			3
#define HEATER_4 			4
#define HEATER_5 			5
#define HEATER_6 			6

typedef enum SA1_status_enum
{
	SA1_NOT_DEPLOYED = 0,
	SA1_DEPLOY_CMD_RCVD,
	SA1_DEPLOYED,
	SA1_HEATER_TIME_OUT
}SA1_status_t;

typedef enum SA2_status_enum
{
	SA2_NOT_DEPLOYED = 0,
	SA2_DEPLOY_CMD_RCVD,
	SA2_DEPLOYED,
	SA2_HEATER_TIME_OUT
}SA2_status_t;

extern void rHAL_Heater_Power(unsigned int Heater_No,unsigned int Heater_Power);
//Function Declarations
void rPOR_Init(void);
unsigned long int Minor_Cycle_Count;
unsigned long int Major_Cycle_Count;
extern unsigned short TC_count;
extern void rPOR_Init();
extern void rTM_Copy_Subframe();
//extern unsigned long int Major_Cycle_Count;

//Telemetry
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

extern void rHAL_TM_Write(void);
extern void rHAL_TM_HW_Status_Update(void);
extern void rTM_Address_Table_Init(void);


//IMU
#define IMU1 1
#define IMU2 0

extern void rIMU_Init(void);
extern void rIMU_1_DB_Init(void);
extern void rIMU_2_DB_Init(void);
extern void rPOR_IMU_Parameters_Init(void);
extern void rIMU_1_DB_Copy(void);
extern void rIMU_2_DB_Copy(void);
extern void rHAL_IMU_POWER(unsigned long int IMU_No,unsigned long int IMU_Power);
extern void rIMU_Angle_Reset(void);

extern unsigned long int inter_HAL_IMU_Data;
extern unsigned long int inter_HAL_IMU_Addr_Count;
extern unsigned long int inter_HAL_IMU_Locations;
extern unsigned long int inter_HAL_IMU_Status_Data;
extern unsigned long int IMU_Config_Done;
extern unsigned long int IMU_Data_Available;
extern unsigned long int IMU_Diag_Done;

//ADC
extern void rHAL_ADC_Read(unsigned long int* ADC_Addr);
extern void rHAL_ADC_TM_Copy(unsigned long int* ADC_Addr);
extern void rHAL_ADC_StatusREG_Enable();

#define ADC_MAX_LIMIT 39
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
	BLK_Disabled = 64,
} BlkExe_Stat;

extern BlkExe_Stat BlkExe_Status;
extern void rBlockTC_Execute(void);
extern void rBlock0_exe(void);
extern uint32 BlkCurrent_Cmd;
extern uint32 rHAL_TC_Read();
extern void rTelecommand(void);

//extern void rReal_Time_TC();				//Real time command processing
//extern void rAbsolute_TimeTag_TC();		//Absolute timetag commmand processing
//extern void rDifferential_TimeTag_TC();	//Differential timetag command processing
//extern void rBlock_TC();					//Bulk telecommand processing


#define TC_data_command_MAX_LIMIT 73
#define TC_func_exe_MAX_LIMIT 129
extern float Resol_Table[TC_data_command_MAX_LIMIT];
extern void(*FuncExecute_Table[TC_func_exe_MAX_LIMIT])(void);
extern void TMTC_Assignment();             //Assigning of TC to TM
extern void rAbsoluteTTC_Execute();

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
extern unsigned long int GPS_Buffer_Data[300];
extern unsigned long int GPS_RCVD_DATA[60];
//extern unsigned long int rHAL_GPS_Read(unsigned long int GPS_No,unsigned long int* GPS_Addr,unsigned long int No_of_Bytes);
extern void rGPS_TM_Extract(void);
extern void rHAL_GPS_POWER(unsigned long int GPS_No,unsigned long int GPS_Power);
extern void rGPS_Buffer_Init();
extern void GPS_1_DATA();

//MTR
//#define MTR_Axis
//#define MTR_Polarity
extern unsigned int MTR_Reset_Flag;
extern unsigned int MTR_Axis;
extern unsigned int MTR_Polarity;
extern unsigned long int MTR_Current_Data;

extern void rHAL_MTR(void);
extern void rHAL_MTR_ON(void);
extern void rHAL_MTR_OFF(void);
extern void MTR_Monitoring();

//RW
extern void RW_Init(void);
extern void rRW_Data_Write();
extern void rRW_Data_Request();
extern void rRW_Data_Read();
extern void rRW_Ping_TC1();
extern void rRW_Ping_TC2();
extern void rRW_Ping_TC3();
extern void rRW_Ping_TC4();
extern void rHAL_RW_POWER(unsigned long int RW_No,unsigned long int RW_Status);

//Solar Arrays
extern unsigned long int TC_HAL_SA_Heater_Timer;
extern unsigned long int SA1_Deploy_cmd_rcvd_time;
extern unsigned long int SA2_Deploy_cmd_rcvd_time;
extern unsigned long int SA1_PanelHeater_Timeout;
extern unsigned long int SA2_PanelHeater_Timeout;

extern void rHAL_SA_Deploy(unsigned int SA_No, unsigned int SA_Status);
extern SA1_status_t SA1_status;
extern SA2_status_t SA2_status;
extern SA1_status_t rHAL_SA1_Deploy_status_check();
extern SA2_status_t rHAL_SA2_Deploy_status_check();
extern void rHAL_SA_Deploy_Status();

//Payload
#define Payload_1	0
#define Payload_2	1
#define X_Tx_ON     1
#define X_Tx_OFF    0
#define s_band_on   1
#define s_band_off  0
//extern void payload_2_on();
extern void PL_TM();
//extern void payload_2_off();
extern void rHAL_PL_CMD_ACQ();
extern void rHAL_PL_CMD_HLT();
extern void rHAL_PL_DIAG();
extern void rHAL_PL_CMD_OFF();
extern void rHAL_PL_CMD_ON();
extern void rHAL_PL_STS_Check();
extern void rHAL_PL_Power(unsigned int Payload_No,unsigned int status);
extern void rHAL_X_Tx_ON_OFF(unsigned int x_status);
extern void rHAL_Antenna_Deploy(unsigned int status);
extern void S_band_tx_on_off(unsigned int s_status);
extern void antennaCommand1(void);
extern void antennaCommand2(void);
extern void antennaCommand3(void);

//TC_Definitions
extern unsigned long int rHAL_GPS_Config(unsigned long int GPS_No,unsigned long int Config_Type);
extern void rHAL_GPS_POWER(unsigned long int GPS_No,unsigned long int GPS_Power);
extern void rIMU1_DB_Execute();
extern void rIMU2_DB_Execute();

//SS
extern void rSS_Main_DB_Execute();
extern void rSS_Redundant_DB_Execute();

extern void rOutput_Latch_Update();
extern void EEPROM_RES();
extern void EEPROM_RST();

//heater
extern void Heater_control_auto_manual();

unsigned long int aaaa; // testing
float bbbb; // testing
unsigned short bbb; // testing
unsigned short ccc; // testing


extern unsigned int m;

extern void gain_sets();//array for gain_sets

//Adcs_testing(To be removed)
extern void inputs(void);

#endif // GLOBAL
