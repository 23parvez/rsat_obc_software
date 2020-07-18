#ifndef HAL_GLOBAL
#define HAL_GLOBAL

#define EXTRACT_LSB_16 0x0000FFFF
#define EXTRACT_LSB_08 0x000000FF
#define EXTRACT_MSB_16 0xFFFF0000
#define EXTRACT_LSB_4  0x0000000F
#define EXTRACT_LSB_16_RESET 0x00000000



#define MINOR_CYCLE_TERMINAL_COUNT 0x00008000u
#define MAJOR_CYCLE_TERMINAL_COUNT 0x00004000u
#define MINOR_CYCLE_RESET 		   0x00000001u
#define MAJOR_CYCLE_RESET 		   0x00000002u
#define SA1_DEPLOY_ST              0x00000000u
#define SA1_DEPLOY_DEF_ST          0x00000200u
#define SA2_DEPLOY_ST              0x00000000u
#define SA2_DEPLOY_DEF_ST          0x00000100u

#pragma pack(1)
union Out_Latch_1
{
	unsigned short data;
	struct
	{
		unsigned filler			:4;
		unsigned FP_CTRL_MTR3_N2:1;
		unsigned FP_CTRL_MTR3_N1:1;
		unsigned FP_CTRL_MTR3_P2:1;
		unsigned FP_CTRL_MTR3_P1:1;
		unsigned FP_CTRL_MTR2_N2:1;
		unsigned FP_CTRL_MTR2_N1:1;
		unsigned FP_CTRL_MTR2_P2:1;
		unsigned FP_CTRL_MTR2_P1:1;
		unsigned FP_CTRL_MTR1_N2:1;
		unsigned FP_CTRL_MTR1_N1:1;
		unsigned FP_CTRL_MTR1_P2:1;
		unsigned FP_CTRL_MTR1_P1:1;
	};
}Out_Latch_1;

#pragma pack(1)
union Out_Latch_2
{
	unsigned short data;
	struct
	{
		unsigned filler		  :5; //4;
		unsigned SPARE2_ON_OFF:1;
	//	unsigned RF_Tx_ON_OFF:1;
		unsigned X_Tx_ON_OFF:1;
		unsigned MTR_ON_OFF	:1;
		unsigned GPS2_RESET	:1;
		unsigned GPS2_ON_OFF:1;
		unsigned GPS1_RESET	:1;
		unsigned GPS1_ON_OFF:1;
		unsigned IMU2_RESET	:1;
		unsigned IMU2_ON_OFF:1;
		unsigned IMU1_RESET	:1;
		unsigned IMU1_ON_OFF:1;
	};
}Out_Latch_2;

#pragma pack(1)
union ST_Out_Latch_2
{
	unsigned short data;
	struct
	{
		unsigned filler		  :5; //4;
		unsigned SPARE2_ON_OFF:1;
	//	unsigned RF_Tx_ON_OFF:1;
		unsigned X_Tx_ON_OFF:1;
		unsigned MTR_ON_OFF	:1;
		unsigned GPS2_RESET	:1;
		unsigned GPS2_ON_OFF:1;
		unsigned GPS1_RESET	:1;
		unsigned GPS1_ON_OFF:1;
		unsigned IMU2_RESET	:1;
		unsigned IMU2_ON_OFF:1;
		unsigned IMU1_RESET	:1;
		unsigned IMU1_ON_OFF:1;
	};
}ST_Out_Latch_2;


#pragma pack(1)
union Out_Latch_3
{
	unsigned short data;
	struct
	{
		unsigned filler		   :3;
		unsigned TM_DS_EN      :1;
		unsigned PL2_ON_OFF	   :1;
		unsigned PL1_ON_OFF	   :1;
		unsigned RW4_ON_OFF	   :1;
		unsigned RW3_ON_OFF	   :1;
        unsigned RW2_ON_OFF	   :1;
		unsigned RW1_ON_OFF	   :1;
		unsigned Heater6_ON_OFF:1;
		unsigned Heater5_ON_OFF:1;
		unsigned Heater4_ON_OFF:1;
		unsigned Heater3_ON_OFF:1;
		unsigned Heater2_ON_OFF:1;
		unsigned Heater1_ON_OFF:1;
	};
}Out_Latch_3;

#pragma pack(1)
union ST_Out_Latch_3
{
	unsigned short data;
	struct
	{
		unsigned filler		   :3;
		unsigned TM_DS_EN      :1;
		unsigned PL2_ON_OFF	   :1;
		unsigned PL1_ON_OFF	   :1;
		unsigned RW4_ON_OFF	   :1;
		unsigned RW3_ON_OFF	   :1;
        unsigned RW2_ON_OFF	   :1;
		unsigned RW1_ON_OFF	   :1;
		unsigned Heater6_ON_OFF:1;
		unsigned Heater5_ON_OFF:1;
		unsigned Heater4_ON_OFF:1;
		unsigned Heater3_ON_OFF:1;
		unsigned Heater2_ON_OFF:1;
		unsigned Heater1_ON_OFF:1;
	};
}ST_Out_Latch_3;


//-----------latch_4--------//
#pragma pack(1)
union Out_latch_4
{
	unsigned short data;
	struct
	{
		unsigned EEPROM_RESET   :1;
		unsigned filler2		:7;
		unsigned MajorCycle_RST	:1;
		unsigned filler1		:5;
		unsigned OBT_Reset		:1;
		unsigned MinorCycle_RST	:1;
	};
}Out_latch_4;


//----------latch_5------//
#pragma pack(1)
union Out_latch_5
{
unsigned short data;
	struct
	{
		unsigned filler			:6;
		unsigned ANTENNA_DEPLOY	:1;
		unsigned SPARE1_ON_OFF	:1;
		unsigned SA2_DEPLOY		:1;
		unsigned SA1_DEPLOY		:1;
		unsigned SA3_ON_OFF		:1;
		unsigned SA2_ON_OFF		:1;
		unsigned SA1_ON_OFF		:1;
		unsigned SEL_2			:1;
		unsigned SEL_1			:1;
		unsigned SEL_0			:1;
	};
}Out_latch_5;

#pragma pack(1)
union ST_Out_latch_5
{
unsigned short data;
	struct
	{
		unsigned filler			:6;
		unsigned ANTENNA_DEPLOY	:1;
		unsigned SPARE1_ON_OFF	:1;
		unsigned SA2_DEPLOY		:1;
		unsigned SA1_DEPLOY		:1;
		unsigned SA3_ON_OFF		:1;
		unsigned SA2_ON_OFF		:1;
		unsigned SA1_ON_OFF		:1;
		unsigned SEL_2			:1;
		unsigned SEL_1			:1;
		unsigned SEL_0			:1;
	};
}ST_Out_latch_5;

//----------latch_6------//
#pragma pack(1)
union Out_latch_6
{
	unsigned short data;
	struct
	{
		unsigned filler 		:15;
		unsigned RF_Tx_ON_OFF	:1;
	};
}Out_latch_6;

#pragma pack(1)
union input_latch_1
{
unsigned short data;
	struct
	{
		unsigned FP_MTR1_mon1_mon2	:2;
		unsigned FP_MTR2_mon1_mon2	:2;
		unsigned FP_MTR3_mon1_mon2	:2;
		unsigned VOBC_TM		    :1;
		unsigned Demod_lock_Ind	    :1;
		unsigned filler			    :7;

	};
}input_latch_1;

union input_latch_2
{
	unsigned short data;
	unsigned char data_8bit[2];
	struct
	{
		unsigned Minor       :1;
		unsigned Major       :1;
		unsigned MUX1_OUT    :1;
		unsigned MUX2_OUT    :1;
		unsigned MUX3_OUT    :1;
		unsigned MUX4_OUT    :1;
		unsigned SA1_deploy_st:1;
		unsigned SA2_deploy_st:1;
		unsigned filler       :8;

	};

}input_latch_2;

#pragma pack(1)
union GPIO
{
	unsigned short data;
	struct
	{
		unsigned PIO_15 :1;
		unsigned PIO_14 :1;
		unsigned PIO_13 :1;
		unsigned PIO_12 :1;
		unsigned PIO_11 :1;
		unsigned PIO_10 :1;
		unsigned PIO_9  :1;
		unsigned PIO_8  :1;
		unsigned PIO_7  :1;
		unsigned PIO_6  :1;
		unsigned PIO_5  :1;
		unsigned PIO_4  :1;
		unsigned PIO_3  :1;
		unsigned PIO_2  :1;
		unsigned PIO_1  :1;
		unsigned PIO_0  :1;
	};
}GPIO_pins;

struct nml_output_latch
{
	unsigned char output_latch_data_1;
	unsigned char output_latch_data_2;
	unsigned char output_latch_data_3;
}nml_output_latch_data;

union heaters_auto_manual
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
	 	 	 unsigned heater_9   :1;
	 	 	 unsigned heater_8   :1;
	 	 	 unsigned heater_7   :1;
	 	 	 unsigned heater_6   :1;
	 	 	 unsigned heater_5   :1;
	 	 	 unsigned heater_4   :1;
	 	 	 unsigned heater_3   :1;
	 	 	 unsigned heater_2   :1;
	 	 	 unsigned heater_1   :1;
		};
}heaters;

struct HAL_RW_Data_Structure
{
	unsigned long int RW_Configure_Register;
	unsigned long int RW_Status_Register_1;
	unsigned long int RW_Status_Register_2;
	unsigned long int RW_Buffer_Register;
};


//payload union
union PL_DATA_STORAGE
{
unsigned char pl_data_1byte[12];
unsigned short pl_data_2byte[6];
struct
{
unsigned Data_command_1:16;
unsigned Data_command_2:16;
unsigned Data_command_3:16;
unsigned Data_command_4:16;
unsigned Data_command_5:16;
unsigned Data_command_6:16;
};
}pl_data_command;

union PL_TM_TX
{
unsigned char  pl_tm_1byte[256];
unsigned short pl_tm_2bytes[128];
unsigned int pl_tm_4bytes[64];
}Pl_tm_data;



#define FDI_NMI_INT_LEVEL 4
unsigned char FDI_NMI_Count;
unsigned char NMI_fail_count;
unsigned int minor_cycle_counter_test;
//unsigned short TC_count;

unsigned long int IO_In_Latch_Register_4_Data;
unsigned long int IO_Latch_Register_4_Data;

unsigned long int inter_at697f_count;
unsigned long int inter_at697f_checksum;
unsigned long int inter_at697_var;

//scrubing of SRAM
unsigned long int ram_scrub_addr;					    // SRAM running pointer address(Power on ram_scrub_addr = 0x40000000)
void s_ram_scrub();

//EEPROM checksum
unsigned char eeprom_flag, ee_blk_no;
unsigned int eeprom_chksum, eeprom_cur_addr, eeprom_blk_end_addr;
unsigned int chksum_arr[8];
void prom_chksum(void);

void EEprom_read(); //EEprom testing routine

//MUX_output_raed
unsigned short MUX_1;
unsigned short MUX_2;
unsigned short MUX_3;
unsigned short MUX_4;

//TC
unsigned long int TC_input_cmd[2];
#define TC_MSB TC_input_cmd[0]
#define TC_LSB TC_input_cmd[1]

//Function Declarations
void Init_Memory(void);
unsigned long int checksum_u32(unsigned long int *db_start_address,unsigned long int size_of_db);
unsigned char checksum_u8(unsigned char* db_start_address,unsigned int size_of_db);
void rOutput_Latch_Update();
void ST_output_update();
void NMI_interrupt_test();
void EEPROM_RST();
void GPIO();
void EEPROM_RES();
void MUX_Output();

//GPS
#define GPS_1 1
#define GPS_2 0

#define NMEA_GSV_Enable  0
#define NMEA_GSV_Disable 1
#define NMEA_GGA_Enable  2
#define NMEA_GGA_Disable 3
#define NMEA_GSA_Enable  4
#define NMEA_GSA_Disable 5
#define FACTORY_RESET    6
#define COLD_START       7
#define NMEA_ZDA_Enable  10
#define NMEA_ZDA_Disable 11
#define NMEA_VTG_Enable	 8
#define NMEA_VTG_Disable 9
#define NMEA_GLL_Enable  14
#define NMEA_GLL_Disable 15

extern unsigned short gps_pulse_mic_cnt;
extern unsigned int GPS_pulse_rcvd;
extern unsigned long int gps_obt_counter;
extern unsigned long int gps_obt_count_prev;

//IO OUTPUT LATCH DATA
unsigned long int IO_Latch_Register_2_Data;
unsigned long int IO_Latch_Register_3_Data;
unsigned long int IO_Latch_Register_4_Data;
unsigned long int IO_Latch_Register_5_Data;

extern void rGPS_pulsecheck();

extern void UTP_selection();
extern void LTP_selection();

#endif //HAL_GLOBAL
