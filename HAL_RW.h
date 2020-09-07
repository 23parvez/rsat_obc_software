#ifndef HAL_RW_H
#define HAL_RW_H

#define RW_BUFFER_MAX_LIMIT 12

#define INIT_BYTES 8
#define PING_BYTES 8
#define PEEK_BYTES 8
#define POKE_BYTES 8

#define CONFIG_TYPE_INIT    0
#define CONFIG_TYPE_PING    1
#define CONFIG_TYPE_PEEK    2
#define CONFIG_TYPE_POKE    3
#define CONFIG_TYPE_NA      15

#define BIT_FIELD_1 1
#define BIT_FIELD_2 2
#define BIT_FIELD_3 3
#define BIT_FIELD_4 4

#define RWHEEL0 0
#define RWHEEL1 1
#define RWHEEL2 2
#define RWHEEL3 3

#define RW_TX_BUF_BUSY  0x00000001

#define ENABLE  1
#define DISABLE 0

#define RW_STATUS_DEFAULT_DATA 0x0000

//RW STATUS REGISTER 1 BIT FIELDS
#define RW_CONFIG_EN            0
#define RW_CONFIG_DONE          1
#define RW_PEEK_DATA_READY      2
#define RW_POKE_ENABLE          3
#define RW_POKE_DONE            4
#define RW_READ_ENABLE          5
#define RW_DATA_NOT_READY       6

//RW STATUS REGISTER 4 BIT FIELDS
#define RW_CONFIG_BYTES          8  // CONFIG_BYTES : |11|10|09|08|
#define RW_CONFIG_TYPE           12 // CONFIG_TYPE  : |15|14|13|12|

#define RW_READ_ENABLE_DATA    (RW_STATUS_DEFAULT_DATA | (CONFIG_TYPE_NA   << RW_CONFIG_TYPE) | (ENABLE     << RW_READ_ENABLE ))
#define RW_POKE_ENABLE_DATA    (RW_STATUS_DEFAULT_DATA | (CONFIG_TYPE_NA   << RW_CONFIG_TYPE) | (ENABLE     << RW_POKE_ENABLE ))
#define RW_EN_CONFIG_DATA      (RW_STATUS_DEFAULT_DATA | (CONFIG_TYPE_NA   << RW_CONFIG_TYPE) | (ENABLE     << RW_CONFIG_EN   ))
#define RW_INIT_CONFIG_DATA    (RW_STATUS_DEFAULT_DATA | (CONFIG_TYPE_INIT << RW_CONFIG_TYPE) | (INIT_BYTES << RW_CONFIG_BYTES))
#define RW_PING_CONFIG_DATA    (RW_STATUS_DEFAULT_DATA | (CONFIG_TYPE_PING << RW_CONFIG_TYPE) | (PING_BYTES << RW_CONFIG_BYTES))
#define RW_PEEK_CONFIG_DATA    (RW_STATUS_DEFAULT_DATA | (CONFIG_TYPE_PEEK << RW_CONFIG_TYPE) | (PEEK_BYTES << RW_CONFIG_BYTES))
#define RW_POKE_CONFIG_DATA    (RW_STATUS_DEFAULT_DATA | (CONFIG_TYPE_POKE << RW_CONFIG_TYPE) | (POKE_BYTES << RW_CONFIG_BYTES))

#define RW_POLY 0x8408

#define reverse_order(a) (((a&0xFF000000) >> 24)|((a&0x00FF0000) >> 8)|((a&0x0000FF00) << 8)|((a&0x000000FF) << 24))
//#define byte_swap(a) (unsigned short)(((a&0xFF00)>>8)|((a&0x00FF)<<8))


struct HAL_RW_Data_Structure RW_1,RW_2,RW_3,RW_4;
//RW TEST
#pragma pack(1)
union RW_TC_Command_u
{
	unsigned char Data[16];
	struct
	{
		unsigned char Start_Byte;
		unsigned char Dest_Addr;
		unsigned char Source_Addr;
		unsigned char Poll_Bit:1;
		unsigned char B:1;
		unsigned char ACK:1;
		unsigned char Cmd_Code:5;
		unsigned char Data_MSB;
		unsigned char Mode_Type;
		float Data_Value;
		unsigned short CRC;
		unsigned char Stop_Byte;
		unsigned char filler[3];
	};
}RW_TC;

#pragma pack(1)
union RW_TM_Command_u
{
	unsigned char Data[8];
	struct
	{
		unsigned char Start_Byte;
		unsigned char Dest_Addr;
		unsigned char Source_Addr;
		unsigned char Poll_Bit:1;
		unsigned char B:1;
		unsigned char ACK:1;
		unsigned char Cmd_Code:5;
		unsigned char Mode_Type;
		unsigned short CRC;
		unsigned char Stop_Byte;
	};
}RW_TMC;


#pragma pack(1)
union RW_TM_Rcvd_u
{
	unsigned char Data[512];
	struct
	{
		unsigned char Start_Byte;
		unsigned char Dest_Addr;
		unsigned char Source_Addr;
		unsigned char Poll_Bit:1;
		unsigned char B:1;
		unsigned char ACK:1;
		unsigned char Cmd_Code:5;
		//unsigned char Data_MSB;
		unsigned char Mode_Type;
		float Data_Value;
		unsigned short CRC;
		unsigned char Stop_Byte;
	};
}RW_TM;

float RW_Wheel_Speed[4];

#pragma pack(1)
union TC_Config_Addr_Write
{
	unsigned char data_8bit[512];
	unsigned short data_16bit[256];
}RW_Buffer_u,RW_Buffer_u_rx;

union int_float
{
	float float_num;
	unsigned int int_num;
}USIF_u;

unsigned char* RW_TM_Raw_Data_ptr;
unsigned char* RW_TM_ptr;

float RWS[4]; //temp testing
float RW_Measured[4];

void rHAL_RW_TC_Write(struct HAL_RW_Data_Structure* RW_No, float  RW_Speed, uint8 RW_ID);
void rHAL_RW_TM_Write (struct HAL_RW_Data_Structure* RW_No, uint8 RW_ID);
int rHAL_RW_TM_Read(struct HAL_RW_Data_Structure* RW_No, union RW_TM_Rcvd_u* RW_TM, int RW_index);
void rRW_SlipFrame_Check(struct HAL_RW_Data_Structure* RW_No_Addr, unsigned char* inter_slipframe_data_addr, int NOB_Slipframe_Check);

/******************Added on 12/10/19 *****************/
void rRW_init_cmd(struct HAL_RW_Data_Structure RW_No, unsigned char RW_ID);

/*****************************************************/

void rHAL_RW_ConfigBuffer_Write(struct HAL_RW_Data_Structure* RW_No_Addr, unsigned short* inter_Buffer_cpy_addr,int inter_NOB_Write);
void rRW_Data_Write(void);
void rRW_Data_Request(void);
void rRW_Data_Read(void);

unsigned char RW_Write_Data[20];
unsigned char* RW_Write_ptr;
unsigned short crc_test = 0;
unsigned char Auto_manual_speed_sel;

unsigned char RW_Data_SlipFrame_TC[20];
int NOB_SFC_TC;					//Number of bytes for Slip-frame check
int NOB_CRC_TC;					//Number of bytes for Cyclic redundancy check

unsigned char RW_Data_SlipFrame_TMC[20];
int NOB_SFC_TMC;				//Number of bytes for Slip-frame check
int NOB_CRC_TMC;				//Number of bytes for Cyclic redundancy check


/****************** Added on 12/10/19 ********************/
unsigned char RW_Data_SlipFrame_Init[20];
int NOB_SFC_Init;				//Number of bytes for Slip-frame check
int NOB_CRC_Init;				//Number of bytes for Cyclic redundancy check
unsigned char* Init_ptr;		//Pointer to structure of data frame
unsigned char* Init_data_ptr;	//Pointer to union (contains data which can be accessed as short or char)
/********************************************************/

unsigned int inter_HAL_RW_Read_Limit;
unsigned int inter_HAL_RW_Read_Addr;
unsigned short aa;

unsigned short temp_short;

//Ping
/******************** Added on 12/10/19 **************************/
#pragma pack(1)
union rw_init_cmd_u
{
	unsigned char Data[11];
	struct
	{
		unsigned char Start_Byte;
		unsigned char Dest_Addr;
		unsigned char Source_Addr;
		unsigned char Poll_Bit:1;
		unsigned char B:1;
		unsigned char ACK:1;
		unsigned char Cmd_Code:5;
		unsigned int  Data_value;
		unsigned short CRC;
		unsigned char Stop_Byte;
	};
}rw_init;

unsigned char NSP_addr_table[4] = {0x34, 0x35, 0x36, 0x41}; // Reaction wheel address table
unsigned char* RW_TM_ptr_init; //transfer to h file

float dak;

#endif /* HAL_RW_H */
