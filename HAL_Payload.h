#ifndef HAL_PAYLOAD
#define HAL_PAYLOAD

#define TRUE 1
#define FALSE 0
#define PL_MAX_LIMIT 8
#define  Payload_DATA_READY 0x00000002

short* Hils_ptr;
short* hils_ptr_sh;
unsigned int       Payload_status_1_data;
unsigned long int* PL_CONFIG_Addr_Ptr;
unsigned short PL_ACK;
unsigned long int* inter_PL_TM_Dest_Addr;
unsigned short* inter_PL_TM_Source_Addr;
unsigned short PL_Debug_Data[4] ={0xA406,0x0003,0x0605,0x0807};
int PL_Exe_Flag;
unsigned long int TC_command;
int Enable_PL_TM_flag;
int PL_TM_write_Flag;
int pl_i;
int pl_buffer_len;
unsigned long int* pl_buffer_addr;
unsigned short PL_Addr_count;

void rHAL_PL_STS_Check();
void rHAL_PL_CMD_ACQ();
void rHAL_PL_CMD_HLT();
void rHAL_PL_DIAG();
void rHAL_PL_CMD_OFF();
void rHAL_PL_CMD_ON();
void rHAL_PL_DIAG_SDCARD(void);
void rHAL_PL_DIAG_FRAM(void);
void rHAL_PL_DEBUG(void);
void rHAL_PL_TEST(void);
void PL_TM_read();


#endif // HAL_PAYLOAD


#ifndef HAL_PAYLOAD
#define HAL_PAYLOAD

#define TRUE 1
#define FALSE 0


unsigned long int* PL_CONFIG_Addr_Ptr;
unsigned short     PL_ACK;
unsigned long int* inter_PL_TM_Dest_Addr;
unsigned short*    inter_PL_TM_Source_Addr;
unsigned short     PL_Debug_Data[4] ={0xA406,0x0003,0x0605,0x0807};
int                PL_Exe_Flag;
unsigned long int  TC_command;
int                Enable_PL_TM_flag;
int                PL_TM_write_Flag;
int                pl_i;


void rHAL_PL_STS_Check();
void rHAL_PL_CMD_ACQ();
void rHAL_PL_CMD_HLT();
void rHAL_PL_DIAG();
void rHAL_PL_CMD_OFF();
void rHAL_PL_CMD_ON();
void rHAL_PL_DIAG_SDCARD(void);
void rHAL_PL_DIAG_FRAM(void);
void rHAL_PL_DEBUG(void);
void rHAL_PL_TEST(void);

#endif // HAL_PAYLOAD
