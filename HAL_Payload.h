#ifndef HAL_PAYLOAD
#define HAL_PAYLOAD

#define TRUE 1
#define FALSE 0

#define PL_MAX_LIMIT 8
#define Payload_DATA_READY 0x00000002
#define Payload_DATA_NOT_READY 0x00000008
#define EXTRACT_LSB_8BITS           0x00FF
#define EXTRACT_LSB_16BITS          0x0000FFFF

#define PL_X_TX_DATA_OFF_CONFIG  	0x00000105	 	// 0000 0001 0000 0101
//#define PL_ACQ_CONFIG            	0x00000505   	// 0000_0101_0000_0101
#define PL_ACQ_CONFIG            	0x00000505   	// 0000_0101_0000_0101
#define PL_STS_CONFIG       		0x00000105   	// 0000_0001_0000_0101
#define PL_DEBUG_CONIG				0x00000505   	// 0000_0101_0000_0101
#define PL_X_DATA_ON_CONFIG 		0x00000585  	// 0000 0101 1000 0101
#define PL_DIAG_CONFIG 				0x00000185   	// 0000_0001_1000_0101
#define PL_TX_TM_CONFIG				0x00007f85	 	// 0111_1111_0000_0101
#define PL_HLT_CONFIG               0x0000011F  	// 0000_0001_0001_1111
#define PL_TX_TM_CONFIG_2           0x00000205  	// 0000_0010_0000_0101

#define PL_STS_TIME  				24
#define PL_STS_NO_ACK 				0xAAFF;
#define PL_TIME_OUT 				0.0016
#define PL_PASS    					0xAA0F
#define PL_FAIL    					0xAAF0
#define PL_PWR_TIMEOUT 				0xEAFF
#define PL_PWR_ON_ACK  				0xEA0F
#define PL_PWR_ON_NACK  			0xEAF0
#define PL_STS_CHK_ACK  			0xCC0F
#define PL_STS_CHK_NACK 			0xCCF0
#define PL_STS_CHK_TIMEOUT 			0xCCFF
#define PL_DIAG_ACK     			0xDD0F
#define PL_DIAG_NACK    			0xDDF0
#define PL_DIAG_TIMEOUT    			0xDDFF
#define PL_HLT_TIMEOUT      		0xBBFF
#define PL_HLT_ACK     			    0xBB0F
#define PL_ACQ_ACK      			0xAA0F
#define PL_ACQ_TIMEOUT     			0xAAFF
#define PL_TX_ON_ACK    			0xEE0F
#define PL_TX_ON_TIMEOUT   			0xEEFF
#define PL_TX_OFF_ACK   			0xFF0F
#define PL_TX_OFF_TIMEOUT  			0xFFFF
#define PL_DEBUG_ACK    			0xDB0F
#define PL_DEBUG_TIEMOUT   			0xDBFF
#define PL_TX_TM_ACK                0xEF0F
#define PL_TX_TM_TIMEOUT            0xEFFF


#define PL_STS_CMD_ID   			0x00005355
#define PL_HLT_CMD_ID				0x00004855
#define PL_DEBUG_HDR_CMD_ID 		0x00004555
#define PL_TX_DATA_ON_HDR_CMD_ID 	0x00005255
#define PL_ACQ_HDR_CMD_ID           0x00005755
#define PL_TX_DATA_OFF_HDR_CMD_ID   0x00004655
#define PL_DIAG_HDR_CMD_ID          0x00004455

#define PL_TM_DATA            		0x10
#define PL_MODE_ALL           		0x1F

void HILS_mode_enable();
void HILS_mode_disable();

uint32 PL_1_on_off_flag;
uint32 PL_2_on_off_flag;
unsigned int pl_data_rcvd;
unsigned int pl_cmd_id;
unsigned short PL_on_off_sts_flag;

union pl_data_rcv
{
unsigned char data_8bits[18];
unsigned short data_16bits[9];
}pl_data_rx;


short* Hils_ptr;
short* hils_ptr_sh;
unsigned long int* pl_config_addr_ptr;
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
unsigned int pl_tx_tm_flag;
unsigned int  pl_sts_chk_flag;
unsigned int tm_ds_en_flag;
unsigned short PL_TM_Status_flag;
unsigned short pl_ack_count;


void rHAL_X_Tx_ON(void);
void rHAL_X_Tx_OFF();
void rHAL_pl_sts_check(void);
void rHAL_pl_cmd_acq(void);
void rHAL_pl_cmd_hlt(void);
void rHAL_pl_diag(void);
void rHAL_pl_x_tx_data_on(void);
void rHAL_pl_x_tx_data_off(void);
void rHAL_pl_debug(void);
void rpl_tm_write();
void rpl_read ();
void pl_tx_tm_2();
void pl_tx_tm();
//void rTC_pl_tx_tm();
void rHAL_tm_ds_en();

void rHAL_pl1_ON();
void rHAL_pl2_ON();
void rHAL_pl1_OFF();
void rHAL_pl2_OFF();

#endif // HAL_PAYLOAD


#ifndef HAL_PAYLOAD
#define HAL_PAYLOAD

#define TRUE 1
#define FALSE 0

unsigned long int* pl_config_addr_ptr;
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
