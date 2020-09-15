//payload code//
#include "Global.h"
#include "HAL_Payload.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Telemetry.h"
#include "TM_Global_Buffer.h"
#include "adcs_VarDeclarations.h"
#include "TC_List.h"
#include "Telecommand.h"

uint32 PL_1_on_off_flag;
uint32 PL_2_on_off_flag;

uint8 pl_HLT_en;

unsigned short hils_datas[63];
void rHILS_payload(union HILS_test* HILS_packets)
{

	int i_hils;
	int temp_1;
	short temp_2;

	Hils_ptr = (short*)&(HILS_packets->HILS_data_16bit[0]);
	pl_config_addr_ptr = (uint32*)PAYLOAD_CONFIG_REGISTER;
	for(i_hils = 0 ; i_hils<= 31 ; i_hils++)
	{
		temp_2 							=  *Hils_ptr++;
		temp_1 							= byte_swap(temp_2);
		REG32(pl_config_addr_ptr++) 	= temp_1;
		hils_datas[i_hils]              = temp_1;
		REG32(PAYLOAD_STATUS2_ADDRESS) 	= 0x00002001;
	}
}

void HILS_mode_enable(void)           //HILS_mode enabling by using a GPIO pin no_6
{
	unsigned short tempdata;

	GPIO_pins.PIO_6 = 1;
	tempdata = GPIO_pins.data;
	IODAT = tempdata;
}

void HILS_mode_disable(void)         //HILS_mode disabling by using a GPIO pin no_6
{

	unsigned short tempdata;

	GPIO_pins.PIO_6 = 0;
	tempdata = GPIO_pins.data;
	IODAT = tempdata;

}


void rHAL_pl1_ON(void)
{
	unsigned short tempdata;
	unsigned char pl_tm_index;

	Out_Latch_3.PL1_ON_OFF = 1;
	Out_Latch_3.PL2_ON_OFF = 0;
	tempdata = Out_Latch_3.data;
	IO_LATCH_REGISTER_3;
	IO_LATCH_REGISTER_3 = tempdata;
	pl_cmd_id = 1;
	PL_1_on_off_flag = 1;
	PL_2_on_off_flag = 0;

	for (pl_tm_index = 0 ; pl_tm_index < PL_TM_BUF_MAX; pl_tm_index++)
	{
		TM.Buffer.TM_pl_data[pl_tm_index] = 0;
	}

}

void rHAL_pl1_OFF(void)
{
	unsigned short tempdata;
	Out_Latch_3.PL1_ON_OFF = 0;
	Out_Latch_3.TM_DS_EN = 0;
	tempdata = Out_Latch_3.data;
	IO_LATCH_REGISTER_3;
	IO_LATCH_REGISTER_3 = tempdata;
	pl_cmd_id = 0;
	PL_1_on_off_flag = 0;

	// Health command disable
	pl_HLT_en = False;

	// Reset the acknowledgement counter when the payload is turned off
	pl_ack_count = False;
}



void rHAL_pl2_ON(void)
{
	unsigned short tempdata;
	unsigned char pl_tm_index;
	Out_Latch_3.PL2_ON_OFF = 1;
	Out_Latch_3.PL1_ON_OFF = 0;
	tempdata = Out_Latch_3.data;
	IO_LATCH_REGISTER_3;
	IO_LATCH_REGISTER_3 = tempdata;
	pl_cmd_id = 1;
	PL_2_on_off_flag = 1;
	PL_1_on_off_flag = 0;
	for(pl_tm_index = 0 ; pl_tm_index < PL_TM_BUF_MAX ; pl_tm_index++)
	{
		TM.Buffer.TM_pl_data[pl_tm_index] = 0;
	}
}

void rHAL_pl2_OFF(void)
{
	unsigned short tempdata;
	Out_Latch_3.PL2_ON_OFF = 0;
	Out_Latch_3.TM_DS_EN = 0;
	tempdata = Out_Latch_3.data;
	IO_LATCH_REGISTER_3;
	IO_LATCH_REGISTER_3 = tempdata;
	pl_cmd_id = 0;
	PL_2_on_off_flag = 0;

	// Health command disable
	pl_HLT_en = False;

	// Reset the acknowledgement counter when the payload is turned off
	pl_ack_count = False;
}

void rHAL_tm_ds_en(void)
{
	unsigned short tempdata;

	Out_Latch_3.TM_DS_EN = 1;
	tempdata = Out_Latch_3.data;
	IO_LATCH_REGISTER_3;
	IO_LATCH_REGISTER_3 = tempdata;
	tm_ds_en_flag = 1;
}

void rHAL_X_Tx_ON(void)
{
	uint16 tempdata;
	Out_Latch_2.X_Tx_ON_OFF = 1;
	tempdata =  Out_Latch_2.data;
	IO_LATCH_REGISTER_2;
	IO_LATCH_REGISTER_2 = tempdata;
}


void rHAL_X_Tx_OFF(void)
{
	unsigned short tempdata;
	Out_Latch_2.X_Tx_ON_OFF = 0;
	tempdata =  Out_Latch_2.data;
	IO_LATCH_REGISTER_2;
	IO_LATCH_REGISTER_2 = tempdata;
}

unsigned int pl_configure_data;
void rHAL_pl_sts_check(void)
{
		REG32(PAYLOAD_CONFIG_REGISTER) = PL_STS_CMD_ID;
		/* No of Configure Bytes (15:7) : 2 bytes
		 * No of Receive Bytes (6:1)    : 2 byte
		 * Configure Bytes  (0)         : 1 */
		REG32(PAYLOAD_STATUS2_ADDRESS) = PL_STS_CONFIG;
		pl_cmd_id = 2;

}

/****************************************************************
 *@function name rHAL_PL_CMD_HLT
 *@return type   NONE
 *@Description   This function sends two bytes of data
 *@				 to check the Health status of PAYLOAD
 *@				 Payload responds with 0xAA0F followed by 13Bytes of
 *@				 health parameters
 ****************************************************************
 */
uint16 pl_hlt_flag;

void rHAL_pl_cmd_hlt(void)
{
	if ((PL_1_on_off_flag || PL_2_on_off_flag) && pl_HLT_en)
	{
		REG32(PAYLOAD_CONFIG_REGISTER) = PL_HLT_CMD_ID;
		/*****************************************************/
		/* No of Configure Bytes (15:7) : 2 bytes
		 * No of Receive Bytes (6:1)    : 15 byte
		 * Configure Bytes  (0)         : 1 */
		REG32(PAYLOAD_STATUS2_ADDRESS) = PL_HLT_CONFIG;
		pl_cmd_id = 3;
	}
		pl_sts_chk_flag = 0;
		PL_TM_Status_flag = 1;

}

/****************************************************************
 *@function name rHAL_PL_DEBUG
 *@return type   NONE
 *@Description   This function enables read and write to the
 	 	 	 	 payload. Data Bytes D1 to D8 is configured
 	 	 	 	 by telecommand. Payload responds with 0XAA0F
 ****************************************************************
 */

void rHAL_pl_debug(void)
{
		if (pl_data_command.pl_data_2byte[0] == PL_DEBUG_HDR_CMD_ID)
		{
			pl_config_addr_ptr =(uint32*)PAYLOAD_CONFIG_REGISTER;
			REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[0];
			REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[1];
			REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[2];
			REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[3];
			REG32(pl_config_addr_ptr) 	= pl_data_command.pl_data_2byte[4];
			/* No of Configure Bytes (15:7) : 10 bytes
			 * No of Receive Bytes (6:1)    : 2 byte
			 * Configure Bytes  (0)         : 1 */
			REG32(PAYLOAD_STATUS2_ADDRESS) = PL_DEBUG_CONIG;
			pl_cmd_id = 4;

		}
		else
		{
			//
		}
}

/****************************************************************
 *@function name rHAL_PL_X_TX_DATA_ON
 *@return type   NONE
 *@Description   This function requests to transmit payload data
  	  	  	  	 stored in SDCARD
  	  	  	  	 Payload responds with 0XAA0F
 ****************************************************************
 */
uint32 pl_x_tx_data_on_flag;
uint32  data_nc;
void rHAL_pl_x_tx_data_on(void)
{

	if (pl_data_command.pl_data_2byte[0] == (uint16)PL_TX_DATA_ON_HDR_CMD_ID)
	{
		data_nc = 2;
		pl_config_addr_ptr =(uint32*)PAYLOAD_CONFIG_REGISTER;
		REG32(pl_config_addr_ptr++)      = pl_data_command.pl_data_2byte[0];
		REG32(pl_config_addr_ptr++) 	 = pl_data_command.pl_data_2byte[1];
		REG32(pl_config_addr_ptr++) 	 = pl_data_command.pl_data_2byte[2];
		REG32(pl_config_addr_ptr++) 	 = pl_data_command.pl_data_2byte[3];
		REG32(pl_config_addr_ptr++) 	 = pl_data_command.pl_data_2byte[4];
		REG32(pl_config_addr_ptr) 	     = pl_data_command.pl_data_2byte[5];
		/********************************************/
		/* No of Configure Bytes (15:7) : 11 bytes
		 * No of Receive Bytes (6:1)    : 2 byte
		 * Configure Bytes  (0)         : 1 */
		REG32(PAYLOAD_STATUS2_ADDRESS) = PL_X_DATA_ON_CONFIG;
		pl_cmd_id = 5;

		// This flag is enabled to send payload command for Health monitoring
		pl_HLT_en = TRUE;
	}
	else
	{
		//
	}

}

/****************************************************************
 *@function name rHAL_PL_CMD_ACQ
 *@return type   NONE
 *@Description   This function commands payload to recieve
    			 data(RAW/EXT/Telemetry).
  	  	  	  	 Payload responds with 0XAA0F
 ****************************************************************
 */
uint32 pl_acq_flag;
uint16 pl_mode;

void rHAL_pl_cmd_acq(void)
{
	if (pl_data_command.pl_data_2byte[0] == PL_ACQ_HDR_CMD_ID)
	{

		pl_config_addr_ptr =(uint32*)PAYLOAD_CONFIG_REGISTER;
		REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[0];
		REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[1];
		REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[2];
		REG32(pl_config_addr_ptr++) = pl_data_command.pl_data_2byte[3];
		REG32(pl_config_addr_ptr) 	= pl_data_command.pl_data_2byte[4];
		pl_mode = pl_data_command.pl_data_2byte[4] & EXTRACT_LSB_8BITS;
		/*****************************************************/
		/* No of Configure Bytes (15:7) : 10 bytes
		 * No of Receive Bytes (6:1)    : 2 byte
		 * Configure Bytes  (0)         : 1 */
		REG32(PAYLOAD_STATUS2_ADDRESS) = PL_ACQ_CONFIG;
		pl_cmd_id = 6;
		pl_acq_flag = TRUE;

		// This flag is enabled to send payload command for Health monitoring
		pl_HLT_en = TRUE;
	}

	else
	{
		//
	}
}


/****************************************************************
 *@function name rHAL_PL_X_TX_DATA_OFF
 *@return type   NONE
 *@Description   This function requests to stop the transmission
   	   	   	     of payload data stored in SDCARD
  	  	  	  	 Payload responds with 0XAA0F
 ****************************************************************
 */
uint32 pl_x_tx_data_off_flag;

uint32 data_x_tx;
void rHAL_pl_x_tx_data_off(void)
{
		REG32(PAYLOAD_CONFIG_REGISTER) = PL_TX_DATA_OFF_HDR_CMD_ID;
		data_x_tx = PL_TX_DATA_OFF_HDR_CMD_ID;
		/* No of Configure Bytes (15:7) : 2 bytes
		 * No of Receive Bytes (6:1)    : 2 byte
		 * Configure Bytes  (0)         : 1 */
		REG32(PAYLOAD_STATUS2_ADDRESS) = PL_X_TX_DATA_OFF_CONFIG;//0000 0001 0000 0101
		pl_cmd_id = 7;

		pl_HLT_en = False;

}


/************* Diagnosis of AIS PAYLOAD ******************************/
uint32 pl_diag_flag;
void rHAL_pl_diag(void)
{
	if (pl_data_command.pl_data_2byte[0] == PL_DIAG_HDR_CMD_ID)
	{

		pl_config_addr_ptr =(uint32*)PAYLOAD_CONFIG_REGISTER;

		REG32(pl_config_addr_ptr++)  = pl_data_command.pl_data_2byte[0];
		REG32(pl_config_addr_ptr) 	 = pl_data_command.pl_data_2byte[1];
		/* No of Configure Bytes (15:7) : 3 bytes
		 * No of Receive Bytes (6:1)    : 2 byte
		 * Configure Bytes  (0)         : 1 */
		REG32(PAYLOAD_STATUS2_ADDRESS) = PL_DIAG_CONFIG;
		pl_cmd_id = 8;
	}
	else
	{
		//
	}

}

unsigned int pl_test10;
uint16 *pl_tm_ptr;
uint16 temp_data_1_pl, temp_data_2_pl;
unsigned short PL_TX_TM_2_EN;
void pl_tx_tm(void)
{
	uint16 tempdata_pl_1,tempdata_pl_2;
	uint32 pl_st_write_index = 0;

	if ( TC_boolean_u.TC_Boolean_Table.pl_tx_tm_flag && pl_acq_flag)
	{
		if(inter_TM_Pl_Buffer)
		{
			PL_TM_Status_flag = 0;
			pl_tm_ptr = &(Pl_tm_data.pl_tm_2bytes[0]);
			pl_config_addr_ptr = (uint32*)PAYLOAD_CONFIG_REGISTER;
			REG32(pl_config_addr_ptr++) = 0x00005055;
			for(pl_st_write_index = 0; pl_st_write_index <= 126; pl_st_write_index++)
			{
				tempdata_pl_1 = *pl_tm_ptr++;
				tempdata_pl_2 = byte_swap(tempdata_pl_1);

				REG32(pl_config_addr_ptr++) = (uint32)(tempdata_pl_2 & EXTRACT_LSB_16BITS);  //swaping the telemetry_data and updating in Pl_config
			}
			tempdata_pl_2 	= *pl_tm_ptr++;
			temp_data_1_pl	= tempdata_pl_2;
			tempdata_pl_2 	= *pl_tm_ptr++;
			temp_data_2_pl  = tempdata_pl_2;

			/* No of Configure Bytes (15:7) : 258 bytes // changed to 256bytes
			 * No of Receive Bytes (6:1)    : 2 byte
			 * Configure Bytes  (0)         : 1
			*/
			REG32(PAYLOAD_STATUS2_ADDRESS) = PL_TX_TM_CONFIG;
			pl_cmd_id = 9;
			inter_TM_Pl_Buffer = FALSE;
			PL_TX_TM_2_EN =TRUE;
			pl_test10++;
		}
		else
		{
			//
		}
	}
	else
	{
		//
	}
}

uint32 pl_test12;
void pl_tx_tm_2(void)
{
//	uint16 tempdata3,tempdata2;
	pl_config_addr_ptr =(uint32*)PAYLOAD_CONFIG_REGISTER;
	if(PL_TX_TM_2_EN)
	{
//		tempdata3 = REG32(PAYLOAD_STATUS2_ADDRESS);
//		tempdata2 = tempdata3 & 0x0001;

		pl_test12++;
		Out_Latch_3.TM_DS_EN = 0;
		IO_LATCH_REGISTER_3 = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;
		IO_LATCH_REGISTER_3;
		Out_Latch_3.TM_DS_EN = 1;
		IO_LATCH_REGISTER_3 = Out_Latch_3.data;
		IO_LATCH_REGISTER_3;

		REG32(pl_config_addr_ptr++) = temp_data_1_pl;
		REG32(pl_config_addr_ptr) 	= temp_data_2_pl;

		REG32(PAYLOAD_STATUS2_ADDRESS) = PL_TX_TM_CONFIG_2;

		PL_TX_TM_2_EN=FALSE;
		pl_cmd_id = 9;
		PL_TM_Status_flag = 1;
	}
}
int32 length;
void rpl_read(void)                                                    //Reading the payload_data
{
	uint16  temp_short1, temp_short2;
	uint32 tempdata;
	int32  pl_Addr_count;
	uint32 Payload_status_2_data;
	uint32  Payload_status_1_data;
	uint16* pl_data_addr;
	Payload_status_2_data = (REG32(PAYLOAD_STATUS2_ADDRESS) & EXTRACT_LSB_16BITS);
	Payload_status_1_data = (REG32(PAYLOAD_STATUS1_ADDRESS) & EXTRACT_LSB_16BITS);
	pl_Addr_count = 0;


	if (Payload_status_1_data & Payload_DATA_READY)
	{
		pl_data_rcvd = 1;
		pl_buffer_len = ((Payload_status_2_data & 0x0000007E)>>1);
		pl_buffer_len = ((pl_buffer_len+1)>>1);
		if(pl_buffer_len > 8) pl_buffer_len = 8;
		length = pl_buffer_len;
		pl_buffer_addr = (uint32*)PAYLOAD_BUFFER_ADDRESS;

		while(pl_Addr_count < pl_buffer_len)
		{
			tempdata  = REG32(pl_buffer_addr++) & EXTRACT_LSB_16BITS;
			temp_short1 = (unsigned short)(tempdata & EXTRACT_LSB_16BITS);
			pl_data_rx.data_16bits[pl_Addr_count] = byte_swap(temp_short1);
			pl_Addr_count++;
		}

	}
	else
	{
		pl_data_rcvd = 0;
	}
}

unsigned int data;
void rpl_tm_write(void)
{
		uint16* pl_data_addr;
		int  pl_Addr_count;
		pl_data_addr = &pl_data_rx.data_16bits[0];
		switch(pl_cmd_id)
		{
			case 1: if (pl_data_rcvd )
					{
						//
					}
					else
					{
						//
					}

					break;

			case 2:	if (pl_data_rcvd )
					{

						if (*pl_data_addr == PL_PASS)
						{
							TM.Buffer.TM_pl_data[0] = PL_STS_CHK_ACK;
							pl_data_rcvd = 0;
							pl_sts_chk_flag = 1;
							pl_ack_count++;

						}
						else if (*pl_data_addr == PL_FAIL)
						{
							TM.Buffer.TM_pl_data[0] = PL_STS_CHK_NACK;
							pl_data_rcvd = 0;

						}
					}
					else
					{
						TM.Buffer.TM_pl_data[0] = PL_STS_CHK_TIMEOUT;

					}
					break;

			case 3: if (pl_data_rcvd )
					{
						unsigned int hlt_count;

						TM.Buffer.TM_pl_data[1] = PL_HLT_ACK;

						for (pl_Addr_count = 2, hlt_count = 1 ; hlt_count < pl_buffer_len; pl_Addr_count++ , hlt_count++)
						{
							TM.Buffer.TM_pl_data[pl_Addr_count] = pl_data_rx.data_16bits[hlt_count];
						}
						pl_data_rcvd = 0;
						pl_ack_count++;
					}
					else
					{
						TM.Buffer.TM_pl_data[1] = PL_HLT_TIMEOUT;
					}
					break;

			case 4: if (pl_data_rcvd )
					{

						TM.Buffer.TM_pl_data[9]= PL_DEBUG_ACK;
						pl_data_rcvd = 0;
						pl_ack_count++;
					}
					else
					{
						TM.Buffer.TM_pl_data[9] = PL_DEBUG_TIEMOUT;

					}
					break;

			case 5: data = 5;
					if (pl_data_rcvd )
					{
						TM.Buffer.TM_pl_data[9] = PL_TX_ON_ACK;
						//data = TM.Buffer.TM_pl_data[9];
						pl_data_rcvd = 0;
						pl_ack_count++;

					}
					else
					{
						TM.Buffer.TM_pl_data[9] = PL_TX_ON_TIMEOUT;

					}
					break;

			case 6:if (pl_data_rcvd )
					{
						 TM.Buffer.TM_pl_data[9] = PL_ACQ_ACK;
						 pl_data_rcvd = 0;
						 pl_ack_count++;

					}
					else
					{
						TM.Buffer.TM_pl_data[9] = PL_ACQ_TIMEOUT;

					}
					break;

			case 7: if (pl_data_rcvd )
					{
						TM.Buffer.TM_pl_data[9] = PL_TX_OFF_ACK;
						pl_data_rcvd = 0;
						pl_ack_count++;

					}
					else
					{
						TM.Buffer.TM_pl_data[9] = PL_TX_OFF_TIMEOUT;

					}
					break;

			case 8: if (pl_data_rcvd )
					{
						if (*pl_data_addr == PL_PASS)
						{
							TM.Buffer.TM_pl_data[9] = PL_DIAG_ACK;
							pl_data_rcvd = 0;
							pl_ack_count++;

						}
						else if (*pl_data_addr == PL_FAIL)
						{
							TM.Buffer.TM_pl_data[9] = PL_DIAG_NACK;
							pl_data_rcvd = 0;

						}
					}
					else
					{
						TM.Buffer.TM_pl_data[9] = PL_DIAG_TIMEOUT;

					}
					break;
			case 9: if (pl_data_rcvd )
					{
						TM.Buffer.TM_pl_data[9] = PL_TX_TM_ACK;
						pl_data_rcvd = 0;
						pl_ack_count++;

					}
					else
					{
						TM.Buffer.TM_pl_data[9] = PL_TX_TM_TIMEOUT;

					}
					break;

			default:
					break;

		}

	pl_cmd_id = FALSE;
	TM.Buffer.TM_pl_ack_count = pl_ack_count;
}

