#include "HAL_Global.h"
#include "HAL_Address.h"
#include "Telemetry.h"
#include "TM_Global_Buffer.h"
#include "Global.h"
#include "TC_List.h"
#include "Telecommand.h"
#include "adcs_VarDeclarations.h"

void Init_Memory(void)
{
	//MCFG1 = 0x10F80A08;//Configure Memory Configuration Register 1
	MCFG1 = 0x10F80A88;
	MCFG2 = 0x00000E6F;//Configure Memory Configuration Register 2
	MCFG3 = 0xC8000300;//Configure Memory Configuration Register 3
}

unsigned long int checksum_u32(unsigned long int *db_start_address,unsigned long int size_of_db)
{
	inter_at697f_checksum = 0x00000000;
	for(inter_at697f_count = 0;inter_at697f_count < size_of_db;inter_at697f_count++)
	{
		inter_at697_var = REG32(db_start_address);
		inter_at697f_checksum = inter_at697f_checksum + inter_at697_var;
		db_start_address = db_start_address + 0x00000001;
	}
	return inter_at697f_checksum;
}

unsigned char checksum_u8(unsigned char* db_start_address,unsigned int size_of_db)
{
	static unsigned char chk_sum_u8;		//Check sum (8 bits)
	static unsigned char inter_val_u8;		//dereferenced value

	for(inter_at697f_count = 0;inter_at697f_count < size_of_db;inter_at697f_count++)
	{
		inter_val_u8 = REG8(db_start_address); // #define REG8(a)  *((volatile unsigned char*) (a))//For 8Bit Addressing
		chk_sum_u8 = chk_sum_u8 ^ inter_val_u8;
		db_start_address = db_start_address + 0x00000001;
	}
	return chk_sum_u8;
}

void rOutput_Latch_Update()
{
	//Update to Telemetry
	//Output Latches
	TM.Buffer.TM_Output_Latch[0] = MTR_LATCH_REGISTER;
	TM.Buffer.TM_Output_Latch[1] = IO_LATCH_REGISTER_2;
	TM.Buffer.TM_Output_Latch[2] = IO_LATCH_REGISTER_3;
	TM.Buffer.TM_Output_Latch[3] = IO_LATCH_REGISTER_4;
	TM.Buffer.TM_Output_Latch[4] = IO_LATCH_REGISTER_5;
	TM.Buffer.TM_Output_Latch[5] = IO_LATCH_REGISTER_6;

	ST_normal.ST_NM_Buffer.TM_Output_Latch[0] = MTR_LATCH_REGISTER;
	ST_normal.ST_NM_Buffer.TM_Output_Latch[1] = IO_LATCH_REGISTER_2;
	ST_normal.ST_NM_Buffer.TM_Output_Latch[2] = IO_LATCH_REGISTER_3;
	ST_normal.ST_NM_Buffer.TM_Output_Latch[3] = IO_LATCH_REGISTER_4;
	ST_normal.ST_NM_Buffer.TM_Output_Latch[4] = IO_LATCH_REGISTER_5;

	ST_special.ST_SP_Buffer.TM_Output_Latch[0] = IO_LATCH_REGISTER_2;

	//Input Latches
	TM.Buffer.TM_Input_Latch[0] = IO_IN_LATCH_REGISTER_1;
	TM.Buffer.TM_Input_Latch[1] = IO_IN_LATCH_REGISTER_2;
	TM.Buffer.TM_Input_Latch[2] = IO_IN_LATCH_REGISTER_3;
	TM.Buffer.TM_Input_Latch[3] = IO_IN_LATCH_REGISTER_4;

}


void ST_output_update()
{
	ST_Out_Latch_2.data = IO_LATCH_REGISTER_2;
	ST_Out_Latch_3.data = IO_LATCH_REGISTER_3;
	ST_Out_latch_5.data = IO_LATCH_REGISTER_5;

	ST_output_latch.GPS1_ON_OFF		= ST_Out_Latch_2.GPS1_ON_OFF;
	ST_output_latch.GPS1_RESET 		= ST_Out_Latch_2.GPS1_RESET;
	ST_output_latch.GPS2_ON_OFF 	= ST_Out_Latch_2.GPS2_ON_OFF;
	ST_output_latch.GPS2_RESET 		= ST_Out_Latch_2.GPS2_RESET;
	ST_output_latch.IMU2_ON_OFF 	= ST_Out_Latch_2.IMU2_ON_OFF;
	ST_output_latch.IMU2_RESET 		= ST_Out_Latch_2.IMU2_RESET;
	ST_output_latch.IMU1_ON_OFF 	= ST_Out_Latch_2.IMU1_ON_OFF;
	ST_output_latch.IMU1_RESET 		= ST_Out_Latch_2.IMU1_RESET;
	ST_output_latch.MTR_ONOFF 		= ST_Out_Latch_2.MTR_ON_OFF;
	ST_output_latch.X_TX_ON_OFF 	= ST_Out_Latch_2.X_Tx_ON_OFF;
	ST_output_latch.heater6 		= ST_Out_Latch_3.Heater6_ON_OFF;
	ST_output_latch.heater5 		= ST_Out_Latch_3.Heater5_ON_OFF;
	ST_output_latch.heater4 		= ST_Out_Latch_3.Heater4_ON_OFF;
	ST_output_latch.heater3 		= ST_Out_Latch_3.Heater3_ON_OFF;
	ST_output_latch.heater2 		= ST_Out_Latch_3.Heater2_ON_OFF;
	ST_output_latch.heater1 		= ST_Out_Latch_3.Heater1_ON_OFF;
	ST_output_latch.RW4_ON_OFF 		= ST_Out_Latch_3.RW4_ON_OFF;
	ST_output_latch.RW3_ON_OFF 		= ST_Out_Latch_3.RW3_ON_OFF;
	ST_output_latch.RW2_ON_OFF		= ST_Out_Latch_3.RW2_ON_OFF;
	ST_output_latch.RW1_ON_OFF 		= ST_Out_Latch_3.RW1_ON_OFF;
	ST_output_latch.Payload2_ON_OFF = ST_Out_Latch_3.PL1_ON_OFF;
	ST_output_latch.Payload1_ON_OFF = ST_Out_Latch_3.PL2_ON_OFF;
	ST_output_latch.SA3_ON_OFF 		= ST_Out_latch_5.SA1_ON_OFF;
	ST_output_latch.SA2_ON_OFF 		= ST_Out_latch_5.SA2_ON_OFF;
	ST_output_latch.SA1_ON_OFF 		= ST_Out_latch_5.SA3_ON_OFF;

}
unsigned int flag_mni;

void NMI_interrupt_test()
{
	// Routine updated on 24 June 2020
	//volatile unsigned int down_counter = 0x000FFFFF; // 460ms counter
	if(TC_boolean_u.TC_Boolean_Table.NMI_test_enable)
	{
		volatile unsigned int down_counter = 0x00116417; // 500ms counter

		GPIO_pins.PIO_5 = 1;
		IODAT = GPIO_pins.data;

		while (down_counter > 0)
		{
			down_counter--;
		}
		GPIO_pins.PIO_5 = 0;
		IODAT = GPIO_pins.data;

		NMI_fail_count++;

	}
	TM.Buffer.TM_NMI_fail_count = NMI_fail_count;
}



unsigned short MUX_data1,MUX_data2,MUX_data3,MUX_data4,test_data_3;
void MUX_Output()
{
	unsigned short tempdata1,tempdata2,tempdata3,tempdata4,tempdata5,tempdata6,tempdata7,tempdata8;
	unsigned short MUX_index;

	for(MUX_index = 0; MUX_index < 8; MUX_index++)
	{

		switch(MUX_index)
		{
		case 0: Out_latch_5.SEL_2 = 0;
				Out_latch_5.SEL_1 = 0;
				Out_latch_5.SEL_0 = 0;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4 ;
				tempdata1 = input_latch_2.data;
				TM.Buffer.TM_MUX_1 = tempdata1 & 0x3c00;

				break;

		case 1: Out_latch_5.SEL_2 = 1;
				Out_latch_5.SEL_1 = 0;
				Out_latch_5.SEL_0 = 0;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4 ;
				tempdata2 = input_latch_2.data;
				TM.Buffer.TM_MUX_2 = tempdata2 & 0x3c00;

				break;

		case 2: Out_latch_5.SEL_2 = 0;
				Out_latch_5.SEL_1 = 1;
				Out_latch_5.SEL_0 = 0;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4 ;
				tempdata3 = input_latch_2.data;
				TM.Buffer.TM_MUX_3 = tempdata3 & 0x3c00;

				break;

		case 3: Out_latch_5.SEL_2 = 1;
				Out_latch_5.SEL_1 = 1;
				Out_latch_5.SEL_0 = 0;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4 ;
				tempdata4 = input_latch_2.data;
				TM.Buffer.TM_MUX_4 = tempdata4 & 0x3c00;

				break;

		case 4: Out_latch_5.SEL_2 = 0;
				Out_latch_5.SEL_1 = 0;
				Out_latch_5.SEL_0 = 1;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4 ;
				tempdata5 = input_latch_2.data;
				TM.Buffer.TM_MUX_5 = tempdata5 & 0x3c00;

				break;

		case 5: Out_latch_5.SEL_2 = 1;
				Out_latch_5.SEL_1 = 0;
				Out_latch_5.SEL_0 = 1;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4 ;
				tempdata6 = input_latch_2.data;
				TM.Buffer.TM_MUX_6 = tempdata6 & 0x3c00;

				break;

		case 6: Out_latch_5.SEL_2 = 0;
				Out_latch_5.SEL_1 = 1;
				Out_latch_5.SEL_0 = 1;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4 ;
				tempdata7 = input_latch_2.data;
				TM.Buffer.TM_MUX_7 = tempdata7 & 0x3c00;

				break;


		case 7: Out_latch_5.SEL_2 = 1;
				Out_latch_5.SEL_1 = 1;
				Out_latch_5.SEL_0 = 1;
				IO_LATCH_REGISTER_5 = Out_latch_5.data;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				IO_LATCH_REGISTER_5;
				input_latch_2.data = IO_IN_LATCH_REGISTER_4;
				tempdata8 = input_latch_2.data;
				TM.Buffer.TM_MUX_8 = tempdata8 & 0x3c00;

				break;


		default:
				break;
		}
	}
}

void EEPROM_RST()
{
	unsigned short tempdata;
	Out_latch_4.EEPROM_RESET = 0;
	tempdata = Out_latch_4.data;
	IO_LATCH_REGISTER_4;
	IO_LATCH_REGISTER_4 = tempdata;
}

void EEPROM_RES()
{
	unsigned short tempdata;
	Out_latch_5.SA2_DEPLOY	 = 0;
	IO_LATCH_REGISTER_5 = Out_latch_5.data;
}

/******************************************************************/
/****************************************************************
 *@function name  s_ram_scrub
 *@return type    NONE
 *@Description    This function reads the data in memory
 *@			      location and writes back the data in same location
 *@			      for correcting single bit error in the memory.
 *@			      The function will be called in every minor cycle.
 *@			      It takes about 2min to scrub 512kB of memory location.
 ******************************************************************/
int ram_scrub_cnt;
void s_ram_scrub(void)
{
	uint32 blk_end_addr;
	uint32 temp_data;	// Temporary variable to read data from RAM location

	if( TC_boolean_u.TC_Boolean_Table.TC_sram_scrub_enable_disable)
	{
		blk_end_addr  = ram_scrub_addr + BLOCK_SIZE;

		if(ram_scrub_addr < RAM_SEG_END_ADDR)
		{
			while(ram_scrub_addr != blk_end_addr)
			{
				ram_scrub_cnt++;
				temp_data 		= REG32(ram_scrub_addr);
				REG32(ram_scrub_addr) = temp_data;
				ram_scrub_addr += 4;
			}
		}
		else
		{
			ram_scrub_addr = RAM_SEG_START_ADDR;
		}
	}
	else
	{
		//
	}
}


/*
EEPROM
	0x00000000	+--------------------+
				|    Block 1 64KB    |
				+--------------------+
				|    Block 2 64KB    |
				+--------------------+
				|    Block 3 64KB    |
				+--------------------+
				|    Block 4 64KB    |
				+--------------------+
				|    Block 5 64KB    |
				+--------------------+
				|    Block 6 64KB    |
				+--------------------+
				|    Block 7 64KB    |
				+--------------------+
				|    Block 8 64KB    |
	0x0007D000	+--------------------+(512KB)

*/


/****************************************/
/** Add this initialization in por_init **/
/*eeprom_cur_addr = EEPROM_START_ADDR;
eeprom_blk_end_addr  = eeprom_cur_addr + EEPROM_BLOCK_SIZE; */
/****************************************/


/****************************************************************
 *@function name  prom_chksum
 *@return type    NONE
 *@Description    This function performs EEPROM checksum generation,
 *@				  by adding the data of all the memory location in a
 *@			      single block and the final added data of each block
 *@				  will be stored in a array and put in to telemetry frame.
 *@
 *@			      The function will be called in every minor cycle, but it
 *@			      is enabled based on a flag set by Func exe telecommand.
 *@
 ******************************************************************/

/****************************************/
void prom_chksum(void)
{
	int eeprom_chksum_index;
	if(eeprom_flag)
	{
		if(eeprom_cur_addr < EEPROM_END_ADDR)
		{
			for(eeprom_chksum_index = 0; eeprom_chksum_index < 64; eeprom_chksum_index++)		// (256 bytes / 4 bytes at a time)
			{
				eeprom_chksum     += REG32(eeprom_cur_addr);
				eeprom_cur_addr   += 4;
			}
			if(eeprom_cur_addr == eeprom_blk_end_addr)
			{
				eeprom_blk_end_addr   = eeprom_cur_addr + EEPROM_BLOCK_SIZE;
				chksum_arr[ee_blk_no] = eeprom_chksum;
				eeprom_chksum         = 0;
				ee_blk_no++;
			}
		}
	}
	else
	{
		eeprom_cur_addr = EEPROM_START_ADDR;
		eeprom_blk_end_addr  = eeprom_cur_addr + EEPROM_BLOCK_SIZE;
		ee_blk_no = 0;
		eeprom_flag = 0;
	}
}

/*unsigned int eeprom_array[10];
unsigned int *ptr_eeprom;
unsigned int eeprom_count;
void EEprom_read()
{
	Out_latch_5.SA2_DEPLOY	 = 0;
	IO_LATCH_REGISTER_5 = Out_latch_5.data;

	Out_latch_4.EEPROM_RESET = 0;
	IO_LATCH_REGISTER_4 = Out_latch_4.data;

	ptr_eeprom = EEPROM_START_ADDR;
	for(i=0;i<=9;i++)
	{
		eeprom_count++;
		eeprom_array[i] = *ptr_eeprom++;
	}
}*/
