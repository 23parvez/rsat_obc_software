#include "HAL_Global.h"
#include "HAL_Address.h"
#include "Telemetry.h"
#include "TM_Global_Buffer.h"
#include "Global.h"
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

	//Input Latches
	TM.Buffer.TM_Input_Latch[0] = IO_IN_LATCH_REGISTER_1;
	TM.Buffer.TM_Input_Latch[1] = IO_IN_LATCH_REGISTER_2;
	TM.Buffer.TM_Input_Latch[2] = IO_IN_LATCH_REGISTER_3;
	TM.Buffer.TM_Input_Latch[3] = IO_IN_LATCH_REGISTER_4;

}
void EEPROM_RST()
{
	unsigned short tempdata;
	Out_latch_4.EEPROM_RESET = 1;
	tempdata = Out_latch_4.data;
	IO_LATCH_REGISTER_4;
	IO_LATCH_REGISTER_4 = tempdata;
}

void EEPROM_RES()
{
	unsigned short tempdata;
	Out_latch_5.SA2_DEPLOY	 = 1;
	IO_LATCH_REGISTER_5 = Out_latch_5.data;
}

