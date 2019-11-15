#ifndef HAL_ADC
#define HAL_ADC

#define ADC_READ_DISABLE 0x0000FFFE
#define EXTRACT_LSB_11   0x000007FF
#define ADC_READ_ENABLE  0x00000001
unsigned long int ADC_Status_Data;
unsigned long int inter_HAL_ADC_Data_Ready;

unsigned long int* ADC_Buffer_Addr;
unsigned long int ADC_Addr_Count;
unsigned long int ADC_Locations;
unsigned long int ADC_Data;
unsigned long int inter_Update_TM_With_ADC_offset;
unsigned long int inter_Update_TM_With_ADC_byte_offset;

//Function Declarations
void rHAL_ADC_Read(unsigned long int* ADC_Addr);
void rHAL_ADC_TM_Copy(unsigned long int* ADC_Addr);
void rHAL_ADC_StatusREG_Enable();


#endif // HAL_ADC

