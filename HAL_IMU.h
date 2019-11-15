#ifndef HAL_IMU
#define HAL_IMU
long int tempat;
//IMU
#define IMU_DATA_BUFFER_MAX_LIMIT     12
#define IMU_DATA_READY      		  0x00000001
#define IMU_READ_ENABLE     		  0x00000004
#define IMU_READ_DATA_READY_DISABLE   0x0000FFF8
#define IMU_CONFIG_ENABLE   		  0x00000008
#define IMU_DIAG_ENABLE     		  0x00000010
#define IMU_SENSOR_ENABLE   		  0x00000020
#define IMU_SENSOR_DISABLE  		  0x0000FFDF
#define IMU_CONFIG_DISABLE 			  0x00000000
#define IMU_DIAG_DISABLE   			  0x00000000
#define IMU_CONFIG_BUFFER_MAX_LIMIT 15

unsigned long int inter_HAL_IMU_Data;
unsigned long int inter_HAL_IMU_Addr_Count;
unsigned long int inter_HAL_IMU_Locations;
unsigned long int* inter_HAL_IMU_Buffer_Addr;
unsigned long int inter_HAL_IMU_Status_Data;
unsigned long int* inter_HAL_IMU_Configure_Address;
unsigned long int IMU_Config_Done;
unsigned long int IMU_Data_Available;
unsigned long int IMU_Diag_Done;

struct HAL_IMU_Data_Structure
{
	unsigned long int IMU_Configure_Register;
    unsigned long int IMU_Buffer_Address;
	unsigned long int IMU_Status_Register;

}IMU_1,IMU_2;

void rIMU_Init(void);
unsigned long int IMU_On;
unsigned long int HAL_IMU_Read(struct HAL_IMU_Data_Structure IMU_No, unsigned long int *IMU_Addr);
unsigned long int HAL_IMU_Config(struct HAL_IMU_Data_Structure* IMU_No, unsigned long int *IMU_Configure_Addr,unsigned long int IMU_Configure_Words);
unsigned long int HAL_IMU_Diag(struct HAL_IMU_Data_Structure* IMU_No,unsigned short IMU_Diag_Addr);
void rHAL_IMU_POWER(unsigned long int IMU_No,unsigned long int IMU_Power);

#endif // HAL_IMU
