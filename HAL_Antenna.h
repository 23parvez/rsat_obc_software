#ifndef HAL_ANTENNA
#define HAL_ANTENNA

#define ANTENNA_SLAVE1 0x00000031
#define ANTENNA_SLAVE2 0x00000032
#define I2C_Write  	   0x00000000
#define I2C_Read       0x00000001

#define HAL_SA_Deploy_Disable 0
#define HAL_SA_Deploy_Enable  1
#define c_HAL_SA_Heater_Timer_Max_Thrsld 10
#define c_HAL_SA_Heater_Timer_Min_Thrsld 5
#define HAL_SA1_SA2_Deploy 0x00001800

unsigned long int inter_HAL_Antenna_Addr;
unsigned long int TC_HAL_SA_Heater_Timer;
unsigned long int SA1_Deploy_cmd_rcvd_time = 0U;
unsigned long int SA2_Deploy_cmd_rcvd_time = 0U;
unsigned long int SA_Command_Type = 0;

unsigned long int TC_HAL_SA_Deploy = HAL_SA_Deploy_Disable; //To be deleted TESTING!!!

void rHAL_Antenna_Write(unsigned long int Command);
void rHAL_Antenna_Read();
void rHAL_SA_Deploy(unsigned int SA_No, unsigned int SA_Status);
void rHAL_Antenna_Deploy(unsigned int status);
void rHAL_SA_Deploy_Status();


#endif // HAL_ANTENNA
