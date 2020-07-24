#ifndef HAL_MTR
#define HAL_MTR

#define MTR_TM_Bit_HIGH 0x00000001u
#define MTR_TM_Bit_LOW  0xFFFFFFFEu
#define MTR_ON  0x00000100u
#define MTR_OFF 0x0000FEFFu

#define No_Current 0
#define Positive   1
#define Negative   -1

#define Roll  1
#define Pitch 2
#define Yaw   3

//unsigned long int MTR_Current_Data;
//int wait_i;

int Roll_Pol;
int Pitch_Pol;
int Yaw_Pol;
unsigned int MTR_Reset_Flag = 0;
void rHAL_MTR_ON(void);
void rHAL_MTR_OFF(void);
void rHAL_MTR(void);
void MTR_Monitoring();
void rHAL_MTR_TC(unsigned long int MTR_Axis,int MTR_Polarity);



#endif // HAL_MTR
