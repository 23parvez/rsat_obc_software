#ifndef HAL_EPS
#define HAL_EPS

unsigned long int inter_HAL_EPS_Hardware_Status_count = 0;
unsigned long int inter_HAL_EPS_Hardware_Status_SEL[8] = {0x00000000,0x00000004,
														  0x00000002,0x00000006,
														  0x00000001,0x00000005,
														  0x00000003,0x00000007};

void rHAL_EPS_shunt_switch();
void rHAL_EPS_Hardware_Status();

#endif // HAL_EPS
