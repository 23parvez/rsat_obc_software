#ifndef TELEMETRY
#define TELEMETRY

//-------------Functions' Declarations-----------------------
//HAL_TM
void rHAL_TM_Write(void);
void rHAL_TM_HW_Status_Update(void);

 // TM Sub-frame Routines
void rTM_Address_Table_Init(void);
void rTM_Copy_Subframe();

#define TMTC_BUFFER_SIZE 16

 // Telemetry buffer
#ifndef NULL
#define NULL ((void*)0)
#endif

#define TM_GBL_MAX_LIMIT sizeof(Buffer)
#define TM_BUFFER_LIMIT 256 // 64 (256 Bytes/4) for TM Buffer Data Type unsigned long int ; 256 for TM Buffer Data Type char
#define NO_OF_SUBFRAMES 16
#define TM_TABLE_MAX_LIMIT ((NO_OF_SUBFRAMES*(TM_BUFFER_LIMIT + 1)) + 1)

 // Address Table
struct FRAME_ADDR_TABLE{
	unsigned char * Addr_Field;
	unsigned long int Length_Field;
}TM_Table[TM_TABLE_MAX_LIMIT];

 // Global TM Buffer
#include "TM_Global_Buffer.h"

 // UNION For Address Pointer
union TM_BUFFER_UNION{
	char Buffer_Global[TM_GBL_MAX_LIMIT];
	struct TM_BUFFER_STUCTURE Buffer;
}TM;

unsigned long int inter_TM_Byte_Count;
char  TM_Buffer[TM_BUFFER_LIMIT];
char* inter_TM_Dest_Addr;
char* inter_TM_Source_Addr;
unsigned long int inter_TM_Main_Buffer_Empty;

//static unsigned long int TMTC_Buffer[TMTC_BUFFER_SIZE];
//TM STATUS REGISTER
//[15]    - Shift Empty Bit                 -Read
//[14]    - Dual Port Memory Full Bit       -Read
//[13]    - Convolution Enable Bit          -Read
//[12]    - Convolution Finish Bit         	-Read
//[11:7]  - Counter Memory Bits     	    -Read
//[6 :1]  - NA                      	    -NA
//[0]     - Write Enable Bit        		-Read/Write

#define TM_DP_MEMORY_FULL 			    0x00004000
#define TM_DP_MEMORY_WRITE_ENABLE 	    0x00000001
#define TM_DP_MEMORY_EMPTY    			0x00000000
#define TM_DP_MEMORY_WRITE_DISABLE      0xFFFFFFFE
#define TM_DP_MEMORY_SIZE               16          // Size of Dual Port Memory in Words(2 Bytes)

unsigned long int* addr_pointer_TM;
unsigned long int* inter_TM_DP_Addr;
unsigned short inter_TM_Status_Data;
int inter_TM_Minor_Cycle_Count;
int inter_HAL_TM_Write_Word_Count;
int TM_Table_Row_No;

unsigned int m;

#endif // TELEMETRY
