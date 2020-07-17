#ifndef TELEMETRY
#define TELEMETRY
 // Global TM Buffer
#include "TM_Global_Buffer.h"
//-------------Functions' Declarations-----------------------
//HAL_TM
void rHAL_TM_Write(void);
void rHAL_TM_HW_Status_Update(void);

 // TM Sub-frame Routines
void rTM_Address_Table_Init(void);
void Norm_ST_1_Table_Init();
void TC_hist_view();
void Spec_ST_Table_Init();
void rTM_Copy_Subframe();
void ST_DUMPING();
void ST_full_dump();
void TC_Hist_dumping();
void rHAL_ST_TM_Write(void);
void rTM_Real_st_write();

#define TMTC_BUFFER_SIZE 16
#define circuar_spec 0
#define circuar_norm 1

 // Telemetry buffer
#ifndef NULL
#define NULL ((void*)0)
#endif


#define TM_GBL_MAX_LIMIT sizeof(Buffer)
#define TM_STR_NRM_LIMIT sizeof(ST_NM_Buffer)
#define TM_STR_SPL_LIMIT sizeof(ST_SP_Buffer)


#define TM_BUFFER_LIMIT 256 // 64 (256 Bytes/4) for TM Buffer Data Type unsigned long int ; 256 for TM Buffer Data Type char
#define NO_OF_SUBFRAMES 16
#define NO_OF_TC_SUBFRAME 1
#define NO_OF_STR_SUBFRAME 4
#define TM_TABLE_MAX_LIMIT ((NO_OF_SUBFRAMES*(TM_BUFFER_LIMIT + 1)) + 1)

#define frame_size 256   //size of four frames
#define curr_frame_size  ((NO_OF_STR_SUBFRAME*(TM_BUFFER_LIMIT + 1)) + 1)  //has to be 1024 for four frames of normal frames of 256 each
#define curr_tc_frame_size ((NO_OF_TC_SUBFRAME*(TM_BUFFER_LIMIT + 1)) + 1)
#define storageSize 256000   //size of the storage telemetry buffer (planned 64KB)
#define TC_storageSize 256
#define TC_final_storageSize 64000
#define bufferSize 256
#define ST_BUFFER_LIMIT_COUNT 1000

 // Address Table
struct FRAME_ADDR_TABLE{
	unsigned char * Addr_Field;
	unsigned long int Length_Field;
}TM_Table[TM_TABLE_MAX_LIMIT];

struct Norm_ST_ADDR_TABLE{
	unsigned char * Addr_Field;
	unsigned long int Length_Field;
}Norm_ST_Table[curr_frame_size];

struct Spec_ST_ADDR_TABLE{
	unsigned char * Addr_Field;
	unsigned long int Length_Field;
}Spec_ST_Table[curr_frame_size];

struct TC_history_TABLE{
	unsigned char * Addr_Field;
	unsigned long int Length_Field;
}TC_history_table[curr_tc_frame_size];


 // UNION For Address Pointer
union TM_BUFFER_UNION{
	char Buffer_Global[TM_GBL_MAX_LIMIT];
	struct TM_BUFFER_STUCTURE Buffer;
}TM;

union TM_ST_NORMAL{
	char Buffer_Global_normal[TM_STR_NRM_LIMIT];
	struct ST_NORMAL ST_NM_Buffer;
}ST_normal;

union TM_ST_SPECIAL
{
	char Buffer_Global_special[TM_STR_SPL_LIMIT];
	struct ST_SPECIAL_table ST_SP_Buffer;
}ST_special;

unsigned long int inter_TM_Byte_Count;
char  TM_Buffer[TM_BUFFER_LIMIT];
char* inter_TM_Dest_Addr;
char* inter_TM_Source_Addr;
unsigned long int inter_TM_Main_Buffer_Empty;
unsigned long int inter_TM_Pl_Buffer;
unsigned short* inter_TM_ST_TC_NS_Write_Source_Addr;
//unsigned char* TCH_write_ptr;
//union TC_Hist *TC_hist_read_ptr;
unsigned short ST_frame_count;
unsigned short ST_TCH_frame_count;
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
#define TM_DP_MEMORY_SIZE               32         // Size of Dual Port Memory in Words(2 Bytes)

unsigned long int* addr_pointer_TM;
unsigned long int* inter_TM_DP_Addr;
unsigned short inter_TM_Status_Data;
int inter_TM_Minor_Cycle_Count;
int inter_TM_real_Minor_Cycle_Count;
int inter_ST_TM_Minor_Cycle_Count;
int inter_ST_TC_TM_Minor_Cycle_Count;
int inter_HAL_TM_Write_Word_Count;
int TM_Table_Row_No;
unsigned int Normal_st_table_page1, Normal_st_table_page2, Normal_st_table_page3;
unsigned int Special_st_table_page1, Special_st_table_page2, Special_st_table_page3;
unsigned short* ST_start_read;
unsigned short* ST_end_read;

//Initialisation of the source and destination array
unsigned char Normal_source[frame_size];        //normal source initialization
unsigned char Special_source[frame_size];       //special source initialization
unsigned char ST_source_Buffer[bufferSize];    //storage area initialization
unsigned char ST_TC_history_Buffer[TC_storageSize];
unsigned char ST_TC_history_final_Buffer[TC_final_storageSize];
unsigned char Storage[storageSize];
unsigned char* Dest_end_addr; 							    // initializing Destination end address
unsigned char* Dest_TC_final_end_addr;
unsigned char* write_str_ptr;
unsigned short* read_str_ptr;
unsigned char* TC_str_ptr;
unsigned int storage_page_count;
unsigned int TM_page_count;
unsigned int TCH_read_ptr;
unsigned int TCH_read_full_ptr;
unsigned int TCH_cpy_ptr;
unsigned char rt_tm_frame_finish ;
unsigned char TCH_start_buffer_ready;
unsigned int TCH_full_dump_finish;
unsigned int TCH_dump_finish;
//unsigned char* TC_write_str_ptr;


//void Special_ST( unsigned int Sampling_Rate, unsigned int OBT_start_time );
int Frame_Address_Select ();
int Sampling_Rate_Select();
//void Normal_ST( unsigned int Sampling_Rate);
void ST_Copy_Subframe(int frame_addr,unsigned int Sampling_Rate );
void StorageTelemetry();
int copy_frame(  unsigned int Circular );
void rTC_final_storage();
void Storage_Telemetry_Write();
void rTCH_full_dump_cpy_buf();
void rTCH_dump_cpy_buf();

// Defining Normal Sampling Rates
#define Normal_Sampling_Rate_A 48	// 6 Secs sampling interval
#define Normal_Sampling_Rate_B 24	// 3 Secs sampling interval

//storage TM variables
unsigned int count_tc_hist;


//Defining Special Sampling Rates
#define Special_Sampling_Rate_A 4	// 0.5 secs sampling interval
#define Special_Sampling_Rate_B 8	// 1 sec sampling interval
#define Special_Sampling_Rate_C 12	// 1.5 secs sampling interval
#define Special_Sampling_Rate_D 16	// 2 secs sampling interval



unsigned char* ST_Dest_Addr;


int Norm_ST_Table_Row_No;
int Spec_ST_Table_Row_No;
int TC_ST_Table_Row_No;

#endif // TELEMETRY
