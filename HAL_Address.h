#ifndef HAL_ADDRESS
#define HAL_ADDRESS

//MEMORY CONFIGURATION REGISTER ADDRESS
#define MCFG1 REG32(0x80000000)
#define MCFG2 REG32(0x80000004)
#define MCFG3 REG32(0x80000008)

//GPIO CONFIGURATION & DATA REGISTERS
#define IODAT REG32(0x800000A0)
#define IODIR REG32(0x800000A4)

#define UAC1  REG32(0x80000078)
#define UAC2  REG32(0x80000088)

//INTERRUPT REGISTERS
#define ITMP_REG 	REG32(0x80000090)//Mask & Priority Register
#define ITP_REG  	REG32(0x80000094)//Pending Register
#define ITF_REG 	REG32(0x80000098)//Force Register
#define ITC_REG  	REG32(0x8000009C)//Clear Register
#define IOIT1_REG   REG32(0x800000A8)//IO_Interrupt Register
#define IOIT2_REG   REG32(0x800000AC)//IO_Interrupt Register

//SUBSYSTEM STATUS REGISTER
#define GPS1_STATUS_REGISTER    REG32(0x20000BFC)
#define GPS1_STATUS_REGISTER2   REG32(0x20000BF8)
#define GPS2_STATUS_REGISTER    REG32(0x200013FC)
#define GPS2_STATUS_REGISTER2    REG32(0x200013F8)
#define ADC_STATUS_REGISTER 	REG32(0x200003FC)
#define TC_STATUS_REGISTER  	REG32(0x20004BFC)
#define TM_STATUS_REGISTER  	REG32(0x200043FC)

#define IMU_1_STATUS_REGISTER   0x20001BFC
#define IMU_2_STATUS_REGISTER   0x200023FC

#define RW1_STATUS_REGISTER     0x2000B3FC
#define RW2_STATUS_REGISTER     0x2000BBFC //RW Addresses interchanged for RW2 and RW3 in FPGA
#define RW3_STATUS_REGISTER     0x2000C3FC
#define RW4_STATUS_REGISTER     0x2000CBFC
//#define RW2_STATUS_REGISTER     0x2000C3FC
//#define RW3_STATUS_REGISTER     0x2000BBFC


//RW1 CONFIGURATION REGISTER
#define RW1_CONFIG_REGISTER_1   0x2000B000
#define RW2_CONFIG_REGISTER_1   0x2000B800
#define RW3_CONFIG_REGISTER_1   0x2000C000
#define RW4_CONFIG_REGISTER_1   0x2000C800
//#define RW2_CONFIG_REGISTER_1   0x2000C000
//#define RW3_CONFIG_REGISTER_1   0x2000B800

//RW2 CONFIGURATION REGISTER
//RW Addresses interchanged for RW2 and RW3 in FPGA
//RW3 CONFIGURATION REGISTER
//RW Addresses interchanged for RW2 and RW3 in FPGA
//RW4 CONFIGURATION REGISTER


#define RW1_BUFFER_BASE     0x2000B000
#define RW2_BUFFER_BASE     0x2000B800  //RW Addresses interchanged for RW2 and RW3 in FPGA
#define RW3_BUFFER_BASE     0x2000C000
#define RW4_BUFFER_BASE     0x2000C800
//#define RW2_BUFFER_BASE     0x2000C000
//#define RW3_BUFFER_BASE     0x2000B800


//IMU Configuration Register
#define IMU_1_CONFIG_BASE        0x20001A00
#define IMU_2_CONFIG_BASE        0x20002200


//GPS1 CONFIGURATION REGISTER
#define GPS1_CONFIG_REGISTER_1 		REG32(0x20000BC0)
#define GPS1_CONFIG_REGISTER_2 		REG32(0x20000BC4)
#define GPS1_CONFIG_REGISTER_3 		REG32(0x20000BC8)
#define GPS1_CONFIG_REGISTER_4 		REG32(0x20000BCC)
#define GPS1_CONFIG_REGISTER_5 		REG32(0x20000BD0)
#define GPS1_CONFIG_REGISTER_6 		REG32(0x20000BD4)
#define GPS1_CONFIG_REGISTER_7 		REG32(0x20000BD8)
#define GPS1_CONFIG_REGISTER_8 		REG32(0x20000BDC)
#define GPS1_CONFIG_REGISTER_9 		REG32(0x20000BE0)
#define GPS1_CONFIG_REGISTER_10 	REG32(0x20000BE4)
#define GPS1_CONFIG_REGISTER_11 	REG32(0x20000BE8)
#define GPS1_CONFIG_REGISTER_12 	REG32(0x20000BEC)
#define GPS1_CONFIG_REGISTER_13 	REG32(0x20000BF0)

//GPS2 CONFIGURATION REGISTER
#define GPS2_CONFIG_REGISTER_1 	    REG32(0x200013C0)
#define GPS2_CONFIG_REGISTER_2 	    REG32(0x200013C8)
#define GPS2_CONFIG_REGISTER_3 	    REG32(0x200013D0)
#define GPS2_CONFIG_REGISTER_4 	    REG32(0x200013D8)
#define GPS2_CONFIG_REGISTER_5 	    REG32(0x200013E0)
#define GPS2_CONFIG_REGISTER_6 	    REG32(0x200013E8)
#define GPS2_CONFIG_REGISTER_7 	    REG32(0x200013F0)


//SUBSYSTEM BUFFER ADDRESS
#define GPS1_BUFFER_BASE 	0x20000800
#define GPS2_BUFFER_BASE	0x20001000
#define ADC_BUFFER_BASE 	0x20000000
#define TC_BUFFER_BASE 	 	0x20004800
#define TM_BUFFER_BASE  	0x20004280
#define IMU_1_BUFFER_BASE   0x20001800
#define IMU_2_BUFFER_BASE   0x20002000

//IO OUTPUT LATCH ADDRESS
#define MTR_LATCH_REGISTER   	REG32(0x20006000)// IO_LATCH_REGISTER_1
#define IO_LATCH_REGISTER_2  	REG32(0x20006800)
#define IO_LATCH_REGISTER_3  	REG32(0x20007000)
#define IO_LATCH_REGISTER_4     REG32(0x20007800)
#define IO_LATCH_REGISTER_5		REG32(0x20008000)

//IO INPUT LATCH ADDRESS
#define IO_IN_LATCH_REGISTER_1 	REG32(0x20008BC0)
#define IO_IN_LATCH_REGISTER_2 	REG32(0x200093C4)
#define IO_IN_LATCH_REGISTER_3 	REG32(0x20009BC8)
#define IO_IN_LATCH_REGISTER_4 	REG32(0x2000A3CC)

//ANTENNA ADDRESS
#define ANTENNA_STATUS_REGISTER 		  0x2000D3FC
#define ANTENNA_SLAVE_ADDRESS   		  0x2000D040
#define ANTENNA_READ_DATA_ADDRESS_BASE 	  0x2000D280
#define ANTENNA_WRITE_DATA_ADDRESS_BASE   0x2000D000

//PAYLOAD ADDRESS
#define PAYLOAD_BUFFER_ADDRESS            0x20005800
#define PAYLOAD_STATUS1_ADDRESS           0x20005BFC
#define PAYLOAD_STATUS2_ADDRESS           0x20005BF8

#define GAIN_SELECT_DATA_ADDR             0x400453F4

#endif /* HAL_ADDRESS */
