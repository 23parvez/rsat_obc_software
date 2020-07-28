#include <math.h>
#include <bcc/bcc.h>
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "adcs_VarDeclarations.h"
#include "Telemetry.h"
#include "TC_List.h"


//#include <time.h>
//clock_t tcl1,tcl2,tcl3;

void OBC_Process();
void rMinCycle_Wait();
void rMinCycle_Flag_Reset(void);
void rMinCycle_Wait(void);

extern void (fdi_nmi_int_handler)(void);
extern void bcc_flush_cache();
extern int bcc_set_trap(int tt, void (*handler)(void));
extern int bcc_int_clear(int source);
extern int bcc_int_unmask(int source);


int main (void)
{
	UAC1         = 0x00000000;				                  //Disable UART1
	UAC2         = 0x00000000;					              //Disable UART2
	IODIR = 0x00002AE4;
    //--------------------------Power ON Initialization Process--------------------------------------------------

	Init_Memory();								              //Memory Init
    rTM_Address_Table_Init();					              //TM Address Table Initialize
    Norm_ST_1_Table_Init();
    Spec_ST_Table_Init();
	rPOR_Init();      	                                      //All POR

    //NMI Initialization
	ITMP_REG     = 0x00200020;					              //Enable NMI with High Priority
	IOIT1_REG    = 0x000000E4;					              //IO_4 Interrupt Enable, Rising Edge
	bcc_set_trap(0x14, fdi_nmi_int_handler);
	bcc_flush_cache();
	bcc_int_clear(FDI_NMI_INT_LEVEL);                         //Clear Interrupt
	bcc_int_unmask(FDI_NMI_INT_LEVEL);                        //Unmask Interrupt

    OBC_Process();                                            //OBC Processing Routine
	return 0;
}

void OBC_Process()
{
	rADCS_Pon_vars();
	//inputs for GPS......//////////////////////////////////////////////////////

	Pos_ECEF_GPS[0] = 6426.417462;
	Pos_ECEF_GPS[1] = 2493.913765;
	Pos_ECEF_GPS[2] = -1.124572057;
	Vel_ECEF_GPS[0] = 0.529197934;
	Vel_ECEF_GPS[1] = -1.397937956;
	Vel_ECEF_GPS[2] = 7.538830744;
	year_GPS        = 2017;
	UTC_mon_GPS     = 3;
	UTC_day_GPS     = 30;
	UTC_hr_GPS      = 20;
	UTC_min_GPS     = 6;
	UTC_sec_GPS     = 4.664;

	GPSDataReady=1;


	rMinCycle_Flag_Reset();			            //Reset Minor Cycle Flag
	rMinCycle_Wait();                               //Wait for next Minor Cycle Terminal Count

	rTM_Copy_Subframe();                            // Generate the the 1st FRAME


	//EEPROM RESET for Different Models
#ifdef QUALIFICATION_MODEL
			EEPROM_RES();
	#elif	PROTO_BOARD
			EEPROM_RST();
	#endif

	while(1)
	{
		//-----------------------------Minor Cycle 1------------------------------------------------------------------

		GPIO_pins.PIO_11 = 1;
		IODAT = GPIO_pins.data;

		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;
	#ifdef ENGINEERING_MODEL
		GPIO_pins.PIO_9 = 1;
		IODAT = GPIO_pins.data;
	#endif

		rHAL_TC_Read();							       // Read TC from FPGA Buffer
		rTelecommand();							       // TC routine for decoding & Execution of Real Time Commands
		rAbsoluteTTC_Execute();					       // Execution of Absolute Time Tag Commands
		//ADCS routines********//

		rIMUDataProcessing();                          // Read the IMU data first
		//rpl_read() ;
		//rpl_tm_write();
		rTM_Real_st_write();

		//Storage_Telemetry_Write();

		rRW_Data_Read();                               // Read the Reaction Wheel speeds
		//s_ram_scrub();                                 // scrubing SRAM
		//prom_chksum();
		//EEprom_read();

		// ADCS Processing begins here

		rErrorComputation();
		rLinearController();
		rHAL_MTR();
		rGPS_pulsecheck();
		//rHILS_packets();
		// ADCS Processing ends here

		pl_tx_tm();
		rRW_Data_Write();                              // Set the Reaction Wheel Speeds


		TMTC_Assignment();					           // Update TMTC Buffer
		rHAL_ADC_StatusREG_Enable();			       // Set ADC Status Register
		rHAL_TM_HW_Status_Update();				       // Update HW status of OBC to TM GBL Buffer
	#ifdef ENGINEERING_MODEL
		GPIO_pins.PIO_9 = 1;
		IODAT = GPIO_pins.data;
	#endif
		GPIO_pins.PIO_6 = 0;
		IODAT = GPIO_pins.data;

		GPIO_pins.PIO_11 = 0;
		IODAT = GPIO_pins.data;

		rMinCycle_Wait();						       // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					       // Increment Minor Cycle Count

		//-----------------------------Minor Cycle 2------------------------------------------------------------------

		//ADCS routines********//

		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;

		TC_status_reset();              // TC_buffer read_enable_reset
		rIMUDataProcessing();

							       // TM_Write Minor Cycle 2
		rTM_Real_st_write();
		//rHAL_pl_sts_check();
		Storage_Telemetry_Write();
		rRW_Data_Request();                            // Request the  wheel speed data from Reaction wheels

		s_ram_scrub();                                 // scrubing SRAM

		//rIMUDataProcessing();                        // IMU Data Processing : Minor Cycle 2
		rErrorComputation();
		rLinearController();
		rGPS_pulsecheck();
		rSunSensorDataProcessing();

		rBDOT_Computation();
		rHAL_MTR();

		//ADCS routines********//
		//Thermister_select();
		rHAL_ADC_Read(ADC_Buffer);				       // Set ADC Read Enable
		rHAL_TM_HW_Status_Update();				       // Update HW status of OBC to TM GBL Buffer
		rHAL_ADC_TM_Copy(ADC_Buffer);		       // Copy ADC Data to TM Global Buffer
		sun_Sensor_data();
		UTP_selection();
		LTP_selection();
		Heater_control_auto_manual();


        //*******GPS_ADCS_Routines*******//

		rGPSDataProcessing();
		rOrbit_Initialization();
		rJulian_Day(year_sel,mon_sel,days_sel,hr_sel,minute_sel,sec_sel);
		rECEFtoECItoECEF();
		rOrbit_Propagation();

		GPIO_pins.PIO_6 = 0;
		IODAT = GPIO_pins.data;

		rMinCycle_Wait();						      // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					      // Increment Minor Cycle Count

		//-----------------------------Minor Cycle 3------------------------------------------------------------------

		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;
		rIMUDataProcessing();

		//rpl_read() ;
		//rpl_tm_write();
					       // TM_Write Minor Cycle 3
		rTM_Real_st_write();
		//Storage_Telemetry_Write();
		rRW_Data_Read();                               // Read the Reaction Wheel speeds
		rBlockTC_Execute();                            // Block_TC_Execution

		s_ram_scrub();                                 // scrubing SRAM

		 //ADCS routines begins//

		rErrorComputation();
		rLinearController();
		rHAL_MTR();
		rGPS_pulsecheck();
		//rHILS_packets();

		rHAL_ADC_StatusREG_Enable();			      // Set ADC Status Register
		rHAL_TM_HW_Status_Update();				      // Update HW status of OBC to TM GBL Buffer
		rHAL_EPS_shunt_switch();

		rOrbitalElements_computation(Pos_ECI, Vel_ECI, Pos_ECEF);
		rNEDtoECEF();
		rSun_Ephemeris();
		rSl_Ecl_OnBrd_detection();
		rGH_generation();
		rMagFieldComp1();
		rMagFieldComp2();
		rReferenceQuatComputation();
		rRefVectorGeneration();
		rRefRate_Computation();
		//ADCS routines Ends//

		rRW_Data_Write();

		//rpl_sts();

		GPIO_pins.PIO_6 = 0;
		IODAT = GPIO_pins.data;

		rMinCycle_Wait();						      // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					      // Increment Minor Cycle Count

		//-----------------------------Minor Cycle 4------------------------------------------------------------------
	    //ADCS routines********//
		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;
		rIMUDataProcessing();
								       // TM_Write Minor Cycle 4
		rTM_Real_st_write();
		//Storage_Telemetry_Write();
		rRW_Data_Request();								// Request the  wheel speed data from Reaction wheels

		s_ram_scrub();                                 // scrubing SRAM

		rRemote_data_view();

		rErrorComputation();
		rLinearController();
		rGPS_pulsecheck();
		rQuestDataProcessing();
		rDAD_quest();

		rAngularMomentumDumping();
		rHAL_MTR();
		//ADCS routines********//

		rHAL_ADC_Read(ADC_Buffer);				      // Set ADC Read Enable
		rHAL_TM_HW_Status_Update();				      // Update HW status of OBC to TM GBL Buffer
	    //rHAL_SA_Deploy_Status();
		rHAL_SA_Deploy_Status_new();

	    GPIO_pins.PIO_6 = 0;
	    IODAT = GPIO_pins.data;

		rMinCycle_Wait();						      // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					      // Increment Minor Cycle Count

		//-----------------------------Minor Cycle 5------------------------------------------------------------------

		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;
		rHAL_TC_Read();							       // Read TC from FPGA Buffer
		rTelecommand();
		rIMUDataProcessing();
		//rpl_read() ;
		//rpl_tm_write();
							       // TM_Write Minor Cycle 5
		rTM_Real_st_write();
		//Storage_Telemetry_Write();
		rRW_Data_Read();                               // Read the Reaction Wheel speeds

		s_ram_scrub();                                 // scrubing SRAM
		//ADCS routines begins//

		rErrorComputation();
		rLinearController();
		rDutyCycleGeneration();
		rScModeSelection();
		rHAL_MTR();
		rGPS_pulsecheck();
		//rHILS_packets();
		//ADCS routines ends//

		rRW_Data_Write();

		rHAL_ADC_StatusREG_Enable();			       // Set ADC Status Register
		rHAL_TM_HW_Status_Update();				       // Update HW status of OBC to TM GBL Buffer

		GPIO_pins.PIO_6 = 0;
		IODAT = GPIO_pins.data;
		//rHAL_pl_cmd_hlt();

		rMinCycle_Wait();						       // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					       // Increment Minor Cycle Count

		//-----------------------------Minor Cycle 6------------------------------------------------------------------
		//ADCS routines********//
		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;

		TC_status_reset();              // TC_buffer read_enable_reset
		rIMUDataProcessing();

		rTM_Real_st_write();
		//Storage_Telemetry_Write();
		rRW_Data_Request();
		pl_tx_tm_2();                            // write last 2 TM bytes

		s_ram_scrub();                                 // scrubing SRAM

		//ADCS routines begins//

		rErrorComputation();
		rLinearController();
		rHAL_MTR();
		rGPS_pulsecheck();
		rExtendedKalmanFilter1_p1();
		//ADCS routines ends//

		rHAL_ADC_Read(ADC_Buffer);				       // Set ADC Read Enable
		rHAL_TM_HW_Status_Update();				       // Update HW status of OBC to TM GBL Buffer

		GPIO_pins.PIO_6 = 0;
		IODAT = GPIO_pins.data;

		rMinCycle_Wait();						       // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					       // Increment Minor Cycle Count

		//-----------------------------Minor Cycle 7------------------------------------------------------------------
		//ADCS routines********//
		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;
		rIMUDataProcessing();

		//pl_tx_tm_2();                            // write last 2 TM bytes
		rTM_Real_st_write();
		rpl_read();
		rpl_tm_write();

		s_ram_scrub();                                 // scrubing SRAM

		//Storage_Telemetry_Write();
		rRW_Data_Read();
		//ADCS routines begins//
		rErrorComputation();
		rLinearController();
		rHAL_MTR();
		rGPS_pulsecheck();
		rExtendedKalmanFilter1_p2();
		//rHILS_packets();
		//ADCS routines ends//
		rRW_Data_Write();

		rHAL_ADC_StatusREG_Enable();			       // Set ADC Status Register
		rHAL_TM_HW_Status_Update();				       // Update HW status of OBC to TM GBL Buffer

		rHAL_pl_cmd_hlt();

		GPIO_pins.PIO_6 = 0;
		IODAT = GPIO_pins.data;

		rMinCycle_Wait();						       // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					       // Increment Minor Cycle Count

		//-----------------------------Minor Cycle 8------------------------------------------------------------------

		GPIO_pins.PIO_6 = 1;
		IODAT = GPIO_pins.data;
		rIMUDataProcessing();

		rTM_Real_st_write();
		//Storage_Telemetry_Write();


		rRW_Data_Request();

		s_ram_scrub();                                 // scrubing SRAM
		//ADCS routines begins//
		rErrorComputation();
		rLinearController();
		rHAL_MTR();
		rGPS_pulsecheck();
		//ADCS routines ends//

		//PL_TM_read();                                  // Read the Payload data
		rHAL_ADC_Read(ADC_Buffer);				       // Set ADC Read Enable
		rHAL_TM_HW_Status_Update();			           // Update HW status of OBC to TM GBL Buffer
		rOutput_Latch_Update();
		ST_output_update();
		rHAL_Antenna_Read();
		MUX_Output();
		rpl_read();
		rpl_tm_write();
		NMI_interrupt_test();

		GPIO_pins.PIO_6 = 0;
		IODAT = GPIO_pins.data;

		rMinCycle_Wait();						       // Wait Till Minor Cycle Terminal Count (16ms)
		Minor_Cycle_Count++;					       // Increment Minor Cycle Count
		Major_Cycle_Count++;					       // Increment Major Cycle Count
	}
}

void rMinCycle_Wait(void)
{
	uint16 tempdata;
	minor_cycle_counter_test++;
	IO_In_Latch_Register_4_Data = IO_IN_LATCH_REGISTER_4 & MINOR_CYCLE_TERMINAL_COUNT;
	while(IO_In_Latch_Register_4_Data != MINOR_CYCLE_TERMINAL_COUNT)		    				//Wait for Minor_Cycle_Terminal_Count
	{
		IO_In_Latch_Register_4_Data = IO_IN_LATCH_REGISTER_4 & MINOR_CYCLE_TERMINAL_COUNT;

	}
	IO_Latch_Register_4_Data	= IO_LATCH_REGISTER_4 | MINOR_CYCLE_RESET;
	tempdata 		= IO_Latch_Register_4_Data;	                                           //Reset Minor Cycle;
	IO_LATCH_REGISTER_4 ;
	IO_LATCH_REGISTER_4 = tempdata ;

	IO_Latch_Register_4_Data 	= IO_LATCH_REGISTER_4 & (~MINOR_CYCLE_RESET);
	tempdata 		= IO_Latch_Register_4_Data;
	IO_LATCH_REGISTER_4;
	IO_LATCH_REGISTER_4 = tempdata;
}

void rMinCycle_Flag_Reset(void)
{
	uint16 tempdata;
	IO_Latch_Register_4_Data  = IO_LATCH_REGISTER_4 | MINOR_CYCLE_RESET;
	tempdata    = IO_Latch_Register_4_Data;	//Reset Minor Cycle;
	IO_LATCH_REGISTER_4;
	IO_LATCH_REGISTER_4 = tempdata ;

	IO_Latch_Register_4_Data  = IO_LATCH_REGISTER_4 & (~MINOR_CYCLE_RESET);
	tempdata      = IO_Latch_Register_4_Data;
	IO_LATCH_REGISTER_4;
	IO_LATCH_REGISTER_4 = tempdata ;
}
