#include<stdio.h>
#include "Global.h"
#include "Telemetry.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "HAL_Global.h"
#include "HAL_Address.h"
#include "adcs_SensorDataProcs.h"
#include "adcs_VarDeclarations.h"



void rPOR_Init(void)
{
	rTC_Suspended_ModePreprocessing();
	//Telemetry
	TM.Buffer.FrameSynch = 0xF9A42BB1;
	TM.Buffer.Sat_ID = 0x74;
	TM.Buffer.Main_Frame = 0;
	TM.Buffer.Sub_Frame = 0;

	//S_band_on(14-10-19)
	//GPIO_pins.PIO_5 = 1;
	IODAT = GPIO_pins.data;

	inter_TM_Main_Buffer_Empty = TRUE;
	inter_TM_Byte_Count = 0;
	addr_pointer_TM  = NULL;
	IO_In_Latch_Register_4_Data = 0;
	IO_Latch_Register_4_Data = 0;
	Minor_Cycle_Count = 0;
	Major_Cycle_Count = 0;
	inter_TM_Dest_Addr = NULL;
	inter_TM_Source_Addr = NULL;
	inter_Update_TM_With_ADC_offset = 0x00000000u;
	inter_Update_TM_With_ADC_byte_offset = 0x00000000u;
	TC_count = 0;

	//IMU
	inter_HAL_IMU_Data = 0x00000000u;
	inter_HAL_IMU_Addr_Count = 0x00000000u;
	inter_HAL_IMU_Locations = 0x00000000u;
	inter_HAL_IMU_Status_Data = 0x00000000u;
	IMU_Config_Done = FALSE;
	IMU_Data_Available = FALSE;
	IMU_Diag_Done = FALSE;
	rIMU_Init();
	//rPOR_IMU_Parameters_Init();
	rIMU_1_DB_Init();
	rIMU_2_DB_Init();
	//TC_boolean_u.TC_Boolean_Table.TC_IMU_Select = IMU2;

	//ADC
	inter_HAL_ADC_Data_Ready = FALSE;
	ADC_Status_Data = 0;
	ADC_Addr_Count = 0;
	ADC_Locations = 0;
	ADC_Data = 0;

	//GPS
	rGPS_Buffer_Init();

	//SunSensor
	rSS_Main_DB_Init();              // Initialize Sun Sensor Main Database (TC)
	rSS_Main_DB_Copy();              // Copy TC Database to Execute Database
	rSS_Redundant_DB_Init();         // Initialize Sun Sensor Redundant Database (TC)
	rSS_Redundant_DB_Copy();         // Copy TC Database to Execute Database

	//MTR
	//MTR_Current_Data   = 0x00000000u;
	Out_Latch_1.FP_CTRL_MTR1_P1 = 0;
	Out_Latch_1.FP_CTRL_MTR1_N2 = 0;
	Out_Latch_1.FP_CTRL_MTR1_P2 = 0;
	Out_Latch_1.FP_CTRL_MTR1_N1 = 0;

	Out_Latch_1.FP_CTRL_MTR2_P1 = 0;
	Out_Latch_1.FP_CTRL_MTR2_N2 = 0;
	Out_Latch_1.FP_CTRL_MTR2_P2 = 0;
	Out_Latch_1.FP_CTRL_MTR2_N1 = 0;

	Out_Latch_1.FP_CTRL_MTR3_P1 = 0;
	Out_Latch_1.FP_CTRL_MTR3_N2 = 0;
	Out_Latch_1.FP_CTRL_MTR3_P2 = 0;
	Out_Latch_1.FP_CTRL_MTR3_N1 = 0;
	//SA PANEL
	TC_data_command_Table.SA_PanelHeater_Timeout = cPANEL_HEATER_TIMEOUT_CONST;

	//RW
	RW_Init();//HAL RW Initialize

	//Solar Array (Panels)
	TC_HAL_SA_Heater_Timer = 0;

	//Block commands
	rInit_Block();
	BlkCurrent_Cmd = 0;
	BlkExe_Status = BLK_Disabled;

	/** pre-defined block execution commands - Block #0 - Testing 08-03-2019
	 *  Block #0 Execute Command: 0xC410000780000000 */
	Block_array[0][0] = 0xC4D0000000000000;//IMU 1 On
	Block_array[0][1] = 0xC4D0000010010000;//IMU 2 On
	Block_Index[0]    = 2;

	/** pre-defined block execution commands - Block #2 - Testing 08-03-2019
	 *  Block #2 Execute Command: 0xC4100007A0000000*/
	Block_array[2][0] = 0xC4D0000020000200;//IMU 1 Off
	Block_array[2][1] = 0xC4D0000030010200;//IMU 2 Off
	Block_Index[2]    = 2;

	// Initialization of NODE for Time-tag TC
	initNodetable();



	TC_data_command_Table.TC_heaters_auto_manual=0x0000;


	//GAIN_DATA_SET.TC_detumbling_bdot_gain_0_00 = 1.00;
	//offset 00
	GAIN_DATA_SET.TC_detumbling_bdot_gain_0_00 = 0.001;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_1_00 = 0.001;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_2_00 = 0.001;

	GAIN_DATA_SET.TC_detumbling_bdot_gain_0_01 = 0.0001;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_1_01 = 0.0001;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_2_01 = 0.0001;

	GAIN_DATA_SET.TC_detumbling_bdot_gain_0_10 = 0.00001;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_1_10 = 0.00001;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_2_10 = 0.00001;

	GAIN_DATA_SET.TC_detumbling_bdot_gain_0_11 = 0.0005;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_1_11 = 0.0005;
	GAIN_DATA_SET.TC_detumbling_bdot_gain_2_11 = 0.0005;

	//offset 01
	GAIN_DATA_SET.TC_detumbling_rate_gain_0_00 = 0.1;
	GAIN_DATA_SET.TC_detumbling_rate_gain_1_00 = 0.1;
	GAIN_DATA_SET.TC_detumbling_rate_gain_2_00 = 0.1;

	GAIN_DATA_SET.TC_detumbling_rate_gain_0_01 = 0.01;
	GAIN_DATA_SET.TC_detumbling_rate_gain_1_01 = 0.01;
	GAIN_DATA_SET.TC_detumbling_rate_gain_2_01 = 0.01;

	GAIN_DATA_SET.TC_detumbling_rate_gain_0_10 = 0.001;
	GAIN_DATA_SET.TC_detumbling_rate_gain_1_10 = 0.001;
	GAIN_DATA_SET.TC_detumbling_rate_gain_2_10 = 0.001;

	GAIN_DATA_SET.TC_detumbling_rate_gain_0_11 = 0.05;
	GAIN_DATA_SET.TC_detumbling_rate_gain_1_11 = 0.05;
	GAIN_DATA_SET.TC_detumbling_rate_gain_2_11 = 0.05;

	//offset 02
	GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_00 = 1500.0;

	GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_01 = 2000.0;

	GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_10 = 1000.0;

	GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_11 = 1200.0;

	//offset 03
	GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_00 = 0.1;

	GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_01 = 0.2;

	GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_10 = 0.05;

	GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_11 = 0.01;

	//offset 04
	GAIN_DATA_SET.TC_momentum_dumping_gain_0_00 = 0.001;

	GAIN_DATA_SET.TC_momentum_dumping_gain_0_01 = 0.0001;

	GAIN_DATA_SET.TC_momentum_dumping_gain_0_10 = 0.005;

	GAIN_DATA_SET.TC_momentum_dumping_gain_0_11 = 0.01;

	//offset 05//TBD
	GAIN_DATA_SET.TC_PanelD_Status_Sel_0_00;

	GAIN_DATA_SET.TC_PanelD_Status_Sel_0_01;

	GAIN_DATA_SET.TC_PanelD_Status_Sel_0_10;

	GAIN_DATA_SET.TC_PanelD_Status_Sel_0_11;

	//offset 06
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_00 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_00 = 1.0;

	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_01 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_01 = 1.0;

	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_10 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_10 = 1.0;

	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_11 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_11 = 1.0;

	//offset 07
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_00 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_00 = 1.0;

	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_01 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_01 = 1.0;

	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_10 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_10 = 1.0;

	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_11 = 0.0;
	GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_11 = 1.0;

	//offset 08
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_00 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_00 = 1.0;

	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_01 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_01 = 1.0;

	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_10 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_10 = 1.0;

	GAIN_DATA_SET. TC_Mag_LPF_Gain_IMU1_0_11 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_11 = 1.0;

	//offset 09
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_00 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_00 = 1.0;

	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_01 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_01 = 1.0;

	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_10 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_10 = 1.0;

	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_11 = 0.0;
	GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_11 = 1.0;

	//offset 10
	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_0_00 = 0.0;
	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_1_00 = 1.0;

	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_0_01 = 0.0;
	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_1_01 = 1.0;

	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_0_10 = 0.0;
	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_1_10 = 1.0;

	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_0_11 = 0.0;
	GAIN_DATA_SET.TC_SS_Currents_LPF_Gain_1_11 = 1.0;

	//offset 11
	GAIN_DATA_SET.TC_GPS_pulse_duration_0_00 = 614.4;

	GAIN_DATA_SET.TC_GPS_pulse_duration_0_01 = 307.2;

	GAIN_DATA_SET.TC_GPS_pulse_duration_0_10 = 61.44;

	GAIN_DATA_SET.TC_GPS_pulse_duration_0_11 = 1.024;

	//offset 12
	GAIN_DATA_SET.TC_KP_0_00 = 0.0020755;
	GAIN_DATA_SET.TC_KP_1_00 = 0.002362;
	GAIN_DATA_SET.TC_KP_2_00 = 0.00211722;

	GAIN_DATA_SET.TC_KP_0_01;
	GAIN_DATA_SET.TC_KP_1_01;
	GAIN_DATA_SET.TC_KP_2_01;

	GAIN_DATA_SET.TC_KP_0_10;
	GAIN_DATA_SET.TC_KP_1_10;
	GAIN_DATA_SET.TC_KP_2_10;

	GAIN_DATA_SET.TC_KP_0_11;
	GAIN_DATA_SET.TC_KP_1_11;
	GAIN_DATA_SET.TC_KP_2_11;
	//offset 13
	GAIN_DATA_SET.TC_KR_0_00 = 0.0373666;
	GAIN_DATA_SET.TC_KR_1_00 = 0.04252567;
	GAIN_DATA_SET.TC_KR_2_00 = 0.0381176;

	GAIN_DATA_SET.TC_KR_0_01;
	GAIN_DATA_SET.TC_KR_1_01;
	GAIN_DATA_SET.TC_KR_2_01;

	GAIN_DATA_SET.TC_KR_0_10;
	GAIN_DATA_SET.TC_KR_1_10;
	GAIN_DATA_SET.TC_KR_2_10;

	GAIN_DATA_SET.TC_KR_0_11;
	GAIN_DATA_SET.TC_KR_1_11;
	GAIN_DATA_SET.TC_KR_2_11;

	//offset 14
	GAIN_DATA_SET.TC_GPS_Validity_Altitude_Threshold_0_00 = 50.0;

	GAIN_DATA_SET.TC_GPS_Validity_Altitude_Threshold_0_01 = 100.0;

	GAIN_DATA_SET.TC_GPS_Validity_Altitude_Threshold_0_10 = 10.0;

	GAIN_DATA_SET.TC_GPS_Validity_Altitude_Threshold_0_11 = 5.0;

	//offset 15
	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_00 = 5500.0;
/*	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_1_00;
	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_2_00;*/

	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_01 = 5000.0;
/*	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_1_01;
	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_2_01;*/

	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_10 = 4500.0;
/*	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_1_10;
	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_2_10;*/

	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_11 = 6000.0;
/*	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_1_11;
	GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_2_11;*/

	//offset 16
	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_0_00 = 1500.0;
/*	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_1_00;
	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_2_00;*/

	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_0_01 = 2000.0;
/*	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_1_01;
	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_2_01;*/

	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_0_10 = 1000.0;
/*	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_1_10;
	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_2_10;*/

	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_0_11 = 500.0;
/*	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_1_11;
	GAIN_DATA_SET.TC_Wh_SpinUD_Thrsld_2_11;*/


	//offset 17
	GAIN_DATA_SET.TC_comd_pitch_rate_0_00 = 0.1;

	GAIN_DATA_SET.TC_comd_pitch_rate_0_01 = 0.5;

	GAIN_DATA_SET.TC_comd_pitch_rate_0_10 = 0.2;

	GAIN_DATA_SET.TC_comd_pitch_rate_0_11 = 1.0;

	//offset 18
	GAIN_DATA_SET.TC_AngDev_SafeModetransit_Thrsld_0_00 = 40.0;

	GAIN_DATA_SET.TC_AngDev_SafeModetransit_Thrsld_0_01 = 30.0;

	GAIN_DATA_SET.TC_AngDev_SafeModetransit_Thrsld_0_10 = 20.0;

	GAIN_DATA_SET.TC_AngDev_SafeModetransit_Thrsld_0_11 = 25.0;

	//offset 19
	GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_00 = 0.005;

	GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_01 = 0.01;

	GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_10 = 0.008;

	GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_11 = 0.0065;

	//offset 20
	GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_00 = 6000.0;

	GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_01 = 5500.0;

	GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_10 = 5000.0;

	GAIN_DATA_SET.TC_SpeedDump_Thrsld_0_11 = 6500.0;

	//offset 21
	GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_00 = 921.6;

	GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_01 = 614.4;

	GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_10 = 1228.8;

	GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_11 = 491.52;



}

